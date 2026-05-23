/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "usb/usb_host.h"
#include "esp_libusb.h"
#include "dsp_processor.h"
#include "frame_decoder.h"
#include "signal_buffer.h"
#include "worker_core1.h"
#include "ingest_core1.h"
#include "resample_256_to_250.h"
#include "fft_sc16_2048.h"
#include "app_config.h"
#include "class_driver.h"
#include "bch_decoder.h"
#include "rtl-sdr.h"
#include "status_logger.h"

#define CLIENT_NUM_EVENT_MSG 5

#define ACTION_OPEN_DEV 0x01
#define ACTION_GET_DEV_INFO 0x02
#define ACTION_GET_DEV_DESC 0x04
#define ACTION_GET_CONFIG_DESC 0x08
#define ACTION_GET_STR_DESC 0x10
#define ACTION_CLOSE_DEV 0x20
#define ACTION_EXIT 0x40
#define ACTION_START_STREAM 0x80

static const char *TAG = "CLASS";
static rtlsdr_dev_t *rtldev = NULL;
static volatile int s_last_gain_dbx10 = -1;

bool class_driver_set_tuner_gain_dbx10(int gain_dbx10)
{
    if (!rtldev) return false;
    if (gain_dbx10 < 0) return false;
    int r = rtlsdr_set_tuner_gain(rtldev, gain_dbx10);
    if (r != 0) return false;
    s_last_gain_dbx10 = gain_dbx10;
    return true;
}

int class_driver_get_tuner_gain_dbx10(void)
{
    return s_last_gain_dbx10;
}
static class_driver_t s_driver_obj = {0};

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = &s_driver_obj;
    switch (event_msg->event)
    {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI(TAG, "New USB device connected at address %d", event_msg->new_dev.address);
        if (driver_obj->dev_addr == 0)
        {
            driver_obj->dev_addr = event_msg->new_dev.address;
            driver_obj->actions |= ACTION_OPEN_DEV;
        }
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGI(TAG, "USB device gone");
        if (driver_obj->dev_hdl != NULL)
        {
            driver_obj->actions |= ACTION_CLOSE_DEV;
        }
        break;
    default:
        ESP_LOGI(TAG, "Unknown USB client event: %d", event_msg->event);
        break;
    }
}

static void action_open_dev(class_driver_t *driver_obj)
{
    ESP_LOGI(TAG, "Opening device at address %d", driver_obj->dev_addr);
    esp_err_t err = rtlsdr_open(&rtldev, driver_obj->dev_addr, driver_obj->client_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open RTL-SDR: %d", err);
        return;
    }
    
    driver_obj->dev_hdl = esp_libusb_get_dev_hdl();
    ESP_LOGI(TAG, "Device opened, handle: %p", driver_obj->dev_hdl);
    
    driver_obj->actions &= ~ACTION_OPEN_DEV;
    driver_obj->actions |= ACTION_START_STREAM;
}

static void action_start_stream(class_driver_t *driver_obj)
{
    ESP_LOGI(TAG, "Configuring RTL-SDR from NVS config...");
    app_config_t cfg;
    app_config_snapshot(&cfg);
    rtlsdr_set_sample_rate(rtldev, cfg.sample_rate_hz);
    rtlsdr_set_center_freq(rtldev, cfg.lo_freq_hz);
    // gain_mode mapping (RTL-SDR side):
    //   TUNER_AGC    -> rtlsdr gain_mode 0 (tuner internal AGC)
    //   MANUAL       -> rtlsdr gain_mode 1 + set_tuner_gain
    //   SOFTWARE_AGC -> rtlsdr gain_mode 1 + initial gain; D16 module
    //                   adjusts at runtime
    if (cfg.gain_mode == GAIN_MODE_TUNER_AGC) {
        rtlsdr_set_tuner_gain_mode(rtldev, 0);
    } else {
        rtlsdr_set_tuner_gain_mode(rtldev, 1);
        // Convert tenths of dB to the RTL-SDR API unit (also tenths of dB).
        rtlsdr_set_tuner_gain(rtldev, cfg.gain_db_x10);
    }
    // Bias tee: RTL-SDR v4 specific. Drives the 5 V bias on the antenna
    // line via the dongle's GPIO 0. Safe no-op on hardware without an
    // active antenna or LNA.
    int br = rtlsdr_set_bias_tee(rtldev, cfg.bias_tee ? 1 : 0);
    ESP_LOGI(TAG, "bias_tee: %s (rc=%d)", cfg.bias_tee ? "ON" : "OFF", br);
    rtlsdr_reset_buffer(rtldev);

    ESP_LOGI(TAG, "Initializing System Buffers...");
    // Allocate PIE-position-sensitive buffers FIRST so they land at
    // known-working addresses regardless of upstream heap growth.
    // See project_heap_position_decode_bug.md.
    resample_256_to_250_alloc_coeffs();
    fft_sc16_2048_init();
    signal_buffer_init();
    worker_core1_init();
    ingest_core1_init();
    bch_decoder_init();
    if (frame_decoder_init() != ESP_OK) {
        ESP_LOGW(TAG, "frame_decoder_init failed; higher-layer "
                 "classification will be silently skipped");
    }

    ESP_LOGI(TAG, "Initializing DSP...");
    dsp_processor_init(worker_core1_push_burst);

    ESP_LOGI(TAG, "Starting Async Stream...");
    esp_libusb_start_stream(driver_obj, 0x81);
    
    driver_obj->actions &= ~ACTION_START_STREAM;
}

static void action_close_dev(class_driver_t *driver_obj)
{
    ESP_LOGI(TAG, "Closing device");
    if (rtldev) {
        rtlsdr_close(rtldev);
        rtldev = NULL;
    }
    driver_obj->dev_hdl = NULL;
    driver_obj->dev_addr = 0;
    driver_obj->actions &= ~ACTION_CLOSE_DEV;
    driver_obj->actions |= ACTION_EXIT;
}

void class_driver_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;
    memset(&s_driver_obj, 0, sizeof(class_driver_t));

    xSemaphoreTake(signaling_sem, portMAX_DELAY);

    // Bring up the Core 1 logger task before we start streaming so the
    // first per-second snapshot has somewhere to land.
    if (status_logger_init() != ESP_OK) {
        ESP_LOGW(TAG, "status_logger_init failed; status logs will be silently dropped");
    }

    ESP_LOGI(TAG, "Registering Client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void *)&s_driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &s_driver_obj.client_hdl));

    // Subscribe this task to the task watchdog. The class_driver loop is
    // intentionally hot — the ringbuffer is constantly draining and we don't
    // want to add an arbitrary vTaskDelay just to keep IDLE0 alive. Reset
    // the watchdog explicitly each iteration instead.
    esp_err_t wdt_rc = esp_task_wdt_add(NULL);
    if (wdt_rc != ESP_OK && wdt_rc != ESP_ERR_INVALID_ARG) {
        ESP_LOGW(TAG, "esp_task_wdt_add returned %d (%s)", wdt_rc, esp_err_to_name(wdt_rc));
    }

    uint32_t out_block_size = 16 * 1024;
    // The raw + converted buffers are owned by ingest_core1 (ping-pong on
    // Core 1). class_driver acquires raw buffers via ingest_core1_acquire_raw
    // and consumes converted buffers via ingest_core1_take_converted.

    // Ping-pong steady-state book-keeping. We start by reading into slot 0;
    // the matching DSP feed for slot 0 happens AFTER slot 1 has been
    // dispatched (one-cycle pipeline). prev_dsp_slot tracks which slot the
    // DSP should next consume; it's -1 on the very first iteration.
    int prev_dsp_slot = -1;
    
    uint64_t total_bytes = 0;
    int64_t start_time = esp_timer_get_time();
    int64_t last_report = start_time;

    // Per-window counters for diagnostic reporting (reset each 1s window).
    uint64_t bytes_window = 0;          // USB bytes received in this window
    uint32_t feed_calls_window = 0;     // dsp_processor_feed calls in this window
    uint64_t dsp_total_time_us = 0;     // sum of dsp_processor_feed wall time
    uint32_t dsp_frame_count = 0;       // FFT frames processed in this window
    // Core 0 cycle stage breakdown. Convert and push happen on Core 1
    // (ingest task) post-Step 5; only read and feed live here now.
    uint64_t cycle_read_us = 0;
    uint64_t cycle_handle_events_us = 0;  // time blocked in usb_host_client_handle_events
    uint64_t cycle_take_converted_us = 0; // time blocked in ingest_core1_take_converted
    uint32_t cycle_iterations = 0;
    int64_t last_idle_log = esp_timer_get_time();
    int64_t last_taskdump = esp_timer_get_time();
    int64_t last_recovery_us = esp_timer_get_time();
    int recovery_attempts = 0;
    const int MAX_RECOVERY_ATTEMPTS = 3;
    const int64_t RECOVERY_INTERVAL_US = 6 * 1000000;

    while (1)
    {
        // Reset task watchdog. The loop runs hot (no vTaskDelay) because the
        // ringbuffer is constantly draining; without this reset the IDLE0
        // task would never get to run and TWDT would trigger every 5 s.
        esp_task_wdt_reset();

        int64_t t_he0 = esp_timer_get_time();
        usb_host_client_handle_events(s_driver_obj.client_hdl, 10);
        cycle_handle_events_us += (uint64_t)(esp_timer_get_time() - t_he0);
        cycle_iterations++;

        // Periodic status / recovery watchdog while no device is open.
        if (s_driver_obj.dev_addr == 0) {
            int64_t now_us = esp_timer_get_time();
            if (now_us - last_idle_log >= 5 * 1000000) {
                ESP_LOGI(TAG, "class_driver waiting: dev_addr=%u actions=0x%02lx (no device enumerated yet)",
                         s_driver_obj.dev_addr, (unsigned long)s_driver_obj.actions);
                last_idle_log = now_us;
            }
            // If nothing has enumerated for RECOVERY_INTERVAL_US, cycle root port power.
            // This forces SOFs to stop and re-evaluates attach state, which recovers
            // most stuck-device cases without requiring physical unplug. Capped at
            // MAX_RECOVERY_ATTEMPTS so we don't loop forever if the hardware is
            // genuinely broken.
            if (recovery_attempts < MAX_RECOVERY_ATTEMPTS &&
                now_us - last_recovery_us >= RECOVERY_INTERVAL_US) {
                ESP_LOGW(TAG, "Recovery: cycling root port power (attempt %d/%d)",
                         recovery_attempts + 1, MAX_RECOVERY_ATTEMPTS);
                esp_err_t r = usb_host_lib_set_root_port_power(false);
                ESP_LOGW(TAG, "  power(false) -> 0x%x (%s)", r, esp_err_to_name(r));
                vTaskDelay(pdMS_TO_TICKS(500));
                r = usb_host_lib_set_root_port_power(true);
                ESP_LOGW(TAG, "  power(true)  -> 0x%x (%s)", r, esp_err_to_name(r));
                recovery_attempts++;
                last_recovery_us = esp_timer_get_time();
            }
        } else {
            // Reset the recovery counter on successful enumeration so we can
            // recover again from a future hot-disconnect.
            recovery_attempts = 0;
            last_recovery_us = esp_timer_get_time();
        }

        if (s_driver_obj.actions & ACTION_OPEN_DEV) action_open_dev(&s_driver_obj);
        if (s_driver_obj.actions & ACTION_START_STREAM) action_start_stream(&s_driver_obj);
        if (s_driver_obj.actions & ACTION_CLOSE_DEV) action_close_dev(&s_driver_obj);
        if (s_driver_obj.actions & ACTION_EXIT) break;

        // The ping-pong infrastructure (ingest task, semaphores, slot
        // buffers) is only created in action_start_stream. Skip the
        // ping-pong path until streaming is active so we don't take a
        // NULL semaphore.
        if (s_driver_obj.dev_addr == 0 || rtldev == NULL) {
            continue;
        }

        // Ping-pong path: read USB into a Core-1-owned raw buffer, dispatch
        // it to the ingest task for convert+push, then consume the previous
        // cycle's converted slot via DSP. Core 1 (ingest) and Core 0 (DSP
        // feed) overlap, which collapses the per-cycle wall time on Core 0
        // from "read + convert + push + feed" to "read + feed".
        int slot_for_read;
        uint8_t *raw = ingest_core1_acquire_raw(&slot_for_read);

        size_t n_read = 0;
        int64_t t_read_start = esp_timer_get_time();
        int read_ok = esp_libusb_read_stream(raw, out_block_size, &n_read, 0);
        int64_t t_read_end = esp_timer_get_time();

        if (read_ok == 0) {
            cycle_read_us += (uint64_t)(t_read_end - t_read_start);
            total_bytes += n_read;
            bytes_window += n_read;

            // Hand the freshly-filled raw buffer to ingest on Core 1.
            // Convert + push happen there; we don't block on completion.
            ingest_core1_dispatch(slot_for_read, n_read);

            // If we have a previous slot in flight, consume it now via DSP.
            // Wait for Core 1's ingest to mark it ready (typically immediate
            // — ingest is faster than feed, so by the time we need the data
            // it's already been converted + signal_buffer_pushed).
            if (prev_dsp_slot >= 0) {
                size_t n_int16 = 0;
                int64_t t_tc0 = esp_timer_get_time();
                int16_t *converted = ingest_core1_take_converted(prev_dsp_slot, &n_int16);
                cycle_take_converted_us += (uint64_t)(esp_timer_get_time() - t_tc0);
                int64_t t_pre_feed = esp_timer_get_time();

                dsp_processor_feed(converted, n_int16 / 2);
                int64_t t_post_feed = esp_timer_get_time();
                dsp_total_time_us += (uint64_t)(t_post_feed - t_pre_feed);
                dsp_frame_count += (n_int16 / 2) / 2048;
                feed_calls_window++;

                // Mark the slot free so ingest can reuse it next cycle.
                ingest_core1_release(prev_dsp_slot);
            }

            prev_dsp_slot = slot_for_read;
        } else {
            // No data this iteration. Release the slot we just acquired so
            // ingest can reuse it (we never dispatched).
            ingest_core1_release(slot_for_read);
        }

        // Producer-side ringbuffer fill is tracked inside esp_libusb's
        // streaming callback (USB-RB log line). The consumer-side HWM
        // sampled here previously was biased low (taken right after a
        // read drained 16 KB) so it has been removed.

        int64_t now = esp_timer_get_time();
        if (now - last_report >= 1000000) {
            // Per-second snapshot. Build a status_snapshot_t on the stack
            // (~150 bytes), pull all the accumulators (each getter resets
            // its internal state), and post to the logger task on Core 1.
            // Posting is non-blocking — if the queue is full, this snapshot
            // is silently dropped. The actual ESP_LOGI / printf / UART
            // formatting work happens on Core 1 at low priority, completely
            // off Core 0's hot read-feed loop.
            //
            // Why this matters: prior versions did the formatting inline
            // here. Measured cost: ~5-10 ms of Core 0 stall per second,
            // which let the 512 KB USB ringbuffer fill past 480 KB and
            // produced exactly 7 rb_full_drops/sec. With the offload,
            // drops go to 0.
            status_snapshot_t snap = {0};
            snap.window_us         = now - last_report;
            snap.elapsed_us        = now - start_time;
            snap.bytes_window      = bytes_window;
            snap.total_bytes       = total_bytes;
            snap.feed_calls_window = feed_calls_window;
            snap.dsp_total_time_us = dsp_total_time_us;
            snap.dsp_frame_count   = dsp_frame_count;
            snap.cycle_read_us     = cycle_read_us;
            snap.cycle_handle_events_us = cycle_handle_events_us;
            snap.cycle_take_converted_us = cycle_take_converted_us;
            snap.cycle_iterations  = cycle_iterations;
            snap.psram_free_bytes  = (int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            esp_libusb_get_stream_stats(&snap.us);
            dsp_processor_get_stage_stats(&snap.dsp);
            ingest_core1_get_stats(&snap.ingest);
            worker_core1_get_stats(&snap.ws);
            (void)status_logger_post(&snap);

#if CONFIG_DIAG_TASK_DUMP
            // Per-task / per-core CPU usage dump every 5 s. Heavy call
            // (allocates, snapshots all tasks, N+1 log lines). Gated
            // behind CONFIG_DIAG_TASK_DUMP (default off); enable via
            // menuconfig when actively debugging task-affinity issues.
            if (now - last_taskdump >= 5 * 1000000) {
                last_taskdump = now;
                UBaseType_t n = uxTaskGetNumberOfTasks();
                TaskStatus_t *ts = malloc(n * sizeof(TaskStatus_t));
                if (ts) {
                    uint32_t total_run = 0;
                    n = uxTaskGetSystemState(ts, n, &total_run);
                    ESP_LOGI(TAG, "Tasks (run-time since boot, %% of total):");
                    for (UBaseType_t i = 0; i < n; i++) {
                        uint32_t pct = (total_run > 0)
                            ? (uint32_t)((100ULL * ts[i].ulRunTimeCounter) / total_run)
                            : 0;
                        ESP_LOGI(TAG, "  %-16s pri=%u state=%d run=%lu (%lu%%) stack_hwm=%lu",
                                 ts[i].pcTaskName, (unsigned)ts[i].uxCurrentPriority,
                                 (int)ts[i].eCurrentState,
                                 (unsigned long)ts[i].ulRunTimeCounter,
                                 (unsigned long)pct,
                                 (unsigned long)ts[i].usStackHighWaterMark);
                    }
                    free(ts);
                }
            }
#endif // CONFIG_DIAG_TASK_DUMP

            last_report = now;
            bytes_window = 0;
            feed_calls_window = 0;
            dsp_total_time_us = 0;
            dsp_frame_count = 0;
            cycle_read_us = 0;
            cycle_handle_events_us = 0;
            cycle_take_converted_us = 0;
            cycle_iterations = 0;
        }
    }

    ESP_LOGI(TAG, "Deregistering Client");
    usb_host_client_deregister(s_driver_obj.client_hdl);
    xSemaphoreGive(signaling_sem);
    vTaskDelete(NULL);
}
