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
#include "signal_buffer.h"
#include "worker_core1.h"
#include "bch_decoder.h"
#include "rtl-sdr.h"

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
    ESP_LOGI(TAG, "Configuring RTL-SDR...");
    rtlsdr_set_sample_rate(rtldev, 2560000);
    rtlsdr_set_center_freq(rtldev, 1626000000);
    rtlsdr_set_tuner_gain_mode(rtldev, 0);
    rtlsdr_reset_buffer(rtldev);

    ESP_LOGI(TAG, "Initializing System Buffers...");
    signal_buffer_init();
    worker_core1_init();
    bch_decoder_init();

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
    uint8_t *buffer = malloc(out_block_size);
    // convert_buf is the source for the AXI-GDMA push into PSRAM. Allocate
    // in DMA-capable internal SRAM with 64-byte cache-line alignment.
    int16_t *convert_buf = heap_caps_aligned_alloc(64, out_block_size * sizeof(int16_t),
                                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    
    uint64_t total_bytes = 0;
    int64_t start_time = esp_timer_get_time();
    int64_t last_report = start_time;

    // Per-window counters for diagnostic reporting (reset each 1s window).
    uint64_t bytes_window = 0;          // USB bytes received in this window
    uint32_t feed_calls_window = 0;     // dsp_processor_feed calls in this window
    uint64_t dsp_total_time_us = 0;     // sum of dsp_processor_feed wall time
    uint32_t dsp_frame_count = 0;       // FFT frames processed in this window
    size_t max_used_rb = 0;             // peak buffer fill observed (consumer-side)
    // Consumer-cycle stage breakdown (sum of microseconds in each stage).
    // Per-cycle = (read_us + convert_us + push_us + feed_us). Helps locate
    // which stage is the throughput bottleneck.
    uint64_t cycle_read_us = 0;
    uint64_t cycle_convert_us = 0;
    uint64_t cycle_push_us = 0;
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

        usb_host_client_handle_events(s_driver_obj.client_hdl, 10);

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

        // If streaming, process data
        size_t n_read = 0;
        int64_t t_read_start = esp_timer_get_time();
        int read_ok = esp_libusb_read_stream(buffer, out_block_size, &n_read, 0);
        int64_t t_read_end = esp_timer_get_time();
        if (read_ok == 0) {
            cycle_read_us += (uint64_t)(t_read_end - t_read_start);
            total_bytes += n_read;
            bytes_window += n_read;

            // Convert RTL-SDR uint8 (DC=128) -> int16 Q15 (DC=0).
            // Original: out[i] = ((int16_t)b[i] - 128) << 8
            // Simplified algebraically (identical for all 0..255):
            //          out[i] = (int16_t)((b[i] << 8) ^ 0x8000)
            // Manually unrolled 4x and reading 32 bits at a time so the
            // compiler can combine into wider loads/stores. buffer and
            // convert_buf are both 64-byte aligned (DMA-capable internal SRAM)
            // so the wide loads are safe.
            const uint8_t  *__restrict src = buffer;
            int16_t        *__restrict dst = convert_buf;
            size_t i = 0;
            // Round n_read down to a multiple of 4; n_read is always 16384
            // in our setup but tail loop is kept for safety.
            size_t n4 = n_read & ~(size_t)3;
            for (; i < n4; i += 4) {
                uint32_t b4 = *(const uint32_t *)(src + i);
                dst[i + 0] = (int16_t)((((b4 >>  0) & 0xff) << 8) ^ 0x8000);
                dst[i + 1] = (int16_t)((((b4 >>  8) & 0xff) << 8) ^ 0x8000);
                dst[i + 2] = (int16_t)((((b4 >> 16) & 0xff) << 8) ^ 0x8000);
                dst[i + 3] = (int16_t)((((b4 >> 24) & 0xff) << 8) ^ 0x8000);
            }
            for (; i < n_read; i++) {
                dst[i] = (int16_t)(((src[i] << 8) ^ 0x8000));
            }
            int64_t t_convert = esp_timer_get_time();
            cycle_convert_us += (uint64_t)(t_convert - t_read_end);

            signal_buffer_push(convert_buf, n_read / 2);
            int64_t t_push = esp_timer_get_time();
            cycle_push_us += (uint64_t)(t_push - t_convert);

            dsp_processor_feed(convert_buf, n_read / 2);
            dsp_total_time_us += (esp_timer_get_time() - t_push);
            dsp_frame_count += (n_read / 2) / 2048;
            feed_calls_window++;

            // Sample ringbuffer fill AFTER read drained 16 KB. This is
            // a low-water-mark-ish view; the producer-side max in
            // usb_stream_stats_t is the authoritative peak.
            size_t used_rb, total_rb;
            esp_libusb_get_ringbuffer_info(&used_rb, &total_rb);
            if (used_rb > max_used_rb) max_used_rb = used_rb;
        }

        int64_t now = esp_timer_get_time();
        if (now - last_report >= 1000000) {
            double window_s = (now - last_report) / 1000000.0;
            double elapsed_s = (now - start_time) / 1000000.0;
            double rate_inst = (bytes_window / (1024.0 * 1024.0)) / window_s;
            double rate_avg  = (total_bytes / (1024.0 * 1024.0)) / elapsed_s;

            float avg_dsp_us = (dsp_frame_count > 0)
                ? (float)dsp_total_time_us / dsp_frame_count : 0;
            float feed_us_avg = (feed_calls_window > 0)
                ? (float)dsp_total_time_us / feed_calls_window : 0;
            // Consumer-side ringbuffer fill peak (% of 512 KB), sampled
            // immediately after each read. Will under-report relative to
            // the producer-side peak because consumer reads precisely the
            // moments that drain the buffer.
            float consumer_peak_pct = 100.0f * (float)max_used_rb / (512.0f * 1024.0f);

            // Pull diagnostic snapshots — every getter resets its accumulators.
            dsp_stage_stats_t dsp_st;
            dsp_processor_get_stage_stats(&dsp_st);
            worker_stats_t ws;
            worker_core1_get_stats(&ws);
            usb_stream_stats_t us;
            esp_libusb_get_stream_stats(&us);

            // USB transfer-level fill ratio (actual / requested) reveals device-side
            // throttling (short transfers) vs host-side back-pressure (rb_full_drops).
            float xfer_fill = (us.total_requested_bytes > 0)
                ? (100.0f * (float)us.total_actual_bytes / (float)us.total_requested_bytes)
                : 0.0f;

            // Consumer-cycle stage means (per call to dsp_processor_feed).
            float feed_n = (feed_calls_window > 0) ? (float)feed_calls_window : 1.0f;
            float read_us_avg    = (float)cycle_read_us    / feed_n;
            float convert_us_avg = (float)cycle_convert_us / feed_n;
            float push_us_avg    = (float)cycle_push_us    / feed_n;

            // Producer-side ringbuffer peak, in % of 512 KB.
            float producer_peak_pct = 100.0f * (float)us.producer_rb_max_used / (512.0f * 1024.0f);
            float drop_fill_pct     = 100.0f * (float)us.producer_rb_used_at_drop / (512.0f * 1024.0f);

            ESP_LOGI(TAG, "USB: rate_inst=%.2f MB/s rate_avg=%.2f MB/s feed_calls=%u "
                          "(avg_per_call=%.0f us) RB-fill_consumer=%.1f%% PSRAM_free=%d",
                     rate_inst, rate_avg, feed_calls_window, feed_us_avg,
                     consumer_peak_pct, heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

            ESP_LOGI(TAG, "USB-XFR: completed=%u short=%u (fill=%.1f%%) "
                          "rb_full_drops=%u status_err=%u resubmit_err=%u last_err=0x%02x",
                     us.completed, us.short_xfers, xfer_fill,
                     us.rb_full_drops, us.status_errors, us.resubmit_errors,
                     us.last_error_status);

            ESP_LOGI(TAG, "USB-RB:  producer_peak_fill=%.1f%% drop_fill=%.1f%% "
                          "(producer_samples=%u, consumer_peak_fill=%.1f%%)",
                     producer_peak_pct, drop_fill_pct, us.producer_samples,
                     consumer_peak_pct);

            ESP_LOGI(TAG, "Consumer-cycle (us avg): read=%.0f convert=%.0f push=%.0f feed=%.0f",
                     read_us_avg, convert_us_avg, push_us_avg, feed_us_avg);

            ESP_LOGI(TAG, "DSP: %u frames, total=%.0f us/frame "
                          "[wind=%.0f fft=%.0f mag=%.0f detect=%.0f base=%.0f]",
                     dsp_frame_count, avg_dsp_us,
                     dsp_st.wind_us, dsp_st.fft_us, dsp_st.mag_us,
                     dsp_st.detect_us, dsp_st.baseline_us);

            ESP_LOGI(TAG, "Worker: queued=%u dropped=%u processed=%u skipped=%u "
                          "qmax=%u avg_burst=%.0f us",
                     ws.bursts_queued, ws.bursts_dropped, ws.bursts_processed,
                     ws.bursts_skipped, ws.queue_high_water, ws.avg_burst_us);

            ESP_LOGI(TAG, "Worker-stages (us): extract=%.0f freq=%.0f fir=%.0f "
                          "resamp=%.0f demod=%.0f bch=%.0f",
                     ws.extract_us, ws.freq_center_us, ws.fir_decim_us,
                     ws.resample_us, ws.demod_us, ws.bch_us);

            // Per-task / per-core CPU usage dump every 5 seconds. Heavy call
            // (allocates an array, snapshots all task counters), so don't run
            // every second. Tells us conclusively which task is hot on which
            // core and whether Core 1 has spare capacity.
            if (now - last_taskdump >= 5 * 1000000) {
                last_taskdump = now;
                UBaseType_t n = uxTaskGetNumberOfTasks();
                TaskStatus_t *ts = malloc(n * sizeof(TaskStatus_t));
                if (ts) {
                    uint32_t total_run = 0;
                    n = uxTaskGetSystemState(ts, n, &total_run);
                    ESP_LOGI(TAG, "Tasks (run-time since boot, %% of total):");
                    // Core affinity isn't easily exposed by this IDF FreeRTOS
                    // build (xCoreID requires sdkconfig flag that needs full
                    // reconfigure; vTaskCoreAffinityGet not in this variant).
                    // Task names are descriptive (class is on C0, worker_core1
                    // on C1, IDLE0/IDLE1 on their respective cores) so just
                    // print name + %CPU.
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

            last_report = now;
            bytes_window = 0;
            feed_calls_window = 0;
            dsp_total_time_us = 0;
            dsp_frame_count = 0;
            max_used_rb = 0;
            cycle_read_us = 0;
            cycle_convert_us = 0;
            cycle_push_us = 0;
        }
    }

    ESP_LOGI(TAG, "Deregistering Client");
    usb_host_client_deregister(s_driver_obj.client_hdl);
    xSemaphoreGive(signaling_sem);
    vTaskDelete(NULL);
}
