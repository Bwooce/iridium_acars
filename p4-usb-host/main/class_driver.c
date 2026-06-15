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
#include "sd_capture.h"
#include "class_driver.h"
#include "bch_decoder.h"
#include "rtl-sdr.h"
#include "status_logger.h"
#include "sd_capture.h"

// Wideband detector handle (#120). class_driver is the owner task; it
// creates the one detector and threads this handle to feed/stats.
static dsp_processor_t *s_dsp = NULL;

#define CLIENT_NUM_EVENT_MSG 5

#define ACTION_OPEN_DEV 0x01
#define ACTION_GET_DEV_INFO 0x02
#define ACTION_GET_DEV_DESC 0x04
#define ACTION_GET_CONFIG_DESC 0x08
#define ACTION_GET_STR_DESC 0x10
#define ACTION_CLOSE_DEV 0x20
#define ACTION_EXIT 0x40
#define ACTION_START_STREAM 0x80

static const char   *TAG               = "CLASS";
static rtlsdr_dev_t *rtldev            = NULL;
static volatile int  s_last_gain_dbx10 = -1;

// Stall-forensics breadcrumb: the consumer loop sets this to its current
// stage; the health watchdog dumps it (class_driver_dump_stall_diag) right
// before rebooting a wedged stream, so we can see WHICH call the loop was
// stuck on (#105 root-cause). Stages are ordered by loop position.
enum { CS_TOP = 0,
       CS_HANDLE_EVENTS,
       CS_ACQUIRE,
       CS_READ,
       CS_DISPATCH,
       CS_TAKE_CONVERTED,
       CS_FEED,
       CS_RELEASE,
       CS_REPORT };
static const char *const k_class_stage_name[] = {
    "top", "handle_events", "acquire_raw", "read_stream", "dispatch",
    "take_converted", "dsp_feed", "release", "report"};
static volatile uint8_t  s_class_stage    = CS_TOP;
static volatile uint64_t s_class_iter     = 0; // loop iterations
static volatile int64_t  s_class_stage_us = 0; // when the stage was entered

static inline void class_stage(uint8_t s)
{
    s_class_stage    = s;
    s_class_stage_us = esp_timer_get_time();
}

// Forensic dump for a wedged USB stream — called by the health watchdog
// (wifi_link.c) just before it esp_restart()s, so every auto-recovery leaves
// a trace on serial. Shows which loop stage `class` is stuck on (and for how
// long), the USB transfer totals, and every task's state + stack high-water.
// Safe to call from any task (reads counters, mallocs, logs — no flash ops).
void class_driver_dump_stall_diag(void)
{
    unsigned st = s_class_stage;
    if (st >= sizeof(k_class_stage_name) / sizeof(k_class_stage_name[0])) st = 0;
    int64_t stuck_ms = (esp_timer_get_time() - s_class_stage_us) / 1000;

    usb_stream_totals_t ut = {0};
    esp_libusb_get_stream_totals(&ut);
    ESP_LOGW(TAG, "STALL DIAG: class stage=%s for %lld ms, iter=%llu | "
                  "USB completed=%llu status_err=%llu short=%llu rb_drops=%llu",
             k_class_stage_name[st], (long long)stuck_ms,
             (unsigned long long)s_class_iter,
             (unsigned long long)ut.completed, (unsigned long long)ut.status_errors,
             (unsigned long long)ut.short_xfers, (unsigned long long)ut.rb_full_drops);

    UBaseType_t   n  = uxTaskGetNumberOfTasks();
    TaskStatus_t *ts = malloc(n * sizeof(TaskStatus_t));
    if (ts) {
        n = uxTaskGetSystemState(ts, n, NULL);
        for (UBaseType_t i = 0; i < n; i++) {
            // eCurrentState: 0=Running 1=Ready 2=Blocked 3=Suspended 4=Deleted
            ESP_LOGW(TAG, "  task %-16s state=%d pri=%u stack_hwm=%lu",
                     ts[i].pcTaskName, (int)ts[i].eCurrentState,
                     (unsigned)ts[i].uxCurrentPriority,
                     (unsigned long)ts[i].usStackHighWaterMark);
        }
        free(ts);
    }
}

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
    switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI(TAG, "New USB device connected at address %d", event_msg->new_dev.address);
        if (driver_obj->dev_addr == 0) {
            driver_obj->dev_addr = event_msg->new_dev.address;
            driver_obj->actions |= ACTION_OPEN_DEV;
        }
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGI(TAG, "USB device gone");
        if (driver_obj->dev_hdl != NULL) {
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
    // The pipeline inits can fail (ESP_ERR_NO_MEM). Proceeding with a
    // half-built pipeline either crashes (acquire on a NULL semaphore)
    // or runs silently dead (signal_buffer_push no-ops without its
    // ring). Treat any failure as fatal for streaming: close the
    // device path and leave the stream NOT started — the health
    // watchdog / operator can see the error instead of a zombie.
    esp_err_t init_rc;
    if ((init_rc = signal_buffer_init()) != ESP_OK ||
        (init_rc = worker_core1_init()) != ESP_OK ||
        (init_rc = ingest_core1_init()) != ESP_OK) {
        ESP_LOGE(TAG, "pipeline init failed (%s) — stream NOT started",
                 esp_err_to_name(init_rc));
        driver_obj->actions &= ~ACTION_START_STREAM;
        return;
    }
    bch_decoder_init();
    if (frame_decoder_init() != ESP_OK) {
        ESP_LOGW(TAG, "frame_decoder_init failed; higher-layer "
                      "classification will be silently skipped");
    }

    ESP_LOGI(TAG, "Initializing DSP...");
    s_dsp = dsp_processor_create(worker_core1_push_burst);
    if (!s_dsp) {
        ESP_LOGE(TAG, "dsp_processor_create failed");
    }

    ESP_LOGI(TAG, "Starting Async Stream...");
    esp_libusb_start_stream(driver_obj, 0x81);

    // Grab the SD-capture writer's DMA-INT scratch AFTER tagger AND
    // USB pool have taken their slices. Both are load-bearing for
    // live decode, so they get first dibs. If this fails (heap too
    // fragmented), SD capture refuses to start but everything else
    // stays operational. esp_libusb_start_stream submits URBs and
    // returns; URB recycling churn ramps up over the next seconds
    // so allocating right here gives us the cleanest window.
    sd_capture_alloc_writer_buf();

    driver_obj->actions &= ~ACTION_START_STREAM;
}

static void action_close_dev(class_driver_t *driver_obj)
{
    ESP_LOGI(TAG, "Closing device");
    if (rtldev) {
        rtlsdr_close(rtldev);
        rtldev = NULL;
    }
    driver_obj->dev_hdl  = NULL;
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
        .is_synchronous    = false,
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async             = {
                        .client_event_callback = client_event_cb,
                        .callback_arg          = (void *)&s_driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &s_driver_obj.client_hdl));

    // class_driver does NOT subscribe to TASK_WDT. We tried it
    // (added then removed) — when no USB device enumerates within
    // the 60 s WDT window, OR when ingest blocks unbounded waiting
    // for the device to start producing, class trips the WDT and
    // reboots. The interesting failure is the device hardware (a
    // wedged controller, a missing dongle); a panic-reboot doesn't
    // fix that and just hides the diagnostic.
    //
    // Real safety: the daemon's own root-port-power cycling
    // (action_open_dev / Recovery) handles a wedged USB controller,
    // and the in-loop "stream-stall watchdog" below catches the
    // post-enumeration case where bytes_window stalls to zero.

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
    int64_t  start_time  = esp_timer_get_time();
    int64_t  last_report = start_time;

    // Per-window counters for diagnostic reporting (reset each 1s window).
    uint64_t bytes_window      = 0; // USB bytes received in this window
    uint32_t feed_calls_window = 0; // dsp_processor_feed calls in this window
    uint64_t dsp_total_time_us = 0; // sum of dsp_processor_feed wall time
    uint32_t dsp_frame_count   = 0; // FFT frames processed in this window
    // Core 0 cycle stage breakdown. Convert and push happen on Core 1
    // (ingest task) post-Step 5; only read and feed live here now.
    uint64_t      cycle_read_us           = 0;
    uint64_t      cycle_handle_events_us  = 0; // time blocked in usb_host_client_handle_events
    uint64_t      cycle_take_converted_us = 0; // time blocked in ingest_core1_take_converted
    uint32_t      cycle_iterations        = 0;
    int64_t       last_idle_log           = esp_timer_get_time();
    int64_t       last_taskdump           = esp_timer_get_time();
    int64_t       last_recovery_us        = esp_timer_get_time();
    int           recovery_attempts       = 0;
    const int     MAX_RECOVERY_ATTEMPTS   = 3;
    const int64_t RECOVERY_INTERVAL_US    = 6 * 1000000;

    // Stream-stall watchdog (task #72): when a device IS enumerated but
    // USB bytes_window stays effectively zero across several seconds,
    // the controller / endpoint has wedged. Cycle the root port power
    // to force re-attach (same recovery as the no-device path above).
    int            stall_seconds         = 0;
    const int      STALL_TRIGGER_SECONDS = 5;          // tolerate brief noise dips
    const uint64_t STALL_BYTES_FLOOR     = 100 * 1024; // <100 KB/s is "stuck", not "quiet"
    int            stall_recoveries      = 0;
    const int      MAX_STALL_RECOVERIES  = 3;

    while (1) {
        // Reset task watchdog. The loop runs hot (no vTaskDelay) because the
        // ringbuffer is constantly draining; without this reset the IDLE0
        // task would never get to run and TWDT would trigger every 5 s.
        // (No esp_task_wdt_reset — class isn't WDT-subscribed; see init.)

        class_stage(CS_HANDLE_EVENTS);
        int64_t t_he0 = esp_timer_get_time();
        // NB: the timeout argument is in TICKS — 10 ticks = 100 ms at
        // the default 100 Hz tick, not the 10 ms some older comments
        // claimed. Deliberately kept: during streaming the client event
        // queue wakes this far sooner, and the long cap keeps the idle
        // loop cheap between events. Don't "fix" to pdMS_TO_TICKS(10)
        // without re-measuring the no-device idle load.
        usb_host_client_handle_events(s_driver_obj.client_hdl, 10);
        cycle_handle_events_us += (uint64_t)(esp_timer_get_time() - t_he0);
        cycle_iterations++;
        s_class_iter++;

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
            last_recovery_us  = esp_timer_get_time();
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
        class_stage(CS_ACQUIRE);
        uint8_t *raw = ingest_core1_acquire_raw(&slot_for_read);

        size_t  n_read       = 0;
        int64_t t_read_start = esp_timer_get_time();
        class_stage(CS_READ);
        int     read_ok    = esp_libusb_read_stream(raw, out_block_size, &n_read, 0);
        int64_t t_read_end = esp_timer_get_time();

        if (read_ok == 0) {
            // Tripwire: an odd-length read would silently invert I/Q
            // pairing for the REST OF THE STREAM (ingest floors n/2;
            // the next read then starts on a Q byte). Never observed —
            // transfers are 16 KB multiples — but if it ever fires we
            // want the log line, not weeks of "demod mysteriously dead".
            if (n_read & 1) {
                static bool s_odd_read_logged = false;
                if (!s_odd_read_logged) {
                    s_odd_read_logged = true;
                    ESP_LOGE(TAG, "ODD-LENGTH USB read (%zu B) — I/Q "
                                  "pairing now suspect until next stream restart",
                             n_read);
                }
            }
            cycle_read_us += (uint64_t)(t_read_end - t_read_start);
            total_bytes += n_read;
            bytes_window += n_read;

            // Optional raw IQ capture (#63). Fast no-op when no
            // capture is active; otherwise copies n_read bytes
            // into a PSRAM stream buffer (non-blocking, drops on
            // overflow). Tap is here — pre-dispatch — so we
            // capture the exact uint8 payload before any conversion.
            sd_capture_write(raw, n_read);

            // Hand the freshly-filled raw buffer to ingest on Core 1.
            // Convert + push happen there; we don't block on completion.
            ingest_core1_dispatch(slot_for_read, n_read);

            // If we have a previous slot in flight, consume it now via DSP.
            // Wait for Core 1's ingest to mark it ready (typically immediate
            // — ingest is faster than feed, so by the time we need the data
            // it's already been converted + signal_buffer_pushed).
            if (prev_dsp_slot >= 0) {
                size_t  n_int16 = 0;
                int64_t t_tc0   = esp_timer_get_time();
                class_stage(CS_TAKE_CONVERTED);
                int16_t *converted = ingest_core1_take_converted(prev_dsp_slot, &n_int16);
                cycle_take_converted_us += (uint64_t)(esp_timer_get_time() - t_tc0);
                int64_t t_pre_feed = esp_timer_get_time();

                class_stage(CS_FEED);
                dsp_processor_feed(s_dsp, converted, n_int16 / 2);
                int64_t t_post_feed = esp_timer_get_time();
                dsp_total_time_us += (uint64_t)(t_post_feed - t_pre_feed);
                dsp_frame_count += (n_int16 / 2) / 2048;
                feed_calls_window++;

                // Mark the slot free so ingest can reuse it next cycle.
                ingest_core1_release(prev_dsp_slot);
            }

            prev_dsp_slot = slot_for_read;
        } else {
            // No data this iteration. Before releasing the slot we just
            // acquired, drain any in-flight slot from the previous cycle.
            // Skipping this deadlocked the loop: with prev_dsp_slot's
            // s_free still held and the rotation pointer already advanced
            // past the slot we're releasing, the next acquire_raw blocks
            // forever on s_free[prev_dsp_slot] — which only this task can
            // give, after a take_converted it can no longer reach. Armed
            // exactly when the stream pauses (dongle hiccup / unplug /
            // quiet ring); the #105/#106 "stuck in acquire_raw" signature.
            if (prev_dsp_slot >= 0) {
                size_t   n_int16   = 0;
                int16_t *converted = ingest_core1_take_converted(prev_dsp_slot, &n_int16);
                dsp_processor_feed(s_dsp, converted, n_int16 / 2);
                ingest_core1_release(prev_dsp_slot);
                prev_dsp_slot = -1;
            }
            // Release the slot we just acquired so ingest can reuse it
            // (we never dispatched).
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
            status_snapshot_t snap       = {0};
            snap.window_us               = now - last_report;
            snap.elapsed_us              = now - start_time;
            snap.bytes_window            = bytes_window;
            snap.total_bytes             = total_bytes;
            snap.feed_calls_window       = feed_calls_window;
            snap.dsp_total_time_us       = dsp_total_time_us;
            snap.dsp_frame_count         = dsp_frame_count;
            snap.cycle_read_us           = cycle_read_us;
            snap.cycle_handle_events_us  = cycle_handle_events_us;
            snap.cycle_take_converted_us = cycle_take_converted_us;
            snap.cycle_iterations        = cycle_iterations;
            snap.psram_free_bytes        = (int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            esp_libusb_get_stream_stats(&snap.us);
            dsp_processor_get_stage_stats(s_dsp, &snap.dsp);
            ingest_core1_get_stats(&snap.ingest);
            worker_core1_get_stats(&snap.ws);
            (void)status_logger_post(&snap);

#if CONFIG_DIAG_TASK_DUMP
            // Per-task / per-core CPU usage dump every 5 s. Heavy call
            // (allocates, snapshots all tasks, N+1 log lines). Gated
            // behind CONFIG_DIAG_TASK_DUMP (default off); enable via
            // menuconfig when actively debugging task-affinity issues.
            if (now - last_taskdump >= 5 * 1000000) {
                last_taskdump    = now;
                UBaseType_t   n  = uxTaskGetNumberOfTasks();
                TaskStatus_t *ts = malloc(n * sizeof(TaskStatus_t));
                if (ts) {
                    uint32_t total_run = 0;
                    n                  = uxTaskGetSystemState(ts, n, &total_run);
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

            // Stream-stall watchdog (#72). Only relevant when a device
            // is enumerated AND we're past initial warm-up (start_time
            // + 3 s) so transient zero-byte windows during enumeration
            // don't trigger.
            if (s_driver_obj.dev_addr != 0 && (now - start_time) > 3 * 1000000) {
                if (bytes_window < STALL_BYTES_FLOOR) {
                    stall_seconds++;
                    ESP_LOGW(TAG, "stream stall #%d/%d (%llu B in last 1s, threshold %llu)",
                             stall_seconds, STALL_TRIGGER_SECONDS,
                             (unsigned long long)bytes_window,
                             (unsigned long long)STALL_BYTES_FLOOR);
                } else {
                    stall_seconds = 0;
                }
                if (stall_seconds >= STALL_TRIGGER_SECONDS &&
                    stall_recoveries < MAX_STALL_RECOVERIES) {
                    ESP_LOGE(TAG, "STREAM STALL: cycling root port power "
                                  "(recovery %d/%d)",
                             stall_recoveries + 1, MAX_STALL_RECOVERIES);
                    esp_err_t r = usb_host_lib_set_root_port_power(false);
                    ESP_LOGW(TAG, "  power(false) -> 0x%x (%s)", r, esp_err_to_name(r));
                    vTaskDelay(pdMS_TO_TICKS(500));
                    r = usb_host_lib_set_root_port_power(true);
                    ESP_LOGW(TAG, "  power(true)  -> 0x%x (%s)", r, esp_err_to_name(r));
                    stall_seconds = 0;
                    stall_recoveries++;
                }
                // NOTE: the esp_restart() escalation that used to live here
                // is gone — it could never fire when this loop itself blocked
                // (e.g. on the Core-1 ingest handoff), which is exactly how
                // the dongle-silent stall wedges the pipeline. Reboot recovery
                // now lives in the independent health watchdog (wifi_link.c,
                // health_wdt) which monitors usb.completed from OUTSIDE this
                // loop. The cheap root-port cycle above stays as an in-loop
                // first-try for the loop-still-alive case (#103/#105).
            } else {
                stall_seconds = 0;
                // Re-arm the stall watchdog when a device re-enumerates
                // — covers the "USB cable yanked and replugged" path.
                if (s_driver_obj.dev_addr != 0) stall_recoveries = 0;
            }

            last_report             = now;
            bytes_window            = 0;
            feed_calls_window       = 0;
            dsp_total_time_us       = 0;
            dsp_frame_count         = 0;
            cycle_read_us           = 0;
            cycle_handle_events_us  = 0;
            cycle_take_converted_us = 0;
            cycle_iterations        = 0;
        }
    }

    ESP_LOGI(TAG, "Deregistering Client");
    usb_host_client_deregister(s_driver_obj.client_hdl);
    xSemaphoreGive(signaling_sem);
    // Wait to be deleted by app_main (same pattern as the daemon task).
    // Self-deleting here raced app_main's vTaskDelete(handle) — the idle
    // task could free this TCB first, making that call a use-after-free.
    vTaskSuspend(NULL);
}
