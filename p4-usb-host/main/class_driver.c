/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "usb/usb_host.h"
#include "esp_libusb.h"
#include "usbring.h"
#include "dsp_processor.h"
#include "frame_decoder.h"
#include "aggregator_ingest.h"
#include "signal_buffer.h"
#include "worker_core1.h"
#include "ingest_core1.h"
#include "resample_256_to_250.h"
#include "scanner.h"
#include "fft_sc16_2048.h"
#include "uw_correlator.h"
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
#define ACTION_RETUNE 0x100

static const char   *TAG               = "CLASS";
static rtlsdr_dev_t *rtldev            = NULL;
static volatile int  s_last_gain_dbx10 = -1;

// Task 7 (stream-pause retune): the actual rtlsdr_set_center_freq() call
// must run on usb_pump (this task), not the caller — it shares the
// control-transfer wait loop's usb_host_client_handle_events() call with
// the pump loop, and the retune's control transfers must not contend
// with in-flight bulk URBs (that contention is what wedges Approach A).
// class_driver_retune() posts the request via ACTION_RETUNE and blocks
// on s_retune_done; the pump loop below does the pause/retune/resume and
// gives the semaphore.
static volatile uint32_t s_pending_retune_hz = 0;
static SemaphoreHandle_t s_retune_done       = NULL;
static volatile bool     s_last_retune_ok    = false;

// T48 (docs/perf-decoupling-design-2026-07-04.md §T48): the combined
// class_driver loop is split into two Core-0 tasks:
//   - usb_pump: this function (class_driver_task) keeps its name/entry
//     point for the extern declarations in usb_host_lib_main.c /
//     smoke_test.c, but its body now only does
//     usb_host_client_handle_events (URB completion -> ring write ->
//     resubmit), enumeration actions, root-port recovery, the no-device
//     idle path, and the 1 Hz snapshot/stall watchdog.
//   - dsp_feed: a new task (dsp_feed_task, below) that runs the stream
//     cycle -- usbring peek/consume, ingest_core1 dispatch/take/release,
//     dsp_processor_feed -- moved VERBATIM (same call order, same
//     blocking semantics) from the old combined loop.
// Stall-forensics breadcrumbs are now two independent sets (pump-side,
// feed-side) since either task can be the one that's wedged. Both are
// dumped together by class_driver_dump_stall_diag().
enum { CS_PUMP_TOP = 0,
       CS_PUMP_HANDLE_EVENTS,
       CS_PUMP_REPORT };
static const char *const k_pump_stage_name[] = {
    "top", "handle_events", "report"};

enum { CS_FEED_TOP = 0,
       CS_FEED_WAIT_DATA, // NEW: blocked on ring-empty (notify or timeout)
       CS_FEED_ACQUIRE,
       CS_FEED_RAW_DONE, // T49a: wait+consume the previous dispatch's ring span
       CS_FEED_READ,
       CS_FEED_DISPATCH,
       CS_FEED_TAKE_CONVERTED,
       CS_FEED_DSP_FEED,
       CS_FEED_RELEASE };
static const char *const k_feed_stage_name[] = {
    "top", "wait_data", "acquire_slot", "raw_done_consume", "read_stream",
    "dispatch", "take_converted", "dsp_feed", "release"};

static _Atomic(uint8_t)  s_pump_stage    = CS_PUMP_TOP;
static _Atomic(uint64_t) s_pump_iter     = 0; // loop iterations
static _Atomic(int64_t)  s_pump_stage_us = 0; // when the stage was entered

static _Atomic(uint8_t)  s_feed_stage    = CS_FEED_TOP;
static _Atomic(uint64_t) s_feed_iter     = 0; // loop iterations
static _Atomic(int64_t)  s_feed_stage_us = 0; // when the stage was entered

static inline void pump_stage(uint8_t s)
{
    atomic_store_explicit(&s_pump_stage, s, memory_order_relaxed);
    atomic_store_explicit(&s_pump_stage_us, esp_timer_get_time(), memory_order_relaxed);
}

static inline void feed_stage(uint8_t s)
{
    atomic_store_explicit(&s_feed_stage, s, memory_order_relaxed);
    atomic_store_explicit(&s_feed_stage_us, esp_timer_get_time(), memory_order_relaxed);
}

// T48: diagnostics the feeder accumulates and the pump's 1 Hz snapshot
// reads (read-and-reset via atomic_exchange, except total_bytes which is
// a lifetime, non-resetting counter). Before the split these were plain
// locals inside the one combined loop; now the writer (dsp_feed) and
// reader (usb_pump) are different tasks, so they need real cross-task
// visibility. Relaxed ordering matches the codebase's existing
// diagnostic-counter idiom (usbring.c, sd_capture.c) -- these feed
// status reporting only, not the decode path.
static _Atomic(uint64_t) s_feed_bytes_window            = 0;
static _Atomic(uint64_t) s_feed_total_bytes             = 0;
static _Atomic(uint32_t) s_feed_calls_window            = 0;
static _Atomic(uint64_t) s_feed_dsp_total_time_us       = 0;
static _Atomic(uint32_t) s_feed_dsp_frame_count         = 0;
static _Atomic(uint64_t) s_feed_cycle_read_us           = 0;
static _Atomic(uint64_t) s_feed_cycle_take_converted_us = 0;
static _Atomic(uint32_t) s_feed_cycle_iterations        = 0;

// T48 feeder lifecycle. Created once in action_start_stream() (after the
// ping-pong infra + ring it depends on already exist), stopped from
// action_close_dev() before this task (usb_pump) deregisters and parks.
// See stop_dsp_feed_task() for the shutdown sequence.
static TaskHandle_t      s_dsp_feed_task_hdl   = NULL;
static SemaphoreHandle_t s_feed_stopped_sem    = NULL;
static volatile bool     s_feed_stop_requested = false;

static void dsp_feed_task(void *arg);

// Ask dsp_feed to stop and wait (bounded) for its acknowledgement. Safe to
// call multiple times (idempotent once s_dsp_feed_task_hdl is NULL). Must
// run on usb_pump, BEFORE it deregisters the USB client -- see
// action_close_dev().
static void stop_dsp_feed_task(void)
{
    if (!s_dsp_feed_task_hdl) return;
    s_feed_stop_requested = true;
    // Wake it immediately if it's blocked in CS_FEED_WAIT_DATA; harmless
    // if it's busy elsewhere in the protocol (it'll notice the flag the
    // next time it reaches the top of its loop).
    xTaskNotifyGive(s_dsp_feed_task_hdl);
    if (s_feed_stopped_sem &&
        xSemaphoreTake(s_feed_stopped_sem, pdMS_TO_TICKS(2000)) != pdTRUE) {
        // dsp_feed is most likely blocked inside the ingest_core1 slot
        // protocol's unbounded-in-effect retry loops (wait_raw_done /
        // take_converted) -- those only return once ingest_core1
        // progresses, which is exactly the pre-existing #106/#110
        // failure mode health_wdt already watches for from outside.
        // Proceed with pump teardown regardless: the device is gone
        // either way, and today's action_close_dev() doesn't actually
        // free anything the feeder could still be touching (rtlsdr_close
        // is a stub -- see T19 in docs/review-2026-07-04-findings.md).
        ESP_LOGW(TAG, "dsp_feed did not confirm stop within 2 s "
                      "(likely blocked in the ingest slot protocol) -- "
                      "proceeding with pump teardown");
    }
    s_dsp_feed_task_hdl = NULL;
    usbring_set_consumer_task(NULL);
}

// Forensic dump for a wedged USB stream — called by the health watchdog
// (wifi_link.c) just before it esp_restart()s, so every auto-recovery leaves
// a trace on serial. Shows which loop stage `usb_pump` AND `dsp_feed` are
// each stuck on (and for how long), the USB transfer totals, and every
// task's state + stack high-water. Safe to call from any task (reads
// counters, mallocs, logs — no flash ops).
void class_driver_dump_stall_diag(void)
{
    unsigned pump_st = atomic_load_explicit(&s_pump_stage, memory_order_relaxed);
    if (pump_st >= sizeof(k_pump_stage_name) / sizeof(k_pump_stage_name[0])) pump_st = 0;
    int64_t pump_stuck_ms = (esp_timer_get_time() -
                             atomic_load_explicit(&s_pump_stage_us, memory_order_relaxed)) /
                            1000;

    unsigned feed_st = atomic_load_explicit(&s_feed_stage, memory_order_relaxed);
    if (feed_st >= sizeof(k_feed_stage_name) / sizeof(k_feed_stage_name[0])) feed_st = 0;
    int64_t feed_stuck_ms = (esp_timer_get_time() -
                             atomic_load_explicit(&s_feed_stage_us, memory_order_relaxed)) /
                            1000;

    usb_stream_totals_t ut = {0};
    esp_libusb_get_stream_totals(&ut);
    ESP_LOGW(TAG, "STALL DIAG: usb_pump stage=%s for %lld ms (iter=%llu) | "
                  "dsp_feed stage=%s for %lld ms (iter=%llu) | "
                  "USB completed=%llu status_err=%llu short=%llu rb_drops=%llu",
             k_pump_stage_name[pump_st], (long long)pump_stuck_ms,
             (unsigned long long)atomic_load_explicit(&s_pump_iter, memory_order_relaxed),
             k_feed_stage_name[feed_st], (long long)feed_stuck_ms,
             (unsigned long long)atomic_load_explicit(&s_feed_iter, memory_order_relaxed),
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

esp_err_t class_driver_retune(uint32_t hz)
{
    if (!rtldev) return ESP_ERR_INVALID_STATE;
    if (!s_retune_done) {
        s_retune_done = xSemaphoreCreateBinary();
        if (!s_retune_done) return ESP_FAIL;
    }
    s_pending_retune_hz = hz;
    s_driver_obj.actions |= ACTION_RETUNE;
    // wait up to 3 s for the pump task to complete the quiesced retune
    if (xSemaphoreTake(s_retune_done, pdMS_TO_TICKS(3000)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    return s_last_retune_ok ? ESP_OK : ESP_FAIL;
}

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
    // uw_correlator's PIE float-FFT scratch (16 KB) must be pinned in DRAM
    // here too: left to the worker's lazy first call it spills to RTCRAM
    // under DRAM pressure and silently mis-decodes (the ~95%->~6% cliff).
    uw_correlator_prealloc_pie_fft();
    // T49b: ingest_core1's convert+resample staging tile (4 KB) is also
    // a PIE-read buffer (resample_arp4.S's MAC input) -- pin it here too
    // while DRAM is plentiful. ingest_core1_init() re-checks and fails
    // fatally if this didn't land in DRAM.
    ingest_core1_prealloc_tile();
    // Same hazard, three more PIE FIR delay lines (T56): D13 envelope-LP
    // FIR + RRC I/Q FIRs (uw_correlator) and the wideband decim FIR I/Q
    // (worker_core1's s_decim). All three were still lazy-allocated on
    // the first burst, so their address depended on heap fragmentation
    // at that point -- pin them here too, while DRAM is still plentiful.
    uw_correlator_prealloc_fir();
    worker_core1_prealloc_fir();
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
#if CONFIG_DEVICE_ROLE_COMBINED_LOOPBACK
    // COMBINED: the worker emits PDUs to the frame_pdu queue instead of
    // calling frame_decoder_push directly (#135). Drain that queue back
    // into frame_decoder in-process so one board exercises the full
    // worker -> PDU -> aggregator -> decode path (#137).
    aggregator_ingest_init();
#endif

    ESP_LOGI(TAG, "Initializing DSP...");
    s_dsp = dsp_processor_create(worker_core1_push_burst);
    if (!s_dsp) {
        ESP_LOGE(TAG, "dsp_processor_create failed");
    }
    scanner_init(s_dsp);

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

    // T48: start the dsp_feed task now that everything it touches
    // (ingest_core1/worker_core1/signal_buffer/s_dsp, and the usbring
    // ring via esp_libusb_start_stream above) exists. By construction
    // the feeder can never run before streaming has actually started —
    // this replaces the old combined loop's `dev_addr==0 || rtldev==NULL`
    // continue-guard with "the task doesn't exist yet".
    //
    // Priority: dsp_feed must stay BELOW this task (usb_pump) so URB
    // completions always preempt the feed step (unchanged latency
    // bound from before the split), and ABOVE httpd (prio 5,
    // http_server.c) so a slow /capture/file download can't starve ring
    // drain and reintroduce the rb_full_drops task #91 fixed — dsp_feed
    // is now the "USB consumer" that rule refers to. Deriving the
    // feeder's priority as "our own priority minus one" keeps
    // pump > feed correct under whatever priority the caller gave this
    // task (production: pump=7 -> feed=6 > httpd 5, see
    // CLASS_TASK_PRIORITY in usb_host_lib_main.c; smoke test: pump=4 ->
    // feed=3, httpd isn't running there).
    s_feed_stop_requested = false;
    if (!s_feed_stopped_sem) s_feed_stopped_sem = xSemaphoreCreateBinary();
    UBaseType_t my_prio   = uxTaskPriorityGet(NULL);
    UBaseType_t feed_prio = (my_prio > (UBaseType_t)(tskIDLE_PRIORITY + 1))
                                ? my_prio - 1
                                : (UBaseType_t)(tskIDLE_PRIORITY + 1);
    BaseType_t  task_ok   = xTaskCreatePinnedToCore(dsp_feed_task, "dsp_feed", 4096,
                                                    NULL, feed_prio,
                                                    &s_dsp_feed_task_hdl, 0);
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore(dsp_feed) failed — streaming "
                      "will not drain the ring; expect rb_full_drops");
        s_dsp_feed_task_hdl = NULL;
    } else {
        usbring_set_consumer_task(s_dsp_feed_task_hdl);
        ESP_LOGI(TAG, "dsp_feed task started (prio %u, usb_pump prio %u)",
                 (unsigned)feed_prio, (unsigned)my_prio);
    }

    driver_obj->actions &= ~ACTION_START_STREAM;
}

static void action_close_dev(class_driver_t *driver_obj)
{
    ESP_LOGI(TAG, "Closing device");

    // T48: stop dsp_feed BEFORE this task (usb_pump) deregisters the USB
    // client / parks below. Mirrors this task's own end-of-life pattern
    // (stop cleanly, then park forever) — see stop_dsp_feed_task()'s doc
    // comment for why this is safe even though today's rtlsdr_close() is
    // a stub (T19, docs/review-2026-07-04-findings.md: replug already
    // requires a reboot, independent of this change).
    stop_dsp_feed_task();

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

    // T48: this loop is now purely event-driven — no stream-path calls.
    // The ring-drain + dispatch + DSP-feed cycle that used to interleave
    // here moved verbatim into dsp_feed_task(), below, which
    // action_start_stream() spawns once streaming actually begins.

    int64_t start_time  = esp_timer_get_time();
    int64_t last_report = start_time;

    uint64_t      cycle_handle_events_us = 0; // time blocked in usb_host_client_handle_events
    int64_t       last_idle_log          = esp_timer_get_time();
    int64_t       last_taskdump          = esp_timer_get_time();
    int64_t       last_recovery_us       = esp_timer_get_time();
    int           recovery_attempts      = 0;
    const int     MAX_RECOVERY_ATTEMPTS  = 3;
    const int64_t RECOVERY_INTERVAL_US   = 6 * 1000000;

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

        pump_stage(CS_PUMP_HANDLE_EVENTS);
        int64_t t_he0 = esp_timer_get_time();
        // NB: the timeout argument is in TICKS — 10 ticks = 100 ms at
        // the default 100 Hz tick, not the 10 ms some older comments
        // claimed. Deliberately kept: during streaming the client event
        // queue wakes this far sooner, and the long cap keeps the idle
        // loop cheap between events. Don't "fix" to pdMS_TO_TICKS(10)
        // without re-measuring the no-device idle load.
        usb_host_client_handle_events(s_driver_obj.client_hdl, 10);
        cycle_handle_events_us += (uint64_t)(esp_timer_get_time() - t_he0);
        atomic_fetch_add_explicit(&s_pump_iter, 1, memory_order_relaxed);

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
        if (s_driver_obj.actions & ACTION_RETUNE) {
            // Task 7: runs here (usb_pump) so the retune's control
            // transfers share this task with the bulk stream's
            // usb_host_client_handle_events() instead of racing it from
            // the caller — see esp_libusb_pause_stream()'s doc comment.
            s_driver_obj.actions &= ~ACTION_RETUNE;
            uint32_t hz = s_pending_retune_hz;
            esp_libusb_pause_stream(&s_driver_obj);
            int r = rtlsdr_set_center_freq(rtldev, hz);
            esp_libusb_resume_stream(&s_driver_obj, 0x81);
            ESP_LOGI(TAG, "retune to %lu Hz -> r=%d (stream resumed)", (unsigned long)hz, r);
            s_last_retune_ok = (r == 0);
            if (s_retune_done) xSemaphoreGive(s_retune_done);
        }
        if (s_driver_obj.actions & ACTION_CLOSE_DEV) action_close_dev(&s_driver_obj);
        if (s_driver_obj.actions & ACTION_EXIT) break;

        // T48: the ring-drain / dispatch / DSP-feed cycle that used to
        // run right here now runs in dsp_feed_task(). This task has
        // nothing else to do per-iteration besides the periodic report
        // below, so it falls straight through to it.

        int64_t now = esp_timer_get_time();
        if (now - last_report >= 1000000) {
            pump_stage(CS_PUMP_REPORT);
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
            //
            // T48: bytes_window/total_bytes/feed_calls_window/
            // dsp_total_time_us/dsp_frame_count/cycle_read_us/
            // cycle_take_converted_us/cycle_iterations are now written by
            // dsp_feed_task and read here via atomic_exchange (read-and-
            // reset in one op) — see the s_feed_* declarations above.
            // cycle_handle_events_us stays a plain local: only this task
            // (usb_pump) ever writes or reads it.
            status_snapshot_t snap       = {0};
            snap.window_us               = now - last_report;
            snap.elapsed_us              = now - start_time;
            snap.bytes_window            = atomic_exchange_explicit(&s_feed_bytes_window, 0, memory_order_relaxed);
            snap.total_bytes             = atomic_load_explicit(&s_feed_total_bytes, memory_order_relaxed);
            snap.feed_calls_window       = atomic_exchange_explicit(&s_feed_calls_window, 0, memory_order_relaxed);
            snap.dsp_total_time_us       = atomic_exchange_explicit(&s_feed_dsp_total_time_us, 0, memory_order_relaxed);
            snap.dsp_frame_count         = atomic_exchange_explicit(&s_feed_dsp_frame_count, 0, memory_order_relaxed);
            snap.cycle_read_us           = atomic_exchange_explicit(&s_feed_cycle_read_us, 0, memory_order_relaxed);
            snap.cycle_handle_events_us  = cycle_handle_events_us;
            snap.cycle_take_converted_us = atomic_exchange_explicit(&s_feed_cycle_take_converted_us, 0, memory_order_relaxed);
            snap.cycle_iterations        = atomic_exchange_explicit(&s_feed_cycle_iterations, 0, memory_order_relaxed);
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
                // T48: use the bytes_window value just exchanged into
                // snap above (already the feeder's window total, already
                // reset) instead of a separate local — same value the
                // old combined loop's bare `bytes_window` local held here.
                if (snap.bytes_window < STALL_BYTES_FLOOR) {
                    stall_seconds++;
                    ESP_LOGW(TAG, "stream stall #%d/%d (%llu B in last 1s, threshold %llu)",
                             stall_seconds, STALL_TRIGGER_SECONDS,
                             (unsigned long long)snap.bytes_window,
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

            // T48: only cycle_handle_events_us is still a plain local
            // (usb_pump-only); everything else was already reset by the
            // atomic_exchange calls above when snap was built.
            last_report            = now;
            cycle_handle_events_us = 0;
        }
    }

    // T48 defensive second call: ACTION_EXIT is only ever set from
    // action_close_dev() (which already calls stop_dsp_feed_task()), so
    // this is a no-op today (s_dsp_feed_task_hdl is already NULL) — kept
    // as insurance against a future path that sets ACTION_EXIT some
    // other way.
    stop_dsp_feed_task();

    ESP_LOGI(TAG, "Deregistering Client");
    usb_host_client_deregister(s_driver_obj.client_hdl);
    xSemaphoreGive(signaling_sem);
    // Wait to be deleted by app_main (same pattern as the daemon task).
    // Self-deleting here raced app_main's vTaskDelete(handle) — the idle
    // task could free this TCB first, making that call a use-after-free.
    vTaskSuspend(NULL);
}

// T48 (docs/perf-decoupling-design-2026-07-04.md §T48): the DSP feed
// task. Runs the stream cycle that used to interleave with
// usb_host_client_handle_events inside the single combined class_driver
// loop — peek raw USB bytes straight out of the usbring PSRAM ring
// (T49a), dispatch the ring region to ingest_core1 on Core 1, consume
// the PREVIOUS cycle's converted slot via dsp_processor_feed, release.
//
// The acquire/raw_done/read/dispatch/take_converted/feed/release
// sequence below — including the drain-in-flight-slot-on-empty branch —
// is UNCHANGED in order and blocking semantics from the old combined
// loop (docs/perf-decoupling-design-2026-07-04.md §T48: "moves
// verbatim"). The only addition is the CS_FEED_WAIT_DATA block at the
// bottom of the "no data" branch, which blocks this task when the ring
// is genuinely empty and nothing is in flight to drain — the old
// combined loop got that idle throttling for free from
// usb_host_client_handle_events' 100 ms cap, which now lives solely in
// usb_pump and no longer runs in this task.
static void dsp_feed_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "dsp_feed task starting");

    uint32_t out_block_size = 16 * 1024;

    // Ping-pong steady-state book-keeping — same roles as the old
    // combined loop's locals of the same names. We start by reading into
    // slot 0; the matching DSP feed for slot 0 happens AFTER slot 1 has
    // been dispatched (one-cycle pipeline). prev_dsp_slot tracks which
    // slot the DSP should next consume; it's -1 on the very first
    // iteration.
    int prev_dsp_slot = -1;

    // T49a: the usbring only supports ONE outstanding un-consumed span
    // (usbring_peek() always views from the current tail — see
    // usbring.h). raw_bytes[slot] records how many ring bytes each
    // slot's last dispatch peeked, so the NEXT read can wait for Core 1
    // to finish reading them (ingest_core1_wait_raw_done) and reclaim
    // them (esp_libusb_consume_stream) before peeking again. This is a
    // synchronisation point separate from (and earlier than) the
    // existing acquire(s_free)/dispatch/take(s_ready)/release(s_free)
    // sequence; it does not change that sequence's order or blocking
    // semantics.
    size_t raw_bytes[INGEST_NUM_SLOTS] = {0};

    while (!s_feed_stop_requested) {
        int slot_for_read;
        feed_stage(CS_FEED_ACQUIRE);
        ingest_core1_acquire_slot(&slot_for_read);

        // T49a: the usbring supports only one outstanding un-consumed
        // span. Before peeking fresh data, reclaim the PREVIOUS cycle's
        // dispatch (if any) once Core 1 confirms (raw_done) it has
        // finished reading it. This is the "acquire-next serialises
        // against convert-done" tradeoff the design doc calls out — a
        // NEW wait, spliced in before the existing acquire/read/
        // dispatch/take/release sequence, not a change to that
        // sequence's own ordering.
        feed_stage(CS_FEED_RAW_DONE);
        if (prev_dsp_slot >= 0) {
            ingest_core1_wait_raw_done(prev_dsp_slot);
            esp_libusb_consume_stream(raw_bytes[prev_dsp_slot]);
        }

        size_t         n_read       = 0;
        const uint8_t *raw          = NULL;
        int64_t        t_read_start = esp_timer_get_time();
        feed_stage(CS_FEED_READ);
        int     read_ok    = esp_libusb_read_stream(&raw, out_block_size, &n_read);
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
            atomic_fetch_add_explicit(&s_feed_cycle_read_us, (uint64_t)(t_read_end - t_read_start), memory_order_relaxed);
            atomic_fetch_add_explicit(&s_feed_total_bytes, n_read, memory_order_relaxed);
            atomic_fetch_add_explicit(&s_feed_bytes_window, n_read, memory_order_relaxed);

            feed_stage(CS_FEED_DISPATCH);
            // Optional raw IQ capture (#63). Fast no-op when no capture
            // is active; otherwise copies n_read bytes into a PSRAM
            // stream buffer (non-blocking, drops on overflow). Tap is
            // here — pre-dispatch — so we capture the exact uint8
            // payload before any conversion. Reads straight from the
            // ring pointer; safe because the region isn't reclaimed
            // (usbring_consume) until next cycle, well after this
            // synchronous call returns.
            sd_capture_write(raw, n_read);

            // Hand the ring region to ingest on Core 1. Convert + push
            // happen there; we don't block on completion.
            ingest_core1_dispatch(slot_for_read, raw, n_read);
            raw_bytes[slot_for_read] = n_read;

            // If we have a previous slot in flight, consume it now via DSP.
            // Wait for Core 1's ingest to mark it ready (typically immediate
            // — ingest is faster than feed, so by the time we need the data
            // it's already been converted + signal_buffer_pushed).
            if (prev_dsp_slot >= 0) {
                size_t  n_int16 = 0;
                int64_t t_tc0   = esp_timer_get_time();
                feed_stage(CS_FEED_TAKE_CONVERTED);
                int16_t *converted = ingest_core1_take_converted(prev_dsp_slot, &n_int16);
                atomic_fetch_add_explicit(&s_feed_cycle_take_converted_us,
                                          (uint64_t)(esp_timer_get_time() - t_tc0), memory_order_relaxed);
                int64_t t_pre_feed = esp_timer_get_time();

                feed_stage(CS_FEED_DSP_FEED);
                dsp_processor_feed(s_dsp, converted, n_int16 / 2);
                int64_t t_post_feed = esp_timer_get_time();
                atomic_fetch_add_explicit(&s_feed_dsp_total_time_us,
                                          (uint64_t)(t_post_feed - t_pre_feed), memory_order_relaxed);
                atomic_fetch_add_explicit(&s_feed_dsp_frame_count, (uint32_t)((n_int16 / 2) / 2048), memory_order_relaxed);
                atomic_fetch_add_explicit(&s_feed_calls_window, 1, memory_order_relaxed);

                // Mark the slot free so ingest can reuse it next cycle.
                feed_stage(CS_FEED_RELEASE);
                ingest_core1_release(prev_dsp_slot);
            }

            prev_dsp_slot = slot_for_read;
        } else {
            // No data this iteration. Before releasing the slot we just
            // acquired, drain any in-flight slot from the previous cycle.
            // Skipping this deadlocked the loop: with prev_dsp_slot's
            // s_free still held and the rotation pointer already advanced
            // past the slot we're releasing, the next acquire_slot blocks
            // forever on s_free[prev_dsp_slot] — which only this task can
            // give, after a take_converted it can no longer reach. Armed
            // exactly when the stream pauses (dongle hiccup / unplug /
            // quiet ring); the #105/#106 "stuck in acquire_slot" signature.
            // (The prev_dsp_slot's ring span, if any, was already reclaimed
            // above at CS_FEED_RAW_DONE — unaffected by whether THIS
            // cycle's read finds new data.)
            if (prev_dsp_slot >= 0) {
                size_t n_int16 = 0;
                feed_stage(CS_FEED_TAKE_CONVERTED);
                int16_t *converted = ingest_core1_take_converted(prev_dsp_slot, &n_int16);
                feed_stage(CS_FEED_DSP_FEED);
                dsp_processor_feed(s_dsp, converted, n_int16 / 2);
                feed_stage(CS_FEED_RELEASE);
                ingest_core1_release(prev_dsp_slot);
                prev_dsp_slot = -1;
            }
            // Release the slot we just acquired so ingest can reuse it
            // (we never dispatched — nothing to reclaim from the ring
            // for it).
            ingest_core1_release(slot_for_read);

            // Ring genuinely empty and nothing in flight: block on the
            // producer's wake notification instead of immediately
            // re-looping (T48 — see usbring_set_consumer_task()).
            // pdTRUE clears the notification count on take, coalescing
            // any number of writes since the last wake into a single
            // wake-and-drain-everything pass. The bounded timeout is a
            // safety net (a write that raced stream-start before this
            // task's handle was registered, or any future refactor that
            // drops a notification) — never a correctness dependency.
            feed_stage(CS_FEED_WAIT_DATA);
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        }

        feed_stage(CS_FEED_TOP);
        atomic_fetch_add_explicit(&s_feed_iter, 1, memory_order_relaxed);
    }

    ESP_LOGI(TAG, "dsp_feed task stopping (device gone)");
    if (s_feed_stopped_sem) xSemaphoreGive(s_feed_stopped_sem);
    // Mirror usb_pump's own end-of-life pattern: stop touching shared
    // state, then park forever. Nobody calls vTaskDelete() on this
    // task (its handle is private to class_driver.c and is cleared by
    // stop_dsp_feed_task() before this point), so self-suspending here —
    // unlike self-deleting — cannot race an external vTaskDelete().
    vTaskSuspend(NULL);
}
