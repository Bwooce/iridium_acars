// See autotune_sched.h.

#include "autotune_sched.h"
#include "app_config.h"
#include "autotune.h"
#include "esp_libusb.h" // usb_stream_totals_t / esp_libusb_get_stream_totals

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h" // xTaskCreatePinnedToCoreWithCaps

static const char *TAG = "AT_SCHED";

static bool s_started = false;

// Steady-state due-check cadence once the scheduler is running. Cheap (a
// config snapshot + a couple of comparisons), so a coarse cadence is fine --
// this only needs to be well under the shortest interval anyone would
// reasonably configure (minutes, not the default hours).
#define SCHED_POLL_MS (30 * 1000)
// Poll cadence while waiting for the USB stream to come up at boot.
#define BOOT_POLL_MS (2 * 1000)
// Settle time after the stream is first observed live before trusting
// "stable" -- mirrors autotune.c's AUTOTUNE_PRIME_MS discard (that one
// covers a post-gain-change transient; this one covers the initial
// enumeration/first-bytes transient), applied once here at boot.
#define STREAM_SETTLE_MS (5 * 1000)

static int64_t now_s(void)
{
    return esp_timer_get_time() / 1000000;
}

// Blocks until esp_libusb's lifetime `completed` counter has advanced at
// least once (proof the RTL-SDR stream is actually feeding the pipeline,
// not just that a device enumerated), then a short settle. Same signal the
// independent health watchdog uses (wifi_link.c's s_stream_live), read
// directly here instead of through that WiFi-STA-gated wrapper -- autotune
// must not depend on WiFi/STA mode being up.
static void wait_for_stream_live(void)
{
    usb_stream_totals_t ut            = {0};
    uint64_t            last          = 0;
    bool                seen_baseline = false;
    for (;;) {
        esp_libusb_get_stream_totals(&ut);
        if (!seen_baseline) {
            seen_baseline = true;
            last          = ut.completed;
        } else if (ut.completed > last) {
            ESP_LOGI(TAG, "USB stream live (completed=%llu); settling %d ms before first pass",
                     (unsigned long long)ut.completed, STREAM_SETTLE_MS);
            vTaskDelay(pdMS_TO_TICKS(STREAM_SETTLE_MS));
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(BOOT_POLL_MS));
    }
}

static void autotune_sched_task(void *arg)
{
    (void)arg;
    wait_for_stream_live();

    // Anchor both clocks to stream-stable time, not raw task-start time, so
    // neither fires immediately off a near-zero elapsed on the first wakeup.
    int64_t anchor        = now_s();
    int64_t last_gain_run = anchor;
    int64_t last_lo_run   = anchor;

    app_config_t cfg;
    app_config_snapshot(&cfg);
    if (cfg.autotune_on_boot) {
        if (cfg.gain_mode == GAIN_MODE_MANUAL) {
            ESP_LOGI(TAG, "autotune_on_boot: running one calibration pass");
            autotune_run_manual();
        } else {
            ESP_LOGW(TAG,
                     "autotune_on_boot set but gain_mode=%d (needs MANUAL/1); skipping boot pass",
                     (int)cfg.gain_mode);
        }
        // Count the boot pass against the gain clock either way, so a short
        // autotune_gain_interval_s doesn't immediately re-fire seconds later
        // even when the boot pass itself was skipped for the wrong mode.
        last_gain_run = now_s();
    }

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SCHED_POLL_MS));

        app_config_snapshot(&cfg);
        // Task-level gate: skip BOTH clocks outright when not in MANUAL
        // mode. autotune_run_manual()/autotune_run_lo_rescan() each also
        // refuse individually (belt and suspenders), but gating here avoids
        // even attempting + logging a refusal every poll. This is also why
        // the default gain_mode (TUNER_AGC) is safe with intervals=3600 out
        // of the box: a device that hasn't been switched to MANUAL never
        // autotunes at all.
        if (cfg.gain_mode != GAIN_MODE_MANUAL) continue;

        int64_t t = now_s();
        if (autotune_is_due(last_gain_run, t, cfg.autotune_gain_interval_s)) {
            ESP_LOGI(TAG, "periodic gain-cal due (interval=%lu s)",
                     (unsigned long)cfg.autotune_gain_interval_s);
            autotune_run_manual();
            last_gain_run = now_s();
        }
        if (autotune_is_due(last_lo_run, t, cfg.autotune_lo_interval_s)) {
            ESP_LOGI(TAG, "periodic LO rescan due (interval=%lu s)",
                     (unsigned long)cfg.autotune_lo_interval_s);
            autotune_run_lo_rescan();
            last_lo_run = now_s();
        }
    }
}

void autotune_sched_init(void)
{
    if (s_started) return;
    s_started = true;

    // Mirrors wifi_link.c's health_wdt_task: low priority, PSRAM stack (this
    // task mostly sleeps -- the calibration dwells are vTaskDelay, not
    // busy work -- so it doesn't compete for Core-1 DSP/worker budget), no
    // core affinity.
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(autotune_sched_task, "autotune_sched",
                                                    4096, NULL, 2, NULL, tskNO_AFFINITY,
                                                    MALLOC_CAP_SPIRAM);
    if (ok != pdPASS) {
        ESP_LOGW(TAG, "task create failed; autotune boot/periodic disabled");
        s_started = false;
    }
}
