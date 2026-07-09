// See autotune.h.

#include "autotune.h"
#include "autotune_gainset.h"
#include "app_config.h"
#include "class_driver.h"
#include "scanner.h"
#include "worker_core1.h"

#include <stdatomic.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "AUTOTUNE";

// Settle/prime window after a gain change + baseline reset, before counting.
// A freshly reset tagger baseline re-primes over a few seconds and emits
// priming-phase false positives (MEMORY: priming-phase FPs beat the legit
// tone); discard that transient. Cheap against the multi-minute dwell.
// Mirrors scanner_scan's settle discard.
#define AUTOTUNE_PRIME_MS 3000

// In-progress guard shared by every autotune entry point (the manual serial
// `autotune` command, the boot-time pass, and the periodic gain/LO
// scheduler in autotune_sched.c). Without this, a hand-typed `autotune` in
// the middle of a scheduled pass -- or the two scheduled clocks landing on
// the same wakeup -- would run two sweeps concurrently, each retuning the
// LO/gain out from under the other. All entry points must go through
// autotune_try_begin()/autotune_end() around their body.
static atomic_bool s_autotune_busy = false;

static bool autotune_try_begin(void)
{
    bool expected = false;
    return atomic_compare_exchange_strong(&s_autotune_busy, &expected, true);
}

static void autotune_end(void)
{
    atomic_store(&s_autotune_busy, false);
}

// Body of the manual/boot/periodic gain-calibration pass. Caller must hold
// the busy guard (autotune_try_begin() already returned true).
static void autotune_run_manual_locked(void)
{
    app_config_t cfg;
    app_config_snapshot(&cfg);

    // Precondition: MANUAL gain mode (see header). Refuse otherwise.
    if (cfg.gain_mode != GAIN_MODE_MANUAL) {
        ESP_LOGW(TAG, "REFUSED: gain_mode=%d; autotune needs MANUAL (1). "
                      "Run: set gain_mode 1 ; reboot ; autotune",
                 (int)cfg.gain_mode);
        return;
    }

    // Save current state so restore runs on every non-abort exit path.
    uint32_t saved_lo   = cfg.lo_freq_hz;
    int      saved_gain = class_driver_get_tuner_gain_dbx10();
    if (saved_gain < 0) saved_gain = cfg.gain_db_x10;

    // Build the coarse gain sweep over the real R828D steps.
    int gains[AUTOTUNE_R828D_N];
    int ng = autotune_build_gain_set(cfg.autotune_gain_min_dbx10,
                                     cfg.autotune_gain_max_dbx10,
                                     cfg.autotune_gain_stride,
                                     gains, AUTOTUNE_R828D_N);
    if (ng <= 0) {
        ESP_LOGW(TAG, "REFUSED: empty gain set (min=%d max=%d stride=%u)",
                 (int)cfg.autotune_gain_min_dbx10,
                 (int)cfg.autotune_gain_max_dbx10,
                 (unsigned)cfg.autotune_gain_stride);
        return;
    }

    uint32_t dwell_ms = cfg.autotune_gain_dwell_s * 1000u;
    ESP_LOGI(TAG,
             "=== autotune manual pass: IRA LO=%lu Hz, %d gains, %lu s dwell each ===",
             (unsigned long)cfg.autotune_ira_lo_hz, ng,
             (unsigned long)cfg.autotune_gain_dwell_s);

    int decoded[AUTOTUNE_R828D_N] = {0};
    int unknown[AUTOTUNE_R828D_N] = {0};

    // Hop to the IRA reference LO (resets the tagger baseline internally).
    if (scanner_hop(cfg.autotune_ira_lo_hz, false) != ESP_OK) {
        ESP_LOGE(TAG, "hop to IRA LO %lu Hz failed; aborting (LO/gain unchanged)",
                 (unsigned long)cfg.autotune_ira_lo_hz);
        return;
    }

    for (int i = 0; i < ng; i++) {
        if (!class_driver_set_tuner_gain_dbx10(gains[i])) {
            ESP_LOGW(TAG, "  gain %d.%d dB: set failed, skipping",
                     gains[i] / 10, gains[i] % 10);
            decoded[i] = -1; // invalid: pick_best won't choose a negative
            continue;
        }
        scanner_reset_baseline();                     // floor moves with gain
        vTaskDelay(pdMS_TO_TICKS(AUTOTUNE_PRIME_MS)); // discard prime transient

        uint32_t d0 = 0, u0 = 0, d1 = 0, u1 = 0;
        worker_core1_get_decode_counts(&d0, &u0);
        vTaskDelay(pdMS_TO_TICKS(dwell_ms));
        worker_core1_get_decode_counts(&d1, &u1);

        decoded[i] = (int)(d1 - d0);
        unknown[i] = (int)(u1 - u0);
        ESP_LOGI(TAG, "  gain %2d.%d dB: bch_decoded=%d bch_unknown=%d",
                 gains[i] / 10, gains[i] % 10, decoded[i], unknown[i]);
    }

    int best = autotune_pick_best(decoded, ng);
    // Only adopt the swept gain if it actually produced decodes. All-zero
    // (e.g. IRA inaudible) is not evidence to overwrite a known-good gain, so
    // fall back to the saved gain and don't persist in that case.
    bool have_signal = (best >= 0 && decoded[best] > 0);
    int  chosen_gain = have_signal ? gains[best] : saved_gain;

    // Restore: park back at the ACARS LO with the chosen gain (every exit).
    if (scanner_hop(saved_lo, false) != ESP_OK) {
        ESP_LOGE(TAG, "WARNING: failed to hop back to ACARS LO %lu Hz",
                 (unsigned long)saved_lo);
    }
    if (!class_driver_set_tuner_gain_dbx10(chosen_gain)) {
        ESP_LOGE(TAG, "WARNING: failed to apply gain %d.%d dB",
                 chosen_gain / 10, chosen_gain % 10);
    }
    scanner_reset_baseline();

    if (have_signal) {
        // Persist so the pick survives reboot (mode is MANUAL, so gain_db_x10
        // is applied at boot). Only gain_db_x10 is written; mode untouched.
        (void)app_config_set_gain_db_x10((int16_t)chosen_gain);
        ESP_LOGI(TAG,
                 "=== autotune done: chose gain %d.%d dB (bch_decoded=%d), "
                 "persisted + parked at LO %lu Hz ===",
                 chosen_gain / 10, chosen_gain % 10, decoded[best],
                 (unsigned long)saved_lo);
    } else {
        ESP_LOGW(TAG,
                 "=== autotune done: no decodes on any gain; restored gain "
                 "%d.%d dB (not persisted), LO %lu Hz ===",
                 chosen_gain / 10, chosen_gain % 10, (unsigned long)saved_lo);
    }

    (void)unknown; // logged per-gain above; secondary signal only
}

void autotune_run_manual(void)
{
    if (!autotune_try_begin()) {
        ESP_LOGW(TAG, "REFUSED: another autotune pass is already in progress");
        return;
    }
    autotune_run_manual_locked();
    autotune_end();
}

void autotune_run_lo_rescan(void)
{
    app_config_t cfg;
    app_config_snapshot(&cfg);

    // Same MANUAL-only precondition as the gain-cal pass (see autotune.h):
    // a live LO sweep fights SOFTWARE_AGC/TUNER_AGC just as much as a gain
    // sweep would, and the density-scan result is only meaningful if the
    // gain that's about to receive it is held steady.
    if (cfg.gain_mode != GAIN_MODE_MANUAL) {
        ESP_LOGW(TAG, "LO rescan REFUSED: gain_mode=%d; needs MANUAL (1)",
                 (int)cfg.gain_mode);
        return;
    }
    if (!autotune_try_begin()) {
        ESP_LOGW(TAG, "LO rescan REFUSED: another autotune pass is already in progress");
        return;
    }

    // Automated multi-hop `scanner_scan()` is VALIDATED SAFE on main
    // (2026-07-09): 42 live retunes across 7 sweeps via the POST /scan test
    // endpoint, 42/42 r=0, 0 wedges, 0 EP0 STALL, 0 worker drops, stream
    // steady at 4.7 MB/s. The earlier "not yet confirmed safe" note here was
    // a STALE NEGATIVE — it predated the control-URB-reuse fix (ce6bfb3, now
    // on main; the reverted 9638816 was re-applied), retune-retry (575982c),
    // and graceful-shutdown. The DMA-internal-heap stash churn across retunes
    // is BENIGN (fully recovered, zero drops). autotune_lo_interval_s=0 stays
    // the DEFAULT (operator opt-in), but enabling it is now safe. See the
    // design doc's "Empirical findings" for why the interval defaults long
    // (hourly) rather than the original 600 s satellite-handoff estimate.
    ESP_LOGI(TAG, "=== autotune LO rescan: sweeping %lu-%lu Hz step %lu Hz ===",
             (unsigned long)SCAN_START_HZ, (unsigned long)SCAN_STOP_HZ,
             (unsigned long)SCAN_STEP_HZ);
    scanner_scan(SCAN_START_HZ, SCAN_STOP_HZ, SCAN_STEP_HZ, SCAN_DWELL_MS);

    // scanner_scan() parks live (persist=false) on the hottest center but
    // doesn't write NVS. For "track the slow-drifting center" to survive a
    // reboot, persist whatever it landed on.
    uint32_t hot_hz = scanner_last_hot_hz();
    if (hot_hz) {
        (void)app_config_set_lo_freq_hz(hot_hz);
        ESP_LOGI(TAG, "=== autotune LO rescan done: persisted center %lu Hz ===",
                 (unsigned long)hot_hz);
    } else {
        ESP_LOGW(TAG, "=== autotune LO rescan done: no hot center found; LO unchanged ===");
    }

    autotune_end();
}
