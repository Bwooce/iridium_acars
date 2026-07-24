// See autotune.h.

#include "autotune.h"
#include "autotune_gainset.h"
#include "app_config.h"
#include "class_driver.h"
#include "scanner.h"
#include "worker_core1.h"
#include "band_select.h" // band_decode_stats_get + band_id_t (band-aware metric)

#include <stdatomic.h>
#include "esp_log.h"
#include "esp_timer.h" // scan-progress elapsed/ETA clock
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

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

// Persist an autotune result to NVS from a stack-safe context.
//
// nvs_commit() calls spi_flash_disable_interrupts_caches_and_other_cpu(),
// which asserts (cache_utils.c:114, esp_task_stack_is_sane_cache_disabled)
// that the running task's stack is NOT in PSRAM — a PSRAM-backed stack becomes
// inaccessible once the cache is disabled for the flash write. But autotune
// runs on autotune_sched's task, whose stack IS in PSRAM (it mostly sleeps, so
// it was placed there to spare internal RAM). Calling app_config_set_*()
// directly therefore aborts the device — the ~24.7-min boot-sweep crash loop.
//
// Route the write through a short-lived task with a default (internal-RAM)
// stack — same reason http_server's tune-apply task uses an internal stack —
// and block until it completes. Transient, so no permanent internal-RAM cost
// (the DMA-INT/URB budget is razor-thin; a permanent 4 KB stack there is risky).
typedef struct {
    bool              is_lo;
    int32_t           val;
    SemaphoreHandle_t done;
} autotune_persist_req_t;

static void autotune_persist_task(void *arg)
{
    autotune_persist_req_t *r = (autotune_persist_req_t *)arg;
    // Copy args out of the caller's PSRAM stack BEFORE any flash op (which
    // disables the cache and would make that PSRAM read fault).
    bool              is_lo = r->is_lo;
    int32_t           val   = r->val;
    SemaphoreHandle_t done  = r->done;
    if (is_lo) {
        (void)app_config_set_lo_freq_hz((uint32_t)val);
    } else {
        (void)app_config_set_gain_db_x10((int16_t)val);
    }
    xSemaphoreGive(done);
    vTaskDelete(NULL);
}

static void autotune_persist(bool is_lo, int32_t val)
{
    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    if (!done) {
        ESP_LOGW(TAG, "persist: no sem; skipping NVS write (val=%ld is_lo=%d)",
                 (long)val, (int)is_lo);
        return;
    }
    autotune_persist_req_t req = {.is_lo = is_lo, .val = val, .done = done};
    // Default xTaskCreate => internal-RAM stack, so nvs_commit's cache-disable
    // is legal. 4096 B mirrors http_server's tune-apply task.
    if (xTaskCreate(autotune_persist_task, "at_persist", 4096, &req, 5, NULL) == pdPASS) {
        xSemaphoreTake(done, portMAX_DELAY);
    } else {
        ESP_LOGW(TAG, "persist: task create failed; NVS write skipped");
    }
    vSemaphoreDelete(done);
}

// Body of the manual/boot/periodic gain-calibration pass. Caller must hold
// the busy guard (autotune_try_begin() already returned true).
// Scan-progress state for the status page: 0=idle, 1=gain cal, 2=LO rescan.
// Lets the operator see WHY reception dips (the on_boot gain sweep) and how much
// is left, instead of guessing. esp_timer µs clock; relaxed atomics (one writer
// = the autotune task, cross-task reads from the http task are torn-read-benign).
static _Atomic int     s_scan_type       = 0;
static _Atomic int64_t s_scan_start_us   = 0;
static _Atomic int64_t s_scan_est_end    = 0;
static _Atomic int     s_scan_gain_dbx10 = 0; // live gain being tested (type 1)

static void scan_begin(int type, int64_t est_dur_us)
{
    int64_t now = esp_timer_get_time();
    atomic_store_explicit(&s_scan_start_us, now, memory_order_relaxed);
    atomic_store_explicit(&s_scan_est_end, now + est_dur_us, memory_order_relaxed);
    atomic_store_explicit(&s_scan_type, type, memory_order_relaxed);
}

static void scan_end(void)
{
    atomic_store_explicit(&s_scan_gain_dbx10, 0, memory_order_relaxed);
    atomic_store_explicit(&s_scan_type, 0, memory_order_relaxed);
}

// Live gain (dB×10) currently being tested during a gain-cal scan; 0 when not
// gain-scanning. For the status page's "config → now" display.
int autotune_scan_cur_gain_dbx10(void)
{
    return atomic_load_explicit(&s_scan_gain_dbx10, memory_order_relaxed);
}

// Public wrappers so the /scan TEST endpoint (which calls scanner_scan directly,
// bypassing autotune_run_lo_rescan) can still drive the status-page scan
// indicator. type: 1=gain, 2=LO. est_dur_s is the expected duration for the ETA.
void autotune_scan_mark(int type, int est_dur_s)
{
    scan_begin(type, (int64_t)est_dur_s * 1000000);
}
void autotune_scan_unmark(void)
{
    scan_end();
}

int autotune_scan_status(int *elapsed_s, int *remaining_s)
{
    int t = atomic_load_explicit(&s_scan_type, memory_order_relaxed);
    if (t == 0) {
        if (elapsed_s) *elapsed_s = 0;
        if (remaining_s) *remaining_s = 0;
        return 0;
    }
    int64_t now = esp_timer_get_time();
    if (elapsed_s) {
        int64_t e  = (now - atomic_load_explicit(&s_scan_start_us, memory_order_relaxed)) / 1000000;
        *elapsed_s = e < 0 ? 0 : (int)e;
    }
    if (remaining_s) {
        int64_t r    = (atomic_load_explicit(&s_scan_est_end, memory_order_relaxed) - now) / 1000000;
        *remaining_s = r < 0 ? 0 : (int)r;
    }
    return t;
}

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

    // Gain-cal maximizes decodes against the Iridium IRA reference beacon (a
    // known always-on downlink at autotune_ira_lo_hz). VDL2 has no equivalent
    // always-on reference, so a decode-maximizing sweep can't discriminate
    // there (see the band-mode plan). Refuse under band=vdl2 — the fill-based
    // ADC-quantization gain sweep is the band-agnostic method instead.
    if ((band_id_t)cfg.band == BAND_VDL2) {
        ESP_LOGW(TAG, "REFUSED: gain-cal unsupported for band=vdl2 (no IRA-equivalent "
                      "reference beacon); use the fill-based gain sweep");
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
    // Publish scan progress: ng gains × (prime + dwell) per step.
    scan_begin(1, (int64_t)ng * (int64_t)(AUTOTUNE_PRIME_MS + dwell_ms) * 1000);
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
        if (!class_driver_set_gain_quiesced(gains[i])) {
            ESP_LOGW(TAG, "  gain %d.%d dB: set failed, skipping",
                     gains[i] / 10, gains[i] % 10);
            decoded[i] = -1; // invalid: pick_best won't choose a negative
            continue;
        }
        atomic_store_explicit(&s_scan_gain_dbx10, gains[i], memory_order_relaxed);
        scanner_reset_baseline();                     // floor moves with gain
        vTaskDelay(pdMS_TO_TICKS(AUTOTUNE_PRIME_MS)); // discard prime transient

        // Band-agnostic funnel snapshot (Iridium: the SAME cumulative BCH
        // decoded/unknown counters as before, via the unified accessor). VDL2
        // is gated out above, so this loop is Iridium-only; the accessor keeps
        // the metric band-correct should that ever change.
        band_decode_stats_t s0 = {0}, s1 = {0};
        band_decode_stats_get((band_id_t)cfg.band, &s0);
        vTaskDelay(pdMS_TO_TICKS(dwell_ms));
        band_decode_stats_get((band_id_t)cfg.band, &s1);

        decoded[i] = (int)(s1.decoded - s0.decoded);
        unknown[i] = (int)(s1.unknown - s0.unknown);
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
    if (!class_driver_set_gain_quiesced(chosen_gain)) {
        ESP_LOGE(TAG, "WARNING: failed to apply gain %d.%d dB",
                 chosen_gain / 10, chosen_gain % 10);
    }
    scanner_reset_baseline();

    if (have_signal) {
        // Persist so the pick survives reboot (mode is MANUAL, so gain_db_x10
        // is applied at boot). Only gain_db_x10 is written; mode untouched.
        // Via autotune_persist (internal-stack task): this runs on the PSRAM-
        // stacked autotune_sched task, so a direct nvs_commit would abort.
        autotune_persist(false, (int32_t)chosen_gain);
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
    scan_end();
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
    // Publish scan progress: one dwell per hop across the band.
    int lo_hops = (int)((SCAN_STOP_HZ - SCAN_START_HZ) / SCAN_STEP_HZ) + 1;
    scan_begin(2, (int64_t)lo_hops * (int64_t)SCAN_DWELL_MS * 1000);
    scanner_scan(SCAN_START_HZ, SCAN_STOP_HZ, SCAN_STEP_HZ, SCAN_DWELL_MS);

    // scanner_scan() parks live (persist=false) on the hottest center but
    // doesn't write NVS. For "track the slow-drifting center" to survive a
    // reboot, persist whatever it landed on.
    uint32_t hot_hz = scanner_last_hot_hz();
    if (hot_hz) {
        // Internal-stack task (see autotune_persist): a direct nvs_commit from
        // this PSRAM-stacked task aborts at the cache-disable sanity assert.
        autotune_persist(true, (int32_t)hot_hz);
        ESP_LOGI(TAG, "=== autotune LO rescan done: persisted center %lu Hz ===",
                 (unsigned long)hot_hz);
    } else {
        ESP_LOGW(TAG, "=== autotune LO rescan done: no hot center found; LO unchanged ===");
    }

    scan_end();
    autotune_end();
}
