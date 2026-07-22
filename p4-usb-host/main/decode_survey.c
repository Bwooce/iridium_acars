// See decode_survey.h for the model. decode_survey_core.h owns the decision
// math (elimination z-test, histogram placement); this file is the task glue:
// scheduling, per-visit counter/histogram deltas, RF hops, status publishing
// and the NVS result blob.

#include "decode_survey.h"
#include "decode_survey_core.h"
#include "scanner.h"       // scanner_hop / scanner_survey / scanner_get_last_map + SCAN_* grid
#include "scanner_map.h"   // scanner_pos_t / scanner_rank_hottest
#include "frame_decoder.h" // LW.DA class counts + rel-freq histogram
#include "app_config.h"    // start-time NVS lo_freq_hz (abort park-back target)
#include "autotune.h"      // don't stack RF actions; status-page scan marker
#include "dsp_processor.h" // FS_DETECT_HZ (RX window width for placement)

#include "esp_iot_log.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include <stdatomic.h>
#include <string.h>
#include <time.h>

static const char *TAG = "DSURVEY";

// Own NVS namespace so a /config rewrite or key audit never touches the survey
// result blob (same rationale as band_health's "bandhp").
#define DS_NVS_NAMESPACE "dsurv"
#define DS_NVS_KEY       "result"
#define DS_MAGIC         0x31565344u // "DSV1" — bump on layout change

// Attribution guard: the worker runs >= 1 s behind RF, so after a hop we drain
// the queue (old-LO in-flight frames finish classifying) before snapshotting,
// and again before the end-of-dwell snapshot. Poll to at most GUARD_MAX; hold
// at least GUARD_MIN so RF lag is covered even when the queue looks empty.
#define DS_GUARD_MIN_MS 2500u
#define DS_GUARD_MAX_MS 4000u

// ---- published state (single-task writer; torn diagnostic reads benign) ----
static atomic_bool     s_running;
static atomic_bool     s_abort;
static _Atomic uint8_t  s_phase;
static _Atomic uint32_t s_cycle;
static _Atomic uint32_t s_elapsed_s;
static _Atomic uint32_t s_budget_s;
static _Atomic uint32_t s_leader_hz;
static _Atomic uint32_t s_pick_hz;

static int         s_n_centers;                 // task-owned once running
static ds_center_t s_centers[DS_MAX_CENTERS];   // task-owned working table
static uint32_t    s_abs_hist[DS_ABS_HIST_BINS]; // task-owned abs-freq map
static uint32_t    s_nvs_lo;                     // LO to restore on abort

// start() args handed to the task.
static int s_arg_budget_h;
static int s_arg_k;

// Persisted result blob (own NVS namespace).
typedef struct {
    uint32_t    magic;
    uint64_t    unix_time; // time(NULL) at completion (0 if clock unset)
    uint32_t    pick_hz;
    uint32_t    leader_hz;
    uint32_t    best_sum;  // Phase-C winning window LW.DA integral
    uint32_t    budget_s;
    uint16_t    n_centers;
    uint16_t    completed; // 1 = finished, 0 = aborted (aborted is not persisted)
    ds_center_t centers[DS_MAX_CENTERS];
} ds_result_blob_t;

bool decode_survey_running(void)
{
    return atomic_load(&s_running);
}

void decode_survey_stop(void)
{
    if (atomic_load(&s_running)) {
        atomic_store(&s_abort, true);
        ESP_LOGW(TAG, "abort requested — will park back on NVS %lu Hz",
                 (unsigned long)s_nvs_lo);
    }
}

void decode_survey_get_status(decode_survey_status_t *out)
{
    if (!out) return;
    out->running   = atomic_load(&s_running);
    out->phase     = (ds_phase_t)atomic_load(&s_phase);
    out->cycle     = atomic_load(&s_cycle);
    out->elapsed_s = atomic_load(&s_elapsed_s);
    out->budget_s  = atomic_load(&s_budget_s);
    out->leader_hz = atomic_load(&s_leader_hz);
    out->pick_hz   = atomic_load(&s_pick_hz);
    out->n_centers = s_n_centers;
    out->alive     = ds_alive_count(s_centers, s_n_centers);
    memcpy(out->centers, s_centers, sizeof(out->centers));
    memcpy(out->abs_hist, s_abs_hist, sizeof(out->abs_hist));
}

// Sleep ms in small steps, returning false immediately if an abort is pending.
static bool survey_delay_ms(uint32_t ms)
{
    const uint32_t step = 250;
    for (uint32_t e = 0; e < ms; e += step) {
        if (atomic_load(&s_abort)) return false;
        uint32_t chunk = (ms - e) < step ? (ms - e) : step;
        vTaskDelay(pdMS_TO_TICKS(chunk));
    }
    return !atomic_load(&s_abort);
}

// Attribution guard: wait until the decoder has drained the frames captured up
// to now (popped >= the pushed count at entry), bounded by GUARD_MAX, and hold
// at least GUARD_MIN so the worker's RF lag is covered. Returns false on abort.
static bool survey_drain_guard(void)
{
    uint64_t pushed = frame_decoder_pushed();
    uint32_t waited = 0;
    while (waited < DS_GUARD_MAX_MS) {
        if (atomic_load(&s_abort)) return false;
        if (frame_decoder_popped() >= pushed && waited >= DS_GUARD_MIN_MS) break;
        vTaskDelay(pdMS_TO_TICKS(100));
        waited += 100;
    }
    return !atomic_load(&s_abort);
}

// Fold the LW.DA rel-freq histogram DELTA (after − before), taken while parked
// at lo_center_hz, into the LO-independent absolute-freq map.
static void survey_fold_hist(uint32_t lo_center_hz,
                             const uint32_t *before, const uint32_t *after)
{
    for (int b = 0; b < FRAME_DECODER_LWDA_FREQ_BINS; b++) {
        uint32_t d = after[b] - before[b]; // monotonic counters; wrap is impossible in a visit
        if (d == 0) continue;
        int32_t  rel = frame_decoder_lwda_freq_bin_center_hz(b);
        int64_t  abs = (int64_t)lo_center_hz + rel;
        if (abs > 0) ds_abs_hist_add(s_abs_hist, (uint32_t)abs, d);
    }
}

// Phase A: run the existing density survey, then build a decode-ranking
// shortlist = top-K density centers (ranked rate respects the IRA down-weight)
// ∪ the density peak's grid neighbours. Returns the shortlist size.
static int survey_build_shortlist(int k)
{
    ESP_LOGI(TAG, "Phase A: density pre-pass (%d sweeps) for shortlist",
             SCAN_SURVEY_SWEEPS);
    scanner_survey(SCAN_START_HZ, SCAN_STOP_HZ, SCAN_STEP_HZ, SCAN_DWELL_MS,
                   SCAN_SURVEY_SWEEPS);

    scanner_pos_t map[SCANNER_MAX_POSITIONS];
    int           nmap = scanner_get_last_map(map, SCANNER_MAX_POSITIONS);
    if (nmap <= 0) {
        ESP_LOGW(TAG, "Phase A: empty density map — nothing to survey");
        return 0;
    }

    // Top-K by RAW density (scanner_rank_hottest_raw — NOT the IRA-penalised
    // ranker): decode arbitrates downstream, so we must not let a frequency
    // heuristic permanently exclude an upper-IDA-tail center before Phase B
    // measures it. An IRA-dense center that lands here costs one recoverable
    // ~5 min visit (lw_da≈0 → eliminated). Zero the winner so the next call
    // returns the runner-up. (The IRA penalty still guards the density-only
    // park inside scanner_survey above — see scanner_rank_hottest_raw docs.)
    scanner_pos_t work[SCANNER_MAX_POSITIONS];
    memcpy(work, map, (size_t)nmap * sizeof(scanner_pos_t));
    uint32_t shortlist[DS_MAX_CENTERS];
    int      ns = 0;
    for (int i = 0; i < k && ns < DS_MAX_CENTERS; i++) {
        int idx = scanner_rank_hottest_raw(work, nmap);
        if (idx < 0 || work[idx].dwell_ms == 0) break;
        shortlist[ns++]              = work[idx].center_hz;
        work[idx].narrowband_bursts  = 0;
        work[idx].dwell_ms           = 0; // exclude from the next pick
    }

    // Union in the density peak's grid neighbours (±1 step) so a real IDA
    // center adjacent to the peak isn't excluded by an unlucky pre-pass.
    int peak = scanner_rank_hottest_raw(map, nmap);
    if (peak >= 0) {
        uint32_t pc       = map[peak].center_hz;
        uint32_t neigh[2] = {pc - SCAN_STEP_HZ, pc + SCAN_STEP_HZ};
        for (int j = 0; j < 2 && ns < DS_MAX_CENTERS; j++) {
            bool in_grid = false, have = false;
            for (int i = 0; i < nmap; i++)
                if (map[i].center_hz == neigh[j]) in_grid = true;
            for (int i = 0; i < ns; i++)
                if (shortlist[i] == neigh[j]) have = true;
            if (in_grid && !have) shortlist[ns++] = neigh[j];
        }
    }

    memset(s_centers, 0, sizeof(s_centers));
    memset(s_abs_hist, 0, sizeof(s_abs_hist));
    for (int i = 0; i < ns; i++) s_centers[i].center_hz = shortlist[i];
    s_n_centers = ns;

    ESP_LOGI(TAG, "Phase A done: shortlist of %d centers", ns);
    for (int i = 0; i < ns; i++)
        ESP_LOGI(TAG, "  candidate %lu Hz", (unsigned long)shortlist[i]);
    return ns;
}

// One round-robin visit at a shortlisted center: hop, settle, guard, snapshot,
// dwell, guard, snapshot, fold. Returns false on abort.
static bool survey_visit(ds_center_t *c)
{
    if (scanner_hop(c->center_hz, false) != ESP_OK) {
        ESP_LOGW(TAG, "visit %lu Hz: hop failed, skipping",
                 (unsigned long)c->center_hz);
        return !atomic_load(&s_abort);
    }
    if (!survey_delay_ms(SCAN_SETTLE_MS)) return false; // tagger re-prime settle
    if (!survey_drain_guard()) return false;            // drain old-LO in-flight

    frame_decoder_class_counts_t cc0 = {0};
    frame_decoder_get_class_counts(&cc0);
    uint32_t reasm0_valid;
    {
        frame_decoder_reasm_stats_t r0 = {0};
        frame_decoder_get_reasm_stats(&r0);
        reasm0_valid = (uint32_t)r0.lw_da_valid;
    }
    uint32_t hist0[FRAME_DECODER_LWDA_FREQ_BINS];
    frame_decoder_get_lwda_freq_hist(hist0, FRAME_DECODER_LWDA_FREQ_BINS);

    if (!survey_delay_ms(DS_VISIT_MS)) return false; // the dwell
    if (!survey_drain_guard()) return false;         // drain this-center in-flight

    frame_decoder_class_counts_t cc1 = {0};
    frame_decoder_get_class_counts(&cc1);
    uint32_t reasm1_valid;
    {
        frame_decoder_reasm_stats_t r1 = {0};
        frame_decoder_get_reasm_stats(&r1);
        reasm1_valid = (uint32_t)r1.lw_da_valid;
    }
    uint32_t hist1[FRAME_DECODER_LWDA_FREQ_BINS];
    frame_decoder_get_lwda_freq_hist(hist1, FRAME_DECODER_LWDA_FREQ_BINS);

    uint32_t d_lwda  = (uint32_t)(cc1.lw_da - cc0.lw_da);
    uint32_t d_valid = reasm1_valid - reasm0_valid;
    ds_center_add_visit(c, d_lwda, d_valid, /*nb*/ 0, DS_VISIT_MS);
    survey_fold_hist(c->center_hz, hist0, hist1);

    ESP_LOGI(TAG, "visit %lu Hz: +%lu LW.DA (+%lu valid) → %lu total over %.1f h",
             (unsigned long)c->center_hz, (unsigned long)d_lwda,
             (unsigned long)d_valid, (unsigned long)c->lw_da,
             (double)c->dwell_ms / 3600000.0);
    return !atomic_load(&s_abort);
}

// Persist the ranked result to NVS. Legal here because the survey runs on an
// internal-RAM xTaskCreate stack (a PSRAM-stack nvs_commit aborts at
// cache_utils.c:114 — same rule as band_health's hour worker).
static void survey_persist_result(uint32_t pick_hz, uint32_t best_sum)
{
    ds_result_blob_t blob = {0};
    blob.magic     = DS_MAGIC;
    blob.unix_time = (uint64_t)time(NULL);
    blob.pick_hz   = pick_hz;
    blob.leader_hz = atomic_load(&s_leader_hz);
    blob.best_sum  = best_sum;
    blob.budget_s  = atomic_load(&s_budget_s);
    blob.n_centers = (uint16_t)s_n_centers;
    blob.completed = 1;
    memcpy(blob.centers, s_centers, sizeof(blob.centers));

    nvs_handle_t h;
    esp_err_t    r = nvs_open(DS_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed (%s); result not persisted",
                 esp_err_to_name(r));
        return;
    }
    r = nvs_set_blob(h, DS_NVS_KEY, &blob, sizeof(blob));
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    if (r != ESP_OK)
        ESP_LOGW(TAG, "result persist failed (%s)", esp_err_to_name(r));
    else
        ESP_LOGI(TAG, "result persisted (pick=%lu Hz)", (unsigned long)pick_hz);
}

static void survey_task(void *arg)
{
    (void)arg;
    int budget_h = s_arg_budget_h;
    int k        = s_arg_k;

    app_config_t cfg;
    app_config_snapshot(&cfg);
    s_nvs_lo = cfg.lo_freq_hz;

    atomic_store(&s_phase, DS_PHASE_SHORTLIST);
    atomic_store(&s_cycle, 0);
    atomic_store(&s_elapsed_s, 0);
    atomic_store(&s_pick_hz, 0);
    atomic_store(&s_leader_hz, 0);
    atomic_store(&s_budget_s, (uint32_t)budget_h * 3600u);

    ESP_LOGW(TAG, "starting decode survey: budget=%d h, k=%d, NVS lo=%lu Hz "
                  "(live-only park; NVS unchanged)",
             budget_h, k, (unsigned long)s_nvs_lo);
    iot_log(IOT_LOG_WARN, "DECODE-SURVEY starting");

    // Mark an RF scan for the whole survey: the HTML scan row shows it and
    // band_health / autotune defer their own RF actions while it runs.
    autotune_scan_mark(2, budget_h * 3600);

    int ns = survey_build_shortlist(k);
    if (ns <= 0) goto done_noresult;
    if (atomic_load(&s_abort)) goto aborted;

    // ---- Phase B: interleaved round-robin decode ranking --------------------
    atomic_store(&s_phase, DS_PHASE_RR);
    int64_t  t0        = esp_timer_get_time();
    uint32_t budget_s  = (uint32_t)budget_h * 3600u;
    uint32_t cyc       = 0;
    while (!ds_converged(s_centers, s_n_centers)) {
        if (atomic_load(&s_abort)) goto aborted;
        uint32_t elapsed = (uint32_t)((esp_timer_get_time() - t0) / 1000000);
        atomic_store(&s_elapsed_s, elapsed);
        if (elapsed >= budget_s) {
            ESP_LOGW(TAG, "Phase B: budget reached (%lu s) — stopping",
                     (unsigned long)budget_s);
            break;
        }
        for (int i = 0; i < s_n_centers; i++) {
            if (s_centers[i].eliminated) continue;
            if (!survey_visit(&s_centers[i])) goto aborted;
        }
        // End-of-cycle elimination + status refresh.
        ds_eliminate_pass(s_centers, s_n_centers, DS_ELIM_Z,
                          DS_FLOOR_DWELL_MS, DS_FLOOR_VISITS);
        int lead = ds_leader_index(s_centers, s_n_centers);
        if (lead >= 0) atomic_store(&s_leader_hz, s_centers[lead].center_hz);
        cyc++;
        atomic_store(&s_cycle, cyc);
        atomic_store(&s_elapsed_s,
                     (uint32_t)((esp_timer_get_time() - t0) / 1000000));
        ESP_LOGI(TAG, "cycle %lu done: %d/%d centers alive, leader=%lu Hz",
                 (unsigned long)cyc, ds_alive_count(s_centers, s_n_centers),
                 s_n_centers, (unsigned long)atomic_load(&s_leader_hz));
    }

    // ---- Phase C: place the 2.5 MHz window on the LW.DA histogram peak ------
    atomic_store(&s_phase, DS_PHASE_PLACE);
    uint32_t best_sum = 0;
    uint32_t pick = ds_best_window_center(s_abs_hist, DS_ABS_HIST_BINS,
                                          FS_DETECT_HZ, SCAN_START_HZ,
                                          SCAN_STOP_HZ, DS_PLACE_STEP_HZ,
                                          &best_sum);
    if (best_sum == 0) {
        // No decodes anywhere — fall back to the Phase B leader (or the NVS LO).
        int lead = ds_leader_index(s_centers, s_n_centers);
        pick     = (lead >= 0) ? s_centers[lead].center_hz : s_nvs_lo;
        ESP_LOGW(TAG, "Phase C: histogram empty — falling back to %lu Hz",
                 (unsigned long)pick);
    }
    atomic_store(&s_pick_hz, pick);
    scanner_hop(pick, false); // LIVE park only — NVS lo_freq_hz stays put
    survey_persist_result(pick, best_sum);
    atomic_store(&s_phase, DS_PHASE_DONE);
    ESP_LOGW(TAG, "decode survey DONE: live-parked %lu Hz (window integral=%lu "
                  "LW.DA); persist manually via POST /tune if it holds up",
             (unsigned long)pick, (unsigned long)best_sum);
    iot_log(IOT_LOG_WARN, "DECODE-SURVEY done");
    goto finish;

aborted:
    atomic_store(&s_phase, DS_PHASE_ABORTED);
    scanner_hop(s_nvs_lo, false); // restore the operator's LO, no result persisted
    ESP_LOGW(TAG, "decode survey ABORTED — parked back on %lu Hz",
             (unsigned long)s_nvs_lo);
    iot_log(IOT_LOG_WARN, "DECODE-SURVEY aborted");
    goto finish;

done_noresult:
    atomic_store(&s_phase, DS_PHASE_DONE);
    ESP_LOGW(TAG, "decode survey ended with no shortlist (no density)");

finish:
    autotune_scan_unmark();
    atomic_store(&s_abort, false);
    atomic_store(&s_running, false);
    vTaskDelete(NULL);
}

esp_err_t decode_survey_start(int budget_h, int k)
{
    bool expected = false;
    if (!atomic_compare_exchange_strong(&s_running, &expected, true))
        return ESP_ERR_INVALID_STATE; // already running
    if (autotune_scan_status(NULL, NULL) != 0) {
        atomic_store(&s_running, false);
        ESP_LOGW(TAG, "refusing: a gain-cal / LO scan is in progress");
        return ESP_ERR_INVALID_STATE;
    }
    s_arg_budget_h = (budget_h > 0) ? budget_h : DS_DEFAULT_BUDGET_H;
    s_arg_k        = (k >= 1 && k <= DS_MAX_CENTERS) ? k : DS_DEFAULT_K;
    atomic_store(&s_abort, false);

    // 6144 B INTERNAL-RAM stack (default xTaskCreate placement): the scanner_
    // survey pre-pass runs fine on 4096 elsewhere, plus headroom for the NVS
    // result commit. MUST NOT be a PSRAM stack — nvs_commit from SPIRAM stack
    // aborts at cache_utils.c:114 (memory feedback_psram_stack_no_flash_write).
    if (xTaskCreate(survey_task, "dsurvey", 6144, NULL, 4, NULL) != pdPASS) {
        atomic_store(&s_running, false);
        ESP_LOGW(TAG, "task spawn failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}
