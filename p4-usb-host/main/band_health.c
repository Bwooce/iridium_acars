// See band_health.h for the model; band_health_core.h for the thresholds.

#include "band_health.h"
#include "band_health_core.h"
#include "app_config.h"    // band_resurvey_auto opt-in flag
#include "autotune.h"      // scan-in-progress check + status-page scan marker
#include "frame_decoder.h" // lifetime LW.DA (IDA/ACARS-bearing) frame count
#include "scanner.h"       // scanner_survey + SCAN_* grid defaults
#include "decode_survey.h" // defer staleness eval while a decode survey runs

#include "esp_iot_log.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include <stdatomic.h>
#include <string.h>

static const char *TAG = "BAND_HP";

// Own NVS namespace: the history blob is machine state, not operator config —
// keeping it out of app_config's namespace means a /config rewrite or key
// audit never touches it.
#define BH_NVS_NAMESPACE "bandhp"
#define BH_NVS_KEY "hist"
#define BH_MAGIC 0x31504842u // "BHP1" — bump on layout change
#define BH_TICK_S 3600u      // one bucket per uptime hour

// Persisted state, written as a single NVS blob once per completed hour
// (~360 B/h — negligible flash wear on the wear-levelled NVS partition).
typedef struct {
    uint32_t magic;
    uint16_t count;      // valid samples (<= BH_HIST_HOURS)
    uint16_t head;       // ring index of the most recent sample
    uint16_t baseline;   // frozen commissioning median; 0 = not yet frozen
    uint16_t cooldown_h; // hours until another fire is allowed
    uint32_t fired_total;
    uint16_t hist[BH_HIST_HOURS]; // IDA (LW.DA) frames per tracked hour
} bh_persist_t;

static bh_persist_t         s_hist; // owned by bh_hour_task (one in flight, see s_busy)
static uint64_t             s_lwda_at_last_tick = 0;
static esp_timer_handle_t   s_timer             = NULL;
static band_health_status_t s_status; // published snapshot (torn reads benign)
static atomic_bool          s_busy;   // hourly worker in flight
static atomic_bool          s_survey_running;
static atomic_bool          s_survey_done_flag; // set by band_health_note_survey

// NVS write — legal here ONLY because bh_hour_task runs on a default
// (internal-RAM) xTaskCreate stack; nvs_commit from a PSRAM-stack task
// aborts at the cache-disable sanity assert (cache_utils.c:114).
static void bh_persist_blob(void)
{
    nvs_handle_t h;
    esp_err_t    r = nvs_open(BH_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed (%s); hour not persisted", esp_err_to_name(r));
        return;
    }
    r = nvs_set_blob(h, BH_NVS_KEY, &s_hist, sizeof(s_hist));
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    if (r != ESP_OK)
        ESP_LOGW(TAG, "history persist failed (%s)", esp_err_to_name(r));
}

// Median over the most recent `span` samples of the ring (order is
// irrelevant to a median, so just walk back from head).
static uint16_t bh_trailing_median(int span)
{
    uint16_t win[BH_HIST_HOURS];
    if (span > (int)s_hist.count) span = (int)s_hist.count;
    if (span <= 0) return 0;
    for (int i = 0; i < span; i++)
        win[i] = s_hist.hist[(s_hist.head + BH_HIST_HOURS - i) % BH_HIST_HOURS];
    return bh_median_u16(win, span);
}

// The auto action: one integrated survey, live-park on the integrated peak.
// Deliberately does NOT persist the new LO — a health-wdt reboot reverts to
// the NVS frequency, so a bad automatic pick can never strand the remote
// device past its next reboot. The operator persists a pick they trust via
// POST /tune or serial `hop <hz> save`.
static void bh_run_auto_survey(void)
{
    app_config_t cfg;
    app_config_snapshot(&cfg);
    ESP_LOGW(TAG, "auto re-survey: %d integrated sweeps starting (LO will hop; "
                  "pick is live-only, NVS lo_hz=%lu Hz unchanged)",
             SCAN_SURVEY_SWEEPS, (unsigned long)cfg.lo_freq_hz);
    iot_log(IOT_LOG_WARN, "BAND-HEALTH auto re-survey starting");

    atomic_store(&s_survey_running, true);
    int hops = (int)((SCAN_STOP_HZ - SCAN_START_HZ) / SCAN_STEP_HZ) + 1;
    autotune_scan_mark(2, hops * (int)(SCAN_DWELL_MS / 1000) * SCAN_SURVEY_SWEEPS);
    scanner_survey(SCAN_START_HZ, SCAN_STOP_HZ, SCAN_STEP_HZ, SCAN_DWELL_MS,
                   SCAN_SURVEY_SWEEPS);
    autotune_scan_unmark();
    atomic_store(&s_survey_running, false);

    ESP_LOGW(TAG, "auto re-survey done: live-parked on %lu Hz (integrated peak; "
                  "persist manually if it holds up)",
             (unsigned long)scanner_last_hot_hz());
    iot_log(IOT_LOG_WARN, "BAND-HEALTH auto re-survey done");
}

// Hourly worker. Spawned by the tick with a default (internal-RAM) stack so
// the NVS persist — and a possible multi-minute survey — are legal here.
static void bh_hour_task(void *arg)
{
    (void)arg;

    // 1) Close the hour's bucket: delta of the lifetime LW.DA counter.
    frame_decoder_class_counts_t cc = {0};
    frame_decoder_get_class_counts(&cc);
    uint64_t delta      = cc.lw_da - s_lwda_at_last_tick;
    s_lwda_at_last_tick = cc.lw_da;
    uint16_t sample     = delta > 0xFFFFu ? 0xFFFFu : (uint16_t)delta;

    s_hist.head              = (uint16_t)((s_hist.head + 1) % BH_HIST_HOURS);
    s_hist.hist[s_hist.head] = sample;
    if (s_hist.count < BH_HIST_HOURS) s_hist.count++;
    if (s_hist.cooldown_h > 0) s_hist.cooldown_h--;
    // An integrated survey completed since the last tick (operator or auto):
    // re-arm the cooldown so we don't fire on top of a fresh re-park.
    if (atomic_exchange(&s_survey_done_flag, false))
        s_hist.cooldown_h = BH_COOLDOWN_HOURS;
    // A decode-based band survey (decode_survey.c) is hopping the LO right now:
    // this hour's bucket is polluted by the hops and we must not fire a
    // re-survey on top of it. Re-arm the cooldown so staleness never fires
    // while it runs (and for a cycle after, so the fresh re-park isn't judged).
    if (decode_survey_running())
        s_hist.cooldown_h = BH_COOLDOWN_HOURS;

    // 2) Freeze the commissioning baseline once 7 days of hours exist. A
    //    zero median (dead receiver throughout commissioning) is not frozen —
    //    we retry each hour until the median is meaningful.
    if (s_hist.baseline == 0 && s_hist.count >= BH_BASELINE_HOURS) {
        uint16_t m = bh_trailing_median(BH_BASELINE_HOURS);
        if (m > 0) {
            s_hist.baseline = m;
            ESP_LOGI(TAG, "commissioning baseline frozen: %u IDA frames/h "
                          "(median of first %u tracked hours)",
                     (unsigned)m, (unsigned)BH_BASELINE_HOURS);
        }
    }

    // 3) Evaluate staleness.
    uint16_t trail = 0;
    bool     below = false, fire = false;
    if (s_hist.count >= BH_MIN_EVAL_HOURS) {
        trail = bh_trailing_median(BH_HIST_HOURS);
        below = bh_below_baseline(trail, s_hist.baseline);
    }
    if (below && s_hist.cooldown_h == 0) {
        if (autotune_scan_status(NULL, NULL) != 0) {
            // A gain-cal / LO scan is mid-flight: don't stack RF actions or
            // judge an hour it polluted. No state change — retry next tick.
            ESP_LOGI(TAG, "stale but a scan/cal is in progress; deferring 1 h");
        } else {
            fire              = true;
            s_hist.cooldown_h = BH_COOLDOWN_HOURS;
            s_hist.fired_total++;
        }
    }

    // 4) Publish the snapshot for /status.
    s_status.hours_tracked   = s_hist.count;
    s_status.last_hour       = sample;
    s_status.trailing_median = trail;
    s_status.baseline        = s_hist.baseline;
    s_status.cooldown_h      = s_hist.cooldown_h;
    s_status.fired_total     = s_hist.fired_total;
    s_status.below_baseline  = below;

    // 5) Persist BEFORE any survey: the cooldown must survive a mid-survey
    //    reboot or the trigger could refire every boot while the band is low.
    bh_persist_blob();

    // 6) Act.
    if (fire) {
        app_config_t cfg;
        app_config_snapshot(&cfg);
        ESP_LOGW(TAG, "band STALE: 7-day trailing median %u IDA frames/h < %u%% "
                      "of commissioning baseline %u/h — integrated re-survey "
                      "recommended (POST /scan?n=%d or serial `survey`); for a "
                      "decode-ranked band-finder use POST /survey%s",
                 (unsigned)trail, (unsigned)BH_TRIGGER_PCT,
                 (unsigned)s_hist.baseline, SCAN_SURVEY_SWEEPS,
                 cfg.band_resurvey_auto ? "" : "; resurvey_auto=0, no RF action");
        iot_log(IOT_LOG_WARN, "BAND-HEALTH stale: re-survey recommended");
        if (cfg.band_resurvey_auto) bh_run_auto_survey();
    }

    atomic_store(&s_busy, false);
    vTaskDelete(NULL);
}

// esp_timer callback (esp_timer task context): keep it trivial — all real
// work (counter delta, median, NVS, survey) happens on the spawned worker.
static void bh_tick(void *arg)
{
    (void)arg;
    bool expected = false;
    if (!atomic_compare_exchange_strong(&s_busy, &expected, true)) {
        // Previous hour's worker still running (an auto survey is ~7 min, so
        // this means something is badly wedged). Skip; the missed hour folds
        // into the next delta, which only biases the bucket HIGH (safe: a
        // high bucket can't cause a false fire).
        ESP_LOGW(TAG, "hourly tick skipped: previous worker still running");
        return;
    }
    // 5120 B internal-RAM stack: two ~340 B median buffers + NVS + the
    // scanner_survey path (which runs fine on 4096 elsewhere). Transient —
    // freed when the worker exits, so no permanent DMA-INT budget cost.
    if (xTaskCreate(bh_hour_task, "bh_hour", 5120, NULL, 4, NULL) != pdPASS) {
        atomic_store(&s_busy, false);
        ESP_LOGW(TAG, "worker create failed; hour folds into the next bucket");
    }
}

void band_health_init(void)
{
    memset(&s_hist, 0, sizeof(s_hist));
    s_hist.magic = BH_MAGIC;

    nvs_handle_t h;
    if (nvs_open(BH_NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        bh_persist_t tmp;
        size_t       sz = sizeof(tmp);
        if (nvs_get_blob(h, BH_NVS_KEY, &tmp, &sz) == ESP_OK &&
            sz == sizeof(tmp) && tmp.magic == BH_MAGIC &&
            tmp.count <= BH_HIST_HOURS && tmp.head < BH_HIST_HOURS) {
            s_hist = tmp;
            ESP_LOGI(TAG, "history restored: %u h tracked, baseline=%u/h, "
                          "cooldown=%u h, fired=%lu",
                     (unsigned)s_hist.count, (unsigned)s_hist.baseline,
                     (unsigned)s_hist.cooldown_h, (unsigned long)s_hist.fired_total);
        } else {
            ESP_LOGI(TAG, "no valid history blob; starting fresh");
        }
        nvs_close(h);
    }

    s_status.hours_tracked = s_hist.count;
    s_status.baseline      = s_hist.baseline;
    s_status.cooldown_h    = s_hist.cooldown_h;
    s_status.fired_total   = s_hist.fired_total;

    const esp_timer_create_args_t args = {
        .callback        = bh_tick,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "band_health",
    };
    if (esp_timer_create(&args, &s_timer) == ESP_OK) {
        esp_timer_start_periodic(s_timer, (uint64_t)BH_TICK_S * 1000000ULL);
    } else {
        ESP_LOGW(TAG, "timer create failed; band-health tracking disabled");
    }
}

void band_health_get_status(band_health_status_t *out)
{
    if (!out) return;
    *out                = s_status;
    out->survey_running = atomic_load(&s_survey_running);
}

void band_health_note_survey(int n_sweeps)
{
    if (n_sweeps >= 2) atomic_store(&s_survey_done_flag, true);
}
