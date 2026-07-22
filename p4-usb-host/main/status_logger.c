// Status-logger task. See status_logger.h for the why.
//
// The implementation is straightforward: one queue (depth 2), one task
// pinned to Core 1 at priority 6 — deliberately ABOVE worker_core1 (4)
// so the STATUS line survives worker saturation; see the rationale at
// the spawn site (status_logger_init) and the CAVEAT about its UART
// cost. The task blocks on xQueueReceive (1100 ms timeout);
// class_driver posts a snapshot once per second.

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"    // esp_reset_reason() — reboot-cause telemetry over iot_log
#include "worker_core1.h"  // A6 hot-boost stats for the STATUS line
#include "panic_capture.h" // remote crash frame (mepc/ra) recovered from RTC
#include "status_logger.h"
#include "esp_iot_log.h"
#include "app_config.h"
#include "dsp_processor.h" // FS_DETECT_HZ (listening bandwidth)
#include "signal_buffer.h"
#include "ingest_core1.h"
#include "esp_libusb.h"
#include "frame_decoder.h" // lw_da / CRC-fail / sbd reception telemetry
#include "wifi_link.h"     // gw-watchdog / stream-stall counts for reboot-cause tracing

static const char *TAG = "CLASS"; // match the original tag for log continuity

static QueueHandle_t s_queue;

// Most recent emitted snapshot, for the HTTP /status page (see
// status_logger_get_last). Written once per second by the logger task;
// read cross-core by the http task. Unlocked — a torn read is benign for
// a diagnostic display and avoids adding a mutex to the 1 Hz hot(ish) path.
static status_snapshot_t s_last;
static volatile bool     s_have_last = false;

// Capacity telemetry accumulated since boot, for remote monitoring when the
// device runs headless outside (poll /status or watch iot_log). Peak vs mean
// answers "is the load transient-bursty or sustained." Updated once/sec in
// emit(); read cross-core by the http task (unlocked — torn read is benign for
// a diagnostic gauge, same rationale as s_last).
static uint32_t s_cap_windows     = 0;    // sampled 1 s windows
static uint64_t s_cap_sum_bursts  = 0;    // Σ tagger bursts/window (for mean)
static uint32_t s_cap_peak_bursts = 0;    // max tagger bursts in any window
static float    s_cap_peak_worker = 0.0f; // max Core-1 worker cap %
static float    s_cap_peak_dsp    = 0.0f; // max Core-0 DSP/tagger cap %
static uint32_t s_cap_worker_ge90 = 0;    // windows with worker cap >= 90 %
// Accepted-burst / backlog-sizing telemetry (Option B scoping). "Accepted" =
// passed the pre-filter and ran the full demod (bursts_processed); it's
// throughput-capped, so also track the queue-drops (bursts lost at the
// SNR-priority queue before they could be serviced — what a backlog would
// salvage) and the pre-filter accept ratio (are those drops real or junk).
static uint32_t s_cap_peak_processed = 0; // max accepted+serviced bursts/window
static uint32_t s_cap_peak_qdrops    = 0; // max bursts dropped/evicted at queue/window
static uint64_t s_cap_sum_processed  = 0; // Σ processed (for accept ratio)
static uint64_t s_cap_sum_triagerej  = 0; // Σ pre-filter rejected (for accept ratio)

// ---- Reception-environment heuristic (status_logger.h / the design doc) ----
// EMAs of the per-window funnel counts (raw counts, not ratios — EMA the
// numerator and denominator separately so a tiny window can't blow up a ratio).
// Updated once/sec in emit(); read cross-core by the http task (unlocked torn
// read benign, same rationale as s_last / s_cap_*).
#define RX_EMA_ALPHA       0.05f // ~20 s time constant at 1 Hz
#define RX_WARMUP_WINDOWS  20    // report INIT until this many windows sampled
#define RX_DWELL_WINDOWS   8     // a new candidate state must persist this long to commit
// Conservative default thresholds — calibrate against the exported raw ratios
// (doc anchors: outdoor interference uw_reach≈0.04; QUIET when the band is idle).
#define RX_QUIET_TAGGED    3.0f  // < this many tagger bursts/window ⇒ QUIET (idle band)
#define RX_UW_INTERFERE    0.10f // reached-BCH/processed below this ⇒ INTERFERENCE (UW-lock collapsed)
#define RX_DECODE_MARGINAL 0.15f // decoded/reached-BCH below this ⇒ MARGINAL (weak Iridium)
#define RX_FAIL_MARGINAL   0.70f // BCH-failed/reached-BCH above this ⇒ MARGINAL
static uint32_t s_rx_windows        = 0;             // windows folded into the EMAs
static float    s_rx_ema_tagged     = 0.0f;          // gone_bursts/window
static float    s_rx_ema_processed  = 0.0f;          // bursts_processed/window
static float    s_rx_ema_reached    = 0.0f;          // (decoded+unknown+failed)/window
static float    s_rx_ema_decoded    = 0.0f;          // bch_decoded/window
static float    s_rx_ema_failed     = 0.0f;          // bch_failed/window
static rx_state_t s_rx_state        = RX_STATE_INIT; // committed (debounced) state
static rx_state_t s_rx_cand         = RX_STATE_INIT;  // candidate awaiting dwell
static uint32_t   s_rx_cand_count   = 0;              // consecutive windows the candidate held

bool status_logger_get_last(status_snapshot_t *out)
{
    if (!s_have_last || !out) return false;
    *out = s_last; // struct copy
    return true;
}

void status_logger_get_capacity(status_capacity_t *out)
{
    if (!out) return;
    out->windows              = s_cap_windows;
    out->peak_bursts          = s_cap_peak_bursts;
    out->mean_bursts          = s_cap_windows ? (float)((double)s_cap_sum_bursts / (double)s_cap_windows) : 0.0f;
    out->peak_worker_cap      = s_cap_peak_worker;
    out->peak_dsp_cap         = s_cap_peak_dsp;
    out->worker_ge90_pct      = s_cap_windows ? (100.0f * (float)s_cap_worker_ge90 / (float)s_cap_windows) : 0.0f;
    out->peak_processed       = s_cap_peak_processed;
    out->peak_queue_drops     = s_cap_peak_qdrops;
    uint64_t pf_total         = s_cap_sum_processed + s_cap_sum_triagerej;
    out->prefilter_accept_pct = pf_total ? (100.0f * (float)s_cap_sum_processed / (float)pf_total) : 0.0f;
}

// Fold one window's funnel counts into the reception EMAs and re-classify.
// Called once/sec from emit() (before the verbose/quiet fork) so it runs
// regardless of build config. Pure arithmetic on the logger task — negligible.
static void rx_update(const status_snapshot_t *s)
{
    uint32_t tagged    = s->dsp.gone_bursts;
    uint32_t processed = s->ws.bursts_processed;
    uint32_t reached   = s->ws.bursts_bch_decoded + s->ws.bursts_bch_unknown +
                         s->ws.bursts_bch_failed;
    uint32_t decoded   = s->ws.bursts_bch_decoded;
    uint32_t failed    = s->ws.bursts_bch_failed;

    float a = RX_EMA_ALPHA;
    s_rx_ema_tagged    += a * ((float)tagged - s_rx_ema_tagged);
    s_rx_ema_processed += a * ((float)processed - s_rx_ema_processed);
    s_rx_ema_reached   += a * ((float)reached - s_rx_ema_reached);
    s_rx_ema_decoded   += a * ((float)decoded - s_rx_ema_decoded);
    s_rx_ema_failed    += a * ((float)failed - s_rx_ema_failed);
    if (s_rx_windows < UINT32_MAX) s_rx_windows++;

    if (s_rx_windows < RX_WARMUP_WINDOWS) {
        s_rx_state = RX_STATE_INIT;
        return;
    }

    // Ratios from the EMAs (guard tiny denominators). uw_reach = of the bursts
    // the worker demod-attempted, how many reached BCH — the interference tell.
    float uw_reach    = s_rx_ema_reached / (s_rx_ema_processed > 1e-3f ? s_rx_ema_processed : 1e-3f);
    float decode_frac = s_rx_ema_decoded / (s_rx_ema_reached > 1e-3f ? s_rx_ema_reached : 1e-3f);
    float fail_frac   = s_rx_ema_failed / (s_rx_ema_reached > 1e-3f ? s_rx_ema_reached : 1e-3f);

    rx_state_t cand;
    if (s_rx_ema_tagged < RX_QUIET_TAGGED) {
        cand = RX_STATE_QUIET; // band idle — too few bursts to judge anything else
    } else if (uw_reach < RX_UW_INTERFERE) {
        cand = RX_STATE_INTERFERENCE; // energy present but almost none locks a UW
    } else if (decode_frac < RX_DECODE_MARGINAL || fail_frac > RX_FAIL_MARGINAL) {
        cand = RX_STATE_MARGINAL; // UW locks but frames are too weak to decode cleanly
    } else {
        cand = RX_STATE_GOOD;
    }

    // Debounce: a new candidate must hold RX_DWELL_WINDOWS consecutive windows
    // before it commits, so a single pass / brief spike can't flip the state.
    if (cand == s_rx_cand) {
        if (s_rx_cand_count < UINT32_MAX) s_rx_cand_count++;
    } else {
        s_rx_cand       = cand;
        s_rx_cand_count = 1;
    }
    if (s_rx_cand_count >= RX_DWELL_WINDOWS || s_rx_state == RX_STATE_INIT) {
        s_rx_state = s_rx_cand;
    }
}

const char *status_reception_state_name(rx_state_t s)
{
    switch (s) {
    case RX_STATE_QUIET:        return "quiet";
    case RX_STATE_INTERFERENCE: return "interference";
    case RX_STATE_MARGINAL:     return "marginal";
    case RX_STATE_GOOD:         return "good";
    case RX_STATE_INIT:
    default:                    return "init";
    }
}

void status_logger_get_reception(status_reception_t *out)
{
    if (!out) return;
    out->state        = s_rx_state;
    out->tagged_ema   = s_rx_ema_tagged;
    out->processed_ema= s_rx_ema_processed;
    out->uw_reach     = s_rx_ema_reached / (s_rx_ema_processed > 1e-3f ? s_rx_ema_processed : 1e-3f);
    out->decode_frac  = s_rx_ema_decoded / (s_rx_ema_reached > 1e-3f ? s_rx_ema_reached : 1e-3f);
    out->fail_frac    = s_rx_ema_failed / (s_rx_ema_reached > 1e-3f ? s_rx_ema_reached : 1e-3f);
}

static const char *reset_reason_name(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:
        return "POWERON";
    case ESP_RST_EXT:
        return "EXT";
    case ESP_RST_SW:
        return "SW"; // esp_restart() — incl. our health-wdt / OTA / C6-recover
    case ESP_RST_PANIC:
        return "PANIC"; // crash / abort()
    case ESP_RST_INT_WDT:
        return "INT_WDT";
    case ESP_RST_TASK_WDT:
        return "TASK_WDT"; // a task starved the task-wdt
    case ESP_RST_WDT:
        return "OTHER_WDT";
    case ESP_RST_DEEPSLEEP:
        return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:
        return "BROWNOUT"; // power sag
    case ESP_RST_SDIO:
        return "SDIO";
    case ESP_RST_USB:
        return "USB";
    case ESP_RST_JTAG:
        return "JTAG";
    default:
        return "UNKNOWN";
    }
}

static void emit(const status_snapshot_t *s)
{
    // Reboot-cause telemetry: usb_host_lib_main logs the reset reason via
    // ESP_LOGW at early boot (serial only, before the iot_log transport is up),
    // so on a wireless deployment the reason is invisible. Re-emit it over
    // iot_log from here — the one path proven to reach the receiver.
    //
    // Gate on WiFi being CONNECTED, not a raw window count: after a crash
    // reboot the STA sometimes re-associates slowly (~10 s), and a fixed
    // first-N-window budget is spent at uptime 1-6 s — before iot_log can
    // deliver — silently dropping both lines (observed 2026-07-12: a crash
    // whose recovery associated at ~10 s lost its PANIC-BT entirely). Also
    // defer CONSUMING the RTC record (panic_capture_report clears it) until
    // connected, so a slow-link boot doesn't burn the one-shot report into a
    // dropped sendto. Repeat a few windows once up for UDP-drop resilience.
    static int  s_rst_emits = 0;
    static bool s_bt_taken  = false;
    static bool s_have_bt   = false;
    static char s_bt[288];
    if (wifi_link_is_connected() && s_rst_emits < 8) {
        if (!s_bt_taken) {
            s_bt_taken = true;
            s_have_bt  = panic_capture_report(s_bt, sizeof(s_bt));
        }
        s_rst_emits++;
        esp_reset_reason_t rr = esp_reset_reason();
        iot_log(IOT_LOG_WARN, "BOOT reset_reason=%s (%d)", reset_reason_name(rr), (int)rr);
        if (s_have_bt) iot_log(IOT_LOG_ERROR, "%s", s_bt);
    }
    // Cache first, BEFORE the verbose/quiet fork: the STATUS-line fields the
    // /status page surfaces must be captured regardless of build config.
    s_last      = *s;
    s_have_last = true;

    // Capacity peak/mean tracking (see the s_cap_* statics). Same
    // worker_pct/dsp_pct formulas as the emit branches below, computed once
    // here so it runs regardless of the verbose/quiet build fork. A handful of
    // comparisons/sec on the low-prio logger task — negligible.
    if (s->window_us > 0) {
        double   wdiv = (double)s->window_us;
        float    dcap = (float)(100.0 * (double)s->dsp_total_time_us / wdiv);
        float    wcap = (float)(100.0 * ((double)s->ws.bursts_processed * (double)s->ws.avg_burst_us + (double)s->ws.bursts_triage_rejected * (double)s->ws.triage_rej_us) / wdiv);
        uint32_t b    = s->dsp.gone_bursts; // tagger bursts dispatched this window
        s_cap_windows++;
        s_cap_sum_bursts += b;
        if (b > s_cap_peak_bursts) s_cap_peak_bursts = b;
        if (dcap > s_cap_peak_dsp) s_cap_peak_dsp = dcap;
        if (wcap > s_cap_peak_worker) s_cap_peak_worker = wcap;
        if (wcap >= 90.0f) s_cap_worker_ge90++;

        uint32_t proc  = s->ws.bursts_processed;
        uint32_t qdrop = s->ws.bursts_dropped + s->ws.bursts_evicted;
        s_cap_sum_processed += proc;
        s_cap_sum_triagerej += s->ws.bursts_triage_rejected;
        if (proc > s_cap_peak_processed) s_cap_peak_processed = proc;
        if (qdrop > s_cap_peak_qdrops) s_cap_peak_qdrops = qdrop;
    }

    // Reception-environment classifier: fold this window's funnel into the EMAs
    // and re-classify (INTERFERENCE / MARGINAL / QUIET / GOOD). Runs every
    // window regardless of the verbose/quiet fork below.
    rx_update(s);

    double window_s = s->window_us / 1000000.0;
    if (window_s <= 0) window_s = 1.0;

    double rate_inst = (s->bytes_window / (1024.0 * 1024.0)) / window_s;

#if CONFIG_STATUS_LOG_VERBOSE
    double elapsed_s = s->elapsed_us / 1000000.0;
    if (elapsed_s <= 0) elapsed_s = 1.0;
    double rate_avg = (s->total_bytes / (1024.0 * 1024.0)) / elapsed_s;

    float avg_dsp_us  = (s->dsp_frame_count > 0)
                            ? (float)s->dsp_total_time_us / s->dsp_frame_count
                            : 0;
    float feed_us_avg = (s->feed_calls_window > 0)
                            ? (float)s->dsp_total_time_us / s->feed_calls_window
                            : 0;

    float xfer_fill = (s->us.total_requested_bytes > 0)
                          ? (100.0f * (float)s->us.total_actual_bytes / (float)s->us.total_requested_bytes)
                          : 0.0f;

    float feed_n      = (s->feed_calls_window > 0) ? (float)s->feed_calls_window : 1.0f;
    float read_us_avg = (float)s->cycle_read_us / feed_n;

    float ingest_n               = (s->ingest.dispatches > 0) ? (float)s->ingest.dispatches : 1.0f;
    float ingest_convert_us_avg  = (float)s->ingest.convert_us_total / ingest_n;
    float ingest_push_us_avg     = (float)s->ingest.push_us_total / ingest_n;
    float ingest_resample_us_avg = (float)s->ingest.resample_us_total / ingest_n;
    float ingest_sbpush_us_avg   = (float)s->ingest.sbpush_us_total / ingest_n;
    float ingest_wait_us_avg     = (s->ingest.consumer_waits > 0)
                                       ? (float)s->ingest.slot_wait_total_us / (float)s->ingest.consumer_waits
                                       : 0.0f;

    // Fill %: against the ACTUAL ring capacity (STREAM_RINGBUF_BYTES).
    // These divided by a stale 512 KB constant from the pre-4 MB era,
    // so the verbose line read up to ~800% at peak.
    float producer_peak_pct = 100.0f * (float)s->us.producer_rb_max_used / (float)STREAM_RINGBUF_BYTES;
    float drop_fill_pct     = 100.0f * (float)s->us.producer_rb_used_at_drop / (float)STREAM_RINGBUF_BYTES;

    ESP_LOGI(TAG, "USB: rate_inst=%.2f MB/s rate_avg=%.2f MB/s feed_calls=%u "
                  "(avg_per_call=%.0f us) PSRAM_free=%d",
             rate_inst, rate_avg, s->feed_calls_window, feed_us_avg,
             s->psram_free_bytes);

    ESP_LOGI(TAG, "USB-XFR: completed=%u short=%u (fill=%.1f%%) "
                  "rb_full_drops=%u status_err=%u resubmit_err=%u last_err=0x%02x",
             s->us.completed, s->us.short_xfers, xfer_fill,
             s->us.rb_full_drops, s->us.status_errors, s->us.resubmit_errors,
             s->us.last_error_status);

    ESP_LOGI(TAG, "USB-RB:  producer_peak_fill=%.1f%% drop_fill=%.1f%% "
                  "(producer_samples=%u)",
             producer_peak_pct, drop_fill_pct, s->us.producer_samples);

    ESP_LOGI(TAG, "Cycle (Core0 us avg): read=%.0f feed=%.0f",
             read_us_avg, feed_us_avg);

    // Per-iteration cycle breakdown. cycle_iterations is the raw
    // loop tick count over the window; if >> feed_calls then the loop
    // is spinning idle in handle_events. take_converted_us tells us
    // how often Core 0 blocks waiting for Core 1 ingest to finish.
    float ci_n            = (s->cycle_iterations > 0) ? (float)s->cycle_iterations : 1.0f;
    float he_avg_per_iter = (float)s->cycle_handle_events_us / ci_n;
    float tc_avg_per_feed = (s->feed_calls_window > 0)
                                ? (float)s->cycle_take_converted_us / (float)s->feed_calls_window
                                : 0.0f;
    float ci_per_sec      = ci_n * 1e6f / (float)(s->window_us > 0 ? s->window_us : 1);
    float he_pct          = 100.0f * (float)s->cycle_handle_events_us / (float)(s->window_us > 0 ? s->window_us : 1);
    float tc_pct          = 100.0f * (float)s->cycle_take_converted_us / (float)(s->window_us > 0 ? s->window_us : 1);
    ESP_LOGI(TAG, "Cycle (Core0): iter=%u (%.0f/s) handle_events=%.0f us/iter (%.1f%% of window)  "
                  "take_converted=%.0f us/feed (%.1f%% of window)",
             s->cycle_iterations, ci_per_sec,
             he_avg_per_iter, he_pct,
             tc_avg_per_feed, tc_pct);

    ESP_LOGI(TAG, "Ingest (Core1 us avg): convert=%.0f push=%.0f "
                  "(resample=%.0f sbpush=%.0f) "
                  "dispatches=%u consumer_waits=%u (avg_wait=%.0f us)",
             ingest_convert_us_avg, ingest_push_us_avg,
             ingest_resample_us_avg, ingest_sbpush_us_avg,
             s->ingest.dispatches, s->ingest.consumer_waits, ingest_wait_us_avg);

    // Capacity %: how much wall-clock each subsystem consumed in this
    // 1-second window. >100 % = falling behind, queue/buffer would
    // eventually overflow; values in the 80-100 % range are the early
    // warning that we're at the edge.
    //
    // DSP (Core 0 front end): continuous IQ stream, so the budget is
    // (dsp_total_time_us / window_us). If feed calls collectively take
    // more than the window's wall time, we can't keep up with the USB.
    //
    // Worker (Core 1 burst processing): bursty work, so the budget is
    // (sum of burst processing times / window_us). avg_burst_us is the
    // running mean of all processed bursts (not just this window), so
    // multiplying it by THIS WINDOW's processed count is an
    // approximation of "would this rate be sustainable" — exact if
    // per-burst times are stable, slightly off during a transient.
    //
    // window_us can be 0 on a degenerate window; guard the divide the
    // same way he_pct/tc_pct above do, otherwise dsp_pct/worker_pct go
    // to inf and the %u cast further down is UB.
    double window_us_div = (s->window_us > 0) ? (double)s->window_us : 1.0;
    double dsp_pct       = (s->window_us > 0)
                               ? 100.0 * (double)s->dsp_total_time_us / window_us_div
                               : 0.0;
    // P1.5a: triage-REJECTED bursts don't appear in bursts_processed /
    // avg_burst_us, but they still consume worker time (~one fast-pass
    // each) — add them so worker_cap reflects the real load.
    double worker_pct = (s->window_us > 0)
                            ? 100.0 * ((double)s->ws.bursts_processed * (double)s->ws.avg_burst_us + (double)s->ws.bursts_triage_rejected * (double)s->ws.triage_rej_us) / window_us_div
                            : 0.0;

    ESP_LOGI(TAG, "DSP: %u steps, total=%.0f us/step, cap=%.1f%% "
                  "[wind=%.0f fft=%.0f mag=%.0f detect=%.0f base=%.0f]",
             s->dsp_frame_count, avg_dsp_us, dsp_pct,
             s->dsp.wind_us, s->dsp.fft_us, s->dsp.mag_us,
             s->dsp.detect_us, s->dsp.baseline_us);

    // fbt: tagger diagnostics. Formatted HERE (Core 1, low prio) from
    // the raw snapshot fields — this line used to be emitted inside
    // dsp_processor_get_stage_stats on Core 0's hot loop.
    {
        uint32_t ts = s->dsp.tag_steps ? s->dsp.tag_steps : 1;
        ESP_LOGI(TAG,
                 "fbt: new=%u gone=%u coal=%u sq=%u sqdrop=%u sqreset=%u "
                 "frames=%u step_us=%u "
                 "wind=%.0f fft=%.0f mag=%.0f det=%.0f base=%.0f "
                 "(us/step, steps=%u)",
                 (unsigned)s->dsp.new_bursts, (unsigned)s->dsp.gone_bursts,
                 (unsigned)s->dsp.coalesced,
                 (unsigned)s->dsp.squelch_events,
                 (unsigned)s->dsp.squelch_dropped,
                 (unsigned)s->dsp.noise_resets,
                 (unsigned)s->dsp.frames, (unsigned)s->dsp.step_us,
                 s->dsp.wind_us, s->dsp.fft_us, s->dsp.mag_us,
                 s->dsp.detect_us, s->dsp.baseline_us, (unsigned)ts);
    }

    // bch_decoded is the REAL decode rate (BCH passed AND classify
    // returned a known frame type — task #111). bch_unknown is BCH
    // passed but iridium_frame_classify => UNKNOWN, i.e. BCH random-
    // noise false-positives. processed includes both plus qpsk_demod
    // successes that fail BCH outright. On the ALBQ raw fixture:
    // processed=58 typically resolves to bch_decoded=19 + bch_failed=27
    // + bch_skipped(short)=12.
    ESP_LOGI(TAG, "Worker: queued=%u dropped=%u evicted=%u processed=%u "
                  "bch_decoded=%u bch_unknown=%u bch_failed=%u "
                  "bch_chase=%u skipped=%u triage_rej=%u "
                  "qmax=%u avg_burst=%.0f us triage_rej_avg=%.0f us cap=%.1f%%",
             s->ws.bursts_queued, s->ws.bursts_dropped,
             s->ws.bursts_evicted, s->ws.bursts_processed,
             s->ws.bursts_bch_decoded, s->ws.bursts_bch_unknown,
             s->ws.bursts_bch_failed, s->ws.bursts_bch_chase_recovered,
             s->ws.bursts_skipped, s->ws.bursts_triage_rejected,
             s->ws.queue_high_water, s->ws.avg_burst_us,
             s->ws.triage_rej_us, worker_pct);

    ESP_LOGI(TAG, "Worker-stages (us): triage=%.0f extract=%.0f freq=%.0f "
                  "fir=%.0f demod=%.0f bch=%.0f",
             s->ws.triage_us, s->ws.extract_us, s->ws.freq_center_us,
             s->ws.fir_decim_us, s->ws.demod_us,
             s->ws.bch_us);
#else
    // Quiet mode: one line of essential health, plus a separate WARN line
    // only when an anomaly counter is nonzero. Real burst/decode events
    // (BURST DETECTED, DEMOD SUCCESS, BCH DECODE SUCCESS, Block1 Data:)
    // are unaffected — they log at their source regardless of this flag.
    //
    // Capacity %: see the verbose branch above for derivation. We
    // surface dsp_cap and worker_cap here too because they're a leading
    // indicator — drops only start once we cross 100 %, so seeing
    // "worker_cap=92%" lets the operator anticipate saturation a few
    // seconds before the first dropped burst.
    //
    // window_us can be 0 on a degenerate window; guard the divide (this
    // branch also feeds the (unsigned) cast below, which is UB on inf).
    double window_us_div = (s->window_us > 0) ? (double)s->window_us : 1.0;
    double dsp_pct       = (s->window_us > 0)
                               ? 100.0 * (double)s->dsp_total_time_us / window_us_div
                               : 0.0;
    // P1.5a: include triage-rejected bursts' wall time (see verbose
    // branch) so worker_cap reflects the fast-pass load too.
    double worker_pct = (s->window_us > 0)
                            ? 100.0 * ((double)s->ws.bursts_processed * (double)s->ws.avg_burst_us + (double)s->ws.bursts_triage_rejected * (double)s->ws.triage_rej_us) / window_us_div
                            : 0.0;

    // bch_decoded = real Iridium frame decodes (BCH pass AND classify
    // known type — task #111). bch_unknown = BCH false positives (random
    // noise corrected into valid codeword with no frame structure);
    // expect this to dominate over bch_decoded under marginal RF.
    //
    // steps= is dsp_frame_count: the number of 2048-sample FFT steps fed
    // through the tagger this window. It's proportional to the USB byte
    // rate by construction (fixed step size) and is NOT a burst or frame
    // rate — do not read it as "N bursts/sec". bursts= (dsp.gone_bursts)
    // is the actual tagger burst-dispatch rate: bursts the tagger
    // finished and handed off this window. This distinction was the
    // source of a 14-hour bench-monitoring misread (steps=~883/s at the
    // nominal USB rate was mistaken for a burst rate).
    // Listening band: LO centre + the ±FS_DETECT_HZ/2 window we cover.
    // Self-documents every capture (which band produced these bursts /
    // decodes) and lets a log reader confirm the tuned LO without the
    // serial [SCMD] interface.
    app_config_t cfg_snap;
    app_config_snapshot(&cfg_snap);
    double lo_mhz   = (double)cfg_snap.lo_freq_hz / 1e6;
    double half_mhz = ((double)FS_DETECT_HZ / 2.0) / 1e6;
    ESP_LOGI(TAG, "STATUS: rate=%.2f MB/s steps=%u bursts=%u processed=%u "
                  "triage_rej=%u bch_decoded=%u bch_unknown=%u drops=%u "
                  "dsp_cap=%.0f%% worker_cap=%.0f%% lo=%.4fMHz band=%.3f-%.3fMHz",
             rate_inst, s->dsp_frame_count, s->dsp.gone_bursts,
             s->ws.bursts_processed, s->ws.bursts_triage_rejected,
             s->ws.bursts_bch_decoded, s->ws.bursts_bch_unknown,
             s->us.rb_full_drops, dsp_pct, worker_pct,
             lo_mhz, lo_mhz - half_mhz, lo_mhz + half_mhz);
    // One machine-parseable STATUS line carries every field that used to also
    // be emitted as ~10 separate IOT_LOG METRIC packets per second — those were
    // redundant with this line and each cost an SDIO UDP send + CRC.
    // (iot_log_metric() remains available in the component API; we just don't
    // spam it every window. pk_acc/pk_qd were dropped from this line
    // 2026-07-18: since-boot peaks resent every second, redundant with
    // /status load{accepted_peak,queue_drops_peak} — the Option B backlog-
    // sizing question they scoped is closed.)
    // ACARS/SBD reception telemetry (since boot): lwda = ACARS-data frames
    // seen, lwda_bad = those that failed the IDA CRC/header gate (marginal SNR,
    // NOT fed downstream), sbd = SBD envelopes completed. Surfaces reception
    // activity + quality even when 0 messages complete — "we ARE hearing them,
    // N% are too weak to CRC" — instead of a silent 0. Full breakdown lives at
    // /diag/reassembler.
    frame_decoder_reasm_stats_t rs;
    frame_decoder_get_reasm_stats(&rs);
    uint32_t lwda_bad = (rs.lw_da >= rs.lw_da_valid) ? (uint32_t)(rs.lw_da - rs.lw_da_valid) : 0;
    worker_hot_stats_t hot; // A6 continuation-boost activity
    worker_core1_get_hot_stats(&hot);

    // Health-watchdog counters for reboot-cause tracing: gwf = consecutive
    // failed gateway pings (reboots at GW_FAIL_LIMIT), stall = consecutive
    // frozen-stream cycles. Watching these climb in the log tells us WHICH
    // watchdog fired before an SW reboot (gateway vs stream), and correlating
    // gwf's climb with an AUTOTUNE-START marker shows if the LO rescan is what
    // knocks the gateway offline.
    int gwf = 0, stall = 0;
    wifi_link_wdt_status(NULL, NULL, &gwf, NULL, &stall);

    // Heap-free trend (KB) — carried in the always-on STATUS line so a slow
    // leak is visible over the connectionless iot_log without HTTP polling
    // (the socket pool is itself a suspect). internal = general SRAM,
    // spiram = 32 MB PSRAM, dma = DMA-INT (the razor-thin USB URB budget).
    uint32_t sramf_kb  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;
    uint32_t psramf_kb = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
    uint32_t dmaf_kb   = heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024;

    iot_log(IOT_LOG_INFO,
            "STATUS rate=%.2f bch_dec=%lu bch_unk=%lu drops=%lu dsp=%u%% wk=%u%% "
            "bursts=%u pk_bursts=%u pk_wk=%u%% "
            "lwda=%lu lwda_bad=%lu sbd=%lu gwf=%d stall=%d "
            "dmaf=%lu sramf=%lu psramf=%lu hot=%lu/%lu",
            rate_inst,
            (unsigned long)s->ws.bursts_bch_decoded,
            (unsigned long)s->ws.bursts_bch_unknown,
            (unsigned long)s->us.rb_full_drops,
            (unsigned)dsp_pct, (unsigned)worker_pct,
            (unsigned)s->dsp.gone_bursts, (unsigned)s_cap_peak_bursts,
            (unsigned)s_cap_peak_worker,
            (unsigned long)rs.lw_da, (unsigned long)lwda_bad,
            (unsigned long)rs.sbd_complete, gwf, stall,
            (unsigned long)dmaf_kb, (unsigned long)sramf_kb, (unsigned long)psramf_kb,
            (unsigned long)hot.published, (unsigned long)hot.boost_pops);

    // Warn proactively when EITHER subsystem crosses 80 % capacity OR
    // any drop / recovery counter ticks. Field names match
    // /diag/recovery_counters exactly — same names in log and endpoint.
    // Component prefixes (usb./sb./ing.) so grepping for one
    // component's counters is straightforward.
    uint32_t sb_fails         = signal_buffer_stash_alloc_fails();
    uint32_t sb_recoveries    = signal_buffer_stash_alloc_recoveries();
    uint32_t sb_dma_to        = signal_buffer_dma_timeouts();
    uint32_t ic_disp_drops    = ingest_core1_dispatch_drops();
    uint32_t ic_slow_waits    = ingest_core1_take_converted_slow_waits();
    uint32_t ic_raw_slow_wait = ingest_core1_raw_done_slow_waits(); // T49a
    uint32_t lu_pool_lost     = esp_libusb_xfer_pool_lost();
    uint32_t sb_audio_drop    = (sb_fails > sb_recoveries) ? (sb_fails - sb_recoveries) : 0;

    uint32_t dma_free    = heap_caps_get_free_size(MALLOC_CAP_DMA);
    uint32_t dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);

    // Trigger on DELTAS of the cumulative accessors, not their absolute
    // values: those counters never reset, so one residual event (e.g.
    // the documented ~110 split-RX errors/min era) used to make the
    // STATUS-ERR line fire every second forever, burying real events.
    // The line still prints cumulative totals — only the trigger is
    // delta-based. (emit() runs on the single logger task; statics are
    // safe.)
    static uint32_t prev_sb_fails = 0, prev_sb_dma_to = 0,
                    prev_ic_disp_drops = 0, prev_ic_slow_waits = 0,
                    prev_ic_raw_slow_wait = 0, prev_lu_pool_lost = 0;
    bool any_recovery = (sb_fails != prev_sb_fails) ||
                        (sb_dma_to != prev_sb_dma_to) ||
                        (ic_disp_drops != prev_ic_disp_drops) ||
                        (ic_slow_waits != prev_ic_slow_waits) ||
                        (ic_raw_slow_wait != prev_ic_raw_slow_wait) ||
                        (lu_pool_lost != prev_lu_pool_lost);
    prev_sb_fails         = sb_fails;
    prev_sb_dma_to        = sb_dma_to;
    prev_ic_disp_drops    = ic_disp_drops;
    prev_ic_slow_waits    = ic_slow_waits;
    prev_ic_raw_slow_wait = ic_raw_slow_wait;
    prev_lu_pool_lost     = lu_pool_lost;

    bool over_capacity = (dsp_pct > 80.0) || (worker_pct > 80.0);
    if (over_capacity || s->us.rb_full_drops || s->us.status_errors ||
        s->us.resubmit_errors || s->ws.bursts_dropped ||
        s->ws.bursts_evicted || s->dsp.squelch_events || any_recovery) {
        // rb_peak: the window's peak usbring fill as % of capacity — the
        // leading indicator for rb_full overflow (drops start at 100%).
        unsigned rb_peak_pct = (unsigned)(100.0f * (float)s->us.producer_rb_max_used /
                                          (float)STREAM_RINGBUF_BYTES);
        ESP_LOGW(TAG,
                 "STATUS-ERR: cap[dsp=%.0f%% worker=%.0f%%] "
                 "usb[rb_full=%u rb_peak=%u%% status_err=%u resubmit_err=%u pool_lost=%u last=0x%02x] "
                 "fbt[sq=%u sqdrop=%u sqreset=%u] "
                 "worker[dropped=%u evicted=%u] "
                 "sb[stash_fails=%u recoveries=%u audio_dropped=%u dma_timeouts=%u] "
                 "ing[dispatch_drops=%u slow_waits=%u raw_slow_waits=%u] "
                 "heap[dma_free=%uKB dma_largest=%uKB]",
                 dsp_pct, worker_pct,
                 s->us.rb_full_drops, rb_peak_pct, s->us.status_errors,
                 s->us.resubmit_errors, lu_pool_lost, s->us.last_error_status,
                 s->dsp.squelch_events, s->dsp.squelch_dropped,
                 s->dsp.noise_resets,
                 s->ws.bursts_dropped, s->ws.bursts_evicted,
                 sb_fails, sb_recoveries, sb_audio_drop, sb_dma_to,
                 ic_disp_drops, ic_slow_waits, ic_raw_slow_wait,
                 dma_free / 1024, dma_largest / 1024);
    }
#endif
}

static void logger_task(void *arg)
{
    (void)arg;
    status_snapshot_t snap;
    while (1) {
        // 1100 ms timeout so iot_log_poll() runs at ~1 Hz even when no USB
        // data is flowing (portMAX_DELAY would block mDNS discovery).
        if (xQueueReceive(s_queue, &snap, pdMS_TO_TICKS(1100)) == pdTRUE) {
            emit(&snap);
        }
        iot_log_poll();

        // uart_log AUTO self-heal: re-evaluate the network-gated console mute
        // every ~1 Hz so WiFi connecting/dropping takes effect within ~1 s
        // with no reboot required (app_config.h 3-state model). OFF/ON modes
        // just re-assert the same forced value each pass — a cheap pointer
        // swap (esp_log_set_vprintf), safe at 1 Hz.
        uart_log_apply(app_config_uart_log_effective(app_config_get_uart_log_mode(),
                                                      wifi_link_is_connected()));
    }
}

esp_err_t status_logger_init(void)
{
    // Queue in PSRAM: 2 × sizeof(status_snapshot_t) (~600 B each = ~1.2 KB)
    // — 1 Hz traffic, latency irrelevant, no reason to take DMA-INT.
    s_queue = xQueueCreateWithCaps(2, sizeof(status_snapshot_t),
                                   MALLOC_CAP_SPIRAM);
    if (!s_queue) return ESP_ERR_NO_MEM;

    // PSRAM stack — see feedback_task_stacks_in_psram memory note.
    // 1 Hz periodic logging; PSRAM stack overhead is negligible.
    //
    // Priority 6 = above worker_core1 (4) on Core 1 (frame_decoder now
    // lives on Core 0 at prio 6 — see frame_decoder.c DECODER_CORE) so
    // the logger always gets its formatting slot even when the worker
    // has a sustained backlog of bursts. Without this, under heavy
    // noise (10 dB tagger threshold, ~145 bursts/sec) the worker
    // preempted the logger indefinitely and the STATUS line
    // disappeared — diagnostics vanished exactly at saturation.
    // CAVEAT (topology review 2026-07-17 §F1): being above the worker
    // means this task's UART output steals from the worker during
    // passes; the console TX path is a busy-spin, so keep STATUS lines
    // lean and keep the console baud high (921600).
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(logger_task, "status_logger",
                                                    6144, NULL, 6, NULL, 1,
                                                    MALLOC_CAP_SPIRAM);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

bool status_logger_post(const status_snapshot_t *snap)
{
    if (!s_queue) return false;
    return xQueueSend(s_queue, snap, 0) == pdTRUE;
}
