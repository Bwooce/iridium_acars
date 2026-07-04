// Status-logger task. See status_logger.h for the why.
//
// The implementation is straightforward: one queue (depth 2), one task
// pinned to Core 1 at priority 1 (lower than ingest at 8 and worker at
// 5, so it never preempts the hot paths). The task blocks on
// xQueueReceive forever; class_driver posts a snapshot once per second.

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "status_logger.h"
#include "esp_iot_log.h"
#include "signal_buffer.h"
#include "ingest_core1.h"
#include "esp_libusb.h"

static const char *TAG = "CLASS"; // match the original tag for log continuity

static QueueHandle_t s_queue;

static void emit(const status_snapshot_t *s)
{
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

    float producer_peak_pct = 100.0f * (float)s->us.producer_rb_max_used / (512.0f * 1024.0f);
    float drop_fill_pct     = 100.0f * (float)s->us.producer_rb_used_at_drop / (512.0f * 1024.0f);

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
    double dsp_pct    = 100.0 * (double)s->dsp_total_time_us / (double)s->window_us;
    double worker_pct = 100.0 * (double)s->ws.bursts_processed * (double)s->ws.avg_burst_us / (double)s->window_us;

    ESP_LOGI(TAG, "DSP: %u frames, total=%.0f us/frame, cap=%.1f%% "
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
                 "fbt: new=%u gone=%u frames=%u step_us=%u "
                 "wind=%.0f fft=%.0f mag=%.0f det=%.0f base=%.0f "
                 "(us/step, steps=%u)",
                 (unsigned)s->dsp.new_bursts, (unsigned)s->dsp.gone_bursts,
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
    ESP_LOGI(TAG, "Worker: queued=%u dropped=%u processed=%u "
                  "bch_decoded=%u bch_unknown=%u bch_failed=%u "
                  "bch_chase=%u skipped=%u "
                  "qmax=%u avg_burst=%.0f us cap=%.1f%%",
             s->ws.bursts_queued, s->ws.bursts_dropped, s->ws.bursts_processed,
             s->ws.bursts_bch_decoded, s->ws.bursts_bch_unknown,
             s->ws.bursts_bch_failed, s->ws.bursts_bch_chase_recovered,
             s->ws.bursts_skipped,
             s->ws.queue_high_water, s->ws.avg_burst_us, worker_pct);

    ESP_LOGI(TAG, "Worker-stages (us): extract=%.0f freq=%.0f fir=%.0f "
                  "resamp=%.0f demod=%.0f bch=%.0f",
             s->ws.extract_us, s->ws.freq_center_us, s->ws.fir_decim_us,
             s->ws.resample_us, s->ws.demod_us, s->ws.bch_us);
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
    double dsp_pct    = 100.0 * (double)s->dsp_total_time_us / (double)s->window_us;
    double worker_pct = 100.0 * (double)s->ws.bursts_processed * (double)s->ws.avg_burst_us / (double)s->window_us;

    // bch_decoded = real Iridium frame decodes (BCH pass AND classify
    // known type — task #111). bch_unknown = BCH false positives (random
    // noise corrected into valid codeword with no frame structure);
    // expect this to dominate over bch_decoded under marginal RF.
    ESP_LOGI(TAG, "STATUS: rate=%.2f MB/s frames=%u processed=%u "
                  "bch_decoded=%u bch_unknown=%u drops=%u "
                  "dsp_cap=%.0f%% worker_cap=%.0f%%",
             rate_inst, s->dsp_frame_count,
             s->ws.bursts_processed, s->ws.bursts_bch_decoded,
             s->ws.bursts_bch_unknown,
             s->us.rb_full_drops, dsp_pct, worker_pct);
    iot_log(IOT_LOG_INFO,
            "STATUS rate=%.2f bch_dec=%lu bch_unk=%lu drops=%lu dsp=%u%% wk=%u%%",
            rate_inst,
            (unsigned long)s->ws.bursts_bch_decoded,
            (unsigned long)s->ws.bursts_bch_unknown,
            (unsigned long)s->us.rb_full_drops,
            (unsigned)dsp_pct, (unsigned)worker_pct);
    iot_log_metric("rate_x100", (int32_t)(rate_inst * 100));
    iot_log_metric("bch_dec", (int32_t)s->ws.bursts_bch_decoded);
    iot_log_metric("drops", (int32_t)s->us.rb_full_drops);
    iot_log_metric("dsp_cap", (int32_t)dsp_pct);
    iot_log_metric("wk_cap", (int32_t)worker_pct);

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
        s->us.resubmit_errors || s->ws.bursts_dropped || any_recovery) {
        ESP_LOGW(TAG,
                 "STATUS-ERR: cap[dsp=%.0f%% worker=%.0f%%] "
                 "usb[rb_full=%u status_err=%u resubmit_err=%u pool_lost=%u last=0x%02x] "
                 "worker[dropped=%u] "
                 "sb[stash_fails=%u recoveries=%u audio_dropped=%u dma_timeouts=%u] "
                 "ing[dispatch_drops=%u slow_waits=%u raw_slow_waits=%u] "
                 "heap[dma_free=%uKB dma_largest=%uKB]",
                 dsp_pct, worker_pct,
                 s->us.rb_full_drops, s->us.status_errors,
                 s->us.resubmit_errors, lu_pool_lost, s->us.last_error_status,
                 s->ws.bursts_dropped,
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
    // Priority 6 = above frame_decoder (4) and worker (3) on Core 1
    // so the logger always gets its sub-millisecond formatting slot
    // even when the worker has a sustained backlog of bursts. This
    // 1 Hz spike can't starve the lower-prio tasks — it's ~200 µs
    // of CPU per second. Without this, under heavy noise (10 dB
    // tagger threshold, ~145 bursts/sec) the worker preempted the
    // logger indefinitely and the STATUS line disappeared.
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
