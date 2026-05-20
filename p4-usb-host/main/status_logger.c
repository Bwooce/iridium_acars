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
#include "esp_log.h"
#include "status_logger.h"

static const char *TAG = "CLASS";  // match the original tag for log continuity

static QueueHandle_t s_queue;

static void emit(const status_snapshot_t *s)
{
    double window_s  = s->window_us  / 1000000.0;
    if (window_s <= 0)  window_s = 1.0;

    double rate_inst = (s->bytes_window / (1024.0 * 1024.0)) / window_s;

#if CONFIG_STATUS_LOG_VERBOSE
    double elapsed_s = s->elapsed_us / 1000000.0;
    if (elapsed_s <= 0) elapsed_s = 1.0;
    double rate_avg  = (s->total_bytes  / (1024.0 * 1024.0)) / elapsed_s;

    float avg_dsp_us = (s->dsp_frame_count > 0)
        ? (float)s->dsp_total_time_us / s->dsp_frame_count : 0;
    float feed_us_avg = (s->feed_calls_window > 0)
        ? (float)s->dsp_total_time_us / s->feed_calls_window : 0;

    float xfer_fill = (s->us.total_requested_bytes > 0)
        ? (100.0f * (float)s->us.total_actual_bytes / (float)s->us.total_requested_bytes)
        : 0.0f;

    float feed_n = (s->feed_calls_window > 0) ? (float)s->feed_calls_window : 1.0f;
    float read_us_avg = (float)s->cycle_read_us / feed_n;

    float ingest_n = (s->ingest.dispatches > 0) ? (float)s->ingest.dispatches : 1.0f;
    float ingest_convert_us_avg = (float)s->ingest.convert_us_total / ingest_n;
    float ingest_push_us_avg    = (float)s->ingest.push_us_total    / ingest_n;
    float ingest_wait_us_avg    = (s->ingest.consumer_waits > 0)
        ? (float)s->ingest.slot_wait_total_us / (float)s->ingest.consumer_waits : 0.0f;

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

    ESP_LOGI(TAG, "Ingest (Core1 us avg): convert=%.0f push=%.0f "
                  "dispatches=%u consumer_waits=%u (avg_wait=%.0f us)",
             ingest_convert_us_avg, ingest_push_us_avg,
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
    double dsp_pct    = 100.0 * (double)s->dsp_total_time_us
                              / (double)s->window_us;
    double worker_pct = 100.0 * (double)s->ws.bursts_processed
                              * (double)s->ws.avg_burst_us
                              / (double)s->window_us;

    ESP_LOGI(TAG, "DSP: %u frames, total=%.0f us/frame, cap=%.1f%% "
                  "[wind=%.0f fft=%.0f mag=%.0f detect=%.0f base=%.0f]",
             s->dsp_frame_count, avg_dsp_us, dsp_pct,
             s->dsp.wind_us, s->dsp.fft_us, s->dsp.mag_us,
             s->dsp.detect_us, s->dsp.baseline_us);

    // bch_decoded is the REAL decode rate (qpsk_demod + BCH passed);
    // processed includes false positives where qpsk_demod found UW
    // but the frame's BCH-protected payload was uncorrectable.
    // On the ALBQ raw fixture: processed=58 typically resolves to
    // bch_decoded=19 + bch_failed=27 + bch_skipped(short)=12.
    ESP_LOGI(TAG, "Worker: queued=%u dropped=%u processed=%u "
                  "bch_decoded=%u bch_failed=%u skipped=%u "
                  "qmax=%u avg_burst=%.0f us cap=%.1f%%",
             s->ws.bursts_queued, s->ws.bursts_dropped, s->ws.bursts_processed,
             s->ws.bursts_bch_decoded, s->ws.bursts_bch_failed,
             s->ws.bursts_skipped, s->ws.queue_high_water, s->ws.avg_burst_us,
             worker_pct);

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
    double dsp_pct    = 100.0 * (double)s->dsp_total_time_us
                              / (double)s->window_us;
    double worker_pct = 100.0 * (double)s->ws.bursts_processed
                              * (double)s->ws.avg_burst_us
                              / (double)s->window_us;

    // bch_decoded = real Iridium frame decodes (qpsk_demod + BCH pass).
    // processed is the qpsk_demod success count (includes BCH false-
    // positives). They diverge when borderline-SNR bursts pass UW
    // correlation but fail BCH.
    ESP_LOGI(TAG, "STATUS: rate=%.2f MB/s frames=%u processed=%u "
                  "bch_decoded=%u drops=%u dsp_cap=%.0f%% worker_cap=%.0f%%",
             rate_inst, s->dsp_frame_count,
             s->ws.bursts_processed, s->ws.bursts_bch_decoded,
             s->us.rb_full_drops, dsp_pct, worker_pct);

    // Warn proactively when EITHER subsystem crosses 80 % capacity OR
    // any drop counter ticks — operator gets the heads-up before the
    // queue overflows.
    bool over_capacity = (dsp_pct > 80.0) || (worker_pct > 80.0);
    if (over_capacity || s->us.rb_full_drops || s->us.status_errors ||
        s->us.resubmit_errors  || s->ws.bursts_dropped) {
        ESP_LOGW(TAG, "STATUS-ERR: dsp_cap=%.0f%% worker_cap=%.0f%% "
                      "rb_full_drops=%u status_err=%u resubmit_err=%u "
                      "worker_dropped=%u last_err=0x%02x",
                 dsp_pct, worker_pct,
                 s->us.rb_full_drops, s->us.status_errors,
                 s->us.resubmit_errors, s->ws.bursts_dropped,
                 s->us.last_error_status);
    }
#endif
}

static void logger_task(void *arg)
{
    status_snapshot_t snap;
    while (1) {
        if (xQueueReceive(s_queue, &snap, portMAX_DELAY) == pdTRUE) {
            emit(&snap);
        }
    }
}

esp_err_t status_logger_init(void)
{
    s_queue = xQueueCreate(2, sizeof(status_snapshot_t));
    if (!s_queue) return ESP_ERR_NO_MEM;

    BaseType_t ok = xTaskCreatePinnedToCore(logger_task, "status_logger",
                                            6144, NULL, 1, NULL, 1);
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

bool status_logger_post(const status_snapshot_t *snap)
{
    if (!s_queue) return false;
    return xQueueSend(s_queue, snap, 0) == pdTRUE;
}
