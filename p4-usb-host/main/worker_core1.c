// worker_core1 — per-burst processing chain (Core 1 task).
//
// Phase 3.6.M cutover (commit 1e386b7): replaces the per-channel
// extract → freq-shift → 40k→250k resample chain with the wideband
// direct-IF path. Each burst now arrives from dsp_processor with a
// precise rel_freq_hz tag (from the FFT bin), and the worker:
//   1. extracts a wideband window from the 2.5 MSPS signal_buffer
//   2. rotates the burst to DC using absolute-phase float rotation
//      (cosf/sinf per sample — see memory note about Q15 incremental
//      phasor magnitude decay over long windows)
//   3. 10× decimates 2.5 MSPS → 250 ksps via direct_if_decim's
//      gri-exact 279-tap Kaiser FIR
//   4. hands the 250 ksps burst to burst_pipeline for D13 / CFO /
//      RRC / UW / pre-rotate / demod
//
// Validated on host as gr-iridium per-stage equivalent (NMSE ≤ −17 dB,
// phase coherence ≥ 0.993 on burst id=30) — commit de72f24.

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "worker_core1.h"
#include "signal_buffer.h"
#include "dsp_processor.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"
#include "burst_pipeline.h"
#include "direct_if_decim.h"
#include "rotate_to_dc.h"
#include "bch_decoder.h"
#include "frame_decoder.h"

static const char *TAG = "WORKER1";

static QueueHandle_t burst_queue = NULL;

// Diagnostic counters. Read & reset by worker_core1_get_stats().
static volatile uint32_t s_bursts_queued = 0;
static volatile uint32_t s_bursts_dropped = 0;
static volatile uint32_t s_bursts_processed = 0;
static volatile uint32_t s_bursts_skipped = 0;
static volatile uint32_t s_queue_high_water = 0;
static volatile uint64_t s_burst_total_us = 0;

// Per-stage timing accumulators, summed over processed bursts only.
static volatile uint64_t s_t_extract_us = 0;
static volatile uint64_t s_t_rotate_us  = 0;
static volatile uint64_t s_t_decim_us   = 0;
static volatile uint64_t s_t_pipeline_us = 0;
static volatile uint64_t s_t_bch_us     = 0;

// Buffer sizes derived from the wideband path's burst window. At
// FS_DETECT_HZ = 2.5 MSPS the tagger publishes length_samples =
// FBT_BURST_POST_LEN (40000 ≈ 16 ms). Add NTAPS-1 of pre-pad so the
// FIR transient lands before the burst proper, then a small safety
// margin.
#define WB_PRE_PAD_SAMPLES   (DIDECIM_NTAPS - 1)
#define WB_EXTRACT_SAFETY    1024
#define WB_EXTRACT_MAX       (40000 + WB_PRE_PAD_SAMPLES + WB_EXTRACT_SAFETY)

// 250 ksps output is at most WB_EXTRACT_MAX / 10 + 1.
#define WB_DECIM_MAX         ((WB_EXTRACT_MAX / DIDECIM_DECIM) + 8)

static int16_t          *s_extract_buf  = NULL;   // 2.5 MSPS wideband window
static int16_t          *s_decim_buf    = NULL;   // 250 ksps post-decim
static direct_if_decim_t s_decim;

// (Absolute-phase rotation lives in common/iridium_decoder/rotate_to_dc.{h,c}
// — shared with the host wideband test. Per-sample cosf/sinf on P4's
// scalar FPU is acceptable for first cutover; task #58 covers a
// cordic/table replacement if profiling shows it dominates.)

void worker_task(void *arg)
{
    ESP_LOGI(TAG, "Worker Task started on Core %d", xPortGetCoreID());
    detected_burst_t burst;

    while (1) {
        if (xQueueReceive(burst_queue, &burst, portMAX_DELAY)) {
            int64_t burst_t0 = esp_timer_get_time();
            ESP_LOGI(TAG, "Worker burst: start=%lu len=%lu rel=%+.0f Hz SNR=%.1f dB",
                     (unsigned long)burst.start_sample_idx,
                     (unsigned long)burst.length_samples,
                     (double)burst.rel_freq_hz,
                     (double)burst.peak_snr_db);

            // Length guard — need enough samples to reach the FIR
            // transient plus the burst content. Wideband detector
            // publishes the gri-default post_len of 40000 samples
            // (~16 ms at 2.5 MSPS), so this is normally fine.
            if (burst.length_samples < 128) {
                s_bursts_skipped++;
                continue;
            }
            uint32_t ext_len = burst.length_samples + WB_PRE_PAD_SAMPLES;
            if (ext_len > WB_EXTRACT_MAX) {
                ext_len = WB_EXTRACT_MAX;
            }

            // 1. Extract wideband window from signal_buffer at FS_DETECT_HZ.
            // start_sample_idx is the burst start in tagger frame; back up
            // by WB_PRE_PAD_SAMPLES so the FIR transient finishes before
            // the burst proper.
            uint32_t ext_start = burst.start_sample_idx
                                  - WB_PRE_PAD_SAMPLES;
            int64_t t_ext0 = esp_timer_get_time();
            signal_buffer_extract(ext_start, ext_len, s_extract_buf);
            int64_t t_ext1 = esp_timer_get_time();
            s_t_extract_us += (uint64_t)(t_ext1 - t_ext0);

            // 2. Absolute-phase rotation to DC. fs is the detector rate
            // (2.5 MSPS in step 1 cutover; ingest_core1 resamples
            // 2.56 → 2.5 before signal_buffer_push).
            int64_t t_rot0 = esp_timer_get_time();
            double phase_step = -2.0 * M_PI * (double)burst.rel_freq_hz
                                 / (double)FS_DETECT_HZ;
            rotate_to_dc(s_extract_buf, (int)ext_len, phase_step);
            int64_t t_rot1 = esp_timer_get_time();
            s_t_rotate_us += (uint64_t)(t_rot1 - t_rot0);

            // 3. 10× decim 2.5 MSPS → 250 ksps with gri's 279-tap Kaiser.
            int64_t t_dec0 = esp_timer_get_time();
            int n_250k = direct_if_decim_process(&s_decim,
                                                  s_extract_buf, (int)ext_len,
                                                  s_decim_buf);
            int64_t t_dec1 = esp_timer_get_time();
            s_t_decim_us += (uint64_t)(t_dec1 - t_dec0);

            if (n_250k <= 64) {
                ESP_LOGD(TAG, "direct_if_decim produced %d samples — too short",
                         n_250k);
                s_bursts_skipped++;
                continue;
            }

            // 4. Per-burst pipeline at 250 ksps.
            int64_t t_pipe0 = esp_timer_get_time();
            burst_pipeline_result_t bres;
            burst_pipeline_process_250khz(s_decim_buf, n_250k, &bres);
            int64_t t_pipe1 = esp_timer_get_time();
            s_t_pipeline_us += (uint64_t)(t_pipe1 - t_pipe0);

            ESP_LOGI(TAG, "D13 start=%d/%d  UW dir=%s off=%d corr=%.3f SNR=%.1f omega=%.3f",
                     bres.burst_start, n_250k,
                     bres.uw_res.direction == UW_DIR_DOWNLINK ? "DL" :
                     bres.uw_res.direction == UW_DIR_UPLINK   ? "UL" : "??",
                     bres.uw_res.uw_offset,
                     (double)bres.uw_res.correction,
                     (double)bres.uw_res.snr_estimate_db,
                     (double)bres.uw_res.omega_per_sym);

            decoded_frame_t frame = bres.frame;
            int64_t t_bch0 = esp_timer_get_time();
            if (bres.demod_ok) {
                ESP_LOGI(TAG, "DEMOD SUCCESS: %s frame (%d bits)",
                         frame.direction == DIR_DOWNLINK ? "DL" : "UL",
                         frame.n_bits);

                if (frame.n_bits >= 24 + 64) {
                    const uint8_t *payload = frame.bits + 24;
                    uint8_t block1[32], block2[32];
                    uint8_t data1[21], data2[21];

                    iridium_deinterleave(payload, block1, block2);
                    int e1 = bch_decode_block(block1, data1);
                    int e2 = bch_decode_block(block2, data2);

                    if (e1 >= 0 && e2 >= 0) {
                        ESP_LOGI(TAG, "BCH DECODE SUCCESS! Errors: %d, %d",
                                 e1, e2);
                    }
                }

                frame_decoder_push(frame.bits, frame.n_bits,
                                   frame.direction, 0u,
                                   burst.peak_bin, burst.peak_snr_db);
                free(frame.bits);
            }
            int64_t t_bch1 = esp_timer_get_time();
            s_t_bch_us += (uint64_t)(t_bch1 - t_bch0);

            s_bursts_processed++;
            s_burst_total_us += (uint64_t)(esp_timer_get_time() - burst_t0);
        }

        // Periodic yield: worker is at prio 5; the frame_decoder at
        // prio 4 must get scheduled. One vTaskDelay(1) every 8 bursts
        // gives ~10 ms of frame_decoder CPU per 8 bursts processed.
        static int yield_counter = 0;
        if (++yield_counter >= 8) {
            yield_counter = 0;
            vTaskDelay(1);
        }
    }
}

esp_err_t worker_core1_init(void)
{
    burst_queue = xQueueCreate(16, sizeof(detected_burst_t));
    if (!burst_queue) return ESP_ERR_NO_MEM;

    // Wideband buffers in PSRAM. The decim buffer can be smaller
    // (10× decimation) but is sized generously since PSRAM is
    // plentiful.
    size_t ext_bytes  = (size_t)WB_EXTRACT_MAX * 2 * sizeof(int16_t);
    size_t dec_bytes  = (size_t)WB_DECIM_MAX   * 2 * sizeof(int16_t);
    s_extract_buf = heap_caps_malloc(ext_bytes, MALLOC_CAP_SPIRAM);
    s_decim_buf   = heap_caps_malloc(dec_bytes, MALLOC_CAP_SPIRAM);
    if (!s_extract_buf || !s_decim_buf) {
        ESP_LOGE(TAG, "Worker buffer alloc failed (ext=%p dec=%p)",
                 s_extract_buf, s_decim_buf);
        return ESP_ERR_NO_MEM;
    }

    // direct_if_decim is idempotent at init; safe to call here.
    direct_if_decim_init(&s_decim);

    xTaskCreatePinnedToCore(worker_task, "worker_core1", 16384, NULL,
                             5, NULL, 1);
    return ESP_OK;
}

void worker_core1_push_burst(const detected_burst_t *burst)
{
    if (burst_queue) {
        s_bursts_queued++;
        UBaseType_t depth = uxQueueMessagesWaiting(burst_queue);
        if (depth > s_queue_high_water) s_queue_high_water = depth;
        if (xQueueSend(burst_queue, burst, 0) != pdTRUE) {
            s_bursts_dropped++;
        }
    }
}

void worker_core1_get_stats(worker_stats_t *out)
{
    uint32_t n = s_bursts_processed;
    out->bursts_queued    = s_bursts_queued;
    out->bursts_dropped   = s_bursts_dropped;
    out->bursts_processed = n;
    out->bursts_skipped   = s_bursts_skipped;
    out->queue_high_water = s_queue_high_water;
    if (n > 0) {
        float fn = (float)n;
        out->avg_burst_us    = (float)s_burst_total_us / fn;
        out->extract_us      = (float)s_t_extract_us   / fn;
        out->freq_center_us  = (float)s_t_rotate_us    / fn;
        out->fir_decim_us    = (float)s_t_decim_us     / fn;
        out->resample_us     = 0.0f;
        out->demod_us        = (float)s_t_pipeline_us  / fn;
        out->bch_us          = (float)s_t_bch_us       / fn;
    } else {
        out->avg_burst_us = out->extract_us = out->freq_center_us =
            out->fir_decim_us = out->resample_us = out->demod_us =
            out->bch_us = 0.0f;
    }
    s_bursts_queued = 0;
    s_bursts_dropped = 0;
    s_bursts_processed = 0;
    s_bursts_skipped = 0;
    s_queue_high_water = 0;
    s_burst_total_us = 0;
    s_t_extract_us = s_t_rotate_us = s_t_decim_us = s_t_pipeline_us
        = s_t_bch_us = 0;
}
