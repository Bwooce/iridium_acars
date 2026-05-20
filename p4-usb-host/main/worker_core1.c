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

// Buffer sizes for the wideband per-burst window. The tagger
// publishes variable-length bursts via gri's gone-event semantics
// (start → stop = last_active + burst_post_len). Single-frame
// bursts span ~16 ms; multi-frame bursts (gri's
// handle_multiple_frames_per_burst case) often run 50-100 ms, with
// gri capping at max_burst_len = sample_rate * 0.09 = 225 ms.
// We size to 250 ms (multiple of 16 complex = 64-byte aligned) so
// any realistic burst fits without truncation.
//
// Cache alignment: tagger reports burst.start_sample_idx as a
// multiple of FBT_FFT_SIZE (2048), so the address (start × 4 bytes
// per complex) is naturally 64-byte aligned. We subtract
// WB_PRE_PAD_SAMPLES; making that subtraction a multiple of 16
// complex samples (= 64 bytes) keeps the extract address aligned
// for esp_cache_msync. DIDECIM_NTAPS - 1 = 279; round up to 288.
#define WB_PRE_PAD_SAMPLES    288     // 18 × 16, ≥ DIDECIM_NTAPS - 1
#define WB_MAX_BURST_SAMPLES  ((int)(FS_DETECT_HZ / 4))   // 250 ms = 625000
#define WB_EXTRACT_SAFETY     1024
#define WB_EXTRACT_MAX        (WB_MAX_BURST_SAMPLES + WB_PRE_PAD_SAMPLES \
                                + WB_EXTRACT_SAFETY)

// 250 ksps output is at most WB_EXTRACT_MAX / 10 + 1.
#define WB_DECIM_MAX          ((WB_EXTRACT_MAX / DIDECIM_DECIM) + 8)

// Per-chunk size for the streaming decim. The PIE FIR scratch needs
// to be in INTERNAL SRAM (dsps_fird_s16_arp4's `esp.vld.128.ip` can't
// service PSRAM); chunking keeps that scratch tiny. 4000 input samples
// per chunk = 8 KB per I/Q channel = ~17 KB total internal SRAM
// (vs ~200 KB if we tried full-burst scratch). 4000 is a multiple of
// DIDECIM_DECIM=10 and of 8 (PIE alignment), so each chunk consumes
// exactly N inputs and produces exactly N/10 outputs; no boundary
// math needed inside the loop. The streaming FIR delay-line state
// in `s_decim` is reset once at the start of each burst (so leftover
// history from previous bursts doesn't bleed in), then preserves
// naturally across chunks within a burst.
#define DECIM_CHUNK_IN        4000

static int16_t          *s_extract_buf  = NULL;   // 2.5 MSPS wideband window
static int16_t          *s_decim_buf    = NULL;   // 250 ksps post-decim
// Deinterleave scratch for direct_if_decim_process_split (PSRAM —
// no DMA so plain heap_caps_malloc with MALLOC_CAP_SPIRAM is fine).
static int16_t          *s_decim_scr_in_i  = NULL;
static int16_t          *s_decim_scr_in_q  = NULL;
static int16_t          *s_decim_scr_out_i = NULL;
static int16_t          *s_decim_scr_out_q = NULL;
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

            // Length guard. Tagger emits stop - start as length, which
            // can range from ~30 ms (single frame) to ~250 ms
            // (multi-frame). Clamp to WB_MAX_BURST_SAMPLES so we never
            // exceed the PSRAM scratch capacity, and reject bursts too
            // short to survive FIR transients.
            if (burst.length_samples < 128) {
                s_bursts_skipped++;
                continue;
            }
            uint32_t safe_len = burst.length_samples;
            if (safe_len > (uint32_t)WB_MAX_BURST_SAMPLES) {
                safe_len = (uint32_t)WB_MAX_BURST_SAMPLES;
            }
            // Round to multiple of DIDECIM_DECIM for a clean integer
            // output count from the decim. Also keeps cache alignment
            // when combined with the 16-aligned WB_PRE_PAD_SAMPLES.
            safe_len -= safe_len % DIDECIM_DECIM;
            uint32_t ext_len = safe_len + WB_PRE_PAD_SAMPLES;
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

            // 2. Rotation to DC via Q15 incremental phasor (task #58
            // step 1). ~30× faster than the cosf/sinf reference on
            // RV-32IMF, with cosf/sinf renormalisation every 128
            // samples to bound the Q15-magnitude-decay drift.
            // Host-validated to NMSE ≤ -40 dB vs the reference and
            // decode-count-equivalent on test_pipeline_wideband_albq
            // (59/133 either way).
            int64_t t_rot0 = esp_timer_get_time();
            double phase_step = -2.0 * M_PI * (double)burst.rel_freq_hz
                                 / (double)FS_DETECT_HZ;
            rotate_to_dc_q15_inc(s_extract_buf, (int)ext_len, phase_step);
            int64_t t_rot1 = esp_timer_get_time();
            s_t_rotate_us += (uint64_t)(t_rot1 - t_rot0);

            // 3. 10× decim 2.5 MSPS → 250 ksps via the split
            // (deinterleave + real-FIR ×2) path. On target this uses
            // esp-dsp's PIE-accelerated dsps_fird_s16_arp4 internally;
            // on host the same code path uses portable C. Reset
            // streaming FIR state between unrelated bursts so leftover
            // history from a previous burst doesn't bleed into this one.
            //
            // Processed in DECIM_CHUNK_IN-sample chunks so the PIE FIR
            // scratch (s_decim_scr_in_i/q) fits in internal SRAM — the
            // PIE path is silently broken on PSRAM inputs (same vld.128
            // constraint as the front-end FFT). Streaming FIR delay
            // line in `s_decim` carries history across chunks within
            // a burst, so outputs concatenate seamlessly.
            int64_t t_dec0 = esp_timer_get_time();
            direct_if_decim_reset_state(&s_decim);
            int n_250k = 0;
            for (int off = 0; off < (int)ext_len; off += DECIM_CHUNK_IN) {
                int chunk = (int)ext_len - off;
                if (chunk > DECIM_CHUNK_IN) chunk = DECIM_CHUNK_IN;
                int n_chunk_out = direct_if_decim_process_split(&s_decim,
                                       s_extract_buf + (size_t)off * 2, chunk,
                                       s_decim_buf + (size_t)n_250k * 2,
                                       s_decim_scr_in_i, s_decim_scr_in_q,
                                       s_decim_scr_out_i, s_decim_scr_out_q);
                n_250k += n_chunk_out;
            }
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
    // Queue depth 32: enough to absorb the per-burst-length variability
    // under gone-trigger (single-frame ~130 ms, multi-frame up to
    // 250 ms processing time). With 16-deep, we saw 5-7 drops per
    // smoke run at peak load. 32 absorbs the variability without
    // hitting the queue cap.
    burst_queue = xQueueCreate(32, sizeof(detected_burst_t));
    if (!burst_queue) return ESP_ERR_NO_MEM;

    // Wideband buffers. Big working surfaces stay in PSRAM (extract +
    // decim output). The PIE FIR scratch must live in INTERNAL SRAM,
    // otherwise `esp.vld.128.ip` inside dsps_fird_s16_arp4 can't
    // service PSRAM access timing and the PIE path silently degrades
    // (we previously measured 22.9 ms/burst on this stage; PIE should
    // do ~1.5 ms). Decim runs in DECIM_CHUNK_IN-sample chunks so the
    // scratch stays tiny (8 KB per I/Q channel) rather than the 100 KB
    // each that a full-burst buffer would need.
    size_t ext_bytes      = (size_t)WB_EXTRACT_MAX * 2 * sizeof(int16_t);
    size_t dec_bytes      = (size_t)WB_DECIM_MAX   * 2 * sizeof(int16_t);
    size_t scr_in_bytes   = (size_t)DECIM_CHUNK_IN      * sizeof(int16_t);
    size_t scr_out_bytes  = (size_t)(DECIM_CHUNK_IN / DIDECIM_DECIM)
                                                        * sizeof(int16_t);
    s_extract_buf      = heap_caps_malloc(ext_bytes,     MALLOC_CAP_SPIRAM);
    s_decim_buf        = heap_caps_malloc(dec_bytes,     MALLOC_CAP_SPIRAM);
    s_decim_scr_in_i   = heap_caps_aligned_alloc(16, scr_in_bytes,  MALLOC_CAP_INTERNAL);
    s_decim_scr_in_q   = heap_caps_aligned_alloc(16, scr_in_bytes,  MALLOC_CAP_INTERNAL);
    s_decim_scr_out_i  = heap_caps_aligned_alloc(16, scr_out_bytes, MALLOC_CAP_INTERNAL);
    s_decim_scr_out_q  = heap_caps_aligned_alloc(16, scr_out_bytes, MALLOC_CAP_INTERNAL);
    if (!s_extract_buf || !s_decim_buf
        || !s_decim_scr_in_i || !s_decim_scr_in_q
        || !s_decim_scr_out_i || !s_decim_scr_out_q) {
        ESP_LOGE(TAG, "Worker buffer alloc failed");
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
