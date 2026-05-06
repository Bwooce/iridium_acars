#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_dsp.h"
#include "esp_timer.h"
#include "dsp_processor.h"
#include "dsp_window_arp4.h"

static const char *TAG = "DSP_PROC";

// Detector threshold: 40× above baseline = 10·log10(40) = 16.02 dB,
// matching the prior float setpoint within 0.02 dB. Tried `>> 5` (32×
// = 15.05 dB) to skip the multiply but the 1 dB drop in selectivity
// produced too many priming-noise false positives. Tried uint64 mul
// for full overflow safety but that cost ~20 us/frame. The compromise:
// uint32 multiply with wrap-around. baseline > 2^26 (= ~67 M) would
// wrap, but our magnitudes max at 2^31 and baseline tracks them — at
// realistic noise levels baseline stays well below 2^25, so wrap is
// not a concern in practice.
#define THRESHOLD_MULT 40u
#define HISTORY_SIZE 128
#define PRIMING_FRAMES 16

// EMA weights in Q15 fixed-point. Priming uses α = 0.5 (16384/32768),
// steady-state α = 1/HISTORY_SIZE. β = 1 - α.
#define ALPHA_PRIMING_Q15  16384u   // 0.5 in Q15
#define BETA_PRIMING_Q15   16384u   // 0.5
#define ALPHA_STEADY_Q15   ((uint32_t)(32768u / HISTORY_SIZE))   // 1/128 = 256
#define BETA_STEADY_Q15    (32768u - ALPHA_STEADY_Q15)           // 32512

/* Buffers - Aligned for PIE, padded for overrun bug.
 *
 * Tried aligning everything to 64 bytes (the P4 L1 D-cache line size)
 * to remove the head/tail line-share between adjacent buffers — but
 * with five 8 KB buffers (each = 128 lines = 2 way-fills in a 64-set
 * 8-way L1 D), identical 64-byte alignment puts them all at the same
 * cache-set offsets and the EMA stage regressed by ~170 us/frame
 * from set-conflict thrashing. The linker's natural placement (each
 * buffer 16-byte aligned but at varying mod-64 offsets) scatters them
 * across L1 sets and is empirically faster. Keep aligned(16).
 *
 * Magnitudes and baseline used to be float32. Converted to uint32 in
 * Step 7b: magnitudes[i] = re² + im² directly (max 2 × 32767² ≈ 2^31,
 * fits int31), baseline tracks magnitudes in same scale. Eliminates
 * 4096 int16→f32 casts/frame in the magnitude loop and lets the EMA
 * run on integer arithmetic instead of scalar f32. window_temp_f32
 * stays float because dsps_wind_blackman_f32 (called once at init)
 * needs it, but it's no longer used as a per-frame scratch buffer.
 */
__attribute__((aligned(16))) static int16_t  fft_in[FFT_SIZE * 2 + 16];
__attribute__((aligned(16))) static int16_t  window_cplx[FFT_SIZE * 2 + 16];
__attribute__((aligned(16))) static float    window_temp_f32[FFT_SIZE + 16];
__attribute__((aligned(16))) static uint32_t magnitudes[FFT_SIZE + 16];
__attribute__((aligned(16))) static uint32_t baseline[FFT_SIZE + 16];

typedef struct {
    bool active;
    uint32_t start_frame;
    int max_bin;
    uint32_t max_snr_q;   // peak ratio magnitudes/baseline as uint32
} active_burst_t;

static active_burst_t current_burst = { .active = false };
static uint32_t frame_count = 0;
static int total_bursts = 0;
static burst_detected_cb_t burst_cb = NULL;

// Per-stage timing accumulators (sum of microseconds across frames since
// last reset). Reset by dsp_processor_get_stage_stats().
static volatile uint64_t s_acc_wind_us = 0;
static volatile uint64_t s_acc_fft_us = 0;
static volatile uint64_t s_acc_mag_us = 0;
static volatile uint64_t s_acc_detect_us = 0;
static volatile uint64_t s_acc_baseline_us = 0;
static volatile uint32_t s_acc_frames = 0;

esp_err_t dsp_processor_init(burst_detected_cb_t cb)
{
    ESP_LOGI(TAG, "Initializing FFT Detector (Size:%d)...", FFT_SIZE);
    burst_cb = cb;

    esp_err_t ret = dsps_fft2r_init_sc16(NULL, FFT_SIZE);
    if (ret != ESP_OK) return ret;

    dsps_wind_blackman_f32(window_temp_f32, FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++) {
        int16_t w = (int16_t)(window_temp_f32[i] / 0.42f * 32767.0f);
        window_cplx[i * 2 + 0] = w;
        window_cplx[i * 2 + 1] = w;
    }

    // Initial baseline: a small non-zero value. The detection threshold is
    // 40 × baseline, so baseline=1 means threshold=40 (very low) and the
    // priming-phase noise will quickly raise it to the actual noise floor.
    for (int i = 0; i < FFT_SIZE; i++) baseline[i] = 1u;

    return ESP_OK;
}

void dsp_processor_feed(const int16_t *samples, size_t n_samples)
{
    // Note: n_samples is complex samples (I,Q pairs)
    // At 2.56 MSPS, we receive 16KB buffers = 8192 IQ pairs
    // Each feed is 16KB = 4 FFT frames (2048 each)
    
    int n_frames = n_samples / FFT_SIZE;
    
    for (int f = 0; f < n_frames; f++) {
        const int16_t *frame_ptr = &samples[f * FFT_SIZE * 2];
        int64_t t0 = esp_timer_get_time();

        // 1. Window — Q15 element-wise multiply, hand-rolled PIE on P4.
        // The scalar loop the compiler emits at -O3 measured 92 μs/frame;
        // the 8-lane esp.vmul.s16 kernel is ~5–15 μs/frame.
        dsp_window_s16(frame_ptr, window_cplx, fft_in, FFT_SIZE * 2);
        int64_t t1 = esp_timer_get_time();

        // 2. FFT
        dsps_fft2r_sc16_arp4(fft_in, FFT_SIZE);
        dsps_bit_rev_sc16_ansi(fft_in, FFT_SIZE);
        int64_t t2 = esp_timer_get_time();

        // 3. Magnitude Squared — uint32 sum-of-squares, linear write.
        // Output is re² + im² (no normalisation; baseline tracks the
        // same scale). With re,im in [-32768, 32767], each squared
        // term ≤ 2^30 and the sum ≤ 2^31, fits comfortably in uint32.
        // No fftshift here — applied at burst-report only.
        for (int i = 0; i < FFT_SIZE; i++) {
            int32_t re = fft_in[i * 2 + 0];
            int32_t im = fft_in[i * 2 + 1];
            magnitudes[i] = (uint32_t)(re * re + im * im);
        }
        int64_t t3 = esp_timer_get_time();

        // 4. Detection — uint32 multiply, no uint64. baseline×40 wraps
        // for baseline > 2^26 (~67 M); but realistic noise levels stay
        // below 2^25 so wrap is not a concern. Matches the prior float
        // setpoint (16 dB) within 0.02 dB.
        bool frame_has_signal = false;
        int peak_bin = -1;
        uint32_t peak_snr_q = 0;

        if (frame_count >= PRIMING_FRAMES) {
            for (int i = 0; i < FFT_SIZE; i++) {
                uint32_t threshold = baseline[i] * THRESHOLD_MULT;
                if (magnitudes[i] > threshold) {
                    frame_has_signal = true;
                    // Peak ratio for SNR_dB at burst end. Division per
                    // exceeding bin only — rare in non-burst frames.
                    uint32_t rel = baseline[i] > 0
                        ? magnitudes[i] / baseline[i]
                        : magnitudes[i];
                    if (rel > peak_snr_q) {
                        peak_snr_q = rel;
                        peak_bin = i;
                    }
                }
            }
        }
        int64_t t4 = esp_timer_get_time();

        if (frame_has_signal) {
            if (!current_burst.active) {
                current_burst.active = true;
                current_burst.start_frame = frame_count;
                current_burst.max_bin = peak_bin;
                current_burst.max_snr_q = peak_snr_q;
            } else if (peak_snr_q > current_burst.max_snr_q) {
                current_burst.max_snr_q = peak_snr_q;
                current_burst.max_bin = peak_bin;
            }
        } else if (current_burst.active) {
            // Convert peak ratio to dB once, at burst-end. log10f cost is
            // negligible because it runs once per burst, not per frame.
            float snr_db = (current_burst.max_snr_q > 0)
                ? 10.0f * log10f((float)current_burst.max_snr_q)
                : 0.0f;
            // The detection / EMA path tracks bins in linear FFT order.
            // Apply the fftshift here at the API boundary so the log
            // line and the worker callback still get the conventional
            // bin 1024 = DC, bin > 1024 = positive freq.
            int shifted_bin = (current_burst.max_bin + FFT_SIZE / 2) & (FFT_SIZE - 1);
            ESP_LOGI(TAG, "BURST DETECTED! Frame:%lu Bin:%d SNR:%.2f dB",
                     current_burst.start_frame, shifted_bin, snr_db);

            if (burst_cb) {
                detected_burst_t burst = {
                    .start_sample_idx = current_burst.start_frame * FFT_SIZE,
                    .length_samples = (frame_count - current_burst.start_frame) * FFT_SIZE,
                    .peak_bin = shifted_bin,
                    .peak_snr_db = snr_db
                };
                burst_cb(&burst);
            }

            total_bursts++;
            current_burst.active = false;
        }

        // 5. Baseline EMA in uint32 with Q15 weights:
        //   b = (β·b + α·m) >> 15
        // Priming phase (first 2× PRIMING_FRAMES) uses α = 0.5 so the
        // baseline converges quickly to the actual noise floor; then
        // switches to α = 1/HISTORY_SIZE for slow tracking.
        //
        // Per-iteration: two uint32 × uint32 → uint64 multiplies, one
        // add, one shift, one store. Simpler than f32 (no casts), no
        // scratch buffer needed (the prior dsps_mulc_f32 / dsps_add_f32
        // chain through window_temp_f32 was three passes — this is one).
        if (!frame_has_signal) {
            uint32_t alpha, beta;
            if (frame_count < PRIMING_FRAMES * 2) {
                alpha = ALPHA_PRIMING_Q15;
                beta  = BETA_PRIMING_Q15;
            } else {
                alpha = ALPHA_STEADY_Q15;
                beta  = BETA_STEADY_Q15;
            }
            for (int i = 0; i < FFT_SIZE; i++) {
                uint64_t b = (uint64_t)beta * baseline[i]
                           + (uint64_t)alpha * magnitudes[i];
                baseline[i] = (uint32_t)(b >> 15);
            }
        }
        int64_t t5 = esp_timer_get_time();

        // Accumulate stage timings for diagnostic reporting.
        s_acc_wind_us     += (uint64_t)(t1 - t0);
        s_acc_fft_us      += (uint64_t)(t2 - t1);
        s_acc_mag_us      += (uint64_t)(t3 - t2);
        s_acc_detect_us   += (uint64_t)(t4 - t3);
        s_acc_baseline_us += (uint64_t)(t5 - t4);
        s_acc_frames++;

        frame_count++;
    }
}

void dsp_processor_get_stage_stats(dsp_stage_stats_t *out)
{
    uint32_t n = s_acc_frames;
    if (n == 0) {
        out->frames = 0;
        out->wind_us = out->fft_us = out->mag_us = out->detect_us =
            out->baseline_us = out->total_us = 0;
        return;
    }
    float fn = (float)n;
    out->frames      = n;
    out->wind_us     = (float)s_acc_wind_us / fn;
    out->fft_us      = (float)s_acc_fft_us / fn;
    out->mag_us      = (float)s_acc_mag_us / fn;
    out->detect_us   = (float)s_acc_detect_us / fn;
    out->baseline_us = (float)s_acc_baseline_us / fn;
    out->total_us    = out->wind_us + out->fft_us + out->mag_us +
                       out->detect_us + out->baseline_us;
    s_acc_wind_us = s_acc_fft_us = s_acc_mag_us = s_acc_detect_us =
        s_acc_baseline_us = 0;
    s_acc_frames = 0;
}
