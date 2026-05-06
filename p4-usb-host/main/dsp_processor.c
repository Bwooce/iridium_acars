#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_dsp.h"
#include "esp_timer.h"
#include "dsp_processor.h"
#include "dsp_window_arp4.h"

static const char *TAG = "DSP_PROC";

#define THRESHOLD_DB 16.0f
#define HISTORY_SIZE 128
#define PRIMING_FRAMES 16

/* Buffers - Aligned for PIE, padded for overrun bug.
 *
 * Tried aligning everything to 64 bytes (the P4 L1 D-cache line size)
 * to remove the head/tail line-share between adjacent buffers — but
 * with five 8 KB buffers (each = 128 lines = 2 way-fills in a 64-set
 * 8-way L1 D), identical 64-byte alignment puts them all at the same
 * cache-set offsets and the EMA stage regressed by ~170 us/frame
 * from set-conflict thrashing. The linker's natural placement (each
 * buffer 16-byte aligned but at varying mod-64 offsets) scatters them
 * across L1 sets and is empirically faster. Keep aligned(16). */
__attribute__((aligned(16))) static int16_t fft_in[FFT_SIZE * 2 + 16];
__attribute__((aligned(16))) static int16_t window_cplx[FFT_SIZE * 2 + 16];
__attribute__((aligned(16))) static float window_temp_f32[FFT_SIZE + 16];
__attribute__((aligned(16))) static float magnitudes[FFT_SIZE + 16];
__attribute__((aligned(16))) static float baseline[FFT_SIZE + 16];

typedef struct {
    bool active;
    uint32_t start_frame;
    int max_bin;
    float max_snr;
} active_burst_t;

static active_burst_t current_burst = { .active = false };
static uint32_t frame_count = 0;
static float threshold_lin;
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

    for (int i = 0; i < FFT_SIZE; i++) baseline[i] = 1.0e-6f;
    threshold_lin = powf(10.0f, THRESHOLD_DB / 10.0f);
    
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

        // 3. Magnitude Squared — linear write, no fftshift here.
        // Previously this loop did `magnitudes[(i + N/2) % N] = ...`,
        // which produced two cache-unfriendly write streams (one to
        // each half of magnitudes[]). Writing linearly keeps the access
        // pattern sequential — input fft_in is also read sequentially,
        // so the whole loop is one streaming-read + one streaming-write.
        // The fftshift transformation is applied below in the detect /
        // report path so the worker still sees the conventional
        // bin 1024 = DC convention.
        float norm = 1.0f / (FFT_SIZE * FFT_SIZE);
        for (int i = 0; i < FFT_SIZE; i++) {
            float re = (float)fft_in[i * 2 + 0];
            float im = (float)fft_in[i * 2 + 1];
            magnitudes[i] = (re * re + im * im) * norm;
        }
        int64_t t3 = esp_timer_get_time();

        // 4. Detection
        bool frame_has_signal = false;
        int peak_bin = -1;
        float peak_snr = 0;

        if (frame_count >= PRIMING_FRAMES) {
            for (int i = 0; i < FFT_SIZE; i++) {
                float threshold = baseline[i] * threshold_lin;
                if (magnitudes[i] > threshold) {
                    frame_has_signal = true;
                    float rel_mag = magnitudes[i] / baseline[i];
                    if (rel_mag > peak_snr) {
                        peak_snr = rel_mag;
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
                current_burst.max_snr = peak_snr;
            } else if (peak_snr > current_burst.max_snr) {
                current_burst.max_snr = peak_snr;
                current_burst.max_bin = peak_bin;
            }
        } else if (current_burst.active) {
            float snr_db = 10.0f * log10f(current_burst.max_snr);
            // The detection / EMA path now tracks bins in linear FFT
            // order (no fftshift in the magnitude loop). Apply the
            // shift here at the API boundary so the log line and the
            // worker callback still get the conventional fftshifted
            // index where bin 1024 = DC, bin > 1024 = positive freq.
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

        // 5. Baseline update — exponential moving average:
        //   baseline = (1 - α) · baseline + α · magnitudes
        // The scalar loop measured ~1.5 ms/frame on P4 (50%+ of DSP time).
        // esp-dsp has no _arp4 (PIE) variant for these f32 ops on P4, only
        // the ANSI fallback, but the calls still let the compiler unroll
        // tighter and avoid a couple of redundant loads vs the inline loop.
        // window_temp_f32 is reused as a scratch buffer (it's only used at
        // init time to compute the Blackman window, then unused).
        if (!frame_has_signal) {
            float alpha = (frame_count < PRIMING_FRAMES * 2) ? 0.5f : (1.0f / HISTORY_SIZE);
            float beta = 1.0f - alpha;
            dsps_mulc_f32(baseline,   baseline,         FFT_SIZE, beta,  1, 1);
            dsps_mulc_f32(magnitudes, window_temp_f32,  FFT_SIZE, alpha, 1, 1);
            dsps_add_f32 (baseline,   window_temp_f32,  baseline, FFT_SIZE, 1, 1, 1);
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
