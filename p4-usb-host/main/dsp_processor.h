#ifndef DSP_PROCESSOR_H
#define DSP_PROCESSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define FFT_SIZE 2048

typedef struct {
    uint32_t start_sample_idx;
    uint32_t length_samples;
    int peak_bin;
    float peak_snr_db;
} detected_burst_t;

typedef void (*burst_detected_cb_t)(const detected_burst_t *burst);

esp_err_t dsp_processor_init(burst_detected_cb_t cb);
void dsp_processor_feed(const int16_t *samples, size_t n_samples);

// Diagnostic stats: average per-frame time in each stage (microseconds),
// computed over frames seen since the last call. Calling this resets the
// internal accumulators so the next call covers a fresh window.
typedef struct {
    uint32_t frames;          // frames in this window
    float wind_us;            // window step
    float fft_us;             // FFT + bit reverse
    float mag_us;             // magnitude squared + fftshift
    float detect_us;          // threshold scan
    float baseline_us;        // baseline EMA update
    float total_us;           // sum (should match class_driver's DSP timer)
} dsp_stage_stats_t;

void dsp_processor_get_stage_stats(dsp_stage_stats_t *out);

#endif
