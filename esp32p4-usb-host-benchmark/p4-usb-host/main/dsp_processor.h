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

#endif
