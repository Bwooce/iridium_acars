#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_dsp.h"
#include "esp_timer.h"
#include "dsp_processor.h"

static const char *TAG = "DSP_PROC";

#define THRESHOLD_DB 16.0f
#define HISTORY_SIZE 128
#define PRIMING_FRAMES 16

/* Buffers - Aligned for PIE, padded for overrun bug */
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
        
        // 1. Window
        for (int i = 0; i < FFT_SIZE * 2; i++) {
            fft_in[i] = (int16_t)(((int32_t)frame_ptr[i] * window_cplx[i]) >> 15);
        }

        // 2. FFT
        dsps_fft2r_sc16_arp4(fft_in, FFT_SIZE);
        dsps_bit_rev_sc16_ansi(fft_in, FFT_SIZE);

        // 3. Magnitude Squared (with shift)
        float norm = 1.0f / (FFT_SIZE * FFT_SIZE);
        for (int i = 0; i < FFT_SIZE; i++) {
            int shift_idx = (i + FFT_SIZE / 2) % FFT_SIZE;
            float re = (float)fft_in[i * 2 + 0];
            float im = (float)fft_in[i * 2 + 1];
            magnitudes[shift_idx] = (re * re + im * im) * norm;
        }

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
            ESP_LOGI(TAG, "BURST DETECTED! Frame:%lu Bin:%d SNR:%.2f dB",
                     current_burst.start_frame, current_burst.max_bin, snr_db);
            
            if (burst_cb) {
                detected_burst_t burst = {
                    .start_sample_idx = current_burst.start_frame * FFT_SIZE,
                    .length_samples = (frame_count - current_burst.start_frame) * FFT_SIZE,
                    .peak_bin = current_burst.max_bin,
                    .peak_snr_db = snr_db
                };
                burst_cb(&burst);
            }
            
            total_bursts++;
            current_burst.active = false;
        }

        // 5. Baseline update
        if (!frame_has_signal) {
            float alpha = (frame_count < PRIMING_FRAMES * 2) ? 0.5f : (1.0f / HISTORY_SIZE);
            float beta = 1.0f - alpha;
            for (int i = 0; i < FFT_SIZE; i++) {
                baseline[i] = beta * baseline[i] + alpha * magnitudes[i];
            }
        }
        
        frame_count++;
    }
}
