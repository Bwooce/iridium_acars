#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include "esp_timer.h"

static const char *TAG = "P4_DSP";

#define FFT_SIZE 2048
#define THRESHOLD_DB 16.0f
#define HISTORY_SIZE 128
#define PRIMING_FRAMES 16

extern const uint8_t burst_slice_sc16_start[] asm("_binary_burst_slice_sc16_bin_start");
extern const uint8_t burst_slice_sc16_end[]   asm("_binary_burst_slice_sc16_bin_end");

/* SC16 Buffers - Aligned for PIE/SIMD, padded by 32 bytes (16 int16s) to prevent arp4 overrun bug */
__attribute__((aligned(16))) static int16_t fft_in[FFT_SIZE * 2 + 16];
__attribute__((aligned(16))) static int16_t window_cplx[FFT_SIZE * 2 + 16];
__attribute__((aligned(16))) static float window_temp_f32[FFT_SIZE + 16];
__attribute__((aligned(16))) static float magnitudes[FFT_SIZE + 16];
__attribute__((aligned(16))) static float baseline[FFT_SIZE + 16];
__attribute__((aligned(16))) static int16_t frame_sram[FFT_SIZE * 2 + 16];

typedef struct {
    bool active;
    int start_frame;
    int max_bin;
    float max_snr;
} active_burst_t;

void dsp_task(void *arg)
{
    ESP_LOGI(TAG, "DSP Task started on Core %d", xPortGetCoreID());

    const int16_t *iq_data = (const int16_t *)burst_slice_sc16_start;
    size_t total_samples = (burst_slice_sc16_end - burst_slice_sc16_start) / (sizeof(int16_t) * 2);
    size_t n_frames = total_samples / FFT_SIZE;

    ESP_LOGI(TAG, "Processing %d samples (%d FFT frames)...", (int)total_samples, (int)n_frames);

    float threshold_lin = powf(10.0f, THRESHOLD_DB / 10.0f);
    int total_bursts = 0;
    active_burst_t current_burst = { .active = false };
    
    int64_t t_total = 0, t_flash = 0, t_wind = 0, t_fft = 0, t_mag = 0, t_detect = 0;

    for (size_t f = 0; f < n_frames; f++) {
        const int16_t *frame_ptr = &iq_data[f * FFT_SIZE * 2];
        
        int64_t t_flash_start = esp_timer_get_time();
        memcpy(frame_sram, frame_ptr, FFT_SIZE * 2 * sizeof(int16_t));
        int64_t t0 = esp_timer_get_time();

        /* 1. Window (SC16) */
        for (int i = 0; i < FFT_SIZE * 2; i++) {
            fft_in[i] = (int16_t)(((int32_t)frame_sram[i] * window_cplx[i]) >> 15);
        }
        int64_t t1 = esp_timer_get_time();

        /* 2. FFT (FORCED 16-bit PIE MACRO) */
        dsps_fft2r_sc16_arp4(fft_in, FFT_SIZE);
        dsps_bit_rev_sc16_ansi(fft_in, FFT_SIZE);
        int64_t t2 = esp_timer_get_time();

        /* 3. Magnitude Squared */
        float norm = 1.0f / (FFT_SIZE * FFT_SIZE);
        for (int i = 0; i < FFT_SIZE; i++) {
            int shift_idx = (i + FFT_SIZE / 2) % FFT_SIZE;
            float re = (float)fft_in[i * 2 + 0];
            float im = (float)fft_in[i * 2 + 1];
            magnitudes[shift_idx] = (re * re + im * im) * norm;
        }
        int64_t t3 = esp_timer_get_time();

        /* 4. Detection & Grouping */
        bool frame_has_signal = false;
        int peak_bin = -1;
        float peak_snr = 0;

        if (f >= PRIMING_FRAMES) {
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
                current_burst.start_frame = (int)f;
                current_burst.max_bin = peak_bin;
                current_burst.max_snr = peak_snr;
            } else if (peak_snr > current_burst.max_snr) {
                current_burst.max_snr = peak_snr;
                current_burst.max_bin = peak_bin;
            }
        } else if (current_burst.active) {
            ESP_LOGI(TAG, "BURST! Start:%d End:%d Bin:%d SNR:%.2f dB",
                     current_burst.start_frame, (int)f-1, current_burst.max_bin, 10.0f * log10f(current_burst.max_snr));
            total_bursts++;
            current_burst.active = false;
        }

        /* 5. Baseline update */
        if (!frame_has_signal) {
            float alpha = (f < PRIMING_FRAMES * 2) ? 0.5f : (1.0f / HISTORY_SIZE);
            float beta = 1.0f - alpha;
            for (int i = 0; i < FFT_SIZE; i++) {
                baseline[i] = beta * baseline[i] + alpha * magnitudes[i];
            }
        }
        int64_t t4 = esp_timer_get_time();

        t_flash += (t0 - t_flash_start);
        t_wind += (t1 - t0);
        t_fft += (t2 - t1);
        t_mag += (t3 - t2);
        t_detect += (t4 - t3);
        t_total += (t4 - t_flash_start);

        if ((f % 16) == 0) {
            vTaskDelay(1); /* Feed watchdog */
        }
    }

    ESP_LOGI(TAG, "DSP Processing Complete.");
    ESP_LOGI(TAG, "Avg Time per Frame: %.2f ms", (float)t_total / n_frames / 1000.0f);
    ESP_LOGI(TAG, "Breakdown: Flash:%.2f Wind:%.2f FFT:%.2f Mag:%.2f Detect:%.2f (ms)",
             (float)t_flash/n_frames/1000.0f, (float)t_wind/n_frames/1000.0f, 
             (float)t_fft/n_frames/1000.0f, (float)t_mag/n_frames/1000.0f, 
             (float)t_detect/n_frames/1000.0f);
    ESP_LOGI(TAG, "Total unique bursts: %d", total_bursts);

    printf("\nTEST_COMPLETE\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Iridium ACARS SC16 (16-bit) DSP on ESP32-P4...");

    /* Initialize dsps with 16-bit support */
    esp_err_t ret = dsps_fft2r_init_sc16(NULL, FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize FFT: %s", esp_err_to_name(ret));
        return;
    }

    /* Prepare SC16 Blackman window */
    dsps_wind_blackman_f32(window_temp_f32, FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++) {
        int16_t w = (int16_t)(window_temp_f32[i] / 0.42f * 32767.0f);
        window_cplx[i * 2 + 0] = w;
        window_cplx[i * 2 + 1] = w;
    }
    
    for (int i = 0; i < FFT_SIZE; i++) baseline[i] = 1.0e-6f;

    /* Pin DSP task to Core 1 with highest priority */
    xTaskCreatePinnedToCore(dsp_task, "dsp_task", 8192, NULL, configMAX_PRIORITIES - 1, NULL, 1);
}
