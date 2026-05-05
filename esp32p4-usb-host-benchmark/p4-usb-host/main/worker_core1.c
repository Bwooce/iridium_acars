#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include "dsps_cplx_gen.h"
#include "dsps_fir.h"
#include "dsps_resampler.h"
#include "esp_heap_caps.h"
#include "worker_core1.h"
#include "signal_buffer.h"
#include "qpsk_demod.h"
#include "bch_decoder.h"

static const char *TAG = "WORKER1";

static QueueHandle_t burst_queue = NULL;

// Decimation factor 32: 2.56 MHz -> 80 kHz
#define DECIM_FACTOR 32
#define FIR_TAPS 64

// Resample 80 kHz -> 50 kHz (Ratio 1.6, Interp 5, Decim 8)
#define RESAMPLE_INTERP 5
#define RESAMPLE_DECIM 8
#define RESAMPLE_TAPS 64

// Buffer for burst extraction (max 50ms = 128k complex samples)
#define MAX_EXTRACT_SAMPLES (128 * 1024)
static int16_t *extract_buf = NULL;
static int16_t *phasor_buf = NULL;
static int16_t *decim_buf = NULL; // 80 kHz buffer
static int16_t *resample_buf = NULL; // 50 kHz buffer

static fir_s16_t fir_i, fir_q;
static int16_t coeffs[FIR_TAPS] __attribute__((aligned(16)));
static int16_t delay_i[FIR_TAPS] __attribute__((aligned(16)));
static int16_t delay_q[FIR_TAPS] __attribute__((aligned(16)));

// Stage 2: 80 kHz -> 50 kHz polyphase resample (interp 5, decim 8). We use the
// lower-level dsps_firmr_* directly because dsps_resampler_mr_init() rejects
// samplerate_factor < 1 (i.e. it can only upsample), and silently leaves the
// struct uninitialised when called for downsampling — leading to a NULL deref
// inside _exec(). dsps_firmr_init_s16 takes interp/decim directly and works
// for both directions.
static fir_s16_t resampler_i, resampler_q;
static int16_t resample_coeffs[RESAMPLE_TAPS * RESAMPLE_INTERP] __attribute__((aligned(16)));
static int16_t resample_delay_i[RESAMPLE_TAPS * RESAMPLE_INTERP] __attribute__((aligned(16)));
static int16_t resample_delay_q[RESAMPLE_TAPS * RESAMPLE_INTERP] __attribute__((aligned(16)));

void worker_task(void *arg)
{
    ESP_LOGI(TAG, "Worker Task started on Core %d", xPortGetCoreID());
    detected_burst_t burst;
    cplx_sig_t phasor_gen;

    while (1) {
        if (xQueueReceive(burst_queue, &burst, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Worker processing burst: Start:%lu Len:%lu Bin:%d SNR:%.2f dB",
                     burst.start_sample_idx, burst.length_samples, burst.peak_bin, burst.peak_snr_db);

            if (burst.length_samples > MAX_EXTRACT_SAMPLES) {
                burst.length_samples = MAX_EXTRACT_SAMPLES;
            }

            // Reject extreme/invalid bins. peak_bin range is 0..2047 with bin 1024 = DC.
            // The phasor frequency math below produces |norm * 2| -> 1.0 at bins 0 and 2047,
            // which is exactly out of dsps_cplx_gen's (-1, 1) valid range and previously
            // caused a Core 1 fault. Drop bursts within a guard band of the spectrum edges
            // and at DC itself (bin 1024) — DC bursts are usually direct-sampling artefacts,
            // not real Iridium signals.
            #define BIN_EDGE_GUARD 4
            if (burst.peak_bin < BIN_EDGE_GUARD ||
                burst.peak_bin >= (2048 - BIN_EDGE_GUARD) ||
                burst.peak_bin == 1024) {
                ESP_LOGW(TAG, "Skipping burst at edge/DC bin %d (likely artefact)", burst.peak_bin);
                continue;
            }

            // 1. Extract from Circular Buffer
            signal_buffer_extract(burst.start_sample_idx, burst.length_samples, extract_buf);

            // 2. Frequency Centering
            float freq_offset = (burst.peak_bin - 1024) * 1250.0f;
            float norm_freq = -freq_offset / 2560000.0f;
            // dsps_cplx_gen accepts normalised frequency in (-1, 1) exclusive.
            // Clamp defensively in case detector ever emits an unusual peak_bin.
            float gen_freq = norm_freq * 2.0f;
            if (gen_freq >= 1.0f)  gen_freq = 0.999f;
            if (gen_freq <= -1.0f) gen_freq = -0.999f;
            dsps_cplx_gen_init(&phasor_gen, S16_FIXED, NULL, 1024, gen_freq, 0);
            dsps_cplx_gen(&phasor_gen, phasor_buf, burst.length_samples);
            cplx_gen_free(&phasor_gen);

            for (uint32_t i = 0; i < burst.length_samples; i++) {
                int32_t x_re = extract_buf[i * 2 + 0];
                int32_t x_im = extract_buf[i * 2 + 1];
                int32_t p_re = phasor_buf[i * 2 + 0];
                int32_t p_im = phasor_buf[i * 2 + 1];
                extract_buf[i * 2 + 0] = (int16_t)((x_re * p_re - x_im * p_im) >> 15);
                extract_buf[i * 2 + 1] = (int16_t)((x_re * p_im + x_im * p_re) >> 15);
            }

            // 3. FIR Decimation (32x) Stage 1 (2.56M -> 80k)
            memset(delay_i, 0, sizeof(delay_i));
            memset(delay_q, 0, sizeof(delay_q));
            fir_i.d_pos = 0;
            fir_q.d_pos = 0;

            int16_t *in_i = malloc(burst.length_samples * sizeof(int16_t));
            int16_t *in_q = malloc(burst.length_samples * sizeof(int16_t));
            for (uint32_t i = 0; i < burst.length_samples; i++) {
                in_i[i] = extract_buf[i * 2 + 0];
                in_q[i] = extract_buf[i * 2 + 1];
            }

            int out_samples_80k = dsps_fird_s16_arp4(&fir_i, in_i, &decim_buf[0], burst.length_samples);
            dsps_fird_s16_arp4(&fir_q, in_q, &decim_buf[MAX_EXTRACT_SAMPLES], burst.length_samples);

            free(in_i);
            free(in_q);

            // 4. Resample Stage 2 (80k -> 50k via interp=5, decim=8)
            int out_samples_50k = dsps_firmr_s16(&resampler_i, &decim_buf[0], &resample_buf[0], out_samples_80k);
            dsps_firmr_s16(&resampler_q, &decim_buf[MAX_EXTRACT_SAMPLES], &resample_buf[MAX_EXTRACT_SAMPLES], out_samples_80k);

            ESP_LOGI(TAG, "Burst processed. Output samples (2sps): %d", out_samples_50k);
            
            // 5. QPSK Demodulation
            decoded_frame_t frame;
            // Pack I and Q back into interleaved for demod
            int16_t *interleaved_2sps = malloc(out_samples_50k * 2 * sizeof(int16_t));
            if (interleaved_2sps) {
                for (int i = 0; i < out_samples_50k; i++) {
                    interleaved_2sps[i * 2 + 0] = resample_buf[i];
                    interleaved_2sps[i * 2 + 1] = resample_buf[MAX_EXTRACT_SAMPLES + i];
                }

                if (qpsk_demod_process(interleaved_2sps, out_samples_50k * 2, &frame)) {
                    // Successfully demodulated!
                    ESP_LOGI(TAG, "DEMOD SUCCESS: %s frame (%d bits)", 
                             (frame.direction == DIR_DOWNLINK) ? "DL" : "UL", frame.n_bits);
                    
                    // 6. BCH Decoding / De-interleaving
                    if (frame.n_bits >= 24 + 64) {
                        const uint8_t *payload = frame.bits + 24;
                        uint8_t block1[32], block2[32];
                        uint8_t data1[21], data2[21];
                        
                        iridium_deinterleave(payload, block1, block2);
                        int e1 = bch_decode_block(block1, data1);
                        int e2 = bch_decode_block(block2, data2);
                        
                        if (e1 >= 0 && e2 >= 0) {
                            ESP_LOGI(TAG, "BCH DECODE SUCCESS! Errors: %d, %d", e1, e2);
                            // Log first few bits as binary for quick check
                            char bin_str[22] = {0};
                            for (int i = 0; i < 21; i++) bin_str[i] = data1[i] ? '1' : '0';
                            ESP_LOGI(TAG, "Block1 Data: %s", bin_str);
                        }
                    }
                    
                    free(frame.bits);
                }
                free(interleaved_2sps);
            }
        }
    }
}

esp_err_t worker_core1_init()
{
    burst_queue = xQueueCreate(16, sizeof(detected_burst_t));
    if (!burst_queue) return ESP_ERR_NO_MEM;

    extract_buf = heap_caps_malloc(MAX_EXTRACT_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    phasor_buf = heap_caps_malloc(MAX_EXTRACT_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    decim_buf = heap_caps_malloc(MAX_EXTRACT_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    resample_buf = heap_caps_malloc(MAX_EXTRACT_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!extract_buf || !phasor_buf || !decim_buf || !resample_buf) return ESP_ERR_NO_MEM;

    // Stage 1 Coefficients (LPF)
    float coeffs_f32[FIR_TAPS];
    dsps_fird_init_s16(&fir_i, coeffs, delay_i, FIR_TAPS, DECIM_FACTOR, 0, 15);
    dsps_fird_init_s16(&fir_q, coeffs, delay_q, FIR_TAPS, DECIM_FACTOR, 0, 15);
    float omega_c = 2.0f * M_PI * 20000.0f / 2560000.0f;
    for (int i = 0; i < FIR_TAPS; i++) {
        float n = i - (FIR_TAPS - 1) / 2.0f;
        float h = (fabsf(n) < 1e-9f) ? (omega_c / M_PI) : (sinf(omega_c * n) / (M_PI * n));
        float w = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FIR_TAPS - 1)));
        coeffs_f32[i] = h * w;
    }
    float sum = 0;
    for (int i = 0; i < FIR_TAPS; i++) sum += coeffs_f32[i];
    for (int i = 0; i < FIR_TAPS; i++) coeffs[i] = (int16_t)(coeffs_f32[i] / sum * 32767.0f);

    // Stage 2 Coefficients (RRC/LPF for resampling)
    // For now, use simple LPF for resampler
    float rcoeffs_f32[RESAMPLE_TAPS * RESAMPLE_INTERP];
    float r_omega_c = 2.0f * M_PI * 20000.0f / (80000.0f * RESAMPLE_INTERP);
    for (int i = 0; i < RESAMPLE_TAPS * RESAMPLE_INTERP; i++) {
        float n = i - (RESAMPLE_TAPS * RESAMPLE_INTERP - 1) / 2.0f;
        float h = (fabsf(n) < 1e-9f) ? (r_omega_c / M_PI) : (sinf(r_omega_c * n) / (M_PI * n));
        float w = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (RESAMPLE_TAPS * RESAMPLE_INTERP - 1)));
        rcoeffs_f32[i] = h * w;
    }
    sum = 0;
    for (int i = 0; i < RESAMPLE_TAPS * RESAMPLE_INTERP; i++) sum += rcoeffs_f32[i];
    for (int i = 0; i < RESAMPLE_TAPS * RESAMPLE_INTERP; i++) resample_coeffs[i] = (int16_t)(rcoeffs_f32[i] / sum * 32767.0f);

    // Multi-rate FIR with interp=5, decim=8 → 5/8 ratio, 80 kHz → 50 kHz.
    // length here is the total filter length (taps × interp = 64 × 5 = 320),
    // matching the size of resample_coeffs and the per-channel delay arrays.
    esp_err_t r_init_i = dsps_firmr_init_s16(&resampler_i, resample_coeffs, resample_delay_i,
                                             RESAMPLE_TAPS * RESAMPLE_INTERP,
                                             RESAMPLE_INTERP, RESAMPLE_DECIM, 0, 15);
    esp_err_t r_init_q = dsps_firmr_init_s16(&resampler_q, resample_coeffs, resample_delay_q,
                                             RESAMPLE_TAPS * RESAMPLE_INTERP,
                                             RESAMPLE_INTERP, RESAMPLE_DECIM, 0, 15);
    if (r_init_i != ESP_OK || r_init_q != ESP_OK) {
        ESP_LOGE(TAG, "Stage 2 resampler init failed: i=%d q=%d", r_init_i, r_init_q);
        return ESP_FAIL;
    }

    xTaskCreatePinnedToCore(worker_task, "worker_core1", 16384, NULL, 5, NULL, 1);
    return ESP_OK;
}

void worker_core1_push_burst(const detected_burst_t *burst)
{
    if (burst_queue) {
        xQueueSend(burst_queue, burst, 0);
    }
}
