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
#include "esp_timer.h"
#include "worker_core1.h"
#include "signal_buffer.h"
#include "qpsk_demod.h"
#include "freq_estimator.h"
#include "bch_decoder.h"
#include "frame_decoder.h"

static const char *TAG = "WORKER1";

static QueueHandle_t burst_queue = NULL;

// Diagnostic counters. Read & reset by worker_core1_get_stats().
static volatile uint32_t s_bursts_queued = 0;     // pushed to queue (incl. dropped)
static volatile uint32_t s_bursts_dropped = 0;    // queue full when push attempted
static volatile uint32_t s_bursts_processed = 0;  // ran end-to-end through the worker
static volatile uint32_t s_bursts_skipped = 0;    // hit edge/length/zero-output guard
static volatile uint32_t s_queue_high_water = 0;  // peak observed depth
static volatile uint64_t s_burst_total_us = 0;    // sum of wall-clock per processed burst

// Per-stage timing accumulators, summed over processed bursts only.
static volatile uint64_t s_t_extract_us = 0;
static volatile uint64_t s_t_freq_center_us = 0;
static volatile uint64_t s_t_fir_decim_us = 0;
static volatile uint64_t s_t_resample_us = 0;
static volatile uint64_t s_t_demod_us = 0;
static volatile uint64_t s_t_bch_us = 0;

// Decimation factor 32: 2.56 MHz -> 80 kHz
#define DECIM_FACTOR 32
#define FIR_TAPS 64

// Resample 80 kHz -> 50 kHz (Ratio 1.6, Interp 5, Decim 8)
#define RESAMPLE_INTERP 5
#define RESAMPLE_DECIM 8
#define RESAMPLE_TAPS 64

// Buffer for burst extraction (max 50ms = 128k complex samples)
#define MAX_EXTRACT_SAMPLES (128 * 1024)
// Padding for arp4 assembly kernels (32 bytes = 16 int16_t elements)
#define DSP_PADDING_ELEMS 16

static int16_t *extract_buf = NULL;
static int16_t *phasor_buf = NULL;
static int16_t *decim_buf = NULL; // 80 kHz buffer
static int16_t *resample_buf = NULL; // 50 kHz buffer

// Pre-allocated stage buffers to avoid runtime fragmentation
static int16_t *stage1_in_i = NULL;
static int16_t *stage1_in_q = NULL;
static int16_t *demod_interleaved = NULL;

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

// Frequency-centring phasor generator. Pre-allocate the sin LUT once and
// reuse the cplx_sig_t across bursts via dsps_cplx_gen_freq_set(), instead
// of init+free per burst (each cplx_gen_init call mallocs ~2 KB internally,
// which dominated the worker time at ~10 ms/burst).
#define PHASOR_LUT_LEN 1024
static cplx_sig_t s_phasor_gen;
static int16_t    s_phasor_lut[PHASOR_LUT_LEN] __attribute__((aligned(16)));

void worker_task(void *arg)
{
    ESP_LOGI(TAG, "Worker Task started on Core %d", xPortGetCoreID());
    detected_burst_t burst;

    while (1) {
        if (xQueueReceive(burst_queue, &burst, portMAX_DELAY)) {
            int64_t burst_t0 = esp_timer_get_time();
            ESP_LOGI(TAG, "Worker processing burst: Start:%lu Len:%lu Bin:%d SNR:%.2f dB",
                     burst.start_sample_idx, burst.length_samples, burst.peak_bin, burst.peak_snr_db);

            // Minimum length guard: need enough samples to survive 32x decimation and filter delay
            if (burst.length_samples < 128) {
                ESP_LOGW(TAG, "Burst too short (%lu samples) — dropping", burst.length_samples);
                s_bursts_skipped++;
                continue;
            }

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
                s_bursts_skipped++;
                continue;
            }

            // 1. Extract from Circular Buffer
            int64_t ts = esp_timer_get_time();
            signal_buffer_extract(burst.start_sample_idx, burst.length_samples, extract_buf);
            int64_t t_extract = esp_timer_get_time();

            // 1b. D8 fine carrier frequency estimation. The channelizer
            // reports peak_bin at channel granularity (40 kHz steps),
            // leaving up to ±20 kHz residual carrier offset — 16×
            // outside the PLL's ~1.25 kHz capture range. Refine by:
            //   (i) pre-mix the first FREQ_EST_FFT_N samples by the
            //       coarse offset into a stack scratch buffer (so the
            //       burst lands near DC of the scratch),
            //  (ii) run the estimator on the scratch → residual_hz
            //       inside the ±20 kHz window,
            // (iii) combine coarse + residual into the freq_offset
            //       that the existing mix-down uses.
            // This costs ~50-100 µs/burst — negligible against the
            // FIR decimation that follows.
            float coarse_offset_hz = (burst.peak_bin - 1024) * 1250.0f;
            int32_t residual_hz = 0;
            if (burst.length_samples >= FREQ_EST_FFT_N) {
                __attribute__((aligned(16))) int16_t pre_mix[FREQ_EST_FFT_N * 2];
                float phi_step = -2.0f * (float)M_PI * coarse_offset_hz / 2560000.0f;
                float c_step = cosf(phi_step), s_step = sinf(phi_step);
                float c = 1.0f, s = 0.0f;       // current phasor
                for (int i = 0; i < FREQ_EST_FFT_N; i++) {
                    int32_t x_re = extract_buf[i * 2 + 0];
                    int32_t x_im = extract_buf[i * 2 + 1];
                    // (x_re + j x_im) × (c + j s)
                    float yr = (float)x_re * c - (float)x_im * s;
                    float yi = (float)x_re * s + (float)x_im * c;
                    pre_mix[i * 2 + 0] = (int16_t)lrintf(yr);
                    pre_mix[i * 2 + 1] = (int16_t)lrintf(yi);
                    // Advance phasor: (c, s) ← (c, s) × (c_step, s_step)
                    float nc = c * c_step - s * s_step;
                    float ns = c * s_step + s * c_step;
                    c = nc; s = ns;
                }
                // Search ±40 kHz (full channelizer channel spacing).
                // The 40 kHz channelizer spacing mismatches Iridium's
                // 41.667 kHz, and the channelizer's ~40 dB adjacent-
                // channel rejection lets it trigger on leakage, so
                // the actual carrier can land outside the ±20 kHz
                // half-channel that you'd expect from a clean assignment.
                residual_hz = freq_estimator_run(pre_mix, FREQ_EST_FFT_N,
                                                  2560000, 40000);
            }

            // 2. Frequency Centering
            float freq_offset = coarse_offset_hz + (float)residual_hz;
            ESP_LOGD(TAG, "freq: coarse=%.0f residual=%d total=%.0f Hz (peak_bin=%d)",
                     (double)coarse_offset_hz, (int)residual_hz,
                     (double)freq_offset, burst.peak_bin);
            float norm_freq = -freq_offset / 2560000.0f;
            // dsps_cplx_gen accepts normalised frequency in (-1, 1) exclusive.
            // Clamp defensively in case detector ever emits an unusual peak_bin.
            float gen_freq = norm_freq * 2.0f;
            if (gen_freq >= 1.0f)  gen_freq = 0.999f;
            if (gen_freq <= -1.0f) gen_freq = -0.999f;
            // Reuse the pre-initialised generator (LUT allocated once at init).
            // Switching frequency on an already-initialised generator avoids
            // the per-burst malloc that was costing ~10 ms on the previous path.
            dsps_cplx_gen_freq_set(&s_phasor_gen, gen_freq);
            dsps_cplx_gen(&s_phasor_gen, phasor_buf, burst.length_samples);

            for (uint32_t i = 0; i < burst.length_samples; i++) {
                int32_t x_re = extract_buf[i * 2 + 0];
                int32_t x_im = extract_buf[i * 2 + 1];
                int32_t p_re = phasor_buf[i * 2 + 0];
                int32_t p_im = phasor_buf[i * 2 + 1];
                extract_buf[i * 2 + 0] = (int16_t)((x_re * p_re - x_im * p_im) >> 15);
                extract_buf[i * 2 + 1] = (int16_t)((x_re * p_im + x_im * p_re) >> 15);
            }
            int64_t t_freq = esp_timer_get_time();

            // 3. FIR Decimation (32x) Stage 1 (2.56M -> 80k)
            memset(delay_i, 0, sizeof(delay_i));
            memset(delay_q, 0, sizeof(delay_q));
            fir_i.d_pos = 0;
            fir_q.d_pos = 0;

            for (uint32_t i = 0; i < burst.length_samples; i++) {
                stage1_in_i[i] = extract_buf[i * 2 + 0];
                stage1_in_q[i] = extract_buf[i * 2 + 1];
            }

            // dsps_fird_s16's length parameter is the OUTPUT length (input/decim).
            // Passing the input length would cause a massive out-of-bounds read.
            int expected_out_80k = burst.length_samples / DECIM_FACTOR;

            // Note: dsps_fird_s16_arp4 on P4 has a bug where it returns an uninitialized
            // register (a6) instead of the output count. We use the expected count.
            dsps_fird_s16_arp4(&fir_i, stage1_in_i, &decim_buf[0], expected_out_80k);
            dsps_fird_s16_arp4(&fir_q, stage1_in_q, &decim_buf[MAX_EXTRACT_SAMPLES + DSP_PADDING_ELEMS], expected_out_80k);
            int out_samples_80k = expected_out_80k;
            int64_t t_fir = esp_timer_get_time();

            if (out_samples_80k <= 2) {
                ESP_LOGD(TAG, "Stage 1 produced %d samples (input %lu) — too short, dropping",
                         out_samples_80k, burst.length_samples);
                s_bursts_skipped++;
                continue;
            }

            // 4. Resample Stage 2 (80k -> 50k via interp=5, decim=8)
            // Note: dsps_firmr_s16's length parameter is the INPUT length.
            int out_samples_50k = dsps_firmr_s16(&resampler_i, &decim_buf[0], &resample_buf[0], out_samples_80k);
            dsps_firmr_s16(&resampler_q, &decim_buf[MAX_EXTRACT_SAMPLES + DSP_PADDING_ELEMS], &resample_buf[MAX_EXTRACT_SAMPLES + DSP_PADDING_ELEMS], out_samples_80k);
            int64_t t_resamp = esp_timer_get_time();

            // Group delay guard: polyphase resampler takes some samples to fill its taps.
            // Drop bursts where resampler didn't have enough samples to emit data.
            if (out_samples_50k <= (RESAMPLE_TAPS / RESAMPLE_DECIM)) {
                ESP_LOGD(TAG, "Stage 2 produced %d samples — too short for demod, skipping", out_samples_50k);
                s_bursts_skipped++;
                continue;
            }

            ESP_LOGI(TAG, "Burst processed: 80k=%d 50k=%d (input len=%lu)",
                     out_samples_80k, out_samples_50k, burst.length_samples);

            // 5. QPSK Demodulation
            decoded_frame_t frame;
            // Pack I and Q back into interleaved for demod
            for (int i = 0; i < out_samples_50k; i++) {
                demod_interleaved[i * 2 + 0] = resample_buf[i];
                demod_interleaved[i * 2 + 1] = resample_buf[MAX_EXTRACT_SAMPLES + DSP_PADDING_ELEMS + i];
            }

            // qpsk_demod assumes its input starts at the unique word.
            // The channelizer's start_sample_idx is at the threshold-
            // crossing point, so the real UW is somewhere INSIDE the
            // extracted burst (after a preamble ramp + envelope slack).
            // Slide the input start in 1-complex-sample steps (4 int16
            // = 0.5 symbol at 2 sps) until the demod finds a UW match.
            // This is the same sliding the host regression test uses
            // for the corpus fixtures.
            const int STEP_INT16 = 4;            // 1 complex = 0.5 symbol
            const int MAX_DEMOD_OFFSET = 200;    // ~100 symbols slack
            bool demod_ok = false;
            int chosen_offset = 0;
            int total_int16 = out_samples_50k * 2;
            for (int sym_off = 0; sym_off < MAX_DEMOD_OFFSET; sym_off++) {
                int int16_off = sym_off * STEP_INT16;
                if (total_int16 - int16_off < 24 * STEP_INT16) break;
                memset(&frame, 0, sizeof(frame));
                if (qpsk_demod_process(demod_interleaved + int16_off,
                                        total_int16 - int16_off, &frame)) {
                    demod_ok = true;
                    chosen_offset = sym_off;
                    break;
                }
            }
            if (demod_ok) {
                ESP_LOGD(TAG, "demod sliding: UW at +%d symbols", chosen_offset);
            }
            int64_t t_demod = esp_timer_get_time();
            int64_t t_bch = t_demod;
            if (demod_ok) {
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
                        char bin_str[22] = {0};
                        for (int i = 0; i < 21; i++) bin_str[i] = data1[i] ? '1' : '0';
                        ESP_LOGI(TAG, "Block1 Data: %s", bin_str);
                    }
                }
                t_bch = esp_timer_get_time();

                // Hand the demodulated bits to the higher-layer
                // decoder via the PSRAM queue. Non-blocking — the
                // decoder runs in its own task on Core 1 and reports
                // drops via per-second status. We pass freq_hz=0 (the
                // worker doesn't know the absolute LO from inside its
                // task; peak_bin carries the relative offset which is
                // what the classifier and reassembler care about).
                frame_decoder_push(frame.bits, frame.n_bits,
                                   frame.direction, 0u,
                                   burst.peak_bin, burst.peak_snr_db);

                free(frame.bits);
            }

            s_bursts_processed++;
            s_burst_total_us  += (uint64_t)(esp_timer_get_time() - burst_t0);
            s_t_extract_us    += (uint64_t)(t_extract - ts);
            s_t_freq_center_us+= (uint64_t)(t_freq - t_extract);
            s_t_fir_decim_us  += (uint64_t)(t_fir - t_freq);
            s_t_resample_us   += (uint64_t)(t_resamp - t_fir);
            s_t_demod_us      += (uint64_t)(t_demod - t_resamp);
            s_t_bch_us        += (uint64_t)(t_bch - t_demod);
        }
    }
}

esp_err_t worker_core1_init()
{
    burst_queue = xQueueCreate(16, sizeof(detected_burst_t));
    if (!burst_queue) return ESP_ERR_NO_MEM;

    // Allocate all buffers in PSRAM with padding for arp4 kernels
    size_t buf_size = (MAX_EXTRACT_SAMPLES + DSP_PADDING_ELEMS) * 2 * sizeof(int16_t);
    extract_buf = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    phasor_buf = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    decim_buf = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    resample_buf = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
    
    stage1_in_i = heap_caps_malloc((MAX_EXTRACT_SAMPLES + DSP_PADDING_ELEMS) * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    stage1_in_q = heap_caps_malloc((MAX_EXTRACT_SAMPLES + DSP_PADDING_ELEMS) * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    demod_interleaved = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);

    if (!extract_buf || !phasor_buf || !decim_buf || !resample_buf || 
        !stage1_in_i || !stage1_in_q || !demod_interleaved) {
        return ESP_ERR_NO_MEM;
    }

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

    // Pre-populate the phasor sin LUT (Q15) and init the generator once.
    // dsps_cplx_gen_init only fills the LUT when called with lut==NULL, so
    // we do the population manually to match what the library would have
    // generated. After this, dsps_cplx_gen_freq_set() is enough per burst.
    for (int i = 0; i < PHASOR_LUT_LEN; i++) {
        float term = (2.0f * (float)M_PI) * ((float)i / (float)PHASOR_LUT_LEN);
        s_phasor_lut[i] = (int16_t)(sinf(term) * 32767.0f);
    }
    esp_err_t pg = dsps_cplx_gen_init(&s_phasor_gen, S16_FIXED, s_phasor_lut,
                                      PHASOR_LUT_LEN, 0.0f, 0.0f);
    if (pg != ESP_OK) {
        ESP_LOGE(TAG, "Phasor generator init failed: %d", pg);
        return ESP_FAIL;
    }

    xTaskCreatePinnedToCore(worker_task, "worker_core1", 16384, NULL, 5, NULL, 1);
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
        out->extract_us      = (float)s_t_extract_us / fn;
        out->freq_center_us  = (float)s_t_freq_center_us / fn;
        out->fir_decim_us    = (float)s_t_fir_decim_us / fn;
        out->resample_us     = (float)s_t_resample_us / fn;
        out->demod_us        = (float)s_t_demod_us / fn;
        out->bch_us          = (float)s_t_bch_us / fn;
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
    s_t_extract_us = s_t_freq_center_us = s_t_fir_decim_us =
        s_t_resample_us = s_t_demod_us = s_t_bch_us = 0;
}
