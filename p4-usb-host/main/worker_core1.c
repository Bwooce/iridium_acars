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
#include "uw_correlator.h"
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

            // D8 alignment with gr-iridium: their burst_downmix pipeline
            // never tries to estimate residual carrier on the raw QPSK
            // burst. QPSK is suppressed-carrier — a plain FFT of QPSK
            // samples returns a noise-driven peak that detunes the
            // burst by random kHz. gr-iridium relies entirely on the
            // per-burst squared-FFT inside burst_downmix (= our
            // uw_correlator's CFO estimate, run on the BPSK preamble+UW
            // window where squaring removes modulation).
            //
            // Worst-case residual after channelizer-only centering:
            // ~±0.8 kHz from the 40 kHz channelizer bin vs 41.667 kHz
            // Iridium-grid mismatch, plus ~±1 kHz Doppler residual.
            // Total ~±2 kHz = ~±0.5 rad/sym at our 25 ksym/s symbol
            // rate, well within uw_correlator's CFO range (±π rad/sym).
            float coarse_offset_hz = (burst.peak_bin - 1024) * 1250.0f;
            float freq_offset = coarse_offset_hz;
            ESP_LOGD(TAG, "freq: coarse=%.0f Hz (peak_bin=%d)",
                     (double)coarse_offset_hz, burst.peak_bin);
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

            // (D10 timing recovery wiring removed. Per gr-iridium's
            // burst_downmix_impl.cc, the correct approach for burst-
            // mode timing is one-shot UW cross-correlation with
            // parabolic peak interpolation, NOT a continuous Gardner
            // loop. New uw_correlator module replaces this hook.)

            // gr-iridium-aligned order (burst_downmix_impl.cc lines
            // 841-880 then 584-650):
            //   1. D13 start_finder on the raw resampled burst.
            //   2. RRC matched filter on the trimmed burst.
            //   3. uw_correlator_find on the trimmed+RRC'd burst.
            //   4. (existing) peak-phase + omega pre-rotation, then
            //      qpsk_demod_process.
            //
            // Search the whole burst for the envelope onset — gr-iridium
            // uses d_search_depth ~1000-3000 samples; we pass the full
            // burst length (the start_finder's internal LP filter
            // smooths noise, so wider search doesn't add false positives).
            int burst_start = uw_correlator_find_burst_start(
                                  demod_interleaved, out_samples_50k,
                                  /*search_max=*/out_samples_50k);
            int16_t *adj_burst = demod_interleaved + burst_start * 2;
            int adj_n = out_samples_50k - burst_start;
            uw_correlator_apply_rrc(adj_burst, adj_burst, adj_n);
            ESP_LOGI(TAG, "D13 burst start: %d (of %d samples)",
                     burst_start, out_samples_50k);
            uw_corr_result_t uw_res;
            uw_correlator_find(adj_burst, adj_n,
                                /*search_complex=*/adj_n - 24,
                                &uw_res);
            bool demod_ok = false;
            ESP_LOGI(TAG, "UW corr: dir=%s offset=%d corr=%.3f SNR=%.1f dB peak=%.2e omega=%.3f",
                     uw_res.direction == UW_DIR_DOWNLINK ? "DL" :
                     uw_res.direction == UW_DIR_UPLINK   ? "UL" : "UNKNOWN",
                     uw_res.uw_offset, (double)uw_res.correction,
                     (double)uw_res.snr_estimate_db, (double)uw_res.peak_value,
                     (double)uw_res.omega_per_sym);
            if (uw_res.direction != UW_DIR_UNKNOWN) {
                // D10b: incorporate the matched-filter peak's sub-
                // sample correction into the integer decimation phase.
                // uw_res.correction ∈ (-0.5, +0.5] tells us where the
                // true symbol-center sits between integer samples. If
                // |correction| > 0.5 we'd round to a different sample;
                // if positive and ≥ 0, the next integer sample is closer
                // to the real centre. We round to the nearest integer
                // shift (0 or +1) and apply it on top of uw_offset so
                // that qpsk_demod's fixed even-sample decimation lands
                // on symbol centres rather than between-symbol midpoints.
                int sub_shift = (uw_res.correction >= 0.5f) ? 1
                              : (uw_res.correction <= -0.5f) ? -1 : 0;
                int int16_off = (uw_res.uw_offset + sub_shift) * 2;
                if (int16_off < 0) int16_off = 0;
                // Pre-rotate burst by exp(+j·peak_phase) AND apply a
                // linear phase ramp to cancel the residual carrier omega
                // estimated from the UW two-half phase diff. After this,
                // the PLL starts with both phi and omega near zero.
                float pmag = sqrtf(uw_res.peak_re * uw_res.peak_re
                                 + uw_res.peak_im * uw_res.peak_im);
                // uw_res.uw_offset is RELATIVE to adj_burst (post-D13
                // trim), so src must start from adj_burst too. The
                // n_rot length is what's left in adj_burst after the UW.
                int n_rot_int16 = adj_n * 2 - int16_off;
                int16_t *src = adj_burst + int16_off;
                if (pmag > 1e-3f) {
                    // Constant phase factor (rotates burst back to UW axes).
                    float rot_re =  uw_res.peak_re / pmag;
                    float rot_im =  uw_res.peak_im / pmag;
                    // Per-SAMPLE phase increment = omega_per_sym/2
                    // (omega is per SYMBOL; burst is at 2 sps). Sign:
                    // burst has phase -omega·sym_idx encoded; we
                    // multiply by exp(+j·omega·sym_idx) to undo it.
                    float dphi = uw_res.omega_per_sym * 0.5f;
                    float c_step = cosf(dphi);
                    float s_step = sinf(dphi);
                    int n_cplx = n_rot_int16 / 2;
                    float pr = rot_re, pi = rot_im;
                    for (int i = 0; i < n_cplx; i++) {
                        float re = (float)src[i * 2 + 0];
                        float im = (float)src[i * 2 + 1];
                        float nr = re * pr - im * pi;
                        float ni = re * pi + im * pr;
                        if (nr >  32767.0f) nr =  32767.0f;
                        if (nr < -32768.0f) nr = -32768.0f;
                        if (ni >  32767.0f) ni =  32767.0f;
                        if (ni < -32768.0f) ni = -32768.0f;
                        src[i * 2 + 0] = (int16_t)nr;
                        src[i * 2 + 1] = (int16_t)ni;
                        // Advance phasor: p ← p · exp(j·dphi)
                        float npr = pr * c_step - pi * s_step;
                        float npi = pr * s_step + pi * c_step;
                        pr = npr; pi = npi;
                    }
                }
                // (Gardner symbol-timing recovery is intentionally not
                // wired into the worker pipeline yet. With default
                // textbook gains it regressed the only burst that
                // decoded under correlator + pre-rotation alone — the
                // loop introduces strobe jitter that the fixed-decim
                // qpsk_demod can't tolerate. sym_timing module retained
                // for offline tuning via tests/host/test_sym_timing_trace.c.)
                memset(&frame, 0, sizeof(frame));
                if (qpsk_demod_process(src, n_rot_int16, &frame)) {
                    demod_ok = true;
                }
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
