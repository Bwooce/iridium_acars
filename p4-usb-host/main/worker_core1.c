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
#include "dsp_processor.h"    // FS_IN_HZ, FFT_SIZE, IRIDIUM_CHANNEL_HZ
#include "qpsk_demod.h"
#include "polyphase_channelizer.h"   // POLYCHAN_M
#include "uw_correlator.h"
#include "burst_pipeline.h"
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

// One-shot per-stage IQ dump. Triggers on the first burst with
// channelizer SNR ≥ DIAG_SNR_TRIGGER_DB so we capture a high-quality
// burst (not noise) for offline analysis. Each stage prints a
// header line with (rms, peak, dc-bias) followed by full IQ at
// stages 3-7, then an ENDDUMP marker. Stages 1-2 (raw + freq-
// shifted, both at 2.56 MSPS) print stats only since they are
// 40+ KB each and serial-bound.
//
// Parse with tests/scripts/parse_worker_diag.py (TBD) — extracts each
// DUMP block as int16 IQ and writes a .cf32 file gr-iridium can ingest.
#define DIAG_SNR_TRIGGER_DB  18.0f
static volatile bool s_diag_armed = true;

static void diag_stats(const char *name, int fs_hz, const int16_t *iq, int n_complex)
{
    if (n_complex <= 0) return;
    int64_t sum_re = 0, sum_im = 0;
    int64_t sum_sq = 0;
    int32_t peak_abs = 0;
    for (int i = 0; i < n_complex; i++) {
        int32_t r = iq[i * 2 + 0];
        int32_t q = iq[i * 2 + 1];
        sum_re += r;
        sum_im += q;
        sum_sq += r * r + q * q;
        int32_t ar = r < 0 ? -r : r;
        int32_t aq = q < 0 ? -q : q;
        if (ar > peak_abs) peak_abs = ar;
        if (aq > peak_abs) peak_abs = aq;
    }
    double rms = (n_complex > 0) ? sqrt((double)sum_sq / (double)n_complex) : 0.0;
    double dc_re = (double)sum_re / (double)n_complex;
    double dc_im = (double)sum_im / (double)n_complex;
    ESP_LOGI(TAG, "DIAG %s fs=%d n=%d rms=%.1f peak=%d dc=(%.1f,%.1f)",
             name, fs_hz, n_complex, rms, (int)peak_abs, dc_re, dc_im);
}

static void diag_dump(const char *name, int fs_hz, const int16_t *iq, int n_complex)
{
    diag_stats(name, fs_hz, iq, n_complex);
    // Bracketed dump that's easy to grep+parse offline. 4 IQ pairs/line
    // keeps serial line lengths under 64 chars.
    printf("DUMP %s n=%d\n", name, n_complex);
    for (int i = 0; i < n_complex; i += 4) {
        char line[96];
        int len = 0;
        len += snprintf(line + len, sizeof(line) - len, "DAT");
        for (int j = 0; j < 4 && (i + j) < n_complex; j++) {
            len += snprintf(line + len, sizeof(line) - len, " %d %d",
                            iq[(i + j) * 2 + 0], iq[(i + j) * 2 + 1]);
        }
        puts(line);
        // Pace serial output — 115200 baud is ~10 KB/sec, a long burst
        // can flood the TX buffer and drop bytes.
        if ((i & 0xff) == 0) vTaskDelay(1);
    }
    puts("ENDDUMP");
    vTaskDelay(pdMS_TO_TICKS(20));
}

// D7+ architectural change: worker consumes channelizer output (40 kHz
// per channel) directly via dsp_processor_extract_channel(). No more
// stage-1 FIR / freq-shift from the raw 2.56 MSPS signal buffer — the
// channelizer already filtered the burst's channel through a 1024-tap
// polyphase, much sharper than the 64-tap Hamming we had at stage 1.
// Matches gr-iridium's burst_downmix architecture.
#define CHANNEL_SAMPLE_HZ (FS_IN_HZ / POLYCHAN_M)   // 40000 Hz

// Resample 40 kHz -> 250 kHz (Interp 25, Decim 4 — gives 40 × 25/4 =
// 250 kHz = 10 sps × 25 ksym/s). 10 sps matches gr-iridium's
// burst_downmix internal rate; uw_correlator's UW_SPS = 10.
#define RESAMPLE_INTERP 25
#define RESAMPLE_DECIM 4
#define RESAMPLE_TAPS 64

// After matched filter + pre-rotation we decimate 5× back to 50 kHz
// (2 sps) so qpsk_demod (which expects 2 sps interleaved IQ) doesn't
// need changing.
#define POST_CORR_DECIM 5

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

// Stage-1 FIR removed (D7+): the channelizer's 1024-tap polyphase filter
// has already shaped this channel's burst better than any 64-tap FIR we
// could put here. Kept the symbol so the linker doesn't complain about
// any dangling reference; will fully delete in a follow-up cleanup.

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

            // 1. Extract the burst's channel from the channelizer's
            // retention ring. The channelizer has already filtered and
            // downconverted this channel through its 1024-tap polyphase
            // (sharper than any per-burst FIR we could afford); pull
            // its int16 IQ directly at the 40 kHz channel rate. No
            // separate freq-shift / stage-1 FIR needed — that path
            // was the legacy single-FFT-detector inheritance.
            int64_t ts = esp_timer_get_time();
            size_t n_channel_cplx = dsp_processor_extract_channel(
                burst.channel, burst.start_sample_idx,
                burst.length_samples, extract_buf);
            int64_t t_extract = esp_timer_get_time();
            if (n_channel_cplx == 0) {
                ESP_LOGW(TAG, "extract_channel returned 0 samples for burst");
                s_bursts_skipped++;
                continue;
            }

            // Decide whether THIS burst gets the one-shot diagnostic.
            bool diag = false;
            if (s_diag_armed && burst.peak_snr_db >= DIAG_SNR_TRIGGER_DB) {
                s_diag_armed = false;
                diag = true;
                ESP_LOGI(TAG, "=== DIAG BEGIN burst SNR=%.1f dB ch=%d len=%lu (%zu @ 40k) ===",
                         (double)burst.peak_snr_db, burst.channel,
                         (unsigned long)burst.length_samples, n_channel_cplx);
                diag_dump("01_channelizer_40k", CHANNEL_SAMPLE_HZ,
                          extract_buf, (int)n_channel_cplx);
            }

            // 2. Worker freq-shift removed: actual Iridium carriers
            // scatter ±5-10 kHz around the nominal 41.667 kHz grid (see
            // iridium.bits in the corpus), so grid-snapping leaves
            // unpredictable residual that uw_correlator's ±π clamp
            // can't always cover. Per-burst pre-PLL diagnostic on the
            // 1-sec ALBQ fixture showed period-5 (=5 kHz residual)
            // tones in the post-rotation signal — meaning >5 kHz left
            // uncancelled. Cleaner approach: pass the signal straight
            // through and let uw_correlator's squared-FFT measure and
            // cancel the FULL residual carrier (worker channel offset
            // + Iridium-grid mismatch + actual scatter). Channels sit
            // at ±20 kHz of LO; squared-FFT measures across ±π rad/sym
            // = ±12.5 kHz at 25 ksym/s. For bursts past ±12.5 kHz of
            // channel centre, uw_correlator's clamp truncates and the
            // PLL has to chase the remainder — same risk as before
            // but now the burst-by-burst measurement is honest.
            (void)burst.channel;   // no longer used for freq math
            int64_t t_freq = esp_timer_get_time();
            // No stage-1 FIR — channelizer already did the filtering.
            int64_t t_fir = t_freq;

            if (diag) {
                diag_stats("02_residual_shifted_40k", CHANNEL_SAMPLE_HZ,
                           extract_buf, (int)n_channel_cplx);
            }

            // 3. Resample 40 kHz -> 250 kHz (interp 25, decim 4).
            // Split interleaved IQ into separate I and Q for the
            // single-channel firmr. extract_buf holds n_channel_cplx
            // complex samples interleaved; stage1_in_i/q are scratch.
            for (size_t i = 0; i < n_channel_cplx; i++) {
                stage1_in_i[i] = extract_buf[i * 2 + 0];
                stage1_in_q[i] = extract_buf[i * 2 + 1];
            }
            int out_samples_250k = dsps_firmr_s16(&resampler_i,
                                                   stage1_in_i,
                                                   &resample_buf[0],
                                                   (int)n_channel_cplx);
            dsps_firmr_s16(&resampler_q,
                           stage1_in_q,
                           &resample_buf[MAX_EXTRACT_SAMPLES + DSP_PADDING_ELEMS],
                           (int)n_channel_cplx);
            int64_t t_resamp = esp_timer_get_time();

            // Group delay guard: polyphase resampler takes a few cycles
            // to fill its tap line.
            if (out_samples_250k <= (RESAMPLE_TAPS / RESAMPLE_DECIM)) {
                ESP_LOGD(TAG, "Resampler produced %d samples — too short for demod, skipping",
                         out_samples_250k);
                s_bursts_skipped++;
                continue;
            }
            int out_samples_50k = out_samples_250k;   // legacy name kept
                                                       // for the rest of
                                                       // the function

            ESP_LOGI(TAG, "Burst processed: ch=%d 40k=%zu 250k=%d (raw_len=%lu)",
                     burst.channel, n_channel_cplx, out_samples_50k,
                     burst.length_samples);

            // 5. QPSK Demodulation
            // Pack I and Q back into interleaved for demod
            for (int i = 0; i < out_samples_50k; i++) {
                demod_interleaved[i * 2 + 0] = resample_buf[i];
                demod_interleaved[i * 2 + 1] = resample_buf[MAX_EXTRACT_SAMPLES + DSP_PADDING_ELEMS + i];
            }

            if (diag) {
                // Reinterleave stage-1 80k output for the dump so the
                // Stage-1 FIR removed (D7+). Just dump the resampler
                // output at 250 kHz; the channelizer-output dump
                // covered the 40 kHz stage upstream.
                diag_dump("03_resamp_250k", 250000,
                          demod_interleaved, out_samples_50k);
            }

            // (D10 timing recovery wiring removed. Per gr-iridium's
            // burst_downmix_impl.cc, the correct approach for burst-
            // mode timing is one-shot UW cross-correlation with
            // parabolic peak interpolation, NOT a continuous Gardner
            // loop. New uw_correlator module replaces this hook.)

            // Hand off to the shared per-burst pipeline (common/
            // iridium_decoder/burst_pipeline.c). Single source of truth
            // for D13 → coarse CFO → freq-correct → RRC → UW correlator
            // → pre-rotation → decim → qpsk_demod; was previously
            // duplicated between this file and test_worker_pipeline_albq.c
            // with subtle drift (sub-sample correction, precision).
            burst_pipeline_result_t bres;
            burst_pipeline_process_250khz(demod_interleaved, out_samples_50k,
                                           &bres);
            ESP_LOGI(TAG, "D13 burst start: %d (of %d samples)",
                     bres.burst_start, out_samples_50k);
            ESP_LOGI(TAG, "UW corr: dir=%s offset=%d corr=%.3f SNR=%.1f dB peak=%.2e omega=%.3f",
                     bres.uw_res.direction == UW_DIR_DOWNLINK ? "DL" :
                     bres.uw_res.direction == UW_DIR_UPLINK   ? "UL" : "UNKNOWN",
                     bres.uw_res.uw_offset, (double)bres.uw_res.correction,
                     (double)bres.uw_res.snr_estimate_db,
                     (double)bres.uw_res.peak_value,
                     (double)bres.uw_res.omega_per_sym);
            if (diag && bres.uw_res.direction != UW_DIR_UNKNOWN) {
                // Best-effort: the shared pipeline left the burst's
                // 2-sps decimated stream at adj_burst+int_base, but
                // there's no longer a separate dump hook for the
                // intermediate buffers. Skip the per-step dumps.
                ESP_LOGI(TAG, "=== DIAG END (shared pipeline) ===");
            }
            uw_corr_result_t uw_res = bres.uw_res;
            bool demod_ok = bres.demod_ok;
            decoded_frame_t frame = bres.frame;
            (void)uw_res;
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

    // Stage 1 FIR removed (D7+). The channelizer already filters and
    // decimates this channel through a 1024-tap polyphase; the legacy
    // 64-tap Hamming stage-1 FIR was both redundant AND coarser. The
    // worker now consumes channelizer output directly.

    // Stage 2 Coefficients: polyphase resampler 40k -> 250k (interp=25,
    // decim=4). Input rate is now CHANNEL_SAMPLE_HZ (= FS_IN_HZ /
    // POLYCHAN_M = 40 kHz); intermediate polyphase rate = 1 MHz; output
    // 250 kHz. Cutoff stays at ±20 kHz (half channel BW) which is
    // Nyquist for the 40 kHz input — so the filter is mostly an
    // interpolation/imaging filter rather than an anti-alias filter.
    float rcoeffs_f32[RESAMPLE_TAPS * RESAMPLE_INTERP];
    const float stage2_poly_rate_hz = (float)CHANNEL_SAMPLE_HZ
                                      * (float)RESAMPLE_INTERP;       // 1 MHz
    const float channel_half_bw_hz = (float)FS_IN_HZ / (2.0f * (float)POLYCHAN_M);
    float r_omega_c = 2.0f * (float)M_PI * channel_half_bw_hz / stage2_poly_rate_hz;
    for (int i = 0; i < RESAMPLE_TAPS * RESAMPLE_INTERP; i++) {
        float n = i - (RESAMPLE_TAPS * RESAMPLE_INTERP - 1) / 2.0f;
        float h = (fabsf(n) < 1e-9f) ? (r_omega_c / M_PI) : (sinf(r_omega_c * n) / (M_PI * n));
        float w = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (RESAMPLE_TAPS * RESAMPLE_INTERP - 1)));
        rcoeffs_f32[i] = h * w;
    }
    // Same overflow-safe normalisation as stage 1: pick the largest
    // shift such that max coeff × scale fits in int16.
    float rcoeff_sum = 0, rcoeff_max = 0;
    const int rN = RESAMPLE_TAPS * RESAMPLE_INTERP;
    for (int i = 0; i < rN; i++) {
        rcoeff_sum += rcoeffs_f32[i];
        float a = fabsf(rcoeffs_f32[i]);
        if (a > rcoeff_max) rcoeff_max = a;
    }
    // Polyphase resampler: each output uses TAPS coeffs (one phase
    // out of the INTERP-many polyphase banks). Scale TOTAL coeff sum
    // to RESAMPLE_INTERP × 2^15 so each phase sums to ~2^15 → unity
    // gain per output with shift_param = 0.
    const float RSUM_TARGET = (float)RESAMPLE_INTERP * 32768.0f;
    float rcoeff_scale = RSUM_TARGET / rcoeff_sum;
    int rcoeff_max_int = 0, rcoeff_sum_int = 0;
    for (int i = 0; i < rN; i++) {
        float v = rcoeffs_f32[i] * rcoeff_scale;
        if (v > (float)INT16_MAX) v = (float)INT16_MAX;
        if (v < (float)INT16_MIN) v = (float)INT16_MIN;
        resample_coeffs[i] = (int16_t)lrintf(v);
        int a = resample_coeffs[i] < 0 ? -resample_coeffs[i] : resample_coeffs[i];
        if (a > rcoeff_max_int) rcoeff_max_int = a;
        rcoeff_sum_int += resample_coeffs[i];
    }
    ESP_LOGI(TAG, "Stage 2 resampler: shift=0 (Q15), max coeff=%d, sum=%d (target %d)",
             rcoeff_max_int, rcoeff_sum_int, (int)RSUM_TARGET);

    // Multi-rate FIR with interp=25, decim=8 → 80 kHz → 250 kHz.
    esp_err_t r_init_i = dsps_firmr_init_s16(&resampler_i, resample_coeffs, resample_delay_i,
                                             rN,
                                             RESAMPLE_INTERP, RESAMPLE_DECIM, 0, 0);
    esp_err_t r_init_q = dsps_firmr_init_s16(&resampler_q, resample_coeffs, resample_delay_q,
                                             rN,
                                             RESAMPLE_INTERP, RESAMPLE_DECIM, 0, 0);
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
        s_phasor_lut[i] = (int16_t)(sinf(term) * (float)INT16_MAX);
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
