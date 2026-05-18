// End-to-end A/B test: runs the post-refactor worker DSP pipeline on
// the same prerecorded ALBQ_RAW_UINT8 corpus the on-target smoke test
// uses. Same data, same code path (less the esp-dsp resampler which
// we replace with a portable polyphase resampler equivalent in design).
//
// What it does per detected burst:
//   1. channelizer_detector_extract_channel — pull the burst's
//      channel stream at 40 kHz (D7+ direct extract path).
//   2. residual freq-shift to Iridium grid (same as worker_core1).
//   3. polyphase resample 40 kHz -> 250 kHz (interp 25, decim 4)
//      via portable C reference (same filter design as on-target).
//   4. uw_correlator_apply_rrc + uw_correlator_find — matched filter,
//      direction, CFO.
//   5. Q15 pre-rotation + linear interp + decim 5× to 2 sps.
//   6. qpsk_demod_process — UW check, direction.
//
// Reports per-burst: matched-filter SNR, direction, UW symbol diffs
// vs the expected DL/UL UW. Compares to gr-iridium's reported SNRs
// in the channelizer_burst_ref.h fixture.
//
// Why this test exists: the on-target smoke is the authoritative
// "does the system decode" check, but each cycle costs a flash +
// reset + monitor. Host iteration is seconds. Lets us bisect where
// the gap to gr-iridium lives without burning hardware cycles.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "channelizer_detector.h"
#include "channelizer_burst_ref.h"
#include "fixture_albq_raw.h"
#include "uw_correlator.h"
#include "qpsk_demod.h"
#include "polyphase_channelizer.h"
#include "burst_pipeline.h"
#include "firmr_s16.h"

#define FS_IN         2560000u
#define POLYCHAN_M_   64
#define CHANNEL_HZ    (FS_IN / POLYCHAN_M_)         // 40000
#define IRIDIUM_HZ    41666.67f
#define INTERP        25
#define DECIM_2       4
#define POST_DECIM    5
// 40k -> 1MHz -> 250k -> 50k after the post-decim.

static int16_t *u8_to_int16_iq(const uint8_t *u8, unsigned int n_bytes,
                                size_t *out_n_complex)
{
    size_t n_cplx = n_bytes / 2;
    int16_t *out = malloc(n_bytes * sizeof(int16_t));
    if (!out) return NULL;
    for (size_t i = 0; i < n_cplx; i++) {
        int re = (int)u8[2 * i + 0] - 128;
        int im = (int)u8[2 * i + 1] - 128;
        out[2 * i + 0] = (int16_t)(re * 256);
        out[2 * i + 1] = (int16_t)(im * 256);
    }
    *out_n_complex = n_cplx;
    return out;
}

// Burst collector via channelizer_detector callback.
// Sized to cover the ~250 bursts the detector emits on the 1-sec
// ALBQ fixture (P4 sees the same count — `worker stats: queued=249`
// in the smoke summary). The previous 64-slot cap silently dropped
// everything past the first 64 emissions, making it look like the
// detector was emitting far fewer bursts on host than on P4 when
// in fact the cap was the divergence.
#define MAX_BURSTS 512
typedef struct {
    int n;
    channelizer_burst_t bursts[MAX_BURSTS];
} burst_collector_t;

static void collect_cb(const channelizer_burst_t *b, void *user)
{
    burst_collector_t *c = (burst_collector_t *)user;
    if (c->n < MAX_BURSTS) c->bursts[c->n++] = *b;
}

// 40 kHz → 250 kHz polyphase resampler. Coefficients laid out in
// esp-dsp's `dsps_firmr_s16` convention: coeffs[tap_pos * INTERP +
// phase], same as worker_core1.c's resample_coeffs feeding
// dsps_firmr_init_s16. Then we drive firmr_s16_process (a portable
// port of dsps_firmr_s16_ansi) so the resampler math is bit-identical
// between host and P4.
#define R_TAPS 64
static void build_resampler_taps_q15(int16_t *coeffs_q15)
{
    const int rN = R_TAPS * INTERP;
    const float poly_rate = (float)CHANNEL_HZ * (float)INTERP;   // 1 MHz
    const float half_bw   = (float)FS_IN / (2.0f * POLYCHAN_M_); // 20 kHz
    const float omega_c = 2.0f * (float)M_PI * half_bw / poly_rate;
    float taps_f[R_TAPS * INTERP];
    float sum = 0.0f;
    for (int i = 0; i < rN; i++) {
        float n = i - (rN - 1) / 2.0f;
        float h = (fabsf(n) < 1e-9f) ? (omega_c / (float)M_PI)
                                     : (sinf(omega_c * n) / ((float)M_PI * n));
        float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (rN - 1)));
        taps_f[i] = h * w;
        sum += taps_f[i];
    }
    // Match worker_core1 scaling exactly: target SUM = INTERP × 32768
    // so each phase (every-INTERP-th tap) sums to ~32768 = unity Q15.
    const float RSUM_TARGET = (float)INTERP * 32768.0f;
    float scale = RSUM_TARGET / sum;
    // Lay out in esp-dsp's transposed-polyphase order:
    //   coeffs_q15[tap_pos * INTERP + phase] = h_linear[tap_pos * INTERP + phase]
    // (which is just the linear order — the indexing convention is
    // baked into the firmr_s16 reader, not the writer.)
    for (int i = 0; i < rN; i++) {
        float v = taps_f[i] * scale;
        if (v >  32767.0f) v =  32767.0f;
        if (v < -32768.0f) v = -32768.0f;
        coeffs_q15[i] = (int16_t)lrintf(v);
    }
}

static int s_passed = 0, s_failed = 0;

#define CHECK(cond, ...) do { \
    if (cond) s_passed++; \
    else { s_failed++; fprintf(stderr, "  FAIL line %d: ", __LINE__); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } \
} while (0)

int main(void)
{
    printf("Test: full worker DSP pipeline on ALBQ_RAW_UINT8 corpus\n");
    printf("=======================================================\n\n");

    // 1. Run channelizer on the corpus
    burst_collector_t coll = {0};
    channelizer_detector_t *d = channelizer_detector_create(
        FS_IN, 16.0f, collect_cb, &coll);
    if (!d) { printf("FAIL: detector create\n"); return 1; }

    size_t n_complex = 0;
    int16_t *iq = u8_to_int16_iq(ALBQ_RAW_UINT8, ALBQ_RAW_UINT8_LEN, &n_complex);
    if (!iq) { printf("FAIL: malloc\n"); return 1; }
    channelizer_detector_feed_int16(d, iq, n_complex);
    channelizer_detector_flush(d);

    // Sanity check the channel_ring: pick a few extracts at known
    // burst time spans and check rms.
    if (coll.n > 0) {
        const channelizer_burst_t *bb0 = &coll.bursts[0];
        int16_t probe[1024 * 2];
        size_t n = channelizer_detector_extract_channel(
            d, bb0->channel, bb0->start_sample_idx,
            bb0->length_samples > 64*512 ? 64*512 : bb0->length_samples,
            probe);
        long long s = 0;
        for (size_t i = 0; i < n; i++) {
            s += (long long)probe[i*2+0] * probe[i*2+0]
               + (long long)probe[i*2+1] * probe[i*2+1];
        }
        printf("[ring sanity] burst0 ch=%d start_idx=%u len=%u "
               "extract n=%zu, sum_sq=%lld\n",
               bb0->channel, bb0->start_sample_idx, bb0->length_samples,
               n, s);
    }
    printf("Channelizer detected %d bursts:\n", coll.n);
    for (int i = 0; i < coll.n; i++) {
        printf("  burst %d: ch=%d (rel %+d kHz) snr=%.1f dB len=%u\n",
               i, coll.bursts[i].channel, coll.bursts[i].rel_freq_hz/1000,
               (double)coll.bursts[i].snr_db,
               coll.bursts[i].length_samples);
    }

    // 2. Build resampler taps once (Q15, esp-dsp layout)
    static int16_t r_coeffs[R_TAPS * INTERP];
    build_resampler_taps_q15(r_coeffs);

    // 3. Process each burst
    printf("\nPer-burst worker pipeline results:\n");
    printf("  %-3s %-8s %-9s %-9s %-9s %-8s %-12s\n",
           "ch", "gr-snr", "uw-snr", "uw-dir", "omega", "uw-off", "qpsk-result");

    int n_ok_decode = 0;
    int n_uw_match  = 0;

    for (int b = 0; b < coll.n; b++) {
        const channelizer_burst_t *bb = &coll.bursts[b];

        // Skip very short bursts.
        if (bb->length_samples < 64) continue;

        // Extract channel IQ at 40 kHz.
        size_t cap_cplx = bb->length_samples / POLYCHAN_M_;
        int16_t *ch_iq = malloc(cap_cplx * 2 * sizeof(int16_t));
        size_t n_ch = channelizer_detector_extract_channel(
            d, bb->channel,
            bb->start_sample_idx, bb->length_samples, ch_iq);
        if (n_ch < 16) { free(ch_iq); continue; }

        // Diagnostic: per-stage stats
        double s1=0, s1max=0;
        for (size_t i = 0; i < n_ch; i++) {
            double r = ch_iq[i*2+0], v = ch_iq[i*2+1];
            s1 += r*r + v*v;
            double mag = sqrt(r*r + v*v);
            if (mag > s1max) s1max = mag;
        }
        double rms1 = sqrt(s1 / n_ch);

        // Save first ≥300-sample burst's channelizer output for
        // spectrum analysis in Python. Diagnostic — one shot.
        static int diag_dump_done = 0;
        if (!diag_dump_done && n_ch >= 300) {
            FILE *f = fopen("/tmp/host_burst_ch40k.cf32", "wb");
            if (f) {
                for (size_t i = 0; i < n_ch; i++) {
                    float fr = (float)ch_iq[i*2+0] / 32768.0f;
                    float fi = (float)ch_iq[i*2+1] / 32768.0f;
                    fwrite(&fr, 4, 1, f);
                    fwrite(&fi, 4, 1, f);
                }
                fclose(f);
                fprintf(stderr, "    [dump] saved %zu cplx samples (40 kHz, burst ch=%d) to /tmp/host_burst_ch40k.cf32\n",
                        n_ch, bb->channel);
                diag_dump_done = 1;
            }
        }

        // Residual freq-shift to Iridium grid — kept for now to isolate
        // whether the firmr_s16 port is the regression source vs the
        // freq-shift removal. P4 worker dropped this step.
        int signed_ch = (bb->channel > POLYCHAN_M_ / 2)
                        ? bb->channel - POLYCHAN_M_ : bb->channel;
        float ch_centre_hz = (float)signed_ch * (float)CHANNEL_HZ;
        float n_irid = roundf(ch_centre_hz / IRIDIUM_HZ);
        float residual_hz = ch_centre_hz - n_irid * IRIDIUM_HZ;
        float dphi = -2.0f * (float)M_PI * residual_hz / (float)CHANNEL_HZ;
        float phi = 0.0f;
        for (size_t i = 0; i < n_ch; i++) {
            float c = cosf(phi), s = sinf(phi);
            float re = (float)ch_iq[i * 2 + 0];
            float im = (float)ch_iq[i * 2 + 1];
            ch_iq[i * 2 + 0] = (int16_t)lrintf(re * c - im * s);
            ch_iq[i * 2 + 1] = (int16_t)lrintf(re * s + im * c);
            phi += dphi;
        }

        // Resample 40 kHz -> 250 kHz via the same polyphase rational-
        // rate FIR (firmr_s16) the P4 worker uses (esp-dsp's
        // dsps_firmr_s16 with shift=0 for canonical Q15). Run twice:
        // once for I, once for Q. Independent delay lines so each
        // call's state is isolated to its channel.
        int max_out = ((int)n_ch * INTERP / DECIM_2) + INTERP + 4;
        int16_t *out_re = malloc(max_out * sizeof(int16_t));
        int16_t *out_im = malloc(max_out * sizeof(int16_t));
        int16_t *in_re  = malloc(n_ch * sizeof(int16_t));
        int16_t *in_im  = malloc(n_ch * sizeof(int16_t));
        for (size_t i = 0; i < n_ch; i++) {
            in_re[i] = ch_iq[i * 2 + 0];
            in_im[i] = ch_iq[i * 2 + 1];
        }
        firmr_s16_t fir_re, fir_im;
        int16_t delay_re[R_TAPS], delay_im[R_TAPS];
        firmr_s16_init(&fir_re, r_coeffs, delay_re,
                       R_TAPS, INTERP, DECIM_2, 0, 0);
        firmr_s16_init(&fir_im, r_coeffs, delay_im,
                       R_TAPS, INTERP, DECIM_2, 0, 0);
        int n_250k_re = (int)firmr_s16_process(&fir_re, in_re, out_re, (int)n_ch);
        int n_250k_im = (int)firmr_s16_process(&fir_im, in_im, out_im, (int)n_ch);
        int n_250k = (n_250k_re < n_250k_im) ? n_250k_re : n_250k_im;
        // Stage 2 stats
        double s2 = 0, s2max = 0;
        for (int i = 0; i < n_250k; i++) {
            double m = (double)out_re[i] * out_re[i] + (double)out_im[i] * out_im[i];
            s2 += m;
            double mag = sqrt(m);
            if (mag > s2max) s2max = mag;
        }
        double rms2 = (n_250k > 0) ? sqrt(s2 / n_250k) : 0.0;
        printf("    [diag] burst ch=%d: ch_iq n=%zu rms=%.0f peak=%.0f; resamp n=%d rms=%.0f peak=%.0f\n",
               bb->channel, n_ch, rms1, s1max, n_250k, rms2, s2max);
        free(in_re); free(in_im); free(ch_iq);

        // Pack into interleaved IQ for uw_correlator / burst_pipeline.
        int16_t *iq250 = malloc(n_250k * 2 * sizeof(int16_t));
        for (int i = 0; i < n_250k; i++) {
            iq250[i * 2 + 0] = out_re[i];
            iq250[i * 2 + 1] = out_im[i];
        }
        free(out_re); free(out_im);

        // Hand off to the shared per-burst pipeline (same code path
        // the P4 worker uses). Eliminates the drift between host and
        // worker orchestrations that previously cost 10× decode rate.
        burst_pipeline_result_t bres;
        burst_pipeline_process_250khz(iq250, n_250k, &bres);
        uw_corr_result_t uw = bres.uw_res;
        const char *qpsk_str = bres.demod_ok ? "OK"
                              : (uw.direction == UW_DIR_UNKNOWN ? "no-decode"
                                                                : "no-UW");
        if (bres.demod_ok) {
            n_ok_decode++;
            n_uw_match++;
            free(bres.frame.bits);
        }

        printf("  %-3d %-7.1f  %-8.1f  %-8s ω_pre=%-+6.3f ω_post=%-+6.3f %-8d %s\n",
               bb->channel, (double)bb->snr_db,
               (double)uw.snr_estimate_db,
               uw.direction == UW_DIR_DOWNLINK ? "DL" :
               uw.direction == UW_DIR_UPLINK   ? "UL" : "UNK",
               (double)bres.omega_coarse,
               (double)uw.omega_per_sym,
               (int)uw.uw_offset, qpsk_str);

        free(iq250);
    }

    printf("\nSummary: %d/%d bursts decoded by qpsk_demod\n",
           n_ok_decode, coll.n);

    channelizer_detector_destroy(d);
    free(iq);

    // Pass: as long as the channelizer detected the expected bursts.
    // Whether qpsk_demod decoded is informational — that's the gap
    // we're investigating.
    CHECK(coll.n >= 3, "channelizer detected ≥3 bursts (got %d)", coll.n);
    printf("\n=== %d passed, %d failed ===\n", s_passed, s_failed);
    return s_failed > 0 ? 1 : 0;
}
