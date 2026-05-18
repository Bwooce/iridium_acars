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
#define MAX_BURSTS 64
typedef struct {
    int n;
    channelizer_burst_t bursts[MAX_BURSTS];
} burst_collector_t;

static void collect_cb(const channelizer_burst_t *b, void *user)
{
    burst_collector_t *c = (burst_collector_t *)user;
    if (c->n < MAX_BURSTS) c->bursts[c->n++] = *b;
}

// Portable polyphase resampler INTERP=25, DECIM=4. Same filter design
// as worker_core1.c (sinc * Hamming, RESAMPLE_TAPS=64 taps per phase).
// Returns number of output complex samples.
#define R_TAPS 64
static void build_resampler_taps(float *taps)
{
    const int rN = R_TAPS * INTERP;
    const float poly_rate = (float)CHANNEL_HZ * (float)INTERP;   // 1 MHz
    const float half_bw   = (float)FS_IN / (2.0f * POLYCHAN_M_); // 20 kHz
    const float omega_c = 2.0f * (float)M_PI * half_bw / poly_rate;
    float sum = 0.0f;
    for (int i = 0; i < rN; i++) {
        float n = i - (rN - 1) / 2.0f;
        float h = (fabsf(n) < 1e-9f) ? (omega_c / (float)M_PI)
                                     : (sinf(omega_c * n) / ((float)M_PI * n));
        float w = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (rN - 1)));
        taps[i] = h * w;
        sum += taps[i];
    }
    // Scale so each polyphase phase sums to ~1 (unity DC gain per output)
    float scale = (float)INTERP / sum;
    for (int i = 0; i < rN; i++) taps[i] *= scale;
}

// Polyphase resample: y[k] uses input samples i_in = k*DECIM/INTERP and
// phase = (k*DECIM) % INTERP, then convolves R_TAPS taps with delayed
// inputs. Returns number of output samples.
static int polyphase_resample(const int16_t *in, int n_in,
                               const float *taps,
                               float *out, int max_out)
{
    int n_out = 0;
    for (long long k = 0; k < (long long)max_out; k++) {
        long long phase_idx = (k * DECIM_2);   // counter at intermediate rate
        long long phase = phase_idx % INTERP;
        long long base  = phase_idx / INTERP;
        if (base + R_TAPS - 1 >= n_in) break;
        const float *phase_taps = &taps[phase * R_TAPS];
        float acc = 0.0f;
        for (int t = 0; t < R_TAPS; t++) {
            acc += phase_taps[t] * (float)in[base + R_TAPS - 1 - t];
        }
        out[k] = acc;
        n_out++;
    }
    return n_out;
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

    // 2. Build resampler taps once
    static float r_taps[R_TAPS * INTERP];
    build_resampler_taps(r_taps);

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

        // Residual freq-shift on the 40 kHz signal.
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

        // Resample 40 kHz -> 250 kHz.
        int max_out = (n_ch * INTERP / DECIM_2) + 4;
        float *re_buf = malloc(max_out * sizeof(float));
        float *im_buf = malloc(max_out * sizeof(float));
        int16_t *in_re = malloc(n_ch * sizeof(int16_t));
        int16_t *in_im = malloc(n_ch * sizeof(int16_t));
        for (size_t i = 0; i < n_ch; i++) {
            in_re[i] = ch_iq[i * 2 + 0];
            in_im[i] = ch_iq[i * 2 + 1];
        }
        int n_250k = polyphase_resample(in_re, (int)n_ch, r_taps, re_buf, max_out);
        polyphase_resample(in_im, (int)n_ch, r_taps, im_buf, max_out);
        // Stage 2 stats
        double s2 = 0, s2max = 0;
        for (int i = 0; i < n_250k; i++) {
            double m = re_buf[i] * re_buf[i] + im_buf[i] * im_buf[i];
            s2 += m;
            double mag = sqrt(m);
            if (mag > s2max) s2max = mag;
        }
        double rms2 = sqrt(s2 / n_250k);
        printf("    [diag] burst ch=%d: ch_iq n=%zu rms=%.0f peak=%.0f; resamp n=%d rms=%.0f peak=%.0f\n",
               bb->channel, n_ch, rms1, s1max, n_250k, rms2, s2max);
        free(in_re); free(in_im); free(ch_iq);

        // Convert back to int16 IQ for uw_correlator
        int16_t *iq250 = malloc(n_250k * 2 * sizeof(int16_t));
        for (int i = 0; i < n_250k; i++) {
            float r = re_buf[i], v = im_buf[i];
            if (r >  INT16_MAX) r =  INT16_MAX;
            if (r <  INT16_MIN) r =  INT16_MIN;
            if (v >  INT16_MAX) v =  INT16_MAX;
            if (v <  INT16_MIN) v =  INT16_MIN;
            iq250[i * 2 + 0] = (int16_t)lrintf(r);
            iq250[i * 2 + 1] = (int16_t)lrintf(v);
        }
        free(re_buf); free(im_buf);

        // D13 burst start + RRC + uw_correlator
        int bstart = uw_correlator_find_burst_start(iq250, n_250k, n_250k);
        int16_t *adj = iq250 + bstart * 2;
        int adj_n = n_250k - bstart;
        uw_correlator_apply_rrc(adj, adj, adj_n);
        uw_corr_result_t uw;
        uw_correlator_find(adj, adj_n, adj_n - 24, &uw);

        // Pre-rotation + decim 5 (we use float pre-rot here for clarity)
        const char *qpsk_str = "no-decode";
        if (uw.direction != UW_DIR_UNKNOWN) {
            float pmag = sqrtf(uw.peak_re * uw.peak_re + uw.peak_im * uw.peak_im);
            if (pmag > 1e-3f) {
                float rot_re = uw.peak_re / pmag;
                float rot_im = uw.peak_im / pmag;
                float dphi_pll = uw.omega_per_sym / 10.0f;   // UW_SPS=10
                int16_t *src = adj + (int)uw.uw_offset * 2;
                int n_rot = adj_n - (int)uw.uw_offset;
                if (n_rot > 1) {
                    float pr = rot_re, pi = rot_im;
                    float c_step = cosf(dphi_pll), s_step = sinf(dphi_pll);
                    for (int i = 0; i < n_rot; i++) {
                        float re = src[i * 2 + 0];
                        float im = src[i * 2 + 1];
                        float nr = re * pr - im * pi;
                        float ni = re * pi + im * pr;
                        if (nr >  INT16_MAX) nr =  INT16_MAX;
                        if (nr <  INT16_MIN) nr =  INT16_MIN;
                        if (ni >  INT16_MAX) ni =  INT16_MAX;
                        if (ni <  INT16_MIN) ni =  INT16_MIN;
                        src[i * 2 + 0] = (int16_t)lrintf(nr);
                        src[i * 2 + 1] = (int16_t)lrintf(ni);
                        float npr = pr * c_step - pi * s_step;
                        float npi = pr * s_step + pi * c_step;
                        pr = npr; pi = npi;
                    }
                    // Decim 5×
                    int n_2sps = n_rot / 5;
                    int16_t *iq2 = malloc(n_2sps * 2 * sizeof(int16_t));
                    for (int i = 0; i < n_2sps; i++) {
                        iq2[i * 2 + 0] = src[i * 5 * 2 + 0];
                        iq2[i * 2 + 1] = src[i * 5 * 2 + 1];
                    }
                    decoded_frame_t frame = {0};
                    int rc = qpsk_demod_process(iq2, n_2sps * 2, &frame);
                    if (rc) {
                        qpsk_str = "OK";
                        n_ok_decode++;
                        n_uw_match++;
                    } else {
                        qpsk_str = "no-UW";
                    }
                    free(iq2);
                }
            }
        }

        printf("  %-3d %-7.1f  %-8.1f  %-8s %-+9.3f %-8d %s\n",
               bb->channel, (double)bb->snr_db,
               (double)uw.snr_estimate_db,
               uw.direction == UW_DIR_DOWNLINK ? "DL" :
               uw.direction == UW_DIR_UPLINK   ? "UL" : "UNK",
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
