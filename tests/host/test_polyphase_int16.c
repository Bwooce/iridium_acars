// Cross-validation: int16/Q14 polyphase channelizer vs float reference.
//
// The float path (polyphase_channelizer_process) is the well-tested
// reference. The int16 path (polyphase_channelizer_process_int16) is
// the hot path for on-target operation — on P4 it dispatches to the
// hand-rolled PIE asm in polyphase_mac_arp4.S; on host it uses a
// scalar C reference with the same memory layout and tap order.
//
// This test verifies the int16 path produces results that agree with
// the float path on detectable signals (within Q14 quantisation noise,
// ~ -84 dB) and that noise-floor channels stay near zero in both. A
// silent miscompile in the asm — e.g. wrong shift, wrong accumulator
// width, off-by-one stride — would land here, before it can degrade
// system-level burst detection below the threshold without warning.
//
// Tolerance rationale (excerpt from task spec):
//   - Q14 taps + 8-tap MAC ≈ -84 dB SNR quantisation noise.
//   - Iridium SNRs are 12-22 dB → channel-power agreement should be
//     within a few percent on the strong channel(s).
//   - Off-channel (noise-floor) bins: both paths should be near zero;
//     exact bit equivalence not expected, only that neither path
//     blows up.
//
// Test cases (see TASK / spec):
//   1. DC input.
//   2. Tone at +250 kHz (non-bin-aligned to exercise off-bin spread).
//   3. Short full-scale burst followed by silence.
//   4. Long PRBS-driven random IQ noise (deterministic).
//
// Both paths get separate channelizer instances. Inputs are the same
// signal converted appropriately (float[-1,1] vs int16 [-32768, +32767]).
//
// AGENTS: keep this file small and standalone — same micro-framework
// as test_polyphase_channelizer.c.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <complex.h>
#include "polyphase_channelizer.h"

static int passed = 0, failed = 0;

#define CHECK(cond, fmt, ...) do {                                       \
    if (!(cond)) {                                                       \
        printf("  FAIL line %d: " fmt "\n", __LINE__, ##__VA_ARGS__);    \
        failed++; return;                                                \
    } else { passed++; }                                                 \
} while (0)

#define FS_IN  2560000u
#define M      POLYCHAN_M    // 64
#define SKIP_ROWS 16         // post-transient skip

// Helper: per-channel mean magnitude² over rows [skip, n_rows).
static void mean_power_float(const float complex *out,
                             int n_rows, int skip,
                             double mag2[M])
{
    int count = n_rows - skip;
    for (int k = 0; k < M; k++) mag2[k] = 0.0;
    for (int row = skip; row < n_rows; row++) {
        for (int k = 0; k < M; k++) {
            float complex y = out[row * M + k];
            mag2[k] += (double)(crealf(y) * crealf(y) + cimagf(y) * cimagf(y));
        }
    }
    for (int k = 0; k < M; k++) mag2[k] /= count;
}

static void mean_power_int16(const int16_t *out_iq,
                             int n_rows, int skip,
                             double mag2[M])
{
    int count = n_rows - skip;
    for (int k = 0; k < M; k++) mag2[k] = 0.0;
    for (int row = skip; row < n_rows; row++) {
        for (int k = 0; k < M; k++) {
            double re = (double)out_iq[(row * M + k) * 2 + 0];
            double im = (double)out_iq[(row * M + k) * 2 + 1];
            mag2[k] += re * re + im * im;
        }
    }
    for (int k = 0; k < M; k++) mag2[k] /= count;
}

static int argmax_double(const double *v, int n)
{
    int best = 0;
    for (int i = 1; i < n; i++) if (v[i] > v[best]) best = i;
    return best;
}

// ---------- Test 1: DC input ----------
//
// Real-valued constant at ~full scale → all energy in channel 0 (DC),
// every other channel should be ~zero. Compare channel-0 magnitude
// between the two paths within ±5%, and check other channels are
// orders of magnitude smaller.

#define N_INPUT_DC  (1024 * M)

static void test_dc_input(void)
{
    printf("Test 1: DC input (float vs int16)\n");

    // Input scale: 0.5 in float, 16384 in int16 (= 0.5 * 32768).
    // Stays away from saturation but exercises a real per-phase MAC.
    const float    dc_float = 0.5f;
    const int16_t  dc_int16 = 16384;

    polyphase_channelizer_t *chf = polyphase_channelizer_create(FS_IN);
    polyphase_channelizer_t *chi = polyphase_channelizer_create(FS_IN);
    CHECK(chf && chi, "create");

    float complex *in_f   = malloc(N_INPUT_DC * sizeof(float complex));
    int16_t       *in_i   = malloc(N_INPUT_DC * 2 * sizeof(int16_t));
    float complex *out_f  = malloc(N_INPUT_DC * sizeof(float complex));
    int16_t       *out_i  = malloc(N_INPUT_DC * 2 * sizeof(int16_t));
    CHECK(in_f && in_i && out_f && out_i, "malloc");

    for (size_t i = 0; i < N_INPUT_DC; i++) {
        in_f[i] = dc_float + 0.0f * I;
        in_i[i * 2 + 0] = dc_int16;
        in_i[i * 2 + 1] = 0;
    }

    size_t nrf = polyphase_channelizer_process(chf, in_f, N_INPUT_DC, out_f);
    size_t nri = polyphase_channelizer_process_int16(chi, in_i, N_INPUT_DC, out_i);
    CHECK(nrf == nri, "row count mismatch %zu vs %zu", nrf, nri);
    CHECK(nrf > SKIP_ROWS + 100, "not enough rows");

    double pf[M], pi[M];
    mean_power_float(out_f, (int)nrf, SKIP_ROWS, pf);
    mean_power_int16(out_i, (int)nri, SKIP_ROWS, pi);

    int kf = argmax_double(pf, M);
    int ki = argmax_double(pi, M);
    printf("    float peak ch %d (p=%.3e)   int16 peak ch %d (p=%.3e)\n",
           kf, pf[kf], ki, pi[ki]);
    CHECK(kf == 0, "float DC didn't land in ch 0 (got %d)", kf);
    CHECK(ki == 0, "int16 DC didn't land in ch 0 (got %d)", ki);

    // Ratio comparison: int16 scale = float * 32768, so power scale =
    // 32768² ≈ 1.07e9. Compare normalised magnitudes.
    double mag_f = sqrt(pf[0]);
    double mag_i = sqrt(pi[0]) / 32768.0;     // back to float-equivalent scale
    double ratio = mag_i / (mag_f + 1e-30);
    printf("    DC channel: float=%.4f, int16/32768=%.4f, ratio=%.4f\n",
           mag_f, mag_i, ratio);
    CHECK(fabs(ratio - 1.0) < 0.05,
          "DC channel magnitude ratio %.4f outside ±5%%", ratio);

    // Other channels: both paths should be << peak. Use a 40 dB
    // headroom check (loose — DC has nominally zero leakage but
    // float roundoff puts the noise floor ~ -120 dB; int16 quantisation
    // raises it to ~ -84 dB. 40 dB is conservative.)
    double max_other_f = 0.0, max_other_i = 0.0;
    int max_other_kf = -1, max_other_ki = -1;
    for (int k = 1; k < M; k++) {
        if (pf[k] > max_other_f) { max_other_f = pf[k]; max_other_kf = k; }
        if (pi[k] > max_other_i) { max_other_i = pi[k]; max_other_ki = k; }
    }
    double rej_f_db = 10.0 * log10(pf[0] / (max_other_f + 1e-60));
    double rej_i_db = 10.0 * log10(pi[0] / (max_other_i + 1e-60));
    printf("    other-ch rejection: float %.1f dB (ch %d), int16 %.1f dB (ch %d)\n",
           rej_f_db, max_other_kf, rej_i_db, max_other_ki);
    CHECK(rej_f_db > 40.0, "float other-ch rejection only %.1f dB", rej_f_db);
    CHECK(rej_i_db > 40.0, "int16 other-ch rejection only %.1f dB", rej_i_db);

    free(in_f); free(in_i); free(out_f); free(out_i);
    polyphase_channelizer_destroy(chf);
    polyphase_channelizer_destroy(chi);
}

// ---------- Test 2: tone at +250 kHz ----------
//
// 250 kHz / 40 kHz per ch = 6.25. Energy will sit between channels 6
// and 7. We don't enforce which is the dominant bin — only that BOTH
// paths agree on the channel of peak, and peak-magnitude ratio is
// within ±10%.

#define N_INPUT_TONE  (4096 * M)

static void test_tone_offset(void)
{
    printf("Test 2: tone at +250 kHz (off-bin) — float vs int16 channel-of-peak\n");

    polyphase_channelizer_t *chf = polyphase_channelizer_create(FS_IN);
    polyphase_channelizer_t *chi = polyphase_channelizer_create(FS_IN);
    CHECK(chf && chi, "create");

    float complex *in_f   = malloc(N_INPUT_TONE * sizeof(float complex));
    int16_t       *in_i   = malloc(N_INPUT_TONE * 2 * sizeof(int16_t));
    float complex *out_f  = malloc(N_INPUT_TONE * sizeof(float complex));
    int16_t       *out_i  = malloc(N_INPUT_TONE * 2 * sizeof(int16_t));
    CHECK(in_f && in_i && out_f && out_i, "malloc");

    // Construct the tone at half scale so we don't risk Q15→Q14 saturation
    // anywhere downstream (the per-phase MAC has 1 bit of headroom by
    // design but the input itself stays at 0.5 to be conservative).
    const double freq_hz = 250000.0;
    const double dphase  = 2.0 * M_PI * freq_hz / (double)FS_IN;
    double phase = 0.0;
    for (size_t i = 0; i < N_INPUT_TONE; i++) {
        double re = 0.5 * cos(phase);
        double im = 0.5 * sin(phase);
        in_f[i] = (float)re + (float)im * I;
        in_i[i * 2 + 0] = (int16_t)lrint(re * 32768.0);
        in_i[i * 2 + 1] = (int16_t)lrint(im * 32768.0);
        phase += dphase;
    }

    size_t nrf = polyphase_channelizer_process(chf, in_f, N_INPUT_TONE, out_f);
    size_t nri = polyphase_channelizer_process_int16(chi, in_i, N_INPUT_TONE, out_i);
    CHECK(nrf == nri, "row count");

    double pf[M], pi[M];
    mean_power_float(out_f, (int)nrf, SKIP_ROWS, pf);
    mean_power_int16(out_i, (int)nri, SKIP_ROWS, pi);

    int kf = argmax_double(pf, M);
    int ki = argmax_double(pi, M);
    printf("    float peak ch %d (p=%.3e), int16 peak ch %d (p=%.3e)\n",
           kf, pf[kf], ki, pi[ki]);
    // Energy spreads across 6 and 7 because 250 kHz / 40 kHz = 6.25.
    CHECK(kf == 6 || kf == 7, "float peak not in {6,7}: got %d", kf);
    CHECK(ki == 6 || ki == 7, "int16 peak not in {6,7}: got %d", ki);
    CHECK(kf == ki, "float and int16 disagree on peak ch (%d vs %d)", kf, ki);

    // Compare summed power over the {6,7} pair so the off-bin spread
    // doesn't trip the ratio test.
    double sum_f = pf[6] + pf[7];
    double sum_i = pi[6] + pi[7];
    double mag_f = sqrt(sum_f);
    double mag_i = sqrt(sum_i) / 32768.0;
    double ratio = mag_i / (mag_f + 1e-30);
    printf("    summed ch{6,7}: float=%.4f, int16/32768=%.4f, ratio=%.4f\n",
           mag_f, mag_i, ratio);
    CHECK(fabs(ratio - 1.0) < 0.10,
          "tone magnitude ratio %.4f outside ±10%%", ratio);

    free(in_f); free(in_i); free(out_f); free(out_i);
    polyphase_channelizer_destroy(chf);
    polyphase_channelizer_destroy(chi);
}

// ---------- Test 3: short burst at full scale ----------
//
// 64 cycles of high-amplitude tone @ DC followed by 64 cycles of silence.
// Both paths should see the burst envelope on ch 0 with similar timing
// and shape. We compare per-row ch-0 magnitudes during the burst window.

#define N_BURST_ON   64
#define N_BURST_OFF  64
#define N_INPUT_BURST  ((N_BURST_ON + N_BURST_OFF) * M)

static void test_burst_envelope(void)
{
    printf("Test 3: burst envelope on ch 0 (full scale on, then silence)\n");

    polyphase_channelizer_t *chf = polyphase_channelizer_create(FS_IN);
    polyphase_channelizer_t *chi = polyphase_channelizer_create(FS_IN);
    CHECK(chf && chi, "create");

    float complex *in_f   = malloc(N_INPUT_BURST * sizeof(float complex));
    int16_t       *in_i   = malloc(N_INPUT_BURST * 2 * sizeof(int16_t));
    float complex *out_f  = malloc(N_INPUT_BURST * sizeof(float complex));
    int16_t       *out_i  = malloc(N_INPUT_BURST * 2 * sizeof(int16_t));
    CHECK(in_f && in_i && out_f && out_i, "malloc");

    // 0.75 amplitude during burst — high but not full scale, to avoid
    // Q14 MAC sum sitting at the saturation rail for ch 0.
    const double amp = 0.75;
    size_t burst_end = (size_t)N_BURST_ON * M;
    for (size_t i = 0; i < N_INPUT_BURST; i++) {
        double re = (i < burst_end) ? amp : 0.0;
        in_f[i] = (float)re + 0.0f * I;
        in_i[i * 2 + 0] = (int16_t)lrint(re * 32768.0);
        in_i[i * 2 + 1] = 0;
    }

    size_t nrf = polyphase_channelizer_process(chf, in_f, N_INPUT_BURST, out_f);
    size_t nri = polyphase_channelizer_process_int16(chi, in_i, N_INPUT_BURST, out_i);
    CHECK(nrf == nri, "row count");

    // Per-row ch-0 magnitude (float scale).
    int center_f = -1, center_i = -1;
    double peak_f = 0.0, peak_i = 0.0;
    for (size_t row = 0; row < nrf; row++) {
        float complex yf = out_f[row * M + 0];
        double mag_f = sqrt((double)(crealf(yf) * crealf(yf) +
                                     cimagf(yf) * cimagf(yf)));
        double rei = (double)out_i[(row * M + 0) * 2 + 0];
        double imi = (double)out_i[(row * M + 0) * 2 + 1];
        double mag_i = sqrt(rei * rei + imi * imi) / 32768.0;
        if (mag_f > peak_f) { peak_f = mag_f; center_f = (int)row; }
        if (mag_i > peak_i) { peak_i = mag_i; center_i = (int)row; }
    }
    printf("    float peak row %d mag %.4f, int16 peak row %d mag %.4f\n",
           center_f, peak_f, center_i, peak_i);
    CHECK(abs(center_f - center_i) <= 1,
          "burst peak timing mismatch: float row %d vs int16 row %d",
          center_f, center_i);
    double ratio = peak_i / (peak_f + 1e-30);
    CHECK(fabs(ratio - 1.0) < 0.10,
          "burst peak magnitude ratio %.4f outside ±10%%", ratio);

    // Silence region (last 32 rows): both should be ~zero compared to peak.
    double silence_f_max = 0.0, silence_i_max = 0.0;
    for (size_t row = nrf - 32; row < nrf; row++) {
        float complex yf = out_f[row * M + 0];
        double mag_f = sqrt((double)(crealf(yf) * crealf(yf) +
                                     cimagf(yf) * cimagf(yf)));
        double rei = (double)out_i[(row * M + 0) * 2 + 0];
        double imi = (double)out_i[(row * M + 0) * 2 + 1];
        double mag_i = sqrt(rei * rei + imi * imi) / 32768.0;
        if (mag_f > silence_f_max) silence_f_max = mag_f;
        if (mag_i > silence_i_max) silence_i_max = mag_i;
    }
    double sil_rej_f = 20.0 * log10(peak_f / (silence_f_max + 1e-30));
    double sil_rej_i = 20.0 * log10(peak_i / (silence_i_max + 1e-30));
    printf("    silence rejection: float %.1f dB, int16 %.1f dB\n",
           sil_rej_f, sil_rej_i);
    CHECK(sil_rej_f > 40.0, "float silence floor too high (%.1f dB)", sil_rej_f);
    CHECK(sil_rej_i > 40.0, "int16 silence floor too high (%.1f dB)", sil_rej_i);

    free(in_f); free(in_i); free(out_f); free(out_i);
    polyphase_channelizer_destroy(chf);
    polyphase_channelizer_destroy(chi);
}

// ---------- Test 4: long PRBS random IQ stability ----------
//
// Feed a deterministic PRBS-driven complex noise stream through both
// paths and confirm:
//   a) Neither blows up (no NaN/Inf for float; no all-saturated for int16).
//   b) Per-channel total powers agree within a factor (Q14 noise floor
//      adds ~ -84 dB but with random noise the per-channel power is
//      effectively flat — small relative shifts are OK).
//
// This catches gross errors (off-by-N stride, wrong head index, bad
// tap order) that would cause the int16 path to diverge unboundedly
// from the float path.

#define N_INPUT_LONG  (8192 * M)

// 32-bit xorshift — deterministic and fast.
static uint32_t prbs_state = 0xDEADBEEF;
static uint32_t prbs_next(void)
{
    uint32_t x = prbs_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    prbs_state = x;
    return x;
}

static void test_random_noise_stability(void)
{
    printf("Test 4: long random IQ stability (%d samples)\n", N_INPUT_LONG);

    polyphase_channelizer_t *chf = polyphase_channelizer_create(FS_IN);
    polyphase_channelizer_t *chi = polyphase_channelizer_create(FS_IN);
    CHECK(chf && chi, "create");

    float complex *in_f   = malloc(N_INPUT_LONG * sizeof(float complex));
    int16_t       *in_i   = malloc(N_INPUT_LONG * 2 * sizeof(int16_t));
    float complex *out_f  = malloc(N_INPUT_LONG * sizeof(float complex));
    int16_t       *out_i  = malloc(N_INPUT_LONG * 2 * sizeof(int16_t));
    CHECK(in_f && in_i && out_f && out_i, "malloc");

    prbs_state = 0x13579BDF;
    for (size_t i = 0; i < N_INPUT_LONG; i++) {
        // Range each sample to ±0.25 (well below saturation) so the
        // post-MAC ch outputs don't hit Q15 saturation in the int16
        // path for any tail of the distribution. PRBS values are
        // mapped uniform in [-0.25, +0.25).
        int32_t ri = (int32_t)(prbs_next() & 0xFFFF) - 32768;   // [-32768, +32767]
        int32_t ii = (int32_t)(prbs_next() & 0xFFFF) - 32768;
        float re = (float)ri / 32768.0f * 0.25f;
        float im = (float)ii / 32768.0f * 0.25f;
        in_f[i] = re + im * I;
        in_i[i * 2 + 0] = (int16_t)lrintf(re * 32768.0f);
        in_i[i * 2 + 1] = (int16_t)lrintf(im * 32768.0f);
    }

    size_t nrf = polyphase_channelizer_process(chf, in_f, N_INPUT_LONG, out_f);
    size_t nri = polyphase_channelizer_process_int16(chi, in_i, N_INPUT_LONG, out_i);
    CHECK(nrf == nri, "row count");

    // (a) no NaN/Inf in float path
    int n_bad_f = 0;
    for (size_t row = SKIP_ROWS; row < nrf; row++) {
        for (int k = 0; k < M; k++) {
            float complex y = out_f[row * M + k];
            if (!isfinite(crealf(y)) || !isfinite(cimagf(y))) n_bad_f++;
        }
    }
    CHECK(n_bad_f == 0, "%d non-finite float outputs", n_bad_f);

    // (b) int16 path never saturates uniformly. Count saturations —
    // legitimate quantisation may push the occasional sample to the
    // rails, but a runaway path would saturate ~every sample.
    int n_sat = 0, n_total = 0;
    for (size_t row = SKIP_ROWS; row < nri; row++) {
        for (int k = 0; k < M; k++) {
            int16_t re = out_i[(row * M + k) * 2 + 0];
            int16_t im = out_i[(row * M + k) * 2 + 1];
            if (re == 32767 || re == -32768 || im == 32767 || im == -32768) {
                n_sat++;
            }
            n_total++;
        }
    }
    double sat_pct = 100.0 * (double)n_sat / (double)n_total;
    printf("    int16 saturation rate: %.3f%% (%d / %d)\n",
           sat_pct, n_sat, n_total);
    CHECK(sat_pct < 1.0, "int16 path saturated on %.2f%% of samples", sat_pct);

    // (c) per-channel total power comparison. With random input the
    // distribution is flat across channels (modulo channelizer
    // gain). Compare path-by-path: each int16 channel's power
    // should be within ±20% of the float channel's, after scale.
    // 20% is loose because random noise has its own variance per
    // channel; tighter bound would flag false positives.
    double pf[M], pi[M];
    mean_power_float(out_f, (int)nrf, SKIP_ROWS, pf);
    mean_power_int16(out_i, (int)nri, SKIP_ROWS, pi);

    int n_bad_ch = 0;
    double max_dev = 0.0;
    int max_dev_k = -1;
    for (int k = 0; k < M; k++) {
        // Compare magnitudes in float scale (sqrt(power) / 32768 for int16).
        double mag_f = sqrt(pf[k]);
        double mag_i = sqrt(pi[k]) / 32768.0;
        double ratio = mag_i / (mag_f + 1e-30);
        double dev = fabs(ratio - 1.0);
        if (dev > max_dev) { max_dev = dev; max_dev_k = k; }
        if (dev > 0.20) n_bad_ch++;
    }
    printf("    per-channel mag ratio worst-case: %.3f at ch %d (%d/%d > 20%%)\n",
           1.0 + max_dev, max_dev_k, n_bad_ch, M);
    CHECK(n_bad_ch == 0,
          "%d channels deviate >20%% between float and int16 paths",
          n_bad_ch);

    free(in_f); free(in_i); free(out_f); free(out_i);
    polyphase_channelizer_destroy(chf);
    polyphase_channelizer_destroy(chi);
}

int main(void)
{
    test_dc_input();
    test_tone_offset();
    test_burst_envelope();
    test_random_noise_stability();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
