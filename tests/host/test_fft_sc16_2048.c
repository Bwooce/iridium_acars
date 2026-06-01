// test_fft_sc16_2048.c — cross-validate the new Q15 2048-pt FFT against
// a textbook float reference. The Q15 FFT (fft_sc16_2048) is what the
// wideband fft_burst_tagger (Phase 3.6.M) will use on the P4; correctness
// is non-negotiable, performance comes after.
//
// Tests:
//   1. Pure-DC input → all energy in bin 0.
//   2. Single-bin complex exponential at bin K → all energy in bin K.
//   3. Two-tone input → energy at both expected bins.
//   4. Random input → matches a direct DFT to within Q15 precision.
//
// The Q15 FFT applies per-stage right-shift by 1, so output magnitudes
// are scaled by 1/N relative to a "no-normalisation" FFT. We compare
// against an explicitly-divided-by-N reference so the comparison is
// apples-to-apples.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <complex.h>

#include "fft_sc16_2048.h"

#define N FFT_SC16_2048_N

static int n_passed = 0;
static int n_failed = 0;

static void report(const char *name, int ok, const char *detail)
{
    if (ok) {
        printf("  [pass] %s\n", name);
        n_passed++;
    } else {
        printf("  [FAIL] %s: %s\n", name, detail);
        n_failed++;
    }
}

// Direct O(N²) DFT in float, normalised by 1/N to match the Q15 FFT's
// per-stage shift convention. Returns interleaved float re/im.
static void float_dft_norm(const float *in_re, const float *in_im,
                           float *out_re, float *out_im, int n)
{
    const double PI = 3.14159265358979323846;
    for (int k = 0; k < n; k++) {
        double sr = 0.0, si = 0.0;
        for (int j = 0; j < n; j++) {
            double ang = -2.0 * PI * (double)k * (double)j / (double)n;
            double c = cos(ang), s = sin(ang);
            sr += (double)in_re[j] * c - (double)in_im[j] * s;
            si += (double)in_re[j] * s + (double)in_im[j] * c;
        }
        out_re[k] = (float)(sr / (double)n);
        out_im[k] = (float)(si / (double)n);
    }
}

// --- Test 1: DC input ---
static void test_dc(void)
{
    int16_t buf[2 * N];
    for (int i = 0; i < N; i++) {
        buf[2 * i + 0] = 1000; // I = 1000 (Q15 ≈ 0.0305)
        buf[2 * i + 1] = 0;
    }
    fft_sc16_2048(buf);
    // After /N normalisation, output[0] should be ~1000 (re), 0 (im).
    // Other bins should be ~0 (within Q15 quantisation noise).
    int  ok          = 1;
    char detail[128] = "";
    if (abs((int)buf[0] - 1000) > 5) {
        snprintf(detail, sizeof(detail), "bin0.re=%d (expected ~1000)", buf[0]);
        ok = 0;
    }
    if (ok && abs((int)buf[1]) > 5) {
        snprintf(detail, sizeof(detail), "bin0.im=%d (expected ~0)", buf[1]);
        ok = 0;
    }
    // Check a few "should be zero" bins
    if (ok) {
        for (int k = 1; k < N; k += 100) {
            int re = buf[2 * k + 0], im = buf[2 * k + 1];
            int mag2 = re * re + im * im;
            if (mag2 > 100) {
                snprintf(detail, sizeof(detail),
                         "bin%d mag²=%d (expected ~0)", k, mag2);
                ok = 0;
                break;
            }
        }
    }
    report("DC input → energy in bin 0 only", ok, detail);
}

// --- Test 2: complex exponential at bin K ---
static void test_tone(int target_bin)
{
    int16_t      buf[2 * N];
    const double PI  = 3.14159265358979323846;
    const double AMP = 8000.0; // moderate Q15 amplitude
    for (int i = 0; i < N; i++) {
        double ang     = 2.0 * PI * (double)target_bin * (double)i / (double)N;
        buf[2 * i + 0] = (int16_t)(AMP * cos(ang));
        buf[2 * i + 1] = (int16_t)(AMP * sin(ang));
    }
    fft_sc16_2048(buf);
    // Peak magnitude should be at target_bin.
    int     peak_k    = 0;
    int32_t peak_mag2 = 0;
    for (int k = 0; k < N; k++) {
        int32_t re = buf[2 * k + 0], im = buf[2 * k + 1];
        int32_t m2 = re * re + im * im;
        if (m2 > peak_mag2) {
            peak_mag2 = m2;
            peak_k    = k;
        }
    }
    char detail[128];
    if (peak_k != target_bin) {
        snprintf(detail, sizeof(detail),
                 "peak at bin %d, expected %d", peak_k, target_bin);
        report("complex exp at bin K → peak at bin K", 0, detail);
        return;
    }
    // Peak magnitude should be ~AMP (within ~10% Q15 quantisation slack).
    int peak_mag = (int)sqrt((double)peak_mag2);
    int err      = abs(peak_mag - (int)AMP);
    if (err > (int)(AMP * 0.10)) {
        snprintf(detail, sizeof(detail),
                 "peak mag=%d, expected ~%d (err=%d)",
                 peak_mag, (int)AMP, err);
        report("complex exp peak magnitude correct", 0, detail);
        return;
    }
    snprintf(detail, sizeof(detail),
             "bin %d: |X|=%d (input AMP=%d)", peak_k, peak_mag, (int)AMP);
    report(detail, 1, "");
}

// --- Test 3: two-tone input ---
static void test_two_tone(int bin_a, int bin_b)
{
    int16_t      buf[2 * N];
    const double PI  = 3.14159265358979323846;
    const double AMP = 6000.0;
    for (int i = 0; i < N; i++) {
        double ang_a   = 2.0 * PI * (double)bin_a * (double)i / (double)N;
        double ang_b   = 2.0 * PI * (double)bin_b * (double)i / (double)N;
        double re      = AMP * (cos(ang_a) + cos(ang_b));
        double im      = AMP * (sin(ang_a) + sin(ang_b));
        buf[2 * i + 0] = (int16_t)re;
        buf[2 * i + 1] = (int16_t)im;
    }
    fft_sc16_2048(buf);
    // Find top 2 bins by magnitude
    int     top1 = -1, top2 = -1;
    int32_t m1 = -1, m2 = -1;
    for (int k = 0; k < N; k++) {
        int32_t re = buf[2 * k + 0], im = buf[2 * k + 1];
        int32_t mag2 = re * re + im * im;
        if (mag2 > m1) {
            m2   = m1;
            top2 = top1;
            m1   = mag2;
            top1 = k;
        } else if (mag2 > m2) {
            m2   = mag2;
            top2 = k;
        }
    }
    int  matched = (top1 == bin_a && top2 == bin_b) || (top1 == bin_b && top2 == bin_a);
    char detail[128];
    snprintf(detail, sizeof(detail), "top bins {%d, %d} expected {%d, %d}",
             top1, top2, bin_a, bin_b);
    report("two-tone → peaks at both expected bins", matched, detail);
}

// --- Test 4: random input vs direct DFT ---
static void test_random_vs_dft(void)
{
    int16_t buf[2 * N];
    float   in_re[N], in_im[N];
    float   ref_re[N], ref_im[N];

    srand(42);
    for (int i = 0; i < N; i++) {
        // Modest amplitude to avoid butterfly clipping
        int16_t r      = (int16_t)(rand() % 4001 - 2000);
        int16_t s      = (int16_t)(rand() % 4001 - 2000);
        buf[2 * i + 0] = r;
        buf[2 * i + 1] = s;
        in_re[i]       = (float)r;
        in_im[i]       = (float)s;
    }

    fft_sc16_2048(buf);
    float_dft_norm(in_re, in_im, ref_re, ref_im, N);

    // Compare per-bin. Q15 FFT has cumulative quantisation error from
    // log2(N) = 11 right-shift stages, plus twiddle quantisation. Expect
    // absolute error to be small-ish (single-digit Q15 units) per bin,
    // larger as a fraction at low-magnitude bins.
    double sum_err2 = 0, sum_ref2 = 0;
    int    max_err   = 0;
    int    max_err_k = 0;
    for (int k = 0; k < N; k++) {
        int q_re  = buf[2 * k + 0];
        int q_im  = buf[2 * k + 1];
        int ref_r = (int)lrintf(ref_re[k]);
        int ref_i = (int)lrintf(ref_im[k]);
        int de    = q_re - ref_r;
        int dim   = q_im - ref_i;
        int err2  = de * de + dim * dim;
        sum_err2 += err2;
        sum_ref2 += (double)ref_r * ref_r + (double)ref_i * ref_i;
        int e = (int)sqrt((double)err2);
        if (e > max_err) {
            max_err   = e;
            max_err_k = k;
        }
    }
    double nmse_db = 10.0 * log10(sum_err2 / sum_ref2);
    char   detail[200];
    // Threshold rationale: Q15 radix-2 FFT with per-stage right-shift
    // accumulates quantisation noise over log2(N) = 11 stages, giving
    // ~-30 to -35 dB SNR for random inputs at moderate amplitude
    // (input rms ~ 1200, output rms scales by 1/sqrt(N)). Per-bin
    // ABSOLUTE error of 1-2 LSB is the bit-correctness signal we
    // actually care about; NMSE is just a convenient one-number
    // summary. -30 dB matches the existing fft_sc16_64 + esp-dsp
    // dsps_fft2r_sc16_ansi reference behaviour at this rate.
    int ok = (nmse_db < -30.0) && (max_err <= 5);
    snprintf(detail, sizeof(detail),
             "NMSE=%.2f dB (limit -30 dB), max bin-err=%d at bin %d",
             nmse_db, max_err, max_err_k);
    report("random input vs direct DFT (NMSE)", ok, detail);
    if (ok) printf("    %s\n", detail);
}

int main(void)
{
    printf("Testing fft_sc16_2048 (N=%d)\n", N);
    fft_sc16_2048_init();

    test_dc();
    test_tone(1);
    test_tone(64);
    test_tone(N / 2 - 1);
    test_two_tone(10, 100);
    test_two_tone(50, N / 2 - 100);
    test_random_vs_dft();

    printf("\n%d passed, %d failed\n", n_passed, n_failed);
    return n_failed == 0 ? 0 : 1;
}
