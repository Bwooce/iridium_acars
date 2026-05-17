// Host test for uw_correlator's CFO (omega_per_sym) output.
//
// Cross-validates the gr-iridium-style square-then-FFT CFO estimator
// against synthetic bursts with known carrier offsets. If this test
// passes but live RF bursts produce nonsense omega values, the issue
// is signal-condition (low SNR, channel timing offset, etc.) — not
// the algorithm. If this test fails, the algorithm has a bug.

#include "uw_correlator.h"

#include <complex.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Iridium parameters. We synthesise at 10 sps (250 ksps) — matches
// the worker's pre-correlator state (uw_correlator hardcoded to SPS=10
// = UW_SPS = gr-iridium's burst_downmix internal rate).
#define FS_SYM       25000.0    // symbol rate
#define SPS          10         // samples per symbol (= UW_SPS)
#define FS_SAMP      (FS_SYM * SPS)

// UW patterns (must match IR_UW_DL / IR_UW_UL from iridium.h, mapped
// to BPSK ±(1+j) on the +1+j / -1-j axis).
static const int UW_DL_SIGN[12] = { +1,-1,-1,-1,-1,+1,+1,+1,-1,+1,+1,-1 };
static const int UW_UL_SIGN[12] = { -1,-1,+1,+1,+1,-1,+1,+1,-1,+1,-1,-1 };

static int s_passed = 0;
static int s_failed = 0;

#define CHECK_NEAR(actual, expected, tol, ...) do {              \
    double _a = (actual), _e = (expected), _t = (tol);           \
    double _d = _a - _e; if (_d < 0) _d = -_d;                   \
    if (_d > _t) {                                               \
        fprintf(stderr, "  FAIL line %d: %s = %.4f (expected %.4f ± %.4f)\n", \
                __LINE__, #actual, _a, _e, _t);                  \
        fprintf(stderr, "  ");                                   \
        fprintf(stderr, __VA_ARGS__);                            \
        fprintf(stderr, "\n");                                   \
        s_failed++;                                              \
    } else {                                                     \
        s_passed++;                                              \
    }                                                            \
} while (0)

// Direction selector for the synthetic burst.
typedef enum { DIR_DL = 0, DIR_UL = 1 } burst_dir_t;

// Build a synthetic 2-sps interleaved int16 burst:
//   - PREAMBLE_LEN syms (DL: all s0; UL: alternating s1,s0)
//   - 12 syms of UW pattern (direction-appropriate)
//   - TAIL_LEN syms of random data (placeholder for the data field)
// All modulated by a constant amplitude and a carrier offset of
// `omega_per_sym` rad/sym. Optional Gaussian noise with σ = sigma.
//
// burst[2i+0..2i+1] is sample i (interleaved I, Q).
// Returns total complex sample count (= PREAMBLE_LEN+12+TAIL_LEN syms × SPS).
static int build_burst_dir(int16_t *burst, int preamble_len, int tail_len,
                            double omega_per_sym, double amp, double sigma,
                            burst_dir_t dir)
{
    const int total_syms = preamble_len + 12 + tail_len;
    const int total_samps = total_syms * SPS;
    const double per_sample_phase = omega_per_sym / (double)SPS;
    const int *uw_signs = (dir == DIR_DL) ? UW_DL_SIGN : UW_UL_SIGN;

    for (int n = 0; n < total_samps; n++) {
        int sym = n / SPS;
        // pick the symbol's BPSK sign
        int sign;
        if (sym < preamble_len) {
            if (dir == DIR_DL) {
                sign = +1;          // DL preamble all s0
            } else {
                // UL preamble alternates s1, s0, s1, s0, ... starting s1.
                sign = (sym & 1) ? +1 : -1;
            }
        } else if (sym < preamble_len + 12) {
            sign = uw_signs[sym - preamble_len];
        } else {
            // simple repeatable random pattern for the data tail
            sign = ((sym * 1103515245u + 12345u) >> 30) & 1 ? +1 : -1;
        }

        // BPSK symbol on +1+j / -1-j axis
        double s_re = sign * amp;
        double s_im = sign * amp;

        // Apply carrier offset: multiply by exp(+j·per_sample_phase·n)
        double c = cos(per_sample_phase * (double)n);
        double s = sin(per_sample_phase * (double)n);
        double v_re = s_re * c - s_im * s;
        double v_im = s_re * s + s_im * c;

        // Add Gaussian noise (Box-Muller, simple and good enough).
        if (sigma > 0.0) {
            double u1 = (rand() + 1.0) / (RAND_MAX + 1.0);
            double u2 = (rand() + 1.0) / (RAND_MAX + 1.0);
            double r  = sqrt(-2.0 * log(u1));
            v_re += sigma * r * cos(2.0 * M_PI * u2);
            v_im += sigma * r * sin(2.0 * M_PI * u2);
        }

        // Saturate to int16
        if (v_re >  32767.0) v_re =  32767.0;
        if (v_re < -32768.0) v_re = -32768.0;
        if (v_im >  32767.0) v_im =  32767.0;
        if (v_im < -32768.0) v_im = -32768.0;
        burst[2 * n + 0] = (int16_t)v_re;
        burst[2 * n + 1] = (int16_t)v_im;
    }
    return total_samps;
}

// Back-compat wrapper: always DL.
static int build_burst(int16_t *burst, int preamble_len, int tail_len,
                       double omega_per_sym, double amp, double sigma)
{
    return build_burst_dir(burst, preamble_len, tail_len,
                           omega_per_sym, amp, sigma, DIR_DL);
}

// Apply the worker's gr-iridium-style pipeline to a burst:
//   1. coarse CFO via uw_correlator_estimate_cfo (square-then-FFT
//      over preamble+UW head)
//   2. pre-rotate by the linear phase ramp the estimate gives
//   3. uw_correlator_find for UW position/direction + residual omega
// Returns the COMBINED omega (coarse + residual) that the worker
// effectively removes from the burst.
static float run_correlator(int16_t *burst, int n_complex,
                             int *out_uw_offset, uw_direction_t *out_dir)
{
    float coarse_omega = uw_correlator_estimate_cfo(burst, n_complex);
    if (coarse_omega != 0.0f) {
        // Per-sample phase = omega / sps (omega is per SYMBOL).
        float dphi = coarse_omega / (float)SPS;
        double c_step = cos(dphi), s_step = sin(dphi);
        double pr = 1.0, pi = 0.0;
        for (int i = 0; i < n_complex; i++) {
            double re = burst[i * 2 + 0];
            double im = burst[i * 2 + 1];
            double nr = re * pr - im * pi;
            double ni = re * pi + im * pr;
            if (nr >  32767.0) nr =  32767.0;
            if (nr < -32768.0) nr = -32768.0;
            if (ni >  32767.0) ni =  32767.0;
            if (ni < -32768.0) ni = -32768.0;
            burst[i * 2 + 0] = (int16_t)nr;
            burst[i * 2 + 1] = (int16_t)ni;
            double npr = pr * c_step - pi * s_step;
            double npi = pr * s_step + pi * c_step;
            pr = npr; pi = npi;
        }
    }
    uw_corr_result_t res;
    // Need room for the full 28-sym sync × sps stride after the search
    // start. = SYNC_LENGTH × SPS samples.
    int search = n_complex - 28 * SPS;
    if (search < 1) search = 1;
    uw_correlator_find(burst, n_complex, search, &res);
    if (out_uw_offset) *out_uw_offset = res.uw_offset;
    if (out_dir) *out_dir = res.direction;
    return coarse_omega + res.omega_per_sym;
}

int main(void)
{
    const int PREAMBLE_LEN = 16;
    const int TAIL_LEN = 30;
    int16_t burst[(PREAMBLE_LEN + 12 + TAIL_LEN) * SPS * 2];
    const double AMP = 8000.0;     // ~quarter-scale int16 to leave headroom for offset

    printf("Test 1: zero CFO, no noise — expect omega ≈ 0\n");
    {
        int n = build_burst(burst, PREAMBLE_LEN, TAIL_LEN, 0.0, AMP, 0.0);
        int off; uw_direction_t dir;
        float omega = run_correlator(burst, n, &off, &dir);
        printf("  uw_offset=%d (expect %d) dir=%d omega=%.4f\n",
               off, PREAMBLE_LEN * SPS, dir, omega);
        CHECK_NEAR(omega, 0.0, 0.05, "zero CFO ground truth");
        // Tolerance = half a symbol (= sps/2 samples) — generous for
        // the matched filter's sub-sample precision.
        CHECK_NEAR(off, PREAMBLE_LEN * SPS, SPS / 2, "UW offset matches preamble length");
    }

    printf("\nTest 2: small positive CFO (+0.2 rad/sym), no noise\n");
    {
        int n = build_burst(burst, PREAMBLE_LEN, TAIL_LEN, 0.2, AMP, 0.0);
        int off; uw_direction_t dir;
        float omega = run_correlator(burst, n, &off, &dir);
        printf("  uw_offset=%d dir=%d omega=%.4f (expect ≈ -0.2 by worker convention)\n",
               off, dir, omega);
        CHECK_NEAR(omega, -0.2, 0.10, "+0.2 rad/sym CFO ground truth (worker expects negation)");
    }

    printf("\nTest 3: small negative CFO (-0.3 rad/sym), no noise\n");
    {
        int n = build_burst(burst, PREAMBLE_LEN, TAIL_LEN, -0.3, AMP, 0.0);
        int off; uw_direction_t dir;
        float omega = run_correlator(burst, n, &off, &dir);
        printf("  uw_offset=%d dir=%d omega=%.4f (expect ≈ +0.3)\n",
               off, dir, omega);
        CHECK_NEAR(omega, +0.3, 0.10, "-0.3 rad/sym CFO ground truth");
    }

    printf("\nTest 4: larger CFO (+0.5 rad/sym), no noise\n");
    {
        int n = build_burst(burst, PREAMBLE_LEN, TAIL_LEN, 0.5, AMP, 0.0);
        int off; uw_direction_t dir;
        float omega = run_correlator(burst, n, &off, &dir);
        printf("  uw_offset=%d dir=%d omega=%.4f (expect ≈ -0.5)\n",
               off, dir, omega);
        CHECK_NEAR(omega, -0.5, 0.15, "+0.5 rad/sym CFO ground truth");
    }

    printf("\nTest 5: moderate noise (σ=2000), small CFO (+0.2 rad/sym)\n");
    {
        srand(42);
        int n = build_burst(burst, PREAMBLE_LEN, TAIL_LEN, 0.2, AMP, 2000.0);
        int off; uw_direction_t dir;
        float omega = run_correlator(burst, n, &off, &dir);
        printf("  uw_offset=%d dir=%d omega=%.4f (expect ≈ -0.2)\n",
               off, dir, omega);
        CHECK_NEAR(omega, -0.2, 0.20, "+0.2 rad/sym CFO + noise");
    }

    // --- UL direction tests (gr-iridium alignment coverage) ---
    printf("\nTest 6: UL direction, zero CFO, no noise\n");
    {
        int n = build_burst_dir(burst, PREAMBLE_LEN, TAIL_LEN,
                                0.0, AMP, 0.0, DIR_UL);
        int off; uw_direction_t dir;
        float omega = run_correlator(burst, n, &off, &dir);
        printf("  uw_offset=%d dir=%d omega=%.4f\n", off, dir, omega);
        CHECK_NEAR(dir, (int)UW_DIR_UPLINK, 0, "UL direction detected");
        CHECK_NEAR(omega, 0.0, 0.05, "UL zero CFO ground truth");
        CHECK_NEAR(off, PREAMBLE_LEN * SPS, SPS / 2, "UL UW offset matches preamble length");
    }

    printf("\nTest 7: UL direction, +0.4 rad/sym CFO, no noise\n");
    {
        int n = build_burst_dir(burst, PREAMBLE_LEN, TAIL_LEN,
                                0.4, AMP, 0.0, DIR_UL);
        int off; uw_direction_t dir;
        float omega = run_correlator(burst, n, &off, &dir);
        printf("  uw_offset=%d dir=%d omega=%.4f (expect ≈ -0.4)\n",
               off, dir, omega);
        CHECK_NEAR(dir, (int)UW_DIR_UPLINK, 0, "UL direction with CFO");
        CHECK_NEAR(omega, -0.4, 0.15, "UL +0.4 rad/sym CFO ground truth");
    }

    // --- Low-SNR sweep on DL ---
    printf("\nTest 8: low SNR sweep (σ ∈ {2000, 4000, 6000})\n");
    {
        const double sigmas[] = { 2000.0, 4000.0, 6000.0 };
        for (int s = 0; s < 3; s++) {
            srand(0xc0fee + s);
            int n = build_burst_dir(burst, PREAMBLE_LEN, TAIL_LEN,
                                    0.1, AMP, sigmas[s], DIR_DL);
            int off; uw_direction_t dir;
            float omega = run_correlator(burst, n, &off, &dir);
            printf("  σ=%.0f: uw_offset=%d dir=%d omega=%.4f\n",
                   sigmas[s], off, dir, omega);
            // At AMP=8000, sigma=6000 is roughly SNR ≈ 2.5 dB per sample.
            // Direction detection should still work; CFO may drift.
            CHECK_NEAR(dir, (int)UW_DIR_DOWNLINK, 0,
                       "direction at sigma=%.0f", sigmas[s]);
        }
    }

    // --- Mixed: large CFO + noise ---
    printf("\nTest 9: large CFO (+0.6) + moderate noise (σ=3000), DL\n");
    {
        srand(0xbeef);
        int n = build_burst_dir(burst, PREAMBLE_LEN, TAIL_LEN,
                                0.6, AMP, 3000.0, DIR_DL);
        int off; uw_direction_t dir;
        float omega = run_correlator(burst, n, &off, &dir);
        printf("  uw_offset=%d dir=%d omega=%.4f (expect ≈ -0.6)\n",
               off, dir, omega);
        CHECK_NEAR(dir, (int)UW_DIR_DOWNLINK, 0, "DL at large CFO + noise");
        CHECK_NEAR(omega, -0.6, 0.25, "+0.6 rad/sym CFO + noise");
    }

    printf("\n=== %d passed, %d failed ===\n", s_passed, s_failed);
    return s_failed > 0 ? 1 : 0;
}
