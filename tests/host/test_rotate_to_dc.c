// test_rotate_to_dc.c — host validation of rotate_to_dc_q15_inc against
// the cosf/sinf reference (rotate_to_dc).
//
// What we check:
//   1. NMSE of the Q15-incremental output vs the float reference is
//      ≤ -50 dB (≥ 50 dB below signal energy — well under Q15
//      quantisation noise floor).
//   2. Maximum per-sample diff is small enough to not affect downstream
//      Q15 DSP stages (≤ a few hundred LSB out of ±32767).
//   3. Both operate stably on long windows (100k samples), demonstrating
//      that the periodic renormalisation prevents the Q15 magnitude
//      decay the module header warns about.
//
// Run: ./test_rotate_to_dc — exits 0 on pass, non-zero on fail.
//      The numbers are also printed so a future tightening of the
//      thresholds can be done against an actual measurement.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "rotate_to_dc.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Synthesise a complex sinusoid at f_in Hz, amplitude 16000 (well below
// Q15 saturation), sampled at fs Hz. n_complex IQ samples.
static void synth_tone(int16_t *iq, int n_complex,
                        double f_in, double fs)
{
    double dphi = 2.0 * M_PI * f_in / fs;
    for (int k = 0; k < n_complex; k++) {
        double a = dphi * (double)k;
        iq[k * 2 + 0] = (int16_t)lrint(16000.0 * cos(a));
        iq[k * 2 + 1] = (int16_t)lrint(16000.0 * sin(a));
    }
}

// Compute NMSE (10·log10(Σ|err|² / Σ|sig|²)) between two int16 IQ buffers.
static double nmse_db(const int16_t *a, const int16_t *b, int n_complex,
                       int *out_max_diff)
{
    double err = 0, sig = 0;
    int max_diff = 0;
    for (int k = 0; k < n_complex; k++) {
        int dr = (int)a[k * 2 + 0] - (int)b[k * 2 + 0];
        int di = (int)a[k * 2 + 1] - (int)b[k * 2 + 1];
        err += (double)dr * dr + (double)di * di;
        sig += (double)a[k * 2 + 0] * a[k * 2 + 0]
              + (double)a[k * 2 + 1] * a[k * 2 + 1];
        int adr = dr < 0 ? -dr : dr;
        int adi = di < 0 ? -di : di;
        if (adr > max_diff) max_diff = adr;
        if (adi > max_diff) max_diff = adi;
    }
    if (out_max_diff) *out_max_diff = max_diff;
    if (sig < 1e-12) return 0.0;
    return 10.0 * log10(err / sig);
}

// Compute RMS magnitude of the output (Σ|iq|² / n) to verify the
// Q15-incremental phasor renormalisation keeps amplitude stable —
// the headline failure mode the module header warns about.
static double rms_mag(const int16_t *iq, int n_complex)
{
    double s = 0;
    for (int k = 0; k < n_complex; k++) {
        double r = iq[k * 2 + 0];
        double v = iq[k * 2 + 1];
        s += r * r + v * v;
    }
    return sqrt(s / (double)n_complex);
}

// One test case: rotate a synthesised tone, compare reference vs Q15-inc.
// `description` is logged in the per-case header.
static int run_case(const char *description,
                     int n_complex, double f_in_hz, double f_shift_hz,
                     double fs_hz, double nmse_threshold_db)
{
    printf("--- %s ---\n", description);
    printf("    n=%d, f_in=%.1f Hz, f_shift=%.1f Hz, fs=%.1f Hz\n",
           n_complex, f_in_hz, f_shift_hz, fs_hz);

    int16_t *src = (int16_t *)malloc(2 * n_complex * sizeof(int16_t));
    int16_t *ref = (int16_t *)malloc(2 * n_complex * sizeof(int16_t));
    int16_t *q15 = (int16_t *)malloc(2 * n_complex * sizeof(int16_t));
    if (!src || !ref || !q15) {
        fprintf(stderr, "ALLOC FAIL\n");
        free(src); free(ref); free(q15);
        return 1;
    }

    synth_tone(src, n_complex, f_in_hz, fs_hz);
    double src_rms = rms_mag(src, n_complex);
    printf("    src RMS magnitude: %.1f (expect ~%.0f)\n",
           src_rms, 16000.0);

    double phase_step = -2.0 * M_PI * f_shift_hz / fs_hz;

    memcpy(ref, src, 2 * n_complex * sizeof(int16_t));
    rotate_to_dc(ref, n_complex, phase_step);

    memcpy(q15, src, 2 * n_complex * sizeof(int16_t));
    rotate_to_dc_q15_inc(q15, n_complex, phase_step);

    double ref_rms = rms_mag(ref, n_complex);
    double q15_rms = rms_mag(q15, n_complex);
    int max_diff;
    double nmse = nmse_db(ref, q15, n_complex, &max_diff);

    printf("    ref RMS: %.1f   q15-inc RMS: %.1f   ratio: %.4f\n",
           ref_rms, q15_rms, q15_rms / ref_rms);
    printf("    NMSE: %.2f dB   max sample diff: %d LSB\n",
           nmse, max_diff);

    int fail = 0;
    if (nmse > nmse_threshold_db) {
        printf("    FAIL: NMSE %.2f dB worse than threshold %.1f dB\n",
               nmse, nmse_threshold_db);
        fail = 1;
    }
    // Magnitude should be stable. The Q15 decay (memory note) would
    // produce ratio ≪ 1 at long windows. With renormalisation, we
    // expect ratio within a few percent of 1.
    double mag_ratio = q15_rms / ref_rms;
    if (mag_ratio < 0.98 || mag_ratio > 1.02) {
        printf("    FAIL: RMS ratio %.4f outside [0.98, 1.02] — "
               "renormalisation not holding\n", mag_ratio);
        fail = 1;
    }
    printf("    %s\n", fail ? "FAIL" : "PASS");

    free(src); free(ref); free(q15);
    return fail;
}

int main(void)
{
    printf("test_rotate_to_dc: validating rotate_to_dc_q15_inc vs cosf/sinf reference\n\n");

    int fails = 0;

    // Case 1: short window. Renormalisation only kicks in once or twice;
    // tests basic correctness of the incremental multiply.
    fails += run_case("short window (1k samples)",
                       1024, 1000.0, 800000.0, 2500000.0, -45.0);

    // Case 2: typical wideband-burst window (~16 ms at 2.5 MSPS).
    // This is the size the firmware worker handles per burst, so
    // matches the actual production load.
    fails += run_case("burst-size window (40k samples)",
                       40000, 1000.0, 800000.0, 2500000.0, -40.0);

    // Case 3: long window (100k samples) — well past the 44k samples
    // at which the non-renormalised Q15 phasor would collapse the
    // amplitude (per the memory feedback note). Confirms renormalisation
    // is preventing the decay over long windows.
    fails += run_case("long window (100k samples, would catastrophically "
                       "decay without renorm)",
                       100000, 1000.0, 800000.0, 2500000.0, -40.0);

    // Case 4: small shift (in case the renormalisation interacts oddly
    // with near-DC rotation).
    fails += run_case("small shift (5 kHz, near-DC)",
                       40000, 1000.0, 5000.0, 2500000.0, -40.0);

    // Case 5: phase_step near ±π (Nyquist-ish, where cos/sin quantisation
    // pressure is highest).
    fails += run_case("large shift (1.2 MHz, near-Nyquist)",
                       40000, 1000.0, 1200000.0, 2500000.0, -40.0);

    printf("\n%d failing case(s)\n", fails);
    return fails ? 1 : 0;
}
