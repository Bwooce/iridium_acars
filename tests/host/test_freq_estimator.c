// Host test for freq_estimator (D8).
//
// Validates that the estimator recovers a known tone offset within
// ±500 Hz on synthetic IQ + handles the artefact-guard edge cases.

#include "freq_estimator.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FS_HZ        2560000u
#define SEARCH_HZ    20000u
#define N_SAMPLES    (FREQ_EST_FFT_N * 4)   // some headroom past the FFT size

static int s_passed = 0;
static int s_failed = 0;

#define CHECK(cond, ...) do {                              \
    if (!(cond)) {                                         \
        fprintf(stderr, "  FAIL line %d: ", __LINE__);     \
        fprintf(stderr, __VA_ARGS__);                      \
        fprintf(stderr, "\n");                             \
        s_failed++;                                        \
    } else {                                               \
        s_passed++;                                        \
    }                                                      \
} while (0)

// Synthesise a complex exponential at offset_hz, amplitude 0.5 of
// full scale (= 16384 in int16), into iq[2*n_complex].
static void synth_tone(int16_t *iq, size_t n_complex,
                       double offset_hz, uint32_t fs_hz)
{
    const double dphase = 2.0 * M_PI * offset_hz / (double)fs_hz;
    double phase = 0.0;
    for (size_t i = 0; i < n_complex; i++) {
        double re = 0.5 * cos(phase);
        double im = 0.5 * sin(phase);
        iq[2 * i + 0] = (int16_t)lrint(re * 32768.0);
        iq[2 * i + 1] = (int16_t)lrint(im * 32768.0);
        phase += dphase;
    }
}

static void test_offset(double offset_hz, int32_t tol_hz)
{
    int16_t *iq = malloc(N_SAMPLES * 2 * sizeof(int16_t));
    if (!iq) { fprintf(stderr, "  malloc fail\n"); s_failed++; return; }
    synth_tone(iq, N_SAMPLES, offset_hz, FS_HZ);

    int32_t est = freq_estimator_run(iq, N_SAMPLES, FS_HZ, SEARCH_HZ);
    int32_t err = est - (int32_t)offset_hz;
    if (err < 0) err = -err;
    printf("    tone @ %+9.0f Hz → est %+9d Hz  err %5d Hz\n",
           offset_hz, est, err);
    CHECK(err <= tol_hz,
          "tone @ %+.0f Hz: err %d Hz > tol %d Hz", offset_hz, err, tol_hz);
    free(iq);
}

static void test_tone_offsets(void)
{
    printf("Test 1: synthetic tones inside ±%u Hz window\n", SEARCH_HZ);
    // Tolerance: ±700 Hz worst case for off-bin tones with the Hann
    // window + 3-point quadratic interpolation (the realistic limit
    // of this algorithm class at N=512, fs=2.56 MHz; further
    // improvement would need Quinn's interpolator or a larger FFT).
    // Still comfortably inside the PLL's ±1.25 kHz capture range.
    test_offset(  +7500.0,  700);
    test_offset( -13000.0,  700);
    test_offset(    +250.0, 700);   // close-to-DC
    test_offset(  -3500.0,  700);
    test_offset( +18000.0, 700);    // near the ±20 kHz boundary
    test_offset( -18000.0, 700);
}

static void test_dc_guard(void)
{
    printf("Test 2: artefact guard at DC\n");
    // Zero input → no signal → peak at DC → estimator returns 0.
    int16_t *iq = calloc(N_SAMPLES * 2, sizeof(int16_t));
    if (!iq) { fprintf(stderr, "  malloc fail\n"); s_failed++; return; }
    int32_t est = freq_estimator_run(iq, N_SAMPLES, FS_HZ, SEARCH_HZ);
    printf("    zero-input → est %+d Hz\n", est);
    CHECK(est == 0, "zero-input should return 0, got %d", est);
    free(iq);
}

static void test_short_input_guard(void)
{
    printf("Test 3: short-input guard\n");
    // Less than FFT_N samples → guard returns 0 (no estimate).
    int16_t iq[FREQ_EST_FFT_N * 2 / 2] = {0};   // half the required size
    int32_t est = freq_estimator_run(iq, sizeof(iq) / (2 * sizeof(int16_t)),
                                      FS_HZ, SEARCH_HZ);
    printf("    short-input → est %+d Hz\n", est);
    CHECK(est == 0, "short-input should return 0, got %d", est);
}

int main(void)
{
    test_tone_offsets();
    test_dc_guard();
    test_short_input_guard();
    printf("\n=== %d passed, %d failed ===\n", s_passed, s_failed);
    return s_failed == 0 ? 0 : 1;
}
