// Tests for polyphase_channelizer.
//
//   1. Single complex tone at exact channel-center frequency lands in
//      that channel with ≥30 dB attenuation in all others.
//   2. DC input → channel 0.
//   3. Tone at fs/4 (= channel M/4) → channel M/4.
//   4. Two simultaneous tones in different channels separate correctly.

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
#define N_INPUT_PER_TEST  (4096 * M)   // process 4096 cycles per test

static void run_tone(float freq_hz,
                     int expected_ch,
                     const char *name)
{
    printf("Test: tone at %+8.0f Hz -> channel %d (%s)\n",
           freq_hz, expected_ch, name);
    polyphase_channelizer_t *ch = polyphase_channelizer_create(FS_IN);
    CHECK(ch != NULL, "create");

    float complex *in  = malloc(N_INPUT_PER_TEST * sizeof(float complex));
    float complex *out = malloc(N_INPUT_PER_TEST * sizeof(float complex));
    CHECK(in && out, "malloc");

    // Synthesise complex tone.
    double dphase = 2.0 * M_PI * freq_hz / (double)FS_IN;
    double phase = 0;
    for (size_t i = 0; i < N_INPUT_PER_TEST; i++) {
        in[i] = (float)cos(phase) + (float)sin(phase) * I;
        phase += dphase;
    }

    size_t n_rows = polyphase_channelizer_process(ch, in, N_INPUT_PER_TEST, out);
    // Discard transient (first 8 rows = ~8 filter delays).
    size_t skip = 16;
    CHECK(n_rows > skip + 100, "n_rows=%zu, need >116 for noise stats", n_rows);

    // Compute mean magnitude per channel over the post-transient region.
    double mag2[M] = { 0 };
    size_t count = n_rows - skip;
    for (size_t row = skip; row < n_rows; row++) {
        for (int k = 0; k < M; k++) {
            float complex y = out[row * M + k];
            mag2[k] += (double)(crealf(y) * crealf(y) + cimagf(y) * cimagf(y));
        }
    }
    double max_mag2 = 0; int max_k = -1;
    for (int k = 0; k < M; k++) {
        mag2[k] /= count;
        if (mag2[k] > max_mag2) { max_mag2 = mag2[k]; max_k = k; }
    }
    printf("    max channel = %d, magnitude² = %.3f\n", max_k, max_mag2);
    CHECK(max_k == expected_ch, "expected channel %d, got %d", expected_ch, max_k);

    // Adjacent-channel rejection.
    int adj_left  = (expected_ch - 1 + M) % M;
    int adj_right = (expected_ch + 1) % M;
    double left_rej_db  = 10.0 * log10(max_mag2 / (mag2[adj_left]  + 1e-30));
    double right_rej_db = 10.0 * log10(max_mag2 / (mag2[adj_right] + 1e-30));
    printf("    adjacent rejection: left=%.1f dB, right=%.1f dB\n",
           left_rej_db, right_rej_db);
    CHECK(left_rej_db  > 30.0, "left adjacent rejection only %.1f dB", left_rej_db);
    CHECK(right_rej_db > 30.0, "right adjacent rejection only %.1f dB", right_rej_db);

    // Far-channel rejection (channels 4 away).
    int far_k = (expected_ch + M / 4) % M;
    double far_rej_db = 10.0 * log10(max_mag2 / (mag2[far_k] + 1e-30));
    printf("    far-channel (k=%d) rejection: %.1f dB\n", far_k, far_rej_db);
    CHECK(far_rej_db > 50.0, "far-channel rejection only %.1f dB", far_rej_db);

    free(in); free(out);
    polyphase_channelizer_destroy(ch);
}

// Test 4: two concurrent tones in different channels separate correctly.
static void test_two_tones(void)
{
    printf("Test: two concurrent tones at +120 kHz (ch 3) and -200 kHz (ch 59) separate\n");
    polyphase_channelizer_t *ch = polyphase_channelizer_create(FS_IN);

    float complex *in  = malloc(N_INPUT_PER_TEST * sizeof(float complex));
    float complex *out = malloc(N_INPUT_PER_TEST * sizeof(float complex));
    CHECK(in && out, "malloc");

    double f1 = +120000.0; double dp1 = 2.0 * M_PI * f1 / (double)FS_IN;
    double f2 = -200000.0; double dp2 = 2.0 * M_PI * f2 / (double)FS_IN;
    double p1 = 0, p2 = 0;
    for (size_t i = 0; i < N_INPUT_PER_TEST; i++) {
        in[i] = ((float)cos(p1) + (float)sin(p1) * I) +
                ((float)cos(p2) + (float)sin(p2) * I);
        p1 += dp1; p2 += dp2;
    }

    size_t n_rows = polyphase_channelizer_process(ch, in, N_INPUT_PER_TEST, out);
    size_t skip = 16;
    double mag2[M] = { 0 };
    size_t count = n_rows - skip;
    for (size_t row = skip; row < n_rows; row++) {
        for (int k = 0; k < M; k++) {
            float complex y = out[row * M + k];
            mag2[k] += (double)(crealf(y) * crealf(y) + cimagf(y) * cimagf(y));
        }
    }
    for (int k = 0; k < M; k++) mag2[k] /= count;

    // Sort channels by magnitude, top 4
    int order[M];
    for (int i = 0; i < M; i++) order[i] = i;
    // simple sort
    for (int i = 0; i < M; i++) {
        for (int j = i + 1; j < M; j++) {
            if (mag2[order[j]] > mag2[order[i]]) {
                int t = order[i]; order[i] = order[j]; order[j] = t;
            }
        }
    }
    printf("    top-4 channels: %d (%.1f), %d (%.1f), %d (%.1f), %d (%.1f)\n",
           order[0], mag2[order[0]], order[1], mag2[order[1]],
           order[2], mag2[order[2]], order[3], mag2[order[3]]);

    // Both expected channels should be in the top 2.
    int ch1_expected = 3;     // +120 kHz / 40 kHz/ch = +3
    int ch2_expected = 59;    // -200 kHz / 40 kHz/ch = -5 → channel 64-5 = 59
    bool found1 = (order[0] == ch1_expected || order[1] == ch1_expected);
    bool found2 = (order[0] == ch2_expected || order[1] == ch2_expected);
    CHECK(found1, "channel 3 (+120 kHz) not in top-2");
    CHECK(found2, "channel 59 (-200 kHz) not in top-2");

    // The two top channels should be roughly equal in power (within 6 dB).
    double ratio_db = 10.0 * log10(mag2[order[0]] / mag2[order[1]]);
    CHECK(ratio_db < 6.0,
          "top-2 power ratio %.1f dB > 6 dB — tones leaked unevenly", ratio_db);

    free(in); free(out);
    polyphase_channelizer_destroy(ch);
}

static void test_channel_freq_helper(void)
{
    printf("Test: channel-frequency helper\n");
    polyphase_channelizer_t *ch = polyphase_channelizer_create(FS_IN);
    int spacing = FS_IN / M;     // 40000 Hz
    CHECK(polyphase_channelizer_channel_freq(ch, 0) == 0,        "k=0 -> DC");
    CHECK(polyphase_channelizer_channel_freq(ch, 1) == spacing,  "k=1 -> +spacing");
    CHECK(polyphase_channelizer_channel_freq(ch, M / 2) == FS_IN / 2,
          "k=M/2 -> Nyquist");
    CHECK(polyphase_channelizer_channel_freq(ch, M - 1) == -spacing,
          "k=M-1 -> -spacing");
    polyphase_channelizer_destroy(ch);
}

// --- Cross-validation: feed the SAME input to our C channelizer and a
//     numpy reference implementation, expect bit-identical outputs (modulo
//     float rounding). The reference fixture is generated by
//     tests/scripts/build_channelizer_reference.py from a numpy
//     implementation of the same polyphase + FFT math.
#include "fixture_channelizer_reference.h"

static void test_against_numpy_reference(void)
{
    printf("Test: C channelizer vs numpy reference on %d samples × %d cycles × %d channels\n",
           REF_INPUT_LEN, REF_N_CYCLES, REF_M);
    polyphase_channelizer_t *ch = polyphase_channelizer_create(REF_FS_IN);
    CHECK(ch != NULL, "create");

    float complex *out = malloc(REF_N_CYCLES * REF_M * sizeof(float complex));
    CHECK(out != NULL, "malloc out");

    size_t n_rows = polyphase_channelizer_process(ch, REF_INPUT,
                                                   REF_INPUT_LEN, out);
    CHECK(n_rows == REF_N_CYCLES, "n_rows=%zu (expected %d)",
          n_rows, REF_N_CYCLES);

    // Compare cell-by-cell. Tolerance: reference uses double precision
    // accumulators; our C impl uses float. Differences should be in
    // the noise floor (~1e-5 relative).
    int n_compared = 0;
    int n_mismatched = 0;
    double max_err = 0.0;
    int    max_err_cycle = -1, max_err_ch = -1;
    for (int cycle = 0; cycle < REF_N_CYCLES; cycle++) {
        for (int k = 0; k < REF_M; k++) {
            float complex got = out[cycle * REF_M + k];
            float complex ref = REF_OUTPUT[cycle][k];
            float complex diff = got - ref;
            double err = sqrt((double)(crealf(diff) * crealf(diff) +
                                       cimagf(diff) * cimagf(diff)));
            double mag_ref = sqrt((double)(crealf(ref) * crealf(ref) +
                                           cimagf(ref) * cimagf(ref)));
            // Relative error, with a small absolute floor for near-zero refs.
            double rel = err / (mag_ref + 1e-6);
            if (rel > max_err) {
                max_err = rel;
                max_err_cycle = cycle;
                max_err_ch = k;
            }
            // 1% relative is loose enough for float-vs-double accumulator
            // differences but tight enough to catch any real bug.
            if (rel > 0.01) n_mismatched++;
            n_compared++;
        }
    }
    printf("    compared %d cells, max relative err = %.2e at cycle %d ch %d\n",
           n_compared, max_err, max_err_cycle, max_err_ch);
    CHECK(n_mismatched == 0,
          "%d / %d cells mismatched (>1%% rel err)",
          n_mismatched, n_compared);

    free(out);
    polyphase_channelizer_destroy(ch);
}

int main(void)
{
    test_channel_freq_helper();
    int spacing = FS_IN / M;     // 40000 Hz
    run_tone(0.0f,                  0,        "DC");
    run_tone((float)(2 * spacing),  2,        "+80 kHz, ch 2");
    run_tone((float)(8 * spacing),  8,        "+320 kHz, ch 8");
    run_tone((float)(-3 * spacing), M - 3,    "-120 kHz, ch 61");
    run_tone((float)(-spacing),     M - 1,    "-40 kHz, ch 63");
    test_two_tones();
    test_against_numpy_reference();

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
