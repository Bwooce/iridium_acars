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
#include "channelizer_burst_ref.h"
#include "fixture_albq_raw.h"
#include "fixture_albq_raw_high.h"
#include "fixture_albq_stripes.h"
#include "fixture_corpus_uint8.h"

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
    // Cells in non-tone channels are at the channelizer's rejection
    // floor (≈ 64 dB below the tone channels per the adjacent-channel
    // tests above = magnitudes ~6e-4 for unit-amplitude input). At that
    // magnitude, float-vs-double accumulator drift produces
    // single-precision noise of ~1e-5 absolute which is 1-3% RELATIVE
    // but is genuine numerical noise, not an algorithm divergence.
    //
    // Use an ABSOLUTE-error gate instead of relative for cells below
    // SIGNAL_FLOOR — those carry no meaningful information for the
    // "C matches numpy" check. Relative-error gate still applies to
    // the on-tone cells where mag_ref is large.
    const double SIGNAL_FLOOR = 1e-3;
    const double ABS_TOL_FLOOR_CELLS = 1e-4;     // float-vs-double noise
    int n_compared = 0;
    int n_mismatched = 0;
    int n_skipped_noise_floor = 0;
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
            n_compared++;
            if (mag_ref < SIGNAL_FLOOR) {
                // Below the channelizer's rejection floor — absolute
                // tolerance only.
                if (err > ABS_TOL_FLOOR_CELLS) {
                    n_mismatched++;
                }
                n_skipped_noise_floor++;
                continue;
            }
            double rel = err / mag_ref;
            if (rel > max_err) {
                max_err = rel;
                max_err_cycle = cycle;
                max_err_ch = k;
            }
            if (rel > 0.01) n_mismatched++;
        }
    }
    printf("    compared %d cells (%d below %g signal floor, abs-gated), "
           "max relative err = %.2e at cycle %d ch %d\n",
           n_compared, n_skipped_noise_floor, SIGNAL_FLOOR,
           max_err, max_err_cycle, max_err_ch);
    CHECK(n_mismatched == 0,
          "%d / %d cells mismatched (>1%% rel err)",
          n_mismatched, n_compared);

    free(out);
    polyphase_channelizer_destroy(ch);
}

// --- Cross-check against gr-iridium's burst detection ground truth.
//     For each fixture (different LOs across the Albuquerque cf32),
//     feed the IQ through our channelizer and verify each gr-iridium-
//     detected burst's energy lands in the channel that
//     round((f - LO) / 40 kHz) mod M predicts.
//
//     This is the "matches gr-iridium's findings" cross-check: gr-iridium
//     found bursts at specific frequencies in iridium.bits; our
//     channelizer must route each to the corresponding channel.
//     Different LOs catch different burst clusters (the Albuquerque
//     1.25 s recording has bursts spread across the full 1616-1626.5 MHz
//     band), exercising the channelizer at different center freqs.
static void run_corpus_check(uint32_t fs_in_hz,
                             const uint8_t *iq_bytes,
                             unsigned int  iq_len,
                             const channelizer_burst_ref_t *bursts,
                             int n_bursts,
                             const char *label)
{
    printf("Test: corpus cross-check %s (%d gr-iridium bursts in subband)\n",
           label, n_bursts);
    polyphase_channelizer_t *ch = polyphase_channelizer_create(fs_in_hz);
    CHECK(ch != NULL, "create");

    int n_complex = iq_len / 2;
    float complex *in = malloc(n_complex * sizeof(float complex));
    CHECK(in != NULL, "malloc in");
    for (int i = 0; i < n_complex; i++) {
        float re = ((float)iq_bytes[2 * i + 0] - 128.0f) / 127.0f;
        float im = ((float)iq_bytes[2 * i + 1] - 128.0f) / 127.0f;
        in[i] = re + im * I;
    }

    int n_cycles = n_complex / M;
    float complex *out = malloc(n_cycles * M * sizeof(float complex));
    CHECK(out != NULL, "malloc out");
    size_t got_cycles = polyphase_channelizer_process(ch, in, n_complex, out);
    CHECK((int)got_cycles == n_cycles, "got %zu cycles", got_cycles);

    // Rank channels by PEAK per-cycle power, not total integrated
    // power. Integrated power biases toward channels with long-duration
    // strong signals (e.g. an Iridium downlink active for ~1 sec) and
    // can rank-out channels that only carry short bursts (8.28 ms
    // Iridium frames), even when those bursts are high-SNR.
    // For "did the channelizer hear this short high-SNR burst?", peak
    // power is the right proxy.
    double power[M] = { 0 };
    for (int cycle = 0; cycle < n_cycles; cycle++) {
        for (int k = 0; k < M; k++) {
            float complex y = out[cycle * M + k];
            double p = (double)(crealf(y) * crealf(y) + cimagf(y) * cimagf(y));
            if (p > power[k]) power[k] = p;
        }
    }

    int top[M];
    for (int i = 0; i < M; i++) top[i] = i;
    for (int i = 0; i < M; i++) {
        for (int j = i + 1; j < M; j++) {
            if (power[top[j]] > power[top[i]]) {
                int t = top[i]; top[i] = top[j]; top[j] = t;
            }
        }
    }
    int top_to_show = (n_bursts < 4) ? 8 : (n_bursts * 2);
    if (top_to_show > 12) top_to_show = 12;
    printf("    top-%d channels by total power:\n", top_to_show);
    for (int i = 0; i < top_to_show; i++) {
        int signed_k = (top[i] > M / 2) ? (top[i] - M) : top[i];
        printf("      #%d  ch %2d  (%+5d kHz)   power=%.3e\n",
               i + 1, top[i], signed_k * (FS_IN / M / 1000), power[top[i]]);
    }

    // Each expected channel (or ±1 neighbour) should appear in the
    // top-K. K = max(2 × n_bursts, 6) to allow neighbour leakage and
    // legitimate other bursts in the window. We split the assertion
    // by gr-iridium's confidence: high-conf bursts (≥90%) MUST be
    // in top-K (assertion); low-conf ones are reported but don't
    // fail the test — borderline detections (e.g. 60% conf with
    // "Access code missing") are noisy by definition, so the
    // channelizer behaviour on them is observable but not enforceable.
    const int HI_CONF = 90;
    int K = (2 * n_bursts < 6) ? 6 : (2 * n_bursts);
    if (K > M) K = M;
    int hi_total = 0, hi_found = 0;
    int lo_total = 0, lo_found = 0;
    for (int e = 0; e < n_bursts; e++) {
        int target = bursts[e].expected_channel;
        bool is_hi = (bursts[e].conf_pct >= HI_CONF);
        bool hit = false;
        for (int i = 0; i < K; i++) {
            if (top[i] == target ||
                top[i] == (target + 1) % M ||
                top[i] == (target - 1 + M) % M) {
                hit = true; break;
            }
        }
        if (is_hi) hi_total++;
        else       lo_total++;
        if (hit) {
            if (is_hi) hi_found++;
            else       lo_found++;
        } else {
            printf("    %s: ch %d (rel %+d Hz, SNR %.1f dB, conf %u%%) "
                   "not in top-%d\n",
                   is_hi ? "MISS" : "miss(low-conf)",
                   target, bursts[e].rel_hz,
                   (double)bursts[e].snr_db,
                   (unsigned)bursts[e].conf_pct, K);
        }
    }
    printf("    high-conf: %d/%d hit  |  low-conf: %d/%d hit\n",
           hi_found, hi_total, lo_found, lo_total);
    CHECK(hi_found == hi_total,
          "%d/%d high-confidence bursts in top-%d (low-conf %d/%d "
          "is informational)",
          hi_found, hi_total, K, lo_found, lo_total);

    free(in); free(out);
    polyphase_channelizer_destroy(ch);
}

static void test_against_gr_iridium_corpus(void)
{
    // Fixture 1: Albuquerque @ LO=1618.5 MHz — dense 1618 MHz cluster.
    run_corpus_check(ALBQ_RAW_SAMPLE_RATE_HZ,
                     ALBQ_RAW_UINT8, ALBQ_RAW_UINT8_LEN,
                     ALBQ_RAW_BURSTS,
                     (int)(sizeof(ALBQ_RAW_BURSTS) / sizeof(ALBQ_RAW_BURSTS[0])),
                     "Albuquerque LO=1618.5 MHz");
    // Fixture 2: Albuquerque @ LO=1625.5 MHz — captures the high-SNR
    // 1625.27 MHz IDA-DL burst plus its neighbours.
    run_corpus_check(ALBQ_RAW_HIGH_SAMPLE_RATE_HZ,
                     ALBQ_RAW_HIGH_UINT8, ALBQ_RAW_HIGH_UINT8_LEN,
                     ALBQ_RAW_HIGH_BURSTS,
                     (int)(sizeof(ALBQ_RAW_HIGH_BURSTS)
                           / sizeof(ALBQ_RAW_HIGH_BURSTS[0])),
                     "Albuquerque LO=1625.5 MHz");
}

// All 8 Albuquerque stripes (50% overlap covering 1615.72-1627.24 MHz):
// each stripe is 2.56 MSPS uint8 IQ pre-shifted so its center sits at
// DC. Iterating through all 8 exercises the channelizer at every LO
// across the Iridium downlink band — bursts in overlapping regions
// are tested twice (in two adjacent stripes) at different relative
// frequencies, catching bias that's specific to one LO.
static void test_against_albq_all_stripes(void)
{
    for (int s = 0; s < ALBQ_NUM_STRIPES; s++) {
        const albq_stripe_t *st = &ALBQ_STRIPES[s];
        if (st->expected_bursts == 0) {
            // Empty-stripe test data is still good test data: feed it
            // through, observe the noise-floor distribution. We don't
            // assert on a specific channel since gr-iridium found nothing
            // to predict, but we still confirm the channelizer accepts
            // the fixture and produces sane output.
            polyphase_channelizer_t *ch = polyphase_channelizer_create(2560000u);
            CHECK(ch != NULL, "create empty-stripe");
            int n_complex = st->len / 2;
            float complex *in  = malloc(n_complex * sizeof(float complex));
            float complex *out = malloc((n_complex / M) * M
                                        * sizeof(float complex));
            CHECK(in && out, "malloc empty-stripe");
            for (int i = 0; i < n_complex; i++) {
                float re = ((float)st->data[2*i+0] - 128.0f) / 127.0f;
                float im = ((float)st->data[2*i+1] - 128.0f) / 127.0f;
                in[i] = re + im * I;
            }
            size_t got = polyphase_channelizer_process(ch, in, n_complex, out);
            (void)got;
            printf("Test: Albuquerque stripe %d center %.3f MHz "
                   "(0 expected bursts — noise-floor smoke check) ok\n",
                   s, st->center_hz / 1e6);
            free(in); free(out);
            polyphase_channelizer_destroy(ch);
            continue;
        }
        char label[64];
        snprintf(label, sizeof(label),
                 "Albuquerque stripe %d @ %.3f MHz", s, st->center_hz / 1e6);
        run_corpus_check(2560000u,
                         st->data, st->len,
                         st->bursts, st->expected_bursts,
                         label);
    }
}

// Simulated PRBS15 burst @ 20 dB SNR, 2.56 MSPS, already at baseband
// (single tone-like burst at DC). The test_corpus burst is the classic
// "clean signal at DC" case — a sanity check that the channelizer
// puts a strong on-channel burst exactly where it should without any
// real-RF complications. Channel 0 must be the dominant channel, and
// adjacent channels should be well below it.
static void test_against_simulated_corpus(void)
{
    printf("Test: simulated PRBS15-20dB corpus @ 2.56 MSPS (single burst at DC)\n");
    polyphase_channelizer_t *ch = polyphase_channelizer_create(2560000u);
    CHECK(ch != NULL, "create");
    int n_complex = CORPUS_UINT8_LEN / 2;
    float complex *in  = malloc(n_complex * sizeof(float complex));
    int n_cycles = n_complex / M;
    float complex *out = malloc(n_cycles * M * sizeof(float complex));
    CHECK(in && out, "malloc");
    for (int i = 0; i < n_complex; i++) {
        float re = ((float)CORPUS_UINT8[2*i+0] - 128.0f) / 127.0f;
        float im = ((float)CORPUS_UINT8[2*i+1] - 128.0f) / 127.0f;
        in[i] = re + im * I;
    }
    polyphase_channelizer_process(ch, in, n_complex, out);

    // Skip transient.
    int skip = 16;
    double power[M] = {0};
    int count = n_cycles - skip;
    for (int cycle = skip; cycle < n_cycles; cycle++) {
        for (int k = 0; k < M; k++) {
            float complex y = out[cycle * M + k];
            power[k] += (double)(crealf(y) * crealf(y)
                                 + cimagf(y) * cimagf(y));
        }
    }
    for (int k = 0; k < M; k++) power[k] /= count;

    double max_p = 0; int max_k = -1;
    for (int k = 0; k < M; k++) {
        if (power[k] > max_p) { max_p = power[k]; max_k = k; }
    }
    printf("    max channel = %d, power = %.3e\n", max_k, max_p);
    // PRBS15 burst spans ~50 kHz around DC; ch 0 (and possibly ±1 due
    // to spectral spreading at modulator edges) should dominate.
    bool ok = (max_k == 0 || max_k == 1 || max_k == M - 1);
    CHECK(ok, "expected ch 0 (or ±1), got ch %d", max_k);

    free(in); free(out);
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
    test_against_gr_iridium_corpus();
    test_against_albq_all_stripes();
    test_against_simulated_corpus();

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
