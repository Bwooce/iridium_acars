// test_snr_gap_measurement — diagnose the per-stage SNR loss between
// our pipeline and gr-iridium's, using the SAME raw uint8 SDR data
// (ALBQ_RAW_UINT8) that both pipelines have ground truth for.
//
// gr-iridium decoded 3 bursts in this fixture at SNRs:
//   25.10 dB at ch 58 (rel -232891 Hz, t=1151.88 ms)
//   20.46 dB at ch 0  (rel +17103 Hz,  t=1160.27 ms)
//   19.16 dB at ch 56 (rel -316217 Hz, t=1153.81 ms)
//
// This test runs OUR channelizer detector on the same input and
// reports:
//   - Per-burst SNR_dB from our detector vs gr-iridium's reported value
//   - The delta = (our_SNR - gr_iridium_SNR) for each burst
//
// Per-stage breakdown (channelizer is just the first stage; full
// pipeline test follows separately).
//
// The test PRINTS per-stage measurements and PASSES regardless of
// the SNR delta — it's a diagnostic instrument, not a regression
// gate. The next-stage gate (channelizer task #41) will use these
// baseline numbers to verify improvement.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "channelizer_detector.h"
#include "channelizer_burst_ref.h"
#include "fixture_albq_raw.h"
#include "fixture_albq_raw_2667.h"

#define M 64

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

#define MAX_BURSTS_OUT 256
typedef struct {
    int n;
    channelizer_burst_t bursts[MAX_BURSTS_OUT];
} burst_collector_t;

static void collect_cb(const channelizer_burst_t *b, void *user)
{
    burst_collector_t *c = (burst_collector_t *)user;
    if (c->n < MAX_BURSTS_OUT) c->bursts[c->n++] = *b;
}

// Match a gr-iridium-reference burst to our detected bursts by
// channel proximity (±1 channel). Returns the best matching burst's
// SNR, or NAN if no match.
static float match_burst_snr(const burst_collector_t *coll,
                             int expected_channel)
{
    float best_snr = -1.0f;
    for (int i = 0; i < coll->n; i++) {
        int diff = (coll->bursts[i].channel - expected_channel + M) % M;
        if (diff > M / 2) diff = M - diff;
        if (diff <= 1) {
            if (coll->bursts[i].snr_db > best_snr) {
                best_snr = coll->bursts[i].snr_db;
            }
        }
    }
    return best_snr;
}

// Run one stage-1 channelizer measurement on the given uint8 fixture.
// Returns mean SNR delta (or NAN if no bursts matched).
static float run_one(uint32_t fs_hz, const uint8_t *u8, unsigned int n_bytes,
                     const channelizer_burst_ref_t *expected, int n_expected,
                     const char *label)
{
    printf("\n----[%s @ fs=%u Hz]----\n", label, fs_hz);
    burst_collector_t coll = {0};
    channelizer_detector_t *d = channelizer_detector_create(
        fs_hz, 16.0f, collect_cb, &coll);
    if (!d) { printf("FAIL: detector create\n"); return 0.0f / 0.0f; }
    size_t n_complex = 0;
    int16_t *iq = u8_to_int16_iq(u8, n_bytes, &n_complex);
    channelizer_detector_feed_int16(d, iq, n_complex);
    free(iq);
    channelizer_detector_flush(d);

    channelizer_detector_stats_t stats;
    channelizer_detector_get_stats(d, &stats);
    printf("  cycles=%u  detections=%u  active-peak=%u\n",
           stats.cycles_processed, stats.bursts_detected,
           stats.channels_active_peak);
    printf("  All detected:\n");
    for (int i = 0; i < coll.n; i++) {
        printf("    ch %2d  (%+5d kHz)  SNR=%5.1f dB  start=%u  len=%u\n",
               coll.bursts[i].channel,
               coll.bursts[i].rel_freq_hz / 1000,
               (double)coll.bursts[i].snr_db,
               coll.bursts[i].start_sample_idx,
               coll.bursts[i].length_samples);
    }
    printf("  %-25s %12s %12s %12s\n",
           "burst", "gr-iridium", "ours", "delta_dB");
    float total_delta = 0.0f;
    int n_matched = 0;
    for (int e = 0; e < n_expected; e++) {
        float gr_snr = expected[e].snr_db;
        float our_snr = match_burst_snr(&coll, expected[e].expected_channel);
        if (our_snr < 0) {
            printf("  ch %-2u rel %+7d Hz  %10.2f dB  %12s  %12s\n",
                   expected[e].expected_channel, expected[e].rel_hz,
                   (double)gr_snr, "MISSED", "—");
        } else {
            float delta = our_snr - gr_snr;
            printf("  ch %-2u rel %+7d Hz  %10.2f dB  %10.2f dB  %+10.2f\n",
                   expected[e].expected_channel, expected[e].rel_hz,
                   (double)gr_snr, (double)our_snr, (double)delta);
            total_delta += delta;
            n_matched++;
        }
    }
    channelizer_detector_destroy(d);
    if (n_matched == 0) return 0.0f / 0.0f;
    float mean = total_delta / n_matched;
    printf("  Mean delta: %+.2f dB over %d matched bursts\n", (double)mean, n_matched);
    return mean;
}

int main(void)
{
    printf("================================================================\n");
    printf("Per-stage SNR gap measurement: 2.56 MHz vs 2.667 MHz channelizer\n");
    printf("Same source bursts (gr-iridium ground truth from iridium.bits)\n");
    printf("Tests task #48 option B (fs change for Iridium-grid alignment).\n");
    printf("================================================================\n");

    int n_baseline = sizeof(ALBQ_RAW_BURSTS) / sizeof(ALBQ_RAW_BURSTS[0]);
    float mean_2560 = run_one(2560000u, ALBQ_RAW_UINT8, ALBQ_RAW_UINT8_LEN,
                               ALBQ_RAW_BURSTS, n_baseline,
                               "BASELINE 2.56 MHz, 40 kHz bins");

    int n_2667 = sizeof(ALBQ_RAW_2667_BURSTS) / sizeof(ALBQ_RAW_2667_BURSTS[0]);
    float mean_2667 = run_one(ALBQ_RAW_2667_SAMPLE_RATE_HZ,
                               ALBQ_RAW_2667_UINT8, ALBQ_RAW_2667_UINT8_LEN,
                               ALBQ_RAW_2667_BURSTS, n_2667,
                               "OPTION B 2.667 MHz, 41.667 kHz bins");

    printf("\n================================================================\n");
    printf("SUMMARY\n");
    printf("  Baseline (2.56 MHz, grid-misaligned): %+.2f dB mean delta\n",
           (double)mean_2560);
    printf("  Option B (2.667 MHz, grid-aligned)  : %+.2f dB mean delta\n",
           (double)mean_2667);
    if (mean_2667 == mean_2667 && mean_2560 == mean_2560) {
        float recovery = mean_2667 - mean_2560;
        printf("  Option B recovery vs baseline       : %+.2f dB\n",
               (double)recovery);
    }
    printf("================================================================\n");
    printf("Done (diagnostic test — always passes).\n");
    return 0;
}
