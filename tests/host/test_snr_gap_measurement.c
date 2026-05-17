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

int main(void)
{
    printf("================================================================\n");
    printf("Per-stage SNR gap measurement on ALBQ_RAW_UINT8\n");
    printf("(same data gr-iridium decoded at known SNRs)\n");
    printf("LO=%u Hz, fixture has %u uint8 samples (%u complex)\n",
           ALBQ_RAW_LO_HZ, ALBQ_RAW_UINT8_LEN, ALBQ_RAW_UINT8_LEN / 2);
    printf("================================================================\n");

    burst_collector_t coll = {0};
    channelizer_detector_t *d = channelizer_detector_create(
        2560000u, 16.0f, collect_cb, &coll);
    if (!d) { printf("FAIL: detector create\n"); return 1; }

    size_t n_complex = 0;
    int16_t *iq = u8_to_int16_iq(ALBQ_RAW_UINT8, ALBQ_RAW_UINT8_LEN, &n_complex);
    if (!iq) { printf("FAIL: u8_to_int16\n"); return 1; }
    channelizer_detector_feed_int16(d, iq, n_complex);
    free(iq);
    channelizer_detector_flush(d);

    channelizer_detector_stats_t stats;
    channelizer_detector_get_stats(d, &stats);
    printf("\n[Stage 1: Channelizer detector]\n");
    printf("  cycles=%u   detections=%u   active-peak=%u\n",
           stats.cycles_processed, stats.bursts_detected,
           stats.channels_active_peak);

    printf("\n  All detected bursts:\n");
    for (int i = 0; i < coll.n; i++) {
        printf("    ch %2d  (%+5d kHz)  SNR=%5.1f dB  start=%u  len=%u\n",
               coll.bursts[i].channel,
               coll.bursts[i].rel_freq_hz / 1000,
               (double)coll.bursts[i].snr_db,
               coll.bursts[i].start_sample_idx,
               coll.bursts[i].length_samples);
    }

    printf("\n  Per-burst delta vs gr-iridium ground truth:\n");
    printf("  %-25s %12s %12s %12s\n",
           "burst", "gr-iridium", "ours", "delta_dB");
    int n_expected = sizeof(ALBQ_RAW_BURSTS) / sizeof(ALBQ_RAW_BURSTS[0]);
    float total_delta = 0.0f;
    int n_matched = 0;
    for (int e = 0; e < n_expected; e++) {
        float gr_snr = ALBQ_RAW_BURSTS[e].snr_db;
        float our_snr = match_burst_snr(&coll,
                                         ALBQ_RAW_BURSTS[e].expected_channel);
        if (our_snr < 0) {
            printf("  ch %-2u rel %+7d Hz  %10.2f dB  %12s  %12s\n",
                   ALBQ_RAW_BURSTS[e].expected_channel,
                   ALBQ_RAW_BURSTS[e].rel_hz,
                   (double)gr_snr, "MISSED", "—");
        } else {
            float delta = our_snr - gr_snr;
            printf("  ch %-2u rel %+7d Hz  %10.2f dB  %10.2f dB  %+10.2f\n",
                   ALBQ_RAW_BURSTS[e].expected_channel,
                   ALBQ_RAW_BURSTS[e].rel_hz,
                   (double)gr_snr, (double)our_snr, (double)delta);
            total_delta += delta;
            n_matched++;
        }
    }
    if (n_matched > 0) {
        printf("\n  Mean SNR delta over %d matched bursts: %+.2f dB\n",
               n_matched, (double)(total_delta / n_matched));
        printf("  (negative = our pipeline loses SNR vs gr-iridium)\n");
    }

    printf("\n[Interpretation]\n");
    printf("  - Most likely sources of SNR loss at this stage:\n");
    printf("    * Channelizer 40 kHz bins vs Iridium 41.667 kHz grid mismatch\n");
    printf("      (task #41) — energy spillage between adjacent channels.\n");
    printf("    * Prototype filter passband/stopband quality (task #45).\n");
    printf("  - gr-iridium uses a different SNR metric (post-demod);\n");
    printf("    part of the delta may be metric difference, not real loss.\n");
    printf("\n");

    channelizer_detector_destroy(d);
    printf("Done (diagnostic test — always passes).\n");
    return 0;
}
