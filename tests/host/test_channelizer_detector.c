// test_channelizer_detector — feeds the channelizer_detector the same
// fixtures we used to validate the bare polyphase_channelizer, and
// asserts the detector emits a burst on each gr-iridium-detected
// channel. This is the integration smoke test for D7 Phase 2 — once
// it passes, we know the detector works end-to-end on real-RF IQ at
// multiple LOs and on the simulated PRBS corpus, and can be wired
// into dsp_processor with confidence.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "channelizer_detector.h"
#include "channelizer_burst_ref.h"
#include "fixture_albq_raw.h"
#include "fixture_albq_raw_high.h"
#include "fixture_corpus_uint8.h"

static int passed = 0, failed = 0;

#define CHECK(cond, fmt, ...) do {                                       \
    if (!(cond)) {                                                       \
        printf("  FAIL line %d: " fmt "\n", __LINE__, ##__VA_ARGS__);    \
        failed++; return;                                                \
    } else { passed++; }                                                 \
} while (0)

#define M 64

// Convert a uint8 IQ buffer (the fixture format) to int16 IQ for
// channelizer_detector_feed_int16. Caller frees.
static int16_t *u8_to_int16_iq(const uint8_t *u8, unsigned int n_bytes,
                                size_t *out_n_complex)
{
    size_t n_cplx = n_bytes / 2;
    int16_t *out = malloc(n_bytes * sizeof(int16_t));
    if (!out) return NULL;
    for (size_t i = 0; i < n_cplx; i++) {
        int re = (int)u8[2 * i + 0] - 128;
        int im = (int)u8[2 * i + 1] - 128;
        // Map int8 range [-128, 127] to int16 with scale 256.
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

static void collect_cb(const channelizer_burst_t *burst, void *user)
{
    burst_collector_t *c = (burst_collector_t *)user;
    if (c->n < MAX_BURSTS_OUT) {
        c->bursts[c->n++] = *burst;
    }
}

static bool channel_close(int a, int b)
{
    int diff = (a - b + M) % M;
    if (diff > M / 2) diff = M - diff;
    return diff <= 1;
}

static void run_detector_on_fixture(uint32_t fs_hz,
                                     const uint8_t *u8, unsigned int n_bytes,
                                     const channelizer_burst_ref_t *expected,
                                     int n_expected,
                                     const char *label)
{
    printf("Test: detector on %s (%d gr-iridium bursts expected)\n",
           label, n_expected);
    burst_collector_t coll = {0};
    channelizer_detector_t *d = channelizer_detector_create(
        fs_hz, 16.0f, collect_cb, &coll);
    CHECK(d != NULL, "create");

    size_t n_complex = 0;
    int16_t *iq = u8_to_int16_iq(u8, n_bytes, &n_complex);
    CHECK(iq != NULL, "u8_to_int16");

    channelizer_detector_feed_int16(d, iq, n_complex);
    free(iq);
    // End-of-stream: flush pending COOLING / IN_BURST bursts so the
    // test sees all detections.
    channelizer_detector_flush(d);

    channelizer_detector_stats_t stats;
    channelizer_detector_get_stats(d, &stats);
    printf("    cycles=%u bursts_emitted=%u channels_active_peak=%u\n",
           stats.cycles_processed, stats.bursts_detected,
           stats.channels_active_peak);
    printf("    detected bursts:\n");
    for (int i = 0; i < coll.n; i++) {
        const channelizer_burst_t *b = &coll.bursts[i];
        printf("      ch %2d  (%+5d kHz)  SNR=%.1f dB  "
               "start=%u  len=%u\n",
               b->channel, b->rel_freq_hz / 1000,
               (double)b->snr_db, b->start_sample_idx,
               b->length_samples);
    }

    // For each high-confidence expected burst, find a detected burst
    // on its channel (or ±1 neighbour). Low-conf ones reported but
    // don't gate the test.
    int hi_total = 0, hi_hit = 0;
    int lo_total = 0, lo_hit = 0;
    for (int e = 0; e < n_expected; e++) {
        bool is_hi = (expected[e].conf_pct >= 90);
        bool hit = false;
        for (int i = 0; i < coll.n; i++) {
            if (channel_close(coll.bursts[i].channel,
                              expected[e].expected_channel)) {
                hit = true; break;
            }
        }
        if (is_hi) { hi_total++; if (hit) hi_hit++; }
        else       { lo_total++; if (hit) lo_hit++; }
        if (!hit) {
            printf("    %s: ch %d (rel %+d Hz, SNR %.1f dB, conf %u%%) "
                   "NOT detected\n",
                   is_hi ? "MISS" : "miss(low-conf)",
                   expected[e].expected_channel,
                   expected[e].rel_hz,
                   (double)expected[e].snr_db,
                   (unsigned)expected[e].conf_pct);
        }
    }
    printf("    high-conf: %d/%d hit  |  low-conf: %d/%d hit\n",
           hi_hit, hi_total, lo_hit, lo_total);
    CHECK(hi_hit == hi_total,
          "%d/%d high-conf bursts detected (low-conf %d/%d "
          "informational)", hi_hit, hi_total, lo_hit, lo_total);

    channelizer_detector_destroy(d);
}

static void test_simulated_corpus(void)
{
    // Single PRBS15 burst at DC; expect at least one detection on
    // channel 0 (or ±1).
    printf("Test: detector on simulated PRBS15 corpus (single burst @ DC)\n");
    burst_collector_t coll = {0};
    channelizer_detector_t *d = channelizer_detector_create(
        2560000u, 16.0f, collect_cb, &coll);
    CHECK(d != NULL, "create");

    size_t n_complex = 0;
    int16_t *iq = u8_to_int16_iq(CORPUS_UINT8, CORPUS_UINT8_LEN, &n_complex);
    CHECK(iq != NULL, "u8_to_int16");
    channelizer_detector_feed_int16(d, iq, n_complex);
    free(iq);
    // End-of-stream: flush pending COOLING / IN_BURST bursts so the
    // test sees all detections.
    channelizer_detector_flush(d);

    channelizer_detector_stats_t stats;
    channelizer_detector_get_stats(d, &stats);
    printf("    cycles=%u bursts_emitted=%u\n",
           stats.cycles_processed, stats.bursts_detected);
    bool found_dc = false;
    for (int i = 0; i < coll.n; i++) {
        printf("      ch %2d  SNR=%.1f dB  start=%u  len=%u\n",
               coll.bursts[i].channel,
               (double)coll.bursts[i].snr_db,
               coll.bursts[i].start_sample_idx,
               coll.bursts[i].length_samples);
        if (channel_close(coll.bursts[i].channel, 0)) {
            found_dc = true;
        }
    }
    CHECK(found_dc, "no burst detected on DC channel");
    channelizer_detector_destroy(d);
}

int main(void)
{
    test_simulated_corpus();
    run_detector_on_fixture(
        ALBQ_RAW_SAMPLE_RATE_HZ,
        ALBQ_RAW_UINT8, ALBQ_RAW_UINT8_LEN,
        ALBQ_RAW_BURSTS,
        (int)(sizeof(ALBQ_RAW_BURSTS) / sizeof(ALBQ_RAW_BURSTS[0])),
        "Albuquerque LO=1618.5 MHz");
    run_detector_on_fixture(
        ALBQ_RAW_HIGH_SAMPLE_RATE_HZ,
        ALBQ_RAW_HIGH_UINT8, ALBQ_RAW_HIGH_UINT8_LEN,
        ALBQ_RAW_HIGH_BURSTS,
        (int)(sizeof(ALBQ_RAW_HIGH_BURSTS) / sizeof(ALBQ_RAW_HIGH_BURSTS[0])),
        "Albuquerque LO=1625.5 MHz");

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
