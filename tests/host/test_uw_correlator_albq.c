// Host regression test: runs the uw_correlator pipeline on a real-RF
// Iridium burst (the Albuquerque corpus's highest-SNR DL frame) and
// asserts gr-iridium-aligned outputs at each stage.
//
// Companion to test_demod_albq.c which validates qpsk_demod end-to-
// end against gr-iridium's decoded bits. This test isolates the
// upstream sync/CFO stage:
//   1. uw_correlator_apply_rrc on the 2-sps int16 burst
//   2. uw_correlator_find_burst_start (D13 envelope onset)
//   3. uw_correlator_find (28-sym preamble+UW matched filter via FFT
//      cross-correlation, RC-shaped reference, Blackman 16× FFT CFO)
//
// Assertions track gr-iridium's burst_downmix_impl.cc behaviour on
// the same data:
//   - direction == DL (matches ALBQ_TRUTH_BITS_DIRECTION)
//   - UW position is in a plausible range (the channelizer's
//     start_sample_idx imprecision means we can't pin it to exact-1,
//     but we know the burst is short-preamble so UW is within the
//     first ~200 samples after D13's trim)
//   - CFO estimate is bounded to ±1.5 rad/sym (anything bigger
//     indicates the squared-FFT picked up a noise peak)
//   - matched-filter peak/floor ≥ 8 dB (the burst gr-iridium decoded
//     at 30 dB SNR should give a crisp matched-filter peak)

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "uw_correlator.h"
#include "fixture_albq_2sps.h"      // ALBQ_2SPS, ALBQ_2SPS_LEN
#include "fixture_albq_truth.h"     // ALBQ_TRUTH_BITS_DIRECTION

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

int main(void)
{
    // Make a mutable copy of the fixture (RRC filter writes back in-place).
    int n_complex = (int)(ALBQ_2SPS_LEN / 2);
    int16_t *burst = malloc(ALBQ_2SPS_LEN * sizeof(int16_t));
    memcpy(burst, ALBQ_2SPS, ALBQ_2SPS_LEN * sizeof(int16_t));

    printf("Albuquerque DL burst: %d complex samples (%d int16)\n",
           n_complex, (int)ALBQ_2SPS_LEN);
    printf("Truth direction: %s (%d bits)\n",
           ALBQ_TRUTH_BITS_DIRECTION, 382 /* ALBQ_TRUTH_BITS_LEN — header has it */);

    // Stage 1: RRC matched filter on the burst.
    uw_correlator_apply_rrc(burst, burst, n_complex);
    printf("RRC matched filter applied.\n");

    // Stage 2: D13 sub-frame burst-edge detection.
    int burst_start = uw_correlator_find_burst_start(burst, n_complex,
                                                      /*search_max=*/256);
    printf("D13 burst start: %d (of %d samples)\n", burst_start, n_complex);
    // The fixture was already trimmed by build_albq_fixture.py to
    // start near the burst envelope onset; D13 should report a small
    // offset (≤ ~64 samples) or zero if the envelope is already at
    // the head.
    CHECK(burst_start >= 0 && burst_start <= 64,
          "D13 burst start in plausible range (got %d)", burst_start);

    int16_t *adj_burst = burst + burst_start * 2;
    int adj_n = n_complex - burst_start;

    // Stage 3: matched-filter UW correlator + Blackman FFT CFO.
    uw_corr_result_t res;
    uw_correlator_find(adj_burst, adj_n, adj_n - 24, &res);

    printf("UW corr: dir=%s offset=%d corr=%.3f SNR=%.1f dB peak=%.2e omega=%.3f\n",
           res.direction == UW_DIR_DOWNLINK ? "DL"
           : res.direction == UW_DIR_UPLINK ? "UL" : "UNKNOWN",
           res.uw_offset, (double)res.correction,
           (double)res.snr_estimate_db, (double)res.peak_value,
           (double)res.omega_per_sym);

    // Direction must match gr-iridium's ground truth.
    CHECK(res.direction == UW_DIR_DOWNLINK,
          "direction = DL (got %d)", (int)res.direction);

    // UW position must be inside the burst with enough data after
    // for the demod (UW + IDA frame body). The fixture is ~1108
    // complex samples covering one TDMA slot plus padding; the UW
    // can land anywhere in the first ~half of the buffer.
    CHECK(res.uw_offset >= 0 && res.uw_offset < adj_n - 24,
          "uw_offset in burst range (got %d, burst=%d)",
          res.uw_offset, adj_n);
    // The fixture builder pads the burst so it contains the full
    // TDMA slot plus context. Verify the UW leaves room for a full
    // 12-sym × 2 sps × 8-bit payload's worth of samples after it.
    CHECK(adj_n - res.uw_offset >= 12 * 2 + 100 * 2,
          "≥100 syms of data after UW (got %d after offset %d)",
          adj_n - res.uw_offset, res.uw_offset);

    // Matched-filter peak SNR: the gr-iridium-decoded burst has
    // ≈30 dB raw SNR, so the RRC-shaped correlation peak should
    // clear ~8 dB above the off-peak floor comfortably.
    CHECK(res.snr_estimate_db >= 8.0f,
          "matched-filter SNR ≥ 8 dB (got %.1f)",
          (double)res.snr_estimate_db);

    // CFO must be bounded — anything beyond ±1.5 rad/sym indicates
    // the FFT clipped to its clamp and didn't find a real tone.
    float abs_omega = res.omega_per_sym < 0 ? -res.omega_per_sym : res.omega_per_sym;
    CHECK(abs_omega <= 1.5f,
          "CFO magnitude ≤ 1.5 rad/sym (got %.3f)",
          (double)res.omega_per_sym);

    // Sanity: the complex peak value should be non-trivial (used
    // for pre-rotation in the worker).
    float peak_mag = res.peak_re * res.peak_re + res.peak_im * res.peak_im;
    CHECK(peak_mag > 1.0f,
          "complex peak magnitude non-trivial (got %.2e)",
          (double)peak_mag);

    free(burst);

    printf("\n=== %d passed, %d failed ===\n", s_passed, s_failed);
    return s_failed > 0 ? 1 : 0;
}
