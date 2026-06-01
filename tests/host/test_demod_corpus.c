// Bit-level regression test for qpsk_demod against the test_corpus.
//
// Drives qpsk_demod_process with a 2-sps int16 IQ fixture extracted from
// test_corpus/prbs15-2M-20dB.sigmf-data (decimated 40× from 2 MSPS to
// 50 ksps in tests/scripts/build_fixtures.py), then compares its bit
// output to the ground-truth bits emitted by upstream gr-iridium /
// iridium-toolkit on the same input.
//
// Tolerance: the upstream tools run cf32 with no quantisation; we feed
// int16 through our PLL/DQPSK chain, which introduces a small amount of
// quantisation noise plus PLL transient bits during convergence. We
// allow ≤2 differences across the whole 382-bit ground-truth string —
// tighter than 1% bit-error and enough to catch any meaningful
// regression in the demod logic.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "qpsk_demod.h"
#include "fixture_corpus_2sps.h"  // CORPUS_2SPS, CORPUS_2SPS_LEN
#include "fixture_ground_truth.h" // GROUND_TRUTH_BITS, *_LEN, *_DIRECTION

static int compare_bits(const uint8_t *demod, int demod_n,
                        const uint8_t *truth, int truth_n,
                        int max_compare)
{
    int n = (demod_n < truth_n) ? demod_n : truth_n;
    if (n > max_compare) n = max_compare;
    // Both demod->bits and truth[] are stored as raw 0/1 bytes (not
    // ASCII '0'/'1'). The fixture builder emits the ground truth bits
    // as literal C integer values 0 and 1.
    int diffs      = 0;
    int first_diff = -1;
    for (int i = 0; i < n; i++) {
        uint8_t d = demod[i] & 1;
        uint8_t t = truth[i] & 1;
        if (d != t) {
            if (first_diff < 0) first_diff = i;
            diffs++;
        }
    }
    if (first_diff >= 0) {
        printf("  first bit mismatch at index %d (demod=%d truth=%d)\n",
               first_diff, demod[first_diff] & 1, truth[first_diff] & 1);
        int s = first_diff - 8 < 0 ? 0 : first_diff - 8;
        int e = first_diff + 24;
        if (e > n) e = n;
        printf("  demod[%d..%d]: ", s, e - 1);
        for (int i = s; i < e; i++)
            putchar('0' + (demod[i] & 1));
        putchar('\n');
        printf("  truth[%d..%d]: ", s, e - 1);
        for (int i = s; i < e; i++)
            putchar('0' + (truth[i] & 1));
        putchar('\n');
    }
    return diffs;
}

int main(void)
{
    int passed = 0, failed = 0;

    printf("Test 1: corpus burst -> qpsk_demod -> compare bits\n");
    printf("  fixture: %u int16 IQ samples (%u complex symbols at 2 sps)\n",
           CORPUS_2SPS_LEN, CORPUS_2SPS_LEN / 2);
    printf("  ground truth: %u bits, direction=%s\n",
           GROUND_TRUTH_BITS_LEN, GROUND_TRUTH_BITS_DIRECTION);

    // qpsk_demod assumes its input begins at the Unique Word (the first
    // 12 symbols are checked against IR_UW_DL / IR_UW_UL). Real bursts
    // include a clock-recovery preamble before the UW and the energy
    // detector slacks the leading edge by a few hundred μs, so the UW
    // is somewhere INSIDE our cropped fixture, not at sample 0. Slide
    // the start offset (in 1-symbol == 4 int16 strides) until the demod
    // succeeds. In production this alignment work is the worker's job;
    // we replicate it here so the regression test stays focused on
    // demod correctness, not alignment.
    decoded_frame_t frame            = {0};
    int             rc               = 0;
    int             alignment_offset = -1;
    // Try up to ~200 symbols of leading offset (covers a generous
    // preamble + envelope slack of ~8 ms at 25 ksym/s).
    const int STEP_INT16         = 4; // 1 symbol = 2 complex × 2 int16
    const int MAX_SYMBOLS_OFFSET = 200;
    for (int sym_off = 0; sym_off < MAX_SYMBOLS_OFFSET; sym_off++) {
        int int16_off = sym_off * STEP_INT16;
        if ((int)CORPUS_2SPS_LEN - int16_off < 24 * STEP_INT16) break;
        memset(&frame, 0, sizeof(frame));
        rc = qpsk_demod_process(CORPUS_2SPS + int16_off,
                                CORPUS_2SPS_LEN - int16_off, &frame);
        if (rc) {
            alignment_offset = sym_off;
            break;
        }
    }

    if (!rc) {
        printf("  FAIL: qpsk_demod_process found no UW match across %d "
               "symbol offsets (preamble misalignment, "
               "fixture corrupt, or demod regression)\n",
               MAX_SYMBOLS_OFFSET);
        failed++;
    } else {
        printf("  UW found after %d-symbol offset (preamble length)\n",
               alignment_offset);
        const char *dir =
            (frame.direction == DIR_DOWNLINK) ? "DL" : (frame.direction == DIR_UPLINK) ? "UL"
                                                                                       : "UNKNOWN";
        printf("  demod success: direction=%s, n_bits=%d\n", dir, frame.n_bits);

        // Direction must match.
        if (strcmp(dir, GROUND_TRUTH_BITS_DIRECTION) != 0) {
            printf("  FAIL: direction mismatch (got %s, expected %s)\n",
                   dir, GROUND_TRUTH_BITS_DIRECTION);
            failed++;
        } else {
            // Compare against the ground truth (as much of it as both sides have).
            int diffs = compare_bits(frame.bits, frame.n_bits,
                                     GROUND_TRUTH_BITS, GROUND_TRUTH_BITS_LEN,
                                     GROUND_TRUTH_BITS_LEN);
            printf("  bit comparison: %d differences across %d bits\n",
                   diffs,
                   (frame.n_bits < (int)GROUND_TRUTH_BITS_LEN
                        ? frame.n_bits
                        : (int)GROUND_TRUTH_BITS_LEN));

            // Tolerance: ≤2 mismatches over the full ground truth.
            const int TOLERANCE = 2;
            if (diffs <= TOLERANCE) {
                printf("  PASS (within tolerance ≤%d)\n", TOLERANCE);
                passed++;
            } else {
                printf("  FAIL (more than %d differences)\n", TOLERANCE);
                failed++;
            }
        }
        free(frame.bits);
        free(frame.soft_bits); // #112
    }

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
