// Bit-level regression test for qpsk_demod against the low-SNR
// corpus variant. Same source signal as test_demod_corpus.c, but with
// synthetic AWGN added before decimation to drop the SNR from the
// corpus's 20 dB to ~10 dB.
//
// Why this matters: at 20 dB SNR the demod has ~10 dB of unused
// margin, so a regression that costs 6-8 dB of demod sensitivity
// would still produce 0 bit errors on the high-SNR corpus. The 10 dB
// fixture exercises the marginal case where a small loss of margin
// produces visible bit errors. The failure mode of a future change
// loses sensitivity gracefully, not all-or-nothing.
//
// Tolerance: at 10 dB SNR with the current f32 implementation we
// measure 0–2 bit errors (PLL converges within the UW; the BER is
// well below the BCH(31,21) t=2 threshold). Allow ≤8 bit errors over
// the 382-bit ground truth — that's <2.1% BER, comfortably within
// what BCH would correct, but tight enough to catch a regression
// that loses 3+ dB of margin.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "qpsk_demod.h"
#include "fixture_corpus_2sps_lowsnr.h"   // CORPUS_2SPS_LOWSNR, *_LEN
#include "fixture_ground_truth.h"          // GROUND_TRUTH_BITS, *_LEN, *_DIRECTION

static int compare_bits(const uint8_t *demod, int demod_n,
                        const uint8_t *truth, int truth_n,
                        int max_compare)
{
    int n = (demod_n < truth_n) ? demod_n : truth_n;
    if (n > max_compare) n = max_compare;
    int diffs = 0;
    for (int i = 0; i < n; i++) {
        if ((demod[i] & 1) != (truth[i] & 1)) diffs++;
    }
    return diffs;
}

int main(void)
{
    int passed = 0, failed = 0;

    printf("Test: low-SNR (~10 dB) corpus burst -> qpsk_demod -> compare bits\n");
    printf("  fixture: %u int16 IQ samples (%u complex symbols at 2 sps)\n",
           CORPUS_2SPS_LOWSNR_LEN, CORPUS_2SPS_LOWSNR_LEN / 2);
    printf("  ground truth: %u bits, direction=%s\n",
           GROUND_TRUTH_BITS_LEN, GROUND_TRUTH_BITS_DIRECTION);

    // Slide UW alignment offset (same approach as test_demod_corpus.c).
    decoded_frame_t frame = { 0 };
    int rc = 0;
    int alignment_offset = -1;
    const int STEP_INT16 = 4;
    const int MAX_SYMBOLS_OFFSET = 200;
    for (int sym_off = 0; sym_off < MAX_SYMBOLS_OFFSET; sym_off++) {
        int int16_off = sym_off * STEP_INT16;
        if ((int)CORPUS_2SPS_LOWSNR_LEN - int16_off < 24 * STEP_INT16) break;
        memset(&frame, 0, sizeof(frame));
        rc = qpsk_demod_process(CORPUS_2SPS_LOWSNR + int16_off,
                                CORPUS_2SPS_LOWSNR_LEN - int16_off,
                                DIR_UNKNOWN, &frame);
        if (rc) {
            alignment_offset = sym_off;
            break;
        }
    }

    if (!rc) {
        printf("  FAIL: qpsk_demod found no UW match across %d offsets at 10 dB SNR\n",
               MAX_SYMBOLS_OFFSET);
        printf("  (regression: prior versions converged within ~100 symbols of preamble)\n");
        failed++;
    } else {
        const char *dir =
            (frame.direction == DIR_DOWNLINK) ? "DL" :
            (frame.direction == DIR_UPLINK) ? "UL" : "UNKNOWN";
        printf("  UW found after %d-symbol offset; direction=%s, n_bits=%d\n",
               alignment_offset, dir, frame.n_bits);

        if (strcmp(dir, GROUND_TRUTH_BITS_DIRECTION) != 0) {
            printf("  FAIL: direction mismatch (got %s, expected %s)\n",
                   dir, GROUND_TRUTH_BITS_DIRECTION);
            failed++;
        } else {
            int diffs = compare_bits(frame.bits, frame.n_bits,
                                     GROUND_TRUTH_BITS, GROUND_TRUTH_BITS_LEN,
                                     GROUND_TRUTH_BITS_LEN);
            int compared = (frame.n_bits < (int)GROUND_TRUTH_BITS_LEN
                            ? frame.n_bits : (int)GROUND_TRUTH_BITS_LEN);
            float ber = compared > 0 ? 100.0f * diffs / (float)compared : 0;
            printf("  bit comparison: %d differences across %d bits (BER %.2f%%)\n",
                   diffs, compared, ber);

            const int TOLERANCE = 8;
            if (diffs <= TOLERANCE) {
                printf("  PASS (within tolerance ≤%d bit errors at ~10 dB SNR)\n",
                       TOLERANCE);
                passed++;
            } else {
                printf("  FAIL (more than %d differences)\n", TOLERANCE);
                failed++;
            }
        }
        free(frame.bits);
    }

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
