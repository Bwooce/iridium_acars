// Host regression test for qpsk_demod against a real-RF Iridium burst.
//
// Source: test_data/iridium_downlink_2022-03-17_albuquerque (USRP B210
// recording, 12 MSPS, 1621.5 MHz center, 1.25 s of full Iridium downlink
// covering 1615.5-1627.5 MHz).
//
// The fixture builder (tests/scripts/build_albq_fixture.py) picks the
// highest-SNR IDA-DL burst that gr-iridium successfully decoded, slices
// the cf32 around it, frequency-shifts that channel to baseband,
// decimates 12 MSPS -> 50 ksps (2 sps at 25 ksym/s), and emits an int16
// IQ header plus the ground-truth bits gr-iridium produced.
//
// This test exercises qpsk_demod on real Iridium DQPSK at real Doppler
// and real per-channel SNR, validating it against the muccc/gr-iridium
// reference. It is the first real-RF host regression in this repo —
// test_demod_corpus.c uses synthetic PRBS-15 from gr-iridium upstream.
//
// TODO: extend to multi-stripe coverage. Right now we test one burst on
// one channel. The recording contains ~80 bursts spread across 1616-1626
// MHz; stripe the cf32 into 2.56 MHz subbands with 50% overlap and run
// every burst through the pipeline. Pattern after this single-stripe
// test once it's green.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include "qpsk_demod.h"
#include "fixture_albq_2sps.h"  // ALBQ_2SPS, ALBQ_2SPS_LEN
#include "fixture_albq_truth.h" // ALBQ_TRUTH_BITS, *_LEN, *_DIRECTION

static int compare_bits(const uint8_t *demod, int demod_n,
                        const uint8_t *truth, int truth_n,
                        int max_compare)
{
    int n = (demod_n < truth_n) ? demod_n : truth_n;
    if (n > max_compare) n = max_compare;
    int diffs = 0, first_diff = -1;
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

    printf("Test: real-RF Albuquerque IDA-DL burst -> qpsk_demod -> compare bits\n");
    printf("  fixture: %u int16 IQ samples (%u complex, 2 sps)\n",
           ALBQ_2SPS_LEN, ALBQ_2SPS_LEN / 2);
    printf("  ground truth: %u bits, direction=%s\n",
           ALBQ_TRUTH_BITS_LEN, ALBQ_TRUTH_BITS_DIRECTION);

    // Search all 4 phase rotations × all sub-symbol offsets, picking
    // whichever combination produces the LOWEST bit-error rate against
    // the ground truth. Why not first match: chance UW matches in the
    // burst tail or noise will pass qpsk_demod's UW check (≤2 errors
    // in 12 symbols is a lax threshold), but their bit stream is
    // uncorrelated with the truth. Ranking by BER picks the real burst.
    // For real-RF bursts with PLL locking on the BPSK chirp preamble
    // before the UW, this should be unambiguous — true match ~0% BER,
    // false matches ~50% BER.
    decoded_frame_t best_frame         = {0};
    int             best_diffs         = INT_MAX;
    int             best_compare       = 0;
    int             best_offset        = -1;
    int             best_rot           = -1;
    const char     *best_dir           = "?";
    const int       STEP_INT16         = 4;
    const int       MAX_SYMBOLS_OFFSET = 500;

    int16_t *rotated = malloc(ALBQ_2SPS_LEN * sizeof(int16_t));
    if (!rotated) {
        fprintf(stderr, "OOM\n");
        return 2;
    }

    for (int rot = 0; rot < 4; rot++) {
        for (unsigned i = 0; i < ALBQ_2SPS_LEN; i += 2) {
            int16_t ii = ALBQ_2SPS[i + 0];
            int16_t qq = ALBQ_2SPS[i + 1];
            int16_t ir, qr;
            switch (rot) {
            case 0:
                ir = ii;
                qr = qq;
                break;
            case 1:
                ir = -qq;
                qr = ii;
                break;
            case 2:
                ir = -ii;
                qr = -qq;
                break;
            default:
                ir = qq;
                qr = -ii;
                break;
            }
            rotated[i + 0] = ir;
            rotated[i + 1] = qr;
        }
        for (int sym_off = 0; sym_off < MAX_SYMBOLS_OFFSET; sym_off++) {
            int int16_off = sym_off * STEP_INT16;
            if ((int)ALBQ_2SPS_LEN - int16_off < 24 * STEP_INT16) break;
            decoded_frame_t f  = {0};
            int             rc = qpsk_demod_process(rotated + int16_off,
                                                    ALBQ_2SPS_LEN - int16_off, &f);
            if (!rc) continue;
            const char *dir =
                (f.direction == DIR_DOWNLINK) ? "DL" : (f.direction == DIR_UPLINK) ? "UL"
                                                                                   : "?";
            if (strcmp(dir, ALBQ_TRUTH_BITS_DIRECTION) != 0) {
                free(f.bits);
                continue; // wrong direction — false UW
            }
            int compare = (f.n_bits < (int)ALBQ_TRUTH_BITS_LEN
                               ? f.n_bits
                               : (int)ALBQ_TRUTH_BITS_LEN);
            int diffs   = 0;
            for (int i = 0; i < compare; i++) {
                if ((f.bits[i] & 1) != (ALBQ_TRUTH_BITS[i] & 1)) diffs++;
            }
            if (diffs < best_diffs ||
                (diffs == best_diffs && compare > best_compare)) {
                free(best_frame.bits);
                best_frame   = f;
                best_diffs   = diffs;
                best_compare = compare;
                best_offset  = sym_off;
                best_rot     = rot;
                best_dir     = dir;
            } else {
                free(f.bits);
            }
        }
    }
    free(rotated);

    if (best_offset < 0) {
        printf("  FAIL: no DL UW match across any rotation × offset\n");
        failed++;
    } else {
        printf("  best match: sym_off=%d rot=%d×90° dir=%s n_bits=%d\n",
               best_offset, best_rot, best_dir, best_frame.n_bits);
        compare_bits(best_frame.bits, best_frame.n_bits,
                     ALBQ_TRUTH_BITS, ALBQ_TRUTH_BITS_LEN,
                     ALBQ_TRUTH_BITS_LEN);
        printf("  bit comparison: %d differences across %d bits (%.2f%% BER)\n",
               best_diffs, best_compare,
               best_compare ? 100.0 * best_diffs / best_compare : 0.0);

        // 1% BER tolerance — real-RF has minor cross-implementation
        // variation from PLL convergence and timing-recovery details
        // even at 30 dB SNR. Tight enough to catch a meaningful
        // regression in the demod chain.
        const int TOLERANCE = (int)(ALBQ_TRUTH_BITS_LEN / 100);
        if (best_diffs <= TOLERANCE) {
            printf("  PASS (within tolerance ≤%d bits = 1%% BER)\n", TOLERANCE);
            passed++;
        } else {
            printf("  FAIL (more than %d bits differ)\n", TOLERANCE);
            failed++;
        }
        free(best_frame.bits);
    }

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
