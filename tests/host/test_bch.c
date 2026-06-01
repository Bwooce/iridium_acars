// Host-side unit tests for bch_decoder.c (BCH(31,21) t=2, generator 1207).
//
// Strategy: encode a known 21-bit message, then test the decoder against
// the clean codeword, single-bit errors at every position, and a sample
// of two-bit errors. Decoder is required to recover the original message
// in all correctable cases.
//
// Build: see CMakeLists.txt; run as ./test_bch.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "bch_decoder.h"

// Generator polynomial for BCH(31,21) t=2 in our project. Same as
// BCH_POLY_RA from bch_decoder.h.
#define BCH_POLY 1207

// Encode a 21-bit message (MSB-first in data[0..20]) into a 31-bit
// codeword (MSB-first in out_block31[0..30]) by computing the syndrome
// remainder of (message << 10) and appending it as the parity bits.
static void bch_encode(const uint8_t *data21, uint8_t *out_block31)
{
    uint32_t msg = 0;
    for (int i = 0; i < 21; i++)
        msg = (msg << 1) | (data21[i] & 1);
    uint32_t shifted = msg << 10;
    // Compute remainder of shifted by BCH_POLY in GF(2).
    uint32_t r         = shifted;
    int      poly_bits = 32 - __builtin_clz(BCH_POLY); // 11
    for (int i = 30; i >= poly_bits - 1; i--) {
        if (r & (1u << i)) r ^= ((uint32_t)BCH_POLY) << (i - poly_bits + 1);
    }
    uint32_t cw = shifted | (r & 0x3FF);
    for (int i = 0; i < 31; i++)
        out_block31[i] = (cw >> (30 - i)) & 1;
}

static int bits_eq(const uint8_t *a, const uint8_t *b, int n)
{
    for (int i = 0; i < n; i++)
        if ((a[i] & 1) != (b[i] & 1)) return 0;
    return 1;
}

static void random_message(uint8_t *out21, unsigned seed)
{
    srand(seed);
    for (int i = 0; i < 21; i++)
        out21[i] = rand() & 1;
}

static int run_one(const uint8_t *msg21, int n_errors, const int *err_positions)
{
    uint8_t cw[31], rcv[31], out_msg[21];
    bch_encode(msg21, cw);
    memcpy(rcv, cw, 31);
    for (int i = 0; i < n_errors; i++)
        rcv[err_positions[i]] ^= 1;

    int rc = bch_decode_block(rcv, out_msg);

    // For 0..2 errors the decoder should report exactly that count and
    // recover the original message. For 3+ errors behaviour is unspecified
    // (BCH(31,21) t=2 corrects up to 2; more is best-effort).
    if (n_errors <= 2) {
        if (rc != n_errors) {
            printf("  FAIL: errors=%d, decoder returned %d\n", n_errors, rc);
            return 0;
        }
        if (!bits_eq(msg21, out_msg, 21)) {
            printf("  FAIL: errors=%d, message not recovered\n", n_errors);
            return 0;
        }
    }
    return 1;
}

int main(void)
{
    bch_decoder_init();

    int passed = 0, failed = 0;

    // Test 1: clean codeword (0 errors), 8 random messages.
    printf("Test 1: clean codeword decode\n");
    for (unsigned seed = 1; seed <= 8; seed++) {
        uint8_t msg[21];
        random_message(msg, seed);
        if (run_one(msg, 0, NULL))
            passed++;
        else
            failed++;
    }

    // Test 2: every single-bit error (31 positions × 4 messages).
    printf("Test 2: single-bit error correction at every position\n");
    for (unsigned seed = 1; seed <= 4; seed++) {
        uint8_t msg[21];
        random_message(msg, seed);
        for (int p = 0; p < 31; p++) {
            int errs[1] = {p};
            if (run_one(msg, 1, errs))
                passed++;
            else {
                failed++;
                printf("    msg seed=%u, error at bit %d\n", seed, p);
            }
        }
    }

    // Test 3: two-bit errors — sample 100 positional pairs across 4 messages.
    printf("Test 3: two-bit error correction (sampled pairs)\n");
    srand(42);
    for (unsigned seed = 1; seed <= 4; seed++) {
        uint8_t msg[21];
        random_message(msg, seed);
        for (int t = 0; t < 100; t++) {
            int p1 = rand() % 31;
            int p2;
            do {
                p2 = rand() % 31;
            } while (p2 == p1);
            int errs[2] = {p1, p2};
            if (run_one(msg, 2, errs))
                passed++;
            else {
                failed++;
                printf("    msg seed=%u, errors at bits %d,%d\n", seed, p1, p2);
            }
        }
    }

    // Test 4: deinterleaver round-trip.
    // Build an interleaved buffer of size 64 (2 × 31 bits + 2 padding ?)
    // by reading the function: it reads in[2*s] / in[2*s+1] for s in
    // [31..1 step -2] and [30..0 step -2]. So the in[] buffer is indexed
    // up to 2*31+1 = 63. Out arrays receive 32 bits each.
    printf("Test 4: deinterleaver structure\n");
    uint8_t in[64], out1[32], out2[32];
    for (int i = 0; i < 64; i++)
        in[i] = (i * 13) & 1; // arbitrary pattern
    iridium_deinterleave(in, out1, out2);
    // Spot-check: out1[0] = in[2*31] = in[62] = (62*13)&1 = 806&1 = 0
    // out1[1] = in[63] = (63*13)&1 = 819&1 = 1
    // out2[0] = in[2*30] = in[60] = (60*13)&1 = 780&1 = 0
    // out2[1] = in[61] = (61*13)&1 = 793&1 = 1
    if (out1[0] == ((62 * 13) & 1) && out1[1] == ((63 * 13) & 1) &&
        out2[0] == ((60 * 13) & 1) && out2[1] == ((61 * 13) & 1)) {
        passed++;
    } else {
        printf("  FAIL: deinterleave spot-check\n");
        failed++;
    }

    // Test 5: Chase-2 soft-decision decoder on clean inputs. Soft
    // inputs are +/- 1000 with sign matching the codeword bit. K=3
    // tries 8 hypotheses; on a clean codeword the unflipped one wins
    // with distance 0.
    printf("Test 5: Chase-2 clean codeword decode\n");
    for (unsigned seed = 1; seed <= 4; seed++) {
        uint8_t msg[21], cw[31], out_msg[21];
        int16_t soft[31];
        random_message(msg, seed);
        bch_encode(msg, cw);
        for (int i = 0; i < 31; i++)
            soft[i] = cw[i] ? -1000 : +1000;
        int rc = bch_decode_block_soft(soft, out_msg, 3);
        if (rc >= 0 && bits_eq(msg, out_msg, 21))
            passed++;
        else {
            failed++;
            printf("  FAIL: seed=%u rc=%d\n", seed, rc);
        }
    }

    // Test 6: Chase-2 corrects 3 errors (beyond hard's t=2 limit) when
    // the LCBs are positioned correctly. We flip 3 bits but mark them
    // as "least reliable" by giving them small magnitude soft values.
    // K=3 should find them and decode correctly.
    printf("Test 6: Chase-2 corrects 3-bit errors at LCBs (K=3)\n");
    srand(123);
    for (int trial = 0; trial < 50; trial++) {
        uint8_t msg[21], cw[31], out_msg[21];
        int16_t soft[31];
        random_message(msg, trial + 1);
        bch_encode(msg, cw);
        // Pick 3 distinct positions for errors.
        int errs[3];
        for (int i = 0; i < 3; i++) {
            int p;
            do {
                p = rand() % 31;
            } while (
                (i > 0 && p == errs[0]) ||
                (i > 1 && p == errs[1]));
            errs[i] = p;
        }
        // Build soft: high magnitude where correct, low magnitude at
        // error positions (= LCBs), with sign matching the CORRUPTED bit.
        for (int i = 0; i < 31; i++) {
            int bit = cw[i];
            int err = (i == errs[0] || i == errs[1] || i == errs[2]);
            if (err) bit ^= 1;
            soft[i] = bit ? -50 : +50; // low magnitude = LCB
            if (!err) soft[i] *= 20;   // high magnitude = trustworthy
        }
        int rc = bch_decode_block_soft(soft, out_msg, 3);
        // Hard decode of the corrupted word should FAIL (or mis-decode);
        // Chase-2 should recover.
        if (rc >= 0 && bits_eq(msg, out_msg, 21))
            passed++;
        else {
            failed++;
            printf("  FAIL: trial %d errs=[%d,%d,%d] rc=%d\n",
                   trial, errs[0], errs[1], errs[2], rc);
        }
    }

    // Test 7: Chase-2 with K=4 corrects 4 errors at LCBs.
    printf("Test 7: Chase-2 corrects 4-bit errors at LCBs (K=4)\n");
    srand(456);
    for (int trial = 0; trial < 50; trial++) {
        uint8_t msg[21], cw[31], out_msg[21];
        int16_t soft[31];
        random_message(msg, trial + 100);
        bch_encode(msg, cw);
        int errs[4];
        for (int i = 0; i < 4; i++) {
            int p;
            do {
                p = rand() % 31;
            } while (
                (i > 0 && p == errs[0]) ||
                (i > 1 && p == errs[1]) ||
                (i > 2 && p == errs[2]));
            errs[i] = p;
        }
        for (int i = 0; i < 31; i++) {
            int bit = cw[i];
            int err = (i == errs[0] || i == errs[1] ||
                       i == errs[2] || i == errs[3]);
            if (err) bit ^= 1;
            soft[i] = bit ? -50 : +50;
            if (!err) soft[i] *= 20;
        }
        int rc = bch_decode_block_soft(soft, out_msg, 4);
        if (rc >= 0 && bits_eq(msg, out_msg, 21))
            passed++;
        else {
            failed++;
            printf("  FAIL: trial %d errs=[%d,%d,%d,%d] rc=%d\n",
                   trial, errs[0], errs[1], errs[2], errs[3], rc);
        }
    }

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
