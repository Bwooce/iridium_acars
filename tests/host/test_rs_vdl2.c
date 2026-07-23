// test_rs_vdl2.c — host tests for the VDL Mode 2 RS(255,249) codec
// (common/vdl2/rs_vdl2.c).
//
// Parameters under test (confirmed against dumpvdl2, src/dumpvdl2.c:
// init_rs_char(8, 0x187, 120, 1, RS_N - RS_K, 0), RS_N=255 RS_K=249):
//   GF(2^8) poly 0x187, fcr=120, prim=1, 6 parity roots, t=3.
//
// Cross-validation: the k_genpoly / k_parity_* fixtures below were produced
// by an INDEPENDENT reference implementation (straight-line Python GF(2^8)
// arithmetic with the exact VDL2 field parameters, kept in the test log of
// this change). Any divergence in field poly, fcr, root spacing, or codeword
// byte order makes these fixtures mismatch — this is the "silently wrong
// parameters" guard. dumpvdl2 itself ships no RS unit vectors; with matching
// (gfpoly, fcr, prim, nroots) the parity bytes of ANY correct implementation
// are bit-identical, so fixture equality here is equivalence with dumpvdl2's
// Karn librs configuration.
//
// Coverage:
//   1. generator polynomial fixture (via encode of an impulse)
//   2. parity fixtures for three data patterns
//   3. decode of clean codewords (0 errors, n_corrected == 0)
//   4. round trips at 1/2/3 forced symbol errors — must fully recover
//   5. 4 forced errors — must detect (-1) or, in the small
//      bounded-distance-decoder alias fraction, land on a DIFFERENT valid
//      codeword; must never return "success" with the original data
//      (that would mean it silently accepted >t corruption as clean)
//   6. burst errors (adjacent symbols) and parity-region errors

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "rs_vdl2.h"

static int s_pass = 0, s_fail = 0;
#define CHECK(cond, ...)                                   \
    do {                                                   \
        if (!(cond)) {                                     \
            fprintf(stderr, "  FAIL line %d: ", __LINE__); \
            fprintf(stderr, __VA_ARGS__);                  \
            fprintf(stderr, "\n");                         \
            s_fail++;                                      \
        } else {                                           \
            s_pass++;                                      \
        }                                                  \
    } while (0)

// --- fixtures from the independent GF(2^8)/0x187/fcr=120 reference ---
// g(x) coefficients, g[j] = coeff of x^j (monic, degree 6):
static const uint8_t k_genpoly[7] = { 0x17, 0x82, 0xD9, 0x3E, 0x63, 0xD9, 0x01 };
// parity for data[i] = (i*7 + 3) & 0xFF:
static const uint8_t k_parity_pattern[6] = { 0x8D, 0x45, 0xE1, 0xFB, 0x7C, 0xE6 };
// parity for data[0] = 1, rest 0 (impulse => parity = x^254 mod g(x)):
static const uint8_t k_parity_impulse[6] = { 0xA5, 0x12, 0xD7, 0x76, 0x12, 0x63 };
// parity for data[i] = i & 0xFF:
static const uint8_t k_parity_counting[6] = { 0xB3, 0xAC, 0x2E, 0xD4, 0xA4, 0xD7 };
// Shortened-block transmitted parity (first `fec` octets of the full
// zero-padded encoding), from the same independent reference:
static const uint8_t k_short20_parity[2] = { 0x6E, 0xB8 }; // L=20, data (7i+3), fec=2
static const uint8_t k_short50_parity[4] = { 0x15, 0xD9, 0xB5, 0x84 }; // L=50, data (3i+1), fec=4
static const uint8_t k_short68_parity[6] = { 0xA9, 0x40, 0x7F, 0xDA, 0x89, 0x02 }; // L=68, data (5i+9), fec=6

// Deterministic LCG so every platform runs the identical corpus.
static uint32_t s_rng = 0x1D4A2C6Bu;
static uint32_t rnd(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return s_rng >> 8;
}

static void fill_pattern(uint8_t *block, int variant)
{
    for (int i = 0; i < RS_VDL2_K; i++) {
        switch (variant) {
        case 0:  block[i] = (uint8_t)((i * 7 + 3) & 0xFF); break;
        case 1:  block[i] = (uint8_t)(i & 0xFF);           break;
        default: block[i] = (uint8_t)(rnd() & 0xFF);       break;
        }
    }
    memset(&block[RS_VDL2_K], 0, RS_VDL2_NROOTS);
}

// Corrupt nerr distinct positions with nonzero XOR deltas.
static void corrupt(uint8_t *block, int nerr, int parity_only)
{
    int used[8];
    for (int k = 0; k < nerr; k++) {
        int pos;
        int fresh;
        do {
            pos = parity_only
                    ? RS_VDL2_K + (int)(rnd() % RS_VDL2_NROOTS)
                    : (int)(rnd() % RS_VDL2_N);
            fresh = 1;
            for (int j = 0; j < k; j++)
                if (used[j] == pos)
                    fresh = 0;
        } while (!fresh);
        used[k] = pos;
        uint8_t delta = (uint8_t)(rnd() & 0xFF);
        if (delta == 0)
            delta = 0xA5;
        block[pos] ^= delta;
    }
}

static int fec_for_len(int L)
{
    if (L < 3)  return 0;
    if (L < 31) return 2;
    if (L < 68) return 4;
    return 6;
}

// Build a shortened block per the dumpvdl2 layout into `block`, and write
// the derived erasure positions (untransmitted parity) into eras/n_eras.
// data[0..L-1] carried, [L..248] zero-filled, first `fec` parity octets at
// [249..], remaining parity positions zeroed and marked as erasures.
static void build_shortened(uint8_t *block, const uint8_t *data, int L,
                            uint8_t *eras, int *n_eras)
{
    int fec = fec_for_len(L);
    memset(block, 0, RS_VDL2_N);
    memcpy(block, data, (size_t)L);
    rs_vdl2_encode(block);                // full 6-parity encode over pad
    memset(&block[RS_VDL2_K + fec], 0, (size_t)(RS_VDL2_NROOTS - fec)); // erase tail
    int f = RS_VDL2_NROOTS - fec;
    for (int i = 0; i < f; i++)
        eras[i] = (uint8_t)(RS_VDL2_K + fec + i);
    *n_eras = f;
}

// Corrupt nerr distinct DATA positions [0..L-1] of a shortened block.
static void corrupt_data(uint8_t *block, int L, int nerr)
{
    int used[8];
    for (int k = 0; k < nerr; k++) {
        int pos, fresh;
        do {
            pos = (int)(rnd() % (uint32_t)L);
            fresh = 1;
            for (int j = 0; j < k; j++)
                if (used[j] == pos)
                    fresh = 0;
        } while (!fresh);
        used[k] = pos;
        uint8_t delta = (uint8_t)(rnd() & 0xFF);
        if (delta == 0)
            delta = 0x5C;
        block[pos] ^= delta;
    }
}

int main(void)
{
    uint8_t block[RS_VDL2_N], ref[RS_VDL2_N];
    int nc;

    // 1. Generator polynomial, checked via the impulse codeword: for
    //    data = x^248 (a single 1 followed by zeros... specifically
    //    data[K-1] = 1, rest 0), the codeword is x^6 + parity where the
    //    parity IS g(x) - x^6, i.e. the low 6 generator coefficients.
    memset(block, 0, sizeof(block));
    block[RS_VDL2_K - 1] = 1; // message x^0 => x^6 after the parity shift
    rs_vdl2_encode(block);
    for (int j = 0; j < 6; j++)
        CHECK(block[RS_VDL2_K + j] == k_genpoly[5 - j],
              "genpoly coeff x^%d: got 0x%02X want 0x%02X",
              5 - j, block[RS_VDL2_K + j], k_genpoly[5 - j]);

    // 2. Parity fixtures (independent-reference cross-validation).
    fill_pattern(block, 0);
    rs_vdl2_encode(block);
    CHECK(memcmp(&block[RS_VDL2_K], k_parity_pattern, 6) == 0,
          "parity mismatch on (i*7+3) pattern");

    memset(block, 0, sizeof(block));
    block[0] = 1;
    rs_vdl2_encode(block);
    CHECK(memcmp(&block[RS_VDL2_K], k_parity_impulse, 6) == 0,
          "parity mismatch on impulse pattern");

    fill_pattern(block, 1);
    rs_vdl2_encode(block);
    CHECK(memcmp(&block[RS_VDL2_K], k_parity_counting, 6) == 0,
          "parity mismatch on counting pattern");

    // All-zero data must encode to all-zero parity (linear code).
    memset(block, 0, sizeof(block));
    rs_vdl2_encode(block);
    for (int j = 0; j < 6; j++)
        CHECK(block[RS_VDL2_K + j] == 0, "zero codeword parity[%d] != 0", j);

    // 3. Clean decode: n_corrected == 0, block untouched.
    fill_pattern(block, 0);
    rs_vdl2_encode(block);
    memcpy(ref, block, sizeof(ref));
    nc = -99;
    CHECK(rs_vdl2_decode(block, &nc) == 0, "clean decode failed");
    CHECK(nc == 0, "clean decode n_corrected=%d want 0", nc);
    CHECK(memcmp(block, ref, sizeof(ref)) == 0, "clean decode modified block");

    // NULL n_corrected must be accepted.
    CHECK(rs_vdl2_decode(block, NULL) == 0, "decode with NULL n_corrected");

    // 4. Round trips at 1..3 errors, random data + random positions.
    for (int nerr = 1; nerr <= 3; nerr++) {
        int trials = 400;
        for (int t = 0; t < trials; t++) {
            fill_pattern(ref, 2);
            rs_vdl2_encode(ref);
            memcpy(block, ref, sizeof(block));
            corrupt(block, nerr, 0);
            nc = -99;
            int rc = rs_vdl2_decode(block, &nc);
            CHECK(rc == 0, "%d-error decode failed (trial %d)", nerr, t);
            if (rc == 0) {
                CHECK(nc == nerr, "%d-error trial %d: n_corrected=%d",
                      nerr, t, nc);
                CHECK(memcmp(block, ref, sizeof(ref)) == 0,
                      "%d-error trial %d: wrong data after decode", nerr, t);
            }
        }
    }

    // Errors confined to the parity region must also correct.
    for (int nerr = 1; nerr <= 3; nerr++) {
        fill_pattern(ref, 2);
        rs_vdl2_encode(ref);
        memcpy(block, ref, sizeof(block));
        corrupt(block, nerr, 1);
        CHECK(rs_vdl2_decode(block, &nc) == 0 &&
              memcmp(block, ref, sizeof(ref)) == 0,
              "%d parity-region errors not corrected", nerr);
    }

    // Adjacent-symbol burst of 3 (a common channel shape).
    fill_pattern(ref, 2);
    rs_vdl2_encode(ref);
    memcpy(block, ref, sizeof(block));
    block[100] ^= 0x5A;
    block[101] ^= 0x3C;
    block[102] ^= 0x81;
    CHECK(rs_vdl2_decode(block, &nc) == 0 && nc == 3 &&
          memcmp(block, ref, sizeof(ref)) == 0,
          "3-symbol burst not corrected");

    // 5. Four errors: bounded-distance behaviour. Every trial must either
    //    (a) return -1 (detected), or (b) alias to a DIFFERENT valid
    //    codeword with exactly t corrections (inherent to any RS decoder;
    //    expected fraction of undetected >t patterns is roughly 1/t! ~ 17%).
    //    Returning the ORIGINAL data would be a miracle/bug; returning
    //    success with an invalid word would be a real miscorrection bug.
    {
        int trials = 1000, detected = 0, aliased = 0;
        for (int t = 0; t < trials; t++) {
            fill_pattern(ref, 2);
            rs_vdl2_encode(ref);
            memcpy(block, ref, sizeof(block));
            corrupt(block, 4, 0);
            nc = -99;
            int rc = rs_vdl2_decode(block, &nc);
            if (rc != 0) {
                detected++;
            } else {
                aliased++;
                CHECK(memcmp(block, ref, sizeof(ref)) != 0,
                      "4-error trial %d: decoder claims to have restored "
                      "the original — impossible for a t=3 decoder", t);
                CHECK(nc == 3, "4-error alias trial %d: n_corrected=%d "
                      "(alias must look like exactly t errors)", t, nc);
                // The aliased output must itself be a valid codeword.
                uint8_t chk[RS_VDL2_N];
                memcpy(chk, block, sizeof(chk));
                int nc2;
                CHECK(rs_vdl2_decode(chk, &nc2) == 0 && nc2 == 0,
                      "4-error alias trial %d: output is not a codeword", t);
            }
        }
        printf("  4-error trials: %d detected, %d aliased (of %d)\n",
               detected, aliased, trials);
        CHECK(detected > trials / 2,
              "4-error detection rate too low: %d/%d", detected, trials);
        CHECK(detected + aliased == trials, "trial accounting");
    }

    // 6. 5 and 6 scattered errors: same contract (detect or valid alias),
    //    plus an all-0xFF garbage block, which must never crash.
    for (int nerr = 5; nerr <= 6; nerr++) {
        int trials = 200;
        for (int t = 0; t < trials; t++) {
            fill_pattern(ref, 2);
            rs_vdl2_encode(ref);
            memcpy(block, ref, sizeof(block));
            corrupt(block, nerr, 0);
            int rc = rs_vdl2_decode(block, &nc);
            if (rc == 0)
                CHECK(memcmp(block, ref, sizeof(ref)) != 0,
                      "%d-error trial %d: impossible exact recovery", nerr, t);
            else
                CHECK(rc == -1, "%d-error trial %d: bad return %d", nerr, t, rc);
        }
    }
    memset(block, 0xFF, sizeof(block));
    (void)rs_vdl2_decode(block, &nc); // any verdict is fine; must not crash
    s_pass++;

    // ---- 7. Shortened-block parity fixtures (cross-validation) ----
    // The first `fec` parity octets of the full zero-padded encoding must
    // match the independent reference for each dumpvdl2 length class.
    {
        uint8_t eras[RS_VDL2_NROOTS];
        int ne;
        uint8_t d20[20], d50[50], d68[68];
        for (int i = 0; i < 20; i++) d20[i] = (uint8_t)((i * 7 + 3) & 0xFF);
        for (int i = 0; i < 50; i++) d50[i] = (uint8_t)((i * 3 + 1) & 0xFF);
        for (int i = 0; i < 68; i++) d68[i] = (uint8_t)((i * 5 + 9) & 0xFF);

        build_shortened(block, d20, 20, eras, &ne);
        CHECK(ne == 4, "L=20 erasure count %d want 4", ne);
        CHECK(memcmp(&block[RS_VDL2_K], k_short20_parity, 2) == 0,
              "L=20 (fec=2) parity mismatch vs reference");
        build_shortened(block, d50, 50, eras, &ne);
        CHECK(ne == 2, "L=50 erasure count %d want 2", ne);
        CHECK(memcmp(&block[RS_VDL2_K], k_short50_parity, 4) == 0,
              "L=50 (fec=4) parity mismatch vs reference");
        build_shortened(block, d68, 68, eras, &ne);
        CHECK(ne == 0, "L=68 erasure count %d want 0", ne);
        CHECK(memcmp(&block[RS_VDL2_K], k_short68_parity, 6) == 0,
              "L=68 (fec=6) parity mismatch vs reference");
    }

    // ---- 8. Shortened-block errata round trips ----
    // fec=2 (f=4): corrects up to 1 error. fec=4 (f=2): up to 2 errors.
    struct { int L, budget; } cases[] = { { 20, 1 }, { 50, 2 } };
    for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        int L = cases[c].L, budget = cases[c].budget;

        // 8a. Error-free shortened block: data must be preserved and the
        // decoder must recover the erased (untransmitted, zero-filled)
        // parity tail, yielding a full valid codeword. n_corrected here
        // counts the non-zero erased parity symbols it filled in (<= f),
        // which is expected, not a data correction.
        {
            uint8_t data[68], eras[RS_VDL2_NROOTS];
            uint8_t full[RS_VDL2_N]; // reference: full 6-parity codeword
            int ne;
            for (int i = 0; i < L; i++) data[i] = (uint8_t)(rnd() & 0xFF);
            memset(full, 0, sizeof(full));
            memcpy(full, data, (size_t)L);
            rs_vdl2_encode(full); // all 6 true parity octets present

            build_shortened(block, data, L, eras, &ne);
            nc = -99;
            CHECK(rs_vdl2_decode_erasures(block, eras, ne, &nc) == 0,
                  "L=%d error-free shortened decode failed", L);
            CHECK(nc >= 0 && nc <= ne, "L=%d error-free n_corrected=%d out of range",
                  L, nc);
            CHECK(memcmp(block, full, sizeof(full)) == 0,
                  "L=%d error-free: block != full codeword", L);
            // The convenience wrapper must agree byte-for-byte.
            build_shortened(block, data, L, eras, &ne);
            CHECK(rs_vdl2_decode_shortened(block, L, &nc) == 0 &&
                  memcmp(block, full, sizeof(full)) == 0,
                  "L=%d error-free via wrapper", L);
        }

        // 8b. Errors within budget must fully recover the data region.
        for (int nerr = 1; nerr <= budget; nerr++) {
            int trials = 300;
            for (int t = 0; t < trials; t++) {
                uint8_t data[68], eras[RS_VDL2_NROOTS], ref[RS_VDL2_N];
                int ne;
                for (int i = 0; i < L; i++) data[i] = (uint8_t)(rnd() & 0xFF);
                build_shortened(ref, data, L, eras, &ne);
                memcpy(block, ref, sizeof(block));
                corrupt_data(block, L, nerr);
                nc = -99;
                int rc = rs_vdl2_decode_erasures(block, eras, ne, &nc);
                CHECK(rc == 0, "L=%d %d-err short decode failed (t=%d)",
                      L, nerr, t);
                if (rc == 0)
                    CHECK(memcmp(block, ref, L) == 0,
                          "L=%d %d-err short: data wrong (t=%d)", L, nerr, t);
            }
        }

        // 8c. Exceeding the budget (budget+1 errors) must NOT false-recover:
        // either -1, or a valid-but-different codeword (never the original
        // data claimed as clean).
        {
            int trials = 500, detected = 0, aliased = 0;
            for (int t = 0; t < trials; t++) {
                uint8_t data[68], eras[RS_VDL2_NROOTS], ref[RS_VDL2_N];
                int ne;
                for (int i = 0; i < L; i++) data[i] = (uint8_t)(rnd() & 0xFF);
                build_shortened(ref, data, L, eras, &ne);
                memcpy(block, ref, sizeof(block));
                corrupt_data(block, L, budget + 1);
                nc = -99;
                int rc = rs_vdl2_decode_erasures(block, eras, ne, &nc);
                if (rc != 0) {
                    detected++;
                } else {
                    aliased++;
                    CHECK(memcmp(block, ref, L) != 0,
                          "L=%d over-budget t=%d: impossible exact recovery",
                          L, t);
                }
            }
            printf("  L=%d over-budget (%d errs): %d detected, %d aliased\n",
                   L, budget + 1, detected, aliased);
            CHECK(detected + aliased == trials, "L=%d over-budget accounting", L);
        }
    }

    // ---- 9. Uncoded pass-through (data_len < 3, fec=0) ----
    {
        memset(block, 0xAB, sizeof(block));
        nc = -99;
        CHECK(rs_vdl2_decode_shortened(block, 2, &nc) == 0 && nc == 0,
              "uncoded (L=2) pass-through must succeed with 0 corrected");
        // A confidence erasure on an uncoded block is meaningless (no
        // parity): must be rejected, not silently accepted.
        uint8_t bad = 5;
        CHECK(rs_vdl2_decode_shortened_erasures(block, 2, &bad, 1, &nc) == -1,
              "uncoded confidence-erasure request must be rejected");
    }

    // ---- 10. rs_vdl2_decode_shortened_erasures across all three shortened
    //          schemes: structural + confidence erasure combination. ----
    // For each size: (b) within hard budget corrects via the plain shortened
    // decoder; (c) beyond the hard budget but confidence-recoverable when the
    // extra error positions are supplied as confidence erasures; (d) beyond
    // the combined erasure capacity fails (no false clean recovery); (f) a
    // full encode->shorten->corrupt->recover round trip is byte-exact.
    {
        // {data_len, fec, hard_t, conf_budget, recoverable_errs,
        //  beyond_capacity_errs}
        struct {
            int L, fec, hard_t, budget, rec_errs, cap_errs;
        } sz[] = {
            { 20, 2, 1, 2, 2, 3 },   // 2-parity: f_struct=4
            { 50, 4, 2, 4, 3, 5 },   // 4-parity: f_struct=2
            { 100, 6, 3, 6, 4, 7 },  // 6-parity shortened: f_struct=0
        };
        for (unsigned c = 0; c < sizeof(sz) / sizeof(sz[0]); c++) {
            int L = sz[c].L, fec = sz[c].fec;
            CHECK(fec_for_len(L) == fec, "L=%d expected fec=%d got %d", L, fec,
                  fec_for_len(L));
            int struct_lo = RS_VDL2_K + fec; // structural region begins here

            // Distinct data error positions spread across [0, L).
            int epos[8];
            for (int i = 0; i < 8; i++) epos[i] = (i * 11 + 1) % L;
            // De-dup (spread may collide for small L): re-space if needed.
            for (int i = 1; i < 8; i++)
                for (int j = 0; j < i; j++)
                    if (epos[i] == epos[j]) epos[i] = (epos[i] + 1) % L;

            uint8_t data[128], full[RS_VDL2_N], eras[RS_VDL2_NROOTS];
            int     ne;
            for (int i = 0; i < L; i++) data[i] = (uint8_t)((i * 13 + 7) & 0xFF);
            memset(full, 0, sizeof(full));
            memcpy(full, data, (size_t)L);
            rs_vdl2_encode(full); // reference full 6-parity codeword

            // (b) within hard budget: plain shortened decoder (no confidence).
            for (int nerr = 1; nerr <= sz[c].hard_t; nerr++) {
                build_shortened(block, data, L, eras, &ne);
                for (int e = 0; e < nerr; e++) {
                    uint8_t d = (uint8_t)(0x30 + e);
                    block[epos[e]] ^= d ? d : 0xA5;
                }
                nc = -99;
                int rc = rs_vdl2_decode_shortened(block, L, &nc);
                CHECK(rc == 0, "L=%d hard %d-err decode failed", L, nerr);
                CHECK(memcmp(block, full, sizeof(full)) == 0,
                      "L=%d hard %d-err: block != full codeword", L, nerr);
            }

            // (c) beyond hard budget, confidence-recoverable: supply the extra
            // error positions as confidence erasures.
            {
                int nerr = sz[c].rec_errs;
                CHECK(nerr > sz[c].hard_t, "L=%d test bug: rec %d <= hard %d",
                      L, nerr, sz[c].hard_t);
                build_shortened(block, data, L, eras, &ne);
                uint8_t conf[RS_VDL2_NROOTS];
                for (int e = 0; e < nerr; e++) {
                    uint8_t d = (uint8_t)(0x40 + e);
                    block[epos[e]] ^= d ? d : 0x5C;
                    conf[e] = (uint8_t)epos[e];
                }
                // Sanity: plain shortened (no confidence) must FAIL here (so
                // the recovery is genuinely the confidence fallback's).
                {
                    uint8_t tmp[RS_VDL2_N];
                    memcpy(tmp, block, sizeof(tmp));
                    int rc_hard = rs_vdl2_decode_shortened(tmp, L, &nc);
                    CHECK(rc_hard != 0 || memcmp(tmp, full, sizeof(full)) != 0,
                          "L=%d %d-err: plain shortened unexpectedly clean",
                          L, nerr);
                }
                nc = -99;
                int rc = rs_vdl2_decode_shortened_erasures(block, L, conf,
                                                           nerr, &nc);
                CHECK(rc == 0, "L=%d %d-err confidence recovery failed",
                      L, nerr);
                CHECK(memcmp(block, full, sizeof(full)) == 0,
                      "L=%d %d-err confidence: block != full codeword",
                      L, nerr);
            }

            // (d) beyond combined capacity: must NOT false-recover. Supply as
            // many confidence erasures as the budget allows; the residual
            // errors exceed 2e+f_total<=6, so either -1 or a valid-but-
            // different codeword — never the original data claimed clean.
            {
                int nerr = sz[c].cap_errs;
                build_shortened(block, data, L, eras, &ne);
                uint8_t conf[RS_VDL2_NROOTS];
                int     ncf = sz[c].budget < nerr ? sz[c].budget : nerr;
                for (int e = 0; e < nerr; e++) {
                    uint8_t d = (uint8_t)(0x50 + e);
                    block[epos[e]] ^= d ? d : 0x3C;
                }
                for (int e = 0; e < ncf; e++) conf[e] = (uint8_t)epos[e];
                nc = -99;
                int rc = rs_vdl2_decode_shortened_erasures(block, L, conf,
                                                           ncf, &nc);
                if (rc == 0)
                    CHECK(memcmp(block, full, sizeof(full)) != 0,
                          "L=%d over-capacity (%d err): impossible exact "
                          "recovery", L, nerr);
                else
                    CHECK(rc == -1, "L=%d over-capacity: bad return %d", L, rc);
            }

            // (e) CONFINEMENT contract on the RS entry point: reject a
            // confidence erasure that is not a transmitted symbol, a
            // duplicate, an out-of-range index, or a budget overrun.
            {
                build_shortened(block, data, L, eras, &ne);
                uint8_t p;
                // structural (untransmitted-parity) region -> reject
                if (fec < RS_VDL2_NROOTS) {
                    p = (uint8_t)struct_lo;
                    CHECK(rs_vdl2_decode_shortened_erasures(block, L, &p, 1,
                                                            &nc) == -1,
                          "L=%d: structural-region conf erasure not rejected",
                          L);
                }
                // zero-pad region [L, RS_K) -> reject (known-zero, not sent)
                if (L < RS_VDL2_K) {
                    p = (uint8_t)L;
                    CHECK(rs_vdl2_decode_shortened_erasures(block, L, &p, 1,
                                                            &nc) == -1,
                          "L=%d: zero-pad conf erasure not rejected", L);
                }
                // duplicate confidence positions -> reject
                {
                    uint8_t dup[2] = { (uint8_t)epos[0], (uint8_t)epos[0] };
                    CHECK(rs_vdl2_decode_shortened_erasures(block, L, dup, 2,
                                                            &nc) == -1,
                          "L=%d: duplicate conf erasure not rejected", L);
                }
                // budget overrun (n_conf > conf_budget => f_total > NROOTS)
                {
                    uint8_t many[RS_VDL2_NROOTS + 1];
                    for (int i = 0; i <= sz[c].budget; i++)
                        many[i] = (uint8_t)epos[i]; // budget+1 distinct data pos
                    CHECK(rs_vdl2_decode_shortened_erasures(
                              block, L, many, sz[c].budget + 1, &nc) == -1,
                          "L=%d: budget overrun (%d conf) not rejected", L,
                          sz[c].budget + 1);
                }
                // A transmitted-PARITY confidence erasure IS valid: erasing a
                // corrupted transmitted-parity octet must recover. (parity
                // errors don't touch the data region, so the frame is intact
                // regardless, but this proves the parity range is accepted.)
                {
                    build_shortened(block, data, L, eras, &ne);
                    int ppos = RS_VDL2_K; // first transmitted parity octet
                    block[ppos] ^= 0x9E;
                    p = (uint8_t)ppos;
                    nc = -99;
                    int rc = rs_vdl2_decode_shortened_erasures(block, L, &p, 1,
                                                               &nc);
                    CHECK(rc == 0 && memcmp(block, full, sizeof(full)) == 0,
                          "L=%d: transmitted-parity conf erasure not accepted",
                          L);
                }
            }

            // (f) round trip: encode -> shorten -> corrupt (beyond hard) ->
            // recover via confidence erasures -> byte-exact vs original data.
            {
                int nerr = sz[c].rec_errs;
                build_shortened(block, data, L, eras, &ne);
                uint8_t conf[RS_VDL2_NROOTS];
                for (int e = 0; e < nerr; e++) {
                    block[epos[e]] ^= 0x7B;
                    conf[e] = (uint8_t)epos[e];
                }
                nc = -99;
                int rc = rs_vdl2_decode_shortened_erasures(block, L, conf,
                                                           nerr, &nc);
                CHECK(rc == 0 && memcmp(block, data, (size_t)L) == 0,
                      "L=%d round-trip: data not byte-exact after recovery", L);
            }
        }
    }

    printf("test_rs_vdl2: %d checks passed, %d failed\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
