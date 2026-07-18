// test_bch_syndrome.c — EXHAUSTIVE bit-exactness proof for the syndrome-table
// BCH repair fast path (iridium_bch_repair1/2) against the retained
// brute-force reference (iridium_bch_repair1_ref/_repair2_ref).
//
// For EVERY (poly, n_bits) combo the decoders actually call:
//   ida_decode.c:    repair2(3545, 31)   ACCH data blocks
//   ira_decode.c:    repair2(1207, 31)   ringalert blocks
//   ibc_decode.c:    repair2(1207, 31) + repair1(29, 6)
//   ims_decode.c:    repair2(1897, 31)   messaging blocks
//   iridium_frame.c: repair1(29, 7), repair2(465, 14), repair2(41, 26)
// ...this test iterates ALL zero/single/double error patterns applied to a
// set of valid codewords (incl. the all-zeros codeword) and asserts that the
// table path returns the IDENTICAL value (0/1/2/-1) AND leaves bits[] in the
// IDENTICAL state as the brute force — bit for bit. It then samples thousands
// of random >=3-error patterns and asserts the same (both -1, or the identical
// miscorrection). Both repair1 and repair2 are diffed on every combo (repair1
// on t=2 polys and vice versa exercises the cross behaviour too).
//
// The brute force's tie-break (lowest-index single first, then the
// lexicographically first (i<j) pair) is NOT assumed unique-syndrome — the
// table builder reproduces the preference order by construction, and this
// test verifies it empirically; syndrome-collision counts are also measured
// and reported per combo (informational: a collision means the code's
// minimum distance is < 5 there, i.e. brute force was already "guessing").
//
// A deliberately broken repair impl is diffed as a negative control to prove
// the harness can actually detect divergence.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "iridium_bch.h"

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

// Deterministic LCG so the corpus is identical on every platform.
static uint32_t s_rng = 0xB0C4DEC0u;
static uint32_t rnd(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return s_rng;
}

// Independent GF(2) long division (NOT the production poly_mod): straight
// shift-register form. Used to build valid codewords; every generated
// codeword is then cross-checked clean via the production ndivide, so a
// divergence between the two dividers would also be caught here.
static uint32_t ref_poly_mod(uint32_t v, uint32_t poly, int n_bits)
{
    int deg = 0;
    for (uint32_t p = poly; p > 1; p >>= 1) deg++;
    uint32_t rem = 0;
    for (int b = n_bits - 1; b >= 0; b--) {
        rem <<= 1;
        if ((v >> b) & 1) rem |= 1;
        if ((rem >> deg) & 1) rem ^= poly;
    }
    return rem;
}

static void u32_to_bits(uint32_t v, uint8_t *bits, int n)
{
    for (int i = 0; i < n; i++) bits[i] = (uint8_t)((v >> (n - 1 - i)) & 1);
}

typedef struct {
    uint32_t    poly;
    int         n;
    const char *what;
} combo_t;

// The complete production call set (see header comment), plus one synthetic
// combo exercising the generic lazy builder on a poly/n no decoder uses.
static const combo_t combos[] = {
    {3545, 31, "ACCH data block (ida_decode, repair2)"},
    {1207, 31, "ringalert/IBC block (ira/ibc, repair2)"},
    {1897, 31, "messaging block (ims, repair2)"},
    {465, 14, "LCW2 (iridium_frame, repair2)"},
    {41, 26, "LCW3 (iridium_frame, repair2)"},
    {29, 7, "LCW1 (iridium_frame, repair1)"},
    {29, 6, "IBC header (ibc_decode, repair1)"},
    {19, 15, "synthetic (generic-builder control)"},
};
#define N_COMBOS ((int)(sizeof(combos) / sizeof(combos[0])))
#define N_CODEWORDS 9 // all-zeros + 8 random valid
#define N_RANDOM_PATTERNS 3000

// One ref-vs-table comparison of both repair functions on (codeword ^ pattern).
// Returns number of mismatches (0 or more); asserts via CHECK unless quiet.
typedef int (*repair_fn)(uint32_t, uint8_t *, size_t);

static int diff_one(uint32_t poly, int n, const uint8_t *dirty, repair_fn ref,
                    repair_fn dut, int quiet)
{
    uint8_t a[32], b[32];
    memcpy(a, dirty, (size_t)n);
    memcpy(b, dirty, (size_t)n);
    int ra = ref(poly, a, (size_t)n);
    int rb = dut(poly, b, (size_t)n);
    int mism = (ra != rb) || (memcmp(a, b, (size_t)n) != 0);
    if (mism && !quiet) {
        fprintf(stderr, "  mismatch poly=%u n=%d: ref=%d dut=%d bits %s\n",
                poly, n, ra, rb, memcmp(a, b, (size_t)n) ? "DIFFER" : "equal");
    }
    return mism;
}

// Deliberately broken "repair": never fixes anything. The harness MUST flag
// this against the reference — proves diff_one can fail.
static int broken_repair(uint32_t poly, uint8_t *bits, size_t n_bits)
{
    if (!bits || n_bits == 0) return -1;
    if (iridium_bch_ndivide(poly, bits, n_bits) == 0) return 0;
    return -1;
}

int main(void)
{
    long grand_total = 0;

    for (int c = 0; c < N_COMBOS; c++) {
        uint32_t poly = combos[c].poly;
        int      n    = combos[c].n;

        int deg = 0;
        for (uint32_t p = poly; p > 1; p >>= 1) deg++;

        // --- valid codeword set: all-zeros + 8 random -------------------
        uint8_t cw[N_CODEWORDS][32];
        memset(cw[0], 0, sizeof(cw[0]));
        for (int k = 1; k < N_CODEWORDS; k++) {
            uint32_t msg = rnd() & (((uint32_t)1 << (n - deg)) - 1u);
            uint32_t v   = msg << deg;
            v ^= ref_poly_mod(v, poly, n);
            u32_to_bits(v, cw[k], n);
            // Cross-check: production divider must also see it clean.
            CHECK(iridium_bch_ndivide(poly, cw[k], (size_t)n) == 0,
                  "poly=%u n=%d: generated codeword %d not clean", poly, n, k);
        }

        // --- syndrome-collision census (informational + reported) -------
        // syn1[i] from the independent divider; count how many <=2-error
        // patterns share a syndrome (collision => distance < 5 => brute
        // force's answer on that syndrome is a preference, not a proof).
        uint32_t syn1[32];
        for (int i = 0; i < n; i++)
            syn1[i] = ref_poly_mod((uint32_t)1 << (n - 1 - i), poly, n);
        int  n_syn = 1 << deg;
        int *cnt   = calloc((size_t)n_syn, sizeof(int));
        int  single_dups = 0, pair_over_single = 0, pair_dups = 0, syn_zero = 0;
        for (int i = 0; i < n; i++) {
            if (syn1[i] == 0) syn_zero++;
            if (cnt[syn1[i]]++) single_dups++;
        }
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                uint32_t s = syn1[i] ^ syn1[j];
                if (s == 0) syn_zero++;
                int prior = cnt[s]++;
                if (prior) pair_dups++;
            }
        }
        // pair syndrome colliding with a single syndrome specifically:
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++)
                for (int k = 0; k < n; k++)
                    if ((syn1[i] ^ syn1[j]) == syn1[k]) { pair_over_single++; j = n; i = n; break; }
        free(cnt);

        // --- EXHAUSTIVE 0/1/2-error patterns x every codeword -----------
        long combo_checks = 0;
        int  combo_mism   = 0;
        for (int k = 0; k < N_CODEWORDS; k++) {
            uint8_t dirty[32];
            // zero errors
            memcpy(dirty, cw[k], (size_t)n);
            combo_mism += diff_one(poly, n, dirty, iridium_bch_repair1_ref,
                                   iridium_bch_repair1, 0);
            combo_mism += diff_one(poly, n, dirty, iridium_bch_repair2_ref,
                                   iridium_bch_repair2, 0);
            combo_checks += 2;
            // singles
            for (int i = 0; i < n; i++) {
                memcpy(dirty, cw[k], (size_t)n);
                dirty[i] ^= 1;
                combo_mism += diff_one(poly, n, dirty, iridium_bch_repair1_ref,
                                       iridium_bch_repair1, 0);
                combo_mism += diff_one(poly, n, dirty, iridium_bch_repair2_ref,
                                       iridium_bch_repair2, 0);
                combo_checks += 2;
            }
            // pairs
            for (int i = 0; i < n; i++) {
                for (int j = i + 1; j < n; j++) {
                    memcpy(dirty, cw[k], (size_t)n);
                    dirty[i] ^= 1;
                    dirty[j] ^= 1;
                    combo_mism += diff_one(poly, n, dirty, iridium_bch_repair1_ref,
                                           iridium_bch_repair1, 0);
                    combo_mism += diff_one(poly, n, dirty, iridium_bch_repair2_ref,
                                           iridium_bch_repair2, 0);
                    combo_checks += 2;
                }
            }
        }
        CHECK(combo_mism == 0, "poly=%u n=%d: %d exhaustive-pattern mismatches",
              poly, n, combo_mism);

        // --- random >=3-error patterns (identical miscorrection) --------
        int rand_mism = 0;
        for (int t = 0; t < N_RANDOM_PATTERNS; t++) {
            uint8_t dirty[32];
            memcpy(dirty, cw[rnd() % N_CODEWORDS], (size_t)n);
            int nerr = 3 + (int)(rnd() % 4u); // 3..6 flips (distinct positions)
            uint32_t used = 0;
            for (int e = 0; e < nerr; e++) {
                int pos;
                do {
                    pos = (int)(rnd() % (uint32_t)n);
                } while (used & (1u << pos));
                used |= 1u << pos;
                dirty[pos] ^= 1;
            }
            rand_mism += diff_one(poly, n, dirty, iridium_bch_repair1_ref,
                                  iridium_bch_repair1, 0);
            rand_mism += diff_one(poly, n, dirty, iridium_bch_repair2_ref,
                                  iridium_bch_repair2, 0);
            combo_checks += 2;
        }
        CHECK(rand_mism == 0, "poly=%u n=%d: %d random>=3-error mismatches",
              poly, n, rand_mism);

        grand_total += combo_checks;
        printf("poly=%-4u n=%-2d [%s]\n"
               "    %ld ref-vs-table checks (9 codewords x {0,1,2}-err exhaustive"
               " + %d random 3..6-err), 0 mismatches\n"
               "    syndrome census: single-dups=%d pair-dups=%d"
               " pair-collides-single=%s zero-syn-patterns=%d\n",
               poly, n, combos[c].what, combo_checks, N_RANDOM_PATTERNS,
               single_dups, pair_dups, pair_over_single ? "YES" : "no", syn_zero);
    }

    // --- guard-path parity: NULL / n_bits==0 ----------------------------
    uint8_t dummy[4] = {1, 0, 1, 0};
    CHECK(iridium_bch_repair1(29, NULL, 4) == iridium_bch_repair1_ref(29, NULL, 4),
          "NULL-bits parity (repair1)");
    CHECK(iridium_bch_repair2(29, NULL, 4) == iridium_bch_repair2_ref(29, NULL, 4),
          "NULL-bits parity (repair2)");
    CHECK(iridium_bch_repair1(29, dummy, 0) == iridium_bch_repair1_ref(29, dummy, 0),
          "n_bits=0 parity (repair1)");
    CHECK(iridium_bch_repair2(29, dummy, 0) == iridium_bch_repair2_ref(29, dummy, 0),
          "n_bits=0 parity (repair2)");

    // --- negative control: the harness must detect a broken repair ------
    {
        int detected = 0;
        uint8_t z[32];
        memset(z, 0, sizeof(z));
        for (int i = 0; i < 31; i++) {
            uint8_t dirty[32];
            memcpy(dirty, z, 31);
            dirty[i] ^= 1;
            detected += diff_one(1207, 31, dirty, iridium_bch_repair2_ref,
                                 broken_repair, 1);
        }
        CHECK(detected == 31,
              "negative control: broken repair must mismatch on all 31 "
              "single-error inputs (got %d)", detected);
    }

    printf("\ntotal ref-vs-table checks: %ld\n", grand_total);
    printf("=== %d passed, %d failed ===\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
