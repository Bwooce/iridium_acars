// Synthetic-frame unit tests for Phase B (BC + LW dispatch).
//
// Builds bit streams by hand that should classify as BC and as each
// LW subtype, plus negative cases (mangled BCH, near-miss LW). Uses
// the same iridium_bch_ndivide / repair primitives as the production
// classifier — the goal is to verify the dispatch order, the
// symbol_reverse step, and the LCW-table indexing, NOT to re-test
// BCH correctness (test_iridium_bch covers that).

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "iridium_frame.h"
#include "iridium_bch.h"

static int passed = 0, failed = 0;

#define CHECK(cond, fmt, ...)                                             \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("  FAIL line %d: " fmt "\n", __LINE__, ##__VA_ARGS__); \
            failed++;                                                     \
            return;                                                       \
        } else {                                                          \
            passed++;                                                     \
        }                                                                 \
    } while (0)

#define UW_LEN 24
static const uint8_t UW_DL_BITS[UW_LEN] = {
    0,
    0,
    1,
    1,
    0,
    0,
    0,
    0,
    0,
    0,
    1,
    1,
    0,
    0,
    0,
    0,
    1,
    1,
    1,
    1,
    0,
    0,
    1,
    1,
};

// Helper: pair-swap len bytes in-place (the inverse of what the
// classifier does internally — used here to construct test inputs that,
// after the classifier's swap, yield specific de-interleaved bits).
static void pair_swap(uint8_t *bits, size_t len)
{
    for (size_t i = 0; i + 1 < len; i += 2) {
        uint8_t t   = bits[i];
        bits[i]     = bits[i + 1];
        bits[i + 1] = t;
    }
}

// BCH-encode a k-bit message into a (k + poly_len-1)-bit codeword in
// the supplied output buffer. Same shift-and-XOR encoder as
// test_iridium_bch.c.
static int u32_bit_length(uint32_t x)
{
    int n = 0;
    while (x) {
        n++;
        x >>= 1;
    }
    return n;
}
static void bch_encode(uint32_t poly, const uint8_t *msg, int k, uint8_t *out)
{
    int poly_len = u32_bit_length(poly);
    int n        = k + poly_len - 1;
    for (int i = 0; i < k; i++)
        out[i] = msg[i] & 1;
    for (int i = k; i < n; i++)
        out[i] = 0;
    uint32_t r = iridium_bch_ndivide(poly, out, n);
    for (int i = 0; i < poly_len - 1; i++) {
        out[k + i] = (uint8_t)((r >> (poly_len - 2 - i)) & 1);
    }
}

// --- BC test: synthesize a frame whose 70 post-UW bits encode a valid
//     IBC structure (6-bit hdr clean against poly=29, then 64 bits
//     forming an interleaved double BCH(31,21) under poly=1207).
static void test_synthetic_bc(void)
{
    printf("Test: synthetic BC frame -> IR_FRAME_BC\n");
    uint8_t bits[UW_LEN + 70];
    memcpy(bits, UW_DL_BITS, UW_LEN);

    // Build the post-UW payload in the orientation upstream sees AFTER
    // pair-swap, then pair-swap once before passing to the classifier
    // (which will swap again, giving the upstream view).
    uint8_t payload[70];
    memset(payload, 0, sizeof(payload));

    // Bits 0..5: 6-bit IBC header that divides cleanly under poly=29.
    // The simplest such codeword is all zeros.
    // (already zeroed above)

    // Bits 6..69: 64-bit double-BCH block. The de_interleave we want to
    // produce two 31-bit codewords clean under poly=1207. Simplest:
    // make the entire 64 bits zero — both halves are all-zero, which
    // trivially divide cleanly under any poly.
    // (already zeroed above)

    // Pair-swap the payload BEFORE inserting into the post-UW region,
    // because the classifier will re-swap it back.
    pair_swap(payload, sizeof(payload));
    memcpy(bits + UW_LEN, payload, sizeof(payload));

    iridium_frame_t f  = {0};
    int             rc = iridium_frame_classify(bits, sizeof(bits),
                                                IR_FRM_DIR_DOWNLINK, &f);
    CHECK(rc == 0, "rc=%d", rc);
    CHECK(f.type == IR_FRAME_BC, "type=%s (expected BC)",
          iridium_frame_type_name(f.type));
}

// --- LW test: synthesize a frame whose 46 post-UW bits encode a valid
//     LCW (3-bit ft, 6-bit lcw2-msg, 21-bit lcw3-msg, all BCH-clean)
//     and check we get the right LW subtype.
static void test_synthetic_lw_da(void)
{
    printf("Test: synthetic LW with ft=2 (DA / SBD) -> IR_FRAME_LW + IR_LW_DA\n");

    // Build the THREE LCW codewords first (in upstream's orientation).
    uint8_t lcw1_msg[3]  = {0, 1, 0};          // ft=2 (= 010 binary)
    uint8_t lcw2_msg[6]  = {0, 1, 0, 1, 0, 1}; // arbitrary 6-bit
    uint8_t lcw3_msg[21] = {0};                // arbitrary 21-bit
    for (int i = 0; i < 21; i++)
        lcw3_msg[i] = (uint8_t)((i * 5 + 1) & 1);

    uint8_t lcw1_cw[7];  // 3 + 4 = 7 bits, poly=29 (5-bit)
    uint8_t lcw2_cw[14]; // 6 + 8 = 14 bits, poly=465 (9-bit)
    uint8_t lcw3_cw[26]; // 21 + 5 = 26 bits, poly=41 (6-bit)
    bch_encode(29u, lcw1_msg, 3, lcw1_cw);
    bch_encode(465u, lcw2_msg, 6, lcw2_cw);
    bch_encode(41u, lcw3_msg, 21, lcw3_cw);

    // Verify they divide cleanly (sanity).
    if (iridium_bch_ndivide(29u, lcw1_cw, 7) != 0) {
        printf("  FAIL: encoded lcw1 doesn't divide poly=29\n");
        failed++;
        return;
    }
    if (iridium_bch_ndivide(465u, lcw2_cw, 14) != 0) {
        printf("  FAIL: encoded lcw2 doesn't divide poly=465\n");
        failed++;
        return;
    }
    if (iridium_bch_ndivide(41u, lcw3_cw, 26) != 0) {
        printf("  FAIL: encoded lcw3 doesn't divide poly=41\n");
        failed++;
        return;
    }

    // Place them into a 46-element 'permuted' array per the upstream
    // de_interleave_lcw split:
    //   permuted[0..6]   = lcw1
    //   permuted[7..19]  = lcw2 first 13 bits  (pad bit at 13 dropped)
    //   permuted[20..45] = lcw3
    uint8_t permuted[46] = {0};
    memcpy(permuted + 0, lcw1_cw, 7);
    memcpy(permuted + 7, lcw2_cw, 13); // 13 of 14 — drop the trailing pad
    memcpy(permuted + 20, lcw3_cw, 26);

    // Apply the inverse of the classifier's permutation. Forward map:
    //   permuted[i] = data[LCW_TBL[i]]
    // Inverse:
    //   data[LCW_TBL[i]] = permuted[i]
    static const uint8_t LCW_TBL[46] = {
        39,
        38,
        35,
        34,
        31,
        30,
        27,
        26,
        23,
        22,
        19,
        18,
        15,
        14,
        11,
        10,
        7,
        6,
        3,
        2,
        40,
        37,
        36,
        33,
        32,
        29,
        28,
        25,
        24,
        21,
        20,
        17,
        16,
        13,
        12,
        9,
        8,
        5,
        4,
        1,
        0,
        45,
        44,
        43,
        42,
        41,
    };
    uint8_t payload[46] = {0};
    for (int i = 0; i < 46; i++) {
        payload[LCW_TBL[i]] = permuted[i];
    }

    // Pair-swap so the classifier's internal swap recovers our payload.
    pair_swap(payload, sizeof(payload));

    uint8_t bits[UW_LEN + 46 + 32]; // extra tail = required min size
    memcpy(bits, UW_DL_BITS, UW_LEN);
    memcpy(bits + UW_LEN, payload, 46);
    memset(bits + UW_LEN + 46, 0, 32); // post-LCW tail

    iridium_frame_t f  = {0};
    int             rc = iridium_frame_classify(bits, sizeof(bits),
                                                IR_FRM_DIR_DOWNLINK, &f);
    CHECK(rc == 0, "rc=%d", rc);
    CHECK(f.type == IR_FRAME_LW, "type=%s (expected LW)",
          iridium_frame_type_name(f.type));
    CHECK(f.lw_subtype == IR_LW_DA, "lw_subtype=%s (expected DA)",
          iridium_lw_subtype_name(f.lw_subtype));
}

// --- LW test: ft=7 (SY) variant
static void test_synthetic_lw_sy(void)
{
    printf("Test: synthetic LW with ft=7 (SY / sync) -> IR_LW_SY\n");
    uint8_t lcw1_msg[3]  = {1, 1, 1}; // ft=7 (= 111 binary)
    uint8_t lcw2_msg[6]  = {0, 1, 0, 1, 0, 1};
    uint8_t lcw3_msg[21] = {0};
    for (int i = 0; i < 21; i++)
        lcw3_msg[i] = (uint8_t)((i * 5 + 1) & 1);

    uint8_t lcw1_cw[7], lcw2_cw[14], lcw3_cw[26];
    bch_encode(29u, lcw1_msg, 3, lcw1_cw);
    bch_encode(465u, lcw2_msg, 6, lcw2_cw);
    bch_encode(41u, lcw3_msg, 21, lcw3_cw);

    uint8_t permuted[46] = {0};
    memcpy(permuted + 0, lcw1_cw, 7);
    memcpy(permuted + 7, lcw2_cw, 13);
    memcpy(permuted + 20, lcw3_cw, 26);

    static const uint8_t LCW_TBL[46] = {
        39,
        38,
        35,
        34,
        31,
        30,
        27,
        26,
        23,
        22,
        19,
        18,
        15,
        14,
        11,
        10,
        7,
        6,
        3,
        2,
        40,
        37,
        36,
        33,
        32,
        29,
        28,
        25,
        24,
        21,
        20,
        17,
        16,
        13,
        12,
        9,
        8,
        5,
        4,
        1,
        0,
        45,
        44,
        43,
        42,
        41,
    };
    uint8_t payload[46] = {0};
    for (int i = 0; i < 46; i++)
        payload[LCW_TBL[i]] = permuted[i];
    pair_swap(payload, sizeof(payload));

    uint8_t bits[UW_LEN + 46 + 32];
    memcpy(bits, UW_DL_BITS, UW_LEN);
    memcpy(bits + UW_LEN, payload, 46);
    memset(bits + UW_LEN + 46, 0, 32);

    iridium_frame_t f  = {0};
    int             rc = iridium_frame_classify(bits, sizeof(bits),
                                                IR_FRM_DIR_DOWNLINK, &f);
    CHECK(rc == 0, "rc=%d", rc);
    CHECK(f.type == IR_FRAME_LW, "type=%s (expected LW)",
          iridium_frame_type_name(f.type));
    CHECK(f.lw_subtype == IR_LW_SY, "lw_subtype=%s (expected SY)",
          iridium_lw_subtype_name(f.lw_subtype));
}

// --- Negative: mangled LW, lcw1 corrupted beyond 1-bit repair → NOT classified.
static void test_synthetic_lw_corrupted_lcw1(void)
{
    printf("Test: LW frame with lcw1 corrupted -> not classified as LW\n");
    uint8_t lcw1_msg[3]  = {0, 1, 0};
    uint8_t lcw2_msg[6]  = {0, 0, 0, 0, 0, 0};
    uint8_t lcw3_msg[21] = {0};

    uint8_t lcw1_cw[7], lcw2_cw[14], lcw3_cw[26];
    bch_encode(29u, lcw1_msg, 3, lcw1_cw);
    bch_encode(465u, lcw2_msg, 6, lcw2_cw);
    bch_encode(41u, lcw3_msg, 21, lcw3_cw);

    // Flip 2 bits in lcw1 — beyond 1-bit-repair budget AND we use
    // strict ndivide (no repair) per upstream non-harder mode.
    lcw1_cw[1] ^= 1;
    lcw1_cw[5] ^= 1;

    uint8_t permuted[46] = {0};
    memcpy(permuted + 0, lcw1_cw, 7);
    memcpy(permuted + 7, lcw2_cw, 13);
    memcpy(permuted + 20, lcw3_cw, 26);

    static const uint8_t LCW_TBL[46] = {
        39,
        38,
        35,
        34,
        31,
        30,
        27,
        26,
        23,
        22,
        19,
        18,
        15,
        14,
        11,
        10,
        7,
        6,
        3,
        2,
        40,
        37,
        36,
        33,
        32,
        29,
        28,
        25,
        24,
        21,
        20,
        17,
        16,
        13,
        12,
        9,
        8,
        5,
        4,
        1,
        0,
        45,
        44,
        43,
        42,
        41,
    };
    uint8_t payload[46] = {0};
    for (int i = 0; i < 46; i++)
        payload[LCW_TBL[i]] = permuted[i];
    pair_swap(payload, sizeof(payload));

    uint8_t bits[UW_LEN + 46 + 32];
    memcpy(bits, UW_DL_BITS, UW_LEN);
    memcpy(bits + UW_LEN, payload, 46);
    memset(bits + UW_LEN + 46, 0, 32);

    iridium_frame_t f  = {0};
    int             rc = iridium_frame_classify(bits, sizeof(bits),
                                                IR_FRM_DIR_DOWNLINK, &f);
    CHECK(rc == 0, "rc=%d", rc);
    CHECK(f.type != IR_FRAME_LW, "expected NOT LW, got %s",
          iridium_frame_type_name(f.type));
}

int main(void)
{
    test_synthetic_bc();
    test_synthetic_lw_da();
    test_synthetic_lw_sy();
    test_synthetic_lw_corrupted_lcw1();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
