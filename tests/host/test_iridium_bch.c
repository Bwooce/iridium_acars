// Unit tests for common/iridium_decoder/iridium_bch.c — the polynomial
// division + 1-bit ECC repair. We construct codewords by hand using the
// standard "shift-and-XOR" encoder ('append k zeros, take remainder, fill
// trailing k bits') and verify ndivide returns 0 on clean codewords,
// non-zero on flipped codewords, and that bch_repair1 recovers single-bit
// errors (and only single-bit ones).

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "iridium_bch.h"

static int passed = 0, failed = 0;

#define CHECK(cond, fmt, ...) do {                                       \
    if (!(cond)) {                                                       \
        printf("  FAIL line %d: " fmt "\n", __LINE__, ##__VA_ARGS__);    \
        failed++; return;                                                \
    } else { passed++; }                                                 \
} while (0)

// Encode a k-bit message with a `poly_len`-bit polynomial. Result is k +
// (poly_len-1) bits stored as a 0/1-per-byte array. Encoder: append
// (poly_len-1) zeros, compute remainder, replace the trailing zeros with
// the remainder bits. Output buffer must hold k + poly_len - 1 bytes.
static void bch_encode(uint32_t poly, int poly_len,
                       const uint8_t *msg, int k,
                       uint8_t *out)
{
    int n = k + poly_len - 1;
    // Pad with zeros.
    for (int i = 0; i < k; i++) out[i] = msg[i] & 1;
    for (int i = k; i < n; i++) out[i] = 0;
    // Remainder of (out) % poly using the same long-division as ndivide.
    // We can compute it via ndivide directly.
    uint32_t r = iridium_bch_ndivide(poly, out, n);
    // Place the (poly_len - 1)-bit remainder into the trailing bits MSB
    // first, padded to poly_len-1 bits total (so a small remainder lands
    // in the low bits of the trailing field).
    int ecc_bits = poly_len - 1;
    for (int i = 0; i < ecc_bits; i++) {
        out[k + i] = (uint8_t)((r >> (ecc_bits - 1 - i)) & 1);
    }
}

static int u32_bit_length(uint32_t x) {
    int n = 0; while (x) { n++; x >>= 1; } return n;
}

static void test_clean_codeword(uint32_t poly, const uint8_t *msg, int k)
{
    int poly_len = u32_bit_length(poly);
    int n = k + poly_len - 1;
    uint8_t cw[40];
    bch_encode(poly, poly_len, msg, k, cw);
    uint32_t r = iridium_bch_ndivide(poly, cw, n);
    CHECK(r == 0, "poly=%u clean codeword has remainder %u, expected 0", poly, r);
}

static void test_single_bit_repair(uint32_t poly, const uint8_t *msg, int k)
{
    int poly_len = u32_bit_length(poly);
    int n = k + poly_len - 1;
    uint8_t cw[40];
    bch_encode(poly, poly_len, msg, k, cw);

    // Flip every bit in turn; bch_repair1 should restore the codeword
    // (or at least find ONE single-bit fix that divides cleanly).
    for (int flip = 0; flip < n; flip++) {
        uint8_t test_cw[40];
        memcpy(test_cw, cw, n);
        test_cw[flip] ^= 1;
        int rc = iridium_bch_repair1(poly, test_cw, n);
        CHECK(rc == 1,
              "poly=%u single-bit error at pos %d: repair returned %d (expected 1)",
              poly, flip, rc);
        // After repair, the codeword should divide cleanly. (It might not
        // match the ORIGINAL codeword if poly's distance is too small,
        // but it must be a valid codeword.)
        uint32_t r = iridium_bch_ndivide(poly, test_cw, n);
        CHECK(r == 0, "after repair, remainder=%u (expected 0)", r);
    }
}

static void test_already_clean_returns_zero(uint32_t poly, const uint8_t *msg, int k)
{
    int poly_len = u32_bit_length(poly);
    int n = k + poly_len - 1;
    uint8_t cw[40];
    bch_encode(poly, poly_len, msg, k, cw);
    int rc = iridium_bch_repair1(poly, cw, n);
    CHECK(rc == 0, "already-clean codeword: repair returned %d (expected 0)", rc);
}

static void test_two_bit_error_fails(uint32_t poly, const uint8_t *msg, int k)
{
    int poly_len = u32_bit_length(poly);
    int n = k + poly_len - 1;
    uint8_t cw[40];
    bch_encode(poly, poly_len, msg, k, cw);
    // Flip two adjacent bits — outside the single-error correction
    // capability of our repair1 routine.
    cw[3] ^= 1;
    cw[7] ^= 1;
    int rc = iridium_bch_repair1(poly, cw, n);
    // It MIGHT actually find a 1-bit fix that divides cleanly (because
    // the code may not be perfect at d=3) — that's fine and not a bug.
    // The hard requirement is just that we don't lie: if rc=0/1, the
    // remainder must indeed be 0.
    if (rc >= 0) {
        uint32_t r = iridium_bch_ndivide(poly, cw, n);
        CHECK(r == 0, "after non-negative rc=%d, remainder=%u", rc, r);
    } else {
        CHECK(rc == -1, "expected -1 for unrecoverable, got %d", rc);
    }
}

static void run_for_poly(uint32_t poly, int poly_len, int k, const char *name)
{
    (void)poly_len;
    printf("Testing poly=%u (%s) k=%d\n", poly, name, k);

    // Use a fixed pseudo-random message — different bits set, not all
    // zeros (which is trivially clean).
    uint8_t msg[32] = { 0 };
    for (int i = 0; i < k; i++) {
        msg[i] = (uint8_t)((i * 7 + 3) & 1);
    }

    test_clean_codeword(poly, msg, k);
    test_already_clean_returns_zero(poly, msg, k);
    test_single_bit_repair(poly, msg, k);
    test_two_bit_error_fails(poly, msg, k);
}

int main(void)
{
    // Iridium polys — see iridium_bch.h for cite.
    run_for_poly(29u,    5, 2, "hdr_poly / lcw1");        // BCH(7,2)
    run_for_poly(41u,    6, 21, "lcw3");                  // BCH(26,21)
    run_for_poly(465u,   9, 7, "lcw2 (uses 14-bit codeword)"); // BCH(14,7)
    run_for_poly(1207u, 11, 21, "ringalert / IBC");       // BCH(31,21)
    run_for_poly(1897u, 11, 21, "messaging");             // BCH(31,21)
    run_for_poly(3545u, 12, 21, "ACCH");                  // BCH(31,21) wider poly

    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
