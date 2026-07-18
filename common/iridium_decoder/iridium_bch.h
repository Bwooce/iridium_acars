#ifndef IRIDIUM_BCH_H
#define IRIDIUM_BCH_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bitwise polynomial division in GF(2). `bits` is a 0/1-per-byte array of
// length `n_bits`. Returns the remainder of the polynomial represented by
// the bits divided by `poly`. Mirrors iridium-toolkit/bch.py:nndivide.
//
// Iridium uses these polynomials at various places in the frame
// hierarchy (per iridium-toolkit/bitsparser.py constants):
//   poly =   29  IBC header CRC          (5-bit)  — BCH(7,2)
//   poly =   41  LCW middle word         (6-bit)  — BCH(26,21)
//   poly =  465  LCW left word           (9-bit)  — BCH(14,7)
//   poly = 1207  ringalert / IBC blocks  (11-bit) — BCH(31,21)
//   poly = 1897  messaging blocks        (11-bit) — BCH(31,21)
//   poly = 3545  ACCH blocks             (12-bit) — BCH(31,21)
uint32_t iridium_bch_ndivide(uint32_t poly, const uint8_t *bits, size_t n_bits);

// Try to correct up to one bit error against `poly`. On success, writes the
// corrected codeword back into `bits[0..n_bits)` (in-place flip of one bit
// or none) and returns 0 (no error) / 1 (one error corrected). Returns -1
// if no single-bit flip yields a clean division.
//
// This is the C equivalent of iridium-toolkit/bch.py:nrepair1.
// Implementation: O(1) syndrome-table lookup (lazy-built per (poly, n_bits),
// ~2 KB max per combo in PSRAM on-device), proven bit-exact against the
// brute-force reference below by tests/host/test_bch_syndrome.c.
int iridium_bch_repair1(uint32_t poly, uint8_t *bits, size_t n_bits);

// Try to correct up to two bit errors against `poly` — C equivalent of
// bch.py:nrepair2 (checks no-flip, then singles, then pairs, preferring
// the lowest-index single and then the lexicographically first pair).
// Implementation: same syndrome-table lookup as repair1 (one division +
// one probe instead of the O(n^2) ≈ 465-division brute force). Used for
// polys 465 / 1207 / 1897 / 3545. Bit-exact with the reference below —
// including identical MIScorrections on >2-error inputs (exhaustively
// tested; see test_bch_syndrome.c).
int iridium_bch_repair2(uint32_t poly, uint8_t *bits, size_t n_bits);

// Brute-force reference implementations (the original production code,
// unchanged): flip each bit / each (i < j) pair and re-divide. Retained
// as the behavioural ground truth for test_bch_syndrome's exhaustive
// table-vs-reference diff, and as the runtime fallback for any
// (poly, n_bits) the table registry can't host (n_bits > 31, arena/slot
// exhaustion). Semantics identical to iridium_bch_repair1/2.
int iridium_bch_repair1_ref(uint32_t poly, uint8_t *bits, size_t n_bits);
int iridium_bch_repair2_ref(uint32_t poly, uint8_t *bits, size_t n_bits);

#ifdef __cplusplus
}
#endif

#endif // IRIDIUM_BCH_H
