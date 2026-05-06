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
int iridium_bch_repair1(uint32_t poly, uint8_t *bits, size_t n_bits);

// Try to correct up to two bit errors against `poly`. Brute-force version
// of bch.py:nrepair2 — checks no-flip, all single flips, then all pairs.
// O(n^2) in n_bits but n_bits ≤ 31 in our use, so worst case ~465 ndivide
// calls per check. Used for polys 465 / 1207 / 1897 / 3545 where upstream
// pre-computes 2-error syndromes; the runtime brute-force does fewer
// redundant divisions and avoids carrying a syndrome table around.
int iridium_bch_repair2(uint32_t poly, uint8_t *bits, size_t n_bits);

#ifdef __cplusplus
}
#endif

#endif // IRIDIUM_BCH_H
