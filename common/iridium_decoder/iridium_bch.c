// BCH polynomial-division helpers used by the Iridium frame classifier.
// Direct C port of iridium-toolkit/bch.py {nndivide, nrepair1}.

#include "iridium_bch.h"

// Convert a 0/1-per-byte bit array into a uint32_t (MSB-first, the upstream
// convention from int(bits, 2)). Limited to n_bits ≤ 32 — the polynomials
// we operate on top out at 12 bits, the largest payload is the 31-bit BCH
// codeword for poly=1207/1897/3545.
static uint32_t bits_to_u32(const uint8_t *bits, size_t n_bits)
{
    uint32_t v = 0;
    for (size_t i = 0; i < n_bits; i++) {
        v = (v << 1) | (bits[i] & 1);
    }
    return v;
}

// Number of bits in `x` (highest bit position + 1, with bit_length(0) = 0
// matching Python int.bit_length).
static int u32_bit_length(uint32_t x)
{
    int n = 0;
    while (x) { n++; x >>= 1; }
    return n;
}

uint32_t iridium_bch_ndivide(uint32_t poly, const uint8_t *bits, size_t n_bits)
{
    if (!bits || n_bits == 0) return 0;
    uint32_t num = bits_to_u32(bits, n_bits);
    if (num == 0) return 0;

    int num_len = u32_bit_length(num);
    int poly_len = u32_bit_length(poly);
    int shift = num_len - poly_len;
    uint32_t pow = (uint32_t)1 << (num_len - 1);

    while (shift >= 0) {
        if (num >= pow) {
            num ^= (poly << shift);
        }
        pow >>= 1;
        shift--;
    }
    return num;
}

int iridium_bch_repair1(uint32_t poly, uint8_t *bits, size_t n_bits)
{
    if (!bits || n_bits == 0) return -1;

    if (iridium_bch_ndivide(poly, bits, n_bits) == 0) {
        return 0;  // already clean
    }

    // Try flipping each bit in turn.
    for (size_t i = 0; i < n_bits; i++) {
        bits[i] ^= 1;
        if (iridium_bch_ndivide(poly, bits, n_bits) == 0) {
            return 1;  // single-bit error corrected (flip remains)
        }
        bits[i] ^= 1;  // revert and try next
    }
    return -1;
}

int iridium_bch_repair2(uint32_t poly, uint8_t *bits, size_t n_bits)
{
    if (!bits || n_bits == 0) return -1;

    if (iridium_bch_ndivide(poly, bits, n_bits) == 0) {
        return 0;  // already clean
    }

    // Single-bit pass first.
    for (size_t i = 0; i < n_bits; i++) {
        bits[i] ^= 1;
        if (iridium_bch_ndivide(poly, bits, n_bits) == 0) {
            return 1;  // 1-bit error
        }
        bits[i] ^= 1;
    }

    // Two-bit pass: O(n^2) brute force. n_bits <= 31 so worst case 465
    // divides — fast enough for the off-real-time classification path.
    for (size_t i = 0; i < n_bits; i++) {
        bits[i] ^= 1;
        for (size_t j = i + 1; j < n_bits; j++) {
            bits[j] ^= 1;
            if (iridium_bch_ndivide(poly, bits, n_bits) == 0) {
                return 2;  // 2-bit error corrected (both flips remain)
            }
            bits[j] ^= 1;
        }
        bits[i] ^= 1;
    }
    return -1;
}
