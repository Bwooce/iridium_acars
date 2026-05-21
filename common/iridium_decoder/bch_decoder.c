#include <string.h>
#include <stdint.h>
#include <limits.h>
#if __has_include("esp_attr.h")
#include "esp_attr.h"
#else
// Host build: EXT_RAM_BSS_ATTR is a no-op (only one address space).
#define EXT_RAM_BSS_ATTR
#endif
#include "bch_decoder.h"

// 8 KiB syndrome -> (errs, locator) LUT. Random access pattern but
// only a few hundred lookups per BCH(31,21) block (and 2 blocks per
// Iridium frame), so the per-lookup PSRAM latency (~30 ns vs ~5 ns
// for internal SRAM) is invisible at frame rates. Lives in PSRAM
// to free DMA-capable internal SRAM for the USB transfer pool.
// See docs/p4-bss-audit.md win #1.
//
// Tested 2026-05-21: moving to regular internal .bss (drop the
// EXT_RAM_BSS_ATTR) made no measurable difference to bch stage
// timing (3580 us baseline -> 3843 us, within run-to-run noise).
// The L2 cache hides PSRAM latency for this read-mostly 8 KB table.
// Reverted to keep 8 KB of internal SRAM available for other uses.
static EXT_RAM_BSS_ATTR struct { int errs; uint32_t locator; } syn_ra[1024];

static uint32_t gf2_remainder(uint32_t poly, uint32_t val)
{
    if (val == 0) return 0;
    int poly_bits = 32 - __builtin_clz(poly);
    for (int i = 31; i >= poly_bits - 1; i--) {
        if (val & (1u << i))
            val ^= poly << (i - poly_bits + 1);
    }
    return val;
}

void bch_decoder_init()
{
    for (int i = 0; i < 1024; i++) {
        syn_ra[i].errs = -1;
        syn_ra[i].locator = 0;
    }

    // Single-bit errors
    for (int b1 = 0; b1 < 31; b1++) {
        uint32_t val = 1u << b1;
        uint32_t r = gf2_remainder(BCH_POLY_RA, val);
        if (r < 1024) {
            syn_ra[r].errs = 1;
            syn_ra[r].locator = val;
        }
    }

    // Two-bit errors
    for (int b1 = 0; b1 < 31; b1++) {
        for (int b2 = b1 + 1; b2 < 31; b2++) {
            uint32_t val = (1u << b1) | (1u << b2);
            uint32_t r = gf2_remainder(BCH_POLY_RA, val);
            if (r < 1024 && syn_ra[r].errs < 0) {
                syn_ra[r].errs = 2;
                syn_ra[r].locator = val;
            }
        }
    }
}

static uint32_t bits_to_uint(const uint8_t *bits, int n)
{
    uint32_t val = 0;
    for (int i = 0; i < n; i++) val = (val << 1) | (bits[i] & 1);
    return val;
}

int bch_decode_block(const uint8_t *block31, uint8_t *out_data)
{
    uint32_t val = bits_to_uint(block31, 31);
    uint32_t syndrome = gf2_remainder(BCH_POLY_RA, val);

    if (syndrome == 0) {
        uint32_t data_val = val >> 10;
        for (int i = 0; i < 21; i++) out_data[20 - i] = (data_val >> i) & 1;
        return 0;
    }

    if (syndrome < 1024 && syn_ra[syndrome].errs >= 0) {
        val ^= syn_ra[syndrome].locator;
        uint32_t data_val = val >> 10;
        for (int i = 0; i < 21; i++) out_data[20 - i] = (data_val >> i) & 1;
        return syn_ra[syndrome].errs;
    }

    return -1;
}

// Chase-2 soft-decision BCH decoder. The hard-decision decoder above
// corrects up to t=2 errors. Chase-2 extends that range by trying
// 2^K hypotheses where each hypothesis flips a subset of the K
// least-reliable bits, hard-decodes each, and picks the hypothesis
// whose result is closest (in soft-distance terms) to the received
// soft input.
//
// Algorithm (Chase, "A class of algorithms for decoding block codes
// with channel measurement information," IEEE TIT 1972):
//   1. Hard-decision from soft inputs.
//   2. Sort by |soft|, pick K least-reliable bit positions.
//   3. For p ∈ {0..2^K-1}: flip the subset of LCBs indicated by p,
//      hard-decode, compute soft distance to received word.
//   4. Output the data bits from the trial with smallest distance.
//
// With K=3, the extra coverage handles up to 3 bit errors at the
// cost of 8 hard-decodes. K=4 → 16 trials → up to 4 errors. The
// marginal SNR gain falls off sharply past K=4; gr-iridium uses
// K=3-4 in their experimental soft path.
//
// Soft distance metric: for each bit i, if the decoded codeword
// bit differs from sign(soft_in[i]), add |soft_in[i]| to distance.
// (Hamming distance weighted by reliability.)
int bch_decode_block_soft(const int16_t *soft_in31, uint8_t *out_data, int K)
{
    if (K < 0) K = 0;
    if (K > 6) K = 6;       // 2^6 = 64 trials is the practical limit

    uint8_t hard[31];
    int16_t soft_abs[31];
    for (int i = 0; i < 31; i++) {
        hard[i] = (soft_in31[i] >= 0) ? 0 : 1;
        soft_abs[i] = (soft_in31[i] < 0) ? (int16_t)(-soft_in31[i]) : soft_in31[i];
    }

    // Find the K positions with smallest |soft| via partial sort.
    // K ≤ 6, N = 31, so a simple O(N×K) loop is cheap.
    int lcb_idx[6];
    for (int k = 0; k < K; k++) lcb_idx[k] = -1;
    for (int i = 0; i < 31; i++) {
        // Insert i into lcb_idx if it's among the K smallest.
        int v = soft_abs[i];
        for (int k = 0; k < K; k++) {
            if (lcb_idx[k] < 0 || v < soft_abs[lcb_idx[k]]) {
                // Shift larger entries right, drop the last.
                for (int j = K - 1; j > k; j--) lcb_idx[j] = lcb_idx[j - 1];
                lcb_idx[k] = i;
                break;
            }
        }
    }

    // Try every flip pattern p ∈ {0..2^K-1}. Track best by soft distance.
    int trials = 1 << K;
    int best_dist = INT32_MAX;
    int best_errs = -1;
    uint8_t best_out[21];
    uint8_t flipped[31];
    uint8_t trial_data[21];
    uint8_t trial_codeword[31];

    for (int p = 0; p < trials; p++) {
        memcpy(flipped, hard, 31);
        for (int k = 0; k < K; k++) {
            if (p & (1 << k)) flipped[lcb_idx[k]] ^= 1;
        }
        int errs = bch_decode_block(flipped, trial_data);
        if (errs < 0) continue;
        // Reconstruct the corrected codeword: re-encode? Faster path:
        // we know `flipped XOR locator == corrected codeword`. The
        // hard decoder returns `out_data` as the upper 21 bits; we
        // reconstruct the full codeword by re-encoding (multiply
        // data × x^10 mod poly).
        uint32_t data_val = 0;
        for (int b = 0; b < 21; b++) data_val = (data_val << 1) | (trial_data[b] & 1);
        uint32_t systematic = data_val << 10;
        uint32_t parity = gf2_remainder(BCH_POLY_RA, systematic);
        uint32_t cw = systematic | parity;
        for (int i = 0; i < 31; i++) trial_codeword[i] = (cw >> (30 - i)) & 1;

        // Soft distance: sum |soft_in| where codeword differs from hard.
        int dist = 0;
        for (int i = 0; i < 31; i++) {
            if (trial_codeword[i] != hard[i]) dist += soft_abs[i];
        }
        if (dist < best_dist) {
            best_dist = dist;
            best_errs = errs + __builtin_popcount(p);   // include the flips
            memcpy(best_out, trial_data, 21);
        }
    }

    if (best_errs < 0) return -1;
    memcpy(out_data, best_out, 21);
    return best_errs;
}

void iridium_deinterleave(const uint8_t *in, uint8_t *out1, uint8_t *out2)
{
    int p = 0;
    for (int s = 31; s >= 1; s -= 2) {
        out1[p++] = in[2 * s];
        out1[p++] = in[2 * s + 1];
    }
    p = 0;
    for (int s = 30; s >= 0; s -= 2) {
        out2[p++] = in[2 * s];
        out2[p++] = in[2 * s + 1];
    }
}
