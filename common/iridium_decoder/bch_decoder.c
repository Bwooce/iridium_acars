#include <string.h>
#include "bch_decoder.h"

static struct { int errs; uint32_t locator; } syn_ra[1024];

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
