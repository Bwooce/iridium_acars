// IBC (Iridium Broadcast) body parser. See ibc_decode.h for the layout.
//
// Direct port of iridium-toolkit/bitsparser.py:IridiumBCMessage init
// (line 1432). Operates on a classified IR_FRAME_BC frame.

#include "ibc_decode.h"
#include "iridium_bch.h"
#include <string.h>

#define UW_BITS         24
#define IBC_HDR_BITS     6
#define IBC_BLOCK_BITS  64
#define IBC_N_BLOCKS     4    // IBC has exactly 4 BCH blocks
#define IBC_DATA_PER_BLK 42   // 2 × BCH(31,21) data bits per 64-bit block

#define IBC_HDR_POLY    29u   // BCH(6, 2)
#define IBC_BLK_POLY  1207u   // BCH(31, 21), ringalert poly

// Decode a 64-bit IBC block into 42 corrected data bits.
//
// Input layout (matches iridium-toolkit's de_interleave at bitsparser.py:1985
// applied to the 64-bit BCH block):
//   - 32 symbols of 2 bits each, in REVERSED bit-pair order (sym[i] =
//     bits[2i+1] || bits[2i])
//   - The "odd" codeword reads symbols at indices 31, 29, 27, ..., 1
//   - The "even" codeword reads symbols at indices 30, 28, ..., 0
// Each codeword is 32 bits: 31-bit BCH(31, 21) + 1 trailing parity bit.
// After BCH-correcting bits[0..30], the first 21 bits are data.
//
// Returns the number of codewords that BCH-decoded cleanly (0, 1, or 2).
// On success, writes 42 corrected data bits into out_data (odd then even).
static int decode_ibc_block(const uint8_t *in64, uint8_t *out_data)
{
    // De-interleave (matches bitsparser de_interleave): build the two
    // 32-bit codewords by walking symbol indices high-to-low in steps
    // of 2.
    uint8_t odd[32], even[32];
    int odd_n = 0, even_n = 0;
    for (int s = 31; s >= 0; s -= 2) {
        // symbol s = (in[2s+1], in[2s])
        odd[odd_n++] = in64[2 * s + 1] & 1;
        odd[odd_n++] = in64[2 * s + 0] & 1;
    }
    for (int s = 30; s >= 0; s -= 2) {
        even[even_n++] = in64[2 * s + 1] & 1;
        even[even_n++] = in64[2 * s + 0] & 1;
    }
    // odd and even are now 32 bits each. Take bits[0..30] as the BCH
    // codeword and try to repair.
    int e_odd  = iridium_bch_repair2(IBC_BLK_POLY, odd,  31);
    int e_even = iridium_bch_repair2(IBC_BLK_POLY, even, 31);
    if (e_odd < 0 && e_even < 0) return 0;
    int ok = 0;
    if (e_odd >= 0) {
        memcpy(out_data + 0, odd, 21);
        ok++;
    } else {
        memset(out_data + 0, 0, 21);
    }
    if (e_even >= 0) {
        memcpy(out_data + 21, even, 21);
        ok++;
    } else {
        memset(out_data + 21, 0, 21);
    }
    return ok;
}

// Read N bits MSB-first from `bits[start..start+n)` into an integer.
static uint32_t pick_bits(const uint8_t *bits, int start, int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++) {
        v = (v << 1) | (bits[start + i] & 1u);
    }
    return v;
}

int ibc_decode(const iridium_frame_t *frame, ibc_decoded_t *out)
{
    if (!frame || !out) return -1;
    memset(out, 0, sizeof(*out));
    out->block1_subtype = -1;

    // Frame body starts after the 24-bit UW. Need header (6) + 4 blocks (256).
    if (frame->n_bits < UW_BITS + IBC_HDR_BITS + IBC_N_BLOCKS * IBC_BLOCK_BITS) {
        return -1;
    }
    const uint8_t *p = frame->bits + UW_BITS;

    // Header: 6 bits, BCH(6, 2) with poly=29. Allow 1-bit repair (matches
    // iridium-toolkit which uses bch_repair1 here).
    uint8_t hdr[6];
    memcpy(hdr, p, IBC_HDR_BITS);
    int e_hdr = iridium_bch_repair1(IBC_HDR_POLY, hdr, IBC_HDR_BITS);
    if (e_hdr < 0) {
        out->header_ok = false;
        out->bc_type   = -1;
        return 0;
    }
    out->header_ok = true;
    // First 2 bits = bc_type (systematic BCH layout: data || parity).
    out->bc_type = (int)pick_bits(hdr, 0, 2);

    // 4 × 64-bit blocks immediately follow the header.
    uint8_t block_data[IBC_DATA_PER_BLK]; // 42-bit decoded payload per block
    p += IBC_HDR_BITS;
    for (int blk = 0; blk < IBC_N_BLOCKS; blk++) {
        int n_ok = decode_ibc_block(p + (size_t)blk * IBC_BLOCK_BITS, block_data);
        if (n_ok == 2) {
            out->n_blocks_ok++;
        } else {
            // Partial / total failure: stop parsing further fields beyond
            // the ones that were already extracted from prior blocks.
            if (blk == 0) out->block0_ok = false;
            if (blk == 1) out->block1_ok = false;
            continue;
        }

        if (blk == 0 && out->bc_type == 0) {
            out->block0_ok    = true;
            out->sv_id        = (int)pick_bits(block_data,  0, 7);
            out->beam_id      = (int)pick_bits(block_data,  7, 6);
            // bit 13 unknown01
            out->slot         = (int)pick_bits(block_data, 14, 1);
            out->sv_blocking  = (int)pick_bits(block_data, 15, 1);
            // bits 16-31  acqu_classes (16-bit) — left for the verbose
            // path; the slot+sv_blocking+sv_id+beam_id give us the key
            // "which satellite/cell" identification.
        } else if (blk == 1 && out->bc_type == 0) {
            out->block1_ok       = true;
            int subtype          = (int)pick_bits(block_data, 0, 6);
            out->block1_subtype  = subtype;
            if (subtype == 1) {
                out->iri_time = pick_bits(block_data, 10, 32);
            } else if (subtype == 2) {
                out->tmsi_expiry = pick_bits(block_data, 10, 32);
            }
        }
        // Blocks 2-3: assignments; not parsed (varies, not on ACARS path)
    }

    return 0;
}

uint64_t ibc_iri_time_to_unix(uint32_t iri_time)
{
    if (iri_time == 0) return 0;
    // ERA3 anchor (active since 2026-01-14): unix 1739556857 = 2025-02-14
    // 18:14:17Z. Tick is 90 ms per LBFC step.
    return (uint64_t)1739556857ull + (uint64_t)iri_time * 90ull / 1000ull;
}
