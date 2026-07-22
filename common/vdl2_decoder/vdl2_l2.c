// See vdl2_l2.h. Port of dumpvdl2's post-demod burst decode
// (src/decode.c decode_vdl2_burst DEC_HEADER length math + DEC_DATA,
// v2.6.0 3f583da), re-implemented in this repo's style (GPL hygiene per
// the plan §GPL-3). Per-step citations inline.

#include "vdl2_l2.h"

#include <string.h>

#include "rs_vdl2.h"    // common/vdl2 — RS(255,249) codec
#include "vdl2_demod.h" // vdl2_hdr_decode / vdl2_burst_body_bits / limits

// Cold single-consumer working buffers -> PSRAM on target (they are
// byte-work scratch for the frame_decoder task, never DMA/PIE-touched).
// Host build: plain .bss. Same guard pattern as bch_decoder.c.
#if __has_include("esp_attr.h")
#include "esp_attr.h"
#endif
#ifndef EXT_RAM_BSS_ATTR
#define EXT_RAM_BSS_ATTR
#endif

// Sizing from the header length cap: VDL2_MAX_FRAME_BITS = 0x3FFF data
// bits -> 2048 data octets -> ceil(2048/249) = 9 RS blocks -> <= 54 FEC
// octets (dumpvdl2 decode.c:45 MAX_FRAME_LENGTH; :233-242 block math).
#define L2_MAX_DATA_OCTETS ((VDL2_MAX_FRAME_BITS + 7) / 8)
#define L2_MAX_BLOCKS ((L2_MAX_DATA_OCTETS + RS_VDL2_K - 1) / RS_VDL2_K)
#define L2_MAX_FEC_OCTETS (L2_MAX_BLOCKS * RS_VDL2_NROOTS)

static EXT_RAM_BSS_ATTR uint8_t s_data[L2_MAX_DATA_OCTETS]; // interleaved data
static EXT_RAM_BSS_ATTR uint8_t s_fec[L2_MAX_FEC_OCTETS];   // interleaved FEC
static EXT_RAM_BSS_ATTR uint8_t s_rs_tab[L2_MAX_BLOCKS][RS_VDL2_N];
static EXT_RAM_BSS_ATTR uint8_t s_octets[L2_MAX_DATA_OCTETS]; // corrected stream

static vdl2_l2_stats_t s_stats; // single writer; torn reads benign

// FEC octet count of a (shortened) block with `len` data octets —
// dumpvdl2 decode.c:124-133 get_fec_octetcount(). Same table
// vdl2_burst_body_bits() uses for the length pre-check; kept verbatim
// here because the per-block loop needs it, not just the total.
static int fec_octetcount(uint32_t len)
{
    if (len < 3) return 0;
    if (len < 31) return 2;
    if (len < 68) return 4;
    return 6;
}

// Pack `n_octets` from one-bit-per-byte `bits` LSB-first — dumpvdl2
// bitstream_read_lsbfirst (src/bitstream.c:70-81): the first bit off
// the air is the LSB of the first octet.
static void pack_lsbfirst(const uint8_t *bits, uint32_t n_octets, uint8_t *out)
{
    for (uint32_t i = 0; i < n_octets; i++) {
        uint8_t b = 0;
        for (int j = 0; j < 8; j++)
            b |= (uint8_t)((bits[8 * i + j] & 1u) << j);
        out[i] = b;
    }
}

// Byte de-interleaver — EXACT port of dumpvdl2 decode.c:135-163
// deinterleave(). Input bytes are distributed round-robin down the
// block rows (row varies fastest, column advances when the last row is
// filled); once the shorter LAST row is exhausted (data streams only —
// last_row_len < fillwidth) it is zero-padded and skipped for the
// remaining columns. `offset` shifts the write window inside each row:
// 0/fillwidth=RS_K for the data pass, RS_K/fillwidth=NROOTS for the FEC
// pass (decode.c:282,293), which lands parity bytes at rs_tab[r][249..]
// — the rs_vdl2 codeword layout.
static int l2_deinterleave(const uint8_t *in, uint32_t len, uint32_t rows,
                           uint8_t out[][RS_VDL2_N], uint32_t fillwidth,
                           uint32_t offset)
{
    if (rows == 0 || fillwidth == 0) return -1;
    uint32_t last_row_len = len % fillwidth;
    if (last_row_len == 0) last_row_len = fillwidth;
    if (fillwidth + offset > RS_VDL2_N) return -2;
    if (len > rows * fillwidth) return -3;
    if (rows > 1 && len - last_row_len < (rows - 1) * fillwidth) return -4;
    uint32_t row = 0, col = offset;
    last_row_len += offset;
    for (uint32_t i = 0; i < len; i++) {
        if (row == rows - 1 && col >= last_row_len) {
            out[row][col] = 0x00;
            row           = 0;
            col++;
        }
        out[row++][col] = in[i];
        if (row == rows) {
            row = 0;
            col++;
        }
    }
    return 0;
}

int vdl2_l2_feed(const uint8_t *bits, int n_bits,
                 avlc_frame_cb_t cb, void *ctx)
{
    s_stats.fed++;
    if (bits == NULL || n_bits < VDL2_HDR_BITS) {
        s_stats.hdr_reject++;
        return VDL2_L2_ERR_HEADER;
    }

    // Re-decode the burst header (bits are already descrambled — the
    // demod validated it once, but only the bit vector crosses the
    // queue, and vdl2_hdr_decode is deterministic + cheap).
    uint32_t w = 0;
    for (int k = 0; k < VDL2_HDR_BITS; k++)
        w = (w << 1) | (bits[k] & 1u);
    uint32_t datalen = 0;
    if (vdl2_hdr_decode(&w, &datalen) < 0) {
        s_stats.hdr_reject++;
        return VDL2_L2_ERR_HEADER;
    }
    int body = vdl2_burst_body_bits(datalen);
    if (body < 0) {
        s_stats.hdr_reject++;
        return VDL2_L2_ERR_HEADER;
    }
    if (n_bits < VDL2_HDR_BITS + body) {
        s_stats.truncated++;
        return VDL2_L2_ERR_TRUNC;
    }

    // Block layout — dumpvdl2 decode.c:233-245: octet count, full-block
    // count, shortened-last-block length, total FEC octets.
    uint32_t datalen_octets = (datalen + 7) / 8;
    uint32_t num_blocks     = datalen_octets / RS_VDL2_K;
    uint32_t last           = datalen_octets % RS_VDL2_K;
    uint32_t fec_octets     = num_blocks * RS_VDL2_NROOTS;
    if (last) num_blocks++;
    fec_octets += (uint32_t)fec_octetcount(last);
    if (last == 0) last = RS_VDL2_K;
    // datalen <= VDL2_MAX_FRAME_BITS is guaranteed by vdl2_burst_body_bits,
    // so datalen_octets/num_blocks/fec_octets fit the static buffers.

    // Bits -> octets: data first, FEC immediately after (decode.c:264-275).
    pack_lsbfirst(bits + VDL2_HDR_BITS, datalen_octets, s_data);
    pack_lsbfirst(bits + VDL2_HDR_BITS + 8 * datalen_octets, fec_octets, s_fec);

    // De-interleave into RS codeword rows (decode.c:279-297). Rows are
    // zeroed first so a shortened last block reads as data + zero-fill
    // + parity + zeroed erasure positions — the rs_vdl2_decode_shortened
    // layout. If the last block carries no FEC (< 3 data octets), its
    // row is excluded from the FEC pass (decode.c:288-291).
    memset(s_rs_tab, 0, num_blocks * RS_VDL2_N);
    if (l2_deinterleave(s_data, datalen_octets, num_blocks, s_rs_tab,
                        RS_VDL2_K, 0) < 0) {
        return VDL2_L2_ERR_INTERNAL;
    }
    uint32_t fec_rows = num_blocks;
    if (fec_octetcount(last) == 0) fec_rows--;
    if (fec_rows > 0 &&
        l2_deinterleave(s_fec, fec_octets, fec_rows, s_rs_tab,
                        RS_VDL2_NROOTS, RS_VDL2_K) < 0) {
        return VDL2_L2_ERR_INTERNAL;
    }

    // RS-correct each block; any uncorrectable block drops the whole
    // transmission (decode.c:311-316 "FEC check failed" -> cleanup).
    for (uint32_t r = 0; r < num_blocks; r++) {
        int corr = 0;
        int rc;
        int fec_this = (r == num_blocks - 1) ? fec_octetcount(last)
                                             : RS_VDL2_NROOTS;
        if (r == num_blocks - 1) {
            rc = rs_vdl2_decode_shortened(s_rs_tab[r], (int)last, &corr);
        } else {
            rc = rs_vdl2_decode(s_rs_tab[r], &corr);
        }
        if (rc != 0) {
            s_stats.rs_blocks_fail++;
            return VDL2_L2_ERR_RS;
        }
        s_stats.rs_blocks_ok++;
        // Corrected-octet accounting excludes the intended erasures of
        // a shortened block (dumpvdl2 decode.c:321-322).
        int fixed = corr - (RS_VDL2_NROOTS - fec_this);
        if (fixed > 0) s_stats.rs_octets_fixed += (uint32_t)fixed;
    }

    // Corrected data octets back into stream order (decode.c:325-328:
    // RS_K per full block, last_block_len_octets for the final one).
    uint32_t off = 0;
    for (uint32_t r = 0; r < num_blocks; r++) {
        uint32_t n = (r == num_blocks - 1) ? last : RS_VDL2_K;
        memcpy(s_octets + off, s_rs_tab[r], n);
        off += n;
    }

    // AVLC deframe over exactly `datalen` BITS — the transmission
    // length is usually not a multiple of 8 because of bit stuffing;
    // dumpvdl2 truncates the padding bits the same way (decode.c:336-342).
    int nf = avlc_deframe_octets(s_octets, (int)datalen, cb, ctx);
    if (nf > 0) s_stats.avlc_frames += (uint32_t)nf;
    return nf;
}

void vdl2_l2_get_stats(vdl2_l2_stats_t *out)
{
    if (out) *out = s_stats;
}
