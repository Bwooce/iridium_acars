// Iridium DA (Data) frame post-LCW decoder. See ida_decode.h.

#include "ida_decode.h"
#include "iridium_bch.h"
#include <string.h>

// DA frames use the ACCH BCH(31,21) polynomial (poly=3545, 12-bit
// syndrome), NOT the ringalert poly=1207. See iridium-toolkit/
// bitsparser.py:IridiumLCWECCMessage line 1294: `self.poly=acch_bch_poly`.
#define ACCH_BCH_POLY 3545u

#define UW_BITS 24
#define LCW_BITS 46
#define DATA_BITS_TOTAL 312
#define CHUNK_124 124
#define END_64 64
#define BCH_CW_BITS 31
// ACCH BCH(31,20): 20 message bits + 11 ECC bits per codeword.
// (NOT BCH(31,21) — that's the ringalert variant for IBC frames.)
#define BCH_MSG_BITS 20

// CRC-16/CCITT-FALSE (a.k.a. CRC-16/IBM-3740):
//   poly = 0x1021, init = 0xFFFF, refin = false, refout = false, xorout = 0.
// Matches `crcmod.predefined.mkPredefinedCrcFun("crc-ccitt-false")` used in
// iridium-toolkit/bitsparser.py:IridiumDAMessage.
static uint16_t crc16_ccitt_false(const uint8_t *data, size_t n_bytes)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < n_bytes; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Pair-swap bits in-place: r[0]<->r[1], r[2]<->r[3], ...
// Mirrors iridium-toolkit/bitsparser.py:symbol_reverse(). qpsk_demod
// emits bits in (high, low) per-symbol order; the parser pair-swaps
// before classification and post-LCW processing.
static void pair_swap(uint8_t *bits, size_t n)
{
    for (size_t i = 0; i + 1 < n; i += 2) {
        uint8_t t   = bits[i];
        bits[i]     = bits[i + 1];
        bits[i + 1] = t;
    }
}

// Generic de_interleave: split N-bit input (N must be even) into two
// N/2-bit outputs (odd, even). Direct port of bitsparser.py's
// de_interleave: build N/2 2-bit symbols from in[2i+1, 2i] pairs, then
// take symbols at indices N/2-1, -3, ... → odd; N/2-2, -4, ... → even.
static void de_interleave(const uint8_t *in, int n_in,
                          uint8_t *odd_out, uint8_t *even_out)
{
    int n_sym    = n_in / 2;
    int even_idx = 0, odd_idx = 0;
    for (int s = n_sym - 1; s >= 0; s -= 2) {
        odd_out[odd_idx++] = in[2 * s + 1] & 1;
        odd_out[odd_idx++] = in[2 * s + 0] & 1;
    }
    for (int s = n_sym - 2; s >= 0; s -= 2) {
        even_out[even_idx++] = in[2 * s + 1] & 1;
        even_out[even_idx++] = in[2 * s + 0] & 1;
    }
}

// One 124-bit chunk → 4 BCH(31,21) codewords.
//
// IridiumDAMessage logic:
//   (b1, b2) = de_interleave(chunk)              # b1, b2 each 62 bits
//   (b1, b2, b3, b4) = slice(b1+b2, 31)          # 4 × 31-bit codewords
//   self.descrambled += [b4, b2, b3, b1]         # reorder
//
// `out_codewords` must hold 4 × 31 = 124 bytes (0/1-per-byte).
static void chunk_124_to_codewords(const uint8_t *chunk, uint8_t *out_codewords)
{
    uint8_t b1[62], b2[62];
    de_interleave(chunk, CHUNK_124, b1, b2);
    // Concatenate b1+b2 (124 bits) and slice into 4 × 31-bit blocks.
    uint8_t cat[124];
    memcpy(cat, b1, 62);
    memcpy(cat + 62, b2, 62);
    // Codeword order in self.descrambled: [b4, b2, b3, b1] where
    //   b1 = cat[ 0..30]
    //   b2 = cat[31..61]
    //   b3 = cat[62..92]
    //   b4 = cat[93..123]
    memcpy(out_codewords + 0 * 31, cat + 93, 31); // b4
    memcpy(out_codewords + 1 * 31, cat + 31, 31); // b2
    memcpy(out_codewords + 2 * 31, cat + 62, 31); // b3
    memcpy(out_codewords + 3 * 31, cat + 0, 31);  // b1
}

// 64-bit tail → 2 BCH(31,21) codewords. IridiumDAMessage logic:
//   (b1, b2) = de_interleave(end)                 # each 32 bits
//   self.descrambled += [b2[1:], b1[1:]]          # strip the leading bit
static void tail_64_to_codewords(const uint8_t *tail, uint8_t *out_codewords)
{
    uint8_t b1[32], b2[32];
    de_interleave(tail, END_64, b1, b2);
    memcpy(out_codewords + 0 * 31, b2 + 1, 31); // b2[1:]
    memcpy(out_codewords + 1 * 31, b1 + 1, 31); // b1[1:]
}

int ida_decode(const iridium_frame_t *frame, ida_decoded_t *out)
{
    if (!frame || !out) return -1;
    if (frame->type != IR_FRAME_LW || frame->lw_subtype != IR_LW_DA) return -1;
    // Need full UW (24) + LCW (46) + 312 data bits.
    if (frame->n_bits < UW_BITS + LCW_BITS + DATA_BITS_TOTAL) return -1;

    memset(out, 0, sizeof(*out));
    out->n_blocks = 10;

    // Apply pair-swap to the post-UW data section. The classifier
    // does this in a local stack buffer; we have to redo it from
    // frame->bits which is in qpsk_demod orientation.
    uint8_t data[DATA_BITS_TOTAL];
    memcpy(data, frame->bits + UW_BITS + LCW_BITS, DATA_BITS_TOTAL);
    pair_swap(data, DATA_BITS_TOTAL);

    // Build all 10 31-bit codewords.
    uint8_t codewords[10 * 31];
    chunk_124_to_codewords(data + 0, codewords + 0 * 4 * 31);
    chunk_124_to_codewords(data + CHUNK_124, codewords + 1 * 4 * 31);
    tail_64_to_codewords(data + 2 * CHUNK_124, codewords + 2 * 4 * 31);

    // BCH-decode each codeword using the ACCH poly with up to 2-bit ECC
    // repair (matches upstream bch_repair() / nrepair2 syndrome table
    // for poly=3545). The first 21 bits of each codeword are the
    // message; the remaining 10 are ECC.
    for (int i = 0; i < 10; i++) {
        uint8_t cw[BCH_CW_BITS];
        memcpy(cw, codewords + i * BCH_CW_BITS, BCH_CW_BITS);
        int errs = iridium_bch_repair2(ACCH_BCH_POLY, cw, BCH_CW_BITS);
        if (errs < 0) {
            // Fail soft: leave this block's slot zero-filled (already
            // zero from the memset above) and move on. We deliberately
            // do NOT compact/shift later blocks into this slot: writing
            // every surviving block at its TRUE position (i *
            // BCH_MSG_BITS) keeps out->bits positionally addressable by
            // block index even under partial decode, so a stray
            // positional read of a failed block's slot sees zeros
            // instead of a later block's bits shifted into its place.
            // The SBD reassembler upstream tolerates partial frames via
            // segment retry, gated on out->ok / out->blocks_ok — not by
            // reading bits[] positionally for a block that failed.
            continue;
        }
        // Take the first 20 bits as the message, written at this
        // block's true position (not a running/compacted cursor).
        memcpy(out->bits + i * BCH_MSG_BITS, cw, BCH_MSG_BITS);
        out->blocks_ok++;
        out->total_errors += errs;
    }
    // n_bits is a *count* of valid bits (blocks_ok * BCH_MSG_BITS), for
    // backward-compatible reporting. It is NOT necessarily a contiguous
    // prefix of bits[] when blocks_ok < n_blocks: failed blocks are
    // zero-filled in place at their true position, not skipped/compacted
    // (see the loop above). Since blocks_ok * BCH_MSG_BITS only reaches
    // the 196-bit threshold below when blocks_ok == n_blocks (10 * 20 =
    // 200; 9 * 20 = 180 < 196), the header/payload/CRC parse further
    // down only ever runs once all 10 blocks are true-position-complete.
    int bit_pos = out->blocks_ok * BCH_MSG_BITS;
    out->n_bits = (uint16_t)bit_pos;
    out->ok     = (out->blocks_ok == out->n_blocks);

    // Parse the 20-bit header per bitsparser.py:IridiumDAMessage.
    // Need at least 196 bits (9*20+16) for header + payload + CRC.
    if (bit_pos < 196) {
        return 0; // BCH ok but not enough decoded data — leave header fields zero
    }
    const uint8_t *b = out->bits;
// Helper: pack n bits MSB-first from b[off..off+n-1]
#define PACKBITS_N(off, n) ({                 \
    uint32_t _v = 0;                          \
    for (int _k = 0; _k < (int)(n); _k++) {   \
        _v = (_v << 1) | (b[(off) + _k] & 1); \
    }                                         \
    _v;                                       \
})
    // Header bit layout matches iridium-toolkit bitsparser.py:1385-1391:
    //   bits[0:3] flags, bit[3]=cont, bit[4]=flag1b, bits[5:8]=ctr,
    //   bits[8:11] flags, bits[11:16]=len. da_cont was previously read
    //   from bit 4, which silently collapsed every multi-fragment opener
    //   (cont=1) into a standalone frame and orphaned its continuation —
    //   breaking all cross-burst SBD reassembly. See tests/host/test_phaseb_cut.c.
    //   NB: bit 4 is NOT the "always ~0 spacer" it was once assumed to be —
    //   empirically it is set on ~13% of frames (2026-07-15 HydraSDR corpus),
    //   strongly correlated with SBD type 0x7605 (da_len=11). Captured below as
    //   da_flag1b for study; its meaning is undecoded (no public spec) and it is
    //   NOT used for reassembly (which keys only on da_cont).
    out->da_flags1 = (uint8_t)PACKBITS_N(0, 3);
    out->da_cont   = (uint8_t)PACKBITS_N(3, 1);
    out->da_flag1b = (uint8_t)PACKBITS_N(4, 1);
    out->da_ctr    = (uint8_t)PACKBITS_N(5, 3);
    out->da_flags2 = (uint8_t)PACKBITS_N(8, 3);
    out->da_len    = (uint8_t)PACKBITS_N(11, 5);
    out->da_flags3 = (uint8_t)PACKBITS_N(16, 1);
    uint8_t zero1  = (uint8_t)PACKBITS_N(17, 3);
    out->header_ok = (zero1 == 0);

    // Pack payload bytes. Per iridium-toolkit/bitsparser.py:1366, the
    // payload is bits[20..(20+da_len*8)] when da_len > 0; when da_len
    // == 0 the payload spans bits[20..end] = up to 23 bytes for our
    // 210-bit bitstream_bch (bits[20:220] truncates to 210). da_len is
    // a header field (0..24, per ida_decode.h) so clamp it against
    // both the payload[] buffer and the bits actually decoded.
    int payload_bits = (out->da_len > 0)
                           ? (int)out->da_len * 8 // = da_len bytes
                           : (bit_pos - 20);      // up to ~190 bits = 23 bytes
    if (payload_bits < 0) payload_bits = 0;
    int max_len = payload_bits / 8;
    if (max_len > 24) max_len = 24;     // payload[] buffer size
    int avail_len = (bit_pos - 20) / 8; // bytes actually decoded
    if (avail_len < 0) avail_len = 0;
    if (max_len > avail_len) max_len = avail_len; // defensive: garbage da_len
    out->payload_len = (uint8_t)max_len;
    for (int byte_i = 0; byte_i < max_len; byte_i++) {
        out->payload[byte_i] = (uint8_t)PACKBITS_N(20 + byte_i * 8, 8);
    }

    // CRC field at bits[9*20 .. 9*20+16] = bits[180..196].
    out->da_crc_reported = (uint16_t)PACKBITS_N(180, 16);

    // D12: validate CRC. Reconstruct the iridium-toolkit byte stream
    //   crcstream = bits[0..20] + "0"*12 + bits[20..-4]
    //             = header + 12 zeros + (data + CRC field bits)
    // i.e. 20 header bits + 12 zero pad + bits[20..195] = 208 bits = 26 bytes.
    // CRC-16/CCITT-FALSE over this stream returns 0 for a valid frame
    // (the CRC's "residual" property: CRC(message || CRC) == 0).
    //
    // iridium-toolkit only computes CRC when da_len > 0 (empty frames
    // have no meaningful CRC). We compute it unconditionally for
    // diagnostic purposes but expose `crc_ok` as false when da_len == 0.
    {
        uint8_t crc_buf[26];
        memset(crc_buf, 0, sizeof(crc_buf));
        // Bits 0..19 (header) → crc_buf[0], crc_buf[1] high nibble.
        for (int b = 0; b < 20; b++) {
            int byte_i = b / 8;
            int bit_i  = 7 - (b % 8);
            crc_buf[byte_i] |= (out->bits[b] & 1) << bit_i;
        }
        // Bits 20..31 of crc_buf = 12 zeros (already 0 from memset).
        // Bits 32..207 = out->bits[20..195] (176 bits).
        for (int b = 0; b < 176; b++) {
            int src    = 20 + b;
            int dst    = 32 + b;
            int byte_i = dst / 8;
            int bit_i  = 7 - (dst % 8);
            if (out->bits[src]) crc_buf[byte_i] |= 1 << bit_i;
        }
        out->da_crc_computed = crc16_ccitt_false(crc_buf, 26);
        out->crc_ok          = (out->da_len > 0) && (out->da_crc_computed == 0);
    }
#undef PACKBITS_N

    return 0;
}
