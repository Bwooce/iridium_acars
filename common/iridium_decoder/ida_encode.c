// LW.DA downlink frame encoder — see ida_encode.h. Structurally mirrors
// ida_decode.c and iridium_frame.c's classify_lw(), running each
// transform in reverse.

#include "ida_encode.h"
#include "iridium_bch.h"
#include "crc16.h"
#include <string.h>

// Same ACCH BCH(31,20) polynomial ida_decode.c uses for the 10 DATA
// blocks (poly=3545, NOT the ringalert poly=1207 or messaging 1897).
#define ACCH_BCH_POLY 3545u

#define UW_BITS 24
#define LCW_BITS 46
#define DATA_BITS_TOTAL 312
#define CHUNK_124 124
#define END_64 64
#define BCH_CW_BITS 31
#define BCH_MSG_BITS 20
#define N_BLOCKS 10

// Canonical downlink UW (iridium_frame.c's UW_DL, duplicated here since
// that array is static to its translation unit). Keep in sync if the
// production UW ever changes.
static const uint8_t UW_DL[UW_BITS] = {
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

// --- shared bit-twiddling primitives (mirror ida_decode.c) ---

// Pair-swap is its own inverse: swapping [0]<->[1], [2]<->[3], ... twice
// returns the original array, so this same routine both applies and
// undoes the qpsk_demod (high,low)-per-symbol orientation swap.
static void pair_swap(uint8_t *bits, size_t n)
{
    for (size_t i = 0; i + 1 < n; i += 2) {
        uint8_t t   = bits[i];
        bits[i]     = bits[i + 1];
        bits[i + 1] = t;
    }
}

// Inverse of ida_decode.c's de_interleave(): given the two N/2-bit
// outputs (odd_out, even_out) de_interleave() would have produced from
// some N-bit `in`, reconstruct that `in`. de_interleave()'s two loops
// visit every symbol index s in [0, n_sym) exactly once (odd_out takes
// the odd-first-half s values, even_out the even-second-half ones), so
// writing each output bit back to its source position is a valid,
// exact inverse.
static void de_interleave_inverse(const uint8_t *odd_out, const uint8_t *even_out,
                                  int n_in, uint8_t *in)
{
    int n_sym   = n_in / 2;
    int odd_idx = 0, even_idx = 0;
    for (int s = n_sym - 1; s >= 0; s -= 2) {
        in[2 * s + 1] = odd_out[odd_idx++];
        in[2 * s + 0] = odd_out[odd_idx++];
    }
    for (int s = n_sym - 2; s >= 0; s -= 2) {
        in[2 * s + 1] = even_out[even_idx++];
        in[2 * s + 0] = even_out[even_idx++];
    }
}

// Inverse of ida_decode.c's chunk_124_to_codewords(): given the 4
// desired 31-bit block codewords (in block order cw0..cw3), produce
// the 124-bit chunk that would decode back to them.
//
// Forward mapping (from ida_decode.c, "descrambled" order [b4,b2,b3,b1]
// with cat = deA(0:62) ++ deB(62:124), codeword comment labels b1..b4):
//   out_codewords[0:31]  = b4 = cat[93:124] = deB[31:62]
//   out_codewords[31:62] = b2 = cat[31:62]  = deA[31:62]
//   out_codewords[62:93] = b3 = cat[62:93]  = deB[0:31]
//   out_codewords[93:124]= b1 = cat[0:31]   = deA[0:31]
// i.e. block0=b4, block1=b2, block2=b3, block3=b1, so:
//   deA[0:31]=cw3  deA[31:62]=cw1  deB[0:31]=cw2  deB[31:62]=cw0
static void chunk_124_from_codewords(const uint8_t *cw0, const uint8_t *cw1,
                                     const uint8_t *cw2, const uint8_t *cw3,
                                     uint8_t *out_chunk)
{
    uint8_t deA[62], deB[62];
    memcpy(deA + 0, cw3, 31);
    memcpy(deA + 31, cw1, 31);
    memcpy(deB + 0, cw2, 31);
    memcpy(deB + 31, cw0, 31);
    de_interleave_inverse(deA, deB, CHUNK_124, out_chunk);
}

// Inverse of ida_decode.c's tail_64_to_codewords(): given the 2 desired
// 31-bit block codewords (cw8, cw9), produce the 64-bit tail. Forward:
//   out_codewords[0:31]  = b2[1:] = cw8   => b2[1:32]=cw8, b2[0] unused
//   out_codewords[31:62] = b1[1:] = cw9   => b1[1:32]=cw9, b1[0] unused
// b1[0]/b2[0] are dropped by ida_decode() (never read back), so any
// value works; use 0.
static void tail_64_from_codewords(const uint8_t *cw8, const uint8_t *cw9,
                                   uint8_t *out_tail)
{
    uint8_t b1[32], b2[32];
    b1[0] = 0;
    b2[0] = 0;
    memcpy(b2 + 1, cw8, 31);
    memcpy(b1 + 1, cw9, 31);
    de_interleave_inverse(b1, b2, END_64, out_tail);
}

// Systematic BCH encode: k-bit message -> (k + poly_len-1)-bit codeword
// (message bits followed by parity). Same shift-and-XOR encoder used by
// tests/host/test_iridium_frame_phaseB.c's bch_encode() /
// test_iridium_bch.c, built on the same iridium_bch_ndivide() the
// production decoder uses for its repair syndrome.
static int u32_bit_length(uint32_t x)
{
    int n = 0;
    while (x) {
        n++;
        x >>= 1;
    }
    return n;
}
static void bch_encode_block(uint32_t poly, const uint8_t *msg, int k, uint8_t *out)
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

// CRC-16/CCITT-FALSE comes from the shared crc16.c (same impl ida_decode.c
// verifies against — needed on both sides of the residue property:
// CRC(message || CRC(message)) == 0 for this variant since its Residue is
// 0x0000 per the standard CRC catalogue). Was a local static bit-serial
// copy; deduped to the shared table-based one (bit-exact, test_crc16_ccitt).

// OR n_bits 0/1-per-byte bits[] (MSB-first) into out[], starting at bit
// offset out_bit_offset. out[] must be pre-zeroed by the caller; this
// only ever sets bits, matching ida_decode.c's crc_buf construction
// exactly (same byte_i = dst/8, bit_i = 7-(dst%8) packing).
static void pack_bits_at(uint8_t *out, int out_bit_offset,
                         const uint8_t *bits, int n_bits)
{
    for (int i = 0; i < n_bits; i++) {
        int dst    = out_bit_offset + i;
        int byte_i = dst / 8;
        int bit_i  = 7 - (dst % 8);
        if (bits[i] & 1) out[byte_i] |= (uint8_t)(1 << bit_i);
    }
}

// Build the 46-bit raw (pre-classifier-pair-swap) LCW section with
// ft=2 (DA/SBD). lcw2/lcw3 content is arbitrary (all-zero) — only the
// 3-bit ft field in lcw1 drives LW sub-classification; ida_decode()
// never reads the LCW bits at all (it works on the frame's DATA
// section, 70 bits further in). Mirrors
// tests/host/test_iridium_frame_phaseB.c's test_synthetic_lw_da().
static void build_lcw_raw_bits(uint8_t out[LCW_BITS])
{
    static const uint8_t lcw1_msg[3]  = {0, 1, 0}; // ft=2 (binary 010) = DA
    static const uint8_t lcw2_msg[6]  = {0, 0, 0, 0, 0, 0};
    static const uint8_t lcw3_msg[21] = {0};

    uint8_t lcw1_cw[7];  // 3 + 4, poly=29  (5-bit)
    uint8_t lcw2_cw[14]; // 6 + 8, poly=465 (9-bit)
    uint8_t lcw3_cw[26]; // 21 + 5, poly=41 (6-bit)
    bch_encode_block(29u, lcw1_msg, 3, lcw1_cw);
    bch_encode_block(465u, lcw2_msg, 6, lcw2_cw);
    bch_encode_block(41u, lcw3_msg, 21, lcw3_cw);

    // Forward de-interleave split: permuted[0..6]=lcw1, [7..19]=lcw2's
    // first 13 bits (trailing pad bit dropped), [20..45]=lcw3.
    uint8_t permuted[LCW_BITS] = {0};
    memcpy(permuted + 0, lcw1_cw, 7);
    memcpy(permuted + 7, lcw2_cw, 13);
    memcpy(permuted + 20, lcw3_cw, 26);

    // Classifier's forward permutation is permuted[i] = data[LCW_TBL[i]];
    // invert it: data[LCW_TBL[i]] = permuted[i].
    static const uint8_t LCW_TBL[LCW_BITS] = {
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
    uint8_t data[LCW_BITS] = {0};
    for (int i = 0; i < LCW_BITS; i++) {
        data[LCW_TBL[i]] = permuted[i];
    }

    // Pair-swap once so the classifier's internal swap recovers `data`.
    pair_swap(data, LCW_BITS);
    memcpy(out, data, LCW_BITS);
}

int ida_encode_da_frame(uint8_t da_cont, uint8_t da_ctr,
                        const uint8_t *payload, uint8_t payload_len,
                        uint8_t out_bits[IDA_ENCODE_FRAME_BITS])
{
    if (!payload || !out_bits) return -1;
    if (payload_len == 0 || payload_len > 24) return -1;
    if (da_ctr > 7) return -1;

    memset(out_bits, 0, IDA_ENCODE_FRAME_BITS);
    memcpy(out_bits, UW_DL, UW_BITS);
    build_lcw_raw_bits(out_bits + UW_BITS);

    // --- Build the 200-bit message: header(20) + payload region(160,
    // bits[20:180]) + CRC(16, bits[180:196]) + unused tail(4). Bit
    // positions and widths mirror ida_decode.c's PACKBITS_N() calls
    // exactly (msg[off] is the MSB of each field).
    uint8_t msg[N_BLOCKS * BCH_MSG_BITS];
    memset(msg, 0, sizeof(msg));
    int p = 0;
    p += 3;                 // da_flags1 = 0 (bits[0:3])
    msg[p++] = da_cont & 1; // bit 3 = cont (iridium-toolkit bitstream_bch[3:4])
    p += 1;                 // bit 4 = spacer (bitstream_bch[4:5], always 0)
    for (int k = 0; k < 3; k++)
        msg[p++] = (uint8_t)((da_ctr >> (2 - k)) & 1);
    p += 3; // da_flags2 = 0
    for (int k = 0; k < 5; k++)
        msg[p++] = (uint8_t)((payload_len >> (4 - k)) & 1);
    p += 1; // da_flags3 = 0
    p += 3; // zero1 = 0 (must stay 0 for header_ok)
    // p == 20 here.

    for (int byte_i = 0; byte_i < payload_len; byte_i++) {
        for (int k = 0; k < 8; k++) {
            msg[20 + byte_i * 8 + k] = (uint8_t)((payload[byte_i] >> (7 - k)) & 1);
        }
    }
    // bits[20+payload_len*8 .. 180) stay 0 (padding before the CRC field).

    // CRC-16/CCITT-FALSE residue trick: compute the CRC over header(20
    // bits, padded to 32 with zeros) + payload region(160 bits) = 24
    // bytes, write that as the transmitted CRC field; ida_decode()'s
    // own recompute over those 24 bytes + the CRC field itself (26
    // bytes total) will then land on the algorithm's Residue (0x0000).
    uint8_t crc_buf[26];
    memset(crc_buf, 0, sizeof(crc_buf));
    pack_bits_at(crc_buf, 0, msg, 20);
    pack_bits_at(crc_buf, 32, msg + 20, 160);
    uint16_t crc_val = crc16_ccitt_false(crc_buf, 24);
    for (int k = 0; k < 16; k++) {
        msg[180 + k] = (uint8_t)((crc_val >> (15 - k)) & 1);
    }
    // bits[196..200) stay 0 (not covered by the CRC, never read back).

    // --- BCH(31,20)-encode each of the 10 blocks. ---
    uint8_t cw[N_BLOCKS][BCH_CW_BITS];
    for (int i = 0; i < N_BLOCKS; i++) {
        bch_encode_block(ACCH_BCH_POLY, msg + i * BCH_MSG_BITS, BCH_MSG_BITS, cw[i]);
    }

    // --- Invert the chunk/tail decompositions to get the DATA section
    // in its post-pair_swap orientation, then pair-swap once more
    // (pair_swap is self-inverse) to get the raw pre-decode bits. ---
    uint8_t data_swapped[DATA_BITS_TOTAL];
    chunk_124_from_codewords(cw[0], cw[1], cw[2], cw[3], data_swapped + 0);
    chunk_124_from_codewords(cw[4], cw[5], cw[6], cw[7], data_swapped + CHUNK_124);
    tail_64_from_codewords(cw[8], cw[9], data_swapped + 2 * CHUNK_124);
    pair_swap(data_swapped, DATA_BITS_TOTAL);

    memcpy(out_bits + UW_BITS + LCW_BITS, data_swapped, DATA_BITS_TOTAL);
    return 0;
}
