// Regression test for ibc_decode's bit orientation.
//
// ibc_decode reads the post-UW body straight out of frame->bits, which
// is in qpsk_demod (un-swapped) orientation. The frame classifier only
// accepts IR_FRAME_BC after applying iridium-toolkit's adjacent-pair
// swap (symbol_reverse); every other decoder (ida_decode, ira_decode,
// ims_decode) redoes that swap before parsing. This test synthesizes a
// BC frame with a NON-palindromic payload (so swapped vs un-swapped
// differ) and checks that ibc_decode recovers the exact fields that
// were encoded — which only happens if it applies the same swap the
// classifier validated against.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "iridium_frame.h"
#include "iridium_bch.h"
#include "ibc_decode.h"

static int passed = 0, failed = 0;

#define CHECK(cond, fmt, ...)                                             \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("  FAIL line %d: " fmt "\n", __LINE__, ##__VA_ARGS__); \
            failed++;                                                     \
            return;                                                       \
        } else {                                                          \
            passed++;                                                     \
        }                                                                 \
    } while (0)

#define UW_LEN 24
static const uint8_t UW_DL_BITS[UW_LEN] = {
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

#define IBC_HDR_BITS 6
#define IBC_BLOCK_BITS 64
#define IBC_N_BLOCKS 4
#define IBC_BODY_BITS (IBC_HDR_BITS + IBC_N_BLOCKS * IBC_BLOCK_BITS) // 262

// Pair-swap bits in-place: the inverse of what the classifier/decoders
// apply internally — used here to construct raw demod-orientation
// inputs that, after the production swap, yield specific bits.
static void pair_swap(uint8_t *bits, size_t len)
{
    for (size_t i = 0; i + 1 < len; i += 2) {
        uint8_t t   = bits[i];
        bits[i]     = bits[i + 1];
        bits[i + 1] = t;
    }
}

static int u32_bit_length(uint32_t x)
{
    int n = 0;
    while (x) {
        n++;
        x >>= 1;
    }
    return n;
}

// BCH-encode a k-bit message into a (k + poly_len-1)-bit systematic
// codeword (data || parity). Same shift-and-XOR encoder used by
// test_iridium_frame_phaseB.c / test_iridium_bch.c.
static void bch_encode(uint32_t poly, const uint8_t *msg, int k, uint8_t *out)
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

// Write `value`'s low `n` bits into bits[start..start+n) MSB-first
// (matches ibc_decode.c's pick_bits: v = (v<<1)|bit).
static void put_bits(uint8_t *bits, int start, int n, uint32_t value)
{
    for (int i = 0; i < n; i++) {
        bits[start + i] = (uint8_t)((value >> (n - 1 - i)) & 1u);
    }
}

// Inverse of ibc_decode.c's decode_ibc_block de-interleave: given the
// two 31-bit BCH codewords (odd/even), build the 64-bit interleaved
// block ibc_decode expects (post pair-swap, i.e. classifier/decoder
// orientation).
static void ibc_interleave_block(const uint8_t *odd31, const uint8_t *even31,
                                 uint8_t *out64)
{
    uint8_t odd[32], even[32];
    memcpy(odd, odd31, 31);
    odd[31] = 0; // trailing bit, unchecked by decode_ibc_block
    memcpy(even, even31, 31);
    even[31] = 0;

    int idx = 0;
    for (int s = 31; s >= 1; s -= 2) {
        out64[2 * s + 1] = odd[idx];
        out64[2 * s + 0] = odd[idx + 1];
        idx += 2;
    }
    idx = 0;
    for (int s = 30; s >= 0; s -= 2) {
        out64[2 * s + 1] = even[idx];
        out64[2 * s + 0] = even[idx + 1];
        idx += 2;
    }
}

// Builds a full valid BC frame (UW + header + 4 blocks) carrying
// sv_id=5, beam_id=10, slot=1, sv_blocking=0 in block 0, and
// block1_subtype=1 with iri_time=0x1A2B3C4D in block 1. Blocks 2/3 are
// all-zero (trivially BCH-clean, fields unused). Returns the frame's
// total bit count and fills `bits_out` (caller-sized buffer of at
// least UW_LEN + IBC_BODY_BITS).
static int build_synthetic_bc_frame(uint8_t *bits_out)
{
    memcpy(bits_out, UW_DL_BITS, UW_LEN);

    // Build the post-UW payload in the orientation ibc_decode sees
    // AFTER its pair-swap (i.e. what the classifier validated).
    uint8_t payload[IBC_BODY_BITS];
    memset(payload, 0, sizeof(payload));

    // Header: 6 bits, BCH(6,2) poly=29, bc_type=0 (2 data bits).
    uint8_t hdr_msg[2] = {0, 0};
    bch_encode(29u, hdr_msg, 2, payload + 0);

    // Block 0: sv_id=5, beam_id=10, slot=1, rest 0.
    uint8_t blk0_data[42];
    memset(blk0_data, 0, sizeof(blk0_data));
    put_bits(blk0_data, 0, 7, 5);  // sv_id
    put_bits(blk0_data, 7, 6, 10); // beam_id
    blk0_data[13] = 0;             // unknown01
    blk0_data[14] = 1;             // slot
    blk0_data[15] = 0;             // sv_blocking
    uint8_t blk0_odd_cw[31], blk0_even_cw[31];
    bch_encode(1207u, blk0_data + 0, 21, blk0_odd_cw);
    bch_encode(1207u, blk0_data + 21, 21, blk0_even_cw);
    ibc_interleave_block(blk0_odd_cw, blk0_even_cw, payload + IBC_HDR_BITS + 0 * IBC_BLOCK_BITS);

    // Block 1: subtype=1 (iri_time), iri_time=0x1A2B3C4D.
    uint8_t blk1_data[42];
    memset(blk1_data, 0, sizeof(blk1_data));
    put_bits(blk1_data, 0, 6, 1);             // subtype
    put_bits(blk1_data, 10, 32, 0x1A2B3C4Du); // iri_time
    uint8_t blk1_odd_cw[31], blk1_even_cw[31];
    bch_encode(1207u, blk1_data + 0, 21, blk1_odd_cw);
    bch_encode(1207u, blk1_data + 21, 21, blk1_even_cw);
    ibc_interleave_block(blk1_odd_cw, blk1_even_cw, payload + IBC_HDR_BITS + 1 * IBC_BLOCK_BITS);

    // Blocks 2/3: all-zero (trivially BCH-clean; fields not asserted).

    // Pair-swap the payload BEFORE inserting into the post-UW region,
    // because ibc_decode (like the classifier) will re-swap it back to
    // qpsk_demod orientation on read — matching test_synthetic_bc in
    // test_iridium_frame_phaseB.c.
    pair_swap(payload, sizeof(payload));
    memcpy(bits_out + UW_LEN, payload, sizeof(payload));

    return UW_LEN + IBC_BODY_BITS;
}

static void test_ibc_decode_orientation(void)
{
    printf("Test: synthetic BC frame classifies as IR_FRAME_BC and "
           "ibc_decode recovers header + block0 + block1 fields\n");

    uint8_t bits[UW_LEN + IBC_BODY_BITS];
    int     n_bits = build_synthetic_bc_frame(bits);

    iridium_frame_t f  = {0};
    int             rc = iridium_frame_classify(bits, (size_t)n_bits,
                                                IR_FRM_DIR_DOWNLINK, &f);
    CHECK(rc == 0, "classify rc=%d", rc);
    CHECK(f.type == IR_FRAME_BC, "type=%s (expected BC)",
          iridium_frame_type_name(f.type));

    ibc_decoded_t d   = {0};
    int           drc = ibc_decode(&f, &d);
    CHECK(drc == 0, "ibc_decode rc=%d", drc);
    CHECK(d.header_ok, "header_ok should be true");
    CHECK(d.bc_type == 0, "bc_type=%d (expected 0)", d.bc_type);
    CHECK(d.n_blocks_ok == 4, "n_blocks_ok=%d (expected 4)", d.n_blocks_ok);

    CHECK(d.block0_ok, "block0_ok should be true");
    CHECK(d.sv_id == 5, "sv_id=%d (expected 5)", d.sv_id);
    CHECK(d.beam_id == 10, "beam_id=%d (expected 10)", d.beam_id);
    CHECK(d.slot == 1, "slot=%d (expected 1)", d.slot);
    CHECK(d.sv_blocking == 0, "sv_blocking=%d (expected 0)", d.sv_blocking);

    CHECK(d.block1_ok, "block1_ok should be true");
    CHECK(d.block1_subtype == 1, "block1_subtype=%d (expected 1)", d.block1_subtype);
    CHECK(d.iri_time == 0x1A2B3C4Du, "iri_time=0x%08x (expected 0x1a2b3c4d)",
          d.iri_time);
}

int main(void)
{
    test_ibc_decode_orientation();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
