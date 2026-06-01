// Iridium frame top-level classifier — port of the dispatch logic from
// iridium-toolkit/bitsparser.py (Message.upgrade -> IridiumMessage).
//
// The bit-stream format we receive is the output of qpsk_demod_process:
// a 0/1-per-byte array where bits[0..23] are the 12-symbol UW and bits[24..]
// is the descrambled payload. Caller has already verified the UW
// matches IR_UW_DL or IR_UW_UL (qpsk_demod returns 1 only when it does)
// and reports `direction`.
//
// We classify by matching fixed header bit patterns and BCH polynomial
// checks. Phase A handled MS / TL via exact 32- / 96-bit prefix matches.
// Phase B (this revision) adds:
//   - BC dispatch  (6-bit hdr_poly=29 CRC + 64-bit double-BCH at poly=1207)
//   - LW dispatch  (46-bit interleaved LCW with three BCH checks at
//                   polys 29 / 41 / 465)
//   - LW sub-classification  via the 3-bit ft field in lcw1.

#include "iridium_frame.h"
#include "iridium_bch.h"
#include <string.h>

// 32-bit "messaging" header (BPSK 0x9669). Frames whose post-UW prefix
// matches this are MS-type.
static const uint8_t HEADER_MESSAGING[32] = {
    0,
    0,
    1,
    1,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    0,
    0,
    1,
    1,
    0,
    0,
    1,
    1,
    0,
    0,
    1,
    1,
    1,
    1,
    1,
    1,
    0,
    0,
    1,
    1,
};

// Canonical 24-bit Unique Word patterns (post-DQPSK Gray-remap, in
// the bit orientation qpsk_demod_process emits). Match upstream's
// iridium_access / uplink_access constants. iridium-toolkit's parser
// rejects any UW with even 1-bit error (default mode); we adopt the
// same strictness so iridium_frame_classify only forwards frames a
// downstream parser would also accept.
//
// qpsk_demod itself accepts up to 2 UW symbol errors at decode time
// (so the bits get emitted), but the strict gate here catches the
// border-line cases that a real production parser would drop.
static const uint8_t UW_DL[24] = {
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
static const uint8_t UW_UL[24] = {
    1,
    1,
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
    1,
    1,
    1,
    1,
    0,
    0,
};

// 96-bit "time/location" header: bits "11" + 94 zeros.
#define HEADER_TIME_LOCATION_LEN 96

// Minimum bit count for any classification. UW (24) + smallest header
// pattern we look at (32 = MS).
#define MIN_PAYLOAD_BITS 32
#define UW_BITS 24

static int bits_equal(const uint8_t *a, const uint8_t *b, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if ((a[i] & 1) != (b[i] & 1)) return 0;
    }
    return 1;
}

// Returns 1 iff bits[0]=1, bits[1]=1, bits[2..95]=0.
static int looks_like_time_location(const uint8_t *p, size_t n)
{
    if (n < HEADER_TIME_LOCATION_LEN) return 0;
    if ((p[0] & 1) != 1 || (p[1] & 1) != 1) return 0;
    for (size_t i = 2; i < HEADER_TIME_LOCATION_LEN; i++) {
        if (p[i] & 1) return 0;
    }
    return 1;
}

// --- BC (broadcast) check --------------------------------------------
// IBC frame has a 6-bit header (hdr_poly=29 CRC clean) followed by a
// 64-bit double-BCH block: two interleaved 31-bit BCH(31,21) codewords
// using ringalert poly=1207. iridium-toolkit/bitsparser.py:289-296.
#define BC_HDR_LEN 6
#define BC_BLOCK_LEN 64

// de_interleave for the 64-bit double-BCH block. Matches the upstream
// bitsparser.py de_interleave: the 64 input bits group into 32 2-bit
// "symbols" (with the second-of-pair coming first); we then unzip
// alternating symbols into two output streams (odd, even), each 32 bits
// = 16 symbols. Caller wants the first 31 bits of each as a BCH(31,21)
// codeword. Output buffers must each have room for 32 bytes.
static void de_interleave_pair(const uint8_t *in, size_t n_in,
                               uint8_t *odd_out, uint8_t *even_out)
{
    // n_in must be even. We don't call this with odd lengths.
    int n_sym = (int)(n_in / 2); // 32 for the 64-bit BC block
    // Upstream symbol mapping: symbol[i] = (in[2i+1], in[2i]).
    // 'even' takes symbols at indices n_sym-2, n_sym-4, ... down to 0.
    // 'odd'  takes symbols at indices n_sym-1, n_sym-3, ... down to 1.
    // Each output gets (n_sym/2) × 2 = n_sym bits total.
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

// 3-way de-interleave for RA frames. Mirrors iridium-toolkit's
// de_interleave3 (bitsparser.py:1991). Input is `n_in` bits arranged as
// `n_in/2` 2-bit symbols (sym[i] = in[2i+1] || in[2i]). The function
// distributes those symbols round-robin into three output codewords --
// "third" gets symbols at indices ..., 3, 0; "second" gets ..., 4, 1;
// "first" gets ..., 5, 2 (each walking down by 3).
//
// Each output gets ceil(n_in/6) symbols × 2 bits = ceil(n_in/3) bits.
// For RA's 96-bit input → three 32-bit outputs.
static void de_interleave3(const uint8_t *in, size_t n_in,
                           uint8_t *first_out, uint8_t *second_out,
                           uint8_t *third_out)
{
    int n_sym     = (int)(n_in / 2);
    int third_idx = 0, second_idx = 0, first_idx = 0;
    for (int s = n_sym - 3; s >= 0; s -= 3) {
        third_out[third_idx++] = in[2 * s + 1] & 1;
        third_out[third_idx++] = in[2 * s + 0] & 1;
    }
    for (int s = n_sym - 2; s >= 0; s -= 3) {
        second_out[second_idx++] = in[2 * s + 1] & 1;
        second_out[second_idx++] = in[2 * s + 0] & 1;
    }
    for (int s = n_sym - 1; s >= 0; s -= 3) {
        first_out[first_idx++] = in[2 * s + 1] & 1;
        first_out[first_idx++] = in[2 * s + 0] & 1;
    }
}

// RA (Ring Alert) classifier. iridium-toolkit/bitsparser.py:317.
// First 96 bits = 3 × 32 = 48 symbols, 3-way interleaved into three
// codewords. Each codeword is 31-bit BCH(31, 21) with poly=1207
// (ringalert poly) + 1 trailing parity bit. All three must divide
// cleanly to classify as RA.
#define RA_HEAD_BITS 96
static int classify_ra(const uint8_t *p, size_t avail)
{
    if (avail < RA_HEAD_BITS) return 0;
    uint8_t cw1[32], cw2[32], cw3[32];
    de_interleave3(p, RA_HEAD_BITS, cw1, cw2, cw3);
    if (iridium_bch_ndivide(1207u, cw1, 31) != 0) return 0;
    if (iridium_bch_ndivide(1207u, cw2, 31) != 0) return 0;
    if (iridium_bch_ndivide(1207u, cw3, 31) != 0) return 0;
    return 1;
}

static int classify_bc(const uint8_t *p, size_t avail)
{
    if (avail < BC_HDR_LEN + BC_BLOCK_LEN) return 0;
    // 6-bit header CRC clean against poly=29.
    if (iridium_bch_ndivide(29u, p, BC_HDR_LEN) != 0) return 0;
    // De-interleave the 64-bit block into odd/even 32-bit halves.
    uint8_t odd[32], even[32];
    de_interleave_pair(p + BC_HDR_LEN, BC_BLOCK_LEN, odd, even);
    // Each half: BCH(31,21) with poly=1207, must divide cleanly.
    // Matches upstream bitsparser.py:294-295 strict dispatch — ECC
    // repair is only invoked on the harder-mode path which we don't
    // run by default. Empirically on our corpus, switching to repair2
    // here produced 3 false-positive BC classifications (random RAW
    // lines that happened to satisfy the 2-bit-correctible code).
    if (iridium_bch_ndivide(1207u, odd, 31) != 0) return 0;
    if (iridium_bch_ndivide(1207u, even, 31) != 0) return 0;
    return 1;
}

// --- LW (Link Control Word) check ------------------------------------
// 46-bit LCW prefix interleaved per a fixed table into three BCH-protected
// words: lcw1=7 bits / poly=29, lcw2=13 bits / poly=465, lcw3=26 bits /
// poly=41. iridium-toolkit/bitsparser.py:466 + de_interleave_lcw at line
// 1998. lcw2 expects "n+1 trailing bit" — we try both 0 and 1, accept
// whichever passes (mirrors the upstream "One bit error expected" comment).
#define LW_LCW_BITS 46

// 1-based table from upstream, converted to 0-based here.
static const uint8_t LCW_TBL[LW_LCW_BITS] = {
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

static int classify_lw(const uint8_t *p, size_t avail, ir_lw_subtype_t *subtype_out)
{
    if (avail < LW_LCW_BITS) return 0;

    // Apply the 46-element permutation.
    uint8_t permuted[LW_LCW_BITS];
    for (size_t i = 0; i < LW_LCW_BITS; i++) {
        permuted[i] = p[LCW_TBL[i]] & 1;
    }
    // Slice into the three sub-words.
    uint8_t lcw1[7];
    uint8_t lcw3[26];
    uint8_t lcw2_pad[14]; // 13 + trailing-bit padding
    memcpy(lcw1, permuted + 0, 7);
    memcpy(lcw3, permuted + 20, 26);
    memcpy(lcw2_pad, permuted + 7, 13);

    // Upstream bitsparser.py:303-311 dispatch test for LW:
    //   - lcw1 must divide cleanly (no ECC repair)
    //   - lcw3 must divide cleanly (no ECC repair)
    //   - lcw2 + '0' bch_repair: if e2==1 try '1', accept only if e2==0
    if (iridium_bch_ndivide(29u, lcw1, 7) != 0) return 0;
    if (iridium_bch_ndivide(41u, lcw3, 26) != 0) return 0;
    // lcw2: append a guess-bit, run 2-error BCH repair. If exactly one
    // error came back, retry with the other guess. Accept only when
    // ZERO errors after repair (matches upstream's `if e2==0`).
    lcw2_pad[13] = 0;
    int e2       = iridium_bch_repair2(465u, lcw2_pad, 14);
    if (e2 == 1) {
        memcpy(lcw2_pad, permuted + 7, 13);
        lcw2_pad[13] = 1;
        e2           = iridium_bch_repair2(465u, lcw2_pad, 14);
    }
    if (e2 != 0) return 0;

    // ft = first 3 bits of lcw1 after BCH repair (the message bits;
    // remaining 4 bits are ECC). bch_repair1 is in-place — lcw1[0..2] is
    // now the cleaned 3-bit message.
    uint8_t ft = (uint8_t)((lcw1[0] << 2) | (lcw1[1] << 1) | lcw1[2]);
    if (subtype_out) *subtype_out = (ir_lw_subtype_t)ft;
    return 1;
}

int iridium_frame_classify(const uint8_t *bits, size_t n_bits,
                           ir_frame_direction_t direction,
                           iridium_frame_t     *out)
{
    if (!bits || !out) return -1;
    if (n_bits < UW_BITS) return -1;

    out->type        = IR_FRAME_UNKNOWN;
    out->direction   = direction;
    out->lw_subtype  = IR_LW_NONE;
    out->bits        = bits;
    out->n_bits      = n_bits;
    out->payload_off = UW_BITS;

    // Strict UW gate. qpsk_demod accepts up to 2 symbol errors at decode
    // time so noisy frames still get bits emitted, but a real-world
    // parser (iridium-toolkit's iridium-parser.py in default mode) only
    // accepts frames with an exact UW match. Mirror that strictness
    // here — leave UNKNOWN if the UW is corrupted, regardless of
    // whether the rest of the bits happen to satisfy MS / TL / BC / LW
    // structure. This tightens our corpus regression to 100% agreement
    // with the parser by rejecting frames the parser wouldn't process.
    const uint8_t *uw_expected = (direction == IR_FRM_DIR_UPLINK)
                                     ? UW_UL
                                     : UW_DL;
    if (!bits_equal(bits, uw_expected, UW_BITS)) {
        return 0; // UW corrupt — leave UNKNOWN
    }

    const uint8_t *p     = bits + UW_BITS;
    size_t         avail = n_bits - UW_BITS;

    if (avail < MIN_PAYLOAD_BITS) {
        return 0; // too short — leave UNKNOWN
    }

    // gr-iridium / our qpsk_demod emit each DQPSK symbol as two bits in
    // (high, low) order. iridium-toolkit's parser does symbol_reverse()
    // on every RAW: line at ingest, swapping adjacent pairs:
    //   bits [a0,a1,a2,a3,a4,a5,...] -> [a1,a0,a3,a2,a5,a4,...]
    // before any of the BCH / interleaver dispatch logic runs. Apply
    // the same swap into a stack buffer so our header constants and the
    // LCW de-interleaver table can match upstream verbatim.
    //
    // We swap only the prefix actually used by the dispatch (BC needs
    // 70 bits, LW needs 46, MS needs 32, TL needs 96). 128 bytes is
    // plenty and fits in the worker task's stack budget.
    uint8_t swapped[128];
    size_t  swap_len = avail < sizeof(swapped) ? avail : sizeof(swapped);
    // Round down to even — we swap in pairs.
    if (swap_len & 1) swap_len--;
    for (size_t i = 0; i + 1 < swap_len; i += 2) {
        swapped[i + 0] = p[i + 1] & 1;
        swapped[i + 1] = p[i + 0] & 1;
    }

    // Order of dispatch matches iridium-toolkit's bitsparser.py:
    //   1. MS  — header_messaging at offset 0 (32 bits exact match)
    //   2. TL  — header_time_location at offset 0 ("11" + 94 zeros)
    //   3. BC  — 6-bit hdr_poly=29 CRC + double BCH(31,21) poly=1207
    //   4. LW  — 46-bit interleaved LCW with three BCH checks
    //
    // We do NOT mimic upstream's frequency-class filtering — sub-
    // classification by frequency band belongs higher up.

    if (swap_len >= sizeof(HEADER_MESSAGING) && bits_equal(swapped, HEADER_MESSAGING, sizeof(HEADER_MESSAGING))) {
        out->type = IR_FRAME_MS;
        return 0;
    }

    if (looks_like_time_location(swapped, swap_len)) {
        out->type = IR_FRAME_TL;
        return 0;
    }

    if (classify_bc(swapped, swap_len)) {
        out->type = IR_FRAME_BC;
        return 0;
    }

    ir_lw_subtype_t lw_sub = IR_LW_NONE;
    if (classify_lw(swapped, swap_len, &lw_sub)) {
        out->type       = IR_FRAME_LW;
        out->lw_subtype = lw_sub;
        return 0;
    }

    // RA (Ring Alert): no header. 3 × 32-bit BCH(31,21) codewords
    // 3-way interleaved over the first 96 bits. iridium-toolkit/
    // bitsparser.py:317-324.
    if (classify_ra(swapped, swap_len)) {
        out->type = IR_FRAME_RA;
        return 0;
    }

    return 0;
}

const char *iridium_frame_type_name(ir_frame_type_t type)
{
    switch (type) {
    case IR_FRAME_MS:
        return "MS";
    case IR_FRAME_TL:
        return "TL";
    case IR_FRAME_BC:
        return "BC";
    case IR_FRAME_LW:
        return "LW";
    case IR_FRAME_RA:
        return "RA";
    case IR_FRAME_UNKNOWN:
        return "??";
    }
    return "??";
}

const char *iridium_lw_subtype_name(ir_lw_subtype_t subtype)
{
    switch (subtype) {
    case IR_LW_VO:
        return "VO";
    case IR_LW_IP:
        return "IP";
    case IR_LW_DA:
        return "DA";
    case IR_LW_U3:
        return "U3";
    case IR_LW_U4:
        return "U4";
    case IR_LW_U5:
        return "U5";
    case IR_LW_U6:
        return "U6";
    case IR_LW_SY:
        return "SY";
    case IR_LW_NONE:
        return "??";
    }
    return "??";
}
