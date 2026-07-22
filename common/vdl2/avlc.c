// avlc.c — VDL Mode 2 AVLC deframer: HDLC flag delimiting, bit-unstuffing,
// CRC-16/X-25 FCS, address/control parse, ACARS-vs-link-management
// classification. See avlc.h for the interface contract.
//
// Every protocol fact here is grounded in dumpvdl2 v2.6.0 (commit 3f583da,
// github.com/szpajder/dumpvdl2) — the cross-validation reference for this
// band, as gr-iridium is for Iridium. Per-step citations:
//
//   flag walk / unstuffing     src/bitstream.c:109-150 bitstream_copy_next_frame
//   LSB-first octet packing    src/bitstream.c:58-81, src/decode.c:326,355
//   octet-boundary frame check src/decode.c:346-349
//   FCS residue check          src/avlc.c:39-40,176-187 (GOOD_FCS 0xF0B8),
//                              src/crc.c:21-64 (reflected 0x1021, init 0xFFFF)
//   address field decode       src/avlc.c:159-162 parse_dlc_addr,
//                              src/avlc.h:29-42 avlc_addr_t,
//                              src/bitstream.c:152-164 reverse()
//   control field / I-S-U      src/avlc.c:49-100 lcf_t, IS_I/IS_S/IS_U
//   ACARS discriminator        src/avlc.c:255-262 (I frame, info begins
//                              0xFF 0xFF 0x01; ACARS block starts at +3 and
//                              goes straight to la_acars_parse_and_reassemble
//                              — src/acars.c:100-114)
//
// The deframing logic is a re-implementation in this repo's style, not a
// copy (GPL hygiene per the plan §GPL-3): behaviourally pinned to the
// reference by tests/host/test_avlc.c, including a differential run against
// dumpvdl2's own bitstream.c recorded in that test's header comment.

#include "avlc.h"

#include <string.h>

#include "crc16.h" // crc16_x25_raw — common/iridium_decoder (shared FCS home)

// Destuffed-frame assembly buffer. Static, not stack: 2 KB would be a large
// bite out of an ESP task stack, and the only caller is a single task (the
// Core-0 frame_decoder task — plan §C4) / single-threaded host tests, the
// same s_-static single-consumer pattern the Iridium L2 modules use.
// NOT reentrant; documented in avlc.h.
static uint8_t s_frame[AVLC_MAX_FRAME_OCTETS];

// ---------------------------------------------------------------------------
// Address field
// ---------------------------------------------------------------------------

// Bit-reverse the low 28 bits of v (result bit k = input bit 27-k).
// Equivalent to dumpvdl2's reverse(v, 28) (src/bitstream.c:152-164).
static uint32_t reverse28(uint32_t v)
{
    uint32_t r = 0;
    for (int k = 0; k < 28; k++) {
        r |= ((v >> k) & 1u) << (27 - k);
    }
    return r;
}

// Decode one 4-octet AVLC address field into the packed 28-bit form
//   [27] status (C/R in the source field, source Air/Ground in the
//        destination field)   [26:24] type   [23:0] specific address.
// Formula is dumpvdl2's parse_dlc_addr (src/avlc.c:159-162) verbatim: each
// octet contributes its upper 7 bits (LSB of each octet is the HDLC address
// extension bit, 0 on the first three octets, 1 on the last), assembled
// low-octet-first then bit-reversed across the 28-bit word. The masks make
// the extension bits of octets 1 and 2 vanish by construction only when
// they are 0 — matching the reference, we do not validate them (real-air
// confirmation item, see test_avlc.c).
static uint32_t parse_dlc_addr(const uint8_t *b)
{
    uint32_t v = (uint32_t)(b[0] >> 1) |
                 ((uint32_t)b[1] << 6) |
                 ((uint32_t)b[2] << 13) |
                 (((uint32_t)b[3] & 0xFEu) << 20);
    return reverse28(v);
}

// ---------------------------------------------------------------------------
// Frame parse + classify (mirrors dumpvdl2 avlc_parse, src/avlc.c:164-265)
// ---------------------------------------------------------------------------

static void parse_and_emit(const uint8_t *buf, int len,
                           avlc_frame_cb_t cb, void *ctx)
{
    avlc_frame_t f;
    memset(&f, 0, sizeof(f));
    f.raw     = buf;
    f.raw_len = len;

    if (len < AVLC_MIN_FRAME_OCTETS) {
        f.kind = AVLC_KIND_TOO_SHORT;
        if (cb) cb(&f, ctx);
        return;
    }
    // HDLC residue check over the whole frame INCLUDING the FCS octets —
    // dumpvdl2 src/avlc.c:177-179.
    if (crc16_x25_raw(buf, (size_t)len) != CRC16_X25_GOOD_RESIDUE) {
        f.kind = AVLC_KIND_BAD_FCS;
        if (cb) cb(&f, ctx);
        return;
    }
    f.fcs_ok = true;

    int plen = len - 2; // strip FCS from the parsed view

    uint32_t dst = parse_dlc_addr(buf);     // destination field first on air
    uint32_t src = parse_dlc_addr(buf + 4); // then source
    f.dst_addr = dst & 0xFFFFFFu;
    f.src_addr = src & 0xFFFFFFu;
    f.dst_type = (uint8_t)((dst >> 24) & 0x7u);
    f.src_type = (uint8_t)((src >> 24) & 0x7u);
    // Status bits: src field carries C/R; dst field carries the SOURCE's
    // Air/Ground status (dumpvdl2 src/avlc.c:341,420 + status_*_descr).
    f.response      = ((src >> 27) & 1u) != 0;
    f.src_on_ground = ((dst >> 27) & 1u) != 0;

    f.control = buf[8];
    f.info     = buf + 9;
    f.info_len = plen - 9; // >= 0 (plen >= 9 given len >= 11)

    if ((f.control & 0x01u) == 0) {
        // I frame — the only AVLC frame type that carries ACARS. The
        // ACARS-over-AVLC discriminator is the info field starting
        // 0xFF 0xFF 0x01 with at least one ACARS octet after it; anything
        // else in an I frame is ATN/X.25 (dumpvdl2 src/avlc.c:255-262).
        if (f.info_len > 3 && f.info[0] == 0xFFu && f.info[1] == 0xFFu &&
            f.info[2] == 0x01u) {
            f.kind      = AVLC_KIND_ACARS;
            f.acars     = f.info + 3;
            f.acars_len = f.info_len - 3;
        } else {
            f.kind = AVLC_KIND_X25;
        }
    } else if ((f.control & 0x03u) == 0x01u) {
        f.kind = AVLC_KIND_SUPERVISORY;
        if (f.info_len == 0) f.info = NULL; // S frames normally carry none
    } else { // (control & 3) == 3
        f.kind = AVLC_KIND_UNNUMBERED; // XID/TEST/DISC/... — link management
    }

    if (cb) cb(&f, ctx);
}

// ---------------------------------------------------------------------------
// Flag walk + unstuffing
// ---------------------------------------------------------------------------

// State-machine port of dumpvdl2's bitstream_copy_next_frame
// (src/bitstream.c:109-150), driven over the whole burst in one call:
//
//  - a 0 after exactly five 1s is a stuffed bit: dropped, not copied;
//  - seven or more consecutive 1s abort the burst (invalid sequence);
//  - a 0 completing six 1s is a flag (0x7E = 01111110 — palindromic, so
//    LSB-first order doesn't matter):
//      * if the flag is exactly the first 8 destuffed bits, it is a leading
//        flag: discard and restart (an opening flag is therefore OPTIONAL —
//        frames back-to-back after a closing flag parse fine, which is how
//        multi-frame VDL2 bursts arrive);
//      * six 1s within the first 7 bits (i.e. a truncated leading flag)
//        abort;
//      * otherwise it closes the frame: the 8 flag bits are stripped and
//        the frame is emitted;
//  - stream exhaustion emits whatever accumulated as a final frame with no
//    closing flag (dumpvdl2 does the same; it normally fails FCS unless the
//    burst really did end flagless-clean);
//  - every emitted frame must be a whole number of octets after
//    destuffing, else the burst aborts (src/decode.c:346-349).
//
// `packed` selects the input bit accessor: RS-output octets (LSB-first) or
// one-bit-per-byte vectors. Identical scan either way.
static int deframe_core(const uint8_t *stream, int n_bits, bool packed,
                        avlc_frame_cb_t cb, void *ctx)
{
    if (stream == NULL || n_bits <= 0) return 0;

    int n_frames = 0;
    int ones     = 0; // consecutive-1s run length
    int j        = 0; // destuffed bit count in the current frame
    // True when the final input bit closed a frame via a flag — then there
    // is no residual to emit (dumpvdl2: copy_next_frame returns 0 and the
    // decode loop breaks without another call, src/decode.c:345,363-365).
    bool closed_at_end = false;

    for (int i = 0; i < n_bits; i++) {
        int bit = packed ? ((stream[i >> 3] >> (i & 7)) & 1)
                         : (stream[i] & 1);

        if (bit == 0 && ones == 5) { // stuffed 0 — drop it
            ones = 0;
            continue;
        }
        if (bit == 1) {
            ones++;
            if (ones > 6) return n_frames; // >= 7 ones: abort burst
        }

        bool emit = false;
        if (bit == 0) {
            if (ones == 6) {   // flag completed (this 0 is its final bit)
                if (j == 7) {  // leading flag: discard, restart frame
                    j = ones = 0;
                    continue;
                }
                if (j < 7) return n_frames; // truncated flag at frame start
                j -= 7;      // strip the 7 already-stored flag bits;
                emit = true; // the closing 0 was never stored
            }
            ones = 0;
        }

        if (!emit) {
            // store destuffed bit LSB-first
            if (j >= AVLC_MAX_FRAME_OCTETS * 8) return n_frames; // oversize
            if ((j & 7) == 0) s_frame[j >> 3] = 0;
            s_frame[j >> 3] |= (uint8_t)(bit << (j & 7));
            j++;
            continue;
        }

        // emit the flag-closed frame
        if ((j & 7) != 0) return n_frames; // not octet-aligned: abort burst
        parse_and_emit(s_frame, j >> 3, cb, ctx);
        n_frames++;
        j             = 0;
        closed_at_end = (i == n_bits - 1);
    }

    if (closed_at_end) return n_frames;

    // Stream exhausted mid-frame: the residual bits form a final, flagless
    // frame (dumpvdl2 emits it too; it is usually TOO_SHORT/BAD_FCS noise
    // and gets counted, not parsed).
    if ((j & 7) != 0) return n_frames; // truncated mid-octet: abort
    parse_and_emit(s_frame, j >> 3, cb, ctx);
    return n_frames + 1;
}

int avlc_deframe_octets(const uint8_t *octets, int n_bits,
                        avlc_frame_cb_t cb, void *ctx)
{
    return deframe_core(octets, n_bits, true, cb, ctx);
}

int avlc_deframe(const uint8_t *bits, int n_bits,
                 avlc_frame_cb_t cb, void *ctx)
{
    return deframe_core(bits, n_bits, false, cb, ctx);
}
