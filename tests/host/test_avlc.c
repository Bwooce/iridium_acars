// test_avlc.c — host tests for the VDL Mode 2 AVLC deframer
// (common/vdl2/avlc.c): synthetic frame construction (addresses + control +
// info + CRC-16/X-25 FCS, bit-stuffed, flag-delimited) round-tripped through
// the production deframe path.
//
// TWO VECTOR SETS:
//  (1) REAL frames — fixture_vdl2_avlc_golden.h: 40 over-the-air AVLC
//      frames (sigidwiki 136.975 MHz capture) with dumpvdl2 2.6.0's parse
//      as ground truth (raw octets incl. FCS + every decoded field). These
//      pin the real address encoding, control-field values and the ACARS
//      discriminator against live traffic. Regenerate with
//      tests/scripts/build_vdl2_avlc_fixture.py.
//  (2) SYNTHETIC frames — built by the independent encoder below for the
//      cases real captures can't systematically cover: FCS rejection on
//      every octet, stuffing edge cases, invalid sequences, multi-frame
//      layouts.
// The builder is an independent implementation of the AVLC frame layout
// as specified by dumpvdl2 v2.6.0 (commit 3f583da):
//   address field encode = inverse of src/avlc.c:159-162 parse_dlc_addr
//   control I/S/U layout = src/avlc.c:49-100
//   ACARS discriminator  = src/avlc.c:255-262 (0xFF 0xFF 0x01 + block)
//   stuffing/flags       = src/bitstream.c:109-150
//   FCS                  = src/avlc.c:39-40,176-187 (residue 0xF0B8)
// Frame-level identity vs dumpvdl2's own output on REAL captures is the V1
// gate (tests/scripts/stagewise_compare.py precedent); items that only real
// air can confirm are listed at the bottom of this comment.
//
// DIFFERENTIAL CROSS-CHECK (run once during development, 2026-07-22, not a
// committed dependency because dumpvdl2 is GPL and not vendored): a harness
// compiling dumpvdl2's pristine src/bitstream.c + src/crc.c against stub
// headers fed 20000 random bit streams (random frames, stuffing, junk
// prefixes/suffixes, bit flips) to bitstream_copy_next_frame + crc16_ccitt
// and to avlc_deframe(): identical frame boundaries, frame bytes and FCS
// verdicts on every stream.
//
// CLOSED BY THE REAL VECTORS (previously real-capture-confirmation items):
//   - address field bit order + status/type bit placement (40 live frames,
//     aircraft/GS/broadcast mix, decode identical to dumpvdl2's);
//   - control-field layouts (real I/S/U frames incl. an SREJ carrying a
//     16-byte info field — S-frames with data DO fly);
//   - the 0xFF 0xFF 0x01 ACARS discriminator on live I frames (9 ACARS
//     frames, and every non-ACARS I frame correctly falls to X.25).
// STILL NEEDS RAW-AIR CONFIRMATION (V1, needs the pre-destuff bitstream,
// which no text decode can show):
//   - TX-side EA (address extension) bit values: dumpvdl2 masks them out
//     on parse and the golden hexdumps are post-parse octets, so the
//     builder's EA=0,0,0,1 remains the HDLC-spec assumption;
//   - real bursts' use of leading flags / back-to-back frame packing /
//     inter-frame fill (our flag-walk semantics are pinned to dumpvdl2's
//     bitstream code by the differential harness instead);
//   - whether ACARS ever rides frames other than I on air (we mirror
//     dumpvdl2: I frames only).

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "avlc.h"
#include "crc16.h"
#include "fixture_vdl2_avlc_golden.h" // 40 real AVLC frames (dumpvdl2 truth)

static int s_pass = 0, s_fail = 0;
#define CHECK(cond, ...)                                   \
    do {                                                   \
        if (!(cond)) {                                     \
            fprintf(stderr, "  FAIL line %d: ", __LINE__); \
            fprintf(stderr, __VA_ARGS__);                  \
            fprintf(stderr, "\n");                         \
            s_fail++;                                      \
        } else {                                           \
            s_pass++;                                      \
        }                                                  \
    } while (0)

// Deterministic LCG (same convention as the other host tests).
static uint32_t s_rng = 0x5EED5EEDu;
static uint8_t rnd(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return (uint8_t)(s_rng >> 24);
}

// ---------------------------------------------------------------------------
// Synthetic AVLC builder (independent of the production code)
// ---------------------------------------------------------------------------

// Independent 28-bit reversal (matches dumpvdl2 reverse(v,28)).
static uint32_t rev28(uint32_t v)
{
    uint32_t r = 0;
    for (int k = 0; k < 28; k++) r |= ((v >> k) & 1u) << (27 - k);
    return r;
}

// Encode one 4-octet AVLC address field — the exact inverse of dumpvdl2's
// parse_dlc_addr. EA (extension) bits 0,0,0,1; each octet carries 7 address
// bits in its upper bits.
static void put_addr(uint8_t *b, uint32_t addr, uint8_t type, int status)
{
    uint32_t val = ((uint32_t)(status & 1) << 27) |
                   ((uint32_t)(type & 7) << 24) | (addr & 0xFFFFFFu);
    uint32_t v = rev28(val); // rev28 is involutive
    b[0]       = (uint8_t)((v & 0x7Fu) << 1);
    b[1]       = (uint8_t)(((v >> 7) & 0x7Fu) << 1);
    b[2]       = (uint8_t)(((v >> 14) & 0x7Fu) << 1);
    b[3]       = (uint8_t)((((v >> 21) & 0x7Fu) << 1) | 1u);
}

// Assemble frame octets: dst + src + control + info + FCS (low byte first).
// Returns total length.
static int build_frame(uint8_t *out,
                       uint32_t dst_addr, uint8_t dst_type, int src_on_ground,
                       uint32_t src_addr, uint8_t src_type, int response,
                       uint8_t control, const uint8_t *info, int info_len)
{
    put_addr(out, dst_addr, dst_type, src_on_ground);
    put_addr(out + 4, src_addr, src_type, response);
    out[8] = control;
    if (info_len > 0) memcpy(out + 9, info, (size_t)info_len);
    int      n   = 9 + info_len;
    uint16_t fcs = crc16_x25(out, (size_t)n);
    out[n++]     = (uint8_t)(fcs & 0xFF);
    out[n++]     = (uint8_t)(fcs >> 8);
    return n;
}

// Bit-stream builder: unpacked bits (one per byte) with HDLC stuffing.
typedef struct {
    uint8_t bits[64 * 1024];
    int     n;
    int     ones; // consecutive-1 run for the stuffer (persists across octets)
} bitvec_t;

static void bv_reset(bitvec_t *v) { v->n = 0; v->ones = 0; }

static void bv_push_raw(bitvec_t *v, int bit) { v->bits[v->n++] = (uint8_t)bit; }

// A flag is emitted verbatim (never stuffed) and resets the stuffer run.
static void bv_flag(bitvec_t *v)
{
    static const uint8_t flag[8] = { 0, 1, 1, 1, 1, 1, 1, 0 };
    for (int i = 0; i < 8; i++) bv_push_raw(v, flag[i]);
    v->ones = 0;
}

// Append frame octets LSB-first with bit stuffing (0 inserted after five 1s).
static void bv_stuffed_octets(bitvec_t *v, const uint8_t *oct, int n)
{
    for (int i = 0; i < n; i++) {
        for (int b = 0; b < 8; b++) {
            int bit = (oct[i] >> b) & 1;
            bv_push_raw(v, bit);
            if (bit) {
                if (++v->ones == 5) {
                    bv_push_raw(v, 0); // stuffed bit
                    v->ones = 0;
                }
            } else {
                v->ones = 0;
            }
        }
    }
}

// Pack an unpacked bit vector into octets LSB-first (the RS-output form).
static int bv_pack(const bitvec_t *v, uint8_t *oct, int cap)
{
    int n_oct = (v->n + 7) / 8;
    if (n_oct > cap) return -1;
    memset(oct, 0, (size_t)n_oct);
    for (int i = 0; i < v->n; i++)
        oct[i >> 3] |= (uint8_t)((v->bits[i] & 1) << (i & 7));
    return n_oct;
}

// ---------------------------------------------------------------------------
// Callback capture
// ---------------------------------------------------------------------------

#define MAX_CAP 16
typedef struct {
    avlc_frame_t f[MAX_CAP];
    uint8_t      raw[MAX_CAP][AVLC_MAX_FRAME_OCTETS];
    uint8_t      acars[MAX_CAP][512];
    int          n;
} capture_t;

static void cap_cb(const avlc_frame_t *f, void *ctx)
{
    capture_t *c = ctx;
    if (c->n >= MAX_CAP) return;
    c->f[c->n] = *f;
    if (f->raw && f->raw_len > 0 && f->raw_len <= AVLC_MAX_FRAME_OCTETS) {
        memcpy(c->raw[c->n], f->raw, (size_t)f->raw_len);
        c->f[c->n].raw = c->raw[c->n];
    }
    if (f->acars && f->acars_len > 0 && f->acars_len <= 512) {
        memcpy(c->acars[c->n], f->acars, (size_t)f->acars_len);
        c->f[c->n].acars = c->acars[c->n];
    }
    c->n++;
}

// Run both entry points over the same stream; check they agree; return the
// octet-entry capture.
static int deframe_both(const bitvec_t *v, capture_t *cap)
{
    capture_t cap_bits;
    memset(cap, 0, sizeof(*cap));
    memset(&cap_bits, 0, sizeof(cap_bits));

    uint8_t oct[8 * 1024];
    int     n_oct = bv_pack(v, oct, (int)sizeof(oct));
    CHECK(n_oct >= 0, "stream too large to pack");

    int r_oct  = avlc_deframe_octets(oct, v->n, cap_cb, cap);
    int r_bits = avlc_deframe(v->bits, v->n, cap_cb, &cap_bits);

    CHECK(r_oct == r_bits, "entry points disagree: octets=%d bits=%d", r_oct,
          r_bits);
    CHECK(cap->n == cap_bits.n, "capture counts disagree: %d vs %d", cap->n,
          cap_bits.n);
    for (int i = 0; i < cap->n && i < cap_bits.n; i++) {
        CHECK(cap->f[i].kind == cap_bits.f[i].kind &&
                  cap->f[i].raw_len == cap_bits.f[i].raw_len &&
                  memcmp(cap->raw[i], cap_bits.raw[i],
                         (size_t)cap->f[i].raw_len) == 0,
              "frame %d differs between entry points", i);
    }
    return r_oct;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// The ACARS block used throughout: mode 2, reg .VH-ABC — content is opaque
// to the deframer (libacars owns it downstream); only byte identity matters.
static const uint8_t k_acars[] = { 0x32, 0x2E, 0x56, 0x48, 0x2D, 0x41, 0x42,
                                   0x43, 0x15, 0x51, 0x30, 0x0A, 0x01, 0x33,
                                   0x34, 0x37, 0x03 };

static void test_acars_roundtrip(void)
{
    // I frame, ground station -> aircraft, ACARS discriminator + block.
    uint8_t info[3 + sizeof(k_acars)];
    info[0] = 0xFF;
    info[1] = 0xFF;
    info[2] = 0x01;
    memcpy(info + 3, k_acars, sizeof(k_acars));

    uint8_t frame[256];
    // control: I frame, sseq=2 rseq=5 poll=1 -> 0b101'1'010'0 = 0xB4
    int n = build_frame(frame, 0x7C1234, AVLC_ADDRTYPE_AIRCRAFT, 0,
                        0x21406A, AVLC_ADDRTYPE_GS_ADM, 1, 0xB4, info,
                        (int)sizeof(info));

    bitvec_t v;
    bv_reset(&v);
    bv_flag(&v);
    bv_stuffed_octets(&v, frame, n);
    bv_flag(&v);

    capture_t cap;
    int       r = deframe_both(&v, &cap);
    CHECK(r == 1 && cap.n == 1, "want exactly 1 frame, got %d", cap.n);
    if (cap.n < 1) return;
    const avlc_frame_t *f = &cap.f[0];
    CHECK(f->kind == AVLC_KIND_ACARS, "kind=%d want ACARS", (int)f->kind);
    CHECK(f->fcs_ok, "fcs_ok must be true");
    CHECK(f->raw_len == n, "raw_len=%d want %d", f->raw_len, n);
    CHECK(f->dst_addr == 0x7C1234 && f->dst_type == AVLC_ADDRTYPE_AIRCRAFT,
          "dst=%06X/%u", f->dst_addr, f->dst_type);
    CHECK(f->src_addr == 0x21406A && f->src_type == AVLC_ADDRTYPE_GS_ADM,
          "src=%06X/%u", f->src_addr, f->src_type);
    CHECK(f->response == true, "C/R must be response");
    CHECK(f->src_on_ground == false, "A/G must be airborne");
    CHECK(f->control == 0xB4, "control=0x%02X", f->control);
    CHECK(f->info_len == (int)sizeof(info), "info_len=%d", f->info_len);
    CHECK(f->acars_len == (int)sizeof(k_acars) &&
              memcmp(f->acars, k_acars, sizeof(k_acars)) == 0,
          "ACARS payload must survive the round trip byte-exact");
}

static void test_classification(void)
{
    static const uint8_t x25_info[] = { 0x10, 0x01, 0x02, 0x0B, 0x00 };
    static const uint8_t acars3[]   = { 0xFF, 0xFF, 0x01 }; // no block!
    static const uint8_t xid_info[] = { 0x82, 0x00, 0x0D, 0x01, 0x02, 0x00,
                                        0x00, 0x10 };
    const struct {
        uint8_t           control;
        const uint8_t    *info;
        int               info_len;
        avlc_frame_kind_t want;
    } cases[6] = {
        { 0x00, x25_info, 5, AVLC_KIND_X25 },
        // discriminator with NOTHING after it is not ACARS (dumpvdl2 len>3)
        { 0x00, acars3, 3, AVLC_KIND_X25 },
        // S frame: Receive Ready, pf=1, rseq=3 -> 0b011'1'00'01 = 0x71
        { 0x71, NULL, 0, AVLC_KIND_SUPERVISORY },
        // U frame XID: mfunc 0x2B, P/F=1 -> ((0x2B|0x04)<<2)|3 = 0xBF
        { 0xBF, xid_info, 8, AVLC_KIND_UNNUMBERED },
        // U frame TEST: mfunc 0x38 -> (0x38<<2)|3 = 0xE3
        { 0xE3, NULL, 0, AVLC_KIND_UNNUMBERED },
        // minimal I frame, empty info -> X25 bucket
        { 0x00, NULL, 0, AVLC_KIND_X25 },
    };

    for (int t = 0; t < 6; t++) {
        uint8_t frame[128];
        int n = build_frame(frame, 0xC88888, AVLC_ADDRTYPE_ALL, 1, 0x7C0FED,
                            AVLC_ADDRTYPE_AIRCRAFT, 0, cases[t].control,
                            cases[t].info, cases[t].info_len);
        bitvec_t v;
        bv_reset(&v);
        bv_flag(&v);
        bv_stuffed_octets(&v, frame, n);
        bv_flag(&v);
        capture_t cap;
        deframe_both(&v, &cap);
        CHECK(cap.n == 1, "case %d: got %d frames", t, cap.n);
        if (cap.n < 1) continue;
        CHECK(cap.f[0].kind == cases[t].want, "case %d: kind=%d want %d", t,
              (int)cap.f[0].kind, (int)cases[t].want);
        CHECK(cap.f[0].info_len == cases[t].info_len,
              "case %d: info_len=%d want %d", t, cap.f[0].info_len,
              cases[t].info_len);
        // min-length frame check rides on case 5
        if (t == 5)
            CHECK(cap.f[0].raw_len == AVLC_MIN_FRAME_OCTETS,
                  "empty-info frame must be exactly %d octets, got %d",
                  AVLC_MIN_FRAME_OCTETS, cap.f[0].raw_len);
    }
}

static void test_fcs_reject(void)
{
    uint8_t info[8] = { 0xFF, 0xFF, 0x01, 'T', 'E', 'S', 'T', '.' };
    uint8_t frame[64];
    int n = build_frame(frame, 0x7C9999, AVLC_ADDRTYPE_AIRCRAFT, 1, 0x114488,
                        AVLC_ADDRTYPE_GS_DEL, 0, 0x00, info, 8);

    // Corrupt each octet in turn (payload, addresses, control, FCS itself):
    // every one must yield BAD_FCS, never a parsed frame.
    for (int corrupt = 0; corrupt < n; corrupt++) {
        uint8_t mangled[64];
        memcpy(mangled, frame, (size_t)n);
        mangled[corrupt] ^= 0x24;
        bitvec_t v;
        bv_reset(&v);
        bv_flag(&v);
        bv_stuffed_octets(&v, mangled, n);
        bv_flag(&v);
        capture_t cap;
        deframe_both(&v, &cap);
        CHECK(cap.n == 1, "corrupt@%d: got %d frames", corrupt, cap.n);
        if (cap.n < 1) continue;
        CHECK(cap.f[0].kind == AVLC_KIND_BAD_FCS && !cap.f[0].fcs_ok,
              "corrupt@%d: kind=%d fcs_ok=%d — corruption must be rejected",
              corrupt, (int)cap.f[0].kind, (int)cap.f[0].fcs_ok);
        CHECK(cap.f[0].acars == NULL && cap.f[0].info == NULL,
              "corrupt@%d: parsed fields must stay clear on bad FCS",
              corrupt);
    }
}

static void test_stuffing_torture(void)
{
    // Payload dense in long 1-runs and raw flag patterns: 0xFF runs force
    // stuffing (including runs spanning octet boundaries), 0x7E in DATA is
    // six consecutive 1s on air — only stuffing prevents a false flag.
    uint8_t info[64];
    info[0] = 0xFF;
    info[1] = 0xFF;
    info[2] = 0x01;
    for (int i = 3; i < 32; i++) info[i] = 0xFF;
    for (int i = 32; i < 48; i++) info[i] = 0x7E;
    // alternating five-1s straddles: 0xF8 0x1F = 11111000 / 00011111
    for (int i = 48; i < 64; i += 2) {
        info[i]     = 0xF8;
        info[i + 1] = 0x1F;
    }

    uint8_t frame[128];
    int n = build_frame(frame, 0xFFFFFF, AVLC_ADDRTYPE_ALL, 0, 0x7FFFFF,
                        AVLC_ADDRTYPE_ALL, 1, 0xFE, info, 64);

    bitvec_t v;
    bv_reset(&v);
    bv_flag(&v);
    bv_stuffed_octets(&v, frame, n);
    bv_flag(&v);
    // stuffing must have inserted bits: stream longer than raw
    CHECK(v.n > 16 + n * 8, "expected stuffed bits (n=%d bits=%d)", n, v.n);

    capture_t cap;
    int       r = deframe_both(&v, &cap);
    CHECK(r == 1 && cap.n == 1, "torture: got %d frames", cap.n);
    if (cap.n < 1) return;
    CHECK(cap.f[0].fcs_ok && cap.f[0].raw_len == n &&
              memcmp(cap.raw[0], frame, (size_t)n) == 0,
          "torture frame must round-trip byte-exact");
    // control 0xFE is even -> I frame; info begins FF FF 01 -> ACARS
    CHECK(cap.f[0].kind == AVLC_KIND_ACARS, "torture kind=%d",
          (int)cap.f[0].kind);
}

static void test_multiframe_and_tail(void)
{
    uint8_t f1[64], f2[64];
    static const uint8_t i1[] = { 0xFF, 0xFF, 0x01, 'A' };
    static const uint8_t i2[] = { 0x10, 0x02 };
    int n1 = build_frame(f1, 0x7C0001, AVLC_ADDRTYPE_AIRCRAFT, 0, 0x224488,
                         AVLC_ADDRTYPE_GS_ADM, 0, 0x00, i1, 4);
    int n2 = build_frame(f2, 0x224488, AVLC_ADDRTYPE_GS_ADM, 0, 0x7C0001,
                         AVLC_ADDRTYPE_AIRCRAFT, 1, 0x02, i2, 2);

    // (a) flag f1 flag f2 flag — the CSMA back-to-back layout: exactly two
    //     frames, no phantom tail frame after the final flag.
    {
        bitvec_t v;
        bv_reset(&v);
        bv_flag(&v);
        bv_stuffed_octets(&v, f1, n1);
        bv_flag(&v);
        bv_stuffed_octets(&v, f2, n2);
        bv_flag(&v);
        capture_t cap;
        int       r = deframe_both(&v, &cap);
        CHECK(r == 2 && cap.n == 2, "(a) got %d frames want 2", cap.n);
        if (cap.n == 2) {
            CHECK(cap.f[0].kind == AVLC_KIND_ACARS &&
                      cap.f[1].kind == AVLC_KIND_X25,
                  "(a) kinds %d,%d", (int)cap.f[0].kind, (int)cap.f[1].kind);
            CHECK(cap.f[1].raw_len == n2 &&
                      memcmp(cap.raw[1], f2, (size_t)n2) == 0,
                  "(a) frame 2 bytes");
        }
    }

    // (b) no leading flag at all (deframer must not require one).
    {
        bitvec_t v;
        bv_reset(&v);
        bv_stuffed_octets(&v, f1, n1);
        bv_flag(&v);
        capture_t cap;
        int       r = deframe_both(&v, &cap);
        CHECK(r == 1 && cap.n == 1 && cap.f[0].kind == AVLC_KIND_ACARS,
              "(b) flagless-open frame must parse");
    }

    // (c) residual octet junk after the last flag: emitted as a final
    //     non-parsed frame (dumpvdl2 parity), never as ACARS.
    {
        bitvec_t v;
        bv_reset(&v);
        bv_flag(&v);
        bv_stuffed_octets(&v, f1, n1);
        bv_flag(&v);
        static const uint8_t junk[2] = { 0x12, 0x34 };
        bv_stuffed_octets(&v, junk, 2);
        capture_t cap;
        int       r = deframe_both(&v, &cap);
        CHECK(r == 2 && cap.n == 2, "(c) got %d frames want 2", cap.n);
        if (cap.n == 2)
            CHECK(cap.f[1].kind == AVLC_KIND_TOO_SHORT,
                  "(c) tail kind=%d want TOO_SHORT", (int)cap.f[1].kind);
    }

    // (d) repeated flags (idle fill) between frames are transparent.
    {
        bitvec_t v;
        bv_reset(&v);
        bv_flag(&v);
        bv_flag(&v);
        bv_flag(&v);
        bv_stuffed_octets(&v, f2, n2);
        bv_flag(&v);
        capture_t cap;
        int       r = deframe_both(&v, &cap);
        CHECK(r == 1 && cap.n == 1 && cap.f[0].kind == AVLC_KIND_X25,
              "(d) flag fill must be transparent, got %d frames", cap.n);
    }
}

static void test_invalid_sequences(void)
{
    uint8_t f1[32];
    static const uint8_t i1[] = { 0xFF, 0xFF, 0x01, 'B' };
    int n1 = build_frame(f1, 0x7C0002, AVLC_ADDRTYPE_AIRCRAFT, 0, 0x224488,
                         AVLC_ADDRTYPE_GS_ADM, 0, 0x00, i1, 4);

    // (a) seven consecutive ones mid-stream: frames before the violation
    //     are kept, everything after is dropped (whole-burst abort).
    {
        bitvec_t v;
        bv_reset(&v);
        bv_flag(&v);
        bv_stuffed_octets(&v, f1, n1);
        bv_flag(&v);
        for (int i = 0; i < 7; i++) bv_push_raw(&v, 1);
        bv_push_raw(&v, 0);
        bv_flag(&v);
        bv_stuffed_octets(&v, f1, n1); // must never be seen
        bv_flag(&v);
        capture_t cap;
        int       r = deframe_both(&v, &cap);
        CHECK(r == 1 && cap.n == 1,
              "(a) 7-ones abort: got %d frames want 1", cap.n);
    }

    // (b) a destuffed frame that is not a whole number of octets: abort,
    //     nothing after the violation emitted.
    {
        bitvec_t v;
        bv_reset(&v);
        bv_flag(&v);
        bv_stuffed_octets(&v, f1, n1);
        bv_push_raw(&v, 0); // 1 extra bit -> misaligned before closing flag
        bv_flag(&v);
        capture_t cap;
        int       r = deframe_both(&v, &cap);
        CHECK(r == 0 && cap.n == 0,
              "(b) misaligned frame must abort, got %d", cap.n);
    }

    // (c) empty / NULL input.
    {
        capture_t cap;
        memset(&cap, 0, sizeof(cap));
        CHECK(avlc_deframe_octets(NULL, 128, cap_cb, &cap) == 0 &&
                  avlc_deframe_octets((const uint8_t *)"", 0, cap_cb, &cap) ==
                      0 &&
                  cap.n == 0,
              "(c) NULL/empty must emit nothing");
    }

    // (d) bare flags only: dumpvdl2 emits one empty (too-short) residual
    //     after the leading-flag restarts consume the stream.
    {
        bitvec_t v;
        bv_reset(&v);
        bv_flag(&v);
        bv_flag(&v);
        capture_t cap;
        int       r = deframe_both(&v, &cap);
        CHECK(r == 1 && cap.n == 1 && cap.f[0].kind == AVLC_KIND_TOO_SHORT &&
                  cap.f[0].raw_len == 0,
              "(d) bare flags: want one empty TOO_SHORT, got %d frames",
              cap.n);
    }
}

static void test_random_roundtrip(void)
{
    // 2000 random frames (random addresses/control/info incl. 1-run-dense
    // bytes, random info lengths 0..200) through stuff -> deframe. Every
    // frame must come back byte-exact with FCS OK. Exercises stuffed bits
    // at every octet-boundary phase the LCG can produce.
    int bad = 0;
    for (int t = 0; t < 2000; t++) {
        uint8_t info[200];
        int     ilen = rnd() % 201;
        for (int i = 0; i < ilen; i++) {
            uint8_t r = rnd();
            // bias toward stuffing-heavy content half the time
            info[i] = (rnd() & 1) ? r : (uint8_t)(r | 0xF8);
        }
        uint8_t  frame[256];
        uint32_t da = ((uint32_t)rnd() << 16 | (uint32_t)rnd() << 8 | rnd()) &
                      0xFFFFFFu;
        uint32_t sa = ((uint32_t)rnd() << 16 | (uint32_t)rnd() << 8 | rnd()) &
                      0xFFFFFFu;
        int n = build_frame(frame, da, rnd() & 7, rnd() & 1, sa, rnd() & 7,
                            rnd() & 1, rnd(), info, ilen);

        bitvec_t v;
        bv_reset(&v);
        bv_flag(&v);
        bv_stuffed_octets(&v, frame, n);
        bv_flag(&v);

        capture_t cap;
        memset(&cap, 0, sizeof(cap));
        uint8_t oct[4096];
        int     n_oct = bv_pack(&v, oct, (int)sizeof(oct));
        if (n_oct < 0 ||
            avlc_deframe_octets(oct, v.n, cap_cb, &cap) != 1 || cap.n != 1 ||
            !cap.f[0].fcs_ok || cap.f[0].raw_len != n ||
            memcmp(cap.raw[0], frame, (size_t)n) != 0) {
            if (bad < 3)
                fprintf(stderr, "  random roundtrip #%d failed (ilen=%d)\n",
                        t, ilen);
            bad++;
        }
    }
    CHECK(bad == 0, "%d/2000 random round trips failed", bad);
}

static void test_address_field_vectors(void)
{
    // Pin the address bit layout with a hand-checkable vector: encode with
    // the builder, then verify the ON-AIR octets directly (not just the
    // round trip) so an encode+decode that were consistently-wrong-together
    // would still be caught.
    //
    // addr=0x000001 type=1 (aircraft) status=0:
    //   packed val28 = 0x1000001; v = rev28(val28):
    //   val28 bits set: 0 and 24 -> v bits set: 27 and 3 -> v = 0x8000008.
    //   octets: b0 = (v&0x7F)<<1 = 0x10, b1 = ((v>>7)&0x7F)<<1 = 0,
    //           b2 = ((v>>14)&0x7F)<<1 = 0, b3 = ((v>>21)&0x7F)<<1|1 = 0x81.
    uint8_t b[4];
    put_addr(b, 0x000001, 1, 0);
    CHECK(b[0] == 0x10 && b[1] == 0x00 && b[2] == 0x00 && b[3] == 0x81,
          "addr vector: got %02X %02X %02X %02X want 10 00 00 81", b[0], b[1],
          b[2], b[3]);
    // extension bits: 0,0,0,1
    CHECK((b[0] & 1) == 0 && (b[1] & 1) == 0 && (b[2] & 1) == 0 &&
              (b[3] & 1) == 1,
          "EA bits must be 0,0,0,1");
}

// ---------------------------------------------------------------------------
// Real over-the-air vectors (fixture_vdl2_avlc_golden.h)
// ---------------------------------------------------------------------------

static char type_class(uint8_t type)
{
    if (type == AVLC_ADDRTYPE_AIRCRAFT) return 'A';
    if (type == AVLC_ADDRTYPE_GS_ADM || type == AVLC_ADDRTYPE_GS_DEL)
        return 'G';
    if (type == AVLC_ADDRTYPE_ALL) return '*';
    return 'R';
}

static void golden_count_cb(const avlc_frame_t *f, void *ctx)
{
    int *res = ctx;
    res[0]++;
    if (f->fcs_ok) res[1]++;
    if (f->kind == AVLC_KIND_ACARS) res[2]++;
}

// Check one deframed frame against its dumpvdl2 ground truth.
static void check_golden_frame(const avlc_frame_t *f,
                               const vdl2_avlc_golden_t *g, int idx)
{
    CHECK(f->fcs_ok, "golden %d: FCS must validate", idx);
    if (!f->fcs_ok) return;
    CHECK(f->raw_len == g->raw_len && memcmp(f->raw, g->raw,
                                             (size_t)g->raw_len) == 0,
          "golden %d: raw bytes must survive byte-exact", idx);
    CHECK(f->src_addr == g->src_addr && f->dst_addr == g->dst_addr,
          "golden %d: addr src=%06X want %06X dst=%06X want %06X", idx,
          f->src_addr, g->src_addr, f->dst_addr, g->dst_addr);
    CHECK(type_class(f->src_type) == g->src_class &&
              type_class(f->dst_type) == g->dst_class,
          "golden %d: type class src=%c want %c dst=%c want %c", idx,
          type_class(f->src_type), g->src_class, type_class(f->dst_type),
          g->dst_class);
    CHECK(f->response == (g->response != 0), "golden %d: C/R", idx);
    CHECK(f->src_on_ground == (g->src_on_ground != 0), "golden %d: A/G",
          idx);
    CHECK(f->control == g->control,
          "golden %d: control=0x%02X want 0x%02X", idx, f->control,
          g->control);

    char want_type = ((f->control & 1) == 0)  ? 'I'
                     : ((f->control & 3) == 1) ? 'S'
                                               : 'U';
    CHECK(want_type == g->avlc_type, "golden %d: I/S/U", idx);

    if (g->is_acars) {
        CHECK(f->kind == AVLC_KIND_ACARS,
              "golden %d: must classify as ACARS (kind=%d)", idx,
              (int)f->kind);
        if (f->kind != AVLC_KIND_ACARS) return;
        // mode char + 7-char registration, parity bit stripped (ACARS
        // characters carry odd parity on air; libacars strips it later)
        CHECK((char)(f->acars[0] & 0x7F) == g->acars_mode,
              "golden %d: ACARS mode '%c' want '%c'", idx, f->acars[0] & 0x7F,
              g->acars_mode);
        int reg_ok = (f->acars_len >= 8);
        for (int i = 0; reg_ok && i < 7 && g->acars_reg[i]; i++)
            reg_ok = ((char)(f->acars[1 + i] & 0x7F) == g->acars_reg[i]);
        CHECK(reg_ok, "golden %d: ACARS reg mismatch", idx);
    } else {
        CHECK(f->kind != AVLC_KIND_ACARS,
              "golden %d: link-mgmt/X.25 frame must NOT route to libacars",
              idx);
        avlc_frame_kind_t want =
            (g->avlc_type == 'I')   ? AVLC_KIND_X25
            : (g->avlc_type == 'S') ? AVLC_KIND_SUPERVISORY
                                    : AVLC_KIND_UNNUMBERED;
        CHECK(f->kind == want, "golden %d: kind=%d want %d", idx,
              (int)f->kind, (int)want);
    }
}

static void test_golden_real_frames(void)
{
    int n_acars = 0;
    // (a) each real frame individually: stuff + flag its raw octets and
    //     run the full production deframe path.
    for (int t = 0; t < VDL2_AVLC_GOLDEN_COUNT; t++) {
        const vdl2_avlc_golden_t *g = &k_vdl2_avlc_golden[t];
        bitvec_t v;
        bv_reset(&v);
        bv_flag(&v);
        bv_stuffed_octets(&v, g->raw, g->raw_len);
        bv_flag(&v);
        capture_t cap;
        int       r = deframe_both(&v, &cap);
        CHECK(r == 1 && cap.n == 1, "golden %d: got %d frames", t, cap.n);
        if (cap.n == 1) check_golden_frame(&cap.f[0], g, t);
        if (cap.n == 1 && cap.f[0].kind == AVLC_KIND_ACARS) n_acars++;
    }
    CHECK(n_acars == 9, "want 9 ACARS frames in the golden set, got %d",
          n_acars);

    // (b) all 40 frames as ONE continuous flag-separated stream (the
    //     back-to-back CSMA layout): every frame recovered, all FCS-valid,
    //     exactly the 9 ACARS frames classified for libacars.
    {
        static bitvec_t v; // ~28 kbit — keep off the stack
        bv_reset(&v);
        bv_flag(&v);
        for (int t = 0; t < VDL2_AVLC_GOLDEN_COUNT; t++) {
            bv_stuffed_octets(&v, k_vdl2_avlc_golden[t].raw,
                              k_vdl2_avlc_golden[t].raw_len);
            bv_flag(&v);
        }
        uint8_t oct[8 * 1024];
        int     n_oct = bv_pack(&v, oct, (int)sizeof(oct));
        CHECK(n_oct > 0, "golden stream must pack");
        int count_res[3] = { 0, 0, 0 }; // frames / fcs-valid / acars
        int r = avlc_deframe_octets(oct, v.n, golden_count_cb, count_res);
        CHECK(r == VDL2_AVLC_GOLDEN_COUNT,
              "continuous stream: got %d frames want %d", r,
              VDL2_AVLC_GOLDEN_COUNT);
        CHECK(count_res[1] == VDL2_AVLC_GOLDEN_COUNT,
              "continuous stream: %d/%d FCS-valid", count_res[1],
              VDL2_AVLC_GOLDEN_COUNT);
        CHECK(count_res[2] == 9, "continuous stream: %d ACARS want 9",
              count_res[2]);
    }
}

int main(void)
{
    test_address_field_vectors();
    test_golden_real_frames();
    test_acars_roundtrip();
    test_classification();
    test_fcs_reject();
    test_stuffing_torture();
    test_multiframe_and_tail();
    test_invalid_sequences();
    test_random_roundtrip();

    printf("\n=== %d passed, %d failed ===\n", s_pass, s_fail);
    return s_fail ? 1 : 0;
}
