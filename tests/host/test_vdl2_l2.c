// test_vdl2_l2 — synthetic round-trip suite for the V3 L2 feed
// (common/vdl2_decoder/vdl2_l2.c): PHY bit vector -> byte pack ->
// RS-block de-interleave -> RS(255,249) correction -> AVLC deframe.
//
// The transmission builder here is an INDEPENDENT implementation of the
// air-side encode (bit stuffing, block segmentation, RS parity via
// rs_vdl2_encode, dumpvdl2's byte interleaver walk written from the
// spec of decode.c:135-163, header word via vdl2_hdr_encode) — the
// two-implementations discipline the scrambler/header tests already
// use. Frames fed in are REAL over-the-air AVLC frames (FCS included)
// from fixture_vdl2_avlc_golden.h, so the deframer's byte-exact output
// can be compared against ground truth.
//
// Covers what the real-capture e2e test cannot: multi-block (>249 data
// octet) transmissions — the sigidwiki capture is single-block only —
// plus per-block error injection up to the RS correction bound, the
// shortened-last-block erasure schemes (2- and 4-parity), uncorrectable
// -> whole-burst drop, and window truncation.

#include <stdio.h>
#include <string.h>

#include "avlc.h"
#include "fixture_vdl2_avlc_golden.h"
#include "rs_vdl2.h"
#include "vdl2_demod.h"
#include "vdl2_l2.h"

static int g_fails = 0;
#define CHECK(cond, ...)                                \
    do {                                                \
        if (!(cond)) {                                  \
            printf("FAIL %s:%d: ", __func__, __LINE__); \
            printf(__VA_ARGS__);                        \
            printf("\n");                               \
            g_fails++;                                  \
        }                                               \
    } while (0)

// ---------------------------------------------------------------------------
// Air-side transmission builder (test-local encode)
// ---------------------------------------------------------------------------

#define MAX_BITS (1 << 15)
#define MAX_OCTETS 2048
#define MAX_BLOCKS 9

// Append one HDLC flag, LSB-first (0x7E — palindromic, order moot).
static int append_flag(uint8_t *bits, int pos)
{
    static const uint8_t f[8] = {0, 1, 1, 1, 1, 1, 1, 0};
    memcpy(bits + pos, f, 8);
    return pos + 8;
}

// Append frame octets LSB-first with HDLC bit stuffing (a 0 after five
// consecutive 1s).
static int append_stuffed(uint8_t *bits, int pos, const uint8_t *oct, int n)
{
    int ones = 0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < 8; j++) {
            uint8_t b   = (uint8_t)((oct[i] >> j) & 1u);
            bits[pos++] = b;
            if (b) {
                if (++ones == 5) {
                    bits[pos++] = 0;
                    ones        = 0;
                }
            } else {
                ones = 0;
            }
        }
    }
    return pos;
}

// dumpvdl2 decode.c:124-133 get_fec_octetcount.
static int fec_octetcount(uint32_t len)
{
    if (len < 3) return 0;
    if (len < 31) return 2;
    if (len < 68) return 4;
    return 6;
}

// Byte interleaver: emits out[i] = tab[row][col] following EXACTLY the
// (row, col) walk of dumpvdl2's deinterleave (decode.c:148-161), so
// the production de-interleave is its inverse by construction.
static void interleave_walk(uint8_t *out, uint32_t len, uint32_t rows,
                            const uint8_t tab[][RS_VDL2_N],
                            uint32_t fillwidth, uint32_t offset)
{
    uint32_t last_row_len = len % fillwidth;
    if (last_row_len == 0) last_row_len = fillwidth;
    uint32_t row = 0, col = offset;
    last_row_len += offset;
    for (uint32_t i = 0; i < len; i++) {
        if (row == rows - 1 && col >= last_row_len) {
            row = 0;
            col++;
        }
        out[i] = tab[row++][col];
        if (row == rows) {
            row = 0;
            col++;
        }
    }
}

typedef struct {
    uint8_t  bits[MAX_BITS]; // PHY vector: 25 hdr + interleaved body
    int      n_bits;
    uint32_t datalen;        // header transmission length (bits)
    uint32_t datalen_octets;
    uint32_t num_blocks;
    uint32_t last; // data octets in the (shortened) last block
    // Interleaved-stream index of data octet (row, k): for error
    // injection targeted at a specific RS block.
    int stream_idx[MAX_BLOCKS][RS_VDL2_K];
} built_tx_t;

// Build a transmission from a list of golden-frame indices:
// flag F0 flag F1 ... flag (single flags between frames — closes the
// previous frame and the next starts immediately, dumpvdl2's real-air
// framing; a leading flag is also tolerated by the deframer).
static void build_tx(built_tx_t *tx, const int *frame_idx, int n_frames)
{
    static uint8_t stuffed[MAX_BITS];
    int            pos = append_flag(stuffed, 0);
    for (int f = 0; f < n_frames; f++) {
        const vdl2_avlc_golden_t *g = &k_vdl2_avlc_golden[frame_idx[f]];
        pos = append_stuffed(stuffed, pos, g->raw, g->raw_len);
        pos = append_flag(stuffed, pos);
    }
    tx->datalen        = (uint32_t)pos;
    tx->datalen_octets = (tx->datalen + 7) / 8;

    // Stream-order data octets (pad bits: zeros — truncated downstream).
    static uint8_t stream[MAX_OCTETS];
    memset(stream, 0, sizeof(stream));
    for (uint32_t i = 0; i < tx->datalen; i++)
        stream[i >> 3] |= (uint8_t)((stuffed[i] & 1u) << (i & 7));

    // Block segmentation (dumpvdl2 decode.c:233-245) + RS parity.
    tx->num_blocks = tx->datalen_octets / RS_VDL2_K;
    tx->last       = tx->datalen_octets % RS_VDL2_K;
    uint32_t fec_octets = tx->num_blocks * RS_VDL2_NROOTS;
    if (tx->last) tx->num_blocks++;
    fec_octets += (uint32_t)fec_octetcount(tx->last);
    if (tx->last == 0) tx->last = RS_VDL2_K;

    static uint8_t tab[MAX_BLOCKS][RS_VDL2_N];
    memset(tab, 0, sizeof(tab));
    uint32_t off = 0;
    for (uint32_t r = 0; r < tx->num_blocks; r++) {
        uint32_t n = (r == tx->num_blocks - 1) ? tx->last : RS_VDL2_K;
        memcpy(tab[r], stream + off, n);
        off += n;
        // Parity over the zero-filled codeword; a shortened block
        // transmits only the first fec_octetcount() parity octets, the
        // rest are the receiver's erasures (dumpvdl2 rs.c:36-44).
        rs_vdl2_encode(tab[r]);
    }

    // Interleave data + FEC into air order.
    static uint8_t data_il[MAX_OCTETS], fec_il[MAX_BLOCKS * RS_VDL2_NROOTS];
    interleave_walk(data_il, tx->datalen_octets, tx->num_blocks, tab,
                    RS_VDL2_K, 0);
    uint32_t fec_rows = tx->num_blocks;
    if (fec_octetcount(tx->last) == 0) fec_rows--;
    interleave_walk(fec_il, fec_octets, fec_rows, tab, RS_VDL2_NROOTS,
                    RS_VDL2_K);

    // Record the interleaved index of every data octet per block so a
    // test can corrupt octets of a CHOSEN block: replay the walk.
    {
        uint32_t last_row_len = tx->datalen_octets % RS_VDL2_K;
        if (last_row_len == 0) last_row_len = RS_VDL2_K;
        uint32_t row = 0, col = 0;
        for (uint32_t i = 0; i < tx->datalen_octets; i++) {
            if (row == tx->num_blocks - 1 && col >= last_row_len) {
                row = 0;
                col++;
            }
            tx->stream_idx[row][col] = (int)i;
            row++;
            if (row == tx->num_blocks) {
                row = 0;
                col++;
            }
        }
    }

    // PHY bit vector: 25-bit header (MSB-first on air) + data + FEC
    // octets LSB-first.
    uint32_t hdr = vdl2_hdr_encode(tx->datalen);
    int      n   = 0;
    for (int k = VDL2_HDR_BITS - 1; k >= 0; k--)
        tx->bits[n++] = (uint8_t)((hdr >> k) & 1u);
    for (uint32_t i = 0; i < 8 * tx->datalen_octets; i++)
        tx->bits[n++] = (uint8_t)((data_il[i >> 3] >> (i & 7)) & 1u);
    for (uint32_t i = 0; i < 8 * fec_octets; i++)
        tx->bits[n++] = (uint8_t)((fec_il[i >> 3] >> (i & 7)) & 1u);
    tx->n_bits = n;
}

// Flip all 8 bits of data octet (block, k) in the PHY vector — one
// guaranteed RS symbol error.
static void corrupt_octet(built_tx_t *tx, uint32_t block, uint32_t k)
{
    int base = VDL2_HDR_BITS + 8 * tx->stream_idx[block][k];
    for (int j = 0; j < 8; j++)
        tx->bits[base + j] ^= 1u;
}

// XOR an arbitrary nonzero 8-bit delta into data octet (block, k) — one RS
// symbol error whose value the caller controls (used by the shortened-block
// fallback search to vary the error pattern until the HARD decoder detects
// rather than aliases).
static void corrupt_octet_delta(built_tx_t *tx, uint32_t block, uint32_t k,
                                uint8_t delta)
{
    int base = VDL2_HDR_BITS + 8 * tx->stream_idx[block][k];
    for (int j = 0; j < 8; j++)
        tx->bits[base + j] ^= (uint8_t)((delta >> j) & 1u);
}

// Soft-confidence helpers for the RS erasure fallback tests. vdl2_l2 uses
// only |soft_bits| for reliability (sign carries the hard decision, which
// we mirror from tx->bits). CONF_HI on every bit by default; weaken_octet
// drops a chosen octet's 8 bits to CONF_LO so its RS symbol is among the
// least-reliable positions the fallback erases (same bit indexing as
// corrupt_octet, so the weakened symbol lines up with the injected error).
#define CONF_HI 4000
#define CONF_LO 3
static void build_soft(const built_tx_t *tx, int16_t *soft)
{
    for (int i = 0; i < tx->n_bits; i++)
        soft[i] = tx->bits[i] ? (int16_t)-CONF_HI : (int16_t)CONF_HI;
}
static void weaken_octet(const built_tx_t *tx, int16_t *soft, uint32_t block,
                         uint32_t k)
{
    int base = VDL2_HDR_BITS + 8 * tx->stream_idx[block][k];
    for (int j = 0; j < 8; j++)
        soft[base + j] = tx->bits[base + j] ? (int16_t)-CONF_LO
                                            : (int16_t)CONF_LO;
}
// As weaken_octet but to an explicit confidence LEVEL, so a test can order
// the erasure ranking (a strictly lower level is selected before CONF_LO
// fillers). Magnitude is what select_weakest ranks; sign mirrors the bits.
static void weaken_octet_lvl(const built_tx_t *tx, int16_t *soft,
                             uint32_t block, uint32_t k, int16_t level)
{
    int base = VDL2_HDR_BITS + 8 * tx->stream_idx[block][k];
    for (int j = 0; j < 8; j++)
        soft[base + j] = tx->bits[base + j] ? (int16_t)-level : level;
}

// ---------------------------------------------------------------------------
// Collector callback
// ---------------------------------------------------------------------------

#define MAX_FRAMES 16
typedef struct {
    int               n;
    avlc_frame_kind_t kind[MAX_FRAMES];
    uint8_t           raw[MAX_FRAMES][256];
    int               raw_len[MAX_FRAMES];
} collect_t;

static void collect_cb(const avlc_frame_t *f, void *ctx)
{
    collect_t *c = (collect_t *)ctx;
    if (c->n >= MAX_FRAMES) return;
    c->kind[c->n]    = f->kind;
    c->raw_len[c->n] = f->raw_len;
    if (f->raw_len > 0 && f->raw_len <= 256)
        memcpy(c->raw[c->n], f->raw, (size_t)f->raw_len);
    c->n++;
}

static void expect_frames(const collect_t *c, const int *frame_idx, int n)
{
    CHECK(c->n == n, "frames %d != %d", c->n, n);
    for (int f = 0; f < n && f < c->n; f++) {
        const vdl2_avlc_golden_t *g = &k_vdl2_avlc_golden[frame_idx[f]];
        CHECK(c->raw_len[f] == g->raw_len, "frame %d len %d != %d", f,
              c->raw_len[f], g->raw_len);
        if (c->raw_len[f] == g->raw_len)
            CHECK(memcmp(c->raw[f], g->raw, (size_t)g->raw_len) == 0,
                  "frame %d bytes differ", f);
    }
}

// ---------------------------------------------------------------------------

// Case A: one small frame, single shortened block, 2-parity scheme
// (11-octet S frame -> ~15 data octets < 31).
static void test_single_small(void)
{
    const int  idx[] = {6}; // k_vdl2g_raw_6, 11-octet S frame
    built_tx_t tx;
    build_tx(&tx, idx, 1);
    CHECK(tx.num_blocks == 1, "blocks %u != 1", tx.num_blocks);
    CHECK(tx.last < 31, "expected 2-parity scheme, last=%u", tx.last);

    collect_t c = {0};
    int       rc = vdl2_l2_feed(tx.bits, NULL, tx.n_bits, collect_cb, &c);
    CHECK(rc == 1, "rc %d != 1", rc);
    expect_frames(&c, idx, 1);

    // 2 parity octets + 4 erasures: exactly one symbol error correctable.
    build_tx(&tx, idx, 1);
    corrupt_octet(&tx, 0, 3);
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, NULL, tx.n_bits, collect_cb, &c);
    CHECK(rc == 1, "1-err rc %d != 1", rc);
    expect_frames(&c, idx, 1);

    // Two symbol errors exceed 2e+f<=6 with f=4. With the erasures
    // consuming all the margin, a >bound pattern may either be detected
    // (ERR_RS) or ALIAS into a different valid codeword (the
    // bounded-distance caveat in rs_vdl2.h) — in which case the garbage
    // octets must fail downstream at the AVLC stuffing/FCS arbiter.
    // Assert the load-bearing property: the original frame is NOT
    // cleanly reproduced. (Deterministic input -> stable outcome; on
    // this pattern the decoder aliases and the deframe emits nothing.)
    build_tx(&tx, idx, 1);
    corrupt_octet(&tx, 0, 3);
    corrupt_octet(&tx, 0, 9);
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, NULL, tx.n_bits, collect_cb, &c);
    bool clean = (rc == 1 && c.n == 1 &&
                  c.raw_len[0] == k_vdl2_avlc_golden[idx[0]].raw_len &&
                  memcmp(c.raw[0], k_vdl2_avlc_golden[idx[0]].raw,
                         (size_t)c.raw_len[0]) == 0 &&
                  c.kind[0] != AVLC_KIND_BAD_FCS);
    CHECK(!clean, "2-err burst decoded CLEAN (rc %d) — beyond-bound "
                  "errors must never masquerade as a good frame", rc);
}

// Case B: three frames back-to-back in one (4-parity) block — the CSMA
// multi-frame burst layout.
static void test_multi_frame(void)
{
    const int  idx[] = {6, 2, 39}; // 11 + 27 + 11 octets
    built_tx_t tx;
    build_tx(&tx, idx, 3);
    CHECK(tx.num_blocks == 1, "blocks %u != 1", tx.num_blocks);
    CHECK(tx.last >= 31 && tx.last < 68, "expected 4-parity, last=%u", tx.last);

    collect_t c = {0};
    int       rc = vdl2_l2_feed(tx.bits, NULL, tx.n_bits, collect_cb, &c);
    CHECK(rc == 3, "rc %d != 3", rc);
    expect_frames(&c, idx, 3);
    CHECK(c.n == 3 && c.kind[0] == AVLC_KIND_SUPERVISORY &&
              c.kind[1] == AVLC_KIND_SUPERVISORY &&
              c.kind[2] == AVLC_KIND_SUPERVISORY,
          "kind mismatch");

    // 4-parity + 2 erasures: up to two symbol errors correctable.
    build_tx(&tx, idx, 3);
    corrupt_octet(&tx, 0, 5);
    corrupt_octet(&tx, 0, 40);
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, NULL, tx.n_bits, collect_cb, &c);
    CHECK(rc == 3, "2-err rc %d != 3", rc);
    expect_frames(&c, idx, 3);
}

// Case C: MULTI-BLOCK transmission (> 249 data octets -> 2 RS blocks),
// clean and with 3 symbol errors injected into EACH block — the
// de-interleave + per-block segmentation proof the single-block real
// capture cannot give.
static void test_multi_block(void)
{
    // 94+69+69+69+75 = 376 frame octets; stuffing + flags push the
    // stream well past 249 -> 2 blocks (full + shortened 6-parity).
    const int  idx[] = {5, 0, 1, 3, 35};
    built_tx_t tx;
    build_tx(&tx, idx, 5);
    CHECK(tx.num_blocks == 2, "blocks %u != 2", tx.num_blocks);
    CHECK(tx.last >= 68, "expected 6-parity last block, last=%u", tx.last);

    collect_t c = {0};
    int       rc = vdl2_l2_feed(tx.bits, NULL, tx.n_bits, collect_cb, &c);
    CHECK(rc == 5, "clean rc %d != 5", rc);
    expect_frames(&c, idx, 5);
    CHECK(c.n == 5 && c.kind[0] == AVLC_KIND_ACARS, "frame0 not ACARS");

    // 3 errors per block = the full t=3 budget in both blocks at once.
    vdl2_l2_stats_t s0, s1;
    vdl2_l2_get_stats(&s0);
    build_tx(&tx, idx, 5);
    corrupt_octet(&tx, 0, 10);
    corrupt_octet(&tx, 0, 100);
    corrupt_octet(&tx, 0, 200);
    corrupt_octet(&tx, 1, 0);
    corrupt_octet(&tx, 1, 60);
    corrupt_octet(&tx, 1, (uint32_t)tx.last - 1);
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, NULL, tx.n_bits, collect_cb, &c);
    CHECK(rc == 5, "3+3-err rc %d != 5", rc);
    expect_frames(&c, idx, 5);
    vdl2_l2_get_stats(&s1);
    CHECK(s1.rs_octets_fixed - s0.rs_octets_fixed == 6,
          "rs_octets_fixed delta %u != 6",
          s1.rs_octets_fixed - s0.rs_octets_fixed);

    // 4 errors in one block: over budget, dropped whole.
    build_tx(&tx, idx, 5);
    corrupt_octet(&tx, 0, 10);
    corrupt_octet(&tx, 0, 100);
    corrupt_octet(&tx, 0, 200);
    corrupt_octet(&tx, 0, 240);
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, NULL, tx.n_bits, collect_cb, &c);
    CHECK(rc == VDL2_L2_ERR_RS, "4-err rc %d != ERR_RS", rc);
}

// Case D: bit vector shorter than the header's length claim.
static void test_truncated(void)
{
    const int  idx[] = {6};
    built_tx_t tx;
    build_tx(&tx, idx, 1);
    collect_t c  = {0};
    int       rc = vdl2_l2_feed(tx.bits, NULL, tx.n_bits - 10, collect_cb, &c);
    CHECK(rc == VDL2_L2_ERR_TRUNC, "rc %d != ERR_TRUNC", rc);
    CHECK(c.n == 0, "truncated emitted %d frames", c.n);
    // Garbage-header vector.
    uint8_t junk[VDL2_HDR_BITS] = {1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1,
                                   0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1};
    rc = vdl2_l2_feed(junk, NULL, VDL2_HDR_BITS, collect_cb, &c);
    CHECK(rc < 0, "junk header rc %d not an error", rc);
}

// Case E: soft-decision RS erasure fallback (the VDL2 analog of Iridium's
// Chase-2). Uses the same 2-block layout as test_multi_block; block 0 is a
// FULL block (all 255 symbols transmitted), the only case the fallback runs.
static void test_erasure_recovery(void)
{
    const int      idx[] = {5, 0, 1, 3, 35};
    built_tx_t     tx;
    static int16_t soft[MAX_BITS];
    // Positions to corrupt in FULL block 0. 4 errors > hard t=3, so
    // hard-decision RS cannot correct them; this exact pattern is the one
    // test_multi_block already proves the hard decoder REJECTS (ERR_RS) —
    // not an alias — so the recovery is genuinely the erasure fallback's.
    const uint32_t  ep[4] = {10, 100, 200, 240};
    vdl2_l2_stats_t s0, s1;

    // (a) 4 symbol errors in block 0, each marked least-reliable: the
    // 6-erasure fallback covers them and RECOVERS the block. rc==5 frames,
    // byte-exact, and rs_erasure_recovered increments by exactly 1.
    build_tx(&tx, idx, 5);
    CHECK(tx.num_blocks == 2, "erasure: blocks %u != 2", tx.num_blocks);
    build_soft(&tx, soft);
    for (int e = 0; e < 4; e++) {
        corrupt_octet(&tx, 0, ep[e]);
        weaken_octet(&tx, soft, 0, ep[e]);
    }
    vdl2_l2_get_stats(&s0);
    collect_t c  = {0};
    int       rc = vdl2_l2_feed(tx.bits, soft, tx.n_bits, collect_cb, &c);
    vdl2_l2_get_stats(&s1);
    CHECK(rc == 5, "erasure(4-err) rc %d != 5", rc);
    expect_frames(&c, idx, 5);
    CHECK(s1.rs_erasure_recovered - s0.rs_erasure_recovered == 1,
          "rs_erasure_recovered delta %u != 1",
          s1.rs_erasure_recovered - s0.rs_erasure_recovered);

    // (b) NEGATIVE — clean burst with soft supplied: hard decode succeeds,
    // the fallback must NOT run (counter unchanged).
    build_tx(&tx, idx, 5);
    build_soft(&tx, soft);
    vdl2_l2_get_stats(&s0);
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, soft, tx.n_bits, collect_cb, &c);
    vdl2_l2_get_stats(&s1);
    CHECK(rc == 5, "clean+soft rc %d != 5", rc);
    expect_frames(&c, idx, 5);
    CHECK(s1.rs_erasure_recovered == s0.rs_erasure_recovered,
          "fallback ran on a clean block (delta %u)",
          s1.rs_erasure_recovered - s0.rs_erasure_recovered);

    // (b') NEGATIVE — 3 errors (within hard t=3): the fast path corrects
    // them, fallback still must NOT run even though soft is present.
    build_tx(&tx, idx, 5);
    build_soft(&tx, soft);
    for (int e = 0; e < 3; e++) {
        corrupt_octet(&tx, 0, ep[e]);
        weaken_octet(&tx, soft, 0, ep[e]);
    }
    vdl2_l2_get_stats(&s0);
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, soft, tx.n_bits, collect_cb, &c);
    vdl2_l2_get_stats(&s1);
    CHECK(rc == 5, "3-err+soft rc %d != 5", rc);
    expect_frames(&c, idx, 5);
    CHECK(s1.rs_erasure_recovered == s0.rs_erasure_recovered,
          "fallback ran when hard decode sufficed (delta %u)",
          s1.rs_erasure_recovered - s0.rs_erasure_recovered);

    // (c) soft_bits == NULL: no fallback. The 4-error block drops the whole
    // burst (ERR_RS); a clean burst still decodes. No crash either way.
    build_tx(&tx, idx, 5);
    for (int e = 0; e < 4; e++) corrupt_octet(&tx, 0, ep[e]);
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, NULL, tx.n_bits, collect_cb, &c);
    CHECK(rc == VDL2_L2_ERR_RS, "4-err NULL-soft rc %d != ERR_RS", rc);

    build_tx(&tx, idx, 5);
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, NULL, tx.n_bits, collect_cb, &c);
    CHECK(rc == 5, "clean NULL-soft rc %d != 5", rc);
    expect_frames(&c, idx, 5);
}

// Case F: A/B proof of the ASCENDING-f erasure SWEEP over a single f=6
// attempt. Constructs a 4-symbol-error pattern (> hard t=3, so hard-decision
// RS is rejected and the fallback runs) that a single f=6/e=0 attempt CANNOT
// solve but the f=2 rung of the sweep DOES:
//
//   - 2 "strong" errors at high-confidence octets (200, 240), NOT weakened.
//   - 2 "weak" errors at octets (10, 100) weakened to the LOWEST level (1) —
//     ranked as the 2 weakest of all 255 positions.
//   - 4 error-free filler octets (20,30,40,50) weakened to CONF_LO (3), so the
//     6 lowest-confidence positions are {two weak errors} + {four fillers}.
//
// The 4 error positions are exactly {10,100,200,240} — the set Case C/E proves
// hard-decision RS DETECTS (VDL2_L2_ERR_RS, not a silent alias), so the
// fallback genuinely runs here.
//
// Single f=6/e=0 erases those 6 lowest-confidence symbols. The 2 strong errors
// (200, 240) lie OUTSIDE that erasure set and e=0 leaves NO error budget, so RS
// is uncorrectable -> the whole burst would drop (VDL2_L2_ERR_RS).
//
// The sweep tries f=2 FIRST: it erases only the 2 weakest positions (the weak
// errors at 10, 100) and RETAINS e<=2 error-correction budget. 2e+f = 2*2 + 2
// = 6 lets it correct the 2 strong out-of-set errors anywhere in the block.
// First rung wins -> block recovered, rs_erasure_recovered += 1.
//
// This is exactly the plan's target: a true error outside the 6 weakest that
// f=6 structurally cannot reach but a low-f errors-anywhere decode can.
// (Empirically confirmed: temporarily reducing the sweep to a single f=6
// attempt makes this case return VDL2_L2_ERR_RS; the sweep returns 5.)
static void test_erasure_fsweep(void)
{
    const int      idx[] = {5, 0, 1, 3, 35}; // same 2-block layout; blk0 full
    built_tx_t     tx;
    static int16_t soft[MAX_BITS];
    const uint32_t err_weak[2]   = {10, 100};       // errors, ranked weakest
    const uint32_t err_strong[2] = {200, 240};      // errors, high confidence
    const uint32_t fill[4]       = {20, 30, 40, 50}; // weak, error-free
    vdl2_l2_stats_t s0, s1;

    build_tx(&tx, idx, 5);
    CHECK(tx.num_blocks == 2, "fsweep: blocks %u != 2", tx.num_blocks);
    build_soft(&tx, soft);
    // 4 symbol errors in FULL block 0: 2 at the weakest positions, 2 at
    // high-confidence positions outside the weak set.
    for (int e = 0; e < 2; e++) {
        corrupt_octet(&tx, 0, err_weak[e]);
        weaken_octet_lvl(&tx, soft, 0, err_weak[e], 1); // strictly weakest
    }
    for (int e = 0; e < 2; e++) corrupt_octet(&tx, 0, err_strong[e]); // stay HI
    // 4 fillers weaken the field so f=6 spends all erasures on non-errors.
    for (int e = 0; e < 4; e++) weaken_octet(&tx, soft, 0, fill[e]); // CONF_LO

    vdl2_l2_get_stats(&s0);
    collect_t c  = {0};
    int       rc = vdl2_l2_feed(tx.bits, soft, tx.n_bits, collect_cb, &c);
    vdl2_l2_get_stats(&s1);
    CHECK(rc == 5, "fsweep rc %d != 5 (f=2 rung should recover 2 out-of-set "
                   "errors that single f=6 cannot)", rc);
    expect_frames(&c, idx, 5);
    CHECK(s1.rs_erasure_recovered - s0.rs_erasure_recovered == 1,
          "fsweep rs_erasure_recovered delta %u != 1",
          s1.rs_erasure_recovered - s0.rs_erasure_recovered);
    // Joint rescue×FCS tap: the golden fixture frames carry valid FCS, so a
    // byte-exact rescue must tally ALL 5 frames as rescued_fcs_ok, none bad.
    CHECK(s1.rescued_fcs_ok - s0.rescued_fcs_ok == 5,
          "fsweep rescued_fcs_ok delta %u != 5",
          s1.rescued_fcs_ok - s0.rescued_fcs_ok);
    CHECK(s1.rescued_fcs_bad == s0.rescued_fcs_bad,
          "fsweep rescued_fcs_bad advanced on a byte-exact rescue");
}

// ---------------------------------------------------------------------------
// Case G/H/I: SHORTENED-last-block soft-decision erasure fallback. Each of the
// three dumpvdl2 shortened schemes gets its own single-block transmission so
// the ONLY RS block IS the shortened last block (is_full == false), exercising
// the shortened fallback path directly:
//   G: 2-parity (last < 31)   f_struct=4, confidence budget 2, hard t=1
//   H: 4-parity (31<=last<68) f_struct=2, confidence budget 4, hard t=2
//   I: 6-parity (68<=last)    f_struct=0, confidence budget 6, hard t=3
//
// IMPORTANT bounded-distance subtlety unique to shortened blocks: because a
// shortened block spends f_struct of its 6 syndromes on structural erasures,
// the HARD decoder has a large errata budget and, for error counts just past
// its correction bound, very often ALIASES to a different valid codeword
// (returning rc==0 with wrong data) rather than DETECTING the corruption. The
// soft fallback only runs when hard decoding *fails* (rc!=0). So a beyond-hard
// error pattern is only fallback-exercising if the hard decoder DETECTS it. We
// therefore SEARCH the error pattern (positions + delta) for one the hard
// (NULL-soft) path rejects with ERR_RS, then prove the fallback recovers it —
// a deterministic, self-adapting way to hit the (c) path on every scheme.
//
// For each scheme: (a) clean -> hard path, fallback NOT invoked, byte-exact;
// (b) errors WITHIN the hard limit -> hard corrects, fallback NOT invoked,
// byte-exact; (c) errors BEYOND the hard limit but confidence-recoverable ->
// fallback fires, byte-exact recovery, rs_erasure_recovered += 1; (d) errors
// beyond erasure capacity too -> NO false-positive clean decode; (e)
// CONFINEMENT -> the (c) recovery is itself the proof: the weakened errors
// sit at CONF_LO while the zero-pad / untransmitted-parity positions carry the
// lowest possible confidence (0). An unconfined selection would erase those
// conf-0 positions and FAIL; a byte-exact recovery proves
// select_weakest_confined stayed inside the transmitted symbols.

// Search a hard-DETECTED (ERR_RS, not aliased) error pattern of `nerr`
// distinct data octets for the single shortened block of `idx`. Returns 1 and
// fills pos[0..nerr) + a per-error delta list on success.
static int find_hard_detected_pattern(const int *idx, int n_frames, int nerr,
                                      uint32_t *pos, uint8_t *delta)
{
    built_tx_t probe;
    build_tx(&probe, idx, n_frames);
    uint32_t last = probe.last;
    static const uint8_t k_deltas[] = { 0xFF, 0x1D, 0x8C, 0x53, 0xA5 };
    for (unsigned di = 0; di < sizeof(k_deltas); di++) {
        uint8_t d = k_deltas[di];
        for (uint32_t stride = 1;
             stride <= (nerr > 1 ? (last - 1) / (uint32_t)(nerr - 1) : last);
             stride++) {
            for (uint32_t base = 0;
                 base + (uint32_t)(nerr - 1) * stride < last; base++) {
                built_tx_t t2;
                build_tx(&t2, idx, n_frames);
                for (int e = 0; e < nerr; e++)
                    corrupt_octet_delta(&t2, 0, base + (uint32_t)e * stride, d);
                collect_t cc = {0};
                int rc = vdl2_l2_feed(t2.bits, NULL, t2.n_bits, collect_cb,
                                      &cc);
                if (rc == VDL2_L2_ERR_RS) {
                    for (int e = 0; e < nerr; e++) {
                        pos[e]   = base + (uint32_t)e * stride;
                        delta[e] = d;
                    }
                    return 1;
                }
            }
        }
    }
    return 0;
}

static void shortened_fallback_case(const char *name, const int *idx,
                                    int n_frames, int hard_t, int rec_errs,
                                    int cap_errs)
{
    built_tx_t      tx;
    static int16_t  soft[MAX_BITS];
    vdl2_l2_stats_t s0, s1;
    collect_t       c;

    // Geometry sanity: single shortened block.
    build_tx(&tx, idx, n_frames);
    CHECK(tx.num_blocks == 1, "%s: blocks %u != 1 (need single shortened)",
          name, tx.num_blocks);
    CHECK(tx.last < RS_VDL2_K, "%s: last %u not shortened", name, tx.last);
    uint32_t last = tx.last;

    // (a) CLEAN with soft supplied: hard path, fallback must NOT run.
    build_soft(&tx, soft);
    vdl2_l2_get_stats(&s0);
    memset(&c, 0, sizeof(c));
    int rc = vdl2_l2_feed(tx.bits, soft, tx.n_bits, collect_cb, &c);
    vdl2_l2_get_stats(&s1);
    CHECK(rc == n_frames, "%s(a) clean rc %d != %d", name, rc, n_frames);
    expect_frames(&c, idx, n_frames);
    CHECK(s1.rs_erasure_recovered == s0.rs_erasure_recovered,
          "%s(a) fallback ran on clean block", name);

    // (b) errors WITHIN the hard limit: fast path corrects to the ORIGINAL
    // (genuine correction, e<=t), fallback NOT run. Use a simple spread.
    build_tx(&tx, idx, n_frames);
    build_soft(&tx, soft);
    for (int e = 0; e < hard_t; e++) {
        uint32_t k = ((uint32_t)e * 7 + 1) % last;
        corrupt_octet(&tx, 0, k);
        weaken_octet(&tx, soft, 0, k);
    }
    vdl2_l2_get_stats(&s0);
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, soft, tx.n_bits, collect_cb, &c);
    vdl2_l2_get_stats(&s1);
    CHECK(rc == n_frames, "%s(b) %d-err(hard) rc %d != %d", name, hard_t, rc,
          n_frames);
    expect_frames(&c, idx, n_frames);
    CHECK(s1.rs_erasure_recovered == s0.rs_erasure_recovered,
          "%s(b) fallback ran when hard decode sufficed", name);

    // (c) errors BEYOND the hard limit but confidence-recoverable: search a
    // hard-DETECTED pattern (see note above), then prove the fallback fires
    // and recovers byte-exact with rs_erasure_recovered += 1.
    {
        uint32_t pos[8];
        uint8_t  del[8];
        int found = find_hard_detected_pattern(idx, n_frames, rec_errs, pos,
                                               del);
        CHECK(found, "%s(c) no hard-detected %d-error pattern found (cannot "
              "exercise the fallback)", name, rec_errs);
        if (found) {
            // Re-confirm the NULL-soft (hard) path drops the burst.
            build_tx(&tx, idx, n_frames);
            for (int e = 0; e < rec_errs; e++)
                corrupt_octet_delta(&tx, 0, pos[e], del[e]);
            collect_t cc = {0};
            int rc2 = vdl2_l2_feed(tx.bits, NULL, tx.n_bits, collect_cb, &cc);
            CHECK(rc2 == VDL2_L2_ERR_RS, "%s(c) hard control rc %d != ERR_RS",
                  name, rc2);

            // Soft: same corruption, error octets marked least-reliable.
            build_tx(&tx, idx, n_frames);
            build_soft(&tx, soft);
            for (int e = 0; e < rec_errs; e++) {
                corrupt_octet_delta(&tx, 0, pos[e], del[e]);
                weaken_octet(&tx, soft, 0, pos[e]);
            }
            vdl2_l2_get_stats(&s0);
            memset(&c, 0, sizeof(c));
            rc = vdl2_l2_feed(tx.bits, soft, tx.n_bits, collect_cb, &c);
            vdl2_l2_get_stats(&s1);
            CHECK(rc == n_frames, "%s(c) %d-err recovery rc %d != %d", name,
                  rec_errs, rc, n_frames);
            expect_frames(&c, idx, n_frames);
            CHECK(s1.rs_erasure_recovered - s0.rs_erasure_recovered == 1,
                  "%s(c) rs_erasure_recovered delta %u != 1", name,
                  s1.rs_erasure_recovered - s0.rs_erasure_recovered);
            // Joint rescue×FCS tap: byte-exact recovery of FCS-valid golden
            // frames must count every frame as rescued_fcs_ok, none bad.
            CHECK(s1.rescued_fcs_ok - s0.rescued_fcs_ok == (uint32_t)n_frames,
                  "%s(c) rescued_fcs_ok delta %u != %d", name,
                  s1.rescued_fcs_ok - s0.rescued_fcs_ok, n_frames);
            CHECK(s1.rescued_fcs_bad == s0.rescued_fcs_bad,
                  "%s(c) rescued_fcs_bad advanced on a byte-exact rescue",
                  name);
        }
    }

    // (d) errors BEYOND erasure capacity: even the full confidence budget
    // cannot cover them. The block must NEVER be reproduced clean (either
    // ERR_RS, or an alias to garbage that fails the AVLC FCS — never a
    // byte-exact frame). Mirrors Case A's beyond-bound assertion.
    build_tx(&tx, idx, n_frames);
    build_soft(&tx, soft);
    for (int e = 0; e < cap_errs; e++) {
        uint32_t k = ((uint32_t)e * 9 + 2) % last;
        corrupt_octet(&tx, 0, k);
        weaken_octet(&tx, soft, 0, k);
    }
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, soft, tx.n_bits, collect_cb, &c);
    const vdl2_avlc_golden_t *g0 = &k_vdl2_avlc_golden[idx[0]];
    bool clean = (rc >= 1 && c.n >= 1 && c.raw_len[0] == g0->raw_len &&
                  memcmp(c.raw[0], g0->raw, (size_t)g0->raw_len) == 0 &&
                  c.kind[0] != AVLC_KIND_BAD_FCS);
    CHECK(!clean, "%s(d) %d-err over-capacity decoded CLEAN (rc %d) — beyond-"
          "capacity errors must never masquerade as a good frame", name,
          cap_errs, rc);
}

static void test_shortened_fallback(void)
{
    // 2-parity: hard t=1, recover 2 (budget 2, e=0), capacity exceeded at 3.
    const int idx_2p[] = {6}; // 11-octet S frame, last<31
    shortened_fallback_case("G/2parity", idx_2p, 1, 1, 2, 3);

    // 4-parity: hard t=2, recover 3 (f_conf=2 + e=1), capacity exceeded at 5.
    const int idx_4p[] = {6, 2, 39}; // 31<=last<68
    shortened_fallback_case("H/4parity", idx_4p, 3, 2, 3, 5);

    // 6-parity shortened: hard t=3, recover 4 (f_conf=2 + e=2), exceeded at 7.
    const int idx_6p[] = {5}; // 94-octet ACARS, 68<=last
    shortened_fallback_case("I/6parity", idx_6p, 1, 3, 4, 7);
}

int main(void)
{
    test_single_small();
    test_multi_frame();
    test_multi_block();
    test_truncated();
    test_erasure_recovery();
    test_erasure_fsweep();
    test_shortened_fallback();

    vdl2_l2_stats_t s;
    vdl2_l2_get_stats(&s);
    printf("stats: fed %u, rs ok/fail %u/%u, fixed %u, erasure_recovered %u, "
           "avlc %u\n",
           s.fed, s.rs_blocks_ok, s.rs_blocks_fail, s.rs_octets_fixed,
           s.rs_erasure_recovered, s.avlc_frames);

    if (g_fails) {
        printf("FAIL: %d check(s)\n", g_fails);
        return 1;
    }
    printf("PASS: vdl2 L2 synthetic round-trip suite\n");
    return 0;
}
