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
    int       rc = vdl2_l2_feed(tx.bits, tx.n_bits, collect_cb, &c);
    CHECK(rc == 1, "rc %d != 1", rc);
    expect_frames(&c, idx, 1);

    // 2 parity octets + 4 erasures: exactly one symbol error correctable.
    build_tx(&tx, idx, 1);
    corrupt_octet(&tx, 0, 3);
    memset(&c, 0, sizeof(c));
    rc = vdl2_l2_feed(tx.bits, tx.n_bits, collect_cb, &c);
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
    rc = vdl2_l2_feed(tx.bits, tx.n_bits, collect_cb, &c);
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
    int       rc = vdl2_l2_feed(tx.bits, tx.n_bits, collect_cb, &c);
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
    rc = vdl2_l2_feed(tx.bits, tx.n_bits, collect_cb, &c);
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
    int       rc = vdl2_l2_feed(tx.bits, tx.n_bits, collect_cb, &c);
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
    rc = vdl2_l2_feed(tx.bits, tx.n_bits, collect_cb, &c);
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
    rc = vdl2_l2_feed(tx.bits, tx.n_bits, collect_cb, &c);
    CHECK(rc == VDL2_L2_ERR_RS, "4-err rc %d != ERR_RS", rc);
}

// Case D: bit vector shorter than the header's length claim.
static void test_truncated(void)
{
    const int  idx[] = {6};
    built_tx_t tx;
    build_tx(&tx, idx, 1);
    collect_t c  = {0};
    int       rc = vdl2_l2_feed(tx.bits, tx.n_bits - 10, collect_cb, &c);
    CHECK(rc == VDL2_L2_ERR_TRUNC, "rc %d != ERR_TRUNC", rc);
    CHECK(c.n == 0, "truncated emitted %d frames", c.n);
    // Garbage-header vector.
    uint8_t junk[VDL2_HDR_BITS] = {1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1,
                                   0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1};
    rc = vdl2_l2_feed(junk, VDL2_HDR_BITS, collect_cb, &c);
    CHECK(rc < 0, "junk header rc %d not an error", rc);
}

int main(void)
{
    test_single_small();
    test_multi_frame();
    test_multi_block();
    test_truncated();

    vdl2_l2_stats_t s;
    vdl2_l2_get_stats(&s);
    printf("stats: fed %u, rs ok/fail %u/%u, fixed %u, avlc %u\n", s.fed,
           s.rs_blocks_ok, s.rs_blocks_fail, s.rs_octets_fixed, s.avlc_frames);

    if (g_fails) {
        printf("FAIL: %d check(s)\n", g_fails);
        return 1;
    }
    printf("PASS: vdl2 L2 synthetic round-trip suite\n");
    return 0;
}
