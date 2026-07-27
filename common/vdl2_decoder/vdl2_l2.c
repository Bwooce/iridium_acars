// See vdl2_l2.h. Port of dumpvdl2's post-demod burst decode
// (src/decode.c decode_vdl2_burst DEC_HEADER length math + DEC_DATA,
// v2.6.0 3f583da), re-implemented in this repo's style (GPL hygiene per
// the plan §GPL-3). Per-step citations inline.

#include "vdl2_l2.h"

#include <stdbool.h>
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

// Soft-decision confidence, mirroring s_data/s_fec/s_rs_tab exactly (built
// only when the caller supplies soft_bits). Per-octet confidence is the
// weakest-link MIN over the octet's 8 constituent bits' |soft_bits|; it is
// de-interleaved through the SAME walk as the data/FEC so s_conf_tab[r][c]
// aligns index-for-index with s_rs_tab[r][c]. int16 to preserve the demod's
// full confidence resolution (avoids ranking ties a uint8 clamp would add).
static EXT_RAM_BSS_ATTR int16_t s_data_conf[L2_MAX_DATA_OCTETS];
static EXT_RAM_BSS_ATTR int16_t s_fec_conf[L2_MAX_FEC_OCTETS];
static EXT_RAM_BSS_ATTR int16_t s_conf_tab[L2_MAX_BLOCKS][RS_VDL2_N];

static vdl2_l2_stats_t s_stats; // single writer; torn reads benign

// Rescued-transmission FCS tap. rs_erasure_recovered counts per RS BLOCK and
// the caller's FCS verdicts arrive per AVLC FRAME, so the aggregates cannot
// answer "does an erasure rescue ever yield an FCS-valid frame?". This
// wrapper closes the gap without any cross-module plumbing: vdl2_l2_feed
// hands avlc_deframe_octets THIS callback with `rescued` = "any RS block in
// the current transmission was erasure-rescued", tallies every yielded
// frame's FCS verdict into rescued_fcs_ok/bad, then forwards to the caller's
// cb untouched. Transmission granularity is deliberate: in a multi-block
// transmission the rescued block's octets may or may not fall inside the
// specific frame checked, but per-octet attribution would need range
// bookkeeping through the destuffer for no extra decision value — the
// gate-the-fallback question only needs "rescues sometimes produce good
// frames" vs "never". TOO_SHORT frames never reach an FCS check and count
// as neither. Instrumentation only — frame delivery is unchanged.
typedef struct {
    avlc_frame_cb_t cb;      // caller's callback (may be NULL = count only)
    void           *ctx;     // caller's context, forwarded untouched
    bool            rescued; // >=1 RS block in this transmission was rescued
} l2_cb_tap_t;

static void l2_avlc_tap(const avlc_frame_t *f, void *ctx)
{
    l2_cb_tap_t *t = (l2_cb_tap_t *)ctx;
    if (t->rescued) {
        if (f->kind == AVLC_KIND_BAD_FCS)
            s_stats.rescued_fcs_bad++;
        else if (f->kind != AVLC_KIND_TOO_SHORT)
            s_stats.rescued_fcs_ok++;
    }
    if (t->cb) t->cb(f, t->ctx);
}

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

// Per-OCTET confidence parallel to pack_lsbfirst: octet i's confidence is
// the MIN over its 8 bits of |soft_bits[bit]| (weakest-link — an octet is
// only as trustworthy as its least-confident bit). |INT16_MIN| is clamped
// to 32767 so the negation cannot overflow.
static void pack_conf_lsbfirst(const int16_t *soft, uint32_t base_bit,
                               uint32_t n_octets, int16_t *out)
{
    for (uint32_t i = 0; i < n_octets; i++) {
        int32_t m = 32767;
        for (int j = 0; j < 8; j++) {
            int32_t a = soft[base_bit + 8 * i + j];
            if (a < 0) a = -a;
            if (a > 32767) a = 32767;
            if (a < m) m = a;
        }
        out[i] = (int16_t)m;
    }
}

// int16 twin of l2_deinterleave() — IDENTICAL (row, col) walk so the
// confidence table lands in lockstep with s_rs_tab. Zero-pad positions of
// a shortened last row get confidence 0; the shortened-block fallback never
// selects them because select_weakest_confined restricts candidates to the
// transmitted symbols (data + transmitted parity) — a confidence 0 here must
// NOT be read as "erase me first".
static int l2_deinterleave_conf(const int16_t *in, uint32_t len, uint32_t rows,
                                int16_t out[][RS_VDL2_N], uint32_t fillwidth,
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
            out[row][col] = 0;
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

// Pick the `count` lowest-confidence positions among block row `r`
// (all RS_VDL2_N positions are candidates — used only for FULL blocks,
// where every symbol is transmitted). Writes distinct block indices into
// erasure_pos[0..count). count must be <= RS_VDL2_NROOTS (the erasure
// budget: 2e+f <= NROOTS). O(count * N), trivial next to the RS decode.
// The f-sweep calls this once per rung; since it always ranks from the
// weakest, the count=2 set is a strict prefix of count=4 and count=6.
static void select_weakest(const int16_t conf[RS_VDL2_N],
                           uint8_t erasure_pos[], int count)
{
    bool taken[RS_VDL2_N] = { false };
    for (int e = 0; e < count; e++) {
        int     best_pos  = -1;
        int32_t best_conf = 0;
        for (int p = 0; p < RS_VDL2_N; p++) {
            if (taken[p]) continue;
            if (best_pos < 0 || conf[p] < best_conf) {
                best_pos  = p;
                best_conf = conf[p];
            }
        }
        taken[best_pos]  = true;
        erasure_pos[e]   = (uint8_t)best_pos;
    }
}

// Confined twin of select_weakest for SHORTENED blocks: the candidate set
// is restricted to the actually TRANSMITTED symbols — data octets
// [0 .. data_len-1] and transmitted parity [RS_K .. RS_K+fec-1]. Zero-pad
// positions [data_len .. RS_K-1] (known-zero, untransmitted) and
// untransmitted-parity positions [RS_K+fec .. 254] (already structural
// erasures) are NEVER selected, even though their confidence is 0 (the
// weakest possible) — picking them would waste the tiny confidence budget on
// a known-good symbol or duplicate a structural erasure. `count` distinct
// weakest transmitted positions are written to erasure_pos[0..count). The
// caller guarantees count <= (number of transmitted symbols); for every
// dumpvdl2 shortened scheme the transmitted count (data + fec) comfortably
// exceeds the confidence budget fec. Like select_weakest, the count=2 set is
// a strict prefix of count=4/6 (weakest-first), so the f-sweep rungs nest.
static void select_weakest_confined(const int16_t conf[RS_VDL2_N],
                                    uint8_t erasure_pos[], int count,
                                    uint32_t data_len, int fec)
{
    uint32_t struct_lo = RS_VDL2_K + (uint32_t)fec; // structural region start
    bool     taken[RS_VDL2_N] = { false };
    for (int e = 0; e < count; e++) {
        int     best_pos  = -1;
        int32_t best_conf = 0;
        for (uint32_t p = 0; p < RS_VDL2_N; p++) {
            bool transmitted = (p < data_len) ||
                               (p >= RS_VDL2_K && p < struct_lo);
            if (!transmitted || taken[p]) continue;
            if (best_pos < 0 || conf[p] < best_conf) {
                best_pos  = (int)p;
                best_conf = conf[p];
            }
        }
        if (best_pos < 0) break; // no more transmitted candidates (defensive)
        taken[best_pos] = true;
        erasure_pos[e]  = (uint8_t)best_pos;
    }
}

int vdl2_l2_feed(const uint8_t *bits, const int16_t *soft_bits, int n_bits,
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

    // Soft-decision confidence table, built in lockstep with the data/FEC
    // de-interleave above so s_conf_tab[r][c] is the reliability of the
    // symbol at s_rs_tab[r][c]. Only when the caller supplied soft_bits.
    bool have_conf = (soft_bits != NULL);
    if (have_conf) {
        pack_conf_lsbfirst(soft_bits, VDL2_HDR_BITS, datalen_octets,
                           s_data_conf);
        pack_conf_lsbfirst(soft_bits, VDL2_HDR_BITS + 8 * datalen_octets,
                           fec_octets, s_fec_conf);
        // memset the confidence rows so a shortened last row's zero-pad and
        // untransmitted-parity positions read as 0 (never chosen — the
        // shortened fallback's select_weakest_confined excludes them).
        memset(s_conf_tab, 0, num_blocks * RS_VDL2_N * sizeof(int16_t));
        if (l2_deinterleave_conf(s_data_conf, datalen_octets, num_blocks,
                                 s_conf_tab, RS_VDL2_K, 0) < 0)
            have_conf = false;
        if (have_conf && fec_rows > 0 &&
            l2_deinterleave_conf(s_fec_conf, fec_octets, fec_rows, s_conf_tab,
                                 RS_VDL2_NROOTS, RS_VDL2_K) < 0)
            have_conf = false;
    }

    // RS-correct each block; any uncorrectable block drops the whole
    // transmission (decode.c:311-316 "FEC check failed" -> cleanup).
    bool any_rescued = false; // >=1 block erasure-rescued (l2_avlc_tap flag)
    for (uint32_t r = 0; r < num_blocks; r++) {
        int corr = 0;
        int rc;
        int fec_this = (r == num_blocks - 1) ? fec_octetcount(last)
                                             : RS_VDL2_NROOTS;
        // A block is FULL (all 255 symbols transmitted, no structural
        // erasure/shortening) when it carries the whole 249 data octets:
        // every non-last block, plus the last block when the octet count
        // divides evenly (last == RS_VDL2_K).
        uint32_t data_this = (r == num_blocks - 1) ? last : RS_VDL2_K;
        bool     is_full   = (data_this == RS_VDL2_K);

        // Hard-decision FIRST (unchanged fast path). Save the received
        // codeword so the erasure retry starts from clean data: a failed
        // rs_vdl2_decode may have partially XOR'd a miscorrection into the
        // block before rejecting it (rs_vdl2.h "contents unspecified").
        // Needed for BOTH the full- and shortened-block fallbacks below.
        uint8_t saved[RS_VDL2_N];
        if (have_conf) memcpy(saved, s_rs_tab[r], RS_VDL2_N);

        if (r == num_blocks - 1) {
            rc = rs_vdl2_decode_shortened(s_rs_tab[r], (int)last, &corr);
        } else {
            rc = rs_vdl2_decode(s_rs_tab[r], &corr);
        }

        bool erasure_rescued = false;
        if (rc != 0 && have_conf && is_full) {
            // Soft-decision erasure fallback (the VDL2 analog of Iridium's
            // Chase-2 fallback after hard-BCH fails). rs_vdl2_decode_erasures
            // corrects 2e+f <= 6: marking the f least-reliable positions as
            // erasures buys error-correction reach beyond the hard t=3.
            //
            // ITERATIVE f-SWEEP (ascending f): try f=2 (still corrects e<=2
            // errors ANYWHERE), then f=4 (e<=1), then f=6 (e=0). Accept the
            // FIRST f that decodes. Smallest f wins = maximum residual
            // error-correction margin = safest: we only *assume* a symbol is
            // erased when a smaller-f attempt has already failed. This
            // strictly dominates a single f=6 attempt (f=6 is merely the last
            // rung): it also catches patterns where a true error lies OUTSIDE
            // the 6 weakest symbols (f=6/e=0 cannot fix those; f=2/e<=2 can),
            // and it lowers miscorrection risk because more parity is left for
            // detection at low f. Because select_weakest always ranks from the
            // weakest, the f=2 erasure set is a prefix of f=4's, which is a
            // prefix of f=6's — the rungs are nested.
            //
            // SAFETY: a full-erasure (f=6) decode always yields *some* valid
            // codeword, so a wrong guess is a miscorrection, not a hard
            // error. That is acceptable here because the downstream AVLC
            // X.25 FCS validates the entire transmission: a bad erasure
            // correction produces garbage octets that fail the FCS
            // (AVLC_KIND_BAD_FCS) — it cannot fabricate a false-positive
            // ACARS. The FCS is the final arbiter, exactly as the frame CRC
            // is for Iridium's speculative decodes. Ascending f only tightens
            // this: lower f leaves more parity for FCS-independent detection.
            static const int k_erasure_sweep[] = { 2, 4, RS_VDL2_NROOTS };
            uint8_t          erasure_pos[RS_VDL2_NROOTS];
            for (unsigned si = 0;
                 si < sizeof(k_erasure_sweep) / sizeof(k_erasure_sweep[0]);
                 si++) {
                int f = k_erasure_sweep[si];
                // Restore the clean received codeword before EACH attempt: a
                // failed erasure decode may also partially mutate the block.
                memcpy(s_rs_tab[r], saved, RS_VDL2_N);
                select_weakest(s_conf_tab[r], erasure_pos, f);
                rc = rs_vdl2_decode_erasures(s_rs_tab[r], erasure_pos, f,
                                             &corr);
                if (rc == 0) {
                    erasure_rescued = true;
                    break;
                }
            }
        } else if (rc != 0 && have_conf && !is_full) {
            // SHORTENED-block soft-decision erasure fallback — the piece that
            // makes erasure recovery apply to real VDL2 traffic (almost all
            // of which is short). The block already spends f_structural =
            // NROOTS - fec parity symbols on the untransmitted-parity
            // structural erasures, so the confidence budget is only
            // NROOTS - f_structural = fec erasures:
            //   2-parity (fec=2): budget 2 -> single rung f_conf=2 (2e+f=6 =>
            //                     e=0; a pure 2-symbol erasure fix, extending
            //                     the hard t=1 to "2 errors iff they are the
            //                     2 weakest transmitted symbols").
            //   4-parity (fec=4): budget 4 -> rungs f_conf=2 (e<=1), 4 (e=0).
            //   6-parity (fec=6, shortened, last<RS_K): budget 6 -> rungs
            //                     2/4/6, identical reach to a full block.
            // Same ASCENDING f-sweep rationale as the full-block branch:
            // smallest f wins (max residual error-correction margin, most
            // parity left for detection), rungs are nested. The confidence
            // erasures are selected from the TRANSMITTED symbols ONLY
            // (select_weakest_confined): a zero-pad or untransmitted-parity
            // position is never chosen (it is known-zero / already
            // structural), which is mandatory here because those positions
            // carry confidence 0 (weakest) and an unconfined select would
            // spend the whole budget on them. FCS-backstop safety is
            // identical to the full-block branch: a wrong erasure guess
            // miscorrects to garbage that fails the AVLC X.25 FCS — it cannot
            // fabricate a false-positive ACARS.
            int budget = fec_this; // == NROOTS - f_structural
            static const int k_short_sweep[] = { 2, 4, RS_VDL2_NROOTS };
            uint8_t          conf_pos[RS_VDL2_NROOTS];
            for (unsigned si = 0;
                 si < sizeof(k_short_sweep) / sizeof(k_short_sweep[0]);
                 si++) {
                int f_conf = k_short_sweep[si];
                if (f_conf > budget) break; // ascending; rest exceed budget
                memcpy(s_rs_tab[r], saved, RS_VDL2_N);
                select_weakest_confined(s_conf_tab[r], conf_pos, f_conf, last,
                                        fec_this);
                rc = rs_vdl2_decode_shortened_erasures(s_rs_tab[r], (int)last,
                                                       conf_pos, f_conf, &corr);
                if (rc == 0) {
                    erasure_rescued = true;
                    break;
                }
            }
        }

        if (rc != 0) {
            s_stats.rs_blocks_fail++;
            return VDL2_L2_ERR_RS;
        }
        s_stats.rs_blocks_ok++;
        // True rescue: hard-decision had FAILED and the erasure fallback
        // succeeded (subset of rs_blocks_ok, like chase_recovered).
        if (erasure_rescued) {
            s_stats.rs_erasure_recovered++;
            any_rescued = true;
        }
        // Corrected-octet accounting excludes the intended erasures of
        // a shortened block (dumpvdl2 decode.c:321-322). For a full-block
        // erasure rescue, fec_this == NROOTS so no adjustment applies.
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
    // Routed through l2_avlc_tap so a rescued transmission's frames get
    // their FCS verdicts tallied (rescued_fcs_ok/bad) before forwarding.
    l2_cb_tap_t tap = { .cb = cb, .ctx = ctx, .rescued = any_rescued };
    int nf = avlc_deframe_octets(s_octets, (int)datalen, l2_avlc_tap, &tap);
    if (nf > 0) s_stats.avlc_frames += (uint32_t)nf;
    return nf;
}

void vdl2_l2_get_stats(vdl2_l2_stats_t *out)
{
    if (out) *out = s_stats;
}
