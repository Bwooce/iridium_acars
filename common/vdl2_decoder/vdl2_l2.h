// vdl2_l2 — VDL Mode 2 L2 feed: descrambled PHY bit vector (header
// included, as emitted by vdl2_demod / carried through frame_queue) ->
// byte packing -> RS-block de-interleave -> RS(255,249) error
// correction (common/vdl2/rs_vdl2.h) -> corrected octet stream ->
// AVLC deframe (common/vdl2/avlc.h). Plan §C4 / phase V3
// (docs/2026-07-22-vdl2-implementation-plan.md).
//
// This is the exact post-demod chain of dumpvdl2's decode_vdl2_burst
// DEC_DATA state (src/decode.c:259-342, v2.6.0 3f583da) — the
// cross-validation reference for this band. The deinterleaver geometry,
// per-block FEC octet counts and whole-burst abort semantics are ports,
// cited per-step in vdl2_l2.c.
//
// Split of responsibilities (mirrors Iridium): vdl2_pipeline.c is
// DSP -> PHY bits on the worker; THIS module is the cheap byte-work L2
// that runs downstream in the emit sink (on device: the Core-0
// frame_decoder task; on host: the test harness). The caller's
// avlc_frame_cb_t receives every deframed AVLC frame and routes
// AVLC_KIND_ACARS payloads onward to libacars.
//
// THREADING: not reentrant (static working buffers, ~9 KB, PSRAM on
// target). Single-consumer only — same contract as avlc.c's deframe
// buffer, and the two are only ever called from the same task.

#pragma once

#include <stdint.h>

#include "avlc.h" // avlc_frame_cb_t (common/vdl2)

#ifdef __cplusplus
extern "C" {
#endif

// Error returns (0..n = AVLC frames emitted).
#define VDL2_L2_ERR_HEADER   (-1) // burst-header re-decode/length reject
#define VDL2_L2_ERR_TRUNC    (-2) // bit vector shorter than the header says
#define VDL2_L2_ERR_RS       (-3) // an RS block was uncorrectable (burst dropped)
#define VDL2_L2_ERR_INTERNAL (-4) // deinterleave geometry error (can't happen
                                  // on a header-validated length; defensive)

// Feed one demodulated VDL2 transmission. bits = descrambled hard bits
// (one per byte, LSB of each byte), bits[0..24] = burst header,
// bits[25..] = data + RS FEC octets as transmitted — exactly
// vdl2_demod_result_t.bits / the frame_queue item payload. n_bits must
// cover the whole transmission (the demod's `complete` flag).
//
// soft_bits: OPTIONAL per-bit soft confidence, aligned index-for-index
// with bits[] over the same n_bits (sign = hard decision, magnitude =
// reliability; vdl2_demod.h). When non-NULL it enables the soft-decision
// RS erasure fallback on ANY block hard-decision decoding cannot correct,
// retried as an erasure decode over its least-reliable symbols:
//   - FULL blocks (all 255 symbols transmitted): the f-sweep marks the f
//     globally weakest symbols as erasures. rs_vdl2_decode_erasures corrects
//     2e+f <= 6, sweeping f ascending (2, 4, 6) and accepting the first that
//     decodes — max error-correction margin at low f, escalating to f=6/e=0
//     only as a last resort.
//   - SHORTENED last blocks (real VDL2 traffic is almost all short): the
//     block already spends f_struct = 6 - fec syndromes on structural
//     (untransmitted-parity) erasures, so the confidence budget is fec. The
//     f-sweep (rs_vdl2_decode_shortened_erasures) marks the fec weakest
//     *transmitted* symbols only — never a zero-pad or untransmitted-parity
//     position — up to that budget. Note: with a large structural budget
//     (2-/4-parity) the hard decoder frequently ALIASES rather than fails on
//     just-past-bound error counts, so the fallback only engages on the
//     subset of patterns hard decoding actually rejects.
// soft_bits == NULL keeps the classic hard-decision-only behaviour
// (backward compatible). The AVLC X.25 FCS is the final arbiter for every
// erasure rescue: a wrong guess miscorrects to garbage that fails the FCS.
//
// Runs header re-decode (cheap, deterministic — the length field is
// needed again here), packs the body LSB-first, de-interleaves into
// RS blocks, corrects each block (full blocks: 6 parity, + optional
// soft-erasure fallback; shortened last block: dumpvdl2's 0/2/4/6-parity
// erasure scheme), assembles the corrected data octets and calls
// avlc_deframe_octets() once over the transmission, firing cb per AVLC
// frame.
//
// Returns the number of AVLC frames emitted, or VDL2_L2_ERR_* (whole
// transmission dropped, mirroring dumpvdl2's per-burst abort). cb may
// be NULL (count only).
int vdl2_l2_feed(const uint8_t *bits, const int16_t *soft_bits, int n_bits,
                 avlc_frame_cb_t cb, void *ctx);

// Cumulative L2 counters (never reset; the /status pattern). Single
// writer = the feeding task; torn reads benign for diagnostics.
typedef struct {
    uint32_t fed;          // transmissions fed
    uint32_t hdr_reject;   // VDL2_L2_ERR_HEADER
    uint32_t truncated;    // VDL2_L2_ERR_TRUNC
    uint32_t rs_blocks_ok; // RS blocks verified/corrected
    uint32_t rs_blocks_fail;   // uncorrectable RS blocks (each drops its burst)
    uint32_t rs_octets_fixed;  // corrected data/parity octets, erasure fills
                               // excluded (dumpvdl2 decode.c:322 accounting)
    uint32_t rs_erasure_recovered; // RS blocks where hard-decision RS failed
                               // but the soft-decision erasure fallback
                               // rescued it — VDL2 analog of Iridium's
                               // bursts_bch_chase_recovered (#112); subset of
                               // rs_blocks_ok.
    // Joint rescue×FCS outcome (transmission granularity — see the
    // l2_avlc_tap comment in vdl2_l2.c). rs_erasure_recovered is per RS
    // BLOCK and the caller's bad_fcs is per AVLC FRAME, so neither can say
    // whether a rescue ever actually yielded a good frame; these two can.
    uint32_t rescued_fcs_ok;  // AVLC frames with a VALID FCS from a
                              // transmission where >=1 RS block was
                              // erasure-rescued — "the rescue produced a
                              // real decode".
    uint32_t rescued_fcs_bad; // ...and the FCS-FAILED frames from those same
                              // transmissions. ok==0 while bad grows = every
                              // rescue miscorrects at this SNR — evidence
                              // for gating the erasure fallback off.
    uint32_t avlc_frames;      // AVLC frames emitted (all kinds)
} vdl2_l2_stats_t;
void vdl2_l2_get_stats(vdl2_l2_stats_t *out);

#ifdef __cplusplus
}
#endif
