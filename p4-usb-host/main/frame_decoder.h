#ifndef FRAME_DECODER_H
#define FRAME_DECODER_H

// Higher-layer decoder task. Pulls bit packets out of a PSRAM queue
// (frame_queue), classifies them via iridium_frame_classify, and (in
// later phases) feeds IDA-LCW frames into the SBD reassembler and
// libacars.
//
// Lifecycle: frame_decoder_init() once at startup, before any worker
// pushes. Returns ESP_OK on success. The task pins to CORE 0 at
// priority 6 (same as dsp_feed; below usb_pump at 7) — moved off
// Core 1 in #123, with a one-frame-per-wake + taskYIELD discipline
// after the 2026-07-08 WDT incident. See DECODER_PRIO/DECODER_CORE in
// frame_decoder.c (the authoritative values) and
// docs/2026-07-17-core-priority-topology-review.md §1.

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
#include "qpsk_demod.h" // ir_direction_t
#include "ida_reassembler.h" // IDA_PARTS_BINS (parts-per-chain histogram size)

#ifdef __cplusplus
extern "C" {
#endif

// Initialise the decoder: allocate PSRAM-backed queue, spawn the
// consumer task. Idempotent — subsequent calls return ESP_OK without
// reinitialising. (The task is deliberately NOT watchdog-subscribed;
// see the rationale in decoder_task().)
esp_err_t frame_decoder_init(void);

// Producer-side push. Called by worker_core1 after qpsk_demod reports
// success (and the bits are still in scope). Bits[] must be 0/1-per-byte
// from qpsk_demod; n_bits is typically 382 for an Iridium burst.
//
// Non-blocking — if the queue is full, the frame is dropped and the
// frame_decoder_dropped() counter advances. Always safe to call from
// worker_core1's task; not ISR-safe (uses memcpy of ~400 bytes).
//
// Returns true on enqueue, false if dropped or if frame_decoder_init
// hasn't been called yet.
// soft_bits/n_soft: qpsk_demod's per-bit soft metrics for the SAME bits
// (Chase-2 soft BCH fallback, task #16). Optional — pass NULL/0 when
// unavailable (aggregator PDU path, smoke corpus, demod soft OOM); the
// chase fallback is then simply skipped for that frame. Truncated to
// FRAME_QUEUE_MAX_SOFT entries (only bits [0, 382) are consumed).
// timestamp_us is the burst's CAPTURE time (from its sample position), not
// decode time, so processing order doesn't reorder emitted timestamps. Pass 0
// to fall back to esp_timer_get_time() at enqueue (callers without a capture
// clock, e.g. the aggregator/corpus paths).
bool frame_decoder_push(const uint8_t *bits, size_t n_bits,
                        const int16_t *soft_bits, size_t n_soft,
                        ir_direction_t direction,
                        uint32_t freq_hz, int peak_bin, float snr_db,
                        uint64_t timestamp_us);

// Stats accessors for the per-second status block.
uint64_t frame_decoder_pushed(void);  // total bursts the worker handed off
uint64_t frame_decoder_popped(void);  // total bursts the decoder processed
uint64_t frame_decoder_dropped(void); // total dropped (queue full)
size_t   frame_decoder_queue_count(void);

// Per-frame-class counts since boot. Useful for the status block.
typedef struct {
    uint64_t unknown;
    uint64_t ms;
    uint64_t tl;
    uint64_t bc;
    uint64_t lw_da;    // LW with subtype DA — these are SBD/ACARS-bearing
    uint64_t lw_other; // LW with any other subtype (VO/IP/SY/U3/U6/...)
} frame_decoder_class_counts_t;
void frame_decoder_get_class_counts(frame_decoder_class_counts_t *out);

// Per-LW.DA-frame relative-frequency histogram for the decode-based band
// survey (decode_survey.c, Phase C). 40 bins spanning the ±FS_DETECT_HZ/2
// detect window; low bin = most-negative baseband offset. Cumulative since
// boot; the survey snapshots deltas per visit and folds them into an
// absolute-freq map at the visit's known LO. Copies min(max_bins, BINS).
#define FRAME_DECODER_LWDA_FREQ_BINS 40
void    frame_decoder_get_lwda_freq_hist(uint32_t *out, int max_bins);
// Signed baseband centre frequency (Hz, relative to the LO) of histogram bin b.
int32_t frame_decoder_lwda_freq_bin_center_hz(int bin);

// Lifetime totals (since boot) of the two ACARS-pipeline counters.
// Cheap relaxed atomic loads; safe to call from any thread.
uint64_t frame_decoder_acars_decoded_total(void);
uint64_t frame_decoder_sbd_complete_total(void);

// Reassembly-chain diagnostics (why don't lw_da frames become messages?).
// Surfaces the internal counters of the three-stage chain so a caller can
// tell reception-completeness (sessions open but never complete → missing
// fragments) from a parse/gate problem (frames rejected before a session
// even opens). Cumulative since boot; relaxed/torn reads are benign for a
// diagnostic. See /diag/reassembler.
typedef struct {
    // frame_decoder gate
    uint64_t lw_da;        // classified LW.DA (SBD/ACARS-bearing) frames
    uint64_t lw_da_valid;  // ...that passed ida_decode (ok+header+crc) → fed to the chain
    uint64_t sbd_complete; // SBD envelopes reassembled
    uint64_t acars_decoded;
    uint64_t acars_fragments; // ACARS blocks buffered, awaiting more (libacars)
    // Stage 1 — ida_reassembler (cross-burst da_cont/da_ctr chaining)
    uint32_t ida_standalone, ida_opened, ida_merged, ida_completed;
    uint32_t ida_orphan, ida_overflow, ida_expired;
    uint32_t ida_orphan_freq; // subset of ida_orphan rejected only on the freq key (Task#6)
    // Parts-per-chain histograms (index = fragment count, clamped to IDA_PARTS_BINS-1):
    // shows whether longer multi-fragment messages complete or fail to reassemble.
    uint32_t ida_parts_completed[IDA_PARTS_BINS];
    uint32_t ida_parts_expired[IDA_PARTS_BINS];
    // Stage 2 — sbd_reassembler (SBD envelope, msgno/msgcnt)
    uint32_t sbd_short, sbd_single, sbd_assembled, sbd_multi, sbd_broken, sbd_filtered;
    // Chain salvage (§9): partials reaped from timed-out IDA chains.
    uint32_t salvage_ok;       // reaped partial that classifies as an SBD type
    uint32_t salvage_rejected; // reaped partial that is non-SBD (control traffic)
    uint32_t dirty_cont;       // clean-demod continuation that failed ONLY its own
                               // crc — the Task C population (counted whether or
                               // not best_effort_decode admitted it)
    uint32_t dirty_emitted;    // Task C: dirty-but-complete chains that classified
                               // SBD and were emitted PARTIAL (best_effort ON)
    uint64_t acars_partial;    // Task B4: salvage.ok chains actually EMITTED as a
                               // PARTIAL /messages row (best_effort_decode gate ON).
                               // Never counted in acars_decoded — display-only,
                               // untrusted, crc_ok always false.
    // Chase-2 soft BCH fallback (task #16; NVS chase2_decode, default OFF).
    uint32_t chase_attempts;   // hard-BCH-failed LW.DA frames that entered the chase
    uint32_t chase_recovered;  // ...that the CRC-16 arbiter accepted (counted in
                               // lw_da_valid too — they are full valid decodes)
    uint32_t chase_crc_checks; // total CRC arbiter checks spent (x 2^-16 =
                               // expected false-accept exposure)
} frame_decoder_reasm_stats_t;
void frame_decoder_get_reasm_stats(frame_decoder_reasm_stats_t *out);

// band=vdl2 decode-funnel counters (V3): PHY frames popped -> RS block
// verdicts -> AVLC frame kinds -> ACARS routed to libacars. All zero
// under band=iridium. Cumulative since boot; cheap relaxed loads (the
// RS trio are plain single-writer reads, torn-read benign). Served in
// /status "decode.vdl2".
typedef struct {
    uint64_t phy_frames; // demodulated VDL2 transmissions popped
    uint64_t l2_fail;    // dropped whole (header/truncation/RS failure)
    uint32_t rs_blocks_ok, rs_blocks_fail, rs_octets_fixed;
    uint64_t avlc_ok;     // FCS-valid AVLC frames (= acars+x25+sup+unnum)
    uint64_t acars;       // ACARS-bearing I frames handed to libacars
    uint64_t x25;         // ATN/X.25 I frames (counted, not decoded — V5)
    uint64_t supervisory; // S frames (RR/REJ/... link management)
    uint64_t unnumbered;  // U frames (XID/TEST/...)
    uint64_t bad_fcs;     // FCS-failed frames
    uint64_t too_short;   // destuffed frames < 11 octets
} frame_decoder_vdl2_stats_t;
void frame_decoder_get_vdl2_stats(frame_decoder_vdl2_stats_t *out);

// Rolling decode-rate counters (#117). Sum of classified-as-known-type
// frames over the last 1 h and 24 h, snapped on a 1-minute esp_timer
// tick. A WARN log fires automatically when 24h>10 && 1h==0 ("we
// used to work, we no longer do") — catches silent DSP wedge.
void frame_decoder_get_rolling_rates(uint32_t *out_1h, uint32_t *out_24h);

#ifdef __cplusplus
}
#endif

#endif // FRAME_DECODER_H
