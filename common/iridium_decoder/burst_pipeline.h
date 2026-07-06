// burst_pipeline.h — per-burst orchestration shared between the P4
// worker (worker_core1.c) and the host pipeline test
// (test_worker_pipeline_albq.c).
//
// Why this module exists: both contexts run the same sequence of
// DSP steps after the channelizer + resample-to-250kHz stage, but
// historically each implemented the orchestration separately. The
// host harness's reimplementation drifted from the worker — missing
// sub-sample timing correction, float vs Q15 precision, etc. — and
// produced 10× fewer decodes on the same input data. Folding the
// orchestration into one shared module eliminates the drift.
//
// Pipeline (gr-iridium burst_downmix order):
//   1. D13 envelope start_finder on the raw 250 kHz burst.
//   2. Squared-FFT CFO estimate on the PRE-RRC signal anchored at
//      D13's start (uw_correlator_estimate_cfo).
//   3. Phase-correct adj_burst by the coarse omega.
//   4. RRC matched filter on the corrected burst.
//   5. UW correlator (matched filter + parabolic interp).
//   6. Pre-rotation: peak-phase + uw_res.correction sub-sample interp
//      + omega_per_sym linear phase ramp, all in Q15.
//   7. 5:1 decimation 10 sps → 2 sps.
//   8. qpsk_demod_process.
//
// The 40 kHz channelizer + 40→250 kHz resample step stays platform-
// specific (esp-dsp's dsps_fird_s16 on P4, portable polyphase on
// host) because the resampler implementations don't have a clean
// portable interface yet. Caller must hand burst_pipeline a fully
// resampled 250 kHz int16 IQ buffer.

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "uw_correlator.h"
#include "qpsk_demod.h"

// ABI note: this struct is used WITHIN one compiled process — never
// marshalled across machines or written to flash. It contains a
// pointer (decoded_frame_t.bits) so sizeof differs between RV32 (P4,
// 4-byte pointer) and x86-64 (host, 8-byte pointer); that's fine
// because each binary uses its own compiler-consistent layout. We
// don't pack the struct; natural alignment is identical between
// the toolchains we target (RISC-V GCC for P4, GCC for x86-64 host,
// both little-endian). Don't add `__attribute__((packed))` here —
// historic ESP32 Xtensa builds crash on misaligned 32-bit loads from
// packed structs; ESP32-P4 (RV32) is more permissive but slow on
// unaligned access, and there's no need.
typedef struct {
    uw_corr_result_t uw_res;       // matched-filter + CFO diagnostic
    decoded_frame_t  frame;        // populated when demod_ok; caller frees frame.bits
    float            omega_coarse; // pre-RRC squared-FFT estimate (rad/sym)
    int              burst_start;  // D13 envelope start (samples into iq250)
    int              n_post_2sps;  // 2-sps samples handed to qpsk_demod
    bool             demod_ok;     // true if qpsk_demod produced a frame
} burst_pipeline_result_t;

// Run the per-burst pipeline on a 250 kHz (10 sps) int16 IQ buffer.
// `iq250` is the resampled burst as interleaved I, Q int16; `n_complex`
// is the number of complex samples (= int16 count / 2).
//
// The function modifies `iq250` IN PLACE — D13 trim is a pointer
// offset, RRC filter writes back over the buffer, pre-rotation and
// decim are in-place. Caller must pass a writeable buffer.
//
// Returns true if the pipeline ran end-to-end (regardless of decode
// outcome — check result->demod_ok for that). Returns false only on
// guard conditions (burst too short for D13 / matched filter).
//
// Side-effects: NONE outside `iq250` and `*result`. No logging, no
// allocation beyond decoded_frame_t.bits (which is the caller's to
// free when demod_ok).
//
// LEGACY SINGLE-FRAME API. For multi-frame bursts this only returns
// the first decoded frame. Prefer burst_pipeline_process_burst() for
// new code so additional sub-frames are not lost.
bool burst_pipeline_process_250khz(int16_t *iq250, int n_complex,
                                   burst_pipeline_result_t *result);

// Callback fired once per successfully-decoded frame within a burst.
// The result is borrowed for the duration of the callback; the
// callback OWNS result->frame.bits and must free it (or hand it off)
// before returning. ctx is the user-provided context pointer.
typedef void (*burst_pipeline_frame_cb)(burst_pipeline_result_t *res, void *ctx);

// Multi-frame variant. Runs D13/CFO/RRC once on the burst, then
// iterates the per-frame stages (UW correlator + pre-rotate + decim +
// qpsk_demod) along the burst -- mirroring gri's
// handle_multiple_frames_per_burst (lib/burst_downmix_impl.cc:890-905).
//
// Behaviour:
//   - First frame: same retry-on-failure logic as the legacy single-
//     frame API to recover bursts where the first try misses the UW.
//   - Subsequent frames: advance the search position by one frame
//     length (191 sym × 10 sps = 1910 samples at 250 ksps), starting
//     from each emitted frame's UW position. Stops when no room is
//     left for another frame.
//   - Each successful frame fires the callback with the result and
//     freshly malloc'd frame.bits (callback owns).
//
// Returns the number of frames emitted (0 if no decode).
int burst_pipeline_process_burst(int16_t *iq250, int n_complex,
                                 burst_pipeline_frame_cb cb, void *ctx);

// ---------------------------------------------------------------------
// P1.5a fast-pass triage (Design A).
//
// The worker's per-burst cost is dominated (~94%) by the first-frame
// retry loop in burst_pipeline_process_burst — up to ~40
// try_decode_frame calls compensating for our over-wide tagger windows
// (task #70); gr-iridium makes ONE UW attempt per frame slot. Junk
// bursts (impulsive broadband RFI) pay that full retry cost before
// failing. The triage pass gives a cheap real verdict first: run the
// identical steps 0-4 head (DC removal / D13 / coarse CFO / pre-rotate
// / RRC) plus ONE try_decode_frame at search_start = 0 on a TRUNCATED
// fixed window, and accept iff it decodes — i.e. iff qpsk_demod's UW
// check (gri's diffs <= 2) passes. On ACCEPT the caller re-extracts the
// FULL window and runs burst_pipeline_process_burst unchanged (retries,
// multi-frame, BCH); on REJECT it drops the burst and counts it.
//
// Triage window length in complex samples at 250 ksps. Every term is an
// existing pipeline constant (no tuned numbers):
//     1750  D13 start-finder search depth (gri's 0.007 × 250 ksps,
//           SEARCH_DEPTH_SAMPLES in burst_pipeline.c)
//   + 1778  SYNC_SEARCH_LEN — the UW correlator's alias-free lag count
//           (2048-pt FFT − 271-sample reference + 1); first-attempt UW
//           offsets can't exceed this
//   + 1910  one frame (191 sym × 10 sps = MAX_FRAME_LEN_NORMAL_10SPS)
//           past the worst-case UW offset, what try_decode_frame's
//           rotate/interp/decim consumes
//   +  300  SYNC_SEARCH_LEN_GUARD margin (covers the +4 interp
//           look-ahead and RRC tail edge effects)
//   = 5738  (~23 ms at 250 ksps)
// A window at least this long gives the triage attempt the SAME
// effective UW search range and frame span as the full path's first
// attempt; the only remaining difference is the per-burst DC mean /
// RRC tail computed over the truncated instead of full window.
#define BURST_PIPELINE_TRIAGE_LEN_250K (1750 + 1778 + 1910 + 300)

// Run the triage verdict chain on `iq250` (typically the first
// BURST_PIPELINE_TRIAGE_LEN_250K complex samples of a burst window;
// shorter bursts pass their whole window). Modifies iq250 IN PLACE
// (same stages as the full pipeline) — the caller must NOT reuse the
// buffer for the escalated full pass; re-extract instead (on-device:
// re-read from the signal ring, exactly like today's full path, so
// decim phase/alignment is untouched).
//
// Returns true = ACCEPT (one-attempt decode succeeded; escalate),
// false = REJECT (drop the burst). Does not consume the one-shot
// force-start/dump diagnostics (those apply to the full call).
bool burst_pipeline_triage(int16_t *iq250, int n_complex);

// Diagnostic (P1.5a positive control): search_start at which the most
// recent burst_pipeline_process_burst call decoded its FIRST frame.
//   0  = first try_decode_frame attempt (the population triage must
//        accept — same single-attempt criterion)
//   >0 = recovered by the retry loop (triage's recall-risk population)
//   -1 = no first frame decoded (or head guard bailed)
int burst_pipeline_last_first_search_start(void);

// Diagnostic: override D13's burst_start for the NEXT call only,
// then auto-clear. Used by the path-C host harness to bypass our
// envelope start-finder when we already have a known-good start
// position (e.g. from gr-iridium's fft_burst_tagger), so we can
// isolate downstream defects from D13 mispositioning errors.
// Pass -1 to leave D13 running normally.
void burst_pipeline_force_start_once(int sample_idx);

// Diagnostic: when non-NULL, the pipeline writes intermediate
// signals (as interleaved float32 IQ in [-1, +1]) to the given
// directory before/after each major stage. Filenames match the
// /tmp/host_signals/ layout that tests/scripts/stagewise_compare.py
// expects:
//   04_post_d13_250k.cf32   — after start_finder trim, pre-CFO
//   05_post_cfo_250k.cf32   — after coarse omega freq-correction
//   06_post_rrc_250k.cf32   — after RRC matched filter
//   07_post_prerot_250k.cf32 — after peak-phase + linear-ramp rotation
// Call burst_pipeline_set_dump_dir(NULL) to disable. Single global
// state — only the next call to burst_pipeline_process_250khz dumps,
// then auto-disables (to avoid dumping every burst).
void burst_pipeline_set_dump_once(const char *dir);

// Diagnostic — read accumulated per-stage wall-times since the last
// call, plus loop iteration counts. Order:
//   out[0]=D13, out[1]=CFO, out[2]=PREROT, out[3]=RRC,
//   out[4]=first try_decode_frame call (whole), out[5]=retry-loop total.
//   out[6]=TDF_UW (matched filter), out[7]=TDF_PREROT (peak-phase rotate),
//   out[8]=TDF_DECIM (sub-sample interp + decim), out[9]=TDF_QPSK
//   (qpsk_demod_process). out[6..9] accumulate across BOTH first and
//   retry calls.
// Returns first/retry call counts via out params so per-call cost
// can be derived.
void burst_pipeline_get_stage_us(uint32_t out[10], uint32_t *first_calls,
                                 uint32_t *retry_calls);
