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
    uw_corr_result_t uw_res;         // matched-filter + CFO diagnostic
    decoded_frame_t  frame;          // populated when demod_ok; caller frees frame.bits
    float            omega_coarse;   // pre-RRC squared-FFT estimate (rad/sym)
    int              burst_start;    // D13 envelope start (samples into iq250)
    int              n_post_2sps;    // 2-sps samples handed to qpsk_demod
    bool             demod_ok;       // true if qpsk_demod produced a frame
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
bool burst_pipeline_process_250khz(int16_t *iq250, int n_complex,
                                    burst_pipeline_result_t *result);

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
