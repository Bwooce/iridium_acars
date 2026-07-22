// band_pipeline — the per-burst demod pipeline interface every band
// implements (VHF/VDL2 foundation; see
// docs/2026-07-22-vdl2-implementation-plan.md §band_pipeline).
//
// Designed from the worker's real call site (p4-usb-host/main/
// worker_core1.c worker_task): after the band-agnostic front half
// (signal_buffer extract → rotate-to-DC at the tagger's rel_freq →
// 10× decim to 250 ksps), the worker makes exactly two per-burst calls:
//
//   1. burst_prefilter(iq250, n, width_bins, &pf)          [cheap junk gate]
//   2. burst_pipeline_process_burst(iq250, n, cb, ctx)      [demod chain]
//
// This interface abstracts those two calls so Iridium's existing
// burst_pipeline (common/iridium_decoder/, adapted by
// iridium_band_pipeline.c) and the future vdl2_pipeline
// (common/vdl2_decoder/) both satisfy one vtable, and the worker
// dispatches through a pointer chosen once at init from the NVS band.
//
// INPUT CONTRACT (both hooks): interleaved int16 I,Q at
// FS_DETECT / 10 = 250 ksps, already rotated so the tagged burst is
// centred at DC. `n_complex` = complex sample count. process_burst may
// modify the buffer IN PLACE (Iridium's does); prefilter must not.
//
// OWNERSHIP: the frame callback OWNS f->bits and f->soft_bits (both
// heap-allocated by the pipeline, soft_bits may be NULL) and must
// free() or hand them off before returning — same contract as
// burst_pipeline_frame_cb (common/iridium_decoder/burst_pipeline.h).
//
// Pure header, no ESP-IDF deps — host tests compile against it as-is.

#pragma once

#include <stdbool.h>
#include <stdint.h>

// One demodulated frame, as handed from a band pipeline to the worker's
// emit callback. "Frame" here is the band's PHY-layer unit (Iridium: one
// 12-bit-UW frame's bit vector; VDL2: one descrambled burst's bit vector
// — RS/AVLC segmentation happens in the band's L2, downstream of this).
typedef struct {
    uint8_t *bits;      // demodulated hard bits, one per byte; callback owns (free())
    int      n_bits;
    int16_t *soft_bits; // per-bit soft metric (sign = hard decision, magnitude =
                        // reliability), same length as bits, or NULL; callback owns
    int  direction;     // band-defined enum (Iridium: ir_direction_t DL/UL;
                        // VDL2: always downlink-from-receiver-POV, 0)
    bool demod_ok;      // false = diagnostic-only callback (Iridium fires the
                        // cb on failed sub-frames too; callback still owns bits)
    float snr_db;       // demod-estimated SNR (band-defined reference point)
    // Band-specific full result for diagnostics (Iridium: the
    // const burst_pipeline_result_t* — UW offset/correction/omega).
    // Valid only for the duration of the callback. May be NULL.
    const void *band_detail;
} band_frame_t;

typedef void (*band_frame_cb_t)(band_frame_t *f, void *ctx);

// Prefilter verdict. The gate booleans mirror the Iridium
// burst_prefilter gates (spectral width / active duration / in-band
// channel SNR) because the worker's continuation-rescue logic
// attributes rejects per failing gate (worker_core1.c pf_rej_hot_*);
// other bands map their own gates onto the closest semantic (or leave
// a gate permanently true if they don't implement it).
typedef struct {
    bool accept;   // final verdict (false = drop the burst before the demod)
    bool width_ok; // spectral-width gate
    bool dur_ok;   // active-duration gate
    bool snr_ok;   // in-band channel SNR gate
} band_prefilter_verdict_t;

// The per-band vtable. All members are required except prefilter
// (NULL = accept everything; the worker synthesises an all-true
// verdict).
typedef struct {
    const char *name; // matches band_profile_t.name

    // Cheap feature discriminator on the decimated window, run before
    // the expensive demod. `width_bins` is the tagger's measured
    // spectral width (<= 0 = unmeasured, must never reject). Fills *v
    // (never NULL) and returns v->accept.
    bool (*prefilter)(const int16_t *iq250, int n_complex, int width_bins,
                      band_prefilter_verdict_t *v);

    // Run the band's demod chain over the burst window; fire cb once
    // per frame (see band_frame_t for the diagnostic demod_ok=false
    // case). Returns the number of SUCCESSFULLY demodulated frames.
    int (*process_burst)(int16_t *iq250, int n_complex,
                         band_frame_cb_t cb, void *ctx);
} band_pipeline_t;
