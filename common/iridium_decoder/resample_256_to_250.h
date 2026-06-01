// resample_256_to_250.h — Q15 rational resampler for the wideband
// burst-tagger input path. Converts 2.56 MSPS (RTL-SDR / our fixture
// rate) → 2.5 MSPS (gr-iridium's wideband-tagger rate via the python
// `resample_poly(cf, up=125, down=128)` in iridium_extractor_flowgraph.py).
//
// Why this exists: the test_pipeline_wideband_albq smoke implementation
// used linear interpolation here, which costs ~3 dB of amplitude on
// the bursts (the linear-interp's frequency response rolls off well
// below Nyquist). Using a proper polyphase rational resampler with
// gri-aligned coefficients gets us within ~0.1 dB of scipy's output.
//
// Implementation: firmr_s16 (the polyphase rational-rate FIR we already
// use elsewhere) with interp=125, decim=128. Filter is a 125-phase
// Kaiser-windowed sinc designed to match scipy's resample_poly default
// (β=5.0, cutoff at 1/max(up,down) × π in the virtual interp×fs frame,
// 257-tap prototype). Complex I/Q handled by two independent firmr_s16
// instances (gri's path is complex, but the math is separable — real
// and imag pass through the same real-coefficient filter).
//
// Memory: ~5 KB for the 1125-tap coefficient table (delay_size × interp
// = 9 × 125), plus 36 bytes of per-instance delay line × 2 channels.

#pragma once
#include <stdint.h>
#include "firmr_s16.h"

#define RS25_INTERP 125
#define RS25_DECIM 128
#define RS25_DELAY_SIZE 9

// Caller-allocated batch_scratch capacity for _process_explicit.
// 8 complex pairs = 32 bytes = half a cache line — enough to
// amortise PSRAM write latency, small enough for any caller stack.
#define RS25_BATCH_COMPLEX 8

typedef struct {
    int16_t coeffs[RS25_DELAY_SIZE * RS25_INTERP]; // legacy layout (used by firmr_s16 host comparison)
    // Circular delay buffers with double-mirror layout (task #58).
    // The 16-slot logical ring is stored as 32 int16 — slots
    // [0..15] are the live ring, slots [16..31] mirror the same
    // data. wpos walks 0..15; each input writes the new sample at
    // BOTH delay[wpos] and delay[wpos+16]. The MAC reads 16
    // contiguous int16 starting at &delay[wpos] — first 9 are real
    // taps, the rest are mirror data that gets multiplied by the
    // coefficient zero-pad (slots [9..15] of every phase row) so
    // it contributes 0 to the sum.
    //
    // This eliminates the per-input 9-element shift (was 18 stores
    // per input, now 4) — see resample_256_to_250.c for the cycle
    // accounting that justifies it.
    int16_t     delay_i[32] __attribute__((aligned(16)));
    int16_t     delay_q[32] __attribute__((aligned(16)));
    int         start_pos; // phase counter (shared by I/Q)
    int         wpos;      // circular write index in [0..15]
    firmr_s16_t fir_i;     // retained for host comparison build only
    firmr_s16_t fir_q;
} resample_256_to_250_t;

// Early one-time allocation of the polyphase-coefficient singleton
// (s_coeffs_pp, 4 KB internal SRAM). Call as the FIRST internal-SRAM
// consumer in boot to guarantee deterministic placement at the
// top-of-RAM address (~0x4ff7e300 on P4), which is required for
// the PIE asm to produce correct outputs.
// See memory note project_heap_position_decode_bug.md.
void resample_256_to_250_alloc_coeffs(void);

// Init the resampler. Generates the 125-phase Kaiser FIR taps
// internally. Idempotent (safe to call multiple times — re-resets
// state).
void resample_256_to_250_init(resample_256_to_250_t *r);

// Process `n_in_complex` complex int16 IQ samples; write outputs to
// `out` (also complex int16). Returns the number of complex output
// samples written.
//
// Output length ≈ n_in × INTERP / DECIM = n_in × 125 / 128.
// Caller must size `out` for at least n_in (a generous upper bound;
// trim using the returned count).
int resample_256_to_250_process(resample_256_to_250_t *r,
                                const int16_t *in_iq, int n_in_complex,
                                int16_t *out_iq);

// Caller-managed-state variant of _process. The polyphase delay
// line uses the double-mirror layout described in
// resample_256_to_250_t — caller MUST provide delay_i[32] and
// delay_q[32] (not [16]) and a wpos write index in [0..15].
// `start_pos_io` is the phase counter as before. Used by the
// split-ingest worker pool so two workers can run concurrently on
// different output slices of the same chunk. Returns number of
// complex outputs written, bounded by max_out.
//
// `batch_scratch` (optional, NULL = direct writes): caller-owned
// 16-int16 (8 complex) buffer that MUST be aligned(16). If
// non-NULL and points to internal SRAM, the PIE MAC writes each
// emit into this scratch and the function memcpy-flushes 32-byte
// chunks to `out_iq` (in PSRAM). This coalesces what would
// otherwise be ~8000 scattered uncached 2-byte PSRAM stores into
// ~1000 32-byte bursts. Pass NULL if the caller stack lives in
// PSRAM (worker tasks) — batching to PSRAM scratch is a wash.
// Measured PSRAM-write cost on the inline ingest_task path
// (internal-SRAM stack) was ~525 µs of the ~3.7 ms resample cost
// (2026-05-24 profile).
int resample_256_to_250_process_explicit(int16_t *delay_i, int16_t *delay_q,
                                         int           *wpos_io,
                                         int           *start_pos_io,
                                         const int16_t *in_iq, int n_in_complex,
                                         int16_t *out_iq, int max_out,
                                         int16_t *batch_scratch);

// "Advance only" — process inputs but do NOT emit outputs. Used to
// pre-position Worker B's (delay_line, wpos, start_pos) to the
// chunk's midpoint before it starts its MAC slice. Same state
// evolution as _process_explicit, just no per-output MAC work.
void resample_256_to_250_advance(int16_t *delay_i, int16_t *delay_q,
                                 int           *wpos_io,
                                 int           *start_pos_io,
                                 const int16_t *in_iq, int n_in_complex);
