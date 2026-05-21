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

#define RS25_INTERP        125
#define RS25_DECIM         128
#define RS25_DELAY_SIZE    9

typedef struct {
    int16_t     coeffs[RS25_DELAY_SIZE * RS25_INTERP];   // legacy layout (used by firmr_s16 host comparison)
    // Linear delay buffers (newest sample at index 0; manually shifted
    // down on each input). 16 int16 wide so the active 9 are flanked
    // by zero-padded tail slots for future PIE 128-bit loads.
    int16_t     delay_i[16] __attribute__((aligned(16)));
    int16_t     delay_q[16] __attribute__((aligned(16)));
    int         start_pos;     // phase counter (shared by I/Q)
    firmr_s16_t fir_i;         // retained for host comparison build only
    firmr_s16_t fir_q;
} resample_256_to_250_t;

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
