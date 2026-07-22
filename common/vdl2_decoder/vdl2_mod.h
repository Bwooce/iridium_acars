// vdl2_mod — synthetic VDL Mode 2 D8PSK burst modulator, for host test
// fixtures ONLY (the ida_encode.c precedent: deterministic known-bits
// waveforms drive the production demod in round-trip tests; nothing on
// the device transmits).
//
// Burst structure modulated (ICAO Annex 10 Vol III / cross-checked
// against what dumpvdl2's receiver expects — see vdl2_demod.h for the
// citation trail):
//   [pad_pre noise-only samples]
//   [ramp_syms transmitter ramp-up symbols, linearly rising amplitude,
//    constant phase — dumpvdl2 ignores these]
//   [16-symbol synchronisation sequence: absolute phases =
//    vdl2_preamble_phase[] + phase0]
//   [data symbols: 25-bit header (3 reserved zeros + 17-bit
//    bit-reversed transmission length + 5 FEC bits, vdl2_hdr_encode)
//    followed by the caller's body bits, all scrambled by the x^15+x+1
//    LFSR from IV 0x6959, mapped 3 bits/symbol MSB-first through the
//    INVERSE Gray map, each symbol advancing the carrier phase by
//    idx * pi/4]
//   [pad_post noise-only samples]
//
// Pulse shaping: FULL raised-cosine, rolloff alpha = 0.6 (ICAO Annex
// 10 Vol III puts the whole RC at the transmitter — Nyquist on its
// own, so the receiver's flat-passband channel filter preserves
// zero-ISI strobes). A root/root split was tried and empirically
// rejected against the real golden capture (see rc_pulse in
// vdl2_mod.c and the resampler note in vdl2_demod.c). The
// waveform is synthesised at an arbitrary output rate by direct
// pulse-superposition at fractional symbol positions, so timing offset
// is continuous-valued, then rotated by a carrier offset and dressed
// with AWGN. The synthesis path shares NO DSP code with the demod
// (independent scrambler implementation included), so a round-trip
// bit-identity check exercises both directions of every protocol
// constant.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double   fs_hz;       // output sample rate (250000 for the pipeline path)
    double   cfo_hz;      // carrier frequency offset to impose
    double   timing_frac; // symbol-timing offset, fraction of T in [0,1)
    double   amp;         // symbol-strobe amplitude, int16 LSB
    double   awgn_sigma;  // AWGN sigma per I/Q component, int16 LSB (0 = none)
    double   phase0_rad;  // carrier phase of preamble symbol 0
    double   rolloff;     // RC alpha
    int      ramp_syms;   // ramp-up symbols before the preamble
    int      pad_pre;     // noise-only samples before the ramp
    int      pad_post;    // noise-only samples after the last symbol tail
    uint32_t seed;        // PRNG seed (noise); same seed = same waveform
} vdl2_mod_params_t;

// fs 250000, cfo 0, timing 0, amp 8000, sigma 0, phase0 0.4, alpha 0.6,
// ramp 5, pads 400/400, seed 1.
void vdl2_mod_params_default(vdl2_mod_params_t *p);

// Modulate one burst. datalen_bits = the header transmission-length
// field; body_bits = the vdl2_burst_body_bits(datalen_bits) bits that
// follow the header on air (data octets + RS FEC octets, unscrambled —
// the modulator scrambles). Writes interleaved int16 IQ; returns the
// complex sample count, or -1 on bad args / capacity overflow.
int vdl2_mod_burst(uint32_t datalen_bits, const uint8_t *body_bits,
                   const vdl2_mod_params_t *p,
                   int16_t *out_iq, int max_complex);

#ifdef __cplusplus
}
#endif
