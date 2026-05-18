// firmr_s16.h — portable polyphase rational-rate FIR resampler.
//
// This is a byte-for-byte port of esp-dsp's dsps_firmr_s16_ansi.c
// (modules/fir/fixed/) so the host test runs the same resampler math
// as the P4 firmware. Previously the host had its own portable
// polyphase implementation with a non-standard coefficient layout
// (contiguous 64-tap blocks per phase rather than the transposed
// "coeffs[tap_pos * interp + phase]" layout esp-dsp uses) — produced
// ~10× fewer decodes on the same data because each phase saw a
// different slice of the sinc filter.
//
// The P4 firmware continues to call esp-dsp's `dsps_firmr_s16`
// directly (the AES3 / ARP4 vector kernels are faster than the ANSI
// path); this module is the host-side equivalent of the same ANSI
// reference. The two must produce bit-identical output on identical
// input + coefficients.
//
// Coefficient layout (matches esp-dsp):
//   coeffs[tap_pos * interp + phase]
//   - coeffs_len = number of stored coefficients = delay_size * interp
//   - delay_size = number of delay-line taps per phase
//   - For each output, phase varies 0..interp-1 in steps of decim;
//     across phases, tap_pos sweeps 0..delay_size-1 against the
//     delay line which is treated as a circular buffer.

#pragma once
#include <stdint.h>

typedef struct {
    int16_t  *coeffs;           // length = delay_size * interp
    int16_t  *delay;            // circular buffer, length = delay_size
    int16_t   delay_size;       // taps per phase
    int16_t   interp;           // interpolation factor
    int16_t   decim;            // decimation factor
    int16_t   shift;            // post-MAC shift (output = (acc >> (15 - shift)))
    int16_t   rounding_val;     // rounding term (default 0x7fff)
    // Mutable state — updated per call:
    int16_t   pos;              // current delay-line write position (counts DOWN)
    int16_t   start_pos;        // current phase index within an INTERP window
} firmr_s16_t;

// Initialise. coeffs must have length = delay_size * interp.
// `delay` is a caller-owned buffer of length `delay_size`.
// `start_pos` is the starting phase index (0 for "fresh stream").
// `shift` matches esp-dsp's `shift` parameter.
void firmr_s16_init(firmr_s16_t *fir,
                    int16_t *coeffs, int16_t *delay,
                    int16_t delay_size, int16_t interp, int16_t decim,
                    int16_t start_pos, int16_t shift);

// Process `input_len` samples; write outputs to `output`.
// Returns the number of output samples produced. Caller must size
// `output` to at least input_len * interp / decim + some margin
// (the phase walk produces between floor(input_len*interp/decim)
// and ceil(input_len*interp/decim) outputs depending on initial
// phase).
int32_t firmr_s16_process(firmr_s16_t *fir,
                          const int16_t *input, int16_t *output,
                          int32_t input_len);
