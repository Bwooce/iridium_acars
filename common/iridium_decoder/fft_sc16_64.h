// fft_sc16_64.h — portable Q15 (sc16) 64-pt FFT shared between host
// and target.
//
// Algorithm: radix-2 DIT (decimation-in-time) with bit-reversed
// twiddle table — bit-for-bit equivalent to esp-dsp's
// dsps_fft2r_sc16_ansi (which is the C reference for the hardware-
// accelerated dsps_fft2r_sc16_arp4 we call on ESP32-P4).
//
// Per-stage right-shift by 1 inside the butterfly normalises the
// final output by 1/N (matches the host's float-roundtrip path that
// explicitly divides by M after the FFT).
//
// Input/output: in-place, 64 complex int16 samples as interleaved
// I, Q, I, Q, ... (length 128 int16). After the call, output is in
// natural frequency order (bit-rev done internally).
//
// Initialisation is one-shot and idempotent; safe to call multiple
// times. Twiddle table lives in module-static memory (256 bytes).

#pragma once
#include <stdint.h>

#define FFT_SC16_64_N 64

// One-shot init. Idempotent; safe to call before each use if you
// don't want to track init state in the caller.
void fft_sc16_64_init(void);

// In-place 64-pt FFT on interleaved IQ. `data` must point to at
// least 128 int16 values (64 complex). Output is in natural order
// (bin 0 = DC, bin 32 = Nyquist).
void fft_sc16_64(int16_t *data);
