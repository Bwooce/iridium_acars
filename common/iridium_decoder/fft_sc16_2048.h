// fft_sc16_2048.h — portable Q15 (sc16) 2048-pt FFT.
//
// Companion to fft_sc16_64. Used by the wideband fft_burst_tagger
// (Phase 3.6.M) which runs a 2048-pt FFT every 2048 samples (= every
// 0.82 ms at 2.5 MSPS). At N=2048 the Iridium burst-detection bin
// width is 1.22 kHz — fine enough to distinguish adjacent Iridium
// channels (41.67 kHz spacing) by ~34 bins.
//
// Algorithm: radix-2 DIT with bit-reversed twiddle table — bit-for-
// bit equivalent to esp-dsp's dsps_fft2r_sc16_ansi (which is the C
// reference for the PIE-accelerated dsps_fft2r_sc16_arp4 we'll call
// on ESP32-P4). Same code path works on host.
//
// Per-stage right-shift by 1 in the butterfly normalises by 1/N. For
// N=2048 that's an 11-bit shrinkage of the output magnitude — well
// within the dynamic range needed for the magnitude² + noise-floor
// EMA in the burst tagger (input peak ±32k → output peak ~ ±32k·
// sqrt(2048)/2048 ≈ ±700, magnitude² fits in int32 trivially).
//
// Input/output: in-place, N complex int16 samples as interleaved
// I, Q, I, Q, ... (length 2*N int16). After the call, output is in
// natural frequency order (bin 0 = DC, bin N/2 = Nyquist).
//
// Memory: 4 KB static twiddle table (N int16). One-shot init.

#pragma once
#include <stdint.h>

#define FFT_SC16_2048_N 2048

// One-shot init. Idempotent; safe to call before each use.
void fft_sc16_2048_init(void);

// In-place 2048-pt FFT on interleaved IQ. `data` must point to at
// least 4096 int16 values (2048 complex). Output in natural order.
void fft_sc16_2048(int16_t *data);
