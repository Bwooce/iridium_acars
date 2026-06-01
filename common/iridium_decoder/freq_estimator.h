// Fine carrier frequency offset estimator for Iridium burst preamble.
// D8 — the companion to D7 (polyphase channelizer). The channelizer
// reports burst frequency at channel granularity (M=64, fs=2.56 MHz
// → 40 kHz spacing), so residual offset within a channel can be up to
// ±20 kHz. The PLL in qpsk_demod has a capture range of ~±1.25 kHz, so
// without this refinement the worker would feed the demod a burst
// whose carrier is 16× outside the loop's lock range.
//
// Algorithm: 256-point complex FFT over the first N raw IQ samples of
// the extracted burst (before any frequency centring), find the bin
// with peak magnitude inside a ±20 kHz search window, apply quadratic
// peak interpolation across the three bins around the peak to refine
// to sub-bin precision. Output: signed offset from DC in Hz.
//
// CPU budget on P4: a single 256-pt FFT is ~10-50 µs scalar; the
// surrounding magnitude/peak-find work is trivial. Runs once per
// burst on Core 1, well inside the worker's existing time slice.

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Estimator FFT size. 512 at fs=2.56 MHz gives 5 kHz / bin —
// the channelizer's ±20 kHz search window maps to ±4 bins. With
// a Hann window the parabolic peak-interpolation model gives
// roughly ±500 Hz precision at typical Iridium SNRs (the 1/2 bin
// width is the rule-of-thumb worst-case error for windowed FFTs).
// We started at 256 but the interpolation bias reached 1.2 kHz —
// 512 halves the bin width and brings worst-case error to ~600 Hz.
#define FREQ_EST_FFT_N 512

// Run the estimator on n_complex int16 IQ samples (interleaved
// I,Q,I,Q,... so the caller passes 2*n_complex int16_t values).
//
//   iq         : interleaved int16 IQ, 16-byte aligned recommended.
//                Must contain at least FREQ_EST_FFT_N complex samples.
//   n_complex  : number of complex samples available; only the first
//                FREQ_EST_FFT_N are used. Must be ≥ FREQ_EST_FFT_N.
//   fs_hz      : input sample rate (e.g. 2_560_000).
//   search_hz  : half-bandwidth of the search window in Hz. Bins
//                whose centre frequency falls outside ±search_hz
//                are ignored. Typically set to half the channelizer
//                channel spacing (= 20_000 for M=64 at 2.56 MHz).
//
// Returns the estimated carrier offset from DC in Hz (signed; positive
// = above DC). Returns 0 if the peak is at DC (artefact guard) or if
// n_complex is below FREQ_EST_FFT_N.
int32_t freq_estimator_run(const int16_t *iq, size_t n_complex,
                           uint32_t fs_hz, uint32_t search_hz);

#ifdef __cplusplus
}
#endif
