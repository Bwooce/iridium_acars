// Unique-Word (UW) cross-correlator for Iridium burst alignment.
//
// gr-iridium's burst_downmix_impl.cc uses FFT-based correlation
// between the received 2-sps burst and the known DL/UL preamble +
// UW patterns. The correlation peak (with parabolic interpolation
// for sub-sample precision) tells us both:
//   - the UW position in the burst (timing alignment), and
//   - the burst direction (DL or UL — whichever pattern correlates
//     strongest).
//
// This is much more robust than a continuous symbol-timing loop for
// burst-mode reception: it's a one-shot estimate from data we already
// have, no per-symbol gain tuning, no convergence transient.
//
// Our implementation uses time-domain correlation (cheaper for the
// short ~12-symbol UW pattern than FFT-based, given our typical burst
// length). The reference patterns are the absolute UW symbols only
// (no preamble, no RRC pulse shape) — gr-iridium uses preamble + UW +
// RRC for higher SNR but the bare UW is enough at the SNRs Iridium
// bursts arrive at (≥15 dB).

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UW_DIR_UNKNOWN = 0,
    UW_DIR_DOWNLINK,
    UW_DIR_UPLINK,
} uw_direction_t;

typedef struct {
    int             uw_offset;       // sample index in burst where UW starts
                                     //  (in 2-sps complex units, NOT int16
                                     //   bytes). The first sample of the
                                     //   UW pattern is burst[2*uw_offset]
                                     //   in interleaved int16 IQ layout.
    float           correction;      // sub-sample fractional offset from
                                     //  parabolic interpolation of the peak;
                                     //  range (-0.5, +0.5) for valid peaks.
    uw_direction_t  direction;       // DL or UL (whichever had the higher peak)
    float           snr_estimate_db; // 10·log10(peak² / off-peak-mean²)
    float           peak_value;      // raw peak magnitude² (for debugging)
    // Complex peak value. The PHASE of this complex number is the
    // residual carrier phase relative to the UW pattern — pre-rotating
    // the burst by conj(peak / |peak|) aligns the constellation so
    // the PLL starts already locked. gr-iridium uses the same trick
    // (burst_downmix_impl.cc, `corr_result` variable).
    float           peak_re;
    float           peak_im;
    // Per-symbol residual carrier frequency in rad/sym, estimated from
    // the phase difference between the first-half and second-half UW
    // correlations. If the burst has a residual freq offset Δω, the
    // two halves' complex correlations differ by exp(j·Δω·6_sym), so
    // angle(half2/half1)/6 = Δω. The worker uses this to apply a
    // linear-phase-ramp correction across the whole burst before
    // calling qpsk_demod, giving the PLL a near-zero starting omega
    // instead of having to chase 0.5+ rad/sym within 12 UW symbols.
    float           omega_per_sym;
} uw_corr_result_t;

// Run correlation on a 2-sps interleaved int16 IQ burst. Searches
// for the UW pattern across the first `search_complex_samples`
// complex samples of the burst. Results written into *out_result.
//
// burst_2sps      : interleaved int16 IQ (2-sps), length = 2 × n_complex
// n_complex       : number of complex samples in burst_2sps
// search_complex  : how many candidate UW start positions to test
//                   (clamped to n_complex - UW_LENGTH × 2)
// out_result      : result; .direction == UW_DIR_UNKNOWN if SNR too low
void uw_correlator_find(const int16_t *burst_2sps, int n_complex,
                         int search_complex,
                         uw_corr_result_t *out_result);

#ifdef __cplusplus
}
#endif
