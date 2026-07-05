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
    int uw_offset;                  // sample index in burst where UW starts
                                    //  (in 2-sps complex units, NOT int16
                                    //   bytes). The first sample of the
                                    //   UW pattern is burst[2*uw_offset]
                                    //   in interleaved int16 IQ layout.
    float correction;               // sub-sample fractional offset from
                                    //  parabolic interpolation of the peak;
                                    //  range (-0.5, +0.5) for valid peaks.
    uw_direction_t direction;       // DL or UL (whichever had the higher peak)
    float          snr_estimate_db; // 10·log10(peak² / off-peak-mean²)
    float          peak_value;      // raw peak magnitude² (for debugging)
    // Complex peak value. The PHASE of this complex number is the
    // residual carrier phase relative to the UW pattern — pre-rotating
    // the burst by conj(peak / |peak|) aligns the constellation so
    // the PLL starts already locked. gr-iridium uses the same trick
    // (burst_downmix_impl.cc, `corr_result` variable).
    float peak_re;
    float peak_im;
    // Per-symbol residual carrier frequency in rad/sym, estimated from
    // the phase difference between the first-half and second-half UW
    // correlations. If the burst has a residual freq offset Δω, the
    // two halves' complex correlations differ by exp(j·Δω·6_sym), so
    // angle(half2/half1)/6 = Δω. The worker uses this to apply a
    // linear-phase-ramp correction across the whole burst before
    // calling qpsk_demod, giving the PLL a near-zero starting omega
    // instead of having to chase 0.5+ rad/sym within 12 UW symbols.
    float omega_per_sym;
} uw_corr_result_t;

// Samples-per-symbol the module operates at. Hardcoded to 10 to
// match gr-iridium's burst_downmix internal rate. Worker must
// produce 10 sps input to these functions; host tests must
// synthesize bursts at 10 sps.
#define UW_SPS 10

// Run correlation on a 10-sps interleaved int16 IQ burst. Searches
// for the UW pattern across the first `search_complex_samples`
// complex samples. Results written into *out_result.
//
// burst       : interleaved int16 IQ at 10 sps × 25 ksym/s = 250 kHz
// n_complex   : number of complex samples
// search_complex : how many candidate UW start positions to test
// out_result  : result; .direction == UW_DIR_UNKNOWN if SNR too low
void uw_correlator_find(const int16_t *burst, int n_complex,
                        int               search_complex,
                        uw_corr_result_t *out_result);

// Coarse CFO estimator for use BEFORE the UW correlator search.
// Squares the first n_in complex samples (which the worker arranges
// to be preamble + UW, both BPSK), windows, FFTs, and returns the
// detected residual carrier omega in rad/sym. Returns 0 if the
// signal-to-noise of the squared FFT peak is too low (peak/mean <
// 5×) — caller leaves the burst unrotated in that case.
//
// burst_2sps : interleaved int16 IQ (2-sps)
// n_complex  : total number of complex samples available
// head_n     : how many samples of the burst head to use (typical
//              56 = 16 preamble + 12 UW × 2 sps; clamped to 24-56)
float uw_correlator_estimate_cfo(const int16_t *burst, int n_complex);

// Apply the root-raised-cosine matched filter to a 2-sps interleaved
// int16 IQ burst, in-place equivalent (in and out may be the same
// buffer). gr-iridium's burst_downmix_impl.cc applies RRC to the
// downconverted burst before sync correlation — that's the matched-
// filter property: when both the reference and burst are RRC-shaped,
// correlation gives optimal SNR.
//
// The shape parameters (β=0.4, 21 taps centred over ±5 symbols) match
// gr-iridium's d_rrc_fir at 2 sps. Output is the same length as
// input, with edge handling that zero-pads input (so the first ~10
// samples of output are slightly attenuated, well outside the UW
// search range in practice).
//
// burst_in    : input interleaved int16 IQ, length 2 × n_complex
// burst_out   : output interleaved int16 IQ (may alias burst_in)
// n_complex   : number of complex samples
void uw_correlator_apply_rrc(const int16_t *burst_in, int16_t *burst_out,
                             int n_complex);

// D13: sub-frame burst-edge detection. Ports gr-iridium's start-
// finding algorithm (burst_downmix_impl.cc, lines 841-880):
//   1. Compute |sample|² per complex sample.
//   2. Low-pass-filter the magnitude² envelope.
//   3. Find max, threshold = max × 0.28.
//   4. First sample where filtered mag² ≥ threshold = burst start.
//   5. Adjust by half_fir_size - pre_start_samples so we don't slice
//      into the leading preamble.
//
// The channelizer's start_sample_idx is at the threshold-crossing
// point on the cross-channel detector, which has frame-level
// granularity (~25 µs at 2.56 MSPS) and tends to land somewhere
// inside the burst envelope rather than at the leading edge. This
// function refines the start to within ±1 sample of the actual
// preamble onset, so the downstream UW correlator + CFO estimator
// see the preamble at burst-head positions.
//
// burst_2sps   : interleaved int16 IQ (2-sps), 2 × n_complex bytes
// n_complex    : total complex samples in burst
// search_max   : maximum samples to search (clamped to n_complex);
//                pass ~3× max-expected-preamble-position for safety
// Returns the burst-start sample offset (in complex samples). Zero
// if no clear envelope rise found (caller leaves burst unshifted).
int uw_correlator_find_burst_start(const int16_t *burst,
                                   int n_complex, int search_max);

// Pre-allocate the PIE float-FFT scratch in DRAM. MUST be called from the
// boot-time early-alloc dance (before the USB/DSP init that fragments
// internal SRAM). If left to the worker's lazy first call, the 16 KB
// scratch can spill into RTCRAM where the PIE vector unit silently
// mis-decodes (project_heap_position_decode_bug). Idempotent, no-op after
// the first successful call.
void uw_correlator_prealloc_pie_fft(void);

// Pre-allocate/pin the three PIE FIR delay lines used by this module
// (D13 envelope-LP FIR + RRC matched-filter I/Q FIRs) in DRAM. MUST be
// called from the same boot-time early-alloc dance as
// uw_correlator_prealloc_pie_fft(), before the USB/DSP init that
// fragments internal SRAM. Left to each FIR's lazy first-call init,
// the delay line (allocated internally by dsps_fird_init_s16 via
// memalign) can land in RTCRAM under DRAM pressure, where the PIE
// vld.128 instructions silently mis-decode
// (project_heap_position_decode_bug). Idempotent, no-op after the
// first successful call. Host build: not defined (device-only PIE FIR
// path), matching uw_correlator_prealloc_pie_fft — callers on host
// never need it.
#if defined(ESP_PLATFORM)
void uw_correlator_prealloc_fir(void);
#endif

#ifdef __cplusplus
}
#endif
