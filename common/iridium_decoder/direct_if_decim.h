// direct_if_decim.h — per-burst direct-IF decimator for the wideband
// burst-tagger path (Phase 3.6.M step 3).
//
// What it does: 10× Q15 decimation from input_sample_rate (2.5 MSPS)
// to burst_sample_rate (250 ksps), on complex IQ. Used after the
// caller rotates the burst to DC via q15_freq_shift_inplace
// (burst_pipeline.c) — the two operations are kept independent for
// trivial unit-testing.
//
// FIR design: matches gr-iridium burst_downmix exactly —
//   firdes.low_pass_2(gain=1, fs=2.5e6, cutoff=20kHz,
//                     transition_width=40kHz, attenuation_dB=40)
// → Kaiser β ≈ 3.4, 141 taps (zero-padded to 144 here so the P4 PIE
// FIR's coeffs_len-%-8 requirement is met). The decim factor (10) is
// fixed by the host fixture rate and our internal 250 ksps target.
//
// Implementation: direct FIR + downsample. Integer decim has no
// multiply-savings opportunity from polyphase decomposition; the
// only benefit would be memory-access patterns and we don't need
// that for batch per-burst processing. ~141 MACs per output sample
// × 4000 outputs per burst (16 ms × 250 ksps) ≈ 560k MACs/burst.
// Runs in microseconds on host; budget on P4 measured in step 4.

#pragma once
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "dsps_fir.h"     // fir_s16_t — esp-dsp PIE FIR state
#endif

#define DIDECIM_DECIM        10        // 10× decim (2.5M → 250k)
// 144 taps = 141 from gri's firdes.low_pass_2 design + 3 zero taps for
// alignment.  Gri calls
//   firdes.low_pass_2(gain=1, fs=2.5e6, cutoff=20e3,
//                     trans_width=40e3, atten=40 dB)
// which is the Kaiser-window design from compute_ntaps_windowed_filter():
//   ntaps = round((40-7.95) / (2.285 * 2π * 40e3/2.5e6)) + 1 = 141
// The earlier 279-tap design here used trans_width=20e3 (half the gri
// value), which gave a sharper roll-off and a measurably different
// broadband shape vs gri's filtered_deci dump (NMSE ≈ -16 dB on burst
// 390 even after time-alignment). Matching gri's actual transition
// width is what brings the post-resample stage into bit-level agreement.
// dsps_fird_s16_arp4 (P4 PIE) requires coeffs_len divisible by 8; we
// pad 141 → 144 with three zero taps. Zero padding preserves the
// frequency response exactly.
#define DIDECIM_NTAPS        144       // gri firdes.low_pass_2 (141) + 3 zero

// Per-channel FIR state for the split (deinterleaved) path on host.
// Mirrors the inner-loop semantics of esp-dsp's fir_s16_t / dsps_fird
// (streaming circular delay line) so the host and target produce the
// same numerical output to within Q15 rounding.
typedef struct {
    int16_t delay[DIDECIM_NTAPS + 16]; // +16 cache-line headroom
    int16_t pos;                       // write index in delay[]
    int16_t d_pos;                     // decimation counter (0..DECIM-1)
} didecim_fir_state_t;

typedef struct {
    int16_t taps[DIDECIM_NTAPS] __attribute__((aligned(16)));
                                       // Q15 taps, DC-gain-normalised.
                                       // 16-byte aligned for the PIE
                                       // path's coeffs pointer.
    didecim_fir_state_t fir_i;         // host I-channel FIR state
    didecim_fir_state_t fir_q;         // host Q-channel FIR state
#ifdef ESP_PLATFORM
    // Target PIE state. dsps_fird_init_s16 (under arp4) ignores any
    // delay buffer we pass and allocates its own internal one — these
    // structs hold the bookkeeping. Initialised in direct_if_decim_init.
    fir_s16_t fir_dsp_i;
    fir_s16_t fir_dsp_q;
    int       fir_dsp_inited;
#endif
} direct_if_decim_t;

// Init the decimator. Builds the Kaiser FIR taps internally
// (β=5.5, cutoff=20 kHz, transition 20-40 kHz at fs=2.5 MSPS).
// Idempotent. Clears the streaming FIR state too.
void direct_if_decim_init(direct_if_decim_t *d);

// Reset the streaming delay-line state for both I and Q FIRs used
// by direct_if_decim_process_split. Call between unrelated bursts
// to avoid history bleed. _init() also calls this.
void direct_if_decim_reset_state(direct_if_decim_t *d);

// Process `n_in` complex int16 IQ input samples (interleaved I,Q),
// write up to `n_in / DIDECIM_DECIM` complex int16 IQ output samples
// to `out`. Returns the number of complex output samples produced.
//
// FIR transient: the first (NTAPS - 1) input samples are partially-
// filtered with zero-padded history (i.e. they see less than NTAPS
// of valid input). Caller can either accept this transient or
// pre-pad the input with at least (NTAPS - 1) samples of context
// before the burst proper.
//
// `n_in` must be ≥ DIDECIM_DECIM (else zero outputs).
int direct_if_decim_process(const direct_if_decim_t *d,
                             const int16_t *input, int n_in,
                             int16_t *out);

// Split (deinterleaved) variant: separates input IQ into two real
// streams, runs a real-FIR + decim on each, then re-interleaves.
// Equivalent to direct_if_decim_process in output shape (one complex
// sample per DIDECIM_DECIM input complex samples) but uses a
// streaming delay-line FIR instead of the centred-zero-pad scalar.
// The numerical output differs slightly because of the alignment
// shift, but burst content survives unchanged through burst_pipeline
// (validated on the ALBQ wideband test).
//
// Caller-provided scratch:
//   scratch_in_i, scratch_in_q  — each holds at least n_in int16
//   scratch_out_i, scratch_out_q — each holds at least (n_in / DIDECIM_DECIM) int16
//
// The reason this exists: the inner real-FIR step on target becomes
// a single call to dsps_fird_s16_arp4 (PIE-accelerated), giving
// ~6× speedup vs the scalar centred-tap implementation. The split-
// then-real-FIR pattern is what makes the PIE acceleration apply —
// the asm only knows real FIR, not complex. Same I/O contract on
// host (portable C inner FIR) and target (PIE asm inner FIR).
//
// Modifies the persistent FIR state inside `d` (delay lines). Calls
// must be sequential per direct_if_decim_t instance.
int direct_if_decim_process_split(direct_if_decim_t *d,
                                   const int16_t *input, int n_in,
                                   int16_t *out,
                                   int16_t *scratch_in_i, int16_t *scratch_in_q,
                                   int16_t *scratch_out_i, int16_t *scratch_out_q);
