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
//   firdes.low_pass_2(1, 2.5e6, burst_width/2=20kHz,
//                     burst_width=40kHz, 40dB)
// → Kaiser β≈5.5, 279 taps. The decim factor (10) is fixed by the
// host fixture rate and our internal 250 ksps target.
//
// Implementation: direct FIR + downsample. Integer decim has no
// multiply-savings opportunity from polyphase decomposition; the
// only benefit would be memory-access patterns and we don't need
// that for batch per-burst processing. ~279 MACs per output sample
// × 4000 outputs per burst (16 ms × 250 ksps) ≈ 1.1M MACs/burst.
// Runs in microseconds on host; budget on P4 measured in step 4.

#pragma once
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "dsps_fir.h"     // fir_s16_t — esp-dsp PIE FIR state
#endif

#define DIDECIM_DECIM        10        // 10× decim (2.5M → 250k)
#define DIDECIM_NTAPS        279       // matches gri's 40 dB Kaiser

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
