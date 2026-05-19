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

#define DIDECIM_DECIM        10        // 10× decim (2.5M → 250k)
#define DIDECIM_NTAPS        279       // matches gri's 40 dB Kaiser

typedef struct {
    int16_t taps[DIDECIM_NTAPS];       // Q15 taps, DC-gain-normalised
} direct_if_decim_t;

// Init the decimator. Builds the Kaiser FIR taps internally
// (β=5.5, cutoff=20 kHz, transition 20-40 kHz at fs=2.5 MSPS).
// Idempotent.
void direct_if_decim_init(direct_if_decim_t *d);

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
