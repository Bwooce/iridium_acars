// Symbol timing recovery for the burst-mode QPSK demod (D10).
//
// Gardner timing error detector + PI loop filter + linear interpolator.
// Operates on 2-sps int16 IQ input and emits one float-complex sample
// per recovered symbol at the loop's estimated optimal strobe.
//
// Why Gardner (vs Mueller-Müller):
//   - Pre-decision: works from raw samples, doesn't need correct hard
//     decisions. The PLL hasn't converged at burst start; M-M's
//     decision-directed TED would feed garbage and diverge.
//   - 2-sps minimum matches our resampler output exactly.
//   - ~16-symbol acquisition fits inside Iridium's preamble before the
//     12-symbol UW check.
//
// Cost: ~22 float ops/symbol. At 50 K symbols/sec total demod
// throughput that's 1.1 Mflops/sec — well under 1% of one P4 core.

#pragma once

#include <complex.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward decl of the diagnostic trace struct; defined below.
struct sym_timing_trace;

typedef struct {
    // Loop state.
    float mu;            // fractional strobe offset ∈ [0, 1)
    float w;             // PI loop integrator output
    // Carry-over history needed for the Gardner TED. The TED needs the
    // current strobe, the half-symbol-prior midpoint, and the previous
    // strobe; we keep the previous strobe + midpoint between calls so a
    // burst can be fed in chunks.
    float complex prev_strobe;
    float complex prev_midpoint;
    int   have_history;  // 0 until first two strobes have been produced
    // Gains. PI loop filter. Caller may override after init for tuning.
    float Kp;
    float Ki;
    // Optional diagnostic trace (NULL = no trace). See sym_timing_trace_t.
    struct sym_timing_trace *trace;
} sym_timing_t;

// Initialise per-burst state. Call once before each burst (the loop
// reacquires from scratch — no state should cross burst boundaries).
void sym_timing_init(sym_timing_t *st);

// Process one block of 2-sps int16 IQ samples. Writes recovered
// symbols (at one strobe each) into out_syms. Returns the number of
// symbols written; this will be roughly n_int16/4 ± 1 depending on the
// loop's strobe-rate adjustments. n_int16 must be a multiple of 4
// (= one complex IQ pair).
//
// out_cap is the capacity of out_syms in complex samples. The caller
// is responsible for sizing it ≥ n_int16/4.
int sym_timing_process(sym_timing_t *st,
                       const int16_t *in_2sps, int n_int16,
                       float complex *out_syms, int out_cap);

// 2-sps-preserving variant: emits a timing-corrected int16 IQ stream
// at the SAME 2-sps rate as the input. Each symbol period produces
// 2 output complex samples: slot 0 = strobe (the corrected sampling
// point), slot 1 = strobe + 0.5 symbol period. The downstream
// qpsk_demod can keep its existing i*4 decimation pattern unchanged
// — slot 0 is always at the optimal strobe.
//
// This is the production-friendly integration: for already-aligned
// input the strobe drift is near zero and output ≈ input, so the
// existing host demod regression tests pass. For real-RF input with
// timing offset, the loop converges and slot 0 lands on the better
// of the two original samples (or an interpolation between).
//
// in_2sps : interleaved I/Q at 2 sps, n_int16 = 2 × n_complex int16.
// out_2sps: must be at least n_int16 in size (same length as input).
void sym_timing_correct_2sps(sym_timing_t *st,
                             const int16_t *in_2sps, int n_int16,
                             int16_t *out_2sps);

// Diagnostic trace buffer — per-symbol PI-loop state. Set via
// sym_timing_set_trace to enable; sym_timing_correct_2sps then
// fills .e/.mu/.w/.n per output symbol. Used by host tooling to
// inspect the loop's behaviour on a known burst and tune Kp/Ki.
#define SYM_TIMING_TRACE_CAP 1024
struct sym_timing_trace {
    float e [SYM_TIMING_TRACE_CAP];
    float mu[SYM_TIMING_TRACE_CAP];
    float w [SYM_TIMING_TRACE_CAP];
    int   n;     // number of valid entries
};
typedef struct sym_timing_trace sym_timing_trace_t;

void sym_timing_set_trace(sym_timing_t *st, sym_timing_trace_t *trace);

#ifdef __cplusplus
}
#endif
