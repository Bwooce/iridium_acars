// See sym_timing.h.
//
// Implementation notes:
//
//   - The input is 2 sps int16 IQ (interleaved I, Q, I, Q, ...) at the
//     resampler output. We treat each complex pair as one input sample.
//   - The strobe is the loop's estimate of where the symbol's peak
//     energy is. Output one float complex per strobe.
//   - Gardner TED at strobe k: e = Re{conj(y_mid) × (y_curr - y_prev)}
//     where y_prev / y_curr are consecutive strobes and y_mid is the
//     half-symbol-between-them sample. At 2 sps the spacings line up
//     directly with input samples (modulo fractional adjustment).
//   - PI loop filter:
//       w[k+1] = w[k] + Ki · e[k]
//       v[k]   = Kp · e[k] + w[k]
//     v controls how much the strobe phase advances. mu accumulates v
//     modulo 1; when mu wraps, the strobe advances by one input sample.
//   - Linear interpolation produces the output sample at fractional
//     position mu between in[i] and in[i+1].
//
// PI gains: from Rice "DSP for Communications" Ch. 8 for normalised
// loop bandwidth Bn·T = 0.01, ζ = 0.707, Gardner Kd = 0.5 (QPSK at
// unit amplitude):
//   Kp ≈ 4·ζ·ωn·T / Kd ≈ 0.055
//   Ki ≈ (ωn·T)² / Kd  ≈ 0.00019
// Acquisition is within ~16 symbols of arbitrary phase — fits inside
// the Iridium preamble before the 12-symbol UW.

#include "sym_timing.h"

#include <stdbool.h>
#include <string.h>

// Textbook PI gains for Bn·T = 0.01, ζ = 0.707, Gardner Kd = 0.5.
// See sym_timing.h header for derivation. The host unit test passes
// with these; real-RF integration needs careful tuning against
// representative bursts (the simple drop-in replacement of fixed
// decimation breaks pre-aligned host fixtures due to per-symbol
// strobe jitter without compensating averaging).
// Restored textbook gains for diagnostic / tuning runs. Production
// qpsk_demod overrides to 0/0 below if it must (the wrapper there
// sets Kp/Ki explicitly after init).
#define DEFAULT_KP   0.055f
#define DEFAULT_KI   0.00019f

void sym_timing_init(sym_timing_t *st)
{
    memset(st, 0, sizeof(*st));
    st->Kp = DEFAULT_KP;
    st->Ki = DEFAULT_KI;
    // mu starts at 0 (= aligned to even 2-sps sample). If the input
    // happens to already be on the better phase, convergence is
    // near-instantaneous; otherwise the loop crosses over within ~8
    // symbols.
}

void sym_timing_set_trace(sym_timing_t *st, sym_timing_trace_t *trace)
{
    if (!st) return;
    st->trace = trace;
    if (trace) trace->n = 0;
}

// Linear interpolation between consecutive complex samples in the
// int16 IQ stream. idx is the integer sample index of the left side;
// frac ∈ [0, 1) is the fractional position to the right.
static inline float complex interp_lin(const int16_t *iq, int idx, float frac)
{
    float l_re = (float)iq[2 * idx + 0];
    float l_im = (float)iq[2 * idx + 1];
    float r_re = (float)iq[2 * (idx + 1) + 0];
    float r_im = (float)iq[2 * (idx + 1) + 1];
    float re = l_re * (1.0f - frac) + r_re * frac;
    float im = l_im * (1.0f - frac) + r_im * frac;
    return re + im * I;
}

int sym_timing_process(sym_timing_t *st,
                       const int16_t *in_2sps, int n_int16,
                       float complex *out_syms, int out_cap)
{
    if (!st || !in_2sps || !out_syms) return 0;
    int n_complex = n_int16 / 2;
    if (n_complex < 4) return 0;       // need room for 2 strobes + 1 mid

    // Loop produces one strobe per ~2 input complex samples (one
    // symbol). We advance through the input by `step_int + step_frac`
    // each strobe, where the integer part is ~2 and the fractional
    // part is mu_drift from the PI filter.
    int   strobe_idx = 0;
    float mu = st->mu;
    float w  = st->w;
    int   n_out = 0;
    float complex prev_strobe = st->prev_strobe;
    float complex prev_mid    = st->prev_midpoint;
    bool  have_history        = (st->have_history != 0);

    while (n_out < out_cap) {
        // Need samples at strobe_idx (current) and strobe_idx+1
        // (the right side of the next strobe's interpolation pair).
        // We also need the midpoint, which is at strobe_idx + 0.5
        // PLUS mu — falls between strobe_idx and strobe_idx+1.
        // Stop if we'd read past the end of the input.
        if (strobe_idx + 1 >= n_complex) break;

        // Current strobe sample (at integer strobe_idx + fractional mu).
        float complex y_curr;
        if (mu <= 0.0f) {
            y_curr = (float)in_2sps[2 * strobe_idx + 0]
                   + (float)in_2sps[2 * strobe_idx + 1] * I;
        } else {
            y_curr = interp_lin(in_2sps, strobe_idx, mu);
        }
        // Midpoint sample (half a symbol earlier — at strobe_idx − 0.5
        // + mu, i.e. between strobe_idx-1 and strobe_idx). At 2 sps a
        // symbol is 2 input samples wide, so the midpoint of the
        // previous symbol period is 1 input sample to the left of the
        // current strobe.
        float complex y_mid;
        if (strobe_idx >= 1) {
            // strobe_idx - 1 is the half-symbol-prior position.
            // Interpolate at (strobe_idx - 1) + mu.
            y_mid = (mu <= 0.0f)
                  ? ((float)in_2sps[2 * (strobe_idx - 1) + 0]
                     + (float)in_2sps[2 * (strobe_idx - 1) + 1] * I)
                  : interp_lin(in_2sps, strobe_idx - 1, mu);
        } else {
            // First strobe of the buffer: no prior sample to use.
            // Fall back to a "no TED update" pass — emit the strobe
            // and let the loop catch up on subsequent ones.
            y_mid = 0.0f + 0.0f * I;
        }

        // Output the current strobe.
        out_syms[n_out++] = y_curr;

        // Gardner TED: needs prev_strobe, y_mid, y_curr.
        // e = Re{ conj(y_mid) × (y_curr − prev_strobe) }
        float e = 0.0f;
        if (have_history) {
            float complex diff = y_curr - prev_strobe;
            // Re{ conj(m) * d } = Re(m)·Re(d) + Im(m)·Im(d)
            e = crealf(y_mid) * crealf(diff)
              + cimagf(y_mid) * cimagf(diff);
            // Normalise by approximate symbol amplitude so loop gain
            // is independent of input level. y_curr magnitude is in
            // int16 scale (~10^3-10^4); divide e by something on that
            // order of magnitude squared to bring it into [-1, 1].
            // 1e7 is a rough match for int16 IQ at ~Iridium burst
            // levels (~5000 magnitude). Adjusted gain values are
            // calibrated against this normalisation.
            e *= 1.0f / 1.0e7f;
        }

        // PI loop filter.
        w += st->Ki * e;
        float v = st->Kp * e + w;

        // Advance strobe by (2 + v) input samples per symbol. Split
        // into integer step + fractional carry kept in mu ∈ [0, 1).
        float advance = 2.0f + v;
        mu += advance;
        int int_step = (int)mu;       // truncates toward 0
        mu -= (float)int_step;
        if (mu < 0.0f) { mu += 1.0f; int_step--; }
        strobe_idx += int_step;
        if (strobe_idx + 1 >= n_complex) break;

        prev_strobe   = y_curr;
        prev_mid      = y_mid;
        have_history  = true;
    }

    st->mu             = mu;
    st->w              = w;
    st->prev_strobe    = prev_strobe;
    st->prev_midpoint  = prev_mid;
    st->have_history   = have_history ? 1 : 0;
    return n_out;
}

// Saturation helper for the int16 output.
static inline int16_t sat_int16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

void sym_timing_correct_2sps(sym_timing_t *st,
                             const int16_t *in_2sps, int n_int16,
                             int16_t *out_2sps)
{
    if (!st || !in_2sps || !out_2sps) return;
    int n_complex = n_int16 / 2;
    if (n_complex < 4) {
        // Not enough samples for a full symbol; pass through.
        memcpy(out_2sps, in_2sps, n_int16 * sizeof(int16_t));
        return;
    }

    int   strobe_idx = 0;       // integer complex sample index
    float mu         = st->mu;
    float w          = st->w;
    float complex prev_strobe = st->prev_strobe;
    float complex prev_slot1  = st->prev_midpoint;  // y_mid for current TED
    bool  have_history        = (st->have_history != 0);

    int out_complex = 0;
    int max_out_complex = n_int16 / 2;

    // The output is 2 complex samples per symbol period. The input
    // advances by ~2 complex samples per symbol (corrected by v).
    while (strobe_idx + 2 < n_complex && out_complex + 1 < max_out_complex) {
        // Slot 0 of output = strobe at (strobe_idx + mu).
        float complex slot0 = interp_lin(in_2sps, strobe_idx, mu);
        // Slot 1 = half-symbol AFTER this strobe = (strobe_idx + 1 + mu).
        float complex slot1 = interp_lin(in_2sps, strobe_idx + 1, mu);

        out_2sps[out_complex * 2 + 0]     = sat_int16(crealf(slot0));
        out_2sps[out_complex * 2 + 1]     = sat_int16(cimagf(slot0));
        out_2sps[out_complex * 2 + 2]     = sat_int16(crealf(slot1));
        out_2sps[out_complex * 2 + 3]     = sat_int16(cimagf(slot1));
        out_complex += 2;

        // Gardner TED. y_mid is the midpoint BETWEEN prev_strobe and
        // slot0 — that's the PREVIOUS iteration's slot1 (which was
        // computed as half-symbol AFTER prev_strobe). The first
        // iteration has no prev_slot1, so e stays at 0.
        float e = 0.0f;
        if (have_history) {
            float complex diff = slot0 - prev_strobe;
            e = crealf(prev_slot1) * crealf(diff)
              + cimagf(prev_slot1) * cimagf(diff);
            e *= 1.0f / 1.0e7f;     // normalise per-symbol amplitude
        }

        // PI loop filter.
        w += st->Ki * e;
        float v = st->Kp * e + w;

        // Trace diagnostic state PRE-advance so symbol index matches
        // the strobe used to produce this output.
        if (st->trace && st->trace->n < SYM_TIMING_TRACE_CAP) {
            int t = st->trace->n++;
            st->trace->e [t] = e;
            st->trace->mu[t] = mu;
            st->trace->w [t] = w;
        }

        // Advance by 2 input complex samples (1 symbol) + v (loop
        // correction in fractional symbol units).
        mu += v;
        int int_step = (int)mu;
        mu -= (float)int_step;
        if (mu < 0.0f) { mu += 1.0f; int_step--; }
        strobe_idx += 2 + int_step;
        if (strobe_idx >= n_complex) break;

        prev_strobe   = slot0;
        prev_slot1    = slot1;
        have_history  = true;
    }

    // Save state.
    st->mu             = mu;
    st->w              = w;
    st->prev_strobe    = prev_strobe;
    st->prev_midpoint  = prev_slot1;
    st->have_history   = have_history ? 1 : 0;

    // Zero any unwritten tail (if loop exited early). Should be rare.
    if (out_complex * 2 < n_int16) {
        memset(&out_2sps[out_complex * 2], 0,
               (n_int16 - out_complex * 2) * sizeof(int16_t));
    }
}
