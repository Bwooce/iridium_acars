// See resample_256_to_250.h.

#include "resample_256_to_250.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// Modified Bessel I0, for Kaiser window. Copy of the same function in
// direct_if_decim.c — could be deduped, but the two modules are
// independent.
static double bessel_i0(double x)
{
    double sum = 1.0, term = 1.0;
    double half_x_sq = (x * x) / 4.0;
    for (int k = 1; k < 50; k++) {
        term *= half_x_sq / (double)(k * k);
        sum += term;
        if (term < 1e-12 * sum) break;
    }
    return sum;
}

// Build the firmr_s16 coefficient table.
//
// scipy.signal.resample_poly(x, up=125, down=128) generates an FIR
// with these defaults (per scipy.signal._signaltools._resample_poly_fir):
//   max_rate    = max(up, down)            = 128
//   ntaps_proto = 2 × 10 × max_rate + 1   = 2561   (proto in the
//                                                   interp×fs virtual
//                                                   frame)
//   cutoff_norm = 1 / max_rate            = 0.0078125  (in cycles/sample
//                                                       in the virtual frame)
//   beta        = 5.0                      (kaiser default)
//
// But the FULL 2561-tap prototype is overkill for our application
// (gri's tagger is robust to a few dB of amplitude variation and a
// few hundred Hz transition-band ripple). We use a SHORTER prototype
// to fit memory budget at ~5 KB:
//   delay_size  = 9
//   ntaps_proto = delay_size × interp = 1125
//   cutoff      = 0.4 / interp (giving ~1.25 MHz passband edge in
//                 the virtual 320 MSPS frame, well past anything we
//                 care about)
//   beta        = 8.0  (60 dB stopband — more than gri spec, cheap
//                       at this length)
//
// Layout for firmr_s16: coeffs[tap_pos * interp + phase]. tap_pos
// counts symbol-spaced taps (0..delay_size-1); phase counts virtual
// sample positions within each output cycle (0..interp-1).
//
// Mapping from the linear-phase prototype: the (tap_pos, phase) entry
// reads prototype[tap_pos × interp + phase]. We sum-normalise so the
// per-phase DC gain is unity (= integral of the LPF response at DC).
static void make_resample_coeffs(int16_t *coeffs)
{
    const int      INTERP    = RS25_INTERP;
    const int      DSIZE     = RS25_DELAY_SIZE;
    const int      NPROTO    = DSIZE * INTERP;            // 1125
    const double   PI        = 3.14159265358979323846;
    const double   beta      = 8.0;
    // Filter cutoff in cycles/sample at the virtual interp×fs rate.
    // We pick 0.4 × (1/interp) so the passband covers ±0.4 × fs_in/2
    // = ±0.4 × 1.28 MHz = ±512 kHz at 2.56 MSPS input. Way more than
    // we need for Iridium (channel grid spans ±400 kHz around LO).
    const double   fc_norm   = 0.4 / (double)INTERP;
    const double   inv_i0    = 1.0 / bessel_i0(beta);
    const int      center    = NPROTO / 2;

    double w[RS25_DELAY_SIZE * RS25_INTERP];
    double sum_phase[RS25_INTERP];
    for (int p = 0; p < INTERP; p++) sum_phase[p] = 0.0;

    for (int k = 0; k < NPROTO; k++) {
        double t = (double)(k - center);
        double sinc = (t == 0.0)
                      ? 2.0 * fc_norm
                      : sin(2.0 * PI * fc_norm * t) / (PI * t);
        double u = 2.0 * (double)k / (double)(NPROTO - 1) - 1.0;
        double arg = beta * sqrt(1.0 - u * u);
        double kw  = bessel_i0(arg) * inv_i0;
        w[k] = sinc * kw;
        sum_phase[k % INTERP] += w[k];
    }
    // Normalise each phase to DC gain = 1 / INTERP (gri convention:
    // the polyphase resampler's per-phase MAC sums divided by the
    // implicit "interp output samples per input cycle" factor). With
    // the firmr_s16 shift=15 (matching dsps_firmr_s16 default), the
    // taps need to sum to ~1.0 per phase to give unity DC gain after
    // the >> 15 in the MAC accumulator. So normalise each phase to
    // sum=1.
    for (int p = 0; p < INTERP; p++) {
        if (sum_phase[p] == 0.0) continue;
        double scale = 1.0 / sum_phase[p];
        for (int t = 0; t < DSIZE; t++) {
            w[t * INTERP + p] *= scale;
        }
    }

    // Quantise to Q15. Per-phase max tap ≈ 0.3 → Q15 ≈ 9830, well
    // within int16. Layout matches firmr_s16: coeffs[tap × interp + phase].
    for (int k = 0; k < NPROTO; k++) {
        double v = w[k] * (double)INT16_MAX;
        if (v >  (double)INT16_MAX) v =  (double)INT16_MAX;
        if (v < -(double)INT16_MAX) v = -(double)INT16_MAX;
        coeffs[k] = (int16_t)lrint(v);
    }
}

void resample_256_to_250_init(resample_256_to_250_t *r)
{
    make_resample_coeffs(r->coeffs);
    memset(r->delay_i, 0, sizeof(r->delay_i));
    memset(r->delay_q, 0, sizeof(r->delay_q));
    firmr_s16_init(&r->fir_i, r->coeffs, r->delay_i,
                   RS25_DELAY_SIZE, RS25_INTERP, RS25_DECIM,
                   /*start_pos=*/ 0, /*shift=*/ 0);
    firmr_s16_init(&r->fir_q, r->coeffs, r->delay_q,
                   RS25_DELAY_SIZE, RS25_INTERP, RS25_DECIM,
                   /*start_pos=*/ 0, /*shift=*/ 0);
}

int resample_256_to_250_process(resample_256_to_250_t *r,
                                 const int16_t *in_iq, int n_in_complex,
                                 int16_t *out_iq)
{
    // Deinterleave I/Q into stack buffers (firmr_s16 wants planar).
    // For the typical fixture this is ~5 MB; allocate dynamically
    // to keep stack frames small.
    int16_t *in_i = (int16_t *)malloc(n_in_complex * sizeof(int16_t));
    int16_t *in_q = (int16_t *)malloc(n_in_complex * sizeof(int16_t));
    if (!in_i || !in_q) { free(in_i); free(in_q); return 0; }
    for (int k = 0; k < n_in_complex; k++) {
        in_i[k] = in_iq[2 * k + 0];
        in_q[k] = in_iq[2 * k + 1];
    }

    int n_max_out = (int)((int64_t)n_in_complex * RS25_INTERP / RS25_DECIM) + 8;
    int16_t *out_i = (int16_t *)malloc(n_max_out * sizeof(int16_t));
    int16_t *out_q = (int16_t *)malloc(n_max_out * sizeof(int16_t));
    if (!out_i || !out_q) {
        free(in_i); free(in_q); free(out_i); free(out_q);
        return 0;
    }

    int n_out_i = firmr_s16_process(&r->fir_i, in_i, out_i, n_in_complex);
    int n_out_q = firmr_s16_process(&r->fir_q, in_q, out_q, n_in_complex);
    int n_out = n_out_i < n_out_q ? n_out_i : n_out_q;

    for (int k = 0; k < n_out; k++) {
        out_iq[2 * k + 0] = out_i[k];
        out_iq[2 * k + 1] = out_q[k];
    }

    free(in_i); free(in_q); free(out_i); free(out_q);
    return n_out;
}
