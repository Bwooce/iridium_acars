// See direct_if_decim.h. Builds a Kaiser-window sinc LPF at init,
// then applies it as a direct-form FIR + integer downsample per call.

#include "direct_if_decim.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

// Modified Bessel function of the first kind, order 0. Used to
// generate Kaiser window weights. Series converges quickly for
// |β| < 20 (we use β ≈ 5.5).
static double bessel_i0(double x)
{
    double sum = 1.0;
    double term = 1.0;
    double half_x_sq = (x * x) / 4.0;
    for (int k = 1; k < 50; k++) {
        term *= half_x_sq / (double)(k * k);
        sum += term;
        if (term < 1e-12 * sum) break;
    }
    return sum;
}

void direct_if_decim_init(direct_if_decim_t *d)
{
    const double FS         = 2500000.0;
    const double F_CUTOFF   = 20000.0;       // burst_width / 2
    const double ATTEN_DB   = 40.0;
    // Transition width is 20 kHz (= burst_width/2). Documented for
    // reference; the tap count (DIDECIM_NTAPS = 279) is locked to
    // this design via the Kaiser formula:
    //   ntaps ≈ (atten - 7.95) / (2.285 * 2π * trans / fs)
    //         = 32.05 / 0.1148 ≈ 279

    // Kaiser β from atten (Oppenheim & Schafer formula).
    double beta;
    if (ATTEN_DB > 50.0) {
        beta = 0.1102 * (ATTEN_DB - 8.7);
    } else if (ATTEN_DB > 21.0) {
        beta = 0.5842 * pow(ATTEN_DB - 21.0, 0.4)
             + 0.07886 * (ATTEN_DB - 21.0);
    } else {
        beta = 0.0;
    }

    // Required ntaps from atten + transition width. We hardcode
    // DIDECIM_NTAPS = 279, so just verify the design is consistent:
    //   ntaps_needed ≈ (atten - 7.95) / (2.285 * 2π * trans / fs)
    //                = (40 - 7.95) / (2.285 * 2π * 20e3 / 2.5e6)
    //                = 32.05 / 0.1148 ≈ 279
    // ✓

    const double PI = 3.14159265358979323846;
    const int    N  = DIDECIM_NTAPS;
    const int    center = N / 2;
    const double fc_norm = F_CUTOFF / FS;       // 0.008
    const double inv_i0_beta = 1.0 / bessel_i0(beta);

    // Build float taps: sinc × Kaiser window, then sum-normalise.
    double w[DIDECIM_NTAPS];
    double sum = 0.0;
    for (int k = 0; k < N; k++) {
        double t = (double)(k - center);
        double sinc = (t == 0.0)
                      ? 2.0 * fc_norm
                      : sin(2.0 * PI * fc_norm * t) / (PI * t);
        // Kaiser window: I0(β·sqrt(1-(2k/(N-1)-1)²)) / I0(β)
        double u = 2.0 * (double)k / (double)(N - 1) - 1.0;
        double arg = beta * sqrt(1.0 - u * u);
        double kw  = bessel_i0(arg) * inv_i0_beta;
        w[k] = sinc * kw;
        sum += w[k];
    }
    if (sum != 0.0) {
        for (int k = 0; k < N; k++) w[k] /= sum;
    }

    // Quantise to Q15. Sum-normalised float taps have peak ~0.05
    // (close to fc_norm * 2 for sinc-shaped LPF) → Q15 peak ≈ 1600,
    // comfortably inside int16. Sum of Q15 taps ≈ 2^15.
    for (int k = 0; k < N; k++) {
        double v = w[k] * (double)INT16_MAX;
        if (v >  (double)INT16_MAX) v = (double)INT16_MAX;
        if (v < -(double)INT16_MAX) v = -(double)INT16_MAX;
        d->taps[k] = (int16_t)lrint(v);
    }
}

int direct_if_decim_process(const direct_if_decim_t *d,
                             const int16_t *input, int n_in,
                             int16_t *out)
{
    if (n_in < DIDECIM_DECIM) return 0;

    const int N = DIDECIM_NTAPS;
    const int n_out = n_in / DIDECIM_DECIM;
    int written = 0;

    for (int k = 0; k < n_out; k++) {
        // Output k uses input centred at sample k * DECIM. FIR with
        // N taps reaches into samples [k*DECIM - N/2, k*DECIM + N/2].
        int center_idx = k * DIDECIM_DECIM;
        int32_t acc_re = 0, acc_im = 0;
        for (int t = 0; t < N; t++) {
            int idx = center_idx + t - N / 2;
            if (idx < 0 || idx >= n_in) continue;     // edge: zero-pad
            int32_t tap = d->taps[t];
            acc_re += tap * (int32_t)input[idx * 2 + 0];
            acc_im += tap * (int32_t)input[idx * 2 + 1];
        }
        out[written * 2 + 0] = (int16_t)(acc_re >> 15);
        out[written * 2 + 1] = (int16_t)(acc_im >> 15);
        written++;
    }
    return written;
}
