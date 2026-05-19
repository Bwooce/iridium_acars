// See direct_if_decim.h. Builds a Kaiser-window sinc LPF at init,
// then applies it as a direct-form FIR + integer downsample per call.

#include "direct_if_decim.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
// Target build: route the per-channel real-FIR through esp-dsp's
// PIE-accelerated dsps_fird_s16_arp4. The same delay-line semantics
// as the portable C inner loop below, so the swap is mechanical
// and produces equivalent output to the host's direct_if_decim_process_split.
#include "dsps_fir.h"
#define USE_DSPS_FIRD_ARP4 1
#endif

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
    // Design with N_DESIGN = 279 (gri's original Kaiser length), then
    // pad with one zero tap at the end so the final array length is
    // DIDECIM_NTAPS = 280 — required by dsps_fird_s16_arp4 (P4 PIE
    // FIR), which falls back to the scalar ANSI implementation if
    // coeffs_len is not a multiple of 8. Padding with zero preserves
    // the filter's frequency response exactly.
    const int    N_DESIGN = 279;
    const int    N        = DIDECIM_NTAPS;
    const int    center = N_DESIGN / 2;
    const double fc_norm = F_CUTOFF / FS;       // 0.008
    const double inv_i0_beta = 1.0 / bessel_i0(beta);

    // Build float taps over the design length, sum-normalise.
    double w[DIDECIM_NTAPS] = { 0 };   // remaining entries stay zero (padding)
    double sum = 0.0;
    for (int k = 0; k < N_DESIGN; k++) {
        double t = (double)(k - center);
        double sinc = (t == 0.0)
                      ? 2.0 * fc_norm
                      : sin(2.0 * PI * fc_norm * t) / (PI * t);
        // Kaiser window: I0(β·sqrt(1-(2k/(N-1)-1)²)) / I0(β)
        double u = 2.0 * (double)k / (double)(N_DESIGN - 1) - 1.0;
        double arg = beta * sqrt(1.0 - u * u);
        double kw  = bessel_i0(arg) * inv_i0_beta;
        w[k] = sinc * kw;
        sum += w[k];
    }
    if (sum != 0.0) {
        for (int k = 0; k < N_DESIGN; k++) w[k] /= sum;
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

    // Reset the streaming FIR delay lines / decimation counters so
    // _process_split can be called immediately after init without
    // first having to call _reset_state. Idempotent — re-init clears
    // the state too.
    direct_if_decim_reset_state(d);

#ifdef USE_DSPS_FIRD_ARP4
    if (!d->fir_dsp_inited) {
        // Init the two dsps_fird_s16 instances. We pass NULL for the
        // delay buffer because the arp4 path allocates its own
        // (memalign'd, +8 slots padding); see dsps_fird_init_s16.c.
        // shift=0 → output is straight Q15 (acc >> 15).
        dsps_fird_init_s16(&d->fir_dsp_i, d->taps, NULL,
                            DIDECIM_NTAPS, DIDECIM_DECIM, 0, 0);
        dsps_fird_init_s16(&d->fir_dsp_q, d->taps, NULL,
                            DIDECIM_NTAPS, DIDECIM_DECIM, 0, 0);
        d->fir_dsp_inited = 1;
    } else {
        // Re-init across calls: zero the internal delay lines. The
        // delay pointer was set up by the first init; we just clear
        // it back to silence so the next burst starts clean.
        for (int i = 0; i < DIDECIM_NTAPS; i++) {
            d->fir_dsp_i.delay[i] = 0;
            d->fir_dsp_q.delay[i] = 0;
        }
        d->fir_dsp_i.pos = 0;
        d->fir_dsp_q.pos = 0;
        d->fir_dsp_i.d_pos = 0;
        d->fir_dsp_q.d_pos = 0;
    }
#endif
}

// Reset the streaming delay-line state for both I and Q FIRs. Called
// implicitly by direct_if_decim_init() and exposed for callers that
// need to drop history between unrelated bursts.
void direct_if_decim_reset_state(direct_if_decim_t *d)
{
    memset(d->fir_i.delay, 0, sizeof(d->fir_i.delay));
    memset(d->fir_q.delay, 0, sizeof(d->fir_q.delay));
    d->fir_i.pos = 0;
    d->fir_q.pos = 0;
    d->fir_i.d_pos = 0;
    d->fir_q.d_pos = 0;

#ifdef USE_DSPS_FIRD_ARP4
    // Also reset the esp-dsp delay lines for the PIE path. dsps_fird's
    // delay buffer is allocated internally by init; we just zero its
    // contents and reset positions. Only valid if init has run.
    if (d->fir_dsp_inited) {
        for (int i = 0; i < DIDECIM_NTAPS; i++) {
            d->fir_dsp_i.delay[i] = 0;
            d->fir_dsp_q.delay[i] = 0;
        }
        d->fir_dsp_i.pos = 0;
        d->fir_dsp_q.pos = 0;
        d->fir_dsp_i.d_pos = 0;
        d->fir_dsp_q.d_pos = 0;
    }
#endif
}

// Portable Q15 real-FIR with circular delay line. Same semantics as
// dsps_fird_s16_ansi (esp-dsp): for each output sample, push DECIM
// new samples into the delay line then compute one MAC of delay ×
// reversed-coeffs, shift back to Q15.
//
// On target this gets replaced by dsps_fird_s16_arp4 inside
// direct_if_decim_process_split — same struct semantics so the swap
// is mechanical. Keeping the portable version here lets the host
// build exercise the same data path the firmware uses.
static int didecim_real_fir(didecim_fir_state_t *fs, const int16_t *taps,
                             const int16_t *in, int n_in, int16_t *out)
{
    const int N = DIDECIM_NTAPS;
    const int n_out = n_in / DIDECIM_DECIM;
    int written = 0;
    int in_pos = 0;
    for (int i = 0; i < n_out; i++) {
        // Push DECIM (or DECIM - d_pos on the first iteration after a
        // partial chunk; we always start aligned so d_pos == 0 here)
        // new samples into the delay line.
        int n_push = DIDECIM_DECIM - fs->d_pos;
        for (int j = 0; j < n_push; j++) {
            if (fs->pos >= N) fs->pos = 0;
            fs->delay[fs->pos++] = in[in_pos++];
        }
        fs->d_pos = 0;

        // Inner product: delay[pos..N) × coeffs[N-1..N-1-(N-pos)]
        // then delay[0..pos) × coeffs[N-1-(N-pos)..0]. The reverse
        // walk on coeffs matches the linear FIR convention.
        int64_t acc = 0;
        int coeff_pos = N - 1;
        for (int n = fs->pos; n < N; n++) {
            acc += (int32_t)taps[coeff_pos--] * (int32_t)fs->delay[n];
        }
        for (int n = 0; n < fs->pos; n++) {
            acc += (int32_t)taps[coeff_pos--] * (int32_t)fs->delay[n];
        }
        out[written++] = (int16_t)(acc >> 15);
    }
    return written;
}

int direct_if_decim_process_split(direct_if_decim_t *d,
                                   const int16_t *input, int n_in,
                                   int16_t *out,
                                   int16_t *scratch_in_i, int16_t *scratch_in_q,
                                   int16_t *scratch_out_i, int16_t *scratch_out_q)
{
    if (n_in < DIDECIM_DECIM) return 0;
    const int n_out = n_in / DIDECIM_DECIM;

    // Deinterleave IQ → two real streams.
    for (int k = 0; k < n_in; k++) {
        scratch_in_i[k] = input[k * 2 + 0];
        scratch_in_q[k] = input[k * 2 + 1];
    }

    // Per-stream FIR + decim. On target the inner call uses
    // dsps_fird_s16_arp4 (PIE-accelerated); on host we use the
    // portable C inner FIR that matches dsps_fird_s16_ansi's
    // semantics. Both produce the same numerical output to within
    // Q15 rounding.
#ifdef USE_DSPS_FIRD_ARP4
    // arp4 takes the OUTPUT count (input length / decim). The
    // function's return value is unreliable on this esp-dsp release
    // (see hardware-notes section of the plan); we use the explicit
    // n_out we computed instead.
    (void)dsps_fird_s16_arp4(&d->fir_dsp_i, scratch_in_i,
                              scratch_out_i, n_out);
    (void)dsps_fird_s16_arp4(&d->fir_dsp_q, scratch_in_q,
                              scratch_out_q, n_out);
#else
    didecim_real_fir(&d->fir_i, d->taps, scratch_in_i, n_in, scratch_out_i);
    didecim_real_fir(&d->fir_q, d->taps, scratch_in_q, n_in, scratch_out_q);
#endif

    // Re-interleave into the caller's output buffer.
    for (int k = 0; k < n_out; k++) {
        out[k * 2 + 0] = scratch_out_i[k];
        out[k * 2 + 1] = scratch_out_q[k];
    }
    return n_out;
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
