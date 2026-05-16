// See uw_correlator.h.
//
// Algorithm:
//   For each candidate UW start position k:
//     corr_dl[k] = | sum_{i=0..11}  uw_dl_cplx[i] * conj(burst[k + 2i]) |²
//     corr_ul[k] = | sum_{i=0..11}  uw_ul_cplx[i] * conj(burst[k + 2i]) |²
//   Find peak across all k for both DL and UL. Pick the larger.
//   Parabolic interpolation of the 3 bins around the peak gives sub-
//   sample timing precision.
//
// uw_dl_cplx[i] is the DL UW pattern mapped to BPSK constellation:
//   IR_UW_DL[i] = 0  → +1 + j      (quadrant 0)
//   IR_UW_DL[i] = 2  → -1 - j      (quadrant 2)
//   (UW values are always {0, 2} — see iridium.h)
// Similarly for UL.
//
// Symbol stride: at 2 sps, consecutive UW symbols are 2 burst samples
// apart. So we step burst by 2 between each UW symbol comparison.

#include "uw_correlator.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#define UW_LENGTH    12
#define SYM_STRIDE   2     // 2 samples per symbol

// gr-iridium-style fine CFO estimator. Squaring a BPSK signal
// (UW symbols at quadrants 0/2 = ±(1+j)) removes the modulation:
//   x² = (±(1+j))² = ±2j → constant ±2j, no information, so any
//   carrier offset Δω becomes a tone at 2Δω after squaring.
// We FFT the squared region, find the peak, parabolic-interpolate
// for sub-bin resolution, divide by 2 to undo squaring. Result is
// returned in rad/sym (assuming the input samples are at 2 sps,
// which matches the rest of this module).
//
// N=64 with hand-rolled radix-2 to keep the call cost tiny — this
// runs once per detected burst, so a few hundred flops is nothing.
#define CFO_FFT_N        128
#define CFO_FFT_LOG      7
#define CFO_INPUT_N      56  // 16 preamble syms + 12 UW syms, ×2 sps.
                              // Both Iridium preambles square to a
                              // constant phasor (DL: all s0; UL: s1,s0
                              // alternating — s1²=s0²=2j), so the whole
                              // 56-sample window is BPSK after squaring
                              // and produces a clean tone at Δω. Falls
                              // back to UW-only if uw_offset is too
                              // small to include the preamble.
#define CFO_PREAMBLE_N   32  // 16 syms × 2 sps available before UW.

static inline float parabolic_interp(float yl, float yc, float yr);

static uint8_t  s_cfo_brev[CFO_FFT_N];
static float    s_cfo_tw_re[CFO_FFT_N / 2];
static float    s_cfo_tw_im[CFO_FFT_N / 2];
// Two precomputed Hann windows: one for the full 56-sample
// preamble+UW window, one for the 24-sample UW-only fallback.
static float    s_cfo_window_full[CFO_INPUT_N];
static float    s_cfo_window_uw[24];
static bool     s_cfo_inited = false;

static void cfo_init(void)
{
    if (s_cfo_inited) return;
    for (int i = 0; i < CFO_FFT_N; i++) {
        uint8_t r = 0, v = (uint8_t)i;
        for (int b = 0; b < CFO_FFT_LOG; b++) {
            r = (uint8_t)((r << 1) | (v & 1));
            v = (uint8_t)(v >> 1);
        }
        s_cfo_brev[i] = r;
    }
    for (int k = 0; k < CFO_FFT_N / 2; k++) {
        double ang = -2.0 * 3.14159265358979323846 * (double)k / (double)CFO_FFT_N;
        s_cfo_tw_re[k] = (float)cos(ang);
        s_cfo_tw_im[k] = (float)sin(ang);
    }
    for (int i = 0; i < CFO_INPUT_N; i++) {
        s_cfo_window_full[i] = 0.5f * (1.0f - cosf(2.0f * 3.14159265358979323846f *
                                                   (float)i / (float)(CFO_INPUT_N - 1)));
    }
    for (int i = 0; i < 24; i++) {
        s_cfo_window_uw[i] = 0.5f * (1.0f - cosf(2.0f * 3.14159265358979323846f *
                                                 (float)i / (float)(24 - 1)));
    }
    s_cfo_inited = true;
}

static void cfo_fft(float *re, float *im)
{
    for (int i = 0; i < CFO_FFT_N; i++) {
        int j = s_cfo_brev[i];
        if (j > i) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
    for (int stride = 1; stride < CFO_FFT_N; stride <<= 1) {
        int span = stride << 1;
        int step = (CFO_FFT_N / 2) / stride;
        for (int k = 0; k < stride; k++) {
            float wr = s_cfo_tw_re[k * step];
            float wi = s_cfo_tw_im[k * step];
            for (int i = k; i < CFO_FFT_N; i += span) {
                float xr = re[i + stride];
                float xi = im[i + stride];
                float tr = wr * xr - wi * xi;
                float ti = wr * xi + wi * xr;
                re[i + stride] = re[i] - tr;
                im[i + stride] = im[i] - ti;
                re[i]          = re[i] + tr;
                im[i]          = im[i] + ti;
            }
        }
    }
}

// Square-then-FFT CFO estimator. Uses preamble + UW (~56 samples)
// when uw_offset is large enough to include the preamble; otherwise
// falls back to UW-only (24 samples). The Iridium preamble (DL: 16×
// (+1+j); UL: alternating (-1-j), (+1+j)) squares to a constant 2j
// phasor — both halves of the squared signal are noise-free DC, so
// the carrier offset becomes a single clean tone at Δω rad/sample
// throughout the whole window. Returns omega_per_sym in rad/sym
// (with the sign convention the worker expects, see end of function).
static float cfo_fine_estimate(const int16_t *burst_2sps, int n_complex,
                                int uw_offset_complex)
{
    cfo_init();

    float re[CFO_FFT_N], im[CFO_FFT_N];
    memset(re, 0, sizeof(re));
    memset(im, 0, sizeof(im));

    // Decide preamble+UW vs UW-only based on what's in range.
    int start, n_in;
    const float *win;
    if (uw_offset_complex >= CFO_PREAMBLE_N &&
        uw_offset_complex - CFO_PREAMBLE_N + CFO_INPUT_N <= n_complex) {
        start = uw_offset_complex - CFO_PREAMBLE_N;
        n_in  = CFO_INPUT_N;
        win   = s_cfo_window_full;
    } else if (uw_offset_complex + 24 <= n_complex) {
        start = uw_offset_complex;
        n_in  = 24;
        win   = s_cfo_window_uw;
    } else {
        return 0.0f;       // burst too short, skip
    }

    // Square the windowed region. (re + j·im)² = (re²-im²) + j·(2·re·im).
    // Normalise by 1/32768 to keep magnitudes in float range and so the
    // window shape, not amplitude, dominates the FFT spectrum.
    const float inv_full = 1.0f / 32768.0f;
    for (int i = 0; i < n_in; i++) {
        float w = win[i];
        float r = (float)burst_2sps[(start + i) * 2 + 0] * inv_full;
        float m = (float)burst_2sps[(start + i) * 2 + 1] * inv_full;
        re[i] = (r * r - m * m) * w;
        im[i] = (2.0f * r * m)   * w;
    }

    cfo_fft(re, im);

    // Find peak + accumulate total spectrum energy for SNR gating.
    float peak_mag = -1.0f;
    int   peak_k   = 0;
    double sum_mag = 0;
    for (int k = 0; k < CFO_FFT_N; k++) {
        float m = re[k] * re[k] + im[k] * im[k];
        sum_mag += m;
        if (m > peak_mag) { peak_mag = m; peak_k = k; }
    }
    if (peak_mag <= 1e-9f) return 0.0f;
    // Gate on peak vs mean off-peak: legitimate squared-preamble
    // tones have peak/mean ≥ N/4 (single bin dominates ~quarter of
    // power). Noise spectra have peak/mean ~ N/N = 1. Require ≥ 5×
    // (about CFO_FFT_N / 25) to reject noise events.
    double off_mean = (sum_mag - peak_mag) / (double)(CFO_FFT_N - 1);
    if (off_mean > 1e-12 && peak_mag / off_mean < 5.0) {
        return 0.0f;
    }

    // Parabolic interpolation around the peak (with wrap).
    int km1 = (peak_k - 1 + CFO_FFT_N) % CFO_FFT_N;
    int kp1 = (peak_k + 1) % CFO_FFT_N;
    float yl = re[km1] * re[km1] + im[km1] * im[km1];
    float yc = peak_mag;
    float yr = re[kp1] * re[kp1] + im[kp1] * im[kp1];
    float delta = parabolic_interp(yl, yc, yr);

    // Convert (signed) bin position to fractional cycles per sample,
    // then to rad/sym. The squared spectrum is at 2·Δω_per_sample =
    // 2·(Δω_per_sym/2) = Δω_per_sym, so dividing by 2 at the end
    // gives back the original per-symbol omega.
    float kf = (float)peak_k + delta;
    if (kf >= (float)CFO_FFT_N / 2.0f) kf -= (float)CFO_FFT_N;
    // Cycles per FFT bin = kf / CFO_FFT_N → rad/sample = 2π·kf/N.
    float rad_per_sample = 2.0f * 3.14159265358979323846f * kf / (float)CFO_FFT_N;
    // 2 sps → rad/sym = 2 × rad/sample; squared so divide by 2 → cancels.
    // Net: omega_per_sym = rad_per_sample.
    // Sign convention: worker multiplies burst by exp(+j·omega/2·n).
    // The burst's carrier offset is encoded as exp(+j·Δω/2·n), so to
    // CANCEL it the per-sample advance must be exp(-j·Δω/2·n) — i.e.,
    // the returned omega_per_sym must be -Δω. The squared FFT finds
    // +Δω, so we negate. (The earlier two-half method already had the
    // negation baked in via the conj order in h2·conj(h1).)
    float omega = -rad_per_sample;
    if (omega >  1.5f) omega =  1.5f;
    if (omega < -1.5f) omega = -1.5f;
    return omega;
}

// UW patterns from iridium.h (IR_UW_DL / IR_UW_UL) mapped to {+1, -1}
// on the BPSK +1+j / -1-j axis. We store just the sign because both I
// and Q have the same sign per symbol.
//   sign = +1 for UW value 0, sign = -1 for UW value 2.
static const int8_t UW_DL_SIGN[UW_LENGTH] = {
    +1, -1, -1, -1, -1, +1, +1, +1, -1, +1, +1, -1
};
static const int8_t UW_UL_SIGN[UW_LENGTH] = {
    -1, -1, +1, +1, +1, -1, +1, +1, -1, +1, -1, -1
};

// Quadratic interpolation of the parabolic peak through three points
// (y_left, y_peak, y_right). Returns the fractional offset from the
// centre bin in [-0.5, +0.5].
static inline float parabolic_interp(float yl, float yc, float yr)
{
    float denom = yl - 2.0f * yc + yr;
    if (denom > -1e-12f && denom < 1e-12f) return 0.0f;
    float delta = 0.5f * (yl - yr) / denom;
    if (delta >  0.5f) delta =  0.5f;
    if (delta < -0.5f) delta = -0.5f;
    return delta;
}

void uw_correlator_find(const int16_t *burst_2sps, int n_complex,
                         int search_complex,
                         uw_corr_result_t *out_result)
{
    if (!out_result) return;
    out_result->uw_offset = 0;
    out_result->correction = 0.0f;
    out_result->direction = UW_DIR_UNKNOWN;
    out_result->snr_estimate_db = 0.0f;
    out_result->peak_value = 0.0f;
    out_result->peak_re = 0.0f;
    out_result->peak_im = 0.0f;
    out_result->omega_per_sym = 0.0f;

    int max_k = n_complex - UW_LENGTH * SYM_STRIDE;
    if (max_k <= 0) return;
    if (search_complex > max_k) search_complex = max_k;
    if (search_complex <= 2) return;

    // Scan correlations across k = 0..search_complex-1. Track the
    // best DL and UL peaks separately.
    float best_dl = 0.0f, best_ul = 0.0f;
    int   best_dl_k = 0, best_ul_k = 0;
    float best_dl_re = 0, best_dl_im = 0;
    float best_ul_re = 0, best_ul_im = 0;
    // Save just-around-peak mags for parabolic interpolation. We
    // store the 3 most recent magnitudes per direction.
    // Simpler: do a second pass after locating the peak.
    // For now: skip interpolation; compute it post-loop.

    // For SNR estimate, accumulate sum of all squared correlations.
    double sum_dl = 0, sum_ul = 0;

    for (int k = 0; k < search_complex; k++) {
        // Accumulate complex correlation for DL and UL at this offset.
        float dl_re = 0, dl_im = 0, ul_re = 0, ul_im = 0;
        for (int i = 0; i < UW_LENGTH; i++) {
            int idx = (k + i * SYM_STRIDE) * 2;
            int br = burst_2sps[idx + 0];
            int bi = burst_2sps[idx + 1];
            int s_dl = UW_DL_SIGN[i];
            int s_ul = UW_UL_SIGN[i];
            // UW symbol = s * (1 + j). conj(burst) = (br, -bi).
            //   uw_dl × conj(burst) = s_dl * (1 + j) * (br - j*bi)
            //                       = s_dl * (br + bi + j*(br - bi))
            dl_re += s_dl * (br + bi);
            dl_im += s_dl * (br - bi);
            ul_re += s_ul * (br + bi);
            ul_im += s_ul * (br - bi);
        }
        float dl_mag2 = dl_re * dl_re + dl_im * dl_im;
        float ul_mag2 = ul_re * ul_re + ul_im * ul_im;
        sum_dl += dl_mag2;
        sum_ul += ul_mag2;
        if (dl_mag2 > best_dl) {
            best_dl = dl_mag2; best_dl_k = k;
            best_dl_re = dl_re; best_dl_im = dl_im;
        }
        if (ul_mag2 > best_ul) {
            best_ul = ul_mag2; best_ul_k = k;
            best_ul_re = ul_re; best_ul_im = ul_im;
        }
    }

    // Pick the better direction.
    uw_direction_t dir;
    int peak_k;
    float peak_mag2;
    double avg_off_peak;
    if (best_dl > best_ul) {
        dir = UW_DIR_DOWNLINK;
        peak_k = best_dl_k;
        peak_mag2 = best_dl;
        avg_off_peak = (sum_dl - best_dl) / (double)(search_complex - 1);
    } else {
        dir = UW_DIR_UPLINK;
        peak_k = best_ul_k;
        peak_mag2 = best_ul;
        avg_off_peak = (sum_ul - best_ul) / (double)(search_complex - 1);
    }

    // SNR estimate: peak² over mean off-peak² (in dB).
    float snr_db = 0.0f;
    if (avg_off_peak > 1e-6) {
        snr_db = 10.0f * log10f((float)(peak_mag2 / avg_off_peak));
    }

    // Reject low-SNR peaks. Threshold of 6 dB is conservative —
    // legitimate Iridium UW correlation peaks are typically 12-20 dB
    // above the off-peak floor; 6 dB filters out noise events.
    if (snr_db < 6.0f) {
        out_result->snr_estimate_db = snr_db;
        out_result->peak_value = peak_mag2;
        return;
    }

    // Parabolic interpolation around the peak. Need the three
    // magnitudes at k-1, k, k+1 for the winning direction.
    float yl = 0, yc = peak_mag2, yr = 0;
    if (peak_k > 0 && peak_k + 1 < search_complex) {
        // Recompute mags at peak_k-1 and peak_k+1 (cheap — 12 multiplies).
        const int8_t *sign = (dir == UW_DIR_DOWNLINK) ? UW_DL_SIGN : UW_UL_SIGN;
        for (int side = 0; side < 2; side++) {
            int k = peak_k + (side ? +1 : -1);
            float re = 0, im = 0;
            for (int i = 0; i < UW_LENGTH; i++) {
                int idx = (k + i * SYM_STRIDE) * 2;
                int br = burst_2sps[idx + 0];
                int bi = burst_2sps[idx + 1];
                int s = sign[i];
                re += s * (br + bi);
                im += s * (br - bi);
            }
            float m2 = re * re + im * im;
            if (side) yr = m2; else yl = m2;
        }
    }
    float correction = parabolic_interp(yl, yc, yr);

    out_result->uw_offset       = peak_k;
    out_result->correction      = correction;
    out_result->direction       = dir;
    out_result->snr_estimate_db = snr_db;
    out_result->peak_value      = peak_mag2;
    if (dir == UW_DIR_DOWNLINK) {
        out_result->peak_re = best_dl_re;
        out_result->peak_im = best_dl_im;
    } else {
        out_result->peak_re = best_ul_re;
        out_result->peak_im = best_ul_im;
    }

    // CFO estimate via gr-iridium-style square-then-FFT over
    // preamble+UW (56 samples) when available, UW-only (24 samples)
    // when uw_offset is too close to the start of the burst to
    // include the preamble. See cfo_fine_estimate() for math.
    out_result->omega_per_sym = cfo_fine_estimate(burst_2sps, n_complex, peak_k);
}
