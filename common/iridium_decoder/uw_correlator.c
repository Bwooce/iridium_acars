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

#define UW_LENGTH    12
#define SYM_STRIDE   2     // 2 samples per symbol

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

    // Two-half phase difference → residual carrier omega estimate.
    // Compute correlations of the first 6 UW symbols and the last 6
    // separately at the winning offset. If the burst has a residual
    // freq offset Δω rad/sym, half2/half1 ≈ exp(j·Δω·6). The angle
    // of that ratio divided by 6 gives Δω. This is much finer than
    // the freq_estimator's ~5 kHz/bin FFT resolution, because the
    // UW correlator's effective freq bin is 1/(12·T) ≈ 2 kHz.
    {
        const int8_t *sign = (dir == UW_DIR_DOWNLINK) ? UW_DL_SIGN : UW_UL_SIGN;
        float h1_re = 0, h1_im = 0, h2_re = 0, h2_im = 0;
        for (int i = 0; i < UW_LENGTH; i++) {
            int idx = (peak_k + i * SYM_STRIDE) * 2;
            int br = burst_2sps[idx + 0];
            int bi = burst_2sps[idx + 1];
            int s  = sign[i];
            float cr = s * (br + bi);
            float ci = s * (br - bi);
            if (i < UW_LENGTH / 2) { h1_re += cr; h1_im += ci; }
            else                    { h2_re += cr; h2_im += ci; }
        }
        // ratio = h2 * conj(h1), angle = atan2(im, re), normalised by
        // 6 symbol periods (midpoint-to-midpoint of the two halves).
        float r_re = h2_re * h1_re + h2_im * h1_im;
        float r_im = h2_im * h1_re - h2_re * h1_im;
        float ang = atan2f(r_im, r_re);
        out_result->omega_per_sym = ang / (float)(UW_LENGTH / 2);
    }
}
