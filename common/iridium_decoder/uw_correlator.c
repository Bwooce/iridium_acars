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

#define UW_LENGTH        12
#define PREAMBLE_LENGTH  16
#define SYNC_LENGTH      (PREAMBLE_LENGTH + UW_LENGTH)   // 28
#define SYM_STRIDE       2     // 2 samples per symbol

// Full sync-word sign patterns: preamble (16 syms) + UW (12 syms),
// each symbol mapped to BPSK ±(1+j) on the +1+j / -1-j axis.
//
// Per gr-iridium burst_downmix_impl.cc::generate_sync_word():
//   DL preamble = 16× s0 (= +(1+j))   → signs all +1
//   UL preamble = 8× (s1, s0)         → signs -1, +1, -1, +1, ...
//   DL UW = { s0, s1, s1, s1, s1, s0, s0, s0, s1, s0, s0, s1 }
//   UL UW = { s1, s1, s0, s0, s0, s1, s0, s0, s1, s0, s1, s1 }
static const int8_t SYNC_DL_SIGN[SYNC_LENGTH] = {
    +1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,   // preamble
    +1,-1,-1,-1,-1,+1,+1,+1,-1,+1,+1,-1                  // UW
};
static const int8_t SYNC_UL_SIGN[SYNC_LENGTH] = {
    -1,+1,-1,+1,-1,+1,-1,+1,-1,+1,-1,+1,-1,+1,-1,+1,   // preamble
    -1,-1,+1,+1,+1,-1,+1,+1,-1,+1,-1,-1                  // UW
};

// RRC pulse shape: β=0.4, 11 taps at 2 sps = 5.5 symbol periods.
// Matches gr-iridium's symbol-period coverage (51 taps at 10 sps =
// 5.1 symbols). Earlier we used 21 taps (10 symbols, 2× coverage)
// which gave slightly higher matched-filter SNR on quiet bursts but
// diverged from gr-iridium's reference behaviour.
#define RRC_BETA       0.4f
#define RRC_NTAPS      11
#define SYNC_RRC_LEN   (SYNC_LENGTH * SYM_STRIDE)   // 56 samples
static float s_rrc_taps[RRC_NTAPS];     // RRC for filtering the burst
static float s_rc_taps[RRC_NTAPS];      // RC for shaping the sync ref
// RRC-shaped sync word references (complex, +1+j axis). Computed
// at init by zero-padding the BPSK symbols at sps stride and
// convolving with the RRC FIR. Used as the matched filter against
// the burst — proper RRC matched-filter SNR.
static float s_sync_dl_re[SYNC_RRC_LEN];
static float s_sync_dl_im[SYNC_RRC_LEN];
static float s_sync_ul_re[SYNC_RRC_LEN];
static float s_sync_ul_im[SYNC_RRC_LEN];
static bool  s_sync_inited = false;

// Generate root-raised-cosine FIR taps. Standard textbook formula;
// matches gr::filter::firdes::root_raised_cosine. Used to RRC-filter
// the received burst (matched-filter side: gr-iridium's d_rrc_fir).
static void make_rrc_taps(float *taps, int ntaps, int sps, float beta)
{
    const float PI = 3.14159265358979323846f;
    int center = (ntaps - 1) / 2;
    float sum = 0.0f;
    for (int n = 0; n < ntaps; n++) {
        float k = (float)(n - center) / (float)sps;     // in symbol periods
        float v;
        float abs_k = k < 0 ? -k : k;
        if (abs_k < 1e-6f) {
            v = 1.0f - beta + 4.0f * beta / PI;
        } else {
            float four_bk = 4.0f * beta * k;
            float denom = PI * k * (1.0f - four_bk * four_bk);
            if (fabsf(denom) < 1e-9f) {
                // Singularity at k = ±1/(4β): use l'Hôpital limit.
                v = (beta / sqrtf(2.0f)) *
                    ((1.0f + 2.0f / PI) * sinf(PI / (4.0f * beta)) +
                     (1.0f - 2.0f / PI) * cosf(PI / (4.0f * beta)));
            } else {
                float num = sinf(PI * (1.0f - beta) * k)
                          + four_bk * cosf(PI * (1.0f + beta) * k);
                v = num / denom;
            }
        }
        taps[n] = v;
        sum += v;
    }
    // Normalise to unit DC gain (matches gr-iridium gain=1.0).
    if (sum != 0.0f) {
        float inv = 1.0f / sum;
        for (int n = 0; n < ntaps; n++) taps[n] *= inv;
    }
}

// Generate raised-cosine FIR taps. RC = RRC ⊛ RRC; gr-iridium uses
// RC on the sync_word reference so that correlating against the
// RRC-filtered burst gives the true matched-filter output.
static void make_rc_taps(float *taps, int ntaps, int sps, float beta)
{
    const float PI = 3.14159265358979323846f;
    int center = (ntaps - 1) / 2;
    float sum = 0.0f;
    for (int n = 0; n < ntaps; n++) {
        float k = (float)(n - center) / (float)sps;     // in symbol periods
        float v;
        float abs_k = k < 0 ? -k : k;
        if (abs_k < 1e-6f) {
            v = 1.0f;
        } else {
            float two_bk = 2.0f * beta * k;
            float denom = 1.0f - two_bk * two_bk;
            if (fabsf(denom) < 1e-9f) {
                // Singularity at k = ±1/(2β)
                v = (PI / 4.0f) * (sinf(PI / (2.0f * beta)) / (PI / (2.0f * beta)));
            } else {
                float sinc = (fabsf(PI * k) < 1e-9f) ? 1.0f
                             : sinf(PI * k) / (PI * k);
                v = sinc * cosf(PI * beta * k) / denom;
            }
        }
        taps[n] = v;
        sum += v;
    }
    if (sum != 0.0f) {
        float inv = 1.0f / sum;
        for (int n = 0; n < ntaps; n++) taps[n] *= inv;
    }
}

// Convolve the BPSK sync impulses (one per symbol, at stride sps)
// with the given pulse-shape FIR to get the matched-filter reference
// at sample rate. Output is complex (±1±j axis): for each symbol with
// sign s, the impulse contributes s·(1+j)·shape[k]. gr-iridium uses
// RC (= RRC ⊛ RRC) on the sync reference so that correlating against
// the RRC-filtered burst gives the optimal matched filter output.
static void build_shaped_sync(const int8_t *signs, const float *shape,
                              float *out_re, float *out_im)
{
    memset(out_re, 0, SYNC_RRC_LEN * sizeof(float));
    memset(out_im, 0, SYNC_RRC_LEN * sizeof(float));
    int center = (RRC_NTAPS - 1) / 2;
    for (int sym = 0; sym < SYNC_LENGTH; sym++) {
        int impulse_pos = sym * SYM_STRIDE;
        int s = signs[sym];                 // ±1
        for (int t = 0; t < RRC_NTAPS; t++) {
            int out_idx = impulse_pos + (t - center);
            if (out_idx < 0 || out_idx >= SYNC_RRC_LEN) continue;
            float contrib = (float)s * shape[t];
            out_re[out_idx] += contrib;
            out_im[out_idx] += contrib;
        }
    }
}

// FFT-based correlation (gr-iridium burst_downmix_impl.cc, lines
// 342-394, 608-650). The reversed-conjugated RC-shaped sync word
// is FFT'd once at init and stored in s_sync_dl_fft / s_sync_ul_fft.
// Per burst we FFT the burst (zero-padded to CORR_FFT_N), multiply
// elementwise by the stored sync FFT, IFFT, and peak-find.
//
// CORR_FFT_N matches gr-iridium's d_corr_fft_size = next_pow2(
//   d_sync_search_len + sync_word_len - 1) = next_pow2(168 + 56 - 1)
//   = 256. Search range = CORR_FFT_N - SYNC_RRC_LEN + 1 = 201 burst
// samples after D13's start-finder trim — covers the worst-case
// envelope misalignment gr-iridium's pipeline is designed for.
#define CORR_FFT_N   256
#define CORR_FFT_LOG 8
static uint16_t s_corr_brev[CORR_FFT_N];
static float    s_corr_tw_re[CORR_FFT_N / 2];
static float    s_corr_tw_im[CORR_FFT_N / 2];
// Pre-computed FFTs of the reversed-conjugated RC-shaped sync
// references, zero-padded to CORR_FFT_N. Used by the burst-
// correlation FFT path. gr-iridium pattern: `volk_32fc_conjugate`
// + `std::reverse` + FFT, stored as `d_dl_preamble_reversed_conj_fft`.
static float    s_sync_dl_fft_re[CORR_FFT_N];
static float    s_sync_dl_fft_im[CORR_FFT_N];
static float    s_sync_ul_fft_re[CORR_FFT_N];
static float    s_sync_ul_fft_im[CORR_FFT_N];

// Generic radix-2 DIT FFT. Used at both CFO and correlation scales
// (CFO_FFT_N and CORR_FFT_N are both 1024 in this build; if they
// ever diverge, give each its own table set).
static void radix2_fft(float *re, float *im, int N, int log_N,
                        const uint16_t *brev,
                        const float *tw_re, const float *tw_im)
{
    for (int i = 0; i < N; i++) {
        int j = brev[i];
        if (j > i) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
    (void)log_N;
    for (int stride = 1; stride < N; stride <<= 1) {
        int span = stride << 1;
        int step = (N / 2) / stride;
        for (int k = 0; k < stride; k++) {
            float wr = tw_re[k * step];
            float wi = tw_im[k * step];
            for (int i = k; i < N; i += span) {
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

// Inverse FFT via conj-FFT-conj/N (gr-iridium uses VOLK's IFFT but
// this is mathematically identical).
static void radix2_ifft(float *re, float *im, int N, int log_N,
                         const uint16_t *brev,
                         const float *tw_re, const float *tw_im)
{
    for (int i = 0; i < N; i++) im[i] = -im[i];
    radix2_fft(re, im, N, log_N, brev, tw_re, tw_im);
    float inv = 1.0f / (float)N;
    for (int i = 0; i < N; i++) { re[i] *= inv; im[i] = -im[i] * inv; }
}

static void sync_init(void)
{
    if (s_sync_inited) return;
    make_rrc_taps(s_rrc_taps, RRC_NTAPS, SYM_STRIDE, RRC_BETA);
    make_rc_taps (s_rc_taps,  RRC_NTAPS, SYM_STRIDE, RRC_BETA);
    build_shaped_sync(SYNC_DL_SIGN, s_rc_taps, s_sync_dl_re, s_sync_dl_im);
    build_shaped_sync(SYNC_UL_SIGN, s_rc_taps, s_sync_ul_re, s_sync_ul_im);

    // Init the correlation FFT tables.
    for (int i = 0; i < CORR_FFT_N; i++) {
        uint16_t r = 0, v = (uint16_t)i;
        for (int b = 0; b < CORR_FFT_LOG; b++) {
            r = (uint16_t)((r << 1) | (v & 1));
            v = (uint16_t)(v >> 1);
        }
        s_corr_brev[i] = r;
    }
    for (int k = 0; k < CORR_FFT_N / 2; k++) {
        double ang = -2.0 * 3.14159265358979323846 * (double)k / (double)CORR_FFT_N;
        s_corr_tw_re[k] = (float)cos(ang);
        s_corr_tw_im[k] = (float)sin(ang);
    }

    // Pre-compute reversed-conjugated sync FFTs. gr-iridium does:
    //   std::reverse(sync_padded.begin(), sync_padded.end());
    //   volk_32fc_conjugate_32fc(sync_padded, sync_padded, ...);
    //   fft_engine.execute(sync_padded → store_buf)
    // For our purposes, the RC-shaped sync is real-imag (1+j axis),
    // so reversed-conjugate is: out[n] = conj(sync[L-1-n]).
    float tmp_re[CORR_FFT_N], tmp_im[CORR_FFT_N];
    const int L = SYNC_RRC_LEN;
    // DL
    memset(tmp_re, 0, sizeof(tmp_re));
    memset(tmp_im, 0, sizeof(tmp_im));
    for (int n = 0; n < L; n++) {
        tmp_re[n] =  s_sync_dl_re[L - 1 - n];
        tmp_im[n] = -s_sync_dl_im[L - 1 - n];      // conjugate
    }
    radix2_fft(tmp_re, tmp_im, CORR_FFT_N, CORR_FFT_LOG,
                s_corr_brev, s_corr_tw_re, s_corr_tw_im);
    memcpy(s_sync_dl_fft_re, tmp_re, sizeof(tmp_re));
    memcpy(s_sync_dl_fft_im, tmp_im, sizeof(tmp_im));
    // UL
    memset(tmp_re, 0, sizeof(tmp_re));
    memset(tmp_im, 0, sizeof(tmp_im));
    for (int n = 0; n < L; n++) {
        tmp_re[n] =  s_sync_ul_re[L - 1 - n];
        tmp_im[n] = -s_sync_ul_im[L - 1 - n];
    }
    radix2_fft(tmp_re, tmp_im, CORR_FFT_N, CORR_FFT_LOG,
                s_corr_brev, s_corr_tw_re, s_corr_tw_im);
    memcpy(s_sync_ul_fft_re, tmp_re, sizeof(tmp_re));
    memcpy(s_sync_ul_fft_im, tmp_im, sizeof(tmp_im));

    s_sync_inited = true;
}

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
// gr-iridium parameters (burst_downmix_impl.cc lines 128-134):
//   d_cfo_est_fft_size = next_pow2(sps × (PREAMBLE_LENGTH_SHORT + 10))
//                      = next_pow2(2 × 26) = 64
//   d_fft_over_size_facor = 16  → effective FFT = 64 × 16 = 1024
//   Window = Blackman (gr::fft::window::WIN_BLACKMAN)
//   No SNR gate.
#define CFO_FFT_N        1024
#define CFO_FFT_LOG      10
#define CFO_INPUT_N      56     // 28 syms (preamble + UW) × 2 sps
#define CFO_PREAMBLE_N   32     // 16 syms × 2 sps; uw_offset_complex
                                // anchors the window's right side

static inline float parabolic_interp(float yl, float yc, float yr);

static uint16_t s_cfo_brev[CFO_FFT_N];
static float    s_cfo_tw_re[CFO_FFT_N / 2];
static float    s_cfo_tw_im[CFO_FFT_N / 2];
// Blackman windows (matches gr::fft::window::WIN_BLACKMAN):
//   w[n] = 0.42 - 0.5·cos(2πn/(N-1)) + 0.08·cos(4πn/(N-1))
// One for the full 56-sample preamble+UW window, one for the
// 24-sample UW-only fallback used when uw_offset doesn't leave
// room for the preamble.
static float    s_cfo_window_full[CFO_INPUT_N];
static float    s_cfo_window_uw[24];
static bool     s_cfo_inited = false;

static void cfo_init(void)
{
    if (s_cfo_inited) return;
    for (int i = 0; i < CFO_FFT_N; i++) {
        uint16_t r = 0, v = (uint16_t)i;
        for (int b = 0; b < CFO_FFT_LOG; b++) {
            r = (uint16_t)((r << 1) | (v & 1));
            v = (uint16_t)(v >> 1);
        }
        s_cfo_brev[i] = r;
    }
    for (int k = 0; k < CFO_FFT_N / 2; k++) {
        double ang = -2.0 * 3.14159265358979323846 * (double)k / (double)CFO_FFT_N;
        s_cfo_tw_re[k] = (float)cos(ang);
        s_cfo_tw_im[k] = (float)sin(ang);
    }
    // Blackman windows (gr::fft::window::WIN_BLACKMAN definition).
    const float PI = 3.14159265358979323846f;
    for (int i = 0; i < CFO_INPUT_N; i++) {
        float t = (float)i / (float)(CFO_INPUT_N - 1);
        s_cfo_window_full[i] = 0.42f - 0.5f * cosf(2.0f * PI * t)
                                     + 0.08f * cosf(4.0f * PI * t);
    }
    for (int i = 0; i < 24; i++) {
        float t = (float)i / (float)(24 - 1);
        s_cfo_window_uw[i] = 0.42f - 0.5f * cosf(2.0f * PI * t)
                                   + 0.08f * cosf(4.0f * PI * t);
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

    // FFT scratch in BSS — at N=1024 it's 8 KB per array, too big
    // for stack on the worker task. Function is single-threaded so
    // static is fine.
    static float re[CFO_FFT_N], im[CFO_FFT_N];
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

    // Find peak (gr-iridium: std::max_element on magnitude²).
    float peak_mag = -1.0f;
    int   peak_k   = 0;
    for (int k = 0; k < CFO_FFT_N; k++) {
        float m = re[k] * re[k] + im[k] * im[k];
        if (m > peak_mag) { peak_mag = m; peak_k = k; }
    }
    if (peak_mag <= 1e-9f) return 0.0f;

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

// (UW-only sign arrays removed — the active correlator uses
// SYNC_*_SIGN[28]. The second half of each SYNC_*_SIGN array is the
// UW pattern, kept synchronised by source.)

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

    // Search range must leave room for the entire SYNC matched filter
    // (preamble + UW = 28 syms × 2 sps = 56 samples). With RRC pulse
    // shape that's also the sample-rate length, since the impulses
    // sit at symbol positions and RRC tails fall within the window.
    int max_k = n_complex - SYNC_RRC_LEN;
    if (max_k <= 0) return;
    if (search_complex > max_k) search_complex = max_k;
    if (search_complex <= 2) return;

    sync_init();    // ensures RRC taps + shaped refs + sync FFTs

    // gr-iridium FFT-based correlation (burst_downmix_impl.cc 608-650):
    //   1. Copy burst into CORR_FFT_N buffer, zero-padded.
    //   2. Forward FFT (d_corr_fft).
    //   3. Multiply elementwise by pre-computed reversed-conj sync FFT
    //      (d_dl_preamble_reversed_conj_fft / _ul_).
    //   4. Inverse FFT (d_corr_dl_ifft / _ul_).
    //   5. Find peak of magnitude² (std::max_element).
    static float burst_re[CORR_FFT_N], burst_im[CORR_FFT_N];
    static float ifft_re[CORR_FFT_N], ifft_im[CORR_FFT_N];
    memset(burst_re, 0, sizeof(burst_re));
    memset(burst_im, 0, sizeof(burst_im));
    int load_n = n_complex < CORR_FFT_N ? n_complex : CORR_FFT_N;
    for (int i = 0; i < load_n; i++) {
        burst_re[i] = (float)burst_2sps[i * 2 + 0];
        burst_im[i] = (float)burst_2sps[i * 2 + 1];
    }
    radix2_fft(burst_re, burst_im, CORR_FFT_N, CORR_FFT_LOG,
                s_corr_brev, s_corr_tw_re, s_corr_tw_im);

    // The convolution peak for cross-correlation (sync * conj(reversed)
    // ⊛ burst) lands at k = search_position + (L-1) where L = sync
    // length. We compensate by shifting the peak index by -(L-1) when
    // reporting, so peak_k becomes the burst-sample offset where the
    // sync STARTS.
    float best_dl = 0.0f, best_ul = 0.0f;
    int   best_dl_k = 0, best_ul_k = 0;
    float best_dl_re = 0, best_dl_im = 0;
    float best_ul_re = 0, best_ul_im = 0;
    double sum_dl = 0, sum_ul = 0;
    int valid_count = 0;
    const int L_minus_1 = SYNC_RRC_LEN - 1;

    // DL path: multiply burst_fft × sync_dl_fft (elementwise complex),
    // IFFT, magnitude-find.
    for (int k = 0; k < CORR_FFT_N; k++) {
        float ar = burst_re[k], ai = burst_im[k];
        float br = s_sync_dl_fft_re[k], bi = s_sync_dl_fft_im[k];
        ifft_re[k] = ar * br - ai * bi;
        ifft_im[k] = ar * bi + ai * br;
    }
    radix2_ifft(ifft_re, ifft_im, CORR_FFT_N, CORR_FFT_LOG,
                 s_corr_brev, s_corr_tw_re, s_corr_tw_im);
    for (int k = 0; k < search_complex; k++) {
        int idx = k + L_minus_1;
        if (idx >= CORR_FFT_N) break;
        float re = ifft_re[idx], im = ifft_im[idx];
        float m2 = re * re + im * im;
        sum_dl += m2;
        if (k == 0) valid_count = 0;
        valid_count++;
        if (m2 > best_dl) {
            best_dl = m2; best_dl_k = k;
            best_dl_re = re; best_dl_im = im;
        }
    }
    // UL path: same with sync_ul_fft.
    for (int k = 0; k < CORR_FFT_N; k++) {
        float ar = burst_re[k], ai = burst_im[k];
        float br = s_sync_ul_fft_re[k], bi = s_sync_ul_fft_im[k];
        ifft_re[k] = ar * br - ai * bi;
        ifft_im[k] = ar * bi + ai * br;
    }
    radix2_ifft(ifft_re, ifft_im, CORR_FFT_N, CORR_FFT_LOG,
                 s_corr_brev, s_corr_tw_re, s_corr_tw_im);
    for (int k = 0; k < search_complex; k++) {
        int idx = k + L_minus_1;
        if (idx >= CORR_FFT_N) break;
        float re = ifft_re[idx], im = ifft_im[idx];
        float m2 = re * re + im * im;
        sum_ul += m2;
        if (m2 > best_ul) {
            best_ul = m2; best_ul_k = k;
            best_ul_re = re; best_ul_im = im;
        }
    }
    if (valid_count <= 1) return;
    (void)search_complex;       // valid_count is the real divisor

    // Pick the better direction.
    uw_direction_t dir;
    int peak_k;
    float peak_mag2;
    double avg_off_peak;
    if (best_dl > best_ul) {
        dir = UW_DIR_DOWNLINK;
        peak_k = best_dl_k;
        peak_mag2 = best_dl;
        avg_off_peak = (sum_dl - best_dl) / (double)(valid_count - 1);
    } else {
        dir = UW_DIR_UPLINK;
        peak_k = best_ul_k;
        peak_mag2 = best_ul;
        avg_off_peak = (sum_ul - best_ul) / (double)(valid_count - 1);
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

    // Parabolic interpolation around the peak. Recompute the RRC-
    // shaped matched-filter magnitudes at peak_k-1 and peak_k+1.
    float yl = 0, yc = peak_mag2, yr = 0;
    if (peak_k > 0 && peak_k + 1 < search_complex) {
        const float *rr = (dir == UW_DIR_DOWNLINK) ? s_sync_dl_re : s_sync_ul_re;
        const float *ii = (dir == UW_DIR_DOWNLINK) ? s_sync_dl_im : s_sync_ul_im;
        for (int side = 0; side < 2; side++) {
            int k = peak_k + (side ? +1 : -1);
            float re = 0, im = 0;
            for (int n = 0; n < SYNC_RRC_LEN; n++) {
                int idx = (k + n) * 2;
                float br = (float)burst_2sps[idx + 0];
                float bi = (float)burst_2sps[idx + 1];
                re += rr[n] * br + ii[n] * bi;
                im += ii[n] * br - rr[n] * bi;
            }
            float m2 = re * re + im * im;
            if (side) yr = m2; else yl = m2;
        }
    }
    float correction = parabolic_interp(yl, yc, yr);

    // peak_k is the SYNC start = PREAMBLE start. Downstream consumers
    // want the UW position. UW is PREAMBLE_LENGTH × SYM_STRIDE = 32
    // complex samples after sync start.
    out_result->uw_offset       = peak_k + PREAMBLE_LENGTH * SYM_STRIDE;
    out_result->correction      = correction;
    out_result->direction       = dir;
    out_result->snr_estimate_db = snr_db;
    out_result->peak_value      = peak_mag2;
    // Sign convention fix: the FFT correlation computes
    //   conv = burst ⊛ conj(reversed(sync))
    //        = Σ conj(sync)·burst    → peak phase = +φ_burst
    // whereas the worker's pre-rotation was designed for the
    // time-domain convention
    //   Σ sync·conj(burst)            → peak phase = -φ_burst
    // (worker multiplies sample by peak/|peak| to UNDO the burst's
    // phase: works for -φ peak, but doubles the phase for +φ peak).
    // Conjugate the peak value here so callers see the time-domain
    // sign convention regardless of which correlation path was used.
    if (dir == UW_DIR_DOWNLINK) {
        out_result->peak_re =  best_dl_re;
        out_result->peak_im = -best_dl_im;
    } else {
        out_result->peak_re =  best_ul_re;
        out_result->peak_im = -best_ul_im;
    }

    // CFO estimate over the preamble+UW window. peak_k is the SYNC
    // start (= preamble start); cfo_fine_estimate's interface takes
    // the UW position (= peak_k + 32), from which it windows the 56
    // samples behind. Result is the residual omega in rad/sym.
    out_result->omega_per_sym = cfo_fine_estimate(
        burst_2sps, n_complex, peak_k + PREAMBLE_LENGTH * SYM_STRIDE);
}

// D13 burst-start finder constants (mirror gr-iridium defaults).
// Their low-pass filter is firdes.low_pass_2(1, fs, 2.5e3, 5e3, 60dB)
// which at 50 ksps yields ~33 taps. We use a simpler triangular
// (Bartlett-windowed) moving average of 17 taps (≈0.34 ms = 8.5
// symbol periods at 25 ksym/s) for the smoothing step — same
// effective time-constant, much cheaper, doesn't ring on signal
// onsets the way a sharp-cutoff LP would.
#define START_LP_NTAPS         17
#define START_THRESHOLD_FRAC   0.28f
// gr-iridium's d_pre_start_samples is a small bias to back off from
// the detected edge so we don't clip the preamble's first symbol.
// They use 1-2 samples; we follow with 2.
#define START_PRE_SAMPLES      2

// D13 scratch sized for the worst-case burst length we see (50 ksps
// × ~50 ms max burst = 2500 samples). Independent of CORR_FFT_N
// because start_finder needs to scan the WHOLE burst (gr-iridium's
// d_search_depth is also a few thousand samples), whereas the
// sync-search FFT operates only on the trimmed window after D13.
#define START_SEARCH_MAX   2560

int uw_correlator_find_burst_start(const int16_t *burst_2sps,
                                    int n_complex, int search_max)
{
    if (n_complex <= START_LP_NTAPS) return 0;
    if (search_max > n_complex) search_max = n_complex;
    if (search_max <= START_LP_NTAPS + 1) return 0;

    // Use a static scratch — bursts are processed one at a time on
    // worker_core1, so no reentrancy concern.
    static float mag2[START_SEARCH_MAX];
    static float smooth[START_SEARCH_MAX];
    if (search_max > START_SEARCH_MAX) search_max = START_SEARCH_MAX;

    // Step 1: per-sample magnitude² of complex burst.
    for (int n = 0; n < search_max; n++) {
        float r = (float)burst_2sps[n * 2 + 0];
        float i = (float)burst_2sps[n * 2 + 1];
        mag2[n] = r * r + i * i;
    }

    // Step 2: smooth with Bartlett-windowed moving average (acts as
    // a near-LP filter; triangular shape reduces high-freq leakage).
    // Coefficients: w[k] = 1 - |k - center|/center, normalised.
    float wsum = 0;
    float w[START_LP_NTAPS];
    int half = START_LP_NTAPS / 2;
    for (int k = 0; k < START_LP_NTAPS; k++) {
        float d = (float)(k - half);
        if (d < 0) d = -d;
        w[k] = 1.0f - d / (float)half;
        wsum += w[k];
    }
    for (int k = 0; k < START_LP_NTAPS; k++) w[k] /= wsum;
    // Apply (valid mode): smooth[n] valid for n in [half, search_max-half-1].
    // For n outside that, copy raw mag² (so threshold search still
    // works at the very start of the burst).
    for (int n = 0; n < search_max; n++) {
        if (n >= half && n + half < search_max) {
            float acc = 0;
            for (int k = 0; k < START_LP_NTAPS; k++) {
                acc += w[k] * mag2[n - half + k];
            }
            smooth[n] = acc;
        } else {
            smooth[n] = mag2[n];
        }
    }

    // Step 3: max of smoothed envelope.
    float max_val = 0;
    for (int n = 0; n < search_max; n++) {
        if (smooth[n] > max_val) max_val = smooth[n];
    }
    if (max_val <= 1.0f) return 0;     // essentially silence

    // Step 4: first crossing of 28% of max.
    float thr = max_val * START_THRESHOLD_FRAC;
    int start = -1;
    for (int n = 0; n < search_max; n++) {
        if (smooth[n] >= thr) { start = n; break; }
    }
    if (start < 0) return 0;

    // Step 5: back off by half_fir - pre_start_samples so we don't
    // chop into the preamble. gr-iridium: `start = max(start +
    // half_fir_size - d_pre_start_samples, 0)`. With our LP being
    // a centred moving average, half_fir_size = (NTAPS-1)/2 = 8.
    int adjust = (START_LP_NTAPS - 1) / 2 - START_PRE_SAMPLES;
    start = start + adjust;
    if (start < 0) start = 0;
    if (start >= n_complex) start = n_complex - 1;
    return start;
}

void uw_correlator_apply_rrc(const int16_t *burst_in, int16_t *burst_out,
                              int n_complex)
{
    sync_init();   // ensures RRC taps are populated
    const int center = (RRC_NTAPS - 1) / 2;
    // We support in-place by reading-before-write via an N-sample
    // ring of the input. For our typical RRC_NTAPS=21 this is a
    // 21-sample × 4-byte ring = 84 bytes of stack — trivial.
    int16_t ring_i[RRC_NTAPS] = {0};
    int16_t ring_q[RRC_NTAPS] = {0};
    int head = 0;
    // The output sample at index n depends on input samples
    // [n - center .. n + center]. We delay output by `center`
    // samples relative to input so the ring always holds the
    // needed window. Burst[in_idx=n-center] is fed in when we
    // emit out[n]. For in_idx < 0 we feed zeros.
    for (int out_idx = -center; out_idx < n_complex + center; out_idx++) {
        // Feed input at (out_idx + center) into the ring.
        int in_idx = out_idx + center;
        int16_t si = 0, sq = 0;
        if (in_idx >= 0 && in_idx < n_complex) {
            si = burst_in[in_idx * 2 + 0];
            sq = burst_in[in_idx * 2 + 1];
        }
        ring_i[head] = si;
        ring_q[head] = sq;
        head = (head + 1) % RRC_NTAPS;
        // Compute output once we've fed enough samples (out_idx >= 0).
        if (out_idx >= 0 && out_idx < n_complex) {
            float acc_re = 0, acc_im = 0;
            // ring[head] is the OLDEST sample (next to be overwritten);
            // it corresponds to the sample at out_idx - center (= the
            // left edge of the filter window).
            for (int t = 0; t < RRC_NTAPS; t++) {
                int ring_idx = (head + t) % RRC_NTAPS;
                acc_re += s_rrc_taps[t] * (float)ring_i[ring_idx];
                acc_im += s_rrc_taps[t] * (float)ring_q[ring_idx];
            }
            if (acc_re >  32767.0f) acc_re =  32767.0f;
            if (acc_re < -32768.0f) acc_re = -32768.0f;
            if (acc_im >  32767.0f) acc_im =  32767.0f;
            if (acc_im < -32768.0f) acc_im = -32768.0f;
            burst_out[out_idx * 2 + 0] = (int16_t)acc_re;
            burst_out[out_idx * 2 + 1] = (int16_t)acc_im;
        }
    }
}

float uw_correlator_estimate_cfo(const int16_t *burst_2sps, int n_complex,
                                  int head_n)
{
    // Reuse cfo_fine_estimate by anchoring at head_n/2 so it starts
    // at sample 0 (its internal start = anchor - CFO_PREAMBLE_N).
    // We don't need head_n parameter for the FFT itself — internally
    // it always picks 56 samples if preamble window fits, else 24.
    (void)head_n;
    if (n_complex <= CFO_PREAMBLE_N + 12 * SYM_STRIDE) return 0.0f;
    return cfo_fine_estimate(burst_2sps, n_complex, CFO_PREAMBLE_N);
}
