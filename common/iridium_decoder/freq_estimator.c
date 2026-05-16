// See freq_estimator.h.
//
// Implementation notes:
//   - Hand-rolled radix-2 Cooley-Tukey FFT for portability (host tests
//     have no esp-dsp dependency). On target the FFT could be swapped
//     for dsps_fft2r_fc32_arp4 for a ~3× speedup; the call site is
//     bounded so an #ifdef wrap is straightforward when needed.
//   - Twiddle tables are cached file-scope statics initialised on the
//     first call. The bit-reversal table is fixed for N=256.
//   - Magnitude computed per bin as r²+i² (no sqrt) since we only
//     need peak find.
//   - Search window: bins whose frequency is within ±search_hz are
//     considered. Bin k's frequency is k * (fs/N) if k < N/2, else
//     (k - N) * (fs/N). For fs=2.56 MHz, N=256, search_hz=20000:
//     bin_width = 10 kHz, ±2 bins each side of DC.
//   - Quadratic peak interpolation: classic 3-point formula
//       δ = 0.5 * (m[k-1] - m[k+1]) / (m[k-1] - 2*m[k] + m[k+1])
//     gives sub-bin offset of the parabolic peak through the three
//     samples. Stable when the denominator is nonzero; we guard.

#include "freq_estimator.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define N      FREQ_EST_FFT_N      // 512
#define LOG2N  9                    // log2(512)

// Bit-reversal table for N=512, 9-bit reverse.
static uint16_t s_brev[N];
static float   s_tw_re[N / 2];      // exp(-j 2π k / N) for k ∈ [0, N/2)
static float   s_tw_im[N / 2];
static float   s_window[N];         // Hann window
static bool    s_tables_inited = false;

static void init_tables(void)
{
    if (s_tables_inited) return;
    for (int i = 0; i < N; i++) {
        uint16_t r = 0;
        uint16_t v = (uint16_t)i;
        for (int b = 0; b < LOG2N; b++) {
            r = (uint16_t)((r << 1) | (v & 1));
            v = (uint16_t)(v >> 1);
        }
        s_brev[i] = r;
    }
    for (int k = 0; k < N / 2; k++) {
        double ang = -2.0 * M_PI * (double)k / (double)N;
        s_tw_re[k] = (float)cos(ang);
        s_tw_im[k] = (float)sin(ang);
    }
    // Hann window: 0.5 * (1 - cos(2π i / (N-1))). Smooths the FFT
    // sidelobes so the quadratic peak-interpolation isn't biased
    // by leakage from neighbouring bins — without the window we
    // saw 2-3 kHz interpolation error at off-bin tone offsets.
    for (int i = 0; i < N; i++) {
        s_window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI *
                                          (float)i / (float)(N - 1)));
    }
    s_tables_inited = true;
}

// In-place radix-2 DIT FFT on N=256 separate real/imag arrays.
// Twiddle access pattern matches polyphase_channelizer.c's cached
// fft_64 — for stride s, twiddle k uses table index k * (N / (2*s)).
static void fft_n256(float *re, float *im)
{
    // Bit-reversal permutation.
    for (int i = 0; i < N; i++) {
        int j = s_brev[i];
        if (j > i) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
    // Cooley-Tukey butterflies.
    for (int stride = 1; stride < N; stride <<= 1) {
        int half = stride;
        int span = stride << 1;
        int step = (N / 2) / half;
        for (int k = 0; k < half; k++) {
            float wr = s_tw_re[k * step];
            float wi = s_tw_im[k * step];
            for (int i = k; i < N; i += span) {
                float xr = re[i + half];
                float xi = im[i + half];
                // t = w * x
                float tr = wr * xr - wi * xi;
                float ti = wr * xi + wi * xr;
                re[i + half] = re[i] - tr;
                im[i + half] = im[i] - ti;
                re[i]        = re[i] + tr;
                im[i]        = im[i] + ti;
            }
        }
    }
}

int32_t freq_estimator_run(const int16_t *iq, size_t n_complex,
                           uint32_t fs_hz, uint32_t search_hz)
{
    if (!iq || n_complex < (size_t)N || fs_hz == 0) return 0;

    init_tables();

    // 1. Convert int16 IQ → float (windowed), separate real / imag arrays.
    float re[N], im[N];
    const float inv_full = 1.0f / 32768.0f;
    for (int i = 0; i < N; i++) {
        float w = s_window[i];
        re[i] = (float)iq[2 * i + 0] * inv_full * w;
        im[i] = (float)iq[2 * i + 1] * inv_full * w;
    }

    // 2. FFT.
    fft_n256(re, im);

    // 3. Magnitude squared and search for peak in ±search_hz window.
    // Bin k frequency: k*(fs/N) for k<N/2, else (k-N)*(fs/N).
    // We treat search_hz as a half-width; convert to a bin-radius.
    float bin_width_hz = (float)fs_hz / (float)N;
    int   bin_radius   = (int)((float)search_hz / bin_width_hz + 0.5f);
    if (bin_radius < 1) bin_radius = 1;
    if (bin_radius > N / 2 - 1) bin_radius = N / 2 - 1;

    float peak_mag = -1.0f;
    int   peak_k   = -1;
    // Positive half: bins [0..bin_radius] (k=0 = DC).
    for (int k = 0; k <= bin_radius; k++) {
        float m = re[k] * re[k] + im[k] * im[k];
        if (m > peak_mag) { peak_mag = m; peak_k = k; }
    }
    // Negative half: bins [N-bin_radius..N-1].
    for (int k = N - bin_radius; k < N; k++) {
        float m = re[k] * re[k] + im[k] * im[k];
        if (m > peak_mag) { peak_mag = m; peak_k = k; }
    }

    // Artefact guard: bail if no peak found OR if the peak magnitude
    // is essentially zero (no signal). The bin-0 case must NOT be
    // an automatic guard hit — legitimate small offsets (e.g. ±3 kHz
    // out of ±10 kHz/bin) land in bin 0 and need to be reported.
    if (peak_k < 0 || peak_mag < 1e-9f) return 0;

    // 4. Quadratic peak interpolation. Compute mag at peak_k±1
    // (with wrap-around mod N). All three magnitudes for the
    // parabola.
    int km1 = (peak_k - 1 + N) % N;
    int kp1 = (peak_k + 1) % N;
    float m0 = re[km1] * re[km1] + im[km1] * im[km1];
    float m1 = peak_mag;
    float m2 = re[kp1] * re[kp1] + im[kp1] * im[kp1];

    float denom = m0 - 2.0f * m1 + m2;
    float delta = 0.0f;
    if (denom < -1e-12f) {  // proper parabola opens downward
        delta = 0.5f * (m0 - m2) / denom;
        if (delta >  0.5f) delta =  0.5f;
        if (delta < -0.5f) delta = -0.5f;
    }

    // 5. Map fractional bin → Hz (signed).
    float kf = (float)peak_k + delta;
    if (peak_k > N / 2) kf -= (float)N;     // wrap to signed half
    float est_hz = kf * bin_width_hz;

    // Clamp to ±search_hz (interpolation can't push outside the
    // sampled bins, but be defensive).
    float lim = (float)search_hz;
    if (est_hz >  lim) est_hz =  lim;
    if (est_hz < -lim) est_hz = -lim;

    return (int32_t)lrintf(est_hz);
}
