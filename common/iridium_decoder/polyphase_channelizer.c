// Polyphase FFT channelizer. See polyphase_channelizer.h for the
// architecture. This first implementation is critically-sampled
// (D = M = 64): each output cycle consumes 64 input samples and
// produces 64 channel outputs. Channel spacing = output rate = fs_in/M.
//
// Reference: Harris, "Multirate Signal Processing for Communication
// Systems" Ch. 9; gr-iridium/lib/fft_channelizer_impl.cc uses the
// FFT-overlap-save variant which is more cache-friendly on desktop
// CPUs but heavier overall — polyphase is a better fit for embedded.

#include "polyphase_channelizer.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct polyphase_channelizer {
    uint32_t            fs_in_hz;

    // Prototype low-pass filter, length = POLYCHAN_FILTER_LEN.
    // h_phase[p][n] = h[n*M + p] for p ∈ [0,M), n ∈ [0,N_taps_per_phase).
    // Layout: h_phase[p * N + n] gives the n'th tap of phase p.
    float               h_phase[POLYCHAN_FILTER_LEN];

    // Per-phase circular delay line. dl[p][n] is the n'th-most-recent
    // input that fell into phase p. Layout: dl[p * N + n].
    float complex       dl[POLYCHAN_FILTER_LEN];

    // Position within the per-phase delay line (newest sample at index dl_head).
    // Phases all share the same head — they advance in lockstep.
    int                 dl_head;
};

// FFT for the channelizer (M=64). Direct radix-2 Cooley-Tukey,
// in-place, no scaling. Bit-reversal permutation up front.
//
// We don't use esp-dsp here because we want the channelizer host-buildable
// (for unit tests) and the M=64 case is small enough that a hand-rolled
// FFT is faster than the function-call overhead of the IDF version.
static void fft_64(float complex *x)
{
    // Bit-reversal permutation (M = 64 = 2^6, so reverse 6 bits).
    static const uint8_t br[64] = {
         0, 32, 16, 48,  8, 40, 24, 56,  4, 36, 20, 52, 12, 44, 28, 60,
         2, 34, 18, 50, 10, 42, 26, 58,  6, 38, 22, 54, 14, 46, 30, 62,
         1, 33, 17, 49,  9, 41, 25, 57,  5, 37, 21, 53, 13, 45, 29, 61,
         3, 35, 19, 51, 11, 43, 27, 59,  7, 39, 23, 55, 15, 47, 31, 63,
    };
    for (int i = 0; i < 64; i++) {
        if (br[i] > i) {
            float complex t = x[i];
            x[i] = x[br[i]];
            x[br[i]] = t;
        }
    }
    // Cooley-Tukey radix-2.
    for (int stride = 1; stride < 64; stride <<= 1) {
        int half = stride;
        int span = stride << 1;
        for (int k = 0; k < half; k++) {
            float ang = -M_PI * (float)k / (float)half;
            float complex w = cosf(ang) + sinf(ang) * I;
            for (int i = k; i < 64; i += span) {
                float complex t = w * x[i + half];
                x[i + half] = x[i] - t;
                x[i]        = x[i] + t;
            }
        }
    }
}

// Build a Hamming-windowed sinc prototype low-pass filter, then
// polyphase-decompose into M phases.
//
//   prototype[k] = sinc(2 * fc/fs * (k - (L-1)/2)) * window[k]
//   fc = fs / (2 * M)   (passband edge at half the channel spacing)
//
// Polyphase decomposition: h_phase[p, n] = h[n*M + p].
static void build_prototype(float h_phase[POLYCHAN_FILTER_LEN])
{
    const int L = POLYCHAN_FILTER_LEN;     // 512
    const int M = POLYCHAN_M;
    const int N = POLYCHAN_N_TAPS_PER_PHASE;
    float h[POLYCHAN_FILTER_LEN];
    float center = (L - 1) / 2.0f;
    // Normalised cutoff = 1/(2M) so each channel passes ±fs/(2M)/2 and
    // adjacent channels (centered at multiples of fs/M) are fully in
    // the stop-band. With a Hamming window at L = M*N taps, the
    // transition band is ~fs/L; for N=8 that's about half a channel
    // spacing, giving ≥40 dB adjacent rejection.
    float cutoff = 1.0f / (2.0f * (float)M);
    float sum = 0.0f;
    for (int k = 0; k < L; k++) {
        float t = (float)k - center;
        float sinc = (t == 0.0f)
                     ? 2.0f * cutoff
                     : sinf(2.0f * (float)M_PI * cutoff * t) / ((float)M_PI * t);
        // Hamming window
        float w = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * (float)k
                                       / (float)(L - 1));
        h[k] = sinc * w;
        sum += h[k];
    }
    // Normalise so the DC response is 1.0 per channel.
    if (sum != 0.0f) {
        float scale = (float)M / sum;
        for (int k = 0; k < L; k++) {
            h[k] *= scale;
        }
    }
    // Polyphase decompose.
    for (int p = 0; p < M; p++) {
        for (int n = 0; n < N; n++) {
            h_phase[p * N + n] = h[n * M + p];
        }
    }
}

polyphase_channelizer_t *polyphase_channelizer_create(uint32_t fs_in_hz)
{
    polyphase_channelizer_t *ch = (polyphase_channelizer_t *)
                                    calloc(1, sizeof(*ch));
    if (!ch) return NULL;
    ch->fs_in_hz = fs_in_hz;
    build_prototype(ch->h_phase);
    ch->dl_head = 0;
    return ch;
}

void polyphase_channelizer_destroy(polyphase_channelizer_t *ch)
{
    free(ch);
}

void polyphase_channelizer_get_prototype(const polyphase_channelizer_t *ch,
                                         float *out, size_t out_len)
{
    if (!ch || !out) return;
    // Re-flatten the polyphase form back to a single linear filter for
    // inspection. h[n*M + p] = h_phase[p*N + n].
    for (int p = 0; p < POLYCHAN_M; p++) {
        for (int n = 0; n < POLYCHAN_N_TAPS_PER_PHASE; n++) {
            size_t linear_idx = (size_t)n * POLYCHAN_M + (size_t)p;
            if (linear_idx < out_len) {
                out[linear_idx] = ch->h_phase[p * POLYCHAN_N_TAPS_PER_PHASE + n];
            }
        }
    }
}

int32_t polyphase_channelizer_channel_freq(const polyphase_channelizer_t *ch,
                                           int k)
{
    if (!ch) return 0;
    // k > M/2 wraps to negative.
    int signed_k = (k > POLYCHAN_M / 2) ? (k - POLYCHAN_M) : k;
    return (int32_t)((int64_t)signed_k * ch->fs_in_hz / POLYCHAN_M);
}

size_t polyphase_channelizer_process(polyphase_channelizer_t *ch,
                                     const float complex *input,
                                     size_t n_input,
                                     float complex *out_block)
{
    if (!ch || !input || !out_block) return 0;
    const int M = POLYCHAN_M;
    const int N = POLYCHAN_N_TAPS_PER_PHASE;
    size_t n_cycles = n_input / M;
    // FFT scratch — one row of M filtered phase outputs.
    float complex fft_buf[POLYCHAN_M];

    for (size_t cycle = 0; cycle < n_cycles; cycle++) {
        const float complex *in = input + cycle * M;
        // Polyphase decomposition: x[mM + p] is consumed by phase p
        // (Type 1, no commutator reversal). With our forward-FFT
        // convention (X[k] = Σ x[n] exp(-j2πkn/N)), this gives
        // channel k centred at +k * fs/M (k > M/2 wraps to negative
        // freq), matching polyphase_channelizer_channel_freq().
        int new_head = (ch->dl_head - 1 + N) % N;
        for (int p = 0; p < M; p++) {
            ch->dl[p * N + new_head] = in[p];
        }
        ch->dl_head = new_head;

        // Compute filtered output per phase: y_p = Σ h_phase[p,n] * dl[p,head+n]
        // The delay line is read newest-first (head, head+1, head+2, ...
        // wrapping mod N). Coefficient h_phase[p, 0] multiplies the newest
        // sample; h_phase[p, N-1] multiplies the oldest.
        for (int p = 0; p < M; p++) {
            const float *hp = &ch->h_phase[p * N];
            const float complex *dlp = &ch->dl[p * N];
            float complex acc = 0;
            for (int n = 0; n < N; n++) {
                int idx = (ch->dl_head + n) % N;
                acc += hp[n] * dlp[idx];
            }
            fft_buf[p] = acc;
        }

        fft_64(fft_buf);

        // Write out the M channels for this cycle.
        for (int k = 0; k < M; k++) {
            out_block[cycle * M + k] = fft_buf[k];
        }
    }
    return n_cycles;
}
