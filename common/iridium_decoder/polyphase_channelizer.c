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

#ifdef ESP_PLATFORM
#include "dsps_fft2r.h"
#include "esp_log.h"
static bool s_dsps_fft64_fc32_inited = false;
static bool s_dsps_fft64_sc16_inited = false;
static void init_dsps_fft64_once(void)
{
    // esp-dsp keeps separate twiddle tables for fc32 vs sc16; both
    // are needed because we run the float path (process) and the
    // int16 path (process_int16) on target. Each init is idempotent
    // across the program lifetime.
    if (!s_dsps_fft64_fc32_inited) {
        esp_err_t err = dsps_fft2r_init_fc32(NULL, 64);
        if (err == ESP_OK) {
            s_dsps_fft64_fc32_inited = true;
        } else {
            ESP_LOGW("POLYCH", "dsps_fft2r_init_fc32(64) failed: %d — "
                                "float FFT falls back to hand-rolled", err);
        }
    }
    if (!s_dsps_fft64_sc16_inited) {
        esp_err_t err = dsps_fft2r_init_sc16(NULL, 64);
        if (err == ESP_OK) {
            s_dsps_fft64_sc16_inited = true;
        } else {
            ESP_LOGW("POLYCH", "dsps_fft2r_init_sc16(64) failed: %d — "
                                "int16 FFT falls back to scalar", err);
        }
    }
}
#endif

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

    // D20 step 3: Q15 / int16 path. Quantised taps and an int16 IQ
    // delay line live alongside the float ones; channelizer clients
    // pick which path via polyphase_channelizer_process (float) vs
    // polyphase_channelizer_process_int16 (int16). The two paths
    // maintain independent state — calling both on the same
    // channelizer instance is supported but not recommended (each
    // would see only half the samples). 16-byte alignment is for
    // the future PIE asm kernel which uses 128-bit vector loads.
    __attribute__((aligned(16))) int16_t h_phase_q15[POLYCHAN_FILTER_LEN];
    __attribute__((aligned(16))) int16_t dl_int16[POLYCHAN_FILTER_LEN * 2];
    int                 dl_head_int16;
};

// D20-step1: cached twiddle factors for fft_64. 32 entries covering
// exp(-j·π·m/32) for m ∈ [0,32). Every per-stage twiddle in the
// 64-point Cooley-Tukey decomposition is one of these, indexed by
// m = k * (32 / half). Built once at first channelizer create() and
// reused for all subsequent FFTs. This eliminates 63 cosf + 63 sinf
// calls per fft_64 invocation — at 40 ksps cycle rate that's 5 M
// trig calls/sec saved on the production datapath, which was the
// dominant cost in the first P4 profile (~42% of core time).
static float complex s_fft_twiddles[32];
static bool s_fft_twiddles_inited = false;

static void init_fft_twiddles(void)
{
    if (s_fft_twiddles_inited) return;
    for (int m = 0; m < 32; m++) {
        float ang = -(float)M_PI * (float)m / 32.0f;
        s_fft_twiddles[m] = cosf(ang) + sinf(ang) * I;
    }
    s_fft_twiddles_inited = true;
}

// FFT for the channelizer (M=64). On target we hand off to esp-dsp's
// PIE/LP-setup-accelerated kernel; on host we keep the hand-rolled
// radix-2 (no esp-dsp dependency for unit tests).
static void fft_64(float complex *x)
{
#ifdef ESP_PLATFORM
    // dsps_fft2r_fc32_arp4 operates on interleaved float pairs
    // (Re, Im, Re, Im, ...) which is exactly the layout of
    // float complex on glibc/gcc; cast through (float *). The
    // routine wraps esp.lp.setup zero-overhead loop + scalar
    // fmadd.s — the "arp4" suffix is loop-overhead removal, NOT
    // PIE vectorisation (P4 PIE is integer-only; float FFT
    // remains scalar there). Bit reversal is a separate ANSI
    // call; the PIE/LP machinery doesn't apply to it.
    if (s_dsps_fft64_fc32_inited) {
        dsps_fft2r_fc32_arp4((float *)x, 64);
        dsps_bit_rev_fc32_ansi((float *)x, 64);
        return;
    }
    // Fall through to the hand-rolled implementation if init
    // failed for any reason.
#endif
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
    // Cooley-Tukey radix-2 with cached twiddles.
    for (int stride = 1; stride < 64; stride <<= 1) {
        int half = stride;
        int span = stride << 1;
        int step = 32 / half;       // table index multiplier for this stage
        for (int k = 0; k < half; k++) {
            float complex w = s_fft_twiddles[k * step];
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

// Quantise the float prototype to Q15 int16 for the D20 step 3 path.
// Maximum tap magnitude in the windowed-sinc prototype is bounded by
// the peak of the central lobe (≈ 1/M scaled by the DC-response
// normalisation, so ≈ 1 in float). To leave one bit of headroom for
// the 8-tap MAC accumulation we scale by 16384 (Q14) rather than
// 32768 (Q15); the consumer shifts right by 14 instead of 15. With
// N=8 taps × max(int16) input × max Q14 tap = 8 × 32767 × 16384 ≈
// 4.3e9, the sum just fits int32 (max 2.1e9 unsigned) only if we
// pre-shift each product; safer to accumulate in int64 and shift at
// the end. The scalar reference does that.
static void quantise_q14(const float *src, int16_t *dst, int n)
{
    for (int i = 0; i < n; i++) {
        float v = src[i] * 16384.0f;
        if      (v >  32767.0f) v =  32767.0f;
        else if (v < -32768.0f) v = -32768.0f;
        dst[i] = (int16_t)lrintf(v);
    }
}

polyphase_channelizer_t *polyphase_channelizer_create(uint32_t fs_in_hz)
{
    polyphase_channelizer_t *ch = (polyphase_channelizer_t *)
                                    calloc(1, sizeof(*ch));
    if (!ch) return NULL;
    ch->fs_in_hz = fs_in_hz;
    build_prototype(ch->h_phase);
    // D20 step 3: pre-compute Q14 (not Q15 — see quantise_q14 comment)
    // taps for the int16 path. Cheap one-shot at create() time.
    quantise_q14(ch->h_phase, ch->h_phase_q15, POLYCHAN_FILTER_LEN);
    ch->dl_head = 0;
    ch->dl_head_int16 = 0;
    init_fft_twiddles();
#ifdef ESP_PLATFORM
    init_dsps_fft64_once();
#endif
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

// TODO(D20): primary hot path. Inner loop is float complex MAC over
// N taps × M phases × n_cycles per call (= ~80 MFLOPS at 2.56 MSPS,
// per the D20 cost table in iridium-acars-implementation-plan.md).
// Target: Q15 taps + int16 IQ + esp.vmulas.s16.qacc (int64 accumulator),
// pattern of dsp_window_arp4.S / dsp_mag_arp4.S. Stop gate: measure on
// real P4 with live SDR feed first; only optimise if Core 0 headroom
// drops below 40%.
size_t polyphase_channelizer_process(polyphase_channelizer_t *ch,
                                     const float complex *input,
                                     size_t n_input,
                                     float complex *out_block)
{
    if (!ch || !input || !out_block) return 0;
    const int M = POLYCHAN_M;
    const int N = POLYCHAN_N_TAPS_PER_PHASE;
    // D20-step1: N must be a power of two for the `& (N-1)` index
    // mask below. The current N=8 satisfies this; if N ever changes,
    // either keep it a power of two or revert to `% N`.
    _Static_assert((POLYCHAN_N_TAPS_PER_PHASE &
                    (POLYCHAN_N_TAPS_PER_PHASE - 1)) == 0,
                   "POLYCHAN_N_TAPS_PER_PHASE must be a power of two");
    const int N_MASK = N - 1;
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
        int new_head = (ch->dl_head - 1 + N) & N_MASK;
        for (int p = 0; p < M; p++) {
            ch->dl[p * N + new_head] = in[p];
        }
        ch->dl_head = new_head;

        // Compute filtered output per phase: y_p = Σ h_phase[p,n] * dl[p,head+n]
        // The delay line is read newest-first (head, head+1, head+2, ...
        // wrapping mod N). Coefficient h_phase[p, 0] multiplies the newest
        // sample; h_phase[p, N-1] multiplies the oldest.
        //
        // D20-step1: explicit unroll for the fixed N=8 case. `& N_MASK`
        // instead of `% N` (N is power of two, asserted above).
        // The complex MAC y += h*x is two scalar fmadd.s on the P4
        // (real and imag parts independently). With this layout the
        // compiler at -O2 emits 8 fmadd.s pairs + 8 loads per phase
        // without trashing registers, no branch overhead.
        const int head = ch->dl_head;
        for (int p = 0; p < M; p++) {
            const float *hp = &ch->h_phase[p * N];
            const float complex *dlp = &ch->dl[p * N];
            float complex acc =
                  hp[0] * dlp[(head + 0) & N_MASK]
                + hp[1] * dlp[(head + 1) & N_MASK]
                + hp[2] * dlp[(head + 2) & N_MASK]
                + hp[3] * dlp[(head + 3) & N_MASK]
                + hp[4] * dlp[(head + 4) & N_MASK]
                + hp[5] * dlp[(head + 5) & N_MASK]
                + hp[6] * dlp[(head + 6) & N_MASK]
                + hp[7] * dlp[(head + 7) & N_MASK];
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

// D20 step 3: scalar int16/Q14 reference implementation of the
// channelizer process. Numerically equivalent to the float path
// within Q14 quantisation noise. On target uses esp-dsp's sc16
// FFT (dsps_fft2r_sc16_arp4, PIE-vectorised int16). On host falls
// back to converting per-phase MAC output through fft_64 — this
// keeps the host test harness independent of esp-dsp.
//
// Scaling notes (see also quantise_q14):
//   - Taps in Q14 (16384 = 1.0 in float). Allows the 8-tap MAC sum
//     to fit int32 without overflow before the shift, and matches
//     the int64 accumulator pattern the eventual PIE kernel will use.
//   - Per-phase output is shift-right-14 from the int64 sum. The
//     float prototype's per-phase sum is 1/M ≈ 0.0156, so a full-
//     scale int16 input produces a per-phase output of ~512. The
//     FFT then mixes 64 such values, peaking at ~M × 512 ≈ 32768
//     for a coherent (DC) input — exactly int16 saturation. For
//     realistic Iridium bursts (energy spread across 1-2 channels,
//     not all 64) the peak channel output is well below saturation.
//     sc16 FFT does internal stage scaling; output is in the same
//     int16 scale as input. If saturation becomes observable in
//     channel-power readings, add an extra shift-right between the
//     MAC and FFT (cost: 6 dB of dynamic range).
size_t polyphase_channelizer_process_int16(polyphase_channelizer_t *ch,
                                            const int16_t *input_iq,
                                            size_t n_input_complex,
                                            int16_t *out_block_iq)
{
    if (!ch || !input_iq || !out_block_iq) return 0;
    const int M = POLYCHAN_M;
    const int N = POLYCHAN_N_TAPS_PER_PHASE;
    _Static_assert((POLYCHAN_N_TAPS_PER_PHASE &
                    (POLYCHAN_N_TAPS_PER_PHASE - 1)) == 0,
                   "POLYCHAN_N_TAPS_PER_PHASE must be a power of two");
    const int N_MASK = N - 1;
    size_t n_cycles = n_input_complex / (size_t)M;

    // FFT scratch — interleaved IQ.
    int16_t fft_buf_i16[POLYCHAN_M * 2];
#ifndef ESP_PLATFORM
    float complex fft_buf_fc32[POLYCHAN_M];
#endif

    for (size_t cycle = 0; cycle < n_cycles; cycle++) {
        // 1. Write M input IQ pairs to delay line at new_head.
        int new_head = (ch->dl_head_int16 - 1 + N) & N_MASK;
        const int16_t *in = input_iq + cycle * M * 2;
        for (int p = 0; p < M; p++) {
            ch->dl_int16[p * N * 2 + new_head * 2 + 0] = in[p * 2 + 0];
            ch->dl_int16[p * N * 2 + new_head * 2 + 1] = in[p * 2 + 1];
        }
        ch->dl_head_int16 = new_head;

        // 2. Per-phase MAC: 8 Q14 taps × 8 complex int16 samples.
        //
        // Future asm path: this is the loop polyphase_channelizer_mac_arp4
        // (D20 step 3, see polyphase_mac_arp4.S) replaces. The current
        // scalar reference is the validation baseline for that kernel.
        const int head = ch->dl_head_int16;
        for (int p = 0; p < M; p++) {
            const int16_t *hp  = &ch->h_phase_q15[p * N];
            const int16_t *dlp = &ch->dl_int16[p * N * 2];
            int64_t acc_re = 0;
            int64_t acc_im = 0;
            for (int n = 0; n < N; n++) {
                int slot = (head + n) & N_MASK;
                int32_t tap = (int32_t)hp[n];
                acc_re += (int64_t)tap * (int32_t)dlp[slot * 2 + 0];
                acc_im += (int64_t)tap * (int32_t)dlp[slot * 2 + 1];
            }
            // Q14 → int16 (unscaled). Saturate just in case.
            int32_t re = (int32_t)(acc_re >> 14);
            int32_t im = (int32_t)(acc_im >> 14);
            if      (re >  32767) re =  32767;
            else if (re < -32768) re = -32768;
            if      (im >  32767) im =  32767;
            else if (im < -32768) im = -32768;
            fft_buf_i16[p * 2 + 0] = (int16_t)re;
            fft_buf_i16[p * 2 + 1] = (int16_t)im;
        }

        // 3. FFT.
#ifdef ESP_PLATFORM
        if (s_dsps_fft64_sc16_inited) {
            dsps_fft2r_sc16_arp4(fft_buf_i16, M);
            dsps_bit_rev_sc16_ansi(fft_buf_i16, M);
        }
        // Else: leave the (MAC-only, no-FFT) data through. Tests
        // would fail loudly; production gates on init success.
#else
        // Host fallback: convert int16 → float complex → fft_64 →
        // back to int16 with saturation. Keeps host tests free of
        // esp-dsp.
        for (int p = 0; p < M; p++) {
            fft_buf_fc32[p] = (float)fft_buf_i16[p * 2 + 0]
                            + (float)fft_buf_i16[p * 2 + 1] * I;
        }
        fft_64(fft_buf_fc32);
        for (int k = 0; k < M; k++) {
            int32_t re = (int32_t)lrintf(crealf(fft_buf_fc32[k]));
            int32_t im = (int32_t)lrintf(cimagf(fft_buf_fc32[k]));
            if      (re >  32767) re =  32767;
            else if (re < -32768) re = -32768;
            if      (im >  32767) im =  32767;
            else if (im < -32768) im = -32768;
            fft_buf_i16[k * 2 + 0] = (int16_t)re;
            fft_buf_i16[k * 2 + 1] = (int16_t)im;
        }
#endif

        // 4. Write out the M channels for this cycle (interleaved).
        memcpy(&out_block_iq[cycle * M * 2], fft_buf_i16,
               sizeof(fft_buf_i16));
    }
    return n_cycles;
}
