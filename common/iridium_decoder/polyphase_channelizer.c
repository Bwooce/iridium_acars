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
#include "q14_fixed.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "dsps_fft2r.h"
#include "esp_log.h"
#include "polyphase_mac_arp4.h"
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

    // D20 step 3: Q14 / int16 path with deinterleave-free 2-copy
    // delay line. Layout per phase (4N=32 int16 = 64 bytes):
    //   dl_int16[p * 4N + 0 .. + 2N-1]    = Re slots, 2-copy ring
    //   dl_int16[p * 4N + 2N .. + 4N-1]   = Im slots, 2-copy ring
    // Each cycle writes the new sample at slot[head] AND slot[head+N]
    // (both Re and Im); the asm reads N consecutive int16 from
    // &dl[p*4N + head] (Re) and &dl[p*4N + 2N + head] (Im) — chrono
    // order oldest→newest, no deinterleave needed.
    // The Q14 taps are stored REVERSED relative to h_phase so h[0]
    // multiplies the oldest slot (matching the new read order).
    // 16-byte alignment is for the PIE 128-bit vector loads.
    __attribute__((aligned(16))) int16_t h_phase_q15[POLYCHAN_FILTER_LEN];
    __attribute__((aligned(16))) int16_t dl_int16[POLYCHAN_FILTER_LEN * 4];
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
    // the stop-band. With a Hamming window at L = M*N taps the
    // transition band is ~fs/L; for N=8 that's about half a channel
    // spacing, giving ≥40 dB adjacent rejection.
    //
    // Task #45 attempted Kaiser β=8.6 (~80 dB stopband) here. Measured
    // change vs Hamming: +0.12 dB at 2.56 MHz, -0.22 dB at 2.667 MHz —
    // not a meaningful improvement. The fundamental limit is filter
    // LENGTH (L=512 = N*M with N=8): for L=512 even a perfect Kaiser
    // can't get the transition band below ~20 kHz, so adjacent-channel
    // leakage at 40 kHz can't be tightened much. Real channel-filter
    // improvement requires N=12 or 16 taps/phase (L=768 or 1024) —
    // more memory and PIE asm reflow. Deferred until D20 PIE rewrite
    // makes the longer filter affordable. Hamming retained.
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

// Quantise the float prototype to Q14 int16 with the per-phase tap
// order REVERSED. The new int16 read path (D20 step 3, deinterleave-
// free 2-copy delay line) reads N consecutive samples chronologically
// (oldest → newest); the conventional polyphase tap order is
// newest → oldest (h_phase[p][0] is the tap for the newest sample).
// Reversing the taps per phase at quantisation time means the asm
// can multiply tap n by the n-th read slot without any index gymnastics.
// Q14 not Q15: leaves one bit of headroom for the 8-tap MAC sum.
static void build_q14_taps_reversed(const float *h_phase_float,
                                    int16_t *h_phase_q14_rev)
{
    const int M = POLYCHAN_M;
    const int N = POLYCHAN_N_TAPS_PER_PHASE;
    for (int p = 0; p < M; p++) {
        for (int n = 0; n < N; n++) {
            float v = h_phase_float[p * N + (N - 1 - n)] * (float)Q14_ONE;
            if      (v > (float)INT16_MAX) v = (float)INT16_MAX;
            else if (v < (float)INT16_MIN) v = (float)INT16_MIN;
            h_phase_q14_rev[p * N + n] = (int16_t)lrintf(v);
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
    // D20 step 3: pre-compute Q14 reversed-order taps for the int16
    // deinterleave-free path. Cheap one-shot at create() time.
    build_q14_taps_reversed(ch->h_phase, ch->h_phase_q15);
    ch->dl_head = 0;
    ch->dl_head_int16 = 0;
    init_fft_twiddles();
#ifdef ESP_PLATFORM
    init_dsps_fft64_once();
    polyphase_mac_pie_init();    // enable unaligned PIE vld for 2-copy reads
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
            // N-tap MAC; the loop is `& N_MASK` so the compiler can
            // unroll cleanly when N is a small power of two.
            float complex acc = 0.0f + 0.0f * I;
            for (int n = 0; n < N; n++) {
                acc += hp[n] * dlp[(head + n) & N_MASK];
            }
            fft_buf[p] = acc;
        }

        fft_64(fft_buf);

        // Normalise by 1/M so a DC input concentrates at channel 0 with
        // unity gain (matches gr-iridium's fft_channelizer_impl.cc and
        // the int16 path's dsps_fft2r_sc16 per-stage scaling).
        const float fft_norm = 1.0f / (float)M;
        for (int k = 0; k < M; k++) {
            out_block[cycle * M + k] = fft_buf[k] * fft_norm;
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
        // 1. Write M input IQ pairs to the 2-copy delay line.
        //   Layout: dl[p*4N + 0..2N-1] = Re (2-copy), dl[p*4N + 2N..4N-1] = Im
        //   The slot at index `head` (current oldest) gets overwritten,
        //   then head advances to the next-oldest position. We write
        //   into BOTH slot[head] and slot[head+N] (the 2-copy) so a
        //   linear read of N samples from &dl[head+1] after the
        //   advance gives chronological oldest→newest.
        int wr_head = ch->dl_head_int16;
        const int16_t *in = input_iq + cycle * M * 2;
        for (int p = 0; p < M; p++) {
            int16_t *re_base = &ch->dl_int16[p * (4 * N)];
            int16_t *im_base = re_base + 2 * N;
            int16_t re_val = in[p * 2 + 0];
            int16_t im_val = in[p * 2 + 1];
            re_base[wr_head]     = re_val;
            re_base[wr_head + N] = re_val;
            im_base[wr_head]     = im_val;
            im_base[wr_head + N] = im_val;
        }
        ch->dl_head_int16 = (wr_head + 1) & N_MASK;

        // 2. All-phase MAC. With the 2-copy delay-line layout the
        // per-phase Re/Im pointers advance by a fixed 64-byte stride
        // and the taps by 16 bytes, so the entire 64-phase loop fits
        // inside a single esp.lp.setup hardware loop in
        // polyphase_mac_all_phases_arp4. The C caller invokes the
        // kernel exactly once per cycle, saving 63 of the 64 asm
        // prologue/epilogue costs.
        const int read_head = ch->dl_head_int16;
#if defined(ESP_PLATFORM) && CHANNELIZER_USE_INT16_PATH
        // PIE asm kernel polyphase_mac_all_phases_arp4 is hardcoded for
        // 8 taps per phase (one xacc vld + one vmulas). With N=16 it
        // would need 2 vld + 2 vmulas per accumulator. NOT updated yet;
        // tracked under D20. Guard with a static assert so anyone who
        // re-enables the int16 fast path with N ≠ 8 hits this comment.
        // TODO(D20): update polyphase_mac_arp4.S for N=16 (or make it
        // size-agnostic) and remove this guard.
        _Static_assert(POLYCHAN_N_TAPS_PER_PHASE == 8,
                       "PIE asm kernel hardcoded for 8 taps - see TODO above");
        polyphase_mac_all_phases_arp4(
            ch->h_phase_q15,
            &ch->dl_int16[read_head],          // Re of phase 0 + head
            &ch->dl_int16[2 * N + read_head],  // Im of phase 0 + head
            fft_buf_i16);
#else
        for (int p = 0; p < M; p++) {
            const int16_t *re_ptr = &ch->dl_int16[p * (4 * N) + read_head];
            const int16_t *im_ptr = re_ptr + 2 * N;
            const int16_t *hp     = &ch->h_phase_q15[p * N];
            int64_t acc_re = 0;
            int64_t acc_im = 0;
            for (int n = 0; n < N; n++) {
                int32_t tap = (int32_t)hp[n];
                acc_re += (int64_t)tap * (int32_t)re_ptr[n];
                acc_im += (int64_t)tap * (int32_t)im_ptr[n];
            }
            int32_t re = (int32_t)(acc_re >> Q14_SHIFT);
            int32_t im = (int32_t)(acc_im >> Q14_SHIFT);
            if      (re > INT16_MAX) re = INT16_MAX;
            else if (re < INT16_MIN) re = INT16_MIN;
            if      (im > INT16_MAX) im = INT16_MAX;
            else if (im < INT16_MIN) im = INT16_MIN;
            fft_buf_i16[p * 2 + 0] = (int16_t)re;
            fft_buf_i16[p * 2 + 1] = (int16_t)im;
        }
#endif

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
        // esp-dsp. The fft_64 output is NOT normalised, so we divide
        // by M after to match the target's dsps_fft2r_sc16_arp4
        // behaviour (per-stage right-shift = /N total). Without this
        // the int16 path saturates on any real input.
        for (int p = 0; p < M; p++) {
            fft_buf_fc32[p] = (float)fft_buf_i16[p * 2 + 0]
                            + (float)fft_buf_i16[p * 2 + 1] * I;
        }
        fft_64(fft_buf_fc32);
        const float fft_norm = 1.0f / (float)M;
        for (int k = 0; k < M; k++) {
            int32_t re = (int32_t)lrintf(crealf(fft_buf_fc32[k]) * fft_norm);
            int32_t im = (int32_t)lrintf(cimagf(fft_buf_fc32[k]) * fft_norm);
            if      (re > INT16_MAX) re = INT16_MAX;
            else if (re < INT16_MIN) re = INT16_MIN;
            if      (im > INT16_MAX) im = INT16_MAX;
            else if (im < INT16_MIN) im = INT16_MIN;
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
