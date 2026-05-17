#ifndef POLYPHASE_CHANNELIZER_H
#define POLYPHASE_CHANNELIZER_H

// Polyphase FFT channelizer. Splits an input complex sample stream
// into M parallel channel streams, each at fs_in / M sample rate.
//
// Architecture (critically-sampled, M=D):
//   Input: complex samples at fs_in (e.g. 2.56 MSPS).
//   Internal: per-phase circular delay lines + an M-point FFT.
//   Output: M complex sample streams, one per channel, each at fs_in/M
//           (e.g. 40 kHz). Channel k's centre frequency in the original
//           input spectrum is k * fs_in/M (modulo fs_in, with k > M/2
//           wrapping to negative).
//
// The channelizer uses a Hamming-windowed-sinc prototype low-pass filter
// of length M * N_taps_per_phase. A short filter (N=8) gives ~40 dB
// adjacent-channel rejection — adequate for Iridium where the closest
// neighbour at 41.667 kHz spacing carries unrelated TDMA traffic.
//
// CPU: ~M*N + M*log(M) ops per output cycle (one cycle per M input
// samples). For M=64, N=8 at 2.56 MSPS input → 40 kHz × 896 ops/cycle
// ≈ 36 Mops/s, ~10% of one P4 core at 360 MHz.

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <complex.h>

#ifdef __cplusplus
extern "C" {
#endif

// We hard-code M=64 for the first implementation; can be templated later
// if we want different channel counts. M MUST be a power of two for the
// internal FFT.
//
// N_TAPS_PER_PHASE = 16 (was 8): bumps the prototype filter length to
// L = 1024 taps (was 512). Narrows the transition band from ~20 kHz to
// ~10 kHz at fs=2.56 MHz, sharpening adjacent-channel rejection
// without changing the window function (Hamming, ~52 dB stopband).
//
// N=16 (not 12) because polyphase_channelizer.c uses `& (N-1)` indexing
// which requires N to be a power of two. N=12 would need `% N` and
// loses the optimisation. We pay 100% more channelizer MAC ops vs N=8
// for cleaner adjacent-channel separation.
//
// This exceeds gr-iridium's PFB tap budget (~300 max per their warning
// in iridium_extractor_flowgraph.py) — they accept wider transition
// because they use the FFT channelizer for high-decim configs like ours
// (`fft_channelizer_impl.cc`, no per-channel filter beyond rect-window
// FFT bins). Our polyphase + tighter filter is a different trade-off.
#define POLYCHAN_M               64
#define POLYCHAN_N_TAPS_PER_PHASE 16
#define POLYCHAN_FILTER_LEN      (POLYCHAN_M * POLYCHAN_N_TAPS_PER_PHASE)

typedef struct polyphase_channelizer polyphase_channelizer_t;

// Allocate a channelizer with a Hamming-windowed-sinc prototype filter.
// `fs_in_hz` is the input sample rate (informational only — used for
// computing channel center frequencies via polyphase_channelizer_channel_freq).
polyphase_channelizer_t *polyphase_channelizer_create(uint32_t fs_in_hz);
void                     polyphase_channelizer_destroy(polyphase_channelizer_t *ch);

// Process one block of n_input complex samples. Writes one row of
// channel outputs (M complex samples) to `out_block[0..M-1]` for each
// completed M-input cycle. `out_block` must have room for at least
// (n_input / M) * M complex outputs (i.e. one full row per cycle).
// Returns the number of complete output rows produced.
//
// Channel k's output is at out_block[row*M + k]. After a fftshift-style
// remap (if you want bin 0 = DC, bin M/2 = Nyquist), apply the
// polyphase_channelizer_channel_freq() helper.
//
// n_input must be a multiple of M for the simple implementation; non-
// multiples are silently truncated.
size_t polyphase_channelizer_process(polyphase_channelizer_t *ch,
                                     const float complex *input,
                                     size_t n_input,
                                     float complex *out_block);

// D20 step 3: int16 IQ path. Same semantics as polyphase_channelizer_process
// but the entire data path is integer (Q14 taps, int16 IQ delay line,
// int64 MAC accumulator, sc16 FFT on target). Saves the int16→float
// conversion the float caller would otherwise do, and is the entry
// point for the future PIE-asm MAC kernel (D20 step 3 sub-task 4).
//
// input_iq is interleaved I,Q,I,Q,... of length 2*n_input_complex int16.
// out_block_iq is interleaved I,Q,... per channel per cycle, total
// (n_input_complex / POLYCHAN_M) * POLYCHAN_M * 2 int16. Returns the
// number of complete cycles produced (= n_input_complex / POLYCHAN_M).
//
// Maintains its own delay-line state separate from the float path —
// the two functions don't share input history. Don't interleave
// calls to both on the same instance.
size_t polyphase_channelizer_process_int16(polyphase_channelizer_t *ch,
                                            const int16_t *input_iq,
                                            size_t n_input_complex,
                                            int16_t *out_block_iq);

// Returns the centre frequency (Hz, signed) of channel index `k` in
// the input spectrum. k=0 → DC, k=M/2 → fs_in/2 = Nyquist, k>M/2 → negative.
int32_t polyphase_channelizer_channel_freq(const polyphase_channelizer_t *ch,
                                           int k);

// For inspection: copy out the prototype filter coefficients (M*N_taps
// floats). Caller passes a buffer of POLYCHAN_FILTER_LEN floats.
void polyphase_channelizer_get_prototype(const polyphase_channelizer_t *ch,
                                         float *out, size_t out_len);

#ifdef __cplusplus
}
#endif

#endif // POLYPHASE_CHANNELIZER_H
