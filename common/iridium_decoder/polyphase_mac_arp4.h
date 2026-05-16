// Public declaration for the PIE polyphase MAC kernel
// (D20 step 3). Implementation in polyphase_mac_arp4.S.
//
// Per-phase 8-tap complex MAC:
//   acc_re = sum over n=0..7 of h[n] * re[n]
//   acc_im = sum over n=0..7 of h[n] * im[n]
//   out[0] = (acc_re >> 14) saturated to int16
//   out[1] = (acc_im >> 14) saturated to int16
//
// `h` is 8 Q14 int16 (16-byte aligned). `re` and `im` are 8 int16
// each (must be 16-byte aligned). The caller is responsible for
// deinterleaving the per-phase delay line into the re[]/im[] arrays
// in the head-rotated order (newest sample at index 0). `out` is
// 4 bytes (two int16, [Re, Im]); no alignment requirement.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void polyphase_mac_phase_arp4(const int16_t *h,
                              const int16_t *re,
                              const int16_t *im,
                              int16_t       *out);

// All-phases-at-once kernel. Runs the per-phase MAC loop inside PIE
// hardware-loop hardware (esp.lp.setup) instead of as 64 separate C
// function calls. Same per-phase math as polyphase_mac_phase_arp4.
//
// h: M*N Q14 taps, 16-byte aligned. Phase p occupies bytes [p*16..p*16+15].
// re: pointer to dl_int16[head] for phase 0; per-phase stride 64 bytes.
// im: pointer to dl_int16[2N+head] for phase 0; per-phase stride 64 bytes.
// out: M*2 int16 output (interleaved [Re, Im] per phase).
//
// Requires polyphase_mac_pie_init() to have been called once first.
void polyphase_mac_all_phases_arp4(const int16_t *h,
                                   const int16_t *re,
                                   const int16_t *im,
                                   int16_t       *out);

// One-shot: enable unaligned PIE vector loads. Idempotent. Must be
// called before the first polyphase_mac_phase_arp4 invocation if
// the re/im pointers passed to that kernel may be at non-16-byte
// offsets (which the D20-step-3 2-copy delay line produces for
// head ∈ 1..7).
void polyphase_mac_pie_init(void);

#ifdef __cplusplus
}
#endif
