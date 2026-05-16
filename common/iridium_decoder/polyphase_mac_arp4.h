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

#ifdef __cplusplus
}
#endif
