#ifndef DSP_MAG_ARP4_H
#define DSP_MAG_ARP4_H

#include <stdint.h>
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_IDF_TARGET_ESP32P4

// PIE kernel — qacc int64-lane variant. Writes 16 int32 slots per iter
// to scratch (4 int64 qacc lanes × 4 int32 slots). Pair-sum in C reads
// offsets 0 and 2 of each 4-int32 group.
void dsp_mag_sq_lanes_s16_arp4(const int16_t *src_complex,
                                int32_t *scratch_int64lanes,
                                int n_complex);

// Default path: scalar magnitude-squared.
//
// PIE int kernel (dsp_mag_sq_lanes_s16_arp4 in dsp_mag_arp4.S) is
// CORRECT (deinterleave-first variant — mag stage 39 → 32 us, -18%
// on the kernel itself), but it costs more in the next stage's L1
// D-cache pressure than it saves on the kernel. Scalar wins overall
// for our specific workload. See dsp_mag_arp4.S for the four
// variants tried and why none beats scalar net.
static inline void dsp_mag_sq_s16(const int16_t *src_complex,
                                  uint32_t *dst,
                                  int n_complex)
{
    for (int i = 0; i < n_complex; i++) {
        int32_t re = src_complex[2 * i + 0];
        int32_t im = src_complex[2 * i + 1];
        dst[i] = (uint32_t)(re * re + im * im);
    }
}

#else  // not ESP32-P4 — scalar fallback

static inline void dsp_mag_sq_s16(const int16_t *src_complex,
                                  uint32_t *dst,
                                  int n_complex)
{
    for (int i = 0; i < n_complex; i++) {
        int32_t re = src_complex[2 * i + 0];
        int32_t im = src_complex[2 * i + 1];
        dst[i] = (uint32_t)(re * re + im * im);
    }
}

#endif

#ifdef __cplusplus
}
#endif

#endif
