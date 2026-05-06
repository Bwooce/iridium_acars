#ifndef DSP_WINDOW_ARP4_H
#define DSP_WINDOW_ARP4_H

#include <stdint.h>
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_IDF_TARGET_ESP32P4

// PIE-vectorised Q15 windowing kernel. See dsp_window_arp4.S.
// `n` must be a multiple of 8 and ≥ 8; all three pointers must be 16-byte
// aligned. No bounds checks at runtime.
void dsp_window_s16_arp4(const int16_t *src, const int16_t *win,
                         int16_t *dst, int n);

static inline void dsp_window_s16(const int16_t *src, const int16_t *win,
                                  int16_t *dst, int n)
{
    dsp_window_s16_arp4(src, win, dst, n);
}

#else  // not ESP32-P4 — fall back to the scalar reference

static inline void dsp_window_s16(const int16_t *src, const int16_t *win,
                                  int16_t *dst, int n)
{
    for (int i = 0; i < n; i++) {
        dst[i] = (int16_t)(((int32_t)src[i] * win[i]) >> 15);
    }
}

#endif

#ifdef __cplusplus
}
#endif

#endif
