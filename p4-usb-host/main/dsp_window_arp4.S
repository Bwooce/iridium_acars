// Q15 element-wise vector multiply for the DSP windowing stage.
//
//   out[i] = (int16_t)(((int32_t)src[i] * win[i]) >> 15);   for i in [0, n)
//
// `n` must be a multiple of 8 and ≥ 8 (caller's responsibility — dsp_processor
// always calls with FFT_SIZE*2 = 4096). All three buffers must be 16-byte
// aligned (the static `fft_in`, `window_cplx`, and the per-frame `frame_ptr`
// satisfy this on the existing build).
//
// Each iteration of the inner loop: two 128-bit loads, one esp.vmul.s16
// (8-lane Q15 multiply, hardware-saturating), one 128-bit store. The
// hardware zero-overhead loop (esp.lp.setup) eliminates branch overhead.
//
// Replaces the scalar loop in dsp_processor.c that the compiler at -O3
// emits as ~5 cycles per element (4096 × ~5 = ~20480 cycles ≈ 56 μs at
// 360 MHz, measured 92 μs end-to-end including memory pressure).
// Expected: 1 cycle per 8-element vector iter ≈ 512 cycles ≈ 1.4 μs core,
// realistically 5-15 μs end-to-end after L1 cache + memory fill.

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4

    .text
    .align  4
    .global dsp_window_s16_arp4
    .type   dsp_window_s16_arp4,@function

// void dsp_window_s16_arp4(const int16_t *src, const int16_t *win,
//                          int16_t *dst, int n);
//   a0 = src    a1 = win    a2 = dst    a3 = n
dsp_window_s16_arp4:
    srli    a3, a3, 3                       // a3 = n / 8 = vector iter count
    beqz    a3, .Lwin_done

    esp.lp.setup    0, a3, .Lwin_last
        esp.vld.128.ip      q0, a0, 16      // q0 = src[i..i+7],  src += 16
        esp.vld.128.ip      q1, a1, 16      // q1 = win[i..i+7],  win += 16
        esp.vmul.s16        q2, q0, q1      // q2[k] = sat((q0[k]*q1[k]) >> 15)
.Lwin_last:
        esp.vst.128.ip      q2, a2, 16      // dst[i..i+7] = q2,  dst += 16

.Lwin_done:
    ret

#endif // CONFIG_IDF_TARGET_ESP32P4
