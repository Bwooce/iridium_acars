// PIE int magnitude-squared kernel — REFERENCE / DOCUMENTATION ONLY.
//
// This file documents two real ESP32-P4 PIE discoveries that aren't
// in any public Espressif source:
//
// 1. **qacc holds 8 INT64 lanes**, not 8 int32. After
//    `vmulas.s16.qacc q0, q0` with int16 inputs producing
//    int32-fitting squared products, each qacc lane holds the int32
//    value zero-extended to int64. Extraction via
//    `esp.st.qacc.{l|h}.{l|h}.128.ip` writes 16 bytes = 2 int64
//    lanes = 4 int32 slots. Of those 4 int32 slots, only slots 0 and
//    2 contain meaningful values; slots 1 and 3 are int64 high32 = 0.
//    To read all 8 magnitudes from one iteration's output, the
//    consumer must use offsets `[i*4]` (low32 of int64 lane i) — NOT
//    contiguous int32 indexing.
//
// 2. **`esp.vmul.s32.s16xs16 q_lo, q_hi, q_a, q_b` outputs `(a*b) >> 16`**
//    (Q15.16 fixed-point — the upper 16 bits of the int32 product).
//    The mnemonic name is misleading: it's a Q15-style multiplication,
//    NOT a full-precision s16²→s32 op. Verified empirically: for
//    src=(-2169,-575), q_lo[0] returned 71 ≈ 2169²>>16 = 71.79,
//    matching shift-by-16 truncation.
//
// Combined effect: PIE has no clean direct-output instruction for
// our exact operation (full-precision int16² + int16²). The qacc
// route works correctly but the 2× memory bandwidth from int64-lane
// extraction (16 int32 slots written, only 8 meaningful) makes it
// 21% SLOWER than the scalar reference. The vmul.s32 route is
// faster but loses 16 bits of precision (smoke test SNR drops to
// 19 dB from the expected 50 dB — magnitudes round to zero in the
// noise floor, breaking baseline-EMA tracking).
//
// Patterns that DO work cleanly on P4 PIE:
//   - Dot product (single accumulator, value reduction): use qacc.
//     esp-nn/esp-dl have many examples.
//   - Q15 vector multiply (Q15-input, Q15-output): use
//     `esp.vmul.s32.s16xs16` after pre-shifting inputs into Q15
//     range — the >> 16 implicit shift is exactly the Q15 mantissa.
//   - 8-lane int16 saturating multiply: use `esp.vmul.s16` (see
//     dsp_window_arp4.S).
//
// One earlier hypothesis ("same-reg `vmulas.s16.qacc q0, q0` is
// broken") turned out to be a misdiagnosis. The wrong-output
// symptom was caused by an incorrect C-side pair-sum that read
// offsets 0 and 1 of scratch (assuming int32 lanes) instead of 0
// and 2 (the correct int64 spacing). With the right pair-sum,
// same-reg vmulas produces correct results — every PIE multiply
// in this kernel uses `q0, q0` cleanly.
//
// The kernel is currently UNUSED. dsp_processor.c calls scalar mag.
// Keeping this .S in the repo as documented learning.

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4

    .text
    .align  4

    .global dsp_mag_sq_lanes_s16_arp4
    .type   dsp_mag_sq_lanes_s16_arp4,@function

// void dsp_mag_sq_lanes_s16_arp4(const int16_t *src_complex,
//                                int32_t *scratch, int n_complex);
// Writes 16 int32 slots per iter (4 int64 qacc lanes × 4 int32
// slots, of which 8 are meaningful low32 squared values at offsets
// [i*4 + 0] and [i*4 + 2] within the iter's 16-slot block, and 8
// are int64 high32 zeros at [i*4 + 1] and [i*4 + 3]).
// Caller must pair-sum scratch[i*4] + scratch[i*4 + 2] to get
// the magnitude of complex sample i.
dsp_mag_sq_lanes_s16_arp4:
    srli    a2, a2, 2
    beqz    a2, .Lmag_done

    mv      t3, a0
    mv      t4, a1

    esp.lp.setup    0, a2, .Lmag_last
        esp.vld.128.ip       q0, t3, 16    // 8 int16 (4 complex)
        esp.zero.qacc                       // qacc = 0 (all 8 int64 lanes)
        esp.vmulas.s16.qacc  q0, q0         // qacc[k] = q0[k]² as int64
        esp.st.qacc.l.l.128.ip t4, 16       // qacc lanes 0,1 → 4 int32 slots
        esp.st.qacc.l.h.128.ip t4, 16       // qacc lanes 2,3
        esp.st.qacc.h.l.128.ip t4, 16       // qacc lanes 4,5
.Lmag_last:
        esp.st.qacc.h.h.128.ip t4, 16       // qacc lanes 6,7

.Lmag_done:
    ret

#endif
