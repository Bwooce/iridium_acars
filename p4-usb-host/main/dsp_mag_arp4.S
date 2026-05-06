// PIE int magnitude-squared kernel — REFERENCE / DOCUMENTATION ONLY.
//
// Context for future maintainers
// ------------------------------
// ESP32-P4's "PIE" (the `esp.*` mnemonics in IDF's
// components/esp_gdbstub/test_gdbstub_host/rv_decode/xesppie.S) is
// **Espressif's customization**, distinct from any documented Andes
// extension. Cross-referenced: Andes's public DSP/Vec libraries
// (github.com/andestech/libdsp, libvec, libnn) target standard
// RISC-V P-extension (DSP) and V-extension (RVV) plus their
// AndeStar V5 RVV additions (XAndesVQMac, XAndesVDot, etc.) — NONE
// of those use the q0-q7 / qacc / xacc / `esp.*` namespace that
// ESP32-P4 uses. The xesppie mnemonics appear to be a
// pre-RVV-standardization SIMD that Espressif customized further.
//
// So when looking for reference implementations: check
//   - esp-dsp (modules/.../*_arp4.S)         — DSP primitives
//   - esp-nn  (src/.../*_esp32p4.{c,S})       — neural-net ops
//   - esp-dl  (dl/base/isa/esp32p4/*.S, 50 kernels for ML ops;
//              dl/vision/image/isa/esp32p4/*.S, 33 kernels for
//              image format conversion)
// The Andes libs (despite running on AndesCore silicon) target a
// different ISA path (standard RVV / RV-DSP), not Espressif's PIE.
//
// The closest production analogue to our magnitude-squared is
// esp-dl's dl_esp32p4_s16_mul.S — uses
//   esp.vmulas.s16.qacc.ld.ip q0, a1, 16, q0, q1   ; fused mul+load
//   esp32p4_s16_128b_vector_shift_result q2, a5    ; srcmb output → int16
//   esp.vst.128.ip q2, a0, 16
// — but that pattern saturates the output to int16, which loses
// precision for our int32-fitting squared values. Production
// ML kernels don't need full int32 mag because their downstream
// activation/quantization clips anyway. Our threshold detector
// needs the precision, hence the slower qacc-quadrant-store path.
//
// PIE assembler conventions evolve across IDF versions. Example
// from dl_esp32p4_color_common.S:
//   #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,5,2)
//       esp.vsld.8 qd, qs, q_tmp
//   #else
//       esp.vsld.8 qd, qs, q_tmp, trunc
//   #endif
// — IDF 5.5.2 added an explicit "trunc" operand to vsld.8. So while
// the underlying silicon is fixed, the assembler/operand grammar
// can change, and PIE asm should be checked against the current IDF
// when using older code samples as a starting point.
//
// What's empirically observed about PIE behaviour, not in any
// public spec:
//
// 1. **For `vmulas.s16.qacc`, results land at every-other int32 slot
//    of the qacc store output, with intervening slots = 0.**
//    Empirical observation only: for inputs (-22,-18,-6,-13, ...),
//    a 2-quadrant store (l.l + l.h, 8 int32 slots) produced
//    [484, 0, 324, 0, 36, 0, 169, 0] = [r0², 0, i0², 0, r1², 0, i1², 0].
//    The intervening zeros are consistent with int64 lane spacing
//    (each squared value zero-extended to int64; high32 always 0
//    since our s16²-summed result fits int31), but I have no public
//    Espressif documentation confirming the underlying register
//    structure. The s8 case in esp-nn writes 16 int32 results across
//    all 4 quadrants of qacc, which is also consistent with
//    multiple physical-layout hypotheses.
//    Practical consequence: for 4 complex inputs (8 s16 lanes
//    squared = 8 results), 2 quadrant stores (l.l + l.h, 8 int32
//    slots) give only 4 of the 8 results — the first 4 squared
//    values (r0², i0², r1², i1²) at slots [0, 2, 4, 6]. To extract
//    all 8 results, do all 4 quadrant stores (16 int32 slots);
//    pair-sum reads scratch[i*4] + scratch[i*4 + 2] per complex
//    sample. So per 4-complex iter:
//      - 16 int32 slots written (= 8 squared values × 2 slots
//        each, with high32 zeros)
//      - 8 useful int32 reads per iter
//      - 4 magnitudes computed
//    The 4× write-vs-read amplification is why this kernel is
//    bandwidth-bound and slower than scalar despite the SIMD
//    multiply.
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
