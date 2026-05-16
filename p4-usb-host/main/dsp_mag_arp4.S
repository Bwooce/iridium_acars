// TODO(D20): this PIE kernel is currently unused (the legacy single-FFT
// detector it served was replaced by channelizer_detector in D7 part 2).
// Kept in-tree as a documented PIE template — specifically as the
// reference design for the per-channel power computation in
// channelizer_detector.c's process_cycles (|y|² = r² + i², same
// kernel shape). See iridium-acars-implementation-plan.md § D20.
//
// PIE int magnitude-squared kernel — deinterleave-first variant.
//
// Key insight: instead of letting qacc lanes hold separate r² and i²
// (forcing pair-sum at the consumer), deinterleave the input into
// q_re = [r0, r1, r2, r3, ...] and q_im = [i0, i1, i2, i3, ...] FIRST,
// then accumulate r² + i² directly into qacc lanes 0-3 via two
// vmulas instructions. The magnitude is already computed inside qacc
// — no consumer-side pair-sum needed.
//
// Per iter (4 complex samples):
//   1 vld          (load 8 int16)
//   2 notq         (copy q0→q1 for distinct-reg vunzip)
//   1 vunzip.16    (deinterleave even/odd 16-bit lanes)
//   1 zero.qacc
//   2 vmulas.s16.qacc  (re² + i² accumulated)
//   2 st.qacc      (lanes 0-1 then lanes 2-3, 32 bytes total)
//
// Output: 4 magnitudes in 32 bytes scratch (8 int32 slots, of which 4
// hold mag values at offsets 0,2,4,6; the int64 high32 slots 1,3,5,7
// are zeros). Scratch consumer reads at offsets [i*2] for n_complex
// magnitudes. Compared to the previous 4-quadrant version: HALF the
// scratch bandwidth (32 vs 64 bytes per iter; 16 KB vs 32 KB total
// for FFT_SIZE = 2048).
//
// Outcome: this kernel itself is faster than scalar (mag stage 39 us
// → 32 us, -18%), but the scratch buffer (16 KB) competes with the
// downstream EMA stage's working set (~24 KB: baseline + magnitudes
// + locals) for L1 D-cache (32 KB total). EMA cost rises from 119 us
// (scalar) to ~370 us (PIE), wiping out the kernel savings. Scalar
// stays in production. The kernel is preserved here as a documented
// reference of the working PIE recipe.
//
// Xhwlp (hardware zero-overhead loop) note: `esp.lp.setup` programs
// CSRs 0x7C6 (LPSTART), 0x7C7 (LPEND), 0x7C8 (LPCOUNT). The vanilla
// AndesCore D45 datasheet cautions against ending the loop body with
// a branch or coprocessor (PIE) instruction. **This caveat does NOT
// apply to ESP32-P4** — every kernel in esp-dsp / esp-nn / esp-dl
// ends its lp.setup body with a PIE store (`esp.vst.128.ip`,
// `esp.st.qacc.*`), our own dsp_window_arp4.S has shipped reliably
// with this pattern, and the deinterleave kernel here works
// correctly with `esp.st.qacc.l.h.128.ip` as the last instruction.
// Espressif's ACE customization on top of AndesCore N310/N308
// handles PIE at loop boundaries cleanly.

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4

    .text
    .align  4
    .global dsp_mag_sq_lanes_s16_arp4
    .type   dsp_mag_sq_lanes_s16_arp4,@function

// void dsp_mag_sq_lanes_s16_arp4(const int16_t *src_complex,
//                                int32_t *scratch, int n_complex);
// Writes 8 int32 slots per iter (4 int64 lanes). Caller reads at
// offset [i*2] for the i'th magnitude.
dsp_mag_sq_lanes_s16_arp4:
    srli    a2, a2, 2                    // n_complex / 4 = vector iter count
    beqz    a2, .Lmag_done

    mv      t3, a0
    mv      t4, a1

    esp.lp.setup    0, a2, .Lmag_last
        esp.vld.128.ip       q0, t3, 16    // q0 = [r0,i0,r1,i1,r2,i2,r3,i3]
        esp.notq             q1, q0
        esp.notq             q1, q1        // q1 = q0 (distinct-reg copy)
        esp.vunzip.16        q0, q1        // q0=[r0,r1,r2,r3,...], q1=[i0,i1,i2,i3,...]
        esp.zero.qacc                       // qacc = 0
        esp.vmulas.s16.qacc  q0, q0         // qacc[0..3] += r0²..r3²
        esp.vmulas.s16.qacc  q1, q1         // qacc[0..3] += i0²..i3² → magnitudes
        esp.st.qacc.l.l.128.ip t4, 16       // store lanes 0,1 (2 mags + 2 zeros)
.Lmag_last:
        esp.st.qacc.l.h.128.ip t4, 16       // store lanes 2,3 (2 mags + 2 zeros)

.Lmag_done:
    ret

#endif
