// D20 step 3 — PIE polyphase MAC kernel for ESP32-P4 (arp4).
//
// Per call: one phase's 8-tap complex MAC.
//   acc_re = sum over n=0..7 of h[n] * re[n]      (Q14 × int16)
//   acc_im = sum over n=0..7 of h[n] * im[n]
//   out[0] = (acc_re >> 14) saturated to int16
//   out[1] = (acc_im >> 14) saturated to int16
//
// h, re, im pointers must be 16-byte aligned (PIE vld requirement).
// The caller in polyphase_channelizer_process_int16 deinterleaves
// the per-phase delay line into aligned stack buffers re_buf[8] /
// im_buf[8] in head-rotated order (newest sample first), then calls
// this for each phase p ∈ [0..M).
//
// Strategy: use the scalar `xacc` accumulator (single 64-bit lane,
// vs `qacc` which has 4 int64 lanes that we'd have to sum scalar-
// side). esp-dsp's dsps_dotprod_s16_arp4 uses the same trio:
//   esp.zero.xacc                — clear accumulator
//   esp.vmulas.s16.xacc q_a, q_b — xacc += sum_{i=0..7}(q_a[i]*q_b[i])
//   esp.srs.s.xacc rd, rsh       — rd = (xacc >> rsh) saturated to int16
// For a single 8-element MAC + saturate that's literally three PIE
// instructions per accumulator. We do it twice — once for Re,
// once for Im — sharing the same q0 = taps.
//
// Register usage:
//   a0 = h (16-byte aligned, 8 int16)
//   a1 = re (16-byte aligned, 8 int16)
//   a2 = im (16-byte aligned, 8 int16)
//   a3 = out (4 bytes, [Re, Im] int16 pair)
//   q0..q2 = h, re, im (loaded once)
//   t1 = shift result
//   t2 = shift count (= 14)

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32P4

    .text
    .align  4
    .global polyphase_mac_pie_init
    .type   polyphase_mac_pie_init, @function

// One-shot: enable unaligned PIE vector loads. Called once from
// polyphase_channelizer_create() on target so the kernel below
// can read &dl[head] for head ∈ [0, N) without faulting on the
// non-16-byte-aligned head values. The same cfg-bit-1 pattern is
// used in esp-dsp's dsps_dotprod_s16_arp4 for the same reason.
polyphase_mac_pie_init:
    esp.movx.r.cfg  t5
    ori             t5, t5, 2
    esp.movx.w.cfg  t5
    ret
    .size polyphase_mac_pie_init, .-polyphase_mac_pie_init

    .align  4
    .global polyphase_mac_phase_arp4
    .type   polyphase_mac_phase_arp4, @function

polyphase_mac_phase_arp4:
    esp.vld.128.ip  q0, a0, 0    // q0 = [h0..h7]
    esp.vld.128.ip  q1, a1, 0    // q1 = [r0..r7]
    esp.vld.128.ip  q2, a2, 0    // q2 = [i0..i7]

    li      t6, 14               // shift count for Q14 → int16

    // ---- acc_re ----
    esp.zero.xacc
    esp.vmulas.s16.xacc  q0, q1
    esp.srs.s.xacc       t5, t6
    sh      t5, 0(a3)

    // ---- acc_im ----
    esp.zero.xacc
    esp.vmulas.s16.xacc  q0, q2
    esp.srs.s.xacc       t5, t6
    sh      t5, 2(a3)

    ret

    .size polyphase_mac_phase_arp4, .-polyphase_mac_phase_arp4

#endif // CONFIG_IDF_TARGET_ESP32P4
