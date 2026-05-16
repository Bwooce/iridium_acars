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

    .align  4
    .global polyphase_mac_all_phases_arp4
    .type   polyphase_mac_all_phases_arp4, @function

// All-phases-at-once variant. Same per-phase math as
// polyphase_mac_phase_arp4 but the per-phase loop runs inside the
// PIE hardware loop instead of as 64 C function calls. Each cycle
// the C caller invokes this exactly once instead of 64 times,
// saving 64 × asm prologue/epilogue (~20 cycles each at 360 MHz =
// ~3.5 us per cycle = ~115 us per 2048-sample frame).
//
// Inputs (RV32 a-class regs are caller-saved, so no register save):
//   a0 = h_phase_q15 (M*N int16 = 1024 bytes total).
//        Phase p occupies bytes [p*16, p*16+15]. Loop advances by 16
//        bytes per iteration via .ip.
//   a1 = &dl_int16[head] (per-phase stride = 4N int16 = 64 bytes).
//        Phase p's Re block starts at &dl_int16[p*32 + head] and is
//        N=8 int16 wide. Loop advances by 64 bytes per iter via .xp.
//   a2 = &dl_int16[2N + head] (per-phase stride = 64 bytes; Im block).
//   a3 = out (M*2 int16 = 256 bytes; 2 int16 = 4 bytes per phase).
//
// The C caller is responsible for having called polyphase_mac_pie_init
// once already (enables unaligned PIE vld so non-zero head values
// don't fault).
polyphase_mac_all_phases_arp4:
    // Save callee-saved s8/s9 (we use them as .xp base registers
    // because the PIE encoding for esp.vld.128.xp restricts the
    // base to the upper-half register file — matrix_mult uses the
    // same x24/x25 pair for the same reason).
    addi    sp, sp, -16
    sw      s8, 0(sp)
    sw      s9, 4(sp)

    mv      s8, a1               // s8 = re_ptr (will be advanced by .xp)
    mv      s9, a2               // s9 = im_ptr (will be advanced by .xp)
    li      t3, 64               // phase stride for re/im (bytes)
    li      t6, 14               // Q14 → int16 shift
    li      t1, 64               // phase loop count (= POLYCHAN_M)

    esp.lp.setup    0, t1, .Lpoly_loop_end
        esp.vld.128.ip   q0, a0, 16    // q0 = 8 taps,  h += 16
        esp.vld.128.xp   q1, s8, t3    // q1 = 8 Re,    re += 64
        esp.vld.128.xp   q2, s9, t3    // q2 = 8 Im,    im += 64

        esp.zero.xacc
        esp.vmulas.s16.xacc  q0, q1
        esp.srs.s.xacc       t5, t6
        sh      t5, 0(a3)

        esp.zero.xacc
        esp.vmulas.s16.xacc  q0, q2
        esp.srs.s.xacc       t5, t6
        sh      t5, 2(a3)
.Lpoly_loop_end:
        addi    a3, a3, 4

    lw      s8, 0(sp)
    lw      s9, 4(sp)
    addi    sp, sp, 16
    ret
    .size polyphase_mac_all_phases_arp4, .-polyphase_mac_all_phases_arp4

#endif // CONFIG_IDF_TARGET_ESP32P4
