// resample_arp4.S — ESP32-P4 PIE 8-lane Q15 polyphase 9-tap MAC.
//
// Used by resample_256_to_250_process()'s inner loop. The C wrapper
// handles the per-input delay-line shift, phase counter, and IQ
// reinterleave; this function does the single hot inner per output:
//
//   acc_i = 0x7fff + sum_{k=0..15}(pc[k] * di[k])   // pc[9..15] = 0
//   acc_q = 0x7fff + sum_{k=0..15}(pc[k] * dq[k])
//   out_i = (int16_t)(acc_i >> 15)
//   out_q = (int16_t)(acc_q >> 15)
//
// The 16-element padding (real 9 taps + 7 zeros) makes the MAC
// exactly two `esp.vmulas.s16.xacc` ops per channel: lanes [0..7]
// then lanes [8..15], where lanes 9..15 contribute 0 because the
// padded taps are zero.
//
// Inputs (all aligned to 16 bytes, 16 int16 each):
//   a0 - pc[16]    per-phase taps; pc[0..8] real, pc[9..15] zero
//   a1 - di[16]    delay-line I (newest at index 0); di[9..15] zero
//   a2 - dq[16]    delay-line Q; dq[9..15] zero
//   a3 - out_i     pointer to int16 to write
//   a4 - out_q     pointer to int16 to write
//
// Output: writes one int16 to *out_i, one int16 to *out_q.

#include "soc/soc_caps.h"

#if defined(__riscv) && defined(SOC_CPU_HAS_PIE)

    .text
    .align  4
    .global resample_125_128_mac_arp4
    .type   resample_125_128_mac_arp4, @function

// Rounding constant 0x7fff loaded into xacc before each MAC sum so
// that the final >>15 acts as round-to-nearest (matches the firmr_s16
// reference's pre-shift `acc += 0x7fff`). 8 bytes -- esp.ld.xacc.ip
// reads 64 bits.
    .section .rodata
    .align  3
.resample_arp4_rnd:
    .long   0x7fff
    .long   0

    .text
resample_125_128_mac_arp4:
    // Save s11 (the esp.srs.s.xacc destination only encodes s8-s11
    // per the PIE register-encoding constraints memory note).
    addi sp, sp, -16
    sw   s11, 0(sp)

    // Enable unaligned 128-bit vld (cfg bit 1). di/dq/pc all come
    // from caller-aligned buffers but the assembler default expects
    // 16-byte alignment; the cfg bit relaxes that universally for
    // safety.
    esp.movx.r.cfg  t6
    ori             t6, t6, 2
    esp.movx.w.cfg  t6

    // Set shift = 15 for esp.srs.s.xacc.
    li              t5, 15

    // Save pc base in t0 so we can reload taps for the Q channel.
    mv              t0, a0

    // ---------- I channel ----------
    // xacc = 0x7fff (rounding bias).
    la              t6, .resample_arp4_rnd
    esp.ld.xacc.ip  t6, 0

    // q0 = pc[0..7], q1 = di[0..7].
    esp.vld.128.ip  q0, a0, 16
    esp.vld.128.ip  q1, a1, 16
    // xacc += sum(q0 * q1).  Fused with load of q0 = pc[8..15].
    esp.vmulas.s16.xacc.ld.ip   q0, a0, 16, q0, q1
    // q1 = di[8..15].
    esp.vld.128.ip  q1, a1, 0
    // xacc += sum(q0 * q1).  (no load needed afterwards.)
    esp.vmulas.s16.xacc         q0, q1
    // result_i = (int32)(xacc >> 15), stored to *out_i.
    esp.srs.s.xacc  s11, t5
    sh              s11, 0(a3)

    // ---------- Q channel ----------
    // Reset xacc, restore pc base.
    la              t6, .resample_arp4_rnd
    esp.ld.xacc.ip  t6, 0
    mv              a0, t0

    // q0 = pc[0..7], q1 = dq[0..7].
    esp.vld.128.ip  q0, a0, 16
    esp.vld.128.ip  q1, a2, 16
    esp.vmulas.s16.xacc.ld.ip   q0, a0, 16, q0, q1
    esp.vld.128.ip  q1, a2, 0
    esp.vmulas.s16.xacc         q0, q1
    esp.srs.s.xacc  s11, t5
    sh              s11, 0(a4)

    lw   s11, 0(sp)
    addi sp, sp, 16
    ret

#endif /* __riscv && SOC_CPU_HAS_PIE */
