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
    .global resample_125_128_enable_pie_cfg
    .type   resample_125_128_mac_arp4, @function
    .type   resample_125_128_enable_pie_cfg, @function

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

// Enable unaligned 128-bit vld (cfg bit 1). Caller must invoke this
// once per process_explicit before entering the MAC loop; the MAC
// kernel below assumes the bit is already set. Hoisting this out
// of the per-emit kernel saves ~3 cycles × ~8000 emits = ~60 µs
// per dispatch.
resample_125_128_enable_pie_cfg:
    esp.movx.r.cfg  t6
    ori             t6, t6, 2
    esp.movx.w.cfg  t6
    ret

// Per-emit MAC kernel. Assumes:
//   - PIE cfg bit 1 (unaligned 128-bit vld) is already set
//     (caller has invoked resample_125_128_enable_pie_cfg)
//   - pc, di, dq buffers are at least 32 bytes accessible from
//     the passed pointer (the second vld goes to base+16)
//
// Uses only caller-saved registers (t0/t3/t5/t6, q0/q1, xacc) so
// there is no prologue/epilogue stack-save dance. srs destination
// is t3 instead of s11 — per the PIE-register-encoding memory note
// the upper-half encoding accepts t3..t6, and unlike s11 it's
// caller-saved.
resample_125_128_mac_arp4:
    li              t5, 15                 // shift constant for srs
    mv              t0, a0                 // save pc base (vld.ip advances a0)

    // ---------- I channel ----------
    la              t6, .resample_arp4_rnd
    esp.ld.xacc.ip  t6, 0                  // xacc = 0x7fff (rounding bias)

    // q0 = pc[0..7], q1 = di[0..7].
    esp.vld.128.ip  q0, a0, 16
    esp.vld.128.ip  q1, a1, 16
    // xacc += sum(q0 * q1).  Fused with load of q0 = pc[8..15].
    esp.vmulas.s16.xacc.ld.ip   q0, a0, 16, q0, q1
    // q1 = di[8..15].
    esp.vld.128.ip  q1, a1, 0
    // xacc += sum(q0 * q1).
    esp.vmulas.s16.xacc         q0, q1
    // result_i = (int16_t)(xacc >> 15), stored to *out_i.
    esp.srs.s.xacc  t3, t5
    sh              t3, 0(a3)

    // ---------- Q channel ----------
    // Reset xacc — t6 still points at .resample_arp4_rnd from above
    // (the post-increment was 0), so no need to re-la.
    esp.ld.xacc.ip  t6, 0
    mv              a0, t0                 // restore pc base

    // q0 = pc[0..7], q1 = dq[0..7].
    esp.vld.128.ip  q0, a0, 16
    esp.vld.128.ip  q1, a2, 16
    esp.vmulas.s16.xacc.ld.ip   q0, a0, 16, q0, q1
    esp.vld.128.ip  q1, a2, 0
    esp.vmulas.s16.xacc         q0, q1
    esp.srs.s.xacc  t3, t5
    sh              t3, 0(a4)

    ret

#endif /* __riscv && SOC_CPU_HAS_PIE */
