// rotate_to_dc_arp4.S — ESP32-P4 PIE 8-lane Q15 complex rotation chunk.
//
// The C wrapper rotate_to_dc_q15_simd_arp4_at() in rotate_to_dc.c
// handles:
//   - per-chunk absolute-phase renorm (cos/sin) and Q15 phasor advance
//   - deinterleave: 8 IQ pairs in caller's iq[16] -> I[8], Q[8] scratch
//   - interleave back: new_I[8], new_Q[8] -> iq[16]
// and calls THIS function once per chunk to do the SIMD multiply.
//
// Inputs (all aligned to 16 bytes, 8 int16 each):
//   a0 - I[8]    real components of the chunk
//   a1 - Q[8]    imag components of the chunk
//   a2 - cs[8]   Q15 cosine values per lane
//   a3 - ss[8]   Q15 sine values per lane
//   a4 - nI[8]   output new_I = I*cs - Q*ss (Q15 saturating)
//   a5 - nQ[8]   output new_Q = I*ss + Q*cs (Q15 saturating)
//
// Q15 semantics: SAR is set to 15 so esp.vmul.s16 does
// (a*b + 0x4000) >> 15 with int16 saturation. esp.vsub.s16 and
// esp.vadd.s16 are saturating int16 adds.

#include "soc/soc_caps.h"

#if defined(__riscv) && defined(SOC_CPU_HAS_PIE)

    .text
    .align  4
    .global rotate_q15_chunk_arp4
    .type   rotate_q15_chunk_arp4, @function

rotate_q15_chunk_arp4:
    // Enable unaligned 128-bit vld (persistent cfg bit). The caller's
    // arrays come from a stack int16_t[8] which the compiler aligns
    // to 2 bytes; PIE vld.128 defaults to 16-byte alignment. The cfg
    // bit relaxes that. Per the polyphase_mac_arp4.S precedent we set
    // it per call rather than relying on a one-shot init at boot.
    esp.movx.r.cfg  t6
    ori             t6, t6, 2
    esp.movx.w.cfg  t6

    // SAR=15 -> esp.vmul.s16 emits (a*b) >> 15 per lane with int16
    // saturation. Matches the scalar reference's q15_mul_round.
    li              t6, 15
    esp.movx.w.sar  t6

    // Load operands (16-byte each = 8 int16).
    esp.vld.128.ip  q0, a0, 0     // q0 <- I[0..7]
    esp.vld.128.ip  q1, a1, 0     // q1 <- Q[0..7]
    esp.vld.128.ip  q2, a2, 0     // q2 <- cs[0..7]
    esp.vld.128.ip  q3, a3, 0     // q3 <- ss[0..7]

    // new_I = (I*cs - Q*ss) per lane, Q15 saturating.
    esp.vmul.s16    q4, q0, q2    // q4[k] = (I[k] * cs[k]) >> 15
    esp.vmul.s16    q5, q1, q3    // q5[k] = (Q[k] * ss[k]) >> 15
    esp.vsub.s16    q6, q4, q5    // q6 = new_I (saturating)

    // new_Q = (I*ss + Q*cs) per lane.
    esp.vmul.s16    q4, q0, q3    // q4[k] = (I[k] * ss[k]) >> 15
    esp.vmul.s16    q5, q1, q2    // q5[k] = (Q[k] * cs[k]) >> 15
    esp.vadd.s16    q7, q4, q5    // q7 = new_Q

    // Store.
    esp.vst.128.ip  q6, a4, 0
    esp.vst.128.ip  q7, a5, 0
    ret

#endif /* __riscv && SOC_CPU_HAS_PIE */
