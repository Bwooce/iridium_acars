// poa_mix_arp4.S — ESP32-P4 PIE Q15 complex integrate-dump for the POA
// channelizer. Self-gates on __riscv && SOC_CPU_HAS_PIE so it compiles to
// nothing on host builds (which run the ANSI-C reference in poa_frontend.c).
//
// Patterns from resample_arp4.S (xacc Q15 MAC) and rotate_to_dc_arp4.S (Q15
// complex mul). All input buffers MUST be internal DRAM, 16-byte aligned, and
// zero-padded to a multiple of 8 int16 with an extra read-ahead vector — see
// project_heap_position_decode_bug and the poa_frontend.c allocator.
//
// The accumulator xacc is a 40-BIT signed accumulator on this silicon (probed
// 2026-08-03: esp.movx.r.xacc.h returns bits[39:32] zero-extended; a large
// negative sum reads back as (2^40 + sum) unless sign-extended from bit 39).
// The kernel sign-extends in-place and writes a proper little-endian int64.
// Max |D| here is ~4.3e11 (400 terms x 2^30) < 2^39 (5.5e11): no overflow, so
// the 40-bit result is bit-identical to the int64 C reference.

#include "soc/soc_caps.h"

#if defined(__riscv) && defined(SOC_CPU_HAS_PIE)

    .text
    .align  4
    .global poa_mix_q15_arp4
    .type   poa_mix_q15_arp4, @function

    .section .rodata
    .align  3
.poa_zero8:
    .long   0
    .long   0

    .text

// void poa_mix_q15_arp4(const int16_t *xr, const int16_t *xi,
//                       const int16_t *wre, const int16_t *wim,
//                       const int16_t *negwim, int nvec,
//                       int64_t *Dre, int64_t *Dim)
//
//   Dre = sum(xr*wre)  + sum(xi*negwim)      over nvec*8 samples
//   Dim = sum(xr*wim)  + sum(xi*wre)
//
// (negwim = -wim precomputed so the subtraction folds into a MAC.) All five
// int16 buffers are 16-byte aligned and zero-padded to nvec*8 (+ read-ahead).
//   a0=xr a1=xi a2=wre a3=wim a4=negwim a5=nvec a6=*Dre a7=*Dim
// PIE vld/ld.xacc base registers must be upper-half-encodable (a0-a7 or
// t3-t6); x5-x7 (t0-t2) are rejected. So the four advancing vector pointers
// live in t3-t6 and the loop counter (a plain addi/bnez, not a vld base) in
// t0. Caller-saved regs only — no prologue.
poa_mix_q15_arp4:
    esp.movx.r.cfg  t6
    ori             t6, t6, 2          // unaligned 128-bit vld (cfg bit 1)
    esp.movx.w.cfg  t6

    // ---------------- Dre = sum(xr*wre) + sum(xi*negwim) ----------------
    la              t6, .poa_zero8
    esp.ld.xacc.ip  t6, 0              // xacc = 0
    mv              t3, a0             // xr working ptr
    mv              t4, a2             // wre working ptr
    mv              t5, a1             // xi working ptr
    mv              t6, a4             // negwim working ptr
    mv              t0, a5             // loop counter = nvec
.Lpoa_dre:
    esp.vld.128.ip  q0, t3, 16
    esp.vld.128.ip  q1, t4, 16
    esp.vmulas.s16.xacc  q0, q1        // xacc += sum(xr * wre)
    esp.vld.128.ip  q0, t5, 16
    esp.vld.128.ip  q1, t6, 16
    esp.vmulas.s16.xacc  q0, q1        // xacc += sum(xi * negwim)
    addi            t0, t0, -1
    bnez            t0, .Lpoa_dre
    esp.movx.r.xacc.l  t3              // t3 = xacc[31:0]
    esp.movx.r.xacc.h  t4              // t4 = xacc[39:32] (zero-extended)
    slli            t4, t4, 24
    srai            t4, t4, 24         // sign-extend from bit 39
    sw              t3, 0(a6)
    sw              t4, 4(a6)          // *Dre (int64, little-endian)

    // ---------------- Dim = sum(xr*wim) + sum(xi*wre) ------------------
    la              t6, .poa_zero8
    esp.ld.xacc.ip  t6, 0
    mv              t3, a0             // xr
    mv              t4, a3             // wim
    mv              t5, a1             // xi
    mv              t6, a2             // wre
    mv              t0, a5
.Lpoa_dim:
    esp.vld.128.ip  q0, t3, 16
    esp.vld.128.ip  q1, t4, 16
    esp.vmulas.s16.xacc  q0, q1        // xacc += sum(xr * wim)
    esp.vld.128.ip  q0, t5, 16
    esp.vld.128.ip  q1, t6, 16
    esp.vmulas.s16.xacc  q0, q1        // xacc += sum(xi * wre)
    addi            t0, t0, -1
    bnez            t0, .Lpoa_dim
    esp.movx.r.xacc.l  t3
    esp.movx.r.xacc.h  t4
    slli            t4, t4, 24
    srai            t4, t4, 24
    sw              t3, 0(a7)
    sw              t4, 4(a7)          // *Dim (int64, little-endian)
    ret

#endif /* __riscv && SOC_CPU_HAS_PIE */
