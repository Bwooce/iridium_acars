// fft_burst_tagger_arp4.S — ESP32-P4 PIE 8-lane kernels for the two
// per-bin scalar hot loops in fft_burst_tagger.c:
//
//   1. fbt_mag_sq_pass_arp4  — mag² of the FFT output. Per bin:
//          dst[k] = re[k]*re[k] + im[k]*im[k]   (int16 IQ in, int32 out)
//      The C reference is mag_sq_pass() in fft_burst_tagger.c. Input is
//      INTERLEAVED complex int16 (re,im,re,im,...); output is one int32
//      per bin. Each result is <= 32767² × 2 = 2147352578 < INT32_MAX,
//      so it fits int32 with the sign bit clear.
//
//   2. fbt_ema_step_arp4 / fbt_ema_step_prime_arp4 — the baseline EMA
//      RMW. Per bin, with cur = min(mag[k], clamp):
//          bsum[k] = bsum[k] - slot[k] + cur ; slot[k] = cur   (step)
//          bsum[k] = bsum[k]           + cur ; slot[k] = cur   (prime)
//      The C references are ema_step_inner() / ema_step_inner_prime().
//      All int32. The clamp is FBT_EMA_SLOT_CLAMP = INT32_MAX/HISTORY,
//      passed in a4 so the constant lives with the C code. baseline_sum
//      is proven never to overflow int32 (HISTORY × clamp < INT32_MAX),
//      so saturating-vs-wrapping vadd/vsub is moot — bit-exact vs scalar.
//
// These are drop-in replacements for the bulk (8- resp. 4-bin aligned)
// portion of the scalar loops; the C wrapper handles any tail (N=2048 is
// a multiple of 8 so there is none in practice) and is the sole path on
// host / non-PIE targets, where this whole file compiles to nothing.
//
// The clamp-run tracking (ema_clamp_track / s_clamp_run) is deliberately
// NOT vectorised: it is a rarely-taken PSRAM side-table update kept in C
// and called unchanged after this kernel, so its exact semantics are
// untouched.
//
// Lane / accumulator contract (validate on device — host has no PIE):
//   - esp.vunzip.16 q0,q1 deinterleaves the 16 int16 across (q0:q1) into
//     q0 = even elements (re[0..7]), q1 = odd elements (im[0..7]). mag²
//     is symmetric in re/im, so an even/odd swap would still be correct;
//     what matters is that lane k maps to bin k, which vunzip preserves.
//   - QACC has 8 lanes; esp.vmulas.s16.qacc q,q accumulates q[k]² into
//     lane k. Each lane value is < 2³¹, so its low 32 bits ARE the value:
//     st.qacc.l.l.128 stores lanes 0..3 (low words), st.qacc.h.l.128
//     stores lanes 4..7 — 8 int32 outputs in bin order.
//
// Registers: caller-saved only (t0/t6, q0..q6), no prologue except the
// 16-byte scratch the EMA kernels use to broadcast the clamp constant.

#include "soc/soc_caps.h"

#if defined(__riscv) && defined(SOC_CPU_HAS_PIE)

    .text
    .align  4
    .global fbt_mag_sq_pass_arp4
    .global fbt_ema_step_arp4
    .global fbt_ema_step_prime_arp4
    .type   fbt_mag_sq_pass_arp4,     @function
    .type   fbt_ema_step_arp4,        @function
    .type   fbt_ema_step_prime_arp4,  @function

// void fbt_mag_sq_pass_arp4(const int16_t *src_iq, int32_t *dst, int n)
//   a0 = src_iq (interleaved complex int16, 2*n int16 total)
//   a1 = dst    (n int32)
//   a2 = n      (number of bins; caller guarantees a multiple of 8)
fbt_mag_sq_pass_arp4:
    // Enable unaligned 128-bit vld (persistent cfg bit 1). Caller buffers
    // are int16/int32 arrays with < 16-byte alignment guarantees.
    esp.movx.r.cfg  t6
    ori             t6, t6, 2
    esp.movx.w.cfg  t6

    srli            t0, a2, 3          // t0 = n / 8 blocks
    beqz            t0, .Lmag_done
.Lmag_loop:
    esp.vld.128.ip  q0, a0, 16         // q0 = iq[0..7]  (re0,im0,..,re3,im3)
    esp.vld.128.ip  q1, a0, 16         // q1 = iq[8..15] (re4,im4,..,re7,im7)
    esp.vunzip.16   q0, q1             // q0 = re[0..7], q1 = im[0..7]
    esp.zero.qacc                      // qacc = 0
    esp.vmulas.s16.qacc  q0, q0        // lane[k] += re[k]²
    esp.vmulas.s16.qacc  q1, q1        // lane[k] += im[k]²
    esp.st.qacc.l.l.128.ip  a1, 16     // dst[0..3] = lanes 0..3 (low word)
    esp.st.qacc.h.l.128.ip  a1, 16     // dst[4..7] = lanes 4..7 (low word)
    addi            t0, t0, -1
    bnez            t0, .Lmag_loop
.Lmag_done:
    ret

// void fbt_ema_step_arp4(int32_t *bsum, int32_t *slot,
//                        const int32_t *mag, int n, int32_t clamp)
//   a0 = bsum  (in/out, int32[n])   a1 = slot (in/out, int32[n])
//   a2 = mag   (in,     int32[n])   a3 = n (multiple of 4)   a4 = clamp
fbt_ema_step_arp4:
    esp.movx.r.cfg  t6
    ori             t6, t6, 2
    esp.movx.w.cfg  t6

    addi            sp, sp, -16
    sw              a4, 0(sp)
    // vldbc base must be an upper-half-encodable register (x16..x31); sp
    // (x2) is rejected. Copy it into t3, then broadcast the clamp word.
    mv              t3, sp
    esp.vldbc.32.ip q3, t3, 0          // q3 = clamp broadcast to 4 lanes

    srli            t0, a3, 2          // t0 = n / 4 blocks
    beqz            t0, .Lema_done
.Lema_loop:
    esp.vld.128.ip  q0, a2, 16         // q0 = mag[k..k+3], advance mag
    esp.vmin.s32    q4, q0, q3         // q4 = cur = min(mag, clamp)
    esp.vld.128.ip  q1, a1, 0          // q1 = slot[k..k+3] (old), no advance
    esp.vld.128.ip  q2, a0, 0          // q2 = bsum[k..k+3],       no advance
    esp.vsub.s32    q5, q2, q1         // q5 = bsum - old
    esp.vadd.s32    q6, q5, q4         // q6 = bsum - old + cur
    esp.vst.128.ip  q6, a0, 16         // bsum[k..k+3] = q6, advance bsum
    esp.vst.128.ip  q4, a1, 16         // slot[k..k+3] = cur, advance slot
    addi            t0, t0, -1
    bnez            t0, .Lema_loop
.Lema_done:
    addi            sp, sp, 16
    ret

// void fbt_ema_step_prime_arp4(int32_t *bsum, int32_t *slot,
//                              const int32_t *mag, int n, int32_t clamp)
// Priming variant: accumulate cur WITHOUT reading the (stale) old slot.
fbt_ema_step_prime_arp4:
    esp.movx.r.cfg  t6
    ori             t6, t6, 2
    esp.movx.w.cfg  t6

    addi            sp, sp, -16
    sw              a4, 0(sp)
    // vldbc base must be an upper-half-encodable register (x16..x31); sp
    // (x2) is rejected. Copy it into t3, then broadcast the clamp word.
    mv              t3, sp
    esp.vldbc.32.ip q3, t3, 0          // q3 = clamp broadcast to 4 lanes

    srli            t0, a3, 2          // t0 = n / 4 blocks
    beqz            t0, .Lprime_done
.Lprime_loop:
    esp.vld.128.ip  q0, a2, 16         // q0 = mag[k..k+3], advance mag
    esp.vmin.s32    q4, q0, q3         // q4 = cur = min(mag, clamp)
    esp.vld.128.ip  q2, a0, 0          // q2 = bsum[k..k+3], no advance
    esp.vadd.s32    q6, q2, q4         // q6 = bsum + cur
    esp.vst.128.ip  q6, a0, 16         // bsum[k..k+3] = q6, advance bsum
    esp.vst.128.ip  q4, a1, 16         // slot[k..k+3] = cur, advance slot
    addi            t0, t0, -1
    bnez            t0, .Lprime_loop
.Lprime_done:
    addi            sp, sp, 16
    ret

#endif /* __riscv && SOC_CPU_HAS_PIE */
