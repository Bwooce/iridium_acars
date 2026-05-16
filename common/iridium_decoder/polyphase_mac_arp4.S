// D20 step 3 — PIE/SIMD polyphase MAC kernel (ESP32-P4, arp4).
//
// STATUS: SCAFFOLD ONLY. Function symbol is exported so the eventual
// caller in polyphase_channelizer.c (target-only int16 path, future
// commit) can link; the implementation body is a stub that returns
// without writing useful data. DO NOT call from production until the
// kernel is filled in AND validated against a host reference.
//
// Why this kernel exists: the polyphase MAC is the dominant compute
// cost in channelizer_detector (~50% of total channelize-detect time
// per smoke-test profile). On P4 scalar float at -O2 with cached
// twiddles (D20 step 1) and esp-dsp's fc32 FFT (D20 step 2), the
// channelizer hits 45% of real-time. Hand-rolled int16 PIE MAC is
// the next big lever; P4 PIE is integer-only (8-lane int16, no
// vector float), so the data path has to go int16 Q15 to use it.
//
// =====================================================================
// ALGORITHM (per cycle, for all M=64 phases):
//   for p in 0..63:
//       y[p] = sum over n=0..7 of h[p,n] * x[p,(head+n) & 7]
//   where h is Q15 int16 (real, length M*N=512)
//         x is interleaved complex int16 IQ (length M*N*2=1024 int16)
//         y is complex int16 Q15 (length M*2=128 int16)
//
// Memory layout (all 16-byte aligned, caller's responsibility):
//   h_phase: 8 Q15 taps per phase, M phases consecutively
//            h_phase[p*8 + n] = tap n of phase p
//   dl:      8 complex IQ samples per phase, M phases consecutively
//            dl[p*16 + 2n + 0] = Re of sample n of phase p
//            dl[p*16 + 2n + 1] = Im of sample n of phase p
//            (interleaved I/Q in delay-line order, NOT in head-order)
//   out:     1 complex output per phase, M phases consecutively
//            out[p*2 + 0] = Re(y[p]),  out[p*2 + 1] = Im(y[p])
//
// `head` rotates [0..7]; the kernel reads dl[p, (head+n) & 7] for
// n=0..7. With N=8 contiguous samples per phase, this is a rotation
// of the read window over the 8-slot buffer.
//
// =====================================================================
// PIE EXECUTION STRATEGY (8 taps × 64 phases → 4 phases in parallel):
//
// One vmulas.s16.qacc takes two 8-lane int16 vectors and accumulates
// pairwise products into the 4 × int64 qacc lanes:
//   qacc[0] += q_a[0]*q_b[0] + q_a[1]*q_b[1]
//   qacc[1] += q_a[2]*q_b[2] + q_a[3]*q_b[3]
//   qacc[2] += q_a[4]*q_b[4] + q_a[5]*q_b[5]
//   qacc[3] += q_a[6]*q_b[6] + q_a[7]*q_b[7]
//
// For one phase, we need a horizontal sum of 8 products → one int32.
// Two options:
//   (a) Pack 4 phases' (taps, samples) into the lanes so each qacc
//       lane accumulates one phase's full 8-product sum. Needs
//       interleaved data layout — phases 0,1,2,3 contribute to
//       lanes 0,1,2,3 simultaneously.
//   (b) Process 1 phase per qacc round, horizontally sum at the end
//       (read out 4 int64, add). Simpler but slower (1/4 throughput).
//
// Plan: (a) — 4 phases in parallel, 16 qacc rounds per cycle, 64 phases
// total. With each round being ~4 PIE ops (2 vld + 2 vmulas) we get
// ~64 PIE ops per cycle = ~10 cycles at 1 op/cycle = ~30 ns per cycle
// on a 360 MHz core. Channelizer cycle rate = 40 ksps → 40k×30ns = 1.2 ms/sec
// = 0.12% of one core. (Compared to ~50% today, that's a 400× speedup
// on this stage alone — close to the theoretical 8× PIE × ~6 from
// removing the per-phase loop overhead.)
//
// Caveats:
//   - Need to deinterleave I/Q from the dl[] buffer into separate Re
//     and Im int16 vectors before MAC (so the same h vector multiplies
//     both). Pattern from dsp_mag_arp4.S (vunzip.16) applies here.
//   - Two vmulas per group of 4 phases: one for Re*h, one for Im*h.
//   - The `head` rotation: we can either (i) rotate reads, requiring
//     index arithmetic per cycle, or (ii) maintain the delay line
//     such that the newest sample is always at slot 0 (rotate writes
//     in C, then asm reads linearly). Choice (ii) keeps asm simple.
//
// =====================================================================
// REGISTER MAP (target — riscv32-esp / arp4):
//   a0 = h_phase pointer (in)
//   a1 = dl pointer (in)
//   a2 = head (in, ignored if delay line is pre-rotated by caller)
//   a3 = out pointer (in)
//   q0..q5 = PIE working vectors (taps, dl_re, dl_im, scratch)
//   qacc   = 4-lane int64 MAC accumulator (zero/load/store via PIE)
//   t0..t3 = scalar scratch (loop counters, phase index)
//
// =====================================================================
// TODO BREAKDOWN (sub-tasks for the next session(s) on D20 step 3):
//   [1] Caller in C: switch polyphase_channelizer to int16 IQ
//       throughout. Add polyphase_channelizer_process_int16 with a
//       Q15 prototype filter (computed from the existing float one
//       at create()). Validate Q15 output against the float path on
//       host (host test, allow some tolerance).
//   [2] Choose delay-line discipline (i) head-rotated reads vs
//       (ii) rotated writes (newest at slot 0). Recommend (ii).
//   [3] Write a scalar C reference of polyphase_channelizer_mac_arp4
//       (`polyphase_mac_arp4_ref`) — 4-phase parallel, all scalar.
//       Validate output bit-for-bit against the unrolled scalar.
//   [4] Convert (3) to PIE asm in this file. Step by step:
//         3a. Bit-reproducing 1-phase-at-a-time PIE (option b above).
//         3b. 4-phase parallel layout with vunzip-based deinterleave.
//         3c. Move all 16 qacc rounds into an esp.lp.setup loop body
//             with no per-iteration RISC-V instructions.
//   [5] Wire up: replace the C MAC loop in polyphase_channelizer.c
//       with a call to polyphase_channelizer_mac_arp4 on target.
//       Keep scalar C path for host (ifdef ESP_PLATFORM).
//   [6] Benchmark on hardware, compare against the 1681 us/frame
//       baseline + D20 step 1+2 numbers. Target: <800 us/frame
//       (real-time) with ≥40% Core 0 headroom.
//
// Until [4] is done, do not link this against a production call site —
// the stub body below returns without writing useful values.
// =====================================================================

#include "sdkconfig.h"

    .text
    .align  4
    .global polyphase_channelizer_mac_arp4
    .type   polyphase_channelizer_mac_arp4, @function

polyphase_channelizer_mac_arp4:
    // TODO(D20 step 3): replace this stub with the kernel body.
    // For now: ret without storing anything so it links cleanly.
    // The C caller is gated on the int16 path which is not wired
    // up yet, so this is unreachable in production.
    ret

    .size polyphase_channelizer_mac_arp4, .-polyphase_channelizer_mac_arp4
