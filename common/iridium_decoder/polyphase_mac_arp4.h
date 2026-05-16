// Public declaration for the PIE/SIMD polyphase MAC kernel
// (D20 step 3). Implementation in polyphase_mac_arp4.S — STUB ONLY
// at present; see that file for the algorithm spec, register map,
// and the sub-task breakdown the next session needs to complete.
//
// This kernel is target-only (uses ESP32-P4 PIE instructions); host
// builds substitute a scalar C reference (TODO: write the reference
// alongside the kernel for bit-equal cross-validation).

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Compute one channelizer-cycle's worth of M=POLYCHAN_M phase outputs.
//
// Inputs:
//   h_phase: M * N_TAPS Q15 int16 taps, contiguous per-phase
//            (h_phase[p * N + n] = tap n of phase p). 16-byte aligned.
//   dl:      M * N complex int16 IQ samples, interleaved
//            (dl[p * N * 2 + 2n + 0] = Re, dl[..+ 1] = Im).
//            The caller is responsible for rotating writes so that
//            sample at position n=0 is the newest. 16-byte aligned.
//   head:    delay-line head [0..N-1]. Ignored by the current stub;
//            the eventual kernel may use it directly or require the
//            caller to pre-rotate (decision deferred — see .S file).
//   out:     M complex int16 Q15 outputs (M*2 int16 total),
//            interleaved (out[p*2+0]=Re, out[p*2+1]=Im).
//
// Sizes are fixed at POLYCHAN_M = 64 phases, N = 8 taps per phase.
// Calling this with the production polyphase_channelizer geometry is
// only valid once the kernel body in polyphase_mac_arp4.S is filled
// in; right now it's a `ret`-only stub.
void polyphase_channelizer_mac_arp4(const int16_t *h_phase,
                                    const int16_t *dl,
                                    int            head,
                                    int16_t       *out);

#ifdef __cplusplus
}
#endif
