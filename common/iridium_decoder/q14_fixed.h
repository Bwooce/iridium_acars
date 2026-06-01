// Q14 fixed-point macros, used by uw_correlator.c's RRC matched
// filter. Q14 (not Q15) so the multiply-accumulate has one bit of
// headroom for RRC taps that occasionally exceed |1.0| around the
// pulse-shaping ringing tails.
//
// Previously this header was part of the polyphase channelizer
// module (which used Q14 throughout); the channelizer was removed
// when the wideband fft_burst_tagger took over as the active front
// end, but uw_correlator's RRC filter still uses these macros.
// Kept as a standalone header rather than inlined into uw_correlator.c
// in case other modules adopt Q14 RRC-shaped filtering later.
#pragma once

#define Q14_SHIFT 14
#define Q14_ONE (1 << Q14_SHIFT) // 16384 — represents 1.0 in Q14
