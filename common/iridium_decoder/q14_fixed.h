// Shared Q14 fixed-point conventions for the int16 DSP path.
//
// Q14 (1.14 signed) is used for filter taps in the channelizer and
// uw_correlator: float taps in [-1,1] are scaled by Q14_ONE and stored
// as int16. A Q14 tap × int16 sample produces an int32 product; sum a
// burst's worth, then >> Q14_SHIFT to undo the scale and return to
// signal-scale int16.
//
// We pick Q14 (not Q15) because DC-gain-normalised taps for a
// 1024-tap polyphase filter sum to 1.0 with peak ~0.05; Q15 would
// give peak tap ~1638 with no margin for accumulation, but Q14 has
// a comfortable 2× headroom and matches the ESP32-P4 PIE vector
// MAC's natural fractional width (the SRS instruction shifts by 14
// after accumulation).

#pragma once

#include <stdint.h>

#define Q14_ONE         16384   // 1.0 in Q14
#define Q14_SHIFT       14      // bits to shift right after Q14 MAC
