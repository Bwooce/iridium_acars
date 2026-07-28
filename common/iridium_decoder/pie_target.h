#pragma once
// PIE (P4 SIMD) target-revision seam.
//
// The ESP32-P4 <v3 (rev v0.x/v1.x engineering samples) and >=v3 silicon
// families are mutually exclusive and have "huge hardware difference"
// (IDF Kconfig.hw_support). The PIE coprocessor / lazy-save behaviour is
// exactly the area expected to need per-family handling — e.g. <v3 carries
// the SOC_CPU_HAS_HWLOOP_STATE_BUG workaround that stopped the ~1-in-600
// PIE-trap panics in dsps_fird_s16_arp4 (2026-07-06); v3 is expected to fix
// it in silicon. See docs/2026-07-28-p4-cpu-revision-variants.md.
//
// This is the SINGLE switch a future v3 PIE variant guards on. The build
// framework (scripts/build.sh --rev, sdkconfig.rev_v3_*.defaults) already
// produces the correct CONFIG_ESP32P4_REV_MIN_FULL; this header just gives the
// PIE asm/C one clean, documented macro instead of open-coding the CONFIG.
//
// USAGE (when real v3 silicon arrives and the PIE code must diverge):
//   #include "pie_target.h"
//   #if PIE_TARGET_P4_V3
//       ... v3 PIE variant ...
//   #else
//       ... current v0.x/v1.x path ...
//   #endif
// Do NOT add empty #if PIE_TARGET_P4_V3 branches ahead of real v3 code —
// there is nothing to put behind them yet and an empty branch only bit-rots.
//
// Works in both C and preprocessed asm (.S): both see sdkconfig.h.

#include "sdkconfig.h"

// 1 when building for ESP32-P4 rev >=3.0 (the >=v3 family), 0 for the
// v0.x/v1.x engineering-sample family (the current/default build).
// CONFIG_ESP32P4_REV_MIN_FULL is major*100+minor: 0/1/100 = <v3, 300/301 = v3.x.
#if defined(CONFIG_ESP32P4_REV_MIN_FULL) && CONFIG_ESP32P4_REV_MIN_FULL >= 300
#define PIE_TARGET_P4_V3 1
#else
#define PIE_TARGET_P4_V3 0
#endif
