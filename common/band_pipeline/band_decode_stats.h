// band_decode_stats — a band-agnostic decode-funnel snapshot, so callers that
// need band-independent success numbers (the autotune gain metric, the
// reception classifier) read ONE accessor instead of branching between the
// Iridium BCH counters (worker_core1) and the VDL2 RS/AVLC counters
// (frame_decoder/vdl2_pipeline). Presentational code that formats band-specific
// field names (the /status + /diag JSON blocks) keeps reading the native
// getters — this is only for consumers that want a single band-agnostic number.
//
// All counts are CUMULATIVE-SINCE-BOOT (like worker_core1_get_bch_cumulative
// and frame_decoder_get_vdl2_stats), so callers snapshot and take deltas.
// Pure header (no ESP-IDF / decoder deps); the accessor lives in main/
// (band_select.c) — the one place allowed to reference both bands' getters.
#pragma once

#include <stdint.h>

typedef struct {
    uint32_t decoded;   // real decoded frames — THE success metric.
                        //   Iridium: BCH-decoded + classified-known (bch_decoded)
                        //   VDL2:    FCS-valid AVLC frames (avlc_ok)
    uint32_t unknown;   // Iridium: BCH-passed but classify=UNKNOWN; VDL2: 0 (n/a)
    uint32_t failed;    // Iridium: BCH uncorrectable; VDL2: FCS-failed (bad_fcs)
    uint32_t recovered; // FEC rescue (a SUBSET of decoded).
                        //   Iridium: Chase-2 soft-BCH; VDL2: RS soft-erasure fallback
} band_decode_stats_t;
