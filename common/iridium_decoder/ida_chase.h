#ifndef IDA_CHASE_H
#define IDA_CHASE_H

// Chase-2 soft-decision BCH fallback for the LW.DA data section
// (task #16). C port of the validated host prototype
// tests/scripts/chase_crc_bch.py — that script is the REFERENCE; any
// behavioural divergence here is a bug (see tests/host/test_chase_parity.c).
//
// Runs ONLY after the hard path failed: ida_decode() returned 0 but
// out->ok == false (>=1 of the 10 BCH(31,20) codewords was
// uncorrectable at t=2). For each FAILED codeword it:
//   1. computes a per-bit reliability = pair-MIN of the two adjacent
//      DQPSK symbol magnitudes (a symbol slip corrupts the differential
//      bits of symbol s AND s+1, so a bit is only as reliable as the
//      weaker of its two parent symbols) from qpsk_demod's int16
//      soft_bits;
//   2. enumerates all 2^L flip patterns over the L least-reliable
//      codeword positions, hard-syndrome-decodes each
//      (iridium_bch_repair2, t<=2), and collects the DISTINCT candidate
//      20-bit messages;
//   3. combines candidates across failed codewords into whole-frame
//      hypotheses and accepts the FIRST one that passes the production
//      header + CRC-16 arbitration (ida_decode_parse_fields: zero1==0,
//      da_len>0, CRC-16/CCITT-FALSE residue==0), bounded by a per-frame
//      CRC-check cap.
//
// SAFETY MODEL (measured on the 2026-07-17 HydraSDR ground-truth set,
// docs + memory project_demod_at_reference_ceiling_2026_07_17): the
// 16-bit CRC is the sole arbiter, so every CRC check is a ~2^-16
// false-accept lottery ticket. The CHECK CAP — not L — is the safety
// knob: L=5/cap-256 recovered +11 frames with 0 false accepts and 0
// truth mismatches over 568 gold + 825 CRC-fail frames; L=6 admitted a
// wrong-but-CRC-valid frame at ANY cap and is forbidden here.
//
// Everything is integer (int16 soft metrics, uint32 codewords, the
// existing syndrome-table BCH + table CRC-16); no allocation. Uses
// static scratch => single-task use only (matches frame_decoder /
// iridium_bch's single-consumer contract).

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "ida_decode.h"
#include "iridium_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

// Compile-time hard ceiling for L (2^L trial patterns per failed
// codeword). 6 is the storage bound; the SAFE validated operating point
// is 5 — see safety model above.
#define IDA_CHASE_MAX_L 6
// Default operating point (pinned by the host prototype's sweep).
#define IDA_CHASE_DEFAULT_L 5
#define IDA_CHASE_DEFAULT_MAX_CRC 256

// Master enable. Default OFF: the shipped hard-decision decoder is
// bit-identical with the toggle off (ida_chase_decode returns 0 without
// touching anything). Returns the previous value.
bool ida_chase_set_enabled(bool enable);
bool ida_chase_get_enabled(void);

// Tune L (1..IDA_CHASE_MAX_L) and the per-frame CRC-check cap
// (1..4096). Out-of-range values are clamped. NOT thread-safe against a
// concurrent ida_chase_decode.
void ida_chase_set_params(int L, int max_crc_checks);
void ida_chase_get_params(int *L, int *max_crc_checks);

// Attempt Chase-2 recovery of `frame` (must be classified LW.DA).
//   soft_bits — qpsk_demod per-bit soft metrics for the SAME frame
//               (sign=hard decision, magnitude=reliability); n_soft
//               entries. Needs coverage of the whole 382-bit frame.
//   out       — the ida_decoded_t ida_decode() just filled; must have
//               been produced by ida_decode()==0 with out->ok==false.
// Returns:
//    1  recovered: *out fully re-populated exactly as a clean hard
//       decode would be (ok=true, blocks_ok=10, header/payload/CRC
//       fields parsed, crc_ok=true) plus chase_used/chase_checks set.
//    0  no recovery (disabled, no soft data, no candidate survived, or
//       the CRC arbiter rejected everything within the cap). *out is
//       NOT modified.
//   -1  invalid arguments.
int ida_chase_decode(const iridium_frame_t *frame,
                     const int16_t *soft_bits, size_t n_soft,
                     ida_decoded_t *out);

// Cumulative counters (single-writer; read from anywhere for /diag).
typedef struct {
    uint32_t attempts;    // frames that entered the chase (hard fail + soft data)
    uint32_t recovered;   // frames the CRC arbiter accepted
    uint32_t crc_checks;  // total CRC-16 arbiter checks spent (collision-floor exposure)
} ida_chase_stats_t;
void ida_chase_get_stats(ida_chase_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif // IDA_CHASE_H
