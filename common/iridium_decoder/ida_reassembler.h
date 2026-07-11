#ifndef IDA_REASSEMBLER_H
#define IDA_REASSEMBLER_H

// Cross-burst IDA (LW.DA) fragment reassembler.
//
// A single physical LW.DA burst carries at most 24 bytes of payload
// (da_len is a 5-bit header field — see ida_decode.h). Real SBD/ACARS
// envelopes are frequently longer than that: our first captured ACARS
// ground truth (2026-07-06, REG A62001, see
// tests/scripts/build_acars_fixture.py) needs 25-37 bytes even for a
// short "demand mode" ping, and gets split across TWO consecutive
// LW.DA time slots of the same conversation, chained via the da_cont
// (continuation) and da_ctr (3-bit sequence) header fields.
//
// This layer performs that chaining BEFORE the merged bytes reach
// sbd_reassembler_feed(), which only understands the SBD envelope's
// OWN (higher-level, msgno/msgcnt sub-header) multi-frame chaining —
// a separate concept operating one layer up. Before this component
// existed, frame_decoder.c fed each physical burst's ida_decode()
// output straight into sbd_reassembler_feed(), which could only ever
// see a truncated SBD envelope for any message needing more than one
// LW.DA burst — i.e. every message in the 2026-07-06 capture (46
// packets assembled from 178 fragments upstream, avg 3.87
// fragments/packet) failed to reassemble at all.
//
// Mirrors iridium-toolkit/iridiumtk/reassembler/ida.py's
// ReassembleIDA.process(): frames chain when consecutive on the same
// (approximate) frequency and link direction, ctr advances by 1 (mod
// 8) each hop, and no more than ~280 ms elapses between hops; a whole
// open chain expires after ~1 s of inactivity.

#include <stdint.h>
#include <stdbool.h>
#include "ida_decode.h"

#ifdef __cplusplus
extern "C" {
#endif

// Concurrent open chains. Real Iridium traffic rarely needs more than
// a handful of simultaneous in-progress multi-burst DA conversations.
#define IDA_REASM_MAX_SESSIONS 4

// Cap large enough for any single SBD sub-fragment body (the 0x10
// sub-header's length byte maxes at 255) plus header overhead. 320
// matches SBD_MAX_PAYLOAD (sbd_reassembler.h) for consistency; real
// ACARS/SBD envelopes are far smaller (our captured ground truth
// needs only 37 bytes across 2 chained LW.DA bursts).
#define IDA_REASM_MAX_BYTES 320

// Tolerances, matching ida.py's ReassembleIDA.process(). ida.py uses a
// tight ±260 Hz deadband against gr-iridium's own fine-tuned frequency
// estimate; our estimator is coarser, so we widen it while staying
// well inside the 40 kHz channel grid (channels can't be confused).
#define IDA_REASM_FREQ_DEADBAND_HZ 5000u
#define IDA_REASM_FRAG_GAP_US (280ULL * 1000ULL)         // max gap between consecutive fragments
#define IDA_REASM_SESSION_TIMEOUT_US (1000ULL * 1000ULL) // max age since last fragment

typedef struct {
    bool     active;
    bool     uplink;
    uint32_t freq_hz;
    uint8_t  next_ctr; // expected da_ctr of the next fragment (mod 8)
    uint64_t last_time_us;
    uint8_t  payload[IDA_REASM_MAX_BYTES];
    int      payload_len;
} ida_reasm_session_t;

typedef struct {
    ida_reasm_session_t sessions[IDA_REASM_MAX_SESSIONS];
    // stats
    uint32_t cnt_standalone; // single-fragment frames (majority of traffic)
    uint32_t cnt_opened;     // new chains opened
    uint32_t cnt_merged;     // fragments appended to an open chain
    uint32_t cnt_completed;  // chains that reached da_cont==0
    uint32_t cnt_orphan;     // continuation fragment with no matching chain
    uint32_t cnt_overflow;   // chain would exceed IDA_REASM_MAX_BYTES, or table full
    uint32_t cnt_expired;    // chains dropped for inactivity
} ida_reassembler_t;

void ida_reassembler_init(ida_reassembler_t *ctx);

// Feed one successfully-BCH-decoded LW.DA frame (caller must already
// have checked ida->ok && ida->header_ok && ida->crc_ok, same
// precondition as sbd_reassembler_feed). `freq_hz` should be the
// frame's estimated RF frequency (used, with a deadband, to avoid
// merging fragments from two concurrent conversations that happen to
// share a ctr sequence).
//
// Returns:
//    1  chain complete (or the frame was already standalone):
//       out_payload[0..*out_len) holds the full merged SBD-envelope
//       bytes, ready for sbd_reassembler_feed().
//    0  fragment consumed, chain still open (out_payload untouched).
//   -1  fragment dropped (orphan continuation, table full, or would
//       overflow the merge buffer) — caller should treat this like
//       "not SBD" and move on.
//
// Note: only the terminal fragment's own CRC covers that frame's OWN
// 210-bit payload region; iridium-toolkit's ida.py chains purely on
// cont/ctr, independent of any single fragment's CRC, and the merged
// output carries no CRC of its own (the SBD/ACARS layers above cover
// integrity via their own CRC-16/CCITT-FALSE / la_crc16_ccitt checks).
int ida_reassembler_feed(ida_reassembler_t *ctx, const ida_decoded_t *ida,
                         bool uplink, uint32_t freq_hz, uint64_t now_us,
                         uint8_t *out_payload, int out_cap, int *out_len);

// A timed-out (incomplete) chain, handed to the driver for best-effort
// PARTIAL salvage instead of being silently discarded. frags = fragments that
// were received before the chain stalled (1 = opener only).
typedef struct {
    uint8_t  payload[IDA_REASM_MAX_BYTES];
    int      payload_len;
    bool     uplink;
    uint32_t freq_hz;
    uint64_t last_time_us;
    uint8_t  frags;
} ida_salvage_t;

// Reap ONE chain that has been idle longer than IDA_REASM_SESSION_TIMEOUT_US:
// copy its buffered partial payload into *out, mark the session inactive, bump
// cnt_expired, return 1. Return 0 when no chain is due. Drain in a loop:
//   ida_salvage_t s;
//   while (ida_reassembler_reap(ctx, now_us, &s)) { /* gate + salvage s */ }
//
// This REPLACES the old auto-expire that ran inside feed() and discarded the
// buffer. The driver now owns expiry, so it can salvage the partial. Contract:
// call reap() to exhaustion each ~1 Hz tick AND immediately before feed(), so
// stale sessions free their table slots before a new opener needs one. Safe:
// find_matching_session already rejects fragments older than IDA_REASM_FRAG_GAP
// _US, so an un-reaped stale session can never wrongly capture a new fragment;
// the only coupling is slot pressure, which reap-before-feed covers.
int ida_reassembler_reap(ida_reassembler_t *ctx, uint64_t now_us,
                         ida_salvage_t *out);

#ifdef __cplusplus
}
#endif

#endif // IDA_REASSEMBLER_H
