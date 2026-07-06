#ifndef SBD_REASSEMBLER_H
#define SBD_REASSEMBLER_H

// SBD (Short Burst Data) reassembler. Takes an already-complete SBD
// envelope byte stream — either a single LW.DA burst's ida_decode()
// payload, or (when that envelope spanned more than one physical
// burst) the merged output of ida_reassembler_feed() — filters for
// SBD-bearing types, and reassembles multi-frame SBD messages.
// Reproduces the state machine from
// iridium-toolkit/iridiumtk/reassembler/sbd.py:ReassembleIDASBD.
//
// NOTE: this is a DIFFERENT, higher layer than ida_reassembler's
// cross-burst chaining. A single SBD envelope handled here (msgcnt==1
// case) may itself have been split across multiple physical LW.DA
// bursts at the IDA layer — see ida_reassembler.h. This module's own
// "multi-frame" concept is the SBD 0x10 sub-header's (msgno, msgcnt)
// fields, entirely separate from da_cont/da_ctr.
//
// SBD message types we recognise:
//   0x06 0x00 (SBD HELLO / mailbox check) — single-frame
//   0x76 0x08..0x0b (downlink data)        — single or multi-frame
//   0x76 0x0c..0x0e (uplink data)          — single or multi-frame
//
// Multi-frame messages carry a 0x10-byte sub-header at the start of
// each frame's payload that gives (length, msgno) and msgcnt comes
// from the prehdr. Sessions time out after 5 s if not completed.
//
// API surface:
//
//   sbd_reassembler_init(*ctx)             — initialise state table.
//   sbd_reassembler_feed(*ctx, payload...) — process one SBD envelope.
//   sbd_reassembler_tick(*ctx, now_us)     — expire stale sessions.
//
// On a successful reassembly (single-frame or final fragment of a
// multi-frame message), feed() returns 1 and writes the result into
// out_msg. Otherwise returns 0 (frame consumed, partial state) or
// -1 (frame not SBD or filtered out).

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum simultaneous in-flight multi-frame sessions. Real Iridium
// traffic has ≤8 concurrent SBD reassemblies typically.
#define SBD_MAX_SESSIONS 8

// Maximum SBD message payload size. Per the SBD protocol max is 270
// bytes; we round up.
#define SBD_MAX_PAYLOAD 320

typedef enum {
    SBD_TYPE_UNKNOWN = 0,
    SBD_TYPE_HELLO_0600,   // 0x06 0x00 mailbox check
    SBD_TYPE_DATA_DL_7608, // 0x76 0x08 downlink data
    SBD_TYPE_DATA_DL_7609,
    SBD_TYPE_DATA_DL_760A,
    SBD_TYPE_DATA_DL_760B,
    SBD_TYPE_DATA_UL_760C, // 0x76 0x0c-0x0e uplink data
    SBD_TYPE_DATA_UL_760D,
    SBD_TYPE_DATA_UL_760E,
} sbd_type_t;

// Returns the on-the-wire protocol byte pair as a short ASCII string
// (e.g. "7608" for SBD_TYPE_DATA_DL_7608, "0600" for HELLO, "????" for
// UNKNOWN). The log lines use this to print the actual protocol type
// instead of the enum ordinal.
const char *sbd_type_wire_name(sbd_type_t t);

typedef struct {
    sbd_type_t type;
    bool       uplink;
    uint64_t   timestamp_us; // first-fragment time for this msg
    uint8_t    payload[SBD_MAX_PAYLOAD];
    uint16_t   payload_len;
    uint8_t    msg_count; // total fragments expected
    uint8_t    msg_no;    // count of fragments received
} sbd_message_t;

// Internal per-session state.
typedef struct {
    bool          active;
    sbd_type_t    type;
    bool          uplink;
    uint8_t       msg_no_next; // expected next msgno
    uint8_t       msg_cnt;
    uint64_t      last_update_us;
    sbd_message_t msg;
} sbd_session_t;

typedef struct {
    sbd_session_t sessions[SBD_MAX_SESSIONS];
    // stats
    uint32_t cnt_short;     // mailbox check / single empty
    uint32_t cnt_single;    // single-frame data
    uint32_t cnt_assembled; // total fragments merged
    uint32_t cnt_multi;     // completed multi-frame messages
    uint32_t cnt_broken;    // expired sessions
    uint32_t cnt_filtered;  // non-SBD frames seen
} sbd_reassembler_t;

void sbd_reassembler_init(sbd_reassembler_t *ctx);

// `payload`/`payload_len` is one complete SBD envelope's worth of
// bytes — typically ida_decode()'s out->payload/payload_len directly,
// or ida_reassembler_feed()'s merged output when the envelope needed
// more than one physical LW.DA burst.
//
// Returns:  1 = complete SBD message produced (out_msg filled)
//           0 = frame consumed, partial state (out_msg untouched)
//          -1 = not SBD (filtered), frame ignored
int sbd_reassembler_feed(sbd_reassembler_t *ctx,
                         const uint8_t     *payload,
                         int                payload_len,
                         bool               uplink,
                         uint64_t           now_us,
                         sbd_message_t     *out_msg);

// Expire any session whose last_update_us is more than 5 s before
// now_us. Should be called periodically (~1 Hz).
void sbd_reassembler_tick(sbd_reassembler_t *ctx, uint64_t now_us);

#ifdef __cplusplus
}
#endif

#endif // SBD_REASSEMBLER_H
