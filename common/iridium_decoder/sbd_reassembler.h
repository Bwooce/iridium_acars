#ifndef SBD_REASSEMBLER_H
#define SBD_REASSEMBLER_H

// SBD (Short Burst Data) reassembler. Takes the byte payload from
// ida_decode'd IDA frames, filters for SBD-bearing types, and
// reassembles multi-frame SBD messages. Reproduces the state machine
// from iridium-toolkit/iridiumtk/reassembler/sbd.py:ReassembleIDASBD.
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
//   sbd_reassembler_init(*ctx)         — initialise state table.
//   sbd_reassembler_feed(*ctx, ida)    — process one IDA decode.
//   sbd_reassembler_tick(*ctx, now_us) — expire stale sessions.
//
// On a successful reassembly (single-frame or final fragment of a
// multi-frame message), feed() returns 1 and writes the result into
// out_msg. Otherwise returns 0 (frame consumed, partial state) or
// -1 (frame not SBD or filtered out).

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ida_decode.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maximum simultaneous in-flight multi-frame sessions. Real Iridium
// traffic has ≤8 concurrent SBD reassemblies typically.
#define SBD_MAX_SESSIONS  8

// Maximum SBD message payload size. Per the SBD protocol max is 270
// bytes; we round up.
#define SBD_MAX_PAYLOAD   320

typedef enum {
    SBD_TYPE_UNKNOWN = 0,
    SBD_TYPE_HELLO_0600,        // 0x06 0x00 mailbox check
    SBD_TYPE_DATA_DL_7608,      // 0x76 0x08 downlink data
    SBD_TYPE_DATA_DL_7609,
    SBD_TYPE_DATA_DL_760A,
    SBD_TYPE_DATA_DL_760B,
    SBD_TYPE_DATA_UL_760C,      // 0x76 0x0c-0x0e uplink data
    SBD_TYPE_DATA_UL_760D,
    SBD_TYPE_DATA_UL_760E,
} sbd_type_t;

typedef struct {
    sbd_type_t  type;
    bool        uplink;
    uint64_t    timestamp_us;  // first-fragment time for this msg
    uint8_t     payload[SBD_MAX_PAYLOAD];
    uint16_t    payload_len;
    uint8_t     msg_count;     // total fragments expected
    uint8_t     msg_no;        // count of fragments received
} sbd_message_t;

// Internal per-session state.
typedef struct {
    bool        active;
    sbd_type_t  type;
    bool        uplink;
    uint8_t     msg_no_next;   // expected next msgno
    uint8_t     msg_cnt;
    uint64_t    last_update_us;
    sbd_message_t msg;
} sbd_session_t;

typedef struct {
    sbd_session_t  sessions[SBD_MAX_SESSIONS];
    // stats
    uint32_t       cnt_short;       // mailbox check / single empty
    uint32_t       cnt_single;      // single-frame data
    uint32_t       cnt_assembled;   // total fragments merged
    uint32_t       cnt_multi;       // completed multi-frame messages
    uint32_t       cnt_broken;      // expired sessions
    uint32_t       cnt_filtered;    // non-SBD frames seen
} sbd_reassembler_t;

void sbd_reassembler_init(sbd_reassembler_t *ctx);

// Returns:  1 = complete SBD message produced (out_msg filled)
//           0 = frame consumed, partial state (out_msg untouched)
//          -1 = not SBD (filtered), frame ignored
int  sbd_reassembler_feed(sbd_reassembler_t *ctx,
                          const ida_decoded_t *ida,
                          bool uplink,
                          uint64_t now_us,
                          sbd_message_t *out_msg);

// Expire any session whose last_update_us is more than 5 s before
// now_us. Should be called periodically (~1 Hz).
void sbd_reassembler_tick(sbd_reassembler_t *ctx, uint64_t now_us);

#ifdef __cplusplus
}
#endif

#endif // SBD_REASSEMBLER_H
