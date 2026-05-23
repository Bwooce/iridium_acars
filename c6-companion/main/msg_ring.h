// In-RAM ring buffer of decoded ACARS messages received from the
// P4 over UART. Browser polls /api/messages?since=<seq>; the ring
// keeps the last N for display.
//
// Per design doc section 6: 200 entries × ~200 bytes = 40 KB.
// Single mutex protects writes (UART task) and reads (HTTP task).

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "iridium_protocol.h"

#define MSG_RING_CAP   200
#define STATS_RING_CAP  60     // 60 × 1-s = 1 minute of history

void msg_ring_init(void);

// Append. Caller owns the message struct. Thread-safe.
void msg_ring_push_acars(const irp_acars_msg_t *m);
void msg_ring_push_status(const irp_status_snap_t *s);

// Snapshot N entries with seq > since_seq into the caller buffer.
// Returns count written. Used by the HTTP /api/messages handler.
int msg_ring_snapshot_acars(irp_acars_msg_t *out, int max,
                             uint32_t since_seq);

// Snapshot full stats history (newest first). Returns count written.
int msg_ring_snapshot_status(irp_status_snap_t *out, int max);

// Last status snapshot (latest). Returns false if none yet.
bool msg_ring_get_latest_status(irp_status_snap_t *out);

// Totals — for /api/status.
typedef struct {
    uint32_t acars_total;
    uint32_t status_total;
    uint32_t last_acars_seq;
    uint64_t boot_at_ms;
} msg_ring_stats_t;
void msg_ring_get_stats(msg_ring_stats_t *out);
