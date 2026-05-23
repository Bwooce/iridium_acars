#pragma once

#include "msg_ring.h"

// UDP push for decoded ACARS messages. Reads `out_host` / `out_port`
// from NVS at start. If either is unset, push is a no-op (the ring
// + HTTP /messages still work for polling consumers).
//
// Wire format: one UDP datagram per message, identical JSON to the
// per-element shape returned by GET /messages — i.e. a single object
// like {"id":...,"t_us":...,"dir":"DL", ..., "txt":"..."}\n. Newline
// terminator lets line-oriented consumers like socat -u udp-recv: -
// stream them straight to a file.
//
// Call acars_push_init() once at boot after wifi_link_start(). The
// module spins a small task that resolves the host lazily and
// reconnects every ~10s on lookup failure.
void acars_push_init(void);

// Producer hook — called from frame_decoder right after msg_ring_push.
// Safe to call when push is disabled (no-op). Non-blocking.
void acars_push_emit(const acars_msg_t *m);
