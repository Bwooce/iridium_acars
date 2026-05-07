// SBD reassembler — see sbd_reassembler.h.

#include "sbd_reassembler.h"
#include <string.h>

#define SBD_TIMEOUT_US  (5ULL * 1000000ULL)   // 5 s

void sbd_reassembler_init(sbd_reassembler_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
}

// Classify by first two payload bytes. Returns SBD_TYPE_UNKNOWN
// for non-SBD frames (caller should drop).
static sbd_type_t classify(const uint8_t *p, int len, bool uplink)
{
    if (len < 2) return SBD_TYPE_UNKNOWN;
    if (p[0] == 0x06 && p[1] == 0x00)        return SBD_TYPE_HELLO_0600;
    if (p[0] == 0x76) {
        if (uplink) {
            if (p[1] == 0x0c) return SBD_TYPE_DATA_UL_760C;
            if (p[1] == 0x0d) return SBD_TYPE_DATA_UL_760D;
            if (p[1] == 0x0e) return SBD_TYPE_DATA_UL_760E;
        } else {
            if (p[1] == 0x08) return SBD_TYPE_DATA_DL_7608;
            if (p[1] == 0x09) return SBD_TYPE_DATA_DL_7609;
            if (p[1] == 0x0a) return SBD_TYPE_DATA_DL_760A;
            if (p[1] == 0x0b) return SBD_TYPE_DATA_DL_760B;
        }
    }
    return SBD_TYPE_UNKNOWN;
}

static int find_free_session(sbd_reassembler_t *ctx)
{
    for (int i = 0; i < SBD_MAX_SESSIONS; i++) {
        if (!ctx->sessions[i].active) return i;
    }
    return -1;
}

// Find an active session whose msg_no_next matches msgno and direction
// matches. Mirrors upstream's reverse-search through self.multi.
static int find_matching_session(sbd_reassembler_t *ctx,
                                 uint8_t msgno, bool uplink)
{
    for (int i = SBD_MAX_SESSIONS - 1; i >= 0; i--) {
        sbd_session_t *s = &ctx->sessions[i];
        if (s->active && s->msg_no_next == msgno && s->uplink == uplink) {
            return i;
        }
    }
    return -1;
}

void sbd_reassembler_tick(sbd_reassembler_t *ctx, uint64_t now_us)
{
    if (!ctx) return;
    for (int i = 0; i < SBD_MAX_SESSIONS; i++) {
        sbd_session_t *s = &ctx->sessions[i];
        if (s->active && now_us > s->last_update_us &&
            (now_us - s->last_update_us) > SBD_TIMEOUT_US) {
            s->active = false;
            ctx->cnt_broken++;
        }
    }
}

int sbd_reassembler_feed(sbd_reassembler_t *ctx,
                         const ida_decoded_t *ida,
                         bool uplink,
                         uint64_t now_us,
                         sbd_message_t *out_msg)
{
    if (!ctx || !ida) return -1;
    if (ida->payload_len < 5) {
        ctx->cnt_filtered++;
        return -1;
    }
    sbd_type_t typ = classify(ida->payload, ida->payload_len, uplink);
    if (typ == SBD_TYPE_UNKNOWN) {
        ctx->cnt_filtered++;
        return -1;
    }

    // Consume the 2-byte type prefix.
    const uint8_t *p = ida->payload + 2;
    int n = ida->payload_len - 2;

    // Parse prehdr + extract msg_count / msg_no per upstream sbd.py.
    int msg_cnt = -1;     // unknown / single-frame
    int msg_no  = 0;
    const uint8_t *prehdr = p;
    int prehdr_len = 0;
    int hdr_payload_len = -1;   // -1 = use rest of payload

    if (typ == SBD_TYPE_HELLO_0600) {
        // 0x06 0x00 path: data[0] must be 0x20.
        if (n < 1 || p[0] != 0x20) {
            ctx->cnt_filtered++;
            return -1;
        }
        // Need at least up through prehdr[15] (msg_cnt). Our IDA payload
        // is at most ~22 bytes so we can't always get a full 29-byte
        // prehdr — Python's slice truncates and so do we.
        if (n < 16) {
            ctx->cnt_filtered++;
            return -1;
        }
        prehdr_len = (n < 29) ? n : 29;
        msg_cnt = prehdr[15];
        msg_no = (msg_cnt == 0) ? 0 : 1;
        p += prehdr_len;
        n -= prehdr_len;
    } else {
        // 0x76 0x08-0x0e path:
        if (typ == SBD_TYPE_DATA_DL_7608) {
            if (n >= 1 && p[0] == 0x26) {
                prehdr_len = 7;
            } else if (n >= 1 && p[0] == 0x20) {
                prehdr_len = 5;
            } else {
                prehdr_len = 7;     // upstream falls through with this
            }
            if (n >= prehdr_len) {
                msg_cnt = prehdr[3];
            }
            if (n < prehdr_len) {
                ctx->cnt_filtered++;
                return -1;
            }
            p += prehdr_len;
            n -= prehdr_len;
        }
        // 0x50 / 0x51 ack/nack uplink mid-handler — skip 3 bytes.
        if (uplink && n >= 3 && (p[0] == 0x50 || p[0] == 0x51)) {
            p += 3;
            n -= 3;
        }
        // Body: optional 0x10 sub-header.
        if (n == 0) {
            msg_no = 0;
        } else if (n > 3 && p[0] == 0x10) {
            hdr_payload_len = p[1];
            msg_no = p[2];
            p += 3;
            n -= 3;
            if (n < hdr_payload_len) {
                // Pkt too short — drop
                ctx->cnt_filtered++;
                return -1;
            }
            if (n > hdr_payload_len) {
                n = hdr_payload_len;
            }
        } else {
            // No sub-header — treat as single-frame
            msg_no = 0;
        }
    }

    // Bound check the data we'll copy.
    if (n > SBD_MAX_PAYLOAD) n = SBD_MAX_PAYLOAD;

    // Sweep stale sessions before doing dispatch.
    sbd_reassembler_tick(ctx, now_us);

    // Three dispatch cases per upstream sbd.py:
    //
    // 1. msg_no == 0: short / mailbox-check pkt — emit immediately
    if (msg_no == 0) {
        ctx->cnt_short++;
        if (out_msg) {
            out_msg->type         = typ;
            out_msg->uplink       = uplink;
            out_msg->timestamp_us = now_us;
            out_msg->payload_len  = (uint16_t)(n > 0 ? n : 0);
            if (n > 0) memcpy(out_msg->payload, p, n);
            out_msg->msg_count = (uint8_t)(msg_cnt > 0 ? msg_cnt : 1);
            out_msg->msg_no    = 0;
        }
        return 1;
    }
    // 2. msg_cnt == 1 && msg_no == 1: single-frame data — emit
    if (msg_cnt == 1 && msg_no == 1) {
        ctx->cnt_single++;
        if (out_msg) {
            out_msg->type         = typ;
            out_msg->uplink       = uplink;
            out_msg->timestamp_us = now_us;
            out_msg->payload_len  = (uint16_t)n;
            memcpy(out_msg->payload, p, n);
            out_msg->msg_count    = 1;
            out_msg->msg_no       = 1;
        }
        return 1;
    }
    // 3. msg_cnt > 1: multi-frame — start or extend session
    if (msg_cnt > 1 && msg_no == 1) {
        // First frame of multi-packet: open a new session.
        int idx = find_free_session(ctx);
        if (idx < 0) {
            ctx->cnt_broken++;
            return 0;       // table full — drop quietly
        }
        sbd_session_t *s = &ctx->sessions[idx];
        s->active        = true;
        s->type          = typ;
        s->uplink        = uplink;
        s->msg_no_next   = 2;          // expect frame #2 next
        s->msg_cnt       = (uint8_t)msg_cnt;
        s->last_update_us = now_us;
        s->msg.type         = typ;
        s->msg.uplink       = uplink;
        s->msg.timestamp_us = now_us;
        s->msg.payload_len  = (uint16_t)n;
        s->msg.msg_count    = (uint8_t)msg_cnt;
        s->msg.msg_no       = 1;
        memcpy(s->msg.payload, p, n);
        ctx->cnt_assembled++;
        return 0;
    }
    if (msg_no > 1) {
        // Continuation — find the session waiting for this msg_no.
        int idx = find_matching_session(ctx, (uint8_t)msg_no, uplink);
        if (idx < 0) {
            ctx->cnt_broken++;
            return 0;
        }
        sbd_session_t *s = &ctx->sessions[idx];
        // Append payload, bounded.
        int avail = SBD_MAX_PAYLOAD - s->msg.payload_len;
        int copy  = n < avail ? n : avail;
        if (copy > 0) {
            memcpy(s->msg.payload + s->msg.payload_len, p, copy);
            s->msg.payload_len = (uint16_t)(s->msg.payload_len + copy);
        }
        s->msg.msg_no = (uint8_t)msg_no;
        s->msg_no_next = (uint8_t)(msg_no + 1);
        s->last_update_us = now_us;
        ctx->cnt_assembled++;

        if (msg_no == s->msg_cnt) {
            // Final fragment — emit.
            ctx->cnt_multi++;
            if (out_msg) {
                *out_msg = s->msg;
            }
            s->active = false;
            return 1;
        }
        return 0;
    }
    ctx->cnt_filtered++;
    return -1;
}
