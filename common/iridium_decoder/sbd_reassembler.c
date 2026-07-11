// SBD reassembler — see sbd_reassembler.h.

#include "sbd_reassembler.h"
#include <string.h>

#define SBD_TIMEOUT_US (5ULL * 1000000ULL) // 5 s

// msg_cnt comes straight off the air (prehdr[3] or prehdr[15]) with no
// upstream range check, so BCH-false-positive noise that happens to
// classify() as an SBD type can hand us any byte 0-255. Each apparent
// "first frame" with msg_cnt>1 opens one of only SBD_MAX_SESSIONS slots
// that then only closes via the 5 s timeout — under a steady trickle of
// noise that saturates the table and starves real multi-frame traffic.
// A legitimate multi-frame message can't realistically need more than
// this many fragments: SBD_MAX_PAYLOAD (320 B) divided by the largest
// possible per-fragment body (IDA payload cap of 24 B, minus 2 B type +
// up to 7 B prehdr + 3 B sub-header) still needs well under 32
// fragments. Bounding msg_cnt here rejects most garbage values before a
// session is ever opened, without touching real traffic.
#define SBD_MSG_CNT_MAX 32

const char *sbd_type_wire_name(sbd_type_t t)
{
    switch (t) {
    case SBD_TYPE_UNKNOWN:
        return "????";
    case SBD_TYPE_HELLO_0600:
        return "0600";
    case SBD_TYPE_DATA_DL_7608:
        return "7608";
    case SBD_TYPE_DATA_DL_7609:
        return "7609";
    case SBD_TYPE_DATA_DL_760A:
        return "760a";
    case SBD_TYPE_DATA_DL_760B:
        return "760b";
    case SBD_TYPE_DATA_UL_760C:
        return "760c";
    case SBD_TYPE_DATA_UL_760D:
        return "760d";
    case SBD_TYPE_DATA_UL_760E:
        return "760e";
    }
    return "????";
}

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
    if (p[0] == 0x06 && p[1] == 0x00) return SBD_TYPE_HELLO_0600;
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

// Find an active session whose msg_no_next matches msgno, direction
// matches, and type matches. Mirrors upstream's reverse-search through
// self.multi, plus a type check upstream leaves as a TODO ("could check
// if 'typ' seems right") — without it, two co-incident noise-opened
// sessions of different wire types but the same msg_no_next/uplink can
// cross-contaminate each other's payload.
static int find_matching_session(sbd_reassembler_t *ctx,
                                 uint8_t msgno, bool uplink, sbd_type_t typ)
{
    for (int i = SBD_MAX_SESSIONS - 1; i >= 0; i--) {
        sbd_session_t *s = &ctx->sessions[i];
        if (s->active && s->msg_no_next == msgno && s->uplink == uplink &&
            s->type == typ) {
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
                         const uint8_t     *payload,
                         int                payload_len,
                         bool               uplink,
                         uint64_t           now_us,
                         sbd_message_t     *out_msg)
{
    if (!ctx || !payload) return -1;
    if (payload_len < 5) {
        ctx->cnt_filtered++;
        return -1;
    }
    sbd_type_t typ = classify(payload, payload_len, uplink);
    if (typ == SBD_TYPE_UNKNOWN) {
        ctx->cnt_filtered++;
        return -1;
    }

    // Consume the 2-byte type prefix.
    const uint8_t *p = payload + 2;
    int            n = payload_len - 2;

    // Parse prehdr + extract msg_count / msg_no per upstream sbd.py.
    int            msg_cnt         = -1; // unknown / single-frame
    int            msg_no          = 0;
    const uint8_t *prehdr          = p;
    int            prehdr_len      = 0;
    int            hdr_payload_len = -1; // -1 = use rest of payload

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
        msg_cnt    = prehdr[15];
        msg_no     = (msg_cnt == 0) ? 0 : 1;
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
                prehdr_len = 7; // upstream falls through with this
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
            msg_no          = p[2];
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
            out_msg->msg_count = 1;
            out_msg->msg_no    = 1;
        }
        return 1;
    }
    // 3. msg_cnt > 1: multi-frame — start or extend session
    if (msg_cnt > 1 && msg_cnt <= SBD_MSG_CNT_MAX && msg_no == 1) {
        // First frame of multi-packet: open a new session.
        int idx = find_free_session(ctx);
        if (idx < 0) {
            ctx->cnt_broken++;
            return 0; // table full — drop quietly
        }
        sbd_session_t *s    = &ctx->sessions[idx];
        s->active           = true;
        s->type             = typ;
        s->uplink           = uplink;
        s->msg_no_next      = 2; // expect frame #2 next
        s->msg_cnt          = (uint8_t)msg_cnt;
        s->last_update_us   = now_us;
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
        int idx = find_matching_session(ctx, (uint8_t)msg_no, uplink, typ);
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
        s->msg.msg_no     = (uint8_t)msg_no;
        s->msg_no_next    = (uint8_t)(msg_no + 1);
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

// See sbd_reassembler.h for the contract. This mirrors ONLY the
// classify()/prehdr/0x10-sub-header parsing done above in feed()
// (roughly its lines up to the dispatch switch) -- it does not touch
// sessions or dispatch at all, since salvage has no state to dispatch
// into. The two truncation-reject points in that parse (prehdr shorter
// than declared, and 0x10 sub-header body shorter than declared) become
// out->truncated = true + best-effort body here instead of a hard
// reject; every other branch (HELLO 0x20 marker check, 0x76 0x08
// prehdr-variant selection, uplink 0x50/0x51 skip, the n<=3-but-p[0]==
// 0x10 fallthrough) is left exactly as feed() has it.
int sbd_salvage_parse(const uint8_t *payload, int payload_len, bool uplink,
                      sbd_salvage_info_t *out)
{
    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    out->msg_cnt = -1;
    if (!payload || payload_len < 0) return 0;

    sbd_type_t typ = classify(payload, payload_len, uplink);
    if (typ == SBD_TYPE_UNKNOWN) return 0;

    // Consume the 2-byte type prefix (classify() guarantees payload_len>=2
    // whenever it returns non-UNKNOWN).
    const uint8_t *p = payload + 2;
    int            n = payload_len - 2;

    int  msg_cnt   = -1;
    int  msg_no    = 0;
    bool truncated = false;

    if (typ == SBD_TYPE_HELLO_0600) {
        // 0x06 0x00 path: data[0] must be 0x20. If we don't even have
        // that byte, we simply don't know yet -- truncated, not a
        // structural mismatch. If we DO have it and it's wrong, that's
        // a real structural reject (matches feed()).
        if (n < 1) {
            truncated = true;
            n         = 0; // no confirmed body
        } else if (p[0] != 0x20) {
            return 0;
        } else if (n < 16) {
            // Have the marker but can't reach prehdr[15] (msg_cnt).
            truncated = true;
            n         = 0; // don't know where prehdr ends -> no body
        } else {
            // Our IDA payload is at most ~22 bytes so we can't always
            // get a full 29-byte prehdr -- Python's slice truncates and
            // so does feed(); that's normal, not "truncated" here.
            int prehdr_len = (n < 29) ? n : 29;
            msg_cnt        = p[15];
            msg_no         = (msg_cnt == 0) ? 0 : 1;
            p += prehdr_len;
            n -= prehdr_len;
        }
    } else {
        // 0x76 0x08-0x0e path:
        if (typ == SBD_TYPE_DATA_DL_7608) {
            int prehdr_len;
            if (n >= 1 && p[0] == 0x26) {
                prehdr_len = 7;
            } else if (n >= 1 && p[0] == 0x20) {
                prehdr_len = 5;
            } else {
                prehdr_len = 7; // upstream falls through with this
            }
            if (n >= prehdr_len) {
                msg_cnt = p[3];
                p += prehdr_len;
                n -= prehdr_len;
            } else {
                // Truncation point #1: can't even read the full prehdr.
                truncated = true;
                n         = 0; // don't know where prehdr ends -> no body
            }
        }
        if (!truncated) {
            // 0x50 / 0x51 ack/nack uplink mid-handler -- skip 3 bytes.
            if (uplink && n >= 3 && (p[0] == 0x50 || p[0] == 0x51)) {
                p += 3;
                n -= 3;
            }
            // Body: optional 0x10 sub-header.
            if (n == 0) {
                msg_no = 0;
            } else if (n > 3 && p[0] == 0x10) {
                int hdr_payload_len = p[1];
                msg_no              = p[2];
                p += 3;
                n -= 3;
                if (n < hdr_payload_len) {
                    // Truncation point #2: declared body runs past what
                    // we have. Salvage what's here instead of rejecting.
                    truncated = true;
                } else if (n > hdr_payload_len) {
                    n = hdr_payload_len;
                }
            } else {
                // No sub-header (or too short to detect one) -- treat as
                // single-frame, whatever's left is the body.
                msg_no = 0;
            }
        }
    }

    if (n < 0) n = 0;
    if (n > SBD_MAX_PAYLOAD) n = SBD_MAX_PAYLOAD;

    out->type      = typ;
    out->truncated = truncated;
    out->msg_no    = msg_no;
    out->msg_cnt   = msg_cnt;
    out->body      = (n > 0) ? p : NULL;
    out->body_len  = n;
    return 1;
}
