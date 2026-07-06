// IDA cross-burst fragment reassembler — see ida_reassembler.h.

#include "ida_reassembler.h"
#include <string.h>

void ida_reassembler_init(ida_reassembler_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
}

static int find_free_session(ida_reassembler_t *ctx)
{
    for (int i = 0; i < IDA_REASM_MAX_SESSIONS; i++) {
        if (!ctx->sessions[i].active) return i;
    }
    return -1;
}

// Reverse-search (like sbd_reassembler's find_matching_session) so the
// most-recently-touched matching chain wins if more than one happens
// to qualify.
static int find_matching_session(ida_reassembler_t *ctx, bool uplink,
                                 uint32_t freq_hz, uint8_t ctr, uint64_t now_us)
{
    for (int i = IDA_REASM_MAX_SESSIONS - 1; i >= 0; i--) {
        ida_reasm_session_t *s = &ctx->sessions[i];
        if (!s->active) continue;
        if (s->uplink != uplink) continue;
        if (ctr != s->next_ctr) continue;
        uint32_t lo = (s->freq_hz > IDA_REASM_FREQ_DEADBAND_HZ)
                          ? (s->freq_hz - IDA_REASM_FREQ_DEADBAND_HZ)
                          : 0u;
        uint32_t hi = s->freq_hz + IDA_REASM_FREQ_DEADBAND_HZ;
        if (freq_hz < lo || freq_hz > hi) continue;
        if (now_us < s->last_time_us) continue; // clock went backwards
        if ((now_us - s->last_time_us) > IDA_REASM_FRAG_GAP_US) continue;
        return i;
    }
    return -1;
}

static void expire_stale(ida_reassembler_t *ctx, uint64_t now_us)
{
    for (int i = 0; i < IDA_REASM_MAX_SESSIONS; i++) {
        ida_reasm_session_t *s = &ctx->sessions[i];
        if (s->active && now_us > s->last_time_us &&
            (now_us - s->last_time_us) > IDA_REASM_SESSION_TIMEOUT_US) {
            s->active = false;
            ctx->cnt_expired++;
        }
    }
}

int ida_reassembler_feed(ida_reassembler_t *ctx, const ida_decoded_t *ida,
                         bool uplink, uint32_t freq_hz, uint64_t now_us,
                         uint8_t *out_payload, int out_cap, int *out_len)
{
    if (!ctx || !ida || !out_payload || !out_len) return -1;
    *out_len = 0;
    expire_stale(ctx, now_us);

    // Fresh sequence (ctr==0) that also terminates here (cont==0) —
    // the common case: a self-contained frame needing no chaining.
    if (ida->da_ctr == 0 && ida->da_cont == 0) {
        int n = (int)ida->payload_len;
        if (n > out_cap) n = out_cap;
        memcpy(out_payload, ida->payload, (size_t)n);
        *out_len = n;
        ctx->cnt_standalone++;
        return 1;
    }

    // Fresh sequence, more fragments to come: open a new chain.
    // (Mirrors ida.py's `elif m.ctr==0 and m.cont: New long packet` —
    // like upstream, a chain longer than 8 fragments would wrap ctr
    // back to 0 while still incomplete and be mis-read as the start of
    // an unrelated chain; real SBD/ACARS envelopes don't get that
    // long, so we don't special-case it, matching upstream behaviour.)
    if (ida->da_ctr == 0 && ida->da_cont != 0) {
        int idx = find_free_session(ctx);
        if (idx < 0) {
            ctx->cnt_overflow++;
            return -1; // session table full — drop quietly
        }
        ida_reasm_session_t *s = &ctx->sessions[idx];
        memset(s, 0, sizeof(*s));
        s->active       = true;
        s->uplink       = uplink;
        s->freq_hz      = freq_hz;
        s->next_ctr     = 1;
        s->last_time_us = now_us;
        int n           = (int)ida->payload_len;
        if (n > IDA_REASM_MAX_BYTES) n = IDA_REASM_MAX_BYTES;
        memcpy(s->payload, ida->payload, (size_t)n);
        s->payload_len = n;
        ctx->cnt_opened++;
        return 0;
    }

    // Continuation fragment (da_ctr > 0): must match an open chain.
    int idx = find_matching_session(ctx, uplink, freq_hz, ida->da_ctr, now_us);
    if (idx < 0) {
        ctx->cnt_orphan++;
        return -1;
    }
    ida_reasm_session_t *s     = &ctx->sessions[idx];
    int                  avail = IDA_REASM_MAX_BYTES - s->payload_len;
    if ((int)ida->payload_len > avail) {
        // Would overflow — drop the whole chain rather than emit a
        // silently-truncated payload.
        s->active = false;
        ctx->cnt_overflow++;
        return -1;
    }
    memcpy(s->payload + s->payload_len, ida->payload, ida->payload_len);
    s->payload_len += ida->payload_len;
    s->last_time_us = now_us;
    ctx->cnt_merged++;

    if (ida->da_cont == 0) {
        // Final fragment — emit the merged payload.
        int n = s->payload_len;
        if (n > out_cap) n = out_cap;
        memcpy(out_payload, s->payload, (size_t)n);
        *out_len  = n;
        s->active = false;
        ctx->cnt_completed++;
        return 1;
    }
    s->next_ctr = (uint8_t)((ida->da_ctr + 1) % 8);
    return 0;
}
