// See airframes_fmt.h. Two wire schemas, one emitter core.
//
// Schema references:
//   VDL2:    dumpvdl2 2.6.0 JSON output (la_json via libacars; the
//            {"vdl2":{"app":...,"t":...,"avlc":{...,"acars":{...}}}}
//            nesting), the shape airframes.io's vdlm2 ingest parses.
//   Iridium: iridium-toolkit reassembler.py -m acars -a json (the flat
//            {"app":...,"source":...,"acars":{...}} shape).
//
// The emitter is a tiny bounded JSON writer instead of sprintf-chaining
// because the omission rules ("empty => omitted, never emitted empty")
// make comma placement stateful: each object tracks whether it has a
// member yet, so any subset of optional fields still serialises to valid
// JSON. Truncation is sticky — once the buffer is too small every later
// append is a no-op and airframes_format returns 0, so a partial (=
// syntactically broken) message can never be transmitted.

#include "airframes_fmt.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

// ---------------------------------------------------------------------------
// Bounded writer

typedef struct {
    char  *out;
    size_t cap;
    size_t len;      // invariant: len <= cap - 1 (room for the final NUL)
    bool   overflow; // sticky; set once anything failed to fit
} jw_t;

static void jw_write(jw_t *w, const char *s, size_t n)
{
    if (w->overflow) return;
    // cap==0 has no room even for the NUL; the subtraction below would
    // wrap, so reject it explicitly.
    if (w->cap == 0 || n > w->cap - 1 - w->len) {
        w->overflow = true;
        return;
    }
    memcpy(w->out + w->len, s, n);
    w->len += n;
}

static void jw_puts(jw_t *w, const char *s) { jw_write(w, s, strlen(s)); }

// Small formatted append (numbers only — every call site's worst case is
// far under the temp buffer). Bounced through a stack temp so overflow
// detection stays in one place (jw_write).
static void jw_fmt(jw_t *w, const char *fmt, ...)
{
    char    tmp[64];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= sizeof tmp) {
        w->overflow = true; // can't happen for our numeric call sites
        return;
    }
    jw_write(w, tmp, (size_t)n);
}

// Quoted, JSON-escaped string. Escapes `"` `\` and all control chars
// < 0x20 (short forms for \n \r \t — the ones real ACARS text actually
// contains — \u00XX for the rest). Deliberately self-contained: the
// device-side json_escape is not host-buildable and this module must be.
static void jw_qstr(jw_t *w, const char *s)
{
    jw_puts(w, "\"");
    for (; *s; s++) {
        unsigned char c = (unsigned char)*s;
        switch (c) {
        case '"':  jw_puts(w, "\\\""); break;
        case '\\': jw_puts(w, "\\\\"); break;
        case '\n': jw_puts(w, "\\n"); break;
        case '\r': jw_puts(w, "\\r"); break;
        case '\t': jw_puts(w, "\\t"); break;
        default:
            if (c < 0x20)
                jw_fmt(w, "\\u%04x", c);
            else
                jw_write(w, s, 1);
        }
    }
    jw_puts(w, "\"");
}

// Object-member helper: emits the separating comma (unless this is the
// first member of the enclosing object) then `"key":`. *first is the
// per-object "no member emitted yet" flag — this is what lets any subset
// of optional fields serialise with correct commas.
static void jw_member(jw_t *w, bool *first, const char *key)
{
    if (!*first) jw_puts(w, ",");
    *first = false;
    jw_puts(w, "\"");
    jw_puts(w, key);
    jw_puts(w, "\":");
}

static void jw_member_str(jw_t *w, bool *first, const char *key,
                          const char *val)
{
    jw_member(w, first, key);
    jw_qstr(w, val);
}

static void jw_member_char(jw_t *w, bool *first, const char *key, char c)
{
    char buf[2] = { c, '\0' };
    jw_member_str(w, first, key, buf);
}

static void jw_member_bool(jw_t *w, bool *first, const char *key, bool v)
{
    jw_member(w, first, key);
    jw_puts(w, v ? "true" : "false");
}

// ---------------------------------------------------------------------------
// Shared ACARS field mappings

// Both schemas render the NAK ack byte (0x15) as "!" — libacars keeps
// the raw byte, the wire formats print the bang.
static char af_ack_char(char ack) { return ack == 0x15 ? '!' : ack; }

// ---------------------------------------------------------------------------
// VDL2: dumpvdl2 2.6.0 native

static void emit_vdl2(jw_t *w, const af_msg_t *m)
{
    // app is unconditional, so the top-level object is never comma-first.
    jw_puts(w, "{\"vdl2\":{\"app\":{\"name\":\"dumpvdl2\",\"ver\":\"2.6.0\"}");
    bool top = false; // app already counts as the first member

    if (m->station_id && m->station_id[0])
        jw_member_str(w, &top, "station", m->station_id);

    if (m->time_valid) {
        jw_member(w, &top, "t");
        jw_fmt(w, "{\"sec\":%lld,\"usec\":%ld}",
               (long long)(m->epoch_us / 1000000),
               (long)(m->epoch_us % 1000000));
    }

    if (m->freq_hz) {
        jw_member(w, &top, "freq");
        jw_fmt(w, "%lu", (unsigned long)m->freq_hz);
    }

    // %.2f: fixed two decimals like dumpvdl2's human output; a pinned,
    // locale-free choice the byte-exact host fixture depends on.
    if (m->sig_level_valid) {
        jw_member(w, &top, "sig_level");
        jw_fmt(w, "%.2f", (double)m->sig_level_dbfs);
    }

    // avlc is always present for VDL2 — it is the carrier of the acars
    // object. Only the src/dst address blocks are optional (a formatter
    // caller may not have the AVLC addresses in hand).
    jw_member(w, &top, "avlc");
    jw_puts(w, "{");
    bool avlc = true;

    if (m->src_addr[0]) {
        jw_member(w, &avlc, "src");
        jw_puts(w, "{\"addr\":");
        jw_qstr(w, m->src_addr);
        if (m->src_type && m->src_type[0]) {
            jw_puts(w, ",\"type\":");
            jw_qstr(w, m->src_type);
        }
        if (m->src_status && m->src_status[0]) {
            jw_puts(w, ",\"status\":");
            jw_qstr(w, m->src_status);
        }
        jw_puts(w, "}");
    }
    if (m->dst_addr[0]) {
        jw_member(w, &avlc, "dst");
        jw_puts(w, "{\"addr\":");
        jw_qstr(w, m->dst_addr);
        if (m->dst_type && m->dst_type[0]) {
            jw_puts(w, ",\"type\":");
            jw_qstr(w, m->dst_type);
        }
        jw_puts(w, "}");
    }

    // ACARS rides in AVLC I-frames; over-the-air those are commands
    // (dumpvdl2 prints "Command" for every ACARS-bearing frame we feed),
    // so both are fixed here rather than plumbed through af_msg_t.
    jw_member_str(w, &avlc, "cr", "Command");
    jw_member_str(w, &avlc, "frame_type", "I");

    jw_member(w, &avlc, "acars");
    jw_puts(w, "{");
    bool ac = true;
    jw_member_bool(w, &ac, "err", m->err);
    jw_member_bool(w, &ac, "crc_ok", m->crc_ok);
    jw_member_bool(w, &ac, "more", m->more);
    // reg VERBATIM — dumpvdl2 keeps libacars's leading '.' (contrast the
    // Iridium "tail" below, which strips it).
    if (m->reg[0]) jw_member_str(w, &ac, "reg", m->reg);
    if (m->mode) jw_member_char(w, &ac, "mode", m->mode);
    if (m->label[0]) jw_member_str(w, &ac, "label", m->label);
    if (m->block_id) jw_member_char(w, &ac, "blk_id", m->block_id);
    if (m->ack) jw_member_char(w, &ac, "ack", af_ack_char(m->ack));
    if (m->flight_id[0]) jw_member_str(w, &ac, "flight", m->flight_id);
    if (m->msg_num[0]) jw_member_str(w, &ac, "msg_num", m->msg_num);
    if (m->msg_num_seq) jw_member_char(w, &ac, "msg_num_seq", m->msg_num_seq);
    if (m->txt && m->txt[0]) jw_member_str(w, &ac, "msg_text", m->txt);

    jw_puts(w, "}}}}"); // acars, avlc, vdl2, top
}

// ---------------------------------------------------------------------------
// Iridium: iridium-toolkit reassembler.py -m acars -a json

static void emit_iridium(jw_t *w, const af_msg_t *m)
{
    jw_puts(w, "{\"app\":{\"name\":\"iridium-toolkit\",\"version\":\"0.0.1\"},"
               "\"source\":{\"transport\":\"iridium\",\"protocol\":\"acars\"");
    if (m->station_id && m->station_id[0]) {
        jw_puts(w, ",\"station_id\":");
        jw_qstr(w, m->station_id);
    }
    jw_puts(w, "},\"acars\":{");
    bool ac = true;

    if (m->time_valid) {
        // ISO8601 with an explicit "+0000" suffix. strftime's %z is NOT
        // used: with a gmtime_r-filled tm it renders the LOCAL zone on
        // platforms whose struct tm lacks tm_gmtoff, and "+0000" is the
        // only correct value for UTC anyway — appending the literal is
        // the deterministic spelling of the same thing.
        time_t    sec = (time_t)(m->epoch_us / 1000000);
        struct tm tm_utc;
        char      ts[40];
        if (gmtime_r(&sec, &tm_utc) &&
            strftime(ts, sizeof ts - 5, "%Y-%m-%dT%H:%M:%S", &tm_utc)) {
            strcat(ts, "+0000");
            jw_member_str(w, &ac, "timestamp", ts);
        }
    }

    jw_member(w, &ac, "errors");
    jw_fmt(w, "%d", m->err ? 1 : 0);
    jw_member_str(w, &ac, "link_direction",
                  m->uplink ? "uplink" : "downlink");
    jw_member_bool(w, &ac, "block_end", !m->more);
    if (m->mode) jw_member_char(w, &ac, "mode", m->mode);

    // tail = reg with ALL leading dots stripped (the '.' is libacars's
    // fixed-width padding, kept by dumpvdl2 but not by iridium-toolkit).
    {
        const char *tail = m->reg;
        while (*tail == '.') tail++;
        if (*tail) jw_member_str(w, &ac, "tail", tail);
    }

    if (m->flight_id[0]) jw_member_str(w, &ac, "flight", m->flight_id);

    // "_\x7f" (underscore + DEL, the general-response label) prints as
    // "_d" — iridium-toolkit's rendering; raw DEL would also be an
    // awkward wire byte.
    if (m->label[0]) {
        jw_member(w, &ac, "label");
        if (m->label[0] == '_' && m->label[1] == '\x7f')
            jw_qstr(w, "_d");
        else
            jw_qstr(w, m->label);
    }

    if (m->block_id) jw_member_char(w, &ac, "block_id", m->block_id);
    if (m->msg_num[0]) jw_member_str(w, &ac, "message_number", m->msg_num);
    if (m->ack) jw_member_char(w, &ac, "ack", af_ack_char(m->ack));
    if (m->txt && m->txt[0]) jw_member_str(w, &ac, "text", m->txt);

    jw_puts(w, "}}"); // acars, top
}

// ---------------------------------------------------------------------------

size_t airframes_format(char *out, size_t cap, const af_msg_t *m)
{
    if (!out || !m) return 0;
    jw_t w = { .out = out, .cap = cap, .len = 0, .overflow = false };
    if (m->band == AF_BAND_VDL2)
        emit_vdl2(&w, m);
    else
        emit_iridium(&w, m);
    if (w.overflow) return 0;
    out[w.len] = '\0';
    return w.len;
}
