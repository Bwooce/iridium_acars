// UDP push of decoded ACARS messages. Producer calls acars_push_emit()
// from frame_decoder; emit() does a non-blocking xQueueSend onto a
// small queue. A dedicated task drains the queue, formats each entry
// as JSON, and sendto()s to the configured host:port. DNS lookup is
// cached and refreshed on send failure.
//
// No-op cleanly when NVS has out_host / out_port unset: the producer
// fast-path returns immediately, and the task never spawns.

#include "acars_push.h"

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>

#include "esp_log.h"

// Cold working buffers -> PSRAM to reclaim internal DMA-INT SRAM (dmaf).
// See docs/p4-bss-audit.md (DMA-INT reclaim, 2026-07-18).
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#else
#ifndef EXT_RAM_BSS_ATTR
#define EXT_RAM_BSS_ATTR
#endif
#endif
#include "esp_timer.h" // esp_timer_get_time — DNS re-resolution backoff
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"

#include "app_config.h"
#include "airframes_fmt.h" // af_msg_t + airframes_format() (common/acars_feed)
#include "net_time.h"      // net_time_epoch_us() — wall clock for the feed
#include "band_profile.h"  // BAND_VDL2 (band id from app_config)
#include "avlc.h"          // AVLC_ADDRTYPE_* (VDL2 address-type strings)
#include "dsp_processor.h" // FFT_SIZE — peak_bin -> absolute-freq math

static const char *TAG = "PUSH";

#define QUEUE_DEPTH 16
#define DNS_REFRESH_US (30LL * 1000000LL)

static QueueHandle_t s_q      = NULL;
static volatile bool s_active = false;

// One UDP egress endpoint: its own socket, resolved address, DNS cache and
// wedge flag, so the two targets (local debug push + airframes.io) fail
// independently — a dead Mac listener must not stall the airframes feed and
// vice-versa. resolved_at_us: timestamp of the last SUCCESSFUL getaddrinfo
// (0 = never); getaddrinfo blocks for seconds against an unreachable
// resolver, so we re-resolve only when the cache is absent/stale (>30 s) or
// the target changed — never per-message. sock_bad: set when sendto fails
// (the lwip UDP fd can wedge after netif churn); the next send recreates it.
typedef struct {
    int                sock;
    struct sockaddr_in dst;
    char               host_resolved[64];
    uint16_t           port_resolved;
    int64_t            resolved_at_us;
    bool               sock_bad;
} push_target_t;

static push_target_t s_dbg = {.sock = -1}; // GET-/messages-style debug JSON
static push_target_t s_af  = {.sock = -1}; // airframes.io wire JSON

static bool resolve_target(push_target_t *t, const char *host, uint16_t port)
{
    if (t->sock < 0) {
        t->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (t->sock < 0) {
            ESP_LOGE(TAG, "socket() failed errno=%d", errno);
            return false;
        }
    }

    struct addrinfo  hints = {.ai_family = AF_INET, .ai_socktype = SOCK_DGRAM};
    struct addrinfo *res   = NULL;
    if (getaddrinfo(host, NULL, &hints, &res) != 0 || !res) {
        ESP_LOGW(TAG, "DNS lookup '%s' failed (will retry on next push)", host);
        return false;
    }
    t->dst          = *(struct sockaddr_in *)res->ai_addr;
    t->dst.sin_port = htons(port);
    freeaddrinfo(res);

    strlcpy(t->host_resolved, host, sizeof(t->host_resolved));
    t->port_resolved  = port;
    t->resolved_at_us = esp_timer_get_time(); // cache fresh for DNS_REFRESH_US
    char ipstr[16];
    inet_ntoa_r(t->dst.sin_addr, ipstr, sizeof(ipstr));
    ESP_LOGI(TAG, "target resolved: %s:%u → %s:%u",
             host, (unsigned)port, ipstr, (unsigned)port);
    return true;
}

// Send one already-formatted datagram to a target, mirroring the original
// inline logic: recreate a wedged socket, re-resolve DNS only when stale/
// changed, ensure a socket exists, sendto, mark bad on failure. Drops the
// message (no retry/queue) on any unrecoverable step — a slow/backed-up WAN
// must never back-pressure the decoder.
static void deliver(push_target_t *t, const char *host, uint16_t port,
                    const char *pkt, size_t len)
{
    if (t->sock_bad) {
        if (t->sock >= 0) close(t->sock);
        t->sock     = -1;
        t->sock_bad = false;
    }

    int64_t now            = esp_timer_get_time();
    bool    target_changed = (t->port_resolved != port ||
                           strncmp(t->host_resolved, host,
                                      sizeof(t->host_resolved)) != 0);
    if (target_changed || t->resolved_at_us == 0 ||
        (now - t->resolved_at_us) > DNS_REFRESH_US) {
        if (!resolve_target(t, host, port)) {
            // No cached address for THIS target → drop. Otherwise keep the
            // stale cache and push the refresh 30 s out (avoid per-message DNS).
            if (target_changed || t->resolved_at_us == 0) return;
            t->resolved_at_us = now;
        }
    }

    if (t->sock < 0) {
        t->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (t->sock < 0) {
            ESP_LOGW(TAG, "socket() failed errno=%d — dropping message", errno);
            return;
        }
    }

    int n = sendto(t->sock, pkt, len, 0,
                   (struct sockaddr *)&t->dst, sizeof(t->dst));
    if (n < 0) {
        ESP_LOGW(TAG, "sendto failed errno=%d (socket recreated on next msg)", errno);
        t->sock_bad = true; // DNS cache stays valid; socket health is separate
    }
}

// Map an AVLC 3-bit address type to the dumpvdl2 "type" string (NULL = omit).
static const char *avlc_type_str(uint8_t t)
{
    switch (t) {
    case AVLC_ADDRTYPE_AIRCRAFT: return "Aircraft";
    case AVLC_ADDRTYPE_GS_ADM:
    case AVLC_ADDRTYPE_GS_DEL:   return "Ground station";
    case AVLC_ADDRTYPE_ALL:      return "All stations";
    default:                     return NULL;
    }
}

// Build the band-agnostic af_msg_t the formatter consumes from a decoded
// acars_msg_t + current config. Time is stamped here (send time ≈ decode
// time; the ring holds a message <1 s). '?' placeholders that acars_deliver
// substitutes for absent mode/block_id are mapped back to 0 so the formatter
// omits them rather than emitting a literal "?".
static void build_af_msg(af_msg_t *af, const acars_msg_t *m,
                         const app_config_t *cfg)
{
    memset(af, 0, sizeof(*af));
    af->band       = (cfg->band == BAND_VDL2) ? AF_BAND_VDL2 : AF_BAND_IRIDIUM;
    int64_t epoch  = net_time_epoch_us();
    af->epoch_us   = epoch;
    af->time_valid = (epoch != 0);
    af->uplink     = m->uplink;
    af->crc_ok     = m->crc_ok;
    af->err        = false; // only crc_ok messages are fed
    af->more       = m->more;
    af->mode       = (m->mode == '?') ? 0 : m->mode;
    af->label[0]   = m->label[0];
    af->label[1]   = m->label[1];
    af->block_id   = (m->block_id == '?') ? 0 : m->block_id;
    memcpy(af->msg_num, m->msg_num, sizeof(af->msg_num));
    af->msg_num_seq = m->msg_num_seq;
    memcpy(af->flight_id, m->flight_id, sizeof(af->flight_id));
    memcpy(af->reg, m->reg, sizeof(af->reg));
    af->ack             = m->ack;
    af->txt             = m->txt;
    // Absolute channel frequency from the tagger's packed center bin + the
    // parked LO, using the same convention as dsp_processor.c:166
    // (signed_bin = center_bin - FFT_SIZE/2; rel = signed_bin*fs/FFT_SIZE)
    // and decode_survey's abs = lo + rel. Assumes a parked LO (this
    // deployment does not live-steer; lo_now would matter otherwise). Only
    // the VDL2 schema emits freq — the Iridium schema has no freq field —
    // but the math is band-agnostic. 0 => omitted by the formatter.
    {
        int32_t center_bin = (int32_t)(m->peak_bin & 0xFFFF);
        int32_t signed_bin = center_bin - (int32_t)(FFT_SIZE / 2);
        int64_t rel_hz     = (int64_t)signed_bin *
                             (int64_t)cfg->sample_rate_hz / (int64_t)FFT_SIZE;
        int64_t abs_hz     = (int64_t)cfg->lo_freq_hz + rel_hz;
        af->freq_hz        = (abs_hz > 0) ? (uint32_t)abs_hz : 0;
    }
    // sig_level intentionally OMITTED (not a TODO): dumpvdl2's sig_level is
    // dBFS — signal power vs full scale, negative — but our only per-burst
    // level metric is snr_db (SNR above the tagger baseline, positive), a
    // different quantity in a different sign convention. The tagger works in
    // a normalised FFT domain, so no absolute dBFS is available. Emitting SNR
    // here would push a positive value into a field airframes treats as dBFS
    // and skew its signal-strength stats — so we omit it rather than mislabel.
    af->sig_level_valid = false;
    if (m->has_avlc) {
        snprintf(af->src_addr, sizeof(af->src_addr), "%06lX",
                 (unsigned long)(m->avlc_src_addr & 0xFFFFFF));
        snprintf(af->dst_addr, sizeof(af->dst_addr), "%06lX",
                 (unsigned long)(m->avlc_dst_addr & 0xFFFFFF));
        af->src_type = avlc_type_str(m->avlc_src_type);
        af->dst_type = avlc_type_str(m->avlc_dst_type);
    }
    af->station_id = (cfg->af_id[0]) ? cfg->af_id : NULL;
}

// Tiny JSON string escape — intentionally duplicated from http_server.c /
// sd_log.c pending a shared header (M17): the three copies stay
// byte-identical so the NDJSON / UDP / HTTP emitters agree. Writes up to
// outsz-1 chars + NUL, returns chars written (excluding the NUL).
static size_t json_escape(char *out, size_t outsz, const char *in)
{
    size_t w = 0;
    if (outsz == 0) return 0;
    for (; *in && w + 7 < outsz; in++) {
        unsigned char c = (unsigned char)*in;
        switch (c) {
        case '"':
            out[w++] = '\\';
            out[w++] = '"';
            break;
        case '\\':
            out[w++] = '\\';
            out[w++] = '\\';
            break;
        case '\n':
            out[w++] = '\\';
            out[w++] = 'n';
            break;
        case '\r':
            out[w++] = '\\';
            out[w++] = 'r';
            break;
        case '\t':
            out[w++] = '\\';
            out[w++] = 't';
            break;
        default:
            if (c < 0x20) {
                // \u00XX
                static const char hex[] = "0123456789abcdef";
                out[w++]                = '\\';
                out[w++]                = 'u';
                out[w++]                = '0';
                out[w++]                = '0';
                out[w++]                = hex[(c >> 4) & 0xf];
                out[w++]                = hex[c & 0xf];
            } else {
                out[w++] = (char)c;
            }
        }
    }
    out[w] = '\0';
    return w;
}

static size_t format_msg(char *buf, size_t cap, const acars_msg_t *m)
{
    // Same shape as one entry from GET /messages. Control chars in
    // ACARS text are common (CR/LF separators in long flight plans);
    // msg_num / flight_id also come off the air and get the same
    // treatment (M17).
    char esc_txt[2 * MSG_RING_TXT_MAX + 8];
    char esc_msgnum[2 * sizeof(m->msg_num) + 1];
    char esc_flight[2 * sizeof(m->flight_id) + 1];
    char esc_mode[8];
    char esc_block[8];
    char esc_label[16];
    json_escape(esc_txt, sizeof(esc_txt), m->txt);
    json_escape(esc_msgnum, sizeof(esc_msgnum), m->msg_num);
    json_escape(esc_flight, sizeof(esc_flight), m->flight_id);
    char mode_buf[2]  = {m->mode, 0};
    char block_buf[2] = {m->block_id, 0};
    char label_buf[3] = {m->label[0], m->label[1], 0};
    json_escape(esc_mode, sizeof(esc_mode), mode_buf);
    json_escape(esc_block, sizeof(esc_block), block_buf);
    json_escape(esc_label, sizeof(esc_label), label_buf);

    int n = snprintf(buf, cap,
                     "{"
                     "\"id\":%llu,"
                     "\"t_us\":%llu,"
                     "\"dir\":\"%s\","
                     "\"mode\":\"%s\","
                     "\"label\":\"%s\","
                     "\"block\":\"%s\","
                     "\"msg_num\":\"%s\","
                     "\"flight\":\"%s\","
                     "\"crc\":%s,"
                     "\"peak_bin\":%ld,"
                     "\"snr_db\":%.1f,"
                     "\"txt\":\"%s\""
                     "}\n",
                     (unsigned long long)m->id,
                     (unsigned long long)m->timestamp_us,
                     m->uplink ? "UL" : "DL",
                     esc_mode,
                     esc_label,
                     esc_block,
                     esc_msgnum,
                     esc_flight,
                     m->crc_ok ? "true" : "false",
                     (long)m->peak_bin,
                     (double)m->snr_db,
                     esc_txt);
    if (n < 0 || (size_t)n >= cap) {
        ESP_LOGW(TAG, "format truncated (n=%d cap=%u)", n, (unsigned)cap);
        if (n < 0) return 0;
        n = (int)cap - 1;
    }
    return (size_t)n;
}

static void push_task(void *arg)
{
    (void)arg;

    acars_msg_t m;
    static EXT_RAM_BSS_ATTR char pkt[2048]; // 2 KB max per UDP datagram; JSON usually ~400 B

    while (1) {
        if (xQueueReceive(s_q, &m, portMAX_DELAY) != pdTRUE) continue;

        // Pull current config each send so /config-style updates take
        // effect without reboot. Cheap (one mutex + struct copy).
        app_config_t cfg;
        app_config_snapshot(&cfg);

        // Target 1: local debug push (/messages-style JSON), unchanged.
        if (cfg.out_host[0] != '\0' && cfg.out_port != 0) {
            size_t len = format_msg(pkt, sizeof(pkt), &m);
            if (len) deliver(&s_dbg, cfg.out_host, cfg.out_port, pkt, len);
        }

        // Target 2: airframes.io. Only trusted (crc_ok) decodes are fed —
        // PARTIAL rows never reach acars_push_emit(), and this is a further
        // belt-and-braces gate. af_on defaults OFF (operator enables once
        // decode volume justifies a feeder).
        if (cfg.af_on && cfg.af_host[0] != '\0' && cfg.af_port != 0 &&
            m.crc_ok) {
            af_msg_t af;
            build_af_msg(&af, &m, &cfg);
            // Leave room for the trailing '\n' airframes expects (newline-
            // delimited JSON, one datagram per message).
            size_t l = airframes_format(pkt, sizeof(pkt) - 1, &af);
            if (l) {
                pkt[l]     = '\n';
                pkt[l + 1] = '\0';
                deliver(&s_af, cfg.af_host, cfg.af_port, pkt, l + 1);
            }
        }
    }
}

void acars_push_init(void)
{
    if (s_active) return;
    // Queue + task in PSRAM — UDP push is network-latency-tolerant
    // (datagram emit, ~ms-scale), no reason to take internal SRAM
    // away from USB DMA. See memory note feedback_usb_pool_size_not_throttle.
    s_q = xQueueCreateWithCaps(QUEUE_DEPTH, sizeof(acars_msg_t),
                               MALLOC_CAP_SPIRAM);
    if (!s_q) {
        ESP_LOGE(TAG, "queue create failed");
        return;
    }
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(push_task, "acars_push",
                                                    4096, NULL, 3, NULL,
                                                    tskNO_AFFINITY,
                                                    MALLOC_CAP_SPIRAM);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        vQueueDelete(s_q);
        s_q = NULL;
        return;
    }
    s_active = true;
    ESP_LOGI(TAG, "UDP push task ready (target read from NVS per-message)");
}

void acars_push_emit(const acars_msg_t *m)
{
    if (!s_active || !s_q || !m) return;
    // Non-blocking — drop if the queue is full (slow network shouldn't
    // back-pressure the decoder).
    (void)xQueueSend(s_q, m, 0);
}
