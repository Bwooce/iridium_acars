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
#include "esp_timer.h" // esp_timer_get_time — DNS re-resolution backoff
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"

#include "app_config.h"

static const char *TAG = "PUSH";

#define QUEUE_DEPTH 16

static QueueHandle_t s_q      = NULL;
static volatile bool s_active = false;

static int                s_sock = -1;
static struct sockaddr_in s_dst;
static char               s_host_resolved[64];
static uint16_t           s_port_resolved = 0;
// DNS re-resolution backoff: timestamp of the last SUCCESSFUL
// getaddrinfo (0 = never). getaddrinfo blocks this task for up to
// seconds when the resolver is unreachable, so we only re-resolve when
// the cache is absent/stale (>30 s) or the target changed — never
// per-message.
static int64_t s_resolved_at_us = 0;
#define DNS_REFRESH_US (30LL * 1000000LL)
// Set when sendto fails — the lwip UDP fd can wedge after netif churn
// (DHCP renew, AP roam). The next send closes + recreates the socket.
static bool s_sock_bad = false;

static bool resolve_target(const char *host, uint16_t port)
{
    if (s_sock < 0) {
        s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s_sock < 0) {
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
    s_dst          = *(struct sockaddr_in *)res->ai_addr;
    s_dst.sin_port = htons(port);
    freeaddrinfo(res);

    strlcpy(s_host_resolved, host, sizeof(s_host_resolved));
    s_port_resolved  = port;
    s_resolved_at_us = esp_timer_get_time(); // cache fresh for DNS_REFRESH_US
    char ipstr[16];
    inet_ntoa_r(s_dst.sin_addr, ipstr, sizeof(ipstr));
    ESP_LOGI(TAG, "target resolved: %s:%u → %s:%u",
             host, (unsigned)port, ipstr, (unsigned)port);
    return true;
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

    // Wait for the queue to fill at least once before bothering to set
    // anything up — saves work if push is enabled but never used.
    acars_msg_t m;
    static char pkt[2048]; // 2 KB max per UDP datagram; JSON usually ~400 B

    while (1) {
        if (xQueueReceive(s_q, &m, portMAX_DELAY) != pdTRUE) continue;

        // Pull current config each send so /config-style updates take
        // effect without reboot. Cheap (one mutex + struct copy).
        app_config_t cfg;
        app_config_snapshot(&cfg);
        if (cfg.out_host[0] == '\0' || cfg.out_port == 0) continue;

        // A previous sendto failed — recreate the socket before this
        // send (resolve_target / the block below makes a fresh one).
        if (s_sock_bad) {
            if (s_sock >= 0) close(s_sock);
            s_sock     = -1;
            s_sock_bad = false;
        }

        // Re-resolve only when needed: target changed, never resolved,
        // or the cached result is older than DNS_REFRESH_US. getaddrinfo
        // can block for seconds against an unreachable resolver, so a
        // per-message retry would stall the queue and hammer DNS when
        // the network flaps; 30 s is fresh enough to track a DNS change.
        int64_t now            = esp_timer_get_time();
        bool    target_changed = (s_port_resolved != cfg.out_port ||
                               strncmp(s_host_resolved, cfg.out_host,
                                          sizeof(s_host_resolved)) != 0);
        if (target_changed || s_resolved_at_us == 0 ||
            (now - s_resolved_at_us) > DNS_REFRESH_US) {
            if (!resolve_target(cfg.out_host, cfg.out_port)) {
                // No cached address for THIS target → drop the message.
                // Otherwise keep using the stale cache and push the
                // refresh 30 s out so we don't retry DNS per message.
                if (target_changed || s_resolved_at_us == 0) continue;
                s_resolved_at_us = now;
            }
        }

        // Make sure we have a socket even when no resolve ran this
        // message (e.g. recreated after a sendto failure above).
        if (s_sock < 0) {
            s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (s_sock < 0) {
                ESP_LOGW(TAG, "socket() failed errno=%d — dropping message", errno);
                continue;
            }
        }

        size_t len = format_msg(pkt, sizeof(pkt), &m);
        if (len == 0) continue;

        int n = sendto(s_sock, pkt, len, 0,
                       (struct sockaddr *)&s_dst, sizeof(s_dst));
        if (n < 0) {
            ESP_LOGW(TAG, "sendto failed errno=%d (socket recreated on next msg)", errno);
            // Mark the fd bad — it gets closed + recreated before the
            // next send. The DNS cache stays valid (its 30 s TTL covers
            // address changes); resolution and socket health are
            // independent failure modes.
            s_sock_bad = true;
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
