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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "app_config.h"

static const char *TAG = "PUSH";

#define QUEUE_DEPTH    16

static QueueHandle_t s_q     = NULL;
static volatile bool s_active = false;

static int  s_sock         = -1;
static struct sockaddr_in s_dst;
static char s_host_resolved[64];
static uint16_t s_port_resolved = 0;

static bool resolve_target(const char *host, uint16_t port)
{
    if (s_sock < 0) {
        s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s_sock < 0) {
            ESP_LOGE(TAG, "socket() failed errno=%d", errno);
            return false;
        }
    }

    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_DGRAM };
    struct addrinfo *res = NULL;
    if (getaddrinfo(host, NULL, &hints, &res) != 0 || !res) {
        ESP_LOGW(TAG, "DNS lookup '%s' failed (will retry on next push)", host);
        return false;
    }
    s_dst = *(struct sockaddr_in *)res->ai_addr;
    s_dst.sin_port = htons(port);
    freeaddrinfo(res);

    strlcpy(s_host_resolved, host, sizeof(s_host_resolved));
    s_port_resolved = port;
    char ipstr[16];
    inet_ntoa_r(s_dst.sin_addr, ipstr, sizeof(ipstr));
    ESP_LOGI(TAG, "target resolved: %s:%u → %s:%u",
             host, (unsigned)port, ipstr, (unsigned)port);
    return true;
}

static size_t format_msg(char *buf, size_t cap, const acars_msg_t *m)
{
    // Same shape as one entry from GET /messages. Reuse a tiny JSON
    // string escape — control chars in ACARS text are common (CR/LF
    // separators in long flight plans).
    char esc_txt[2 * MSG_RING_TXT_MAX + 8];
    size_t w = 0;
    for (const char *p = m->txt; *p && w + 7 < sizeof(esc_txt); p++) {
        unsigned char c = (unsigned char)*p;
        switch (c) {
        case '"':  esc_txt[w++] = '\\'; esc_txt[w++] = '"';  break;
        case '\\': esc_txt[w++] = '\\'; esc_txt[w++] = '\\'; break;
        case '\n': esc_txt[w++] = '\\'; esc_txt[w++] = 'n';  break;
        case '\r': esc_txt[w++] = '\\'; esc_txt[w++] = 'r';  break;
        case '\t': esc_txt[w++] = '\\'; esc_txt[w++] = 't';  break;
        default:
            if (c < 0x20) {
                static const char hex[] = "0123456789abcdef";
                esc_txt[w++] = '\\'; esc_txt[w++] = 'u';
                esc_txt[w++] = '0'; esc_txt[w++] = '0';
                esc_txt[w++] = hex[(c >> 4) & 0xf];
                esc_txt[w++] = hex[c & 0xf];
            } else {
                esc_txt[w++] = (char)c;
            }
        }
    }
    esc_txt[w] = '\0';

    int n = snprintf(buf, cap,
        "{"
            "\"id\":%llu,"
            "\"t_us\":%llu,"
            "\"dir\":\"%s\","
            "\"mode\":\"%c\","
            "\"label\":\"%.2s\","
            "\"block\":\"%c\","
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
        m->mode,
        m->label,
        m->block_id,
        m->msg_num,
        m->flight_id,
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
    static char pkt[2048];     // 2 KB max per UDP datagram; JSON usually ~400 B

    while (1) {
        if (xQueueReceive(s_q, &m, portMAX_DELAY) != pdTRUE) continue;

        // Pull current config each send so /config-style updates take
        // effect without reboot. Cheap (one mutex + struct copy).
        app_config_t cfg;
        app_config_snapshot(&cfg);
        if (cfg.out_host[0] == '\0' || cfg.out_port == 0) continue;

        // Re-resolve if host/port changed since last send, or if we've
        // never resolved.
        if (s_port_resolved != cfg.out_port ||
            strncmp(s_host_resolved, cfg.out_host, sizeof(s_host_resolved)) != 0) {
            if (!resolve_target(cfg.out_host, cfg.out_port)) continue;
        }

        size_t len = format_msg(pkt, sizeof(pkt), &m);
        if (len == 0) continue;

        int n = sendto(s_sock, pkt, len, 0,
                       (struct sockaddr *)&s_dst, sizeof(s_dst));
        if (n < 0) {
            ESP_LOGW(TAG, "sendto failed errno=%d (will re-resolve next msg)", errno);
            // Force re-resolve on next message — could be a stale DHCP
            // lease, AP roam, etc.
            s_port_resolved = 0;
            s_host_resolved[0] = '\0';
        }
    }
}

void acars_push_init(void)
{
    if (s_active) return;
    s_q = xQueueCreate(QUEUE_DEPTH, sizeof(acars_msg_t));
    if (!s_q) {
        ESP_LOGE(TAG, "queue create failed");
        return;
    }
    BaseType_t ok = xTaskCreate(push_task, "acars_push", 4096, NULL, 3, NULL);
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
