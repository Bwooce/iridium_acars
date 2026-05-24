// Minimal captive-portal DNS responder.
//
// Behaviour: for every incoming UDP/53 query, we reflect the question
// section back with QR=1, RA=1, ANCOUNT=1, and append a single A
// record pointing at 192.168.4.1 (the AP IP). TTL is 60 s so phones
// retry after a minute. We don't actually parse the question name —
// any A query to any name gets the same response. That's what we want
// for the captive-portal probe.
//
// One task, one socket, ~512 B max payload. Idle CPU cost when no
// queries: zero (blocked in recvfrom).

#include "captive_dns.h"

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"

static const char *TAG = "DNS";
static volatile bool s_running = false;

#define DNS_PORT          53
#define MAX_DNS_PACKET    512
#define AP_IP_BE          0x0104A8C0u   // 192.168.4.1 in big-endian (network order)

// DNS header layout (RFC 1035 §4.1.1).
typedef struct __attribute__((packed)) {
    uint16_t id;
    uint16_t flags;       // QR/Opcode/AA/TC/RD/RA/Z/RCODE
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
} dns_hdr_t;

// Walk a QNAME starting at offset and return the byte offset just past
// its terminating 0 byte (or 0 on parse error).
static size_t skip_qname(const uint8_t *buf, size_t len, size_t off)
{
    while (off < len) {
        uint8_t lbl = buf[off];
        if (lbl == 0) return off + 1;
        if ((lbl & 0xc0) == 0xc0) return off + 2;     // compressed pointer
        off += 1 + lbl;
    }
    return 0;
}

static void dns_task(void *arg)
{
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed errno=%d", errno);
        s_running = false;
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(DNS_PORT),
        .sin_addr   = { .s_addr = htonl(INADDR_ANY) },
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind(:53) failed errno=%d", errno);
        close(sock);
        s_running = false;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "captive-portal DNS responder listening on UDP/53");

    uint8_t buf[MAX_DNS_PACKET];
    while (1) {
        struct sockaddr_in src;
        socklen_t srclen = sizeof(src);
        int n = recvfrom(sock, buf, sizeof(buf), 0,
                         (struct sockaddr *)&src, &srclen);
        if (n < (int)sizeof(dns_hdr_t)) continue;

        dns_hdr_t *h = (dns_hdr_t *)buf;
        uint16_t flags = ntohs(h->flags);
        uint16_t qd    = ntohs(h->qdcount);
        // Only respond to standard queries (QR=0, OPCODE=0) with at
        // least one question. Other opcodes get silently dropped.
        if ((flags & 0x8000) != 0 || ((flags >> 11) & 0x0f) != 0 || qd == 0) {
            continue;
        }

        // Find the end of the (last) question — just past QNAME + QTYPE(2) + QCLASS(2).
        size_t q_off = sizeof(dns_hdr_t);
        for (uint16_t i = 0; i < qd; i++) {
            size_t after_name = skip_qname(buf, n, q_off);
            if (after_name == 0 || after_name + 4 > (size_t)n) {
                q_off = 0;
                break;
            }
            q_off = after_name + 4;
        }
        if (q_off == 0 || q_off > (size_t)n) continue;

        // Build the response in place. Set flags QR=1, RA=1, RCODE=0.
        h->flags   = htons(0x8180);
        h->ancount = htons(1);
        h->nscount = 0;
        h->arcount = 0;

        // Append a single A record pointing at the AP IP.
        // Name is a compressed pointer to the question name at offset 0x0c.
        uint8_t *ans = buf + q_off;
        size_t room = sizeof(buf) - q_off;
        if (room < 16) continue;
        ans[0] = 0xc0; ans[1] = 0x0c;             // NAME = ptr to offset 12
        ans[2] = 0x00; ans[3] = 0x01;             // TYPE = A
        ans[4] = 0x00; ans[5] = 0x01;             // CLASS = IN
        ans[6] = 0x00; ans[7] = 0x00;             // TTL = 60s (high)
        ans[8] = 0x00; ans[9] = 0x3c;             // TTL = 60s (low)
        ans[10] = 0x00; ans[11] = 0x04;           // RDLENGTH = 4
        uint32_t ip_be = AP_IP_BE;
        memcpy(ans + 12, &ip_be, 4);              // RDATA = 192.168.4.1

        int resp_len = (int)q_off + 16;
        int sent = sendto(sock, buf, resp_len, 0,
                          (struct sockaddr *)&src, srclen);
        if (sent < 0) {
            ESP_LOGW(TAG, "sendto failed errno=%d", errno);
        }
    }
}

esp_err_t captive_dns_start(void)
{
    if (s_running) return ESP_OK;
    s_running = true;

    // DNS responder stack in PSRAM — UDP/53 handler is latency-tolerant
    // (phone captive-portal probe, retries on timeout). Saves ~3 KB
    // internal SRAM for USB DMA pool.
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(dns_task, "captive_dns",
                                                     3072, NULL, 3, NULL,
                                                     tskNO_AFFINITY,
                                                     MALLOC_CAP_SPIRAM);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        s_running = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
