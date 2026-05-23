#include "http_server.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "esp_system.h"

#include "wifi_link.h"
#include "app_config.h"
#include "msg_ring.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HTTP";
static httpd_handle_t s_server = NULL;

static void deferred_reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "rebooting to apply new Wi-Fi config");
    esp_restart();
}

static esp_err_t status_get(httpd_req_t *req)
{
    app_config_t cfg;
    app_config_snapshot(&cfg);

    const esp_app_desc_t *app = esp_app_get_description();

    uint32_t ip = wifi_link_ip_u32();
    char ip_str[16];
    if (wifi_link_is_ap_mode()) {
        // SoftAP default gateway is 192.168.4.1.
        snprintf(ip_str, sizeof(ip_str), "192.168.4.1");
    } else if (ip != 0) {
        snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d",
                 (int)((ip >> 0) & 0xff), (int)((ip >> 8) & 0xff),
                 (int)((ip >> 16) & 0xff), (int)((ip >> 24) & 0xff));
    } else {
        snprintf(ip_str, sizeof(ip_str), "0.0.0.0");
    }

    int64_t uptime_us = esp_timer_get_time();

    // Small fixed JSON. No allocator games; fits comfortably in a
    // single TCP segment.
    char body[512];
    int n = snprintf(body, sizeof(body),
        "{"
            "\"build\":\"%s\","
            "\"build_time\":\"%s\","
            "\"wifi_mode\":\"%s\","
            "\"wifi_ssid\":\"%s\","
            "\"wifi_up\":%s,"
            "\"ip\":\"%s\","
            "\"uptime_s\":%lld,"
            "\"station_id\":\"%s\","
            "\"lo_freq_hz\":%u,"
            "\"sample_rate_hz\":%u"
        "}",
        app->version,
        app->date,
        wifi_link_is_ap_mode() ? "AP" : "STA",
        wifi_link_ssid(),
        wifi_link_is_connected() ? "true" : "false",
        ip_str,
        (long long)(uptime_us / 1000000),
        cfg.station_id,
        (unsigned)cfg.lo_freq_hz,
        (unsigned)cfg.sample_rate_hz);

    if (n < 0 || n >= (int)sizeof(body)) {
        ESP_LOGW(TAG, "status body truncated (n=%d, cap=%d)", n, (int)sizeof(body));
        n = sizeof(body) - 1;
        body[n] = '\0';
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}

// Minimal HTML form for Wi-Fi credentials. Self-contained, no JS, no
// external assets. Posts urlencoded form data to /config.
static const char s_index_html[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Iridium ACARS — config</title>"
    "<style>"
    "body{font-family:system-ui,sans-serif;max-width:480px;margin:2em auto;padding:0 1em;color:#222;background:#fafafa}"
    "h1{font-size:1.3em}"
    "label{display:block;margin:1em 0 .3em;font-size:.9em;color:#555}"
    "input[type=text],input[type=password]{width:100%;padding:.5em;border:1px solid #ccc;border-radius:4px;font-size:1em;box-sizing:border-box}"
    "button{margin-top:1.5em;padding:.7em 1.5em;border:0;background:#1976d2;color:#fff;border-radius:4px;font-size:1em}"
    "small{color:#888}"
    "</style></head><body>"
    "<h1>Iridium ACARS</h1>"
    "<p>Configure Wi-Fi credentials. The device will reboot and connect.</p>"
    "<form method=\"POST\" action=\"/config\">"
    "<label>Wi-Fi SSID</label>"
    "<input type=\"text\" name=\"ssid\" required maxlength=\"32\">"
    "<label>Wi-Fi password</label>"
    "<input type=\"password\" name=\"psk\" maxlength=\"63\">"
    "<button type=\"submit\">Save &amp; reboot</button>"
    "</form>"
    "<p><small>Current status: <a href=\"/status\">/status</a></small></p>"
    "</body></html>";

// Shown only in STA mode (we're already at the form when in AP).
static const char s_index_reset_block[] =
    "<hr style=\"margin-top:2em\">"
    "<form method=\"POST\" action=\"/reset\" onsubmit=\"return confirm('Clear Wi-Fi credentials and reboot to AP mode?');\">"
    "<button type=\"submit\" style=\"background:#a00\">Reset Wi-Fi → AP mode</button>"
    "</form>";

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send_chunk(req, s_index_html, sizeof(s_index_html) - 1);
    if (!wifi_link_is_ap_mode()) {
        httpd_resp_send_chunk(req, s_index_reset_block,
                              sizeof(s_index_reset_block) - 1);
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

// URL-decode a single %XX or '+' from src to dst in-place. Returns bytes
// written. dst may equal src (decode shrinks).
static size_t url_decode(char *dst, const char *src, size_t srclen)
{
    size_t w = 0;
    for (size_t i = 0; i < srclen; i++) {
        char c = src[i];
        if (c == '+') {
            dst[w++] = ' ';
        } else if (c == '%' && i + 2 < srclen) {
            char buf[3] = { src[i+1], src[i+2], 0 };
            dst[w++] = (char)strtol(buf, NULL, 16);
            i += 2;
        } else {
            dst[w++] = c;
        }
    }
    return w;
}

// Extract field 'key' from urlencoded body. Returns ESP_OK on found,
// writes NUL-terminated decoded value into out[0..outsz-1].
static esp_err_t form_field(const char *body, size_t blen,
                            const char *key,
                            char *out, size_t outsz)
{
    size_t klen = strlen(key);
    const char *p = body;
    const char *end = body + blen;
    while (p < end) {
        const char *amp = memchr(p, '&', end - p);
        const char *seg_end = amp ? amp : end;
        const char *eq = memchr(p, '=', seg_end - p);
        if (eq && (size_t)(eq - p) == klen && memcmp(p, key, klen) == 0) {
            size_t vlen = seg_end - (eq + 1);
            if (vlen >= outsz) vlen = outsz - 1;
            size_t w = url_decode(out, eq + 1, vlen);
            if (w >= outsz) w = outsz - 1;
            out[w] = '\0';
            return ESP_OK;
        }
        if (!amp) break;
        p = amp + 1;
    }
    out[0] = '\0';
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t config_post(httpd_req_t *req)
{
    char body[256];
    int total = 0;
    while (total < (int)sizeof(body) - 1) {
        int n = httpd_req_recv(req, body + total, sizeof(body) - 1 - total);
        if (n <= 0) {
            if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
            break;
        }
        total += n;
    }
    body[total] = '\0';

    char ssid[33] = {0};
    char psk[64]  = {0};
    if (form_field(body, total, "ssid", ssid, sizeof(ssid)) != ESP_OK ||
        ssid[0] == '\0') {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "ssid required\n", HTTPD_RESP_USE_STRLEN);
    }
    form_field(body, total, "psk", psk, sizeof(psk));  // psk optional (open AP)

    ESP_LOGI(TAG, "/config POST: saving ssid='%s' (psk %s)",
             ssid, psk[0] ? "set" : "empty");

    esp_err_t r1 = app_config_set_wifi_ssid(ssid);
    esp_err_t r2 = app_config_set_wifi_psk(psk);
    if (r1 != ESP_OK || r2 != ESP_OK) {
        ESP_LOGE(TAG, "NVS write failed: ssid=%s psk=%s",
                 esp_err_to_name(r1), esp_err_to_name(r2));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "nvs write failed\n", HTTPD_RESP_USE_STRLEN);
    }

    // Tell the user, then reboot after a delay so the reply is fully
    // flushed before the SDIO Wi-Fi tears down.
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    const char *ok =
        "<!doctype html><html><body style=\"font-family:system-ui;max-width:480px;margin:2em auto;padding:0 1em\">"
        "<h1>Saved — rebooting</h1>"
        "<p>The device will connect to the Wi-Fi network in a few seconds. "
        "Check your router for the new client, or browse to its address.</p>"
        "</body></html>";
    httpd_resp_send(req, ok, HTTPD_RESP_USE_STRLEN);

    // 1-second deferred restart — let the TCP FIN out before tearing
    // down Wi-Fi.
    xTaskCreate(deferred_reboot_task, "reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

// Escape a string into a JSON value. Writes up to outsz-1 chars + NUL.
// Returns chars written (not counting NUL).
static size_t json_escape(char *out, size_t outsz, const char *in)
{
    size_t w = 0;
    if (outsz == 0) return 0;
    for (; *in && w + 7 < outsz; in++) {
        unsigned char c = (unsigned char)*in;
        switch (c) {
        case '"':  out[w++] = '\\'; out[w++] = '"';  break;
        case '\\': out[w++] = '\\'; out[w++] = '\\'; break;
        case '\n': out[w++] = '\\'; out[w++] = 'n';  break;
        case '\r': out[w++] = '\\'; out[w++] = 'r';  break;
        case '\t': out[w++] = '\\'; out[w++] = 't';  break;
        default:
            if (c < 0x20) {
                // \u00XX
                static const char hex[] = "0123456789abcdef";
                out[w++] = '\\'; out[w++] = 'u'; out[w++] = '0'; out[w++] = '0';
                out[w++] = hex[(c >> 4) & 0xf];
                out[w++] = hex[c & 0xf];
            } else {
                out[w++] = (char)c;
            }
        }
    }
    out[w] = '\0';
    return w;
}

static esp_err_t messages_get(httpd_req_t *req)
{
    uint64_t since_id = 0;
    char qbuf[64];
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        char val[24];
        if (httpd_query_key_value(qbuf, "since", val, sizeof(val)) == ESP_OK) {
            since_id = strtoull(val, NULL, 10);
        }
    }

    static acars_msg_t s_snap[MSG_RING_CAPACITY];   // ~9 KB; fine on logger task stack? no — too big.
    // BSS-allocated above to avoid the 6 KB http_server task stack.
    size_t n = msg_ring_snapshot(since_id, s_snap, MSG_RING_CAPACITY);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    // Stream: {"total":N,"messages":[ {...}, {...} ]}
    char chunk[600];
    int len = snprintf(chunk, sizeof(chunk),
                       "{\"total\":%llu,\"messages\":[",
                       (unsigned long long)msg_ring_total());
    httpd_resp_send_chunk(req, chunk, len);

    char esc_txt[2 * MSG_RING_TXT_MAX + 8];
    char esc_flight[16];
    char esc_msgnum[16];
    char esc_label[8];
    for (size_t i = 0; i < n; i++) {
        const acars_msg_t *m = &s_snap[i];

        char label_buf[3] = { m->label[0], m->label[1], 0 };
        json_escape(esc_label,  sizeof(esc_label),  label_buf);
        json_escape(esc_msgnum, sizeof(esc_msgnum), m->msg_num);
        json_escape(esc_flight, sizeof(esc_flight), m->flight_id);
        json_escape(esc_txt,    sizeof(esc_txt),    m->txt);

        len = snprintf(chunk, sizeof(chunk),
            "%s{"
                "\"id\":%llu,"
                "\"t_us\":%llu,"
                "\"dir\":\"%s\","
                "\"mode\":\"%c\","
                "\"label\":\"%s\","
                "\"block\":\"%c\","
                "\"msg_num\":\"%s\","
                "\"flight\":\"%s\","
                "\"crc\":%s,"
                "\"peak_bin\":%ld,"
                "\"snr_db\":%.1f,"
                "\"txt\":\"%s\""
            "}",
            (i == 0) ? "" : ",",
            (unsigned long long)m->id,
            (unsigned long long)m->timestamp_us,
            m->uplink ? "UL" : "DL",
            m->mode,
            esc_label,
            m->block_id,
            esc_msgnum,
            esc_flight,
            m->crc_ok ? "true" : "false",
            (long)m->peak_bin,
            (double)m->snr_db,
            esc_txt);
        if (len > 0) {
            httpd_resp_send_chunk(req, chunk, len);
        }
    }
    httpd_resp_send_chunk(req, "]}", 2);
    return httpd_resp_send_chunk(req, NULL, 0);   // end of chunked response
}

static esp_err_t reset_post(httpd_req_t *req)
{
    ESP_LOGW(TAG, "/reset POST: clearing Wi-Fi NVS + rebooting to AP mode");

    esp_err_t r1 = app_config_set_wifi_ssid("");
    esp_err_t r2 = app_config_set_wifi_psk("");
    if (r1 != ESP_OK || r2 != ESP_OK) {
        ESP_LOGE(TAG, "NVS clear failed: ssid=%s psk=%s",
                 esp_err_to_name(r1), esp_err_to_name(r2));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "nvs clear failed\n", HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    const char *ok =
        "<!doctype html><html><body style=\"font-family:system-ui;max-width:480px;margin:2em auto;padding:0 1em\">"
        "<h1>Reset — rebooting to AP mode</h1>"
        "<p>Wi-Fi credentials cleared. The device will reboot and come "
        "back up as an open AP. Re-join it to configure new credentials.</p>"
        "</body></html>";
    httpd_resp_send(req, ok, HTTPD_RESP_USE_STRLEN);

    xTaskCreate(deferred_reboot_task, "reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

esp_err_t http_server_start(void)
{
    if (s_server) return ESP_OK;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port    = 80;
    cfg.max_uri_handlers = 8;
    cfg.lru_purge_enable = true;
    cfg.stack_size     = 6144;
    cfg.task_priority  = 4;

    esp_err_t r = httpd_start(&s_server, &cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(r));
        s_server = NULL;
        return r;
    }

    httpd_uri_t routes[] = {
        { .uri = "/",         .method = HTTP_GET,  .handler = index_get,    .user_ctx = NULL },
        { .uri = "/status",   .method = HTTP_GET,  .handler = status_get,   .user_ctx = NULL },
        { .uri = "/messages", .method = HTTP_GET,  .handler = messages_get, .user_ctx = NULL },
        { .uri = "/config",   .method = HTTP_POST, .handler = config_post,  .user_ctx = NULL },
        { .uri = "/reset",    .method = HTTP_POST, .handler = reset_post,   .user_ctx = NULL },
    };
    for (size_t i = 0; i < sizeof(routes)/sizeof(routes[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &routes[i]));
    }

    ESP_LOGI(TAG, "HTTP server up on port 80 — GET /, /status, /messages; POST /config");
    return ESP_OK;
}
