// HTTP server for the C6 companion. Static asset serving from
// SPIFFS partition "storage" + REST API endpoints over the
// message/stats ring buffers and Wi-Fi/system status.

#include "web.h"
#include "msg_ring.h"
#include "wifi.h"
#include "uart_link.h"
#include "iridium_protocol.h"

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "nvs.h"

static const char *TAG = "WEB";
static const char *NVS_NS = "iridium_c";

// ---------- helpers -------------------------------------------------------

static esp_err_t respond_json(httpd_req_t *req, const char *json, size_t len)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, len);
}

// Crude JSON escape for arbitrary text. Skips control chars.
static void json_escape(char *out, size_t cap, const char *in, size_t in_len)
{
    size_t o = 0;
    for (size_t i = 0; i < in_len && o + 2 < cap; i++) {
        char c = in[i];
        if (c == 0) break;
        if (c == '"' || c == '\\') {
            if (o + 2 >= cap) break;
            out[o++] = '\\'; out[o++] = c;
        } else if ((uint8_t)c < 0x20) {
            continue;   // strip control
        } else {
            out[o++] = c;
        }
    }
    out[o] = '\0';
}

// ---------- /api/messages -------------------------------------------------

static esp_err_t h_messages(httpd_req_t *req)
{
    uint32_t since_seq = 0;
    char qbuf[64];
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(qbuf, "since", val, sizeof(val)) == ESP_OK) {
            since_seq = (uint32_t)strtoul(val, NULL, 10);
        }
    }
    irp_acars_msg_t buf[32];
    int n = msg_ring_snapshot_acars(buf, 32, since_seq);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char line[768];
    httpd_resp_sendstr_chunk(req, "[");
    for (int i = 0; i < n; i++) {
        char text_esc[IRP_ACARS_TEXT_LEN * 2 + 4];
        json_escape(text_esc, sizeof(text_esc), buf[i].text,
                    strnlen(buf[i].text, IRP_ACARS_TEXT_LEN));
        int len = snprintf(line, sizeof(line),
            "%s{\"seq\":%u,\"ts_us\":%llu,\"dir\":\"%s\",\"mode\":\"%c\","
            "\"label\":\"%.2s\",\"flight\":\"%.6s\",\"msg_num\":\"%.4s\","
            "\"text\":\"%s\",\"snr_db\":%.1f,\"freq_hz\":%u}",
            (i == 0) ? "" : ",",
            (unsigned)buf[i].seq, (unsigned long long)buf[i].ts_us,
            (buf[i].direction == 1) ? "UL" : "DL",
            buf[i].mode ? buf[i].mode : '?',
            buf[i].label, buf[i].flight, buf[i].msg_num,
            text_esc, (double)buf[i].snr_db, (unsigned)buf[i].freq_hz);
        if (len > 0 && len < (int)sizeof(line)) {
            httpd_resp_sendstr_chunk(req, line);
        }
    }
    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// ---------- /api/stats ----------------------------------------------------

static esp_err_t h_stats(httpd_req_t *req)
{
    irp_status_snap_t hist[STATS_RING_CAP];
    int n = msg_ring_snapshot_status(hist, STATS_RING_CAP);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr_chunk(req, "{\"history\":[");
    char line[256];
    for (int i = 0; i < n; i++) {
        int len = snprintf(line, sizeof(line),
            "%s{\"rate_mb_s\":%.2f,\"dsp_cap\":%.1f,\"worker_cap\":%.1f,"
            "\"drops\":%u,\"frames\":%u,\"processed\":%u,\"acars_decoded\":%u}",
            (i == 0) ? "" : ",",
            (double)hist[i].rate_mb_s, (double)hist[i].dsp_cap,
            (double)hist[i].worker_cap,
            (unsigned)hist[i].drops, (unsigned)hist[i].frames,
            (unsigned)hist[i].processed, (unsigned)hist[i].acars_decoded);
        if (len > 0 && len < (int)sizeof(line)) {
            httpd_resp_sendstr_chunk(req, line);
        }
    }
    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

// ---------- /api/status ---------------------------------------------------

static esp_err_t h_status(httpd_req_t *req)
{
    msg_ring_stats_t ms;
    msg_ring_get_stats(&ms);
    uint64_t now_ms = esp_timer_get_time() / 1000;
    char buf[512];
    int len = snprintf(buf, sizeof(buf),
        "{\"wifi_mode\":\"%s\",\"wifi_rssi\":%d,\"p4_ready\":%s,"
        "\"c6_uptime_ms\":%llu,\"acars_total\":%u,\"status_total\":%u,"
        "\"last_acars_seq\":%u}",
        wifi_mode_str(), wifi_rssi(),
        uart_link_p4_ready() ? "true" : "false",
        (unsigned long long)now_ms,
        (unsigned)ms.acars_total,
        (unsigned)ms.status_total,
        (unsigned)ms.last_acars_seq);
    return respond_json(req, buf, len);
}

// ---------- /api/config (GET) ---------------------------------------------

static esp_err_t h_config_get(httpd_req_t *req)
{
    nvs_handle_t h;
    char ssid[33] = {0};
    size_t s_len = sizeof(ssid);
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_str(h, "wifi_ssid", ssid, &s_len);
        nvs_close(h);
    }
    char buf[256];
    int len = snprintf(buf, sizeof(buf),
        "{\"wifi_ssid\":\"%s\"}",  // wifi_psk never exposed
        ssid);
    return respond_json(req, buf, len);
}

// ---------- /api/config (POST: save Wi-Fi creds) --------------------------
//
// Simple body parser for application/x-www-form-urlencoded
// Expected: wifi_ssid=...&wifi_psk=...
static int kv_find(const char *body, int blen, const char *key, char *out, int cap)
{
    int klen = strlen(key);
    int i = 0;
    while (i + klen < blen) {
        bool match = (i == 0 || body[i - 1] == '&');
        if (match && !memcmp(body + i, key, klen) && body[i + klen] == '=') {
            i += klen + 1;
            int j = 0;
            while (i < blen && body[i] != '&' && j < cap - 1) {
                char c = body[i++];
                if (c == '+') c = ' ';
                if (c == '%' && i + 1 < blen) {
                    char h[3] = { body[i], body[i+1], 0 };
                    c = (char)strtol(h, NULL, 16);
                    i += 2;
                }
                out[j++] = c;
            }
            out[j] = '\0';
            return j;
        }
        i++;
    }
    return -1;
}

static esp_err_t h_config_post(httpd_req_t *req)
{
    char body[256];
    int n = httpd_req_recv(req, body, sizeof(body) - 1);
    if (n <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no body");
        return ESP_FAIL;
    }
    body[n] = '\0';
    char ssid[33] = {0};
    char psk [65] = {0};
    kv_find(body, n, "wifi_ssid", ssid, sizeof(ssid));
    kv_find(body, n, "wifi_psk",  psk,  sizeof(psk));

    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "nvs");
        return ESP_FAIL;
    }
    if (ssid[0]) nvs_set_str(h, "wifi_ssid", ssid);
    if (psk[0])  nvs_set_str(h, "wifi_psk",  psk);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "Saved Wi-Fi creds: ssid='%s' (reboot to apply)", ssid);
    return respond_json(req, "{\"ok\":true}", 11);
}

// ---------- Static SPIFFS file serving ------------------------------------

static esp_err_t h_root(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/index.html", "r");
    if (!f) {
        // Fallback inline HTML if SPIFFS not provisioned. Tiny page
        // pointing at the REST API.
        const char *html =
            "<!doctype html><html><body><h1>Iridium ACARS (C6)</h1>"
            "<p>SPIFFS not provisioned. Try "
            "<a href=\"/api/status\">/api/status</a>, "
            "<a href=\"/api/messages\">/api/messages</a>, "
            "<a href=\"/api/stats\">/api/stats</a>."
            "</body></html>";
        return httpd_resp_send(req, html, strlen(html));
    }
    httpd_resp_set_type(req, "text/html");
    char buf[512];
    size_t r;
    while ((r = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, r) != ESP_OK) break;
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// Captive portal: every unknown URL gets the root page. Phones
// detect this and pop the portal UI automatically.
static esp_err_t h_captive_fallback(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}

// ---------- Init ----------------------------------------------------------

static esp_err_t spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 4,
        .format_if_mount_failed = true,
    };
    esp_err_t r = esp_vfs_spiffs_register(&conf);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "SPIFFS mount failed (%s); falling back to inline HTML",
                 esp_err_to_name(r));
    } else {
        size_t used = 0, total = 0;
        esp_spiffs_info("storage", &total, &used);
        ESP_LOGI(TAG, "SPIFFS mounted: %u/%u bytes used", (unsigned)used, (unsigned)total);
    }
    return ESP_OK;
}

esp_err_t web_init(void)
{
    spiffs_mount();

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.max_uri_handlers = 12;

    httpd_handle_t srv = NULL;
    esp_err_t r = httpd_start(&srv, &cfg);
    if (r != ESP_OK) { ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(r)); return r; }

    httpd_uri_t root      = { .uri = "/",                  .method = HTTP_GET,  .handler = h_root,        };
    httpd_uri_t messages  = { .uri = "/api/messages",      .method = HTTP_GET,  .handler = h_messages,    };
    httpd_uri_t stats     = { .uri = "/api/stats",         .method = HTTP_GET,  .handler = h_stats,       };
    httpd_uri_t status    = { .uri = "/api/status",        .method = HTTP_GET,  .handler = h_status,      };
    httpd_uri_t cfg_get   = { .uri = "/api/config",        .method = HTTP_GET,  .handler = h_config_get,  };
    httpd_uri_t cfg_post  = { .uri = "/api/config",        .method = HTTP_POST, .handler = h_config_post, };
    httpd_uri_t fallback  = { .uri = "/*",                 .method = HTTP_GET,  .handler = h_captive_fallback };

    httpd_register_uri_handler(srv, &root);
    httpd_register_uri_handler(srv, &messages);
    httpd_register_uri_handler(srv, &stats);
    httpd_register_uri_handler(srv, &status);
    httpd_register_uri_handler(srv, &cfg_get);
    httpd_register_uri_handler(srv, &cfg_post);
    httpd_register_uri_handler(srv, &fallback);

    ESP_LOGI(TAG, "HTTP server up");
    return ESP_OK;
}
