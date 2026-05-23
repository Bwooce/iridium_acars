#include "http_server.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_app_desc.h"
#include "esp_timer.h"

#include "wifi_link.h"
#include "app_config.h"

static const char *TAG = "HTTP";
static httpd_handle_t s_server = NULL;

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

    httpd_uri_t status_uri = {
        .uri      = "/status",
        .method   = HTTP_GET,
        .handler  = status_get,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &status_uri));

    ESP_LOGI(TAG, "HTTP server up on port 80 — GET /status");
    return ESP_OK;
}
