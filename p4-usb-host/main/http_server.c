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
#include "esp_attr.h" // EXT_RAM_BSS_ATTR — big static buffers go to PSRAM .bss

#include "wifi_link.h"
#include "app_config.h"
#include "msg_ring.h"
#include "frame_decoder.h"
#include "ota_runner.h"
#include "class_driver.h" // class_driver_prepare_for_reboot()
#include "scanner.h"      // scanner_scan() — /scan sustained-hop test endpoint
#include "autotune.h"     // autotune_run_manual() — /gaincal manual trigger
#include "sd_log.h"
#include "sd_capture.h"
#include "acars_push.h"
#include "esp_libusb.h"
#include "fault_inject.h"
#include "worker_core1.h"
#include "worker_dcfine.h"
#include "status_logger.h" // status_logger_get_last() — /status HTML dashboard
#include "aggregator_ingest.h"
#include "frame_link.h"
#include "dsp_processor.h"
#include "signal_buffer.h"
#include "ingest_core1.h"
#include "burst_pipeline.h"

#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"

static const char    *TAG      = "HTTP";
static httpd_handle_t s_server = NULL;

// Defined below (near /messages); fwd-declared so status_get / ota_get can
// escape user-controlled strings (SSID, out_host, ota_url, error text)
// before embedding them in JSON (M17).
static size_t json_escape(char *out, size_t outsz, const char *in);

// Human-facing HTML dashboard for GET /status when the client is a browser
// (Accept: text/html). Defined after the shared page-chrome helpers.
static esp_err_t status_html_get(httpd_req_t *req);

// Pre-allocated PSRAM read buffer for /capture/file (T49b: was DMA-INT,
// moved to PSRAM to return 4 KB to the tight USB-pool budget). SDMMC
// reads into it via the driver's bounce path; fine for this rare manual
// download. Still pre-allocated at http_server_start so the handler
// never does a lazy per-download alloc.
static uint8_t *s_download_buf = NULL;
#define DOWNLOAD_BUF_BYTES 4096

// Max URI handlers the httpd will accept. Used both to size the IDF
// httpd slot table AND in a static_assert on the routes[] array length,
// so adding a route past the limit breaks the build instead of panic-
// looping at boot. Each slot is ~32 bytes; 32 slots = ~1 KB negligible.
#define HTTPD_URI_LIMIT 32

// NVS-write + reboot helper. MUST run with an internal-SRAM stack:
// nvs_commit() takes spi_flash_disable_interrupts_caches_and_other_cpu(),
// which makes PSRAM (cached) inaccessible. A task whose own stack
// lives in PSRAM hits an assert and aborts the moment it dereferences
// any local during the cache-disabled window. The httpd server task
// is PSRAM-stacked (cfg.task_caps), so it cannot do NVS writes
// itself — it must hand the work off to this task.
typedef struct {
    char     ssid[33];
    char     psk[64];
    char     out_host[64];
    uint16_t out_port;
    char     ota_url[128];
    bool     bias_tee;
    bool     clear_only; // true = reset_post path (clear SSID+PSK, ignore other fields)
} nvs_save_args_t;

static void nvs_save_and_reboot_task(void *arg)
{
    nvs_save_args_t *a = (nvs_save_args_t *)arg;

    esp_err_t r1, r2, r3 = ESP_OK, r4 = ESP_OK, r5 = ESP_OK, r6 = ESP_OK;
    if (a->clear_only) {
        r1 = app_config_set_wifi_ssid("");
        r2 = app_config_set_wifi_psk("");
    } else {
        r1 = app_config_set_wifi_ssid(a->ssid);
        r2 = app_config_set_wifi_psk(a->psk);
        r3 = app_config_set_out_host(a->out_host);
        r4 = app_config_set_out_port(a->out_port);
        r5 = app_config_set_ota_url(a->ota_url);
        r6 = app_config_set_bias_tee(a->bias_tee);
    }
    if (r1 || r2 || r3 || r4 || r5 || r6) {
        ESP_LOGE(TAG, "NVS write failed: ssid=%s psk=%s host=%s port=%s ota=%s bias=%s",
                 esp_err_to_name(r1), esp_err_to_name(r2),
                 esp_err_to_name(r3), esp_err_to_name(r4),
                 esp_err_to_name(r5), esp_err_to_name(r6));
    }

    free(a);

    // Brief grace period so the already-sent HTTP response + TCP FIN
    // get out before Wi-Fi tears down.
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "rebooting to apply new config");
    class_driver_prepare_for_reboot(); // park tuner so the dongle survives the reboot
    esp_restart();
}

// /tune apply task. The NVS write (app_config_set_lo_freq_hz → nvs_commit)
// does a flash op that disables the cache, so it MUST run on a task with an
// internal-SRAM stack — NOT the httpd task, whose stack is in PSRAM (a PSRAM
// stack faults the moment the cache is disabled; same footgun as #92 / the
// /config-save crash). Plain xTaskCreate gives an internal stack. The LO is
// programmed at stream start, so reboot to apply.
static void tune_apply_reboot_task(void *arg)
{
    uint32_t  hz = (uint32_t)(uintptr_t)arg;
    esp_err_t r  = app_config_set_lo_freq_hz(hz);
    ESP_LOGI(TAG, "/tune: set lo_freq_hz=%u (%s) — rebooting to apply",
             (unsigned)hz, esp_err_to_name(r));
    vTaskDelay(pdMS_TO_TICKS(500));
    class_driver_prepare_for_reboot(); // park tuner so the dongle survives the reboot
    esp_restart();
}

static esp_err_t status_get(httpd_req_t *req)
{
    // Content negotiation: a browser (Accept: text/html) gets the human
    // dashboard; curl / monitors / Accept:*/* fall through to the JSON
    // below, UNCHANGED — the existing /status API is preserved (no known
    // consumer sends Accept: text/html; verified against scripts/).
    //
    // Don't gate on the ESP_OK return: browsers send long Accept headers
    // that overflow this buffer and return ESP_ERR_HTTPD_RESULT_TRUNC, but
    // "text/html" leads the value so the (NUL-terminated) truncated copy
    // still contains it. On not-found the buffer stays "" (zero-init), so
    // an unconditional strstr is correct either way.
    char accept[160] = {0};
    httpd_req_get_hdr_value_str(req, "Accept", accept, sizeof(accept));
    if (strstr(accept, "text/html")) {
        return status_html_get(req);
    }

    app_config_t cfg;
    app_config_snapshot(&cfg);

    const esp_app_desc_t *app = esp_app_get_description();

    uint32_t ip = wifi_link_ip_u32();
    char     ip_str[16];
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
    frame_decoder_class_counts_t cc = {0};
    frame_decoder_get_class_counts(&cc);
    uint64_t acars_total = frame_decoder_acars_decoded_total();
    uint64_t sbd_total   = frame_decoder_sbd_complete_total();
    uint64_t msgs_total  = msg_ring_total();
    uint32_t rate_1h = 0, rate_24h = 0;
    frame_decoder_get_rolling_rates(&rate_1h, &rate_24h); // #117
    sd_log_stats_t sd = {0};
    sd_log_get_stats(&sd);
    usb_stream_totals_t usbt = {0};
    esp_libusb_get_stream_totals(&usbt);

    // Health watchdog (#104 gateway + #105 USB stream) state.
    uint32_t wdt_gw            = 0;
    bool     wdt_armed         = false;
    int      wdt_fails         = 0;
    bool     wdt_stream_live   = false;
    int      wdt_stream_stalls = 0;
    wifi_link_wdt_status(&wdt_gw, &wdt_armed, &wdt_fails,
                         &wdt_stream_live, &wdt_stream_stalls);

    // JSON-escape the free-form string fields (M17): SSID, push host and
    // OTA URL are operator input; mount_error carries errno/driver text.
    // Any embedded quote/backslash would otherwise break the JSON. 2× the
    // source size + 1 covers the worst case (every char escaping to two).
    char ssid_esc[2 * 33 + 1]; // wifi_link_ssid() is a char[33] SSID
    char host_esc[2 * sizeof(cfg.out_host) + 1];
    char ota_esc[2 * sizeof(cfg.ota_url) + 1];
    char mnt_err_esc[2 * sizeof(sd.mount_error) + 1];
    char station_id_esc[2 * sizeof(cfg.station_id) + 1];
    json_escape(ssid_esc, sizeof(ssid_esc), wifi_link_ssid());
    json_escape(host_esc, sizeof(host_esc), cfg.out_host);
    json_escape(ota_esc, sizeof(ota_esc), cfg.ota_url);
    json_escape(mnt_err_esc, sizeof(mnt_err_esc), sd.mount_error);
    json_escape(station_id_esc, sizeof(station_id_esc), cfg.station_id);

    char body[1600];
    int  n = snprintf(body, sizeof(body),
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
                       "\"sample_rate_hz\":%u,"
                       "\"bias_tee\":%s,"
                       "\"udp_push\":{\"host\":\"%s\",\"port\":%u,\"enabled\":%s},"
                       "\"ota_url\":\"%s\","
                       "\"usb\":{"
                       "\"completed\":%llu,\"rb_full_drops\":%llu,"
                       "\"status_errors\":%llu,\"short_xfers\":%llu"
                       "},"
                       "\"decode\":{"
                       "\"messages_total\":%llu,"
                       "\"acars_decoded\":%llu,"
                       "\"sbd_complete\":%llu,"
                       "\"rate_1h\":%u,\"rate_24h\":%u,"
                       "\"frames\":{"
                       "\"ms\":%llu,\"tl\":%llu,\"bc\":%llu,"
                       "\"lw_da\":%llu,\"lw_other\":%llu,\"unknown\":%llu"
                       "}"
                       "},"
                       "\"health_wdt\":{\"gw\":\"%u.%u.%u.%u\",\"gw_armed\":%s,\"gw_fails\":%d,"
                       "\"stream_live\":%s,\"stream_stalls\":%d},"
                       "\"sd\":{"
                       "\"mounted\":%s,\"log_open\":%s,"
                       "\"messages_written\":%u,\"bytes_written\":%llu,"
                       "\"write_errors\":%u,"
                       "\"log_path\":\"%s\",\"mount_error\":\"%s\""
                       "}"
                       "}",
                      app->version,
                      app->date,
                     wifi_link_is_ap_mode() ? "AP" : "STA",
                      ssid_esc,
                     wifi_link_is_connected() ? "true" : "false",
                      ip_str,
                      (long long)(uptime_us / 1000000),
                      station_id_esc,
                      (unsigned)cfg.lo_freq_hz,
                      (unsigned)cfg.sample_rate_hz,
                     cfg.bias_tee ? "true" : "false",
                      host_esc,
                      (unsigned)cfg.out_port,
                     (cfg.out_host[0] && cfg.out_port) ? "true" : "false",
                      ota_esc,
                      (unsigned long long)usbt.completed,
                      (unsigned long long)usbt.rb_full_drops,
                      (unsigned long long)usbt.status_errors,
                      (unsigned long long)usbt.short_xfers,
                      (unsigned long long)msgs_total,
                      (unsigned long long)acars_total,
                      (unsigned long long)sbd_total,
                      (unsigned)rate_1h, (unsigned)rate_24h,
                      (unsigned long long)cc.ms, (unsigned long long)cc.tl,
                      (unsigned long long)cc.bc, (unsigned long long)cc.lw_da,
                      (unsigned long long)cc.lw_other, (unsigned long long)cc.unknown,
                      (unsigned)(wdt_gw & 0xff), (unsigned)((wdt_gw >> 8) & 0xff),
                      (unsigned)((wdt_gw >> 16) & 0xff), (unsigned)((wdt_gw >> 24) & 0xff),
                     wdt_armed ? "true" : "false", wdt_fails,
                     wdt_stream_live ? "true" : "false", wdt_stream_stalls,
                     sd.mounted ? "true" : "false",
                     sd.log_open ? "true" : "false",
                      (unsigned)sd.messages_written,
                      (unsigned long long)sd.bytes_written,
                      (unsigned)sd.write_errors,
                      sd.log_path,
                      mnt_err_esc);

    if (n < 0 || n >= (int)sizeof(body)) {
        ESP_LOGW(TAG, "status body truncated (n=%d, cap=%d)", n, (int)sizeof(body));
        n       = sizeof(body) - 1;
        body[n] = '\0';
    }

#if !CONFIG_DEVICE_ROLE_STANDALONE
    // PDU-link health (#138): worker output-ring depth/drops and, on the
    // aggregator, per-source liveness. Spliced in before the root closing
    // brace so the JSON shape is unchanged for STANDALONE builds.
    if (n > 1 && body[n - 1] == '}') {
        aggregator_ingest_stats_t ai;
        aggregator_ingest_get_stats(&ai);
        uint64_t now_us = (uint64_t)esp_timer_get_time();
        n--; // drop the root closing brace; re-added below
        int m = snprintf(body + n, sizeof(body) - n,
                         ",\"pdu_link\":{\"forwarded\":%u,\"queue_depth\":%u,"
                         "\"dropped\":%u,\"sources\":[",
                         (unsigned)ai.forwarded, (unsigned)ai.pdu_queue_depth,
                         (unsigned)ai.pdu_dropped);
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
        for (uint32_t i = 0; i < ai.n_sources && n < (int)sizeof(body); i++) {
            uint64_t age_ms = ai.sources[i].last_seen_us
                                  ? (now_us - ai.sources[i].last_seen_us) / 1000
                                  : 0;
            m               = snprintf(body + n, sizeof(body) - n,
                                       "%s{\"id\":\"%08lx\",\"count\":%u,\"age_ms\":%llu}",
                         i ? "," : "", (unsigned long)ai.sources[i].source_id,
                                       (unsigned)ai.sources[i].count, (unsigned long long)age_ms);
            if (m > 0) {
                n += m;
                if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
            }
        }
        // Close sources[], then the SPI transport counters (#136).
        frame_link_stats_t fl;
        frame_link_get_stats(&fl);
        m = snprintf(body + n, sizeof(body) - n,
                     "],\"spi\":{\"tx\":%u,\"rx\":%u,\"crc_err\":%u,"
                     "\"bus_err\":%u,\"queue_full\":%u}}}",
                     (unsigned)fl.frames_tx, (unsigned)fl.frames_rx,
                     (unsigned)fl.crc_errors, (unsigned)fl.bus_errors,
                     (unsigned)fl.queue_full);
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
        body[n] = '\0';
    }
#endif

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}

// /diag/histograms (#116). Cumulative-since-boot SNR + BCH histograms
// for operator diagnostics. Closes the "/status can't tell antenna-
// empty from demod-broken" gap. UW Hamming histogram is deferred —
// requires decoded_frame_t API change to plumb dl_diffs back.
static esp_err_t diag_histograms_get(httpd_req_t *req)
{
    worker_histograms_t h = {0};
    worker_core1_get_histograms(&h);
    uint32_t bp_stage_us[10] = {0};
    uint32_t bp_first_calls = 0, bp_retry_calls = 0;
    burst_pipeline_get_stage_us(bp_stage_us, &bp_first_calls, &bp_retry_calls); // P1.5c

    char body[3072]; // P1.5c: grew from 2048 to fit snr_pushed/duration_pushed/stage_us
    int  n = 0;
    int  m;
    m = snprintf(body + n, sizeof(body) - n,
                 "{\"snr_total\":%u,\"bch_total\":%u,\"freq_total\":%u,"
                 "\"snr_bin_dB_width\":1,\"snr\":[",
                 (unsigned)h.snr_total, (unsigned)h.bch_total,
                 (unsigned)h.freq_total);
    if (m > 0) {
        n += m;
        if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
    }
    for (int i = 0; i < 32; i++) {
        if (n >= (int)sizeof(body) - 1) break;
        m = snprintf(body + n, sizeof(body) - n,
                     "%s%u", i ? "," : "", (unsigned)h.snr[i]);
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
    }
    // bch layout: bch[(e1+1)*4 + (e2+1)] for e in {-1=fail, 0,1,2=corrected}.
    if (n < (int)sizeof(body) - 1) {
        m = snprintf(body + n, sizeof(body) - n,
                     "],\"bch_index\":\"(e1+1)*4+(e2+1), e in {-1=fail,0,1,2}\","
                     "\"bch\":[");
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
    }
    for (int i = 0; i < 16; i++) {
        if (n >= (int)sizeof(body) - 1) break;
        m = snprintf(body + n, sizeof(body) - n,
                     "%s%u", i ? "," : "", (unsigned)h.bch[i]);
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
    }
    // freq layout (T59): band occupancy of ALL detections. bin i spans
    // [-FS/2 + i·62500, ...) Hz relative to the LO (FS=2.5 MHz, 40 bins).
    if (n < (int)sizeof(body) - 1) {
        m = snprintf(body + n, sizeof(body) - n,
                     "],\"freq_bin_Hz_width\":62500,\"freq_bin0_Hz\":-1250000,"
                     "\"freq\":[");
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
    }
    for (int i = 0; i < 40; i++) {
        if (n >= (int)sizeof(body) - 1) break;
        m = snprintf(body + n, sizeof(body) - n,
                     "%s%u", i ? "," : "", (unsigned)h.freq[i]);
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
    }
    // P1.5c: push-side SNR + duration-class histograms — same bin layout
    // as snr[] above, but recorded for EVERY burst pushed to the worker
    // PQ (not just what got popped), so the evicted/shed population is
    // visible. duration_pushed[0]=impulse-length, [1]=plausible-length
    // (BURST_DURATION_CLASS_MIN_SAMPLES, worker_core1.c).
    if (n < (int)sizeof(body) - 1) {
        m = snprintf(body + n, sizeof(body) - n,
                     "],\"snr_pushed_total\":%u,\"duration_pushed_total\":%u,"
                     "\"snr_pushed\":[",
                     (unsigned)h.snr_pushed_total, (unsigned)h.duration_pushed_total);
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
    }
    for (int i = 0; i < 32; i++) {
        if (n >= (int)sizeof(body) - 1) break;
        m = snprintf(body + n, sizeof(body) - n,
                     "%s%u", i ? "," : "", (unsigned)h.snr_pushed[i]);
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
    }
    if (n < (int)sizeof(body) - 1) {
        m = snprintf(body + n, sizeof(body) - n,
                     "],\"duration_pushed_index\":\"0=impulse-length,1=plausible-length\","
                     "\"duration_pushed\":[%u,%u]",
                     (unsigned)h.duration_pushed[0], (unsigned)h.duration_pushed[1]);
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
    }
    // P1.5c: burst_pipeline per-stage wall-time accumulators (read-and-
    // reset; caller computes rates from repeated polls). Order matches
    // burst_pipeline_get_stage_us's doc comment: D13, CFO, PREROT, RRC,
    // first try_decode_frame, retry-loop total, TDF_UW, TDF_PREROT,
    // TDF_DECIM, TDF_QPSK.
    if (n < (int)sizeof(body) - 1) {
        m = snprintf(body + n, sizeof(body) - n,
                     ",\"stage_us_index\":\"D13,CFO,PREROT,RRC,TDF_first,TDF_retry,"
                     "TDF_UW,TDF_PREROT,TDF_DECIM,TDF_QPSK\","
                     "\"stage_us\":[%u,%u,%u,%u,%u,%u,%u,%u,%u,%u],"
                     "\"stage_first_calls\":%u,\"stage_retry_calls\":%u",
                     (unsigned)bp_stage_us[0], (unsigned)bp_stage_us[1],
                     (unsigned)bp_stage_us[2], (unsigned)bp_stage_us[3],
                     (unsigned)bp_stage_us[4], (unsigned)bp_stage_us[5],
                     (unsigned)bp_stage_us[6], (unsigned)bp_stage_us[7],
                     (unsigned)bp_stage_us[8], (unsigned)bp_stage_us[9],
                     (unsigned)bp_first_calls, (unsigned)bp_retry_calls);
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
    }
    if (n < (int)sizeof(body) - 1) {
        m = snprintf(body + n, sizeof(body) - n, "}\n");
        if (m > 0) {
            n += m;
            if (n >= (int)sizeof(body)) n = sizeof(body) - 1;
        }
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}

// /diag/dcfine — fine near-DC occupancy histogram (WORKER_DCFINE_BINS
// buckets, ~1221 Hz each, DC ±234 kHz). Separate endpoint because the
// 384-value array does not fit /diag/histograms' shared 3072-byte body.
static esp_err_t diag_dcfine_get(httpd_req_t *req)
{
    static uint32_t dc[WORKER_DCFINE_BINS];
    uint32_t        total = 0;
    worker_core1_get_dcfine(dc, WORKER_DCFINE_BINS, &total);

    static EXT_RAM_BSS_ATTR char body[4096];
    int         n = 0, m;
    // bin0_Hz = -HALF * width; width = WORKER_DCFINE_BIN_HZ_REPORT = round(FS/N) = 1221.
    // Consumer: offset_Hz(i) = bin0_Hz + i*width; i=HALF is DC.
    m = snprintf(body, sizeof(body),
                 "{\"dcfine_bin0_Hz\":%d,\"dcfine_bin_Hz_width\":%d,"
                 "\"dcfine_total\":%u,\"dcfine\":[",
                 WORKER_DCFINE_BIN0_HZ_REPORT, WORKER_DCFINE_BIN_HZ_REPORT, (unsigned)total);
    if (m > 0) n = m;
    for (int i = 0; i < WORKER_DCFINE_BINS; i++) {
        if (n >= (int)sizeof(body) - 2) break;
        m = snprintf(body + n, sizeof(body) - n, "%s%u", i ? "," : "", (unsigned)dc[i]);
        if (m > 0) n += m;
    }
    if (n < (int)sizeof(body) - 2) n += snprintf(body + n, sizeof(body) - n, "]}");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, n);
}

// Escape src into dst for use inside an HTML attribute value
// (covered chars: " & < >). Returns bytes written excluding NUL.
// Truncates if dst is too small. Always NUL-terminates.
static size_t html_attr_escape(char *dst, size_t cap, const char *src)
{
    size_t w = 0;
    if (cap == 0) return 0;
    for (const char *p = src; *p; p++) {
        const char *rep     = NULL;
        size_t      rep_len = 0;
        switch (*p) {
        case '"':
            rep     = "&quot;";
            rep_len = 6;
            break;
        case '&':
            rep     = "&amp;";
            rep_len = 5;
            break;
        case '<':
            rep     = "&lt;";
            rep_len = 4;
            break;
        case '>':
            rep     = "&gt;";
            rep_len = 4;
            break;
        default:
            rep     = p;
            rep_len = 1;
            break;
        }
        if (w + rep_len + 1 > cap) break;
        memcpy(dst + w, rep, rep_len);
        w += rep_len;
    }
    dst[w] = '\0';
    return w;
}

// Shared page chrome — ONE source of nav truth for every HTML page (DRY).
// send_page_head() emits doctype + head + minimal inline CSS + the nav bar;
// the caller then streams its body and finishes with send_page_foot().
// refresh_s > 0 adds a meta-refresh (used by the live /status dashboard).
// All responses are chunked, so callers must set no content-length.
static void send_page_head(httpd_req_t *req, const char *title, int refresh_s)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    char refresh[64] = {0};
    if (refresh_s > 0) {
        snprintf(refresh, sizeof(refresh),
                 "<meta http-equiv=\"refresh\" content=\"%d\">", refresh_s);
    }
    static EXT_RAM_BSS_ATTR char head[1536]; // httpd task stack is only 6144 B (PSRAM) — big
                            // buffers MUST be static (single serve task = race-free)
                            // or the HTML handlers overflow the stack and reset the conn
    int  n = snprintf(head, sizeof(head),
                      "<!doctype html><html><head><meta charset=\"utf-8\">"
                       "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
                       "%s<title>%s</title><style>"
                       "body{font-family:system-ui,sans-serif;max-width:640px;margin:0 auto 2em;padding:0 1em;color:#222;background:#fafafa}"
                       "h1{font-size:1.3em}h2{font-size:1.05em;margin-top:1.8em;color:#555}"
                       "nav{margin:0 -1em 1.2em;padding:.6em 1em;background:#1976d2}"
                       "nav a{color:#fff;text-decoration:none;margin-right:1em;font-size:.95em}"
                       "nav a:hover{text-decoration:underline}"
                       "label{display:block;margin:1em 0 .3em;font-size:.9em;color:#555}"
                       "input[type=text],input[type=password],input[type=number],select{width:100%%;padding:.5em;border:1px solid #ccc;border-radius:4px;font-size:1em;box-sizing:border-box}"
                       "button{margin-top:1.5em;padding:.7em 1.5em;border:0;background:#1976d2;color:#fff;border-radius:4px;font-size:1em}"
                       "small{color:#888}table{border-collapse:collapse;width:100%%}"
                       "td,th{text-align:left;padding:.25em .5em;border-bottom:1px solid #eee;font-size:.9em}"
                       "td.v{font-family:ui-monospace,monospace;text-align:right}"
                       "</style></head><body>"
                       "<nav><a href=\"/\">Config</a><a href=\"/status\">Status</a>"
                       "<a href=\"/messages\">Messages</a><a href=\"/tasks\">Tasks</a>"
                       "<a href=\"/capture/status\">Capture</a></nav>"
                       "<h1>%s</h1>",
                      refresh, title, title);
    if (n < 0) n = 0;
    if (n > (int)sizeof(head)) n = sizeof(head);
    httpd_resp_send_chunk(req, head, n);
}

static void send_page_foot(httpd_req_t *req)
{
    static const char foot[] =
        "<p><small>JSON APIs: <a href=\"/status\">/status</a> (curl) · "
        "<a href=\"/messages\">/messages</a> · <a href=\"/ota\">/ota</a></small></p>"
        "</body></html>";
    httpd_resp_send_chunk(req, foot, sizeof(foot) - 1);
    httpd_resp_send_chunk(req, NULL, 0);
}

// Shown only in STA mode (we're already at the form when in AP).
static const char s_index_reset_block[] =
    "<hr style=\"margin-top:2em\">"
    "<form method=\"POST\" action=\"/reset\" onsubmit=\"return confirm('Clear Wi-Fi credentials and reboot to AP mode?');\">"
    "<button type=\"submit\" style=\"background:#a00\">Reset Wi-Fi → AP mode</button>"
    "</form>";

static esp_err_t index_get(httpd_req_t *req)
{
    send_page_head(req, "Iridium ACARS", 0);
    httpd_resp_send_chunk(req,
                          "<p>Configure the device. Wi-Fi changes reboot on save; "
                          "UDP push fields take effect immediately. SDR tuning changes "
                          "reboot to re-program the tuner.</p>",
                          HTTPD_RESP_USE_STRLEN);

    // Build the form with the current config pre-filled so the user
    // can see what's saved (SSID was missing from the rendered form
    // before; clicking Save with the empty SSID field bounced the
    // form because of `required` validation).
    app_config_t cfg;
    app_config_snapshot(&cfg);

    char ssid_esc[2 * sizeof(cfg.wifi_ssid) + 8];
    char psk_esc[2 * sizeof(cfg.wifi_psk) + 8];
    char host_esc[2 * sizeof(cfg.out_host) + 8];
    char ota_esc[2 * sizeof(cfg.ota_url) + 8];
    html_attr_escape(ssid_esc, sizeof(ssid_esc), cfg.wifi_ssid);
    html_attr_escape(psk_esc, sizeof(psk_esc), cfg.wifi_psk);
    html_attr_escape(host_esc, sizeof(host_esc), cfg.out_host);
    html_attr_escape(ota_esc, sizeof(ota_esc), cfg.ota_url);

    static EXT_RAM_BSS_ATTR char form[2048]; // static: 6144 B httpd stack (see head[] note)
    int  n = snprintf(form, sizeof(form),
                      "<form method=\"POST\" action=\"/config\">"
                       "<h2>Wi-Fi</h2>"
                       "<label>SSID</label>"
                       "<input type=\"text\" name=\"ssid\" required maxlength=\"32\" value=\"%s\">"
                       "<label>Password</label>"
                       "<input type=\"password\" name=\"psk\" maxlength=\"63\" value=\"%s\">"
                       "<h2>SDR (bias tee)</h2>"
                       "<label><input type=\"checkbox\" name=\"bias_tee\" value=\"1\"%s> "
                       "Enable RTL-SDR v4 bias tee (5 V on antenna line, for active antennas / LNAs)</label>"
                       "<h2>ACARS push (optional, UDP)</h2>"
                       "<label>Host (IP or hostname; leave empty to disable)</label>"
                       "<input type=\"text\" name=\"out_host\" maxlength=\"63\" value=\"%s\">"
                       "<label>Port</label>"
                       "<input type=\"number\" name=\"out_port\" min=\"0\" max=\"65535\" placeholder=\"e.g. 6700\" value=\"%u\">"
                       "<h2>OTA</h2>"
                       "<label>Firmware URL (http:// or https://)</label>"
                       "<input type=\"text\" name=\"ota_url\" maxlength=\"127\" placeholder=\"http://server/p4-usb-host.bin\" value=\"%s\">"
                       "<button type=\"submit\">Save &amp; reboot</button>"
                       "</form>",
                      ssid_esc, psk_esc,
                     cfg.bias_tee ? " checked" : "",
                      host_esc,
                      (unsigned)cfg.out_port,
                      ota_esc);
    if (n < 0) n = 0;
    if (n > (int)sizeof(form)) n = sizeof(form);
    httpd_resp_send_chunk(req, form, n);

    // Separate SDR-tuning form, posting to /sdrcfg (a body-parsed endpoint
    // that NEVER touches Wi-Fi). All fields are pre-filled from the live
    // config so a submit rewrites SDR config safely — untouched fields
    // resubmit their current values (same Wi-Fi-safety principle as the
    // form above). /tune and /autotune are query-param endpoints (curl/
    // scripts); a no-JS form can't drive them, hence one consolidated POST.
    char sdrform[1400];
    int  sn = snprintf(sdrform, sizeof(sdrform),
                       "<form method=\"POST\" action=\"/sdrcfg\">"
                        "<h2>SDR tuning</h2>"
                        "<label>LO frequency (Hz, Iridium 1615000000-1628000000)</label>"
                        "<input type=\"number\" name=\"lo_hz\" min=\"1615000000\" max=\"1628000000\" required value=\"%u\">"
                        "<label>Gain mode</label>"
                        "<select name=\"gain_mode\">"
                        "<option value=\"0\"%s>Tuner AGC</option>"
                        "<option value=\"1\"%s>Manual</option>"
                        "<option value=\"2\"%s>Software AGC</option>"
                        "</select>"
                        "<label>Manual gain (dB; used only in Manual mode)</label>"
                        "<input type=\"number\" name=\"gain_db\" step=\"0.1\" required value=\"%.1f\">"
                        "<label>Tagger threshold (dB above noise floor)</label>"
                        "<input type=\"number\" name=\"tag_thr\" step=\"0.1\" required value=\"%.1f\">"
                        "<label>Coalesce min bursts (0/1 = disabled)</label>"
                        "<input type=\"number\" name=\"coal_n\" min=\"0\" max=\"255\" required value=\"%u\">"
                        "<label>Autotune LO re-scan interval (s; 0 = off)</label>"
                        "<input type=\"number\" name=\"at_lo_s\" min=\"0\" required value=\"%u\">"
                        "<label>Autotune gain re-cal interval (s; 0 = off)</label>"
                        "<input type=\"number\" name=\"at_gain_s\" min=\"0\" required value=\"%u\">"
                        "<button type=\"submit\">Apply &amp; reboot</button>"
                        "</form>",
                       (unsigned)cfg.lo_freq_hz,
                      cfg.gain_mode == GAIN_MODE_TUNER_AGC ? " selected" : "",
                      cfg.gain_mode == GAIN_MODE_MANUAL ? " selected" : "",
                      cfg.gain_mode == GAIN_MODE_SOFTWARE_AGC ? " selected" : "",
                       (double)cfg.gain_db_x10 / 10.0,
                       (double)cfg.tagger_threshold_db,
                       (unsigned)cfg.coalesce_min_bursts,
                       (unsigned)cfg.autotune_lo_interval_s,
                       (unsigned)cfg.autotune_gain_interval_s);
    if (sn < 0) sn = 0;
    if (sn > (int)sizeof(sdrform)) sn = sizeof(sdrform);
    httpd_resp_send_chunk(req, sdrform, sn);

    // Graceful reboot control — always available (both AP and STA). Parks the
    // tuner first (see reboot_post), so it's the safe way to restart without a
    // physical power-cycle. Lives here rather than on /status because that
    // page's 5 s meta-refresh would cancel the confirm() dialog mid-click.
    static const char reboot_block[] =
        "<hr style=\"margin-top:2em\">"
        "<form method=\"POST\" action=\"/reboot\" "
        "onsubmit=\"return confirm('Reboot the device now? The stream drops for ~10 s.');\">"
        "<button type=\"submit\" style=\"background:#e67e22\">Reboot device</button>"
        "</form>";
    httpd_resp_send_chunk(req, reboot_block, sizeof(reboot_block) - 1);

    if (!wifi_link_is_ap_mode()) {
        httpd_resp_send_chunk(req, s_index_reset_block,
                              sizeof(s_index_reset_block) - 1);
    }
    send_page_foot(req);
    return ESP_OK;
}

// GET /status (browser variant). Surfaces the live STATUS-line fields as an
// HTML table with a 5 s meta-refresh. Reads the last logged snapshot via
// status_logger_get_last() (non-resetting — it does NOT call the resetting
// worker_core1_get_stats(), which would steal counts from the logger's own
// 1 s window) plus cumulative getters (decode counts, USB totals, DMA heap,
// stash fails). curl / monitors get the JSON variant (see status_get).
static esp_err_t status_html_get(httpd_req_t *req)
{
    send_page_head(req, "Status", 5); // 5 s meta-refresh — live dashboard

    app_config_t cfg;
    app_config_snapshot(&cfg);
    const esp_app_desc_t *app = esp_app_get_description();

    status_snapshot_t s;
    bool              have = status_logger_get_last(&s);

    // Cumulative, non-resetting counters — safe to read from the http task.
    uint32_t bch_dec = 0, bch_unk = 0;
    worker_core1_get_decode_counts(&bch_dec, &bch_unk);
    usb_stream_totals_t usbt = {0};
    esp_libusb_get_stream_totals(&usbt);
    uint32_t dma_free    = heap_caps_get_free_size(MALLOC_CAP_DMA);
    uint32_t dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    uint32_t stash_fails = signal_buffer_stash_alloc_fails();

    double lo_mhz   = (double)cfg.lo_freq_hz / 1e6;
    double half_mhz = ((double)FS_DETECT_HZ / 2.0) / 1e6;

    char gain_str[28];
    if (cfg.gain_mode == GAIN_MODE_MANUAL)
        snprintf(gain_str, sizeof(gain_str), "%.1f dB (manual)", (double)cfg.gain_db_x10 / 10.0);
    else if (cfg.gain_mode == GAIN_MODE_SOFTWARE_AGC)
        snprintf(gain_str, sizeof(gain_str), "software AGC");
    else
        snprintf(gain_str, sizeof(gain_str), "tuner AGC");

    int64_t uptime_s = esp_timer_get_time() / 1000000;
    static EXT_RAM_BSS_ATTR char body[2700]; // static: 6144 B httpd stack (see send_page_head note)
    int     n = snprintf(body, sizeof(body),
                         "<p><small>build %s &middot; %s &middot; uptime %llds &middot; "
                             "auto-refresh 5 s</small></p>",
                         app->version, app->date, (long long)uptime_s);
    httpd_resp_send_chunk(req, body, n > 0 ? n : 0);

    if (!have) {
        httpd_resp_send_chunk(req,
                              "<p><em>No STATUS snapshot yet — the USB stream may still be "
                              "starting. Reload in a moment.</em></p>",
                              HTTPD_RESP_USE_STRLEN);
        send_page_foot(req);
        return ESP_OK;
    }

    // Derived per-window figures — same formulas as status_logger emit().
    double rate = 0, dsp_cap = 0, worker_cap = 0;
    if (s.window_us > 0) {
        double w   = (double)s.window_us;
        rate       = (s.bytes_window / (1024.0 * 1024.0)) / (w / 1e6);
        dsp_cap    = 100.0 * (double)s.dsp_total_time_us / w;
        worker_cap = 100.0 * ((double)s.ws.bursts_processed * (double)s.ws.avg_burst_us + (double)s.ws.bursts_triage_rejected * (double)s.ws.triage_rej_us) /
                     w;
    }

    n = snprintf(body, sizeof(body),
                 "<table><tr><th>Metric</th><th>Value</th></tr>"
                 "<tr><td>USB rate</td><td class=v>%.2f MB/s</td></tr>"
                 "<tr><td>FFT steps / window</td><td class=v>%u</td></tr>"
                 "<tr><td>Bursts dispatched</td><td class=v>%u</td></tr>"
                 "<tr><td>Processed</td><td class=v>%u</td></tr>"
                 "<tr><td>Triage rejected</td><td class=v>%u</td></tr>"
                 "<tr><td>BCH decoded / unknown (window)</td><td class=v>%u / %u</td></tr>"
                 "<tr><td>BCH decoded / unknown (since boot)</td><td class=v>%u / %u</td></tr>"
                 "<tr><td>rb_full drops (window)</td><td class=v>%u</td></tr>"
                 "<tr><td>rb_full drops (since boot)</td><td class=v>%llu</td></tr>"
                 "<tr><td>DSP capacity</td><td class=v>%.0f %%</td></tr>"
                 "<tr><td>Worker capacity</td><td class=v>%.0f %%</td></tr>"
                 "<tr><td>LO frequency</td><td class=v>%.4f MHz</td></tr>"
                 "<tr><td>Listening band</td><td class=v>%.3f - %.3f MHz</td></tr>"
                 "<tr><td>Gain</td><td class=v>%s</td></tr>"
                 "<tr><td>DMA-INT free / largest</td><td class=v>%u / %u KB</td></tr>"
                 "<tr><td>Stash alloc fails</td><td class=v>%u</td></tr>"
                 "</table>",
                 rate, (unsigned)s.dsp_frame_count, (unsigned)s.dsp.gone_bursts,
                 (unsigned)s.ws.bursts_processed, (unsigned)s.ws.bursts_triage_rejected,
                 (unsigned)s.ws.bursts_bch_decoded, (unsigned)s.ws.bursts_bch_unknown,
                 (unsigned)bch_dec, (unsigned)bch_unk,
                 (unsigned)s.us.rb_full_drops, (unsigned long long)usbt.rb_full_drops,
                 dsp_cap, worker_cap,
                 lo_mhz, lo_mhz - half_mhz, lo_mhz + half_mhz,
                 gain_str, (unsigned)(dma_free / 1024), (unsigned)(dma_largest / 1024),
                 (unsigned)stash_fails);
    if (n < 0) n = 0;
    if (n > (int)sizeof(body)) n = sizeof(body);
    httpd_resp_send_chunk(req, body, n);

    send_page_foot(req);
    return ESP_OK;
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
            char buf[3] = {src[i + 1], src[i + 2], 0};
            dst[w++]    = (char)strtol(buf, NULL, 16);
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
    size_t      klen = strlen(key);
    const char *p    = body;
    const char *end  = body + blen;
    while (p < end) {
        const char *amp     = memchr(p, '&', end - p);
        const char *seg_end = amp ? amp : end;
        const char *eq      = memchr(p, '=', seg_end - p);
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
    // Form body buffer. static (not stack, not heap): esp_http_server
    // runs every handler on its single serve task, so one static buffer
    // is race-free and avoids a per-request alloc. 1024 covers the full
    // form worst case (ssid 32 + psk 63 + out_host 63 + ota_url 127,
    // each up to 3× expanded by %XX url-encoding, plus keys) — the old
    // 256-byte buffer silently truncated long PSK+URL combinations.
    static EXT_RAM_BSS_ATTR char body[1024];
    if (req->content_len >= sizeof(body)) {
        httpd_resp_set_status(req, "413 Payload Too Large");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "form body too large\n", HTTPD_RESP_USE_STRLEN);
    }
    int total    = 0;
    int timeouts = 0;
    while (total < (int)req->content_len) {
        int n = httpd_req_recv(req, body + total, sizeof(body) - 1 - total);
        if (n <= 0) {
            if (n == HTTPD_SOCK_ERR_TIMEOUT) {
                // Bounded retry: the old unconditional `continue` could
                // spin this single serve task forever against a client
                // that stalls mid-body (recv timeout is 2 s, so 3 retries
                // ≈ 8 s worst case before we give up).
                if (++timeouts <= 3) continue;
                httpd_resp_set_status(req, "408 Request Timeout");
                httpd_resp_set_type(req, "text/plain");
                httpd_resp_send(req, "body recv timeout\n", HTTPD_RESP_USE_STRLEN);
                return ESP_FAIL;
            }
            break; // client closed / socket error — parse what we got
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
    form_field(body, total, "psk", psk, sizeof(psk)); // psk optional (open AP)

    // Optional UDP push target. Both fields empty / 0 = disabled.
    char out_host[64]  = {0};
    char out_port_s[8] = {0};
    form_field(body, total, "out_host", out_host, sizeof(out_host));
    form_field(body, total, "out_port", out_port_s, sizeof(out_port_s));
    uint16_t out_port = (uint16_t)strtoul(out_port_s, NULL, 10);

    // Optional OTA URL.
    char ota_url[128] = {0};
    form_field(body, total, "ota_url", ota_url, sizeof(ota_url));

    // Bias-tee checkbox: present in form body only if checked (HTML form
    // convention). form_field returns ESP_OK iff the key is present.
    char bias_tee_s[4] = {0};
    bool bias_tee      = (form_field(body, total, "bias_tee", bias_tee_s,
                                     sizeof(bias_tee_s)) == ESP_OK);

    ESP_LOGI(TAG, "/config POST: ssid='%s' (psk %s), bias_tee=%d, out=%s:%u, ota_url=%s",
             ssid, psk[0] ? "set" : "empty", (int)bias_tee,
             out_host[0] ? out_host : "(none)", (unsigned)out_port,
             ota_url[0] ? ota_url : "(none)");

    // Marshal form fields into a heap-allocated struct and hand off
    // to an internal-SRAM-stacked worker that does the NVS writes
    // and the reboot. See nvs_save_and_reboot_task comment.
    nvs_save_args_t *args = calloc(1, sizeof(*args));
    if (!args) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "alloc failed\n", HTTPD_RESP_USE_STRLEN);
    }
    strlcpy(args->ssid, ssid, sizeof(args->ssid));
    strlcpy(args->psk, psk, sizeof(args->psk));
    strlcpy(args->out_host, out_host, sizeof(args->out_host));
    args->out_port = out_port;
    strlcpy(args->ota_url, ota_url, sizeof(args->ota_url));
    args->bias_tee   = bias_tee;
    args->clear_only = false;

    // Send the OK page BEFORE spawning the writer — once it runs,
    // NVS commit disables flash cache, which could disrupt the
    // socket send (esp_netif/lwip work on cached PSRAM too).
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    const char *ok =
        "<!doctype html><html><body style=\"font-family:system-ui;max-width:480px;margin:2em auto;padding:0 1em\">"
        "<h1>Saved — rebooting</h1>"
        "<p>The device will connect to the Wi-Fi network in a few seconds. "
        "Check your router for the new client, or browse to its address.</p>"
        "</body></html>";
    httpd_resp_send(req, ok, HTTPD_RESP_USE_STRLEN);

    BaseType_t spawned = xTaskCreatePinnedToCore(nvs_save_and_reboot_task,
                                                 "nvs_save", 4096, args, 5, NULL, tskNO_AFFINITY);
    if (spawned != pdPASS) {
        ESP_LOGE(TAG, "nvs_save task spawn failed — NVS not written, no reboot");
        free(args);
    }
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

// GET /messages (browser variant). Renders the decoded-ACARS ring as an HTML
// table styled like the /status dashboard (shared chrome + CSS from
// send_page_head), newest-first, with a 10 s meta-refresh. curl / monitors
// get the JSON variant below (content-negotiated in messages_get). Streams one
// chunk per row so no single buffer has to hold the whole ring.
static esp_err_t messages_html_get(httpd_req_t *req)
{
    send_page_head(req, "Messages", 10); // 10 s meta-refresh — live feed

    static EXT_RAM_BSS_ATTR acars_msg_t s_snap[MSG_RING_CAPACITY];
    size_t n = msg_ring_snapshot(0, s_snap, MSG_RING_CAPACITY);
    uint64_t total = msg_ring_total();
    int64_t  now_us = esp_timer_get_time();

    // Cumulative decode-type totals (since boot) — same getters the /status
    // JSON uses. These count every classified frame, not just what fits in the
    // 32-entry ring, so they keep climbing after the ring wraps.
    frame_decoder_class_counts_t cc = {0};
    frame_decoder_get_class_counts(&cc);
    uint64_t acars_total = frame_decoder_acars_decoded_total();
    uint64_t sbd_total   = frame_decoder_sbd_complete_total();

    static EXT_RAM_BSS_ATTR char body[768];
    int  bn = snprintf(body, sizeof(body),
                       "<p><small>%llu messages in ring &middot; showing last %u "
                       "&middot; auto-refresh 10 s</small></p>"
                       "<h2>Decoded totals (since boot)</h2>"
                       "<table><tr><th>ACARS</th><th>SBD</th><th>MS</th><th>TL</th>"
                       "<th>BC</th><th>LW·DA</th><th>LW·oth</th><th>Unknown</th></tr>"
                       "<tr><td class=v>%llu</td><td class=v>%llu</td><td class=v>%llu</td>"
                       "<td class=v>%llu</td><td class=v>%llu</td><td class=v>%llu</td>"
                       "<td class=v>%llu</td><td class=v>%llu</td></tr></table>",
                       (unsigned long long)total, (unsigned)n,
                       (unsigned long long)acars_total, (unsigned long long)sbd_total,
                       (unsigned long long)cc.ms, (unsigned long long)cc.tl,
                       (unsigned long long)cc.bc, (unsigned long long)cc.lw_da,
                       (unsigned long long)cc.lw_other, (unsigned long long)cc.unknown);
    if (bn < 0) bn = 0;
    if (bn > (int)sizeof(body)) bn = sizeof(body);
    httpd_resp_send_chunk(req, body, bn);

    if (n == 0) {
        httpd_resp_send_chunk(req,
                              "<p><em>No ACARS messages in the ring yet. IRA ring-alert "
                              "frames are common; IDA data frames (the ones with text) "
                              "are rarer — leave it running.</em></p>",
                              HTTPD_RESP_USE_STRLEN);
        send_page_foot(req);
        return ESP_OK;
    }

    httpd_resp_send_chunk(req, "<h2>Recent messages</h2>", HTTPD_RESP_USE_STRLEN);

    httpd_resp_send_chunk(req,
                          "<table><tr><th>Age</th><th>Dir</th><th>Mode</th><th>Label</th>"
                          "<th>Flight</th><th>Msg#</th><th>CRC</th><th>SNR</th><th>Text</th></tr>",
                          HTTPD_RESP_USE_STRLEN);

    static EXT_RAM_BSS_ATTR char esc_txt[2 * MSG_RING_TXT_MAX + 8];
    static char esc_flight[16];
    static char esc_label[16];
    static char esc_msgnum[24];
    static EXT_RAM_BSS_ATTR char row[2 * MSG_RING_TXT_MAX + 512];
    // Newest first: the snapshot is oldest→newest, so walk it in reverse.
    for (size_t k = n; k > 0; k--) {
        const acars_msg_t *m = &s_snap[k - 1];

        double age_s = (double)(now_us - (int64_t)m->timestamp_us) / 1e6;
        char   age[16];
        if (age_s < 0) age_s = 0;
        if (age_s < 120.0)      snprintf(age, sizeof(age), "%.0fs", age_s);
        else if (age_s < 7200.0) snprintf(age, sizeof(age), "%.0fm", age_s / 60.0);
        else                     snprintf(age, sizeof(age), "%.1fh", age_s / 3600.0);

        char label_buf[3] = {m->label[0], m->label[1], 0};
        html_attr_escape(esc_label, sizeof(esc_label), label_buf);
        html_attr_escape(esc_msgnum, sizeof(esc_msgnum), m->msg_num);
        html_attr_escape(esc_flight, sizeof(esc_flight), m->flight_id);
        html_attr_escape(esc_txt, sizeof(esc_txt), m->txt);

        int rn = snprintf(row, sizeof(row),
                          "<tr><td class=v>%s</td><td>%s</td><td class=v>%c</td>"
                          "<td class=v>%s</td><td class=v>%s</td><td class=v>%s</td>"
                          "<td><span style=\"color:%s\">%s</span></td>"
                          "<td class=v>%.1f</td><td>%s</td></tr>",
                          age,
                          m->uplink ? "UL" : "DL",
                          (m->mode >= 0x20 && m->mode < 0x7f) ? m->mode : '?',
                          esc_label,
                          esc_flight,
                          esc_msgnum,
                          m->crc_ok ? "#2e7d32" : "#c62828",
                          m->crc_ok ? "OK" : "bad",
                          (double)m->snr_db,
                          esc_txt);
        if (rn < 0) rn = 0;
        if (rn > (int)sizeof(row)) rn = sizeof(row);
        httpd_resp_send_chunk(req, row, rn);
    }

    httpd_resp_send_chunk(req, "</table>", 8);
    send_page_foot(req);
    return ESP_OK;
}

static esp_err_t messages_get(httpd_req_t *req)
{
    // Content negotiation: browsers get the HTML dashboard; curl / monitors /
    // Accept:*/* fall through to the JSON below, UNCHANGED. Same rationale as
    // status_get — see its comment on not gating on the return code.
    char accept[160] = {0};
    httpd_req_get_hdr_value_str(req, "Accept", accept, sizeof(accept));
    if (strstr(accept, "text/html")) {
        return messages_html_get(req);
    }

    uint64_t since_id = 0;
    char     qbuf[64];
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        char val[24];
        if (httpd_query_key_value(qbuf, "since", val, sizeof(val)) == ESP_OK) {
            since_id = strtoull(val, NULL, 10);
        }
    }

    // ~9 KB snapshot — too big for the 6 KB httpd task stack, so static.
    // EXT_RAM_BSS_ATTR moves it to PSRAM .bss: internal SRAM is reserved
    // for the USB DMA pool, and the ring copy is latency-tolerant (the
    // file's other big buffers already live in PSRAM via heap_caps).
    static EXT_RAM_BSS_ATTR acars_msg_t s_snap[MSG_RING_CAPACITY];
    size_t                              n = msg_ring_snapshot(since_id, s_snap, MSG_RING_CAPACITY);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    // Stream: {"total":N,"messages":[ {...}, {...} ]}
    char chunk[600];
    int  len = snprintf(chunk, sizeof(chunk),
                        "{\"total\":%llu,\"messages\":[",
                        (unsigned long long)msg_ring_total());
    if (len >= (int)sizeof(chunk)) len = sizeof(chunk) - 1;
    httpd_resp_send_chunk(req, chunk, len);

    char esc_txt[2 * MSG_RING_TXT_MAX + 8];
    char esc_flight[16];
    char esc_msgnum[16];
    char esc_label[8];
    char esc_mode[8];
    char esc_block[8];
    for (size_t i = 0; i < n; i++) {
        const acars_msg_t *m = &s_snap[i];

        char label_buf[3] = {m->label[0], m->label[1], 0};
        char mode_buf[2]  = {m->mode, 0};
        char block_buf[2] = {m->block_id, 0};
        json_escape(esc_label, sizeof(esc_label), label_buf);
        json_escape(esc_msgnum, sizeof(esc_msgnum), m->msg_num);
        json_escape(esc_flight, sizeof(esc_flight), m->flight_id);
        json_escape(esc_mode, sizeof(esc_mode), mode_buf);
        json_escape(esc_block, sizeof(esc_block), block_buf);
        json_escape(esc_txt, sizeof(esc_txt), m->txt);

        len = snprintf(chunk, sizeof(chunk),
                       "%s{"
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
                       "}",
                       (i == 0) ? "" : ",",
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
        if (len >= (int)sizeof(chunk)) len = sizeof(chunk) - 1;
        if (len > 0) {
            httpd_resp_send_chunk(req, chunk, len);
        }
    }
    httpd_resp_send_chunk(req, "]}", 2);
    return httpd_resp_send_chunk(req, NULL, 0); // end of chunked response
}

static esp_err_t ota_post(httpd_req_t *req)
{
    esp_err_t r = ota_runner_start();
    if (r == ESP_ERR_INVALID_STATE) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "OTA already running\n", HTTPD_RESP_USE_STRLEN);
    }
    if (r != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "OTA failed to start\n", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    const char *html =
        "<!doctype html><html><body style=\"font-family:system-ui;max-width:480px;margin:2em auto;padding:0 1em\">"
        "<h1>OTA started</h1>"
        "<p>The device is downloading the new firmware from the configured URL. "
        "On success it will reboot automatically — typically 30-90 seconds. "
        "Poll <a href=\"/ota\">/ota</a> for progress.</p>"
        "</body></html>";
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ota_get(httpd_req_t *req)
{
    ota_status_t s;
    ota_runner_get_status(&s);
    const char *state = "idle";
    switch (s.state) {
    case OTA_RUNNING:
        state = "running";
        break;
    case OTA_SUCCESS:
        state = "success";
        break;
    case OTA_FAILED:
        state = "failed";
        break;
    default:
        break;
    }
    // last_error carries free-form text (URLs, esp_err strings) — escape
    // it so a quote/backslash can't break the JSON (M17).
    char err_esc[2 * sizeof(s.last_error) + 1];
    json_escape(err_esc, sizeof(err_esc), s.last_error);
    char body[560];
    int  n = snprintf(body, sizeof(body),
                      "{\"state\":\"%s\",\"http_status\":%d,\"bytes_written\":%d,\"last_error\":\"%s\"}",
                      state, s.http_status, s.bytes_written, err_esc);
    if (n < 0) n = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}

// GET /diag/dsp_health (#118). Two-second DSP-pipeline heartbeat. Takes
// counter snapshots 2 s apart and returns deltas + a pass/fail verdict.
// Converts "did the OTA / refactor / config-change break the DSP?" from
// an offline-test-only question into a one-curl yes/no.
//
// What this proves on PASS:
//   - USB stream is producing samples (usb.completed climbing)
//   - dsp_processor is consuming samples (tagger FFT frames climbing)
//   - frame_decoder is running (its class counts are read but not gated)
// What it does NOT prove: any real Iridium frame decoded. That requires
// real signal arriving, which is the antenna's job. The heartbeat just
// asserts the chain is alive.
//
// Concurrency: the inner sleep is on the httpd task; class_driver +
// worker continue running normally throughout. Idempotent; safe to call
// from a cron / monitor.
static esp_err_t diag_dsp_health_get(httpd_req_t *req)
{
    // Snapshot 1: use the race-free cumulative counter (#127). The
    // previous implementation called dsp_processor_get_stage_stats which
    // RESETS its per-window counter — status_logger calls the same API
    // once per second, so a 2 s window heartbeat lost 50-100% of its
    // frames whenever status_logger fired between snapshots and reported
    // a phantom dsp_ok=false. dsp_processor_get_total_fft_frames is
    // monotonic; subtracting two snapshots gives an exact count.
    uint64_t            dsp_frames0 = dsp_processor_get_total_fft_frames(dsp_processor_default());
    usb_stream_totals_t usb0        = {0};
    esp_libusb_get_stream_totals(&usb0);
    uint64_t acars0 = frame_decoder_acars_decoded_total();
    int64_t  t0     = esp_timer_get_time();

    // 2-second window. Real production has plenty of FFT frames in 2 s
    // (~1900 at 2.56 MSPS with FBT_FFT_SIZE=2048).
    vTaskDelay(pdMS_TO_TICKS(2000));

    uint64_t            dsp_frames1 = dsp_processor_get_total_fft_frames(dsp_processor_default());
    usb_stream_totals_t usb1        = {0};
    esp_libusb_get_stream_totals(&usb1);
    uint64_t acars1 = frame_decoder_acars_decoded_total();
    int64_t  t1     = esp_timer_get_time();

    uint64_t usb_delta   = usb1.completed - usb0.completed;
    uint64_t dsp_frames  = dsp_frames1 - dsp_frames0;
    uint64_t acars_delta = acars1 - acars0;
    int64_t  window_us   = t1 - t0;

    // PASS criteria: both USB and DSP must show forward progress in the
    // 2 s window. Thresholds are loose — at 2.56 MSPS / 16 KB transfers
    // we expect ~600 completed/s × 2 s = ~1200; at FBT_FFT_SIZE 2048 we
    // expect ~1900 FFT frames/s × 2 s = ~3800.
    bool usb_ok = (usb_delta > 200);  // ~10% of nominal — generous floor
    bool dsp_ok = (dsp_frames > 200); // ditto
    bool pass   = usb_ok && dsp_ok;

    char body[512];
    int  n = snprintf(body, sizeof(body),
                      "{\"window_us\":%lld,"
                       "\"usb_completed_delta\":%llu,\"usb_ok\":%s,"
                       "\"dsp_fft_frames\":%u,\"dsp_ok\":%s,"
                       "\"acars_decoded_delta\":%llu,"
                       "\"pass\":%s}\n",
                      (long long)window_us,
                      (unsigned long long)usb_delta, usb_ok ? "true" : "false",
                      (unsigned)dsp_frames, dsp_ok ? "true" : "false",
                      (unsigned long long)acars_delta,
                     pass ? "true" : "false");
    if (n < 0 || n >= (int)sizeof(body)) n = sizeof(body) - 1;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (!pass) httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_send(req, body, n);
}

// GET /diag/recovery_counters — surface every defensive failure-mode
// counter the firmware tracks. These are mostly "have-never-fired"
// guards for #106-class deadlocks; exposing them turns dormant
// instrumentation into useful diag the watch loop can poll without
// log scraping. Aligned with the #122 fault-injection plan.
//
// JSON nested by source component; field names match the per-counter
// docstrings in signal_buffer.h / ingest_core1.h / esp_libusb.h. Same
// names also appear in the STATUS-ERR log line so log-vs-endpoint
// reading is consistent.
static esp_err_t diag_recovery_counters_get(httpd_req_t *req)
{
    uint32_t sb_fails      = signal_buffer_stash_alloc_fails();
    uint32_t sb_recoveries = signal_buffer_stash_alloc_recoveries();
    uint32_t sb_audio_drop = (sb_fails > sb_recoveries) ? (sb_fails - sb_recoveries) : 0;
    char     body[640];
    int      n = snprintf(body, sizeof(body),
                          "{"
                               "\"signal_buffer\":{"
                               "\"stash_alloc_fails\":%u,"
                               "\"stash_alloc_recoveries\":%u,"
                               "\"audio_dropped\":%u,"
                               "\"dma_timeouts\":%u"
                               "},"
                               "\"ingest_core1\":{"
                               "\"dispatch_drops\":%u,"
                               "\"slow_waits\":%u,"
                               "\"raw_slow_waits\":%u"
                               "},"
                               "\"esp_libusb\":{"
                               "\"xfer_pool_lost\":%u"
                               "}"
                               "}\n",
                          (unsigned)sb_fails, (unsigned)sb_recoveries, (unsigned)sb_audio_drop,
                          (unsigned)signal_buffer_dma_timeouts(),
                          (unsigned)ingest_core1_dispatch_drops(),
                          (unsigned)ingest_core1_take_converted_slow_waits(),
                          (unsigned)ingest_core1_raw_done_slow_waits(),
                          (unsigned)esp_libusb_xfer_pool_lost());
    if (n < 0 || n >= (int)sizeof(body)) n = sizeof(body) - 1;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}

// POST /debug/inject — synthesise one ACARS message and push it through
// the exact same fan-out the real decode path uses (frame_decoder.c):
// RAM ring, UDP push, and the SD NDJSON log. This proves the decode ->
// persistence path end-to-end on the live device, which otherwise never
// runs at bench SNR (no real decodes). Internal-LAN debug aid (task #102).
static esp_err_t debug_inject_post(httpd_req_t *req)
{
    acars_msg_t m  = {0};
    m.timestamp_us = (uint64_t)esp_timer_get_time();
    m.uplink       = false;
    m.mode         = 'A';
    m.label[0]     = 'Q';
    m.label[1]     = '0';
    m.label[2]     = '\0';
    m.block_id     = '1';
    strlcpy(m.msg_num, "T001", sizeof(m.msg_num));
    strlcpy(m.flight_id, "TEST01", sizeof(m.flight_id));
    m.crc_ok   = true;
    m.peak_bin = -1;
    m.snr_db   = 0.0f;
    strlcpy(m.txt, "debug/inject write-path proof", sizeof(m.txt));

    msg_ring_push(&m);
    acars_push_emit(&m);
    sd_log_emit(&m);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req,
                           "{\"result\":\"injected\",\"sinks\":[\"msg_ring\",\"udp\",\"sd_log\"]}",
                           HTTPD_RESP_USE_STRLEN);
}

#if CONFIG_FAULT_INJECT
// POST /debug/fault_inject?site=<name>&count=<N> — schedule N synthetic
// failures at a recovery site (#122). Verify via /diag/recovery_counters
// that the matching counter climbs while the stream keeps running. Only
// compiled when CONFIG_FAULT_INJECT=y (test build).
static esp_err_t debug_fault_inject_post(httpd_req_t *req)
{
    char query[96]   = {0};
    char site_s[32]  = {0};
    char count_s[12] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "site", site_s, sizeof(site_s)) != ESP_OK ||
        site_s[0] == '\0') {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req,
                               "usage: POST /debug/fault_inject?site=<name>&count=<N>\n"
                               "sites: dma_submit dma_submit_wrap dispatch_queue "
                               "take_converted urb_submit\n",
                               HTTPD_RESP_USE_STRLEN);
    }
    fi_site_t site = fault_inject_site_from_name(site_s);
    if (site >= FI_SITE_COUNT) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "unknown site\n", HTTPD_RESP_USE_STRLEN);
    }
    uint32_t count = 1;
    if (httpd_query_key_value(query, "count", count_s, sizeof(count_s)) == ESP_OK && count_s[0]) {
        count = (uint32_t)strtoul(count_s, NULL, 10);
    }
    fault_inject_request(site, count);
    char body[128];
    int  n = snprintf(body, sizeof(body),
                      "{\"result\":\"ok\",\"site\":\"%s\",\"count\":%u,\"pending\":%u}\n",
                      fault_inject_site_name(site), (unsigned)count,
                      (unsigned)fault_inject_remaining(site));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}
#endif // CONFIG_FAULT_INJECT

// POST /tune?hz=<lo_freq_hz> — set the SDR centre frequency dynamically and
// reboot to apply (the LO is programmed at stream start, class_driver.c).
// Touches ONLY lo_freq_hz; WiFi and all other config are preserved (unlike
// /config, which rewrites everything). Lets us retune across the Iridium band
// without reflashing — e.g. 1626 MHz (simplex/system frames) vs ~1622 MHz
// (duplex/user channels where SBD/ACARS rides).
static esp_err_t tune_post(httpd_req_t *req)
{
    char query[64] = {0};
    char hz_s[24]  = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "hz", hz_s, sizeof(hz_s)) != ESP_OK ||
        hz_s[0] == '\0') {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req,
                               "usage: POST /tune?hz=<lo_freq_hz>  (e.g. 1622000000)\n",
                               HTTPD_RESP_USE_STRLEN);
    }
    uint32_t hz = (uint32_t)strtoul(hz_s, NULL, 10);
    // Iridium L-band downlink is 1616.0-1626.5 MHz; allow a little slack so
    // the 2.56 MHz window can be centred near either edge.
    if (hz < 1615000000u || hz > 1628000000u) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "text/plain");
        char m[112];
        int  mn = snprintf(m, sizeof(m),
                           "hz=%u out of Iridium band [1615000000, 1628000000]\n", (unsigned)hz);
        return httpd_resp_send(req, m, mn);
    }
    // Reply first, then do the NVS write + reboot on an internal-stack task
    // (the write can't run on this PSRAM-stacked httpd task — see
    // tune_apply_reboot_task). 4096 B internal stack covers nvs_commit +
    // esp_restart.
    char body[96];
    int  n = snprintf(body, sizeof(body),
                      "{\"result\":\"ok\",\"lo_freq_hz\":%u,\"reboot\":true}", (unsigned)hz);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, body, n);
    if (xTaskCreate(tune_apply_reboot_task, "tune_apply", 4096,
                    (void *)(uintptr_t)hz, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "/tune: failed to spawn apply task");
    }
    return ESP_OK;
}

// POST /scan — TEST endpoint: drives ONE full scanner_scan() multi-hop sweep
// (1616-1626 MHz, 2.5 MHz step, ~5 live retunes, 2 s dwell each) on an
// internal-stack task, exercising the live-retune scanner path under real
// streaming. Purpose: verify sustained hopping survives on main (URB-reuse +
// retune-retry + graceful-shutdown all landed) before investing in the
// async-memcpy bypass. Call repeatedly for dozens of hops; watch serial for
// wedge / worker drops. Returns immediately.
static void scan_test_task(void *arg)
{
    (void)arg;
    ESP_LOGW("SCANTEST", "=== /scan: starting scanner_scan sweep ===");
    scanner_scan(SCAN_START_HZ, SCAN_STOP_HZ, SCAN_STEP_HZ, SCAN_DWELL_MS);
    ESP_LOGW("SCANTEST", "=== /scan: sweep complete ===");
    vTaskDelete(NULL);
}

static esp_err_t scan_post(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");
    // prio 4: below worker/ingest so the scan orchestration (mostly waiting on
    // retune completion + dwell) can't starve the DSP hot path.
    if (xTaskCreate(scan_test_task, "scan_test", 4096, NULL, 4, NULL) != pdPASS) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "failed to spawn scan task\n");
    }
    return httpd_resp_sendstr(req, "scan sweep started — watch serial\n");
}

// POST /autotune?lo=<s>&gain=<s> — set the periodic autotune intervals in NVS
// and reboot to apply. Both default to 3600 (hourly) if the param is omitted;
// pass 0 to disable. Exists because autotune config was serial-only (which
// resets the board via DTR) — HTTP avoids that. NVS write can't run on this
// PSRAM-stacked httpd task, so it hands off to an internal-stack task (same
// pattern as tune_apply_reboot_task), which grace­fully parks the tuner first.
typedef struct { uint32_t lo_s; uint32_t gain_s; uint32_t dwell_s; } autotune_cfg_args_t;
static void autotune_cfg_reboot_task(void *arg)
{
    autotune_cfg_args_t *a = (autotune_cfg_args_t *)arg;
    esp_err_t r1 = app_config_set_autotune_lo_interval_s(a->lo_s);
    esp_err_t r2 = app_config_set_autotune_gain_interval_s(a->gain_s);
    esp_err_t r3 = ESP_OK;
    if (a->dwell_s > 0) r3 = app_config_set_autotune_gain_dwell_s(a->dwell_s); // 0 = leave unchanged
    ESP_LOGI(TAG, "/autotune: lo_interval_s=%lu gain_interval_s=%lu gain_dwell_s=%lu (%s/%s/%s) — rebooting",
             (unsigned long)a->lo_s, (unsigned long)a->gain_s, (unsigned long)a->dwell_s,
             esp_err_to_name(r1), esp_err_to_name(r2), esp_err_to_name(r3));
    free(a);
    vTaskDelay(pdMS_TO_TICKS(500));
    class_driver_prepare_for_reboot(); // park tuner so the dongle survives the reboot
    esp_restart();
}
static esp_err_t autotune_post(httpd_req_t *req)
{
    char     query[96] = {0}, s[16] = {0};
    uint32_t lo = 3600, gain = 3600; // default hourly if omitted
    uint32_t dwell = 0;              // 0 = leave the per-gain dwell unchanged
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "lo", s, sizeof(s)) == ESP_OK) lo = (uint32_t)strtoul(s, NULL, 10);
        if (httpd_query_key_value(query, "gain", s, sizeof(s)) == ESP_OK) gain = (uint32_t)strtoul(s, NULL, 10);
        if (httpd_query_key_value(query, "dwell", s, sizeof(s)) == ESP_OK) dwell = (uint32_t)strtoul(s, NULL, 10);
    }
    autotune_cfg_args_t *a = malloc(sizeof(*a));
    if (!a) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "oom\n");
    }
    a->lo_s   = lo;
    a->gain_s = gain;
    a->dwell_s = dwell;
    char body[128];
    int  n = snprintf(body, sizeof(body),
                      "{\"lo_interval_s\":%lu,\"gain_interval_s\":%lu,\"gain_dwell_s\":%lu,\"reboot\":true}",
                      (unsigned long)lo, (unsigned long)gain, (unsigned long)dwell);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, body, n);
    if (xTaskCreate(autotune_cfg_reboot_task, "at_cfg", 4096, a, 5, NULL) != pdPASS) {
        free(a);
        ESP_LOGE(TAG, "/autotune: failed to spawn apply task");
    }
    return ESP_OK;
}

// POST /gaincal[?dwell=<s>] — manually trigger the autotune GAIN-CAL now
// (autotune_run_manual): hop to the IRA reference LO, sweep the R828D gain steps,
// measure bch decodes per gain, pick the best, park back at the ACARS LO + apply.
// Runs on an internal-SRAM-stack task (it does NVS writes — chosen gain, optional
// dwell — which must NOT run on the PSRAM-stacked httpd task). Optional
// ?dwell=<s> overrides the per-gain dwell (default 180 s) so a full sweep can be
// validated in ~90 s (e.g. dwell=10). Unlike the serial `autotune` command, this
// does NOT reset the board (no DTR). Watch serial for "GAINCAL" + per-gain lines.
static void gaincal_task(void *arg)
{
    uint32_t dwell = (uint32_t)(uintptr_t)arg;
    if (dwell > 0) {
        esp_err_t r = app_config_set_autotune_gain_dwell_s(dwell);
        ESP_LOGW("GAINCAL", "manual trigger: dwell override = %lu s (%s)",
                 (unsigned long)dwell, esp_err_to_name(r));
    }
    ESP_LOGW("GAINCAL", "=== manual gain-cal starting (watch for wedge/hang) ===");
    autotune_run_manual();
    ESP_LOGW("GAINCAL", "=== manual gain-cal returned cleanly ===");
    vTaskDelete(NULL);
}
static esp_err_t gaincal_post(httpd_req_t *req)
{
    char     query[48] = {0}, s[12] = {0};
    uint32_t dwell = 0; // 0 = use the configured dwell
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "dwell", s, sizeof(s)) == ESP_OK) {
        dwell = (uint32_t)strtoul(s, NULL, 10);
    }
    httpd_resp_set_type(req, "text/plain");
    // prio 4: below worker/ingest — mostly waits on dwells + validated retunes.
    if (xTaskCreate(gaincal_task, "gaincal", 6144, (void *)(uintptr_t)dwell, 4, NULL) != pdPASS) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "failed to spawn gaincal task\n");
    }
    return httpd_resp_sendstr(req, "gain-cal started — watch serial (GAINCAL / gain steps)\n");
}
// POST /sdrcfg — apply the SDR-tuning knobs from the config form. Body is
// urlencoded (a no-JS form posts its inputs in the body; /tune and /autotune
// are query-param endpoints for curl and can't be driven from a form, so this
// one consolidated endpoint covers LO + gain mode/level + tagger threshold +
// coalesce + both autotune intervals in a single form/reboot). It NEVER calls
// a Wi-Fi setter, so a partial/garbled submit here can't strand the device
// off-network. The NVS writes can't run on this PSRAM-stacked httpd task, so
// it hands off to an internal-SRAM-stacked task that writes all keys and
// reboots (same pattern as tune_apply_reboot_task); LO/tagger/coalesce/gain
// are programmed at stream/detector start, hence the reboot to apply.
typedef struct {
    uint32_t    lo_hz;
    gain_mode_t gain_mode;
    int16_t     gain_dbx10;
    float       tag_thr_db;
    uint8_t     coal_n;
    uint32_t    at_lo_s;
    uint32_t    at_gain_s;
} sdrcfg_args_t;

static void sdrcfg_apply_reboot_task(void *arg)
{
    sdrcfg_args_t *a  = (sdrcfg_args_t *)arg;
    esp_err_t      r1 = app_config_set_lo_freq_hz(a->lo_hz);
    esp_err_t      r2 = app_config_set_gain_mode(a->gain_mode);
    esp_err_t      r3 = app_config_set_gain_db_x10(a->gain_dbx10);
    esp_err_t      r4 = app_config_set_tagger_threshold_db(a->tag_thr_db);
    esp_err_t      r5 = app_config_set_coalesce_min_bursts(a->coal_n);
    esp_err_t      r6 = app_config_set_autotune_lo_interval_s(a->at_lo_s);
    esp_err_t      r7 = app_config_set_autotune_gain_interval_s(a->at_gain_s);
    ESP_LOGI(TAG,
             "/sdrcfg: lo=%u mode=%d gain_dbx10=%d tag=%.1f coal=%u at_lo=%u at_gain=%u "
             "(%s/%s/%s/%s/%s/%s/%s) — rebooting to apply",
             (unsigned)a->lo_hz, (int)a->gain_mode, (int)a->gain_dbx10,
             (double)a->tag_thr_db, (unsigned)a->coal_n, (unsigned)a->at_lo_s,
             (unsigned)a->at_gain_s, esp_err_to_name(r1), esp_err_to_name(r2),
             esp_err_to_name(r3), esp_err_to_name(r4), esp_err_to_name(r5),
             esp_err_to_name(r6), esp_err_to_name(r7));
    free(a);
    vTaskDelay(pdMS_TO_TICKS(500));
    class_driver_prepare_for_reboot(); // park tuner so the dongle survives the reboot
    esp_restart();
}

static esp_err_t sdrcfg_post(httpd_req_t *req)
{
    static EXT_RAM_BSS_ATTR char body[512]; // single serve task — static is race-free (see config_post)
    if (req->content_len >= sizeof(body)) {
        httpd_resp_set_status(req, "413 Payload Too Large");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "form body too large\n", HTTPD_RESP_USE_STRLEN);
    }
    int total = 0, timeouts = 0;
    while (total < (int)req->content_len) {
        int r = httpd_req_recv(req, body + total, sizeof(body) - 1 - total);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts <= 3) continue;
            break;
        }
        total += r;
    }
    body[total] = '\0';

    // Every numeric field is pre-filled by the form, so a MISSING field means
    // a truncated/garbled body — reject rather than silently writing 0 (which
    // would misconfigure gain/threshold). No bias_tee here: it stays on the
    // Wi-Fi-safe /config form where it's already preserved.
    char lo_s[16] = {0}, gm_s[4] = {0}, gain_s[12] = {0}, tag_s[12] = {0},
         coal_s[6] = {0}, atlo_s[12] = {0}, atg_s[12] = {0};
    if (form_field(body, total, "lo_hz", lo_s, sizeof(lo_s)) != ESP_OK ||
        form_field(body, total, "gain_mode", gm_s, sizeof(gm_s)) != ESP_OK ||
        form_field(body, total, "gain_db", gain_s, sizeof(gain_s)) != ESP_OK ||
        form_field(body, total, "tag_thr", tag_s, sizeof(tag_s)) != ESP_OK ||
        form_field(body, total, "coal_n", coal_s, sizeof(coal_s)) != ESP_OK ||
        form_field(body, total, "at_lo_s", atlo_s, sizeof(atlo_s)) != ESP_OK ||
        form_field(body, total, "at_gain_s", atg_s, sizeof(atg_s)) != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "all SDR fields required (submit the form intact)\n",
                               HTTPD_RESP_USE_STRLEN);
    }

    uint32_t lo_hz = (uint32_t)strtoul(lo_s, NULL, 10);
    if (lo_hz < 1615000000u || lo_hz > 1628000000u) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "text/plain");
        char m[112];
        int  mn = snprintf(m, sizeof(m),
                           "lo_hz=%u out of Iridium band [1615000000, 1628000000]\n",
                           (unsigned)lo_hz);
        return httpd_resp_send(req, m, mn);
    }

    unsigned gm = (unsigned)strtoul(gm_s, NULL, 10);
    if (gm > 2) gm = 0;
    double        gain_db    = strtod(gain_s, NULL);
    int16_t       gain_dbx10 = (int16_t)(gain_db * 10.0 + (gain_db >= 0 ? 0.5 : -0.5));
    float         tag_thr    = strtof(tag_s, NULL);
    unsigned long coal       = strtoul(coal_s, NULL, 10);
    if (coal > 255) coal = 255;

    sdrcfg_args_t *a = calloc(1, sizeof(*a));
    if (!a) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "alloc failed\n", HTTPD_RESP_USE_STRLEN);
    }
    a->lo_hz      = lo_hz;
    a->gain_mode  = (gain_mode_t)gm;
    a->gain_dbx10 = gain_dbx10;
    a->tag_thr_db = tag_thr;
    a->coal_n     = (uint8_t)coal;
    a->at_lo_s    = (uint32_t)strtoul(atlo_s, NULL, 10);
    a->at_gain_s  = (uint32_t)strtoul(atg_s, NULL, 10);

    // Reply BEFORE spawning the writer (NVS commit disables flash cache,
    // which can disrupt the socket send — mirror config_post's ordering).
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    const char *ok =
        "<!doctype html><html><body style=\"font-family:system-ui;max-width:480px;margin:2em auto;padding:0 1em\">"
        "<h1>SDR config saved — rebooting</h1>"
        "<p>The device is applying the new tuning and will reboot in a few seconds. "
        "<a href=\"/\">Back to config</a> &middot; <a href=\"/status\">Status</a></p>"
        "</body></html>";
    httpd_resp_send(req, ok, HTTPD_RESP_USE_STRLEN);

    if (xTaskCreate(sdrcfg_apply_reboot_task, "sdrcfg", 4096, a, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "/sdrcfg: failed to spawn apply task");
        free(a);
    }
    return ESP_OK;
}

static esp_err_t sd_mount_post(httpd_req_t *req)
{
    esp_err_t      r = sd_log_force_mount();
    sd_log_stats_t s;
    sd_log_get_stats(&s);
    char body[256];
    int  n = snprintf(body, sizeof(body),
                      "{\"result\":\"%s\",\"mounted\":%s,\"log_path\":\"%s\",\"mount_error\":\"%s\"}",
                      esp_err_to_name(r),
                     s.mounted ? "true" : "false",
                      s.log_path,
                      s.mount_error);
    if (n < 0) n = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, (r == ESP_OK) ? "200 OK" : "503 Service Unavailable");
    return httpd_resp_send(req, body, n);
}

// GET /tasks — text dump of FreeRTOS task list (vTaskList). Tells us
// which tasks are Ready/Running/Blocked/Suspended and which CPU
// they're on. Essential for diagnosing "the X task isn't running"
// cases where ESP_LOGI silence alone isn't a positive signal.
//
// Output format (vTaskList):
//   Name State Prio StackHighWater Number CPU
// State letters: R running, B blocked, S suspended, X deleted.
static esp_err_t tasks_get(httpd_req_t *req)
{
    // vTaskList/vTaskGetRunTimeStats output is unbounded — it scales with
    // the LIVE task count, not a compile-time constant, and both write
    // with no length argument. Size from uxTaskGetNumberOfTasks():
    // ~40-80 B per task per dump, 128 B/task is generous headroom, ×2
    // for the two dumps, +512 for our headers. A fixed 4096 overflowed
    // once enough tasks were running.
    size_t cap = (size_t)uxTaskGetNumberOfTasks() * 128 * 2 + 512;
    char  *buf = heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    if (!buf) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "oom", HTTPD_RESP_USE_STRLEN);
    }
    // Header so the output's columns are obvious to a human reader.
    int n = snprintf(buf, cap,
                     "Name             State Prio Stack Num CPU\n"
                     "-------------------------------------------\n");
    vTaskList(buf + n);
    size_t off = strlen(buf);
    off += snprintf(buf + off, cap - off,
                    "\n=== CPU runtime stats (since boot) ===\n"
                    "Name             Time%%\n"
                    "----------------------\n");
    vTaskGetRunTimeStats(buf + off);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    heap_caps_free(buf);
    return ESP_OK;
}

// GET /sd/list — JSON array of files under /sdcard/acars/ with their
// on-disk sizes. Lets the operator confirm a capture file actually
// landed on the card before attempting a download (and lets external
// monitors decide when to pull or prune).
static esp_err_t sd_list_get(httpd_req_t *req)
{
    DIR *d = opendir("/sdcard/acars");
    if (!d) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"error\":\"opendir failed\"}",
                               HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send_chunk(req, "[", 1);
    struct dirent *e;
    bool           first = true;
    char           line[160];
    while ((e = readdir(d)) != NULL) {
        if (e->d_name[0] == '.') continue;
        char path[320];
        snprintf(path, sizeof(path), "/sdcard/acars/%s", e->d_name);
        struct stat st;
        bool        ok = (stat(path, &st) == 0);
        long long   sz = ok ? (long long)st.st_size : -1;
        long long   mt = ok ? (long long)st.st_mtime : 0;
        int         n  = snprintf(line, sizeof(line),
                                  "%s{\"name\":\"%s\",\"size\":%lld,\"mtime\":%lld}",
                         first ? "" : ",", e->d_name, sz, mt);
        if (n >= (int)sizeof(line)) n = sizeof(line) - 1;
        if (n > 0) httpd_resp_send_chunk(req, line, n);
        first = false;
    }
    closedir(d);
    httpd_resp_send_chunk(req, "]", 1);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// POST /sd/format — wipe + reformat the SD card. Destroys all data
// on the card. Takes 30-180 s depending on card size; httpd timeout
// gets bumped via the task WDT bump inside sd_log_force_format.
static esp_err_t sd_format_post(httpd_req_t *req)
{
    // Refuse if a capture is currently writing — otherwise format
    // would yank the file out from under the writer.
    sd_capture_stats_t cs;
    sd_capture_get_stats(&cs);
    if (cs.active) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_send(req,
                               "capture is active — POST /capture/stop first\n",
                               HTTPD_RESP_USE_STRLEN);
    }
    esp_err_t r = sd_log_force_format();
    char      body[128];
    int       n = snprintf(body, sizeof(body),
                           "{\"result\":\"%s\"}", esp_err_to_name(r));
    if (n < 0) n = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, (r == ESP_OK) ? "200 OK" : "500 Internal Server Error");
    return httpd_resp_send(req, body, n);
}

// POST /sd/delete?name=<filename> — unlink one file under
// /sdcard/acars/. FIFO eviction policy is driven by the host-side
// monitor (uses /sd/list to pick the oldest). Refuses to delete the
// active capture file, refuses path traversal.
static esp_err_t sd_delete_post(httpd_req_t *req)
{
    char query[96] = {0};
    char name[64]  = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK ||
        name[0] == '\0' ||
        strstr(name, "..") || strchr(name, '/')) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req,
                               "?name=<bare-filename> required (no slashes, no ..)\n",
                               HTTPD_RESP_USE_STRLEN);
    }

    char path[96];
    int  wrote = snprintf(path, sizeof(path), "/sdcard/acars/%s", name);
    if (wrote <= 0 || wrote >= (int)sizeof(path)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "name too long\n", HTTPD_RESP_USE_STRLEN);
    }

    sd_capture_stats_t cs;
    sd_capture_get_stats(&cs);
    if (cs.active && cs.file_open && strcmp(path, cs.path) == 0) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req,
                               "refusing to delete the active capture file — stop first\n",
                               HTTPD_RESP_USE_STRLEN);
    }

    int r = unlink(path);
    if (r != 0) {
        httpd_resp_set_status(req, (errno == ENOENT) ? "404 Not Found" : "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        char msg[96];
        int  mn = snprintf(msg, sizeof(msg), "unlink failed: %s\n", strerror(errno));
        return httpd_resp_send(req, msg, mn);
    }
    char body[128];
    int  n = snprintf(body, sizeof(body),
                      "{\"result\":\"ESP_OK\",\"deleted\":\"%s\"}", name);
    if (n < 0) n = 0;
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, n);
}

// POST /capture/start — body is optional JSON like {"target_bytes":N}
// (default = unlimited). Returns the resulting file path + initial
// stats. 503 if SD mount fails or capture is already active.
static esp_err_t capture_start_post(httpd_req_t *req)
{
    char body[128] = {0};
    int  len       = req->content_len;
    if (len >= (int)sizeof(body)) {
        // Oversized body: previously this fell straight through with an
        // empty body (as if no JSON was posted) instead of reporting the
        // problem. Reject explicitly, mirroring config_post's oversized-body
        // handling above. esp_http_server drains any unread body bytes for
        // us once the handler returns (httpd_req_delete), so this doesn't
        // leave the connection mid-body for the next keep-alive request.
        httpd_resp_set_status(req, "413 Payload Too Large");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "body too large\n", HTTPD_RESP_USE_STRLEN);
    }
    if (len > 0) {
        int got = httpd_req_recv(req, body, len);
        if (got <= 0)
            body[0] = '\0';
        else
            body[got] = '\0';
    }

    uint64_t target = 0;
    // Tiny "find a number after target_bytes" parse; full JSON is
    // overkill for one field. Accepts "target_bytes":N or
    // "target_bytes": N variants.
    const char *p = strstr(body, "target_bytes");
    if (p) {
        p = strchr(p, ':');
        if (p) {
            p++;
            while (*p == ' ')
                p++;
            target = strtoull(p, NULL, 10);
        }
    }

    // Optional mode flag: {"mode":"bursts"} switches to per-burst
    // IQ record capture (lossless on any SD card — tagged bursts
    // only, ~40-160 KB per burst, ~0.5-10 bursts/sec real rate).
    // Default is continuous (raw uint8 USB stream, needs fast card).
    bool burst_mode = false;
    if (strstr(body, "\"bursts\"") || strstr(body, "burst")) {
        burst_mode = true;
    }

    esp_err_t          r = burst_mode ? sd_capture_start_bursts()
                                      : sd_capture_start(target);
    sd_capture_stats_t s;
    sd_capture_get_stats(&s);
    char resp[256];
    int  n = snprintf(resp, sizeof(resp),
                      "{\"result\":\"%s\",\"active\":%s,\"mode\":\"%s\",\"path\":\"%s\","
                       "\"target_bytes\":%llu}",
                      esp_err_to_name(r),
                     s.active ? "true" : "false",
                     burst_mode ? "bursts" : "continuous",
                      s.path,
                      (unsigned long long)s.bytes_target);
    if (n < 0) n = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, (r == ESP_OK) ? "200 OK" : "503 Service Unavailable");
    return httpd_resp_send(req, resp, n);
}

// POST /capture/stop — flushes and closes the current capture file.
// Idempotent in spirit; returns 409 if no capture was active. A slow
// drain (multi-MB stream-buffer backlog) returns 202 with
// "stopping":true — the writer task finishes the close asynchronously;
// poll /capture/status for active:false. We must NOT block here: this
// runs on the single httpd serve task (see task #101).
static esp_err_t capture_stop_post(httpd_req_t *req)
{
    esp_err_t r = sd_capture_stop();
    // ESP_ERR_TIMEOUT = stop accepted, drain still in progress.
    bool               stopping = (r == ESP_ERR_TIMEOUT);
    sd_capture_stats_t s;
    sd_capture_get_stats(&s);
    char resp[300];
    int  n = snprintf(resp, sizeof(resp),
                      "{\"result\":\"%s\",\"stopping\":%s,\"path\":\"%s\","
                       "\"bytes_captured\":%llu,\"bytes_dropped\":%lu,"
                       "\"write_errors\":%lu}",
                      esp_err_to_name(r),
                     stopping ? "true" : "false",
                      s.path,
                      (unsigned long long)s.bytes_captured,
                      (unsigned long)s.bytes_dropped,
                      (unsigned long)s.write_errors);
    if (n < 0) n = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, (r == ESP_OK) ? "200 OK"
                               : stopping    ? "202 Accepted"
                                             : "409 Conflict");
    return httpd_resp_send(req, resp, n);
}

// GET /capture/file?name=iq-NNNN.u8 — stream a captured file back
// to the client. The file lives under /sdcard/acars/ and we
// validate that `name` is a bare filename (no traversal). 404 if
// the file doesn't exist, 409 if the capture is still writing to
// that file (we don't want to serve a partial / in-flight write).
static esp_err_t capture_file_get(httpd_req_t *req)
{
    char query[96] = {0};
    char name[64]  = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "name", name, sizeof(name)) != ESP_OK ||
        name[0] == '\0') {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req,
                               "?name=<filename> required (try /capture/status for the current path)\n",
                               HTTPD_RESP_USE_STRLEN);
    }

    // Validate same-path checks via sd_capture_open_for_read, but
    // immediately drop down to the POSIX fd. fread/fopen-style IO
    // was returning 0 bytes on otherwise-valid multi-MB files after
    // the device had been running ~10 min — the libc FILE buffering
    // path (or its interaction with FATFS' sector cache) goes stale
    // somehow under sustained-capture pressure. The raw fd path
    // doesn't go through that buffering and stays reliable.
    FILE *fp = sd_capture_open_for_read(name);
    if (!fp) {
        if (errno == EBUSY) {
            httpd_resp_set_status(req, "409 Conflict");
            httpd_resp_set_type(req, "text/plain");
            return httpd_resp_send(req,
                                   "capture is still writing this file — stop first\n",
                                   HTTPD_RESP_USE_STRLEN);
        }
        httpd_resp_set_status(req, "404 Not Found");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "no such file\n", HTTPD_RESP_USE_STRLEN);
    }
    int fd = fileno(fp);

    // Size for the log line — fstat() goes through the same VFS
    // layer as the fd; consistent with what we'll actually read.
    struct stat st;
    long long   size_stat = (fstat(fd, &st) == 0) ? (long long)st.st_size : -1;
    ESP_LOGI(TAG, "/capture/file: serving %s (size=%lld bytes, fd=%d)",
             name, size_stat, fd);

    httpd_resp_set_type(req, "application/octet-stream");
    char disp[96];
    snprintf(disp, sizeof(disp), "attachment; filename=\"%s\"", name);
    httpd_resp_set_hdr(req, "Content-Disposition", disp);

    // Use the pre-allocated DMA-INT read buffer (s_download_buf,
    // allocated once in http_server_start while DMA-INT still had
    // 139 KB largest contiguous). The buf lives at a known address
    // so SDMMC can DMA straight into it without going through its
    // stash buffer.
    if (!s_download_buf) {
        fclose(fp);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "download buf unavailable",
                               HTTPD_RESP_USE_STRLEN);
    }

    // No cooperative yield here: the USB consumer (class task) runs at a
    // higher priority than httpd (see CLASS_TASK_PRIORITY) and is
    // event-driven, so it preempts this download loop whenever USB data
    // arrives and keeps the ringbuffer drained. A vTaskDelay() yield was
    // tried and didn't help — httpd outranked the consumer at the time,
    // so the tick just bounced straight back to httpd. See task #91.
    size_t total_sent  = 0;
    int    read_errors = 0;
    while (1) {
        ssize_t n = read(fd, s_download_buf, DOWNLOAD_BUF_BYTES);
        if (n < 0) {
            ESP_LOGW(TAG, "/capture/file: read() err=%d after %zu bytes",
                     errno, total_sent);
            if (++read_errors > 3) break;
            continue;
        }
        if (n == 0) break; // EOF
        if (httpd_resp_send_chunk(req, (const char *)s_download_buf, n) != ESP_OK) {
            fclose(fp);
            return ESP_FAIL;
        }
        total_sent += (size_t)n;
    }
    ESP_LOGI(TAG, "/capture/file: sent %zu bytes", total_sent);

    fclose(fp);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// GET /capture/status — live snapshot, whether capture is active
// or not. The path/bytes fields persist across stop so the operator
// can see how much they captured after stopping.
static esp_err_t capture_status_get(httpd_req_t *req)
{
    sd_capture_stats_t s;
    sd_capture_get_stats(&s);
    char resp[320];
    int  n = snprintf(resp, sizeof(resp),
                      "{\"active\":%s,\"file_open\":%s,\"path\":\"%s\","
                       "\"bytes_captured\":%llu,\"bytes_target\":%llu,"
                       "\"bytes_dropped\":%lu,\"write_errors\":%lu,"
                       "\"start_us\":%lld}",
                     s.active ? "true" : "false",
                     s.file_open ? "true" : "false",
                      s.path,
                      (unsigned long long)s.bytes_captured,
                      (unsigned long long)s.bytes_target,
                      (unsigned long)s.bytes_dropped,
                      (unsigned long)s.write_errors,
                      (long long)s.start_us);
    if (n < 0) n = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, resp, n);
}

static esp_err_t reset_post(httpd_req_t *req)
{
    ESP_LOGW(TAG, "/reset POST: clearing Wi-Fi NVS + rebooting to AP mode");

    // Same defer-to-internal-stack pattern as config_post — NVS
    // commit can't run on this PSRAM-stacked httpd task.
    nvs_save_args_t *args = calloc(1, sizeof(*args));
    if (!args) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "alloc failed\n", HTTPD_RESP_USE_STRLEN);
    }
    args->clear_only = true;

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    const char *ok =
        "<!doctype html><html><body style=\"font-family:system-ui;max-width:480px;margin:2em auto;padding:0 1em\">"
        "<h1>Reset — rebooting to AP mode</h1>"
        "<p>Wi-Fi credentials cleared. The device will reboot and come "
        "back up as an open AP. Re-join it to configure new credentials.</p>"
        "</body></html>";
    httpd_resp_send(req, ok, HTTPD_RESP_USE_STRLEN);

    BaseType_t spawned = xTaskCreatePinnedToCore(nvs_save_and_reboot_task,
                                                 "nvs_save", 4096, args, 5, NULL, tskNO_AFFINITY);
    if (spawned != pdPASS) {
        ESP_LOGE(TAG, "nvs_save task spawn failed — NVS not cleared, no reboot");
        free(args);
    }
    return ESP_OK;
}

// Plain restart (no NVS write). Parks the tuner first so the RTL-SDR survives
// the reboot (same graceful path as the config reboots — see
// tune_apply_reboot_task); without this the dongle latches mid-I2C and the
// next boot has to re-enumerate a wedged tuner. Runs on its own internal-SRAM
// task because esp_restart() + class_driver_prepare_for_reboot() must not run
// on the PSRAM-stacked httpd task. The short delay lets the HTTP reply flush.
static void reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGW(TAG, "/reboot: operator-requested restart");
    class_driver_prepare_for_reboot();
    esp_restart();
}

// POST /reboot — operator-triggered graceful restart. Content-negotiated:
// browsers (Accept: text/html) get a self-refreshing "rebooting" page that
// returns to /status; curl / scripts get JSON. Reply is sent BEFORE the task
// is spawned so the client sees the confirmation before the link drops.
static esp_err_t reboot_post(httpd_req_t *req)
{
    char accept[160] = {0};
    httpd_req_get_hdr_value_str(req, "Accept", accept, sizeof(accept));
    if (strstr(accept, "text/html")) {
        httpd_resp_set_type(req, "text/html; charset=utf-8");
        httpd_resp_sendstr(req,
            "<!doctype html><html><head><meta charset=\"utf-8\">"
            "<meta http-equiv=\"refresh\" content=\"12;url=/status\"></head>"
            "<body style=\"font-family:system-ui;max-width:480px;margin:2em auto;padding:0 1em\">"
            "<h1>Rebooting…</h1>"
            "<p>Parking the tuner and restarting — typically ~10 s. This page "
            "will return to <a href=\"/status\">Status</a> automatically.</p>"
            "</body></html>");
    } else {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"result\":\"ok\",\"reboot\":true}");
    }

    if (xTaskCreate(reboot_task, "reboot", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "/reboot: failed to spawn reboot task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t http_server_start(void)
{
    if (s_server) return ESP_OK;

    // T49b: moved from DMA-INT to PSRAM to give the USB transfer pool
    // back 4 KB of the critically-tight DMA-internal budget (the pool
    // was short of its 8 transfers; the convert/resample tile needs the
    // room). The SDMMC read into a PSRAM buffer uses the driver's bounce
    // ("stash") path instead of DMA-ing directly — acceptable because
    // /capture/file is a rare, manual, already-stream-disruptive
    // operation, not the hot path. Still pre-allocated here (before the
    // pool/tagger) so the download handler never does a lazy alloc.
    s_download_buf = heap_caps_aligned_alloc(64, DOWNLOAD_BUF_BYTES,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_download_buf) {
        ESP_LOGI(TAG, "pre-allocated %d-byte PSRAM download buf @ %p",
                 DOWNLOAD_BUF_BYTES, s_download_buf);
    } else {
        ESP_LOGW(TAG, "download buf alloc failed — /capture/file will 500");
    }

    httpd_config_t cfg   = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = 80;
    cfg.max_uri_handlers = HTTPD_URI_LIMIT;
    cfg.lru_purge_enable = true;
    cfg.stack_size       = 12288; // HTML dashboard render (status_html_get: snapshot+cfg on stack + framework) overflowed 6144 (Stack protection fault)
    // Pin to Core 0: Core 1 is ~98% saturated (ingest + worker), so a
    // no-affinity httpd task can get parked there and barely run. Core 0
    // has ~23% idle headroom. Prio 5 sits BELOW class_driver (6) on
    // Core 0 — deliberate, so USB drain preempts HTTP work (see
    // CLASS_TASK_PRIORITY in usb_host_lib_main.c).
    // recv/send timeouts cut to 2 s (default 5 s): under the
    // esp_hosted SDIO TX throttle a blocking send() can wedge this single
    // serve task, making the WHOLE server unreachable; a short timeout
    // bounds that worst case so one stalled connection can't hold off
    // accept() for everyone. See task #101.
    cfg.task_priority     = 5;
    cfg.core_id           = 0;
    cfg.recv_wait_timeout = 2;
    cfg.send_wait_timeout = 2;
    // HTTP server task stack in PSRAM — default task_caps is
    // MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT, which on P4 also satisfies
    // MALLOC_CAP_DMA and steals from USB pool. esp_http_server is
    // request/response over TCP, latency-tolerant; PSRAM stack is fine.
    cfg.task_caps = MALLOC_CAP_SPIRAM;

    esp_err_t r = httpd_start(&s_server, &cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(r));
        s_server = NULL;
        return r;
    }

    // Adding a route past HTTPD_URI_LIMIT past would cause
    // httpd_register_uri_handler to return ESP_ERR_HTTPD_HANDLERS_FULL
    // -> the ESP_ERROR_CHECK below aborts -> panic-loop on boot. The
    // static_assert after the array initializer catches the overflow at
    // build time so we never re-pay the "1372 reboots to find it" tax
    // (see commit 2260ae9 / feedback_httpd_max_uri_handlers memory note).
    httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = index_get, .user_ctx = NULL},
        {.uri = "/status", .method = HTTP_GET, .handler = status_get, .user_ctx = NULL},
        {.uri = "/diag/histograms", .method = HTTP_GET, .handler = diag_histograms_get, .user_ctx = NULL},
        {.uri = "/diag/dcfine", .method = HTTP_GET, .handler = diag_dcfine_get, .user_ctx = NULL},
        {.uri = "/diag/dsp_health", .method = HTTP_GET, .handler = diag_dsp_health_get, .user_ctx = NULL},
        {.uri = "/diag/recovery_counters", .method = HTTP_GET, .handler = diag_recovery_counters_get, .user_ctx = NULL},
        {.uri = "/messages", .method = HTTP_GET, .handler = messages_get, .user_ctx = NULL},
        {.uri = "/ota", .method = HTTP_GET, .handler = ota_get, .user_ctx = NULL},
        {.uri = "/config", .method = HTTP_POST, .handler = config_post, .user_ctx = NULL},
        {.uri = "/reset", .method = HTTP_POST, .handler = reset_post, .user_ctx = NULL},
        {.uri = "/ota", .method = HTTP_POST, .handler = ota_post, .user_ctx = NULL},
        {.uri = "/debug/inject", .method = HTTP_POST, .handler = debug_inject_post, .user_ctx = NULL},
#if CONFIG_FAULT_INJECT
        {.uri = "/debug/fault_inject", .method = HTTP_POST, .handler = debug_fault_inject_post, .user_ctx = NULL},
#endif
        {.uri = "/tune", .method = HTTP_POST, .handler = tune_post, .user_ctx = NULL},
        {.uri = "/sdrcfg", .method = HTTP_POST, .handler = sdrcfg_post, .user_ctx = NULL},
        {.uri = "/sd/mount", .method = HTTP_POST, .handler = sd_mount_post, .user_ctx = NULL},
        {.uri = "/sd/format", .method = HTTP_POST, .handler = sd_format_post, .user_ctx = NULL},
        {.uri = "/sd/delete", .method = HTTP_POST, .handler = sd_delete_post, .user_ctx = NULL},
        {.uri = "/sd/list", .method = HTTP_GET, .handler = sd_list_get, .user_ctx = NULL},
        {.uri = "/tasks", .method = HTTP_GET, .handler = tasks_get, .user_ctx = NULL},
        {.uri = "/capture/start", .method = HTTP_POST, .handler = capture_start_post, .user_ctx = NULL},
        {.uri = "/capture/stop", .method = HTTP_POST, .handler = capture_stop_post, .user_ctx = NULL},
        {.uri = "/capture/status", .method = HTTP_GET, .handler = capture_status_get, .user_ctx = NULL},
        {.uri = "/capture/file", .method = HTTP_GET, .handler = capture_file_get, .user_ctx = NULL},
        {.uri = "/scan", .method = HTTP_POST, .handler = scan_post, .user_ctx = NULL},
        {.uri = "/autotune", .method = HTTP_POST, .handler = autotune_post, .user_ctx = NULL},
        {.uri = "/gaincal", .method = HTTP_POST, .handler = gaincal_post, .user_ctx = NULL},
        {.uri = "/reboot", .method = HTTP_POST, .handler = reboot_post, .user_ctx = NULL},
    };
    _Static_assert(sizeof(routes) / sizeof(routes[0]) <= HTTPD_URI_LIMIT,
                   "route count exceeds HTTPD_URI_LIMIT; bump HTTPD_URI_LIMIT "
                   "in http_server.c before adding more routes (see "
                   "feedback_httpd_max_uri_handlers memory)");
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &routes[i]));
    }

    ESP_LOGI(TAG, "HTTP server up on port 80 — GET /, /status, /messages, /ota, /capture/status, /capture/file; "
                  "POST /config, /reset, /ota, /sd/mount, /capture/start, /capture/stop");
    return ESP_OK;
}
