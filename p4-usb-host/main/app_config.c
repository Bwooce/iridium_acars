// See app_config.h. NVS namespace "iridium" holds individual keys.
// We keep keys flat (not blob-encoded) so a future C6 web UI can
// edit them one at a time without round-tripping the whole struct.

#include "app_config.h"
#include "dsp_processor.h" // FS_IN_HZ, IRIDIUM_CENTER_FREQ_HZ defaults

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG    = "APP_CFG";
static const char *NVS_NS = "iridium";

// Default values. Conservative for development (TUNER_AGC, no
// bias-tee). Production deployments override via the C6 web UI
// (D17) once that lands.
#define DEFAULT_LO_FREQ_HZ IRIDIUM_CENTER_FREQ_HZ
#define DEFAULT_SAMPLE_RATE_HZ FS_IN_HZ
#define DEFAULT_GAIN_MODE GAIN_MODE_TUNER_AGC
#define DEFAULT_GAIN_DB_X10 350 // 35.0 dB (rec'd for live)
#define DEFAULT_BIAS_TEE false
#define DEFAULT_TAGGER_THRESHOLD_DB 10.0f
#define DEFAULT_COALESCE_MIN_BURSTS 0 // 0 = coalescer disabled (gri-parity dispatch)
#define DEFAULT_DCMASK_LO 1           // lo>hi => disabled by default
#define DEFAULT_DCMASK_HI (-1)
#define DEFAULT_AUTOTUNE_GAIN_DWELL_S 180      // 55 s too noisy (design §Dwell adequacy)
#define DEFAULT_AUTOTUNE_IRA_LO_HZ 1626200000u // IRA simplex allocation, fixed
#define DEFAULT_AUTOTUNE_GAIN_MIN_DBX10 80     // skip the deaf 0..7.7 dB low end
#define DEFAULT_AUTOTUNE_GAIN_MAX_DBX10 460    // skip the 48/49.6 dB saturating top
#define DEFAULT_AUTOTUNE_GAIN_STRIDE 3         // coarse: every 3rd R828D step
#define DEFAULT_AUTOTUNE_ON_BOOT false         // don't eat a boot on every reboot
// Split cadence (design doc §Recalibration period): gain drifts SLOWLY (RFI/
// thermal) and its cal is EXPENSIVE (~30 min IRA sweep, ~10 gains × 180 s), so
// run it rarely — every 12 h = only ~4% duty off-ACARS. The LO/density re-scan
// is satellite-driven (~100 min LEO, frequent handoffs) and cheap (~1 min), so
// run it more often. NB: best-LO is noisy/mean-reverting (see
// reference_freq_coverage_analysis) — 30 min is a safe interim until multi-scan
// averaging lands; the design's ~600 s ideal would thrash on single snapshots.
#define DEFAULT_AUTOTUNE_GAIN_INTERVAL_S 43200u // 12 h (RFI/thermal-driven, slow, ~30 min pass)
#define DEFAULT_AUTOTUNE_LO_INTERVAL_S 1800u    // 30 min (satellite-driven; validated 42-hop scan).
                                               // Only runs when gain_mode==MANUAL. Each run logs a
                                               // distinctive AUTOTUNE-START marker so any future wedge
                                               // can be correlated to the hop that caused it.
#define DEFAULT_STATION_ID "p4-iridium-1"
#define DEFAULT_WIFI_SSID ""
#define DEFAULT_WIFI_PSK ""

static app_config_t      s_cfg;
static SemaphoreHandle_t s_cfg_mu = NULL;

// --- NVS helpers ---------------------------------------------------------

static esp_err_t nvs_get_u32_or(nvs_handle_t h, const char *k, uint32_t *v, uint32_t def)
{
    esp_err_t r = nvs_get_u32(h, k, v);
    if (r == ESP_ERR_NVS_NOT_FOUND) {
        *v = def;
        return ESP_OK;
    }
    return r;
}
static esp_err_t nvs_get_i16_or(nvs_handle_t h, const char *k, int16_t *v, int16_t def)
{
    esp_err_t r = nvs_get_i16(h, k, v);
    if (r == ESP_ERR_NVS_NOT_FOUND) {
        *v = def;
        return ESP_OK;
    }
    return r;
}
static esp_err_t nvs_get_u8_or(nvs_handle_t h, const char *k, uint8_t *v, uint8_t def)
{
    esp_err_t r = nvs_get_u8(h, k, v);
    if (r == ESP_ERR_NVS_NOT_FOUND) {
        *v = def;
        return ESP_OK;
    }
    return r;
}
static esp_err_t nvs_get_str_or(nvs_handle_t h, const char *k, char *out, size_t cap,
                                const char *def)
{
    size_t    len = cap;
    esp_err_t r   = nvs_get_str(h, k, out, &len);
    if (r == ESP_ERR_NVS_NOT_FOUND) {
        strncpy(out, def, cap - 1);
        out[cap - 1] = '\0';
        return ESP_OK;
    }
    return r;
}

// Same pattern for a float, stored as a u32 by bit-cast (NVS has
// no native float type). Trivial and survives reboots.
static esp_err_t nvs_get_f32_or(nvs_handle_t h, const char *k, float *v, float def)
{
    uint32_t  raw;
    esp_err_t r = nvs_get_u32(h, k, &raw);
    if (r == ESP_ERR_NVS_NOT_FOUND) {
        *v = def;
        return ESP_OK;
    }
    if (r != ESP_OK) return r;
    memcpy(v, &raw, sizeof(float));
    return ESP_OK;
}
static esp_err_t nvs_set_f32(nvs_handle_t h, const char *k, float v)
{
    uint32_t raw;
    memcpy(&raw, &v, sizeof(uint32_t));
    return nvs_set_u32(h, k, raw);
}

// --- Public API ----------------------------------------------------------

esp_err_t app_config_init(void)
{
    if (s_cfg_mu) return ESP_OK; // idempotent

    s_cfg_mu = xSemaphoreCreateMutex();
    if (!s_cfg_mu) return ESP_ERR_NO_MEM;

    // Apply defaults FIRST so the struct is well-defined even if
    // NVS init fails completely.
    s_cfg.lo_freq_hz               = DEFAULT_LO_FREQ_HZ;
    s_cfg.sample_rate_hz           = DEFAULT_SAMPLE_RATE_HZ;
    s_cfg.gain_mode                = DEFAULT_GAIN_MODE;
    s_cfg.gain_db_x10              = DEFAULT_GAIN_DB_X10;
    s_cfg.bias_tee                 = DEFAULT_BIAS_TEE;
    s_cfg.tagger_threshold_db      = DEFAULT_TAGGER_THRESHOLD_DB;
    s_cfg.coalesce_min_bursts      = DEFAULT_COALESCE_MIN_BURSTS;
    s_cfg.dcmask_lo                = DEFAULT_DCMASK_LO;
    s_cfg.dcmask_hi                = DEFAULT_DCMASK_HI;
    s_cfg.autotune_gain_dwell_s    = DEFAULT_AUTOTUNE_GAIN_DWELL_S;
    s_cfg.autotune_ira_lo_hz       = DEFAULT_AUTOTUNE_IRA_LO_HZ;
    s_cfg.autotune_gain_min_dbx10  = DEFAULT_AUTOTUNE_GAIN_MIN_DBX10;
    s_cfg.autotune_gain_max_dbx10  = DEFAULT_AUTOTUNE_GAIN_MAX_DBX10;
    s_cfg.autotune_gain_stride     = DEFAULT_AUTOTUNE_GAIN_STRIDE;
    s_cfg.autotune_on_boot         = DEFAULT_AUTOTUNE_ON_BOOT;
    s_cfg.autotune_gain_interval_s = DEFAULT_AUTOTUNE_GAIN_INTERVAL_S;
    s_cfg.autotune_lo_interval_s   = DEFAULT_AUTOTUNE_LO_INTERVAL_S;
    strncpy(s_cfg.station_id, DEFAULT_STATION_ID, APP_CONFIG_STATION_ID_LEN - 1);
    s_cfg.station_id[APP_CONFIG_STATION_ID_LEN - 1] = '\0';
    s_cfg.wifi_ssid[0]                              = '\0';
    s_cfg.wifi_psk[0]                               = '\0';
    s_cfg.out_host[0]                               = '\0';
    s_cfg.out_port                                  = 0;
    s_cfg.ota_url[0]                                = '\0';

    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase, performing");
        (void)nvs_flash_erase();
        r = nvs_flash_init();
    }
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "nvs_flash_init failed (%s); using defaults",
                 esp_err_to_name(r));
        return ESP_OK; // defaults already populated
    }

    nvs_handle_t h;
    r = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (r == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No NVS namespace yet; using defaults");
        return ESP_OK;
    }
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed (%s); using defaults",
                 esp_err_to_name(r));
        return ESP_OK;
    }

    uint8_t gm = (uint8_t)DEFAULT_GAIN_MODE;
    uint8_t bt = (uint8_t)DEFAULT_BIAS_TEE;
    nvs_get_u32_or(h, "lo_hz", &s_cfg.lo_freq_hz, DEFAULT_LO_FREQ_HZ);
    nvs_get_u32_or(h, "rate_hz", &s_cfg.sample_rate_hz, DEFAULT_SAMPLE_RATE_HZ);
    nvs_get_u8_or(h, "gain_mode", &gm, (uint8_t)DEFAULT_GAIN_MODE);
    nvs_get_i16_or(h, "gain_dbx10", &s_cfg.gain_db_x10, DEFAULT_GAIN_DB_X10);
    nvs_get_u8_or(h, "bias_tee", &bt, (uint8_t)DEFAULT_BIAS_TEE);
    nvs_get_f32_or(h, "tag_thr", &s_cfg.tagger_threshold_db, DEFAULT_TAGGER_THRESHOLD_DB);
    nvs_get_u8_or(h, "coal_n", &s_cfg.coalesce_min_bursts, DEFAULT_COALESCE_MIN_BURSTS);
    nvs_get_i16_or(h, "dcmask_lo", &s_cfg.dcmask_lo, DEFAULT_DCMASK_LO);
    nvs_get_i16_or(h, "dcmask_hi", &s_cfg.dcmask_hi, DEFAULT_DCMASK_HI);
    nvs_get_u32_or(h, "at_dwell_s", &s_cfg.autotune_gain_dwell_s, DEFAULT_AUTOTUNE_GAIN_DWELL_S);
    nvs_get_u32_or(h, "at_ira_hz", &s_cfg.autotune_ira_lo_hz, DEFAULT_AUTOTUNE_IRA_LO_HZ);
    nvs_get_i16_or(h, "at_g_min", &s_cfg.autotune_gain_min_dbx10, DEFAULT_AUTOTUNE_GAIN_MIN_DBX10);
    nvs_get_i16_or(h, "at_g_max", &s_cfg.autotune_gain_max_dbx10, DEFAULT_AUTOTUNE_GAIN_MAX_DBX10);
    nvs_get_u8_or(h, "at_g_strd", &s_cfg.autotune_gain_stride, DEFAULT_AUTOTUNE_GAIN_STRIDE);
    uint8_t at_on_boot = (uint8_t)DEFAULT_AUTOTUNE_ON_BOOT;
    nvs_get_u8_or(h, "at_on_boot", &at_on_boot, (uint8_t)DEFAULT_AUTOTUNE_ON_BOOT);
    s_cfg.autotune_on_boot = (bool)at_on_boot;
    nvs_get_u32_or(h, "at_g_ivl_s", &s_cfg.autotune_gain_interval_s, DEFAULT_AUTOTUNE_GAIN_INTERVAL_S);
    nvs_get_u32_or(h, "at_lo_ivl_s", &s_cfg.autotune_lo_interval_s, DEFAULT_AUTOTUNE_LO_INTERVAL_S);
    nvs_get_str_or(h, "station", s_cfg.station_id, APP_CONFIG_STATION_ID_LEN,
                   DEFAULT_STATION_ID);
    nvs_get_str_or(h, "wifi_ssid", s_cfg.wifi_ssid, APP_CONFIG_WIFI_SSID_LEN, "");
    nvs_get_str_or(h, "wifi_psk", s_cfg.wifi_psk, APP_CONFIG_WIFI_PSK_LEN, "");
    nvs_get_str_or(h, "out_host", s_cfg.out_host, APP_CONFIG_OUT_HOST_LEN, "");
    uint16_t out_port = 0;
    {
        size_t sz = sizeof(out_port);
        if (nvs_get_blob(h, "out_port", &out_port, &sz) != ESP_OK ||
            sz != sizeof(out_port)) {
            out_port = 0;
        }
    }
    s_cfg.out_port = out_port;
    nvs_get_str_or(h, "ota_url", s_cfg.ota_url, APP_CONFIG_OTA_URL_LEN, "");
    // gm is whatever byte was stored in NVS — validate against the
    // known enum range before the cast; a stale/corrupt/foreign value
    // must not become an out-of-range gain_mode_t.
    if (gm > (uint8_t)GAIN_MODE_SOFTWARE_AGC) {
        ESP_LOGW(TAG, "NVS gain_mode=%u out of range; using default", gm);
        gm = (uint8_t)DEFAULT_GAIN_MODE;
    }
    s_cfg.gain_mode = (gain_mode_t)gm;
    s_cfg.bias_tee  = (bool)bt;

    nvs_close(h);
    return ESP_OK;
}

void app_config_snapshot(app_config_t *out)
{
    if (!out || !s_cfg_mu) return;
    xSemaphoreTake(s_cfg_mu, portMAX_DELAY);
    *out = s_cfg;
    xSemaphoreGive(s_cfg_mu);
}

const app_config_t *app_config_get_locked(void)
{
    if (!s_cfg_mu) return NULL;
    xSemaphoreTake(s_cfg_mu, portMAX_DELAY);
    return &s_cfg;
}

void app_config_unlock(void)
{
    if (s_cfg_mu) xSemaphoreGive(s_cfg_mu);
}

// --- Setters --------------------------------------------------------------

static esp_err_t commit_one_u32(const char *k, uint32_t v)
{
    nvs_handle_t h;
    esp_err_t    r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_u32(h, k, v);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}
static esp_err_t commit_one_u8(const char *k, uint8_t v)
{
    nvs_handle_t h;
    esp_err_t    r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_u8(h, k, v);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}
static esp_err_t commit_one_i16(const char *k, int16_t v)
{
    nvs_handle_t h;
    esp_err_t    r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_i16(h, k, v);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}
static esp_err_t commit_one_f32(const char *k, float v)
{
    nvs_handle_t h;
    esp_err_t    r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_f32(h, k, v);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}
static esp_err_t commit_one_str(const char *k, const char *v)
{
    nvs_handle_t h;
    esp_err_t    r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_str(h, k, v);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}

#define SET_FIELD_NUM(field_setter_name, member, ctype, nvs_key, commit_fn) \
    esp_err_t field_setter_name(ctype v)                                    \
    {                                                                       \
        if (!s_cfg_mu) return ESP_ERR_INVALID_STATE;                        \
        xSemaphoreTake(s_cfg_mu, portMAX_DELAY);                            \
        s_cfg.member = v;                                                   \
        xSemaphoreGive(s_cfg_mu);                                           \
        return commit_fn(nvs_key, v);                                       \
    }

SET_FIELD_NUM(app_config_set_lo_freq_hz, lo_freq_hz, uint32_t, "lo_hz", commit_one_u32)
SET_FIELD_NUM(app_config_set_sample_rate_hz, sample_rate_hz, uint32_t, "rate_hz", commit_one_u32)
SET_FIELD_NUM(app_config_set_gain_db_x10, gain_db_x10, int16_t, "gain_dbx10", commit_one_i16)
SET_FIELD_NUM(app_config_set_tagger_threshold_db, tagger_threshold_db, float, "tag_thr", commit_one_f32)
SET_FIELD_NUM(app_config_set_coalesce_min_bursts, coalesce_min_bursts, uint8_t, "coal_n", commit_one_u8)
SET_FIELD_NUM(app_config_set_dcmask_lo, dcmask_lo, int16_t, "dcmask_lo", commit_one_i16)
SET_FIELD_NUM(app_config_set_dcmask_hi, dcmask_hi, int16_t, "dcmask_hi", commit_one_i16)
SET_FIELD_NUM(app_config_set_autotune_gain_dwell_s, autotune_gain_dwell_s, uint32_t, "at_dwell_s", commit_one_u32)
SET_FIELD_NUM(app_config_set_autotune_ira_lo_hz, autotune_ira_lo_hz, uint32_t, "at_ira_hz", commit_one_u32)
SET_FIELD_NUM(app_config_set_autotune_gain_min_dbx10, autotune_gain_min_dbx10, int16_t, "at_g_min", commit_one_i16)
SET_FIELD_NUM(app_config_set_autotune_gain_max_dbx10, autotune_gain_max_dbx10, int16_t, "at_g_max", commit_one_i16)
SET_FIELD_NUM(app_config_set_autotune_gain_stride, autotune_gain_stride, uint8_t, "at_g_strd", commit_one_u8)
SET_FIELD_NUM(app_config_set_autotune_gain_interval_s, autotune_gain_interval_s, uint32_t, "at_g_ivl_s", commit_one_u32)
SET_FIELD_NUM(app_config_set_autotune_lo_interval_s, autotune_lo_interval_s, uint32_t, "at_lo_ivl_s", commit_one_u32)

esp_err_t app_config_set_autotune_on_boot(bool v)
{
    if (!s_cfg_mu) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_cfg_mu, portMAX_DELAY);
    s_cfg.autotune_on_boot = v;
    xSemaphoreGive(s_cfg_mu);
    return commit_one_u8("at_on_boot", (uint8_t)v);
}

esp_err_t app_config_set_gain_mode(gain_mode_t mode)
{
    if (!s_cfg_mu) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_cfg_mu, portMAX_DELAY);
    s_cfg.gain_mode = mode;
    xSemaphoreGive(s_cfg_mu);
    return commit_one_u8("gain_mode", (uint8_t)mode);
}
esp_err_t app_config_set_bias_tee(bool on)
{
    if (!s_cfg_mu) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_cfg_mu, portMAX_DELAY);
    s_cfg.bias_tee = on;
    xSemaphoreGive(s_cfg_mu);
    return commit_one_u8("bias_tee", (uint8_t)on);
}

static esp_err_t set_str_field(char *dst, size_t cap, const char *k, const char *src)
{
    if (!s_cfg_mu || !src) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_cfg_mu, portMAX_DELAY);
    strncpy(dst, src, cap - 1);
    dst[cap - 1] = '\0';
    xSemaphoreGive(s_cfg_mu);
    return commit_one_str(k, src);
}
esp_err_t app_config_set_station_id(const char *id)
{
    return set_str_field(s_cfg.station_id, APP_CONFIG_STATION_ID_LEN, "station", id);
}
esp_err_t app_config_set_wifi_ssid(const char *ssid)
{
    return set_str_field(s_cfg.wifi_ssid, APP_CONFIG_WIFI_SSID_LEN, "wifi_ssid", ssid);
}
esp_err_t app_config_set_wifi_psk(const char *psk)
{
    return set_str_field(s_cfg.wifi_psk, APP_CONFIG_WIFI_PSK_LEN, "wifi_psk", psk);
}
esp_err_t app_config_set_out_host(const char *host)
{
    return set_str_field(s_cfg.out_host, APP_CONFIG_OUT_HOST_LEN, "out_host", host);
}
esp_err_t app_config_set_out_port(uint16_t port)
{
    if (!s_cfg_mu) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_cfg_mu, portMAX_DELAY);
    s_cfg.out_port = port;
    xSemaphoreGive(s_cfg_mu);
    nvs_handle_t h;
    esp_err_t    r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_blob(h, "out_port", &port, sizeof(port));
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}
esp_err_t app_config_set_ota_url(const char *url)
{
    return set_str_field(s_cfg.ota_url, APP_CONFIG_OTA_URL_LEN, "ota_url", url);
}

void app_config_log(void)
{
    if (!s_cfg_mu) return;
    app_config_t c;
    app_config_snapshot(&c);
    static const char *MODES[] = {"TUNER_AGC", "MANUAL", "SOFTWARE_AGC"};
    int                mi      = (int)c.gain_mode;
    if (mi < 0 || mi > 2) mi = 0;
    ESP_LOGI(TAG, "lo=%u Hz  rate=%u Hz", (unsigned)c.lo_freq_hz, (unsigned)c.sample_rate_hz);
    ESP_LOGI(TAG, "gain mode=%s  manual=%.1f dB  bias_tee=%d",
             MODES[mi], c.gain_db_x10 / 10.0f, c.bias_tee);
    ESP_LOGI(TAG, "tagger threshold=%.1f dB  station_id='%s'",
             (double)c.tagger_threshold_db, c.station_id);
    ESP_LOGI(TAG, "gone-burst coalescer: %s (coal_n=%u)",
             c.coalesce_min_bursts >= 2 ? "ENABLED (non-gri heuristic)" : "disabled",
             (unsigned)c.coalesce_min_bursts);
    ESP_LOGI(TAG, "autotune: on_boot=%d gain_interval_s=%lu lo_interval_s=%lu",
             (int)c.autotune_on_boot, (unsigned long)c.autotune_gain_interval_s,
             (unsigned long)c.autotune_lo_interval_s);
    ESP_LOGI(TAG, "wifi_ssid='%s'  wifi_psk=%s",
             c.wifi_ssid, (c.wifi_psk[0] ? "(set)" : "(unset)"));
    if (c.out_host[0] && c.out_port) {
        ESP_LOGI(TAG, "UDP push: %s:%u", c.out_host, (unsigned)c.out_port);
    } else {
        ESP_LOGI(TAG, "UDP push: disabled (out_host/out_port unset)");
    }
    if (c.ota_url[0]) {
        ESP_LOGI(TAG, "OTA URL: %s", c.ota_url);
    } else {
        ESP_LOGI(TAG, "OTA URL: unset (POST /ota will fail until set)");
    }
}
