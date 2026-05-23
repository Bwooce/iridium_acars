// See app_config.h. NVS namespace "iridium" holds individual keys.
// We keep keys flat (not blob-encoded) so a future C6 web UI can
// edit them one at a time without round-tripping the whole struct.

#include "app_config.h"
#include "dsp_processor.h"     // FS_IN_HZ, IRIDIUM_CENTER_FREQ_HZ defaults

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "APP_CFG";
static const char *NVS_NS = "iridium";

// Default values. Conservative for development (TUNER_AGC, no
// bias-tee). Production deployments override via the C6 web UI
// (D17) once that lands.
#define DEFAULT_LO_FREQ_HZ           IRIDIUM_CENTER_FREQ_HZ
#define DEFAULT_SAMPLE_RATE_HZ       FS_IN_HZ
#define DEFAULT_GAIN_MODE            GAIN_MODE_TUNER_AGC
#define DEFAULT_GAIN_DB_X10          350      // 35.0 dB (rec'd for live)
#define DEFAULT_BIAS_TEE             false
#define DEFAULT_TAGGER_THRESHOLD_DB  14.0f
#define DEFAULT_STATION_ID           "p4-iridium-1"
#define DEFAULT_WIFI_SSID            ""
#define DEFAULT_WIFI_PSK             ""

static app_config_t       s_cfg;
static SemaphoreHandle_t  s_cfg_mu = NULL;

// --- NVS helpers ---------------------------------------------------------

static esp_err_t nvs_get_u32_or(nvs_handle_t h, const char *k, uint32_t *v, uint32_t def)
{
    esp_err_t r = nvs_get_u32(h, k, v);
    if (r == ESP_ERR_NVS_NOT_FOUND) { *v = def; return ESP_OK; }
    return r;
}
static esp_err_t nvs_get_i16_or(nvs_handle_t h, const char *k, int16_t *v, int16_t def)
{
    esp_err_t r = nvs_get_i16(h, k, v);
    if (r == ESP_ERR_NVS_NOT_FOUND) { *v = def; return ESP_OK; }
    return r;
}
static esp_err_t nvs_get_u8_or(nvs_handle_t h, const char *k, uint8_t *v, uint8_t def)
{
    esp_err_t r = nvs_get_u8(h, k, v);
    if (r == ESP_ERR_NVS_NOT_FOUND) { *v = def; return ESP_OK; }
    return r;
}
static esp_err_t nvs_get_str_or(nvs_handle_t h, const char *k, char *out, size_t cap,
                                 const char *def)
{
    size_t len = cap;
    esp_err_t r = nvs_get_str(h, k, out, &len);
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
    uint32_t raw;
    esp_err_t r = nvs_get_u32(h, k, &raw);
    if (r == ESP_ERR_NVS_NOT_FOUND) { *v = def; return ESP_OK; }
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
    if (s_cfg_mu) return ESP_OK;     // idempotent

    s_cfg_mu = xSemaphoreCreateMutex();
    if (!s_cfg_mu) return ESP_ERR_NO_MEM;

    // Apply defaults FIRST so the struct is well-defined even if
    // NVS init fails completely.
    s_cfg.lo_freq_hz            = DEFAULT_LO_FREQ_HZ;
    s_cfg.sample_rate_hz        = DEFAULT_SAMPLE_RATE_HZ;
    s_cfg.gain_mode             = DEFAULT_GAIN_MODE;
    s_cfg.gain_db_x10           = DEFAULT_GAIN_DB_X10;
    s_cfg.bias_tee              = DEFAULT_BIAS_TEE;
    s_cfg.tagger_threshold_db   = DEFAULT_TAGGER_THRESHOLD_DB;
    strncpy(s_cfg.station_id, DEFAULT_STATION_ID, APP_CONFIG_STATION_ID_LEN - 1);
    s_cfg.station_id[APP_CONFIG_STATION_ID_LEN - 1] = '\0';
    s_cfg.wifi_ssid[0] = '\0';
    s_cfg.wifi_psk[0]  = '\0';

    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase, performing");
        (void)nvs_flash_erase();
        r = nvs_flash_init();
    }
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "nvs_flash_init failed (%s); using defaults",
                 esp_err_to_name(r));
        return ESP_OK;       // defaults already populated
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
    nvs_get_u32_or(h, "lo_hz",    &s_cfg.lo_freq_hz,     DEFAULT_LO_FREQ_HZ);
    nvs_get_u32_or(h, "rate_hz",  &s_cfg.sample_rate_hz, DEFAULT_SAMPLE_RATE_HZ);
    nvs_get_u8_or (h, "gain_mode",&gm,                   (uint8_t)DEFAULT_GAIN_MODE);
    nvs_get_i16_or(h, "gain_dbx10",&s_cfg.gain_db_x10,   DEFAULT_GAIN_DB_X10);
    nvs_get_u8_or (h, "bias_tee", &bt,                   (uint8_t)DEFAULT_BIAS_TEE);
    nvs_get_f32_or(h, "tag_thr",  &s_cfg.tagger_threshold_db, DEFAULT_TAGGER_THRESHOLD_DB);
    nvs_get_str_or(h, "station",  s_cfg.station_id, APP_CONFIG_STATION_ID_LEN,
                   DEFAULT_STATION_ID);
    nvs_get_str_or(h, "wifi_ssid",s_cfg.wifi_ssid,  APP_CONFIG_WIFI_SSID_LEN, "");
    nvs_get_str_or(h, "wifi_psk", s_cfg.wifi_psk,   APP_CONFIG_WIFI_PSK_LEN,  "");
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
    esp_err_t r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_u32(h, k, v);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}
static esp_err_t commit_one_u8(const char *k, uint8_t v)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_u8(h, k, v);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}
static esp_err_t commit_one_i16(const char *k, int16_t v)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_i16(h, k, v);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}
static esp_err_t commit_one_f32(const char *k, float v)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_f32(h, k, v);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}
static esp_err_t commit_one_str(const char *k, const char *v)
{
    nvs_handle_t h;
    esp_err_t r = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (r != ESP_OK) return r;
    r = nvs_set_str(h, k, v);
    if (r == ESP_OK) r = nvs_commit(h);
    nvs_close(h);
    return r;
}

#define SET_FIELD_NUM(field_setter_name, member, ctype, nvs_key, commit_fn)        \
    esp_err_t field_setter_name(ctype v)                                            \
    {                                                                               \
        if (!s_cfg_mu) return ESP_ERR_INVALID_STATE;                                \
        xSemaphoreTake(s_cfg_mu, portMAX_DELAY);                                    \
        s_cfg.member = v;                                                           \
        xSemaphoreGive(s_cfg_mu);                                                   \
        return commit_fn(nvs_key, v);                                               \
    }

SET_FIELD_NUM(app_config_set_lo_freq_hz,     lo_freq_hz,     uint32_t, "lo_hz",     commit_one_u32)
SET_FIELD_NUM(app_config_set_sample_rate_hz, sample_rate_hz, uint32_t, "rate_hz",   commit_one_u32)
SET_FIELD_NUM(app_config_set_gain_db_x10,    gain_db_x10,    int16_t,  "gain_dbx10",commit_one_i16)
SET_FIELD_NUM(app_config_set_tagger_threshold_db, tagger_threshold_db, float, "tag_thr", commit_one_f32)

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

void app_config_log(void)
{
    if (!s_cfg_mu) return;
    app_config_t c;
    app_config_snapshot(&c);
    static const char *MODES[] = { "TUNER_AGC", "MANUAL", "SOFTWARE_AGC" };
    int mi = (int)c.gain_mode;
    if (mi < 0 || mi > 2) mi = 0;
    ESP_LOGI(TAG, "lo=%u Hz  rate=%u Hz", (unsigned)c.lo_freq_hz, (unsigned)c.sample_rate_hz);
    ESP_LOGI(TAG, "gain mode=%s  manual=%.1f dB  bias_tee=%d",
             MODES[mi], c.gain_db_x10 / 10.0f, c.bias_tee);
    ESP_LOGI(TAG, "tagger threshold=%.1f dB  station_id='%s'",
             (double)c.tagger_threshold_db, c.station_id);
    ESP_LOGI(TAG, "wifi_ssid='%s'  wifi_psk=%s",
             c.wifi_ssid, (c.wifi_psk[0] ? "(set)" : "(unset)"));
}
