// NVS-backed runtime configuration (D18).
//
// Holds all settings that should persist across reboots: SDR
// parameters (LO, sample rate, gain mode/value, bias tee), tagger
// threshold, station identification, Wi-Fi credentials (for C6
// companion in D17).
//
// Lifecycle: app_config_init() at boot loads from NVS, applying
// compile-time defaults for missing keys. The in-memory struct is
// then read via app_config_get(). Updates go through the
// app_config_set_*() setters which call nvs_commit() so values
// persist across the next boot.
//
// Thread-safety: a single mutex serializes read+write. Hot-path
// readers should snapshot the struct under app_config_lock() /
// app_config_unlock() rather than holding the lock across DSP work.

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define APP_CONFIG_STATION_ID_LEN  32
#define APP_CONFIG_WIFI_SSID_LEN   32
#define APP_CONFIG_WIFI_PSK_LEN    64

typedef enum {
    GAIN_MODE_TUNER_AGC      = 0,   // R820T/R828D internal AGC. Default
                                    // for indoor/no-antenna development.
    GAIN_MODE_MANUAL         = 1,   // Caller picks a fixed gain_db_x10.
                                    // Recommended for live Iridium: ~35 dB.
    GAIN_MODE_SOFTWARE_AGC   = 2,   // D16 software AGC adjusts gain based
                                    // on noise-floor / saturation signals.
} gain_mode_t;

typedef struct {
    uint32_t   lo_freq_hz;            // RTL-SDR tuner LO frequency
    uint32_t   sample_rate_hz;        // RTL-SDR sample rate
    gain_mode_t gain_mode;
    int16_t    gain_db_x10;           // tenths of dB; e.g. 350 = 35.0 dB.
                                      // -1 = closest available gain step.
                                      // Only used when gain_mode != TUNER_AGC.
    bool       bias_tee;              // RTL-SDR v4 bias tee on/off
    float      tagger_threshold_db;   // FFT burst tagger SNR threshold

    char       station_id[APP_CONFIG_STATION_ID_LEN];   // for upstream/log identification
    char       wifi_ssid[APP_CONFIG_WIFI_SSID_LEN];     // for D17 C6 wireless
    char       wifi_psk[APP_CONFIG_WIFI_PSK_LEN];       // for D17 C6 wireless
} app_config_t;

// Initialise from NVS. Missing keys get compile-time defaults.
// Idempotent. Returns ESP_OK on success (defaults loaded even if
// NVS init fails — the system stays operational with defaults).
esp_err_t app_config_init(void);

// Snapshot the live config. Copies struct into *out under the
// internal lock; caller is then free to read without contention.
void app_config_snapshot(app_config_t *out);

// Lock/unlock helpers for hot-path readers that want to inspect
// a single field without copying the whole struct. Keep critical
// sections SHORT — under 1 microsecond.
const app_config_t *app_config_get_locked(void);
void app_config_unlock(void);

// Setters. Each writes the struct field AND commits to NVS.
// Returns ESP_OK on NVS success; the in-memory struct is updated
// regardless so subsequent reads see the new value even if NVS
// commit fails.
esp_err_t app_config_set_lo_freq_hz(uint32_t hz);
esp_err_t app_config_set_sample_rate_hz(uint32_t hz);
esp_err_t app_config_set_gain_mode(gain_mode_t mode);
esp_err_t app_config_set_gain_db_x10(int16_t v);
esp_err_t app_config_set_bias_tee(bool on);
esp_err_t app_config_set_tagger_threshold_db(float db);
esp_err_t app_config_set_station_id(const char *id);
esp_err_t app_config_set_wifi_ssid(const char *ssid);
esp_err_t app_config_set_wifi_psk(const char *psk);

// Log the current config (info-level). Useful at boot for diagnostics.
void app_config_log(void);
