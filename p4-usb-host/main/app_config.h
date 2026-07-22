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

#include <stdarg.h> // uart_log_null_vprintf
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define APP_CONFIG_STATION_ID_LEN 32
#define APP_CONFIG_WIFI_SSID_LEN 32
#define APP_CONFIG_WIFI_PSK_LEN 64
#define APP_CONFIG_OUT_HOST_LEN 64     // UDP push target (hostname or IP); empty = disabled
#define APP_CONFIG_IOT_LOG_HOST_LEN 64 // iot_log unicast target (hostname or IP); empty = disabled
#define APP_CONFIG_OTA_URL_LEN 128     // D19 OTA pull URL (http://… or https://…); empty = disabled

typedef enum {
    GAIN_MODE_TUNER_AGC = 0,    // R820T/R828D internal AGC. Default
                                // for indoor/no-antenna development.
    GAIN_MODE_MANUAL = 1,       // Caller picks a fixed gain_db_x10.
                                // Recommended for live Iridium: ~35 dB.
    GAIN_MODE_SOFTWARE_AGC = 2, // D16 software AGC adjusts gain based
                                // on noise-floor / saturation signals.
} gain_mode_t;

// Console UART log mode values (NVS "uart_log", stored as u8). See the
// uart_log field doc below for the full 3-state model.
#define UART_LOG_MODE_OFF 0u  // force console log off, always
#define UART_LOG_MODE_ON 1u   // force console log on, always
#define UART_LOG_MODE_AUTO 2u // effective_on = !network_up (default)

typedef struct {
    // Receive-band soft-switch (VHF/VDL2 foundation). Value is a
    // band_id_t (common/band_pipeline/band_profile.h): 0 = iridium
    // (default), 1 = vdl2. Kept as u8 here so this header stays
    // decoupled from the band component; out-of-range NVS bytes are
    // clamped to iridium at load. Selects the band profile (tagger
    // window/threshold defaults, default LO when lo_hz is unset) and
    // the worker's band_pipeline at boot — reboot to apply. An
    // explicitly-set lo_hz always wins over the band's default LO.
    // NVS key "band".
    uint8_t     band;
    uint32_t    lo_freq_hz;     // RTL-SDR tuner LO frequency
    uint32_t    sample_rate_hz; // RTL-SDR sample rate
    gain_mode_t gain_mode;
    int16_t     gain_db_x10;   // tenths of dB; e.g. 350 = 35.0 dB.
                               // -1 = closest available gain step.
                               // Only used when gain_mode != TUNER_AGC.
    bool  bias_tee;            // RTL-SDR v4 bias tee on/off
    float tagger_threshold_db; // FFT burst tagger SNR threshold

    // Chain-salvage best-effort display (Task B4). Gated OFF by default:
    // when false, ida_salvage_drain() only counts salvage.ok (unchanged
    // behaviour); when true it ALSO pushes a hard-tagged PARTIAL row to
    // the /messages ring (display-only — never acars_push/sd_log). NVS
    // key "best_eff".
    bool best_effort_decode;

    // Chase-2 soft-decision BCH fallback on hard-BCH-failed LW.DA
    // frames (task #16, common/iridium_decoder/ida_chase.c). Default
    // OFF: with the toggle off the decode path is bit-identical to the
    // shipped hard-decision decoder (the fallback is never invoked).
    // ON = A/B measurement mode: expected ~+2% additional CRC-valid
    // LW.DA frames at the validated L=5/cap-256 operating point (host
    // ground-truth run 2026-07-17: +11 on 514 baseline, 0 false
    // accepts). NVS key "chase2"; toggle via POST /chase2?on=0|1
    // (applies live — frame_decoder re-snapshots per hard-fail frame).
    bool chase2_decode;

    // Console UART log mode (NVS "uart_log", u8; default UART_LOG_MODE_AUTO).
    // The console TX path is a VFS busy-spin (topology review 2026-07-17
    // §F1): every ESP_LOGx char spins the calling task until FIFO space,
    // and UART TX drains at baud rate whether or not anything is attached.
    // Three states (UART_LOG_MODE_* above):
    //   OFF  — force console log off (null esp_log vprintf hook),
    //          regardless of network state.
    //   ON   — force console log on (normal vprintf), regardless of
    //          network state.
    //   AUTO — effective_on = !network_up. Telemetry already flows via
    //          iot_log UDP / HTTP (SEPARATE explicit paths, unaffected
    //          either way) once the device has network, so AUTO mutes
    //          the console then to avoid paying for the busy-spin for
    //          nothing; with no network (bench/field, serial cable the
    //          only channel) AUTO logs locally so the console stays
    //          useful. network_up is wifi_link_is_connected()
    //          (wifi_link.h) — Ethernet is NOT included in this check
    //          yet; WiFi via the C6 companion is the only active
    //          transport as of this design, so that's fine for now.
    // AUTO is re-evaluated continuously by status_logger's 1 Hz loop, not
    // just at boot — a live, self-healing gate: WiFi connecting or
    // dropping takes effect within ~1 s, no reboot required. serial_cmd
    // RX + its uart_puts replies still work in every mode (direct UART,
    // not ESP_LOG), and panic/ROM output is unaffected — serial recovery
    // stays possible regardless of mode. Toggle via POST /uartlog?on=0|1
    // or ?auto=1 (applies live) or serial `set uart_log 0|1|2` (persists
    // + applies live). Values 0/1 are backward compatible with the old
    // bool NVS storage; new installs default to 2 (AUTO).
    uint8_t uart_log;

    // P1.5 companion heuristic (NON-GRI; gr-iridium has no equivalent):
    // same-instant multi-bin gone-burst coalescing in dsp_processor.
    // One band-wide impulse tags MANY narrow bursts with near-identical
    // start times; when >= this many gone-bursts in one tagger step
    // share a start within one FFT step (2048 samples), only the
    // strongest is dispatched to the worker and the rest are dropped
    // (counted in the fbt: coal= diagnostic). 0 (default) or 1 =
    // DISABLED — burst-for-burst identical to gr-iridium's dispatch.
    // Applied at detector create; reboot (or scanner re-create) to
    // change. NVS key "coal_n".
    uint8_t coalesce_min_bursts;

    // Near-DC tagger exclusion window (near-DC tagger-mask work). Signed
    // FFT-bin offsets from DC; lo>hi = disabled. Applied at detector create
    // and live-settable. NVS keys "dcmask_lo"/"dcmask_hi".
    int16_t dcmask_lo;
    int16_t dcmask_hi;

    // Autotune RF-recalibration knobs (2026-07-08 autotune design). NVS
    // keys "at_dwell_s"/"at_ira_hz"/"at_g_min"/"at_g_max"/"at_g_strd".
    // Consumed by the manual `autotune` serial command; not hot-path.
    uint32_t autotune_gain_dwell_s;   // per-gain dwell during calibration (>=55 s
                                      // is too noisy per the design; default 180)
    uint32_t autotune_ira_lo_hz;      // IRA reference LO for gain calibration
    int16_t  autotune_gain_min_dbx10; // lower bound of the swept gain range (tenths dB)
    int16_t  autotune_gain_max_dbx10; // upper bound of the swept gain range (tenths dB)
    uint8_t  autotune_gain_stride;    // coarse sweep: sample every Nth R828D step

    // Boot-time + periodic auto-run (2026-07-08 boot/periodic extension).
    // NVS keys "at_on_boot"/"at_g_ivl_s"/"at_lo_ivl_s". Consumed by
    // autotune_sched.c; 0 for either interval = that clock disabled.
    bool autotune_on_boot;             // run one calibration pass after the
                                       // stream comes up stable at boot. Default
                                       // FALSE — dev/smoke/normal reboots
                                       // shouldn't eat a multi-minute cal.
    uint32_t autotune_gain_interval_s; // periodic IRA gain re-cal period (s);
                                       // RFI/thermal-driven, slow. Default 3600.
    uint32_t autotune_lo_interval_s;   // periodic LO density re-scan period (s);
                                       // SLOW center-tracking only — best-LO is
                                       // mean-reverting (empirical finding), NOT
                                       // momentum, so this must NOT be short
                                       // (do not default 600). Default 3600.

    // Band-health auto re-survey opt-in (band_health.c). Default FALSE
    // (park-don't-steer): the staleness detector always tracks and logs, but
    // only acts on the RF — one integrated multi-sweep survey + live re-park,
    // never an NVS LO write — when this is set. NVS key "bh_auto".
    bool band_resurvey_auto;

    char     station_id[APP_CONFIG_STATION_ID_LEN];     // for upstream/log identification
    char     wifi_ssid[APP_CONFIG_WIFI_SSID_LEN];       // for D17 C6 wireless
    char     wifi_psk[APP_CONFIG_WIFI_PSK_LEN];         // for D17 C6 wireless
    char     out_host[APP_CONFIG_OUT_HOST_LEN];         // UDP push target host; empty = no push
    uint16_t out_port;                                  // UDP push target port; 0 = no push
    char     iot_log_host[APP_CONFIG_IOT_LOG_HOST_LEN]; // iot_log unicast target host; empty = disabled
    char     ota_url[APP_CONFIG_OTA_URL_LEN];           // D19 OTA pull URL; empty = disabled
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
void                app_config_unlock(void);

// Setters. Each writes the struct field AND commits to NVS.
// Returns ESP_OK on NVS success; the in-memory struct is updated
// regardless so subsequent reads see the new value even if NVS
// commit fails.
// v is a band_id_t value; out-of-range clamps to 0 (iridium), same as
// the load-time guard. Takes effect on next boot (profile + pipeline
// are resolved once at create/init time).
esp_err_t app_config_set_band(uint8_t v);

esp_err_t app_config_set_lo_freq_hz(uint32_t hz);
esp_err_t app_config_set_sample_rate_hz(uint32_t hz);
esp_err_t app_config_set_gain_mode(gain_mode_t mode);
esp_err_t app_config_set_gain_db_x10(int16_t v);
esp_err_t app_config_set_bias_tee(bool on);
esp_err_t app_config_set_best_effort_decode(bool on);

// mode is UART_LOG_MODE_OFF/ON/AUTO (0/1/2); out-of-range values are
// clamped to AUTO. Persists AND updates the in-memory struct; callers
// that also want the change to take effect immediately should combine
// this with uart_log_apply()/app_config_uart_log_effective() (see
// http_server.c uartlog_post / serial_cmd.c cmd_set for the pattern).
esp_err_t app_config_set_uart_log(uint8_t mode);

// Read the persisted/live uart_log mode (0/1/2) without a full snapshot.
uint8_t app_config_get_uart_log_mode(void);

// Pure helper (no deps): map (mode, network_up) -> effective on/off.
// mode: OFF=false always, ON=true always, else (AUTO, or any
// out-of-range value) = !network_up.
bool app_config_uart_log_effective(uint8_t mode, bool network_up);

esp_err_t app_config_set_chase2_decode(bool on);
// RAM-only apply of the live chase2 flag: instant, flash-free, safe from ANY
// task (incl. the PSRAM-stack httpd task, which must never nvs_commit). Lets a
// handler apply + honestly report the live state, then hand the NVS persist to
// an internal-stack task. Does NOT persist across reboot on its own.
void app_config_set_chase2_decode_ram(bool on);

// The uart_log null-sink vprintf hook, and a live apply helper (installs the
// null hook or restores the default UART vprintf; safe from any task —
// esp_log_set_vprintf is a pointer swap). Persisting is separate
// (app_config_set_uart_log); boot applies the effective value in app_main,
// and status_logger's 1 Hz loop re-applies it continuously for AUTO mode.
int  uart_log_null_vprintf(const char *fmt, va_list ap);
void uart_log_apply(bool on);
esp_err_t app_config_set_tagger_threshold_db(float db);
esp_err_t app_config_set_coalesce_min_bursts(uint8_t n);
esp_err_t app_config_set_dcmask_lo(int16_t v);
esp_err_t app_config_set_dcmask_hi(int16_t v);
esp_err_t app_config_set_autotune_gain_dwell_s(uint32_t v);
esp_err_t app_config_set_autotune_ira_lo_hz(uint32_t v);
esp_err_t app_config_set_autotune_gain_min_dbx10(int16_t v);
esp_err_t app_config_set_autotune_gain_max_dbx10(int16_t v);
esp_err_t app_config_set_autotune_gain_stride(uint8_t v);
esp_err_t app_config_set_autotune_on_boot(bool v);
esp_err_t app_config_set_autotune_gain_interval_s(uint32_t v);
esp_err_t app_config_set_autotune_lo_interval_s(uint32_t v);
esp_err_t app_config_set_band_resurvey_auto(bool v);
esp_err_t app_config_set_station_id(const char *id);
esp_err_t app_config_set_wifi_ssid(const char *ssid);
esp_err_t app_config_set_wifi_psk(const char *psk);
esp_err_t app_config_set_out_host(const char *host);
esp_err_t app_config_set_out_port(uint16_t port);
esp_err_t app_config_set_iot_log_host(const char *host);
esp_err_t app_config_set_ota_url(const char *url);

// Log the current config (info-level). Useful at boot for diagnostics.
void app_config_log(void);
