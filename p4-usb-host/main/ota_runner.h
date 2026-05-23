#pragma once

#include "esp_err.h"
#include <stdbool.h>

// D19 — pull-style OTA over Wi-Fi (via the C6 esp_hosted slave).
//
// Reads `ota_url` from NVS (set via /config or app_config_set_ota_url),
// downloads the image via esp_https_ota, stages into the inactive
// app partition, marks the new partition for boot, and reboots. The
// IDF bootloader handles two-slot rollback: if the new firmware
// doesn't call esp_ota_mark_app_valid_cancel_rollback() within its
// rollback window (we do this from app_main shortly after boot once
// the core services are confirmed healthy), the bootloader reverts
// on the next reset.
//
// Concurrency: only one OTA can be in flight. ota_runner_start()
// returns ESP_ERR_INVALID_STATE if another is running.

// Trigger an OTA pull. Non-blocking — spawns a worker task that
// performs the HTTP fetch + flash write off the calling thread.
// Errors during fetch are logged but the running firmware keeps
// going (no reboot on failure). Use ota_runner_get_status() to
// poll progress from the HTTP /ota handler if desired.
esp_err_t ota_runner_start(void);

typedef enum {
    OTA_IDLE = 0,
    OTA_RUNNING,
    OTA_SUCCESS,        // staged + flipped; reboot pending
    OTA_FAILED,
} ota_state_t;

typedef struct {
    ota_state_t state;
    int         http_status;     // last HTTP status code, or 0 if pre-request
    int         bytes_written;   // total bytes flashed
    char        last_error[128]; // human-readable; empty on success
} ota_status_t;

void ota_runner_get_status(ota_status_t *out);

// Call once early in app_main, after the core services have come up.
// Marks the current firmware as "valid" so the bootloader stops
// counting boots against the rollback window. Safe to call any
// number of times; only the first matters.
void ota_runner_mark_valid(void);
