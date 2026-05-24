#pragma once

#include "esp_err.h"
#include "msg_ring.h"

#include <stdbool.h>
#include <stdint.h>

// SD card log writer for decoded ACARS messages.
//
// Mounts /sdcard on boot (SDMMC 4-bit, Slot 0). If no card is present
// or the mount fails, all subsequent calls are silent no-ops — boot
// continues normally without persistent logging.
//
// On successful mount, opens /sdcard/acars/log-<boot_ts>.ndjson (one
// file per boot) and appends one NDJSON line per acars_msg_t. Same
// schema as GET /messages and the UDP push, so any consumer can read
// the file directly.
//
// Capacity is generous (a busy environment is ~few msg/sec × few
// hundred bytes = a few KB/s, well under any SD card's sustained
// write rate). One flush every ~1 s keeps data durable without
// hammering the FATFS layer.

// One-shot at boot: spawns the writer task + queue. Does NOT mount the
// card. The first acars_msg_t through sd_log_emit() triggers a
// lazy mount in the writer task — saves ~25 KB internal SRAM on
// no-card boots and on the gap between boot and the first decode.
esp_err_t sd_log_init(void);

// Append a decoded ACARS message to the open log file. Non-blocking
// (writes through a small queue to a low-prio writer task). Safe to
// call when SD is absent or unmounted — drops silently.
void sd_log_emit(const acars_msg_t *m);

// Force a mount attempt now (e.g. after the user inserts a card and
// hits POST /sd/mount). Returns the mount result. Idempotent: returns
// ESP_OK if already mounted.
esp_err_t sd_log_force_mount(void);

// Snapshot stats for /status surfacing.
typedef struct {
    bool     mounted;
    bool     log_open;
    uint32_t messages_written;
    uint32_t write_errors;
    uint64_t bytes_written;
    char     log_path[64];        // empty if no log open
    char     mount_error[64];     // empty on success
} sd_log_stats_t;

void sd_log_get_stats(sd_log_stats_t *out);
