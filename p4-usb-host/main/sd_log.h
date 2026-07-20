#pragma once

#include "esp_err.h"
#include "msg_ring.h"
#include "sdkconfig.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
// Compile-time gate: CONFIG_ENABLE_SD_LOG. When the flag is OFF
// (default), the whole subsystem (SDMMC driver + FATFS + this
// module) is excluded from the build and all public functions
// become inline no-ops. Linking SDMMC + FATFS costs ~50 KB of
// internal-SRAM static reservations that compete with USB host
// DMA pool and break live-SDR throughput; opt-in only.

// Snapshot stats for /status surfacing (struct always available so
// callers compile in either build).
typedef struct {
    bool     mounted;
    bool     log_open;
    uint32_t messages_written;
    uint32_t write_errors;
    uint64_t bytes_written;
    char     log_path[64];    // empty if no log open
    char     mount_error[64]; // empty on success
} sd_log_stats_t;

#if CONFIG_ENABLE_SD_LOG

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

// Unmount, wipe + reformat the card to FAT32, remount. Destroys all
// data on the card. Used to recover from "ENOSPC even though the
// card has space" after accumulated test captures fill the volume.
// May take 30-180 s depending on card size — caller (HTTP handler)
// should not hold any locks during the call.
esp_err_t sd_log_force_format(void);

void sd_log_get_stats(sd_log_stats_t *out);

// Auto-fallback: drop the SD bus clock to 20 MHz for the next mount (after
// sustained write failure at 40 MHz HS, or a mount failure). In-RAM; resets to
// 40 MHz on reboot. Called by the capture writer's circuit-breaker.
void sd_log_downclock(void);

#else /* !CONFIG_ENABLE_SD_LOG — provide inline no-op stubs. */

static inline esp_err_t sd_log_init(void)
{
    return ESP_OK;
}
static inline void sd_log_emit(const acars_msg_t *m)
{
    (void)m;
}
static inline esp_err_t sd_log_force_mount(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}
static inline esp_err_t sd_log_force_format(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}
static inline void sd_log_downclock(void)
{
}
static inline void sd_log_get_stats(sd_log_stats_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    // "disabled-in-build" marker doubles as the /status hint that
    // SDMMC isn't even linked into this firmware image.
    snprintf(out->mount_error, sizeof(out->mount_error),
             "disabled (CONFIG_ENABLE_SD_LOG=n)");
}

#endif /* CONFIG_ENABLE_SD_LOG */
