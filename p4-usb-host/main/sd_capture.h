#pragma once

#include "esp_err.h"
#include "sdkconfig.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// On-demand raw IQ capture to SD card.
//
// Records the RTL-SDR's raw uint8 stream (interleaved I,Q at offset
// 127.5 — the format coming over USB before any conversion) to a
// file on the SD card. Started/stopped via HTTP, not enabled by
// default. Useful for offline DSP debugging (replay through
// gr-iridium or our host test fixtures).
//
// Architecture:
//   - producer: class_driver (Core 0) calls sd_capture_write() right
//     after each USB transfer is read. Non-blocking; copies bytes
//     into a PSRAM stream buffer and drops on overflow.
//   - writer task on Core 0 (low priority) drains the stream buffer
//     and fwrites to FATFS. Periodic 1 s fflush.
//
// Data rate: at the current 3.95 MB/s sustained USB rate, SDMMC
// has 3-7x headroom for the writes. The capture path is the same
// uint8 stream the firmware would otherwise convert to int16 and
// resample, so a file replayed through the RAW_IRIDIUM smoke
// fixture (or any uint8-aware tool) should reproduce identical
// decode behaviour.
//
// Compile-time gate: shares CONFIG_ENABLE_SD_LOG with the ACARS
// NDJSON log. When OFF, all public functions become inline no-ops.

typedef struct {
    bool     active;             // capture currently running
    bool     file_open;
    uint64_t bytes_captured;     // bytes written to file
    uint64_t bytes_target;       // 0 = unlimited
    uint32_t bytes_dropped;      // bytes that didn't fit in stream buffer
    uint32_t write_errors;
    char     path[64];           // empty until started
    int64_t  start_us;           // boot-relative
} sd_capture_stats_t;

#if CONFIG_ENABLE_SD_LOG

// One-shot at boot: spawns the writer task. Does NOT mount the
// SD card or allocate the stream buffer — both happen on
// sd_capture_start(). Idempotent.
esp_err_t sd_capture_init(void);

// Begin capture. `target_bytes` = stop after this many bytes (0
// = unlimited; user must POST /capture/stop). Triggers a lazy SD
// mount if not already mounted. Returns ESP_ERR_INVALID_STATE if
// a capture is already active.
esp_err_t sd_capture_start(uint64_t target_bytes);

// Flush + close the current file. Returns ESP_ERR_INVALID_STATE
// if no capture is active.
esp_err_t sd_capture_stop(void);

// Producer entry point — class_driver calls this with each USB
// transfer's payload. Fast no-op if no capture is active.
void sd_capture_write(const uint8_t *data, size_t n);

void sd_capture_get_stats(sd_capture_stats_t *out);

#else  /* !CONFIG_ENABLE_SD_LOG — provide inline no-op stubs. */

static inline esp_err_t sd_capture_init(void) { return ESP_OK; }
static inline esp_err_t sd_capture_start(uint64_t target_bytes)
{
    (void)target_bytes; return ESP_ERR_NOT_SUPPORTED;
}
static inline esp_err_t sd_capture_stop(void) { return ESP_ERR_NOT_SUPPORTED; }
static inline void sd_capture_write(const uint8_t *data, size_t n)
{
    (void)data; (void)n;
}
static inline void sd_capture_get_stats(sd_capture_stats_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    snprintf(out->path, sizeof(out->path), "disabled (CONFIG_ENABLE_SD_LOG=n)");
}

#endif  /* CONFIG_ENABLE_SD_LOG */
