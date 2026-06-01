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
    bool     active; // capture currently running
    bool     file_open;
    uint64_t bytes_captured; // bytes written to file
    uint64_t bytes_target;   // 0 = unlimited
    uint32_t bytes_dropped;  // bytes that didn't fit in stream buffer
    uint32_t write_errors;
    char     path[64]; // empty until started
    int64_t  start_us; // boot-relative
} sd_capture_stats_t;

#if CONFIG_ENABLE_SD_LOG

// One-shot at boot: spawns the writer task. Does NOT mount the
// SD card or allocate the stream buffer — both happen on
// sd_capture_start(). Does NOT allocate the writer's DMA-INT fwrite
// buf either — see sd_capture_alloc_writer_buf below. Idempotent.
esp_err_t sd_capture_init(void);

// Allocate the 64 KB DMA-INT fwrite scratch the writer task uses.
// Must be called AFTER the tagger has taken its ~66 KB contiguous
// DMA-INT block (action_start_stream → dsp_processor_init) but
// BEFORE USB transfer pool churn starts fragmenting the heap
// (esp_libusb_start_stream). Idempotent. Returns ESP_ERR_NO_MEM
// if DMA-INT no longer has a 64 KB contiguous free block.
esp_err_t sd_capture_alloc_writer_buf(void);

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

// Open a previously-captured file under /sdcard/acars/ for reading.
// `name` is the bare filename (no path). Returns a FILE* the caller
// must fclose, or NULL on error. Used by the HTTP /capture/file
// download handler to stream the .u8 file back over the wire.
// Forbids ".." anywhere in the name as a minimal traversal guard.
FILE *sd_capture_open_for_read(const char *name);

// Burst-mode capture: instead of streaming continuous USB bytes,
// record just the per-burst IQ windows the worker_core1 task
// extracts from signal_buffer. At ~0.5-10 bursts/sec real rate +
// ~40-160 KB per burst record, easily lossless on any SD card
// (no contention with the 4.5 MB/s USB ingest path).
//
// File format: sequence of binary records, each:
//   40 B header (struct sd_capture_burst_hdr_t below, little-endian)
//   length_samples * 4 bytes IQ (interleaved int16, 2.5 MSPS)
//
// Magic "BRST" lets a parser resync if the file is truncated mid-
// record. Header size is fixed so a parser doesn't need TLV logic.

#define SD_CAPTURE_BURST_MAGIC 0x54535242u /* "BRST" little-endian */

typedef struct __attribute__((packed)) {
    uint32_t magic;          /* SD_CAPTURE_BURST_MAGIC */
    uint32_t seq;            /* monotonic per-capture, starts at 0 */
    uint64_t t_us;           /* esp_timer_get_time at burst-begin */
    uint32_t length_samples; /* complex samples that follow (×4 bytes) */
    float    rel_freq_hz;    /* tagger-reported offset from LO */
    float    peak_snr_db;
    float    magnitude_db;
    float    noise_db;
    uint32_t reserved; /* zero; pads to 40 bytes for future fields */
} sd_capture_burst_hdr_t;
_Static_assert(sizeof(sd_capture_burst_hdr_t) == 40,
               "burst header layout must be 40 bytes (file format)");

// Called by sd_capture_start when mode == burst. Sets the writer
// into burst-mode (continuous bytes from sd_capture_write are
// ignored; only burst records flow). Caller usually invokes
// sd_capture_start_bursts() rather than the underlying mode flag.
esp_err_t sd_capture_start_bursts(void);

// Per-burst record. _begin writes the header to the stream buffer;
// _chunk appends raw IQ; _end signals "burst complete" (currently
// a no-op since header.length_samples + the stream buffer model
// make burst boundaries implicit, but reserved for future framing).
//
// All three are non-blocking — if the stream buffer can't absorb
// the data (writer back-pressured), bytes_dropped ticks up and the
// record is incomplete. The "BRST" magic prefix means downstream
// parsers can resync on the next valid header.
//
// Worker MUST call _begin before any _chunk for a given burst,
// and _end after the final chunk. Caller is one task (worker_core1)
// so no concurrency between burst records.
void sd_capture_record_burst_begin(uint32_t length_samples,
                                   float    rel_freq_hz,
                                   float    peak_snr_db,
                                   float    magnitude_db,
                                   float    noise_db);
void sd_capture_record_burst_chunk(const int16_t *iq, size_t n_complex);
void sd_capture_record_burst_end(void);

#else /* !CONFIG_ENABLE_SD_LOG — provide inline no-op stubs. */

static inline esp_err_t sd_capture_init(void)
{
    return ESP_OK;
}
static inline esp_err_t sd_capture_alloc_writer_buf(void)
{
    return ESP_OK;
}
static inline esp_err_t sd_capture_start(uint64_t target_bytes)
{
    (void)target_bytes;
    return ESP_ERR_NOT_SUPPORTED;
}
static inline esp_err_t sd_capture_stop(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}
static inline void sd_capture_write(const uint8_t *data, size_t n)
{
    (void)data;
    (void)n;
}
static inline void sd_capture_get_stats(sd_capture_stats_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    snprintf(out->path, sizeof(out->path), "disabled (CONFIG_ENABLE_SD_LOG=n)");
}
static inline esp_err_t sd_capture_start_bursts(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}
static inline void sd_capture_record_burst_begin(uint32_t length_samples,
                                                 float    rel_freq_hz,
                                                 float    peak_snr_db,
                                                 float    magnitude_db,
                                                 float    noise_db)
{
    (void)length_samples;
    (void)rel_freq_hz;
    (void)peak_snr_db;
    (void)magnitude_db;
    (void)noise_db;
}
static inline void sd_capture_record_burst_chunk(const int16_t *iq, size_t n_complex)
{
    (void)iq;
    (void)n_complex;
}
static inline void sd_capture_record_burst_end(void)
{
}

#endif /* CONFIG_ENABLE_SD_LOG */
