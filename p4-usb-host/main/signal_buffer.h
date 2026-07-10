#ifndef SIGNAL_BUFFER_H
#define SIGNAL_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// 16MB circular buffer. The ring stores int8 IQ (2 bytes/complex): the RTL
// ADC is 8-bit and ingest's uint8->int16 conversion is exactly (byte-128)<<8,
// so the low 8 bits of every stored int16 are always zero. Storing int8 =
// int16>>8 and re-expanding int16 = int8<<8 at read time is bit-exact
// lossless, so 16 MB holds ~3.36 s at 2.5 MSPS (int8, 2 B/complex).
// The detector runs behind live ingest under high signal load, so a larger
// freshness window lets bursts survive the detector lag long enough to decode.
//
// CANNOT be grown (measured 2026-07-10, the "grow the ring" hedge from the
// Option-B scoping): both 20 MB and 24 MB OOM the 4 MB usbring allocated after
// this (`LIBUSB: Failed to create stream ring in PSRAM: ESP_ERR_NO_MEM`) and
// kill the stream. PSRAM is at its ceiling here — 16 MB signal + 4 MB usbring +
// ~12 MB task/http PSRAM stacks ≈ full 32 MB. So a bigger stale-drop horizon is
// NOT available on this board; reducing per-burst demod cost is the only
// structural compute lever left (see project_worker_compute_profile_2026_07_10).
#define SIGNAL_BUF_SIZE (16 * 1024 * 1024)

// Ring capacity in COMPLEX samples — the SINGLE source of truth. The ring
// stores int8 IQ = 2 bytes/complex, so capacity = SIGNAL_BUF_SIZE/2. EVERY
// stale/ring-span/lap check (signal_buffer.c's total_cap AND worker_core1.c's
// pre-reject + lag log) MUST derive from this, never a hardcoded /N: when the
// element width last changed (int16->int8) the worker's private SIGNAL_BUF_SIZE/4
// copies silently kept the old half-size capacity and rejected still-valid
// bursts, defeating the larger window. Centralised so that can't recur.
#define SIGNAL_BUF_CAPACITY_COMPLEX (SIGNAL_BUF_SIZE / 2)

esp_err_t signal_buffer_init();
void      signal_buffer_push(const int16_t *samples, size_t n_samples);

// Invalidate L2 cache lines covering [start_idx, start_idx + length) of the
// circular buffer (modulo wrap). Call once per burst before reading chunks
// via signal_buffer_read_chunk() so the DMA-written data from Core 0 is
// visible on Core 1. Cost ~1 ms for a 2.5 MB window.
void signal_buffer_invalidate_range(uint32_t start_idx, uint32_t length);

// Read `length` complex samples from circular_buf into `dest` (typically
// internal-SRAM scratch). Handles wrap. Caller must have already
// invalidated the L2 lines for the burst's full range -- no msync here.
// Skips the PSRAM intermediate s_extract_buf, saving ~3 ms/burst of
// PSRAM write traffic when the chunked decim loop reads directly.
void signal_buffer_read_chunk(uint32_t start_idx, uint32_t length, int16_t *dest);

// Current write position (in complex samples). Useful for callers that
// need to gauge how much capacity remains before a queued burst's
// window gets overwritten.
uint32_t signal_buffer_head(void);

// 64-bit monotonic cumulative complex-sample count committed to the ring
// (T44's absolute clock). Used to measure how far a burst's start index lags
// the producer (staleness magnitude) and to derive capture timestamps.
uint64_t signal_buffer_head_total(void);

// Wall-clock (esp_timer µs) of cumulative sample index 0. Convert a burst
// start index to its capture time as: stream_epoch_us + idx * 1e6 / rate.
// Returns 0 until the first push has occurred.
uint64_t signal_buffer_stream_epoch_us(void);

// Recovery-counter accessors (surfaced in /diag/recovery_counters and
// in the STATUS-ERR log line when nonzero).
//
//   stash_alloc_fails      — IDF dma_utils "no mem for stash buffer"
//                            events. esp_async_memcpy returns
//                            ESP_ERR_NO_MEM; we count + try to
//                            recover. This is the same event the
//                            LOG_VERSION_2 silencing kept out of the
//                            UART log; the counter is the visible
//                            replacement for the lost log lines.
//   stash_alloc_recoveries — simple-path failures that recovered via
//                            CPU memcpy (#126E). Audio-dropped =
//                            stash_alloc_fails - stash_alloc_recoveries
//                            (wrap-path failures fall through to drop).
//   dma_timeouts           — s_dma_done wait timeout (a previous DMA's
//                            give never came). Indicates upstream
//                            wedge; has never fired in production.
uint32_t signal_buffer_stash_alloc_fails(void);
uint32_t signal_buffer_stash_alloc_recoveries(void);
uint32_t signal_buffer_dma_timeouts(void);

// True iff [start_idx, start_idx + length) of the circular buffer still
// holds the original-pushed data — i.e. the producer hasn't yet lapped
// onto the burst window. Returns false when the burst is stale and
// would extract garbage; the worker uses this to skip rather than
// decode noise. `start_idx` is the tagger's 64-bit cumulative complex-
// sample index (T44) so the check disambiguates full ring laps rather
// than only the most-recent lap.
bool signal_buffer_burst_valid(uint64_t start_idx, uint32_t length);

#endif
