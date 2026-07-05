#ifndef SIGNAL_BUFFER_H
#define SIGNAL_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// 16MB circular buffer = ~1.6s at 2.56 MSPS SC16 (T59 experiment): the
// detector runs behind live ingest under high signal load, so a 4MB/~400ms
// ring made every burst stale (ring-lapped) before the worker could read it.
// A larger freshness window lets bursts survive the detector lag long enough
// to decode. 16 MB fits the 32 MB PSRAM alongside the 4 MB tagger baseline.
// Multiple of 64 (wrap-path DMA alignment) and /4 is a multiple of 16.
#define SIGNAL_BUF_SIZE (16 * 1024 * 1024)

esp_err_t signal_buffer_init();
void      signal_buffer_push(const int16_t *samples, size_t n_samples);
void      signal_buffer_extract(uint32_t start_idx, uint32_t length, int16_t *dest);

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
