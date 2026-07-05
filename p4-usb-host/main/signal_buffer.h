#ifndef SIGNAL_BUFFER_H
#define SIGNAL_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// 4MB Circular buffer = ~400ms at 2.56 MSPS SC16
#define SIGNAL_BUF_SIZE (4 * 1024 * 1024)

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
