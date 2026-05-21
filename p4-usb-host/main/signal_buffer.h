#ifndef SIGNAL_BUFFER_H
#define SIGNAL_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// 4MB Circular buffer = ~400ms at 2.56 MSPS SC16
#define SIGNAL_BUF_SIZE (4 * 1024 * 1024)

esp_err_t signal_buffer_init();
void signal_buffer_push(const int16_t *samples, size_t n_samples);
void signal_buffer_extract(uint32_t start_idx, uint32_t length, int16_t *dest);

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

#endif
