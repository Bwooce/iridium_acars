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

#endif
