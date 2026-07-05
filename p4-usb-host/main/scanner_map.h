#pragma once
#include <stdint.h>

#define SCANNER_MAX_POSITIONS 16

typedef struct {
    uint32_t center_hz;
    uint32_t narrowband_bursts;
    uint32_t all_bursts;
    float    mean_snr_db;
    uint32_t dwell_ms;
} scanner_pos_t;

int   scanner_enumerate_centers(uint32_t start_hz, uint32_t stop_hz,
                                uint32_t step_hz, uint32_t *out, int max);
float scanner_pos_narrowband_rate(const scanner_pos_t *p);
int   scanner_rank_hottest(const scanner_pos_t *pos, int n);
