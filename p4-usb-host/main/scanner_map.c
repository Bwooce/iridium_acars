#include "scanner_map.h"

int scanner_enumerate_centers(uint32_t start_hz, uint32_t stop_hz,
                              uint32_t step_hz, uint32_t *out, int max)
{
    if (step_hz == 0 || start_hz > stop_hz || max <= 0) return 0;
    int n = 0;
    for (uint32_t f = start_hz; f <= stop_hz && n < max; f += step_hz)
        out[n++] = f;
    return n;
}

float scanner_pos_narrowband_rate(const scanner_pos_t *p)
{
    if (!p || p->dwell_ms == 0) return 0.0f;
    return (float)p->narrowband_bursts * 1000.0f / (float)p->dwell_ms;
}

int scanner_rank_hottest(const scanner_pos_t *pos, int n)
{
    if (!pos || n <= 0) return -1;
    int   best      = 0;
    float best_rate = scanner_pos_narrowband_rate(&pos[0]);
    for (int i = 1; i < n; i++) {
        float r = scanner_pos_narrowband_rate(&pos[i]);
        if (r > best_rate) {
            best_rate = r;
            best      = i;
        }
    }
    return best;
}
