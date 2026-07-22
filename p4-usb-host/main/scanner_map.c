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

void scanner_pos_accumulate(scanner_pos_t *acc, const scanner_pos_t *p)
{
    if (!acc || !p) return;
    // Burst-weighted running mean: sweeps land on different satellite beams,
    // so per-sweep burst counts differ wildly — weight SNR by how much each
    // sweep actually heard rather than averaging sweep means equally.
    uint32_t total = acc->all_bursts + p->all_bursts;
    if (p->all_bursts > 0)
        acc->mean_snr_db = (acc->mean_snr_db * (float)acc->all_bursts +
                            p->mean_snr_db * (float)p->all_bursts) /
                           (float)total;
    acc->all_bursts = total;
    acc->narrowband_bursts += p->narrowband_bursts;
    acc->dwell_ms += p->dwell_ms; // summed dwell => rate() = mean over sweeps
}

// Rank-only IRA-region penalty: centers in the ring-alert simplex sub-band are
// deprioritised so density can't chase the (IDA-barren) IRA flood. See
// SCANNER_IRA_* in scanner_map.h.
static inline float scanner_pos_ranked_rate(const scanner_pos_t *p)
{
    float r = scanner_pos_narrowband_rate(p);
    if (p && p->center_hz >= SCANNER_IRA_REGION_LO_HZ) r *= SCANNER_IRA_RANK_WEIGHT;
    return r;
}

int scanner_rank_hottest(const scanner_pos_t *pos, int n)
{
    if (!pos || n <= 0) return -1;
    int   best      = 0;
    float best_rate = scanner_pos_ranked_rate(&pos[0]);
    for (int i = 1; i < n; i++) {
        float r = scanner_pos_ranked_rate(&pos[i]);
        if (r > best_rate) {
            best_rate = r;
            best      = i;
        }
    }
    return best;
}
