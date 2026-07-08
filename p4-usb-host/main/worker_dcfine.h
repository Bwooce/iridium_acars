#pragma once
// Pure (no device deps) fine near-DC diagnostic constants + bucket math.
// Split out of worker_core1.h (which pulls in esp_err.h/dsp_processor.h) so
// it is host-testable. Included by worker_core1.c, http_server.c, and the
// host test. (near-DC tagger-mask work, 2026-07-08.)
#include <math.h>

// Per-FFT-bin occupancy over DC ±WORKER_DCFINE_HALF bins, to pin the
// tuner-internal DC/LO artifact and watch it drift over time.
// Bin width = FS_DETECT_HZ / FBT_FFT_SIZE = 2500000/2048 ≈ 1220.703 Hz.
#define WORKER_DCFINE_HALF 192
#define WORKER_DCFINE_BINS (2 * WORKER_DCFINE_HALF) // 384

// Map a detection's offset-from-LO (Hz) to a fine-histogram bucket.
// Returns 0..WORKER_DCFINE_BINS-1, or -1 if outside the ±HALF window.
static inline int worker_dcfine_index(float rel_freq_hz)
{
    // rel / (FS/N) = rel * N / FS  → signed bin offset from DC.
    int off = (int)lrintf(rel_freq_hz * 2048.0f / 2500000.0f);
    int idx = off + WORKER_DCFINE_HALF;
    if (idx < 0 || idx >= WORKER_DCFINE_BINS) return -1;
    return idx;
}
