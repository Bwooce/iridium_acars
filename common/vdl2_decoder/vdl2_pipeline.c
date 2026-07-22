// See vdl2_pipeline.h — band_pipeline_t implementation for VDL Mode 2.
// process_burst = vdl2_demod_burst() in a loop (VDL2 is CSMA on one
// channel: consecutive transmissions can merge into a single tagger
// window, so after each decoded frame we resume scanning the remainder
// — the plan's multi-frame model). RS(255,249) + AVLC + libacars run
// downstream in the emit sink (plan C2/C3/V3), fed the descrambled
// PHY bits emitted here.

#include "vdl2_pipeline.h"

#include <stdatomic.h>
#include <stdlib.h>

#include "vdl2_demod.h"

// Relaxed atomics: incremented on the worker task, read from diagnostic
// contexts (httpd/serial) — same idiom as worker_core1.c's counters.
static _Atomic uint32_t s_bursts_seen = 0;
static _Atomic uint32_t s_sync_ok     = 0; // preamble+header locked
static _Atomic uint32_t s_frames_ok   = 0; // complete frames emitted

// Cap on frames pulled out of ONE tagger window. A window is at most
// ~260 ms (worker WB_MAX_BURST_SAMPLES); the shortest legal burst is
// ~10 ms, but merged back-to-back transmissions beyond a handful in
// one window means something is wrong — bound the loop.
#define VDL2_MAX_FRAMES_PER_WINDOW 8

static bool vdl2_prefilter(const int16_t *iq250, int n_complex, int width_bins,
                           band_prefilter_verdict_t *v)
{
    (void)iq250;
    (void)n_complex;
    (void)width_bins;
    // Accept everything: no VDL2-calibrated junk gates exist yet (V1
    // capture calibration decides them); the demod's training-sequence
    // lock is the effective gate and is cheap at 10.5 kBd.
    v->accept = v->width_ok = v->dur_ok = v->snr_ok = true;
    return true;
}

static int vdl2_process_burst(int16_t *iq250, int n_complex,
                              band_frame_cb_t cb, void *ctx)
{
    atomic_fetch_add_explicit(&s_bursts_seen, 1u, memory_order_relaxed);

    int n_ok   = 0;
    int cursor = 0;
    for (int f = 0; f < VDL2_MAX_FRAMES_PER_WINDOW; f++) {
        vdl2_demod_result_t res;
        if (!vdl2_demod_burst(iq250 + 2 * cursor, n_complex - cursor, &res))
            break; // no (further) preamble lock in the window
        atomic_fetch_add_explicit(&s_sync_ok, 1u, memory_order_relaxed);

        band_frame_t frame = {
            .bits      = res.bits,
            .n_bits    = res.n_bits,
            .soft_bits = res.soft_bits,
            .direction = 0, // VDL2: single direction from the receiver's POV
            // demod_ok=false marks a header-locked but window-truncated
            // frame: diagnostic callback, bits still owned by the sink
            // (band_pipeline.h contract).
            .demod_ok    = res.complete,
            .snr_db      = res.snr_db,
            .band_detail = &res,
        };
        if (cb) {
            cb(&frame, ctx);
        } else {
            free(res.bits);
            free(res.soft_bits);
        }
        if (res.complete) {
            n_ok++;
            atomic_fetch_add_explicit(&s_frames_ok, 1u, memory_order_relaxed);
        }

        // Resume scanning after this frame (back-to-back CSMA bursts).
        int adv = res.consumed_complex_250k;
        if (adv < 1) adv = 1;
        cursor += adv;
        if (cursor >= n_complex) break;
    }
    return n_ok;
}

const band_pipeline_t *vdl2_pipeline(void)
{
    static const band_pipeline_t p = {
        .name          = "vdl2",
        .prefilter     = vdl2_prefilter,
        .process_burst = vdl2_process_burst,
    };
    return &p;
}

uint32_t vdl2_pipeline_bursts_seen(void)
{
    return atomic_load_explicit(&s_bursts_seen, memory_order_relaxed);
}

uint32_t vdl2_pipeline_sync_count(void)
{
    return atomic_load_explicit(&s_sync_ok, memory_order_relaxed);
}

uint32_t vdl2_pipeline_frames_ok(void)
{
    return atomic_load_explicit(&s_frames_ok, memory_order_relaxed);
}
