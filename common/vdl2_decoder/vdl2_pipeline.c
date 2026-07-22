// See vdl2_pipeline.h — foundation stub; the D8PSK/RS/AVLC chain plugs
// in here per docs/2026-07-22-vdl2-implementation-plan.md.

#include "vdl2_pipeline.h"

#include <stdatomic.h>

// Relaxed atomic: incremented on the worker task, read from diagnostic
// contexts (httpd/serial) — same idiom as worker_core1.c's counters.
static _Atomic uint32_t s_bursts_seen = 0;

static bool vdl2_prefilter(const int16_t *iq250, int n_complex, int width_bins,
                           band_prefilter_verdict_t *v)
{
    (void)iq250;
    (void)n_complex;
    (void)width_bins;
    // Accept everything: no VDL2-calibrated junk gates exist yet, and a
    // recall-safe stub must never reject (the Iridium prefilter's gates
    // are grounded in gr-iridium constants that don't transfer).
    v->accept = v->width_ok = v->dur_ok = v->snr_ok = true;
    return true;
}

static int vdl2_process_burst(int16_t *iq250, int n_complex,
                              band_frame_cb_t cb, void *ctx)
{
    (void)iq250;
    (void)n_complex;
    (void)cb;
    (void)ctx;
    atomic_fetch_add_explicit(&s_bursts_seen, 1u, memory_order_relaxed);
    // TODO(Phase V3, docs/2026-07-22-vdl2-implementation-plan.md):
    //   vdl2_demod_burst() -> rs_255_249_decode() -> avlc_deframe(),
    //   then fire cb once per FCS-valid AVLC frame.
    return 0;
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
