// test_vdl2_pipeline_stub — interface conformance for the VDL2
// foundation stub (common/vdl2_decoder/vdl2_pipeline.c). The stub must:
//   - expose a well-formed band_pipeline_t (name "vdl2", both hooks set),
//   - accept every burst at the prefilter with an all-true verdict
//     (recall-safe: no VDL2-calibrated gates exist yet),
//   - demodulate nothing (process_burst returns 0, never fires the cb),
//   - count bursts (vdl2_pipeline_bursts_seen) so a live band=vdl2 soak
//     can prove the front-end plumbing before any demod exists.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "band_pipeline.h"
#include "vdl2_pipeline.h"

static int s_cb_fired = 0;
static void must_not_fire(band_frame_t *f, void *ctx)
{
    (void)f;
    (void)ctx;
    s_cb_fired++;
}

int main(void)
{
    const band_pipeline_t *p = vdl2_pipeline();
    assert(p != NULL);
    assert(strcmp(p->name, "vdl2") == 0);
    assert(p->prefilter != NULL);
    assert(p->process_burst != NULL);

    int16_t iq[64] = {0};

    // Prefilter: accept-all, every gate true (worker's continuation
    // logic reads the gate booleans — they must be defined, not junk).
    band_prefilter_verdict_t v;
    memset(&v, 0, sizeof(v));
    assert(p->prefilter(iq, 32, /*width_bins=*/9999, &v) == true);
    assert(v.accept && v.width_ok && v.dur_ok && v.snr_ok);

    // process_burst: zero frames, callback never fires, bursts counted.
    uint32_t before = vdl2_pipeline_bursts_seen();
    int      n      = p->process_burst(iq, 32, must_not_fire, NULL);
    assert(n == 0);
    assert(s_cb_fired == 0);
    assert(vdl2_pipeline_bursts_seen() == before + 1);
    n = p->process_burst(iq, 32, must_not_fire, NULL);
    assert(n == 0 && s_cb_fired == 0);
    assert(vdl2_pipeline_bursts_seen() == before + 2);

    printf("PASS: vdl2 stub pipeline conforms to band_pipeline_t\n");
    return 0;
}
