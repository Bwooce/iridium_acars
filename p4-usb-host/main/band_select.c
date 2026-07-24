// See band_select.h.

#include "band_select.h"

#include "iridium_band_pipeline.h"
#include "vdl2_pipeline.h"

#include "frame_decoder.h" // frame_decoder_get_vdl2_stats (VDL2 funnel)
#include "worker_core1.h"  // worker_core1_get_bch_cumulative (Iridium funnel)

#include <string.h>

const band_pipeline_t *band_select_pipeline(band_id_t id)
{
    switch (id) {
    case BAND_VDL2:
        return vdl2_pipeline();
    case BAND_IRIDIUM:
    default: // stale/corrupt NVS byte -> the safe default, like band_profile_get
        return iridium_band_pipeline();
    }
}

void band_decode_stats_get(band_id_t id, band_decode_stats_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (id == BAND_VDL2) {
        frame_decoder_vdl2_stats_t vd = {0};
        frame_decoder_get_vdl2_stats(&vd);
        out->decoded   = (uint32_t)vd.avlc_ok; // FCS-valid AVLC frames
        out->unknown   = 0;                    // no Iridium-style UNKNOWN class in VDL2
        out->failed    = (uint32_t)vd.bad_fcs;
        out->recovered = vd.rs_erasure_recovered;
    } else {
        // Iridium (and the clamp default). Same cumulative counters the
        // pre-refactor autotune read via worker_core1_get_decode_counts.
        uint32_t d = 0, u = 0, f = 0, c = 0;
        worker_core1_get_bch_cumulative(&d, &u, &f, &c);
        out->decoded   = d;
        out->unknown   = u;
        out->failed    = f;
        out->recovered = c;
    }
}
