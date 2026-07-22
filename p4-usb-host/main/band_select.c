// See band_select.h.

#include "band_select.h"

#include "iridium_band_pipeline.h"
#include "vdl2_pipeline.h"

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
