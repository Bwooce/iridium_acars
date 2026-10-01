// See band_profile.h. Table-only — no logic beyond bounds clamping,
// so the host test can pin every field.

#include "band_profile.h"

#include <string.h>

static const band_profile_t s_profiles[BAND_COUNT] = {
    [BAND_IRIDIUM] = {
        .id                  = BAND_IRIDIUM,
        .frontend            = BAND_FE_BURST_TAGGER,
        .name                = "iridium",
        .default_lo_hz       = BAND_IRIDIUM_LO_HZ,
        .detect_fs_hz        = BAND_IRIDIUM_FS_HZ,
        .fbt_pre_len         = BAND_IRIDIUM_FBT_PRE_LEN,
        .fbt_post_len        = BAND_IRIDIUM_FBT_POST_LEN,
        .fbt_width_bins      = BAND_IRIDIUM_FBT_WIDTH_BINS,
        .tagger_threshold_db = BAND_IRIDIUM_TAG_THR_DB,
    },
    [BAND_VDL2] = {
        .id                  = BAND_VDL2,
        .frontend            = BAND_FE_BURST_TAGGER,
        .name                = "vdl2",
        .default_lo_hz       = BAND_VDL2_LO_HZ,
        .detect_fs_hz        = BAND_VDL2_FS_HZ,
        .fbt_pre_len         = BAND_VDL2_FBT_PRE_LEN,
        .fbt_post_len        = BAND_VDL2_FBT_POST_LEN,
        .fbt_width_bins      = BAND_VDL2_FBT_WIDTH_BINS,
        .tagger_threshold_db = BAND_VDL2_TAG_THR_DB,
    },
    [BAND_POA] = {
        .id                  = BAND_POA,
        .frontend            = BAND_FE_CHANNELIZED,
        .name                = "poa",
        .default_lo_hz       = BAND_POA_LO_HZ,
        .detect_fs_hz        = BAND_POA_FS_HZ,
        // Tagger fields UNUSED (channelized front end) — mirror Iridium so any
        // accidental tagger-path use degrades gracefully. See band_profile.h.
        .fbt_pre_len         = BAND_IRIDIUM_FBT_PRE_LEN,
        .fbt_post_len        = BAND_IRIDIUM_FBT_POST_LEN,
        .fbt_width_bins      = BAND_IRIDIUM_FBT_WIDTH_BINS,
        .tagger_threshold_db = BAND_IRIDIUM_TAG_THR_DB,
    },
};

const band_profile_t *band_profile_get(band_id_t id)
{
    if ((unsigned)id >= (unsigned)BAND_COUNT) id = BAND_IRIDIUM;
    return &s_profiles[id];
}

band_id_t band_profile_from_str(const char *s)
{
    if (!s) return BAND_IRIDIUM;
    for (int i = 0; i < (int)BAND_COUNT; i++) {
        if (strcmp(s, s_profiles[i].name) == 0) return (band_id_t)i;
    }
    return BAND_IRIDIUM;
}
