// test_band_profile — pins the band-profile table (VHF/VDL2 foundation).
//
// The critical assertion block is IRIDIUM-IDENTITY: the iridium profile
// must equal the historical hard-coded constants from
// p4-usb-host/main/dsp_processor.{c,h} exactly, because
// dsp_processor_create now configures the tagger from the profile and
// band=iridium (the default) must be bit-identical to the pre-band
// firmware. The integer fields are ALSO _Static_asserted in
// dsp_processor.c; this test additionally pins the float threshold
// (which a _Static_assert can't) and the table wiring itself.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "band_profile.h"

// Historical constants, copied literally from the pre-band firmware
// (dsp_processor.h / dsp_processor.c). Do NOT "refactor" these to use
// the BAND_IRIDIUM_* macros — the point is an independent pin.
#define LEGACY_IRIDIUM_LO_HZ 1626000000u  // IRIDIUM_CENTER_FREQ_HZ
#define LEGACY_FS_DETECT_HZ 2500000u      // FS_DETECT_HZ
#define LEGACY_FBT_PRE_LEN (2 * 2048)     // FBT_BURST_PRE_LEN = 2 × FBT_FFT_SIZE
#define LEGACY_FBT_POST_LEN 40000         // FBT_BURST_POST_LEN
#define LEGACY_FBT_WIDTH 32               // FBT_BURST_WIDTH
#define LEGACY_TAG_THR_DB 14.0f           // FBT_THRESHOLD_DB

static void test_iridium_identity(void)
{
    const band_profile_t *p = band_profile_get(BAND_IRIDIUM);
    assert(p != NULL);
    assert(p->id == BAND_IRIDIUM);
    assert(strcmp(p->name, "iridium") == 0);
    assert(p->default_lo_hz == LEGACY_IRIDIUM_LO_HZ);
    assert(p->detect_fs_hz == LEGACY_FS_DETECT_HZ);
    assert(p->fbt_pre_len == LEGACY_FBT_PRE_LEN);
    assert(p->fbt_post_len == LEGACY_FBT_POST_LEN);
    assert(p->fbt_width_bins == LEGACY_FBT_WIDTH);
    assert(p->tagger_threshold_db == LEGACY_TAG_THR_DB); // exact: same literal
    assert(p->frontend == BAND_FE_BURST_TAGGER);         // Iridium uses the tagger
    printf("iridium identity: OK (pre=%d post=%d width=%d thr=%.1f fs=%u lo=%u)\n",
           p->fbt_pre_len, p->fbt_post_len, p->fbt_width_bins,
           (double)p->tagger_threshold_db, p->detect_fs_hz, p->default_lo_hz);
}

static void test_out_of_range_clamps_to_iridium(void)
{
    // Stale/corrupt NVS byte must yield the iridium profile, never NULL
    // or out-of-bounds table reads.
    const band_profile_t *p = band_profile_get((band_id_t)BAND_COUNT);
    assert(p == band_profile_get(BAND_IRIDIUM));
    p = band_profile_get((band_id_t)0xFF);
    assert(p == band_profile_get(BAND_IRIDIUM));
    printf("out-of-range clamp: OK\n");
}

static void test_from_str(void)
{
    assert(band_profile_from_str("iridium") == BAND_IRIDIUM);
    assert(band_profile_from_str("vdl2") == BAND_VDL2);
    assert(band_profile_from_str("poa") == BAND_POA); // POA is now a real band
    // Unknown / NULL tokens default to iridium (the safe band).
    assert(band_profile_from_str("") == BAND_IRIDIUM);
    assert(band_profile_from_str(NULL) == BAND_IRIDIUM);
    // Name round-trip for every profile.
    for (int i = 0; i < (int)BAND_COUNT; i++) {
        const band_profile_t *p = band_profile_get((band_id_t)i);
        assert(band_profile_from_str(p->name) == (band_id_t)i);
    }
    printf("from_str: OK\n");
}

static void test_vdl2_profile_sanity(void)
{
    const band_profile_t *p = band_profile_get(BAND_VDL2);
    assert(p->id == BAND_VDL2);
    assert(strcmp(p->name, "vdl2") == 0);

    // Front-end rate must stay at the shared 2.5 MSPS path (the whole
    // point of the soft-switch: ingest/signal_buffer/tagger unchanged).
    assert(p->detect_fs_hz == LEGACY_FS_DETECT_HZ);

    // Every VDL2 channel (136.650–136.975 MHz, 25 kHz grid) must fall
    // inside the LO ± fs/2 window with margin for the tagger's edge
    // bins (use ± fs/4 — far stricter than needed, still passes since
    // the whole band spans only ~350 kHz).
    const uint32_t lo = p->default_lo_hz;
    for (uint32_t ch = 136650000u; ch <= 136975000u; ch += 25000u) {
        int64_t off = (int64_t)ch - (int64_t)lo;
        assert(off > -(int64_t)p->detect_fs_hz / 4);
        assert(off < (int64_t)p->detect_fs_hz / 4);
        // No channel may land at DC (DC-removal / near-DC handling
        // would eat it — memory: don't centre the downconvert on a
        // burst). Keep every channel at least half a channel away.
        assert(off >= 12500 || off <= -12500);
    }

    // Tagger window params are positive and the width is narrower than
    // Iridium's (25 kHz VDL2 channel < 41.667 kHz Iridium channel).
    assert(p->fbt_pre_len > 0 && p->fbt_post_len > 0);
    assert(p->fbt_width_bins > 0 && p->fbt_width_bins <= LEGACY_FBT_WIDTH);
    assert(p->tagger_threshold_db > 0.0f);
    assert(p->frontend == BAND_FE_BURST_TAGGER); // VDL2 also uses the tagger
    printf("vdl2 profile sanity: OK (lo=%u, all channels off-DC in-window)\n", lo);
}

static void test_poa_profile_sanity(void)
{
    const band_profile_t *p = band_profile_get(BAND_POA);
    assert(p->id == BAND_POA);
    assert(strcmp(p->name, "poa") == 0);

    // POA is the CHANNELIZED front end — it must NOT be routed through the
    // burst tagger (that's the load-bearing decision of the POA plan §1).
    assert(p->frontend == BAND_FE_CHANNELIZED);

    // Shared 2.5 MSPS front-end rate (channelizer decimates /200 -> 12.5 kHz).
    assert(p->detect_fs_hz == LEGACY_FS_DETECT_HZ);
    assert(p->default_lo_hz == 130800000u);

    // The site's POA channel set must fall inside LO ± fs/2, off-DC, so the
    // channelizer's per-channel mix lands each within the captured window.
    const uint32_t lo = p->default_lo_hz;
    const uint32_t chans[] = {131550000u, 130450000u, 130425000u, 130025000u};
    for (unsigned i = 0; i < sizeof(chans)/sizeof(chans[0]); i++) {
        int64_t off = (int64_t)chans[i] - (int64_t)lo;
        assert(off > -(int64_t)p->detect_fs_hz / 2);
        assert(off <  (int64_t)p->detect_fs_hz / 2);
        assert(off >= 25000 || off <= -25000); // >= 25 kHz DC clearance (acarsdec chooseFc)
    }
    printf("poa profile sanity: OK (lo=%u, channelized, channels off-DC in-window)\n", lo);
}

int main(void)
{
    test_iridium_identity();
    test_out_of_range_clamps_to_iridium();
    test_from_str();
    test_vdl2_profile_sanity();
    test_poa_profile_sanity();
    printf("PASS: band_profile\n");
    return 0;
}
