// POA per-region channel presets: region name -> {LO, channel CSV}.
//
// Frequencies are from AUTHORITATIVE sources (NOT guesses): airframes.io
// observed feeder data (github.com/airframesio/data frequency-stats) +
// SITA/ARINC allocations + sigidwiki. See docs/2026-08-04-poa-multiregion-
// plan.md for the full sourced table + confidence per region.
//
// Each region's LO + channels are chosen so every channel sits INSIDE the
// LO +/-1.25 MHz window AND off-DC (>~0.1 MHz from LO, so the LO/DC spike
// doesn't land on a channel's baseband).
//
// NORTH AMERICA COMPROMISE (documented): NA's active POA channels span
// 129.125 -> 131.825 MHz = ~2.7 MHz — WIDER than one 2.5 MHz capture window,
// and the platform ceiling is ~2.5 MSPS (2.8/3.2 MSPS wedge the RTL dongle —
// device-side FIFO/latency limit, see [[reference_p4_max_sample_rate_2500]] +
// docs/2026-08-04-poa-multiregion-plan.md). Critically, NA's TWO BUSIEST
// channels — 131.550 (SITA worldwide primary) and 129.125 (ARINC worldwide
// primary) — are 2.425 MHz apart and CANNOT share one window. So NA is split
// into TWO complementary presets, and the operator picks the half that matches
// their traffic:
//   - "north_america"  (LO 130.875): the 131.550/SITA half + mid-band ARINC
//     (130.025/130.450/131.125/131.725). SACRIFICES 129.125.
//   - "americas_arinc" (LO 129.800): the 129.125/ARINC-low half
//     (129.125/130.025/130.425/130.450). SACRIFICES 131.550/131.725/131.125.
// (130.025/130.450 are mid-band and appear in BOTH — reachable from either LO.)
// Capturing ALL of NA at once needs a SECOND receiver on the other half
// (multi-receiver, task #17) — it is NOT a single-radio sample-rate knob.
// Every other region's set fits one window, so this compromise is NA-only.
//
// Pure C (header-only) so both the firmware setter and a host test use it.
#pragma once

#include <stdint.h>
#include <string.h>

typedef struct {
    const char *name;   // stable key (dropdown value, NVS "po_region", serial arg)
    const char *label;  // human-readable label for the UI
    uint32_t    lo_hz;  // channelizer LO
    const char *chans;  // channel CSV of MHz (fed to poa_chans_parse)
} poa_region_t;

// Confidence (from the research): VERY HIGH — australia, se_asia; HIGH — europe,
// north_america, worldwide 131.550; MEDIUM — south_america, japan (allocation-
// only). Africa/Middle-East omitted (unconfirmed / n=1 — don't ship a guess).
static const poa_region_t POA_REGIONS[] = {
    // name             label                   LO (Hz)      channels (MHz CSV)
    {"australia",      "Australia / Oceania",  130800000u, "131.550,131.450"},
    {"se_asia",        "SE Asia",              130800000u, "131.550,131.450"},
    {"japan",          "Japan",                130800000u, "131.450"},
    {"europe",         "Europe",               131000000u, "131.725,131.825,131.525"},
    {"north_america",  "North America (SITA 131.5)",  130875000u, "130.025,130.450,131.125,131.550,131.725"},
    {"americas_arinc", "Americas (ARINC 129.1)",      129800000u, "129.125,130.025,130.425,130.450"},
    {"south_america",  "South America",        130900000u, "131.550,131.725,131.525"},
    {"worldwide",      "Worldwide (generic)",  130800000u, "131.550,131.450,131.725"},
};
#define POA_REGION_COUNT ((int)(sizeof(POA_REGIONS) / sizeof(POA_REGIONS[0])))

// Return the region with this name, or NULL if unknown.
static inline const poa_region_t *poa_region_lookup(const char *name)
{
    if (!name) return 0;
    for (int i = 0; i < POA_REGION_COUNT; i++)
        if (strcmp(name, POA_REGIONS[i].name) == 0) return &POA_REGIONS[i];
    return 0;
}
