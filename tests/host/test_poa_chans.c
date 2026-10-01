// Unit test for the shared POA channel-list CSV parser (poa_chans_parse),
// which backs BOTH dsp_processor's boot-time channel resolution and
// app_config's runtime po_chans setter validation. If this parser is wrong,
// the setter accepts CSVs the device then parses differently.
#include "poa_chans.h"
#include "poa_regions.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static int fails = 0;
#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        if (!(cond)) { printf("FAIL: %s\n", msg); fails++; }                    \
    } while (0)

int main(void)
{
    uint32_t f[8];

    // Australia set — 4 channels, exact Hz conversion (round-to-nearest).
    int n = poa_chans_parse("131.550,131.450,131.475,131.525", f, 8);
    CHECK(n == 4, "AU set count == 4");
    CHECK(f[0] == 131550000u, "131.550 -> 131550000 Hz");
    CHECK(f[1] == 131450000u, "131.450 -> 131450000 Hz");
    CHECK(f[2] == 131475000u, "131.475 -> 131475000 Hz");
    CHECK(f[3] == 131525000u, "131.525 -> 131525000 Hz");

    // 8-channel discovery set — fills exactly, no overflow.
    n = poa_chans_parse("130.025,130.425,130.450,131.125,131.450,131.475,131.525,131.550",
                        f, 8);
    CHECK(n == 8, "discovery set count == 8");
    CHECK(f[3] == 131125000u, "131.125 -> 131125000 Hz");

    // max clamp: 9 tokens into a 4-slot buffer -> stops at 4.
    n = poa_chans_parse("1,2,3,4,5,6,7,8,9", f, 4);
    CHECK(n == 4, "clamps to max=4");

    // whitespace + trailing comma tolerated.
    n = poa_chans_parse(" 131.550 , 130.025 ,", f, 8);
    CHECK(n == 2, "whitespace/trailing-comma -> 2");

    // empty / NULL -> 0.
    CHECK(poa_chans_parse("", f, 8) == 0, "empty -> 0");
    CHECK(poa_chans_parse(NULL, f, 8) == 0, "NULL -> 0");

    // tokens <= 1 MHz are skipped (malformed/zero).
    n = poa_chans_parse("0,131.550", f, 8);
    CHECK(n == 1 && f[0] == 131550000u, "leading 0 skipped, 131.550 kept");

    // non-numeric stops parsing.
    n = poa_chans_parse("131.550,junk,130.025", f, 8);
    CHECK(n == 1, "stops at non-numeric token");

    // absurd MHz (> ~4294) is skipped, not cast (double->uint32 UB guard).
    n = poa_chans_parse("9999.0,131.550", f, 8);
    CHECK(n == 1 && f[0] == 131550000u, "9999 MHz skipped, 131.550 kept");

    // --- region table (poa_regions.h) validity ---
    // Every region's channels must: parse (1..8), sit inside LO +/-1.25 MHz,
    // and stay off-DC (>50 kHz from LO, so the LO/DC spike misses a channel).
    for (int r = 0; r < POA_REGION_COUNT; r++) {
        const poa_region_t *R = &POA_REGIONS[r];
        uint32_t ch[8];
        int nc = poa_chans_parse(R->chans, ch, 8);
        char msg[128];
        snprintf(msg, sizeof(msg), "region '%s' parses 1..8 channels", R->name);
        CHECK(nc >= 1 && nc <= 8, msg);
        for (int i = 0; i < nc; i++) {
            long d = (long)ch[i] - (long)R->lo_hz;
            long ad = d < 0 ? -d : d;
            snprintf(msg, sizeof(msg), "region '%s' ch %.4f in LO+/-1.25MHz",
                     R->name, ch[i] / 1e6);
            CHECK(ad <= 1250000L, msg);
            snprintf(msg, sizeof(msg), "region '%s' ch %.4f off-DC (>50kHz from LO)",
                     R->name, ch[i] / 1e6);
            CHECK(ad > 50000L, msg);
        }
    }
    // Australia preset must be exactly the confirmed AU/NZ set.
    const poa_region_t *au = poa_region_lookup("australia");
    CHECK(au && strcmp(au->chans, "131.550,131.450") == 0, "australia == 131.550,131.450");
    CHECK(poa_region_lookup("does_not_exist") == NULL, "unknown region -> NULL");

    if (fails) { printf("test_poa_chans: %d FAILED\n", fails); return 1; }
    printf("test_poa_chans: all passed\n");
    return 0;
}
