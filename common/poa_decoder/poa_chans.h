// Shared POA channel-list CSV parser: "131.550,130.025,..." -> freqs in Hz.
//
// Pure C (host-testable), used by BOTH dsp_processor (boot-time channel
// resolution) and app_config (the runtime po_chans setter's validation), so a
// validator can never accept a CSV the real parser would parse differently.
// Callers range-check the returned freqs (this only rejects <= 1 MHz junk).
#pragma once

#include <stdint.h>
#include <stdlib.h>

// Parse a CSV of MHz into out[] (Hz), up to `max` entries. Returns the count
// (<= max). Skips tokens <= 1 MHz (malformed/empty); stops at the first
// non-numeric token. A leading '\0'/NULL yields 0.
static inline int poa_chans_parse(const char *csv, uint32_t *out, int max)
{
    if (!csv || !csv[0]) return 0;
    int         n = 0;
    const char *s = csv;
    while (*s && n < max) {
        char  *end;
        double mhz = strtod(s, &end);
        if (end == s) break;
        // Upper bound guards the double->uint32 cast (UB for MHz > ~4294);
        // 4000 MHz is far above any real airband channel, so this rejects only
        // garbage, not valid input. Callers still range-check to the VHF band.
        if (mhz > 1.0 && mhz < 4000.0) out[n++] = (uint32_t)(mhz * 1e6 + 0.5);
        s = end;
        while (*s == ',' || *s == ' ' || *s == '\t') s++;
    }
    return n;
}
