#pragma once
// Pure (no device deps) R828D gain-step table + gain-set generation and
// peak-pick selection for the autotune RF-recalibration routine. Split into a
// header-only unit (like worker_dcfine.h) so the arithmetic is host-testable
// without pulling in the class_driver / FreeRTOS / RTL-SDR device stack.
//
// NOTE: AUTOTUNE_R828D_GAINS duplicates the r82xx_gains[] table in
// librtlsdr.c rtlsdr_get_tuner_gains() (the 29 discrete R828D steps, tenths
// of a dB). The device sweep could instead query rtlsdr_get_tuner_gains() for
// the live steps; this static copy exists only so the selection logic can be
// unit-tested on the host. Keep the two in sync.
#include <stdlib.h> // abs

// 29 discrete R828D gain steps in tenths of a dB (ascending).
static const int AUTOTUNE_R828D_GAINS[] = {
    0,   9,   14,  27,  37,  77,  87,  125, 144, 157,
    166, 197, 207, 229, 254, 280, 297, 328, 338, 364,
    372, 386, 402, 421, 434, 439, 445, 480, 496,
};
#define AUTOTUNE_R828D_N \
    ((int)(sizeof(AUTOTUNE_R828D_GAINS) / sizeof(AUTOTUNE_R828D_GAINS[0])))

// Snap a requested gain (tenths of dB) to the nearest real R828D step.
static inline int autotune_snap_gain(int dbx10)
{
    int best  = AUTOTUNE_R828D_GAINS[0];
    int bestd = abs(dbx10 - best);
    for (int i = 1; i < AUTOTUNE_R828D_N; i++) {
        int d = abs(dbx10 - AUTOTUNE_R828D_GAINS[i]);
        if (d < bestd) {
            bestd = d;
            best  = AUTOTUNE_R828D_GAINS[i];
        }
    }
    return best;
}

// Build the coarse sweep gain set: the real R828D steps within
// [min_dbx10, max_dbx10], sampled every `stride` steps, with the top
// in-range step always appended (the inverted-U peak often sits near the
// high end, so we never want to skip the last candidate). Values written to
// out[0..return-1] (ascending); returns the count (<= cap, <= AUTOTUNE_R828D_N).
// Returns 0 if no table step falls in range or the arguments are invalid.
static inline int autotune_build_gain_set(int min_dbx10, int max_dbx10,
                                          int stride, int *out, int cap)
{
    if (!out || cap <= 0) return 0;
    if (stride < 1) stride = 1;

    // Table is ascending; find the first and last in-range indices.
    int lo = -1, hi = -1;
    for (int i = 0; i < AUTOTUNE_R828D_N; i++) {
        int g = AUTOTUNE_R828D_GAINS[i];
        if (g >= min_dbx10 && g <= max_dbx10) {
            if (lo < 0) lo = i;
            hi = i;
        }
    }
    if (lo < 0) return 0;

    int n = 0;
    for (int i = lo; i <= hi && n < cap; i += stride) {
        out[n++] = AUTOTUNE_R828D_GAINS[i];
    }
    // Ensure the top in-range step is present (stride may have skipped it).
    if (n < cap && (n == 0 || out[n - 1] != AUTOTUNE_R828D_GAINS[hi])) {
        out[n++] = AUTOTUNE_R828D_GAINS[hi];
    }
    return n;
}

// Pick the gain index that maximizes the decode count. Inverted-U objective
// (see design doc): argmax over decoded[]. Strict `>` keeps the earliest
// (lowest-gain) index on a tie — the conservative/cleaner operating point,
// and Poisson noise makes fine adjacent-gain ranking meaningless anyway.
// Returns -1 for an empty/NULL curve.
static inline int autotune_pick_best(const int *decoded, int n)
{
    if (!decoded || n <= 0) return -1;
    int best = 0;
    for (int i = 1; i < n; i++) {
        if (decoded[i] > decoded[best]) best = i;
    }
    return best;
}
