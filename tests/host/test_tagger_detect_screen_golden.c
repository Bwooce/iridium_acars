// test_tagger_detect_screen_golden.c — correctness harness for the ESP32-P4
// PIE detect-scan pre-screen kernel fbt_detect_screen_arp4 (added to
// common/iridium_decoder/fft_burst_tagger_arp4.S) and its C integration in
// create_new_bursts_internal (fft_burst_tagger.c).
//
// WHAT THIS PROVES (and what it can't)
// ------------------------------------
// The host has no PIE unit — the .S compiles to nothing and the tagger runs
// the scalar detect scan. So this test cannot execute the arp4 instructions.
// Like test_tagger_mag_ema_golden.c it re-implements, byte-for-byte:
//   (a) the EXACT threshold test `above_threshold()`  (fft_burst_tagger.c),
//   (b) the screen shift derivation  s = clamp(24 - floor(log2(thr_q15)),0,24)
//       (fft_burst_tagger.c init), and
//   (c) a lane MODEL of the screen kernel: per bin S = mag > (base>>s);
//       per 16-bin group the flag is the OR over its bins.
// and asserts the two properties the design's bit-exactness rests on:
//   1. SUPERSET — every (mag,base) that passes the exact test also passes the
//      screen — swept over threshold_db 0..30 (incl the a>24 clamp) and over
//      boundary/extreme operands (base=0, base=INT32_MAX, mag=MAG_MAX,
//      lhs=rhs±1). If this ever fails, the screen would drop a real burst.
//   2. BYTE-IDENTICAL SCAN — running the exact test only on bins of flagged
//      groups (the device path) yields the SAME peak bins, in the SAME order,
//      as scanning every bin (the scalar path), over synthetic mag/base arrays.
// Negative controls confirm the asserts have teeth (a too-strict screen shift
// MUST break the superset).
//
// It does NOT prove the silicon produces these bytes — that is the on-device
// scalar-vs-PIE diff (fbt_detect_screen_diff) + the RAW GOLDEN matched-count
// equality check. See the .S header for the device-only lane assumptions.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "fft_burst_tagger.h" // FBT_FFT_SIZE, FBT_HISTORY_SIZE

#define N        FBT_FFT_SIZE
#define HIST     FBT_HISTORY_SIZE       // 512 = 2^9
#define MAG_MAX  2147352578             // 32767^2 * 2, the mag² hard bound

// ---- byte-for-byte the exact test in fft_burst_tagger.c:above_threshold ----
static int exact_above(int32_t mag2, int32_t base, int32_t thr_q15)
{
    int64_t lhs = (int64_t)mag2 * (int64_t)HIST;
    int64_t rhs = ((int64_t)base * (int64_t)thr_q15) >> 15;
    return lhs > rhs;
}

// ---- byte-for-byte the shift derivation in fft_burst_tagger_init ----
static int screen_shift_for(int32_t thr_q15)
{
    if (thr_q15 <= 0) return -1;
    int a = 31 - __builtin_clz((uint32_t)thr_q15);
    int s = 24 - a;
    if (s < 0)  s = 0;
    if (s > 24) s = 24;
    return s;
}

// ---- C model of one lane of fbt_detect_screen_arp4: mag > (base>>s) ----
static int screen_lane(int32_t mag2, int32_t base, int s)
{
    return mag2 > (base >> s); // base >= 0 so arith >> == logical >>
}

// Byte-for-byte the threshold_q15 formula in fft_burst_tagger_init.
static int32_t thr_q15_for_db(double db)
{
    double t_lin = pow(10.0, db / 10.0);
    return (int32_t)(t_lin * 32768.0 + 0.5);
}

// ---------------------- property 1: SUPERSET -----------------------------
static int check_superset(void)
{
    int fails = 0;
    // Sweep threshold 0..30 dB (14 dB default is mid-range; >~27 dB hits a>24
    // clamp -> s=0). At each, sweep a wide operand grid incl. exact boundaries.
    for (int db10 = 0; db10 <= 300; db10 += 1) {   // 0.0 .. 30.0 dB in 0.1 steps
        double db = db10 / 10.0;
        int32_t thr = thr_q15_for_db(db);
        int s = screen_shift_for(thr);
        if (s < 0) continue;
        // Deterministic pseudo-random + boundary operands.
        uint64_t r = 0x9e3779b97f4a7c15ULL ^ (uint64_t)db10;
        for (int i = 0; i < 4000; i++) {
            r = r * 6364136223846793005ULL + 1442695040888963407ULL;
            int32_t base = (int32_t)((r >> 33) % 600000000);        // 0..6e8 (baseline range)
            r = r * 6364136223846793005ULL + 1442695040888963407ULL;
            int32_t mag  = (int32_t)((r >> 33) % (uint32_t)MAG_MAX); // 0..MAG_MAX
            // Also probe the exact boundary: mag right around rhs/HIST.
            for (int k = 0; k < 3; k++) {
                int32_t m = mag;
                if (k == 1) { // nudge mag to just below/at the exact threshold
                    int64_t rhs = ((int64_t)base * (int64_t)thr) >> 15;
                    m = (int32_t)((rhs / HIST));       // ~exact boundary
                } else if (k == 2) {
                    int64_t rhs = ((int64_t)base * (int64_t)thr) >> 15;
                    int64_t mm = (rhs / HIST) + 1;
                    m = (mm > MAG_MAX) ? MAG_MAX : (int32_t)mm;
                }
                if (exact_above(m, base, thr) && !screen_lane(m, base, s)) {
                    if (fails < 6)
                        fprintf(stderr,
                            "SUPERSET FAIL db=%.1f thr=%d s=%d mag=%d base=%d "
                            "(exact=1 screen=0)\n", db, thr, s, m, base);
                    fails++;
                }
            }
        }
        // Hard boundary operands at this threshold.
        int32_t bnd_base[] = {0, 1, INT32_MAX, INT32_MAX / 2, 512, 600000000};
        int32_t bnd_mag[]  = {0, 1, MAG_MAX, MAG_MAX / 2, 512};
        for (unsigned bi = 0; bi < sizeof(bnd_base)/sizeof(bnd_base[0]); bi++)
            for (unsigned mi = 0; mi < sizeof(bnd_mag)/sizeof(bnd_mag[0]); mi++) {
                int32_t b = bnd_base[bi], m = bnd_mag[mi];
                if (exact_above(m, b, thr) && !screen_lane(m, b, s)) {
                    if (fails < 6)
                        fprintf(stderr, "SUPERSET FAIL(bnd) db=%.1f mag=%d base=%d\n",
                                db, m, b);
                    fails++;
                }
            }
    }
    return fails;
}

// ------------- property 2: byte-identical scan (scalar vs screened) -------
// Peak = bin passing the exact test, within [margin, N-margin), not masked.
// (dc-mask / carrier-saturation are orthogonal side tables — modelled off
// here; the production body applies them identically in both paths.)
static int scan_scalar(const int32_t *mag, const int32_t *base, int32_t thr,
                       int margin, int *out_bins)
{
    int n = 0;
    for (int bin = margin; bin < N - margin; bin++)
        if (exact_above(mag[bin], base[bin], thr)) out_bins[n++] = bin;
    return n;
}
static int scan_screened(const int32_t *mag, const int32_t *base, int32_t thr,
                         int s, int margin, int *out_bins)
{
    // group flags (kernel model): flag[g] = OR over the 16 bins of group g
    int n = 0;
    for (int g = 0; g < N / 16; g++) {
        int any = 0;
        for (int j = 0; j < 16; j++)
            if (screen_lane(mag[g*16+j], base[g*16+j], s)) { any = 1; break; }
        if (!any) continue;
        int lo = g * 16, hi = lo + 16;
        if (lo < margin)     lo = margin;
        if (hi > N - margin) hi = N - margin;
        for (int bin = lo; bin < hi; bin++)
            if (exact_above(mag[bin], base[bin], thr)) out_bins[n++] = bin;
    }
    return n;
}

static int check_scan_identical(void)
{
    static int32_t mag[N], base[N];
    static int sc[N], scr[N];
    int fails = 0;
    uint64_t r = 0xd1b54a32d192ed03ULL;
    // A spread of thresholds incl default 14 dB and the a>24 regime.
    double dbs[] = {6.0, 10.0, 14.0, 20.0, 28.0};
    for (unsigned di = 0; di < sizeof(dbs)/sizeof(dbs[0]); di++) {
        int32_t thr = thr_q15_for_db(dbs[di]);
        int s = screen_shift_for(thr);
        int margin = 16; // burst_width/2 default
        for (int trial = 0; trial < 40; trial++) {
            for (int i = 0; i < N; i++) {
                r = r * 6364136223846793005ULL + 1442695040888963407ULL;
                base[i] = (int32_t)((r >> 33) % 20000000);          // noisy floor
                r = r * 6364136223846793005ULL + 1442695040888963407ULL;
                // Mostly noise; occasional strong bins (real bursts).
                mag[i]  = (int32_t)((r >> 33) % 100000);
                if (((r >> 20) & 0x3f) == 0) mag[i] += (int32_t)((r >> 40) % 5000000);
            }
            int ns  = scan_scalar(mag, base, thr, margin, sc);
            int nsr = scan_screened(mag, base, thr, s, margin, scr);
            if (ns != nsr || memcmp(sc, scr, (size_t)ns * sizeof(int))) {
                if (fails < 6)
                    fprintf(stderr, "SCAN MISMATCH db=%.1f trial=%d scalar=%d "
                            "screened=%d\n", dbs[di], trial, ns, nsr);
                fails++;
            }
        }
    }
    return fails;
}

// ------------- negative control: a too-strict screen MUST break superset ---
static int check_negative_control(void)
{
    // A screen shift one SMALLER than derived is stricter (rejects more) — it
    // must drop at least one true peak somewhere, i.e. the superset must FAIL.
    int32_t thr = thr_q15_for_db(14.0);
    int s = screen_shift_for(thr);
    if (s <= 0) return 1; // can't make it stricter; control inapplicable
    int broke = 0;
    uint64_t r = 0x123456789abcdefULL;
    for (int i = 0; i < 2000000 && !broke; i++) {
        r = r * 6364136223846793005ULL + 1442695040888963407ULL;
        int32_t base = (int32_t)((r >> 33) % 600000000);
        r = r * 6364136223846793005ULL + 1442695040888963407ULL;
        int32_t mag  = (int32_t)((r >> 33) % (uint32_t)MAG_MAX);
        if (exact_above(mag, base, thr) && !screen_lane(mag, base, s - 1)) broke = 1;
    }
    return broke ? 0 : 1; // 0 = control worked (found a break); 1 = FAILED to break
}

int main(void)
{
    int rc = 0;
    int f1 = check_superset();
    printf("superset (exact-true => screen-true, 0..30 dB + boundaries): %s\n",
           f1 == 0 ? "PASS" : "FAIL");
    if (f1) { printf("  %d violations\n", f1); rc = 1; }

    int f2 = check_scan_identical();
    printf("scan-identical (scalar peaks == screened peaks): %s\n",
           f2 == 0 ? "PASS" : "FAIL");
    if (f2) { printf("  %d mismatches\n", f2); rc = 1; }

    int f3 = check_negative_control();
    printf("negative-control (stricter screen breaks superset): %s\n",
           f3 == 0 ? "PASS" : "FAIL");
    if (f3) { printf("  control did NOT catch a too-strict screen\n"); rc = 1; }

    printf("%s\n", rc == 0 ? "ALL PASS" : "FAILURES");
    return rc;
}
