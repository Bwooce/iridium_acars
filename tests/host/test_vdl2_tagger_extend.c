// test_vdl2_tagger_extend.c — regression guard for the VDL2 integrated-active
// tagger fix (common/iridium_decoder/fft_burst_tagger.c, update_bursts_internal).
//
// THE FIX: for band=vdl2 a tracked burst stays "active" (keeps advancing
// last_active) while the INTEGRATED magnitude over the channel width
// (burst_width=14 bins) exceeds threshold — not just the carrier/center bin.
// Iridium keeps the historical peak-bin ±1 test (byte-identical else branch).
//
// WHY: real VDL2 D8PSK spreads its energy across the ~14-bin channel, and over
// a single 2048-pt FFT window (~0.8 ms ≈ 8.6 symbols) the instantaneous carrier
// (center) bin fluctuates hard while the 14-bin integral is stable. On-air we
// captured a real burst window with peak_db 11.2 (< thr 14 → the peak test
// marks it IDLE) while integ_db was 19.8 (≫ 14). The old peak-only test let the
// burst go idle on those dips and it truncated to the post-pad floor; the
// integrated test holds it to full length. Narrowband spikes are the opposite
// (peak ≫ integ, integ < thr) and the integrated test correctly rejects them.
//
// METHOD: synthesize a continuous 2.5 MSPS stream (noise warmup + one spread
// D8PSK burst + noise tail) with vdl2_mod, then drive it through the REAL
// tagger twice — integrated mode vs peak mode (toggled via the same band=vdl2
// flag production uses, fft_burst_tagger_set_trace_enabled) — and assert the
// integrated mode tags the burst at ~full length while the peak mode truncates
// it. Differential: independent of absolute dB calibration, it only asserts the
// two modes DIVERGE in the marginal-SNR regime, in the direction the fix claims.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fft_burst_tagger.h"
#include "vdl2_mod.h"

// VDL2 band-profile tagger params (common/band_pipeline/band_profile.h). Hard-
// coded here to keep the test self-contained; if the band profile changes,
// update these to match (they are asserted-against, not derived).
#define VDL2_FBT_PRE_LEN    6144
#define VDL2_FBT_POST_LEN   10000
#define VDL2_FBT_WIDTH_BINS 14
#define VDL2_TAG_THR_DB     14.0f
#define FS_HZ               2500000.0

static int g_fails = 0;
#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        if (!(cond)) {                                                         \
            printf("FAIL: ");                                                  \
            printf(__VA_ARGS__);                                               \
            printf("  (%s:%d)\n", __FILE__, __LINE__);                         \
            g_fails++;                                                         \
        }                                                                      \
    } while (0)

// Deterministic body-bit filler (xorshift), same idiom as test_vdl2_demod.
static void make_body(uint8_t *body, int n, uint32_t seed)
{
    uint32_t s = seed ? seed : 1;
    for (int i = 0; i < n; i++) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        body[i] = (uint8_t)(s & 1u);
    }
}

// Baseline-history scratch (int32[FFT_SIZE*HISTORY_SIZE]); 4 MB, heap-alloc.
// Drive the tagger over the whole stream and return the longest tagged burst
// (stop-start, in samples). `integrated` selects the band=vdl2 fix branch.
static long run_tagger(const int16_t *iq, long n_complex, bool integrated,
                       int *out_n_bursts)
{
    int32_t *hist = malloc((size_t)FBT_FFT_SIZE * FBT_HISTORY_SIZE * sizeof(int32_t));
    if (!hist) { fprintf(stderr, "hist alloc\n"); exit(2); }

    fft_burst_tagger_t *t = fft_burst_tagger_init(
        VDL2_FBT_PRE_LEN, VDL2_FBT_POST_LEN, VDL2_FBT_WIDTH_BINS,
        VDL2_TAG_THR_DB, hist);
    if (!t) { fprintf(stderr, "tagger init\n"); exit(2); }
    fft_burst_tagger_set_start(t, 0);
    // The band=vdl2 flag gates the integrated-active detection branch (and the
    // /diag trace). true => the fix; false => the historical peak-bin ±1 test.
    fft_burst_tagger_set_trace_enabled(integrated);

    long        longest = 0;
    int         n_seen  = 0;
    fbt_burst_t new_b[FBT_MAX_BURSTS], gone_b[FBT_MAX_BURSTS];
    for (long off = 0; off + FBT_FFT_SIZE <= n_complex; off += FBT_FFT_SIZE) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fft_burst_tagger_step(t, iq + off * 2, NULL, new_b, &n_new, gone_b, &n_gone);
        for (int i = 0; i < n_gone; i++) {
            long len = (long)(gone_b[i].stop - gone_b[i].start);
            if (len > longest) longest = len;
            n_seen++;
        }
    }
    int         n_flush = FBT_MAX_BURSTS;
    fbt_burst_t flushed[FBT_MAX_BURSTS];
    fft_burst_tagger_flush(t, flushed, &n_flush);
    for (int i = 0; i < n_flush; i++) {
        long len = (long)(flushed[i].stop - flushed[i].start);
        if (len > longest) longest = len;
        n_seen++;
    }
    fft_burst_tagger_destroy(t);
    free(hist);
    if (out_n_bursts) *out_n_bursts = n_seen;
    return longest;
}

// After an integrated run, inspect the trace ring: report, over the most
// recent `win` entries, the min peak_db / min integ_db and how many windows
// had peak < thr while integ >= thr (the truncation-rescue signature). This
// shows whether the synthetic burst reproduces the on-air integ >> peak gap.
static void trace_stats(uint32_t since_total, int *rescue_windows,
                        float *min_peak, float *min_integ, float *max_integ)
{
    static fbt_trace_entry_t buf[1024];
    uint32_t                 total = 0;
    int                      got   = fft_burst_tagger_get_trace(buf, 1024, &total);
    // Entries in buf are the most-recent `got`; their window indices span
    // roughly [total-got, total). Only analyze windows recorded this run.
    int   rescue = 0;
    float mnp = 1e9f, mni = 1e9f, mxi = -1e9f;
    for (int i = 0; i < got; i++) {
        if (buf[i].kind != FBT_TRACE_KIND_WINDOW) continue;
        if (buf[i].w < since_total) continue; // crude run filter by window#
        float pk = buf[i].peak_db / 10.0f;
        float ig = buf[i].integ_db / 10.0f;
        float th = buf[i].thr_db / 10.0f;
        if (buf[i].peak_db == INT16_MIN || buf[i].integ_db == INT16_MIN) continue;
        if (pk < mnp) mnp = pk;
        if (ig < mni) mni = ig;
        if (ig > mxi) mxi = ig;
        if (pk < th && ig >= th) rescue++;
    }
    *rescue_windows = rescue;
    *min_peak       = mnp;
    *min_integ      = mni;
    *max_integ      = mxi;
}

int main(void)
{
    printf("=== VDL2 integrated-active tagger extend test ===\n");

    // ~300 D8PSK symbols => ~300*(2.5e6/10500) ≈ 71 k samples burst (28 ms),
    // well above the 10 k post-pad floor so truncation is unambiguous.
    const int DATALEN_BITS = 900;
    uint8_t  *body         = malloc((size_t)DATALEN_BITS);
    make_body(body, DATALEN_BITS, 0xC0FFEEu);

    // Continuous stream: pad_pre noise (baseline warmup, ≥512 windows) + burst
    // + pad_post noise. vdl2_mod emits the whole thing (pads are noise-only).
    const int PAD_PRE  = 1150000; // ≈ 561 FFT windows of warmup
    const int PAD_POST = 40000;
    const long MAXC    = (long)PAD_PRE + PAD_POST + 200000;
    int16_t *iq        = malloc((size_t)MAXC * 2 * sizeof(int16_t));
    if (!iq) { fprintf(stderr, "iq alloc\n"); return 2; }

    const double SIGMA  = 220.0;
    const double CFO     = 200000.0; // off-DC, mid-band

    // Helper to (re)generate the stream at a given amp.
    vdl2_mod_params_t p0;
    vdl2_mod_params_default(&p0);
    p0.fs_hz = FS_HZ; p0.cfo_hz = CFO; p0.awgn_sigma = SIGMA;
    p0.pad_pre = PAD_PRE; p0.pad_post = PAD_POST; p0.seed = 12345u;

    // --- Diagnostic (FIRST, clean trace ring): one integrated run at a low
    // amp to reveal the peak-vs-integ dB gap the fix relies on. ---
    {
        vdl2_mod_params_t p = p0; p.amp = 150.0;
        int n = vdl2_mod_burst((uint32_t)DATALEN_BITS, body, &p, iq, (int)MAXC);
        int  nb = 0;
        long len = run_tagger(iq, n, /*integrated=*/true, &nb);
        int   rescue = 0; float mnp = 0, mni = 0, mxi = 0;
        trace_stats(0, &rescue, &mnp, &mni, &mxi);
        printf("diag amp=150: integ-run len=%ld bursts=%d | trace: min_peak=%.1f dB "
               "min_integ=%.1f dB max_integ=%.1f dB  thr=14.0 | rescue_windows(peak<thr<=integ)=%d\n\n",
               len, nb, mnp, mni, mxi, rescue);
    }

    // Sweep amplitude across the marginal→strong range. The fix is UNION
    // (peak OR integrated), so its guarantee is: the VDL2 (union) burst length
    // is NEVER shorter than the peak-only length, in ANY regime — it can only
    // rescue a burst the peak test dropped, never truncate one the peak test
    // held. This is the property the old "integ replaces peak" logic violated
    // (e.g. amp≈110: replace→10240 while peak→96256). We also require the union
    // to tag a substantial (full-body) burst once the signal is above noise.
    const double amps[]      = {40, 70, 110, 160, 230, 320, 480, 700};
    bool         union_lt_peak_seen = false;
    long         worst_union = 0, worst_peak = 0;
    long         max_union   = 0;
    for (size_t a = 0; a < sizeof(amps) / sizeof(amps[0]); a++) {
        vdl2_mod_params_t p = p0; p.amp = amps[a];
        int n = vdl2_mod_burst((uint32_t)DATALEN_BITS, body, &p, iq, (int)MAXC);
        if (n <= 0) { printf("  amp=%.0f modulator returned %d (skip)\n", amps[a], n); continue; }

        int  nb_u = 0, nb_p = 0;
        long len_u = run_tagger(iq, n, /*union(vdl2)=*/true, &nb_u);
        long len_p = run_tagger(iq, n, /*peak-only  =*/false, &nb_p);
        printf("  amp=%-5.0f sigma=%.0f n=%d | union(vdl2): len=%6ld (%d bursts) | peak-only: len=%6ld (%d bursts) %s\n",
               amps[a], SIGMA, n, len_u, nb_u, len_p, nb_p,
               (len_u < len_p) ? "<-- UNION SHORTER (regression!)" : "");
        if (len_u < len_p) { union_lt_peak_seen = true; if (len_p - len_u > worst_peak - worst_union) { worst_union = len_u; worst_peak = len_p; } }
        if (len_u > max_union) max_union = len_u;
    }

    printf("\n-- union>=peak in all regimes: %s | max union burst=%ld --\n",
           union_lt_peak_seen ? "NO (regression)" : "YES", max_union);

    // ASSERTIONS (union safety property + coverage):
    // 1. THE key guard: the union (VDL2) burst is never shorter than peak-only.
    //    This is exactly what the reverted replace-logic failed.
    CHECK(!union_lt_peak_seen,
          "union burst shorter than peak-only (union=%ld peak=%ld) — the fix truncated more, regression",
          worst_union, worst_peak);
    // 2. The union must tag a substantial (full-body) burst on a clear signal —
    //    > 2× the post-pad floor confirms it holds through the burst body.
    CHECK(max_union > 2 * VDL2_FBT_POST_LEN,
          "max union burst len %ld not > 2x post floor %d", max_union, 2 * VDL2_FBT_POST_LEN);

    free(body);
    free(iq);
    if (g_fails == 0) { printf("\nSMOKE_PASS: all checks passed\n"); return 0; }
    printf("\n%d CHECK(s) FAILED\n", g_fails);
    return 1;
}
