// test_tagger_back_to_back.c — does the tagger detect a continuation burst
// that arrives 90 ms after its opener on the SAME bin?
//
// Motivation (2026-07-17 P4/HydraSDR correlation): every P4-received 0x7608
// ACARS opener loses its continuation, yet the HydraSDR shows those
// continuations on air at equal strength, +90 ms (one Iridium TDMA frame) on
// the same frequency, CRC:OK. The continuation-fate counters exonerated the
// worker queue (cont_stale=0 under 6.6k stale-drops), and the hot-bin SNR
// exemption already rescues triage — so the remaining un-instrumented step is
// DETECTION. This test feeds synthetic equal-power burst pairs through the
// production fft_burst_tagger and simply counts what gets tagged.
//
// Scenarios:
//   A  two bursts, same bin, 90 ms start-to-start        -> expect 2 tags
//   B  same bin, 200 ms gap                              -> control
//   C  same bin, 500 ms gap                              -> control
//   D  90 ms gap, second burst on a bin +400 kHz away    -> same-bin isolation
//   E  as A, but a persistent carrier elsewhere keeps n_bursts>0 (EMA frozen)
//      for the whole scenario — the realistic "busy pass" condition
//   F  ARQ storm: 13 bursts at 0.9 s spacing, same bin (real pattern seen at
//      14:13 in the correlation: 13 retransmissions, ALL missed by the P4)
//
// A failure here (missing tags in A/E/F) reproduces the field signature and
// pins the loss at the detector; all-pass pushes the investigation to demod.

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fft_burst_tagger.h"

#define FS 2500000
#define N FBT_FFT_SIZE

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

static int s_passed = 0, s_failed = 0;
#define CHECK(cond, ...)                                   \
    do {                                                   \
        if (!(cond)) {                                     \
            fprintf(stderr, "  FAIL line %d: ", __LINE__); \
            fprintf(stderr, __VA_ARGS__);                  \
            fprintf(stderr, "\n");                         \
            s_failed++;                                    \
        } else {                                           \
            s_passed++;                                    \
        }                                                  \
    } while (0)

// Deterministic 32-bit LCG (identical on every platform — no libc rand).
static uint32_t s_rng = 0x12345678u;
static inline int16_t noise_sample(void)
{
    // Sum of 4 uniforms ≈ gaussian; sigma ≈ 512 in s16 units (the u8→s16
    // <<8 scale the device ingest uses: ~2 LSB of the 8-bit ADC).
    int32_t acc = 0;
    for (int i = 0; i < 4; i++) {
        s_rng = s_rng * 1664525u + 1013904223u;
        acc += (int32_t)(s_rng >> 20) - 2048; // ±2048 uniform
    }
    return (int16_t)(acc / 4);
}

// A burst on the schedule: complex tone at `bin` (DC-centred numbering),
// amplitude `amp`, [start, start+len) in absolute samples.
typedef struct {
    uint64_t start;
    uint64_t len;
    int      bin; // DC-centred (N/2 = DC), matches fbt_burst_t.center_bin
    int16_t  amp;
} synth_burst_t;

// Synthesize one 2048-complex chunk at absolute offset `off`: noise plus any
// scheduled bursts (phase-continuous tones keyed to absolute sample index).
static void synth_chunk(int16_t *iq, uint64_t off, const synth_burst_t *b, int nb)
{
    for (int i = 0; i < N; i++) {
        iq[2 * i + 0] = noise_sample();
        iq[2 * i + 1] = noise_sample();
    }
    for (int k = 0; k < nb; k++) {
        // Overlap of [off, off+N) with the burst
        uint64_t lo = b[k].start, hi = b[k].start + b[k].len;
        uint64_t s  = off > lo ? off : lo;
        uint64_t e  = (off + N) < hi ? (off + N) : hi;
        if (s >= e) continue;
        double w = 2.0 * M_PI * (double)(b[k].bin - N / 2) / (double)N; // rad/sample
        for (uint64_t n = s; n < e; n++) {
            int    i  = (int)(n - off);
            double ph = w * (double)n;
            iq[2 * i + 0] = (int16_t)(iq[2 * i + 0] + (int)(b[k].amp * cos(ph)));
            iq[2 * i + 1] = (int16_t)(iq[2 * i + 1] + (int)(b[k].amp * sin(ph)));
        }
    }
}

// Run one scenario: fresh tagger, prime on noise, then feed the schedule.
// Returns the number of NEW bursts tagged with center_bin within ±3 of
// `want_bin` and start inside/near one of the scheduled windows (loose:
// any tag on the right bin after priming counts — the synth band is
// otherwise empty there). Also reports per-burst tag times for diagnosis.
static int run_scenario(const char *name, const synth_burst_t *sched, int nsched,
                        int want_bin, uint64_t total_samples, int *out_tags_at_want)
{
    static fft_burst_tagger_t *t; // re-init every call (init allocs? check: takes caller history)
    t = fft_burst_tagger_init(/*burst_pre_len=*/2 * N,
                              /*burst_post_len=*/(int)(FS * 16e-3),
                              /*burst_width=*/32,
                              /*threshold_db=*/14.0f, s_baseline_history);
    if (!t) {
        fprintf(stderr, "tagger init failed\n");
        exit(2);
    }
    fft_burst_tagger_set_start(t, 0);
    s_rng = 0x12345678u; // identical noise every scenario

    // Priming preamble: pure noise. FBT_HISTORY_SIZE steps + margin.
    uint64_t prime_samples = (uint64_t)(FBT_HISTORY_SIZE + 32) * N;

    static int16_t iq[2 * N];
    fbt_burst_t    new_b[FBT_MAX_BURSTS], gone_b[FBT_MAX_BURSTS];
    int            tags_want = 0, tags_other = 0;

    for (uint64_t off = 0; off < prime_samples + total_samples; off += N) {
        // Schedule is relative to the END of priming.
        synth_burst_t shifted[8];
        int           nb = nsched < 8 ? nsched : 8;
        for (int k = 0; k < nb; k++) {
            shifted[k] = sched[k];
            shifted[k].start += prime_samples;
        }
        synth_chunk(iq, off, shifted, nb);
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fft_burst_tagger_step(t, iq, /*lookback=*/NULL, new_b, &n_new, gone_b, &n_gone);
        for (int i = 0; i < n_new; i++) {
            int d = new_b[i].center_bin - want_bin;
            if (d < 0) d = -d;
            if (d <= 3) {
                tags_want++;
                printf("    [%s] tag #%d bin=%d mag=%.1f dB noise=%.1f dB start=%llu (t=%.1f ms after priming)\n",
                       name, tags_want, new_b[i].center_bin,
                       (double)new_b[i].magnitude_db, (double)new_b[i].noise_db,
                       (unsigned long long)new_b[i].start,
                       ((double)new_b[i].start - (double)prime_samples) / FS * 1000.0);
            } else {
                tags_other++;
            }
        }
    }
    printf("  [%s] tags on target bin: %d (elsewhere: %d)\n", name, tags_want, tags_other);
    if (out_tags_at_want) *out_tags_at_want = tags_other;
    fft_burst_tagger_destroy(t);
    return tags_want;
}

int main(void)
{
    const int      BIN   = N / 2 + 328;                  // ≈ +400 kHz from DC
    const int      BIN2  = N / 2 + 656;                  // ≈ +800 kHz (control D)
    const int      BIN_E = N / 2 - 400;                  // carrier for scenario E
    const uint64_t BLEN  = (uint64_t)(FS * 8.3e-3);      // 8.3 ms Iridium burst
    const uint64_t MS90  = (uint64_t)(FS * 90e-3);
    const int16_t  AMP   = 3000;                         // ~18-20 dB over sigma-512 floor

    printf("=== tagger back-to-back detection (production params, thr=14 dB) ===\n");

    // A: same bin, 90 ms start-to-start
    {
        synth_burst_t s[] = {{0, BLEN, BIN, AMP}, {MS90, BLEN, BIN, AMP}};
        int           got = run_scenario("A:90ms-same-bin", s, 2, BIN, MS90 + BLEN + FS / 2, NULL);
        CHECK(got == 2, "A: expected 2 tags, got %d — 90 ms same-bin continuation MISSED", got);
    }
    // B: 200 ms gap
    {
        uint64_t      g   = (uint64_t)(FS * 200e-3);
        synth_burst_t s[] = {{0, BLEN, BIN, AMP}, {g, BLEN, BIN, AMP}};
        int           got = run_scenario("B:200ms-same-bin", s, 2, BIN, g + BLEN + FS / 2, NULL);
        CHECK(got == 2, "B: expected 2 tags, got %d", got);
    }
    // C: 500 ms gap
    {
        uint64_t      g   = (uint64_t)(FS * 500e-3);
        synth_burst_t s[] = {{0, BLEN, BIN, AMP}, {g, BLEN, BIN, AMP}};
        int           got = run_scenario("C:500ms-same-bin", s, 2, BIN, g + BLEN + FS / 2, NULL);
        CHECK(got == 2, "C: expected 2 tags, got %d", got);
    }
    // D: 90 ms gap, different bin — isolates same-bin effects
    {
        synth_burst_t s[]  = {{0, BLEN, BIN, AMP}, {MS90, BLEN, BIN2, AMP}};
        int           got1 = run_scenario("D:90ms-diff-bin(first)", s, 2, BIN, MS90 + BLEN + FS / 2, NULL);
        int           got2 = run_scenario("D:90ms-diff-bin(second)", s, 2, BIN2, MS90 + BLEN + FS / 2, NULL);
        CHECK(got1 == 1, "D: expected 1 tag on bin1, got %d", got1);
        CHECK(got2 == 1, "D: expected 1 tag on bin2, got %d", got2);
    }
    // E: A + persistent carrier elsewhere (n_bursts>0 → EMA frozen throughout)
    {
        uint64_t      span = MS90 + BLEN + FS / 2;
        synth_burst_t s[]  = {{0, BLEN, BIN, AMP},
                              {MS90, BLEN, BIN, AMP},
                              {0, span, BIN_E, AMP}}; // carrier the whole time
        int           got  = run_scenario("E:90ms-busy-band", s, 3, BIN, span, NULL);
        CHECK(got == 2, "E: expected 2 tags with busy band, got %d — EMA-freeze interaction", got);
    }
    // F: ARQ storm — 13 bursts, 0.9 s apart, same bin (the real 14:13 pattern)
    {
        synth_burst_t s[8]; // schedule capacity is 8/chunk; run in two batches of 7+6
        int           total = 0;
        for (int batch = 0; batch < 2; batch++) {
            int nb = batch == 0 ? 7 : 6;
            for (int k = 0; k < nb; k++) {
                s[k].start = (uint64_t)((batch * 7 + k) * 0.9 * FS) - (uint64_t)(batch * 7 * 0.9 * FS);
                s[k].len   = BLEN;
                s[k].bin   = BIN;
                s[k].amp   = AMP;
            }
            char label[32];
            snprintf(label, sizeof(label), "F:ARQ-storm-b%d", batch);
            total += run_scenario(label, s, nb, BIN,
                                  (uint64_t)(nb * 0.9 * FS) + FS / 2, NULL);
        }
        CHECK(total == 13, "F: expected 13 tags across storm, got %d", total);
    }

    printf("\n=== %d passed, %d failed ===\n", s_passed, s_failed);
    return s_failed > 0 ? 1 : 0;
}
