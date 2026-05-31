// test_burst_pipeline_mendeley.c — feed Mendeley dataset bursts through
// burst_pipeline_process_burst and tally decodes by SNR.
//
// Source: Oligeri/Sciancalepore dataset (DOI 10.17632/xcxspv8c2r.2). The
// committed fixture (fixture_mendeley_burst_sample.h, ~1 MB) holds a
// balanced 100-burst slice with SNR span 0–30+ dB across multiple Iridium
// satellites and beam IDs. Built by tests/scripts/build_mendeley_fixture.py.
//
// What this catches that the in-repo ALBQ tests don't:
//   - 100× the satellite-ID diversity (one Iridium plane vs 6)
//   - 100× the Doppler / pointing geometry variety (Doha, 2 months)
//   - SNR coverage well below ALBQ's ~30 dB ceiling
//
// Run with `ctest` like any other host test. Skips cleanly if the
// generated fixture isn't present (CI without the cache).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "burst_pipeline.h"

#if __has_include("fixture_mendeley_burst_sample.h")
#  include "fixture_mendeley_burst_sample.h"
#  define HAVE_FIXTURE 1
#else
#  define HAVE_FIXTURE 0
#endif

#if HAVE_FIXTURE

// SNR-bucket boundaries (dB) for reporting.
static const float SNR_BUCKETS[] = {  0.0f,  5.0f, 10.0f, 15.0f, 20.0f, 25.0f, 30.0f, 40.0f };
#define N_BUCKETS  (sizeof(SNR_BUCKETS) / sizeof(SNR_BUCKETS[0]) - 1)

static int bucket_idx(float snr_db) {
    for (int i = 0; i < (int)N_BUCKETS; i++) {
        if (snr_db < SNR_BUCKETS[i + 1]) return i;
    }
    return (int)N_BUCKETS - 1;
}

int main(int argc, char **argv)
{
    (void)argc; (void)argv;

    printf("Mendeley burst pipeline test\n");
    printf("  fixture: %u bursts, %u IQ samples total\n",
           (unsigned)MENDELEY_FIXTURE_N_BURSTS,
           (unsigned)MENDELEY_FIXTURE_TOTAL_IQ_SAMPLES);

    int per_bucket_total[N_BUCKETS]   = {0};
    int per_bucket_demod[N_BUCKETS]   = {0};
    int per_bucket_bch[N_BUCKETS]     = {0};
    int total_demod = 0, total_bch = 0;

    for (uint32_t i = 0; i < MENDELEY_FIXTURE_N_BURSTS; i++) {
        const mendeley_burst_meta_t *m = &mendeley_meta[i];
        float snr_db = (float)m->snr_db_q4 / 16.0f;
        int b = bucket_idx(snr_db);
        per_bucket_total[b]++;

        // The fixture IQ is at whatever rate the dataset was captured
        // at (TBD — set after inspecting the actual data). We rotate
        // through burst_pipeline_process_burst which expects 250 ksps
        // int16 IQ; the resample step is the caller's responsibility
        // when these don't match. For now run it directly and accept
        // mismatch may lower decode rates — the test still surfaces
        // pipeline-level changes via the trend, not the absolute count.
        int16_t *iq = (int16_t *)&mendeley_iq[m->offset_iq * 2];
        // Make a writable copy because burst_pipeline does in-place
        // operations (DC removal, pre-rotation).
        int n = (int)m->n_complex;
        int16_t *work = malloc((size_t)n * 2 * sizeof(int16_t));
        if (!work) { fprintf(stderr, "OOM\n"); return 2; }
        memcpy(work, iq, (size_t)n * 2 * sizeof(int16_t));

        burst_pipeline_result_t res;
        memset(&res, 0, sizeof(res));
        bool ok = burst_pipeline_process_250khz(work, n, &res);
        if (ok && res.demod_ok) {
            per_bucket_demod[b]++;
            total_demod++;
            if (res.frame.n_bits >= 24 + 64 && res.bch_block1_ok && res.bch_block2_ok) {
                per_bucket_bch[b]++;
                total_bch++;
            }
        }
        if (res.frame.bits)      free(res.frame.bits);
        if (res.frame.soft_bits) free(res.frame.soft_bits);
        free(work);
    }

    printf("\nSNR-bucket breakdown (total / demod_ok / bch_ok):\n");
    for (int i = 0; i < (int)N_BUCKETS; i++) {
        if (per_bucket_total[i] == 0) continue;
        printf("  [%5.1f .. %5.1f) dB:  %3d / %3d / %3d  (%.0f%% demod, %.0f%% bch)\n",
               (double)SNR_BUCKETS[i], (double)SNR_BUCKETS[i + 1],
               per_bucket_total[i], per_bucket_demod[i], per_bucket_bch[i],
               100.0 * per_bucket_demod[i] / per_bucket_total[i],
               100.0 * per_bucket_bch[i] / per_bucket_total[i]);
    }
    printf("\nOverall: demod_ok = %d / %u (%.0f%%); bch_ok = %d / %u (%.0f%%)\n",
           total_demod, (unsigned)MENDELEY_FIXTURE_N_BURSTS,
           100.0 * total_demod / MENDELEY_FIXTURE_N_BURSTS,
           total_bch, (unsigned)MENDELEY_FIXTURE_N_BURSTS,
           100.0 * total_bch / MENDELEY_FIXTURE_N_BURSTS);

    // PASS criterion is INTENTIONALLY weak for now — we don't have a
    // calibration of what to expect on this fixture. The point of this
    // test today is to surface regressions via the BUCKET BREAKDOWN
    // when something changes, not to gate merges on a tight number.
    //
    // Once we've established a baseline on a few firmware revisions,
    // tighten this (e.g. require ≥ 80% bch_ok at SNR ≥ 15 dB).
    if (total_demod == 0) {
        printf("FAIL: zero demod successes — pipeline appears broken on this fixture\n");
        return 1;
    }
    printf("PASS\n");
    return 0;
}

#else  // !HAVE_FIXTURE

int main(void) {
    printf("SKIP: fixture_mendeley_burst_sample.h not generated.\n");
    printf("Run: python3 tests/scripts/fetch_mendeley_iridium.py\n");
    printf("     python3 tests/scripts/build_mendeley_fixture.py\n");
    return 0;
}

#endif
