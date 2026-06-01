// test_direct_if_decim.c — validate the C direct-IF decimator against
// the Python scipy implementation that drives path-C.
//
// Path-C generates per-burst 250 ksps cf32 files at
// /tmp/host_direct_if/burst_NNN.cf32 via scipy.signal.resample_poly
// with the same gri-aligned Kaiser FIR (141 taps, trans=40kHz). The
// C decimator in direct_if_decim.c implements the same filter. For
// ONE specific burst, we:
//   1. Read the path-C-generated cf32 (Python's "ground truth")
//   2. Re-derive what the C decimator would produce on the same
//      pre-rotated raw input (load that from raw_cf32 dump)
//   3. Compare NMSE
//
// Because Python and C may differ by ≤1 LSB on rounding details,
// expect NMSE around -50 dB or better.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "direct_if_decim.h"

#define FS_RAW 2500000
#define DECIM DIDECIM_DECIM

// Manifest entry (parsed from /tmp/host_direct_if/manifest.csv)
typedef struct {
    int    idx;
    int    gri_id;
    double timestamp_ms;
    long   abs_freq_hz;
    long   freq_offset_hz;
    int    n_samples_250k;
    int    confidence_pct;
    char   filename[64];
} manifest_t;

static int parse_manifest_first(const char *path, manifest_t *m)
{
    FILE *fh = fopen(path, "r");
    if (!fh) return -1;
    char line[512];
    if (!fgets(line, sizeof(line), fh)) {
        fclose(fh);
        return -1;
    } // header
    if (!fgets(line, sizeof(line), fh)) {
        fclose(fh);
        return -1;
    } // first row
    int ok = sscanf(line, "%d,%d,%lf,%ld,%ld,%d,%d,%63s",
                    &m->idx, &m->gri_id, &m->timestamp_ms,
                    &m->abs_freq_hz, &m->freq_offset_hz,
                    &m->n_samples_250k, &m->confidence_pct, m->filename);
    fclose(fh);
    return (ok == 8) ? 0 : -1;
}

static int load_cf32(const char *path, float **out, int *n_complex)
{
    FILE *fh = fopen(path, "rb");
    if (!fh) return -1;
    fseek(fh, 0, SEEK_END);
    long bytes = ftell(fh);
    fseek(fh, 0, SEEK_SET);
    int    n   = (int)(bytes / 8); // 2 × float32
    float *buf = (float *)malloc(2 * n * sizeof(float));
    if (!buf) {
        fclose(fh);
        return -1;
    }
    size_t got = fread(buf, sizeof(float), 2 * n, fh);
    fclose(fh);
    if ((int)got != 2 * n) {
        free(buf);
        return -1;
    }
    *out       = buf;
    *n_complex = n;
    return 0;
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    // 1) Load manifest, pick burst 0.
    manifest_t m;
    if (parse_manifest_first("/tmp/host_direct_if/manifest.csv", &m) < 0) {
        fprintf(stderr,
                "Could not read /tmp/host_direct_if/manifest.csv.\n"
                "Run: python3 tests/scripts/direct_if_dump.py\n");
        return 2;
    }
    printf("Using path-C burst %d (gri_id=%d, ts=%.2f ms, "
           "freq_offset=%ld Hz)\n",
           m.idx, m.gri_id, m.timestamp_ms, m.freq_offset_hz);

    // 2) Load the Python-generated 250 ksps reference output.
    char ref_path[256];
    snprintf(ref_path, sizeof(ref_path),
             "/tmp/host_direct_if/%s", m.filename);
    float *ref   = NULL;
    int    n_ref = 0;
    if (load_cf32(ref_path, &ref, &n_ref) < 0) {
        fprintf(stderr, "Could not read %s\n", ref_path);
        return 2;
    }
    printf("Python reference: n=%d (250 ksps)\n", n_ref);

    // 3) Load the pre-rotation 2.5 MSPS cf32 (the input to the
    //    Python rotate+decim) — direct_if_dump.py saved this as
    //    fixture_albq_raw_2500k.cf32.
    float *raw   = NULL;
    int    n_raw = 0;
    if (load_cf32("/tmp/host_direct_if/fixture_albq_raw_2500k.cf32",
                  &raw, &n_raw) < 0) {
        fprintf(stderr, "Could not read fixture_albq_raw_2500k.cf32\n");
        return 2;
    }

    // 4) Slice + rotate (same math the Python does): start at
    //    timestamp - 4096 raw samples, length = PRE_SAMPLES + POST.
    int start_sample = (int)(m.timestamp_ms * 1e-3 * FS_RAW + 0.5);
    int begin        = start_sample - 4096;
    if (begin < 0) begin = 0;
    int end = begin + 44096;
    if (end > n_raw) end = n_raw;
    int      slice_n = end - begin;
    int16_t *iq_pre  = (int16_t *)malloc(2 * slice_n * sizeof(int16_t));
    if (!iq_pre) {
        free(raw);
        free(ref);
        return 2;
    }

    const double TWO_PI     = 6.28318530717958647693;
    double       phase_step = -TWO_PI * (double)m.freq_offset_hz / (double)FS_RAW;
    for (int i = 0; i < slice_n; i++) {
        double phase = phase_step * (double)(begin + i);
        double c = cos(phase), s = sin(phase);
        double in_r  = raw[(begin + i) * 2 + 0];
        double in_i  = raw[(begin + i) * 2 + 1];
        double out_r = in_r * c - in_i * s;
        double out_i = in_r * s + in_i * c;
        // Q15 conversion: cf32 ~[-1, +1] → int16 × 32768
        out_r *= 32768.0;
        out_i *= 32768.0;
        if (out_r > 32767) out_r = 32767;
        if (out_r < -32768) out_r = -32768;
        if (out_i > 32767) out_i = 32767;
        if (out_i < -32768) out_i = -32768;
        iq_pre[2 * i + 0] = (int16_t)lrint(out_r);
        iq_pre[2 * i + 1] = (int16_t)lrint(out_i);
    }

    // 5) Run the C decimator.
    direct_if_decim_t dec;
    direct_if_decim_init(&dec);

    int      max_out = slice_n / DECIM + 8;
    int16_t *iq_post = (int16_t *)malloc(2 * max_out * sizeof(int16_t));
    if (!iq_post) {
        free(iq_pre);
        free(raw);
        free(ref);
        return 2;
    }
    int n_post = direct_if_decim_process(&dec, iq_pre, slice_n, iq_post);
    printf("C decimator produced n=%d output complex samples\n", n_post);

    // 6) Compare against Python reference. The two should be very
    //    similar but not bit-identical (scipy uses float64, we use
    //    int32 accumulator + Q15 truncation).
    int    n_cmp    = n_post < n_ref ? n_post : n_ref;
    double sum_err2 = 0, sum_ref2 = 0;
    int    max_err_re = 0, max_err_im = 0;
    for (int i = 0; i < n_cmp; i++) {
        // Convert reference cf32 to the C's Q15 scale.
        double ref_re = ref[2 * i + 0] * 32768.0;
        double ref_im = ref[2 * i + 1] * 32768.0;
        double err_re = (double)iq_post[2 * i + 0] - ref_re;
        double err_im = (double)iq_post[2 * i + 1] - ref_im;
        sum_err2 += err_re * err_re + err_im * err_im;
        sum_ref2 += ref_re * ref_re + ref_im * ref_im;
        if (fabs(err_re) > max_err_re) max_err_re = (int)fabs(err_re);
        if (fabs(err_im) > max_err_im) max_err_im = (int)fabs(err_im);
    }
    double nmse_db = (sum_ref2 > 0)
                         ? 10.0 * log10(sum_err2 / sum_ref2)
                         : 1e9;
    printf("\nComparison vs Python scipy reference:\n");
    printf("  n_cmp: %d samples\n", n_cmp);
    printf("  NMSE: %.2f dB (limit -40 dB)\n", nmse_db);
    printf("  max bin error: re=%d  im=%d\n", max_err_re, max_err_im);

    free(iq_post);
    free(iq_pre);
    free(raw);
    free(ref);

    if (nmse_db < -40.0) {
        printf("\n[pass] C decimator matches Python within precision budget\n");
        return 0;
    }
    printf("\n[FAIL] NMSE above -40 dB threshold\n");
    return 1;
}
