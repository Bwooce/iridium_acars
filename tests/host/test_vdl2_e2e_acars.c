// test_vdl2_e2e_acars — V3 ACCEPTANCE: the FULL VDL2 receive chain on a
// real over-the-air capture, demod -> RS(255,249) -> AVLC -> libacars,
// asserting it reproduces the dumpvdl2 golden ACARS set.
//
// Input: the sigidwiki VDL2 capture at 210 ksps S16_LE (channel-order
// corrected), /Users/bruce/iridium_capture/vdl2_ref/
// vdl2_sigidwiki_210k_s16le.raw (override: env VDL2_RAW_PATH). SKIPs
// (exit 0) when absent — the capture-dependent-test convention.
//
// Ground truth: dumpvdl2 2.6.0's decode of the SAME file
// (vdl2_sigidwiki_golden_decode.txt): 40 AVLC frames, 9 ACARS
// messages. The expected ACARS registrations/modes come from
// fixture_vdl2_avlc_golden.h (generated from that golden decode), so
// this test and the golden can never drift apart silently.
//
// Chain under test (all PRODUCTION sources, no test doubles):
//   vdl2_demod_burst   (common/vdl2_decoder/vdl2_demod.c)  D8PSK demod
//   vdl2_l2_feed       (common/vdl2_decoder/vdl2_l2.c)     RS de-interleave
//                                                          + correct + AVLC
//   la_acars_parse_and_reassemble (vendored libacars)      ACARS parse
// with the direction derived from the AVLC source address type exactly
// as the firmware's vdl2_avlc_cb does (dumpvdl2 src/acars.c:100-108).
//
// Floors: ALL 9 golden ACARS messages matched by (registration,
// mode), and >= 40 FCS-valid AVLC frames (= dumpvdl2's own count).
//
// Measured at V3 integration (2026-07-22): 8/9 ACARS matched (missed:
// the LN-RPA label-SA Media Advisory), 26 FCS-valid AVLC frames —
// demod-quality-bound, marginal bursts at EVM 0.15-0.2 rad aliased
// the short 2-parity RS blocks and FCS arbitrated them out.
// Measured at V4 (2026-07-22, sharp 144-tap channel filter +
// fractional symbol timing — demod EVM median 0.182 -> 0.128 rad):
// 9/9 golden ACARS matched INCLUDING the Media Advisory, 43 FCS-valid
// AVLC frames (three MORE than the dumpvdl2 golden's 40), 10 ACARS
// parsed (the extra: an HB-IJW label-A9 uplink dumpvdl2 did not
// decode), 0 L2 failures, 2 bad-FCS frames. The 9/9 floor is exact —
// reproducing the full golden set IS the acceptance criterion; the
// FCS floor sits at the oracle's 40 with the 3-frame surplus as
// slack.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include <libacars/acars.h>
#include <libacars/libacars.h>
#include <libacars/reassembly.h>

#include "avlc.h"
#include "firmr_s16.h"
#include "fixture_vdl2_avlc_golden.h"
#include "vdl2_demod.h"
#include "vdl2_l2.h"

#define RAW_PATH_DEFAULT \
    "/Users/bruce/iridium_capture/vdl2_ref/vdl2_sigidwiki_210k_s16le.raw"
#define FS_RAW 210000

static int g_fails = 0;
#define CHECK(cond, ...)                                \
    do {                                                \
        if (!(cond)) {                                  \
            printf("FAIL %s:%d: ", __func__, __LINE__); \
            printf(__VA_ARGS__);                        \
            printf("\n");                               \
            g_fails++;                                  \
        }                                               \
    } while (0)

// ---- 210 k -> 250 k front door (same as test_vdl2_real_capture) ----
#define UP_DSIZE 12
static int16_t s_up_coeffs[UP_DSIZE * 25];

static int resample_210_to_250(const int16_t *in, int n_in, int16_t *out,
                               int max_out)
{
    firmr_s16_t fi, fq;
    int16_t     di[UP_DSIZE], dq[UP_DSIZE];
    firmr_s16_init(&fi, s_up_coeffs, di, UP_DSIZE, 25, 21, 0, 0);
    firmr_s16_init(&fq, s_up_coeffs, dq, UP_DSIZE, 25, 21, 0, 0);
    int16_t in_i[256], in_q[256], out_i[512], out_q[512];
    int     n_out = 0;
    for (int base = 0; base < n_in; base += 256) {
        int len = n_in - base;
        if (len > 256) len = 256;
        for (int k = 0; k < len; k++) {
            // Halve: full-scale input + Q15 ripple would wrap int16.
            in_i[k] = (int16_t)(in[2 * (base + k) + 0] / 2);
            in_q[k] = (int16_t)(in[2 * (base + k) + 1] / 2);
        }
        int ni = (int)firmr_s16_process(&fi, in_i, out_i, len);
        int nq = (int)firmr_s16_process(&fq, in_q, out_q, len);
        int n  = (ni < nq) ? ni : nq;
        if (n_out + n > max_out) n = max_out - n_out;
        for (int k = 0; k < n; k++) {
            out[2 * (n_out + k) + 0] = out_i[k];
            out[2 * (n_out + k) + 1] = out_q[k];
        }
        n_out += n;
        if (n_out >= max_out) break;
    }
    return n_out;
}

// ---- decoded-ACARS collector ----

#define MAX_ACARS 32
typedef struct {
    char reg[16];
    char mode;
    char label[3];
    int  claimed; // greedy golden matching
} dec_acars_t;

static dec_acars_t  s_dec[MAX_ACARS];
static int          s_n_dec       = 0;
static int          s_avlc_ok     = 0; // FCS-valid frames, all kinds
static int          s_avlc_acars  = 0;
static int          s_avlc_badfcs = 0;
static la_reasm_ctx *s_reasm      = NULL;

// Walk the proto tree for the ACARS payload node (the frame_decoder.c
// find_acars_msg idiom).
extern la_type_descriptor const la_DEF_acars_message;
static la_acars_msg            *find_acars_msg(la_proto_node *node)
{
    while (node) {
        if (node->td == &la_DEF_acars_message && node->data)
            return (la_acars_msg *)node->data;
        node = node->next;
    }
    return NULL;
}

static void avlc_cb(const avlc_frame_t *f, void *ctx)
{
    (void)ctx;
    if (f->fcs_ok) s_avlc_ok++;
    if (f->kind == AVLC_KIND_BAD_FCS) s_avlc_badfcs++;
    if (f->kind != AVLC_KIND_ACARS) return;
    s_avlc_acars++;

    // Direction from the source address type — the firmware rule
    // (frame_decoder.c vdl2_avlc_cb; dumpvdl2 src/acars.c:100-108).
    la_msg_dir dir = (f->src_type == AVLC_ADDRTYPE_AIRCRAFT)
                         ? LA_MSG_DIR_AIR2GND
                         : LA_MSG_DIR_GND2AIR;
    struct timeval rx_time;
    gettimeofday(&rx_time, NULL);
    la_proto_node *node = la_acars_parse_and_reassemble(
        f->acars, (size_t)f->acars_len, dir, s_reasm, rx_time);
    if (!node) return;
    la_acars_msg *a = find_acars_msg(node);
    if (a && (a->reasm_status == LA_REASM_COMPLETE ||
              a->reasm_status == LA_REASM_SKIPPED) &&
        s_n_dec < MAX_ACARS) {
        const char *reg = a->reg;
        while (*reg == '.')
            reg++; // strip libacars's '.' left-padding
        snprintf(s_dec[s_n_dec].reg, sizeof(s_dec[s_n_dec].reg), "%s", reg);
        s_dec[s_n_dec].mode     = a->mode;
        s_dec[s_n_dec].label[0] = a->label[0];
        s_dec[s_n_dec].label[1] = a->label[1];
        s_dec[s_n_dec].label[2] = '\0';
        s_dec[s_n_dec].claimed  = 0;
        printf("  ACARS %s: reg=%s mode=%c label=%s crc=%s\n",
               dir == LA_MSG_DIR_AIR2GND ? "DL" : "UL", s_dec[s_n_dec].reg,
               a->mode ? a->mode : '?', s_dec[s_n_dec].label,
               a->crc_ok ? "OK" : "BAD");
        s_n_dec++;
    }
    la_proto_tree_destroy(node);
}

int main(void)
{
    const char *path = getenv("VDL2_RAW_PATH");
    if (!path || !path[0]) path = RAW_PATH_DEFAULT;
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        printf("SKIP: real capture not present (%s)\n", path);
        return 0;
    }
    fseek(fp, 0, SEEK_END);
    long bytes = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    int      n210 = (int)(bytes / 4);
    int16_t *raw  = (int16_t *)malloc((size_t)n210 * 2 * sizeof(int16_t));
    if (!raw || (long)fread(raw, 4, (size_t)n210, fp) != (long)n210) {
        printf("FAIL: could not read %s\n", path);
        fclose(fp);
        return 1;
    }
    fclose(fp);

    if (!vdl2_lpf_design_q15(s_up_coeffs, UP_DSIZE, 25,
                             60000.0 / ((double)FS_RAW * 25), 8.0)) {
        printf("FAIL: upsampler design alloc\n");
        return 1;
    }
    int      max250 = (int)((int64_t)n210 * 25 / 21) + 8;
    int16_t *iq250  = (int16_t *)malloc((size_t)max250 * 2 * sizeof(int16_t));
    int      n250   = resample_210_to_250(raw, n210, iq250, max250);
    free(raw);
    printf("capture: %d complex @ 250 kHz (%.1f s)\n", n250, n250 / 250000.0);

    s_reasm = la_reasm_ctx_new();
    if (!s_reasm) {
        printf("FAIL: la_reasm_ctx_new\n");
        return 1;
    }

    // Windowed scan (the test_vdl2_real_capture pattern: 2 s windows,
    // 0.25 s step-back so a window-straddling burst is retried whole).
    const int WIN = 500000, OVERLAP = 62500;
    int cursor = 0, n_phy = 0, n_l2_fail = 0;
    while (cursor < n250 - 1000) {
        int len = n250 - cursor;
        if (len > WIN) len = WIN;
        vdl2_demod_result_t r;
        if (!vdl2_demod_burst(iq250 + 2 * (size_t)cursor, len, &r)) {
            if (len < WIN) break;
            cursor += WIN - OVERLAP;
            continue;
        }
        if (r.complete) {
            n_phy++;
            int rc = vdl2_l2_feed(r.bits, r.n_bits, avlc_cb, NULL);
            if (rc < 0) n_l2_fail++;
        }
        free(r.bits);
        free(r.soft_bits);
        cursor += r.consumed_complex_250k;
    }
    free(iq250);

    // Golden expectations: the fixture's ACARS rows (reg + mode), which
    // build_vdl2_avlc_fixture.py generated from the dumpvdl2 golden.
    int n_golden = 0, n_matched = 0;
    int total = (int)(sizeof(k_vdl2_avlc_golden) / sizeof(k_vdl2_avlc_golden[0]));
    for (int gi = 0; gi < total; gi++) {
        const vdl2_avlc_golden_t *g = &k_vdl2_avlc_golden[gi];
        if (!g->is_acars) continue;
        n_golden++;
        const char *greg = g->acars_reg;
        while (*greg == '.')
            greg++;
        for (int d = 0; d < s_n_dec; d++) {
            if (!s_dec[d].claimed && strcmp(s_dec[d].reg, greg) == 0 &&
                s_dec[d].mode == g->acars_mode) {
                s_dec[d].claimed = 1;
                n_matched++;
                break;
            }
        }
    }

    printf("SUMMARY: phy %d (l2_fail %d), avlc fcs_ok %d (acars %d, "
           "bad_fcs %d), acars parsed %d, golden %d, matched %d\n",
           n_phy, n_l2_fail, s_avlc_ok, s_avlc_acars, s_avlc_badfcs, s_n_dec,
           n_golden, n_matched);

    // The golden decode carries exactly 9 ACARS in 40 AVLC frames.
    CHECK(n_golden == 9, "golden fixture ACARS count %d != 9", n_golden);
    CHECK(s_avlc_ok >= 40, "FCS-valid AVLC frames %d < 40 (= the dumpvdl2 "
                           "golden count; measured 43 at V4 — see header)",
          s_avlc_ok);
    // THE acceptance floor: every golden ACARS message reproduced
    // end-to-end (registration + mode matched).
    CHECK(n_matched >= 9, "golden ACARS matched %d < 9 (of %d)", n_matched,
          n_golden);

    if (g_fails) {
        printf("FAIL: %d check(s)\n", g_fails);
        return 1;
    }
    printf("PASS: vdl2 end-to-end ACARS acceptance (%d/%d golden matched)\n",
           n_matched, n_golden);
    return 0;
}
