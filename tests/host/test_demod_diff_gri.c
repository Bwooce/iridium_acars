// test_demod_diff_gri.c — per-burst demod A/B against gr-iridium on a
// real wideband capture.
//
// Input: a directory produced by tests/scripts/demod_diff_extract.py:
//   manifest.csv                — one row per gr-iridium burst line
//   burst_<NNNNN>.sc16          — 250 ksps int16 IQ window per burst
//
// For every burst we run the SAME burst_pipeline_process_burst() the
// P4 worker runs (multi-frame, D13 running normally — no force-start),
// classify each decoded frame, run ida_decode on LW.DA frames, and
// attribute the outcome to a stage:
//   SHORT    guard bailed (window too short)
//   NO_UW    pipeline ran, UW correlator never locked, no frame
//   NO_DEMOD UW found but qpsk_demod produced no frame
//   NO_DA    frames decoded but none classified LW.DA (IDA rows only)
//   DA_BCH   LW.DA frame, payload BCH blocks failed
//   DA_HDR   BCH ok, header sanity (zero1) failed
//   DA_CRC   header ok, CRC-16 failed
//   DA_OK    full clean decode (== device lw_da_valid criterion)
//
// Decisive output: of the bursts gr-iridium decoded CRC:OK, the
// fraction WE decode, stratified by gri's SNR estimate. A smooth
// falloff with SNR where we get ~all strong bursts = SNR-edge; flat
// losses at good SNR = a systematic demod gap (the stage column then
// names the suspect).
//
// Usage: test_demod_diff_gri <dir> [--csv] [--dumpbits <file>] [--ida-only]
//   --csv prints one row per burst (stdout); the summary tables always
//   print at the end.
//   --dumpbits <file>: for every IDA-row burst, write one line per
//     demodulated frame with the RAW demod bits and per-bit soft
//     metrics (phase-0 bit-error analysis; see
//     tests/scripts/demod_diff_biterr.py):
//       B,<idx>,<frame_no>,<n_bits>,<dir>,<bitstring>,<soft;soft;...>
//   --ida-only: process only manifest rows with type == IDA (fast
//     ablation runs; the all-types demod table is then meaningless).

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "burst_pipeline.h"
#include "ida_decode.h"
#include "iridium_frame.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"

#define MAX_COMPLEX 16384

typedef struct {
    int    idx;
    char   type[8];
    double time_ms;
    long   abs_freq_hz;
    long   off_hz;
    double snr_db;
    int    conf_pct;
    int    gri_crc_ok;
    int    gri_cont, gri_ctr, gri_len;
    int    n_complex;
    char   filename[64];
} man_entry_t;

typedef enum {
    ST_SHORT = 0,
    ST_NO_UW,
    ST_NO_DEMOD,
    ST_NO_DA,
    ST_DA_BCH,
    ST_DA_HDR,
    ST_DA_CRC,
    ST_DA_OK,
    ST_N
} stage_t;

static const char *stage_name[ST_N] = {
    "SHORT", "NO_UW", "NO_DEMOD", "NO_DA",
    "DA_BCH", "DA_HDR", "DA_CRC", "DA_OK",
};

// Per-burst collection filled by the frame callback.
#define DUMP_MAX_FRAMES 8
typedef struct {
    int  n_frames;
    char first_type[8]; // classify name of first decoded frame
    // Best LW.DA seen in this burst (best = furthest stage).
    bool          saw_da;
    ida_decoded_t best_ida;
    int           best_rank; // 0 none, 1 bch-fail, 2 hdr-fail, 3 crc-fail, 4 ok
    // --dumpbits capture: copies of each demodulated frame's raw bits
    // + soft metrics (callback owns/frees the originals).
    bool     capture;
    int      n_kept;
    uint8_t *kept_bits[DUMP_MAX_FRAMES];
    int16_t *kept_soft[DUMP_MAX_FRAMES];
    int      kept_nbits[DUMP_MAX_FRAMES];
    int      kept_dir[DUMP_MAX_FRAMES]; // 0=DL 1=UL
} frames_ctx_t;

static int ida_rank(const ida_decoded_t *ida)
{
    if (!ida->ok) return 1;
    if (!ida->header_ok) return 2;
    if (!ida->crc_ok) return 3;
    return 4;
}

static void on_frame(burst_pipeline_result_t *res, void *ctx_)
{
    frames_ctx_t *ctx = (frames_ctx_t *)ctx_;
    ctx->n_frames++;

    iridium_frame_t      f;
    ir_frame_direction_t dir = (res->uw_res.direction == UW_DIR_UPLINK)
                                   ? IR_FRM_DIR_UPLINK
                                   : IR_FRM_DIR_DOWNLINK;
    if (iridium_frame_classify(res->frame.bits, res->frame.n_bits, dir, &f) == 0) {
        const char *tn = iridium_frame_type_name(f.type);
        if (f.type == IR_FRAME_LW)
            tn = iridium_lw_subtype_name(f.lw_subtype);
        if (ctx->n_frames == 1)
            snprintf(ctx->first_type, sizeof(ctx->first_type), "%s", tn);
        if (f.type == IR_FRAME_LW && f.lw_subtype == IR_LW_DA) {
            ida_decoded_t ida = {0};
            if (ida_decode(&f, &ida) == 0) {
                int r = ida_rank(&ida);
                if (r > ctx->best_rank) {
                    ctx->best_rank = r;
                    ctx->best_ida  = ida;
                }
                ctx->saw_da = true;
            }
        }
    } else if (ctx->n_frames == 1) {
        snprintf(ctx->first_type, sizeof(ctx->first_type), "??");
    }
    if (ctx->capture && ctx->n_kept < DUMP_MAX_FRAMES && res->frame.bits) {
        int k              = ctx->n_kept++;
        int nb             = res->frame.n_bits;
        ctx->kept_nbits[k] = nb;
        ctx->kept_dir[k]   = (res->frame.direction == DIR_UPLINK) ? 1 : 0;
        ctx->kept_bits[k]  = malloc(nb);
        memcpy(ctx->kept_bits[k], res->frame.bits, nb);
        ctx->kept_soft[k] = NULL;
        if (res->frame.soft_bits) {
            ctx->kept_soft[k] = malloc(nb * sizeof(int16_t));
            memcpy(ctx->kept_soft[k], res->frame.soft_bits, nb * sizeof(int16_t));
        }
    }
    free(res->frame.bits);
    free(res->frame.soft_bits); // #112
}

static int load_manifest(const char *path, man_entry_t **out)
{
    FILE *fh = fopen(path, "r");
    if (!fh) {
        fprintf(stderr, "manifest missing: %s\n", path);
        return -1;
    }
    int          cap = 8192, n = 0;
    man_entry_t *e   = malloc(cap * sizeof(*e));
    char         line[512];
    if (!fgets(line, sizeof(line), fh)) { // header
        fclose(fh);
        free(e);
        return 0;
    }
    while (fgets(line, sizeof(line), fh)) {
        if (n == cap) {
            cap *= 2;
            e = realloc(e, cap * sizeof(*e));
        }
        man_entry_t *m = &e[n];
        if (sscanf(line, "%d,%7[^,],%lf,%ld,%ld,%lf,%d,%d,%d,%d,%d,%d,%63s",
                   &m->idx, m->type, &m->time_ms, &m->abs_freq_hz, &m->off_hz,
                   &m->snr_db, &m->conf_pct, &m->gri_crc_ok, &m->gri_cont,
                   &m->gri_ctr, &m->gri_len, &m->n_complex, m->filename) == 13)
            n++;
    }
    fclose(fh);
    *out = e;
    return n;
}

// SNR strata (gri-estimated dB).
#define N_BINS 5
static int snr_bin(double snr)
{
    if (snr < 18) return 0;
    if (snr < 22) return 1;
    if (snr < 26) return 2;
    if (snr < 30) return 3;
    return 4;
}
static const char *bin_name[N_BINS] = {"<18", "18-22", "22-26", "26-30", ">=30"};

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <extract-dir> [--csv]\n", argv[0]);
        return 2;
    }
    const char *dir       = argv[1];
    bool        emit_csv  = false;
    bool        ida_only  = false;
    FILE       *dump_fh   = NULL;
    for (int a = 2; a < argc; a++) {
        if (strcmp(argv[a], "--csv") == 0) {
            emit_csv = true;
        } else if (strcmp(argv[a], "--ida-only") == 0) {
            ida_only = true;
        } else if (strcmp(argv[a], "--no-harder") == 0) {
            iridium_frame_classify_set_harder(false);
        } else if (strcmp(argv[a], "--dumpbits") == 0 && a + 1 < argc) {
            dump_fh = fopen(argv[++a], "w");
            if (!dump_fh) {
                fprintf(stderr, "cannot open dumpbits file %s\n", argv[a]);
                return 2;
            }
        } else {
            fprintf(stderr, "unknown arg: %s\n", argv[a]);
            return 2;
        }
    }

    char man_path[512];
    snprintf(man_path, sizeof(man_path), "%s/manifest.csv", dir);
    man_entry_t *entries = NULL;
    int          n       = load_manifest(man_path, &entries);
    if (n <= 0) return 2;
    fprintf(stderr, "loaded %d manifest rows from %s\n", n, man_path);

    if (emit_csv)
        printf("idx,type,snr_db,gri_crc_ok,stage,n_frames,first_type,"
               "burst_start,uw_snr_db,omega,blocks_ok,da_ctr,da_len,da_crc\n");

    static int16_t bufA[2 * MAX_COMPLEX], bufB[2 * MAX_COMPLEX];
    static int16_t raw[2 * MAX_COMPLEX];

    // Decisive table: rows where gri decoded IDA CRC:OK.
    int gold_tot[N_BINS] = {0}, gold_stage[N_BINS][ST_N] = {{0}};
    // Secondary: IDA rows gri demodulated but CRC-failed ("---").
    int nak_tot[N_BINS] = {0}, nak_ok[N_BINS] = {0};
    // Demod-level (any type): did we decode >=1 frame from the window?
    int any_tot[N_BINS] = {0}, any_frames[N_BINS] = {0};
    int skipped = 0;

    for (int i = 0; i < n; i++) {
        man_entry_t *m = &entries[i];
        if (m->n_complex == 0 || strcmp(m->filename, "SKIP") == 0) {
            skipped++;
            continue;
        }
        if (ida_only && strcmp(m->type, "IDA") != 0) continue;
        char path[600];
        snprintf(path, sizeof(path), "%s/%s", dir, m->filename);
        FILE *fh = fopen(path, "rb");
        if (!fh) {
            skipped++;
            continue;
        }
        int nc = (int)fread(raw, 2 * sizeof(int16_t), MAX_COMPLEX, fh);
        fclose(fh);
        if (nc < 64) {
            skipped++;
            continue;
        }
        memcpy(bufA, raw, (size_t)nc * 2 * sizeof(int16_t));
        memcpy(bufB, raw, (size_t)nc * 2 * sizeof(int16_t));

        // Pass 1 — legacy single-frame API purely for stage diagnostics
        // (uw lock, D13 start, coarse omega). It mutates bufA.
        burst_pipeline_result_t res;
        memset(&res, 0, sizeof(res));
        bool ran = burst_pipeline_process_250khz(bufA, nc, &res);
        if (res.demod_ok) {
            free(res.frame.bits);
            free(res.frame.soft_bits);
        }
        bool have_uw = (res.uw_res.direction != UW_DIR_UNKNOWN);

        // Pass 2 — the production multi-frame path on a pristine copy.
        frames_ctx_t ctx = {0};
        snprintf(ctx.first_type, sizeof(ctx.first_type), "-");
        ctx.capture = (dump_fh != NULL && strcmp(m->type, "IDA") == 0);
        int frames  = burst_pipeline_process_burst(bufB, nc, on_frame, &ctx);

        stage_t st;
        if (!ran)
            st = ST_SHORT;
        else if (frames == 0)
            st = have_uw ? ST_NO_DEMOD : ST_NO_UW;
        else if (!ctx.saw_da)
            st = ST_NO_DA;
        else if (ctx.best_rank <= 1)
            st = ST_DA_BCH;
        else if (ctx.best_rank == 2)
            st = ST_DA_HDR;
        else if (ctx.best_rank == 3)
            st = ST_DA_CRC;
        else
            st = ST_DA_OK;

        if (ctx.capture) {
            // Per-burst header line, then one B-line per demod frame.
            fprintf(dump_fh, "H,%d,%s,%d,%.2f,%d\n", m->idx,
                    stage_name[st], m->gri_crc_ok, m->snr_db, ctx.n_kept);
            for (int k = 0; k < ctx.n_kept; k++) {
                fprintf(dump_fh, "B,%d,%d,%d,%d,", m->idx, k,
                        ctx.kept_nbits[k], ctx.kept_dir[k]);
                for (int j = 0; j < ctx.kept_nbits[k]; j++)
                    fputc('0' + (ctx.kept_bits[k][j] & 1), dump_fh);
                fputc(',', dump_fh);
                if (ctx.kept_soft[k]) {
                    for (int j = 0; j < ctx.kept_nbits[k]; j++)
                        fprintf(dump_fh, "%d%c", (int)ctx.kept_soft[k][j],
                                j + 1 == ctx.kept_nbits[k] ? '\n' : ';');
                } else {
                    fputc('\n', dump_fh);
                }
            }
        }
        for (int k = 0; k < ctx.n_kept; k++) {
            free(ctx.kept_bits[k]);
            free(ctx.kept_soft[k]);
        }

        int b = snr_bin(m->snr_db);
        any_tot[b]++;
        if (frames > 0) any_frames[b]++;
        bool is_ida = (strcmp(m->type, "IDA") == 0);
        if (is_ida && m->gri_crc_ok) {
            gold_tot[b]++;
            gold_stage[b][st]++;
        } else if (is_ida) {
            nak_tot[b]++;
            if (st == ST_DA_OK) nak_ok[b]++;
        }

        if (emit_csv)
            printf("%d,%s,%.2f,%d,%s,%d,%s,%d,%.1f,%+.4f,%d,%d,%d,%d\n",
                   m->idx, m->type, m->snr_db, m->gri_crc_ok, stage_name[st],
                   frames, ctx.first_type, res.burst_start,
                   res.uw_res.snr_estimate_db, res.omega_coarse,
                   ctx.saw_da ? ctx.best_ida.blocks_ok : -1,
                   ctx.saw_da ? ctx.best_ida.da_ctr : -1,
                   ctx.saw_da ? ctx.best_ida.da_len : -1,
                   ctx.saw_da ? (int)ctx.best_ida.crc_ok : -1);
    }

    // ---- summary tables (stderr so --csv output stays clean) ----
    fprintf(stderr, "\n== DECISIVE: gri IDA CRC:OK bursts — our outcome by gri SNR ==\n");
    fprintf(stderr, "%6s %5s %6s %7s | %5s %5s %8s %5s %6s %6s %6s\n",
            "snr", "n", "DA_OK", "ours%", "SHORT", "NO_UW", "NO_DEMOD",
            "NO_DA", "DA_BCH", "DA_HDR", "DA_CRC");
    int gt = 0, gk = 0;
    for (int b = 0; b < N_BINS; b++) {
        int ok = gold_stage[b][ST_DA_OK];
        gt += gold_tot[b];
        gk += ok;
        fprintf(stderr, "%6s %5d %6d %6.1f%% | %5d %5d %8d %5d %6d %6d %6d\n",
                bin_name[b], gold_tot[b], ok,
                gold_tot[b] ? 100.0 * ok / gold_tot[b] : 0.0,
                gold_stage[b][ST_SHORT], gold_stage[b][ST_NO_UW],
                gold_stage[b][ST_NO_DEMOD], gold_stage[b][ST_NO_DA],
                gold_stage[b][ST_DA_BCH], gold_stage[b][ST_DA_HDR],
                gold_stage[b][ST_DA_CRC]);
    }
    fprintf(stderr, "%6s %5d %6d %6.1f%%\n", "TOTAL", gt, gk,
            gt ? 100.0 * gk / gt : 0.0);

    fprintf(stderr, "\n== gri IDA CRC-FAIL bursts we fully decode (SAFETY: false-accepts) ==\n");
    int nak_tot_all = 0, nak_ok_all = 0;
    for (int b = 0; b < N_BINS; b++) {
        fprintf(stderr, "%6s %5d gri-nak, ours DA_OK %4d\n",
                bin_name[b], nak_tot[b], nak_ok[b]);
        nak_tot_all += nak_tot[b];
        nak_ok_all += nak_ok[b];
    }
    fprintf(stderr, "%6s %5d gri-nak, ours DA_OK %4d  <-- MUST be 0\n",
            "TOTAL", nak_tot_all, nak_ok_all);

    fprintf(stderr, "\n== all burst types: windows yielding >=1 demod frame ==\n");
    for (int b = 0; b < N_BINS; b++)
        fprintf(stderr, "%6s %5d bursts, frames>=1: %5d (%.1f%%)\n",
                bin_name[b], any_tot[b], any_frames[b],
                any_tot[b] ? 100.0 * any_frames[b] / any_tot[b] : 0.0);

    fprintf(stderr, "\nskipped (edge/short/missing): %d\n", skipped);
    if (dump_fh) fclose(dump_fh);
    free(entries);
    return 0;
}
