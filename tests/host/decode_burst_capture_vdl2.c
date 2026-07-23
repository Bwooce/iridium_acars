// decode_burst_capture_vdl2.c — replay a .u8 burst-capture file from the
// device through the host VDL Mode 2 decode chain.
//
// Sibling of decode_burst_capture.c (the Iridium replay tool): identical
// file parsing and per-burst front end (rotate to DC at rel_freq_hz +
// direct_if_decim 10× → 250 ksps, exactly what worker_core1 does), but
// instead of the Iridium burst_pipeline it feeds each 250 ksps burst to
// the VDL2 pipeline:
//
//   vdl2_demod_burst  (common/vdl2_decoder/vdl2_demod.c)  D8PSK demod
//                     — input contract is 250 ksps IQ centred at DC
//                       (VDL2_FS_IN_HZ); the demod resamples 250k→105k
//                       internally. This matches the device path
//                       2.5M → (worker 10× decim) → 250k → vdl2.
//   vdl2_l2_feed      (common/vdl2_decoder/vdl2_l2.c)  RS(255,249)
//                     de-interleave + correct + AVLC deframe
//   la_acars_parse_and_reassemble (vendored libacars)  ACARS parse
//
// The per-burst demod loop mirrors vdl2_pipeline.c's vdl2_process_burst
// (back-to-back CSMA frames within one tagger window), and the AVLC→
// libacars glue mirrors test_vdl2_e2e_acars.c's avlc_cb (direction from
// the AVLC source-address type, dumpvdl2 src/acars.c:100-108).
//
// Purpose: given a device burst capture where the live VDL2 demod synced
// only a fraction of tagged bursts, decide whether the non-syncers are
// (a) real VDL2 the host decoder CAN sync (→ a live demod gap) or (b)
// not decodable VDL2 at all (→ other in-band signals / genuinely
// marginal). The host decoder here is the SAME production source the
// firmware runs, so a materially higher host sync rate implicates the
// live path (config/windowing/decim), not the demod algorithm.
//
// Output: one CSV row per burst + a summary (sync/avlc/acars totals and
// the SNR distribution of synced-vs-not). Optional --dump <dir> writes
// each burst's raw 2.5 MSPS int16 IQ (pre-rotation, as captured) to
// <dir>/burst_<seq>.cs16 for a dumpvdl2 cross-check.
//
// Build via tests/host/CMakeLists.txt target decode_burst_capture_vdl2.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <sys/time.h>

#include <libacars/acars.h>
#include <libacars/libacars.h>
#include <libacars/reassembly.h>

#include "direct_if_decim.h"
#include "rotate_to_dc.h"
#include "vdl2_demod.h"
#include "vdl2_l2.h"
#include "avlc.h"

#define SD_CAPTURE_BURST_MAGIC 0x54535242u /* "BRST" little-endian */

// Must match sd_capture_burst_hdr_t in sd_capture.h.
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;
    uint64_t t_us;
    uint32_t length_samples;
    float    rel_freq_hz;
    float    peak_snr_db;
    float    magnitude_db;
    float    noise_db;
    uint32_t reserved;
} burst_hdr_t;

#define INPUT_FS_HZ 2500000
#define VDL2_MAX_FRAMES_PER_WINDOW 8

extern la_type_descriptor const la_DEF_acars_message;
static la_acars_msg *find_acars_msg(la_proto_node *node)
{
    while (node) {
        if (node->td == &la_DEF_acars_message && node->data)
            return (la_acars_msg *)node->data;
        node = node->next;
    }
    return NULL;
}

// Per-burst accumulator, handed to avlc_cb via vdl2_l2_feed's ctx.
typedef struct {
    la_reasm_ctx *reasm;
    int           avlc_ok;   // FCS-valid AVLC frames (all kinds)
    int           avlc_acars;
    int           acars;     // ACARS messages parsed (complete/skipped)
    char          detail[512]; // "flight|label|txt; ..." for the CSV
} burst_ctx_t;

static void csv_escape_append(char *dst, size_t dstsz, const char *src)
{
    // Append src to dst, replacing characters that would break the CSV
    // line (comma, quote, CR/LF) with spaces. Best-effort, bounded.
    size_t len = strlen(dst);
    for (const char *p = src; *p && len + 2 < dstsz; p++) {
        char c = *p;
        if (c == ',' || c == '"' || c == '\n' || c == '\r' || c == '\t')
            c = ' ';
        if (c < 0x20 || (unsigned char)c > 0x7e)
            c = '.';
        dst[len++] = c;
    }
    dst[len] = '\0';
}

static void avlc_cb(const avlc_frame_t *f, void *ctx_)
{
    burst_ctx_t *bc = (burst_ctx_t *)ctx_;
    if (f->fcs_ok) bc->avlc_ok++;
    if (f->kind != AVLC_KIND_ACARS) return;
    bc->avlc_acars++;

    la_msg_dir dir = (f->src_type == AVLC_ADDRTYPE_AIRCRAFT)
                         ? LA_MSG_DIR_AIR2GND
                         : LA_MSG_DIR_GND2AIR;
    struct timeval rx_time;
    gettimeofday(&rx_time, NULL);
    la_proto_node *node = la_acars_parse_and_reassemble(
        f->acars, (size_t)f->acars_len, dir, bc->reasm, rx_time);
    if (!node) return;
    la_acars_msg *a = find_acars_msg(node);
    if (a && (a->reasm_status == LA_REASM_COMPLETE ||
              a->reasm_status == LA_REASM_SKIPPED)) {
        bc->acars++;
        const char *reg = a->reg;
        while (*reg == '.')
            reg++; // strip libacars '.' left-padding
        const char *fid = a->flight_id;
        while (*fid == '.')
            fid++;

        char one[400];
        char txtbuf[300];
        const char *txt = a->txt ? a->txt : "";
        // Copy at most a chunk of the (possibly long) text field.
        snprintf(txtbuf, sizeof(txtbuf), "%.256s", txt);
        snprintf(one, sizeof(one), "%s[%s] reg=%s flt=%s lbl=%.2s crc=%s txt='%s'",
                 bc->detail[0] ? " || " : "",
                 dir == LA_MSG_DIR_AIR2GND ? "DL" : "UL",
                 reg, fid, a->label, a->crc_ok ? "OK" : "BAD", txtbuf);
        csv_escape_append(bc->detail, sizeof(bc->detail), one);

        // Also echo a human line to stderr so the run log carries the
        // full (un-escaped) decode.
        fprintf(stderr,
                "  ACARS %s: reg=%s flt=%s mode=%c lbl=%.2s blk=%c "
                "msgno=%.4s crc=%s txt=\"%s\"\n",
                dir == LA_MSG_DIR_AIR2GND ? "DL" : "UL", reg, fid,
                a->mode ? a->mode : '?', a->label,
                a->block_id ? a->block_id : '?', a->msg_num,
                a->crc_ok ? "OK" : "BAD", txt);
    }
    la_proto_tree_destroy(node);
}

// SNR bucket helper for the synced-vs-not distribution.
#define SNR_LO 10
#define SNR_HI 32
#define SNR_NBUCKETS (SNR_HI - SNR_LO + 1)
static int snr_bucket(float snr)
{
    int b = (int)lroundf(snr) - SNR_LO;
    if (b < 0) b = 0;
    if (b >= SNR_NBUCKETS) b = SNR_NBUCKETS - 1;
    return b;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <capture.u8> [--dump <dir>]\n", argv[0]);
        return 2;
    }
    const char *dump_dir = NULL;
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--dump") == 0 && i + 1 < argc)
            dump_dir = argv[++i];
    }

    FILE *fp = fopen(argv[1], "rb");
    if (!fp) {
        fprintf(stderr, "fopen(%s): %s\n", argv[1], strerror(errno));
        return 1;
    }

    printf("seq,rel_freq_hz,peak_snr_db,synced,header_ok,rs_ok,rs_fixed,"
           "rs_eras,avlc_ok,acars,acars_detail\n");

    direct_if_decim_t dec;
    direct_if_decim_init(&dec);

    const int MAX_IN  = 700000;
    const int MAX_OUT = MAX_IN / DIDECIM_DECIM;
    int16_t  *iq25    = malloc((size_t)MAX_IN * 2 * sizeof(int16_t));
    int16_t  *iq25_raw = malloc((size_t)MAX_IN * 2 * sizeof(int16_t)); // pre-rotation copy for --dump
    int16_t  *iq250   = malloc((size_t)MAX_OUT * 2 * sizeof(int16_t));
    int16_t  *si      = malloc((size_t)MAX_IN * sizeof(int16_t));
    int16_t  *sq      = malloc((size_t)MAX_IN * sizeof(int16_t));
    int16_t  *so      = malloc((size_t)MAX_OUT * sizeof(int16_t));
    int16_t  *sp      = malloc((size_t)MAX_OUT * sizeof(int16_t));
    if (!iq25 || !iq25_raw || !iq250 || !si || !sq || !so || !sp) {
        fprintf(stderr, "out of memory\n");
        return 1;
    }

    la_reasm_ctx *reasm = la_reasm_ctx_new();
    if (!reasm) {
        fprintf(stderr, "la_reasm_ctx_new() failed\n");
        return 1;
    }

    int bursts_seen = 0, bursts_too_short = 0;
    int n_synced = 0, n_header_ok = 0, n_avlc_bursts = 0, n_acars_bursts = 0;
    int tot_synced_frames = 0, tot_complete_frames = 0;
    int tot_avlc_ok = 0, tot_acars = 0;

    int synced_hist[SNR_NBUCKETS] = {0};
    int notsync_hist[SNR_NBUCKETS] = {0};

    while (1) {
        burst_hdr_t h;
        size_t      got = fread(&h, 1, sizeof(h), fp);
        if (got == 0) break;
        if (got != sizeof(h)) {
            fprintf(stderr, "short header read (got %zu) — file truncated\n", got);
            break;
        }
        if (h.magic != SD_CAPTURE_BURST_MAGIC) {
            fprintf(stderr, "bad magic 0x%08x at burst %d — stop\n",
                    h.magic, bursts_seen);
            break;
        }
        if (h.length_samples == 0 || (int)h.length_samples > MAX_IN) {
            fprintf(stderr, "burst seq=%u: implausible length=%u — stop\n",
                    h.seq, h.length_samples);
            break;
        }
        size_t need = (size_t)h.length_samples * 2 * sizeof(int16_t);
        if (fread(iq25, 1, need, fp) != need) {
            fprintf(stderr, "burst seq=%u: short IQ read — file truncated\n",
                    h.seq);
            break;
        }
        bursts_seen++;

        if (dump_dir) {
            memcpy(iq25_raw, iq25, need);
            char path[512];
            snprintf(path, sizeof(path), "%s/burst_%u.cs16", dump_dir, h.seq);
            FILE *df = fopen(path, "wb");
            if (df) {
                fwrite(iq25_raw, 1, need, df);
                fclose(df);
            } else {
                fprintf(stderr, "  (dump) fopen(%s): %s\n", path, strerror(errno));
            }
        }

        // Front end — identical to decode_burst_capture.c: rotate the
        // tagged channel to DC, then 10× decimate to 250 ksps.
        double phase_step = -2.0 * M_PI * (double)h.rel_freq_hz / (double)INPUT_FS_HZ;
        rotate_to_dc_q15_simd(iq25, (int)h.length_samples, phase_step);

        direct_if_decim_reset_state(&dec);
        int n250 = direct_if_decim_process_split(&dec,
                                                 iq25, (int)h.length_samples,
                                                 iq250, si, sq, so, sp);
        if (n250 <= 64) {
            bursts_too_short++;
            printf("%u,%.0f,%.1f,0,0,0,0,0,0,\n",
                   h.seq, h.rel_freq_hz, h.peak_snr_db);
            notsync_hist[snr_bucket(h.peak_snr_db)]++;
            continue;
        }

        // VDL2 demod loop — mirrors vdl2_pipeline.c::vdl2_process_burst.
        burst_ctx_t     bc = {0};
        bc.reasm           = reasm;
        vdl2_l2_stats_t l2_before, l2_after;
        vdl2_l2_get_stats(&l2_before);

        int synced = 0, header_ok = 0;
        int cursor = 0;
        for (int fr = 0; fr < VDL2_MAX_FRAMES_PER_WINDOW; fr++) {
            vdl2_demod_result_t res;
            if (!vdl2_demod_burst(iq250 + 2 * cursor, n250 - cursor, &res))
                break;
            synced++;
            tot_synced_frames++;
            if (res.complete) {
                header_ok++;
                tot_complete_frames++;
                // Feed complete transmissions to L2 (RS + AVLC + ACARS).
                vdl2_l2_feed(res.bits, res.soft_bits, res.n_bits, avlc_cb, &bc);
            }
            free(res.bits);
            free(res.soft_bits);
            int adv = res.consumed_complex_250k;
            if (adv < 1) adv = 1;
            cursor += adv;
            if (cursor >= n250) break;
        }

        vdl2_l2_get_stats(&l2_after);
        unsigned rs_ok    = l2_after.rs_blocks_ok - l2_before.rs_blocks_ok;
        unsigned rs_fixed = l2_after.rs_octets_fixed - l2_before.rs_octets_fixed;
        unsigned rs_eras  = l2_after.rs_erasure_recovered - l2_before.rs_erasure_recovered;

        if (synced)       n_synced++;
        if (header_ok)    n_header_ok++;
        if (bc.avlc_ok)   n_avlc_bursts++;
        if (bc.acars)     n_acars_bursts++;
        tot_avlc_ok += bc.avlc_ok;
        tot_acars   += bc.acars;

        if (synced)
            synced_hist[snr_bucket(h.peak_snr_db)]++;
        else
            notsync_hist[snr_bucket(h.peak_snr_db)]++;

        printf("%u,%.0f,%.1f,%d,%d,%u,%u,%u,%d,%d,%s\n",
               h.seq, h.rel_freq_hz, h.peak_snr_db,
               synced ? 1 : 0, header_ok ? 1 : 0, rs_ok, rs_fixed, rs_eras,
               bc.avlc_ok, bc.acars, bc.detail);
    }

    fclose(fp);
    la_reasm_ctx_destroy(reasm);

    fprintf(stderr,
            "\n== %s VDL2 summary ==\n"
            "  bursts                : %d\n"
            "  too short (<64 @250k) : %d\n"
            "  synced (>=1 lock)     : %d (%.1f%%)\n"
            "  header/complete burst : %d\n"
            "  bursts w/ AVLC FCS-ok : %d\n"
            "  bursts w/ ACARS       : %d\n"
            "  ---- frame totals ----\n"
            "  demod locks           : %d\n"
            "  complete frames       : %d\n"
            "  AVLC FCS-ok frames    : %d\n"
            "  ACARS parsed          : %d\n",
            argv[1], bursts_seen, bursts_too_short,
            n_synced, bursts_seen ? 100.0 * n_synced / bursts_seen : 0.0,
            n_header_ok, n_avlc_bursts, n_acars_bursts,
            tot_synced_frames, tot_complete_frames, tot_avlc_ok, tot_acars);

    fprintf(stderr, "\n  SNR distribution (peak_snr_db, per burst):\n");
    fprintf(stderr, "   dB :  synced  not-synced\n");
    for (int b = 0; b < SNR_NBUCKETS; b++) {
        if (synced_hist[b] == 0 && notsync_hist[b] == 0) continue;
        int db = SNR_LO + b;
        fprintf(stderr, "   %2d%c: %6d  %10d\n",
                db, (b == SNR_NBUCKETS - 1) ? '+' : ' ',
                synced_hist[b], notsync_hist[b]);
    }

    return 0;
}
