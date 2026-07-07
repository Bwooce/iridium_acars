// test_phaseb_cut.c — validate the device decode DSP chain against a
// known-good real IQ slice.
//
// Feeds a runtime 2.56 MSPS int16 IQ file (produced by
// tests/scripts/phaseb_cut_to_256.py from a phaseb_cuts cut) through
// the EXACT device ingest+decode chain:
//
//   resample_256_to_250 (2.56 → 2.5 MSPS, firmware ingest)
//     → fft_burst_tagger (detection)
//     → per burst: rotate_to_dc + direct_if_decim (→250 ksps)
//     → burst_pipeline_process_burst (D13/CFO/RRC/UW/PLL/demod, multi-frame)
//     → iridium_frame_classify
//     → for LW.DA: ida_decode → acars_tail_feed (SBD reassembly + libacars)
//
// This is test_pipeline_wideband_resampled's ingest+tagger+pipeline
// harness spliced onto decode_burst_capture's IDA→SBD→ACARS tail, so a
// single run answers: does OUR chain recover the manifest-proven ACARS
// message from a real capture?
//
// Usage: test_phaseb_cut <file_256.ci16> [expect_substr]
//   Exits 0 if (expect_substr absent) or (an emitted ACARS/SBD line or
//   flight id contains expect_substr).

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include "fft_burst_tagger.h"
#include "fft_sc16_2048.h"
#include "direct_if_decim.h"
#include "resample_256_to_250.h"
#include "rotate_to_dc.h"
#include "burst_pipeline.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"
#include "iridium_frame.h"
#include "ida_decode.h"
#include "sbd_reassembler.h"
#include "acars_tail.h"
#include <libacars/reassembly.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define INPUT_FS_HZ 2500000
#define BURST_PRE_LEN (2 * FBT_FFT_SIZE)
#define BURST_POST_LEN ((int)(INPUT_FS_HZ * 16e-3))
#define BURST_WINDOW_LEN ((int)(INPUT_FS_HZ * 250 / 1000)) // 625000
#define BURST_WINDOW_250K (BURST_WINDOW_LEN / DIDECIM_DECIM)

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

typedef struct {
    sbd_reassembler_t *sbd;
    ida_reassembler_t *ida_reasm;
    la_reasm_ctx      *reasm;
    uint32_t           burst_freq_hz;
    // counters
    int frames_decoded, frames_lw, sbd_messages, acars_decoded;
    // hit tracking
    const char *expect;
    int         hit;
} decode_ctx_t;

static void print_hex(const uint8_t *buf, size_t n)
{
    for (size_t i = 0; i < n; i++)
        printf("%02x", buf[i]);
}

static void on_frame(burst_pipeline_result_t *res, void *ctx_)
{
    decode_ctx_t *ctx = (decode_ctx_t *)ctx_;
    ctx->frames_decoded++;

    iridium_frame_t      f;
    ir_frame_direction_t dir = (res->uw_res.direction == UW_DIR_UPLINK)
                                   ? IR_FRM_DIR_UPLINK
                                   : IR_FRM_DIR_DOWNLINK;
    if (iridium_frame_classify(res->frame.bits, res->frame.n_bits, dir, &f) == 0 &&
        f.type == IR_FRAME_LW) {
        ctx->frames_lw++;
        if (f.lw_subtype == IR_LW_DA) {
            ida_decoded_t ida    = {0};
            int           rc_ida = ida_decode(&f, &ida);
            if (getenv("PHASEB_VERBOSE")) {
                printf("  IDA freq=%uHz rc=%d ok=%d hdr=%d crc=%d ctr=%u cont=%u len=%u pay=",
                       ctx->burst_freq_hz, rc_ida, ida.ok, ida.header_ok,
                       ida.crc_ok, ida.da_ctr, ida.da_cont, ida.payload_len);
                print_hex(ida.payload, ida.payload_len);
                printf("\n");
            }
            if (rc_ida == 0 && ida.ok && ida.header_ok && ida.crc_ok) {
                acars_tail_result_t tail;
                int                 rc_tail = acars_tail_feed(ctx->sbd, ctx->reasm,
                                                              ctx->ida_reasm, &ida,
                                                              dir == IR_FRM_DIR_UPLINK,
                                                              ctx->burst_freq_hz,
                                                              0 /*t_us*/, &tail);
                if (rc_tail == 1 && tail.sbd_ready) {
                    ctx->sbd_messages++;
                    printf("SBD: type=%s %s len=%u (msg %u/%u) payload=",
                           sbd_type_wire_name(tail.sbd.type),
                           tail.sbd.uplink ? "UL" : "DL",
                           tail.sbd.payload_len, tail.sbd.msg_no,
                           tail.sbd.msg_count);
                    print_hex(tail.sbd.payload, tail.sbd.payload_len);
                    printf("\n");
                    // Registration/text of a demand-mode ACK lives in the
                    // SBD payload, not libacars flight_id — match there too.
                    if (ctx->expect) {
                        char hex[2 * 320 + 1];
                        for (int i = 0; i < tail.sbd.payload_len && i < 320; i++)
                            sprintf(hex + 2 * i, "%02x", tail.sbd.payload[i]);
                        if (strstr(hex, ctx->expect)) ctx->hit = 1;
                    }
                    if (tail.acars_ready) {
                        ctx->acars_decoded++;
                        printf("ACARS: %s mode=%c label='%.2s' block=%c "
                               "msgnum='%.4s' flight='%.6s' crc=%s txt=\"%s\"\n",
                               tail.sbd.uplink ? "UL" : "DL",
                               tail.mode ? tail.mode : '?',
                               tail.label, tail.block_id ? tail.block_id : '?',
                               tail.msg_num, tail.flight_id,
                               tail.crc_ok ? "OK" : "BAD", tail.txt);
                        if (ctx->expect &&
                            (strstr(tail.flight_id, ctx->expect) ||
                             strstr(tail.txt, ctx->expect)))
                            ctx->hit = 1;
                    }
                }
            }
        }
    }
    free(res->frame.bits);
    free(res->frame.soft_bits);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <file_256.ci16> [expect_substr]\n", argv[0]);
        return 2;
    }
    const char *path   = argv[1];
    const char *expect = (argc >= 3) ? argv[2] : NULL;

    FILE *fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "fopen(%s): %s\n", path, strerror(errno));
        return 1;
    }
    fseek(fp, 0, SEEK_END);
    long bytes = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    int      n_raw = (int)(bytes / 4); // complex int16 samples at 2.56 MSPS
    int16_t *iq256 = malloc((size_t)n_raw * 2 * sizeof(int16_t));
    if (!iq256) {
        fprintf(stderr, "oom iq256\n");
        return 1;
    }
    if (fread(iq256, sizeof(int16_t), (size_t)n_raw * 2, fp) != (size_t)n_raw * 2) {
        fprintf(stderr, "short read\n");
        return 1;
    }
    fclose(fp);
    printf("Loaded %d complex @ 2.56 MSPS (%.3f s)\n", n_raw, (double)n_raw / 2.56e6);

    // resample 2.56 → 2.5 MSPS (firmware ingest)
    int      n_resamp_max = (int)((double)n_raw * 125.0 / 128.0 + 16);
    int16_t *iq25         = malloc((size_t)n_resamp_max * 2 * sizeof(int16_t));
    if (!iq25) {
        fprintf(stderr, "oom iq25\n");
        return 1;
    }
    resample_256_to_250_t rs;
    resample_256_to_250_init(&rs);
    int n25 = resample_256_to_250_process(&rs, iq256, n_raw, iq25);
    printf("Resampled: %d complex @ 2.5 MSPS\n", n25);

    // Tagger
    fft_burst_tagger_t *t = fft_burst_tagger_init(BURST_PRE_LEN, BURST_POST_LEN,
                                                  /*burst_width=*/32,
                                                  /*threshold_db=*/14.0f,
                                                  s_baseline_history);
    if (!t) {
        fprintf(stderr, "tagger init\n");
        return 1;
    }
    fft_burst_tagger_set_start(t, 0);

    typedef struct {
        uint64_t start, stop;
        int      center_bin;
    } tag_t;
    enum { MAX_TAGS = 8192 };
    tag_t      *tags   = malloc(sizeof(tag_t) * MAX_TAGS);
    int         n_tags = 0;
    fbt_burst_t new_b[FBT_MAX_BURSTS], gone_b[FBT_MAX_BURSTS];
    for (int off = 0; off + FBT_FFT_SIZE <= n25; off += FBT_FFT_SIZE) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fft_burst_tagger_step(t, iq25 + off * 2, NULL, new_b, &n_new, gone_b, &n_gone);
        for (int i = 0; i < n_gone && n_tags < MAX_TAGS; i++) {
            tags[n_tags].start      = gone_b[i].start;
            tags[n_tags].stop       = gone_b[i].stop;
            tags[n_tags].center_bin = gone_b[i].center_bin;
            n_tags++;
        }
    }
    {
        int         n_flush = FBT_MAX_BURSTS;
        fbt_burst_t flushed[FBT_MAX_BURSTS];
        fft_burst_tagger_flush(t, flushed, &n_flush);
        for (int i = 0; i < n_flush && n_tags < MAX_TAGS; i++) {
            tags[n_tags].start      = flushed[i].start;
            tags[n_tags].stop       = flushed[i].stop;
            tags[n_tags].center_bin = flushed[i].center_bin;
            n_tags++;
        }
    }
    printf("Tagger emitted %d bursts\n", n_tags);

    // Per-burst decode chain
    direct_if_decim_t dec;
    direct_if_decim_init(&dec);
    int16_t *w25  = malloc(2 * BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *w250 = malloc(2 * BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *si   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *sq   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *so   = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *sp   = malloc(BURST_WINDOW_250K * sizeof(int16_t));

    sbd_reassembler_t sbd;
    sbd_reassembler_init(&sbd);
    ida_reassembler_t ida_reasm;
    ida_reassembler_init(&ida_reasm);
    la_reasm_ctx *reasm = la_reasm_ctx_new();
    if (!reasm) {
        fprintf(stderr, "la_reasm_ctx_new\n");
        return 1;
    }

    int pipeline_ok = 0, uw_found = 0, frames_total = 0;
    int t_frames_lw = 0, t_sbd = 0, t_acars = 0, hit = 0;
    for (int i = 0; i < n_tags; i++) {
        int64_t begin = (int64_t)tags[i].start, end = (int64_t)tags[i].stop;
        if (begin < 0 || end > n25 || end <= begin) continue;
        int win_len = (int)(end - begin);
        if (win_len > BURST_WINDOW_LEN) win_len = BURST_WINDOW_LEN;
        win_len -= win_len % DIDECIM_DECIM;
        if (win_len < DIDECIM_DECIM) continue;

        memcpy(w25, iq25 + begin * 2, win_len * 2 * sizeof(int16_t));
        double ps = rotate_to_dc_phase_step_from_bin(tags[i].center_bin, FBT_FFT_SIZE);
        rotate_to_dc_q15_simd(w25, win_len, ps);

        direct_if_decim_reset_state(&dec);
        int n_out = direct_if_decim_process_split(&dec, w25, win_len, w250,
                                                  si, sq, so, sp);
        if (n_out <= 64) continue;

        // abs freq for ida_reasm channel identity: center_bin → offset
        double       off_hz = (double)(tags[i].center_bin - FBT_FFT_SIZE / 2) * (double)INPUT_FS_HZ / (double)FBT_FFT_SIZE;
        decode_ctx_t dctx   = {0};
        dctx.sbd            = &sbd;
        dctx.ida_reasm      = &ida_reasm;
        dctx.reasm          = reasm;
        dctx.burst_freq_hz  = (uint32_t)fabs(off_hz);
        dctx.expect         = expect;
        int frames          = burst_pipeline_process_burst(w250, n_out, on_frame, &dctx);
        if (frames > 0) pipeline_ok++;
        frames_total += frames;
        t_frames_lw += dctx.frames_lw;
        t_sbd += dctx.sbd_messages;
        t_acars += dctx.acars_decoded;
        if (dctx.hit) hit = 1;
        (void)uw_found;
    }

    printf("  ida_reasm: standalone=%u opened=%u merged=%u completed=%u "
           "orphan=%u overflow=%u expired=%u\n",
           ida_reasm.cnt_standalone, ida_reasm.cnt_opened, ida_reasm.cnt_merged,
           ida_reasm.cnt_completed, ida_reasm.cnt_orphan, ida_reasm.cnt_overflow,
           ida_reasm.cnt_expired);
    la_reasm_ctx_destroy(reasm);
    fft_burst_tagger_destroy(t);
    printf("\n== %s summary ==\n"
           "  bursts tagged : %d\n"
           "  pipeline ok   : %d\n"
           "  frames decoded: %d\n"
           "  LW.DA frames  : %d\n"
           "  SBD messages  : %d\n"
           "  ACARS decoded : %d\n",
           path, n_tags, pipeline_ok, frames_total, t_frames_lw, t_sbd, t_acars);

    free(iq256);
    free(iq25);
    free(tags);
    free(w25);
    free(w250);
    free(si);
    free(sq);
    free(so);
    free(sp);

    if (expect) {
        if (hit) {
            printf("PASS: found expected substring \"%s\"\n", expect);
            return 0;
        }
        printf("FAIL: expected substring \"%s\" not found\n", expect);
        return 1;
    }
    return 0;
}
