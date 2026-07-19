// decode_continuous_capture.c — replay a CONTINUOUS raw-uint8 IQ file
// (the RTL-SDR's raw USB stream, as captured by sd_capture.h "continuous"
// mode) through the P4's OWN wideband decode pipeline compiled for host.
//
// This is the missing input variant: decode_burst_capture.c reads the
// device's BURST-record format (40 B "BRST" header + int16), and
// test_pipeline_wideband_albq.c runs the right pipeline but is hard-wired
// to the scipy-resampled ALBQ cf32 fixture + baked golden compare. This
// tool takes an arbitrary continuous uint8 capture at 2.5 MSPS and runs:
//
//   fft_burst_tagger (detect) -> per-burst rotate-to-DC + 10x decim ->
//   burst_pipeline_process_burst (demod + UW + frame emit) ->
//   iridium_frame_classify -> for LW.DA: ida_decode -> acars_tail_feed
//   (ida_reassembler -> sbd_reassembler -> libacars ACARS parse).
//
// It is the exact same decode chain frame_decoder.c runs on-device, just
// fed from a file. The front end (tagger + per-burst mixer) is copied
// verbatim from test_pipeline_wideband_albq.c; the tail (classify + IDA +
// SBD + ACARS) is copied verbatim from decode_burst_capture.c.
//
// Input format (see p4-usb-host/main/sd_capture.h continuous mode):
//   continuous raw uint8 IQ, interleaved I,Q, DC offset 128
//   (unsigned 0-255), fs = 2.5 MSPS. No per-burst headers.
//
// uint8 -> int16 scaling matches tests/scripts/direct_if_dump.py:
//   float  = (u8 - 128) / 128        (range [-1, +0.992])
//   int16  = float * 32768           => int16 = (u8 - 128) * 256
// clamped to [-32768, 32767]. The captures are ALREADY at 2.5 MSPS so
// (unlike direct_if_dump.py's 2.56 MSPS ALBQ fixture) NO 125/128 resample
// is applied — that would corrupt the sample rate the pipeline assumes.
//
// Build via tests/host/CMakeLists.txt target decode_continuous_capture.
// Usage: decode_continuous_capture <capture.u8> [--threshold_db N]

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include "fft_burst_tagger.h"
#include "fft_sc16_2048.h"
#include "direct_if_decim.h"
#include "rotate_to_dc.h"
#include "burst_pipeline.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"
#include "iridium_frame.h"
#include "ida_decode.h"
#include "sbd_reassembler.h"
#include "ida_reassembler.h"
#include "acars_tail.h"
#include <libacars/reassembly.h>

#define INPUT_FS_HZ 2500000
#define BURST_PRE_LEN (2 * FBT_FFT_SIZE)            // = 4096
#define BURST_POST_LEN ((int)(INPUT_FS_HZ * 16e-3)) // = 40000
#define BURST_WINDOW_MAX_MS 250
#define BURST_WINDOW_LEN ((int)(INPUT_FS_HZ * BURST_WINDOW_MAX_MS / 1000)) // = 625000
#define BURST_WINDOW_250K (BURST_WINDOW_LEN / DIDECIM_DECIM)

// gri default is 512×2048 int32 = 4 MB; static so it isn't on the stack.
static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

typedef struct {
    // frame classification
    int frames_decoded;
    int frames_ms, frames_tl, frames_bc, frames_lw, frames_ra, frames_unk;
    int frames_lw_da, frames_lw_other;
    int frames_dl, frames_ul;
    // IDA-level BCH (only meaningful for LW.DA frames that reach ida_decode)
    int ida_attempts;   // LW.DA frames handed to ida_decode
    int ida_clean;      // ok && total_errors == 0
    int ida_corrected;  // ok && total_errors  > 0
    int ida_failed;     // !ok (a BCH block failed repair)
    int ida_crc_ok;     // header_ok && crc_ok (frame fully valid)
    int ida_bch_errors; // sum of corrected bit errors across clean+corrected
    int sbd_messages;
    int acars_decoded;
} run_summary_t;

typedef struct {
    run_summary_t     *bs;
    sbd_reassembler_t *sbd;
    ida_reassembler_t *ida_reasm;
    la_reasm_ctx      *reasm;
    uint64_t           burst_t_us;
    uint32_t           burst_freq_hz;
} decode_ctx_t;

static void print_hex(const uint8_t *buf, size_t n)
{
    for (size_t i = 0; i < n; i++) printf("%02x", buf[i]);
}

// Per-decoded-frame callback (mirrors decode_burst_capture.c on_frame +
// frame_decoder.c process_one()).
static void on_frame(burst_pipeline_result_t *res, void *ctx_)
{
    decode_ctx_t  *ctx = (decode_ctx_t *)ctx_;
    run_summary_t *bs  = ctx->bs;
    bs->frames_decoded++;

    iridium_frame_t      f;
    ir_frame_direction_t dir = (res->uw_res.direction == UW_DIR_UPLINK)
                                   ? IR_FRM_DIR_UPLINK
                                   : IR_FRM_DIR_DOWNLINK;
    if (iridium_frame_classify(res->frame.bits, res->frame.n_bits, dir, &f) == 0) {
        switch (f.type) {
        case IR_FRAME_MS: bs->frames_ms++; break;
        case IR_FRAME_TL: bs->frames_tl++; break;
        case IR_FRAME_BC: bs->frames_bc++; break;
        case IR_FRAME_LW:
            bs->frames_lw++;
            if (f.lw_subtype == IR_LW_DA) {
                bs->frames_lw_da++;
                ida_decoded_t ida    = {0};
                int           rc_ida = ida_decode(&f, &ida);
                if (rc_ida == 0) {
                    bs->ida_attempts++;
                    if (!ida.ok) {
                        bs->ida_failed++;
                    } else if (ida.total_errors == 0) {
                        bs->ida_clean++;
                    } else {
                        bs->ida_corrected++;
                        bs->ida_bch_errors += ida.total_errors;
                    }
                    if (ida.ok && ida.header_ok && ida.crc_ok) {
                        bs->ida_crc_ok++;
                        acars_tail_result_t tail;
                        int rc_tail = acars_tail_feed(
                            ctx->sbd, ctx->reasm, ctx->ida_reasm, &ida,
                            dir == IR_FRM_DIR_UPLINK, ctx->burst_freq_hz,
                            ctx->burst_t_us, &tail);
                        if (rc_tail == 1 && tail.sbd_ready) {
                            bs->sbd_messages++;
                            printf("SBD: type=%s %s len=%u (msg %u/%u) payload=",
                                   sbd_type_wire_name(tail.sbd.type),
                                   tail.sbd.uplink ? "UL" : "DL",
                                   tail.sbd.payload_len, tail.sbd.msg_no,
                                   tail.sbd.msg_count);
                            print_hex(tail.sbd.payload, tail.sbd.payload_len);
                            printf("\n");
                            if (tail.acars_ready) {
                                bs->acars_decoded++;
                                printf("ACARS: %s mode=%c label='%.2s' block=%c "
                                       "msgnum='%.4s' flight='%.6s' crc=%s txt=\"%s\"\n",
                                       tail.sbd.uplink ? "UL" : "DL",
                                       tail.mode ? tail.mode : '?', tail.label,
                                       tail.block_id ? tail.block_id : '?',
                                       tail.msg_num, tail.flight_id,
                                       tail.crc_ok ? "OK" : "BAD", tail.txt);
                            }
                        }
                    }
                }
            } else {
                bs->frames_lw_other++;
            }
            break;
        case IR_FRAME_RA: bs->frames_ra++; break;
        default: bs->frames_unk++; break;
        }
    } else {
        bs->frames_unk++;
    }
    if (dir == IR_FRM_DIR_UPLINK) bs->frames_ul++;
    else bs->frames_dl++;

    free(res->frame.bits);
    free(res->frame.soft_bits);
}

// Load a continuous uint8 IQ file, convert to int16 IQ at the wideband
// pipeline's expected scale. Returns complex-sample count, or -1.
static long load_u8_iq(const char *path, int16_t **out_iq)
{
    FILE *fh = fopen(path, "rb");
    if (!fh) {
        fprintf(stderr, "fopen(%s): %s\n", path, strerror(errno));
        return -1;
    }
    fseek(fh, 0, SEEK_END);
    long bytes = ftell(fh);
    fseek(fh, 0, SEEK_SET);
    long n = bytes / 2; // 2 uint8 (I,Q) per complex sample
    if (n <= 0) { fclose(fh); return -1; }

    uint8_t *u8 = malloc((size_t)n * 2);
    int16_t *s16 = malloc((size_t)n * 2 * sizeof(int16_t));
    if (!u8 || !s16) { free(u8); free(s16); fclose(fh); return -1; }
    size_t got = fread(u8, 1, (size_t)n * 2, fh);
    fclose(fh);
    if (got != (size_t)n * 2) {
        fprintf(stderr, "short read: got %zu of %ld\n", got, n * 2);
        // still usable up to got; truncate
        n = (long)(got / 2);
    }
    for (long k = 0; k < n * 2; k++) {
        int v = ((int)u8[k] - 128) * 256; // (u8-128)/128 * 32768
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        s16[k] = (int16_t)v;
    }
    free(u8);
    *out_iq = s16;
    return n;
}

typedef struct {
    uint64_t start_sample;
    uint64_t stop_sample;
    int      center_bin;
} tag_t;

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <capture.u8> [--threshold_db N]\n", argv[0]);
        return 2;
    }
    const char *path         = argv[1];
    float       threshold_db = 10.0f; // same as test_pipeline_wideband_albq
    for (int a = 2; a < argc; a++) {
        if (strcmp(argv[a], "--threshold_db") == 0 && a + 1 < argc)
            threshold_db = (float)atof(argv[++a]);
    }

    int16_t *iq25 = NULL;
    long     n25  = load_u8_iq(path, &iq25);
    if (n25 <= 0) return 2;
    printf("== %s ==\n", path);
    printf("Loaded %ld complex samples at 2.5 MSPS (%.2f s), threshold=%.1f dB\n",
           n25, (double)n25 / 2.5e6, threshold_db);

    fft_burst_tagger_t *t = fft_burst_tagger_init(
        BURST_PRE_LEN, BURST_POST_LEN, /*burst_width=*/32,
        threshold_db, s_baseline_history);
    if (!t) { fprintf(stderr, "tagger init\n"); free(iq25); return 2; }
    fft_burst_tagger_set_start(t, 0);

    // Collect all GONE burst tags across the whole file (growable).
    long   tags_cap = 4096;
    tag_t *tags     = malloc((size_t)tags_cap * sizeof(tag_t));
    long   n_tags   = 0;
    fbt_burst_t new_bursts[FBT_MAX_BURSTS];
    fbt_burst_t gone_bursts[FBT_MAX_BURSTS];
    for (long off = 0; off + FBT_FFT_SIZE <= n25; off += FBT_FFT_SIZE) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fft_burst_tagger_step(t, iq25 + off * 2, NULL,
                              new_bursts, &n_new, gone_bursts, &n_gone);
        for (int i = 0; i < n_gone; i++) {
            if (n_tags >= tags_cap) {
                tags_cap *= 2;
                tags = realloc(tags, (size_t)tags_cap * sizeof(tag_t));
            }
            tags[n_tags].start_sample = gone_bursts[i].start;
            tags[n_tags].stop_sample  = gone_bursts[i].stop;
            tags[n_tags].center_bin   = gone_bursts[i].center_bin;
            n_tags++;
        }
    }
    {
        int         n_flush = FBT_MAX_BURSTS;
        fbt_burst_t flushed[FBT_MAX_BURSTS];
        fft_burst_tagger_flush(t, flushed, &n_flush);
        for (int i = 0; i < n_flush; i++) {
            if (n_tags >= tags_cap) {
                tags_cap *= 2;
                tags = realloc(tags, (size_t)tags_cap * sizeof(tag_t));
            }
            tags[n_tags].start_sample = flushed[i].start;
            tags[n_tags].stop_sample  = flushed[i].stop;
            tags[n_tags].center_bin   = flushed[i].center_bin;
            n_tags++;
        }
    }
    printf("Tagger emitted %ld gone bursts\n", n_tags);

    // Per-burst processing scratch.
    direct_if_decim_t dec;
    direct_if_decim_init(&dec);
    int16_t *window_25  = malloc(2 * BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *window_250 = malloc(2 * BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_in_i   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_in_q   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *scr_out_i  = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *scr_out_q  = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    if (!window_25 || !window_250 || !scr_in_i || !scr_in_q || !scr_out_i || !scr_out_q) {
        fprintf(stderr, "alloc\n");
        return 2;
    }

    // Persistent IDA->SBD->ACARS tail state (mirrors frame_decoder.c globals).
    sbd_reassembler_t sbd;
    sbd_reassembler_init(&sbd);
    ida_reassembler_t ida_reasm;
    ida_reassembler_init(&ida_reasm);
    la_reasm_ctx *reasm = la_reasm_ctx_new();
    if (!reasm) { fprintf(stderr, "la_reasm_ctx_new failed\n"); return 2; }

    run_summary_t total       = {0};
    int           pipeline_ok = 0; // bursts that produced >=1 frame
    int           uw_found    = 0; // never used to gate; informational

    for (long i = 0; i < n_tags; i++) {
        int64_t begin = (int64_t)tags[i].start_sample;
        int64_t end   = (int64_t)tags[i].stop_sample;
        if (begin < 0 || end > n25 || end <= begin) continue;
        int win_len = (int)(end - begin);
        if (win_len > BURST_WINDOW_LEN) win_len = BURST_WINDOW_LEN;
        win_len -= win_len % DIDECIM_DECIM;
        if (win_len < DIDECIM_DECIM) continue;

        memcpy(window_25, iq25 + begin * 2, (size_t)win_len * 2 * sizeof(int16_t));

        double phase_step =
            rotate_to_dc_phase_step_from_bin(tags[i].center_bin, FBT_FFT_SIZE);
        rotate_to_dc_q15_simd(window_25, win_len, phase_step);

        direct_if_decim_reset_state(&dec);
        int n_out = direct_if_decim_process_split(&dec, window_25, win_len,
                                                  window_250, scr_in_i, scr_in_q,
                                                  scr_out_i, scr_out_q);
        if (n_out <= 64) continue;

        int   signed_bin  = tags[i].center_bin - FBT_FFT_SIZE / 2;
        float rel_freq_hz = (float)signed_bin * (float)INPUT_FS_HZ / (float)FBT_FFT_SIZE;

        decode_ctx_t dctx = {
            .bs            = &total,
            .sbd           = &sbd,
            .ida_reasm     = &ida_reasm,
            .reasm         = reasm,
            .burst_t_us    = (uint64_t)((double)begin / 2.5), // begin/2.5e6 * 1e6
            .burst_freq_hz = (uint32_t)(rel_freq_hz < 0 ? -rel_freq_hz : rel_freq_hz),
        };
        int frames = burst_pipeline_process_burst(window_250, n_out, on_frame, &dctx);
        if (frames > 0) pipeline_ok++;
    }
    (void)uw_found;

    printf("\n-- %s summary --\n", path);
    printf("  bursts tagged     : %ld\n", n_tags);
    printf("  pipeline decoded  : %d bursts produced >=1 frame\n", pipeline_ok);
    printf("  frames decoded    : %d\n", total.frames_decoded);
    printf("    MS/TL/BC/LW/RA  : %d/%d/%d/%d/%d  unknown: %d\n",
           total.frames_ms, total.frames_tl, total.frames_bc, total.frames_lw,
           total.frames_ra, total.frames_unk);
    printf("    LW.DA / LW.other: %d / %d\n", total.frames_lw_da, total.frames_lw_other);
    printf("    DL / UL         : %d / %d\n", total.frames_dl, total.frames_ul);
    printf("  IDA(LW.DA) BCH    : attempts=%d clean=%d corrected=%d failed=%d "
           "(corrected_bit_errors=%d)\n",
           total.ida_attempts, total.ida_clean, total.ida_corrected,
           total.ida_failed, total.ida_bch_errors);
    printf("  IDA header+CRC ok : %d\n", total.ida_crc_ok);
    printf("  SBD messages      : %d\n", total.sbd_messages);
    printf("  ACARS decoded     : %d\n", total.acars_decoded);

    // Machine-parseable one-liner for cross-file roll-up.
    printf("RESULT %s bursts=%ld pipeline_ok=%d frames=%d ms=%d tl=%d bc=%d "
           "lw=%d ra=%d unk=%d lwda=%d ida_clean=%d ida_corr=%d ida_fail=%d "
           "ida_crc_ok=%d sbd=%d acars=%d\n",
           path, n_tags, pipeline_ok, total.frames_decoded, total.frames_ms,
           total.frames_tl, total.frames_bc, total.frames_lw, total.frames_ra,
           total.frames_unk, total.frames_lw_da, total.ida_clean,
           total.ida_corrected, total.ida_failed, total.ida_crc_ok,
           total.sbd_messages, total.acars_decoded);

    fft_burst_tagger_destroy(t);
    la_reasm_ctx_destroy(reasm);
    free(window_25); free(window_250);
    free(scr_in_i); free(scr_in_q); free(scr_out_i); free(scr_out_q);
    free(tags); free(iq25);
    return 0;
}
