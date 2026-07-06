// decode_burst_capture.c — replay a .u8 burst-capture file from the
// device through the exact host pipeline the firmware runs on Core 1.
//
// File format (see p4-usb-host/main/sd_capture.h):
//   Sequence of records, each:
//     40 B header (magic="BRST", seq, t_us, length_samples,
//                   rel_freq_hz, peak_snr_db, magnitude_db, noise_db,
//                   reserved=0)
//     length_samples * 4 bytes interleaved int16 IQ at 2.5 MSPS
//
// Per burst this tool:
//   1. rotates raw 2.5 MSPS IQ to DC at rel_freq_hz (matches worker)
//   2. decimates 10× to 250 ksps (direct_if_decim)
//   3. runs burst_pipeline_process_burst (multi-frame)
//   4. classifies each decoded frame
//   5. for LW.DA frames: runs ida_decode -> sbd_reassembler_feed ->
//      libacars ACARS parse (the same tail glue frame_decoder.c runs
//      on-device, shared via acars_tail.c) and prints any assembled
//      SBD payload / parsed ACARS text
//
// Output: one CSV row per burst plus a summary at the end, on top of
// "SBD: " / "ACARS: " lines for anything the tail glue produces. CSV
// makes it easy to roll up across many capture files in the monitor.
//
// Build via tests/host/CMakeLists.txt target decode_burst_capture.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include "direct_if_decim.h"
#include "rotate_to_dc.h"
#include "burst_pipeline.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"
#include "iridium_frame.h"
#include "ida_decode.h"
#include "sbd_reassembler.h"
#include "acars_tail.h"
#include <libacars/reassembly.h>

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

typedef struct {
    int frames_decoded;
    int frames_ms, frames_tl, frames_bc, frames_lw, frames_ra, frames_unk;
    int frames_dl, frames_ul;
    int sbd_messages;
    int acars_decoded;
} burst_summary_t;

// Context handed to on_frame() via burst_pipeline_process_burst's ctx
// pointer: the per-burst summary counters plus the IDA->SBD->ACARS
// tail state, which (like frame_decoder.c's s_sbd / s_reasm_ctx) must
// persist across frames/bursts so multi-frame SBD sessions and
// multi-block ACARS reassembly work.
typedef struct {
    burst_summary_t    bs;
    sbd_reassembler_t *sbd;
    la_reasm_ctx      *reasm;
    uint64_t           burst_t_us; // capture-header timestamp for
                                   // this burst; used as a stand-in
                                   // for the worker's per-frame
                                   // timestamp (not tracked at this
                                   // offline granularity).
} decode_ctx_t;

static void print_hex(const uint8_t *buf, size_t n)
{
    for (size_t i = 0; i < n; i++)
        printf("%02x", buf[i]);
}

static void on_frame(burst_pipeline_result_t *res, void *ctx_)
{
    decode_ctx_t    *ctx = (decode_ctx_t *)ctx_;
    burst_summary_t *bs  = &ctx->bs;
    bs->frames_decoded++;

    iridium_frame_t      f;
    ir_frame_direction_t dir = (res->uw_res.direction == UW_DIR_UPLINK)
                                   ? IR_FRM_DIR_UPLINK
                                   : IR_FRM_DIR_DOWNLINK;
    if (iridium_frame_classify(res->frame.bits, res->frame.n_bits, dir, &f) == 0) {
        switch (f.type) {
        case IR_FRAME_MS:
            bs->frames_ms++;
            break;
        case IR_FRAME_TL:
            bs->frames_tl++;
            break;
        case IR_FRAME_BC:
            bs->frames_bc++;
            break;
        case IR_FRAME_LW:
            bs->frames_lw++;
            // Mirrors frame_decoder.c's process_one() IR_FRAME_LW /
            // IR_LW_DA dispatch: run ida_decode, and on a clean
            // header+CRC feed the shared IDA->SBD->ACARS tail glue.
            if (f.lw_subtype == IR_LW_DA) {
                ida_decoded_t ida    = {0};
                int           rc_ida = ida_decode(&f, &ida);
                if (rc_ida == 0 && ida.ok && ida.header_ok && ida.crc_ok) {
                    acars_tail_result_t tail;
                    int                 rc_tail = acars_tail_feed(ctx->sbd, ctx->reasm, &ida,
                                                                  dir == IR_FRM_DIR_UPLINK,
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
                                   "msgnum='%.4s' flight='%.6s' crc=%s "
                                   "txt=\"%s\"\n",
                                   tail.sbd.uplink ? "UL" : "DL",
                                   tail.mode ? tail.mode : '?',
                                   tail.label, tail.block_id ? tail.block_id : '?',
                                   tail.msg_num, tail.flight_id,
                                   tail.crc_ok ? "OK" : "BAD", tail.txt);
                        }
                    }
                }
            }
            break;
        case IR_FRAME_RA:
            bs->frames_ra++;
            break;
        default:
            bs->frames_unk++;
            break;
        }
    } else {
        bs->frames_unk++;
    }
    if (dir == IR_FRM_DIR_UPLINK)
        bs->frames_ul++;
    else
        bs->frames_dl++;

    free(res->frame.bits);
    free(res->frame.soft_bits); // #112
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <capture.u8> [--csv]\n", argv[0]);
        return 2;
    }
    bool emit_csv = (argc >= 3 && strcmp(argv[2], "--csv") == 0);

    FILE *fp = fopen(argv[1], "rb");
    if (!fp) {
        fprintf(stderr, "fopen(%s): %s\n", argv[1], strerror(errno));
        return 1;
    }

    if (emit_csv) {
        printf("seq,length_samples,rel_freq_hz,peak_snr_db,frames,ms,tl,bc,lw,ra,unk,dl,ul\n");
    }

    // Reused decim state — reset between bursts.
    direct_if_decim_t dec;
    direct_if_decim_init(&dec);

    // Decimation worker scratch buffers — sized for the largest burst
    // we might see. WB_MAX_BURST_SAMPLES (worker_core1.c) is 625000;
    // round up a touch. The split-resample variant needs scratch in
    // chunks of DECIM_CHUNK_IN samples. Allocate worst-case once.
    const int MAX_IN  = 700000;
    const int MAX_OUT = MAX_IN / DIDECIM_DECIM;
    int16_t  *iq25    = malloc((size_t)MAX_IN * 2 * sizeof(int16_t));
    int16_t  *iq250   = malloc((size_t)MAX_OUT * 2 * sizeof(int16_t));
    int16_t  *si      = malloc((size_t)MAX_IN * sizeof(int16_t));
    int16_t  *sq      = malloc((size_t)MAX_IN * sizeof(int16_t));
    int16_t  *so      = malloc((size_t)MAX_OUT * sizeof(int16_t));
    int16_t  *sp      = malloc((size_t)MAX_OUT * sizeof(int16_t));
    if (!iq25 || !iq250 || !si || !sq || !so || !sp) {
        fprintf(stderr, "out of memory\n");
        return 1;
    }

    // IDA -> SBD -> ACARS tail state, persistent across bursts/frames
    // (mirrors frame_decoder.c's s_sbd / s_reasm_ctx globals).
    sbd_reassembler_t sbd;
    sbd_reassembler_init(&sbd);
    la_reasm_ctx *reasm = la_reasm_ctx_new();
    if (!reasm) {
        fprintf(stderr, "la_reasm_ctx_new() failed\n");
        return 1;
    }

    int             bursts_seen = 0, bursts_too_short = 0, bursts_decode_ok = 0;
    int             frames_total = 0;
    burst_summary_t total        = {0};

    while (1) {
        burst_hdr_t h;
        size_t      got = fread(&h, 1, sizeof(h), fp);
        if (got == 0) break;
        if (got != sizeof(h)) {
            fprintf(stderr, "short header read (got %zu) — file truncated\n", got);
            break;
        }
        if (h.magic != SD_CAPTURE_BURST_MAGIC) {
            fprintf(stderr, "bad magic 0x%08x at burst %d — resync skipped\n",
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

        // Rotate to DC at the freq the worker tagged.
        double phase_step = -2.0 * M_PI * (double)h.rel_freq_hz / (double)INPUT_FS_HZ;
        rotate_to_dc_q15_simd(iq25, (int)h.length_samples, phase_step);

        // Decimate 10× to 250 ksps. Use the split-state API so the
        // host result matches the firmware's chunked call sequence
        // when only one chunk is fed (state at start matches state
        // after a reset).
        direct_if_decim_reset_state(&dec);
        int n250 = direct_if_decim_process_split(&dec,
                                                 iq25, (int)h.length_samples,
                                                 iq250, si, sq, so, sp);
        if (n250 <= 64) {
            bursts_too_short++;
            if (emit_csv) {
                printf("%u,%u,%.0f,%.1f,0,0,0,0,0,0,0,0,0\n",
                       h.seq, h.length_samples, h.rel_freq_hz, h.peak_snr_db);
            }
            continue;
        }

        decode_ctx_t dctx       = {0};
        dctx.sbd                = &sbd;
        dctx.reasm              = reasm;
        dctx.burst_t_us         = h.t_us;
        int              frames = burst_pipeline_process_burst(iq250, n250, on_frame, &dctx);
        burst_summary_t *bs     = &dctx.bs;
        if (frames > 0) bursts_decode_ok++;
        frames_total += frames;
        total.frames_decoded += bs->frames_decoded;
        total.frames_ms += bs->frames_ms;
        total.frames_tl += bs->frames_tl;
        total.frames_bc += bs->frames_bc;
        total.frames_lw += bs->frames_lw;
        total.frames_ra += bs->frames_ra;
        total.frames_unk += bs->frames_unk;
        total.frames_dl += bs->frames_dl;
        total.frames_ul += bs->frames_ul;
        total.sbd_messages += bs->sbd_messages;
        total.acars_decoded += bs->acars_decoded;

        if (emit_csv) {
            printf("%u,%u,%.0f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                   h.seq, h.length_samples, h.rel_freq_hz, h.peak_snr_db,
                   bs->frames_decoded, bs->frames_ms, bs->frames_tl, bs->frames_bc,
                   bs->frames_lw, bs->frames_ra, bs->frames_unk,
                   bs->frames_dl, bs->frames_ul);
        }
    }

    fclose(fp);
    la_reasm_ctx_destroy(reasm);

    fprintf(stderr,
            "\n== %s summary ==\n"
            "  bursts            : %d\n"
            "  too short (<64)   : %d\n"
            "  decode ok         : %d (%.1f%%)\n"
            "  frames            : %d\n"
            "    MS/TL/BC/LW/RA : %d/%d/%d/%d/%d  unknown: %d\n"
            "    DL/UL           : %d/%d\n"
            "  SBD messages      : %d\n"
            "  ACARS decoded     : %d\n",
            argv[1],
            bursts_seen, bursts_too_short,
            bursts_decode_ok,
            bursts_seen ? 100.0 * bursts_decode_ok / bursts_seen : 0.0,
            frames_total,
            total.frames_ms, total.frames_tl, total.frames_bc, total.frames_lw,
            total.frames_ra, total.frames_unk,
            total.frames_dl, total.frames_ul,
            total.sbd_messages, total.acars_decoded);

    return 0;
}
