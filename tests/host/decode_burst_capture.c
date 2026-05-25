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
//
// Output: one CSV row per burst plus a summary at the end. CSV makes
// it easy to roll up across many capture files in the monitor.
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

#define SD_CAPTURE_BURST_MAGIC  0x54535242u   /* "BRST" little-endian */

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
} burst_summary_t;

static void on_frame(burst_pipeline_result_t *res, void *ctx)
{
    burst_summary_t *bs = (burst_summary_t *)ctx;
    bs->frames_decoded++;

    iridium_frame_t f;
    ir_frame_direction_t dir = (res->uw_res.direction == UW_DIR_UPLINK)
                                ? IR_FRM_DIR_UPLINK : IR_FRM_DIR_DOWNLINK;
    if (iridium_frame_classify(res->frame.bits, res->frame.n_bits, dir, &f) == 0) {
        switch (f.type) {
            case IR_FRAME_MS: bs->frames_ms++; break;
            case IR_FRAME_TL: bs->frames_tl++; break;
            case IR_FRAME_BC: bs->frames_bc++; break;
            case IR_FRAME_LW: bs->frames_lw++; break;
            case IR_FRAME_RA: bs->frames_ra++; break;
            default:          bs->frames_unk++; break;
        }
    } else {
        bs->frames_unk++;
    }
    if (dir == IR_FRM_DIR_UPLINK) bs->frames_ul++;
    else                          bs->frames_dl++;

    free(res->frame.bits);
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
    const int MAX_IN = 700000;
    const int MAX_OUT = MAX_IN / DIDECIM_DECIM;
    int16_t *iq25  = malloc((size_t)MAX_IN * 2 * sizeof(int16_t));
    int16_t *iq250 = malloc((size_t)MAX_OUT * 2 * sizeof(int16_t));
    int16_t *si    = malloc((size_t)MAX_IN * sizeof(int16_t));
    int16_t *sq    = malloc((size_t)MAX_IN * sizeof(int16_t));
    int16_t *so    = malloc((size_t)MAX_OUT * sizeof(int16_t));
    int16_t *sp    = malloc((size_t)MAX_OUT * sizeof(int16_t));
    if (!iq25 || !iq250 || !si || !sq || !so || !sp) {
        fprintf(stderr, "out of memory\n");
        return 1;
    }

    int bursts_seen = 0, bursts_too_short = 0, bursts_decode_ok = 0;
    int frames_total = 0;
    burst_summary_t total = {0};

    while (1) {
        burst_hdr_t h;
        size_t got = fread(&h, 1, sizeof(h), fp);
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

        burst_summary_t bs = {0};
        int frames = burst_pipeline_process_burst(iq250, n250, on_frame, &bs);
        if (frames > 0) bursts_decode_ok++;
        frames_total += frames;
        total.frames_decoded += bs.frames_decoded;
        total.frames_ms += bs.frames_ms;
        total.frames_tl += bs.frames_tl;
        total.frames_bc += bs.frames_bc;
        total.frames_lw += bs.frames_lw;
        total.frames_ra += bs.frames_ra;
        total.frames_unk += bs.frames_unk;
        total.frames_dl += bs.frames_dl;
        total.frames_ul += bs.frames_ul;

        if (emit_csv) {
            printf("%u,%u,%.0f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                    h.seq, h.length_samples, h.rel_freq_hz, h.peak_snr_db,
                    bs.frames_decoded, bs.frames_ms, bs.frames_tl, bs.frames_bc,
                    bs.frames_lw, bs.frames_ra, bs.frames_unk,
                    bs.frames_dl, bs.frames_ul);
        }
    }

    fclose(fp);

    fprintf(stderr,
        "\n== %s summary ==\n"
        "  bursts            : %d\n"
        "  too short (<64)   : %d\n"
        "  decode ok         : %d (%.1f%%)\n"
        "  frames            : %d\n"
        "    MS/TL/BC/LW/RA : %d/%d/%d/%d/%d  unknown: %d\n"
        "    DL/UL           : %d/%d\n",
        argv[1],
        bursts_seen, bursts_too_short,
        bursts_decode_ok,
        bursts_seen ? 100.0 * bursts_decode_ok / bursts_seen : 0.0,
        frames_total,
        total.frames_ms, total.frames_tl, total.frames_bc, total.frames_lw,
        total.frames_ra, total.frames_unk,
        total.frames_dl, total.frames_ul);

    return 0;
}
