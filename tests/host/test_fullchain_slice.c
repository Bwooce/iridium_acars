// test_fullchain_slice.c — equal-bandwidth end-to-end P4-chain vs
// gr-iridium comparison on a native 2.5 MSPS int16 IQ slice.
//
// Feeds a runtime 2.5 MSPS ci16 (interleaved int16 IQ) file — the P4's
// exact Path-A bandwidth/format, e.g. a slice decimated from a HydraSDR
// capture — through the FULL device receive chain:
//
//   fft_burst_tagger (detection, device Path-A params)
//     → per burst: rotate_to_dc + direct_if_decim (→250 ksps)
//     → burst_prefilter (P1.5b production gate, incl. the A6 hot-bin
//       SNR exemption for open-IDA-chain channels — emulated serially)
//     → burst_pipeline_process_burst (D13/CFO/RRC/UW/PLL/demod, multi-frame)
//     → iridium_frame_classify
//     → for LW.DA: ida_decode → ida_reassembler → sbd_reassembler →
//       libacars (via acars_tail_feed, the same tail frame_decoder.c runs)
//
// Compared with test_phaseb_cut.c (2.56 MSPS input, no prefilter, no
// timestamps): this harness (a) skips resample_256_to_250 because the
// input is already at the device's native 2.5 MSPS, (b) runs the
// production burst_prefilter gate exactly as worker_core1.c does
// (width/duration/channel-SNR + hot-channel SNR-only escalation),
// (c) stamps every frame with its RF-derived timestamp so the IDA
// fragment-chain gap/expiry logic (700 ms / 1 s) runs for real, and
// (d) breaks the report into the deliverable's three tiers: bursts
// tagged / LW.DA frames decoded / ACARS reassembled.
//
// The point: gr-iridium sees the SAME file at the SAME bandwidth, so
// any yield gap is OURS — and the per-tier counters say whether it is
// the tagger (fewer bursts), the demod (bursts without frames), or the
// reassembly tail (frames without ACARS).
//
// Usage: test_fullchain_slice <slice_2500k.ci16> [expect_substr]
//   Exits 0 if (expect_substr absent) or an emitted ACARS field / SBD
//   payload hex contains expect_substr; else 1.
//
// Env knobs (diagnosis only — defaults match the device):
//   TAG_THR=<db>  tagger threshold, default 10.0 (DEFAULT_TAGGER_THRESHOLD_DB)
//   PF=0          disable burst_prefilter (attribute prefilter losses)
//   SCALE=<n>     integer amplitude multiplier on load (quantisation checks)

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
#include "burst_prefilter.h"
#include "qpsk_demod.h"
#include "uw_correlator.h"
#include "iridium_frame.h"
#include "ida_decode.h"
#include "ida_reassembler.h"
#include "sbd_reassembler.h"
#include "acars_tail.h"
#include <libacars/reassembly.h>

#define INPUT_FS_HZ 2500000
// Device Path-A tagger params (p4-usb-host/main/dsp_processor.c):
// pre = 2*FFT = 4096, post = 40000, width = 32 bins, thr = NVS tag_thr
// (default DEFAULT_TAGGER_THRESHOLD_DB = 10.0 in app_config.c).
#define BURST_PRE_LEN (2 * FBT_FFT_SIZE)
#define BURST_POST_LEN ((int)(INPUT_FS_HZ * 16e-3)) // 40000
#define BURST_WINDOW_LEN ((int)(INPUT_FS_HZ * 250 / 1000)) // 625000 (250 ms cap)
#define BURST_WINDOW_250K (BURST_WINDOW_LEN / DIDECIM_DECIM)

// RF-sample-derived timestamp, µs at 2.5 MSPS: t = n * 1e6 / 2.5e6 = n * 2 / 5.
#define SAMPLE_TO_US(n) ((uint64_t)(n) * 2ULL / 5ULL)

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

// ---------------------------------------------------------------------
// A6 hot-bin emulation. On device, frame_decoder publishes a hot bin
// when an IDA chain opens (rc_reasm==0) and the worker exempts an
// SNR-only prefilter reject on that channel while the entry is fresh
// (~FRAG_GAP). This harness is serial (a burst's frames reach the
// reassembler before the next burst runs), so a tiny local table gives
// the same semantics. Keyed on tagger center_bin; ±4 bins ≈ ±5 kHz =
// IDA_REASM_FREQ_DEADBAND_HZ.
#define HOT_MAX 8
#define HOT_BIN_TOL 4
#define HOT_TTL_US (700ULL * 1000ULL) // = IDA_REASM_FRAG_GAP_US
typedef struct {
    int      bin;
    uint64_t ts_us;
    bool     live;
} hot_ent_t;
static hot_ent_t s_hot[HOT_MAX];

static void hot_publish(int bin, uint64_t now_us)
{
    int oldest = 0;
    for (int i = 0; i < HOT_MAX; i++) {
        if (s_hot[i].live && abs(s_hot[i].bin - bin) <= HOT_BIN_TOL) {
            s_hot[i].ts_us = now_us; // refresh
            return;
        }
        if (s_hot[i].ts_us < s_hot[oldest].ts_us) oldest = i;
        if (!s_hot[i].live) { oldest = i; break; }
    }
    s_hot[oldest] = (hot_ent_t){ .bin = bin, .ts_us = now_us, .live = true };
}
static void hot_clear(int bin)
{
    for (int i = 0; i < HOT_MAX; i++)
        if (s_hot[i].live && abs(s_hot[i].bin - bin) <= HOT_BIN_TOL)
            s_hot[i].live = false;
}
static bool hot_match(int bin, uint64_t now_us)
{
    for (int i = 0; i < HOT_MAX; i++)
        if (s_hot[i].live && abs(s_hot[i].bin - bin) <= HOT_BIN_TOL &&
            now_us - s_hot[i].ts_us <= HOT_TTL_US)
            return true;
    return false;
}

// ---------------------------------------------------------------------
static void print_hex(const uint8_t *buf, size_t n)
{
    for (size_t i = 0; i < n; i++)
        printf("%02x", buf[i]);
}

typedef struct {
    sbd_reassembler_t *sbd;
    ida_reassembler_t *ida_reasm;
    la_reasm_ctx      *reasm;
    uint32_t           burst_freq_hz; // ida_reasm channel-identity key
    uint64_t           burst_ts_us;   // RF-derived burst start time
    int                center_bin;

    // per-run counters (shared across bursts via pointer aliasing below)
    int frames_decoded;
    int class_unknown, class_ms, class_tl, class_bc, class_ra, class_lw_other;
    int lw_da;          // classified LW.DA frames (any BCH state)
    int lw_da_clean;    // ida ok + header ok + CRC ok  ← "IDA frames decoded"
    int lw_da_dirty;    // ida ok + header ok + CRC FAIL (device would only
                        // best-effort these when best_effort_decode=on; the
                        // firmware default is OFF, so we count but don't feed)
    int sbd_messages, acars_decoded;

    const char *expect;
    int         hit;
} decode_ctx_t;

static void on_frame(burst_pipeline_result_t *res, void *ctx_)
{
    decode_ctx_t *ctx = (decode_ctx_t *)ctx_;
    ctx->frames_decoded++;

    iridium_frame_t      f;
    ir_frame_direction_t dir = (res->frame.direction == DIR_UPLINK)
                                   ? IR_FRM_DIR_UPLINK
                                   : IR_FRM_DIR_DOWNLINK;
    if (iridium_frame_classify(res->frame.bits, res->frame.n_bits, dir, &f) != 0) {
        ctx->class_unknown++;
        goto out;
    }
    switch (f.type) {
    case IR_FRAME_MS: ctx->class_ms++; break;
    case IR_FRAME_TL: ctx->class_tl++; break;
    case IR_FRAME_BC: ctx->class_bc++; break;
    case IR_FRAME_RA: ctx->class_ra++; break;
    case IR_FRAME_LW:
        if (f.lw_subtype != IR_LW_DA) {
            ctx->class_lw_other++;
            break;
        }
        ctx->lw_da++;
        {
            ida_decoded_t ida    = {0};
            int           rc_ida = ida_decode(&f, &ida);
            printf("  IDA bin=%d freq=%uHz t=%.3fs rc=%d ok=%d hdr=%d crc=%d "
                   "ctr=%u cont=%u len=%u pay=",
                   ctx->center_bin, ctx->burst_freq_hz,
                   (double)ctx->burst_ts_us / 1e6, rc_ida, ida.ok,
                   ida.header_ok, ida.crc_ok, ida.da_ctr, ida.da_cont,
                   ida.payload_len);
            print_hex(ida.payload, ida.payload_len);
            printf("\n");
            bool hdr_ok = (rc_ida == 0 && ida.ok && ida.header_ok);
            if (hdr_ok && !ida.crc_ok) ctx->lw_da_dirty++;
            if (!(hdr_ok && ida.crc_ok)) break; // firmware clean gate
            ctx->lw_da_clean++;

            acars_tail_result_t tail;
            int rc_tail = acars_tail_feed(ctx->sbd, ctx->reasm, ctx->ida_reasm,
                                          &ida, dir == IR_FRM_DIR_UPLINK,
                                          ctx->burst_freq_hz, ctx->burst_ts_us,
                                          &tail);
            // A6 hot-bin bookkeeping (mirrors frame_decoder.c's publish/clear
            // around ida_reassembler_feed_ex).
            if (rc_tail == 0)
                hot_publish(ctx->center_bin, ctx->burst_ts_us);
            else if (rc_tail == 1 && !(ida.da_ctr == 0 && ida.da_cont == 0))
                hot_clear(ctx->center_bin);

            if (rc_tail == 1 && tail.sbd_ready) {
                ctx->sbd_messages++;
                printf("SBD: type=%s %s len=%u (msg %u/%u) payload=",
                       sbd_type_wire_name(tail.sbd.type),
                       tail.sbd.uplink ? "UL" : "DL", tail.sbd.payload_len,
                       tail.sbd.msg_no, tail.sbd.msg_count);
                print_hex(tail.sbd.payload, tail.sbd.payload_len);
                printf("\n");
                if (ctx->expect) {
                    char hex[2 * 320 + 1];
                    for (int i = 0; i < tail.sbd.payload_len && i < 320; i++)
                        sprintf(hex + 2 * i, "%02x", tail.sbd.payload[i]);
                    if (strstr(hex, ctx->expect)) ctx->hit = 1;
                }
                if (tail.acars_ready) {
                    ctx->acars_decoded++;
                    // strip libacars's '.'-padding from reg (frame_decoder.c)
                    const char *reg = tail.reg;
                    while (*reg == '.') reg++;
                    printf("ACARS: %s mode=%c reg=%s label='%.2s' block=%c "
                           "msgnum='%.4s' flight='%.6s' crc=%s txt=\"%s\"\n",
                           tail.sbd.uplink ? "UL" : "DL",
                           tail.mode ? tail.mode : '?', reg, tail.label,
                           tail.block_id ? tail.block_id : '?', tail.msg_num,
                           tail.flight_id, tail.crc_ok ? "OK" : "BAD",
                           tail.txt);
                    if (ctx->expect && (strstr(reg, ctx->expect) ||
                                        strstr(tail.flight_id, ctx->expect) ||
                                        strstr(tail.txt, ctx->expect)))
                        ctx->hit = 1;
                }
            }
        }
        break;
    default: ctx->class_unknown++; break;
    }
out:
    free(res->frame.bits);
    free(res->frame.soft_bits);
}

static int cmp_tag_start(const void *a_, const void *b_);

typedef struct {
    uint64_t start, stop;
    int      center_bin;
    int      width_bins;
} tag_t;

static int cmp_tag_start(const void *a_, const void *b_)
{
    const tag_t *a = (const tag_t *)a_, *b = (const tag_t *)b_;
    if (a->start < b->start) return -1;
    if (a->start > b->start) return 1;
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <slice_2500k.ci16> [expect_substr]\n", argv[0]);
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
    int      n25  = (int)(bytes / 4); // interleaved int16 I,Q
    int16_t *iq25 = malloc((size_t)n25 * 2 * sizeof(int16_t));
    if (!iq25) {
        fprintf(stderr, "oom iq25\n");
        return 1;
    }
    if (fread(iq25, sizeof(int16_t), (size_t)n25 * 2, fp) != (size_t)n25 * 2) {
        fprintf(stderr, "short read\n");
        return 1;
    }
    fclose(fp);
    // Optional amplitude scaling (quantisation-headroom diagnosis only).
    int scale = getenv("SCALE") ? atoi(getenv("SCALE")) : 1;
    if (scale > 1) {
        for (long k = 0; k < (long)n25 * 2; k++) {
            int32_t v = (int32_t)iq25[k] * scale;
            iq25[k]   = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
        }
        printf("SCALE=%d applied\n", scale);
    }
    printf("Loaded %d complex @ 2.5 MSPS (%.3f s) from %s\n", n25,
           (double)n25 / 2.5e6, path);

    // Tagger — device Path-A parameters.
    float tag_thr = getenv("TAG_THR") ? (float)atof(getenv("TAG_THR")) : 10.0f;
    printf("Tagger: thr=%.1f dB pre=%d post=%d width=32\n", (double)tag_thr,
           BURST_PRE_LEN, BURST_POST_LEN);
    fft_burst_tagger_t *t = fft_burst_tagger_init(BURST_PRE_LEN, BURST_POST_LEN,
                                                  /*burst_width=*/32, tag_thr,
                                                  s_baseline_history);
    if (!t) {
        fprintf(stderr, "tagger init\n");
        return 1;
    }
    fft_burst_tagger_set_start(t, 0);

    enum { MAX_TAGS = 8192 };
    tag_t *tags   = malloc(sizeof(tag_t) * MAX_TAGS);
    int    n_tags = 0;
    fbt_burst_t new_b[FBT_MAX_BURSTS], gone_b[FBT_MAX_BURSTS];
    for (int off = 0; off + FBT_FFT_SIZE <= n25; off += FBT_FFT_SIZE) {
        int n_new = FBT_MAX_BURSTS, n_gone = FBT_MAX_BURSTS;
        fft_burst_tagger_step(t, iq25 + off * 2, NULL, new_b, &n_new, gone_b,
                              &n_gone);
        for (int i = 0; i < n_gone && n_tags < MAX_TAGS; i++) {
            tags[n_tags++] = (tag_t){ .start      = gone_b[i].start,
                                      .stop       = gone_b[i].stop,
                                      .center_bin = gone_b[i].center_bin,
                                      .width_bins = gone_b[i].width_bins };
        }
    }
    { // flush still-active bursts at end-of-stream
        int         n_flush = FBT_MAX_BURSTS;
        fbt_burst_t flushed[FBT_MAX_BURSTS];
        fft_burst_tagger_flush(t, flushed, &n_flush);
        for (int i = 0; i < n_flush && n_tags < MAX_TAGS; i++) {
            tags[n_tags++] = (tag_t){ .start      = flushed[i].start,
                                      .stop       = flushed[i].stop,
                                      .center_bin = flushed[i].center_bin,
                                      .width_bins = flushed[i].width_bins };
        }
    }
    printf("Tagger emitted %d bursts\n", n_tags);
    // Feed the reassembler in RF-time order (gone-event order can invert
    // start order for overlapping bursts; the IDA chain gap gate assumes
    // roughly monotonic timestamps, like the device's queue order).
    qsort(tags, (size_t)n_tags, sizeof(tag_t), cmp_tag_start);

    // Per-burst decode chain buffers.
    direct_if_decim_t dec;
    direct_if_decim_init(&dec);
    int16_t *w25  = malloc(2 * BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *w250 = malloc(2 * BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *si   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *sq   = malloc(BURST_WINDOW_LEN * sizeof(int16_t));
    int16_t *so   = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    int16_t *sp   = malloc(BURST_WINDOW_250K * sizeof(int16_t));
    if (!w25 || !w250 || !si || !sq || !so || !sp) {
        fprintf(stderr, "alloc\n");
        return 1;
    }

    sbd_reassembler_t sbd;
    sbd_reassembler_init(&sbd);
    ida_reassembler_t ida_reasm;
    ida_reassembler_init(&ida_reasm);
    la_reasm_ctx *reasm = la_reasm_ctx_new();
    if (!reasm) {
        fprintf(stderr, "la_reasm_ctx_new\n");
        return 1;
    }

    bool pf_enabled = !(getenv("PF") && atoi(getenv("PF")) == 0);
    printf("burst_prefilter: %s (PF_THRESH_DB=%.1f, hot-bin SNR exemption on)\n",
           pf_enabled ? "ON (production gate)" : "OFF (PF=0)",
           (double)PF_THRESH_DB);

    int pipeline_ok = 0, frames_total = 0;
    int pf_rej = 0, pf_rej_width = 0, pf_rej_dur = 0, pf_rej_snr = 0;
    int pf_hot_rescued = 0;
    decode_ctx_t acc = {0}; // accumulated counters
    acc.sbd          = &sbd;
    acc.ida_reasm    = &ida_reasm;
    acc.reasm        = reasm;
    acc.expect       = expect;
    uint64_t last_ts = 0;

    for (int i = 0; i < n_tags; i++) {
        int64_t begin = (int64_t)tags[i].start, end = (int64_t)tags[i].stop;
        if (begin < 0 || end > n25 || end <= begin) continue;
        int win_len = (int)(end - begin);
        if (win_len > BURST_WINDOW_LEN) win_len = BURST_WINDOW_LEN;
        win_len -= win_len % DIDECIM_DECIM;
        if (win_len < DIDECIM_DECIM) continue;

        memcpy(w25, iq25 + begin * 2, (size_t)win_len * 2 * sizeof(int16_t));
        double ps = rotate_to_dc_phase_step_from_bin(tags[i].center_bin,
                                                     FBT_FFT_SIZE);
        rotate_to_dc_q15_simd(w25, win_len, ps);

        direct_if_decim_reset_state(&dec);
        int n_out = direct_if_decim_process_split(&dec, w25, win_len, w250, si,
                                                  sq, so, sp);
        if (n_out <= 64) continue; // worker_core1 "too short" gate

        uint64_t ts_us = SAMPLE_TO_US(begin);
        last_ts        = ts_us;

        // P1.5b production prefilter (worker_core1.c step 3), incl. the A6
        // hot-channel SNR-only escalation and the (currently 0 dB) global
        // SNR margin. Width/duration rejects always stand.
        if (pf_enabled) {
            burst_prefilter_result_t pf;
            bool ok = burst_prefilter(w250, n_out, tags[i].width_bins, &pf);
            if (!ok) {
                bool rescue = false;
                if (hot_match(tags[i].center_bin, ts_us) && pf.width_ok &&
                    pf.dur_ok && !pf.snr_ok) {
                    rescue = true; // open-chain channel, SNR-only failure
                    pf_hot_rescued++;
                }
                if (!rescue) {
                    pf_rej++;
                    if (!pf.width_ok) pf_rej_width++;
                    else if (!pf.dur_ok) pf_rej_dur++;
                    else pf_rej_snr++;
                    printf("PF-REJECT tag=%d bin=%d t=%.3fs width=%d(%s) "
                           "active=%d(%s) chsnr=%.1fdB(%s)\n",
                           i, tags[i].center_bin, (double)ts_us / 1e6,
                           pf.width_bins, pf.width_ok ? "ok" : "FAIL",
                           pf.active_len, pf.dur_ok ? "ok" : "FAIL",
                           (double)pf.channel_snr_db, pf.snr_ok ? "ok" : "FAIL");
                    continue;
                }
            }
        }

        decode_ctx_t dctx  = acc; // copy accumulated pointers/counters
        dctx.center_bin    = tags[i].center_bin;
        dctx.burst_ts_us   = ts_us;
        // Channel-identity key for ida_reasm's ±5 kHz deadband: bin → Hz
        // offset from the band edge (always positive, monotonic in bin, so
        // adjacent-channel chains can't collide the way |offset| could).
        dctx.burst_freq_hz = (uint32_t)((int64_t)tags[i].center_bin *
                                        INPUT_FS_HZ / FBT_FFT_SIZE);

        int frames = burst_pipeline_process_burst(w250, n_out, on_frame, &dctx);
        if (frames > 0) pipeline_ok++;
        frames_total += frames;
        acc = dctx; // carry counters (incl. hit) forward
    }

    // End-of-stream: reap any chain still open so incomplete multi-burst
    // messages are visible as PARTIAL salvage, not silently lost.
    int salvaged = 0;
    {
        ida_salvage_t sv;
        while (ida_reassembler_reap(&ida_reasm, last_ts + 2 * IDA_REASM_SESSION_TIMEOUT_US,
                                    &sv)) {
            salvaged++;
            printf("SALVAGE (incomplete chain): frags=%u dirty=%d len=%d pay=",
                   sv.frags, sv.dirty, sv.payload_len);
            print_hex(sv.payload, (size_t)sv.payload_len);
            printf("\n");
        }
    }

    printf("  ida_reasm: standalone=%u opened=%u merged=%u completed=%u "
           "orphan=%u overflow=%u expired=%u\n",
           ida_reasm.cnt_standalone, ida_reasm.cnt_opened, ida_reasm.cnt_merged,
           ida_reasm.cnt_completed, ida_reasm.cnt_orphan, ida_reasm.cnt_overflow,
           ida_reasm.cnt_expired);
    la_reasm_ctx_destroy(reasm);
    fft_burst_tagger_destroy(t);

    printf("\n== %s summary ==\n"
           "  bursts tagged    : %d\n"
           "  prefilter reject : %d (width=%d dur=%d snr=%d) hot-rescued=%d\n"
           "  pipeline ok      : %d\n"
           "  frames decoded   : %d\n"
           "  classified       : MS=%d TL=%d BC=%d RA=%d LW.other=%d unk=%d\n"
           "  LW.DA frames     : %d (clean bch+hdr+crc: %d, dirty-crc: %d)\n"
           "  SBD messages     : %d\n"
           "  ACARS decoded    : %d\n"
           "  salvage (partial): %d\n",
           path, n_tags, pf_rej, pf_rej_width, pf_rej_dur, pf_rej_snr,
           pf_hot_rescued, pipeline_ok, frames_total, acc.class_ms,
           acc.class_tl, acc.class_bc, acc.class_ra, acc.class_lw_other,
           acc.class_unknown, acc.lw_da, acc.lw_da_clean, acc.lw_da_dirty,
           acc.sbd_messages, acc.acars_decoded, salvaged);

    free(iq25);
    free(tags);
    free(w25);
    free(w250);
    free(si);
    free(sq);
    free(so);
    free(sp);

    if (expect) {
        if (acc.hit) {
            printf("PASS: found expected substring \"%s\"\n", expect);
            return 0;
        }
        printf("FAIL: expected substring \"%s\" not found\n", expect);
        return 1;
    }
    return 0;
}
