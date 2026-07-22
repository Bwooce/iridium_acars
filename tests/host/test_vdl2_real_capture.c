// test_vdl2_real_capture — real-signal smoke for the VDL2 D8PSK demod.
//
// Input: the sigidwiki over-the-air VDL2 capture, pre-converted to
// 210 ksps S16_LE interleaved IQ (channel-order corrected — the source
// WAV has I/Q REVERSED; reading it naively mirror-images the spectrum
// and every burst syncs-but-fails-decode):
//   /Users/bruce/iridium_capture/vdl2_ref/vdl2_sigidwiki_210k_s16le.raw
// (override with env VDL2_RAW_PATH). The dumpvdl2 golden decode of the
// SAME capture (vdl2_sigidwiki_golden_decode.txt: 40 AVLC frames,
// 9 ACARS messages, per-frame SNR ~23-31 dB) is the reference this
// smoke is calibrated against; full frame-hex identity is checked at
// L2 integration, this test validates the DEMOD layer on real RF:
//
//   1. resample 210 k -> 250 k (25/21) to enter the production demod
//      through its band_pipeline-rate front door,
//   2. scan the whole capture, demodulating every locked burst,
//   3. for every complete single-RS-block frame, pack the descrambled
//      bits to octets (LSB-first, dumpvdl2 bitstream_read_lsbfirst
//      convention) and run the in-tree RS(255,249) decoder
//      (common/vdl2/rs_vdl2.h) over the block. An RS-clean block means
//      the demodulated bytes are EXACTLY what was transmitted — an
//      end-to-end air-truth check of resampler + sync + slicer +
//      descrambler + header decode with no synthetic assumptions.
//
// If the capture file is absent the test SKIPs (exit 0) — same
// fixture-dependent convention as the other capture-driven tests.
//
// Assertions (floors, golden-calibrated): >= 30 complete frames and
// >= 30 RS-clean blocks over the capture (golden: 40 AVLC frames; a
// burst can carry several AVLC frames, so burst count <= 40).

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "firmr_s16.h"
#include "rs_vdl2.h"
#include "vdl2_demod.h"

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

// 210 k -> 250 k: interp 25 / decim 21. Wide lowpass (60 kHz): the
// demod's own 9 kHz channel filter does the selection; this stage only
// suppresses the interpolation images.
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
            // Halve the input: full-scale WAV + Q15 filter ripple would
            // wrap the resampler's unsaturated int16 cast.
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

// Pack bits (1/byte) to octets LSB-first (dumpvdl2
// bitstream_read_lsbfirst, bitstream.c:70-81).
static void pack_lsbfirst(const uint8_t *bits, int n_octets, uint8_t *out)
{
    for (int i = 0; i < n_octets; i++) {
        uint8_t b = 0;
        for (int j = 0; j < 8; j++)
            b |= (uint8_t)((bits[8 * i + j] & 1u) << j);
        out[i] = b;
    }
}

// RS-verify a complete single-block frame's descrambled bits. Returns
// -2 multi-block (skipped), -1 RS fail, else corrected-symbol count.
static int rs_check_frame(const vdl2_demod_result_t *r)
{
    uint32_t octets = (r->datalen_bits + 7) / 8;
    if (octets > RS_VDL2_K) return -2;
    int fec;
    if (octets < 3)
        return -2; // uncoded (can't happen: header rejects fec==0)
    else if (octets < 31)
        fec = 2;
    else if (octets < 68)
        fec = 4;
    else
        fec = 6;
    uint8_t block[RS_VDL2_N];
    memset(block, 0, sizeof(block));
    pack_lsbfirst(r->bits + VDL2_HDR_BITS, (int)octets, block);
    pack_lsbfirst(r->bits + VDL2_HDR_BITS + 8 * octets, fec,
                  block + RS_VDL2_K);
    int n_corr = 0;
    if (rs_vdl2_decode_shortened(block, (int)octets, &n_corr) != 0) return -1;
    return n_corr;
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
    int      n210 = (int)(bytes / 4); // complex samples
    int16_t *raw  = (int16_t *)malloc((size_t)n210 * 2 * sizeof(int16_t));
    if (!raw || (long)fread(raw, 4, (size_t)n210, fp) != (long)n210) {
        printf("FAIL: could not read %s\n", path);
        fclose(fp);
        return 1;
    }
    fclose(fp);
    printf("capture: %d complex @ %d Hz (%.1f s)\n", n210, FS_RAW,
           (double)n210 / FS_RAW);

    if (!vdl2_lpf_design_q15(s_up_coeffs, UP_DSIZE, 25,
                             60000.0 / ((double)FS_RAW * 25), 8.0)) {
        printf("FAIL: upsampler design alloc\n");
        return 1;
    }
    int      max250 = (int)((int64_t)n210 * 25 / 21) + 8;
    int16_t *iq250  = (int16_t *)malloc((size_t)max250 * 2 * sizeof(int16_t));
    int      n250   = resample_210_to_250(raw, n210, iq250, max250);
    free(raw);
    printf("resampled: %d complex @ 250000 Hz\n", n250);

    // Windowed scan: 2 s windows, 0.25 s step-back overlap so a burst
    // split by a window boundary is retried whole in the next window.
    const int WIN = 500000, OVERLAP = 62500;
    int cursor = 0, n_sync = 0, n_complete = 0, n_rs_ok = 0, n_rs_fail = 0,
        n_multiblock = 0, n_corr_total = 0, n_flag7e = 0;
    double snr_min = 1e9, snr_max = -1e9;
    while (cursor < n250 - 1000) {
        int len = n250 - cursor;
        if (len > WIN) len = WIN;
        vdl2_demod_result_t r;
        if (!vdl2_demod_burst(iq250 + 2 * (size_t)cursor, len, &r)) {
            if (len < WIN) break; // tail exhausted
            cursor += WIN - OVERLAP;
            continue;
        }
        n_sync++;
        if (r.complete) {
            n_complete++;
            int rs = rs_check_frame(&r);
            if (rs == -2) {
                n_multiblock++;
            } else if (rs < 0) {
                n_rs_fail++;
            } else {
                n_rs_ok++;
                n_corr_total += rs;
            }
            // AVLC starts with a 0x7E flag: descrambled body byte 0 is
            // 01111110 LSB-first.
            uint8_t first = 0;
            pack_lsbfirst(r.bits + VDL2_HDR_BITS, 1, &first);
            if (first == 0x7E) n_flag7e++;
            if (r.snr_db < snr_min) snr_min = r.snr_db;
            if (r.snr_db > snr_max) snr_max = r.snr_db;
            printf("frame @%.3fs: datalen %u bits, hdr_corr %d, "
                   "cfo %+.0f Hz, evm %.3f rad, snr %.1f dB, rs %s(%d), "
                   "body[0]=0x%02X\n",
                   (double)(cursor + r.consumed_complex_250k) / 250000.0,
                   r.datalen_bits, r.hdr_synd_weight, (double)r.cfo_hz,
                   (double)r.evm_rms, (double)r.snr_db,
                   rs == -2 ? "SKIP" : (rs < 0 ? "FAIL" : "ok"), rs, first);
        } else {
            printf("truncated frame @%.3fs: %d/%d bits\n",
                   (double)cursor / 250000.0, r.n_bits, r.n_bits_needed);
        }
        free(r.bits);
        free(r.soft_bits);
        cursor += r.consumed_complex_250k;
    }

    printf("SUMMARY: sync %d, complete %d, rs_ok %d (corr %d), rs_fail %d, "
           "multiblock %d, avlc-flag-first %d, snr %.1f..%.1f dB\n",
           n_sync, n_complete, n_rs_ok, n_corr_total, n_rs_fail, n_multiblock,
           n_flag7e, snr_min, snr_max);

    // Floors calibrated against the dumpvdl2 golden decode of this
    // capture (40 AVLC frames across the transmissions; strong signal).
    CHECK(n_complete >= 30, "complete frames %d < 30", n_complete);
    CHECK(n_rs_ok >= 30, "RS-clean frames %d < 30", n_rs_ok);
    CHECK(n_flag7e >= 30, "frames starting with AVLC flag %d < 30", n_flag7e);

    free(iq250);
    if (g_fails) {
        printf("FAIL: %d check(s)\n", g_fails);
        return 1;
    }
    printf("PASS: vdl2 demod real-capture smoke\n");
    return 0;
}
