// test_vdl2_demod — synthetic round-trip validation of the VDL2 D8PSK
// demodulator (common/vdl2_decoder/vdl2_demod.c) against the
// independent test modulator (vdl2_mod.c).
//
// No real VDL2 capture is baked in here (the real-signal smoke lives in
// test_vdl2_real_capture.c); this suite proves, deterministically:
//   - the (25,20) header block code round-trips and corrects any
//     single-bit error (constants from dumpvdl2 decode.c:55-100),
//   - the transmission-length -> body-bits arithmetic pins dumpvdl2's
//     get_fec_octetcount()/RS-block rules at every boundary,
//   - the scrambler is an involution (and mod/demod use two independent
//     implementations of it, so round-trip identity cross-checks them),
//   - modulate -> demodulate recovers the EXACT transmitted bits at
//     clean SNR, under carrier offset (+/-1.5 kHz), under fractional
//     symbol-timing offsets, and under AWGN with margin,
//   - pure noise never produces a false frame,
//   - truncated bursts come back flagged incomplete with a
//     prefix-identical bit vector,
//   - the vdl2_pipeline vtable demodulates two back-to-back bursts in
//     one window (CSMA merge) and honours the callback-owns-bits
//     contract.
//
// Checks are real if/fail-count checks (NOT bare assert — a Release
// build must not void them).

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "band_pipeline.h"
#include "vdl2_demod.h"
#include "vdl2_mod.h"
#include "vdl2_pipeline.h"

static int g_fails = 0;
#define CHECK(cond, ...)                             \
    do {                                             \
        if (!(cond)) {                               \
            printf("FAIL %s:%d: ", __func__, __LINE__); \
            printf(__VA_ARGS__);                     \
            printf("\n");                            \
            g_fails++;                               \
        }                                            \
    } while (0)

static uint32_t s_rng = 0xC0FFEE01u;
static uint32_t rnd(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_rng;
}

static void make_body(uint8_t *body, int n, uint32_t seed)
{
    uint32_t s = seed ? seed : 1;
    for (int i = 0; i < n; i++) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        body[i] = (uint8_t)(s & 1u);
    }
}

// Eb/N0 (dB) -> AWGN sigma for the modulator, from
// Eb/N0 = (amp^2 / bitrate) * fs / (2 sigma^2) at strobe amplitude amp.
static double ebn0_to_sigma(double ebn0_db, double amp, double fs)
{
    return amp * sqrt(fs / (2.0 * 3.0 * (double)VDL2_SYMBOL_RATE_HZ)) *
           pow(10.0, -ebn0_db / 20.0);
}

#define MOD_MAX_COMPLEX 80000

// Modulate one burst + demod it + compare. Returns demod success;
// fills *res (caller frees bits/soft_bits on success). With mism_out ==
// NULL every comparison is a hard CHECK; with mism_out != NULL the
// comparisons are quiet and the bit-mismatch count is reported instead
// (for the informational low-SNR waterline sweep).
static bool roundtrip_ex(uint32_t datalen, const vdl2_mod_params_t *p,
                         uint32_t body_seed, vdl2_demod_result_t *res,
                         int truncate_permille, int *mism_out)
{
    int body_n = vdl2_burst_body_bits(datalen);
    if (body_n < 0) {
        printf("bad datalen %u in roundtrip\n", datalen);
        g_fails++;
        return false;
    }
    uint8_t *body = (uint8_t *)malloc((size_t)body_n);
    int16_t *iq   = (int16_t *)malloc(MOD_MAX_COMPLEX * 2 * sizeof(int16_t));
    make_body(body, body_n, body_seed);
    int n = vdl2_mod_burst(datalen, body, p, iq, MOD_MAX_COMPLEX);
    if (n < 0) {
        printf("modulator overflow/args (datalen %u)\n", datalen);
        g_fails++;
        free(body);
        free(iq);
        return false;
    }
    if (truncate_permille > 0 && truncate_permille < 1000)
        n = (int)((int64_t)n * truncate_permille / 1000);

    bool ok = vdl2_demod_burst(iq, n, res);
    if (ok) {
        // Bit identity over whatever was demodulated: header must
        // re-encode to the transmitted word; body prefix must match.
        uint32_t hdr_tx = vdl2_hdr_encode(datalen);
        int      mism   = 0;
        for (int k = 0; k < VDL2_HDR_BITS && k < res->n_bits; k++)
            if (res->bits[k] != ((hdr_tx >> (VDL2_HDR_BITS - 1 - k)) & 1u))
                mism++;
        for (int k = VDL2_HDR_BITS; k < res->n_bits; k++)
            if (res->bits[k] != body[k - VDL2_HDR_BITS]) mism++;
        if (mism_out) {
            *mism_out = (res->datalen_bits == datalen) ? mism : -1;
        } else {
            CHECK(res->datalen_bits == datalen, "datalen %u != %u",
                  res->datalen_bits, datalen);
            CHECK(res->n_bits_needed == VDL2_HDR_BITS + body_n,
                  "needed %d != %d", res->n_bits_needed,
                  VDL2_HDR_BITS + body_n);
            CHECK(mism == 0, "%d bit mismatches (datalen %u, n_bits %d)",
                  mism, datalen, res->n_bits);
            // Soft-bit sign convention: bit 0 -> non-negative.
            int soft_bad = 0;
            for (int k = 0; k < res->n_bits; k++) {
                if (res->bits[k] && res->soft_bits[k] > 0) soft_bad++;
                if (!res->bits[k] && res->soft_bits[k] < 0) soft_bad++;
            }
            CHECK(soft_bad == 0, "%d soft-bit sign violations", soft_bad);
        }
    }
    free(body);
    free(iq);
    return ok;
}

static bool roundtrip(uint32_t datalen, const vdl2_mod_params_t *p,
                      uint32_t body_seed, vdl2_demod_result_t *res,
                      int truncate_permille)
{
    return roundtrip_ex(datalen, p, body_seed, res, truncate_permille, NULL);
}

static void t_header_codec(void)
{
    const uint32_t lens[] = {17, 131, 300, 992, 0x1FFF, 0x3FFF};
    for (size_t i = 0; i < sizeof(lens) / sizeof(lens[0]); i++) {
        uint32_t w = vdl2_hdr_encode(lens[i]);
        CHECK((w >> 22) == 0, "reserved bits set in encode(%u)", lens[i]);
        uint32_t got = 0;
        uint32_t ww  = w;
        int      sw  = vdl2_hdr_decode(&ww, &got);
        CHECK(sw == 0, "clean word syndrome %d", sw);
        CHECK(got == lens[i], "decode %u != %u", got, lens[i]);
        // Every single-bit channel error must be corrected. Flips in
        // the 3 reserved bits (b 22..24) are silently erased BEFORE the
        // syndrome (dumpvdl2 decode.c:209 forces them to 0), so those
        // decode clean with weight 0; all others must be flagged.
        for (int b = 0; b < VDL2_HDR_BITS; b++) {
            uint32_t flip = w ^ (1u << b);
            got           = 0;
            sw            = vdl2_hdr_decode(&flip, &got);
            if (b >= VDL2_HDR_TRLEN_BITS + VDL2_HDR_FEC_BITS) {
                CHECK(sw == 0, "reserved-bit flip %d not masked", b);
            } else {
                CHECK(sw >= 1, "flip bit %d not flagged (len %u)", b,
                      lens[i]);
            }
            CHECK(got == lens[i], "flip bit %d miscorrected: %u != %u", b,
                  got, lens[i]);
        }
    }
}

static void t_body_bits(void)
{
    // Boundary pins for dumpvdl2 decode.c get_fec_octetcount() + the
    // fec_octets==0 reject (decode.c:124-133, 250-255).
    CHECK(vdl2_burst_body_bits(0) == -1, "datalen 0 accepted");
    CHECK(vdl2_burst_body_bits(16) == -1, "2 octets (no FEC) accepted");
    CHECK(vdl2_burst_body_bits(17) == 8 * (3 + 2), "3 octets");
    CHECK(vdl2_burst_body_bits(240) == 8 * (30 + 2), "30 octets");
    CHECK(vdl2_burst_body_bits(241) == 8 * (31 + 4), "31 octets");
    CHECK(vdl2_burst_body_bits(536) == 8 * (67 + 4), "67 octets");
    CHECK(vdl2_burst_body_bits(537) == 8 * (68 + 6), "68 octets");
    CHECK(vdl2_burst_body_bits(1992) == 8 * (249 + 6), "full RS block");
    CHECK(vdl2_burst_body_bits(2000) == 8 * (250 + 6),
          "250 octets: 2nd block <3 octets carries no FEC");
    CHECK(vdl2_burst_body_bits(2016) == 8 * (252 + 6 + 2), "249+3 octets");
    CHECK(vdl2_burst_body_bits(0x3FFF) > 0, "max length rejected");
    CHECK(vdl2_burst_body_bits(0x4000) == -1, "over-max accepted");
}

static void t_scrambler(void)
{
    uint8_t  a[257], b[257];
    for (int i = 0; i < 257; i++)
        a[i] = b[i] = (uint8_t)(rnd() & 1u);
    uint16_t l1 = VDL2_LFSR_IV;
    vdl2_scramble(a, 257, &l1);
    int changed = 0;
    for (int i = 0; i < 257; i++)
        changed += (a[i] != b[i]);
    CHECK(changed > 64, "keystream suspiciously sparse (%d flips)", changed);
    uint16_t l2 = VDL2_LFSR_IV;
    vdl2_scramble(a, 257, &l2);
    CHECK(memcmp(a, b, 257) == 0, "scramble not an involution");
    CHECK(l1 == l2, "LFSR state diverged");
    // Split application == one-shot application (position property the
    // demod's header/body two-step relies on).
    uint16_t l3 = VDL2_LFSR_IV;
    vdl2_scramble(a, 100, &l3);
    vdl2_scramble(a + 100, 157, &l3);
    uint16_t l4 = VDL2_LFSR_IV;
    vdl2_scramble(b, 257, &l4);
    CHECK(memcmp(a, b, 257) == 0, "split scramble differs");
}

static void t_clean_roundtrip(void)
{
    vdl2_mod_params_t p;
    const uint32_t    lens[]   = {200, 992, 1992, 2500};
    const double      phases[] = {0.0, 1.3, -2.0, 2.9};
    for (size_t i = 0; i < sizeof(lens) / sizeof(lens[0]); i++) {
        vdl2_mod_params_default(&p);
        p.phase0_rad = phases[i];
        vdl2_demod_result_t r;
        bool ok = roundtrip(lens[i], &p, 0x1234 + (uint32_t)i, &r, 0);
        CHECK(ok, "clean demod failed (datalen %u)", lens[i]);
        if (!ok) continue;
        CHECK(r.complete, "clean frame incomplete (datalen %u)", lens[i]);
        CHECK(r.hdr_synd_weight == 0, "clean header corrected?");
        CHECK(fabsf(r.cfo_hz) < 30.f, "cfo est %.1f Hz at 0", (double)r.cfo_hz);
        CHECK(r.snr_db > 20.f, "clean snr proxy %.1f dB", (double)r.snr_db);
        free(r.bits);
        free(r.soft_bits);
    }
}

static void t_cfo(void)
{
    const double cfos[] = {-1500, -600, -200, 200, 600, 1500};
    for (size_t i = 0; i < sizeof(cfos) / sizeof(cfos[0]); i++) {
        vdl2_mod_params_t p;
        vdl2_mod_params_default(&p);
        p.cfo_hz = cfos[i];
        vdl2_demod_result_t r;
        bool ok = roundtrip(992, &p, 0x77, &r, 0);
        CHECK(ok && r.complete, "cfo %.0f Hz: demod failed", cfos[i]);
        if (!ok) continue;
        CHECK(fabs((double)r.cfo_hz - cfos[i]) < 60.0,
              "cfo est %.1f vs true %.0f", (double)r.cfo_hz, cfos[i]);
        free(r.bits);
        free(r.soft_bits);
    }
}

static void t_timing(void)
{
    const double fracs[] = {0.0, 0.25, 0.5, 0.75};
    for (size_t i = 0; i < sizeof(fracs) / sizeof(fracs[0]); i++) {
        vdl2_mod_params_t p;
        vdl2_mod_params_default(&p);
        p.timing_frac = fracs[i];
        vdl2_demod_result_t r;
        bool ok = roundtrip(992, &p, 0x88, &r, 0);
        CHECK(ok && r.complete, "timing %.2f: demod failed", fracs[i]);
        if (ok) {
            free(r.bits);
            free(r.soft_bits);
        }
    }
}

static void t_awgn(void)
{
    // Gate 1 — bit-IDENTITY over the whole 1065-bit frame at Eb/N0 =
    // 24 dB, 20/20 trials, with a +/-400 Hz offset stacked on. Why
    // 24 dB and not lower: differential detection (+~3 dB vs coherent)
    // through a non-matched ~9 kHz channel filter (+~2 dB) means the
    // per-symbol error rate only reaches "zero errors in 355 symbols,
    // every trial" around 22 dB (measured waterline below; matches the
    // 2Q(pi/8 / sigma_phi) hand calculation). dumpvdl2 has the same
    // receive structure, and the real-capture reference frames run
    // 23-31 dB — this is the regime the demod actually serves.
    int pass_hi = 0;
    for (int t = 0; t < 20; t++) {
        vdl2_mod_params_t p;
        vdl2_mod_params_default(&p);
        p.seed       = 1000 + (uint32_t)t;
        p.awgn_sigma = ebn0_to_sigma(24.0, p.amp, p.fs_hz);
        p.cfo_hz     = (t % 2) ? 400.0 : -400.0; // noise + offset together
        vdl2_demod_result_t r;
        int mism = -1;
        if (roundtrip_ex(992, &p, 0x9000 + (uint32_t)t, &r, 0, &mism)) {
            if (r.complete && mism == 0) pass_hi++;
            free(r.bits);
            free(r.soft_bits);
        }
    }
    CHECK(pass_hi == 20, "Eb/N0 24 dB: %d/20 bit-exact", pass_hi);

    // Gate 2 — RS-serviceable at Eb/N0 = 18 dB: the downstream
    // RS(255,249) corrects 3 symbols/block, so a frame with every
    // mismatch confined to <= 3 octet-aligned symbols is still a
    // delivered frame. Floor: >= 8/10 trials sync AND land within
    // 24 mismatched bits.
    int sync18 = 0, ok18 = 0;
    for (int t = 0; t < 10; t++) {
        vdl2_mod_params_t p;
        vdl2_mod_params_default(&p);
        p.seed       = 2000 + (uint32_t)t;
        p.awgn_sigma = ebn0_to_sigma(18.0, p.amp, p.fs_hz);
        vdl2_demod_result_t r;
        int mism = -1;
        if (roundtrip_ex(992, &p, 0x8000 + (uint32_t)t, &r, 0, &mism)) {
            sync18++;
            if (r.complete && mism >= 0 && mism <= 24) ok18++;
            free(r.bits);
            free(r.soft_bits);
        }
    }
    CHECK(sync18 >= 8, "Eb/N0 18 dB: only %d/10 synced", sync18);
    CHECK(ok18 >= 8, "Eb/N0 18 dB: only %d/10 within RS-scale damage", ok18);

    // Waterline report (informational, no assert): where bit-identity
    // starts to crumble.
    for (double ebn0 = 22.0; ebn0 >= 10.0; ebn0 -= 2.0) {
        int pass = 0, sync = 0, worst = 0;
        for (int t = 0; t < 10; t++) {
            vdl2_mod_params_t p;
            vdl2_mod_params_default(&p);
            p.seed       = 4000 + (uint32_t)t;
            p.awgn_sigma = ebn0_to_sigma(ebn0, p.amp, p.fs_hz);
            vdl2_demod_result_t r;
            int mism = -1;
            if (roundtrip_ex(992, &p, 0xA000 + (uint32_t)t, &r, 0, &mism)) {
                sync++;
                if (r.complete && mism == 0) pass++;
                if (mism > worst) worst = mism;
                free(r.bits);
                free(r.soft_bits);
            }
        }
        printf("INFO: Eb/N0 %.0f dB: sync %d/10, bit-exact %d/10, "
               "worst mismatch %d bits\n",
               ebn0, sync, pass, worst);
    }
}

static void t_false_sync_on_noise(void)
{
    int false_syncs = 0;
    for (int t = 0; t < 50; t++) {
        int16_t *iq = (int16_t *)malloc(30000 * 2 * sizeof(int16_t));
        uint32_t s  = 0xBEEF0000 + (uint32_t)t;
        for (int i = 0; i < 30000 * 2; i++) {
            // Uniform-ish noise, sigma ~2300 LSB.
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            iq[i] = (int16_t)((int32_t)(s & 0x1FFF) - 4096);
        }
        vdl2_demod_result_t r;
        if (vdl2_demod_burst(iq, 30000, &r)) {
            false_syncs++;
            free(r.bits);
            free(r.soft_bits);
        }
        free(iq);
    }
    CHECK(false_syncs == 0, "%d false frames on pure noise", false_syncs);
}

static void t_truncated(void)
{
    vdl2_mod_params_t p;
    vdl2_mod_params_default(&p);
    vdl2_demod_result_t r;
    bool ok = roundtrip(992, &p, 0x55, &r, 550); // keep 55% of the burst
    CHECK(ok, "truncated burst: header should still lock");
    if (ok) {
        CHECK(!r.complete, "truncated burst reported complete");
        CHECK(r.n_bits < r.n_bits_needed, "n_bits %d !< needed %d", r.n_bits,
              r.n_bits_needed);
        CHECK(r.n_bits > VDL2_HDR_BITS, "no body bits demodulated");
        free(r.bits);
        free(r.soft_bits);
    }
}

// ---- pipeline vtable: two back-to-back bursts in one window ----

typedef struct {
    int      n_frames;
    int      n_complete;
    int      n_bits[4];
    uint8_t *bits[4];
} sink_t;

static void sink_cb(band_frame_t *f, void *ctx)
{
    sink_t *s = (sink_t *)ctx;
    if (s->n_frames < 4) {
        s->bits[s->n_frames]   = f->bits; // take ownership
        s->n_bits[s->n_frames] = f->n_bits;
        s->n_frames++;
        free(f->soft_bits);
    } else {
        free(f->bits);
        free(f->soft_bits);
    }
    if (f->demod_ok) s->n_complete++;
}

static void t_pipeline_two_bursts(void)
{
    const uint32_t len1 = 400, len2 = 736;
    int            b1 = vdl2_burst_body_bits(len1), b2 = vdl2_burst_body_bits(len2);
    uint8_t *body1 = (uint8_t *)malloc((size_t)b1);
    uint8_t *body2 = (uint8_t *)malloc((size_t)b2);
    make_body(body1, b1, 0xAA1);
    make_body(body2, b2, 0xAA2);

    int16_t *iq = (int16_t *)malloc(2 * MOD_MAX_COMPLEX * 2 * sizeof(int16_t));
    vdl2_mod_params_t p;
    vdl2_mod_params_default(&p);
    int n1 = vdl2_mod_burst(len1, body1, &p, iq, MOD_MAX_COMPLEX);
    p.seed       = 77;
    p.phase0_rad = -1.0;
    int n2 = vdl2_mod_burst(len2, body2, &p, iq + 2 * n1, MOD_MAX_COMPLEX);
    CHECK(n1 > 0 && n2 > 0, "modulator failed (%d, %d)", n1, n2);

    sink_t sink;
    memset(&sink, 0, sizeof(sink));
    const band_pipeline_t *bp = vdl2_pipeline();
    int n_ok = bp->process_burst(iq, n1 + n2, sink_cb, &sink);
    CHECK(n_ok == 2, "process_burst returned %d (want 2)", n_ok);
    CHECK(sink.n_frames == 2 && sink.n_complete == 2,
          "sink saw %d frames / %d complete", sink.n_frames, sink.n_complete);
    if (sink.n_frames == 2) {
        CHECK(sink.n_bits[0] == VDL2_HDR_BITS + b1, "frame1 %d bits",
              sink.n_bits[0]);
        CHECK(sink.n_bits[1] == VDL2_HDR_BITS + b2, "frame2 %d bits",
              sink.n_bits[1]);
        int mism = 0;
        for (int k = 0; k < b1; k++)
            mism += (sink.bits[0][VDL2_HDR_BITS + k] != body1[k]);
        for (int k = 0; k < b2; k++)
            mism += (sink.bits[1][VDL2_HDR_BITS + k] != body2[k]);
        CHECK(mism == 0, "%d body-bit mismatches across the two frames", mism);
    }
    for (int i = 0; i < sink.n_frames; i++)
        free(sink.bits[i]);
    free(body1);
    free(body2);
    free(iq);
}

int main(void)
{
    t_header_codec();
    t_body_bits();
    t_scrambler();
    t_clean_roundtrip();
    t_cfo();
    t_timing();
    t_awgn();
    t_false_sync_on_noise();
    t_truncated();
    t_pipeline_two_bursts();

    if (g_fails) {
        printf("FAIL: %d check(s) failed\n", g_fails);
        return 1;
    }
    printf("PASS: vdl2 demod round-trip suite\n");
    return 0;
}
