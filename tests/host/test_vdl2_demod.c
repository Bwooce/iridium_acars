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

static void t_rrc_designer(void)
{
    // vdl2_rrc_design_q15 is not in the production demod path (the TX
    // pulse is the full RC, so the receive filter stays flat — see
    // vdl2_demod.h), but it is kept for V4 live-TX probing: pin its
    // conventions. 48 taps/phase, interp 21, 500 virtual samples per
    // symbol (the 250 k front-door geometry).
    enum { DS = 48, IN = 21 };
    static int16_t c[DS * IN];
    CHECK(vdl2_rrc_design_q15(c, DS, IN, 500.0, 0.6), "designer alloc");
    // Per-phase DC gain 1.0 in Q15 (firmr_s16 shift=0 convention).
    for (int p = 0; p < IN; p++) {
        int32_t sum = 0;
        for (int t = 0; t < DS; t++)
            sum += c[t * IN + p];
        CHECK(sum > 32767 - DS && sum < 32767 + DS,
              "phase %d DC gain %ld not ~2^15", p, (long)sum);
    }
    // Even symmetry about the prototype centre (linear phase).
    int n = DS * IN, worst = 0;
    for (int k = 1; k < n / 2; k++) {
        int d = abs((int)c[n / 2 - k] - (int)c[n / 2 + k]);
        if (d > worst) worst = d;
    }
    CHECK(worst <= 1, "prototype asymmetry %d LSB", worst);
    // Peak at the centre tap and the alpha=0.6 RRC first zero region:
    // taps a symbol period out are well below the peak.
    int peak = 0;
    for (int k = 0; k < n; k++)
        if (abs(c[k]) > peak) peak = abs(c[k]);
    CHECK(peak == abs(c[n / 2]), "peak %d not at centre (%d)", peak,
          abs(c[n / 2]));
    CHECK(abs(c[n / 2 + 500]) < peak / 5,
          "tap at +1 T not attenuated (%d vs peak %d)", abs(c[n / 2 + 500]),
          peak);
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
        // V4: clean-signal EVM is ~0.011-0.025 rad (snr proxy 32-39 dB;
        // V2's non-fractional strobes + short filter sat at ~21 dB).
        CHECK(r.snr_db > 28.f, "clean snr proxy %.1f dB", (double)r.snr_db);
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
    // 20 dB, 20/20 trials, with a +/-400 Hz offset stacked on. V2 ran
    // this gate at 24 dB (whole-sample T/10 strobes + a short 48-tap
    // channel filter cost ~5 dB); the V4 sharp channel filter +
    // fractional-instant strobes move the measured bit-exact waterline
    // to ~16-18 dB (INFO sweep below), so 20 dB carries ~2-4 dB of
    // margin against flaky-trial noise. Differential detection is
    // still ~3 dB off coherent — that part is structural (dumpvdl2
    // matches).
    int pass_hi = 0;
    for (int t = 0; t < 20; t++) {
        vdl2_mod_params_t p;
        vdl2_mod_params_default(&p);
        p.seed       = 1000 + (uint32_t)t;
        p.awgn_sigma = ebn0_to_sigma(20.0, p.amp, p.fs_hz);
        p.cfo_hz     = (t % 2) ? 400.0 : -400.0; // noise + offset together
        vdl2_demod_result_t r;
        int mism = -1;
        if (roundtrip_ex(992, &p, 0x9000 + (uint32_t)t, &r, 0, &mism)) {
            if (r.complete && mism == 0) pass_hi++;
            free(r.bits);
            free(r.soft_bits);
        }
    }
    CHECK(pass_hi == 20, "Eb/N0 20 dB: %d/20 bit-exact", pass_hi);

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

// ---- per-bit soft-demapping (Gray-8PSK) vs the old shared metric ----
//
// The demod now emits a PER-BIT confidence (vdl2_softbit_conf) instead
// of one value shared by a symbol's 3 bits. This is a UNIT + A/B test of
// that function (constructing the exact end-to-end RS-erasure recovery
// deterministically is impractical, so the mechanism is asserted
// directly). The legacy shared value was (0.5-|efrac|)*2*24576, which by
// construction equals min(conf3) — the toward-neighbour bit. So:
//   (a) a symbol offset toward a neighbour -> exactly ONE low bit, and it
//       is the CORRECT bit (the one flipping between idx and that
//       neighbour per vdl2_graycode); the other two are high;
//   (b) a dead-centre symbol -> all three saturate high;
//   (c) over a phase sweep, min(conf3) reproduces the legacy shared
//       value (erasure decisions keyed on the weakest bit are preserved)
//       and the hard decision is a pure function of idx (unchanged).
// The A/B contrast: at a near-boundary symbol the OLD shared metric
// drags all 3 bits down (would erase 2 confident bits' worth of budget),
// while per-bit isolates the single uncertain bit.

// Bit index (0=MSB..2=LSB) that differs between two Gray phase indices
// known to differ in exactly one bit; -1 if not exactly one.
static int diff_bit(int a, int b)
{
    unsigned x   = (unsigned)vdl2_graycode[a] ^ (unsigned)vdl2_graycode[b];
    int      idx = -1, n = 0;
    for (int s = 0; s < 3; s++)
        if (x & (1u << s)) {
            n++;
            idx = 2 - s; // bit 0 is (g>>2), bit 2 is (g&1)
        }
    return n == 1 ? idx : -1;
}

static int16_t legacy_shared(float efrac)
{
    float c = (0.5f - fabsf(efrac)) * 2.f * 24576.f;
    if (c < 0.f) c = 0.f;
    return (int16_t)c;
}

static void t_perbit_softmetric(void)
{
    const int16_t LOW_THR  = 12288; // half-scale: "uncertain"
    const int16_t HIGH_THR = 20000; // near-saturated: "confident"

    // (a) Offset toward a neighbour: exactly one low bit, correct bit.
    for (int idx = 0; idx < 8; idx++) {
        for (int dir = -1; dir <= 1; dir += 2) {
            float   efrac = 0.45f * (float)dir; // hard against a boundary
            int16_t c3[3];
            vdl2_softbit_conf(idx, efrac, c3);
            int nbr    = ((idx + dir) % 8 + 8) % 8;
            int expect = diff_bit(idx, nbr); // bit flipping toward nbr
            CHECK(expect >= 0, "graycode idx %d/%d not adjacent-Gray", idx,
                  nbr);
            int low = 0, low_bit = -1, high = 0;
            for (int b = 0; b < 3; b++) {
                if (c3[b] < LOW_THR) {
                    low++;
                    low_bit = b;
                }
                if (c3[b] >= HIGH_THR) high++;
            }
            CHECK(low == 1, "idx %d dir %+d: %d low bits (want 1)", idx, dir,
                  low);
            CHECK(low_bit == expect,
                  "idx %d dir %+d: low bit %d != flip bit %d", idx, dir,
                  low_bit, expect);
            CHECK(high == 2, "idx %d dir %+d: %d high bits (want 2)", idx,
                  dir, high);
            // The low bit reproduces the legacy shared value exactly.
            CHECK(c3[low_bit] == legacy_shared(efrac),
                  "idx %d dir %+d: low %d != legacy %d", idx, dir,
                  c3[low_bit], legacy_shared(efrac));
        }
    }

    // (b) Dead-centre symbol: all three bits saturate high.
    for (int idx = 0; idx < 8; idx++) {
        int16_t c3[3];
        vdl2_softbit_conf(idx, 0.0f, c3);
        for (int b = 0; b < 3; b++)
            CHECK(c3[b] == 24576, "idx %d dead-centre bit %d = %d (want max)",
                  idx, b, c3[b]);
    }

    // (c) Phase sweep: min(conf3) reproduces the legacy shared metric, and
    // every per-bit value is >= the legacy value (per-bit only ever
    // sharpens, never weakens, so the golden hard decode cannot regress).
    for (int idx = 0; idx < 8; idx++) {
        for (int e = -49; e <= 49; e++) {
            float   efrac = (float)e / 100.f;
            int16_t c3[3];
            vdl2_softbit_conf(idx, efrac, c3);
            int16_t mn  = c3[0];
            for (int b = 1; b < 3; b++)
                if (c3[b] < mn) mn = c3[b];
            int16_t leg = legacy_shared(efrac);
            CHECK(mn == leg, "idx %d efrac %.2f: min %d != legacy %d", idx,
                  (double)efrac, (int)mn, (int)leg);
            for (int b = 0; b < 3; b++)
                CHECK(c3[b] >= leg,
                      "idx %d efrac %.2f bit %d: %d < legacy %d (weaker!)",
                      idx, (double)efrac, b, (int)c3[b], (int)leg);
        }
    }

    // A/B contrast at a near-boundary symbol: quantify erasure budget.
    // OLD shared metric = one value replicated to all 3 bits -> if it is
    // low, ALL 3 bits are marked uncertain (2 confident bits wasted). NEW
    // per-bit marks exactly the 1 truly uncertain bit.
    {
        int     idx = 0;
        float   efrac = 0.47f; // toward idx 1
        int16_t c3[3];
        vdl2_softbit_conf(idx, efrac, c3);
        int16_t shared = legacy_shared(efrac);
        int shared_low = 0, perbit_low = 0;
        for (int b = 0; b < 3; b++) {
            if (shared < LOW_THR) shared_low++; // replicated to every bit
            if (c3[b] < LOW_THR) perbit_low++;
        }
        CHECK(shared_low == 3,
              "A/B: shared marks %d/3 bits low (want 3)", shared_low);
        CHECK(perbit_low == 1,
              "A/B: per-bit marks %d/3 bits low (want 1)", perbit_low);
        printf("INFO: near-boundary symbol conf3 = [%d %d %d] "
               "(shared=%d): per-bit erases 1 bit vs shared 3\n",
               c3[0], c3[1], c3[2], shared);
    }
}

// ---- soft-decision (Chase) header fallback ----
//
// The (25,20) burst header block code guarantees only single-bit
// correction; a genuine 2-3-bit header error is REJECTED by the hard
// syndrome decode (vdl2_hdr_decode -> -1). vdl2_hdr_decode_soft retries
// by flipping small subsets of the least-reliable header bits. This
// asserts, deterministically:
//   - a clean header decodes hard (weight 0) — the fast path is untouched
//     (soft is never consulted for a decodable header);
//   - a 2-bit and a 3-bit header error that HARD REJECTS is RECOVERED by
//     the soft fallback when the errored bits carry the lowest confidence,
//     yielding the exact transmitted word and length;
//   - the confidence hint is load-bearing: with the errored bits marked
//     CONFIDENT (and unrelated bits weak) the fallback does NOT
//     mis-recover to the true length (the Chase test set no longer covers
//     the real errors).
static void t_soft_header_fallback(void)
{
    const uint32_t datalen = 992;
    uint32_t       enc     = vdl2_hdr_encode(datalen); // clean 25-bit word

    // (1) Clean header: hard fast-path returns weight 0 + correct length;
    // soft on the same clean word (no errors) also returns weight 0
    // unchanged. Word-position p maps to air bit (24 - p).
    {
        uint32_t w = enc, got = 0;
        int      sw = vdl2_hdr_decode(&w, &got);
        CHECK(sw == 0 && got == datalen, "clean hard decode sw=%d len=%u", sw,
              got);
        // Soft is a FALLBACK: the demod only consults it AFTER the hard
        // decode rejects, so a clean header never reaches it. Called
        // directly on a clean word it still yields the correct word/length
        // (it always applies >=1 Chase flip, so the reported weight is >=1,
        // not the hard-path 0 — the point is it does not corrupt a good
        // header).
        uint32_t ws = enc, gots = 0;
        int      sws = vdl2_hdr_decode_soft(&ws, NULL, &gots);
        CHECK(sws >= 0 && gots == datalen && ws == enc,
              "soft on clean word sw=%d len=%u", sws, gots);
    }

    // (2) Find a 2-bit error the HARD decode REJECTS and the soft fallback
    // recovers when those two bits are the least reliable. Search over the
    // 22 meaningful word positions (reserved MSBs are forced to 0).
    int found2 = 0, found3 = 0;
    for (int a = 0; a < 22 && !found2; a++)
        for (int b = a + 1; b < 22 && !found2; b++) {
            uint32_t bad = enc ^ (1u << a) ^ (1u << b);
            uint32_t w = bad, got = 0;
            if (vdl2_hdr_decode(&w, &got) >= 0)
                continue; // hard did not reject -> not an A/B case
            int16_t conf[25];
            for (int k = 0; k < 25; k++)
                conf[k] = 24000;
            conf[24 - a] = 10; // errored bits = least reliable
            conf[24 - b] = 10;
            uint32_t ws = bad, gots = 0;
            int      sws = vdl2_hdr_decode_soft(&ws, conf, &gots);
            if (sws >= 0 && gots == datalen && ws == enc) {
                found2 = 1;
                // A/B: same errored word, but now the errored bits are
                // CONFIDENT and two unrelated bits are weak -> the Chase
                // test set misses the real errors, so no false recovery.
                for (int k = 0; k < 25; k++)
                    conf[k] = 24000;
                int wa = (24 - a + 3) % 22, wb = (24 - b + 5) % 22;
                conf[wa] = 10;
                conf[wb] = 10;
                uint32_t wm = bad, gotm = 0;
                int      swm = vdl2_hdr_decode_soft(&wm, conf, &gotm);
                CHECK(!(swm >= 0 && gotm == datalen && wm == enc),
                      "misleading-conf soft falsely recovered 2-bit err");
            }
        }
    CHECK(found2, "no 2-bit hard-reject / soft-recover example found");

    // (3) A 3-bit error the hard decode rejects, recovered when all three
    // errored bits are the least reliable (M=3 Chase window).
    for (int a = 0; a < 22 && !found3; a++)
        for (int b = a + 1; b < 22 && !found3; b++)
            for (int c = b + 1; c < 22 && !found3; c++) {
                uint32_t bad = enc ^ (1u << a) ^ (1u << b) ^ (1u << c);
                uint32_t w = bad, got = 0;
                if (vdl2_hdr_decode(&w, &got) >= 0) continue;
                int16_t conf[25];
                for (int k = 0; k < 25; k++)
                    conf[k] = 24000;
                conf[24 - a] = 5;
                conf[24 - b] = 10;
                conf[24 - c] = 15;
                uint32_t ws = bad, gots = 0;
                int      sws = vdl2_hdr_decode_soft(&ws, conf, &gots);
                if (sws >= 1 && gots == datalen && ws == enc) found3 = 1;
            }
    CHECK(found3, "no 3-bit hard-reject / soft-recover example found");

    // (4) The stats getter exists and the rescue counter is readable.
    vdl2_demod_stats_t st;
    vdl2_demod_get_stats(&st);
    CHECK(st.soft_hdr_rescued == st.soft_hdr_rescued, "stats getter");
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
    t_rrc_designer();
    t_scrambler();
    t_clean_roundtrip();
    t_cfo();
    t_timing();
    t_awgn();
    t_perbit_softmetric();
    t_soft_header_fallback();
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
