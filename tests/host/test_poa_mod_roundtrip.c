// test_poa_mod_roundtrip — synthetic POA golden round-trip.
//
// Builds KNOWN ACARS blocks with poa_mod (the acarsdec-inverse modulator),
// modulates them as 2.5 MSPS MSK-on-AM IQ, and feeds them through the REAL
// chain the device runs — poa_frontend (channelizer) -> poa_decoder (MSK demod
// + ACARS L2) — asserting the decoder recovers each block byte-for-byte with
// err == 0 and crc_fixed == false. No field capture needed: the modulator is
// the inverse of the demod, so this round-trip IS the correctness proof and
// gives diverse, controlled, reproducible coverage.
//
// This test NEVER skips (unlike the capture-dependent poa_decoder/poa_frontend
// tests) — the signal is generated in-process.
//
// The exact-text / len / err / crc_fixed assertions are numeric-INDEPENDENT:
// the decoded TEXT does not depend on whether the demod runs in double or
// single precision, so this fixture is equally valid against the current
// poa_decoder and the parallel single-precision rewrite. level_db (the one
// genuinely numerics-dependent field) is deliberately NOT asserted, and the
// SNR threshold is PRINTED not pinned (pinning it would fit the test to one
// decoder's numerics).

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "poa_mod.h"
#include "poa_frontend.h"

#define FS   2500000u
#define LO   130800000u
#define CHAN 131550000u
#define MAXC 5000000               // max complex samples per burst buffer

// ---- decoded-block capture ----
typedef struct {
    int           len, err, crc_fixed;
    unsigned char txt[POA_TXT_MAX];
} cap_t;

static cap_t g_caps[32];
static int   g_ncap;

static void on_block(const poa_block_t *b, void *u)
{
    (void)u;
    if (g_ncap >= (int)(sizeof(g_caps) / sizeof(g_caps[0]))) return;
    cap_t *c = &g_caps[g_ncap++];
    c->len = b->len; c->err = b->err; c->crc_fixed = (int)b->crc_fixed;
    int n = b->len < POA_TXT_MAX ? b->len : POA_TXT_MAX;
    memcpy(c->txt, b->txt, (size_t)n);
}

static void printable(const unsigned char *t, int n, char *out)
{
    for (int i = 0; i < n; i++) {
        unsigned char c = t[i];
        out[i] = (c >= 0x20 && c < 0x7f) ? (char)c : '.';
    }
    out[n] = 0;
}

// Feed one burst through a FRESH frontend and return the single decoded block
// (or NULL). Isolated per call so decoder state can't leak between messages.
static cap_t *decode_once(const int16_t *iq, int n)
{
    g_ncap = 0;
    uint32_t chans[1] = { CHAN };
    poa_frontend_t *fe = poa_frontend_create(FS, LO, chans, 1, on_block, NULL);
    assert(fe);
    int off = 0;
    while (off < n) {                          // exercise the streaming feed
        int chunk = n - off > 262144 ? 262144 : n - off;
        poa_frontend_feed(fe, &iq[2 * off], chunk);
        off += chunk;
    }
    poa_frontend_destroy(fe);
    return g_ncap == 1 ? &g_caps[0] : NULL;
}

// Assert a clean, exact recovery of `msg`.
static void expect_clean(const poa_mod_msg_t *msg, const poa_mod_params_t *p,
                         const char *name, int16_t *iq)
{
    uint8_t expect[256];
    int elen = poa_mod_build_block(msg, expect, (int)sizeof(expect));
    assert(elen > 0);
    for (int i = 0; i < elen; i++) expect[i] &= 0x7f; // decoder strips parity

    int n = poa_mod_block(msg, p, 0, iq, MAXC);
    assert(n > 0 && n % 6250 == 0);

    cap_t *c = decode_once(iq, n);
    char got[POA_TXT_MAX + 1], exp[POA_TXT_MAX + 1];
    if (!c) { printf("FAIL %-10s: no block decoded\n", name); assert(0); }
    printable(c->txt, c->len, got);
    printable(expect, elen, exp);
    printf("  %-10s len=%d(exp %d) err=%d fixed=%d: '%s'\n",
           name, c->len, elen, c->err, c->crc_fixed, got);
    if (c->len != elen || memcmp(c->txt, expect, (size_t)elen) != 0)
        printf("       expected: '%s'\n", exp);
    assert(c->len == elen);
    assert(memcmp(c->txt, expect, (size_t)elen) == 0); // exact bytes
    assert(c->err == 0);                               // no residual parity err
    assert(c->crc_fixed == 0);                         // clean CRC, not repaired
}

int main(void)
{
    int16_t *iq = malloc((size_t)MAXC * 2 * sizeof(int16_t));
    assert(iq);
    poa_mod_params_t p; poa_mod_params_default(&p);

    printf("== diverse message set (exact recovery, err=0, crc_fixed=0) ==\n");

    // 1) Downlink position report (JQ0404-style: reg + flight id in text).
    poa_mod_msg_t dl_pos = {0};
    dl_pos.mode = '2'; strcpy(dl_pos.reg, ".VH-VGD"); dl_pos.ack = 0x15;
    dl_pos.label[0] = '3'; dl_pos.label[1] = 'L'; dl_pos.block_id = '2';
    dl_pos.text = "M31ASJQ0404,33.849/E151.307/UTC2243"; dl_pos.text_len = -1;
    dl_pos.final_block = 1;
    expect_clean(&dl_pos, &p, "DL 3L", iq);

    // 2) Downlink, label Q0 (short text).
    poa_mod_msg_t dl_q0 = {0};
    dl_q0.mode = '2'; strcpy(dl_q0.reg, ".N815NN"); dl_q0.ack = 0x15;
    dl_q0.label[0] = 'Q'; dl_q0.label[1] = '0'; dl_q0.block_id = '4';
    dl_q0.text = "AAL123,OK"; dl_q0.text_len = -1; dl_q0.final_block = 1;
    expect_clean(&dl_q0, &p, "DL Q0", iq);

    // 3) Downlink, label H1 (long free text ~180 chars).
    static char longtxt[200];
    for (int i = 0; i < 180; i++) longtxt[i] = (char)('A' + (i % 26));
    longtxt[180] = 0;
    poa_mod_msg_t dl_h1 = {0};
    dl_h1.mode = '2'; strcpy(dl_h1.reg, ".VH-OQA"); dl_h1.ack = 0x15;
    dl_h1.label[0] = 'H'; dl_h1.label[1] = '1'; dl_h1.block_id = '7';
    dl_h1.text = longtxt; dl_h1.text_len = -1; dl_h1.final_block = 1;
    expect_clean(&dl_h1, &p, "DL H1 long", iq);

    // 4) Uplink, label 5Z (block id is a letter for uplink).
    poa_mod_msg_t ul_5z = {0};
    ul_5z.mode = '2'; strcpy(ul_5z.reg, ".JA801A"); ul_5z.ack = 0x15;
    ul_5z.label[0] = '5'; ul_5z.label[1] = 'Z'; ul_5z.block_id = 'X';
    ul_5z.text = "REQUEST CLIMB FL350"; ul_5z.text_len = -1; ul_5z.final_block = 1;
    expect_clean(&ul_5z, &p, "UL 5Z", iq);

    // 5) Uplink ACK, label "_d" (on-air label[1] = 0x7f). This is the empty-ack
    //    case; keep a short text so it stays a valid block. The 0x7f sits at
    //    index 10 (< 20) so the decoder's DLE(0x7f) end-of-text shortcut, which
    //    is gated on blk_len > 20, never triggers on it.
    poa_mod_msg_t ul_ack = {0};
    ul_ack.mode = '2'; strcpy(ul_ack.reg, ".C-FGGP"); ul_ack.ack = 'A';
    ul_ack.label[0] = '_'; ul_ack.label[1] = (char)0x7f; ul_ack.block_id = 'Y';
    ul_ack.text = "ACK"; ul_ack.text_len = -1; ul_ack.final_block = 1;
    expect_clean(&ul_ack, &p, "UL _d", iq);

    // 6) Multi-block message: block 1 ends ETB (not final), block 2 ends ETX.
    poa_mod_msg_t mb1 = {0}, mb2 = {0};
    mb1.mode = '2'; strcpy(mb1.reg, ".VH-VZT"); mb1.ack = 0x15;
    mb1.label[0] = 'H'; mb1.label[1] = '1'; mb1.block_id = '3';
    mb1.text = "PART1OFLONGMESSAGE-CONTINUED"; mb1.text_len = -1; mb1.final_block = 0;
    mb2 = mb1; mb2.block_id = '4';
    mb2.text = "PART2ENDOFLONGMESSAGE"; mb2.final_block = 1;
    expect_clean(&mb1, &p, "MB1 ETB", iq);
    expect_clean(&mb2, &p, "MB2 ETX", iq);

    // ---- streaming/back-to-back: two bursts through ONE frontend, decoder
    //      state carried, phase_ref accumulated (grid stays aligned). ----
    printf("== back-to-back streaming (2 bursts, one frontend) ==\n");
    {
        g_ncap = 0;
        uint32_t chans[1] = { CHAN };
        poa_frontend_t *fe = poa_frontend_create(FS, LO, chans, 1, on_block, NULL);
        assert(fe);
        int64_t ref = 0;
        const poa_mod_msg_t *seq[2] = { &mb1, &mb2 };
        for (int s = 0; s < 2; s++) {
            int n = poa_mod_block(seq[s], &p, ref, iq, MAXC);
            assert(n > 0);
            poa_frontend_feed(fe, iq, n);
            ref += n;
        }
        poa_frontend_destroy(fe);
        printf("  decoded %d/2 blocks back-to-back\n", g_ncap);
        assert(g_ncap == 2);
        for (int s = 0; s < 2; s++) {
            uint8_t e[256];
            int el = poa_mod_build_block(seq[s], e, (int)sizeof e);
            for (int i = 0; i < el; i++) e[i] &= 0x7f;
            assert(g_caps[s].len == el);
            assert(memcmp(g_caps[s].txt, e, (size_t)el) == 0);
            assert(g_caps[s].err == 0 && g_caps[s].crc_fixed == 0);
        }
    }

    // ---- SNR sweep on one message ----
    // Generate the clean burst ONCE, then per SNR point copy + add AWGN (a
    // controlled experiment: same signal, only noise varies). Wideband SNR =
    // signal power / (2*sigma^2); the 200:1 integrate-dump adds ~23 dB before
    // the demod, so clean decodes persist to strongly negative wideband SNR.
    printf("== SNR sweep (DL 3L); wideband SNR, ~23 dB integrate-dump gain ==\n");
    int n = poa_mod_block(&dl_pos, &p, 0, iq, MAXC);
    assert(n > 0);
    double sigpow = poa_mod_signal_power(iq, n);
    printf("  clean signal power = %.1f (amp=%.0f depth=%.2f)\n", sigpow, p.amp, p.mod_depth);

    uint8_t expect[256];
    int elen = poa_mod_build_block(&dl_pos, expect, (int)sizeof expect);
    for (int i = 0; i < elen; i++) expect[i] &= 0x7f;

    int16_t *noisy = malloc((size_t)n * 2 * sizeof(int16_t));
    assert(noisy);

    const double snrs[] = { 20, 15, 10, 5, 0, -3, -6, -9, -12, -15, -18, -21, -24, -30 };
    const int NS = (int)(sizeof(snrs) / sizeof(snrs[0]));
    double clean_floor = 1e9;   // lowest SNR with exact clean (err=0,fixed=0) decode
    double deliver_floor = 1e9; // lowest SNR that still delivered any block
    int total_clip = 0;
    for (int i = 0; i < NS; i++) {
        double snr = snrs[i];
        double sigma = poa_mod_sigma_for_snr(sigpow, snr);
        memcpy(noisy, iq, (size_t)n * 2 * sizeof(int16_t));
        int clip = poa_mod_add_awgn(noisy, n, sigma, 0xC0FFEEu + (uint32_t)i);
        total_clip += clip;
        cap_t *c = decode_once(noisy, n);
        int exact = c && c->len == elen && memcmp(c->txt, expect, (size_t)elen) == 0;
        int clean = exact && c->err == 0 && c->crc_fixed == 0;
        printf("  SNR %+6.1f dB  sigma=%8.1f  %s  clip=%d\n", snr, sigma,
               !c ? "no-decode" : clean ? "CLEAN(exact,err0)" :
               exact ? "exact(repaired/err)" : "block-but-wrong-text", clip);
        if (c) deliver_floor = snr;
        if (clean) clean_floor = snr;
    }
    printf("  -> clean-decode floor  ~ %+.1f dB (wideband)\n", clean_floor);
    printf("  -> any-delivery floor  ~ %+.1f dB (wideband)\n", deliver_floor);
    printf("  -> total clipped int16 across full sweep = %d\n", total_clip);
    // Note: int16 dynamic range caps how much noise is representable, so the
    // deepest SNR points (far below the clean floor) DO clip. Clipping is NOT
    // the limiter at the threshold, though: the decoder is amplitude-invariant
    // (v /= lvl+1e-8), and the default amp (peak ~1900) was chosen so the sweep
    // is clipping-free THROUGH the clean-decode floor. Verified independently:
    // dropping amp 4x leaves the clean floor unchanged while zeroing clipping
    // down past it, i.e. the reported threshold is set by SNR, not by clipping.

    // Assertions: comfortable high SNR must decode cleanly with NO clipping (so
    // clipping is provably not the limiter near threshold); the run must not
    // crash at the noise floor. The threshold itself is reported, not pinned.
    {
        double sigma = poa_mod_sigma_for_snr(sigpow, 10.0);
        memcpy(noisy, iq, (size_t)n * 2 * sizeof(int16_t));
        int clip = poa_mod_add_awgn(noisy, n, sigma, 0x1234u);
        cap_t *c = decode_once(noisy, n);
        assert(c && c->len == elen && memcmp(c->txt, expect, (size_t)elen) == 0);
        assert(c->err == 0 && c->crc_fixed == 0);
        assert(clip == 0); // +10 dB must not clip int16
    }
    assert(clean_floor < 0.0);   // processing gain must buy sub-0 dB clean decode

    free(noisy);
    free(iq);
    printf("PASS: test_poa_mod_roundtrip (synthetic POA golden through real chain)\n");
    return 0;
}
