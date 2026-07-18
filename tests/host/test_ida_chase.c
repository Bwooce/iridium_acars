// test_ida_chase.c — synthetic unit gate for the Chase-2 soft-decision
// BCH fallback (common/iridium_decoder/ida_chase.c, task #16).
//
// Uses ida_encode to build ground-truth LW.DA frames, injects
// controlled >t=2 error bursts into chosen BCH codewords with matching
// low soft-confidence markers, and asserts:
//   1. hard path fails (ida_decode ok=false) — the chase population;
//   2. toggle OFF  -> ida_chase_decode returns 0 and *out is untouched
//      (bit-identical shipped decoder);
//   3. toggle ON   -> recovery: out rewritten to the exact clean-decode
//      result (ok, header_ok, crc_ok, payload byte-identical to the
//      encoded ground truth, chase_used/chase_checks set);
//   4. clean frames are never entered (chase refuses out->ok==true);
//   5. errors OUTSIDE the reliability window are NOT recovered and *out
//      is not modified (no partial writes on failure);
//   6. multi-codeword failures recover via the cartesian CRC arbiter;
//   7. determinism: identical inputs -> identical outputs, twice.
//
// The real-capture parity gate against the Python reference lives in
// test_chase_parity.c (fixture-dependent, build-only); this file is the
// self-contained ctest gate.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ida_chase.h"
#include "ida_decode.h"
#include "ida_encode.h"
#include "iridium_frame.h"

static int g_fail = 0;
#define CHECK(cond, ...)                          \
    do {                                          \
        if (!(cond)) {                            \
            g_fail++;                             \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);         \
            fprintf(stderr, "\n");                \
        }                                         \
    } while (0)

// Map codeword-bit -> data-section bit index (0..311), derived by
// probing the production transform exactly like ida_chase does
// internally. Independent re-derivation here keeps the test honest.
static uint16_t g_map[IDA_DECODE_N_CW][IDA_DECODE_CW_BITS];
static void build_map(void)
{
    uint8_t data[IDA_DECODE_DATA_BITS];
    uint8_t cw[IDA_DECODE_N_CW_BITS];
    for (int p = 0; p < IDA_DECODE_DATA_BITS; p++) {
        memset(data, 0, sizeof(data));
        data[p] = 1;
        ida_decode_build_codewords(data, cw);
        for (int j = 0; j < IDA_DECODE_N_CW_BITS; j++)
            if (cw[j]) {
                g_map[j / IDA_DECODE_CW_BITS][j % IDA_DECODE_CW_BITS] =
                    (uint16_t)p;
                break;
            }
    }
}

// Reference reliability (mirrors ida_chase's pair-min rule).
static int conf_of_bit(const int16_t *soft, int fb)
{
    int sym = fb / 2;
    int m   = abs((int)soft[2 * sym]);
    if (sym > 0) {
        int mp = abs((int)soft[2 * (sym - 1)]);
        if (mp < m) m = mp;
    }
    return m;
}

// Build uniform soft metrics consistent with bits[] (sign carries the
// hard decision like qpsk_demod's output).
static void make_soft(const uint8_t *bits, int n, int16_t mag, int16_t *soft)
{
    for (int i = 0; i < n; i++)
        soft[i] = bits[i] ? (int16_t)-mag : mag;
}

// Set the soft MAGNITUDE of frame bit fb's parent symbol (both bits of
// the symbol), preserving signs.
static void set_sym_mag(int16_t *soft, int fb, int16_t mag)
{
    int sym = fb / 2;
    for (int b = 0; b < 2; b++) {
        int i   = 2 * sym + b;
        soft[i] = (soft[i] < 0) ? (int16_t)-mag : mag;
    }
}

// Find a 3-position error set for codeword `cwi` such that, after
// marking the three parent symbols with distinct low magnitudes in
// `soft`, all three positions fall inside the production L-window.
// Needed because confidence is SYMBOL-level: lowering an error bit's
// symbol also lowers its sibling bit and the next symbol's bits
// (pair-min rule), and those shadows often land in the SAME codeword
// and tie — so not every hand-picked triple is chase-recoverable.
// Deterministic scan; asserts success. Writes the chosen codeword-bit
// indices to err_k[3] and applies the soft markers.
static int window_covers(const int16_t *soft, int cwi, int L,
                         const int *err_k, int n_err);
static void pick_recoverable_triple(int16_t *soft, int cwi, int base_mag,
                                    int *err_k)
{
    for (int a = 0; a < IDA_DECODE_CW_BITS - 2; a++)
        for (int bb = a + 1; bb < IDA_DECODE_CW_BITS - 1; bb++)
            for (int c = bb + 1; c < IDA_DECODE_CW_BITS; c++) {
                int     ks[3] = {a, bb, c};
                int16_t saved[6];
                int     fbs[3];
                for (int e = 0; e < 3; e++) {
                    fbs[e]  = IDA_DECODE_DATA_OFF + g_map[cwi][ks[e]];
                    int sym = fbs[e] / 2;
                    saved[2 * e]     = soft[2 * sym];
                    saved[2 * e + 1] = soft[2 * sym + 1];
                }
                for (int e = 0; e < 3; e++)
                    set_sym_mag(soft, fbs[e], (int16_t)(base_mag + 10 * e));
                if (window_covers(soft, cwi, IDA_CHASE_DEFAULT_L, ks, 3)) {
                    err_k[0] = a;
                    err_k[1] = bb;
                    err_k[2] = c;
                    return;
                }
                // restore and keep searching
                for (int e = 0; e < 3; e++) {
                    int sym          = fbs[e] / 2;
                    soft[2 * sym]     = saved[2 * e];
                    soft[2 * sym + 1] = saved[2 * e + 1];
                }
            }
    fprintf(stderr, "pick_recoverable_triple: no triple found for cw %d\n",
            cwi);
    exit(1);
}

// Classify helper.
static int classify_da(const uint8_t *bits, iridium_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    if (iridium_frame_classify(bits, IDA_ENCODE_FRAME_BITS,
                               IR_FRM_DIR_DOWNLINK, f) != 0)
        return -1;
    return (f->type == IR_FRAME_LW && f->lw_subtype == IR_LW_DA) ? 0 : -1;
}

// Verify the L least-reliable positions of codeword `cwi` (using the
// same strict-'<' tie rule as production) include every index in
// err_k[]. Returns 1 if the test setup is valid.
static int window_covers(const int16_t *soft, int cwi, int L,
                         const int *err_k, int n_err)
{
    int conf[IDA_DECODE_CW_BITS];
    for (int k = 0; k < IDA_DECODE_CW_BITS; k++)
        conf[k] = conf_of_bit(soft, IDA_DECODE_DATA_OFF + g_map[cwi][k]);
    int sel[IDA_CHASE_MAX_L];
    int n_sel = 0;
    for (int k = 0; k < IDA_DECODE_CW_BITS; k++) {
        int pos = n_sel;
        while (pos > 0 && conf[k] < conf[sel[pos - 1]]) pos--;
        if (pos < L) {
            if (n_sel < L) n_sel++;
            for (int j = n_sel - 1; j > pos; j--) sel[j] = sel[j - 1];
            sel[pos] = k;
        }
    }
    for (int e = 0; e < n_err; e++) {
        int found = 0;
        for (int s = 0; s < n_sel; s++)
            if (sel[s] == err_k[e]) found = 1;
        if (!found) return 0;
    }
    return 1;
}

int main(void)
{
    build_map();
    ida_chase_set_params(IDA_CHASE_DEFAULT_L, IDA_CHASE_DEFAULT_MAX_CRC);

    // Ground-truth frame.
    const uint8_t payload[] = {0x10, 0x87, 0x02, 0x03, 0xC1, 0x21,
                               0x41, 0x56, 0x2E, 0x41, 0x42, 0x43};
    uint8_t clean[IDA_ENCODE_FRAME_BITS];
    CHECK(ida_encode_da_frame(0, 0, payload, sizeof(payload), clean) == 0,
          "ida_encode_da_frame failed");

    iridium_frame_t f;
    CHECK(classify_da(clean, &f) == 0, "clean frame must classify LW.DA");
    ida_decoded_t ref = {0};
    CHECK(ida_decode(&f, &ref) == 0 && ref.ok && ref.header_ok && ref.crc_ok,
          "clean frame must hard-decode ok");
    CHECK(!ref.chase_used && ref.chase_checks == 0,
          "hard path must not set chase provenance");

    // ---- T4: chase refuses a clean decode (out->ok true) -------------
    {
        int16_t soft[IDA_ENCODE_FRAME_BITS];
        make_soft(clean, IDA_ENCODE_FRAME_BITS, 8000, soft);
        ida_decoded_t d = ref;
        ida_chase_set_enabled(true);
        CHECK(ida_chase_decode(&f, soft, IDA_ENCODE_FRAME_BITS, &d) == 0,
              "chase must return 0 on an already-ok decode");
        CHECK(memcmp(&d, &ref, sizeof(d)) == 0,
              "chase must not touch an already-ok decode");
        ida_chase_set_enabled(false);
    }

    // ---- T1..T3: single-codeword 3-error burst, marked unreliable ----
    {
        // Corrupt 3 bits of codeword 2 (> t=2 so the hard path fails).
        const int cwi = 2;
        int       err_k[3];
        uint8_t   bits[IDA_ENCODE_FRAME_BITS];
        memcpy(bits, clean, sizeof(bits));
        int16_t soft[IDA_ENCODE_FRAME_BITS];
        make_soft(bits, IDA_ENCODE_FRAME_BITS, 8000, soft);
        // Distinct low magnitudes at the error bits' parent symbols so
        // they land inside the chase window (searched, not hand-picked —
        // symbol-level confidence shadows sibling bits; see helper).
        pick_recoverable_triple(soft, cwi, 40, err_k);
        for (int e = 0; e < 3; e++)
            bits[IDA_DECODE_DATA_OFF + g_map[cwi][err_k[e]]] ^= 1;
        CHECK(window_covers(soft, cwi, IDA_CHASE_DEFAULT_L, err_k, 3),
              "test setup: error bits must fall inside the L window");

        iridium_frame_t fc;
        CHECK(classify_da(bits, &fc) == 0, "corrupted frame must still classify");
        ida_decoded_t d = {0};
        CHECK(ida_decode(&fc, &d) == 0, "ida_decode rc");
        CHECK(!d.ok && d.blocks_ok == 9, "hard path must fail exactly cw %d", cwi);

        // T2: toggle OFF -> untouched.
        ida_decoded_t before = d;
        CHECK(ida_chase_get_enabled() == false, "default toggle must be OFF");
        CHECK(ida_chase_decode(&fc, soft, IDA_ENCODE_FRAME_BITS, &d) == 0,
              "disabled chase must return 0");
        CHECK(memcmp(&d, &before, sizeof(d)) == 0,
              "disabled chase must not modify out");

        // T3: toggle ON -> exact recovery.
        ida_chase_set_enabled(true);
        int rc = ida_chase_decode(&fc, soft, IDA_ENCODE_FRAME_BITS, &d);
        CHECK(rc == 1, "chase must recover (rc=%d)", rc);
        CHECK(d.ok && d.header_ok && d.crc_ok, "recovered flags");
        CHECK(d.blocks_ok == 10 && d.n_bits == 200, "recovered geometry");
        CHECK(d.chase_used && d.chase_checks >= 1 &&
                  d.chase_checks <= IDA_CHASE_DEFAULT_MAX_CRC,
              "chase provenance (checks=%u)", (unsigned)d.chase_checks);
        CHECK(d.payload_len == ref.payload_len &&
                  memcmp(d.payload, ref.payload, ref.payload_len) == 0,
              "recovered payload must equal ground truth");
        CHECK(memcmp(d.bits, ref.bits, sizeof(ref.bits)) == 0,
              "recovered message bits must equal ground truth");

        // T7: determinism — run again on a fresh out.
        ida_decoded_t d2 = {0};
        CHECK(ida_decode(&fc, &d2) == 0 && !d2.ok, "re-decode");
        CHECK(ida_chase_decode(&fc, soft, IDA_ENCODE_FRAME_BITS, &d2) == 1,
              "second chase run must also recover");
        CHECK(memcmp(&d2, &d, sizeof(d)) == 0, "chase must be deterministic");
        ida_chase_set_enabled(false);
    }

    // ---- T5: errors OUTSIDE the reliability window are not recovered -
    {
        const int cwi     = 5;
        const int err_k[] = {20, 25, 30};
        uint8_t   bits[IDA_ENCODE_FRAME_BITS];
        memcpy(bits, clean, sizeof(bits));
        for (int e = 0; e < 3; e++)
            bits[IDA_DECODE_DATA_OFF + g_map[cwi][err_k[e]]] ^= 1;
        int16_t soft[IDA_ENCODE_FRAME_BITS];
        make_soft(bits, IDA_ENCODE_FRAME_BITS, 8000, soft);
        // Mark five NON-error positions of this codeword as least
        // reliable, pinning the window away from the real errors.
        for (int k = 0; k < 5; k++)
            set_sym_mag(soft, IDA_DECODE_DATA_OFF + g_map[cwi][k],
                        (int16_t)(40 + 10 * k));
        CHECK(!window_covers(soft, cwi, IDA_CHASE_DEFAULT_L, err_k, 3),
              "test setup: error bits must be OUTSIDE the window");

        iridium_frame_t fc;
        CHECK(classify_da(bits, &fc) == 0, "T5 classify");
        ida_decoded_t d = {0};
        CHECK(ida_decode(&fc, &d) == 0 && !d.ok, "T5 hard fail");
        ida_decoded_t before = d;
        ida_chase_set_enabled(true);
        CHECK(ida_chase_decode(&fc, soft, IDA_ENCODE_FRAME_BITS, &d) == 0,
              "chase must NOT recover out-of-window errors");
        CHECK(memcmp(&d, &before, sizeof(d)) == 0,
              "failed chase must not modify out");
        ida_chase_set_enabled(false);
    }

    // ---- T6: two failed codewords, cartesian CRC arbitration ---------
    {
        const int cw_a = 1, cw_b = 8;
        int       ka[3], kb[3];
        uint8_t   bits[IDA_ENCODE_FRAME_BITS];
        memcpy(bits, clean, sizeof(bits));
        int16_t soft[IDA_ENCODE_FRAME_BITS];
        make_soft(bits, IDA_ENCODE_FRAME_BITS, 8000, soft);
        // Search cw_a first, then cw_b against the soft array that
        // already carries cw_a's low symbols (cross-codeword shadows).
        pick_recoverable_triple(soft, cw_a, 40, ka);
        pick_recoverable_triple(soft, cw_b, 45, kb);
        for (int e = 0; e < 3; e++) {
            bits[IDA_DECODE_DATA_OFF + g_map[cw_a][ka[e]]] ^= 1;
            bits[IDA_DECODE_DATA_OFF + g_map[cw_b][kb[e]]] ^= 1;
        }
        CHECK(window_covers(soft, cw_a, IDA_CHASE_DEFAULT_L, ka, 3),
              "T6 setup: cw_a errors in window");
        CHECK(window_covers(soft, cw_b, IDA_CHASE_DEFAULT_L, kb, 3),
              "T6 setup: cw_b errors in window");

        iridium_frame_t fc;
        CHECK(classify_da(bits, &fc) == 0, "T6 classify");
        ida_decoded_t d = {0};
        CHECK(ida_decode(&fc, &d) == 0, "T6 rc");
        CHECK(!d.ok && d.blocks_ok == 8, "T6 hard path must fail 2 cws");

        ida_chase_set_enabled(true);
        int rc = ida_chase_decode(&fc, soft, IDA_ENCODE_FRAME_BITS, &d);
        CHECK(rc == 1, "T6 chase must recover both codewords (rc=%d)", rc);
        CHECK(d.ok && d.crc_ok && d.chase_used, "T6 recovered flags");
        CHECK(memcmp(d.bits, ref.bits, sizeof(ref.bits)) == 0,
              "T6 recovered message bits must equal ground truth");
        ida_chase_set_enabled(false);
    }

    // ---- T8: stats counters advanced ---------------------------------
    {
        ida_chase_stats_t st;
        ida_chase_get_stats(&st);
        CHECK(st.attempts >= 3, "attempts counted (%u)", (unsigned)st.attempts);
        CHECK(st.recovered >= 2, "recoveries counted (%u)", (unsigned)st.recovered);
        CHECK(st.crc_checks >= st.recovered, "crc checks counted (%u)",
              (unsigned)st.crc_checks);
    }

    // ---- T9: no-soft / short-soft guards ------------------------------
    {
        uint8_t bits[IDA_ENCODE_FRAME_BITS];
        memcpy(bits, clean, sizeof(bits));
        bits[IDA_DECODE_DATA_OFF + g_map[0][3]] ^= 1;
        bits[IDA_DECODE_DATA_OFF + g_map[0][7]] ^= 1;
        bits[IDA_DECODE_DATA_OFF + g_map[0][12]] ^= 1;
        iridium_frame_t fc;
        CHECK(classify_da(bits, &fc) == 0, "T9 classify");
        ida_decoded_t d = {0};
        CHECK(ida_decode(&fc, &d) == 0 && !d.ok, "T9 hard fail");
        ida_decoded_t before = d;
        ida_chase_set_enabled(true);
        int16_t soft_short[100] = {0};
        CHECK(ida_chase_decode(&fc, NULL, 0, &d) == 0, "NULL soft -> 0");
        CHECK(ida_chase_decode(&fc, soft_short, 100, &d) == 0,
              "short soft -> 0");
        CHECK(memcmp(&d, &before, sizeof(d)) == 0, "guards must not modify out");
        ida_chase_set_enabled(false);
    }

    if (g_fail) {
        fprintf(stderr, "test_ida_chase: %d FAILURES\n", g_fail);
        return 1;
    }
    printf("test_ida_chase: all checks passed\n");
    return 0;
}
