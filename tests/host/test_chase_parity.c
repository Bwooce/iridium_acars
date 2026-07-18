// test_chase_parity.c — parity gate: production Chase-2 C port
// (ida_chase.c) vs the Python reference prototype
// (tests/scripts/chase_crc_bch.py) on the SAME captured input.
//
// Input 1 (<dumpbits>): the per-burst demod bits + int16 soft metrics
// file written by `test_demod_diff_gri --dumpbits` on the 2026-07-17
// HydraSDR ground-truth extract (H/B line format, see that harness).
// Input 2 (optional, --expected <csv>): per-burst reference outcomes
// emitted by the Python prototype (chase_expected.py:
// idx,gold,status,checks,msgs_hex with L=5/cap-256). When given, every
// burst's status AND recovered/da_ok 200-bit message must match the
// reference exactly; any mismatch is a hard failure.
//
// Mirrors chase_crc_bch.py's evaluation loop exactly:
//   - per burst, pick the best DA-classified frame (max hard-BCH
//     blocks_ok; first wins ties) among frames that carry soft metrics;
//   - hard path: production classify (harder ON = default) + ida_decode;
//   - chase: production ida_chase_decode at L=5 / cap 256;
//   - score gold (gri CRC:OK) and safety (gri CRC-FAIL) populations
//     separately. Safety recoveries MUST be 0.
//
// Fixture-dependent => build-only (no add_test), same convention as
// test_demod_diff_gri. The self-contained ctest gate is test_ida_chase.c.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ida_chase.h"
#include "ida_decode.h"
#include "iridium_frame.h"

#define MAX_BITS 4096

typedef struct {
    int     have;
    uint8_t bits[MAX_BITS];
    int16_t soft[MAX_BITS];
    int     n_bits;
    int     has_soft;
} frame_rec_t;

typedef struct {
    int         idx;
    int         gold;
    int         n_frames;
    frame_rec_t fr[8];
} burst_rec_t;

// Outcome per burst, chase_crc_bch.py status vocabulary.
typedef enum { OUT_NONE, OUT_FAIL, OUT_DA_OK, OUT_RECOVERED } outcome_t;
static const char *out_name[] = {"none", "fail", "da_ok", "recovered"};

static outcome_t eval_burst(const burst_rec_t *b, ida_decoded_t *out_ida)
{
    // best DA frame = max hard blocks_ok, first wins ties (mirrors
    // best_da_frame() in the reference).
    int             best = -1, best_ok = -1;
    iridium_frame_t bf;
    ida_decoded_t   bd;
    for (int k = 0; k < b->n_frames; k++) {
        const frame_rec_t *fr = &b->fr[k];
        if (!fr->has_soft || fr->n_bits < 382) continue;
        iridium_frame_t f;
        if (iridium_frame_classify(fr->bits, (size_t)fr->n_bits,
                                   IR_FRM_DIR_DOWNLINK, &f) != 0)
            continue;
        if (f.type != IR_FRAME_LW || f.lw_subtype != IR_LW_DA) continue;
        ida_decoded_t d = {0};
        if (ida_decode(&f, &d) != 0) continue;
        if (d.blocks_ok > best_ok) {
            best_ok = d.blocks_ok;
            best    = k;
            bf      = f;
            bd      = d;
        }
    }
    if (best < 0) return OUT_NONE;

    if (bd.ok) {
        *out_ida = bd;
        return (bd.header_ok && bd.crc_ok) ? OUT_DA_OK : OUT_FAIL;
    }
    if (ida_chase_decode(&bf, b->fr[best].soft,
                         (size_t)b->fr[best].n_bits, &bd) == 1) {
        *out_ida = bd;
        return OUT_RECOVERED;
    }
    return OUT_FAIL;
}

// Format the 200-bit message as the reference's msgs_hex
// ("xxxxx.xxxxx...." — ten 20-bit words, lowercase %05x, dot-joined).
static void msgs_hex(const ida_decoded_t *d, char *out /* >= 60 */)
{
    char *p = out;
    for (int i = 0; i < 10; i++) {
        uint32_t m = 0;
        for (int b = 0; b < 20; b++)
            m = (m << 1) | (d->bits[i * 20 + b] & 1);
        p += sprintf(p, "%05x%s", m, i == 9 ? "" : ".");
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr,
                "usage: %s <dumpbits.txt> [--expected <csv>] [--csv]\n",
                argv[0]);
        return 2;
    }
    const char *dump_path = argv[1];
    const char *exp_path  = NULL;
    int         emit_csv  = 0;
    for (int a = 2; a < argc; a++) {
        if (strcmp(argv[a], "--expected") == 0 && a + 1 < argc)
            exp_path = argv[++a];
        else if (strcmp(argv[a], "--csv") == 0)
            emit_csv = 1;
    }

    // Production chase at the validated operating point, enabled.
    ida_chase_set_params(5, 256);
    ida_chase_set_enabled(true);

    FILE *fh = fopen(dump_path, "r");
    if (!fh) {
        fprintf(stderr, "cannot open %s\n", dump_path);
        return 2;
    }

    // Load expected outcomes (idx -> status string + msgs) if given.
    typedef struct {
        int  idx;
        char status[16];
        char msgs[64];
    } exp_t;
    exp_t *exp   = NULL;
    int    n_exp = 0;
    if (exp_path) {
        FILE *eh = fopen(exp_path, "r");
        if (!eh) {
            fprintf(stderr, "cannot open %s\n", exp_path);
            return 2;
        }
        int cap = 4096;
        exp     = malloc(cap * sizeof(*exp));
        char ln[512];
        while (fgets(ln, sizeof(ln), eh)) {
            if (n_exp == cap) {
                cap *= 2;
                exp = realloc(exp, cap * sizeof(*exp));
            }
            exp_t *e   = &exp[n_exp];
            e->msgs[0] = '\0';
            int gold, checks;
            int n = sscanf(ln, "%d,%d,%15[^,],%d,%63s",
                           &e->idx, &gold, e->status, &checks, e->msgs);
            if (n >= 3) n_exp++;
        }
        fclose(eh);
        fprintf(stderr, "loaded %d expected rows from %s\n", n_exp, exp_path);
    }

    // Stream bursts out of the dump; evaluate each on its H->B group.
    burst_rec_t b;
    memset(&b, 0, sizeof(b));
    b.idx = -1;
    int  gold_daok = 0, gold_recov = 0, gold_fail = 0;
    int  nak_daok = 0, nak_recov = 0;
    int  n_burst = 0, mismatches = 0, compared = 0;
    char line[16384];

#define FLUSH_BURST()                                                        \
    do {                                                                     \
        if (b.idx >= 0) {                                                    \
            ida_decoded_t ida = {0};                                         \
            outcome_t     oc  = eval_burst(&b, &ida);                        \
            n_burst++;                                                       \
            if (b.gold) {                                                    \
                if (oc == OUT_DA_OK) gold_daok++;                            \
                else if (oc == OUT_RECOVERED) gold_recov++;                  \
                else gold_fail++;                                            \
            } else {                                                         \
                if (oc == OUT_DA_OK) nak_daok++;                             \
                else if (oc == OUT_RECOVERED) nak_recov++;                   \
            }                                                                \
            char mh[64] = "";                                                \
            if (oc == OUT_DA_OK || oc == OUT_RECOVERED) msgs_hex(&ida, mh);  \
            if (emit_csv)                                                    \
                printf("%d,%d,%s,%s\n", b.idx, b.gold, out_name[oc], mh);    \
            if (exp) {                                                       \
                for (int e = 0; e < n_exp; e++) {                            \
                    if (exp[e].idx != b.idx) continue;                       \
                    compared++;                                              \
                    const char *ours = (oc == OUT_NONE) ? "none"             \
                                                        : out_name[oc];      \
                    if (strcmp(exp[e].status, ours) != 0 ||                  \
                        strcmp(exp[e].msgs, mh) != 0) {                      \
                        mismatches++;                                        \
                        fprintf(stderr,                                      \
                                "MISMATCH idx=%d: ref=%s/%s ours=%s/%s\n",   \
                                b.idx, exp[e].status, exp[e].msgs, ours, mh);\
                    }                                                        \
                    break;                                                   \
                }                                                            \
            }                                                                \
        }                                                                    \
        memset(&b, 0, sizeof(b));                                            \
        b.idx = -1;                                                          \
    } while (0)

    while (fgets(line, sizeof(line), fh)) {
        if (line[0] == 'H') {
            FLUSH_BURST();
            int idx, gold, nk;
            char stage[16];
            double snr;
            if (sscanf(line, "H,%d,%15[^,],%d,%lf,%d",
                       &idx, stage, &gold, &snr, &nk) == 5) {
                b.idx  = idx;
                b.gold = gold;
            }
        } else if (line[0] == 'B' && b.idx >= 0 && b.n_frames < 8) {
            // B,<idx>,<frame_no>,<n_bits>,<dir>,<bits>,<soft;...>
            frame_rec_t *fr = &b.fr[b.n_frames];
            char        *p  = line;
            int          field = 0;
            char        *bits_s = NULL, *soft_s = NULL;
            for (char *q = line; *q; q++) {
                if (*q == ',') {
                    field++;
                    if (field == 5) bits_s = q + 1;
                    if (field == 6) {
                        *q     = '\0';
                        soft_s = q + 1;
                    }
                }
            }
            (void)p;
            if (!bits_s) continue;
            int nb = 0;
            for (char *q = bits_s; *q == '0' || *q == '1'; q++) {
                if (nb < MAX_BITS) fr->bits[nb] = (uint8_t)(*q - '0');
                nb++;
            }
            if (nb > MAX_BITS) nb = MAX_BITS;
            fr->n_bits   = nb;
            fr->has_soft = 0;
            if (soft_s && *soft_s && *soft_s != '\n') {
                int   ns = 0;
                char *q  = soft_s;
                while (*q && *q != '\n' && ns < MAX_BITS) {
                    fr->soft[ns++] = (int16_t)strtol(q, &q, 10);
                    if (*q == ';') q++;
                }
                if (ns >= nb) fr->has_soft = 1;
            }
            b.n_frames++;
        }
    }
    FLUSH_BURST();
#undef FLUSH_BURST
    fclose(fh);

    fprintf(stderr, "\n== chase parity (production C, L=5 cap=256) ==\n");
    fprintf(stderr, "bursts evaluated: %d\n", n_burst);
    fprintf(stderr, "GOLD:   hard DA_OK %d  chase-recovered %d  fail %d\n",
            gold_daok, gold_recov, gold_fail);
    fprintf(stderr, "SAFETY: hard DA_OK %d  chase-false-accepts %d  <-- MUST be 0\n",
            nak_daok, nak_recov);
    ida_chase_stats_t st;
    ida_chase_get_stats(&st);
    fprintf(stderr, "chase: attempts %u recovered %u crc_checks %u "
                    "(expected collisions ~%.4f)\n",
            (unsigned)st.attempts, (unsigned)st.recovered,
            (unsigned)st.crc_checks, (double)st.crc_checks / 65536.0);
    if (exp) {
        fprintf(stderr, "parity vs reference: %d compared, %d mismatches\n",
                compared, mismatches);
        free(exp);
        if (mismatches || compared == 0) return 1;
    }
    if (nak_recov != 0) return 1;
    return 0;
}
