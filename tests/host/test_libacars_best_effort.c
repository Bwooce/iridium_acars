// Host validation for the libacars best-effort/partial decode work
// (docs/superpowers/plans/2026-07-07-libacars-best-effort-decode.md,
// Phase H = design §§1-8, host-side only).
//
// This file grows across the phase's commits:
//   - positive control (design §6a): the 7 intact fixtures cited in
//     libacars/examples/{cpdlc,adsc}_get_position.c decode cleanly, and
//     render byte-identically with best_effort_decode ON vs OFF (an
//     intact fixture never takes the partial path either way).
//   - later commits add: the bit-flip fuzz (§6b) and the ZK-NNC
//     acceptance demo (§6c).
//
// Fixtures are the full ARINC-622 text messages ("/GSADDR.IMI.<air_reg><hex
// payload+CRC>"), decoded via la_arinc_parse() exactly like the upstream
// example programs do -- this exercises the same code path production
// uses (la_arinc_parse -> la_cpdlc_parse / la_adsc_parse).

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <libacars/libacars.h>
#include <libacars/arinc.h>
#include <libacars/cpdlc.h>
#include <libacars/adsc.h>
#include <libacars/util.h>

static int passed = 0, failed = 0;

#define CHECK(cond, fmt, ...)                                             \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("  FAIL line %d: " fmt "\n", __LINE__, ##__VA_ARGS__); \
            failed++;                                                     \
        } else {                                                          \
            passed++;                                                     \
        }                                                                 \
    } while (0)

typedef enum { FIXTURE_CPDLC,
               FIXTURE_ADSC } fixture_kind_t;

typedef struct {
    char const    *name;
    char const    *arinc_text; // "/GSADDR.IMI.<air_reg><hex...>"
    fixture_kind_t kind;
} fixture_t;

// design §6a citations:
//   CPDLC (3): libacars/examples/cpdlc_get_position.c:33,37-39
//   ADS-C (4): libacars/examples/adsc_get_position.c:26,30-32
static const fixture_t FIXTURES[] = {
    {"SOUCAYA", "/SOUCAYA.AT1."
                "HL8251243F880C3D903BB412903604FE326C2479F4A64F7F62528B1A9CF8382738186AC28B16668E013DF464D8A7F0",
     FIXTURE_CPDLC},
    {"MSTEC7X", "/MSTEC7X.AT1."
                "VT-ANKA094D88C3D903BB465D0723053B2E5123CFA53279400014B0894A2C6A73CBD8F52447AF1244CB4C9B94600089D65C84314892694587510528B1A9CF41169D440C1AB36A08B42",
     FIXTURE_CPDLC},
    {"MELCAYA", "/MELCAYA.AT1."
                "ZK-OKC253C21CC3D903BA178F96618F0B28024B83127CD7886A12E9D85266B927584A9169C1A8EEB2800EEA7",
     FIXTURE_CPDLC},
    {"BOMASAI", "/BOMASAI.ADS."
                "VT-ANB072501A070A988CA73248F0E5DC10200000F5EE1ABC000102B885E0A19F5",
     FIXTURE_ADSC},
    {"AUHASMO", "/AUHASMO.ADS."
                "A6-PFE0724D9586A36C92B2DCF1F0E74A8E4807C0F7219AF407C10422E9E08A1C4",
     FIXTURE_ADSC},
    {"CTUE1YA", "/CTUE1YA.ADS."
                "HB-JNB1424AB686D9308CA2EBA1D0D24A2C06C1B48CA004A248050667908CA004BF6",
     FIXTURE_ADSC},
    {"YQXE2YA", "/YQXE2YA.ADS."
                "SP-LRH1424FD087806C0B527769F0D2500B877ED00B5401E2516707755C01340B768",
     FIXTURE_ADSC},
};
#define NUM_FIXTURES (int)(sizeof(FIXTURES) / sizeof(FIXTURES[0]))

// design §6a: intact fixtures must decode with err == false and produce a
// non-empty rendering, both with best_effort_decode OFF (today's baseline).
static void test_positive_control_mode_off(void)
{
    printf("Test: positive control, 7 intact fixtures decode cleanly (mode OFF)\n");

    for (int i = 0; i < NUM_FIXTURES; i++) {
        fixture_t const *fx   = &FIXTURES[i];
        la_proto_node   *node = la_arinc_parse(fx->arinc_text, LA_MSG_DIR_AIR2GND);
        CHECK(node != NULL, "%s: la_arinc_parse returned NULL", fx->name);
        if (node == NULL) {
            continue;
        }

        bool err = true;
        if (fx->kind == FIXTURE_CPDLC) {
            la_proto_node *cpdlc_node = la_proto_tree_find_cpdlc(node);
            CHECK(cpdlc_node != NULL, "%s: no CPDLC node in tree", fx->name);
            if (cpdlc_node != NULL) {
                err = ((la_cpdlc_msg const *)cpdlc_node->data)->err;
            }
        } else {
            la_proto_node *adsc_node = la_proto_tree_find_adsc(node);
            CHECK(adsc_node != NULL, "%s: no ADS-C node in tree", fx->name);
            if (adsc_node != NULL) {
                err = ((la_adsc_msg_t const *)adsc_node->data)->err;
            }
        }
        CHECK(err == false, "%s: err flag set on an intact fixture", fx->name);

        la_vstring *vstr = la_proto_tree_format_text(NULL, node);
        CHECK(vstr != NULL && vstr->len > 0, "%s: empty text rendering", fx->name);
        if (vstr != NULL) {
            la_vstring_destroy(vstr, true);
        }

        la_proto_tree_destroy(node);
    }
}

// Renders a fixture's full proto tree as text+json and returns both as
// newly-allocated strings (caller frees). Destroys the tree before
// returning.
static void render_fixture(fixture_t const *fx, char **text_out, char **json_out)
{
    la_proto_node *node = la_arinc_parse(fx->arinc_text, LA_MSG_DIR_AIR2GND);
    if (node == NULL) {
        *text_out = NULL;
        *json_out = NULL;
        return;
    }
    la_vstring *tv = la_proto_tree_format_text(NULL, node);
    la_vstring *jv = la_proto_tree_format_json(NULL, node);
    *text_out      = tv != NULL ? strdup(tv->str) : NULL;
    *json_out      = jv != NULL ? strdup(jv->str) : NULL;
    if (tv != NULL) la_vstring_destroy(tv, true);
    if (jv != NULL) la_vstring_destroy(jv, true);
    la_proto_tree_destroy(node);
}

// design §6a: intact fixtures must render BYTE-IDENTICAL whether
// best_effort_decode is OFF (today's default) or ON -- an intact
// fixture never takes the partial path either way, so flipping the flag
// must not change a single byte of output.
static void test_positive_control_mode_on_matches_off(void)
{
    printf("Test: positive control, mode ON renders byte-identical to mode OFF\n");

    for (int i = 0; i < NUM_FIXTURES; i++) {
        fixture_t const *fx = &FIXTURES[i];

        la_config_set_bool("best_effort_decode", false);
        char *text_off = NULL, *json_off = NULL;
        render_fixture(fx, &text_off, &json_off);

        la_config_set_bool("best_effort_decode", true);
        char *text_on = NULL, *json_on = NULL;
        render_fixture(fx, &text_on, &json_on);

        la_config_set_bool("best_effort_decode", false); // restore default

        CHECK(text_off != NULL && text_on != NULL, "%s: NULL text rendering", fx->name);
        CHECK(json_off != NULL && json_on != NULL, "%s: NULL json rendering", fx->name);
        if (text_off != NULL && text_on != NULL) {
            CHECK(strcmp(text_off, text_on) == 0, "%s: text rendering differs ON vs OFF", fx->name);
        }
        if (json_off != NULL && json_on != NULL) {
            CHECK(strcmp(json_off, json_on) == 0, "%s: json rendering differs ON vs OFF", fx->name);
        }

        free(text_off);
        free(json_off);
        free(text_on);
        free(json_on);
    }
}

// design §8: a truncated ADS-C payload (real-world analog: broken SBD
// reassembly cutting a message's tail) must -- with best_effort_decode
// ON -- keep the tags parsed before the break, tag msg->partial, and
// record failed_tag/err_offset; with the flag OFF, behaviour must match
// today exactly (err set, no partial labelling). Also exercises the
// la_adsc_tag_parse leak fix (t->type set before parsing): under a
// plain (non-ASan) build this only proves the fields come out right:
// the actual leak-freed-or-not is what the ASan fuzz build (later
// commit) verifies.
static void test_adsc_truncated_payload(void)
{
    printf("Test: truncated ADS-C payload -> partial labelling when flag ON\n");

    // BOMASAI fixture (examples/adsc_get_position.c:26) with the last 20
    // hex characters (10 bytes) chopped off -- long enough to still
    // contain at least one complete tag before the cut, short enough to
    // guarantee a truncated final tag.
    fixture_t const bomasai = FIXTURES[3];
    char const      full[]  = "/BOMASAI.ADS.VT-ANB072501A070A988CA73248F0E5DC10200000F5EE1ABC000102B885E0A19F5";
    CHECK(strcmp(full, bomasai.arinc_text) == 0, "BOMASAI fixture text drifted -- update the copy above");
    size_t full_len = strlen(full);
    char   truncated[128];
    CHECK(full_len - 20 < sizeof(truncated), "truncated buffer too small");
    memcpy(truncated, full, full_len - 20);
    truncated[full_len - 20] = '\0';

    for (int mode = 0; mode < 2; mode++) {
        bool const best_effort = (mode == 1);
        la_config_set_bool("best_effort_decode", best_effort);

        la_proto_node *node = la_arinc_parse(truncated, LA_MSG_DIR_AIR2GND);
        CHECK(node != NULL, "truncated BOMASAI: la_arinc_parse returned NULL");
        if (node == NULL) {
            continue;
        }
        la_proto_node *adsc_node = la_proto_tree_find_adsc(node);
        CHECK(adsc_node != NULL, "truncated BOMASAI: no ADS-C node in tree");
        if (adsc_node != NULL) {
            la_adsc_msg_t const *msg = adsc_node->data;
            CHECK(msg->err == true, "truncated BOMASAI: err not set (mode=%d)", best_effort);
            CHECK(msg->partial == best_effort,
                  "truncated BOMASAI: partial=%d, expected %d (mode=%d)", msg->partial, best_effort, best_effort);
            if (best_effort) {
                CHECK(msg->err_offset > 0, "truncated BOMASAI: err_offset == 0 with flag ON");
            } else {
                CHECK(msg->err_offset == 0, "truncated BOMASAI: err_offset set with flag OFF");
                CHECK(msg->failed_tag == 0, "truncated BOMASAI: failed_tag set with flag OFF");
            }

            la_vstring *tv = la_proto_tree_format_text(NULL, node);
            if (best_effort) {
                CHECK(tv != NULL && strstr(tv->str, "PARTIAL/UNTRUSTED") != NULL,
                      "truncated BOMASAI: no PARTIAL banner in text with flag ON");
            } else {
                CHECK(tv != NULL && strstr(tv->str, "Malformed ADS-C message") != NULL,
                      "truncated BOMASAI: missing today's-behaviour banner with flag OFF");
            }
            if (tv != NULL) la_vstring_destroy(tv, true);

            la_vstring *jv = la_proto_tree_format_json(NULL, node);
            CHECK(jv != NULL && strstr(jv->str, "\"partial\":") != NULL, "truncated BOMASAI: no \"partial\" key in json");
            if (jv != NULL) la_vstring_destroy(jv, true);
        }
        la_proto_tree_destroy(node);
    }
    la_config_set_bool("best_effort_decode", false); // restore default
}

// Returns the exact bytes production code feeds to la_cpdlc_parse() /
// la_adsc_parse(): hex-decodes the payload after the fixed
// "/GSADDR.IMI.<air_reg>" prefix (1+7+1+3+1+7 = 20 chars, true for all 7
// fixtures -- 7-char ground address, 3-char IMI, 7-char air_reg) and
// strips the trailing 2-byte CRC, mirroring la_arinc_parse()
// (arinc.c: air_reg(7), la_slurp_hexstring(), then
// `buflen -= LA_ARINC_CRC_LEN`). Caller frees *buf via free() (matches
// LA_XCALLOC/free pairing used throughout libacars).
static size_t get_fixture_payload(fixture_t const *fx, uint8_t **buf)
{
    char const *hexpart_lit = fx->arinc_text + 20;
    char       *hexpart     = strdup(hexpart_lit);
    size_t      n           = la_slurp_hexstring(hexpart, buf);
    free(hexpart);
    if (n < 2) {
        return 0;
    }
    return n - 2; // strip CRC
}

// design §6b: flip every bit of every one of the 7 fixtures' actual
// decode buffers (~2,048 iterations total -- close to the design's
// ~2,250 estimate) with best_effort_decode ON, under ASan+UBSan
// (HOST_TESTS_ASAN). Assertions:
//   - no sanitizer hits (enforced externally by the ASan build itself --
//     a crash here means the whole ctest run fails)
//   - every RC_FAIL (err == true) yields either PARTIAL output or a
//     clean "Unparseable"/"Malformed" rendering -- never a NULL/garbage
//     render
//   - every RC_OK-full flip (err == false) renders IDENTICALLY whether
//     best_effort_decode is ON or OFF -- the flag must never change
//     behaviour on a message that fully decoded
//   - ADS-C only: value-byte flips in fixed-length groups must not
//     desync framing. Operationalized via a ground-truth table built by
//     truncating the ORIGINAL (unflipped) buffer at every length and
//     recording where that truncation's decode gives up (msg->err_offset)
//     -- that marks the start of "the tag containing this byte
//     position". A real flip's failure (if any) must not land on an
//     EARLIER tag than that ground truth, ie. corruption must not
//     cascade backward through already-parsed tags.
static void test_bitflip_fuzz(void)
{
    printf("Test: bit-flip fuzz, every bit of all 7 fixtures (best_effort_decode ON)\n");
    la_config_set_bool("best_effort_decode", true);

    long total_iters = 0, rc_ok_full = 0, rc_fail = 0, partials = 0;

    for (int i = 0; i < NUM_FIXTURES; i++) {
        fixture_t const *fx   = &FIXTURES[i];
        uint8_t         *orig = NULL;
        size_t           len  = get_fixture_payload(fx, &orig);
        CHECK(orig != NULL && len > 0, "%s: failed to extract raw payload", fx->name);
        if (orig == NULL || len == 0) {
            free(orig);
            continue;
        }

        // ADS-C ground truth: tag_start_at[b] = byte offset of the tag
        // that a decode truncated to (b+1) bytes fails inside of (or, in
        // the rare case that truncation lands exactly on a trailing
        // boundary, the truncation length itself).
        size_t *tag_start_at = NULL;
        if (fx->kind == FIXTURE_ADSC) {
            tag_start_at = calloc(len, sizeof(size_t));
            for (size_t trunc = 1; trunc <= len; trunc++) {
                la_proto_node *n = la_adsc_parse(orig, (int)trunc, LA_MSG_DIR_AIR2GND, ARINC_MSG_ADS);
                if (n != NULL) {
                    la_adsc_msg_t const *m  = n->data;
                    tag_start_at[trunc - 1] = (m->err && m->partial) ? m->err_offset : trunc;
                    la_proto_tree_destroy(n);
                }
            }
        }

        for (size_t bit = 0; bit < len * 8; bit++) {
            total_iters++;
            uint8_t *mut = malloc(len);
            memcpy(mut, orig, len);
            mut[bit / 8] ^= (uint8_t)(1u << (7 - (bit % 8)));

            la_proto_node *node = (fx->kind == FIXTURE_CPDLC)
                                      ? la_cpdlc_parse(mut, (int)len, LA_MSG_DIR_AIR2GND)
                                      : la_adsc_parse(mut, (int)len, LA_MSG_DIR_AIR2GND, ARINC_MSG_ADS);

            bool err = true, partial = false;
            if (node != NULL) {
                if (fx->kind == FIXTURE_CPDLC) {
                    la_cpdlc_msg const *m = node->data;
                    err                   = m->err;
                    partial               = m->partial;
                } else {
                    la_adsc_msg_t const *m = node->data;
                    err                    = m->err;
                    partial                = m->partial;
                }
            }

            // A CPDLC partial success has err == false too (design §4:
            // "ON + consumed>0 -> ... err=false"), so err alone can't
            // tell "genuinely fully decoded" apart from "best-effort
            // salvage" -- must also check partial. (ADS-C keeps err ==
            // true on any tag failure regardless of the flag, design §8,
            // so this reduces to the plain !err check there.)
            bool full_success = !err && !partial;

            if (full_success) {
                rc_ok_full++;
                la_vstring *v_on    = la_proto_tree_format_text(NULL, node);
                char       *text_on = v_on != NULL ? strdup(v_on->str) : NULL;
                if (v_on != NULL) la_vstring_destroy(v_on, true);
                la_proto_tree_destroy(node);

                la_config_set_bool("best_effort_decode", false);
                la_proto_node *node_off = (fx->kind == FIXTURE_CPDLC)
                                              ? la_cpdlc_parse(mut, (int)len, LA_MSG_DIR_AIR2GND)
                                              : la_adsc_parse(mut, (int)len, LA_MSG_DIR_AIR2GND, ARINC_MSG_ADS);
                la_config_set_bool("best_effort_decode", true);
                la_vstring *v_off    = node_off != NULL ? la_proto_tree_format_text(NULL, node_off) : NULL;
                char       *text_off = v_off != NULL ? strdup(v_off->str) : NULL;
                if (v_off != NULL) la_vstring_destroy(v_off, true);
                if (node_off != NULL) la_proto_tree_destroy(node_off);

                CHECK(text_on != NULL && text_off != NULL && strcmp(text_on, text_off) == 0,
                      "%s bit %zu: RC_OK-full rendering differs ON vs OFF", fx->name, bit);
                free(text_on);
                free(text_off);
            } else {
                rc_fail++;
                if (partial) partials++;

                la_vstring *v = node != NULL ? la_proto_tree_format_text(NULL, node) : NULL;
                CHECK(v != NULL, "%s bit %zu: NULL/empty rendering on non-full-success", fx->name, bit);
                if (v != NULL) la_vstring_destroy(v, true);

                if (fx->kind == FIXTURE_ADSC && node != NULL && partial) {
                    la_adsc_msg_t const *m            = node->data;
                    size_t               ground_truth = tag_start_at[bit / 8];
                    CHECK(m->err_offset >= ground_truth,
                          "%s bit %zu: ADS-C framing desync (err_offset=%zu < tag start %zu)",
                          fx->name, bit, m->err_offset, ground_truth);
                }

                if (node != NULL) la_proto_tree_destroy(node);
            }

            free(mut);
        }

        free(tag_start_at);
        free(orig);
    }

    la_config_set_bool("best_effort_decode", false); // restore default

    printf("  %ld iterations: %ld RC_OK(full), %ld RC_FAIL (%ld yielded PARTIAL output)\n",
           total_iters, rc_ok_full, rc_fail, partials);
    CHECK(total_iters > 2000, "fewer bit-flip iterations than expected: %ld", total_iters);
}

int main(void)
{
    test_positive_control_mode_off();
    test_positive_control_mode_on_matches_off();
    test_adsc_truncated_payload();
    test_bitflip_fuzz();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
