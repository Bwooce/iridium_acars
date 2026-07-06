// Host validation for the libacars best-effort/partial decode work
// (docs/superpowers/plans/2026-07-07-libacars-best-effort-decode.md,
// Phase H = design §§1-8, host-side only).
//
// This file grows across the phase's commits:
//   - positive control (design §6a): the 7 intact fixtures cited in
//     libacars/examples/{cpdlc,adsc}_get_position.c decode cleanly, and
//     render byte-identically with best_effort_decode ON vs OFF (an
//     intact fixture never takes the partial path either way).
//   - bit-flip fuzz (§6b).
//   - ZK-NNC acceptance demo (§6c): the real, previously-"Unparseable"
//     FANS-1/A message from the 2026-07-06 milestone capture.
//
// Fixtures are the full ARINC-622 text messages ("/GSADDR.IMI.<air_reg><hex
// payload+CRC>"), decoded via la_arinc_parse() exactly like the upstream
// example programs do -- this exercises the same code path production
// uses (la_arinc_parse -> la_cpdlc_parse / la_adsc_parse).

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <libacars/libacars.h>
#include <libacars/arinc.h>
#include <libacars/cpdlc.h>
#include <libacars/adsc.h>
#include <libacars/acars.h>
#include <libacars/crc.h>
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
    la_msg_dir     dir;
} fixture_t;

// design §6a citations:
//   CPDLC (3): libacars/examples/cpdlc_get_position.c:33,37-39
//   ADS-C (4): libacars/examples/adsc_get_position.c:26,30-32
// UPLINK-ADSC is synthetic (no uplink example ships with libacars): a
// periodic contract request (tag 0x07), contract number 5, with six
// sub-tags covering every request-sub-tag parser shape -- reporting
// interval (0x0B, 1B), flight ID modulus (0x0C, 1B), vertical speed
// event (0x12, 1B), altitude range (0x13, 4B), waypoint-change report
// (0x14, empty/no parser), aircraft intent (0x15, 2B). It exists
// because all example fixtures are downlink, so the uplink tag table
// and la_adsc_contract_request_parse's nested sub-tag loop were never
// fuzzed -- which hid a real formatter crash (see
// test_adsc_review_repros). Trailing "FFFF" is a dummy ARINC CRC:
// arinc.c computes crc_ok but never gates parsing on it.
static const fixture_t FIXTURES[] = {
    {"SOUCAYA", "/SOUCAYA.AT1."
                "HL8251243F880C3D903BB412903604FE326C2479F4A64F7F62528B1A9CF8382738186AC28B16668E013DF464D8A7F0",
     FIXTURE_CPDLC, LA_MSG_DIR_AIR2GND},
    {"MSTEC7X", "/MSTEC7X.AT1."
                "VT-ANKA094D88C3D903BB465D0723053B2E5123CFA53279400014B0894A2C6A73CBD8F52447AF1244CB4C9B94600089D65C84314892694587510528B1A9CF41169D440C1AB36A08B42",
     FIXTURE_CPDLC, LA_MSG_DIR_AIR2GND},
    {"MELCAYA", "/MELCAYA.AT1."
                "ZK-OKC253C21CC3D903BA178F96618F0B28024B83127CD7886A12E9D85266B927584A9169C1A8EEB2800EEA7",
     FIXTURE_CPDLC, LA_MSG_DIR_AIR2GND},
    {"BOMASAI", "/BOMASAI.ADS."
                "VT-ANB072501A070A988CA73248F0E5DC10200000F5EE1ABC000102B885E0A19F5",
     FIXTURE_ADSC, LA_MSG_DIR_AIR2GND},
    {"AUHASMO", "/AUHASMO.ADS."
                "A6-PFE0724D9586A36C92B2DCF1F0E74A8E4807C0F7219AF407C10422E9E08A1C4",
     FIXTURE_ADSC, LA_MSG_DIR_AIR2GND},
    {"CTUE1YA", "/CTUE1YA.ADS."
                "HB-JNB1424AB686D9308CA2EBA1D0D24A2C06C1B48CA004A248050667908CA004BF6",
     FIXTURE_ADSC, LA_MSG_DIR_AIR2GND},
    {"YQXE2YA", "/YQXE2YA.ADS."
                "SP-LRH1424FD087806C0B527769F0D2500B877ED00B5401E2516707755C01340B768",
     FIXTURE_ADSC, LA_MSG_DIR_AIR2GND},
    {"UPLINK-ADSC", "/AKLCDYA.ADS."
                    "ZKNNC107050B8A0C021205130FA003E81415030AFFFF",
     FIXTURE_ADSC, LA_MSG_DIR_GND2AIR},
};
#define NUM_FIXTURES (int)(sizeof(FIXTURES) / sizeof(FIXTURES[0]))

// design §6a: intact fixtures must decode with err == false and produce a
// non-empty rendering, both with best_effort_decode OFF (today's baseline).
static void test_positive_control_mode_off(void)
{
    printf("Test: positive control, %d intact fixtures decode cleanly (mode OFF)\n", NUM_FIXTURES);

    for (int i = 0; i < NUM_FIXTURES; i++) {
        fixture_t const *fx   = &FIXTURES[i];
        la_proto_node   *node = la_arinc_parse(fx->arinc_text, fx->dir);
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
    la_proto_node *node = la_arinc_parse(fx->arinc_text, fx->dir);
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
// la_adsc_tag_parse leak fix (failed-tag data freed at the failure
// site): under a plain (non-ASan) build this only proves the fields
// come out right; the actual leak-freed-or-not is what the ASan build
// verifies.
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

// Regression tests for the two proven repros from the 2026-07-07
// top-tier review of this branch.
static void test_adsc_review_repros(void)
{
    printf("Test: review repros -- contract-request formatter crash + flag-OFF fabricated data\n");

    // Repro 1 (crash): uplink contract request (tag 0x07) whose sub-tag
    // 0x0B (reporting interval) is truncated to zero group bytes. With
    // the earlier type-before-parse leak fix, the failed sub-tag ended
    // up with type set + data NULL, and
    // la_adsc_contract_request_format_text() handed the NULL data
    // straight to la_adsc_reporting_interval_format_text() -> NULL
    // deref. Must render cleanly (no crash under ASan) in both modes.
    //
    // Repro 2 (flag-OFF contract violation): downlink noncompliance
    // notification (tag 0x05) declaring 2 groups but carrying only 1.
    // The parser fails after populating half of t->data; rendering that
    // partially-populated allocation fabricated a calloc-zero "Tag 0"
    // group -- with the flag OFF, where output must be bit-for-bit
    // upstream's. Upstream renders exactly "-- Unparseable tag 5" +
    // "-- Malformed ADS-C message".
    struct {
        char const    *name;
        uint8_t const *payload;
        int            len;
        la_msg_dir     dir;
        char const    *expect_off_text; // exact flag-OFF rendering (upstream-equivalent)
    } const repros[] = {
        {"uplink 07 01 0B", (uint8_t const[]){0x07, 0x01, 0x0B}, 3, LA_MSG_DIR_GND2AIR,
         "-- Unparseable tag 7\n-- Malformed ADS-C message\n"},
        {"downlink 05 01 02 07 40", (uint8_t const[]){0x05, 0x01, 0x02, 0x07, 0x40}, 5, LA_MSG_DIR_AIR2GND,
         "-- Unparseable tag 5\n-- Malformed ADS-C message\n"},
    };

    for (size_t r = 0; r < sizeof(repros) / sizeof(repros[0]); r++) {
        for (int mode = 0; mode < 2; mode++) {
            bool const flag = (mode == 1);
            la_config_set_bool("best_effort_decode", flag);
            la_proto_node *node = la_adsc_parse(repros[r].payload, repros[r].len,
                                                repros[r].dir, ARINC_MSG_ADS);
            CHECK(node != NULL, "%s: NULL node", repros[r].name);
            if (node == NULL) {
                continue;
            }
            la_adsc_msg_t const *m = node->data;
            CHECK(m->err == true, "%s: err not set (flag=%d)", repros[r].name, flag);
            CHECK(m->partial == flag, "%s: partial=%d, expected %d", repros[r].name, m->partial, flag);

            // Formatting must not crash (repro 1's NULL deref) and must
            // never surface fabricated group data (repro 2's "Tag 0").
            la_vstring *tv = la_proto_tree_format_text(NULL, node);
            CHECK(tv != NULL, "%s: NULL text render (flag=%d)", repros[r].name, flag);
            if (tv != NULL) {
                if (!flag) {
                    CHECK(strcmp(tv->str, repros[r].expect_off_text) == 0,
                          "%s: flag-OFF text differs from upstream-equivalent:\n%s",
                          repros[r].name, tv->str);
                } else {
                    CHECK(strstr(tv->str, "PARTIAL/UNTRUSTED") != NULL,
                          "%s: no partial banner with flag ON", repros[r].name);
                }
                CHECK(strstr(tv->str, "Tag 0") == NULL,
                      "%s: fabricated zero-group data rendered (flag=%d)", repros[r].name, flag);
                la_vstring_destroy(tv, true);
            }
            la_vstring *jv = la_proto_tree_format_json(NULL, node);
            CHECK(jv != NULL, "%s: NULL json render (flag=%d)", repros[r].name, flag);
            if (jv != NULL) la_vstring_destroy(jv, true);

            la_proto_tree_destroy(node);
        }
    }
    la_config_set_bool("best_effort_decode", false); // restore default
}

// Returns the exact bytes production code feeds to la_cpdlc_parse() /
// la_adsc_parse(): hex-decodes the payload after the fixed
// "/GSADDR.IMI<air_reg>" prefix (1+7+1+3+7 = 19 chars, true for all
// fixtures -- 7-char ground address, 3-char IMI, then the 7-char
// air_reg field which INCLUDES the dot separator: arinc.c copies
// LA_ARINC_AIR_REG_LEN bytes from payload+LA_ARINC_IMI_LEN, ie. from
// the '.' right after "AT1"/"ADS" -- observe "air_addr":".ZK-NNC" in
// the JSON output) and strips the trailing 2-byte CRC, mirroring
// la_arinc_parse() (la_slurp_hexstring(), then
// `buflen -= LA_ARINC_CRC_LEN`). Caller frees *buf via free() (matches
// LA_XCALLOC/free pairing used throughout libacars).
static size_t get_fixture_payload(fixture_t const *fx, uint8_t **buf)
{
    char const *hexpart_lit = fx->arinc_text + 19;
    char       *hexpart     = strdup(hexpart_lit);
    size_t      n           = la_slurp_hexstring(hexpart, buf);
    free(hexpart);
    if (n < 2) {
        return 0;
    }
    return n - 2; // strip CRC
}

// Collects the tag values of an ADS-C parse's tag_list in order.
// Returns the count; writes up to max values into out.
static int collect_adsc_tags(la_proto_node *node, uint8_t *out, int max)
{
    la_adsc_msg_t const *m = node->data;
    int                  n = 0;
    for (la_list *l = m->tag_list; l != NULL && n < max; l = la_list_next(l)) {
        out[n++] = ((la_adsc_tag_t const *)l->data)->tag;
    }
    return n;
}

// design §6b: flip every bit of every fixture's actual decode buffer
// (~2,240 iterations total, vs the design's ~2,250 estimate) with
// best_effort_decode ON, under ASan+UBSan (HOST_TESTS_ASAN). Assertions:
//   - no sanitizer hits (enforced externally by the ASan build itself --
//     a crash here means the whole ctest run fails)
//   - every failed decode yields either PARTIAL output or a clean
//     "Unparseable"/"Malformed" rendering -- never a NULL/garbage render
//   - every flip that still fully decodes renders IDENTICALLY (text AND
//     json) whether best_effort_decode is ON or OFF -- the flag must
//     never change behaviour on a message that fully decoded
//   - ADS-C framing survival (design §6b's "value-byte flips in
//     fixed-length groups must NOT desync framing (subsequent group
//     tags still decode)"): for downlink fixtures, a flip at any
//     NON-tag-byte position must still parse to a FULL success with a
//     tag sequence identical to the original's -- the corrupted value
//     cannot move any subsequent tag boundary because every group in
//     these fixtures is fixed-length (asserted below: no tag 4 NACK /
//     tag 5 noncompliance, the downlink variable-length exceptions).
//     Tag-byte flips are exempt (a flipped tag value legitimately
//     reframes or kills the parse -- the documented exception), as is
//     the uplink fixture (contract-request sub-tag lists are the third
//     documented variable-framing exception). Tag-byte positions are
//     derived from a truncation sweep of the ORIGINAL buffer: byte b
//     starts a tag iff b == 0 or a decode truncated to exactly b bytes
//     succeeds (b is then a clean inter-tag boundary).
static void test_bitflip_fuzz(void)
{
    printf("Test: bit-flip fuzz, every bit of all %d fixtures (best_effort_decode ON)\n", NUM_FIXTURES);
    la_config_set_bool("best_effort_decode", true);

    long total_iters = 0, rc_ok_full = 0, rc_fail = 0, partials = 0, framing_checked = 0;

    for (int i = 0; i < NUM_FIXTURES; i++) {
        fixture_t const *fx   = &FIXTURES[i];
        uint8_t         *orig = NULL;
        size_t           len  = get_fixture_payload(fx, &orig);
        CHECK(orig != NULL && len > 0, "%s: failed to extract raw payload", fx->name);
        if (orig == NULL || len == 0) {
            free(orig);
            continue;
        }

        // ADS-C ground truth for the framing assertion (downlink only).
        uint8_t orig_tags[64];
        int     orig_tag_cnt   = 0;
        bool   *is_tag_byte    = NULL;
        bool    strong_framing = false;
        if (fx->kind == FIXTURE_ADSC && fx->dir == LA_MSG_DIR_AIR2GND) {
            // Original tag sequence (also the fixed-length precondition:
            // tags 4/5 are the downlink variable-length groups; the
            // strong framing assertion is only valid without them).
            la_proto_node *on = la_adsc_parse(orig, (int)len, fx->dir, ARINC_MSG_ADS);
            CHECK(on != NULL && ((la_adsc_msg_t const *)on->data)->err == false,
                  "%s: original payload no longer parses cleanly", fx->name);
            orig_tag_cnt   = collect_adsc_tags(on, orig_tags, (int)(sizeof orig_tags));
            strong_framing = true;
            for (int k = 0; k < orig_tag_cnt; k++) {
                if (orig_tags[k] == 4 || orig_tags[k] == 5) {
                    strong_framing = false; // variable-length group present
                }
            }
            la_proto_tree_destroy(on);

            // Tag-byte map via truncation: a truncation of L bytes
            // parses cleanly iff L is an inter-tag boundary, so byte
            // offset b starts a tag iff b == 0 or truncation to b
            // succeeds.
            is_tag_byte    = calloc(len, sizeof(bool));
            is_tag_byte[0] = true;
            for (size_t trunc = 1; trunc < len; trunc++) {
                la_proto_node *n = la_adsc_parse(orig, (int)trunc, fx->dir, ARINC_MSG_ADS);
                if (n != NULL) {
                    if (((la_adsc_msg_t const *)n->data)->err == false) {
                        is_tag_byte[trunc] = true;
                    }
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
                                      ? la_cpdlc_parse(mut, (int)len, fx->dir)
                                      : la_adsc_parse(mut, (int)len, fx->dir, ARINC_MSG_ADS);

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

            // Framing-survival assertion (see the test header comment).
            if (fx->kind == FIXTURE_ADSC && strong_framing && !is_tag_byte[bit / 8]) {
                framing_checked++;
                CHECK(full_success,
                      "%s bit %zu: value-byte flip desynced framing (parse failed)", fx->name, bit);
                if (node != NULL && full_success) {
                    uint8_t mut_tags[64];
                    int     mut_cnt = collect_adsc_tags(node, mut_tags, (int)(sizeof mut_tags));
                    CHECK(mut_cnt == orig_tag_cnt && memcmp(mut_tags, orig_tags, (size_t)orig_tag_cnt) == 0,
                          "%s bit %zu: value-byte flip changed the tag sequence", fx->name, bit);
                }
            }

            if (full_success) {
                rc_ok_full++;
                la_vstring *v_on    = la_proto_tree_format_text(NULL, node);
                la_vstring *j_on    = la_proto_tree_format_json(NULL, node);
                char       *text_on = v_on != NULL ? strdup(v_on->str) : NULL;
                char       *json_on = j_on != NULL ? strdup(j_on->str) : NULL;
                if (v_on != NULL) la_vstring_destroy(v_on, true);
                if (j_on != NULL) la_vstring_destroy(j_on, true);
                la_proto_tree_destroy(node);

                la_config_set_bool("best_effort_decode", false);
                la_proto_node *node_off = (fx->kind == FIXTURE_CPDLC)
                                              ? la_cpdlc_parse(mut, (int)len, fx->dir)
                                              : la_adsc_parse(mut, (int)len, fx->dir, ARINC_MSG_ADS);
                la_config_set_bool("best_effort_decode", true);
                la_vstring *v_off    = node_off != NULL ? la_proto_tree_format_text(NULL, node_off) : NULL;
                la_vstring *j_off    = node_off != NULL ? la_proto_tree_format_json(NULL, node_off) : NULL;
                char       *text_off = v_off != NULL ? strdup(v_off->str) : NULL;
                char       *json_off = j_off != NULL ? strdup(j_off->str) : NULL;
                if (v_off != NULL) la_vstring_destroy(v_off, true);
                if (j_off != NULL) la_vstring_destroy(j_off, true);
                if (node_off != NULL) la_proto_tree_destroy(node_off);

                CHECK(text_on != NULL && text_off != NULL && strcmp(text_on, text_off) == 0,
                      "%s bit %zu: full-success text rendering differs ON vs OFF", fx->name, bit);
                CHECK(json_on != NULL && json_off != NULL && strcmp(json_on, json_off) == 0,
                      "%s bit %zu: full-success json rendering differs ON vs OFF", fx->name, bit);
                free(text_on);
                free(text_off);
                free(json_on);
                free(json_off);
            } else {
                rc_fail++;
                if (partial) partials++;

                la_vstring *v = node != NULL ? la_proto_tree_format_text(NULL, node) : NULL;
                CHECK(v != NULL, "%s bit %zu: NULL/empty rendering on non-full-success", fx->name, bit);
                if (v != NULL) la_vstring_destroy(v, true);

                if (node != NULL) la_proto_tree_destroy(node);
            }

            free(mut);
        }

        free(is_tag_byte);
        free(orig);
    }

    la_config_set_bool("best_effort_decode", false); // restore default

    printf("  %ld iterations: %ld full-success, %ld failed/partial (%ld yielded PARTIAL output), "
           "%ld strong framing checks\n",
           total_iters, rc_ok_full, rc_fail, partials, framing_checked);
    CHECK(total_iters > 2000, "fewer bit-flip iterations than expected: %ld", total_iters);
    CHECK(framing_checked > 500, "framing-survival assertion barely exercised: %ld", framing_checked);
}

// design §6c: the real ZK-NNC FANS-1/A message from the 2026-07-06
// HydraSDR milestone capture (~/iridium_bits/acars-milestone2-*.txt),
// which renders only "Unparseable FANS-1/A message" today. Builds a
// real ACARS frame around it (mirroring test_libacars_link.c's
// canonical-frame construction) with label "H1" so la_acars_parse()
// dispatches into the real la_arinc_parse() -> la_cpdlc_parse() chain
// itself (acars.c's la_acars_apps_parse_and_reassemble, not a direct
// la_arinc_parse() call like the other fixtures in this file), for
// maximum fidelity to how a live capture would actually reach this code.
static void test_zknnc_acceptance(void)
{
    printf("Test: ZK-NNC real-capture acceptance demo (design §6c)\n");

    uint8_t buf[128];
    size_t  i = 0;
    buf[i++]  = '2';
    memcpy(buf + i, "ZK-NNC1", 7);
    i += 7;
    buf[i++] = 0x15; // ack: NAK
    buf[i++] = 'H';
    buf[i++] = '1';  // label H1
    buf[i++] = '5';  // block_id digit -> downlink
    buf[i++] = 0x02; // STX
    memcpy(buf + i, "M001", 4);
    i += 4;
    memcpy(buf + i, "ZKNNC1", 6);
    i += 6;
    char const *txt = "/BNECAYA.AT1.ZK-NNC21A95A5D848F166C18769B166429F5694E8A136B00C314";
    memcpy(buf + i, txt, strlen(txt));
    i += strlen(txt);
    buf[i++]     = 0x03; // ETX
    uint16_t crc = la_crc16_ccitt(buf, (uint32_t)i, 0);
    buf[i++]     = (uint8_t)(crc & 0xff);
    buf[i++]     = (uint8_t)((crc >> 8) & 0xff);
    buf[i++]     = 0x7f; // DEL

    // Baseline: flag OFF must match "current behaviour" exactly --
    // today's code renders "Unparseable FANS-1/A message" for this
    // capture and nothing else.
    la_config_set_bool("best_effort_decode", false);
    la_proto_node *node_off = la_acars_parse(buf, (int)i, LA_MSG_DIR_AIR2GND);
    CHECK(node_off != NULL, "ZK-NNC: la_acars_parse returned NULL (flag OFF)");
    la_proto_node *cn_off = node_off != NULL ? la_proto_tree_find_cpdlc(node_off) : NULL;
    CHECK(cn_off != NULL, "ZK-NNC: no CPDLC node in tree (flag OFF)");
    if (cn_off != NULL) {
        la_cpdlc_msg const *m = cn_off->data;
        CHECK(m->err == true, "ZK-NNC: err not set with flag OFF (today's baseline should be unparseable)");
        CHECK(m->partial == false, "ZK-NNC: partial set with flag OFF (should be impossible)");
    }
    la_vstring *v_off = node_off != NULL ? la_proto_tree_format_text(NULL, node_off) : NULL;
    CHECK(v_off != NULL && strstr(v_off->str, "Unparseable FANS-1/A message") != NULL,
          "ZK-NNC: missing today's baseline \"Unparseable\" text with flag OFF");
    if (v_off != NULL) la_vstring_destroy(v_off, true);
    if (node_off != NULL) la_proto_tree_destroy(node_off);

    // best_effort_decode ON: PARTIAL banner + header msgID + element
    // TYPE name + nonzero consumed_bits.
    la_config_set_bool("best_effort_decode", true);
    la_proto_node *node_on = la_acars_parse(buf, (int)i, LA_MSG_DIR_AIR2GND);
    CHECK(node_on != NULL, "ZK-NNC: la_acars_parse returned NULL (flag ON)");
    la_proto_node *cn_on = node_on != NULL ? la_proto_tree_find_cpdlc(node_on) : NULL;
    CHECK(cn_on != NULL, "ZK-NNC: no CPDLC node in tree (flag ON)");
    if (cn_on != NULL) {
        la_cpdlc_msg const *m = cn_on->data;
        CHECK(m->partial == true, "ZK-NNC: partial not set with flag ON");
        CHECK(m->consumed_bits > 0, "ZK-NNC: consumed_bits == 0 with flag ON");
    }
    la_vstring *v_on = node_on != NULL ? la_proto_tree_format_text(NULL, node_on) : NULL;
    CHECK(v_on != NULL, "ZK-NNC: NULL text rendering with flag ON");
    if (v_on != NULL) {
        // "display only" is common to both banner wordings (desync and
        // trailing-junk) -- this particular message happens to hit the
        // trailing-junk case (a complete, if semantically-reserved,
        // structure decodes from the first few bytes; see the money-shot
        // printout below).
        CHECK(strstr(v_on->str, "display only") != NULL, "ZK-NNC: no PARTIAL/NOTE banner with flag ON");
        CHECK(strstr(v_on->str, "Msg ID:") != NULL, "ZK-NNC: header msgID not rendered with flag ON");
        // Pinned to the specific, deterministic, real-message result:
        // this hex decodes (from the CHOICE index survivable per design
        // §2) to the reserved/placeholder downlink element "dM118NULL".
        // Not test-fitting -- this is the actual, correct-per-ASN.1-spec
        // output for THIS real capture; pinning it catches regressions.
        CHECK(strstr(v_on->str, "dM118NULL") != NULL, "ZK-NNC: expected element TYPE name \"dM118NULL\" not found");
        printf("--- ZK-NNC decoded output (best_effort_decode ON) ---\n%s", v_on->str);
        la_vstring_destroy(v_on, true);
    }
    if (node_on != NULL) la_proto_tree_destroy(node_on);

    la_config_set_bool("best_effort_decode", false); // restore default
}

int main(void)
{
    test_positive_control_mode_off();
    test_positive_control_mode_on_matches_off();
    test_adsc_truncated_payload();
    test_adsc_review_repros();
    test_bitflip_fuzz();
    test_zknnc_acceptance();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
