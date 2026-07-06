// Host validation for the libacars best-effort/partial decode work
// (docs/superpowers/plans/2026-07-07-libacars-best-effort-decode.md,
// Phase H = design §§1-8, host-side only).
//
// This file grows across the phase's commits:
//   - positive control (design §6a): the 7 intact fixtures cited in
//     libacars/examples/{cpdlc,adsc}_get_position.c decode cleanly.
//     Initially (this commit) only mode OFF is exercised, because the
//     best_effort_decode config flag and la_cpdlc_msg/la_adsc_msg_t
//     partial-state fields don't exist yet -- this commit only proves
//     the host build wiring (design §6d) and pins today's baseline
//     behaviour before any decode-path code is touched.
//   - later commits add: mode ON == mode OFF byte-identical rendering
//     for the same 7 fixtures, the bit-flip fuzz (§6b), and the ZK-NNC
//     acceptance demo (§6c).
//
// Fixtures are the full ARINC-622 text messages ("/GSADDR.IMI.<air_reg><hex
// payload+CRC>"), decoded via la_arinc_parse() exactly like the upstream
// example programs do -- this exercises the same code path production
// uses (la_arinc_parse -> la_cpdlc_parse / la_adsc_parse).

#include <stdio.h>
#include <stdbool.h>
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

int main(void)
{
    test_positive_control_mode_off();
    printf("\n=== %d passed, %d failed ===\n", passed, failed);
    return failed == 0 ? 0 : 1;
}
