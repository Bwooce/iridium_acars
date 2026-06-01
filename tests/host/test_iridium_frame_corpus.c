// Regression test: run iridium_frame_classify across every burst from
// the Albuquerque corpus and cross-check against iridium-toolkit's
// iridium-parser.py classification.
//
// Phase A only does exact-match MS / TL detection; BC and LW require
// BCH polynomial checks that come in Phase B. So most corpus entries
// (IDA / ISY / IIU / I36 / IBC) currently expect IR_FRAME_LW or
// IR_FRAME_BC but our classifier returns IR_FRAME_UNKNOWN. We measure
// agreement and report — failure threshold is set per phase.
//
// The point of this test in Phase A is twofold:
//   1. **Soundness:** if we EVER incorrectly classify an IDA/ISY/etc.
//      frame as MS or TL, that's a false positive — fail the build.
//   2. **Coverage:** count how many of the corpus's 82 entries match
//      our classifier output. As phases B (BC/LW BCH check) and beyond
//      land, this number rises until eventual full agreement.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "iridium_frame.h"
#include "fixture_albq_frames_corpus.h"

static const char *type_name(ir_frame_type_t t)
{
    return iridium_frame_type_name(t);
}

int main(void)
{
    int agree           = 0;
    int phase_a_correct = 0;
    int false_positive  = 0;
    int known_unhandled = 0;

    int counts_actual[5]   = {0}; // UNKNOWN, MS, TL, BC, LW
    int counts_expected[5] = {0};

    printf("Test: iridium_frame_classify vs iridium-toolkit on %d corpus bursts\n",
           ALBQ_FRAME_CORPUS_LEN);

    for (unsigned int i = 0; i < ALBQ_FRAME_CORPUS_LEN; i++) {
        const albq_frame_corpus_entry_t *e  = &ALBQ_FRAME_CORPUS[i];
        iridium_frame_t                  f  = {0};
        int                              rc = iridium_frame_classify(e->bits, e->n_bits,
                                                                     e->expected_direction, &f);
        if (rc != 0) {
            printf("  entry %u (%s): classify rc=%d\n",
                   i, e->parser_class, rc);
            return 1;
        }

        if ((int)f.type < 5) counts_actual[f.type]++;
        if ((int)e->expected_type < 5) counts_expected[e->expected_type]++;

        // Phase A correctness: any frame we classify as MS or TL must
        // be a frame the parser also called MS or TL. Otherwise it's a
        // false-positive header match — broken classifier.
        if (f.type == IR_FRAME_MS || f.type == IR_FRAME_TL) {
            if (f.type == e->expected_type) {
                phase_a_correct++;
            } else {
                printf("  FALSE POSITIVE entry %u (%s): we said %s, "
                       "parser said %s. SNR=%.1f, freq=%u Hz\n",
                       i, e->parser_class, type_name(f.type),
                       e->parser_class, e->snr_db, e->freq_hz);
                false_positive++;
            }
        }

        if (f.type == e->expected_type)
            agree++;
        else if (f.type == IR_FRAME_UNKNOWN && (e->expected_type == IR_FRAME_LW || e->expected_type == IR_FRAME_BC)) {
            // Expected — Phase A does not classify LW/BC yet.
            known_unhandled++;
        }
    }

    printf("\nClassifier output histogram:\n");
    printf("  UNKNOWN: %d (parser-expected: %d)\n",
           counts_actual[IR_FRAME_UNKNOWN], counts_expected[IR_FRAME_UNKNOWN]);
    printf("  MS:      %d (parser-expected: %d)\n",
           counts_actual[IR_FRAME_MS], counts_expected[IR_FRAME_MS]);
    printf("  TL:      %d (parser-expected: %d)\n",
           counts_actual[IR_FRAME_TL], counts_expected[IR_FRAME_TL]);
    printf("  BC:      %d (parser-expected: %d)  [Phase B]\n",
           counts_actual[IR_FRAME_BC], counts_expected[IR_FRAME_BC]);
    printf("  LW:      %d (parser-expected: %d)  [Phase B]\n",
           counts_actual[IR_FRAME_LW], counts_expected[IR_FRAME_LW]);

    printf("\nAgreement: %d/%d (%.1f%%) match exactly\n",
           agree, ALBQ_FRAME_CORPUS_LEN,
           100.0 * agree / ALBQ_FRAME_CORPUS_LEN);
    printf("Phase-A specific:\n");
    printf("  correctly classified MS+TL: %d\n", phase_a_correct);
    printf("  false positives:            %d\n", false_positive);
    printf("  known-unhandled (LW/BC):    %d (Phase B will pick these up)\n",
           known_unhandled);

    if (false_positive > 0) {
        printf("\nFAIL: %d false-positive header matches\n", false_positive);
        return 1;
    }

    // Phase A pass criterion: zero false positives. We don't require
    // hitting any specific MS/TL count because the corpus might not
    // contain any (the Albuquerque capture is duplex-band only).
    printf("\nPASS (no false positives)\n");
    return 0;
}
