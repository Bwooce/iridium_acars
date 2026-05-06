// Real-RF regression for ida_decode against the Albuquerque corpus.
//
// Iterates over every frame in fixture_albq_frames_corpus.h, classifies
// it via iridium_frame_classify, and for those that come back as
// IR_FRAME_LW with subtype IR_LW_DA runs them through ida_decode.
// Reports per-frame BCH success counts. The pass criterion: every
// IDA frame should have at least 8 of 10 BCH blocks decode (real
// noisy bursts may lose 1-2 blocks; the SBD reassembler upstream
// tolerates partial frames via segment retry).

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "iridium_frame.h"
#include "ida_decode.h"
#include "fixture_albq_frames_corpus.h"

int main(void)
{
    int n_ida_classified = 0;
    int n_decode_ok      = 0;        // all 10 blocks decoded cleanly
    int n_decode_partial = 0;        // ≥8 blocks decoded (tolerable)
    int n_decode_bad     = 0;        // <8 blocks decoded
    int total_blocks_ok  = 0;
    int total_errors     = 0;

    printf("Test: ida_decode on every IDA frame in the Albuquerque corpus (n=%u total entries)\n",
           ALBQ_FRAME_CORPUS_LEN);

    for (unsigned int i = 0; i < ALBQ_FRAME_CORPUS_LEN; i++) {
        const albq_frame_corpus_entry_t *e = &ALBQ_FRAME_CORPUS[i];
        iridium_frame_t f = { 0 };
        if (iridium_frame_classify(e->bits, e->n_bits,
                                   e->expected_direction, &f) != 0) {
            printf("  classify rc != 0 on entry %u (%s)\n", i, e->parser_class);
            return 1;
        }
        if (f.type != IR_FRAME_LW || f.lw_subtype != IR_LW_DA) {
            continue;   // not an IDA frame; skip
        }
        n_ida_classified++;

        ida_decoded_t d = { 0 };
        int rc = ida_decode(&f, &d);
        if (rc != 0) {
            printf("  ida_decode rc=%d on IDA entry %u (parser=%s)\n",
                   rc, i, e->parser_class);
            n_decode_bad++;
            continue;
        }

        total_blocks_ok += d.blocks_ok;
        total_errors    += d.total_errors;

        if (d.blocks_ok == 10)      n_decode_ok++;
        else if (d.blocks_ok >= 8)  n_decode_partial++;
        else                        n_decode_bad++;
    }

    printf("\nResults (over %d IDA frames):\n", n_ida_classified);
    printf("  Clean decode (10/10 blocks):  %d (%.1f%%)\n",
           n_decode_ok,
           n_ida_classified ? 100.0 * n_decode_ok / n_ida_classified : 0.0);
    printf("  Partial decode (8-9/10 ok):   %d (%.1f%%)\n",
           n_decode_partial,
           n_ida_classified ? 100.0 * n_decode_partial / n_ida_classified : 0.0);
    printf("  Bad decode (<8/10 ok):        %d (%.1f%%)\n",
           n_decode_bad,
           n_ida_classified ? 100.0 * n_decode_bad / n_ida_classified : 0.0);
    if (n_ida_classified > 0) {
        printf("  Avg blocks_ok / frame:        %.1f\n",
               (double)total_blocks_ok / n_ida_classified);
        printf("  Avg BCH errors corrected:     %.1f / frame\n",
               (double)total_errors / n_ida_classified);
    }

    if (n_ida_classified == 0) {
        printf("\nFAIL: no IDA frames in corpus — fixture regenerator broken?\n");
        return 1;
    }
    if (n_decode_bad > 0) {
        printf("\nFAIL: %d IDA frames had <8/10 BCH blocks decode\n", n_decode_bad);
        return 1;
    }

    printf("\nPASS: all %d IDA frames decoded with ≥8/10 BCH blocks\n",
           n_ida_classified);
    return 0;
}
