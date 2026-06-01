// Diagnostic tool: run sym_timing_correct_2sps on the corpus fixture
// at the textbook gains and dump (symbol, e, mu, w) to stdout for
// offline analysis. Not part of the regression suite — purely a
// tuning aid.

#include "sym_timing.h"
#include "fixture_corpus_2sps.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    // Allow Kp/Ki to be passed as argv[1]/argv[2] for sweep scripts.
    float kp = 0.055f, ki = 0.00019f;
    if (argc >= 2) kp = (float)atof(argv[1]);
    if (argc >= 3) ki = (float)atof(argv[2]);

    sym_timing_t st;
    sym_timing_init(&st);
    st.Kp = kp;
    st.Ki = ki;

    sym_timing_trace_t trace;
    sym_timing_set_trace(&st, &trace);

    int16_t *corrected = malloc(CORPUS_2SPS_LEN * sizeof(int16_t));
    if (!corrected) {
        fprintf(stderr, "malloc fail\n");
        return 1;
    }

    sym_timing_correct_2sps(&st, CORPUS_2SPS, (int)CORPUS_2SPS_LEN,
                            corrected);

    fprintf(stderr, "# Corpus: %u int16 samples (%u complex)\n",
            CORPUS_2SPS_LEN, CORPUS_2SPS_LEN / 2);
    fprintf(stderr, "# Gains: Kp=%g Ki=%g\n", kp, ki);
    fprintf(stderr, "# Trace samples: %d\n", trace.n);

    // Mean & RMS of e — to spot bias.
    double sum_e = 0, sum_e2 = 0;
    for (int i = 0; i < trace.n; i++) {
        sum_e += trace.e[i];
        sum_e2 += (double)trace.e[i] * trace.e[i];
    }
    double mean_e = trace.n > 0 ? sum_e / trace.n : 0;
    double rms_e  = trace.n > 0 ? sqrt(sum_e2 / trace.n) : 0;
    fprintf(stderr, "# e: mean=%.6f rms=%.6f (mean/rms = bias ratio = %.3f)\n",
            mean_e, rms_e, rms_e > 0 ? mean_e / rms_e : 0);

    // mu trajectory: initial, final, max excursion
    float mu_min = trace.n > 0 ? trace.mu[0] : 0;
    float mu_max = trace.n > 0 ? trace.mu[0] : 0;
    for (int i = 0; i < trace.n; i++) {
        if (trace.mu[i] < mu_min) mu_min = trace.mu[i];
        if (trace.mu[i] > mu_max) mu_max = trace.mu[i];
    }
    fprintf(stderr, "# mu: start=%.4f end=%.4f range=[%.4f, %.4f]\n",
            trace.n > 0 ? trace.mu[0] : 0,
            trace.n > 0 ? trace.mu[trace.n - 1] : 0,
            mu_min, mu_max);

    // Tab-separated per-symbol dump on stdout.
    printf("# idx\te\tmu\tw\n");
    for (int i = 0; i < trace.n; i++) {
        printf("%d\t%.6f\t%.6f\t%.6f\n", i, trace.e[i], trace.mu[i], trace.w[i]);
    }

    free(corrected);
    return 0;
}
