// Host test: after priming, fft_burst_tagger_reset_baseline() returns the
// tagger to the un-primed state so it re-learns the noise floor. Behavior is
// observed through the public step() return value (false while re-priming).
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "fft_burst_tagger.h"

int main(void)
{
    int32_t *hist = calloc((size_t)FBT_FFT_SIZE * FBT_HISTORY_SIZE, sizeof(int32_t));
    assert(hist);
    fft_burst_tagger_t *t = fft_burst_tagger_init(2 * FBT_FFT_SIZE, 40000, 32, 10.0f, hist);
    assert(t);

    int16_t in[2 * FBT_FFT_SIZE];
    int16_t look[2 * (2 * FBT_FFT_SIZE)];
    memset(in, 0, sizeof(in));
    memset(look, 0, sizeof(look));

    // Prime: step until step() reports primed (returns true).
    fbt_burst_t nb[16], gb[16];
    int         nn, ng;
    bool        primed = false;
    for (int i = 0; i < FBT_HISTORY_SIZE + 4 && !primed; i++) {
        nn     = 16;
        ng     = 16;
        primed = fft_burst_tagger_step(t, in, look, nb, &nn, gb, &ng);
    }
    assert(primed); // sanity: tagger reached primed state

    // Reset — next step must report un-primed again.
    fft_burst_tagger_reset_baseline(t);
    nn         = 16;
    ng         = 16;
    bool after = fft_burst_tagger_step(t, in, look, nb, &nn, gb, &ng);
    assert(after == false);

    fft_burst_tagger_destroy(t);
    free(hist);
    return 0;
}
