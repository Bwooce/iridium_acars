// TDD for the configurable near-DC exclusion window. Feeds seeded noise to
// prime the per-bin EMA, then a strong tone at a chosen bin, and checks that
// the DC-mask window gates NEW bursts at that bin — and ONLY there.
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "fft_burst_tagger.h"

#define N FBT_FFT_SIZE // 2048
#define DC (N / 2)     // 1024
#define FS 2500000.0

static int32_t s_baseline_history[FBT_HISTORY_SIZE * FBT_FFT_SIZE];

// Fill n_complex interleaved int16 IQ with seeded uniform noise in [-amp,amp].
static void gen_noise(int16_t *buf, int n_complex, unsigned *seed, int amp)
{
    for (int i = 0; i < 2 * n_complex; i++)
        buf[i] = (int16_t)((int)(rand_r(seed) % (2 * amp + 1)) - amp);
}

// Add a complex tone at FFT bin `bin` (absolute, 0..N-1) with amplitude amp.
static void add_tone(int16_t *buf, int n_complex, int bin, int amp, double *phase)
{
    double w = 2.0 * M_PI * (double)(bin - DC) / (double)N; // cycles/sample
    for (int i = 0; i < n_complex; i++) {
        buf[2 * i + 0] = (int16_t)(buf[2 * i + 0] + (int)(amp * cos(*phase)));
        buf[2 * i + 1] = (int16_t)(buf[2 * i + 1] + (int)(amp * sin(*phase)));
        *phase += w;
    }
}

// Run the tagger over `prime` noise-only steps then `active` noise+tone steps
// at `tone_bin`; return how many NEW bursts landed within ±16 bins of tone_bin.
static int run_case(int tone_bin, int mask_lo, int mask_hi)
{
    fft_burst_tagger_t *t = fft_burst_tagger_init(
        2 * FBT_FFT_SIZE, (int)(FS * 16e-3), 32, 10.0f, s_baseline_history);
    fft_burst_tagger_set_start(t, 0);
    fft_burst_tagger_set_dc_mask(t, mask_lo, mask_hi);

    int16_t     buf[2 * N];
    fbt_burst_t nb[FBT_MAX_BURSTS], gb[FBT_MAX_BURSTS];
    unsigned    seed  = 1234;
    double      phase = 0.0;
    int         hits  = 0;

    for (int s = 0; s < FBT_HISTORY_SIZE + 60; s++) {
        gen_noise(buf, N, &seed, 200);
        if (s >= FBT_HISTORY_SIZE + 8) add_tone(buf, N, tone_bin, 9000, &phase);
        int  nn = FBT_MAX_BURSTS, ng = FBT_MAX_BURSTS;
        bool primed = fft_burst_tagger_step(t, buf, NULL, nb, &nn, gb, &ng);
        if (primed)
            for (int i = 0; i < nn; i++)
                if (abs(nb[i].center_bin - tone_bin) <= 16) hits++;
    }
    fft_burst_tagger_destroy(t);
    return hits;
}

int main(void)
{
    int fails = 0;
    int tone  = DC - 40; // ~ -49 kHz, in the near-DC artifact region

    // A) Mask DISABLED (lo>hi) → the tone triggers bursts.
    int a = run_case(tone, 1, -1);
    printf("A disabled: hits=%d (want >0)\n", a);
    if (a <= 0) fails++;

    // B) Mask window COVERS the tone bin → zero bursts there.
    int b = run_case(tone, -44, -36);
    printf("B covered:  hits=%d (want 0)\n", b);
    if (b != 0) fails++;

    // C) Mask window ELSEWHERE (does not cover the tone) → tone still triggers.
    int c = run_case(tone, 30, 38);
    printf("C elsewhere:hits=%d (want >0)\n", c);
    if (c <= 0) fails++;

    if (fails) {
        printf("test_tagger_dc_mask: %d FAILURES\n", fails);
        return 1;
    }
    printf("test_tagger_dc_mask: PASS\n");
    return 0;
}
