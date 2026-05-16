#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "qpsk_demod.h"
#include "sym_timing.h"

static const char *TAG = "QPSK";

static const int IR_UW_DL[] = { 0, 2, 2, 2, 2, 0, 0, 0, 2, 0, 0, 2 };
static const int IR_UW_UL[] = { 2, 2, 0, 0, 0, 2, 0, 0, 2, 0, 2, 2 };
static const int DQPSK_MAP[] = { 0, 2, 3, 1 };

// Second-order PLL gains. ALPHA is the phase (proportional) term;
// BETA is the frequency (integral) term. Critically-damped second-
// order rule of thumb: beta ≈ alpha² / 4. With alpha = 0.2 → beta
// = 0.01. The frequency integrator lets the loop track a residual
// carrier offset that the first-order phase-only loop couldn't
// (a constant freq offset of f Hz produces a phase error that
// integrates monotonically; omega_hat accumulates the integral and
// supplies it as a feed-forward to phi_hat).
//
// D9 motivation: D8 produces ±700 Hz residual carrier. Drift rate at
// 700 Hz / 25 ksym/s = ~10°/symbol. The old first-order loop at
// ALPHA=0.2 only corrected ~2° per symbol = couldn't keep up. Over
// the 12-symbol UW the constellation rotated ~120° while phase
// correction kept up with ~24°. With the frequency integrator
// settling, omega_hat absorbs the constant rate and phase tracking
// catches up within a few symbols.
// ALPHA = phase (proportional) gain, BETA = frequency (integral) gain.
// First-order rule of thumb is beta ≈ alpha²/4 = 0.01 for critical
// damping. But that's tuned for STEADY-STATE tracking — for FAST
// ACQUISITION (the 12-symbol UW lock window) we need much more
// aggressive integral gain or omega_hat hasn't caught up in time.
// At 0.1, omega_hat reaches the true offset rate within ~6 symbols
// for ±700 Hz residual (verified by simulation). Trades steady-
// state noise for acquisition speed — acceptable for burst-mode
// demod where each burst is a fresh acquisition.
#define PLL_ALPHA       0.2f
#define PLL_BETA        0.1f
#define M_SQRT1_2f      0.70710678f

int qpsk_demod_process(const int16_t *samples_2sps, int n_samples, decoded_frame_t *out)
{
    // n_samples is the number of int16_t values (I, Q interleaved) at 2 sps.
    // Each complex sample is 2 int16_t.
    int n_complex_samples_2sps = n_samples / 2;
    
    // We want to decimate 2sps to 1sps (1 symbol per sample).
    int n_symbols = n_complex_samples_2sps / 2;

    if (n_symbols < IR_UW_LENGTH) {
        ESP_LOGD(TAG, "Not enough symbols for UW check (%d < %d)", n_symbols, IR_UW_LENGTH);
        return 0;
    }

    float complex *symbols = malloc(n_symbols * sizeof(float complex));
    float complex *pll_out = malloc(n_symbols * sizeof(float complex));
    int *hard_decisions = malloc(n_symbols * sizeof(int));
    
    if (!symbols || !pll_out || !hard_decisions) {
        ESP_LOGE(TAG, "Failed to allocate demod buffers");
        free(symbols); free(pll_out); free(hard_decisions);
        return 0;
    }

    // 1. Simple decimation to 1 sps. D10 (sym_timing.c) is built and
    // unit-tested but NOT wired here. Direct integration broke the
    // host demod regressions because Gardner introduces per-symbol
    // strobe jitter that the pre-aligned host fixtures don't have.
    // The clean integration design: a sym_timing variant that
    // PRESERVES 2-sps output format (replace bad samples with
    // interpolated good ones) so qpsk_demod's decimation can still
    // pick the right sample. That's a separate design exercise.
    for (int i = 0; i < n_symbols; i++) {
        symbols[i] = (float)samples_2sps[i * 4 + 0]
                   + (float)samples_2sps[i * 4 + 1] * _Complex_I;
    }

    // 2. Second-order PLL (D9). Tracks both phase (phi_hat) and
    // frequency (omega_hat, rad/sym). Per symbol:
    //   pll_out = symbol × phi_hat
    //   err = arg(conj(x_hat) × pll_out)            // signed phase error
    //   phi_hat ← phi_hat × exp(-j(α·err + ω_hat))   // phase + freq feed-fwd
    //   ω_hat += β · err                              // frequency integrator
    float complex phi_hat = 1.0f + 0.0f * _Complex_I;
    float omega_hat = 0.0f;
    for (int i = 0; i < n_symbols; i++) {
        pll_out[i] = symbols[i] * phi_hat;

        float re = crealf(pll_out[i]);
        float im = cimagf(pll_out[i]);

        // Hard decision (QPSK: pi/4, 3pi/4, -3pi/4, -pi/4)
        float complex x_hat;
        if (re >= 0 && im >= 0)      { x_hat = M_SQRT1_2f + M_SQRT1_2f * _Complex_I; hard_decisions[i] = 0; }
        else if (re < 0 && im >= 0) { x_hat = -M_SQRT1_2f + M_SQRT1_2f * _Complex_I; hard_decisions[i] = 1; }
        else if (re < 0 && im < 0)  { x_hat = -M_SQRT1_2f - M_SQRT1_2f * _Complex_I; hard_decisions[i] = 2; }
        else                        { x_hat = M_SQRT1_2f - M_SQRT1_2f * _Complex_I; hard_decisions[i] = 3; }

        float complex er = conjf(x_hat) * pll_out[i];
        float er_mag = cabsf(er);
        float angle = (er_mag > 1e-10f) ? cargf(er / er_mag) : 0.0f;

        // Combined rotation: alpha·err (proportional) + omega_hat (integral).
        // phi_hat *= exp(-j*total) to oppose the measured drift.
        float total = PLL_ALPHA * angle + omega_hat;
        float complex correction = cosf(total) + sinf(total) * _Complex_I;
        phi_hat = conjf(correction) * phi_hat;
        // Normalise to prevent drift.
        float mag = cabsf(phi_hat);
        if (mag > 1e-10f) phi_hat /= mag;

        // Frequency integrator update. omega_hat is in rad/sym.
        omega_hat += PLL_BETA * angle;
    }

    // 3. UW Check. The PLL has 4 stable phase points (90° ambiguity);
    // it may converge to any of the 4 rotations of the symbol
    // constellation. The UWs are absolute-quadrant patterns, so we
    // test all 4 rotations to find the match. DQPSK below is
    // rotation-invariant so the bits emerge unchanged regardless of
    // which rotation matched.
    int dl_diffs = IR_UW_LENGTH + 1;
    int ul_diffs = IR_UW_LENGTH + 1;
    int dl_rot = 0, ul_rot = 0;
    for (int rot = 0; rot < 4; rot++) {
        int dl = 0, ul = 0;
        for (int i = 0; i < IR_UW_LENGTH; i++) {
            int v = (hard_decisions[i] - rot + 4) & 3;
            dl += (v != IR_UW_DL[i]);
            ul += (v != IR_UW_UL[i]);
        }
        if (dl < dl_diffs) { dl_diffs = dl; dl_rot = rot; }
        if (ul < ul_diffs) { ul_diffs = ul; ul_rot = rot; }
    }

    int chosen_rot = 0;
    if (dl_diffs <= 2)      { out->direction = DIR_DOWNLINK; chosen_rot = dl_rot; }
    else if (ul_diffs <= 2) { out->direction = DIR_UPLINK;   chosen_rot = ul_rot; }
    else                    { out->direction = DIR_UNKNOWN; }

    // Apply the chosen rotation to hard_decisions so the DQPSK decode
    // below produces bits anchored to the right quadrant reference.
    // DQPSK is differential so uniform rotation doesn't change the
    // diff sequence — but it does change the first-symbol baseline,
    // which the downstream consumers may rely on.
    if (out->direction != DIR_UNKNOWN && chosen_rot != 0) {
        for (int i = 0; i < n_symbols; i++) {
            hard_decisions[i] = (hard_decisions[i] - chosen_rot + 4) & 3;
        }
    }

    if (out->direction == DIR_UNKNOWN) {
        // Diagnostic: show how close we were to each UW + the actual
        // hard decisions for the first 12 symbols. Helps tell apart
        // "PLL never locked" (random hard_decisions) from "wrong
        // burst alignment" (decisions structured but offset).
        ESP_LOGD(TAG,
            "UW no match: dl=%d (rot %d) ul=%d (rot %d) omega=%.4f hd[0..11]=[%d %d %d %d %d %d %d %d %d %d %d %d]",
            dl_diffs, dl_rot, ul_diffs, ul_rot, (double)omega_hat,
            hard_decisions[0], hard_decisions[1], hard_decisions[2],
            hard_decisions[3], hard_decisions[4], hard_decisions[5],
            hard_decisions[6], hard_decisions[7], hard_decisions[8],
            hard_decisions[9], hard_decisions[10], hard_decisions[11]);
        free(symbols); free(pll_out); free(hard_decisions);
        return 0;
    }

    // 4. DQPSK Decode
    int old_sym = 0;
    out->bits = malloc(n_symbols * 2);
    out->n_bits = n_symbols * 2;
    for (int i = 0; i < n_symbols; i++) {
        int diff = (hard_decisions[i] - old_sym + 4) % 4;
        old_sym = hard_decisions[i];
        int decoded = DQPSK_MAP[diff];
        out->bits[2 * i + 0] = (decoded >> 1) & 1;
        out->bits[2 * i + 1] = decoded & 1;
    }

    ESP_LOGI(TAG, "Successfully demodulated %s frame, %d bits", 
             (out->direction == DIR_DOWNLINK) ? "DL" : "UL", out->n_bits);

    free(symbols); free(pll_out); free(hard_decisions);
    return 1;
}
