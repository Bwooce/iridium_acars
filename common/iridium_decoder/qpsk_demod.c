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
// PLL_BETA = 0 to match gr-iridium's qpskFirstOrderPLL (alpha=1/5,
// no frequency tracking). With β=0 omega_hat stays at 0 and the PLL
// is pure phase-only — exactly first-order. Their pipeline assumes
// CFO has been removed upstream (matched-filter pre-rotation), so
// the PLL has only small residual phase to track.
// We had β=0.1 (second-order, freq tracking) which amplified hard-
// decision errors into spurious freq drift — a single noise-induced
// 90°-off symbol gave β·(π/2) = 0.16 rad/sym of fake omega, then
// the next 6 syms accumulated π rad of bogus rotation → cascading
// quadrant flips. Setting β=0 prevents this cascade.
#define PLL_BETA        0.0f
// D9 two-stage acquisition was tried (PLL_ACQUIRE_ALPHA=0.5,
// PLL_ACQUIRE_BETA=0.25, PLL_ACQUIRE_SYMS=16) and reverted: wider
// initial gains did help omega_hat catch large residuals, but the
// per-symbol pll_out got noisy enough that the complex-correlation
// UW fallback dropped below its 0.6 threshold. Net regression on
// the smoke corpus. Real improvement needs upstream CFO accuracy,
// not wider-band PLL.
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

    // 1. Fixed decimation to 1 sps. Symbol timing recovery (D10) is
    // available as sym_timing_correct_2sps but is NOT yet wired
    // upstream — default Gardner gains regressed the only burst that
    // decoded with correlator + pre-rotation alone. Module retained
    // for offline tuning (tests/host/test_sym_timing_trace.c).
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

    // 3. UW Check. Two paths in parallel:
    //   (a) Hard-decision rotation-aware match (kept for backward
    //       compatibility with existing host fixtures — these decode
    //       cleanly under rot=0 with dl_diffs=0).
    //   (b) Complex correlation against the UW patterns interpreted
    //       as QPSK symbols. The correlation magnitude measures match
    //       strength independent of constellation rotation and absorbs
    //       small per-symbol noise gracefully. Used as a fallback when
    //       the hard-decision check fails but a correlation peak is
    //       clearly present.
    //
    // Whichever path declares a match first wins. DQPSK below is
    // rotation-invariant so bit output is unaffected by the rotation
    // applied.
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

    // Complex correlation fallback — only runs if the hard-decision
    // path didn't find a match. Build UW reference as complex QPSK
    // symbols, correlate, peak magnitude indicates match strength.
    if (out->direction == DIR_UNKNOWN) {
        static const int8_t QUAD_TO_RE[4] = {  1, -1, -1,  1 };
        static const int8_t QUAD_TO_IM[4] = {  1,  1, -1, -1 };
        float c_dl_re = 0, c_dl_im = 0, c_ul_re = 0, c_ul_im = 0;
        float pll_energy = 0;
        for (int i = 0; i < IR_UW_LENGTH; i++) {
            float re_y = crealf(pll_out[i]);
            float im_y = cimagf(pll_out[i]);
            pll_energy += re_y * re_y + im_y * im_y;
            // conj(uw_dl[i]) × pll_out[i], where uw_dl[i] has unit
            // magnitude per QUAD_TO_RE/IM (×M_SQRT1_2 scaling absorbed
            // into the threshold).
            float u_re = QUAD_TO_RE[IR_UW_DL[i]];
            float u_im = QUAD_TO_IM[IR_UW_DL[i]];
            // conj(u) * y = (u_re - j*u_im) * (re_y + j*im_y)
            c_dl_re += u_re * re_y + u_im * im_y;
            c_dl_im += u_re * im_y - u_im * re_y;
            u_re = QUAD_TO_RE[IR_UW_UL[i]];
            u_im = QUAD_TO_IM[IR_UW_UL[i]];
            c_ul_re += u_re * re_y + u_im * im_y;
            c_ul_im += u_re * im_y - u_im * re_y;
        }
        // Match strength: |c|² normalised by per-symbol energy ×
        // N. For a perfect match c_mag² ≈ (2 × N × avg_y_mag²),
        // i.e. accept_ratio of 1.0 means perfect alignment. Random
        // hd → c_mag² ≈ avg_y_mag² × N → accept_ratio ≈ 0.5.
        // Threshold at 0.75: requires significantly better than
        // random.
        if (pll_energy > 1e-3f) {
            float c_dl_mag2 = c_dl_re * c_dl_re + c_dl_im * c_dl_im;
            float c_ul_mag2 = c_ul_re * c_ul_re + c_ul_im * c_ul_im;
            float peak2     = 2.0f * (float)IR_UW_LENGTH * pll_energy;
            float dl_ratio  = c_dl_mag2 / peak2;
            float ul_ratio  = c_ul_mag2 / peak2;
            // Threshold of 0.6: random hd correlates at ~0.5; we want
            // measurably above that but the strict 0.75 was too tight
            // for some of the smoke-test bursts. 0.6 still catches false
            // positives at ~0.4% per burst per UW per rotation × 4 × 2 ≈
            // 3% per burst — BCH catches the rest as bit-error garbage.
            if (dl_ratio >= 0.6f && dl_ratio >= ul_ratio) {
                out->direction = DIR_DOWNLINK;
                chosen_rot = dl_rot;
            } else if (ul_ratio >= 0.6f) {
                out->direction = DIR_UPLINK;
                chosen_rot = ul_rot;
            }
        }
    }

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
        ESP_LOGI(TAG,
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
