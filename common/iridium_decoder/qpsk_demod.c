#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "qpsk_demod.h"
#include "sym_timing.h"

static const char *TAG = "QPSK";

// Diagnostic: dump per-symbol PLL state for the first burst that
// passes UW detection. Gated by env var QPSK_DUMP=<path> on host
// (no-op on target builds because getenv returns NULL there for
// embedded paths anyway). Used to localise where our hard-decision
// stream diverges from gri's expected symbols past the UW. One-shot.
static int s_qpsk_dump_done = 0;

static const int IR_UW_DL[] = {0, 2, 2, 2, 2, 0, 0, 0, 2, 0, 0, 2};
static const int IR_UW_UL[] = {2, 2, 0, 0, 0, 2, 0, 0, 2, 0, 2, 2};
// DQPSK Gray-code mapping for our pipeline's conventions.
//   diff 0 → 00, diff 1 → 10, diff 2 → 11, diff 3 → 01.
// EMPIRICAL: alternative {0, 1, 3, 2} (the more common Iridium-toolkit
// Gray direction) broke frame classification entirely (all 58 bursts
// went UNKNOWN, BCH-fail rose 27 → 36). This value is correct for our
// downstream deinterleave + BCH + frame_decoder chain — do not change
// without re-validating end-to-end against the RAW_IRIDIUM smoke
// frame-class counts.
static const int DQPSK_MAP[] = {0, 2, 3, 1};

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
#define PLL_ALPHA 0.2f
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
#define PLL_BETA 0.0f
// D9 two-stage acquisition was tried (PLL_ACQUIRE_ALPHA=0.5,
// PLL_ACQUIRE_BETA=0.25, PLL_ACQUIRE_SYMS=16) and reverted: wider
// initial gains did help omega_hat catch large residuals, but the
// per-symbol pll_out got noisy enough that hard-decision UW check
// dropped below its diffs<=2 threshold. Net regression on the smoke
// corpus. Real improvement needs upstream CFO accuracy, not wider-
// band PLL.
#define M_SQRT1_2f 0.70710678f

// Iridium frame is at most 191 symbols (MAX_FRAME_LEN_NORMAL_10SPS / 5 / 2
// in burst_pipeline.c). 256 leaves headroom and is power-of-two for
// stack alignment.
#define QPSK_MAX_SYMBOLS 256

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
    if (n_symbols > QPSK_MAX_SYMBOLS) {
        // Cap rather than reject: production callers feed at most one
        // frame (191 syms ≤ 256), but the host test_demod_albq does a
        // brute-force rotation×offset search over ALBQ_2SPS_LEN (~554
        // syms) and benefits from us processing whatever first window
        // fits. Truncating mirrors what gri does -- it processes the
        // first frame_size symbols and ignores the rest.
        ESP_LOGD(TAG, "n_symbols %d > QPSK_MAX_SYMBOLS %d -- truncating",
                 n_symbols, QPSK_MAX_SYMBOLS);
        n_symbols = QPSK_MAX_SYMBOLS;
    }

    // Buffers are sized for the maximum frame length and live on the
    // caller's stack -- avoiding 3 malloc/free pairs per try_decode_frame
    // call (with multi-frame this fires ~3.5x per burst). Worker stack
    // is 16 KB on P4 (xTaskCreatePinnedToCore in worker_core1.c:638);
    // 3.8 KB of locals here fits with headroom. On host all callers
    // run on the main thread with default 8 MB stack.
    float complex symbols[QPSK_MAX_SYMBOLS];
    float complex pll_out[QPSK_MAX_SYMBOLS];
    int           hard_decisions[QPSK_MAX_SYMBOLS];

    // 1. Fixed decimation to 1 sps. Symbol timing recovery (D10) is
    // available as sym_timing_correct_2sps but is NOT yet wired
    // upstream — default Gardner gains regressed the only burst that
    // decoded with correlator + pre-rotation alone. Module retained
    // for offline tuning (tests/host/test_sym_timing_trace.c).
    for (int i = 0; i < n_symbols; i++) {
        symbols[i] = (float)samples_2sps[i * 4 + 0] + (float)samples_2sps[i * 4 + 1] * _Complex_I;
    }

    // 2. First-order PLL (matches gr-iridium's qpskFirstOrderPLL). Per
    // symbol:
    //   pll_out = symbol × phi_hat
    //   err = arg(conj(x_hat) × pll_out)            // signed phase error
    //   phi_hat ← phi_hat × exp(-j·α·err)            // phase correction
    //
    // PLL_BETA = 0 here (gri-aligned, no frequency integrator), so the
    // omega_hat term is statically zero and elided. The omega_hat
    // local is kept so the diagnostic "UW no match" log can still
    // report it without ifdef gymnastics.
    //
    // Per-symbol optimisations vs the historical version:
    //   - cargf(er / |er|) → cargf(er): phase is invariant under
    //     positive-real scaling, so the magnitude division cancels
    //     and the cabsf is unnecessary.
    //   - per-symbol phi_hat normalisation removed: hard decisions
    //     depend only on the SIGNS of re/im, which are insensitive
    //     to phi_hat magnitude drift. Float roundoff over 191 mults
    //     is sub-1e-4 anyway -- no quadrant slips.
    //   - PLL_BETA path elided (compile-time 0).
    float complex phi_hat   = 1.0f + 0.0f * _Complex_I;
    float         omega_hat = 0.0f;
    // Power-decay truncation (task #73, gri-aligned). Track the running
    // peak magnitude across the burst; if three consecutive symbols
    // come in below peak/8, the actual signal has ended and the rest
    // is noise — truncate to before those 3 symbols so downstream
    // BCH / UW / DQPSK isn't fed garbage from past-end samples.
    // See gr-iridium iridium_qpsk_demod_impl.cc:216-225.
    // Compare in squared magnitude (max/8 linear ≡ max²/64) so no sqrt
    // per symbol.
    float max_mag2  = 0.0f;
    int   low_count = 0;
    int   n_eff     = n_symbols;
    for (int i = 0; i < n_symbols; i++) {
        pll_out[i] = symbols[i] * phi_hat;

        float re = crealf(pll_out[i]);
        float im = cimagf(pll_out[i]);

        // Hard decision (QPSK: pi/4, 3pi/4, -3pi/4, -pi/4)
        float complex x_hat;
        if (re >= 0 && im >= 0) {
            x_hat             = M_SQRT1_2f + M_SQRT1_2f * _Complex_I;
            hard_decisions[i] = 0;
        } else if (re < 0 && im >= 0) {
            x_hat             = -M_SQRT1_2f + M_SQRT1_2f * _Complex_I;
            hard_decisions[i] = 1;
        } else if (re < 0 && im < 0) {
            x_hat             = -M_SQRT1_2f - M_SQRT1_2f * _Complex_I;
            hard_decisions[i] = 2;
        } else {
            x_hat             = M_SQRT1_2f - M_SQRT1_2f * _Complex_I;
            hard_decisions[i] = 3;
        }

        float complex er    = conjf(x_hat) * pll_out[i];
        float         angle = cargf(er);

        // First-order phase correction: phi_hat *= exp(-j·α·angle).
        float total = PLL_ALPHA * angle;
        float c = cosf(total), s = sinf(total);
        // exp(-j·t) = cos(t) - j·sin(t). Multiply: (c - j·s) * phi_hat.
        float ph_re = crealf(phi_hat);
        float ph_im = cimagf(phi_hat);
        phi_hat     = (c * ph_re + s * ph_im) + (c * ph_im - s * ph_re) * _Complex_I;

        // Power-decay tracking on the PRE-PLL symbol magnitude (matches
        // gri — its d_magnitude_f is volk_32fc_magnitude_32f over the
        // raw `burst` input, not the PLL output). No sqrt: max_mag2 is
        // max(re² + im²).
        float sym_re = crealf(symbols[i]);
        float sym_im = cimagf(symbols[i]);
        float mag2   = sym_re * sym_re + sym_im * sym_im;
        if (mag2 > max_mag2) max_mag2 = mag2;
        if (mag2 < max_mag2 * (1.0f / 64.0f)) {
            if (++low_count == 3) {
                n_eff = i - 2; // drop the 3 low symbols themselves
                if (n_eff < 0) n_eff = 0;
                break;
            }
        } else {
            low_count = 0;
        }
    }
    if (n_eff < n_symbols) {
        ESP_LOGD(TAG, "power-decay truncation: %d → %d symbols", n_symbols, n_eff);
        n_symbols = n_eff;
    }
    // After truncation we may not have enough symbols left for the UW
    // (e.g. an extremely short burst with noise immediately after the
    // preamble). Reject before the UW check reads past the valid range.
    if (n_symbols < IR_UW_LENGTH) {
        ESP_LOGD(TAG, "truncated below UW length (%d < %d), reject",
                 n_symbols, IR_UW_LENGTH);
        return 0;
    }

    // 3. UW Check — exact port of gr-iridium's check_sync_word()
    // (lib/iridium_qpsk_demod_impl.cc:291). For each direction (DL/UL):
    //   diffs = sum_i |hard_decisions[i] - UW[i]|  with the wrap-around
    //   case diff==3 normalised to 1 (adjacent quadrants).
    //   Accept if diffs <= 2.
    //
    // This metric gives partial credit per symbol (a 1-quadrant slip
    // costs 1, a 2-quadrant slip costs 2, never costs 3) and lets a
    // single-symbol total slip pass. gri does NOT search rotations and
    // does NOT have a complex-correlation fallback -- pre-rotation in
    // burst_pipeline (peak_re/peak_im conjugate) is expected to align
    // the UW to absolute quadrants before this check runs.
    //
    // Earlier versions had a 4-rotation search plus a 0.6-threshold
    // complex-correlation fallback. The fallback was explicitly tuned
    // "for some smoke-test bursts" (~3% false positives accepted),
    // which is test-tailoring rather than gri-alignment.
    int dl_diffs = 0;
    int ul_diffs = 0;
    for (int i = 0; i < IR_UW_LENGTH; i++) {
        int d = hard_decisions[i] - IR_UW_DL[i];
        if (d < 0) d = -d;
        if (d == 3) d = 1;
        dl_diffs += d;
        d = hard_decisions[i] - IR_UW_UL[i];
        if (d < 0) d = -d;
        if (d == 3) d = 1;
        ul_diffs += d;
    }
    bool dl_uw_ok = (dl_diffs <= 2);
    bool ul_uw_ok = (ul_diffs <= 2);
    if (dl_uw_ok)
        out->direction = DIR_DOWNLINK;
    else if (ul_uw_ok)
        out->direction = DIR_UPLINK;
    else
        out->direction = DIR_UNKNOWN;

    if (out->direction == DIR_UNKNOWN) {
        // Diagnostic: show how close we were to each UW + the actual
        // hard decisions for the first 12 symbols. Helps tell apart
        // "PLL never locked" (random hard_decisions) from "wrong
        // burst alignment" (decisions structured but offset).
        ESP_LOGI(TAG,
                 "UW no match: dl_diffs=%d ul_diffs=%d omega=%.4f hd[0..11]=[%d %d %d %d %d %d %d %d %d %d %d %d]",
                 dl_diffs, ul_diffs, (double)omega_hat,
                 hard_decisions[0], hard_decisions[1], hard_decisions[2],
                 hard_decisions[3], hard_decisions[4], hard_decisions[5],
                 hard_decisions[6], hard_decisions[7], hard_decisions[8],
                 hard_decisions[9], hard_decisions[10], hard_decisions[11]);
        return 0;
    }

    // 4. DQPSK Decode + per-bit soft metric for Chase-2 BCH (#112).
    //
    // QPSK confidence per symbol = |pll_out[i]| (the distance from
    // origin in the constellation plane). At low SNR the constellation
    // points are pulled toward origin, reducing |pll_out|; at high SNR
    // they sit near the canonical (±1, ±1)/√2 points. The same
    // confidence is assigned to both bits derived from a symbol — a
    // conservative simplification (real per-axis soft would split |re|
    // for one bit and |im| for the other, but DQPSK couples the two
    // axes nonlinearly so the symbol-magnitude is the safe approximation).
    //
    // Scale: max_mag2 (from step 3) bounds |pll_out|² across the burst;
    // we normalise so the brightest symbol sits near INT16 mid-scale,
    // giving Chase-2 plenty of soft headroom without overflow.
    float mag_scale = 0.0f;
    if (max_mag2 > 0.0f) {
        mag_scale = 16384.0f / sqrtf(max_mag2); // brightest symbol → ~16k
    }
    int old_sym    = 0;
    out->bits      = malloc(n_symbols * 2);
    out->soft_bits = malloc(n_symbols * 2 * sizeof(int16_t));
    out->n_bits    = n_symbols * 2;
    for (int i = 0; i < n_symbols; i++) {
        int diff             = (hard_decisions[i] - old_sym + 4) % 4;
        old_sym              = hard_decisions[i];
        int     decoded      = DQPSK_MAP[diff];
        uint8_t b0           = (decoded >> 1) & 1;
        uint8_t b1           = decoded & 1;
        out->bits[2 * i + 0] = b0;
        out->bits[2 * i + 1] = b1;
        if (out->soft_bits) {
            float re  = crealf(pll_out[i]);
            float im  = cimagf(pll_out[i]);
            float mag = sqrtf(re * re + im * im) * mag_scale;
            if (mag > 32000.0f) mag = 32000.0f;
            int16_t conf = (int16_t)mag;
            // Sign convention: bit=0 → positive soft, bit=1 → negative.
            out->soft_bits[2 * i + 0] = b0 ? (int16_t)-conf : conf;
            out->soft_bits[2 * i + 1] = b1 ? (int16_t)-conf : conf;
        }
    }

    ESP_LOGI(TAG, "Successfully demodulated %s frame, %d bits",
             (out->direction == DIR_DOWNLINK) ? "DL" : "UL", out->n_bits);

#ifndef ESP_PLATFORM
    // One-shot per-symbol dump for the first UW-locked burst. Writes
    // i, re_in, im_in, re_pll, im_pll, hd, bit_hi, bit_lo per row so an
    // external script can compare against gri's expected hd / bits and
    // pinpoint the symbol index where PLL phase or sample timing
    // diverges. Set QPSK_DUMP=/tmp/qpsk_dump.csv to enable.
    if (!s_qpsk_dump_done) {
        const char *path = getenv("QPSK_DUMP");
        if (path && path[0]) {
            FILE *fp = fopen(path, "w");
            if (fp) {
                fprintf(fp, "i,re_in,im_in,re_pll,im_pll,hd,bit_hi,bit_lo\n");
                int prev = 0;
                for (int i = 0; i < n_symbols; i++) {
                    int diff    = (hard_decisions[i] - prev + 4) % 4;
                    prev        = hard_decisions[i];
                    int decoded = DQPSK_MAP[diff];
                    fprintf(fp, "%d,%.6f,%.6f,%.6f,%.6f,%d,%d,%d\n",
                            i,
                            (double)crealf(symbols[i]),
                            (double)cimagf(symbols[i]),
                            (double)crealf(pll_out[i]),
                            (double)cimagf(pll_out[i]),
                            hard_decisions[i],
                            (decoded >> 1) & 1, decoded & 1);
                }
                fclose(fp);
                ESP_LOGI(TAG, "QPSK_DUMP: wrote %d symbols to %s",
                         n_symbols, path);
            }
            s_qpsk_dump_done = 1;
        }
    }
#endif

    return 1;
}
