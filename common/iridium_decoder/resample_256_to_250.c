// See resample_256_to_250.h.

#include "resample_256_to_250.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#if __has_include("esp_attr.h")
#include "esp_attr.h"
#else
#define EXT_RAM_BSS_ATTR
#endif

// Per-phase contiguous tap layout. Each phase gets 16 int16 slots
// (9 real taps + 7 zero pad), so the PIE-asm hot path can do two
// 8-lane vmulas covering all 16 with the pad contributing 0. INTERP
// = 125 phases × 16 = 2000 int16 = 4 KB. Singleton (read-only after
// init), placed in PSRAM .bss because adding it to the struct's
// internal-SRAM footprint broke the boot-time DMA-pool reserve.
// Access cost is L2 cache only (read-mostly).
#define RS25_PADDED_TAPS 16
static EXT_RAM_BSS_ATTR int16_t s_coeffs_pp[RS25_PADDED_TAPS * RS25_INTERP]
    __attribute__((aligned(16)));
static int s_coeffs_pp_ready = 0;

#if defined(__riscv) && __has_include("soc/soc_caps.h")
#include "soc/soc_caps.h"
#endif

// PIE-accelerated 9-tap MAC, defined in resample_arp4.S. Pads to 16
// taps (zero-padded). Computes one output sample pair from
// pre-loaded delay lines + per-phase tap pointer. Self-gates on
// __riscv && SOC_CPU_HAS_PIE in the .S file.
#if defined(__riscv) && defined(SOC_CPU_HAS_PIE)
extern void resample_125_128_mac_arp4(const int16_t *pc,
                                       const int16_t *di,
                                       const int16_t *dq,
                                       int16_t *out_i,
                                       int16_t *out_q);
#define RS25_USE_PIE_ASM 1
#else
#define RS25_USE_PIE_ASM 0
#endif

// Modified Bessel I0, for Kaiser window. Copy of the same function in
// direct_if_decim.c — could be deduped, but the two modules are
// independent.
static double bessel_i0(double x)
{
    double sum = 1.0, term = 1.0;
    double half_x_sq = (x * x) / 4.0;
    for (int k = 1; k < 50; k++) {
        term *= half_x_sq / (double)(k * k);
        sum += term;
        if (term < 1e-12 * sum) break;
    }
    return sum;
}

// Build the firmr_s16 coefficient table.
//
// scipy.signal.resample_poly(x, up=125, down=128) generates an FIR
// with these defaults (per scipy.signal._signaltools._resample_poly_fir):
//   max_rate    = max(up, down)            = 128
//   ntaps_proto = 2 × 10 × max_rate + 1   = 2561   (proto in the
//                                                   interp×fs virtual
//                                                   frame)
//   cutoff_norm = 1 / max_rate            = 0.0078125  (in cycles/sample
//                                                       in the virtual frame)
//   beta        = 5.0                      (kaiser default)
//
// But the FULL 2561-tap prototype is overkill for our application
// (gri's tagger is robust to a few dB of amplitude variation and a
// few hundred Hz transition-band ripple). We use a SHORTER prototype
// to fit memory budget at ~5 KB:
//   delay_size  = 9
//   ntaps_proto = delay_size × interp = 1125
//   cutoff      = 0.4 / interp (giving ~1.25 MHz passband edge in
//                 the virtual 320 MSPS frame, well past anything we
//                 care about)
//   beta        = 8.0  (60 dB stopband — more than gri spec, cheap
//                       at this length)
//
// Layout for firmr_s16: coeffs[tap_pos * interp + phase]. tap_pos
// counts symbol-spaced taps (0..delay_size-1); phase counts virtual
// sample positions within each output cycle (0..interp-1).
//
// Mapping from the linear-phase prototype: the (tap_pos, phase) entry
// reads prototype[tap_pos × interp + phase]. We sum-normalise so the
// per-phase DC gain is unity (= integral of the LPF response at DC).
static void make_resample_coeffs(int16_t *coeffs)
{
    const int      INTERP    = RS25_INTERP;
    const int      DSIZE     = RS25_DELAY_SIZE;
    const int      NPROTO    = DSIZE * INTERP;            // 1125
    const double   PI        = 3.14159265358979323846;
    const double   beta      = 8.0;
    // Filter cutoff in cycles/sample at the virtual interp×fs rate.
    // We pick 0.4 × (1/interp) so the passband covers ±0.4 × fs_in/2
    // = ±0.4 × 1.28 MHz = ±512 kHz at 2.56 MSPS input. Way more than
    // we need for Iridium (channel grid spans ±400 kHz around LO).
    const double   fc_norm   = 0.4 / (double)INTERP;
    const double   inv_i0    = 1.0 / bessel_i0(beta);
    const int      center    = NPROTO / 2;

    // 1125 doubles = 9000 bytes — too large to live on this task's stack
    // (class_driver_task is 4 KB and would overflow before bessel_i0
    // returns). Allocate on the heap for the design pass only; freed
    // before this function returns.
    double *w = (double *)malloc(sizeof(double) * RS25_DELAY_SIZE * RS25_INTERP);
    if (!w) return;
    double sum_phase[RS25_INTERP];
    for (int p = 0; p < INTERP; p++) sum_phase[p] = 0.0;

    for (int k = 0; k < NPROTO; k++) {
        double t = (double)(k - center);
        double sinc = (t == 0.0)
                      ? 2.0 * fc_norm
                      : sin(2.0 * PI * fc_norm * t) / (PI * t);
        double u = 2.0 * (double)k / (double)(NPROTO - 1) - 1.0;
        double arg = beta * sqrt(1.0 - u * u);
        double kw  = bessel_i0(arg) * inv_i0;
        w[k] = sinc * kw;
        sum_phase[k % INTERP] += w[k];
    }
    // Normalise each phase to DC gain = 1 / INTERP (gri convention:
    // the polyphase resampler's per-phase MAC sums divided by the
    // implicit "interp output samples per input cycle" factor). With
    // the firmr_s16 shift=15 (matching dsps_firmr_s16 default), the
    // taps need to sum to ~1.0 per phase to give unity DC gain after
    // the >> 15 in the MAC accumulator. So normalise each phase to
    // sum=1.
    for (int p = 0; p < INTERP; p++) {
        if (sum_phase[p] == 0.0) continue;
        double scale = 1.0 / sum_phase[p];
        for (int t = 0; t < DSIZE; t++) {
            w[t * INTERP + p] *= scale;
        }
    }

    // Quantise to Q15. Per-phase max tap ≈ 0.3 → Q15 ≈ 9830, well
    // within int16. Layout matches firmr_s16: coeffs[tap × interp + phase].
    for (int k = 0; k < NPROTO; k++) {
        double v = w[k] * (double)INT16_MAX;
        if (v >  (double)INT16_MAX) v =  (double)INT16_MAX;
        if (v < -(double)INT16_MAX) v = -(double)INT16_MAX;
        coeffs[k] = (int16_t)lrint(v);
    }

    free(w);
}

void resample_256_to_250_init(resample_256_to_250_t *r)
{
    make_resample_coeffs(r->coeffs);
    // Build per-phase contiguous layout, padded to RS25_PADDED_TAPS=16
    // so the PIE-asm inner can do exactly two vmulas covering all
    // 16 lanes (the last 7 padding zeros contribute 0). Indexed as
    // s_coeffs_pp[phase * 16 + tap]; tap [0..8] = original
    // coeffs[tap × INTERP + phase], tap [9..15] = 0.
    memset(s_coeffs_pp, 0, sizeof(s_coeffs_pp));
    for (int phase = 0; phase < RS25_INTERP; phase++) {
        for (int tap = 0; tap < RS25_DELAY_SIZE; tap++) {
            s_coeffs_pp[phase * RS25_PADDED_TAPS + tap] =
                r->coeffs[tap * RS25_INTERP + phase];
        }
    }
    s_coeffs_pp_ready = 1;
    memset(r->delay_i, 0, sizeof(r->delay_i));
    memset(r->delay_q, 0, sizeof(r->delay_q));
    r->start_pos = 0;
    // firmr_s16 retained for host comparison / regression coverage;
    // the production path no longer routes through it.
    firmr_s16_init(&r->fir_i, r->coeffs, r->delay_i,
                   RS25_DELAY_SIZE, RS25_INTERP, RS25_DECIM,
                   /*start_pos=*/ 0, /*shift=*/ 0);
    firmr_s16_init(&r->fir_q, r->coeffs, r->delay_q,
                   RS25_DELAY_SIZE, RS25_INTERP, RS25_DECIM,
                   /*start_pos=*/ 0, /*shift=*/ 0);
}

int resample_256_to_250_process(resample_256_to_250_t *r,
                                 const int16_t *in_iq, int n_in_complex,
                                 int16_t *out_iq)
{
    // Fast specialised polyphase 125/128 path. Bit-exact to the
    // firmr_s16 reference (which is itself a port of esp-dsp's
    // dsps_firmr_s16_ansi) -- same Q15 rounding (`acc += 0x7fff`),
    // same right-shift by 15, same per-phase normalised coefficients.
    // Optimisations vs the previous firmr_s16-based path:
    //   - no per-call malloc/free (was ~256 KB of I/Q + output
    //     scratch allocated and freed every dispatch);
    //   - no separate deinterleave/reinterleave (process IQ
    //     interleaved in a single fused loop);
    //   - linear delay buffer with unrolled 9-sample memmove
    //     (eliminates the circular pos+wrap branch inside the
    //     inner tap loop, which prevented unrolling there);
    //   - per-phase contiguous coefficients (built in init) so
    //     the 9-tap inner is a small contiguous load of
    //     coeffs_pp[phase*DSIZE .. +9].
    //
    // Measured on LIVE_SDR: drops the 17.9 ms/dispatch resample
    // step that was throttling USB ingest to 0.85 MB/s.
    int16_t *__restrict di = r->delay_i;
    int16_t *__restrict dq = r->delay_q;
    const int16_t *__restrict coeffs_pp = s_coeffs_pp;
    int start_pos = r->start_pos;
    int n_out = 0;
    (void)s_coeffs_pp_ready;   // future: gate fast path on init

    for (int i = 0; i < n_in_complex; i++) {
        // Shift delay line: 9 elements, newest at index 0. Manually
        // unrolled so the compiler can use the eight 16-bit slots
        // as registers and avoid memmove call overhead. delay[9..15]
        // are tail-pad slots for future PIE 128-bit loads; they
        // stay zero and the inner loop only consumes [0..8].
        di[8] = di[7]; di[7] = di[6]; di[6] = di[5]; di[5] = di[4];
        di[4] = di[3]; di[3] = di[2]; di[2] = di[1]; di[1] = di[0];
        di[0] = in_iq[2 * i + 0];
        dq[8] = dq[7]; dq[7] = dq[6]; dq[6] = dq[5]; dq[5] = dq[4];
        dq[4] = dq[3]; dq[3] = dq[2]; dq[2] = dq[1]; dq[1] = dq[0];
        dq[0] = in_iq[2 * i + 1];

        // For 125/128 ratio the inner-m loop fires at most once per
        // input: m starts at start_pos, m += 128 exits the loop;
        // start_pos -= 125 for the next iteration. Across 42 inputs
        // we emit 41 outputs (one skipped where m wraps past 125),
        // matching the 125/128 ratio.
        if (start_pos < RS25_INTERP) {
            const int16_t *__restrict pc =
                &coeffs_pp[start_pos * RS25_PADDED_TAPS];
#if RS25_USE_PIE_ASM
            // PIE asm: two esp.vmulas.s16.xacc per channel (covers
            // 16 zero-padded taps), one esp.srs.s.xacc shift per
            // output. Bit-exact to the C fallback below (same Q15
            // 0x7fff rounding, same >>15 truncation).
            resample_125_128_mac_arp4(pc, di, dq,
                                       &out_iq[2 * n_out + 0],
                                       &out_iq[2 * n_out + 1]);
#else
            // C fallback (host build / non-PIE targets). int64 acc:
            // max 9 × Q30 = ~9.6e9 exceeds int32 signed range. Adding
            // the 0x7fff rounding term first matches the firmr_s16
            // reference.
            int64_t acc_i = 0x7fff;
            int64_t acc_q = 0x7fff;
            for (int k = 0; k < RS25_DELAY_SIZE; k++) {
                int32_t c = pc[k];
                acc_i += (int32_t)di[k] * c;
                acc_q += (int32_t)dq[k] * c;
            }
            out_iq[2 * n_out + 0] = (int16_t)(acc_i >> 15);
            out_iq[2 * n_out + 1] = (int16_t)(acc_q >> 15);
#endif
            n_out++;
            start_pos += RS25_DECIM;
        }
        start_pos -= RS25_INTERP;
    }

    r->start_pos = start_pos;
    return n_out;
}
