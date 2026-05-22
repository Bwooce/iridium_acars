// See resample_256_to_250.h.

#include "resample_256_to_250.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#if __has_include("esp_attr.h")
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#endif

// Per-phase contiguous tap layout. Each phase gets 16 int16 slots
// (9 real taps + 7 zero pad), so the PIE-asm hot path can do two
// 8-lane vmulas covering all 16 with the pad contributing 0. INTERP
// = 125 phases × 16 = 2000 int16 = 4 KB. Singleton (read-only after
// init).
//
// Placement: heap-allocated in internal SRAM at init time via
// MALLOC_CAP_INTERNAL. PIE vld instructions cannot efficiently
// fetch from .spm.data (TCM — see task #75), and PSRAM reads cost
// ~315 ns/MAC, dominating the 14 ns PIE-instruction floor.
//
// We can't use a static .dram.bss array: that lands BEFORE the
// PSRAM driver's 144 KB DMA-pool reserve, fragmenting the largest
// contiguous internal-SRAM region below the reserve's threshold
// and panic'ing boot with ESP_ERR_NO_MEM. Runtime heap_caps_alloc
// runs AFTER the DMA pool is set up, so it allocates from the
// non-reserved internal pool and avoids the conflict.
//
// On host builds we fall back to a plain static array — no DMA
// pool, no internal/external distinction.
#define RS25_PADDED_TAPS 16
#if defined(ESP_PLATFORM)
static int16_t *s_coeffs_pp = NULL;
#else
static int16_t  s_coeffs_pp_storage[RS25_PADDED_TAPS * RS25_INTERP]
    __attribute__((aligned(16)));
static int16_t *s_coeffs_pp = s_coeffs_pp_storage;
#endif
static int s_coeffs_pp_ready = 0;

#if defined(__riscv) && __has_include("soc/soc_caps.h")
#include "soc/soc_caps.h"
#endif

// PIE-accelerated 9-tap MAC, defined in resample_arp4.S. Pads to 16
// taps (zero-padded). Computes one output sample pair from
// pre-loaded delay lines + per-phase tap pointer. Self-gates on
// __riscv && SOC_CPU_HAS_PIE in the .S file.
#if defined(__riscv) && defined(SOC_CPU_HAS_PIE) && !defined(RS25_DISABLE_PIE_ASM)
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

    // First-time allocation of the per-phase tap singleton. On
    // ESP_PLATFORM we request MALLOC_CAP_INTERNAL (NOT _DMA) so the
    // request comes out of the post-DMA-reserve internal-SRAM pool;
    // a static .dram.bss array would fragment the pre-reserve heap
    // and trip esp_psram's 144 KB DMA-pool reservation (boot panic
    // with ESP_ERR_NO_MEM). On host the storage is a plain static
    // array already aliased to s_coeffs_pp.
#if defined(ESP_PLATFORM)
    if (s_coeffs_pp == NULL) {
        s_coeffs_pp = (int16_t *)heap_caps_aligned_alloc(
            16, RS25_PADDED_TAPS * RS25_INTERP * sizeof(int16_t),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (s_coeffs_pp == NULL) {
            // Boot will crash downstream when the resampler is hit;
            // this is fatal and the caller has no fallback.
            return;
        }
        // Log the actual address so we can verify the alloc landed
        // in internal SRAM (0x4FFxxxxx region) and not PSRAM
        // (0x48xxxxxx). A PSRAM fallback would preserve correctness
        // but eat the perf gain.
        ESP_LOGI("RS25", "s_coeffs_pp allocated at %p (size=%u B)",
                 s_coeffs_pp,
                 (unsigned)(RS25_PADDED_TAPS * RS25_INTERP * sizeof(int16_t)));
    }
#endif

    // Build per-phase contiguous layout, padded to RS25_PADDED_TAPS=16
    // so the PIE-asm inner can do exactly two vmulas covering all
    // 16 lanes (the last 7 padding zeros contribute 0). Indexed as
    // s_coeffs_pp[phase * 16 + tap]; tap [0..8] = original
    // coeffs[tap × INTERP + phase], tap [9..15] = 0.
    memset(s_coeffs_pp, 0,
           RS25_PADDED_TAPS * RS25_INTERP * sizeof(int16_t));
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

// Caller-managed-state core. Same Q15 math as the legacy entry below
// (bit-exact, same 0x7fff rounding + >>15) but the (delay, phase)
// live in caller-owned buffers. Used directly by the split-ingest
// worker pool; the legacy resample_256_to_250_t wrapper delegates
// here too.
int resample_256_to_250_process_explicit(int16_t *delay_i, int16_t *delay_q,
                                          int *start_pos_io,
                                          const int16_t *in_iq, int n_in_complex,
                                          int16_t *out_iq, int max_out)
{
    int16_t *__restrict di = delay_i;
    int16_t *__restrict dq = delay_q;
    const int16_t *__restrict coeffs_pp = s_coeffs_pp;
    int start_pos = *start_pos_io;
    int n_out = 0;
    (void)s_coeffs_pp_ready;

    for (int i = 0; i < n_in_complex; i++) {
        // Shift delay line: 9 elements, newest at index 0.
        // delay[9..15] are tail-pad slots for PIE 128-bit loads.
        di[8] = di[7]; di[7] = di[6]; di[6] = di[5]; di[5] = di[4];
        di[4] = di[3]; di[3] = di[2]; di[2] = di[1]; di[1] = di[0];
        di[0] = in_iq[2 * i + 0];
        dq[8] = dq[7]; dq[7] = dq[6]; dq[6] = dq[5]; dq[5] = dq[4];
        dq[4] = dq[3]; dq[3] = dq[2]; dq[2] = dq[1]; dq[1] = dq[0];
        dq[0] = in_iq[2 * i + 1];

        if (start_pos < RS25_INTERP) {
            if (n_out < max_out) {
                const int16_t *__restrict pc =
                    &coeffs_pp[start_pos * RS25_PADDED_TAPS];
#if RS25_USE_PIE_ASM
                resample_125_128_mac_arp4(pc, di, dq,
                                           &out_iq[2 * n_out + 0],
                                           &out_iq[2 * n_out + 1]);
#else
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
            }
            start_pos += RS25_DECIM;
        }
        start_pos -= RS25_INTERP;
    }

    *start_pos_io = start_pos;
    return n_out;
}

// State-advance only: walks delay + phase counter through the input
// range without emitting outputs. Used by split-ingest's Worker B to
// pre-position its (delay, start_pos) to the chunk midpoint while
// Worker A processes the first half concurrently. ~30 ns/input
// scalar — the loop body is just two delay-shifts + a phase update.
void resample_256_to_250_advance(int16_t *delay_i, int16_t *delay_q,
                                  int *start_pos_io,
                                  const int16_t *in_iq, int n_in_complex)
{
    int16_t *__restrict di = delay_i;
    int16_t *__restrict dq = delay_q;
    int start_pos = *start_pos_io;

    for (int i = 0; i < n_in_complex; i++) {
        di[8] = di[7]; di[7] = di[6]; di[6] = di[5]; di[5] = di[4];
        di[4] = di[3]; di[3] = di[2]; di[2] = di[1]; di[1] = di[0];
        di[0] = in_iq[2 * i + 0];
        dq[8] = dq[7]; dq[7] = dq[6]; dq[6] = dq[5]; dq[5] = dq[4];
        dq[4] = dq[3]; dq[3] = dq[2]; dq[2] = dq[1]; dq[1] = dq[0];
        dq[0] = in_iq[2 * i + 1];

        if (start_pos < RS25_INTERP) {
            start_pos += RS25_DECIM;
        }
        start_pos -= RS25_INTERP;
    }

    *start_pos_io = start_pos;
}

int resample_256_to_250_process(resample_256_to_250_t *r,
                                 const int16_t *in_iq, int n_in_complex,
                                 int16_t *out_iq)
{
    // Legacy entry point: delegates to the caller-managed-state core.
    // n_in_complex is an upper bound on emitted outputs (the 125/128
    // ratio guarantees out_count <= in_count), so max_out = n_in.
    return resample_256_to_250_process_explicit(r->delay_i, r->delay_q,
                                                 &r->start_pos,
                                                 in_iq, n_in_complex,
                                                 out_iq, n_in_complex);
}
