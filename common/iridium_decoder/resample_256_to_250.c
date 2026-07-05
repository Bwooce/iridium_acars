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
#include "esp_memory_utils.h" // esp_ptr_in_dram — s_coeffs_pp placement guard
#define RS25_HOT IRAM_ATTR
#else
#define RS25_HOT
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
static int16_t s_coeffs_pp_storage[RS25_PADDED_TAPS * RS25_INTERP]
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
                                      int16_t       *out_i,
                                      int16_t       *out_q);
// Hoisted out of the per-emit MAC kernel — caller invokes once per
// process_explicit call. See comment in resample_arp4.S.
extern void resample_125_128_enable_pie_cfg(void);
#define RS25_USE_PIE_ASM 1
#else
#define RS25_USE_PIE_ASM 0
#endif

// Q15 saturation helper: clamp int32 to int16 range.
static inline int16_t q15_saturate(int32_t x)
{
    if (x > INT16_MAX) return INT16_MAX;
    if (x < INT16_MIN) return INT16_MIN;
    return (int16_t)x;
}

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
    const int    INTERP = RS25_INTERP;
    const int    DSIZE  = RS25_DELAY_SIZE;
    const int    NPROTO = DSIZE * INTERP; // 1125
    const double PI     = 3.14159265358979323846;
    const double beta   = 8.0;
    // Filter cutoff in cycles/sample at the virtual interp×fs rate.
    // We pick 0.4 × (1/interp) so the passband covers ±0.4 × fs_in/2
    // = ±0.4 × 1.28 MHz = ±512 kHz at 2.56 MSPS input. Way more than
    // we need for Iridium (channel grid spans ±400 kHz around LO).
    const double fc_norm = 0.4 / (double)INTERP;
    const double inv_i0  = 1.0 / bessel_i0(beta);
    const int    center  = NPROTO / 2;

    // 1125 doubles = 9000 bytes — too large to live on this task's stack
    // (class_driver_task is 4 KB and would overflow before bessel_i0
    // returns). Allocate on the heap for the design pass only; freed
    // before this function returns.
    double *w = (double *)malloc(sizeof(double) * RS25_DELAY_SIZE * RS25_INTERP);
    if (!w) return;
    double sum_phase[RS25_INTERP];
    for (int p = 0; p < INTERP; p++)
        sum_phase[p] = 0.0;

    for (int k = 0; k < NPROTO; k++) {
        double t    = (double)(k - center);
        double sinc = (t == 0.0)
                          ? 2.0 * fc_norm
                          : sin(2.0 * PI * fc_norm * t) / (PI * t);
        double u    = 2.0 * (double)k / (double)(NPROTO - 1) - 1.0;
        double arg  = beta * sqrt(1.0 - u * u);
        double kw   = bessel_i0(arg) * inv_i0;
        w[k]        = sinc * kw;
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
        if (v > (double)INT16_MAX) v = (double)INT16_MAX;
        if (v < -(double)INT16_MAX) v = -(double)INT16_MAX;
        coeffs[k] = (int16_t)lrint(v);
    }

    free(w);
}

void resample_256_to_250_alloc_coeffs(void)
{
    // Allocates s_coeffs_pp ONLY (no per-instance state). Provided
    // as a separate entry point so callers can ensure the polyphase
    // coefficient table lands at a deterministic internal-SRAM
    // address by calling this BEFORE any other heap-touching init.
    //
    // Why: a latent bug in the PIE asm path requires s_coeffs_pp at
    // exactly 0x4ff7e300 on P4 — any other heap-shift breaks decode
    // (matched 61 → 44). See memory note
    // project_heap_position_decode_bug.md. By calling this first,
    // the TLSF allocator places s_coeffs_pp at the highest free block
    // (which currently maps to the magic address).
#if defined(ESP_PLATFORM)
    if (s_coeffs_pp == NULL) {
        s_coeffs_pp = (int16_t *)heap_caps_aligned_alloc(
            16, RS25_PADDED_TAPS * RS25_INTERP * sizeof(int16_t),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_coeffs_pp) {
            ESP_LOGE("RS25", "s_coeffs_pp INTERNAL alloc FAILED");
            return;
        }
        // Hard guard: the PIE vector unit garbles data on non-DRAM
        // (RTCRAM/TCM). Refuse a non-DRAM placement rather than
        // mis-decode silently -- mirrors uw_correlator's
        // pie_fft_fc32_init / uw_correlator_prealloc_fir guards.
        if (!esp_ptr_in_dram(s_coeffs_pp)) {
            ESP_LOGE("RS25", "s_coeffs_pp landed OUTSIDE DRAM at %p -> PIE asm "
                             "would MIS-DECODE. Call "
                             "resample_256_to_250_alloc_coeffs() earlier in boot.",
                     s_coeffs_pp);
            heap_caps_free(s_coeffs_pp);
            s_coeffs_pp = NULL; // downstream: resample_256_to_250_init treats
                                // this as fatal (PIE asm would crash otherwise)
            return;
        }
        ESP_LOGI("RS25", "s_coeffs_pp allocated at %p (size=%u B) [early]",
                 s_coeffs_pp,
                 (unsigned)(RS25_PADDED_TAPS * RS25_INTERP * sizeof(int16_t)));
    }
#endif
}

void resample_256_to_250_init(resample_256_to_250_t *r)
{
    make_resample_coeffs(r->coeffs);

    // Lazy fallback: if early alloc wasn't called, do it here.
    // Production callers should call resample_256_to_250_alloc_coeffs
    // before any other internal-SRAM consumer to guarantee placement.
    resample_256_to_250_alloc_coeffs();
#if defined(ESP_PLATFORM)
    if (s_coeffs_pp == NULL) {
        return; // fatal; downstream PIE asm will crash
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
    r->wpos      = 0;
    // firmr_s16 retained for host comparison / regression coverage;
    // the production path no longer routes through it.
    firmr_s16_init(&r->fir_i, r->coeffs, r->delay_i,
                   RS25_DELAY_SIZE, RS25_INTERP, RS25_DECIM,
                   /*start_pos=*/0, /*shift=*/0);
    firmr_s16_init(&r->fir_q, r->coeffs, r->delay_q,
                   RS25_DELAY_SIZE, RS25_INTERP, RS25_DECIM,
                   /*start_pos=*/0, /*shift=*/0);
}

// Caller-managed-state core. Same Q15 math as the legacy entry below
// (bit-exact, same 0x7fff rounding + >>15) but the (delay, wpos, phase)
// live in caller-owned buffers. Used directly by the split-ingest
// worker pool; the legacy resample_256_to_250_t wrapper delegates
// here too.
//
// Layout (task #58): the delay buffer is 32 int16, but only 16 hold
// unique data — slots [0..15] are the live ring and [16..31] mirror
// the same values. Each input writes the new sample at BOTH
// delay[wpos] and delay[wpos+16], then wpos walks backwards (wraps
// at 0). The MAC reads 16 contiguous int16 starting at
// &delay[wpos]; for any wpos in [0..15] that window fits inside the
// [0..31] buffer, the first 9 entries are the true taps in
// "newest..8-old" order, and entries [9..15] are mirror data that
// gets multiplied by the per-phase coefficient zero-pad (slots
// [9..15] are 0 by construction in make_resample_coeffs +
// resample_256_to_250_init).
//
// This eliminates the per-input 9-element shift (was 18 stores per
// input — 9 each for I and Q including the new sample; now 4
// stores: newest + mirror, per channel). The MAC math is unchanged
// — the same PIE asm reads the same 16-int16 window and produces a
// bit-identical result.
// Output batch size for the optional batch_scratch path: 8 complex
// = 32 bytes = half a cache line. Big enough to amortise the cost
// of an uncached PSRAM round-trip across multiple emits; small
// enough that the scratch fits trivially on a caller's stack.
#define RS25_BATCH_COMPLEX 8

RS25_HOT int resample_256_to_250_process_explicit(int16_t *delay_i, int16_t *delay_q,
                                                  int           *wpos_io,
                                                  int           *start_pos_io,
                                                  const int16_t *in_iq, int n_in_complex,
                                                  int16_t *out_iq, int max_out,
                                                  int16_t *batch_scratch)
{
    int16_t *__restrict di              = delay_i;
    int16_t *__restrict dq              = delay_q;
    const int16_t *__restrict coeffs_pp = s_coeffs_pp;
    int wpos                            = *wpos_io;
    int start_pos                       = *start_pos_io;
    int n_out                           = 0;
    int batch_n                         = 0; // 0..RS25_BATCH_COMPLEX (only used when batch_scratch != NULL)
    (void)s_coeffs_pp_ready;

#if RS25_USE_PIE_ASM
    // Enable unaligned 128-bit PIE vld once before the loop; the
    // hot per-emit MAC kernel now assumes this is set.
    resample_125_128_enable_pie_cfg();
#endif

    for (int i = 0; i < n_in_complex; i++) {
        // Circular write with mirror copy. wpos walks 0..15
        // backwards (newest sample at delay[wpos]). The mirror at
        // delay[wpos+16] keeps a contiguous 9-tap window readable
        // at &delay[wpos] regardless of how wpos wraps.
        wpos          = (wpos + 15) & 15;
        int16_t i_s   = in_iq[2 * i + 0];
        int16_t q_s   = in_iq[2 * i + 1];
        di[wpos]      = i_s;
        di[wpos + 16] = i_s;
        dq[wpos]      = q_s;
        dq[wpos + 16] = q_s;

        if (start_pos < RS25_INTERP) {
            if (n_out < max_out) {
                const int16_t *__restrict pc =
                    &coeffs_pp[start_pos * RS25_PADDED_TAPS];
                // Pick the per-emit write destination. With
                // batch_scratch, the MAC writes go to an aligned
                // internal-SRAM scratch and we flush in 32-byte
                // bursts; without, we write straight to out_iq in
                // PSRAM as before.
                int16_t *out_i = batch_scratch
                                     ? &batch_scratch[2 * batch_n + 0]
                                     : &out_iq[2 * n_out + 0];
                int16_t *out_q = batch_scratch
                                     ? &batch_scratch[2 * batch_n + 1]
                                     : &out_iq[2 * n_out + 1];
#if RS25_USE_PIE_ASM
                resample_125_128_mac_arp4(pc, &di[wpos], &dq[wpos],
                                          out_i, out_q);
#else
                // 0x7fff (not the round-to-nearest 0x4000) is DELIBERATE:
                // it matches esp-dsp's firmr_s16 pre-shift convention and
                // the PIE asm kernel (resample_arp4.S loads the same
                // constant into xacc), keeping scalar and PIE bit-exact.
                // Cost: ~+0.5 LSB systematic bias — invisible at Q15 scale.
                int64_t acc_i = 0x7fff;
                int64_t acc_q = 0x7fff;
                for (int k = 0; k < RS25_DELAY_SIZE; k++) {
                    int32_t c = pc[k];
                    acc_i += (int32_t)di[wpos + k] * c;
                    acc_q += (int32_t)dq[wpos + k] * c;
                }
                *out_i = q15_saturate(acc_i >> 15);
                *out_q = q15_saturate(acc_q >> 15);
#endif
                n_out++;
                if (batch_scratch) {
                    batch_n++;
                    if (batch_n == RS25_BATCH_COMPLEX) {
                        // Flush full batch to PSRAM as one 32-byte memcpy.
                        memcpy(&out_iq[2 * (n_out - RS25_BATCH_COMPLEX)],
                               batch_scratch,
                               RS25_BATCH_COMPLEX * 2 * sizeof(int16_t));
                        batch_n = 0;
                    }
                }
            }
            start_pos += RS25_DECIM;
        }
        start_pos -= RS25_INTERP;
    }

    // Final partial batch flush.
    if (batch_scratch && batch_n > 0) {
        memcpy(&out_iq[2 * (n_out - batch_n)],
               batch_scratch,
               batch_n * 2 * sizeof(int16_t));
    }

    *wpos_io      = wpos;
    *start_pos_io = start_pos;
    return n_out;
}

// State-advance only: walks delay + phase counter through the input
// range without emitting outputs. Used by split-ingest's Worker B to
// pre-position its (delay, wpos, start_pos) to the chunk midpoint
// while Worker A processes the first half concurrently. Same write
// pattern as _process_explicit so the resulting state is identical.
void resample_256_to_250_advance(int16_t *delay_i, int16_t *delay_q,
                                 int           *wpos_io,
                                 int           *start_pos_io,
                                 const int16_t *in_iq, int n_in_complex)
{
    int16_t *__restrict di = delay_i;
    int16_t *__restrict dq = delay_q;
    int wpos               = *wpos_io;
    int start_pos          = *start_pos_io;

    for (int i = 0; i < n_in_complex; i++) {
        wpos          = (wpos + 15) & 15;
        int16_t i_s   = in_iq[2 * i + 0];
        int16_t q_s   = in_iq[2 * i + 1];
        di[wpos]      = i_s;
        di[wpos + 16] = i_s;
        dq[wpos]      = q_s;
        dq[wpos + 16] = q_s;

        if (start_pos < RS25_INTERP) {
            start_pos += RS25_DECIM;
        }
        start_pos -= RS25_INTERP;
    }

    *wpos_io      = wpos;
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
                                                &r->wpos,
                                                &r->start_pos,
                                                in_iq, n_in_complex,
                                                out_iq, n_in_complex,
                                                /*batch_scratch=*/NULL);
}
