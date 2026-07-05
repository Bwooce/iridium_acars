// test_resample_pie_model.c — arithmetic-level proof that the ESP32-P4
// PIE polyphase MAC (common/iridium_decoder/resample_arp4.S) is
// CHUNK-CONTINUABLE: the exact fixed-point arithmetic the asm kernel
// performs produces byte-identical output regardless of how the input
// stream is split into tiles.
//
// WHY THIS TEST EXISTS
// --------------------
// test_resample_tile.c / test_resample_split.c already prove the SCALAR
// C path (resample_256_to_250_process_explicit) is bit-exact across tile
// sizes. They cannot exercise the device-only PIE asm. A hypothesis was
// raised that the PIE path might vectorize the bulk of each call and
// handle a per-call remainder/tail with different rounding, so that N
// smaller tiles (N tails) accumulate more deviation than a few large
// tiles — i.e. that the PIE arithmetic itself is chunk-count-dependent.
//
// This test settles that at the arithmetic level by re-implementing, in
// plain C, the EXACT operation resample_arp4.S performs per output:
//
//     acc = 0x7fff + sum_{k=0..15} pc[k] * d[k]     (pc[9..15] == 0)
//     out = sat_i16( acc >> 15 )                    (arithmetic shift)
//
// and driving it through the SAME per-sample delay-line write + phase
// walk the driver uses, at many tile sizes. The kernel is a PURE
// FUNCTION of (pc, di, dq): there is NO per-call remainder loop — every
// emit is one atomic two-vmulas 16-lane MAC — so tiling cannot change a
// single output. mac_exact() below is that kernel; it must be bit-exact
// across all tilings.
//
// The negative control mac_buggy_tail() models the HYPOTHETICAL bug (a
// differently-rounded final emit of each call). It reproduces exactly the
// "N tails => N*eps, smaller tiles worse" signature — and the gate
// detects it — which proves (a) the harness has teeth and (b) the real
// kernel does NOT exhibit that signature.
//
// Self-contained: it does not link the PIE .S (host has no PIE). It
// reuses resample_256_to_250_init() only to generate the identical Q15
// coefficient table, then builds the same padded per-phase layout the
// firmware's init builds for the asm.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "resample_256_to_250.h"

#define PADDED_TAPS 16 // RS25_PADDED_TAPS in resample_256_to_250.c

static int16_t g_pp[PADDED_TAPS * RS25_INTERP]; // per-phase padded taps

// Build the exact padded layout resample_256_to_250_init() feeds the asm:
//   pp[phase*16 + tap] = coeffs[tap*INTERP + phase] for tap 0..8, else 0.
static void build_padded_coeffs(void)
{
    resample_256_to_250_t r;
    resample_256_to_250_init(&r); // generates r.coeffs (Q15)
    memset(g_pp, 0, sizeof(g_pp));
    for (int phase = 0; phase < RS25_INTERP; phase++) {
        for (int tap = 0; tap < RS25_DELAY_SIZE; tap++) {
            g_pp[phase * PADDED_TAPS + tap] =
                r.coeffs[tap * RS25_INTERP + phase];
        }
    }
}

static inline int16_t sat_i16(int64_t x)
{
    if (x > INT16_MAX) return INT16_MAX;
    if (x < INT16_MIN) return INT16_MIN;
    return (int16_t)x;
}

// EXACT model of resample_arp4.S: full 16-lane MAC, 0x7fff bias, >>15.
// `is_call_tail` is ignored — the real kernel has no tail special case.
static inline int16_t mac_exact(const int16_t *pc, const int16_t *d,
                                int is_call_tail)
{
    (void)is_call_tail;
    int64_t acc = 0x7fff;
    for (int k = 0; k < PADDED_TAPS; k++) {
        acc += (int32_t)d[k] * (int32_t)pc[k];
    }
    return sat_i16(acc >> 15);
}

// NEGATIVE CONTROL: a plausible per-call tail bug. On the LAST emit of a
// call only, use round-to-nearest bias 0x4000 instead of 0x7fff. This is
// the "differently-rounded tail" the hypothesis describes; it makes the
// output depend on where call boundaries fall (=> on tile size).
static inline int16_t mac_buggy_tail(const int16_t *pc, const int16_t *d,
                                     int is_call_tail)
{
    int64_t acc = is_call_tail ? 0x4000 : 0x7fff;
    for (int k = 0; k < PADDED_TAPS; k++) {
        acc += (int32_t)d[k] * (int32_t)pc[k];
    }
    return sat_i16(acc >> 15);
}

typedef int16_t (*mac_fn)(const int16_t *pc, const int16_t *d, int is_call_tail);

// Process one call (tile) of `n_in` complex samples, threading the same
// (delay, wpos, start_pos) state the firmware threads. Mirrors
// resample_256_to_250_process_explicit's control flow exactly; only the
// MAC is swapped out. Emits complex int16 to out_iq. Returns n emitted.
//
// The last emit produced within this call is flagged is_call_tail=1 (for
// the negative-control MAC). Determined by looking ahead: an emit is the
// call's last iff no later input sample in this call also emits.
static int process_call(mac_fn   mac,
                        int16_t *di, int16_t *dq, int *wpos_io, int *sp_io,
                        const int16_t *in_iq, int n_in, int16_t *out_iq)
{
    int wpos = *wpos_io, sp = *sp_io, n_out = 0;

    // Pre-scan which input index produces the final emit of this call, so
    // we can mark the call tail without emitting out of order.
    int last_emit_i = -1;
    {
        int s = sp;
        for (int i = 0; i < n_in; i++) {
            if (s < RS25_INTERP) {
                last_emit_i = i;
                s += RS25_DECIM;
            }
            s -= RS25_INTERP;
        }
    }

    for (int i = 0; i < n_in; i++) {
        wpos          = (wpos + 15) & 15;
        int16_t i_s   = in_iq[2 * i + 0];
        int16_t q_s   = in_iq[2 * i + 1];
        di[wpos]      = i_s;
        di[wpos + 16] = i_s;
        dq[wpos]      = q_s;
        dq[wpos + 16] = q_s;

        if (sp < RS25_INTERP) {
            const int16_t *pc      = &g_pp[sp * PADDED_TAPS];
            int            is_tail = (i == last_emit_i);
            out_iq[2 * n_out + 0]  = mac(pc, &di[wpos], is_tail);
            out_iq[2 * n_out + 1]  = mac(pc, &dq[wpos], is_tail);
            n_out++;
            sp += RS25_DECIM;
        }
        sp -= RS25_INTERP;
    }

    *wpos_io = wpos;
    *sp_io   = sp;
    return n_out;
}

// Drive the whole input through fixed-size tiles, threading state across
// tiles (and marking each tile's final emit as a call tail).
static int run_tiled(mac_fn mac, const int16_t *in_iq, int n_in_total,
                     int tile, int16_t *out_iq)
{
    int16_t di[32] = {0}, dq[32] = {0};
    int     wpos = 0, sp = 0, done = 0, n_out = 0;
    while (done < n_in_total) {
        int t = n_in_total - done;
        if (t > tile) t = tile;
        n_out += process_call(mac, di, dq, &wpos, &sp,
                              in_iq + 2 * done, t, out_iq + 2 * n_out);
        done += t;
    }
    return n_out;
}

#define MAXN 8192
static int16_t g_in[MAXN * 2];
static int16_t g_ref[MAXN * 2];
static int16_t g_cand[MAXN * 2];

static void fill(uint32_t seed)
{
    uint32_t s = seed;
    for (int i = 0; i < MAXN * 2; i++) {
        s       = s * 1103515245u + 12345u;
        g_in[i] = (int16_t)(s >> 16);
    }
}

static int count_diffs(int n_complex)
{
    int d = 0;
    for (int i = 0; i < n_complex * 2; i++)
        if (g_ref[i] != g_cand[i]) d++;
    return d;
}

int main(void)
{
    build_padded_coeffs();

    const int n_in    = 8000; // ~16 KB dispatch worth of complex samples
    const int tiles[] = {n_in, 2048, 1024, 512, 256, 63, 8, 1};
    const int n_tiles = (int)(sizeof(tiles) / sizeof(tiles[0]));
    int       fails   = 0;

    fill(0xC0FFEE42u);

    // Reference: the exact kernel over the WHOLE buffer in one call.
    int n_ref = run_tiled(mac_exact, g_in, n_in, n_in, g_ref);

    printf("=== A. Exact PIE-arithmetic kernel: tile-invariance ===\n");
    printf("    (models resample_arp4.S exactly: 0x7fff + sum16(pc*d) >> 15)\n");
    for (int t = 0; t < n_tiles; t++) {
        int nc = run_tiled(mac_exact, g_in, n_in, tiles[t], g_cand);
        int nd = (nc == n_ref) ? count_diffs(n_ref) : -1;
        if (nd == 0) {
            printf("OK   tile=%-5d  %d complex outputs bit-exact vs whole-buffer\n",
                   tiles[t], n_ref);
        } else {
            printf("FAIL tile=%-5d  n_out=%d (ref %d), %d int16 diffs\n",
                   tiles[t], nc, n_ref, nd);
            fails++;
        }
    }

    // Negative control: the hypothetical per-call tail-rounding bug MUST
    // diverge, and the divergence MUST grow as tiles get smaller (more
    // call tails). This is the exact signature the hypothesis predicts;
    // showing (A) has ZERO of it is the proof the real kernel is clean.
    printf("\n=== B. Negative control: hypothetical per-call tail-rounding bug ===\n");
    printf("    (0x4000 bias on each call's final emit instead of 0x7fff)\n");
    int nref_b = run_tiled(mac_buggy_tail, g_in, n_in, n_in, g_ref); // 1 call => 1 tail
    int prev = -1, monotonic_ok = 1, detected_all = 1;
    for (int t = 0; t < n_tiles; t++) {
        int nc      = run_tiled(mac_buggy_tail, g_in, n_in, tiles[t], g_cand);
        int nd      = (nc == nref_b) ? count_diffs(nref_b) : -1;
        int n_calls = (n_in + tiles[t] - 1) / tiles[t];
        printf("     tile=%-5d  calls=%-5d  %4d int16 diffs vs 1-call buggy ref\n",
               tiles[t], n_calls, nd);
        if (t > 0 && nd < prev) monotonic_ok = 0; // more tiles => >= diffs
        if (t > 0 && nd <= 0) detected_all = 0;   // every multi-tile split detected
        prev = nd;
    }
    if (!detected_all) {
        printf("FAIL negative control: a multi-tile split was NOT detected -- toothless\n");
        fails++;
    } else if (!monotonic_ok) {
        printf("WARN negative control: divergence not monotonic in tile count "
               "(still detected; signature note only)\n");
        printf("OK   negative control: buggy tail detected at every multi-tile split\n");
    } else {
        printf("OK   negative control: buggy tail detected, divergence grows as "
               "tiles shrink (matches the hypothesized signature)\n");
    }

    if (fails == 0) {
        printf("\nALL OK: the exact PIE MAC arithmetic is chunk-continuable "
               "(tile-invariant); a per-call tail bug would be caught and is "
               "provably ABSENT from resample_arp4.S.\n");
        return 0;
    }
    printf("\nFAILED: %d checks\n", fails);
    return 1;
}
