// test_tagger_mag_ema_golden.c — bit-exact golden harness for the two
// ESP32-P4 PIE kernels added in common/iridium_decoder/fft_burst_tagger_arp4.S:
//
//   1. fbt_mag_sq_pass_arp4  — per-bin mag²: dst[k]=re[k]²+im[k]²
//   2. fbt_ema_step_arp4 / fbt_ema_step_prime_arp4 — the baseline EMA RMW
//      with the FBT_EMA_SLOT_CLAMP clamp.
//
// WHY THIS TEST EXISTS / WHAT IT CAN AND CANNOT PROVE
// ---------------------------------------------------
// The host has no PIE unit, so fft_burst_tagger_arp4.S compiles to NOTHING
// on host and the tagger's C path (mag_sq_pass / ema_step_inner[_prime])
// runs the scalar reference. This test therefore cannot execute the actual
// arp4 instructions. Instead — exactly like test_resample_pie_model.c — it:
//
//   (a) re-implements the SCALAR reference (byte-for-byte the C in
//       fft_burst_tagger.c), and
//   (b) re-implements a faithful C MODEL of what the arp4 kernel does at
//       the lane/accumulator level (vunzip deinterleave, qacc lane order,
//       vmin.s32 clamp, vsub/vadd int32 RMW), and
//   (c) asserts (a) == (b) over a realistic FFT-output buffer, pins the
//       result to a FROZEN golden (FNV-1a hash + hand-checkable spot
//       values), and runs negative controls that MUST be caught.
//
// This guards the *contract* the asm must satisfy: if someone edits the
// asm's lane math, they must update the model here, and the model is held
// to the frozen golden + the scalar reference. It does NOT prove the real
// silicon produces these bytes — that requires a DEVICE check (the RAW
// GOLDEN matched count must be unchanged after flashing). See the header
// comment in fft_burst_tagger_arp4.S for the device-only assumptions
// (qacc lane order, st.qacc low-word extraction, vunzip even/odd).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "fft_burst_tagger.h" // FBT_FFT_SIZE, FBT_HISTORY_SIZE

#define N            FBT_FFT_SIZE
// Mirrors FBT_EMA_SLOT_CLAMP in fft_burst_tagger.c (kept private there).
#define EMA_CLAMP    (INT32_MAX / FBT_HISTORY_SIZE)

// ---- frozen golden values (see BOOTSTRAP note in main) -------------------
#define GOLD_MAG_FNV   0x86bc079294e1be4cULL
#define GOLD_EMA_FNV   0xa5ed4d92b43d5178ULL
// -------------------------------------------------------------------------

// ===================== mag²: scalar reference ============================
// Byte-for-byte mag_sq_pass() in fft_burst_tagger.c.
static void mag_scalar(const int16_t *iq, int32_t *dst, int n)
{
    for (int k = 0; k < n; k++) {
        int32_t re = iq[2 * k + 0];
        int32_t im = iq[2 * k + 1];
        dst[k]     = re * re + im * im;
    }
}

// ===================== mag²: arp4 lane MODEL =============================
// Mirrors fbt_mag_sq_pass_arp4 exactly, one 8-bin block at a time:
//   vld q0=iq[0..7], q1=iq[8..15]; vunzip.16 -> q0=re[0..7], q1=im[0..7];
//   zero.qacc; vmulas q0,q0; vmulas q1,q1; st.qacc.{l,h}.l.128 -> dst.
// Each qacc lane holds re²+im² (< 2³¹), so the stored low 32-bit word IS
// the value. Lane k -> bin k.
static void mag_arp4_model(const int16_t *iq, int32_t *dst, int n)
{
    for (int b = 0; b + 8 <= n; b += 8) {
        // vunzip.16 of (q0:q1) = 16 interleaved int16 -> evens / odds.
        int16_t re[8], im[8];
        for (int k = 0; k < 8; k++) {
            re[k] = iq[2 * (b + k) + 0]; // even element -> lane k of q0
            im[k] = iq[2 * (b + k) + 1]; // odd  element -> lane k of q1
        }
        int64_t lane[8];
        for (int k = 0; k < 8; k++) lane[k]  = (int64_t)((int32_t)re[k] * re[k]);
        for (int k = 0; k < 8; k++) lane[k] += (int64_t)((int32_t)im[k] * im[k]);
        // st.qacc low words: lanes 0..3 then 4..7, in bin order.
        for (int k = 0; k < 8; k++) dst[b + k] = (int32_t)(uint32_t)(lane[k] & 0xffffffffu);
    }
}

// ===================== EMA: scalar reference ============================
static void ema_step_scalar(int32_t *bsum, int32_t *slot,
                            const int32_t *mag, int n)
{
    for (int k = 0; k < n; k++) {
        int32_t old = slot[k];
        int32_t cur = mag[k];
        if (cur > EMA_CLAMP) cur = EMA_CLAMP;
        bsum[k] = bsum[k] - old + cur;
        slot[k] = cur;
    }
}
static void ema_prime_scalar(int32_t *bsum, int32_t *slot,
                             const int32_t *mag, int n)
{
    for (int k = 0; k < n; k++) {
        int32_t cur = mag[k];
        if (cur > EMA_CLAMP) cur = EMA_CLAMP;
        bsum[k] += cur;
        slot[k] = cur;
    }
}

// ===================== EMA: arp4 lane MODEL =============================
// Mirrors fbt_ema_step_arp4 / _prime_arp4: 4-lane int32 vmin.s32 (clamp),
// vsub.s32 (bsum-old), vadd.s32 (+cur). Elementwise, no cross-lane mixing,
// so arithmetically identical to the scalar reference — its role is to
// document the vector op sequence and give the negative controls a target.
static void ema_step_model(int32_t *bsum, int32_t *slot,
                           const int32_t *mag, int n)
{
    for (int b = 0; b + 4 <= n; b += 4) {
        for (int k = 0; k < 4; k++) {
            int32_t m   = mag[b + k];
            int32_t cur = (m < EMA_CLAMP) ? m : EMA_CLAMP; // vmin.s32
            int32_t d   = bsum[b + k] - slot[b + k];       // vsub.s32
            bsum[b + k] = d + cur;                          // vadd.s32
            slot[b + k] = cur;                              // vst
        }
    }
}
static void ema_prime_model(int32_t *bsum, int32_t *slot,
                            const int32_t *mag, int n)
{
    for (int b = 0; b + 4 <= n; b += 4) {
        for (int k = 0; k < 4; k++) {
            int32_t m   = mag[b + k];
            int32_t cur = (m < EMA_CLAMP) ? m : EMA_CLAMP; // vmin.s32
            bsum[b + k] = bsum[b + k] + cur;               // vadd.s32
            slot[b + k] = cur;
        }
    }
}

// ============================ helpers ===================================
static uint64_t fnv1a(const void *p, size_t nbytes)
{
    const uint8_t *b = (const uint8_t *)p;
    uint64_t h = 1469598103934665603ULL;
    for (size_t i = 0; i < nbytes; i++) { h ^= b[i]; h *= 1099511628211ULL; }
    return h;
}

// Deterministic FFT-output buffer: an LCG stream reinterpreted as int16,
// so bin magnitudes cover the full range including some that exceed the
// EMA clamp (needed to exercise the clamp path).
static void fill_iq(int16_t *iq, int n_complex, uint32_t seed)
{
    uint32_t s = seed;
    for (int i = 0; i < n_complex * 2; i++) {
        s = s * 1103515245u + 12345u;
        iq[i] = (int16_t)(s >> 16);
    }
}

static int cmp_i32(const int32_t *a, const int32_t *b, int n)
{
    int d = 0;
    for (int i = 0; i < n; i++) if (a[i] != b[i]) d++;
    return d;
}

// ============================ buffers ===================================
static int16_t g_iq[N * 2];
static int32_t g_mag_ref[N];
static int32_t g_mag_mod[N];

static int32_t g_bsum_ref[N];
static int32_t g_bsum_mod[N];
static int32_t g_slot_ref[N];
static int32_t g_slot_mod[N];
static int32_t g_step_mag[N];

int main(void)
{
    int fails = 0;

    // ---------------------------------------------------------------
    // 1. mag²: scalar reference vs arp4 lane model, over a full FFT frame
    // ---------------------------------------------------------------
    fill_iq(g_iq, N, 0xBEEF1234u);

    // Planted, hand-checkable sentinels (survive the LCG fill).
    g_iq[2 * 10 + 0] = 100;  g_iq[2 * 10 + 1] = -50;   // 100²+50²   = 12500
    g_iq[2 * 11 + 0] = 0;    g_iq[2 * 11 + 1] = 0;     // 0
    g_iq[2 * 12 + 0] = 32767; g_iq[2 * 12 + 1] = 32767;// 2*32767²   = 2147352578
    g_iq[2 * 13 + 0] = -32768; g_iq[2 * 13 + 1] = 0;   // 32768²     = 1073741824

    mag_scalar(g_iq, g_mag_ref, N);
    mag_arp4_model(g_iq, g_mag_mod, N);

    printf("=== A. mag² : scalar reference vs arp4 lane model ===\n");
    int nd = cmp_i32(g_mag_ref, g_mag_mod, N);
    if (nd == 0) {
        printf("OK   %d bins bit-identical (scalar == arp4 model)\n", N);
    } else {
        printf("FAIL %d/%d bins differ between scalar and arp4 model\n", nd, N);
        fails++;
    }

    // Hand-checkable spot values (independent of the reference formula).
    struct { int bin; int32_t want; } spot[] = {
        {10, 12500}, {11, 0}, {12, 2147352578}, {13, 1073741824},
    };
    for (unsigned i = 0; i < sizeof(spot) / sizeof(spot[0]); i++) {
        if (g_mag_ref[spot[i].bin] != spot[i].want ||
            g_mag_mod[spot[i].bin] != spot[i].want) {
            printf("FAIL spot bin %d: ref=%d model=%d want=%d\n", spot[i].bin,
                   g_mag_ref[spot[i].bin], g_mag_mod[spot[i].bin], spot[i].want);
            fails++;
        }
    }

    uint64_t mag_fnv = fnv1a(g_mag_ref, sizeof(g_mag_ref));
    printf("     mag[] FNV-1a = 0x%016llx (frozen 0x%016llx)\n",
           (unsigned long long)mag_fnv, (unsigned long long)GOLD_MAG_FNV);
    if (mag_fnv != GOLD_MAG_FNV) {
        printf("FAIL mag golden hash mismatch (reference formula changed?)\n");
        fails++;
    }

    // ---------------------------------------------------------------
    // 2. EMA: scalar reference vs arp4 lane model, over a prime+step run
    //    that exercises the clamp path (some mag values exceed EMA_CLAMP).
    // ---------------------------------------------------------------
    memset(g_bsum_ref, 0, sizeof(g_bsum_ref));
    memset(g_bsum_mod, 0, sizeof(g_bsum_mod));
    memset(g_slot_ref, 0, sizeof(g_slot_ref));
    memset(g_slot_mod, 0, sizeof(g_slot_mod));

    printf("\n=== B. baseline EMA : scalar reference vs arp4 lane model ===\n");
    int ema_fail = 0;
    // 6 priming writes, then 6 primed steps. Each write uses g_mag_ref
    // scaled/offset per iteration so the clamp bites on the big-mag bins.
    for (int it = 0; it < 12; it++) {
        for (int k = 0; k < N; k++) {
            // Vary the magnitude per iteration; keep some bins over clamp.
            int64_t v = (int64_t)g_mag_ref[k] + (int64_t)it * 7919;
            if (v < 0) v = -v;
            if (v > INT32_MAX) v = INT32_MAX;
            g_step_mag[k] = (int32_t)v;
        }
        if (it < 6) {
            ema_prime_scalar(g_bsum_ref, g_slot_ref, g_step_mag, N);
            ema_prime_model(g_bsum_mod, g_slot_mod, g_step_mag, N);
        } else {
            ema_step_scalar(g_bsum_ref, g_slot_ref, g_step_mag, N);
            ema_step_model(g_bsum_mod, g_slot_mod, g_step_mag, N);
        }
        if (cmp_i32(g_bsum_ref, g_bsum_mod, N) != 0 ||
            cmp_i32(g_slot_ref, g_slot_mod, N) != 0) {
            printf("FAIL EMA diverged at iteration %d (%s)\n", it,
                   it < 6 ? "prime" : "step");
            ema_fail = 1;
            break;
        }
    }
    if (!ema_fail) {
        printf("OK   bsum[] and slot[] bit-identical across 6 prime + 6 step "
               "iterations\n");
    } else {
        fails++;
    }

    // Clamp sentinel: the big-mag bins (12,13) must be pinned at EMA_CLAMP
    // in slot[] (they exceed the clamp every iteration).
    if (g_slot_ref[12] != EMA_CLAMP || g_slot_mod[12] != EMA_CLAMP ||
        g_slot_ref[13] != EMA_CLAMP || g_slot_mod[13] != EMA_CLAMP) {
        printf("FAIL clamp sentinel: slot[12]=%d slot[13]=%d (want %d)\n",
               g_slot_ref[12], g_slot_ref[13], EMA_CLAMP);
        fails++;
    } else {
        printf("     clamp sentinel OK: slot[12]=slot[13]=%d (EMA_CLAMP)\n",
               EMA_CLAMP);
    }

    uint64_t ema_fnv = fnv1a(g_bsum_ref, sizeof(g_bsum_ref)) ^
                       (fnv1a(g_slot_ref, sizeof(g_slot_ref)) * 3);
    printf("     bsum/slot FNV-1a = 0x%016llx (frozen 0x%016llx)\n",
           (unsigned long long)ema_fnv, (unsigned long long)GOLD_EMA_FNV);
    if (ema_fnv != GOLD_EMA_FNV) {
        printf("FAIL EMA golden hash mismatch (reference formula changed?)\n");
        fails++;
    }

    // ---------------------------------------------------------------
    // 3. Negative controls — the gate MUST catch each injected bug.
    // ---------------------------------------------------------------
    printf("\n=== C. Negative controls (each must be DETECTED) ===\n");
    int teeth = 0, checks = 0;

    // C1: mag model with a within-block lane swap (bins k and k+1). A wrong
    //     qacc lane order / vunzip mapping would look like this.
    {
        int32_t bad[N];
        mag_arp4_model(g_iq, bad, N);
        for (int b = 0; b + 8 <= N; b += 8) { int32_t t = bad[b]; bad[b] = bad[b+1]; bad[b+1] = t; }
        checks++;
        if (cmp_i32(g_mag_ref, bad, N) != 0) { teeth++; printf("OK   C1 lane-swap detected\n"); }
        else { printf("FAIL C1 lane-swap NOT detected\n"); }
    }
    // C2: mag model dropping the im² term (only re²).
    {
        int32_t bad[N];
        for (int b = 0; b + 8 <= N; b += 8)
            for (int k = 0; k < 8; k++) {
                int32_t re = g_iq[2*(b+k)]; bad[b+k] = re*re;
            }
        checks++;
        if (cmp_i32(g_mag_ref, bad, N) != 0) { teeth++; printf("OK   C2 missing-im² detected\n"); }
        else { printf("FAIL C2 missing-im² NOT detected\n"); }
    }
    // C3: EMA model without the clamp (min removed). Diverges on big bins.
    {
        int32_t bs[N], sl[N];
        memset(bs, 0, sizeof(bs)); memset(sl, 0, sizeof(sl));
        for (int b = 0; b + 4 <= N; b += 4)
            for (int k = 0; k < 4; k++) { bs[b+k] += g_mag_ref[b+k]; sl[b+k] = g_mag_ref[b+k]; }
        int32_t rbs[N], rsl[N];
        memset(rbs, 0, sizeof(rbs)); memset(rsl, 0, sizeof(rsl));
        ema_prime_scalar(rbs, rsl, g_mag_ref, N);
        checks++;
        if (cmp_i32(rsl, sl, N) != 0) { teeth++; printf("OK   C3 missing-clamp detected\n"); }
        else { printf("FAIL C3 missing-clamp NOT detected\n"); }
    }
    // C4: EMA step model forgetting to subtract the old slot.
    {
        int32_t bs[N], rbs[N], rsl[N];
        for (int k = 0; k < N; k++) { bs[k] = rbs[k] = 1000 + k; rsl[k] = 5 + (k & 63); }
        // buggy: bsum += cur (no -old)
        for (int b = 0; b + 4 <= N; b += 4)
            for (int k = 0; k < 4; k++) {
                int32_t m = g_mag_ref[b+k]; int32_t cur = m < EMA_CLAMP ? m : EMA_CLAMP;
                bs[b+k] = bs[b+k] + cur;
            }
        ema_step_scalar(rbs, rsl, g_mag_ref, N);
        checks++;
        if (cmp_i32(rbs, bs, N) != 0) { teeth++; printf("OK   C4 missing-subtract detected\n"); }
        else { printf("FAIL C4 missing-subtract NOT detected\n"); }
    }
    if (teeth != checks) { printf("FAIL negative controls toothless (%d/%d)\n", teeth, checks); fails++; }

    // ---------------------------------------------------------------
    if (fails == 0) {
        printf("\nALL OK: the arp4 kernels' arithmetic contract is bit-exact "
               "vs the scalar reference and matches the frozen golden.\n"
               "NOTE: device (real PIE) bit-exactness is UNVERIFIED here — the "
               "RAW GOLDEN matched count must be unchanged after flashing.\n");
        return 0;
    }
    printf("\nFAILED: %d checks\n", fails);
    return 1;
}
