// rs_vdl2.c — Reed-Solomon RS(255,249) over GF(2^8) for VDL Mode 2.
// Syndrome + Berlekamp-Massey + Chien search + Forney; integer-only,
// no allocation, ~770 B of tables (log/antilog pair + generator poly).
//
// Parameters match dumpvdl2 (Karn librs char init):
//   init_rs_char(8 /*symsize*/, 0x187 /*gfpoly*/, 120 /*fcr*/,
//                1 /*prim*/, 6 /*nroots*/, 0 /*pad*/)
// i.e. field poly x^8+x^7+x^2+x+1, generator roots alpha^120..alpha^125.
// Cross-validated on host against an independent reference implementation
// (tests/host/test_rs_vdl2.c fixtures).

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <string.h>

#include "rs_vdl2.h"

#define GF_POLY 0x187 // x^8 + x^7 + x^2 + x + 1 (primitive over GF(2))
#define GF_FCR  120   // first consecutive generator root exponent

// Antilog table doubled (indices 0..508 reachable as log a + log b) so
// gf_mul needs no mod-255. gf_log[0] is never read: every use is guarded
// by a zero check.
static uint8_t gf_exp[510];
static uint8_t gf_log[256];

// g(x) = prod_{i=0..5} (x - alpha^(fcr+i)); gpoly[j] = coeff of x^j,
// gpoly[6] = 1 (monic). Only the encoder uses it.
static uint8_t gpoly[RS_VDL2_NROOTS + 1];

// Lazy-init guard, same release/acquire pattern as bch_decoder.c: the
// tables are deterministic and idempotent to fill, but a reader that
// observes s_inited == true must also observe fully-written tables.
static atomic_bool s_inited = false;

static void rs_init(void)
{
    if (atomic_load_explicit(&s_inited, memory_order_acquire))
        return;

    unsigned x = 1;
    for (int i = 0; i < 255; i++) {
        gf_exp[i]  = (uint8_t)x;
        gf_log[x]  = (uint8_t)i;
        x <<= 1;
        if (x & 0x100)
            x ^= GF_POLY;
    }
    for (int i = 255; i < 510; i++)
        gf_exp[i] = gf_exp[i - 255];

    // Build the generator polynomial by multiplying in one root at a time.
    uint8_t g[RS_VDL2_NROOTS + 1] = { 1 };
    for (int i = 0; i < RS_VDL2_NROOTS; i++) {
        uint8_t root = gf_exp[GF_FCR + i];
        for (int j = i + 1; j > 0; j--) {
            uint8_t hi = g[j - 1];                      // c * x term
            uint8_t lo = g[j] ? gf_exp[gf_log[g[j]] + gf_log[root]] : 0;
            g[j] = hi ^ lo;                             // c*x + c*root
        }
        g[0] = g[0] ? gf_exp[gf_log[g[0]] + gf_log[root]] : 0;
    }
    memcpy(gpoly, g, sizeof(gpoly));

    atomic_store_explicit(&s_inited, true, memory_order_release);
}

static inline uint8_t gf_mul(uint8_t a, uint8_t b)
{
    if (a == 0 || b == 0)
        return 0;
    return gf_exp[gf_log[a] + gf_log[b]];
}

void rs_vdl2_encode(uint8_t block[RS_VDL2_N])
{
    rs_init();

    // LFSR division: parity = M(x)*x^6 mod g(x). par[0] holds the
    // highest-order remainder coefficient (x^5), matching the codeword
    // layout block[249] = x^5 ... block[254] = x^0.
    uint8_t par[RS_VDL2_NROOTS] = { 0 };
    for (int i = 0; i < RS_VDL2_K; i++) {
        uint8_t fb = block[i] ^ par[0];
        for (int j = 0; j < RS_VDL2_NROOTS - 1; j++)
            par[j] = par[j + 1] ^ gf_mul(fb, gpoly[RS_VDL2_NROOTS - 1 - j]);
        par[RS_VDL2_NROOTS - 1] = gf_mul(fb, gpoly[0]);
    }
    memcpy(&block[RS_VDL2_K], par, RS_VDL2_NROOTS);
}

// syn[i] = c(alpha^(fcr+i)) by Horner from block[0] (highest power).
// Returns true if any syndrome is nonzero.
static bool rs_syndromes(const uint8_t block[RS_VDL2_N],
                         uint8_t syn[RS_VDL2_NROOTS])
{
    bool nonzero = false;
    for (int i = 0; i < RS_VDL2_NROOTS; i++) {
        int log_root = GF_FCR + i;
        while (log_root >= 255)
            log_root -= 255;
        uint8_t s = 0;
        for (int j = 0; j < RS_VDL2_N; j++) {
            if (s)
                s = gf_exp[gf_log[s] + log_root];
            s ^= block[j];
        }
        syn[i] = s;
        nonzero |= (s != 0);
    }
    return nonzero;
}

// Berlekamp-Massey over a length-ns syndrome sequence. Returns the degree
// L of the minimal locator (== number of unknown-position errors) and
// writes the locator into lambda[0..L] with lambda[0] == 1. ns <= NROOTS.
static int berlekamp_massey(const uint8_t *syn, int ns,
                            uint8_t lambda[RS_VDL2_NROOTS + 1])
{
    uint8_t prev[RS_VDL2_NROOTS + 1] = { 1 }; // last length-change B(x)
    memset(lambda, 0, RS_VDL2_NROOTS + 1);
    lambda[0] = 1;
    int L = 0;      // current register length
    int m = 1;      // steps since last length change
    uint8_t b = 1;  // discrepancy at last length change (nonzero)

    for (int n = 0; n < ns; n++) {
        uint8_t d = syn[n];
        for (int i = 1; i <= L; i++)
            d ^= gf_mul(lambda[i], syn[n - i]);

        if (d == 0) {
            m++;
            continue;
        }

        uint8_t coef = gf_exp[gf_log[d] + 255 - gf_log[b]]; // d / b
        if (2 * L <= n) {
            uint8_t tmp[RS_VDL2_NROOTS + 1];
            memcpy(tmp, lambda, sizeof(tmp));
            for (int i = 0; i + m <= RS_VDL2_NROOTS; i++)
                lambda[i + m] ^= gf_mul(coef, prev[i]);
            L = n + 1 - L;
            memcpy(prev, tmp, sizeof(prev));
            b = d;
            m = 1;
        } else {
            for (int i = 0; i + m <= RS_VDL2_NROOTS; i++)
                lambda[i + m] ^= gf_mul(coef, prev[i]);
            m++;
        }
    }
    return L;
}

// Given the full errata locator psi(x) of degree dpsi (product of the
// error locator and the erasure locator; == plain error locator when there
// are no erasures), locate every errata symbol via a Chien search and
// correct its magnitude in place via Forney. Returns 0 on success (block
// is a valid codeword; *n_corrected = symbols actually changed) or -1 if
// the locator is inconsistent (doesn't fully split, derivative vanishes,
// or the corrected block still fails the syndrome check). syn holds the
// original syndromes; it is used for Forney and clobbered by the recheck.
static int apply_errata(uint8_t block[RS_VDL2_N], uint8_t syn[RS_VDL2_NROOTS],
                        const uint8_t psi[RS_VDL2_NROOTS + 1], int dpsi,
                        int *n_corrected)
{
    // Chien search: a root at x = alpha^q means locator X = alpha^p with
    // p = (255 - q) % 255; position power p maps to block index 254 - p.
    int     nroots = 0;
    uint8_t root_q[RS_VDL2_NROOTS];  // q with psi(alpha^q) == 0
    uint8_t err_pos[RS_VDL2_NROOTS]; // corresponding block indices
    for (int q = 0; q < 255; q++) {
        uint8_t v = psi[0];
        for (int i = 1; i <= dpsi; i++) {
            if (psi[i])
                v ^= gf_exp[(gf_log[psi[i]] + i * q) % 255];
        }
        if (v == 0) {
            if (nroots == RS_VDL2_NROOTS)
                return -1; // more roots than a degree-<=6 poly can have
            int p = (255 - q) % 255;
            root_q[nroots]  = (uint8_t)q;
            err_pos[nroots] = (uint8_t)(RS_VDL2_N - 1 - p);
            nroots++;
        }
    }
    if (nroots != dpsi)
        return -1; // locator doesn't split over GF(256): uncorrectable

    // Forney: omega(x) = S(x) * psi(x) mod x^6, then
    // e_k = X_k^(1-fcr) * omega(X_k^-1) / psi'(X_k^-1).
    uint8_t omega[RS_VDL2_NROOTS] = { 0 };
    for (int i = 0; i < RS_VDL2_NROOTS; i++) {
        uint8_t acc = 0;
        for (int j = 0; j <= i && j <= dpsi; j++)
            acc ^= gf_mul(psi[j], syn[i - j]);
        omega[i] = acc;
    }

    int changed = 0;
    for (int k = 0; k < nroots; k++) {
        int q = root_q[k];          // X_k^-1 = alpha^q
        int p = (255 - q) % 255;    // X_k    = alpha^p

        uint8_t num = 0;            // omega(X_k^-1)
        for (int i = 0; i < RS_VDL2_NROOTS; i++) {
            if (omega[i])
                num ^= gf_exp[(gf_log[omega[i]] + i * q) % 255];
        }
        // psi'(X_k^-1): odd-power terms only (char-2 formal derivative)
        uint8_t den = 0;
        for (int i = 1; i <= dpsi; i += 2) {
            if (psi[i])
                den ^= gf_exp[(gf_log[psi[i]] + (i - 1) * q) % 255];
        }
        if (den == 0)
            return -1; // derivative vanished at a root: inconsistent

        // num == 0 is legal here: an erased symbol whose true value equals
        // the zero fill has magnitude 0. Apply it (a no-op) and let the
        // final syndrome recheck arbitrate any genuine inconsistency.
        if (num != 0) {
            // e = num/den * X^(1-fcr); exponent kept non-negative.
            int e_log = gf_log[num] + 255 - gf_log[den]
                      + (p * (255 + 1 - GF_FCR)) % 255;
            block[err_pos[k]] ^= gf_exp[e_log % 255];
            changed++;
        }
    }

    // The corrected block must be a valid codeword. This is what catches a
    // miscorrection (locator split but to the wrong positions/magnitudes).
    if (rs_syndromes(block, syn))
        return -1;

    if (n_corrected)
        *n_corrected = changed;
    return 0;
}

int rs_vdl2_decode(uint8_t block[RS_VDL2_N], int *n_corrected)
{
    return rs_vdl2_decode_erasures(block, NULL, 0, n_corrected);
}

int rs_vdl2_decode_erasures(uint8_t block[RS_VDL2_N],
                            const uint8_t *erasure_pos, int n_erasures,
                            int *n_corrected)
{
    rs_init();

    if (n_erasures < 0 || n_erasures > RS_VDL2_NROOTS)
        return -1; // f > s: no error-correcting capacity possible

    uint8_t syn[RS_VDL2_NROOTS];
    if (!rs_syndromes(block, syn)) {
        // Already a valid codeword (erasure positions, if any, happened to
        // hold their correct values — e.g. correct parity, or genuine 0).
        if (n_corrected)
            *n_corrected = 0;
        return 0;
    }

    int f = n_erasures;

    // Erasure locator gamma(x) = prod (1 - X_j x), X_j = alpha^(254 - pos).
    uint8_t gamma[RS_VDL2_NROOTS + 1] = { 1 };
    for (int e = 0; e < f; e++) {
        int pos = erasure_pos[e];
        if (pos < 0 || pos >= RS_VDL2_N)
            return -1;
        uint8_t x = gf_exp[(RS_VDL2_N - 1 - pos)]; // alpha^(254 - pos)
        for (int i = e + 1; i > 0; i--)
            gamma[i] ^= gf_mul(x, gamma[i - 1]);
    }

    // Modified (Forney) syndromes: T_k = coeff at x^(f+k) of gamma(x)*S(x),
    // k = 0 .. (s-f-1). Berlekamp-Massey on T yields the error locator for
    // the unknown-position errors only.
    uint8_t u[RS_VDL2_NROOTS] = { 0 }; // gamma*S mod x^s, u[i] = coeff x^i
    for (int i = 0; i < RS_VDL2_NROOTS; i++) {
        uint8_t acc = 0;
        for (int a = 0; a <= i && a <= f; a++)
            acc ^= gf_mul(gamma[a], syn[i - a]);
        u[i] = acc;
    }

    int ns = RS_VDL2_NROOTS - f; // number of modified syndromes
    uint8_t tsyn[RS_VDL2_NROOTS];
    for (int k = 0; k < ns; k++)
        tsyn[k] = u[f + k];

    uint8_t lambda[RS_VDL2_NROOTS + 1];
    int L = berlekamp_massey(tsyn, ns, lambda);

    // Correction budget: 2*e + f <= s.
    if (2 * L + f > RS_VDL2_NROOTS)
        return -1;

    // Errata locator psi(x) = lambda(x) * gamma(x), degree L + f.
    uint8_t psi[RS_VDL2_NROOTS + 1] = { 0 };
    for (int a = 0; a <= L; a++) {
        if (!lambda[a])
            continue;
        for (int b = 0; b <= f; b++)
            if (gamma[b])
                psi[a + b] ^= gf_mul(lambda[a], gamma[b]);
    }
    int dpsi = L + f;

    return apply_errata(block, syn, psi, dpsi, n_corrected);
}

int rs_vdl2_decode_shortened(uint8_t block[RS_VDL2_N], int data_len,
                             int *n_corrected)
{
    // dumpvdl2 FEC-octet schedule (get_fec_octetcount).
    int fec;
    if (data_len < 3)
        fec = 0;
    else if (data_len < 31)
        fec = 2;
    else if (data_len < 68)
        fec = 4;
    else
        fec = RS_VDL2_NROOTS;

    if (fec == 0) {
        // Uncoded block: nothing to verify here; the frame CRC is the only
        // integrity check. Report clean pass-through.
        if (n_corrected)
            *n_corrected = 0;
        return 0;
    }

    // Untransmitted parity positions [RS_K + fec .. 254] are the erasures.
    int f = RS_VDL2_NROOTS - fec;
    uint8_t erasures[RS_VDL2_NROOTS];
    for (int i = 0; i < f; i++)
        erasures[i] = (uint8_t)(RS_VDL2_K + fec + i);

    return rs_vdl2_decode_erasures(block, erasures, f, n_corrected);
}
