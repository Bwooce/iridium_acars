// BCH polynomial-division helpers used by the Iridium frame classifier.
// Direct C port of iridium-toolkit/bch.py {nndivide, nrepair1}.
//
// Repair strategy (2026-07-17): iridium_bch_repair1/2 are SYNDROME-TABLE
// lookups — one polynomial division to get the syndrome, then an O(1)
// table probe mapping syndrome -> error position(s). The original
// brute-force search (flip each bit / each pair and re-divide, O(n^2)
// divisions) is retained verbatim as iridium_bch_repair1_ref /
// iridium_bch_repair2_ref: it is the behavioural REFERENCE, and
// tests/host/test_bch_syndrome.c proves the table path bit-exact against
// it over EVERY 0/1/2-error pattern (plus random >=3-error patterns,
// where identical MIScorrection is asserted) for every (poly, n_bits)
// the decoders call.
//
// Why bit-exactness holds by construction: iridium_bch_ndivide computes
// the GF(2) polynomial remainder of the codeword value, which is LINEAR
// under XOR: syn(cw ^ e_i ^ e_j) = syn(cw) ^ syn(e_i) ^ syn(e_j). So
// "flip bit i (and j) then re-divide == 0" is exactly
// "syn1[i] (^ syn1[j]) == syndrome". The table is built singles-first in
// ascending index order, then pairs in ascending (i, j) lexicographic
// order, never overwriting an occupied slot — which reproduces the
// brute-force's exact preference order (lowest single, then lowest pair)
// even for polynomials whose <=2-error syndromes are NOT unique.

#include "iridium_bch.h"

#ifdef ESP_PLATFORM
// EXT_RAM_BSS_ATTR: the syndrome arena (~9 KB .bss) must live in PSRAM,
// not internal RAM — internal .bss starves the DMA-INT/USB-URB budget
// (see feedback_dma_int_budget_audit). Decoder path is not DMA-fed, so
// PSRAM latency is fine.
#include "esp_attr.h"
#else
#define EXT_RAM_BSS_ATTR
#endif

// Convert a 0/1-per-byte bit array into a uint32_t (MSB-first, the upstream
// convention from int(bits, 2)). Limited to n_bits ≤ 32 — the polynomials
// we operate on top out at 12 bits, the largest payload is the 31-bit BCH
// codeword for poly=1207/1897/3545.
static uint32_t bits_to_u32(const uint8_t *bits, size_t n_bits)
{
    // T32: guard the documented n_bits <= 32 contract instead of silently
    // truncating. All current callers pass n_bits <= 31 (the largest BCH
    // codeword we handle -- see iridium_bch.h); clamp defensively so a
    // future caller passing something larger gets a bounded (if wrong)
    // result instead of quietly losing the high bits with no signal.
    // This does not change the result for any valid (n_bits <= 32) input.
    if (n_bits > 32) n_bits = 32;
    uint32_t v = 0;
    for (size_t i = 0; i < n_bits; i++) {
        v = (v << 1) | (bits[i] & 1);
    }
    return v;
}

// Number of bits in `x` (highest bit position + 1, with bit_length(0) = 0
// matching Python int.bit_length).
static int u32_bit_length(uint32_t x)
{
    int n = 0;
    while (x) {
        n++;
        x >>= 1;
    }
    return n;
}

// Core of iridium_bch_ndivide on an integer-packed codeword: GF(2)
// polynomial remainder of `num` mod `poly`. Factored out (identical
// arithmetic to the pre-refactor loop) so the syndrome-table builder can
// compute single-bit syndromes with the EXACT same divider the runtime
// syndrome comes from.
static uint32_t poly_mod_u32(uint32_t num, uint32_t poly)
{
    // An all-zero codeword divides to remainder 0 -- i.e. it always
    // reports "clean" -- same as upstream iridium-toolkit/bch.py:nndivide
    // (0 // poly == 0). Intentional and load-bearing: BC/RA header
    // false-positive-rate stats are measured against this exact
    // behavior. Do NOT special-case it; a change here would shift those
    // stats without a matching upstream change. (T32 note, pre-refactor.)
    if (num == 0) return 0;

    int      num_len  = u32_bit_length(num);
    int      poly_len = u32_bit_length(poly);
    int      shift    = num_len - poly_len;
    uint32_t pow      = (uint32_t)1 << (num_len - 1);

    while (shift >= 0) {
        if (num >= pow) {
            num ^= (poly << shift);
        }
        pow >>= 1;
        shift--;
    }
    return num;
}

uint32_t iridium_bch_ndivide(uint32_t poly, const uint8_t *bits, size_t n_bits)
{
    if (!bits || n_bits == 0) return 0;
    return poly_mod_u32(bits_to_u32(bits, n_bits), poly);
}

// ---------------------------------------------------------------------------
// Brute-force reference implementations (the pre-2026-07-17 production
// code, byte-for-byte). Kept as the ground truth the syndrome tables are
// exhaustively diffed against (test_bch_syndrome), and as the runtime
// fallback for any (poly, n_bits) the table registry can't host.
// ---------------------------------------------------------------------------

int iridium_bch_repair1_ref(uint32_t poly, uint8_t *bits, size_t n_bits)
{
    if (!bits || n_bits == 0) return -1;

    if (iridium_bch_ndivide(poly, bits, n_bits) == 0) {
        return 0; // already clean
    }

    // Try flipping each bit in turn.
    for (size_t i = 0; i < n_bits; i++) {
        bits[i] ^= 1;
        if (iridium_bch_ndivide(poly, bits, n_bits) == 0) {
            return 1; // single-bit error corrected (flip remains)
        }
        bits[i] ^= 1; // revert and try next
    }
    return -1;
}

int iridium_bch_repair2_ref(uint32_t poly, uint8_t *bits, size_t n_bits)
{
    if (!bits || n_bits == 0) return -1;

    if (iridium_bch_ndivide(poly, bits, n_bits) == 0) {
        return 0; // already clean
    }

    // Single-bit pass first.
    for (size_t i = 0; i < n_bits; i++) {
        bits[i] ^= 1;
        if (iridium_bch_ndivide(poly, bits, n_bits) == 0) {
            return 1; // 1-bit error
        }
        bits[i] ^= 1;
    }

    // Two-bit pass: O(n^2) brute force. n_bits <= 31 so worst case 465
    // divides — fast enough for the off-real-time classification path.
    for (size_t i = 0; i < n_bits; i++) {
        bits[i] ^= 1;
        for (size_t j = i + 1; j < n_bits; j++) {
            bits[j] ^= 1;
            if (iridium_bch_ndivide(poly, bits, n_bits) == 0) {
                return 2; // 2-bit error corrected (both flips remain)
            }
            bits[j] ^= 1;
        }
        bits[i] ^= 1;
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Syndrome-table fast path.
//
// One table per distinct (poly, n_bits) combo, built lazily on first use.
// Table size = 2^deg(poly) uint16 entries (the syndrome is a remainder of
// degree < deg(poly), so it always indexes in range). Entry encoding:
//   0xFFFF                -> no <=2-bit error pattern has this syndrome
//   0x8000 | i            -> single-bit error at index i (lowest such i)
//   (i << 5) | j          -> two-bit error at (i, j), i < j <= 30, the
//                            lexicographically first such pair; value
//                            <= (29<<5)|30 = 958 so bit 15 is never set.
//
// Known production combos and their table sizes (entries):
//   3545/31 (ACCH data blocks, ida_decode)   deg 11 -> 2048
//   1207/31 (ringalert + IBC blocks)         deg 10 -> 1024
//   1897/31 (messaging blocks, ims_decode)   deg 10 -> 1024
//    465/14 (LCW2, iridium_frame)            deg  8 ->  256
//     41/26 (LCW3, iridium_frame)            deg  5 ->   32
//     29/7  (LCW1, iridium_frame)            deg  4 ->   16
//     29/6  (IBC header, ibc_decode)         deg  4 ->   16
// Total 4416 entries = 8832 B; the arena below has a little slack for a
// future combo. Anything that doesn't fit (or n_bits > 31, or a slot
// shortage) transparently falls back to the brute-force reference, which
// is bit-identical by definition.
//
// Concurrency: lazy build is NOT thread-safe. All repair callers run on
// the single frame_decoder task on-device (and single-threaded host
// tests); if a second repair-calling task is ever added, pre-build the
// tables at init or add a lock here.
// ---------------------------------------------------------------------------

#define SYN_EMPTY 0xFFFFu
#define SYN_SINGLE_FLAG 0x8000u
#define SYN_MAX_SLOTS 8
#define SYN_ARENA_ENTRIES 4608 // 4416 needed by the known combos + slack
#define SYN_MAX_N_BITS 31      // i/j must fit 5 bits; matches all callers

typedef struct {
    uint32_t  poly;
    uint16_t  n_bits;
    uint8_t   deg; // u32_bit_length(poly) - 1; table has 1<<deg entries
    uint16_t *tbl;
} syn_slot_t;

static EXT_RAM_BSS_ATTR uint16_t s_syn_arena[SYN_ARENA_ENTRIES];
static syn_slot_t                s_syn_slots[SYN_MAX_SLOTS];
static int                       s_syn_nslots;
static size_t                    s_syn_arena_used;

// Find-or-build the syndrome table for (poly, n_bits). NULL -> caller
// must use the brute-force reference.
static const syn_slot_t *syn_get_slot(uint32_t poly, size_t n_bits)
{
    for (int k = 0; k < s_syn_nslots; k++) {
        if (s_syn_slots[k].poly == poly && s_syn_slots[k].n_bits == n_bits) {
            return &s_syn_slots[k];
        }
    }

    int deg = u32_bit_length(poly) - 1;
    if (deg < 1 || n_bits < 1 || n_bits > SYN_MAX_N_BITS) return 0;
    if (s_syn_nslots >= SYN_MAX_SLOTS) return 0;
    size_t tbl_len = (size_t)1 << deg;
    if (tbl_len > SYN_ARENA_ENTRIES - s_syn_arena_used) return 0;

    uint16_t *tbl = &s_syn_arena[s_syn_arena_used];
    for (size_t s = 0; s < tbl_len; s++) tbl[s] = SYN_EMPTY;

    // Single-bit syndromes: bit index i (MSB-first) is the monomial
    // x^(n_bits-1-i). Same divider as the runtime syndrome computation.
    uint32_t syn1[SYN_MAX_N_BITS];
    for (size_t i = 0; i < n_bits; i++) {
        syn1[i] = poly_mod_u32((uint32_t)1 << (n_bits - 1 - i), poly);
    }

    // Fill order == brute-force preference order: all singles (ascending
    // i) before any pair, pairs in ascending (i, j); never overwrite.
    for (size_t i = 0; i < n_bits; i++) {
        if (tbl[syn1[i]] == SYN_EMPTY) {
            tbl[syn1[i]] = (uint16_t)(SYN_SINGLE_FLAG | i);
        }
    }
    for (size_t i = 0; i < n_bits; i++) {
        for (size_t j = i + 1; j < n_bits; j++) {
            uint32_t s = syn1[i] ^ syn1[j];
            if (tbl[s] == SYN_EMPTY) {
                tbl[s] = (uint16_t)((i << 5) | j);
            }
        }
    }

    syn_slot_t *slot = &s_syn_slots[s_syn_nslots];
    slot->poly       = poly;
    slot->n_bits     = (uint16_t)n_bits;
    slot->deg        = (uint8_t)deg;
    slot->tbl        = tbl;
    s_syn_arena_used += tbl_len;
    s_syn_nslots++; // publish last (single-task assumption, see above)
    return slot;
}

int iridium_bch_repair1(uint32_t poly, uint8_t *bits, size_t n_bits)
{
    if (!bits || n_bits == 0) return -1;

    uint32_t syn = iridium_bch_ndivide(poly, bits, n_bits);
    if (syn == 0) return 0; // already clean

    const syn_slot_t *slot = syn_get_slot(poly, n_bits);
    if (!slot || (syn >> slot->deg) != 0) {
        // No table (unhosted combo) — or a syndrome outside the table,
        // which poly_mod_u32's remainder bound makes impossible; the
        // guard is pure insurance. Brute-force is bit-identical.
        return iridium_bch_repair1_ref(poly, bits, n_bits);
    }
    uint16_t e = slot->tbl[syn];
    if (e != SYN_EMPTY && (e & SYN_SINGLE_FLAG)) {
        bits[e & 0x1F] ^= 1;
        return 1; // single-bit error corrected (flip remains)
    }
    // SYN_EMPTY, or only a 2-bit pattern matches: repair1 can't fix it.
    return -1;
}

int iridium_bch_repair2(uint32_t poly, uint8_t *bits, size_t n_bits)
{
    if (!bits || n_bits == 0) return -1;

    uint32_t syn = iridium_bch_ndivide(poly, bits, n_bits);
    if (syn == 0) return 0; // already clean

    const syn_slot_t *slot = syn_get_slot(poly, n_bits);
    if (!slot || (syn >> slot->deg) != 0) {
        return iridium_bch_repair2_ref(poly, bits, n_bits);
    }
    uint16_t e = slot->tbl[syn];
    if (e == SYN_EMPTY) return -1;
    if (e & SYN_SINGLE_FLAG) {
        bits[e & 0x1F] ^= 1;
        return 1; // 1-bit error
    }
    bits[(e >> 5) & 0x1F] ^= 1;
    bits[e & 0x1F] ^= 1;
    return 2; // 2-bit error corrected (both flips remain)
}
