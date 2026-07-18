// Chase-2 soft-decision BCH fallback for LW.DA frames. See ida_chase.h.
//
// Reference implementation: tests/scripts/chase_crc_bch.py (host
// prototype, validated against gr-iridium ground truth 2026-07-17).
// Parity is pinned by tests/host/test_chase_parity.c on the same dump
// the prototype ran on; the synthetic unit gate is
// tests/host/test_ida_chase.c.
//
// One deliberate divergence from the Python: candidate messages are
// enumerated in ASCENDING numeric order (the Python collects them in a
// `set`, whose iteration order is a CPython hash-table artefact). The
// order only matters when a frame has MULTIPLE CRC-valid hypotheses
// (never observed; would itself be a CRC collision) or when the check
// cap truncates the search — in both cases ascending order is the
// deterministic choice. Recovery outcomes on the reference corpus are
// identical (see test_chase_parity.c).

#include "ida_chase.h"
#include "iridium_bch.h"
#include <string.h>

#ifdef ESP_PLATFORM
// EXT_RAM_BSS_ATTR: keep the chase scratch (s_map + per-call cand/rcw, ~3.2 KB
// .bss) in PSRAM, not internal RAM — internal .bss starves the DMA-INT/USB
// budget and (as the RAW smoke caught 2026-07-18) fragments it below the
// fft_burst_tagger's 65 KB alloc. Chase is a rare Core-0 fail-path, so PSRAM
// latency is fine. See feedback_dma_int_budget_audit.
#include "esp_attr.h"
#else
#define EXT_RAM_BSS_ATTR
#endif

#define ACCH_BCH_POLY 3545u
#define FRAME_BITS_NEEDED (IDA_DECODE_DATA_OFF + IDA_DECODE_DATA_BITS) // 382

// ---- module state (single-task contract, see header) ----------------
static bool s_enabled = false;
static int  s_L       = IDA_CHASE_DEFAULT_L;
static int  s_max_crc = IDA_CHASE_DEFAULT_MAX_CRC;

static ida_chase_stats_t s_stats;

// Codeword-bit -> frame-data-section-bit index map: s_map[cw][k] is the
// index (0..311, relative to frame bit 70) of the received bit that
// lands at bit k of codeword cw. Built once by probing the production
// transform with one-hot inputs, so it can never drift from
// ida_decode_build_codewords().
static EXT_RAM_BSS_ATTR uint16_t s_map[IDA_DECODE_N_CW][IDA_DECODE_CW_BITS];
static bool     s_map_built = false;

static void build_index_map(void)
{
    uint8_t data[IDA_DECODE_DATA_BITS];
    uint8_t cw[IDA_DECODE_N_CW_BITS];
    for (int p = 0; p < IDA_DECODE_DATA_BITS; p++) {
        memset(data, 0, sizeof(data));
        data[p] = 1;
        ida_decode_build_codewords(data, cw);
        for (int j = 0; j < IDA_DECODE_N_CW_BITS; j++) {
            if (cw[j]) {
                s_map[j / IDA_DECODE_CW_BITS][j % IDA_DECODE_CW_BITS] =
                    (uint16_t)p;
                break; // one-hot in => exactly one hot bit out
            }
        }
    }
    s_map_built = true;
}

// ---- public config --------------------------------------------------
bool ida_chase_set_enabled(bool enable)
{
    bool prev = s_enabled;
    s_enabled = enable;
    return prev;
}
bool ida_chase_get_enabled(void) { return s_enabled; }

void ida_chase_set_params(int L, int max_crc_checks)
{
    if (L < 1) L = 1;
    if (L > IDA_CHASE_MAX_L) L = IDA_CHASE_MAX_L;
    if (max_crc_checks < 1) max_crc_checks = 1;
    if (max_crc_checks > 4096) max_crc_checks = 4096;
    s_L       = L;
    s_max_crc = max_crc_checks;
}
void ida_chase_get_params(int *L, int *max_crc_checks)
{
    if (L) *L = s_L;
    if (max_crc_checks) *max_crc_checks = s_max_crc;
}

void ida_chase_get_stats(ida_chase_stats_t *out)
{
    if (out) *out = s_stats;
}

// ---- helpers --------------------------------------------------------

// Per-frame-bit reliability: pair-MIN of the parent symbol's magnitude
// and the previous symbol's magnitude (DQPSK differential coupling —
// mirrors chase_crc_bch.py frame_conf()). `fb` is an absolute frame bit
// index; caller guarantees soft coverage.
static inline int32_t bit_conf(const int16_t *soft, int fb)
{
    int     sym = fb >> 1;
    int32_t m   = soft[2 * sym];
    if (m < 0) m = -m;
    if (sym > 0) {
        int32_t mp = soft[2 * (sym - 1)];
        if (mp < 0) mp = -mp;
        if (mp < m) m = mp;
    }
    return m;
}

// Hard-decode one 31-bit codeword (0/1-per-byte) with the production
// t<=2 repair. Returns the 20-bit message (>=0) or -1 if uncorrectable.
// `cw` is modified in place on success (repair semantics).
static int32_t hard_decode_cw(uint8_t cw[IDA_DECODE_CW_BITS])
{
    if (iridium_bch_repair2(ACCH_BCH_POLY, cw, IDA_DECODE_CW_BITS) < 0)
        return -1;
    uint32_t msg = 0;
    for (int b = 0; b < 20; b++)
        msg = (msg << 1) | (cw[b] & 1);
    return (int32_t)msg;
}

// Insert `v` into a sorted-unique ascending list. Returns new count.
static int insert_unique(uint32_t *list, int n, uint32_t v)
{
    int lo = 0;
    while (lo < n && list[lo] < v) lo++;
    if (lo < n && list[lo] == v) return n;
    for (int j = n; j > lo; j--) list[j] = list[j - 1];
    list[lo] = v;
    return n + 1;
}

// ---- main entry -----------------------------------------------------
int ida_chase_decode(const iridium_frame_t *frame,
                     const int16_t *soft_bits, size_t n_soft,
                     ida_decoded_t *out)
{
    if (!frame || !out) return -1;
    if (!s_enabled) return 0;
    if (frame->type != IR_FRAME_LW || frame->lw_subtype != IR_LW_DA) return -1;
    if (out->ok) return 0;                       // nothing to chase
    if (!soft_bits || n_soft < FRAME_BITS_NEEDED) return 0;
    if (frame->n_bits < FRAME_BITS_NEEDED) return 0;

    if (!s_map_built) build_index_map();

    const int L      = s_L;
    const int n_trial = 1 << L;

    // Received codewords — the exact words the hard path saw.
    static EXT_RAM_BSS_ATTR uint8_t rcw[IDA_DECODE_N_CW_BITS];
    ida_decode_build_codewords(frame->bits + IDA_DECODE_DATA_OFF, rcw);

    // Hard-decode pass: identical outcome to ida_decode()'s loop
    // (same repair2 on the same words), re-run here so this module
    // needs no side-channel from ida_decode beyond `out`.
    int32_t msgs[IDA_DECODE_N_CW];
    int     failed[IDA_DECODE_N_CW];
    int     n_failed = 0;
    for (int i = 0; i < IDA_DECODE_N_CW; i++) {
        uint8_t cw[IDA_DECODE_CW_BITS];
        memcpy(cw, rcw + i * IDA_DECODE_CW_BITS, IDA_DECODE_CW_BITS);
        msgs[i] = hard_decode_cw(cw);
        if (msgs[i] < 0) failed[n_failed++] = i;
    }
    if (n_failed == 0) return 0; // inconsistent with !out->ok; be safe

    s_stats.attempts++;

    // Candidate 20-bit messages per failed codeword (distinct, sorted
    // ascending). Static scratch: 10 * 64 * 4 B = 2.5 KB.
    static EXT_RAM_BSS_ATTR uint32_t cand[IDA_DECODE_N_CW][1 << IDA_CHASE_MAX_L];
    int             n_cand[IDA_DECODE_N_CW];

    for (int f = 0; f < n_failed; f++) {
        int            i  = failed[f];
        const uint8_t *cw = rcw + i * IDA_DECODE_CW_BITS;

        // Reliability of each codeword bit = conf of its source frame bit.
        int32_t conf[IDA_DECODE_CW_BITS];
        for (int k = 0; k < IDA_DECODE_CW_BITS; k++)
            conf[k] = bit_conf(soft_bits,
                               IDA_DECODE_DATA_OFF + s_map[i][k]);

        // L least-reliable positions. Insertion with strict '<' keeps
        // the LOWEST index on ties — matches the reference's stable
        // sort + first-L slice.
        int lcb[IDA_CHASE_MAX_L];
        int n_lcb = 0;
        for (int k = 0; k < IDA_DECODE_CW_BITS; k++) {
            int pos = n_lcb;
            while (pos > 0 && conf[k] < conf[lcb[pos - 1]]) pos--;
            if (pos < L) {
                if (n_lcb < L) n_lcb++;
                for (int j = n_lcb - 1; j > pos; j--) lcb[j] = lcb[j - 1];
                lcb[pos] = k;
            }
        }

        // Enumerate the 2^L flip patterns; collect distinct messages.
        int nc = 0;
        for (int mset = 0; mset < n_trial; mset++) {
            uint8_t trial[IDA_DECODE_CW_BITS];
            memcpy(trial, cw, IDA_DECODE_CW_BITS);
            for (int j = 0; j < n_lcb; j++)
                if (mset & (1 << j)) trial[lcb[j]] ^= 1;
            int32_t m = hard_decode_cw(trial);
            if (m >= 0) nc = insert_unique(cand[i], nc, (uint32_t)m);
        }
        if (nc == 0) return 0; // a failed cw with no candidate -> dead
        n_cand[i] = nc;
    }

    // Cartesian product over the failed codewords' candidate lists
    // (odometer), CRC-16-arbitrated, bounded by the check cap.
    int odo[IDA_DECODE_N_CW] = {0};
    int checks = 0;
    while (checks < s_max_crc) {
        // Assemble the 200-bit message hypothesis.
        ida_decoded_t trial;
        memset(&trial, 0, sizeof(trial));
        trial.n_blocks  = IDA_DECODE_N_CW;
        trial.blocks_ok = IDA_DECODE_N_CW;
        trial.n_bits    = IDA_DECODE_N_CW * 20;
        trial.ok        = true;
        for (int i = 0; i < IDA_DECODE_N_CW; i++) {
            uint32_t m = (msgs[i] >= 0)
                             ? (uint32_t)msgs[i]
                             : cand[i][odo[i]];
            for (int b = 0; b < 20; b++)
                trial.bits[i * 20 + b] = (uint8_t)((m >> (19 - b)) & 1);
        }
        checks++;
        s_stats.crc_checks++;

        // Production field parse + CRC arbitration (bit-identical to
        // the hard path's acceptance criterion: zero1==0 via header_ok,
        // da_len>0 + CRC residue==0 via crc_ok).
        ida_decode_parse_fields(&trial);
        if (trial.header_ok && trial.crc_ok) {
            // total_errors: hard-path corrections are unknown here
            // (repair2 already applied); report the Hamming distance
            // between the received words and the accepted hypothesis'
            // re-encoded words is overkill — keep the hard blocks' count
            // semantics by leaving total_errors as "chased" marker-only.
            trial.total_errors = out->total_errors;
            trial.chase_used   = true;
            trial.chase_checks = (uint16_t)checks;
            *out               = trial;
            s_stats.recovered++;
            return 1;
        }

        // Advance odometer (failed codewords only, first-listed fastest —
        // matches itertools.product's rightmost-fastest over the same
        // list order after accounting for the ascending candidate order).
        int f = n_failed - 1;
        while (f >= 0) {
            int i = failed[f];
            if (++odo[i] < n_cand[i]) break;
            odo[i] = 0;
            f--;
        }
        if (f < 0) break; // exhausted all hypotheses
    }
    return 0;
}
