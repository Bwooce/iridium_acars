#ifndef HOT_BIN_TABLE_H
#define HOT_BIN_TABLE_H

// A6 continuation-priority boost: a tiny table of the detect-FFT bins of currently
// OPEN IDA chains. The frame_decoder task (Core 0) publishes/clears entries; the
// worker's burst_priority() (Core 1) and the tagger callback (Core 0) query it to
// boost bursts on a channel where a chain is mid-reassembly, so the continuation is
// decoded before its ring samples lapse. See
// docs/2026-07-15-a6-continuation-priority-boost-spec.md.
//
// Dependency-free (host-testable): time is passed in as now_ms (esp_timer/1000 on
// device), never read internally. Single writer (decoder task); readers are lock-free.
// Per-entry release/acquire on expiry_ms after a relaxed bin store guarantees an
// acquire reader that sees a live expiry also sees the matching bin; any interleaving
// only makes an entry momentarily invisible (a missed boost) — never a torn/foreign
// bin paired with a live expiry. 32-bit atomics only (64-bit is not lock-free on RV32).
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>

#define HOT_BIN_ENTRIES  4   // == IDA_REASM_MAX_SESSIONS (static_assert in worker_core1.c)
#define HOT_BIN_DEADBAND 4   // bins; == floor(IDA_REASM_FREQ_DEADBAND_HZ 5000 /
                             // (FS_DETECT_HZ 2500000 / FFT_SIZE 2048 = 1220.7 Hz/bin))

typedef struct {
    _Atomic uint32_t bin;       // detect-FFT bin (BURST_PEAK_BIN space)
    _Atomic uint32_t expiry_ms; // deadline in the caller's ms clock; 0 = empty
} hot_bin_entry_t;

typedef struct {
    hot_bin_entry_t  e[HOT_BIN_ENTRIES];
    _Atomic bool     enabled;   // runtime A/B gate
    _Atomic uint32_t published; // publish/refresh calls
    _Atomic uint32_t cleared;   // clear-on-complete that found a live entry
} hot_bin_table_t;

// Initialise: all entries empty, enabled = true (A6 default ON).
void hot_bin_table_init(hot_bin_table_t *t);

// Publish/refresh a chain's bin with a fresh TTL (now_ms + ttl_ms). Reuses an entry
// already within HOT_BIN_DEADBAND, else a free/expired slot, else the soonest-expiring.
void hot_bin_table_publish(hot_bin_table_t *t, uint32_t bin, uint32_t now_ms, uint32_t ttl_ms);

// Clear a live entry within HOT_BIN_DEADBAND of `bin` (chain completed). No-op if none.
void hot_bin_table_clear(hot_bin_table_t *t, uint32_t bin, uint32_t now_ms);

// Invalidate every entry (LO retune: bins are LO-relative and now meaningless).
void hot_bin_table_clear_all(hot_bin_table_t *t);

// True iff enabled AND `bin` is within HOT_BIN_DEADBAND of a live entry at now_ms.
bool hot_bin_table_match(const hot_bin_table_t *t, uint32_t bin, uint32_t now_ms);

void     hot_bin_table_set_enabled(hot_bin_table_t *t, bool on);
bool     hot_bin_table_enabled(const hot_bin_table_t *t);
uint32_t hot_bin_table_published(const hot_bin_table_t *t);
uint32_t hot_bin_table_cleared(const hot_bin_table_t *t);

#endif // HOT_BIN_TABLE_H
