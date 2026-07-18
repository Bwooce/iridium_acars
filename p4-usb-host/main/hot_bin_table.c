#include "hot_bin_table.h"
#include <stdlib.h> // abs()

static inline bool within(uint32_t a, uint32_t b)
{
    return (uint32_t)abs((int)a - (int)b) <= HOT_BIN_DEADBAND;
}

void hot_bin_table_init(hot_bin_table_t *t)
{
    for (int i = 0; i < HOT_BIN_ENTRIES; i++) {
        atomic_store_explicit(&t->e[i].bin, 0, memory_order_relaxed);
        atomic_store_explicit(&t->e[i].expiry_ms, 0, memory_order_relaxed);
    }
    atomic_store_explicit(&t->enabled, true, memory_order_relaxed);
    atomic_store_explicit(&t->published, 0, memory_order_relaxed);
    atomic_store_explicit(&t->cleared, 0, memory_order_relaxed);
}

// Re-arm an entry: close the window (expiry=0, release), set bin (relaxed), then
// open the window (expiry=deadline, release). The trailing release publishes the bin.
static inline void arm(hot_bin_entry_t *e, uint32_t bin, uint32_t deadline)
{
    atomic_store_explicit(&e->expiry_ms, 0, memory_order_release);
    atomic_store_explicit(&e->bin, bin, memory_order_relaxed);
    atomic_store_explicit(&e->expiry_ms, deadline, memory_order_release);
}

void hot_bin_table_publish(hot_bin_table_t *t, uint32_t bin, uint32_t now_ms, uint32_t ttl_ms)
{
    const uint32_t deadline = now_ms + ttl_ms;
    int      free_i = -1, soon_i = 0;
    uint32_t soon_exp = UINT32_MAX;
    for (int i = 0; i < HOT_BIN_ENTRIES; i++) {
        uint32_t exp  = atomic_load_explicit(&t->e[i].expiry_ms, memory_order_acquire);
        bool     live = (exp != 0 && now_ms < exp);
        if (live && within(atomic_load_explicit(&t->e[i].bin, memory_order_relaxed), bin)) {
            arm(&t->e[i], bin, deadline);                 // refresh existing chain
            atomic_fetch_add_explicit(&t->published, 1, memory_order_relaxed);
            return;
        }
        if (!live && free_i < 0) free_i = i;              // first empty/expired slot
        if (exp < soon_exp) { soon_exp = exp; soon_i = i; }
    }
    arm(&t->e[(free_i >= 0) ? free_i : soon_i], bin, deadline);
    atomic_fetch_add_explicit(&t->published, 1, memory_order_relaxed);
}

void hot_bin_table_clear(hot_bin_table_t *t, uint32_t bin, uint32_t now_ms)
{
    for (int i = 0; i < HOT_BIN_ENTRIES; i++) {
        uint32_t exp = atomic_load_explicit(&t->e[i].expiry_ms, memory_order_acquire);
        if (exp == 0 || now_ms >= exp) continue;
        if (within(atomic_load_explicit(&t->e[i].bin, memory_order_relaxed), bin)) {
            atomic_store_explicit(&t->e[i].expiry_ms, 0, memory_order_release);
            atomic_fetch_add_explicit(&t->cleared, 1, memory_order_relaxed);
            return;
        }
    }
}

void hot_bin_table_clear_all(hot_bin_table_t *t)
{
    for (int i = 0; i < HOT_BIN_ENTRIES; i++)
        atomic_store_explicit(&t->e[i].expiry_ms, 0, memory_order_release);
}

bool hot_bin_table_match(const hot_bin_table_t *t, uint32_t bin, uint32_t now_ms)
{
    if (!atomic_load_explicit(&t->enabled, memory_order_relaxed)) return false;
    for (int i = 0; i < HOT_BIN_ENTRIES; i++) {
        uint32_t exp = atomic_load_explicit(&t->e[i].expiry_ms, memory_order_acquire);
        if (exp == 0 || now_ms >= exp) continue;
        if (within(atomic_load_explicit(&t->e[i].bin, memory_order_relaxed), bin)) return true;
    }
    return false;
}

void     hot_bin_table_set_enabled(hot_bin_table_t *t, bool on) { atomic_store_explicit(&t->enabled, on, memory_order_relaxed); }
bool     hot_bin_table_enabled(const hot_bin_table_t *t)        { return atomic_load_explicit(&t->enabled, memory_order_relaxed); }
uint32_t hot_bin_table_published(const hot_bin_table_t *t)      { return atomic_load_explicit(&t->published, memory_order_relaxed); }
uint32_t hot_bin_table_cleared(const hot_bin_table_t *t)        { return atomic_load_explicit(&t->cleared, memory_order_relaxed); }
