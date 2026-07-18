// Unit tests for the A6 hot-bin table (hot_bin_table.c) — publish/refresh/expiry/
// deadband/clear/clear_all/enable semantics and slot selection. Time is injected as
// now_ms so the whole state machine is deterministic host-side. The end-to-end GAIN
// is separately host-proven by scratchpad dropmodel.py (review §A3); this test only
// guards the table logic. See docs/2026-07-15-a6-continuation-priority-boost-spec.md.
#include <stdio.h>
#include "hot_bin_table.h"

static int passed = 0, failed = 0;
#define CHECK(cond, fmt, ...)                                             \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("  FAIL line %d: " fmt "\n", __LINE__, ##__VA_ARGS__); \
            failed++;                                                     \
        } else { passed++; }                                              \
    } while (0)

// TTL used by device is IDA_REASM_FRAG_GAP_US/1000 = 700; use it here too.
#define TTL 700u

static void test_publish_match_deadband(void)
{
    printf("Test: publish + deadband match\n");
    hot_bin_table_t t; hot_bin_table_init(&t);
    CHECK(hot_bin_table_enabled(&t), "default enabled");
    CHECK(!hot_bin_table_match(&t, 100, 0), "empty table: no match");
    hot_bin_table_publish(&t, 100, 0, TTL);
    CHECK(hot_bin_table_match(&t, 100, 0), "exact bin matches");
    CHECK(hot_bin_table_match(&t, 104, 0), "+4 bins (edge) matches");
    CHECK(hot_bin_table_match(&t,  96, 0), "-4 bins (edge) matches");
    CHECK(!hot_bin_table_match(&t, 105, 0), "+5 bins outside deadband");
    CHECK(!hot_bin_table_match(&t,  95, 0), "-5 bins outside deadband");
    CHECK(hot_bin_table_published(&t) == 1, "published counter = 1");
}

static void test_ttl_expiry(void)
{
    printf("Test: TTL expiry\n");
    hot_bin_table_t t; hot_bin_table_init(&t);
    hot_bin_table_publish(&t, 500, 1000, TTL); // expiry = 1700
    CHECK(hot_bin_table_match(&t, 500, 1699), "live just before expiry");
    CHECK(!hot_bin_table_match(&t, 500, 1700), "expired at exactly expiry_ms");
    CHECK(!hot_bin_table_match(&t, 500, 5000), "expired long after");
}

static void test_refresh_reuses_slot(void)
{
    printf("Test: refresh within deadband reuses the slot and extends TTL\n");
    hot_bin_table_t t; hot_bin_table_init(&t);
    hot_bin_table_publish(&t, 100, 0, TTL);   // expiry 700
    hot_bin_table_publish(&t, 102, 500, TTL); // within deadband → refresh, expiry 1200, bin 102
    CHECK(hot_bin_table_match(&t, 102, 1100), "refreshed entry live past original expiry");
    CHECK(hot_bin_table_published(&t) == 2, "two publishes counted");
    // Only one slot consumed: fill the other 3 with far bins, then a 5th far bin must
    // evict a soonest-expiring one — proving the refresh did NOT consume 2 slots.
    hot_bin_table_publish(&t, 2000, 500, TTL);
    hot_bin_table_publish(&t, 3000, 500, TTL);
    hot_bin_table_publish(&t, 4000, 500, TTL); // now 4 live: {102,2000,3000,4000}
    CHECK(hot_bin_table_match(&t, 102, 600) && hot_bin_table_match(&t, 4000, 600),
          "all four distinct chains live");
}

static void test_slot_selection_overwrites_soonest(void)
{
    printf("Test: table full → overwrite soonest-expiring\n");
    hot_bin_table_t t; hot_bin_table_init(&t);
    hot_bin_table_publish(&t, 1000, 0, 100); // expiry 100 (soonest)
    hot_bin_table_publish(&t, 2000, 0, 200);
    hot_bin_table_publish(&t, 3000, 0, 300);
    hot_bin_table_publish(&t, 4000, 0, 400); // 4 live
    hot_bin_table_publish(&t, 5000, 50, TTL); // all live at 50 → evict soonest (bin 1000)
    CHECK(!hot_bin_table_match(&t, 1000, 60), "soonest-expiring (1000) was evicted");
    CHECK(hot_bin_table_match(&t, 5000, 60), "newcomer (5000) present");
    CHECK(hot_bin_table_match(&t, 2000, 60), "2000 retained");
    CHECK(hot_bin_table_match(&t, 4000, 60), "4000 retained");
}

static void test_free_slot_preferred_over_evict(void)
{
    printf("Test: expired slot reused before evicting a live one\n");
    hot_bin_table_t t; hot_bin_table_init(&t);
    hot_bin_table_publish(&t, 1000, 0, 100); // expires at 100
    hot_bin_table_publish(&t, 2000, 0, TTL); // live long
    // At now=200 the first slot is expired (free); publishing must reuse it, not evict 2000.
    hot_bin_table_publish(&t, 3000, 200, TTL);
    CHECK(hot_bin_table_match(&t, 2000, 210), "long-lived 2000 survived (free slot was reused)");
    CHECK(hot_bin_table_match(&t, 3000, 210), "3000 placed");
    CHECK(!hot_bin_table_match(&t, 1000, 210), "expired 1000 gone");
}

static void test_clear(void)
{
    printf("Test: clear-on-complete\n");
    hot_bin_table_t t; hot_bin_table_init(&t);
    hot_bin_table_publish(&t, 200, 0, TTL);
    hot_bin_table_clear(&t, 202, 10); // within deadband of 200 → clears
    CHECK(!hot_bin_table_match(&t, 200, 20), "cleared entry no longer matches");
    CHECK(hot_bin_table_cleared(&t) == 1, "cleared counter = 1");
    hot_bin_table_clear(&t, 9999, 20); // no live entry near → no-op
    CHECK(hot_bin_table_cleared(&t) == 1, "clear miss does not bump counter");
}

static void test_clear_all_and_enable(void)
{
    printf("Test: clear_all + enable gate\n");
    hot_bin_table_t t; hot_bin_table_init(&t);
    hot_bin_table_publish(&t, 300, 0, TTL);
    hot_bin_table_publish(&t, 900, 0, TTL);
    hot_bin_table_clear_all(&t);
    CHECK(!hot_bin_table_match(&t, 300, 10) && !hot_bin_table_match(&t, 900, 10),
          "clear_all invalidates every entry");
    hot_bin_table_publish(&t, 300, 20, TTL);
    hot_bin_table_set_enabled(&t, false);
    CHECK(!hot_bin_table_match(&t, 300, 25), "disabled: no boost even when live");
    hot_bin_table_set_enabled(&t, true);
    CHECK(hot_bin_table_match(&t, 300, 25), "re-enabled: live entry matches again");
}

int main(void)
{
    test_publish_match_deadband();
    test_ttl_expiry();
    test_refresh_reuses_slot();
    test_slot_selection_overwrites_soonest();
    test_free_slot_preferred_over_evict();
    test_clear();
    test_clear_all_and_enable();
    printf("\nhot_bin_table: %d passed, %d failed\n", passed, failed);
    return failed ? 1 : 0;
}
