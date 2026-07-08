#pragma once
// Boot-time + periodic auto-run scheduling for the autotune RF-recalibration
// engine (2026-07-08 boot/periodic extension of the manual `autotune`
// design). Two independent clocks, both anchored to when the USB stream is
// first observed live (not raw boot time, so the first fire doesn't land in
// USB-enumeration chaos):
//   - autotune_gain_interval_s -- periodic IRA gain re-cal (autotune_run_manual).
//   - autotune_lo_interval_s   -- periodic LO density re-scan
//     (autotune_run_lo_rescan). SLOW by design -- see the design doc's
//     "Empirical findings": best-LO is mean-reverting, not momentum, so this
//     must stay long (default hourly), not the original 600 s satellite-
//     handoff estimate.
// autotune_on_boot additionally runs ONE gain-cal pass right after the first
// anchor point, before the periodic loop starts.
//
// The due-check arithmetic below is pure (no FreeRTOS/esp_timer deps) so
// it's host-testable in isolation, mirroring autotune_gainset.h /
// worker_dcfine.h. Task creation (which does need the device stack) lives
// in autotune_sched.c.
#include <stdint.h>
#include <stdbool.h>

// True if `interval_s` seconds have elapsed between `last_run_s` and
// `now_s` (same monotonic clock, e.g. esp_timer_get_time()/1000000).
// interval_s == 0 means that clock is disabled -> never due. A negative
// elapsed time (clock skew / bad inputs) is treated as "not due" rather than
// underflowing into an immediate fire.
static inline bool autotune_is_due(int64_t last_run_s, int64_t now_s, uint32_t interval_s)
{
    if (interval_s == 0) return false;
    int64_t elapsed = now_s - last_run_s;
    if (elapsed < 0) return false;
    return elapsed >= (int64_t)interval_s;
}

// Start the boot-trigger + periodic-reschedule task (idempotent; a second
// call is a no-op). Call once from app_main after the USB/DSP front end
// (class_driver_task, which wires the scanner) has been created -- the task
// itself blocks until the stream is observed live before touching anything,
// so exact ordering relative to class_driver's startup isn't critical.
void autotune_sched_init(void);
