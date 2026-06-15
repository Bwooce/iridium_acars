# Fault-injection harness + tests for recovery counters — plan

**Status: IMPLEMENTED 2026-06-15.** All five sites wired behind
`CONFIG_FAULT_INJECT` (default off → hooks fold to `return false`, production
binary byte-identical), `POST /debug/fault_inject?site=&count=` added, and
`tests/scripts/fault_inject_recovery.sh` validates each site on a live device.
End-to-end run (device built with `CONFIG_FAULT_INJECT=y`): **all 5 sites PASS**
— each `/diag/recovery_counters` value climbed by the injected count, no reboot,
USB stream kept advancing (no #106-class deadlock). One refinement vs the plan:
`urb_submit` injection is non-destructive (it exercises the pool-lost accounting
+ log but resubmits the URB, so the in-flight pool isn't actually shrunk and the
test is repeatable). The original design plan follows for reference.

---

**Original plan (design):** Drafted 2026-05-31 after the day's other tasks
shipped.

## Why this is a plan rather than a commit

The recovery paths that #122 wants to exercise are currently load-bearing
and well-tested by *real* events (signal_buffer's s_dma_submit_errors
fires ~5/min in production; the #106 deadlock recovery itself has
absorbed 100+ live failures across multiple multi-day soaks). The
remaining benefit of synthetic injection is for the rare paths that
production hasn't tripped:

- `s_dispatch_drops` (ingest queue full — has never fired in months)
- `s_take_converted_slow_waits` (#110 — has never fired)
- `s_xfer_pool_lost` (#124 — has never fired)
- Wrap-path DMA failure (#107 — fires in theory when a wrap coincides
  with an async_memcpy ESP_ERR_NO_MEM, not seen in soak)

These four are the ones a fault-injection harness would actually add
coverage to. The harness is ~half a day of careful work; doing it
without the time to be careful is how you get the kind of regression
that #126 produced.

## What "done" looks like

A single Kconfig option `CONFIG_FAULT_INJECT` (default off in
production builds, opt-in for the test build) compiles in:

1. **`fault_inject.{c,h}`** — tiny module exposing:
   ```c
   typedef enum {
       FI_SITE_DMA_SUBMIT,        // signal_buffer_push's esp_async_memcpy
       FI_SITE_DMA_SUBMIT_WRAP,   // ditto, but only on the wrap-path first call
       FI_SITE_DISPATCH_QUEUE,    // ingest_core1_dispatch's xQueueSend
       FI_SITE_TAKE_CONVERTED,    // ingest_core1_take_converted's xSemaphoreTake
       FI_SITE_URB_SUBMIT,        // esp_libusb stream_transfer_cb resubmit
       FI_SITE_COUNT,
   } fi_site_t;

   // Schedule N synthetic failures at the given site. Decremented
   // each time fault_inject_should_fail() returns true. Idempotent;
   // additional requests accumulate the count.
   void fault_inject_request(fi_site_t site, uint32_t count);

   // Site code calls this in its hot path — returns true and
   // decrements when a synthetic failure is scheduled, else false.
   // Compiled to `return false` when CONFIG_FAULT_INJECT is off, so
   // production builds pay zero cost.
   bool fault_inject_should_fail(fi_site_t site);
   ```
   Backing storage: `static volatile uint32_t s_fi_remaining[FI_SITE_COUNT];`

2. **Site instrumentation** — at each fault site, the existing code
   path is wrapped with `if (fault_inject_should_fail(SITE)) goto
   error_path;`. Five sites, five 3-line additions. Examples:
   ```c
   // signal_buffer.c, in signal_buffer_push, before esp_async_memcpy:
   if (fault_inject_should_fail(FI_SITE_DMA_SUBMIT)) {
       r = ESP_ERR_NO_MEM;
   } else {
       r = esp_async_memcpy(...);
   }
   ```

3. **HTTP endpoint** `POST /debug/fault_inject?site=<name>&count=<N>`
   in `http_server.c`. Returns 200 with current schedule; 400 on bad
   site name; 503 if CONFIG_FAULT_INJECT is off.

4. **Host-side test script** `tests/scripts/fault_inject_recovery.sh`
   — for each site:
   - Snapshot the corresponding counter via /status (or /diag/* once
     a counter endpoint exists — see follow-up below)
   - POST a 5-failure burst
   - Wait 5 s for the failures to fire
   - Snapshot again, assert counter went up by 5
   - Assert usb.completed continued to climb (no deadlock)
   - Assert no STALL DIAG / no reboot

Exit nonzero on any failure. CI runs the script against a flashed
device.

## Follow-up: getter parity

Several recovery counters lack /status surfacing. Before the test can
read them, they need exposure:

- `s_dispatch_drops` — already has `ingest_core1_dispatch_drops()`,
  needs adding to /status JSON or a new /diag/recovery_counters
- `s_take_converted_slow_waits` — has `ingest_core1_take_converted_slow_waits()`,
  same need
- `s_dma_submit_errors`, `s_dma_timeouts` — have getters, not in /status
- `s_xfer_pool_lost` — no getter yet

Sensible new endpoint: `GET /diag/recovery_counters` returning a flat
JSON of every recovery counter and its current value. ~30 lines, one
commit.

## Test order

1. Add `GET /diag/recovery_counters` (depends on nothing).
2. Add `fault_inject` module + Kconfig flag.
3. Wire each site one at a time, validate after each.
4. Add `tests/scripts/fault_inject_recovery.sh`.

Estimated: half-day for steps 1-2, half-day for steps 3-4. Splits
cleanly into two commits if you want to land step 1 alone.

## What this doc does NOT solve

- **ISR-context faults** — some of these recovery paths fire from ISR
  context (URB completion callback). Injection has to be safe from
  ISR. The `s_fi_remaining[]` array is volatile uint32_t — safe to
  decrement from ISR.
- **Multi-fault interactions** — injecting two faults in quick
  succession may unmask new bugs (e.g. dma_submit fails AND
  dispatch_queue is full at the same instant). The script tests each
  site in isolation; full pairwise sweep is a separate harness.
- **Boot-time fault injection** — if a fault should fire before httpd
  is up, you can't request it via HTTP. Use a Kconfig
  `CONFIG_FAULT_INJECT_BOOT_<SITE>=N` flag for that case if needed
  later; out of scope here.
