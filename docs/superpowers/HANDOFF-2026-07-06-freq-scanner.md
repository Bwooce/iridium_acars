# Handoff — Iridium frequency scanner + T60 (2026-07-06)

Read this first if you're taking over. It captures branch state, what works, what's
deferred, and how to continue. Companion docs: the design spec and plan under
`docs/superpowers/specs/` and `docs/superpowers/plans/` (both dated 2026-07-06),
and the SDD progress ledger at `.superpowers/sdd/progress.md` (git-ignored scratch).

## TL;DR

- **T60 (narrowband spectral-width priority)** — DONE, production-verified on device,
  on branch `t60-narrowband-priority` (commit 7f77955). Smoke gate is DEFERRED
  because it exposes a latent PIE-save deadlock (see below), not because T60 is
  wrong.
- **Frequency scanner Phase 1** — built + reviewed (Tasks 1–8) on branch
  `freq-scanner` (HEAD d392249, which contains T60 + the scanner). The live-retune
  mechanism works: **manual `hop <hz>` retunes reliably with no reboot**. The
  automated `scan` sweep is **NOT reliable yet** — blocked by DMA-internal heap
  tightness (the immediate follow-up).
- Nothing is pushed. `freq-scanner` is local-only (pre-push smoke gate blocks it).

## Branches

| Branch | HEAD | Contents |
|---|---|---|
| `main` | 5659b35 | baseline (T59 PQ + crash fix + review docs) |
| `t60-narrowband-priority` | 7f77955 | T60 feature only (off main) |
| `freq-scanner` | d392249 | T60 + scanner spec/plan + Tasks 1–8 (off t60) |

`freq-scanner` commits after T60: 2dd0cf2 (spec), 1829473 (plan), then 7b8090f,
76e9d2f, 9cc99c4, 62c594e, 9cdea46, bf7bd92, 2819109, 8ba6943, 5fb83d7, d392249.
Working tree clean.

**Not pushed:** the pre-push hook (`.githooks/pre-push`) requires a `Smoke-verified:`
trailer on DSP-touching commits, and the device-smoke GOLDEN currently HANGS (the
PIE-save deadlock below), so it can't be run to a pass. Pushing needs either the
PIE fix or an honest `Smoke-skip:` decision — a human call.

## What works (verified on device)

- **`hop <hz> [save]`** serial command → live LO retune, no reboot. Verified:
  `stream paused (live_xfers=0)` → full tuner reprogram (R82xx PLL to hz+3.57MHz IF)
  → `stream resumed: N transfers re-submitted` → `OK hopped`. serial_cmd stays
  responsive; no crash. `save` also persists to NVS `lo_hz`.
- **`scan [start stop step dwell_ms]`** and **`map`** commands exist and produce a
  ranked narrowband-density map + park-on-hottest — the mechanism is proven (a full
  map printed with per-position nb/s when hops succeed).
- Host unit tests: `tests/host/test_scanner_map` (enumeration/ranking, 10 asserts),
  `tests/host/test_scanner_tagger_reset`. Both pass.

## Deferred / broken — the TODO list for the next session

1. **Scanner `scan` reliability (DMA-internal budget) — TOP priority.**
   Rapid back-to-back retunes exhaust the tight DMA-internal heap (each retune is
   dozens of control transfers, each alloc'ing a buffer). Mid-scan the device throws
   `retune err=263` (ESP_ERR_TIMEOUT), `err=-1` (tuner fail), then `USBH: EP 0 STALL`
   + `stash_alloc_fail (NO_MEM)`. **Single/manual hop is unaffected.** Fix: reclaim
   internal DMA-capable RAM (move a consumer to PSRAM) or bump
   `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL` so control-transfer churn has headroom.
   See memory `feedback_dma_int_budget_audit` (keep pre-stream DMA-INT free ≥ ~62 KB;
   this build is ~35 KB). Boot log shows `transfer_alloc #4 failed` → only ~4 of 8
   USB transfers alloc (streams fine, but no slack). This is the same tightness that
   capped the transfer pool.
2. **T60 smoke gate (PIE-save deadlock).** Memory `project_pie_save_deadlock_smoke`.
   Device-smoke GOLDEN HP-WDT-hangs at `rtos_save_pie_coproc` — esp-dsp PIE
   `esp.lp.setup` loop state isn't saved on a task switch. Smoke-fixture-timing only;
   production ran clean (but a related worker-stall recovery reboot WAS seen live —
   see the memory update). This blocks both the T60 gate and pushing `freq-scanner`.
3. **Final whole-branch review** — package generated at
   `.superpowers/sdd/review-7f77955..d392249.diff` (12 commits, 95 KB). NOT yet run.
   Dispatch a capable-model reviewer over it before merge. Minor-findings roll-up is
   at the bottom of `.superpowers/sdd/progress.md`.
4. **Scanner Phase 2** (not started) — the autonomous scan→dwell→resume state
   machine with hysteresis. Needs its own spec; Phase 1 is the manual core it builds
   on. Only worth doing once (1) makes scanning reliable.

## Minor findings carried forward (from per-task reviews; see ledger)

- T3: density-counter tap sits after `if (!p->user_cb) return;` in
  `dispatch_gone_burst` — latent trap only if a `cb==NULL` detector is ever created
  (none today).
- T7: `dsp_feed`/`usbring_reset` race → one-cycle stale ring-count glitch (guarded,
  not a crash); `s_driver_obj.actions` bitmask now has a 3rd non-atomic writer.
- T8: `fi_urb` fault-injection path doesn't restore `s_live_xfers` (test-only drift);
  `esp_libusb_pause_stream` comment still says "frees" (stale).

## Architecture (scanner)

- `p4-usb-host/main/scanner.{c,h}` — control-plane module: `scanner_hop`,
  `scanner_scan`, `scanner_print_last_map`, wired via `scanner_init(s_dsp)` at
  `class_driver.c` (after the production `dsp_processor_create`).
- `p4-usb-host/main/scanner_map.{c,h}` — pure host-tested logic (center enumeration,
  density ranking).
- Retune path: `scanner_hop` → `class_driver_retune(hz)` POSTS `ACTION_RETUNE` +
  waits on a semaphore; the `usb_pump` task (`class_driver_task`) executes the
  quiesced retune: `esp_libusb_pause_stream` (streaming=false, drain to
  `s_live_xfers==0`) → `rtlsdr_set_center_freq` → `esp_libusb_resume_stream`
  (usbring_reset + re-submit the SAME parked transfers, NO re-alloc). Running the
  retune on the pump task is essential — control transfers can't complete while the
  bulk stream is live (that was the original Approach-A hang).
- Density: `dsp_processor` taps `dispatch_gone_burst`, counting bursts with T60
  `width_bins <= 48` (narrowband). `dsp_processor_read_reset_density` /
  `dsp_processor_reset_tagger_baseline`.

## Dev loop / how to continue

- Build: `./scripts/build.sh` — Flash: `./scripts/flash.sh` (SEPARATE invocations;
  stop the serial logger first to free the port).
- Serial control (device on `/dev/ttyACM0`, 115200): commands `hop/scan/map/config/
  set/get/reboot`. Opening the port via pyserial resets the device (CH343) — open,
  wait ~9 s for boot+stream, then send commands to the running stream (that's what
  exercises live retune). Use `dtr=False, rts=False`.
- Serial capture: `scripts/serial_logger.sh run` (auto-reconnects, no reset).
- Device-smoke GOLDEN: `scripts/smoke_run.sh raw` (currently HANGS with T60 — the
  PIE deadlock). sdkconfig is gitignored and gets smoke-tainted by smoke_run; reset
  the `CONFIG_SMOKE_TEST_*` flags to production before a normal build.

## Bench state at handoff

Device running the scanner build, tuned **1622 MHz** (NVS `lo_hz`, targeting the
duplex/ACARS user channels), bias-tee/LNA ON, streaming ~4.9 MB/s, stable. A
lightweight bench monitor is running (scratchpad `bench_monitor.sh`) — wakes only
on a real decode / hard hang / crash-loop; heartbeats to `monitor_heartbeats.log`.

**Still ZERO real decodes** (0 UW locks, 0 real frame types) after hours — the
antenna/RF is the binding constraint, not firmware. Getting the first ACARS needs a
real 1.6 GHz antenna with sky view; see memory `bench-rf-context-iridium-always-in-view`.
