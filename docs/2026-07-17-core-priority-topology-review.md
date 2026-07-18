# Core-assignment & priority topology review (ESP32-P4 Iridium SDR)

**Date:** 2026-07-17
**Scope:** FreeRTOS task pins, priorities, yields, and lock scope in
`p4-usb-host/main/` + `common/iridium_decoder/`, against the measured problem:
Core-1 worker saturation during passes (pk_wk 130–197 %, ~1 k stale-drops/h)
lapping multi-burst ACARS continuations out of the 3.36 s sample ring.
**Method:** every claim checked against source at the cited file:line.
Anything not verifiable from source (mostly live CPU percentages) is marked
UNVERIFIED. Build config from `p4-usb-host/sdkconfig`
(STANDALONE role, sdkconfig:1312; tick = 100 Hz, sdkconfig:2978; TASK_WDT 60 s
with idle tasks NOT subscribed, sdkconfig:2498–2504; console UART0 @ 115200,
sdkconfig:2414/2418).

**Verdict up front:** the topology is fundamentally sound and most of the
folklore fixes have already been made (frame_decoder is already on Core 0; the
prio-9 tagger helper is already disabled; the one-PIE-owner rule is honoured).
Core-1 worker saturation is a **demand** problem (worker demand alone is
130–197 % of a core at peak), not a priority-inversion problem — no
re-pinning or re-prioritisation can conjure the missing half-core. The real
topology-level findings are: (1) a genuine, removable CPU leak — the console
UART busy-wait at 115200 baud burns CPU in whichever task logs, including the
worker itself and the prio-6 status_logger above it; (2) several stale
priority comments that have already misled project memory; (3) two armed
landmines (dead resample-worker tasks that would violate the one-PIE-owner
rule if ever revived; sd_capture at prio 5 above the worker during captures).

---

## 1. Task inventory

### 1.1 Application tasks (production STANDALONE build)

Role-gated tasks that do NOT run in this build (`CONFIG_DEVICE_ROLE_STANDALONE=y`,
sdkconfig:1312): `flink_slave`/`flink_master` (frame_link.c:308/341, prio 6 core 1),
`agg_ingest` (aggregator_ingest.c:88, prio 4 core 1), smoke task (usb_host_lib_main.c:329,
`CONFIG_SMOKE_TEST_MODE` only). Excluded below.

| Task | Core | Prio | Stack (loc) | Created at | Run pattern | PIE? |
|---|---|---|---|---|---|---|
| `ingest_core1` | 1 | **8** | 8192 internal | ingest_core1.c:641-642 | Event: blocks on `s_dispatch` queue (ingest_core1.c:256); ~305 wakes/s @ 5 MB/s (16 KB dispatches); per wake: AGC scan + uint8→int16 convert via 1024-complex internal tile + memcpy (Path A: **no resample**, ingest_core1.c:375-386) + `signal_buffer_push` (blocks on prior DMA) | No (Path A removed the PIE resample MAC from this path) |
| `rs_worker_b` | 1 | 7 | 4096 PSRAM | ingest_core1.c:678-681 | **DEAD**: blocks forever on `ulTaskNotifyTake` (ingest_core1.c:166); split path unreachable — `s_split_pct` hardwired 0, no setter exists (ingest_core1.c:394-399) | Would be (resample_arp4.S) if revived |
| `status_logger` | 1 | **6** | 6144 PSRAM | status_logger.c:542-545 | Periodic 1 Hz: `xQueueReceive` 1100 ms timeout (status_logger.c:513-520); per wake: format + ESP_LOG (UART!) + `iot_log_poll` | No |
| `sd_capture` | 1 | **5** | 6144 PSRAM | sd_capture.c:50-51,439-443 | Blocking `xStreamBufferReceive`; idle unless raw-IQ capture active; during capture: SD writes at stream rate | No |
| `worker_core1` | 1 | **4** | 16384 PSRAM | worker_core1.c:1450-1451 | Blocks on `s_pq_items` (worker_core1.c:1082); per burst: extract+rotate+decim+prefilter+demod, up to ~hundreds of ms; `vTaskDelay(1)` every 8 processed bursts (worker_core1.c:1330-1334) | **Yes** — rotate_to_dc_arp4, direct_if_decim FIR, fft_sc16_2048 (prefilter burst_prefilter.c:219-220, UW correlator), fft_burst_tagger_arp4 kernels |
| `daemon` (USB host lib) | 1 | **4** | 4096 internal | usb_host_lib_main.c:52,388-396 | Event-driven `usb_host_lib_handle_events`; mostly blocked. USB DWC ISR (~300/s) registered on Core 1 (usb_host_lib_main.c:381-387) | No |
| `agc` | 1 | 3 | 3072 PSRAM | agc.c:85-91 | 1 Hz periodic, µs-scale work | No |
| `usb_pump` (class_driver) | 0 | **7** | 4096 internal | usb_host_lib_main.c:66,398-405 | Event loop: `usb_host_client_handle_events` (100 ms cap, class_driver.c:616); URB completion callbacks run here → 16 KB CPU memcpy into 4 MB PSRAM usbring per URB (esp_libusb.c:206-241); 1 Hz snapshot post | No |
| `dsp_feed` | 0 | **6** (pump−1) | 4096 internal | class_driver.c:484-490 | Continuous while data: ring peek → dispatch to ingest → `dsp_processor_feed` (tagger FFT) per 16 KB block (class_driver.c:904-1072); blocks on notify only when ring empty (class_driver.c:1057) | **Yes** — fft_sc16_2048 + fft_burst_tagger_arp4 mag/detect |
| `frame_decoder` | **0** | **6** | 6144 PSRAM | frame_decoder.c:68-92,752-757 | Poll loop: non-blocking `frame_queue_pop`; ONE frame per wake then `taskYIELD()`; `vTaskDelay(1)` when empty (frame_decoder.c:674-722) | No — IDA/SBD/libacars parsing is scalar (BCH runs in worker context, worker_core1.c:866+) |
| `httpd` | 0 | **5** | 12288 | http_server.c:2719,2730-2731 | Event-driven, socket-bound | No |
| `ota` | 0 | 5 | 8192 internal | ota_runner.c:215-216 (deliberately pinned Core 0, comment :211-214) | Transient, user-triggered; stream is maintenance-paused during OTA (class_driver.c:710-728) | No |
| `sd_log` | 0 | 3 | 6144 PSRAM | sd_log.c:62-63,547-550 | 1 Hz flush + per-message dequeue | No |
| `acars_push` | any | 3 | 4096 PSRAM | acars_push.c:273-276 (tskNO_AFFINITY) | Event: queue depth 16 (acars_push.c:32), UDP send per msg | No |
| `captive_dns` | any | 3 | 3072 PSRAM | captive_dns.c:194-197 | Blocking UDP/53 | No |
| `serial_cmd` | any | 3 | serial_cmd.c:468 (unpinned) | Blocking UART read | No |
| `autotune_sched` | any | 2 | 4096 PSRAM | autotune_sched.c:138-140 | Sleeps; dwells are vTaskDelay | No |
| `health_wdt` | any | 2 | 4096 PSRAM | wifi_link.c:393-395 | Periodic; sole USB-stall reboot authority | No |
| One-shot HTTP/config helpers | any | 4–5 | 4096–6144 | http_server.c:1162 (`nvs_save` 5), :1831 (4), :1864 (`scan_test` 4), :1931/:1965/:2141/:2613 (5), :2002 (4); c6_ota.c:154 (5); autotune.c:94 (`at_persist` 5) | Transient, seconds-scale, mostly reboot-bound | No |

### 1.2 IDF-created tasks

| Task | Core | Prio | Notes |
|---|---|---|---|
| `esp_timer` | 0 | 22 | sdkconfig:2550-2552 (affinity CPU0); prio is IDF-fixed |
| esp_hosted SDIO threads | any | **23** | port_esp_hosted_host_os.h:64-66 (`DFLT_TASK_PRIO 23`), created via plain `xTaskCreate` (port_esp_hosted_host_os.c:190/197) → unpinned. I/O-blocked; can briefly preempt anything on either core when SDIO traffic flows |
| `tiT` (lwip) | any | 18 | sdkconfig:3230, 3418 (no affinity) |
| `Tmr Svc` | — | 1 | sdkconfig:3010 |
| IDLE0/IDLE1 | 0/1 | 0 | **Not** TASK_WDT-subscribed (sdkconfig:2503-2504) |
| `ipc0`/`ipc1` | 0/1 | 24 | IDF default (UNVERIFIED value, standard) |

### 1.3 Memory-note corrections (project memory is stale — code wins)

| Memory claim | Code reality |
|---|---|
| worker prio 5 | **prio 4** since the "goldilocks" change (worker_core1.c:1440-1449); memory's `project_core1_cpu_budget_scheduling` is pre-#123 |
| frame_decoder Core 1 prio 4 | **Core 0 prio 6** since #123 / 2026-05-31 and the 2026-07-08 WDT fix (frame_decoder.c:69-92) |
| status_logger prio 1 | **prio 6, Core 1, above the worker** (status_logger.c:536-545) |
| ingest ~75 % of Core 1 | that figure included the 125/128 resample; **Path A removed it** (ingest_core1.c:375-386). Current ingest duty is convert+memcpy+DMA only — estimate 10–20 % (UNVERIFIED; measure §7) |
| "fbt_pipe (9)" on Core 1 | pipeline helper **disabled** since 2026-05-25 (fft_burst_tagger.c:512-525); the worker-create comment (worker_core1.c:1445-1448) describes history, not present |
| dsp_feed "prio 6 = caller prio-1, class_driver.c:488" | correct (pump 7 → feed 6) |

Stale in-source comments worth fixing when next touching these files (they are
what corrupted the memory notes): ingest_core1.c:638-640 ("worker_core1 (5)"),
status_logger.c:535-541 ("frame_decoder (4) and worker (3)"),
worker_core1.c:1324-1325 ("same-prio tasks (frame_decoder, both at 4)" —
frame_decoder is on the other core now), signal_buffer.c:52 ("producer …
Core 0" — the producer `ingest_task` is Core 1).

---

## 2. PIE-ownership audit (hard constraint)

Tasks that actually execute PIE vector code today:

- **Core 0: `dsp_feed` only.** `dsp_processor_feed` → `fft_burst_tagger_step`
  → `fft_sc16_2048` + fft_burst_tagger_arp4.S kernels. `usb_pump` is plain
  memcpy/event code; `frame_decoder`'s call tree (ida/ibc/ira/ims/tl decode,
  sbd_reassembler, libacars, msg_ring — frame_decoder.c includes, :1-40) is
  scalar; BCH runs in the **worker's** context inside `worker_emit_frame`
  (worker_core1.c:866+), not in frame_decoder.
- **Core 1: `worker_core1` only.** rotate_to_dc_arp4.S, direct_if_decim FIR,
  `fft_sc16_2048` (burst_prefilter.c:219-220, UW correlator), all inside the
  single worker task.

**Rule is honoured — exactly one PIE owner per core.** Two landmines:

1. **`rs_worker_a` (Core 0, prio 7) and `rs_worker_b` (Core 1, prio 7)**
   (ingest_core1.c:670-685) execute `resample_256_to_250_process_explicit`
   (PIE MAC, resample_arp4.S) *if ever notified*. The split path is dead
   (`s_split_pct` hardwired 0, no setter — ingest_core1.c:394-399), so they
   block forever and never touch PIE. But if anyone revives the split, **both
   cores instantly get a second PIE owner** (rs_worker_a vs dsp_feed on
   Core 0; rs_worker_b vs worker on Core 1) — the exact coprocessor-lazy-save
   wedge class. They also burn 2×4 KB PSRAM and clutter `/tasks`.
   Recommendation: delete the two spawns (and the split branch, or
   compile-time-gate it) at the next ingest_core1.c touch.
2. Smoke build only: the smoke task (Core 0, prio 5) drives DSP directly —
   the known smoke-only `rtos_save_pie_coproc` hang. Unchanged, production-clean.

Any proposal below that moves work between cores was checked against this
rule; none moves PIE code.

---

## 3. Core 1 starvation analysis

Priority ladder actually present on Core 1 during streaming:
`ingest(8) > rs_worker_b(7, dead) > status_logger(6) > sd_capture(5) >
worker(4) = daemon(4) > agc(3) > idle`, plus unpinned floaters
(esp_hosted 23, lwip 18, transient config helpers 4–5).

**What preempts the worker, quantified by run pattern:**

| Preemptor | Wakes | Cost per wake | Duty | Verdict |
|---|---|---|---|---|
| `ingest` (8) | ~305/s (one per 16 KB dispatch from dsp_feed) | convert 8192 complex through 1024-complex tiles + 32 KB memcpy + DMA program/wait | est. 10–20 % (UNVERIFIED — `Ingest (Core1 us avg)` line, verbose build, gives exact convert/push µs) | **Necessary at line rate** (§5); not the lever |
| `status_logger` (6) | 1/s | formatting ~sub-ms **plus UART busy-wait** (§6-F1): quiet build emits STATUS (~250 B) every second and STATUS-ERR (~400 B) whenever dsp>80 % or worker>80 % (status_logger.c:441-485) — i.e. every second of every pass | ~3–6 % during passes at 115200 | **Fixable — F1** |
| `sd_capture` (5) | only during raw-IQ capture | SD write bursts at ~5 MB/s stream rate | 0 normally; material during captures | **Accept, document — F4** |
| `daemon` (4) | USB lib events (enumeration/hub); ISR ~300/s on this core | µs-scale | ≪1 % | fine |
| esp_hosted (23) / lwip (18) floaters | per SDIO packet | µs–ms | small at telemetry rates | fine (OTA case is maintenance-paused anyway) |
| transient prio-5 config helpers (unpinned) | user actions | seconds, usually ending in reboot | negligible | fine |

**Is frame_decoder ever starved during passes?** The question's premise is
stale — frame_decoder is on **Core 0 prio 6** (frame_decoder.c:69-92). Its
historical starvation (prio 4 under the usb_pump(7)↔dsp_feed(6) ping-pong,
60 s TASK_WDT abort, 2026-07-08) is exactly why it sits at 6 with a
one-frame-per-wake + `taskYIELD` discipline (frame_decoder.c:700-712). Under
flood it still gets a same-priority time-slice rotation with dsp_feed every
tick (configUSE_TIME_SLICING=1, FreeRTOSConfig.h:94), i.e. ≥~100 frame-wakes/s
plus every dsp_feed block — comfortably above the worker's emit rate. The
64-slot frame_queue (frame_decoder.c:67) absorbs bursts; `frame_queue_push`
drops (with a counter) rather than blocking the worker
(frame_queue.c:77-106). The reassembler reap clock is now extrapolated RF
time (frame_decoder.c:57-65, 682-696), so decoder lag no longer mis-reaps
chains. No action.

**Is status_logger starved?** No — it is *above* the worker (prio 6), the
opposite of the memory note. The 1100 ms `xQueueReceive` timeout keeps
`iot_log_poll` alive even with no traffic (status_logger.c:513-517).

---

## 4. Core 0 starvation analysis

Ladder: `esp_timer(22) > floaters(23/18) > usb_pump(7) > dsp_feed(6) =
frame_decoder(6) > httpd(5) = ota(5) > sd_log(3) > idle`.

- **dsp_feed at 93–97 %** (memory, consistent with the `DSP: … cap=%`
  telemetry) plus usb_pump's per-URB 16 KB memcpys plus frame_decoder's
  decode slices ≈ Core 0 effectively saturated during passes.
- **httpd(5) starves during passes → /diag timeouts are plausible and
  expected**: prio 5 runs only when everything ≥6 blocks; with dsp_feed
  continuously ready (it only blocks when the usbring is empty —
  class_driver.c:1047-1057), httpd gets the 3–7 % scraps. This is by design:
  the prior rb_full_drops incident (#91) is why the USB consumer must
  outrank httpd (usb_host_lib_main.c:59-65). **Do not raise httpd.**
- **Correctness under httpd starvation:**
  - OTA: `ota` (5, Core 0) runs with the stream *maintenance-paused*
    (class_driver.c:710-728) — dsp_feed goes idle, OTA gets the core. OK.
  - Config/reboot: one-shots are prio 5 unpinned — they can land on Core 1
    (above worker 4) and complete. OK.
  - `health_wdt` (2, unpinned): during a genuine USB stall dsp_feed blocks
    (no data) and CPU frees, so the watchdog can act — its starvation window
    is exactly when it isn't needed. Acceptable, worth knowing.

---

## 5. The central question: why does ingest (8) outrank the starving worker (4)?

Because the 3.36 s ring is **downstream** of ingest, not upstream. The
elastic buffer upstream of ingest is the 4 MB usbring
(esp_libusb.h:73) ≈ **0.8 s** at 5 MB/s. The chain is:

USB URBs → (usb_pump memcpy) → usbring [0.8 s] → (dsp_feed dispatch) →
ingest convert → signal ring [3.36 s] → tagger descriptors → PQ → worker.

If ingest waits behind a saturated worker on Core 1, dsp_feed's
`take_converted` blocks on Core 0 (class_driver.c:1000, unbounded-in-effect —
ingest_core1.c:792-823), the tagger stops stepping, and the usbring fills in
~0.8 s; then `usbring_write` drops whole transfers (rb_full_drops) and the
tagger misses **everything**, not just continuations. During saturation the
worker holds the core for up to ~hundreds of ms per burst with only one
10 ms yield per 8 bursts — ingest below the worker would starve for seconds
per pass. That is strictly worse than stale-drops.

The deeper point: **priority does not change ingest's CPU share.** Ingest's
demand is fixed by the byte rate (~305 bounded dispatches/s); prio 8 only
sets its *latency*. Whether ingest is at 8 or 5, the worker gets the same
number of leftover cycles; inverting it below the worker converts a bounded
latency into unbounded sample loss. The ring "decouples" latency, and it does
— for the worker, which is why the worker can run at 4 at all. There is no
inversion to fix here. (The one theoretically shaveable cost: ingest's
convert is scalar C on a prio-8 task — but it is already tile-fused and
cache-friendly (T49b), and vectorising it would add a PIE user to Core 1.
Don't.)

Sub-questions from the brief:

- **Move frame_decoder to Core 0?** Already done (#123). Confirmed correct.
- **Unpin low-duty tasks?** acars_push, captive_dns, serial_cmd,
  autotune_sched, health_wdt are already `tskNO_AFFINITY`; agc (1 Hz, µs
  work, Core 1 prio 3) and sd_log (Core 0 prio 3) are pinned for good,
  documented reasons (agc reads a Core-1-written volatile at trivial cost;
  sd_log deliberately keeps SDMMC work off Core 1 — sd_log.c:543-547).
  Unpinning agc would buy ~nothing (its cost is ~µs/s) and add cross-core
  cache noise. **Not worth it.**
- **Invert ingest below worker during passes (dynamic)?** No — see above;
  also dynamic priority juggling under load is exactly the thrash this
  review exists to prevent.

---

## 6. Findings & recommendations

### F1 — Console UART busy-wait taxes both hot cores (the one real leak)

The console is UART0 at **115200** (sdkconfig:2414/2418) with no UART VFS
driver mode selected (`uart_vfs_dev_use_driver` is called nowhere), so every
`ESP_LOGx` character exits through `uart_tx_char`, which **spin-waits** for
FIFO space (esp-idf/components/esp_driver_uart/src/uart_vfs.c:186-196). At
115200 that is ~87 µs of pure spin per character once the 128 B FIFO is full.

Who pays, in whose context:
- `status_logger` (Core 1, **prio 6, above the worker**): quiet-mode STATUS
  line every second (status_logger.c:380-387, ~250 B) plus STATUS-ERR
  (status_logger.c:485-506, ~400 B) which triggers whenever dsp>80 % or
  worker>80 % — i.e. **continuously during passes**. ≈650 B/s ≈ 55–60 ms/s of
  Core-1 spin at prio 6 ⇒ **~5 % stolen from the worker at exactly the wrong
  time** (estimate; measure below).
- `worker_core1` itself: `DEMOD SUCCESS` (worker_core1.c:894-897) + BCH
  outcome lines per decoded frame — ~8–12 ms of spin per frame *inside*
  `avg_burst_us`.
- `frame_decoder` (Core 0, prio 6): `FRAME:`/ACARS lines steal the same way
  from dsp_feed's core.

**Recommendation (do now):** raise `CONFIG_ESP_CONSOLE_UART_BAUDRATE`
115200 → 921600 (CH343 handles it; update `scripts/serial_logger.sh` to
match). 8× cheaper ⇒ the ~5 % Core-1 tax drops to <1 %. No code changes, no
behaviour changes, fully compatible with the stash design.
**Risk:** low — bench serial tooling must switch baud; esptool flashing baud
is independent.
**Measure:** `/tasks` (http_server.c:2174-2203 — vTaskList + run-time stats,
CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y, sdkconfig:3024): diff
`status_logger`/`worker_core1` run-time counters over a fixed window before
and after, during a pass; also watch `wk=%` in the STATUS line.
**Alternative rejected:** dropping status_logger below the worker — it would
silence diagnostics precisely at saturation (the regression that put it at 6
in the first place, status_logger.c:536-541).

### F2 — Dead resample workers: PIE landmine + clutter (do after stash)

`rs_worker_a/b` (prio 7, Cores 0/1) are unreachable but spawned
(ingest_core1.c:670-685; dead-path proof :394-399). Zero scheduling cost
today; catastrophic (dual PIE owners on BOTH cores) if the split is ever
re-enabled without re-reading this history. Delete the spawns and the split
branch, or hard-gate them behind a `#if` with a comment citing the
one-PIE-owner rule. Risk: none (code is provably unreachable). Win: removes
the wedge landmine, 8 KB PSRAM, two `/tasks` rows and a false prio-7 signal.

### F3 — Stale scheduling comments actively mislead (do after stash, doc-only)

Fix the four comments listed in §1.3. They have already propagated wrong
priorities into project memory twice. Zero runtime risk.

### F4 — sd_capture (prio 5, Core 1) outranks the worker during captures (document, don't change)

During a raw-IQ capture the SD writer competes *above* the worker
(sd_capture.c:50-51). Lowering it below the worker would overflow the 4 MB
stream buffer (~0.8 s at line rate) at saturation and tear the capture —
captures exist to record exactly these busy windows, so the current priority
is a deliberate fidelity-over-throughput trade. Action: none; just know that
soak numbers taken with a capture running under-report worker capacity.
(A one-line note where WRITER_PRIO is defined would prevent a future
"quick fix".)

### F5 — Worker's `vTaskDelay(1)` per 8 bursts: keep, fix the comment

worker_core1.c:1324-1334. Costs ≤10 ms per 8 *processed* bursts (skips don't
count) ≈ 0.3–1 % at saturation. What it actually buys today: agc (prio 3,
Core 1) and unpinned prio-3 tasks get a guaranteed window; IDLE1 is not
WDT-monitored (sdkconfig:2504) so it is not WDT-required. `taskYIELD()` would
NOT substitute — yield only rotates same-priority tasks (daemon), it never
runs prio 3. Removing it entirely to reclaim <1 % while silently starving agc
for whole passes is a bad trade. Keep; update the stale comment (it names
frame_decoder, which left the core).

### F6 — Yield audit, remaining loops (no changes)

- `dsp_feed`: no explicit yield and none wanted — it must outrun the usbring;
  it blocks naturally on empty ring (class_driver.c:1057) and on the ingest
  slot protocol. Anything below prio 6 on Core 0 is intentionally
  best-effort during passes.
- `frame_decoder`: the ONE-frame-then-`taskYIELD` + `vTaskDelay(1)`-on-empty
  pattern (frame_decoder.c:700-722) is precisely right for an equal-priority
  cohabitant of dsp_feed; `taskYIELD` keeps full drain rate (re-runs on the
  next slice/block) where `vTaskDelay(1)` would cap it at ~100 frames/s.
- `ingest`: fully event-blocked (queue + semaphores); nothing to add.
- `usb_pump`: event-blocked with a 100 ms cap (class_driver.c:608-617);
  deliberate, commented, leave it.
- ESP-IDF semantics for the record: `taskYIELD()` = reschedule among
  same-priority ready tasks on this core, free but useless against
  lower-prio starvation; `vTaskDelay(0)` ≈ yield; `vTaskDelay(1)` = sleep
  until the next tick (≤10 ms at 100 Hz) letting ANY lower-priority task run;
  time-slicing already rotates equal-priority tasks every tick
  (FreeRTOSConfig.h:94).

### F7 — `s_pq_lock` / cross-core lock audit (healthy; one bounded inversion, accept)

- `s_pq_lock` is a FreeRTOS **mutex** (priority inheritance) —
  worker_core1.c:1372. Producer side = `worker_core1_push_burst` running in
  **dsp_feed's context (Core 0, prio 6)** via the tagger callback
  (class_driver.c:446); consumer = worker (Core 1, prio 4).
- Hold times are O(BURST_PQ_CAP=64) linear scans with float priority evals
  (insert worker_core1.c:250-310, extract :314-331) — single-digit µs; no
  I/O, no logging, no allocation under the lock. The push-side stale reject
  and histograms run *before* taking the lock (worker_core1.c:1470-1502). Good.
- One real (bounded, rare) inversion: ingest(8) can preempt the worker while
  it holds `s_pq_lock`; inheritance raises the worker to 6, which does not
  beat 8, so dsp_feed (Core 0) can stall on the mutex for up to one ingest
  dispatch (~ms) on top of the µs hold. Worst case a ~1 ms hiccup in the
  tagger a few times an hour — not worth converting to a spinlock (portMUX
  would disable preemption around a 64-entry float scan and block Core 0 by
  spinning instead; strictly worse).
- `frame_queue` (worker→decoder) and `hot_bin_table` are lock-free
  atomics/seqlock (frame_queue.c, hot_bin_table.c:12-55,
  signal_buffer.c:70-82) — no cross-core blocking on the hot path. Good.

### F8 — Compatibility with the hot-bin sample stash (given)

The stash adds a bounded memcpy in **dsp_feed context (Core 0)** and a read
shim in the worker. Core 0 has only ~3–7 % headroom, so: (a) F1's baud bump
is *complementary* (frame_decoder's UART spin comes out of the same Core-0
budget the stash will draw on); (b) after the stash lands, watch
`rb_full_drops` and `dsp_cap` — if Core 0 tips past 100 %, the stash copy is
the first suspect. Nothing in this review moves work *onto* Core 0, for that
reason. (Idea considered and rejected: relocating per-frame BCH from worker
to frame_decoder — it is scalar so PIE-legal, but the worker is
UW-correlator-bound, `bch=` in the Worker-stages line is small, and Core 0
has no room. Revisit only if measured `bch_us` says otherwise.)

---

## 7. Cheap measurements (all existing, no new code)

1. `/tasks` (http_server.c:2776): per-task run-time counters since boot —
   sample twice, diff, divide by wall time. Quantifies F1 exactly
   (status_logger + worker deltas during a pass at 115200 vs 921600).
2. STATUS line (1 Hz, quiet build): `dsp=%`, `wk=%`, `pk_wk=%`,
   `drops=` (status_logger.c:380-439) — the before/after for any change here.
3. Verbose build (`CONFIG_STATUS_LOG_VERBOSE`): `Ingest (Core1 us avg)` line
   gives convert/push/sbpush µs per dispatch — pins the actual ingest duty %
   (replaces the stale 75 % memory figure).
4. `CONFIG_DIAG_TASK_DUMP` (class_driver.c:778-805): 5 s per-task CPU dumps
   when actively debugging affinity.
5. `hot.cont_stale` vs `cont_pri` in `/diag/reassembler` — already deployed;
   decides whether the stash (ring-age loss) or PQ policy (priority loss) is
   the binding constraint, per the stash design's own gate.

---

## 8. Ranked shortlist

**Do now**
1. **Console baud 115200 → 921600** (sdkconfig + serial_logger.sh). Recovers
   an estimated ~5 % of Core 1 (status_logger spin above the worker) plus
   per-frame spin inside the worker and frame_decoder, during passes —
   config-only, near-zero risk. Validate via `/tasks` run-time deltas + `wk=%`.
2. **Measure, don't assume, ingest duty**: one verbose-build pass to replace
   the stale "ingest ≈75 %" figure before any future Core-1 budget debate.

**Do after stash**
3. Delete/hard-gate the dead `rs_worker_a/b` spawns + split branch
   (PIE-landmine removal, ingest_core1.c:648-685).
4. Fix the four stale scheduling comments (§1.3) so memory stops inheriting
   wrong priorities.
5. Re-check Core-0 headroom (`dsp_cap`, `rb_full_drops`) once the stash's
   dsp_feed-context memcpy is live.

**Don't do (and why)**
- Invert/lower ingest below the worker — converts bounded stale-drops into
  unbounded usbring overflow within ~0.8 s (§5).
- Raise httpd above 5 or dsp_feed above usb_pump — re-opens the #91
  rb_full_drops class.
- Move frame_decoder anywhere — its Core-0/prio-6/yield design is the fix
  for two prior WDT incidents; it is not starving anything measurably.
- Remove the worker's 8-burst `vTaskDelay(1)` — buys <1 %, silently starves
  agc during passes.
- Unpin agc/sd_log or re-pin the low-duty floaters — µs-scale duty, no win,
  adds jitter.
- Convert `s_pq_lock` to a spinlock/portMUX — the mutex hold is µs and the
  inversion window is rarer and cheaper than spinning Core 0 would be.
- Any dynamic (load-dependent) priority scheme — the deficit is demand
  (pk_wk 130–197 %), not arbitration; the stash + A6 boost attack the demand
  side where the signal actually is.
