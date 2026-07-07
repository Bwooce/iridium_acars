# Production-viable PIE-ownership re-tiering — design (2026-07-07)

**Scope:** design only, read-only investigation (no edits, no device). Produces an
implementation-ready plan to *permanently* avoid the PIE coprocessor-save wedge on
the ESP32-P4 by guaranteeing **one PIE owner per core by construction**, so the
FreeRTOS lazy owner-swap (`pxPortUpdateCoprocOwner` → `pie_save_regs`/
`pie_restore_regs`) never runs its faulting body.

**Builds on (not re-derived):** `project_pie_save_deadlock_smoke`,
`project_core1_cpu_budget_scheduling`, and the three sibling docs
`2026-07-07-pie-{wedge-plan,regression-archaeology,wedge-research}.md`. Confirmed
prior facts I take as given: the hang is an `esp.vst.128`/`esp.vld.128` Q-register
op wedging inside the coproc owner-swap; **single-PIE-owner-per-core provably
avoids it** (`CONFIG_DIAG_SINGLE_PIE_OWNER` ran smoke RAW to GOLDEN completion,
zero HP-WDT); every save/restore-side software fix (patches 0003/0004/0005/0006)
is falsified; the necessary condition (2 PIE owners on Core 1) was **added** by
commit `baaff6d` (2026-05-22) which PIE-ised the resample. Production is stable
(4 h+); the wedge is smoke-gate / on-device-IQ-replay only.

---

## 0. Bottom line up front

- There are **exactly three active PIE-executing tasks**, on **two** cores →
  pigeonhole forces ≥2 owners on one core unless we merge two into one TCB or
  scalarize one. (`rs_worker_a/b` are dormant dead code — **not** owners; verified.)
- **Scalarizing is not viable for either candidate.** Scalar resample ≈ **250%**
  of a core (confirmed). Scalar tagger FFT ≈ **+36%** Core 0 and, worse, achieves
  nothing on its own (Core 1 still has 2 owners); the only way scalar-tagger helps
  is to *also* relocate resample onto Core 0, which drives Core 0 to **~120–130%**.
  Both rejected on CPU arithmetic (worked below).
- **The clean, proven-mechanism fix is a MERGE, not a pin:** fold the continuous
  front-end PIE work (convert + **resample** + signal_buffer push) into the **same
  TCB that already runs the tagger FFT** — `dsp_feed` on **Core 0** — and retire
  `ingest_core1` as a separate task. Result: **Core 0 = one PIE owner** (dsp_feed:
  convert+resample+tagger), **Core 1 = one PIE owner** (worker_core1, alone). No
  owner-swap on either core → the wedge is eliminated by design, matching the
  configuration already proven to reach GOLDEN.
- **Bonus:** worker_core1 gains a near-dedicated Core 1 (today it shares with
  ingest's 75%), which directly helps the real goal-blocker (worker is the decode
  bottleneck at ~13 bursts/s).
- **The one real risk** is Core 0 CPU headroom (this partially reverts the T48/T49
  cross-core offload). It is a *measurable* risk gated by `/tasks` + `rb_full_drops`,
  with an in-plan rebalance (move `frame_decoder` Core 0→Core 1) if needed.

Recommendation rank: **R1 = merge (option c)** ≫ R2 = ship-with-Smoke-skip
(status quo) ≫ (rejected) scalarize-tagger, merge-into-worker, PIE-mutex.

---

## 1. The exact PIE-user / task / core / cost table

All three PIE-executing tasks, their affinity (`xTaskCreatePinnedToCore*` args),
priority, PIE kernels, dispatch rate, and per-second PIE CPU cost. Line numbers
verified in the current tree (branch `main`, HEAD `9638816`).

| Task | Core | Prio | PIE kernel(s) it runs | Dispatch rate | PIE CPU cost | Create site |
|---|---|---|---|---|---|---|
| **`dsp_feed`** (tagger) | **0** | **6** (`my_prio−1`, pump=7) | `fft_sc16_2048` → `dsps_fft2r_sc16_arp4` (8-lane Q15 FFT) | **1221 FFT/s** = 2.5 MS/s ÷ 2048 | **~18% Core 0** for the FFT stage (≈147 µs/FFT) | `class_driver.c:422` |
| **`ingest_core1`** (resample) | **1** | **8** | `resample_125_128_mac_arp4` (`esp.vmulas.s16.xacc` 9-tap MAC) | **~125 dispatch/s** | **~31% Core 1** (125 × 2.5 ms; part of ingest's ~75%) | `ingest_core1.c:641` |
| **`worker_core1`** (decode) | **1** | **4** | `rotate_q15_chunk_arp4` + `dsps_fird_s16_arp4` (RRC/decim FIRs) + `dsps_fft2r_fc32_arp4` (UW correlator, `CORR_USE_FLOAT_FFT=1`) | ~13 bursts/s decoded (tagger offers 70–130/s) | **~76–103 ms/burst**; ~16% Core 1 typical, spikes with burst load | `worker_core1.c:1220` |
| ~~`rs_worker_a`~~ | 0 | 7 | **DORMANT — never runs PIE** | 0 | 0 | `ingest_core1.c:670` |
| ~~`rs_worker_b`~~ | 1 | 7 | **DORMANT — never runs PIE** | 0 | 0 | `ingest_core1.c:678` |

**Sources / anchors:**
- Tagger core/prio: `class_driver.c:418-424` (`feed_prio = my_prio−1`; production
  `usb_pump` = `CLASS_TASK_PRIORITY 7`, `usb_host_lib_main.c:64` → feed = 6).
- FFT rate: `fft_burst_tagger.h:5-6` "2.5 MSPS, 2048-pt FFT every 2048 samples, no
  overlap" → 2.5e6 / 2048 = **1220.7 ≈ 1221 FFT/s**. Executed at
  `fft_burst_tagger.c:1112` / `:1146` inside `dsp_feed`'s `dsp_processor_feed`
  (`class_driver.c:861`/`:892`).
- Tagger FFT duty: `fft_burst_tagger.h:15` "~18% core load at N=2048" (the Q15/PIE
  path); PIE FFT is "~3× the ANSI scalar" (`fft_sc16_2048.c:2`).
- Resample core/prio: `ingest_core1.c:641-642` (Core 1, prio 8). PIE MAC at
  `ingest_core1.c:374` (`resample_256_to_250_process_explicit` →
  `resample_125_128_mac_arp4`, gated `resample_256_to_250.c:59`,
  `!defined(RS25_DISABLE_PIE_ASM)`). Cost: `ingest_core1.c:665-669`
  "~125 dispatches/s × 2.5 ms = ~30% duty cycle."
- Worker core/prio: `worker_core1.c:1220` (Core 1, prio 4). Cost:
  `worker_core1.c:50` "~13 bursts/s (~76 ms each)"; `:1198` "~85 ms/burst."
- `rs_worker` dormancy: spawned at `ingest_core1.c:670/678` but the only
  `xTaskNotifyGive(s_worker_*.task)` is at `:483-484`, **inside the
  `split_pct>0` branch** which never executes (`s_split_pct=0` hardwired,
  `ingest_core1.c:90`; "Split path retired by T49b," `:395-398`). A task that
  never executes a PIE instruction is never a coproc owner. **Excluded.**
- **No other Core-0 task runs PIE** (verified): `frame_decoder.c` and
  `http_server.c` contain zero `arp4`/`esp.v`/`fft_sc16`/`rotate_q15`/`resample`
  references; `usb_pump` is pure USB. So Core 0's *only* PIE executor today is
  `dsp_feed`.

### Current PIE-owner map (the problem)

```
Core 0:  dsp_feed  ──► fft_sc16_2048            (1 PIE owner)   ← no swap here
Core 1:  ingest_core1 ──► resample MAC     ┐
         worker_core1 ──► rotate/fird/fc32 ┘   (2 PIE OWNERS)   ← the wedge lives here
```

The lazy owner-swap only executes when a core has **two** tasks that both run PIE
and the scheduler switches between them mid-stream. Core 1 has that; Core 0 does
not. `baaff6d` created the Core-1 pair (`ingest_core1.c` resample PIE-isation,
2026-05-22). Everything below aims to return to **one PIE owner per core**.

---

## 2. Data-flow reality (governs which merges are feasible)

The continuous front-end is a **cross-core producer/consumer with 1-deep
pipelining** (T48/T49, `docs/perf-decoupling-design-2026-07-04.md §1`):

```
usb_pump (Core0, prio7)  ── URBs ──►  4 MB PSRAM stream ring
       │
dsp_feed (Core0, prio6)  ── reads ring ─┐
       ├─ ingest_core1_dispatch(raw)  ──┼──► queue s_dispatch ─► ingest_core1 (Core1)
       │                                │                              │  convert uint8→int16  [C3]
       │   (takes PREVIOUS slot back)   │                              │  resample 256→250 PIE [C4]
       └─ ingest_core1_take_converted ◄─┘                              │  signal_buffer_push   [C5/C6]
       └─ dsp_processor_feed()  ── runs tagger FFT (Core0 PIE)
                                                                worker_core1 (Core1) ◄─ signal_buffer (per-burst decode)
```

Key facts (from `class_driver.c:844-892`, `ingest_core1.c:340-388`,
`perf-decoupling-design-2026-07-04.md §1`):

- **`dsp_feed` is the ring consumer** and already round-trips through
  `ingest_core1` every cycle (`dispatch` → `take_converted` → `feed`).
- **`ingest_core1` = convert (C3) + resample-PIE (C4) + signal_buffer push (C5/C6)**,
  all continuous, streaming, sample-driven.
- **The tagger and the resample already sit on the same logical data path** — the
  only reason resample is on a *different core* is the T48/T49 offload to keep
  `dsp_feed` lean for ring-drain. This is the crucial point: tagger + resample are
  both **continuous** and **already coupled**, so folding them into one TCB is
  natural. The worker is **event-driven per-burst** (blocks on the SNR PQ
  semaphore) — a poor merge partner for anything continuous.

---

## 3. Design question 1 — which single user is cheapest to scalarize? (arithmetic)

### 3a. Scalarize resample — REJECTED (confirmed, re-derived)

PIE resample = ~31% of a core (`ingest_core1.c:665`). The P4 PIE MAC is 8-lane
int16 → ~8× scalar (`feedback_prefer_int16_over_float`). Scalar resample ≈
**31% × 8 ≈ 248% ≈ 250%** of a core — **2.5 cores of work**. Impossible on any
single HP core. This is exactly why the proven `CONFIG_DIAG_SINGLE_PIE_OWNER`
build (which forces scalar resample) is a diagnostic only, not shippable. **Reject.**

### 3b. Scalarize the tagger FFT — REJECTED (worked arithmetic)

- PIE tagger FFT: ~18% Core 0 @ 1221 FFT/s ⇒ ≈ **147 µs/FFT** PIE.
- Scalar FFT ≈ 3× PIE (`fft_sc16_2048.c:2`) ⇒ ≈ **441 µs/FFT** ⇒
  1221 × 441 µs ≈ **539 ms/s ≈ 54% of Core 0** for the FFT stage alone —
  a **+36%** Core-0 increase over today's 18%.
- **But scalarizing the tagger fixes nothing by itself:** the wedge is on **Core 1**
  (resample + worker). Making Core 0 PIE-free leaves Core 1 with two owners. To
  *use* a PIE-free Core 0 you must **relocate resample onto Core 0** as its sole
  PIE owner (worker alone on Core 1). Then Core 0 must carry:
  `usb_pump` + tagger non-FFT stages (window/mag²/EMA) + **54% scalar FFT** +
  **31% resample** + `httpd` + `frame_decoder`.
  Even generously estimating the Core-0 non-FFT base at 35–45%, the total lands at
  **≈ 120–130% > 100%. Infeasible.**
- Robust to the source discrepancy: `fft_sc16_2048.c:11-12` mentions a "~1.5 ms FFT
  speedup," which would imply scalar FFT ≈ 1.65 ms × 1221 ≈ **200% of a core** —
  i.e. even *more* infeasible. The 18%/3× anchor (self-consistent, and the explicit
  budget figure in `fft_burst_tagger.h:15`) is the one I use; the alternative only
  strengthens the rejection. **Reject either way.**

### 3c. Scalarize the worker — REJECTED (essential)

The worker IS the decode pipeline (D13/CFO/RRC/UW/QPSK/BCH). Its FIRs and
correlator FFT are the reason PIE exists here and it is already the throughput
bottleneck at ~76–103 ms/burst. Scalarizing it would multiply the bottleneck and
gut decode. **Reject.**

**Conclusion Q1/Q2:** no single user can be scalarized within budget. The fix must
be *topological* (one PIE owner per core via merge), not by scalarizing.

---

## 4. Design question 3 — architectural alternatives (feasibility)

### (a) Pin resample→Core 0, worker→Core 1 (pure affinity move) — REJECTED

Moving resample to Core 0 while it remains a **separate task** from `dsp_feed`
makes Core 0 host **two** PIE tasks (tagger + resample) → the owner-swap simply
**moves to Core 0**; the wedge relocates, not disappears. It only works if the
tagger is simultaneously scalarized (§3b, infeasible) *or* resample is merged into
the tagger's TCB (that is option (c), below). Pure pinning does not achieve
single-owner. **Reject as stated.**

### (b) Merge resample into `worker_core1` (Core 1) — REJECTED (structural mismatch)

Would give Core 1 = worker{decode + resample} one TCB, Core 0 = tagger alone.
But:
- **Continuous vs event-driven mismatch:** the worker blocks on the SNR priority
  queue and runs in ~76–103 ms bursts ~13×/s; resample must run continuously at
  125 dispatch/s tracking the USB stream. Interleaving a continuous 31%-duty
  streaming job into an event-driven per-burst task requires restructuring the
  worker into a polling super-loop — large, risky churn on the most
  correctness-sensitive task.
- **Throughput:** concentrating resample + decode on one core re-creates the
  contention `frame_decoder`'s #123 move was fighting, and steals from the
  worker's already-tight budget.
- **Data flow:** the worker doesn't produce the resampled stream the tagger
  consumes; resample feeds `signal_buffer` which both the tagger *and* worker read.
  Putting the producer inside one consumer is backwards. **Reject.**

### (c) Merge resample (+convert +push) into the tagger task `dsp_feed` (Core 0) — RECOMMENDED

Fold C3/C4/C5 (convert + resample-PIE + signal_buffer push) **inline into
`dsp_feed`** — the task that already takes the converted slot back and runs the
tagger FFT — and **retire the separate `ingest_core1` task and its dispatch
queue**. Result:

```
Core 0:  dsp_feed ──► convert + resample-PIE + push + tagger-FFT   (ONE TCB = ONE PIE owner)
Core 1:  worker_core1 ──► rotate/fird/fc32   (ALONE = ONE PIE owner)
```

Because convert+resample+tagger now run in a **single TCB**, the PIE owner on
Core 0 never changes → `pxPortUpdateCoprocOwner` never runs its save/restore body
on Core 0. Core 1 has a single PIE task (worker) → no swap there either. **This is
exactly the single-owner-per-core configuration already proven to reach GOLDEN**,
achieved *without scalarizing anything*.

- **Why it's the cleanest:** tagger + resample are both continuous and already on
  the same data path; the merge *removes* the cross-core dispatch/take/release
  handshake and the 1-deep double-buffer, simplifying the hot loop. It is
  substantially a controlled partial-revert of the T48/T49 cross-core split
  (git history can guide the diff), retaining the PIE asm.
- **Latency/throughput impact:** the convert+resample that ran concurrently on
  Core 1 now runs serially in `dsp_feed` on Core 0 before the tagger step. Net
  Core-0 duty rises ~31% (resample) + convert (memcpy loop, ~few %). Worker gains a
  near-dedicated Core 1 (ingest's ~75% leaves) — a real decode-throughput win.
- **The gating risk is Core-0 CPU** (see §7). It is measurable and mitigated.

### (d) Make the two Core-1 PIE tasks mutually non-preemptible (same-prio cooperative / PIE mutex) — REJECTED (evidence)

The owner-swap does **not** fire mid-kernel; it fires on the *incoming* task's
**first PIE instruction after any context switch** away from the other owner.
`vTaskSuspendAll` already failed because it defers only the scheduler while
**every interrupt entry still juggles coproc state** (`project_pie_save_deadlock_smoke`,
wedge-plan §2). A same-priority cooperative pairing or a PIE-ownership mutex cannot
prevent the hardware lazy-swap that a tick/USB interrupt-driven switch triggers —
the two tasks are still *distinct owners*, so the swap body still runs. Per-kernel
`csrci mstatus` interrupt-masking makes a *kernel* un-preemptible but does not stop
the swap that occurs later when the other task next uses PIE. The only robust way
to stop the swap is to make the two consumers the **same owner (same TCB)** — i.e.
option (c). **Reject.**

---

## 5. Design question 4 — recommendation & ranking

| Rank | Option | Effort | Risk | CPU headroom outcome | Wedge mechanism |
|---|---|---|---|---|---|
| **R1** | **(c) Merge convert+resample into `dsp_feed` (Core 0); worker sole owner Core 1** | **Medium** (partial T48 revert; keep PIE asm) | **Medium** — Core 0 saturation / `rb_full_drops` | Core 1 frees ~75% (worker win); Core 0 +~35% (must verify <~90%) | **Proven** — single owner/core = no swap (reached GOLDEN) |
| R2 | Ship freq-scanner **with honest `Smoke-skip`** (status quo; keep patches 0003/0004) | None | Low | unchanged | Accepts the gate gap; production stable |
| ✗ | (b) Merge resample into worker | High | High | Bad (re-contends Core 1) | would work but structurally wrong |
| ✗ | (a) Pin resample→Core 0 (separate task) | Low | — | — | **does not fix** (swap moves to Core 0) |
| ✗ | Scalarize tagger and relocate resample to Core 0 | Medium | — | **~120–130% Core 0** | infeasible on CPU |
| ✗ | Scalarize resample | Low | — | **~250%** | infeasible on CPU |
| ✗ | (d) PIE mutex / cooperative non-preempt | Medium | High | — | falsified (vTaskSuspendAll precedent) |

**Recommended: R1.** It is the lowest-risk option that (i) yields one PIE owner
per core *by construction* (the only mechanism ever proven to avoid the wedge),
(ii) needs no scalarization, (iii) has *positive* CPU headroom on the previously
contended Core 1, and (iv) is validated by an already-green deterministic gate
(single-owner reached GOLDEN). Its single risk (Core 0 headroom) is measurable and
carries an in-plan mitigation.

If, after implementation, Core 0 headroom cannot be met even with the §7
rebalance, fall back to **R2** (ship with `Smoke-skip`) — production is stable and
the wedge is smoke-gate-only.

---

## 6. Implementation sketch (files / tasks)

Task-topology change only — **the PIE kernels themselves are untouched, so the DSP
numerics are bit-identical** (this is a regression-safe topology move, not a DSP
change). Keep the existing heap-position early-alloc dance intact.

1. **`p4-usb-host/main/class_driver.c` — `dsp_feed_task` (Core 0):**
   - Replace the `ingest_core1_dispatch(...)` → `ingest_core1_take_converted(...)`
     → `dsp_processor_feed(...)` → `ingest_core1_release(...)` handshake
     (`:844`, `:861`, `:892`) with an **inline** call sequence:
     convert (C3) → `resample_256_to_250_process_explicit` (C4, keeps PIE) →
     `signal_buffer_push` (C5) → `dsp_processor_feed` (tagger).
   - Keep `sd_capture_write(raw, ...)` (`:838`) pre-conversion (unchanged tap).
   - Keep `feed_prio = 6` (> httpd 5) so ring-drain still preempts httpd — this is
     the `rb_full_drops`/#91 invariant and must not regress.
   - The PIE early-alloc already lives here (`resample_256_to_250_alloc_coeffs()`
     `:333`, `fft_sc16_2048_init()` `:334`) — leave in place; both PIE consumers
     stay on Core 0 exactly as `project_heap_position_decode_bug` requires.

2. **`p4-usb-host/main/ingest_core1.c` — retire the task:**
   - Extract the convert+resample+push body (the loop at `:340-388`) into a plain
     callable function (e.g. `ingest_core1_process_inline(slot, raw, n)`), invoked
     synchronously by `dsp_feed` on Core 0.
   - **Delete the `ingest_task` spawn** (`:641`) and the `s_dispatch` queue
     (`:633`); no more cross-core round trip.
   - **Delete the dormant `rs_worker_a`/`rs_worker_b` spawns** (`:670`, `:678`) and
     the dead split path (`:395`+, `:483-484`). This removes the *latent* second-
     owner risk (a Core-0 `rs_worker_a` that would become a 2nd Core-0 owner if the
     split path were ever re-enabled) — important hygiene for a "by construction"
     guarantee.

3. **`p4-usb-host/main/worker_core1.c` — unchanged** (stays Core 1, prio 4), now
   the **sole** PIE owner on Core 1.

4. **Optional rebalance — `p4-usb-host/main/frame_decoder.c`:** if §7 shows Core 0
   over budget, move `DECODER_CORE 0 → 1` (`:44`). Core 1 now has ample room
   (ingest's 75% is gone) and both worker + frame_decoder are event-driven, so this
   does not re-create the #123 preemption. **Caveat:** #123 moved it to Core 0 for
   an alloc/stack reason — keep its `MALLOC_CAP_SPIRAM` stack; only the core
   affinity changes. Re-check the tagger-init fragmentation note before flipping.

5. **Pre-req sanity checks before coding:**
   - Reconfirm `frame_decoder` / `http_server` execute **no** PIE (done: zero
     `arp4`/`esp.v` refs) so Core 0's only PIE task remains `dsp_feed`.
   - Confirm `dsp_processor_feed`'s internal path (tagger) is the *only* PIE inside
     `dsp_feed` besides the newly-inlined resample.

---

## 7. CPU-budget validation & headroom proof (gating)

The design's only risk is Core 0 saturation. Prove headroom empirically before
trusting it:

- **Query `/tasks`** (`vTaskList` + `vTaskGetRunTimeStats`,
  `project_core1_cpu_budget_scheduling`) under bench load (≥4.5 MB/s, tagger
  emitting 70–130 bursts/s):
  - **Core 0 total < ~90%, IDLE0 > 10%** with the merged `dsp_feed`.
  - `dsp_feed` must keep draining the ring: **`rb_full_drops` == 0** and
    `ingest_core1_take_converted_slow_waits` gone (the handshake is removed).
  - **Core 1:** worker unstarved; IDLE1 healthy (ingest's 75% freed).
- If Core 0 ≥ 90% or `rb_full_drops` > 0 → apply the §6.4 rebalance
  (`frame_decoder` → Core 1) and re-measure. If still over → fall back to R2.

**Predicted budget (to be confirmed on device):**
- Core 0 today ≈ usb_pump + tagger(all stages) + httpd + decoder. Merge adds
  resample ~31% + convert (few %). Estimated post-merge Core 0 ≈ 85–100% — **tight,
  hence the measurement gate and the rebalance option.**
- Core 1 today ≈ 99% (ingest 75% + worker 16% + friends). Post-merge Core 1 ≈
  worker + daemon + status_logger + sd_capture ≈ **well under 50%** — comfortable,
  and better for decode.

---

## 8. Smoke-gate validation plan

The deterministic smoke RAW GOLDEN gate is the authoritative test (host cannot
reproduce — it doesn't compile the asm; `wedge-plan §5`). Single-owner already
proved the mechanism; this design must show the *no-scalar* single-owner build
both **passes GOLDEN** and **hits the perf bar** (unlike the scalar diagnostic,
which failed only on the expected 35658 µs > 4500 µs perf line).

1. **Primary — smoke RAW GOLDEN ×3 consecutive:** each run must reach GOLDEN eval,
   **`matched ≥ 40`**, and **zero HP-WDT / `rst:0x7`** and zero SW resets.
   (Every pre-fix RAW run wedged at the 2nd burst callback; the single-owner
   diagnostic ran to completion — this build should do both *and* pass perf since
   resample stays PIE.)
2. **REAL smoke GOLDEN** likewise green.
3. **Regression parity:** decode `matched`/recall vs the current baseline must be
   **unchanged** — the merge is a topology change with bit-identical kernels; any
   delta signals an accidental data-path bug (ordering/latency), not DSP.
4. **Production soak ≥ 2 h** at the bench LO: zero PIE panics
   (`Coprocessors must not be used in ISRs!`), zero `worker_cap` spike →
   SW_CPU_RESET, sustained 4.88 MB/s / 0 `rb_full_drops`, decode rate ≥ baseline.
5. **`Smoke-verified:` trailer** on the commit (mandatory per
   `feedback_device_smoke_mandatory_after_dsp` / `.githooks/pre-push`) — now
   honestly earnable on the RAW/REAL variants for the first time, which also
   unblocks on-device IQ replay (phaseb-slice validation).

**Gotchas to carry (cost hours previously, `wedge-plan §5`):** `smoke_run.sh`
leaves `CONFIG_SMOKE_TEST_MODE=y` in the gitignored sdkconfig — restore before any
production build; its `/tmp/smoke_run/sdkconfig.prod.bak` can be stale (re-apply
config changes to it). Always confirm *which* failure you're seeing (an early
alloc failure once masqueraded as a "cure").

---

## 9. Risks & mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| **Core 0 saturates → `rb_full_drops` returns (#91)** | Medium | `/tasks` gate (§7); keep `dsp_feed` prio 6 > httpd 5; rebalance `frame_decoder`→Core 1; else fall back to R2 |
| **`frame_decoder` starves on a busy Core 0 → WDT** (old 10 dB-WDT mode, `feedback_task_wdt_subscription_pitfalls`) | Low–Med | Move `frame_decoder`→Core 1 (§6.4); it's event-driven and Core 1 is now free |
| Serializing convert+resample+tag adds pipeline latency vs the old overlap | Low | Bounded (few hundred µs/step); watch tagger step timing + `dsp_feed` cycle stats; worker is the throughput limiter, not the tagger |
| Heap-position / PIE-buffer fragmentation shifts (`project_heap_position_decode_bug`) | Low | Keep the existing early-alloc order in `class_driver.c:333-334`; removing ingest's internal buffers *frees* internal SRAM (net favorable) |
| Latent re-enable of the dead split path resurrects a 2nd owner | Eliminated | Delete `rs_worker_a/b` + split path (§6.2) |
| Merge accidentally alters sample ordering/decimation phase | Low | Parity gate (§8.3): matched/recall must be identical |

---

## 10. Why this is the right stopping point

- The mechanism is **proven**, not hypothesized: single-owner-per-core is the one
  configuration that reached GOLDEN. R1 achieves it without the scalar-perf
  penalty that made the diagnostic build unshippable.
- Every cheaper save/restore-side lever is exhausted (0003/0004/0005/0006, IDF
  bump — all falsified in the sibling docs). Re-tiering is the remaining path the
  prior sessions explicitly named; this doc makes it concrete and bounds its risk.
- It is *net positive* for the real goal (worker gets a dedicated core → more
  decode headroom), so it earns its keep even beyond greening the smoke gate.

**Deliverable status:** design complete, no code changed. Implementation is a
single focused branch: merge in `class_driver.c` + `ingest_core1.c`, optional
`frame_decoder.c` core flip, then the §7/§8 gates. Author on top-tier +
independent top-tier review of the `dsp_feed` hot-loop change (it touches the USB
ring-drain invariant), per the sibling-plan agent guidance.
```

