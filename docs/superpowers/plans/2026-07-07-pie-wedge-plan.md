# PIE coprocessor-save wedge — description & remediation plan (2026-07-07)

## 1. What the wedge is, mechanically

**Symptom.** Under the device-smoke RAW/REAL fixture (a dense ~1/16-realtime IQ
replay), Core 1 wedges with `Core1 Saved PC = rtos_save_pie_coproc`
(esp-idf portasm.S:378) and the hardware watchdog resets the chip
(`rst:0x7 HP_SYS_HP_WDT_RESET`). It fires deterministically **right after the
2nd burst callback**. In production (real-time USB timing) the same machinery
historically produced only rare `Guru: Coprocessors must not be used in ISRs!`
panics (each auto-recovered); the current integrated stack has run **4 h clean
in production** — so today the wedge is effectively **smoke-gate-only**, but it
blocks (a) honest `Smoke-verified` trailers on the RAW/REAL variants and
(b) any on-device IQ-replay validation.

**The machinery.** On the ESP32-P4 the PIE vector unit is **lazily**
context-switched. Every interrupt entry disables PIE; on a switch to a
*different* task PIE stays disabled; the next PIE instruction the incoming task
runs raises an illegal-instruction trap, which routes (via the EXT_ILL CSR
classification in vectors.S) to `rtos_save_pie_coproc`. That routine swaps the
coprocessor owner (`pxPortUpdateCoprocOwner`), saves the *former* owner's
Q0–Q7/QACC/UA_STATE/XACC/SAR with `esp.vst.128.ip`, restores the incoming
owner's, then `mret`s to retry the faulting instruction. **The routine contains
no loop** — so a PC parked there is not a spin; it means either a **trap
re-entry storm** (the retried instruction keeps re-faulting, and because the
trap runs interrupts-masked the tick starves → HP-WDT) or a **wedged 128-bit
vector access** inside the save.

**Key structural facts** (all verified this cycle):
- HWLP (`esp.lp.setup` zero-overhead loops) is a **separate** coprocessor,
  saved *eagerly* at interrupt entry — the PIE trap path never parks it.
- Two P4 silicon bugs are adjacent: `SOC_CPU_HAS_HWLOOP_STATE_BUG` and a
  spurious EXT_ILL HWLP reason bit. Their IDF software workarounds are
  **compiled out** at `CONFIG_ESP32P4_REV_MIN_FULL=100`.
- **Two PIE-owning tasks per core** is what makes the swap machinery run at all:
  Core 0 = class/dsp_feed (tagger `fft_sc16_2048`) + rs_worker_a (resample);
  Core 1 = worker_core1 (burst_pipeline: `direct_if_decim` FIR, `uw_correlator`
  fc32 FFT + RRC FIRs) + rs_worker_b (resample).
- Only **vendored esp-dsp** kernels use `esp.lp.setup` (`dsps_fird_s16_arp4`,
  `dsps_fft2r_fc32_arp4`, `dsps_fft2r_sc16_arp4`). Our own asm
  (`rotate_to_dc_arp4.S`, `resample_arp4.S`) uses vector MACs, not HWLP.

## 2. Ruled out (do NOT re-litigate — evidence in project_pie_save_deadlock_smoke)

- **HWLP removal** (patches/0002, addi/bnez replacing esp.lp.setup; zero
  esp.lp.setup in the linked image, toolchain-proven, bit-equivalent): removed
  the *production* panic site; **smoke still hangs**.
- **REV_MIN=0** (re-enables the silicon-bug workarounds): production soak clean;
  **smoke still hangs**.
- **Coproc save areas → internal RAM** (patches/0003, eager per-task pool
  because the trap can't malloc): mechanism-sound, shipped for production;
  **smoke still hangs with all three active** (2026-07-06 20:17).
- `vTaskSuspendAll` around PIE kernels: no effect — it defers only the
  scheduler; interrupt-entry coproc juggling still happens mid-window.
- Red herrings cleared earlier: coredump-in-smoke, ISR_STACKSIZE, struct growth.
- **No Kconfig exists** to disable/eager-ize lazy PIE save.
- Retracted this session: "patched-kernels × triage interaction" — that
  crash-loop was **smoke-tainted sdkconfig** booting the RAW fixture, not a real
  regression. Production timing is stable.

**Two manifestations are now distinguished:** production panic (the ISR-guard
firing on corrupted state, auto-recovered) vs smoke hard-hang (deterministic
HP-WDT). The three fixes addressed the production side; the smoke hang survives
all of them.

## 3. Surviving hypotheses

**H-A — owner-swap save/restore is the culprit.** The hang only exists because
two PIE owners share a core, so the swap body runs. Something in the
save/restore (re-enable that doesn't stick across `mret` → immediate re-fault →
storm; or owner ping-pong livelock under the dense-replay preemption cadence).
**H-B — PIE execution itself faults under preemption**, independent of the swap
(a genuine silicon corner the dense timing hits). If true, even one owner per
core would still wedge.

These are cleanly separable and NOT yet separated — that is the gating gap.

## 4. Plan

### Phase 0 — DISCRIMINATE first (cheap, highest information; ~1–2 h)

Lesson from four insufficient "mechanism-sound" fixes: stop shipping candidate
fixes; run the discriminator. Two experiments, both mostly pre-built:

0a. **Single-PIE-owner-per-core** (P2 partial work is ~80% done — scratchpad
    `p2_partial_tracked.diff`: a `CONFIG_DIAG_SINGLE_PIE_OWNER` Kconfig +
    `RS25_DISABLE_PIE_ASM` compile flag forcing *both* resample workers onto the
    scalar MAC path, leaving tagger (Core 0) and worker_core1 (Core 1) as the
    sole PIE user per core). Finish it (verify the `RS25_DISABLE_PIE_ASM` gate
    actually exists in resample_256_to_250.c, or add it), build, run smoke RAW.
    - Hang **gone** → **H-A confirmed**: the swap machinery is the fault.
    - Hang **persists** → **H-B confirmed**: PIE execution under preemption.
    Cost note: scalar resample is ~8× the PIE cost; under smoke's 1/16-realtime
    replay this is trivially affordable (it's a diagnostic build, not ship).

0b. **Coredump/exact-PC forensics.** Enable UART coredump for one smoke RAW run
    and `esp-coredump` the wedge: exact faulting instruction, MTVAL, MEPC, and
    whether MEPC is re-entering the same address (storm) vs stuck in a single
    128-bit store (bad access). This directly tests "re-entry storm" vs "wedged
    access" independent of 0a.

### Phase 1 — FIX, branched on the Phase 0 result

**Regardless of branch, ship the diagnostic net first** (P2 partial started it —
the `g_coproc_trap_watch` / `_coproc_trap_storm` symbols in `p2_partial_idf.diff`,
left INCOMPLETE/unlinked): a **bounded retry counter in the trap path** that,
after N consecutive re-faults, panics with the faulting context instead of
wedging silently. This converts every future HP-WDT into an attributable panic —
diagnostic gold even if it isn't the cure, and it makes production strictly safer
(a storm becomes a clean recoverable reboot with a backtrace, not a wedge).

- **If H-A (swap machinery):**
  1. Re-examine patches/0003 for a bug — it was mechanism-sound but the first
     test was a false-cure (failed earlier at dsp_processor_create). Confirm the
     internal-RAM pool is actually being used on the wedge path (add a one-shot
     log; check `sa_intpool != NULL` for worker/rs_worker).
  2. If 0003 is correctly active and it still hangs, the fault is in the
     save/restore *logic* not the memory: the never-fully-tried **IDF portasm
     patch** — check whether PIE re-enable sticks across the `mret` (the enable
     bit write vs the CSR the hardware actually gates on, per-core banked), and
     whether a genuinely-illegal (non-PIE) instruction misclassified via the
     buggy EXT_ILL bit gets routed into the PIE save and loops.
  3. Architectural fallback if the swap itself is unfixable: **make single-owner
     a SHIP config** — permanently pin PIE to one task per core. Requires moving
     resample off PIE on the contended core (CPU-budget audit — Core 1 ingest is
     75%, so this may need re-tiering which task owns PIE, not just scalarizing).

- **If H-B (PIE execution under preemption):**
  1. **Per-kernel interrupt masking** — mask interrupts (`csrci mstatus,8`)
     around each esp-dsp PIE kernel *window* (µs-scale, unlike the whole-region
     vTaskSuspendAll that failed and starved frame_decoder). This makes the
     kernel un-preemptible so no mid-kernel save can occur. Cost: bounded latency
     spikes; measure against the USB/frame-decoder WDT margins.
  2. **IDF bump** — check the esp-idf changelog past our v6.1 pin for P4 PIE
     coproc-save fixes; a newer port may resolve it upstream (weigh against the
     churn of re-vendoring our patches 0001–0003).

### Phase 2 — VALIDATE (the gate that closes #4 and #11's device side)

- Smoke RAW GOLDEN **passes ×3 consecutive** (`matched >= 40`).
- Production soak ≥2 h with zero PIE panics.
- Then RAW/REAL commits carry honest `Smoke-verified` trailers, and on-device
  IQ replay (the phaseb-slice validation) becomes possible.

## 5. Sequencing & agent guidance

- Phase 0 is one focused agent (the P2 partial work + coredump), top-tier or
  mid — it's mostly finishing built work + running a deterministic gate.
- The bounded-retry watchdog and whichever Phase-1 fix are trust-bearing asm on
  the context-switch path → top-tier author + independent top-tier review
  (the libacars review this week proved that pass catches real crashes host
  gates can't).
- **Gotchas that cost hours, encoded so they don't recur:** smoke_run.sh leaves
  `CONFIG_SMOKE_TEST_MODE=y` in the gitignored sdkconfig → always restore before
  a production build (69 smoke banners in a crash-looping log = this). The
  `/tmp/smoke_run/sdkconfig.prod.bak` is reused and can be stale — re-apply
  config changes to it. Always check WHICH failure you're seeing (0003's
  false-cure was an earlier alloc failure, not the wedge). Rotating
  /tmp/p4_serial.log breaks line-offset log windows.
- Host tests can't reproduce this (they don't compile the asm) — smoke GOLDEN is
  the only gate. Do not attempt to gate it on host.
