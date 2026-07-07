# ESP32-P4 PIE coprocessor save/restore hang — research findings

**Date:** 2026-07-07
**Scope:** Read-only research to inform a fix decision. No repo edits, no device work.
**Author:** research agent (Opus 4.8)

## The problem (restated, build-matched)

On ESP32-P4 (RISC-V, dual HP core), our local esp-idf checkout
(`v6.1-dev-5215-g0d92878008`, committed 2026-06-08, at
`/home/bruce/dev/iridium_acars/esp-idf`) hard-hangs a core inside FreeRTOS's
**lazy PIE coprocessor save/restore**. The core wedges on a single PIE 128-bit
Q-register op (`.insn 0x82012{2,6,a,e}3b`, i.e. `esp.vst.128`/`esp.vld.128`)
inside `rtos_save_pie_coproc`
(`esp-idf/components/freertos/FreeRTOS-Kernel/portable/riscv/portasm.S`), on the
fall-through past a `_norestore` branch — the `\restore_coproc_regs` expansion at
`portasm.S:130`, i.e. the `pie_restore_regs` Q-register run at `portasm.S:345-352`
(the 4th consecutive op = `esp.vld.128.ip q3`, `portasm.S:348`). It does **not**
re-fault (no trap storm — the in-tree `xPortCoprocTrapStormCheck` watchdog counts
zero repeat traps) and does **not** run the panic handler (no coredump possible —
the core is genuinely stopped) → HP-WDT reset. It fires when **two PIE-owning
tasks share a core** and the lazy owner-swap runs `pxPortUpdateCoprocOwner`
(`port.c:913`) under dense timing. Our device-smoke fixture reproduces it every
run; production hits it rarely (~1/hr, auto-recovers via SW reset).

Confirmed ruled out (per prior sessions): memory region (save area moved to
internal 16-byte-aligned RAM via patches/0003 and it still hangs), re-entry
storm (trap watchdog saw none), our own DSP kernels. The proven avoidance is
single-PIE-owner-per-core (no swap → the save/restore body never runs), but that
costs ~250% scalar-resample duty and is not shippable as-is.

---

## Q1 — Is this a documented Espressif issue or erratum? **No documented erratum. Evidence of absence, not just absence of evidence.**

### 1a. Official ESP32-P4 SoC errata: no CPU/coprocessor/PIE entry

The complete ESP32-P4 errata summary
([docs.espressif.com/projects/esp-chip-errata/…/esp32p4/02-errata-summary](https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32p4/02-errata-summary/index.html))
lists 13 errata: RMT-176, I2C-308, MSPI-749/750/751, ROM-764/770/816,
Analog-765, DMA-767, APM-560, ECDSA_DS-836/837. **None** relate to the CPU core,
any coprocessor, PIE/SIMD, FPU, hardware loop, or QACC/XACC/SAR/UA_STATE
register save/restore. There is no published silicon erratum covering this
failure.

### 1b. IDF's own P4 CPU-bug flags: exactly two, neither is PIE

`esp-idf/components/soc/esp32p4/include/soc/soc_caps.h` declares only two P4 CPU
"bug" capability macros (verified in our checkout, lines 199-203):

```
soc_caps.h:199  SOC_CPU_HAS_FPU_EXT_ILL_BUG   1   // EXT_ILL CSR doesn't support FLW/FSW
soc_caps.h:201  SOC_CPU_HAS_HWLOOP_STATE_BUG  1   // HWLOOP state doesn't go DIRTY after last loop instruction
soc_caps.h:203  SOC_CPU_HAS_PIE               1   // (capability, NOT a bug flag)
```

There is **no third bug flag** for PIE Q-register / QACC / XACC / SAR / UA_STATE
save/restore. Espressif's own HAL therefore does not model our failure as a known
silicon defect. The two flags that exist both concern the **HWLP** coprocessor
(`esp.lp.setup` hardware loops), not the PIE vector register file we hang in.

### 1c. Adjacent-but-distinct known issue: esp-dsp #102 (HW-loop alignment, "DSP-158")

The closest public report is **esp-dsp issue #102, "Problematic HW Loops on
ESP32-P4"** ([github.com/espressif/esp-dsp/issues/102](https://github.com/espressif/esp-dsp/issues/102),
opened 2025-03-24, rev 1.0 DevKit): on P4 rev 1.0, an `esp.lp.setup` loop-setup
instruction that is **not 4-byte aligned** makes the CPU "go haywire" → PMP
faults, load/store faults, instruction-fetch errors. Workaround: `.balignw 4,
0x0001` (insert a `c.nop`) before the loop setup.

This is **HWLP**, not PIE, and it is an alignment fault, not a stall on a vector
store. It matters for us only as corroboration that (i) the P4 has real,
undocumented instruction-level CPU corners around its custom coprocessors, and
(ii) those corners can be **instruction-alignment sensitive** — a thread worth
keeping in mind for the PIE Q-reg hang (see Q3c), even though #102 itself is a
different mechanism and a different coprocessor.

### 1d. Precedent for coproc-register save bugs in the IDF port (not silicon)

- **esp-idf #11690 "FPU registers not properly switched for ESP32-S3"**
  ([github.com/espressif/esp-idf/issues/11690](https://github.com/espressif/esp-idf/issues/11690))
  — a *software* port bug (wrong registers switched), fixed in the port, not
  silicon. Shows this class of bug historically has been a port defect.
- Our own tree already carries **`b25cb2906c` "fix(freertos): fix xesppie
  registers save/restore"** (Alexey Lapshin, 2025-03-04), which *added the
  previously-missing `q3` save/restore* and repacked XACC/SAR/FFT_BIT_WIDTH.
  Before that fix the port literally skipped `q3`. That is a software
  correctness fix, already applied here (see Q2).

### Q1 verdict

**I could not find any documented Espressif erratum, `soc_caps` bug flag, or
public GitHub issue describing a hang in the PIE Q-register/XACC/SAR
save/restore path.** The only P4 CPU bug flags are HWLP-related; the only public
P4 coprocessor-instruction bug report (esp-dsp #102) is an HWLP alignment fault,
not a PIE vector-store stall. So: this is either an **undocumented silicon
corner** in the PIE register file (plausible — the P4 has undocumented HWLP
corners) or a **latent port-software interaction** that Espressif has not
triaged. There is no vendor acknowledgement to lean on.

---

## Q2 — Has esp-idf master/dev changed the RISC-V PIE save/restore since our checkout? **No. An IDF bump would ship byte-identical PIE save/restore code. The user's skepticism is correct.**

Method: queried `github.com/espressif/esp-idf` master via `gh api` for every
commit touching the two relevant port files, and diffed master's `portasm.S`
against our checkout.

### 2a. `portasm.S` on master is byte-identical to our checkout

`diff` of `master:portasm.S` (raw.githubusercontent.com, fetched 2026-07-07)
against `git show HEAD:…/riscv/portasm.S` in our checkout → **empty diff**. The
lazy PIE save/restore assembly (`pie_save_regs`/`pie_restore_regs`,
`generate_coprocessor_routine`, `pxPortUpdateCoprocOwner` call site) is
**unchanged on master**. The newest master commits that touch this file are ones
we already have:

| SHA | date | subject | in our tree? |
|---|---|---|---|
| `94b526c9fe47` | 2025-06-17 | fix(freertos): Avoid core switch deadlock on start | yes |
| `38abf982161a` | 2026-02-12 | feat: add support for PIE coprocessor on the ESP32-S31 | yes |
| `b25cb2906c7d` | 2025-03-04 | **fix(freertos): fix xesppie registers save/restore** | **yes** |
| `0bc169e73549` | 2025-03-04 | fix(freertos): optimize HWLP context switch by disabling it when unused | yes |
| `c26879d29e52` | 2025-02-18 | fix(freertos): workaround a hardware bug related to HWLP coprocessor | yes |
| `82668dd3feea` | 2024-04-30 | fix(riscv): make HWLP feature use direct saving of lazy saving | yes |
| `55acc5e5e7f1` | 2024-04-30 | feat(riscv): add support for PIE coprocessor and HWLP feature | yes |

Every PIE/HWLP fix that exists on master is **already in our checkout.** The one
squarely on-point (`b25cb2906c`, the "fix xesppie registers save/restore" that
added `q3` and repacked XACC/SAR) is present at `portasm.S:277/348` etc.

### 2b. `port.c` has 3 master-only commits — none touch the PIE register path

Master `port.c` has three commits newer than our checkout (verified NOT ancestors
of our HEAD):

| SHA | date | subject | relevance |
|---|---|---|---|
| `3a18ec3b7fc9` | 2026-06-18 | feat: make FREERTOS_PORT_THREAD_SAFE_CLAIM optional | SMP task-claim gating — not coproc |
| `90f6eeb32785` | 2026-06-03 | feat: introduce thread-safe context management functions | SMP context-claim API (portmacro + tasks.c); adds claim/release, does **not** change `pxPortUpdateCoprocOwner`/`pxPortGetCoprocArea` or the save/restore path |
| `104f8ff55b00` | 2026-06-08 | fix: ZCMP workaround — restore RISC-V interrupt threshold | interrupt-threshold CSR, unrelated |

None of these alter `pie_save_regs`, `pie_restore_regs`, `pxPortUpdateCoprocOwner`
(`port.c:913`), or `pxPortGetCoprocArea` (`port.c:834`). The "thread-safe context
management" work is about SMP task-context *claim* races on startup/deletion, not
the coprocessor register file we hang in.

### Q2 verdict

**An IDF bump to current master would deliver the exact same PIE Q-register
save/restore instructions we already run** (`portasm.S` identical) and no change
to the coproc owner/area logic in `port.c`. The evidence does **not** support an
IDF bump as a fix for this hang. The only way a future IDF bump helps is if
Espressif *later* lands a new erratum workaround — and per Q1 there is no such
erratum on record yet, so there is nothing queued. Recommendation: do **not**
spend effort on an IDF bump for this. (A bump may be worth doing for *other*
reasons, but it will not move this needle.)

---

## Q3 — Avoidance strategies that don't pay the single-owner CPU cost

### (a) Force EAGER (non-lazy) coproc save — **not available as Kconfig, and would NOT avoid the hang**

- **No Kconfig exists.** `grep` across `components/*/Kconfig*` finds no
  lazy/eager coprocessor-save option; the RISC-V port hardwires lazy save via
  the illegal-instruction trap (`rtos_save_pie_coproc`). Switching to eager save
  would require a port patch, not a config flag.
- **It would not help — it would likely make things worse.** The hang is *inside*
  `pie_restore_regs`/`pie_save_regs` themselves (the `esp.vld.128 q3` op). Eager
  save just changes *when* those instructions run (on every context switch that
  touched PIE, instead of on the next owner's first PIE use). The faulting
  instruction is the same; running it more often would raise the hit rate, not
  remove it. **Reject.**

### (b) Schedule the two PIE consumers so they never share a core — **feasible in principle, this is the real lever, but needs re-tiering not just pinning**

Our PIE consumers: tagger-FFT (Core 0), worker DSP (Core 1), plus **two resample
workers**. The hang requires two PIE *owners* on one core swapping ownership. The
proven-good `CONFIG_DIAG_SINGLE_PIE_OWNER` experiment (2026-07-07, branch
`p2-single-owner`) confirms: one PIE owner per core ⇒ `pxPortUpdateCoprocOwner`
never runs the save/restore body ⇒ zero hangs.

The question is whether we can get one-owner-per-core **without scalarizing**
(scalar resample = ~250% duty). That means the resample workers must be *pinned*
so that each core has at most one PIE-using task, while keeping their PIE asm:

- Core 0: tagger (PIE) — no other PIE task.
- Core 1: worker DSP (PIE) — no other PIE task.
- The two resample workers must then go... nowhere free. Both cores already host
  a PIE owner. So "pin so they never share" is only possible if the resample
  workers are **merged onto the same core as an existing PIE owner but as the
  *same task*** (impossible — they're separate tasks and would swap ownership),
  or if resample stops using PIE on the contended cores.

**Assessment:** true one-owner-per-core requires **PIE-ownership re-tiering**, not
just affinity pinning — e.g. fold resample PIE work into the worker/tagger task
context (so it's the same TCB, no owner swap), or dedicate the resamplers to a
core that has no other PIE user. Given only two HP cores and two existing PIE
owners, this is an architectural change (which task *is* the PIE owner on each
core), not a scheduler tweak. It is the **most likely-to-succeed** path because
it is the only mechanism *proven* to avoid the fault, but it is real engineering.
A cheaper variant: run one resample worker's PIE kernels *inside a critical
section that also owns the core's PIE* so no foreign task can swap in — but that
reintroduces the interrupt-window problem that defeated `vTaskSuspendAll` before.

### (c) Barrier / fence / NOP spacing between the Q-reg ops — **speculative, low-confidence, but the cheapest experiment and has an adjacent precedent**

There is **no public documentation** stating `esp.vld.128`/`esp.vst.128` need
inter-instruction spacing or fences (the PIE intro article
[developer.espressif.com/blog/2024/12/pie-introduction](https://developer.espressif.com/blog/2024/12/pie-introduction/)
mentions only 128-bit *data alignment*, no instruction hazards). So this is a
hypothesis, not an evidenced fix.

However, **esp-dsp #102 is a precedent that P4 custom-coprocessor instructions
are alignment/pipeline sensitive** — an unaligned `esp.lp.setup` makes the core
"go haywire." Our fault is on the 4th of a *tight run of eight* `esp.vld.128.ip`
ops with no intervening instructions, restoring from a freshly-written save area.
It is plausible (unproven) that:
  - a specific *alignment* of the restore sequence, or
  - a hazard between back-to-back `esp.vld.128.ip` (post-increment) ops,
triggers a silicon stall.

Cheap discriminating experiments (in rough order):
  1. **`.balignw 4,0x0001` before `pie_restore_regs`/`pie_save_regs`** (mirror the
     esp-dsp #102 fix) — trivial, and directly tests the alignment hypothesis.
  2. **Insert `nop`/`fence` between the Q-reg ops** (or between the Q-reg block
     and the QACC block) — tests the pipeline-hazard hypothesis. Cheap to try,
     cheap to measure against the deterministic smoke gate.
  3. **Split the post-increment run**: replace `esp.vld.128.ip qN,\frame,16` with
     explicit address arithmetic + non-incrementing loads, to rule out a
     post-increment address-unit hazard.

These are low-confidence but **very cheap to falsify** because the smoke fixture
reproduces the hang every run — a positive result would be a small, upstreamable
port patch. Worth one focused afternoon *before* committing to the re-tiering
work in (b).

### (d) Disable PIE features so fewer regs are saved — **not viable**

The save area is fixed-shape: 8 Q-regs + QACC(H/L) + UA_STATE + XACC/SAR/FFT
packing (`portasm.S:272-385`). There is no sub-feature toggle that reduces the Q
registers, and the hang is on `q3` (a plain vector reg every PIE kernel uses).
You cannot save "fewer" without breaking correctness. **Reject.**

### (e) esp-dsp / IDF config that changes PIE state management — **nothing applicable beyond what's tried**

- The two P4 silicon workarounds that IDF *does* gate
  (`SOC_CPU_HAS_HWLOOP_STATE_BUG`, the EXT_ILL/FPU handling) are HWLP/FPU, not
  PIE, and are compiled out under `CONFIG_ESP32P4_REV_MIN_FULL=100`
  (rev ≤ 1 only). The `CONFIG_ESP32P4_REV_MIN=0` experiment (re-enabling them)
  already **did not** fix the smoke hang (memory 2026-07-06), consistent with
  the fault being PIE, not HWLP.
- Our patches/0002 (remove `esp.lp.setup` from esp-dsp kernels) and patches/0003
  (coproc save area in internal RAM) are already in play; 0003 is now *genuinely*
  applied and the hang persists at the same instruction — so PSRAM-save-area is
  ruled out. There is no further esp-dsp/IDF knob that changes how the PIE
  *register file* is saved.

---

## Q4 — Concrete recommendation & ranking

**Framing (unchanged and important):** production is stable (proven 4h+ with the
shipped patches 0002/0003); the hard wedge is **smoke-gate / on-device-IQ-replay
only**. It blocks a test gate, not deployment. That materially lowers the
urgency and argues for a cheap-experiment-first posture.

Ranked fix paths (effort / likelihood / risk):

| Rank | Path | Effort | Likelihood | Risk | Notes |
|---|---|---|---|---|---|
| **1** | **(c) alignment/fence micro-experiments** on `pie_save_regs`/`pie_restore_regs` (`.balignw`, then `nop`/`fence`, then de-post-increment) | **Low** (hours; deterministic gate) | **Low–Medium** — speculative but has the esp-dsp #102 alignment precedent | Low (tiny, revertible port patch) | Try first *because* it's cheap and the gate is deterministic. A hit = small upstreamable patch. |
| **2** | **(b) PIE-ownership re-tiering** to one PIE owner per core without scalarizing | **High** (architectural: which TCB owns PIE per core) | **High** — only *proven* avoidance | Medium (re-architects task→core→PIE mapping; must not starve frame_decoder) | The dependable fix if (c) fails. Fold resample PIE into the owning task, or dedicate a core. |
| **3** | **Ship freq-scanner with honest `Smoke-skip`**, keep patches 0003/0004, pursue (c)/(b) as follow-up | **Low** | n/a (accepts the gap) | Low (production proven stable; DSP proven on real IQ) | The pragmatic default per prior sessions' verdict. |
| **4** | **Redesign the 0004 watchdog** to catch a *persisted-PC* single-instruction hang (tick-driven same-PC check) | Medium | n/a (turns silent wedge into attributable panic + enables coredump/JTAG) | Low | Not a fix but greatly improves diagnosability and gives an in-field recovery path; worth doing regardless. |
| **X** | **IDF bump** | Medium | **~Zero for this bug** | Medium (churn) | Master's PIE save/restore is byte-identical; no queued erratum workaround. **Do not do this expecting a fix.** |

### Does the evidence support an IDF-bump attempt? **No.**
Master `portasm.S` is byte-for-byte identical to our checkout, the only relevant
fix (`b25cb2906c`) is already applied, and no ESP32-P4 erratum or `soc_caps` bug
flag exists for the PIE register file. An IDF bump would re-ship the same
faulting instruction sequence. The evidence points **elsewhere**: first to a
cheap silicon-hazard experiment on the save/restore sequence itself (c), and if
that fails, to architectural PIE-ownership re-tiering (b), with shipping-behind-a
-Smoke-skip (3) as the honest interim.

### Honest gaps in this research
- I found **no** vendor confirmation of the root cause. Whether this is a true
  undocumented silicon corner or a subtle port-software interaction is **not
  settled by public sources** — it is inferred from our own build-matched
  disassembly plus the absence of any documented erratum.
- The alignment/fence hypothesis (c) has an *adjacent* precedent (esp-dsp #102)
  but **no direct evidence** for the PIE vector ops. It is proposed because it is
  cheap to falsify against a deterministic gate, not because sources support it.
- I did not have JTAG traces; the single-instruction wedge cannot emit a
  coredump (confirmed prior), so instruction-level ground truth beyond the Saved
  PC is unavailable without hardware debug — which is exactly why (4) is worth
  building before (c)/(b).

---

## Sources
- ESP32-P4 SoC Errata summary — https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32p4/02-errata-summary/index.html
- esp-dsp issue #102 "Problematic HW Loops on ESP32-P4" (DSP-158) — https://github.com/espressif/esp-dsp/issues/102
- esp-idf issue #11690 "FPU registers not properly switched for ESP32-S3" — https://github.com/espressif/esp-idf/issues/11690
- ESP32-P4 PIE introduction blog — https://developer.espressif.com/blog/2024/12/pie-introduction/
- esp-idf master `portasm.S` (raw) — https://raw.githubusercontent.com/espressif/esp-idf/master/components/freertos/FreeRTOS-Kernel/portable/riscv/portasm.S
- esp-idf commits: `b25cb2906c`, `94b526c9fe47`, `38abf982161a`, `0bc169e73549`, `c26879d29e52`, `82668dd3feea`, `55acc5e5e7f1` (freertos riscv port); `3a18ec3b7fc9`, `90f6eeb32785`, `104f8ff55b00` (master-only port.c) — via `gh api repos/espressif/esp-idf/commits`
- Local: `esp-idf/.../riscv/portasm.S` (fault site `:130`, `:345-352`), `esp-idf/.../riscv/port.c` (`:834`, `:913`), `esp-idf/components/soc/esp32p4/include/soc/soc_caps.h:199-203`
