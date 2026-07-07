# PIE save/restore wedge — regression archaeology: "what did WE change?"

**Date:** 2026-07-07
**Scope:** Read-only investigation. No repo edits, no device. Complements
`2026-07-07-pie-wedge-research.md` (which covered the IDF/erratum/alignment angle
and concluded an IDF bump won't help). This doc answers the *different* question:
**what did we add or do differently that turned a working setup into a freezing
one**, via git archaeology of the PIE task/affinity layout and a state-management
comparison of our PIE asm vs the esp-dsp reference and the IDF save/restore.

**Confirmed facts I build on (not re-derived):** hard HP-WDT hang (no coredump)
on `esp.vst.128`/`esp.vld.128` inside `rtos_save_pie_coproc`'s
`pie_save_regs`/`pie_restore_regs`; single-PIE-owner-per-core avoids it; dense
smoke fixture hits it every run, production rarely (auto-recovers). Ruled out:
memory region, re-entry storm, fence/nop/alignment (patch 0005), IDF bump
(master `portasm.S` byte-identical).

---

## TL;DR

1. **The two-PIE-owner-per-core condition was ADDED, not always present.** The
   necessary condition (two tasks that both execute PIE-Q instructions, pinned to
   the same core, so the lazy owner-swap runs `pie_save_regs`/`pie_restore_regs`)
   was **created on Core 1 by commit `baaff6d` (2026-05-22 08:34) "resample_256_to_250:
   PIE asm for 9-tap MAC"**. Before that, resample was scalar, so `ingest_core1`
   (Core 1) was *not* a PIE owner and each core had exactly **one** PIE-Q owner —
   which is precisely the proven-good single-owner layout. `baaff6d` turned
   `ingest_core1` into a second PIE-Q owner sharing Core 1 with `worker_core1`
   (already a PIE-Q owner since ~2026-05-20). **This is the concrete "what we
   added."** T60 (`7f77955`, 2026-07-05) only shifted timing — it did not
   introduce the pairing, consistent with the memory note.

2. **`rs_worker_a`/`rs_worker_b` are a red herring** — they are spawned but the
   split path that would notify them is **dead code** (`s_split_pct` hardwired 0,
   "retired by T49b"). They block forever on `ulTaskNotifyTake` and **never
   execute a PIE instruction**, so they are *not* active PIE owners. Ignore them
   in the affinity analysis. The active pair is `ingest_core1` + `worker_core1`,
   both on **Core 1** — which matches the "parks Core 1" note in patch 0005.

3. **State-management difference / the strongest NEW lead:** every PIE-Q kernel
   we run — ours (`resample_arp4.S`, `rotate_to_dc_arp4.S`) *and* esp-dsp's
   (`dsps_fird_s16_arp4.S`) — sets **PIE CFG bit 1 (unaligned 128-bit vld) and
   never clears it**. **The IDF FreeRTOS port does NOT context-switch the CFG
   register** (verified: `esp.movx.w.cfg` appears nowhere in `portasm.S`; only
   Q0–7/QACC/UA_STATE/XACC/SAR/SAR_BYTES/FFT_BIT_WIDTH are saved). So after the
   first PIE kernel runs, the core is left in **unaligned-vld mode permanently**,
   and the port's own `esp.vld.128`/`esp.vst.128`/`esp.ld.ua.state` in the
   save/restore body execute **in unaligned mode against a stale UA_STATE**. This
   was **never tested** — patch 0005 added fences/nops/alignment but left CFG set
   the whole time. **New cheap fix:** force CFG bit 1 = 0 around the save/restore
   body (≈6 instructions in the trap handler), or clear it at each kernel's exit.

4. **Honest caveat:** esp-dsp's `dsps_fird_s16_arp4` sets the same unaligned bit,
   so *single-owner* esp-dsp projects also leave CFG set — they just never run the
   save/restore body, so it's harmless for them. What is **unique to us** is the
   *conjunction*: (a) a second PIE-Q owner per core (our `baaff6d`) that makes the
   save/restore body execute, **while** (b) the core is in unaligned-vld mode. The
   CFG-mode hypothesis is only *distinguishable* from "generic latent IDF/silicon
   bug we merely provoke" by running the experiment in §4 — but it is cheap,
   deterministically gated, and untried.

---

## Q1 — Git archaeology: when did two PIE-Q owners first share a core?

### The active PIE-Q owners and their core affinity (current tree)

| Task | Core | Prio | PIE-Q kernels it runs | File:line |
|---|---|---|---|---|
| `dsp_feed` (tagger) | **0** | dynamic (my_prio−1) | `fft_sc16_2048` (`esp.v*` Q15 FFT) | `class_driver.c:422` |
| `ingest_core1` | **1** | 8 | `resample_125_128_mac_arp4` (`esp.vmulas.s16.xacc`) | `ingest_core1.c:641`, call at `:378` |
| `worker_core1` | **1** | 4 | `rotate_q15_chunk_arp4` (`esp.vmul.s16`) + `dsps_fird_s16_arp4` (`esp.vmulas.s16.xacc`) | `worker_core1.c:1220`; calls at `:863`, `uw_correlator.c:1983/2220/2222` |
| `rs_worker_a` | 0 | 7 | *(dormant — never notified)* | `ingest_core1.c:670` |
| `rs_worker_b` | 1 | 7 | *(dormant — never notified)* | `ingest_core1.c:678` |

- **`rs_worker_a/b` are dormant.** `s_split_pct` is `static volatile uint8_t = 0`
  (`ingest_core1.c:90`) with **no live setter** ("Split path retired by T49b …
  hardwired 0", `ingest_core1.c:395-398`). The only `xTaskNotifyGive(s_worker_*.task)`
  calls are inside the `split_pct>0` branch (`ingest_core1.c:483-484`) which never
  executes. So they never own PIE. **They do not participate in the wedge.**
- **Core 0 has one active PIE-Q owner** (the tagger). `rs_worker_a` dormant.
- **Core 1 has two active PIE-Q owners** (`ingest_core1` resample + `worker_core1`
  rotate/fird). This is the pair whose lazy owner-swap runs the faulting
  `pie_save_regs`/`pie_restore_regs`. Corroborated by patch 0005's own comment
  that the wedge "parks Core 1."

### Timeline of when each became a PIE-Q owner (git dates, +1000)

| Commit | Date | Effect on PIE-owner layout |
|---|---|---|
| `bf0bb9f` | 2026-05-19 22:58 | cut over to wideband `fft_burst_tagger` front end |
| `ebb5cb2` | 2026-05-19 21:18 | `fft_sc16_2048` Q15 FFT → **tagger becomes a PIE-Q owner (Core 0)** |
| `e3b709a` | 2026-05-20 14:49 | `rotate_to_dc_arp4.S` Q15 rotation → **worker_core1 becomes a PIE-Q owner (Core 1)** (worker also uses `dsps_fird_s16_arp4`) |
| **`baaff6d`** | **2026-05-22 08:34** | **`resample_256_to_250` PIE asm → `ingest_core1` (Core 1) becomes a SECOND PIE-Q owner on Core 1.** ← the pairing is born |
| `e007db9`/`aad9735` | 2026-05-22 13:40 / 17:26 | split-resample workers added (later retired to dead code) — **not** the cause |
| `c9e75a4` | 2026-05-24 16:00 | resample wpos/batching/PIE trim (#58) — timing change, pairing already present |
| `7f77955` (T60) | 2026-07-05 23:28 | worker narrowband triage → shifted worker timing; **provokes** the latent race more densely, did **not** introduce it |

### Verdict on Q1

**The two-owner-per-core layout was NOT always present.** Until `baaff6d`
(2026-05-22), resample ran as scalar C inside `ingest_core1`, so Core 1 had a
single PIE-Q owner (`worker_core1`) and Core 0 had a single PIE-Q owner (tagger)
— i.e. the *exact* single-owner-per-core configuration that is *proven to avoid*
the hang. `baaff6d` PIE-ised the resample inner loop, making `ingest_core1` a
second PIE-Q owner on Core 1. **That commit is the concrete regression origin of
the *necessary condition*** — the earliest point the lazy owner-swap can run the
faulting save/restore. The 233-frame / 14 h "ran clean" and 30-min-stability
(2026-05-24, memory) runs are all *after* `baaff6d`, consistent with the fault
being present-but-rare (SW-reset auto-recovery) rather than absent then. T60
(2026-07-05) only densified the timing; it is not the origin.

So this is **not** "the layout was always there and only load changed." A specific
commit **added** the second PIE owner. The important honest nuance: `baaff6d`
exposed a latent fault in the *IDF/silicon save/restore path* — it is "what we
added" in the sense of *what makes the buggy path reachable*, not necessarily a
bug in our own asm. Whether the save/restore itself can be made robust is Q2–Q4.

---

## Q2 — Our PIE asm vs reference: state-management differences

### What the IDF save/restore actually saves (ground truth from our checkout)

`portasm.S:272-385` (`pie_save_regs`/`pie_restore_regs`) saves/restores, in order:
Q0–Q7 (`esp.vst.128.ip`/`esp.vld.128.ip`), QACC_H/L (`esp.st/ld.qacc.*`),
**UA_STATE** (`esp.st/ld.ua.state.ip`, `:288`/`:359`), XACC (`esp.st/ld.u.xacc`),
then SAR_BYTES/FFT_BIT_WIDTH/SAR (via `esp.movx.r/w.{sar.bytes,fft.bit.width,sar}`).
**The CFG register is not among them** — `grep` for `movx.*cfg` in `portasm.S`
returns nothing (the only `movx` there are for sar/fft.bit.width). Confirmed
against `rvruntime-frames.h:234` (`RV_PIE_UA_STATE` exists; there is **no**
`RV_PIE_CFG` field). **CFG is a per-core CPU state the OS never context-switches.**

### Difference (a): does reference code clear/reset QACC / UA_STATE at entry/exit? — No, and neither do we; not a divergence

- Our `resample_125_128_mac_arp4` (`resample_arp4.S:71-107`) primes XACC with the
  rounding bias each channel (`esp.ld.xacc.ip`) and drains via `esp.srs.s.xacc`;
  it does **not** touch QACC, and leaves XACC/SAR in a defined state. It does not
  `esp.zero.qacc` or reset UA_STATE at exit — **but neither does esp-dsp's
  `dsps_fird_s16_arp4`** (`dsps_fird_s16_arp4.S`), which also just primes XACC and
  drains with `esp.srs.s.xacc`. No divergence here. (All these are XACC users, and
  XACC/SAR *are* context-switched, so leftover XACC/SAR is harmless.)
- Our `rotate_q15_chunk_arp4` (`rotate_to_dc_arp4.S:31-65`) sets **SAR=15** at
  entry (`esp.movx.w.sar`) and uses only Q0–Q7 + SAR; no QACC. SAR is
  context-switched by the port, so leaving SAR=15 is harmless. No divergence.

### Difference (b) — THE one that matters: the unaligned CFG bit is set and never cleared, and CFG is not context-switched

Both of our kernels and esp-dsp's fird enable **CFG bit 1 (unaligned 128-bit
vld)** identically:

| Kernel | Enable site | Cleared? |
|---|---|---|
| ours: `resample_125_128_enable_pie_cfg` | `resample_arp4.S:54-58` (`esp.movx.r.cfg` / `ori t6,t6,2` / `esp.movx.w.cfg`) — hoisted, "set once at create, assume it persists" | **no** |
| ours: `rotate_q15_chunk_arp4` | `rotate_to_dc_arp4.S:37-39` (`ori t6,t6,2`) — set per call | **no** |
| esp-dsp: `dsps_fird_s16_arp4` | `dsps_fird_s16_arp4.S:50-52` (`or t6,t6,2`) — set per call | **no** |
| memory note | `feedback_p4_pie_asm_constraints.md:54-65` ("Rule 5 — unaligned 128-bit vector loads need cfg bit 1 … we set it once at channelizer create() and assume it persists") | documents the persist-forever intent |

Because the port never saves/restores/forces CFG, once *any* PIE-Q kernel has run
on a core, **that core is left in unaligned-vld mode permanently**, including
during every subsequent `pie_save_regs`/`pie_restore_regs`. The save/restore's own
`esp.vld.128.ip`/`esp.vst.128.ip` (to the 16-byte-aligned save frame) and its
`esp.ld.ua.state`/`esp.st.ua.state` therefore run **in unaligned mode**. Note the
restore *ordering*: Q0–Q7 are reloaded *before* UA_STATE is restored
(`portasm.S:346-359`), so during the eight `esp.vld.128.ip` the unaligned-load
datapath is driven by whatever **stale UA_STATE** the previous owner left. The
confirmed wedge PC is the 4th of those loads (`esp.vld.128.ip q3`).

### Difference (c) — QACC width/overflow — not implicated

We don't use QACC in `resample`/`rotate`; fird uses XACC not QACC. No unusual
width/overflow usage vs reference. Not a lead.

### Difference (d) — does esp-dsp assume the caller manages PIE state we don't?

esp-dsp kernels **self-manage** the state they need at entry (they re-set the
unaligned CFG bit each call, set SAR where needed). They do **not** assume the
caller pre-clears anything, and they do **not** clean up CFG on exit. So there is
no "esp-dsp expects the caller to reset X that we skipped" gap. The gap is the
opposite: **nobody** (not us, not esp-dsp, not the IDF port) ever returns CFG to
its reset (aligned) value, so the mode leaks into the OS context-switch path.

---

## Q3 — The UA_STATE / unaligned-access angle (the strong lead), stated precisely

**Claim:** the IDF `pie_save_regs`/`pie_restore_regs` were written/validated
assuming the CFG unaligned-access bit is at its **reset value (0 = strict 16-byte
alignment)** — the save frame is 16-byte aligned and the code reads as
"aligned-only" moves. Our workload leaves the core in **unaligned mode (CFG bit
1 = 1)** at all times (Q2b). Whether an *aligned* `esp.vld.128`/`esp.vst.128`
behaves identically with unaligned mode enabled is **undocumented** (public PIE
material — the Espressif PIE blog and dev-portal discussion — describes only
128-bit *data* alignment, not an instruction/mode hazard, and gives no UA_STATE
priming semantics; searches returned nothing on `esp.ld.ua.state` behavior). On
Tensilica/Cadence-style unaligned SIMD loads (which "xesppie" resembles), an
unaligned-capable `vld` routes through an alignment/rotate datapath keyed on
UA_STATE even when the low address bits are zero; if that datapath depends on a
*valid* UA_STATE and the port reloads Q-registers **before** restoring UA_STATE
(it does), the loads run against a **stale/garbage** alignment state. That is a
plausible microarchitectural stall trigger, and it **matches the observed
signature**: sometimes garbage-that-recovers (production, rare), sometimes a hard
pipeline wedge (dense smoke) depending on the stale UA_STATE contents.

**Why this is consistent with "single-owner avoids it":** with one PIE owner per
core, `pxPortUpdateCoprocOwner` never runs the save/restore body, so the
unaligned-mode-during-save/restore condition never arises — even though CFG is
still left set. The mode only bites when the body actually executes, which
requires the second owner we added in `baaff6d`.

**Why esp-dsp-only projects don't report it:** typical esp-dsp demos have a single
PIE consumer → single owner per core → save/restore body never runs → CFG-left-set
is harmless. Our conjunction (two owners + unaligned mode) is not their
configuration.

**This was not tested.** Patch 0005 added fence/nop/`.balignw` inside the
save/restore but **left CFG bit 1 set the entire time** — it never forced aligned
mode. So the "run the save/restore in strict-aligned mode" hypothesis is untried.

---

## Q4 — Synthesis: most likely "what we added," and a NEW cheap fix

### Ranked hypotheses for "what we did differently"

1. **(Highest confidence — proven necessary) We added a second PIE-Q owner per
   core.** `baaff6d` (2026-05-22) PIE-ised the resample, making `ingest_core1` a
   second PIE-Q owner on Core 1 alongside `worker_core1`. This is the git-dated
   origin of the necessary condition; single-owner-per-core provably avoids the
   hang. *Caveat:* this exposes a latent IDF/silicon fault in the save/restore
   rather than being a bug in our asm per se.
2. **(Novel, testable, medium confidence) We (and esp-dsp) leave the core in
   unaligned-vld mode, and the IDF port runs its save/restore in that mode against
   a stale UA_STATE.** CFG bit 1 is set by every PIE-Q kernel and never cleared,
   and the port never context-switches CFG (Q2b/Q3). This is the candidate
   *mechanism* for why running the save/restore body *hangs* — and it is the one
   state-management difference that is (i) real, (ii) untested, and (iii) points
   at a cheap fix distinct from everything already exhausted.

These are **complementary, not competing**: #1 is why the save/restore *runs*; #2
is a candidate for why *running it hangs*. Combined statement of "what we added":
*a second PIE-Q owner per core (baaff6d) whose owner-swap forces the IDF
save/restore to execute while the core is in unaligned-vld mode (CFG bit 1 left
set) with a stale UA_STATE — a mode/state combination the port was not validated
against.*

### The NEW cheap fix (distinct from region / fence / align / re-tier / IDF-bump)

**Force CFG unaligned mode OFF around the register-file save/restore**, then
restore it. In the trap handler / around `pie_save_regs`+`pie_restore_regs`
(`portasm.S`), ~6 instructions:

```
esp.movx.r.cfg  <tmp>            # save caller CFG
andi            <tmp2>, <tmp>, ~2  # clear bit 1 (strict 16-byte alignment)
esp.movx.w.cfg  <tmp2>
   ... pie_save_regs / pie_restore_regs (frame is 16-byte aligned) ...
esp.movx.w.cfg  <tmp>            # restore caller CFG
```

The save frame is already 16-byte aligned (patch 0003 put it in internal
16-aligned RAM), so strict-aligned loads/stores are correct. If the hang is caused
by the save/restore running in unaligned mode against stale UA_STATE, this removes
the trigger while leaving our kernels' unaligned loads (from 2-byte-aligned
scratch) untouched.

**Cheaper still, to *decide* the hypothesis fast** (falsify before patching the
port): clear CFG bit 1 at the `ret` of `rotate_q15_chunk_arp4`
(`rotate_to_dc_arp4.S`), `resample_125_128_mac_arp4` (`resample_arp4.S`), and add
a post-call CFG clear in the C wrappers around `dsps_fird_s16_arp4`
(`uw_correlator.c`) and the resample dispatch. This shrinks the window in which
CFG is set from "always" to "only inside a kernel body," which should sharply drop
the deterministic-smoke hit rate *if* CFG-mode is the trigger. It is not airtight
(preemption mid-kernel still leaves CFG set at the trap), so the airtight fix is
the port-handler force-CFG-0 above — but the kernel-exit clear is a fast, cheap
discriminating experiment against the deterministic gate.

**Decision value:** if forcing CFG=0 around the save/restore stops the smoke
wedge, we have a small, upstreamable port patch and a *root-cause explanation*
("IDF save/restore is not robust to a non-reset CFG unaligned bit"). If it does
**not**, we have cheaply falsified the last distinct software hypothesis and
strengthened the case that this is a genuine latent IDF/silicon corner that only
architectural single-owner-per-core re-tiering (research doc §Q3b) or shipping
behind a `Smoke-skip` (research doc §Q4-rank-3) can address.

### Honest bottom line

- We **did** add the trigger: `baaff6d` created the two-PIE-owner-per-core
  condition on Core 1 (git-proven). It is not a case of "always there, only load
  changed."
- The **one untested software lever** the code comparison surfaces is the
  unaligned-CFG-mode-during-save/restore (CFG is not context-switched; every
  kernel leaves it set; patch 0005 never cleared it). It is a hypothesis, not a
  proven cause — esp-dsp sets the same bit, so it only bites in conjunction with
  our second owner — but it is cheap, deterministically gated, and genuinely
  distinct from the exhausted region/fence/align/re-tier/IDF-bump options.

---

## Evidence index (file:line / commit)

- Two-owner origin: `baaff6d` 2026-05-22 08:34 "resample_256_to_250: PIE asm for
  9-tap MAC"; worker PIE-Q since `e3b709a` 2026-05-20; tagger PIE-Q since
  `ebb5cb2` 2026-05-19; T60 `7f77955` 2026-07-05.
- Core affinity: `ingest_core1.c:641` (Core 1), `worker_core1.c:1220` (Core 1),
  `class_driver.c:422` (Core 0, dsp_feed), `ingest_core1.c:670/678` (rs_workers,
  Core 0/1, **dormant**).
- rs_workers dormant: `ingest_core1.c:90` (`s_split_pct=0`), `:395-398` (retired),
  `:483-484` (notify only in dead branch).
- Active PIE-Q calls on Core 1: resample `ingest_core1.c:378`; rotate
  `worker_core1.c:863`; fird `uw_correlator.c:1983/2220/2222`.
- Our kernels enable-and-leave unaligned CFG bit: `resample_arp4.S:54-58`,
  `rotate_to_dc_arp4.S:37-39`. esp-dsp same: `dsps_fird_s16_arp4.S:50-52`.
  Memory: `feedback_p4_pie_asm_constraints.md:54-65`.
- IDF save/restore saves UA_STATE but NOT CFG: `portasm.S:272-385` (UA_STATE at
  `:288`/`:359`), no `movx.*cfg`; `rvruntime-frames.h:234` (`RV_PIE_UA_STATE`, no
  CFG field).
- Patch 0005 tried fence/nop/align, never touched CFG:
  `patches/0005-freertos-riscv-pie-coproc-save-restore-spacing-fence-align.patch`.

## Sources (external)
- ESP32-P4 PIE introduction blog — https://developer.espressif.com/blog/2024/12/pie-introduction/
- PIE dev-portal discussion #353 — https://github.com/espressif/developer-portal/discussions/353
- esp-dsp #102 (HWLP alignment on P4, adjacent precedent) — https://github.com/espressif/esp-dsp/issues/102
  (Public PIE docs give no `esp.ld.ua.state`/unaligned-mode hazard semantics —
  the UA_STATE-during-save/restore mechanism in Q3 is inferred, not documented.)
