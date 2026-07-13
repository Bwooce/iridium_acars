# Autogain v2 — passive-primary drift control + coarse ACARS-LO knee fallback

**Date:** 2026-07-13 · **Status:** design proposal (supersedes the gain-cal half of
`2026-07-12-autogain-redesign-proposal.md`, which was **DISPROVEN** on host — see
`2026-07-13-autogain-gate1-results.md`). LO-rescan half unchanged.

> **Review status (Fable, `2026-07-13-autogain-v2-review.md`): BUILD-WITH-REVISIONS.**
> Architecture accepted (faithful to gate-1 + the v1-review Verdict; no v1 flaw
> reintroduced). **Sequencing changed by the review: ship Part A's *sampling* in
> shadow mode FIRST** (log-only, ~1–2 weeks) to reconcile a ~20× decode-rate
> contradiction in the source docs (54–66/min deployed vs 2.6–13/pass sim) and to
> set every Part-B constant from real data. Part B is BLOCKED on revisions R1–R5
> (below). Also **price the "simpler alternative"** (collapse B0+B2 into one
> exposure-stopped full-range coarse curve → gain = measured-knee + fixed-dB margin)
> before building the 4-arm machinery — it's immune to the all-arms-deaf trap and
> likely dominant at the measured rates. See the review for R1–R5 + all findings.

## Why v1 died (one paragraph)
The v1 5-arm ±1-step knee-finder was killed by host gate-1: at the real pass rate
`D_max ≈ 4–6` (not 16), so the Poisson band swallows the `0.75·D_max` plateau floor
→ the picker returns the lowest arm 100% of the time → gain drifts DOWN into
deafness (Critical 1); and measuring the strong IRA broadcast to set a gain for
weak ACARS lands 10–25 dB below the ACARS knee (Critical 2, 65–99% yield loss).
**Two lessons: (a) the data supports at most a coarse deaf/good/saturating call,
never a ±1-step plateau rank; (b) the objective must be measured on the operating
ACARS signal, at zero or minimal downtime.**

## Design principle
**Prefer measuring the thing we care about (ACARS yield at the operating gain/LO)
passively, over perturbing it with an active sweep.** So:
- **Primary = passive drift estimator** (no RF actions, zero downtime, right signal).
- **Fallback = a rare, coarse, ACARS-LO active sweep**, triggered only when the
  passive estimator says drift is real — and only after the ACARS gain curve has
  been measured once.

---

## Part A (SHIP FIRST) — passive drift estimator

Rides `autotune_sched_task`'s existing hourly telemetry poll. No gain/LO changes.

**Inputs (existing cumulative reads):** `worker_core1_get_decode_counts(&D,&U)` and
`bch_failed_from_hist(&h)` → `F`. All at the *operating* gain + LO.

**Baseline:** captured right after any calibration / manual gain set (or first boot
with signal): the trailing-week median of the 24 h decode total `D_day`, and the
garbage ratio `Ḡ = G/(D+G)` where `G = U + F`. Persisted (NVS) alongside the gain.

**Per-hour update:** rolling deltas. Two drift signals:
1. **Garbage rise:** `G/(D+G)` exceeds baseline by an absolute **+0.15 for 3
   consecutive hours** → suspect gain too high / RFI riser.
2. **Yield fall:** 24 h `D_day` < **0.5 × trailing-week median** → suspect gain too
   low / deaf, or antenna/RFI change.

**Action on trigger:** request one active recalibration (Part B) and reset the
3-hour counters. Rate-limited to at most one request / `autotune_gain_interval_s`.
Purely a *trigger*; it never changes gain itself.

**Caveats already known (must document at the call site):**
- `bch_failed_from_hist` is pre-Chase and double-counts Chase-recovered frames
  (review MINOR) — the ratio is biased but *consistent* run-to-run, which is all a
  drift detector needs (it compares deltas to its own baseline, not to truth).
- Decode is pass-bursty, so hourly `D` is noisy → the 3-consecutive-hours and
  weekly-median smoothing are load-bearing, not decoration.

**Cost:** ~50 lines in the poll loop; no RF, no downtime, no wedge risk. This alone
replaces the "erratic daily sweep" with a signal that watches the mission directly.

---

## Part B (GATED) — coarse ACARS-LO knee sweep (the rare fallback)

Only runs when Part A triggers (or an explicit `POST /autotune`). **Two hard
prerequisites** before this is trusted/enabled:

### B0 (prerequisite) — measure the real ACARS gain→decode curve, once.
Gate-1's load-bearing unknown: the ACARS knee position is unmeasured (only 3 IRA
points + a qualitative plateau exist). Before B is enabled, run **one** slow manual
background sweep across the **full R828D table (0–49.6 dB)** at the **operating
ACARS LO**, over enough passes to get ≥MIN_COUNTS per gain, and plot `D(gain)` and
`G(gain)`. This (a) locates the ACARS knee and saturation edge for real, (b)
verifies garbage is monotone in gain above 34 dB (review Important 4), (c) sets the
coarse thresholds + margin below. It is a measurement, not a shipped loop.

### B1 — mechanism
- **Operating ACARS LO** (not IRA). Accept the slower count rate: more rounds /
  longer sweep (background). This is the Critical-2 fix and is non-negotiable.
- **Interleaved round-robin** slices (kept from v1 §2 — sound for exposure fairness;
  rotate arm order each round; 3 s prime-discard + `SLICE_S` slice).
- **Coarse arm set** — wider spacing than v1, because we can only resolve 3 states:
  offsets `{-8, -4, 0, +4}` table steps from incumbent (config), clipped + deduped.

### B2 — objective: 3-state classification, margin above the low edge (NO plateau rank)
Per arm accumulate `D_i` (real decodes) and `G_i` (garbage). Then:
```
D_max = max_i D_i
if D_max < MIN_COUNTS_ACARS:            -> no signal; keep incumbent; persist nothing
classify each arm:
  good        : D_i >= GOOD_FRAC * D_max            (GOOD_FRAC ~ 0.5, wide guard)
  saturating  : gain above a good arm AND (D_i drops below good  OR  median decoded
                SNR is >= SAT_SNR_DROP_DB below a lower-gain arm)   // compression telltale
  deaf        : everything else below the good band
low_edge  = lowest-gain "good" arm
chosen    = low_edge gain + MARGIN_STEPS   (MARGIN_STEPS ~ 2; never sit ON the knee),
            clamped to the highest non-saturating good arm
```
Rationale: gate-1 showed the fine plateau rank is noise at `D_max≈6`; a 3-state
call with a wide guard band is the most the data supports, and the fixed margin
keeps us safely *above* the knee where a bad estimate can't strand us deaf.

### B3 — hysteresis (unchanged spirit from v1 §4, coarser)
No-signal → keep. Chosen == incumbent → done. Otherwise move toward `chosen`,
clamped to `MAX_STEPS_PER_RUN` table steps/run (walk, don't jump). Persist via the
internal-stack `autotune_persist` task only when changed (PSRAM-stack rule; the
~24.7-min crash-loop fix already in place, commit 1479bd6).

### B4 — acceptance = ACARS-YIELD PRESERVATION (not ±1-step stability)
The v1 acceptance ("5 sweeps within ±1 step") is invalid — the degenerate
always-step-down passes it. Replace: **after convergence, the chosen gain must not
lose real ACARS decode yield over several passes at the operating LO versus the
manually-found good gain** (43.4/40.2 today). Precision is worthless if the point
is wrong.

---

## Validation gates (before enabling the scheduler)
1. **Host (MUST re-run — do NOT claim the existing result):** the gate-1 sim's
   coarse/ACARS-LO ≤1-step / 93–100%-yield result was produced with the **v1 arm
   set `{-6,-4,-2,0,+2}` and a 2-step margin** — NOT this design's `{-8,-4,0,+4}` /
   `MARGIN_STEPS=2` / SNR-saturation rule. Re-run `autogain_sim.py` with the **v2
   configuration**, the **measured** ACARS curve from B0, AND a **perturbed-curve
   family** (knee ±3 steps, slope ×0.5–2) before any device code. Fix the
   margin-vs-4-step-spacing mismatch (margin ≥ spacing/2 in dB, or 2-step-spaced
   arms near the low edge) so `chosen` is never an unmeasured midpoint gain. (R2)
2. **B0 device measurement** — the real ACARS `D(gain)`/`G(gain)` curve.
3. **Device B gate** — ACARS-yield-preservation (B4) + stream health (no wedge, no
   `rate=0`) across the quiesced gain sets (gate-1 predecessor scanner work
   validated LO hops 42/42; gain-quiesce at this cadence still needs the soak).
4. Only then arm Part A's trigger to fire Part B automatically.

## Cost / cadence
- Part A: continuous, free.
- Part B: rare (trigger-driven), background at the ACARS LO, so decode is NOT parked
  on IRA — the 1%/day downtime argument (and its hidden "one whole pass" cost) goes
  away. Slower per-arm counts are the price; auto-extend + patience cover it.

## Open risks (honest)
1. **B0 needs real passes** — a full-table ACARS-LO curve at single-digit counts/arm
   is a multi-hour, multi-pass measurement; it can only be gathered when the sky is
   busy. Reception-gated like everything else.
2. **Coarse thresholds** (`GOOD_FRAC`, `MARGIN_STEPS`, `SAT_SNR_DROP_DB`) are set
   from B0; if the ACARS curve is flatter/steeper than the sparse data suggests they
   need retuning — but the 3-state design is far more forgiving than v1's ±1-step.
3. **Gain-quiesce churn** at sweep cadence still unproven (validation gate 3).
4. **Passive estimator false-triggers** on a quiet-sky week (yield-fall) — mitigated
   by the weekly-median denominator + rate-limit; worst case is one wasted rare sweep.
   (Review Imp-2/3: freeze the yield baseline at calibration, re-baseline on any
   LO/gain change, and persist the daily ring across reboots or a flaky site has the
   trigger chronically disarmed.)
