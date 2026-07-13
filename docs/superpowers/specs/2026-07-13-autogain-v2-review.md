# Adversarial review (Fable) — Autogain v2 design (2026-07-13)

Reviewing `2026-07-13-autogain-v2-design.md` against the gate-1 sim
(`tools/analysis/autogain_sim.py`, `2026-07-13-autogain-gate1-results.md`), the v1
review's Verdict, and the code. **Verdict: BUILD-WITH-REVISIONS** — Part A first
(shadow mode); Part B blocked on R1–R5.

## Got right (no action)
Architecture faithfully implements the gate-1 recommendation + v1-review Verdict:
passive-primary, coarse 3-state with fixed margin ABOVE the low edge, operating-
ACARS-LO ("non-negotiable"), ACARS-yield acceptance, B0 = the "measure G(gain)"
de-risk. No v1 flaw structurally reintroduced. ±1-step plateau rank is dead.
Counter discipline right (`get_decode_counts`/`get_histograms` cumulative, not the
reset-on-read `get_stats`). PSRAM-persist discipline + trigger/actuator separation
correct. Interleaved round-robin + 3 s prime kept. Validation gates correctly
ordered.

## CRITICAL 1 — the drift→recalibrate loop doesn't close.
Part B's "`D_max < MIN_COUNTS_ACARS` → keep incumbent" default is inherited from a
good-incumbent context, but Part B runs *because Part A said the incumbent is bad*.
So: drift detected → sweep → no-signal → keep the bad incumbent → re-trigger daily
forever, no escalation, no alert. Gate-1's reassuring "safely freezes at the good
incumbent" flips sign here. Worse: from a drifted-low incumbent the `{-8,-4,0,+4}`
arms can be ALL deaf → D_max≈0 → keep. **v2 dropped v1's recenter-up escalation.**
And MIN_COUNTS_ACARS / R / MAX_STEPS_PER_RUN / extend-cap have no values.
→ **R3:** specify the constants (from measured ACARS-LO rate); restore recenter-up;
after N fruitless A→B cycles widen to a full-range coarse probe (always brackets
the knee) or emit an operator alert; never A→B→no-op silently.

## CRITICAL 2 — claims sim validation for a config the sim never ran.
The sim's coarse picker used the **v1** arm set `{-6,-4,-2,0,+2}` (autogain_sim.py:78),
but v2 specifies `{-8,-4,0,+4}`. `MARGIN_STEPS=2` was validated against 2-step
spacing (low_edge+2 = the next measured arm); at 4-step spacing a one-arm
misclassification is a −4-step error the +2 margin can't cover, and
`chosen=low_edge+2` selects an **unmeasured midpoint gain**. The sim's saturating
rule is count-based (`D_j≥2·D_i`, :307-310); v2 specifies an SNR rule never simmed.
The 93–100%/≤1-step headline belongs to a different picker — exactly the "quoting
validation the config lacks" that killed v1.
→ **R2:** re-run `autogain_sim.py` with the v2 arm set/margin/both saturation rules
AND a perturbed-curve family, before any device code; fix margin-vs-spacing.

## CRITICAL 3 — the empirical base is internally contradictory by ~20×.
v1 finding #1 (deployed, ACARS LO): 54–66/min during a pass ≈ 110–260/pass. Gate-1
sim: 13/pass IRA, ACARS-LO ×0.2–1.0 = 2.6–13/pass. A **20–100× discrepancy** in the
quantity that sets whether Critical-1's freeze dominates, B0's duration, and every
Part-B constant. v2 inherits both without noticing.
→ **R1 (do first, free):** ship Part A's *sampling* in shadow mode; use 1–2 weeks of
its telemetry to reconcile the rate and set the constants.

## IMPORTANT 1 — B0 stop rule impossible + B0 not one-time.
"≥MIN_COUNTS per gain" never terminates (deaf/sat gains yield ~0 forever) → make it
exposure-based (passes/wall-time per gain), counts as output. Duration: half-day to
multi-day at real rates. And the curve drifts (the premise of autogain), so B0 needs
a refresh policy — and Part B runs when B0's thresholds are least likely to hold.
Sim should test a *family* of perturbed curves (knee ±3 steps, slope ×0.5–2).

## IMPORTANT 2 — Part A trigger constants ungrounded; +0.15 fails at both extremes.
Garbage trigger `G/(D+G)`: if baseline Ḡ≥0.85 it mathematically can't fire
(ceiling); in quiet hours D→0 drives the ratio →1 spuriously (floor). Yield trigger
"< 0.5× **trailing**-week median" is self-referential — a 10%/week drift walks the
median down and never trips. → shadow-mode to measure Ḡ + hourly-ratio noise; pool
3 h counts with a min-denominator test; freeze the yield baseline at calibration.

## IMPORTANT 3 — Part A baseline lifecycle unspecified.
Counters are cumulative-since-boot; a reboot wipes the 7-day ring → a flaky site
(the one needing drift detection) has trigger 2 chronically disarmed. LO rescan
(enabled, persists new LO, autotune.c:329-335) invalidates the baseline. Autotune-
busy hours pollute deltas. → persist the daily ring (internal-stack task), re-
baseline on any LO/gain change, exclude `s_autotune_busy` hours, handle post-reboot
delta resets.

## IMPORTANT 4 — B2 saturation telltale needs data the firmware doesn't record.
`s_hist_snr` (worker_core1.c:981) records EVERY popped burst's peak SNR — junk-
dominated, gain-flat (finding #2); there is no decoded-only SNR accumulator. The SNR
telltale needs new instrumentation (record peak_snr_db at the BCH-pass site,
~:846-861) and is unsimmed. → make the count-based rule primary; SNR diagnostic only
until B0 justifies it. (A median over ≤6 decodes/arm is a coin flip anyway.)

## IMPORTANT 5 — B4 acceptance statistically toothless.
Unpaired pass comparison: pass yield CV≈1, so n=5 passes/arm → yield-ratio 1σ≈63%;
detects only >2× loss, not the 20–40% erosion that matters (needs ~90 passes/arm).
→ paired **interleaved A/B** (alternate chosen vs manual gain within the same passes)
cancels pass-strength variance → ~10–30 passes total.

## MINOR
1. "rides the existing hourly telemetry poll" — `autotune_sched_task` polls every 30 s
   and reads NO telemetry (autotune_sched.c:23,91-126); Part A must add sampling +
   persistence (still ~50 lines, but not "existing").
2. Chase double-count co-moves with the drifting signal (conservatively toward
   triggering) — say that, not "consistent".
3. Table-step margin is heterogeneous in dB (2 steps ≈ 1.3 dB up top, ≈4–6 dB near
   30 dB) — set the margin in dB, convert.
4. Background-sweep cost isn't zero: `{-8,-4,0,+4}` runs the operating LO at ~0.6×
   yield during the sweep — fine for a rare event, but say so.
5. Doc nits: duplicate "2." in open-risks; "43.4/40.2 today" vs 33.8 plateau bound —
   state the reference gain; restate SLICE_S=20 s (v1 superseded).

## SIMPLER ALTERNATIVE (price before building B1/B2)
At ACARS-LO rates, B0 (full-table background curve) and B2 (4-arm sweep) are the same
cost class. **Collapse them:** recalibration = re-run the exposure-stopped full-range
coarse curve (~every 3rd–4th table step) and set gain = measured knee + fixed dB
margin. One code path, no GOOD_FRAC/arm-geometry, immune to the all-arms-deaf trap
(full range always brackets the knee), and B0's "one-time" question dissolves (B0 IS
the recalibration). Strictly dominant at the optimistic rate; no slower where it
matters at the pessimistic rate.

## Required revisions
- **R1 (Crit 3, first, free):** ship Part A sampling in shadow mode; 1–2 weeks →
  reconcile rate, measure Ḡ + noise, set thresholds/MIN_COUNTS_ACARS/R.
- **R2 (Crit 2):** re-run the sim with the v2 config + perturbed-curve family; fix
  margin-vs-spacing.
- **R3 (Crit 1):** specify constants; restore recenter-up; add full-range-probe /
  operator-alert escalation after N fruitless A→B cycles.
- **R4 (Imp 1/3):** exposure-based B0 stop + refresh policy; Part A baseline
  lifecycle (persist ring, re-baseline on LO/gain change, exclude busy hours, reboot).
- **R5 (Imp 4/5):** decoded-SNR accumulator or demote SNR telltale to diagnostic
  (count rule primary); B4 = paired interleaved A/B with a stated power target.
