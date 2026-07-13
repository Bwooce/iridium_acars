# Autogain validation-gate-1 results (2026-07-13) — v1 DISPROVEN on host

Host Monte-Carlo (`tools/analysis/autogain_sim.py`, 1000 sweeps + 400×30 drift
chains, stable across seeds) testing the `2026-07-12-autogain-redesign-proposal.md`
picker against the `2026-07-12-autogain-review.md` reseeded pass process. Verdict:
**do NOT build the v1 5-arm ±1-step knee-finder.** Both Critical findings confirmed.

## Results (base case)

| | v1 knee-picker | coarse-3-state @ IRA | coarse-3-state @ ACARS-LO |
|---|---|---|---|
| D_max (median, p10–p90) | **6, [0,13]** (design assumed 16) | same | same |
| pick when signal present | **offset −6, 100%** (collapse) | fixes collapse | fixes collapse |
| cross-run drift from 43.4 dB | craters 43.4→…→14.4 (or freezes) | not fully stable (spread 4) | **≤1 step spread** |
| ACARS yield at converged gain | **1–35% (65–99% LOSS)** | 6–35% | **93–100%** |

- **Critical 1 (count starvation) — CONFIRMED.** Real D_max ≈ 4–6 (not 16), so the
  Poisson band `2·isqrt(D_max+1)` swallows the `0.75·D_max` plateau floor → every
  arm with ≥1 decode is "on plateau" → picker returns the lowest arm every time →
  gain drifts systematically DOWN into deafness. Curve-shape-independent (robust).
- **Critical 2 (wrong signal) — CONFIRMED.** Measuring on strong IRA and applying
  to weak ACARS: even a "correct" IRA pick sits 10–25 dB below the ACARS plateau.
  Coarse-3-state on the IRA LO fixes the collapse but stays ACARS-poor; **only
  coarse-3-state + operating ACARS LO is both stable AND not-deaf** (98–100% yield;
  at the realistic reduced ACARS-LO rate it safely freezes at the good incumbent).
- **Pass-rate sensitivity:** v1 collapses HARDER with more counts (×2 → 0% ACARS);
  only "safe" at ×0.5 by accident (no signal → freeze). Failure is gated by
  MIN_COUNTS × pass-rate, not the ACARS knee.

## Load-bearing caveat
The **magnitude** of ACARS loss depends on the IRA↔ACARS knee gap, which is
UNMEASURED — only 3 IRA points (0/2/6/0/1) + a qualitative ACARS "flat plateau
43.4–33.8, knee below 33.8" exist. The sim assumed ACARS-knee ~31 dB / IRA-peak
~25 dB. The COLLAPSE itself (Critical 1) is curve-independent and solid; the loss
magnitude is not. **Before any device code, the real ACARS gain→decode curve must
be measured** (a slow background sweep at the operating LO, full 0–49.6 dB once).

## Recommendation (evidence-based redesign)
1. **Kill the v1 5-arm knee-finder.** Host-disproven.
2. **Ship the passive drift estimator first** (proposal §5 / review Important 5):
   ~50 lines on the hourly telemetry poll, tracks garbage-ratio `G/(D+G)` + decode
   total at the OPERATING gain/LO, requests recalibration on drift. Zero downtime,
   right signal, sidesteps both Criticals. Low-risk primary.
3. **If an active sweep is kept**, it must be **coarse-3-state (deaf/good/saturating,
   wide guard, fixed margin ABOVE the low edge) + operating ACARS LO** — and gated
   on the measured ACARS curve (above). Interleaved round-robin (§2) is kept.
4. **Acceptance = ACARS-yield preservation** at the operating LO (not ±1-step
   stability, which the degenerate always-step-down also passes).

Sim: `tools/analysis/autogain_sim.py` (`python3 tools/analysis/autogain_sim.py`).
