# Adversarial review — Autogain knee-finder redesign (2026-07-12)

Reviewing `2026-07-12-autogain-redesign-proposal.md` against the source it depends on and
today's empirical ground truth. Severity-ranked. Code/number citations inline.

---

## Verified-correct claims (the design got these right — no action)

- **`worker_core1_get_decode_counts()` is non-resetting.** `worker_core1.c:1418-1425`
  plain `atomic_load` of `s_bch_decoded_cum` / `s_bch_unknown_cum`, no reset. ✔
- **`worker_core1_get_histograms()` is non-resetting.** `worker_core1.c:1370-1400`, plain
  copy. `worker_core1_get_stats()` (`:1442+`, `atomic_exchange`) is the only reset-on-read
  path and is drained by status_logger. **Counter-ownership claim holds** — nothing else
  resets the cumulative counters or histograms. ✔ (Attack 6, first half: no conflict.)
- **`bch_failed_from_hist` bin set {0,1,2,3,4,8,12} is CORRECT.** `hist_bch_record()`
  (`:319-324`) writes bin `(clamp(e1)+1)*4 + (clamp(e2)+1)`, `e∈{-1..2}`. `e1==-1`→bins
  {0,1,2,3}; `e2==-1`→bins {0,4,8,12}; union = {0,1,2,3,4,8,12}. Mapping verified. ✔
- **Busy guard is shared** between the knee sweep and the LO rescan: single
  `s_autotune_busy` CAS (`autotune.c:33-43`), taken by every entry point. (Attack 6, second
  half: no race with the periodic LO rescan.) ✔
- **`class_driver_set_gain_quiesced()` already exists** (`class_driver.h:25`,
  `class_driver.c:293,682`) and does pause-URB → I2C-set-gain → resume — **no DDC/sample-rate
  realloc**, unlike a full retune. So per-slice gain churn is lighter than an LO hop.

## ATTACK 2 result — baseline settle vs 3 s prime: **NOT a problem (de-risked).**

`fft_burst_tagger_reset_baseline()` (`fft_burst_tagger.c:571+`) sets `history_primed=false`;
the floor re-primes over `FBT_HISTORY_SIZE = 512` FFT steps. Each step advances `d_index += N`
= 2048 samples (`:1253`) at `FS_DETECT_HZ = 2.5 MHz` (`dsp_processor.h:26`) = 0.819 ms.
**Full re-prime = 512 × 0.819 ms ≈ 0.42 s.** `AUTOTUNE_PRIME_MS = 3000` (`autotune.c:24`) is
~7× that. The 20 s slice is not corrupted by baseline transient. The design's 3 s prime is
comfortably adequate. **No change needed.**

---

## CRITICAL 1 — Per-arm sample starvation makes the plateau test degenerate. (Make-or-break.)

The whole picker rests on `D_i` being large enough that `THETA·D_max − K√(D_max+1)`
separates the deaf arm from the plateau. Work the arithmetic from **today's ground truth**,
not the design's optimistic budget:

- Decode is satellite-pass-bursty: **~0 between passes, 54–66/min during a pass, ~2 passes
  per ~18 min** (design finding #1 — it calls this "the core problem").
- A full sweep is 5 arms × (20 s + 3 s) × 8 rounds ≈ **15.3 min → ~1.7 passes of wall time.**
- The design's "6 IRA decodes/min" budget comes from **a single 55 s dwell** on the
  2026-07-08 curve (`§5`). Given finding #1, that dwell *caught a pass*; it is a
  **during-pass** rate, not an average. Between passes the rate is ~0.

  → Total decodes over the sweep ≈ (decodes per pass on IRA) × (passes captured, minus the
  13% lost to prime discards: 120 s of 920 s). Even generously ~13 decodes/pass × 1.7
  passes × 0.87 ≈ **~19 total, spread across 5 arms ≈ 3.8/arm mean; D_max ≈ 6–8** (the
  luckiest arm). **Single digits — not the D_i ≈ 16 the design asserts.** The design is
  ~2–3× too optimistic, and it is **internally contradictory**: it cannot both call
  pass-burstiness "the core problem" (finding #1) and assume a *sustained* 6/min for its
  count budget (§5).

**Why single-digit D_max breaks the picker** (integer form, `on_plateau`):
`D_i·100 ≥ 75·D_max − 100·2·isqrt(D_max+1)`.

| D_max | RHS/100 (plateau floor on D_i) | Effect |
|------:|-------------------------------:|--------|
| 16 (design) | 4.0 | deaf arm (D≈2) correctly excluded |
| 12 (MIN_COUNTS gate) | 3.0 | marginal |
| **9 (realistic)** | **0.75 → D_i ≥ 1** | **any arm with ≥1 decode is "on plateau"** |
| **7 (realistic)** | **0.5 → D_i ≥ 1** | **same — floor collapses to noise** |

At the counts this install actually produces, the Poisson band `2√(D_max+1)` is **as large
as `D_max` itself**, so the plateau floor sits at ~0. **Every arm that saw even one stray
decode passes the plateau test, and the picker unconditionally returns the lowest-gain arm
(offset −6).** The knee detection — the entire point — fails precisely in the low-count
regime that ground truth says is the norm.

**Downstream failure mode is worse than today's random thrash: it is a *systematic
downward drift.*** Each run the lowest arm (inc_idx−6) wins, step_delta=6 > DEADBAND=2, so the
clamp moves gain **down 4 table steps every run**. It keeps stepping down until gain is low
enough that during-pass IRA decodes over the whole sweep fall below `MIN_COUNTS=12` → "no
signal" → freeze. So autogain converges **just into the deaf region**, gated not by the ACARS
sensitivity knee but by the arbitrary interaction of `MIN_COUNTS` with the IRA pass rate.

**Corollary that poisons the acceptance test (see Important 3):** this degenerate
always-step-down *does* converge and *will* pass "5 sweeps within ±1 step" — at a wrong,
too-low gain. The stability metric cannot distinguish correct convergence from convergence
into deafness.

*The interleaving in §2 is sound for what it does — it equalizes pass **exposure** across
arms. But it does nothing for total **count**: 1.7 passes cannot be averaged, no matter how
finely you slice them. Equal exposure of near-zero signal is still near-zero signal.*

**Mitigation / de-risk:**
1. Before writing any device code, run the **host Poisson simulation the design already
   specifies (validation gate 1)** but seed it with the *measured* pass process (~1.7 passes
   per sweep, ~13 decodes/pass, ~0 between) — **not** a sustained λ. Report the distribution
   of the chosen gain over 1000 sweeps. Prediction: it collapses to "lowest arm" with a
   downward-biased mean. If it does, v1 is disproven on the host for free.
2. Raise `MIN_COUNTS` well above the band-collapse point (needs D_max ≳ 16 for the floor to
   exclude a deaf arm) — but that forces auto-extend to ~30+ min sweeps at this rate and
   often still declares no-signal. This trades the bug for permanent non-convergence.
3. The honest fix is to stop trying to resolve the plateau at all and only make the
   coarse, count-robust decision the data can support (deaf / good / saturating — see Verdict).

## CRITICAL 2 — The knee is measured on the wrong signal population (IRA), applied to ACARS.

The sweep parks on `autotune_ira_lo_hz` (1626.2 MHz, strong constant simplex broadcast);
the gain we actually care about is for **weak duplex ACARS at 1618–1620 MHz**. The design
itself flags this as "biggest" open risk (§7.1), but it is more than a risk — it is a
**validity error in what the objective measures**:

- The knee is the gain at which *real decode craters* — a pure **SNR-margin** phenomenon.
  Strong IRA frames keep decoding down to a much **lower** gain than weak ACARS frames do.
  So the IRA knee sits *below* the ACARS knee. "Lowest gain on the IRA plateau" therefore
  selects a gain that is still fine for IRA but **already past the ACARS knee — deaf for the
  mission signal.** This compounds Critical 1's downward bias in the same direction.
- The design imports **ACARS-LO** empirical findings (garbage −38% from 43.4→33.8 dB,
  "flat plateau") to justify an **IRA-LO** measurement (§0 vs §2). Today's flat-plateau
  evidence was gathered at the operating LO; the sweep does not measure there.
- "Decide with one back-to-back cross-check" (§7.1) is inadequate: if the IRA→ACARS knee
  offset is itself a function of RFI, thermal state, and satellite geometry (all of which
  drift — that is *why* autogain exists), a single-shot offset measured once is not a stable
  calibration constant.

**Mitigation:** the only measurement whose objective matches the mission is the **operating
ACARS LO** — which the design lists as a "future opt-in" (§2, §7.1b) precisely because it
needs 3–4× the rounds (a 1–2 h background sweep). Combined with Critical 1, this means the
count problem is *worse* on the correct LO, not better. This is the central tension the
redesign must resolve, not defer.

---

## IMPORTANT 3 — The acceptance metric cannot detect the Critical-1 failure.

"5 consecutive sweeps within ±1 table step" (§4, gate 2) is satisfied by the degenerate
always-step-down converging into deafness (Critical 1). **A passing acceptance run is not
evidence of correctness.** Replace/augment with an **ACARS-yield gate**: after convergence,
the chosen gain must not lose real ACARS decode yield (measured at the *operating* LO over
several passes) versus the manually-found good gain (43.4/33.8 today). Precision is worthless
if the converged point is wrong.

## IMPORTANT 4 — "Garbage monotone in gain" underpins the no-λ elegance but is unmeasured.

`§1.2` claims "lowest gain on plateau = min garbage" *because* garbage is monotone-increasing
(finding #3). But finding #3 rests on **two points** (43.4 and 33.8 dB) plus a partial third
(29.7). Behaviour at 44/48/49.6 dB (tuner saturation) and any mid-band RFI riser is
unmeasured. If garbage is **non-monotone** (a plausible AGC/compression bump at high gain, or
an RFI-driven spike at a specific gain), then "lowest plateau gain" no longer minimizes
garbage and the elegance argument fails silently — there is no λ or guard that would catch it
(the purity guard is WARN-only, §1.2). *Note this is second-order:* `G` is **not used in v1
selection** (only implicitly via the lowest-gain rule), so a non-monotonicity mis-justifies
the design rather than breaking the picker directly. Still, the "no λ needed" claim should be
downgraded to "assumed, unverified above 34 dB." **De-risk:** extend the one-time manual
sweep to the full 0–49.6 dB range once and plot G(gain) before relying on monotonicity.

## IMPORTANT 5 — Downtime cost is understated; passive estimator likely wins on ROI.

"15 min/day ≈ 1% decode downtime" (§5) is a **wall-clock** figure, but yield is bursty: 15 min
parked on IRA can straddle **one or two entire ACARS passes**, and passes are where ~all daily
decodes live. At a site seeing ~2 passes / 18 min, missing one pass is not 1% — it can be
**5–15% of that day's ACARS**. Meanwhile the design's own **passive drift monitor** (§5,
~50 lines, no RF actions, reads `get_decode_counts` + `bch_failed_from_hist` deltas at the
*operating* gain/LO) measures the thing we actually care about with **zero downtime and on the
correct signal population** — sidestepping Critical 2 entirely. The cost/benefit favors making
the passive estimator primary and the active sweep rare/opt-in, not the reverse.

## IMPORTANT 6 — Quiesce churn: 5× the cycles, unproven at cadence (design is honest here).

~40 pause/set/resume per sweep vs 8 today, daily and unattended. The LO-rescan's
DMA-internal-heap-churn safety "is not yet confirmed" (`autotune.h:37-39`, backlog #7), and
project memory repeatedly ties `rate=0` stalls to DMA-INT pressure. Mitigating facts: gain
quiesce does **not** realloc the DDC/URB pool (unlike a retune), and only 2 LO hops per sweep
(same as today). So the incremental risk is the 40× **bulk-stream pause/resume**, not buffer
realloc. The design correctly defers this to device-smoke gate 2 — acceptable **provided the
soak actually watches `Pre-stream DMA-internal heap free=` and `rate>0` across all ~200
cycles**, per the memory notes, not just "no crash."

## MINOR — `bch_failed_from_hist` is pre-Chase and double-counts.

`hist_bch_record()` runs at `:800` **before** the Chase-2 rescue (`:807-824`) and before the
post-Chase `s_bursts_bch_failed++` (`:861`). So a frame that fails hard-BCH but is
Chase-recovered lands in `F` (a fail bin) **and** in `D` or `U`. `G = ΔU + ΔF` therefore
double-counts Chase-recovered frames. **Harmless for v1** (G is telemetry-only, not used in
selection) and the design footnotes it — but the `D − 0.15·G` telemetry score (§1.2) and the
passive monitor's `G/(D+G)` ratio (§5) inherit the bias; document it where those are used.

---

## VERDICT: **NEEDS-REDESIGN**

Two independent Critical findings both invalidate the **core objective and its measurement**,
and they compound in the same (downward-bias) direction:

1. **Count starvation (Critical 1):** at this install's real pass rate, D_max ≈ 6–8 (not 16),
   the Poisson band swallows the plateau floor, the picker degenerates to "always pick the
   lowest arm," and gain drifts systematically down into deafness. The interleaving fixes
   exposure fairness but not total count — 1.7 passes cannot be averaged.
2. **Wrong signal (Critical 2):** the SNR knee is measured on strong IRA and applied to weak
   ACARS; the IRA knee sits below the ACARS knee, so even a *correct* pick is too low for the
   mission signal. The correct-LO variant makes the count problem worse, not better.

And the acceptance gate (Important 3) cannot detect either failure.

The interleaved round-robin idea is genuinely better than the shipped contiguous-dwell sweep
and worth keeping. But **do not build the 5-arm knee-finder as specified.** Before ~400 lines
go in, the redesign must:

- **Prove the picker on the host against the *measured* pass process** (not a sustained λ) —
  gate 1 reseeded per Critical 1. If the pick collapses to "lowest arm," stop.
- **Abandon ±1-step plateau resolution.** The data supports at most a **coarse 3-state
  decision** (deaf / good / saturating) with a wide guard band and a fixed **margin above the
  low edge** (never sit *on* the knee). Collapse the objective to that.
- **Measure at the operating ACARS LO** (accept the 1–2 h background-sweep cost) or prove a
  *stable* IRA→ACARS offset across RFI/thermal/geometry — a single cross-check is not proof.
- **Make the passive drift estimator (§5) the primary mechanism** and the active sweep a rare,
  triggered fallback; it measures the right signal at zero downtime.
- **Replace the acceptance metric** with an ACARS-yield-preservation gate at the operating LO.

If the host simulation (gate 1, reseeded) shows the coarse 3-state picker is robust at
D_max ≈ 6–8 and the ACARS-LO measurement is adopted, this can drop to
BUILD-WITH-REVISIONS — but that is a materially different, smaller design than the one under
review.
