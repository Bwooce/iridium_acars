# Autogain redesign — interleaved knee-finding gain calibration

**Date:** 2026-07-12 · **Status:** design proposal (supersedes the gain-cal half of
[2026-07-08-autotune-design.md](2026-07-08-autotune-design.md); the LO-rescan half is unchanged)
**Problem:** the shipped gain sweep (`autotune_run_manual()`, `p4-usb-host/main/autotune.c`)
picks erratic gains run-to-run (observed 20.7 / 28.0 / 38.6 / 43.4 dB) because its
per-gain 180 s dwell measures *whether a satellite pass happened to land in that
window*, not the gain. It costs ~24 min with decode parked on the IRA LO, and is
currently disabled in production for exactly this instability.

## 0. What the 2026-07-12 manual gain search established (design inputs)

Manual sweep 43.4 → 40.2 → 33.8 → 29.7 dB on the deployed unit, ~18 min windows:

1. **Decode yield is satellite-pass-bursty**: ~0 real decodes between passes,
   54–66/min during a pass every few minutes. Pass timing is the dominant noise
   source in any per-dwell count. This is the core problem.
2. **Front-end DSP load is gain-independent** (~93% dsp, ~2.3 bursts/s quiet-period
   detection at every gain — the tagger's adaptive baseline re-learns any floor).
   Gain does NOT buy or cost front-end CPU. CPU is not part of the objective.
3. **Garbage scales with gain**: bursts that demod but fail integrity
   (`bch_failed` + `bch_unknown`) fell −38% total bursts / −20% unknown from
   43.4→33.8 dB. Garbage is monotonically increasing in gain over the region of
   interest.
4. **Real decode is roughly flat 43.4→33.8** (66 vs 54/min peaks — within
   pass-to-pass variance). The knee (where real decode actually craters) was not
   yet located; it is below 33.8 or near 29.7.
5. The tradeoff is **sensitivity vs false-alarm rate**, not CPU.

Consequence: the gain–decode curve is a **wide plateau with a knee at the low end**
(and eventual saturation loss at the very top, per the 2026-07-08 IRA curve:
0/2/**6**/0/1 at 8/15/25/35/44 dB). Trying to rank gains *inside* the plateau is
statistically meaningless at any affordable dwell; the useful, findable object is
the **plateau's lower edge**. The algorithm below is a knee-finder, not a
peak-finder.

## 1. Objective function

### 1.1 Inputs (all existing, all cumulative/delta-safe)

Per measurement slice, snapshot before/after (both are plain reads — they do NOT
steal counts from status_logger, unlike `worker_core1_get_stats()` which is
reset-on-read and owned by the STATUS drain in `class_driver.c:775`):

- `worker_core1_get_decode_counts(&decoded, &unknown)` — cumulative
  `bch_decoded` (passed BCH **and** classified to a known frame type = real
  decode) and `bch_unknown` (passed BCH, classified UNKNOWN = near-pure BCH
  false positive).
- `worker_core1_get_histograms(&h)` — cumulative. `bch_failed` is derived from
  the BCH outcome histogram `h.bch[16]`, bin `(e1+1)*4 + (e2+1)`, `e = -1` =
  uncorrectable:

  ```c
  // BCH-uncorrectable frames: e1 == -1 (bins 0..3) or e2 == -1 (bins 4, 8, 12).
  static uint32_t bch_failed_from_hist(const worker_histograms_t *h) {
      static const int fail_bins[] = {0, 1, 2, 3, 4, 8, 12};
      uint32_t s = 0;
      for (int i = 0; i < 7; i++) s += h->bch[fail_bins[i]];
      return s;
  }
  ```

  Caveat: `hist_bch_record()` (worker_core1.c:800) records the **pre-Chase**
  outcome, so Chase-2-recovered frames count in both `failed` and `decoded`.
  Fine for a tiebreak/diagnostic; do not treat `failed` as exact.

Per candidate gain (arm) *i*, accumulated over all its slices:

- **D_i** = Δ`bch_decoded` — real decodes. Primary signal.
- **G_i** = Δ`bch_unknown` + Δ`bch_failed` — garbage. Diagnostic / secondary.

### 1.2 The objective: lowest gain on the decode plateau (knee rule)

Because the interleaving in §2 gives every arm **identical total exposure and
near-identical pass exposure**, raw accumulated `D_i` are directly comparable —
no per-arm rate normalization is needed.

```
D_max    = max_i D_i
plateau  = { i : D_i >= THETA * D_max  -  K * sqrt(D_max + 1) }    // THETA = 0.75, K = 2
chosen   = lowest-gain member of plateau
```

- The `K·sqrt(D_max+1)` term is the Poisson noise band: two arms are
  "statistically tied" when `|D_a − D_b| <= 2·sqrt(D_a + D_b)`; the plateau
  band is the one-sided version of that anchored at the max.
- `THETA = 0.75` encodes "we accept up to ~25% true decode loss as the price of
  the knee" — deliberately loose because finding #4 says the real curve is flat,
  so genuine plateau members lose ~nothing; only cratered (deaf/saturated) arms
  fall below 0.75·D_max − band.
- **Garbage enters implicitly**: garbage is monotone-increasing in gain
  (finding #3), so "lowest gain on the plateau" *is* "min garbage subject to
  no significant real-decode loss". No λ to tune.

**Why not `D − λ·G` (the penalty form)?** It is the same idea, but λ has units
that depend on pass intensity: a pass inflates D and G by different factors, so
a λ tuned on one sweep mis-weights the next. The constrained/knee form is
scale-free and needs no tuning. (If a penalty form is ever wanted for a
one-number telemetry score, `D − 0.15·G` matches the observed slopes, but it is
not used for selection.)

**Why not decode purity `D/(D+G)` as the objective?** Already ruled out: a
nearly-deaf gain is very pure. Purity is kept as a **sanity guard** only: if the
chosen arm's purity is < 0.5× the best arm's purity, log a WARN (something is
pathological — e.g. an RFI riser at that gain) but still apply the knee rule.

**Why not SNR-histogram objectives?** Evaluated, rejected as primary:

- `snr[]`/`snr_pushed[]` counts in high bins are still pass-driven for real
  frames (real bursts only exist during passes), so they inherit the same
  timing noise as decode counts — and between passes `snr_pushed[]` is
  gain-flat by finding #2 (the adaptive baseline makes quiet-period detection
  SNR gain-invariant), i.e. it carries **no gain information at all** when
  there's no traffic.
- Median decoded-frame SNR is confounded by ADC compression: above the plateau
  the reported SNR *drops* (27.9 → 20.5 dB in the 2026-07-08 curve) — useful as
  a **saturation telltale**, not as a maximization target (maximizing it would
  under-gain, same failure as purity).
- Kept as telemetry: per-arm decoded-SNR max and spread are logged each sweep;
  an arm whose max decoded SNR is ≥6 dB below a lower-gain arm's is flagged
  `saturated` in the log (diagnostic only in v1).

**How pass timing is normalized out:** not by a statistic at all — by the
measurement schedule (§2). Equal exposure is the normalizer; every statistic
computed on top of it is automatically pass-fair in expectation.

## 2. Pass-timing robustness — interleaved round-robin slices

Replace "one contiguous 180 s dwell per gain" with **many short slices per gain,
round-robin across all candidate gains, repeated for R rounds**:

```
for round r in 0..R-1:
    for each arm i (in rotated order, start index = r mod n_arms):
        set gain_i (quiesced) → reset baseline → 3 s prime discard
        snapshot counters; dwell SLICE_S; snapshot counters; accumulate into arm i
```

Why this beats the alternatives:

- A satellite pass lasts ~1–4 min. A full round of 5 arms at (20 s slice + 3 s
  prime) is ~115 s — **shorter than a pass**, so any pass sprays decodes across
  *all* arms in roughly equal shares instead of landing wholesale in one arm's
  window. Over R = 8 rounds (~8 passes-worth of wall time) the per-arm pass
  exposure converges to equal. The between-dwell pass variance that dominates
  the current sweep becomes *shared* variance that cancels in arm-vs-arm
  comparison.
- **Rotating the arm order each round** (cyclic shift) removes the residual
  phase bias (arm 0 always being measured at round-start times).
- **Much longer dwells** (the other fix) need ~1 h per gain to average over
  pass arrivals — 5+ h per sweep. Rejected on cost.
- **Pass-invariant ratio statistics** (per-burst decode fraction) are
  self-normalizing but under-gain as a sole objective (§1.2). Not needed once
  exposure is equalized.
- The cumulative histograms don't help by themselves — they accumulate
  *through* the same schedule, so they are exactly as pass-exposed as the
  counts; it is the interleaving that makes them fair, not their cumulativeness.

Slice length: **SLICE_S = 20 s** (config). Must dominate the 3 s prime (13%
overhead) and give the re-primed tagger baseline a stable floor; must stay well
under pass duration. 15–30 s all work; 20 s is the default.

Reference LO: the sweep still hops to `autotune_ira_lo_hz` (1626.2 MHz IRA)
as today — IRA is the densest, most constant reference, so counts accumulate
~5–10× faster than on the ACARS LO between passes. IRA is still
beam/pass-modulated (which is why the current sweep is erratic despite using
it), but interleaving fixes that regardless of LO. *(Alternative — run the
sweep at the operating ACARS LO so real decode never pauses: works with the
identical algorithm, but needs ~5–10× more rounds for the same counts, i.e. a
1–2 h background sweep. Viable as a future opt-in; not the default.)*

## 3. Search strategy — fixed incumbent-centered arm set, no mid-sweep adaptivity

**Candidate set (5 arms):** the incumbent gain plus offsets in R828D **table
steps** (`AUTOTUNE_R828D_GAINS[]`, autotune_gainset.h):

```
offsets = { -6, -4, -2, 0, +2 }        // table indices relative to incumbent
```

clipped to `[autotune_gain_min_dbx10, autotune_gain_max_dbx10]` and
de-duplicated. E.g. incumbent 43.4 dB (table idx 24) → arms {33.8, 37.2, 40.2,
43.4, 44.5} (idx 18/20/22/24/26). This brackets the suspected knee region
(finding #4: knee is at or below 33.8) on the first run and walks further down
on subsequent runs.

Why this shape and not the alternatives:

- **Full/coarse 29-step sweep** (current): wastes rounds on obviously deaf
  gains (0–15 dB) and can't afford enough exposure per arm. Rejected.
- **Golden-section / ternary**: assumes a noiseless unimodal objective; a
  Poisson evaluation at ~15 counts routinely inverts adjacent comparisons and a
  bracket-discard search then discards the wrong half permanently. Rejected.
- **Multi-armed bandit / successive halving**: adaptive elimination *mid-stream*
  breaks the equal-pass-exposure property that §2 exists to provide (an arm
  dropped after round 2 was measured against a different pass distribution
  than the survivors' rounds 3–8). Sound only if elimination happens at round
  boundaries with paired stats — that is a legitimate v2 optimization
  ("racing": after round 4, drop any arm with `D_i < 0.25·D_max − band` and
  redistribute its slices), but v1 stays non-adaptive for trustworthiness.
- **Hill-climb with hysteresis** is what the *cross-run* behavior amounts to
  (§4): each run moves the incumbent at most a few steps toward the knee, so
  successive daily runs walk down the plateau edge and then sit still. The
  per-run measurement is a fixed design; the adaptivity lives between runs,
  where the pass distribution has fully re-randomized.

**Recenter-up rule:** if the *highest* arm is the only plateau member (incumbent
somehow ended up below the knee, e.g. after an RFI environment change), the run
moves up (§4) and the scheduler is asked to re-run after 1 h instead of the
normal interval, so recovery upward takes hours, not days.

**Auto-extend rule (count adequacy):** after R rounds, if `D_max <
AUTOTUNE_MIN_COUNTS` (default 12), run up to R more rounds (once). If still
short, declare no-signal: keep incumbent, persist nothing (same philosophy as
the current `have_signal` guard).

## 4. Stability across runs — incumbent-sticky hysteresis

The chronic failure to beat is thrash between statistically-tied gains. Rules,
applied in order after `chosen` is computed:

1. **No signal** (`D_max < AUTOTUNE_MIN_COUNTS` after auto-extend): keep
   incumbent, do not persist, WARN log. (Mirrors current behavior.)
2. **Chosen == incumbent**: done, nothing persisted.
3. **Deadband (the tie case):** if the incumbent is itself on the plateau,
   move only if `chosen` is **≥2 table steps below** the incumbent. A 1-step
   difference between plateau members is inside measurement noise by
   construction — staying put is free and stops thrash. (This implements
   "prefer the lowest gain within measurement-noise of the peak" *with*
   hysteresis: we prefer lower, but only when the step down is meaningful.)
4. **Incumbent off-plateau** (deaf or saturated): move toward `chosen`
   regardless of step count — off-plateau membership already encodes a
   statistically significant decode deficit (`D_inc < 0.75·D_max − 2√(D_max+1)`).
5. **Movement clamp:** apply at most **4 table steps per run** toward `chosen`
   (walk, don't jump). One anomalous sweep can then mis-step the gain by at
   most ~5 dB, recovered by the next run — the same mean-reversion posture the
   LO side adopted after the 53 h HydraSDR analysis.
6. **Persist** the applied gain via the existing `autotune_persist(false, gain)`
   internal-stack task **only when it actually changed** (autotune_sched's task
   stack is PSRAM; a direct `nvs_commit` there is the ~24.7-min crash-loop —
   the delegation pattern is mandatory and already in place).

Acceptance metric for the redesign: **5 consecutive sweeps on the deployed unit
must land within ±1 table step** (vs today's 20.7–43.4 dB spread).

## 5. Cost & scheduling

**Per-sweep cost:** 5 arms × (20 s + 3 s) × 8 rounds ≈ **15.3 min** (vs ~24 min
today), during which real ACARS decode is parked on the IRA LO. Exposure per arm
= 160 s. Expected counts on the IRA reference at plateau gains ~6/min (2026-07-08
curve: 6 decodes / 55 s at the 25 dB peak) → **D_i ≈ 16 on-plateau, ≤2 deaf** —
separable: `16 − 2 = 14 > 2·sqrt(18) ≈ 8.5`. The auto-extend rule covers
installs where the IRA rate is lower.

**Cadence:** gain drift is RFI/thermal — slow. Default
`autotune_gain_interval_s = 86400` (daily; 15 min/day ≈ 1% decode downtime),
`autotune_on_boot` unchanged (optional). Early re-run (1 h) only on the
recenter-up flag (§3).

**Passive drift trigger (complement, recommended):** rather than re-running on a
blind clock, a ~50-line monitor riding the existing hourly telemetry snapshots
(cumulative `get_decode_counts` + `bch_failed_from_hist` deltas at the
*operating* gain/LO) tracks the garbage ratio `G/(D+G)` and the daily decode
total against the post-calibration baseline; if the garbage ratio drifts by more
than an absolute +0.15 for 3 consecutive hours, or the 24 h decode total falls
below half of the trailing week's median, request an early recalibration. Cheap,
no extra RF actions, catches RFI-environment changes days sooner than a fixed
interval. (v1 can ship without it; the hook is trivial to add to
`autotune_sched_task`'s poll loop.)

**Fully-passive gain control** (dither the operating gain ±1 step during normal
operation, no dedicated sweep) was evaluated and rejected as the primary
mechanism: at ACARS-LO decode rates a single ±1-step comparison needs hours of
paired exposure for significance (pass burstiness again), it perturbs
production reception continuously, and a ±1-step decision is exactly the
comparison §4 declares unresolvable. Revisit only if sweep downtime ever
matters at 1%/day.

## 6. Concrete algorithm

New pure header `autotune_knee.h` (host-testable, like `autotune_gainset.h`)
holds `build_arms()`, `pick_plateau_low()`, and the hysteresis rule; device code
in `autotune.c` replaces the body of `autotune_run_manual_locked()`. All entry
points, the busy guard (`autotune_try_begin/end`), the MANUAL-gain-mode refusal,
scan-progress publication (`scan_begin/scan_end`, `s_scan_gain_dbx10`), and
`autotune_sched.c` are unchanged.

```c
// ---- config (NVS, defaults) ----
// autotune_slice_s        = 20     // per-slice dwell
// autotune_rounds         = 8      // rounds (auto-extends once to 16 on low counts)
// autotune_gain_interval_s= 86400  // daily
// (autotune_gain_dwell_s / autotune_gain_stride retire with the old sweep;
//  autotune_gain_min/max_dbx10 and autotune_ira_lo_hz are reused as-is)

#define ARM_OFFSETS      {-6, -4, -2, 0, +2}   // R828D table-index offsets
#define N_ARMS_MAX       5
#define PLATEAU_THETA_PCT 75                   // integer percent
#define PLATEAU_K         2
#define MIN_COUNTS       12                    // D_max gate (auto-extend below this)
#define DEADBAND_STEPS   2                     // min table-step move when incumbent on plateau
#define MAX_STEPS_PER_RUN 4

static void autotune_run_knee_locked(void)
{
    app_config_t cfg; app_config_snapshot(&cfg);
    if (cfg.gain_mode != GAIN_MODE_MANUAL) { /* refuse, as today */ return; }

    uint32_t saved_lo   = cfg.lo_freq_hz;
    int      incumbent  = class_driver_get_tuner_gain_dbx10();
    if (incumbent < 0)  incumbent = cfg.gain_db_x10;
    incumbent = autotune_snap_gain(incumbent);
    int inc_idx = r828d_index_of(incumbent);            // table index (new helper)

    // Arms: incumbent-centered offsets, clipped to [gain_min, gain_max], deduped.
    int arm_gain[N_ARMS_MAX]; int n_arms = build_arms(inc_idx, cfg, arm_gain);
    if (n_arms < 2) { /* refuse: degenerate range */ return; }

    uint32_t D[N_ARMS_MAX] = {0}, U[N_ARMS_MAX] = {0}, F[N_ARMS_MAX] = {0};
    bool     ok[N_ARMS_MAX]; for (i) ok[i] = true;

    if (scanner_hop(cfg.autotune_ira_lo_hz, false) != ESP_OK) return;  // abort, unchanged

    int rounds = cfg.autotune_rounds;
    scan_begin(1, (int64_t)n_arms * rounds *
                  (AUTOTUNE_PRIME_MS + cfg.autotune_slice_s * 1000) * 1000);

    for (int pass = 0; pass < 2; pass++) {              // pass 1 = auto-extend
        for (int r = 0; r < rounds; r++) {
            for (int k = 0; k < n_arms; k++) {
                int i = (r + k) % n_arms;               // rotate order each round
                if (!ok[i]) continue;
                if (!class_driver_set_gain_quiesced(arm_gain[i])) { ok[i] = false; continue; }
                atomic_store(&s_scan_gain_dbx10, arm_gain[i]);
                scanner_reset_baseline();
                vTaskDelay(pdMS_TO_TICKS(AUTOTUNE_PRIME_MS));   // discard prime transient

                uint32_t d0, u0, d1, u1; worker_histograms_t h0, h1;
                worker_core1_get_decode_counts(&d0, &u0);
                worker_core1_get_histograms(&h0);
                vTaskDelay(pdMS_TO_TICKS(cfg.autotune_slice_s * 1000));
                worker_core1_get_decode_counts(&d1, &u1);
                worker_core1_get_histograms(&h1);

                D[i] += d1 - d0;  U[i] += u1 - u0;
                F[i] += bch_failed_from_hist(&h1) - bch_failed_from_hist(&h0);
            }
            ESP_LOGI(TAG, "round %d/%d: D=[...] G=[...]", r + 1, rounds);  // running curve
        }
        if (max_u32(D, n_arms) >= MIN_COUNTS || pass == 1) break;
        ESP_LOGW(TAG, "low counts (D_max=%u); extending %d more rounds", ...);
    }

    // ---- selection (pure, host-tested) ----
    uint32_t Dmax = max_u32(D, n_arms);
    bool     have_signal = (Dmax >= MIN_COUNTS);
    int      chosen = incumbent;
    if (have_signal) {
        // plateau: D[i]*100 >= THETA_PCT*Dmax - 100*K*isqrt(Dmax+1)  (integer form)
        int lo = -1, inc_on_plateau = 0;
        for (int i = 0; i < n_arms; i++) {
            if (!ok[i]) continue;
            bool on = on_plateau(D[i], Dmax);
            if (on && (lo < 0 || arm_gain[i] < arm_gain[lo])) lo = i;
            if (on && arm_gain[i] == incumbent) inc_on_plateau = 1;
        }
        int cand = arm_gain[lo];
        int step_delta = inc_idx - r828d_index_of(cand);        // >0 = moving down
        if (cand == incumbent)                             chosen = incumbent;
        else if (inc_on_plateau && abs(step_delta) < DEADBAND_STEPS)
                                                           chosen = incumbent;  // tie: stay
        else                                               // clamp walk to 4 steps/run
            chosen = AUTOTUNE_R828D_GAINS[clamp(r828d_index_of(cand),
                                                inc_idx - MAX_STEPS_PER_RUN,
                                                inc_idx + MAX_STEPS_PER_RUN)];
        // purity sanity guard (WARN only) + saturation telltale logged here
        // recenter-up flag: chosen above incumbent -> ask sched for a 1 h re-run
    }

    // ---- restore + persist (unchanged discipline) ----
    scanner_hop(saved_lo, false);
    class_driver_set_gain_quiesced(chosen);
    scanner_reset_baseline();
    if (have_signal && chosen != incumbent)
        autotune_persist(false, chosen);      // internal-stack task: PSRAM-stack rule
    ESP_LOGI(TAG, "=== knee-cal done: incumbent %d.%d -> chosen %d.%d dB; "
                  "curve D=[..] G=[..] plateau=[..] ===", ...);
    iot_log(IOT_LOG_WARN, "AUTOTUNE-DONE gain-cal ...");  // wireless: UDP log + /status
    scan_end();
}
```

Notes:

- **Counters**: only `worker_core1_get_decode_counts()` and
  `worker_core1_get_histograms()` (both plain cumulative reads) — never
  `worker_core1_get_stats()` (reset-on-read, owned by status_logger).
- **Gain writes**: `class_driver_set_gain_quiesced()` per slice (pause → set →
  resume), ~40 quiesce cycles per sweep vs 8 today — needs a one-time
  device-smoke soak (the LO-hop equivalent was validated 42/42 in the scanner
  work, but gain-quiesce at this cadence hasn't been).
- **Arithmetic**: all-integer selection (integer `isqrt`, percent-scaled
  plateau test) so `autotune_knee.h` unit tests are exact on host; float would
  also be acceptable here (low-rate control), but integer keeps host/device
  bit-identical.
- **Telemetry**: per-round running curve to ESP_LOG + final curve via
  `iot_log`; `/status` continues to show scan type/ETA/current-test-gain via
  the untouched `autotune_scan_status()` / `autotune_scan_cur_gain_dbx10()`.

### Validation plan (gates before enabling the scheduler)

1. **Host**: unit-test `autotune_knee.h` selection + hysteresis against
   synthetic Poisson pass-train traces (simulate passes as 2 min bursts of
   rate-λ decodes every 5–8 min, replay 100 sweeps): assert pick spread ≤1 step
   and zero thrash between plateau-tied arms. Positive control: feed it the
   2026-07-08 measured curve (0/2/6/0/1) and today's flat-plateau data; it must
   pick 25.4-ish and ≤33.8 respectively.
2. **Device**: 5 consecutive on-demand sweeps → picks within ±1 table step
   (acceptance metric, §4), plus stream-health check (no wedge, no `rate=0`)
   across the ~200 quiesced gain sets.
3. Only then flip `autotune_gain_interval_s` back on (daily).

## Open questions / risks

1. **IRA→ACARS gain transfer** (biggest): the knee measured on the IRA LO
   (1626.2 MHz, strong constant broadcast) may sit at a different gain than the
   ACARS-band knee (weaker frames, different RFI) — today's manual flat-plateau
   evidence was gathered at the operating LO, the sweep measures at IRA. If
   validation shows the IRA knee is offset, either (a) apply a fixed
   step-offset margin (+1–2 steps above the IRA knee), or (b) run the identical
   algorithm at the operating LO with 3–4× rounds as a slow background sweep.
   Decide with one manual cross-check: run the sweep on both LOs back-to-back.
2. **IRA count-rate budget per install**: 160 s/arm gives ~16 counts only if
   this install sees ~6 IRA decodes/min at plateau gain. The auto-extend rule
   doubles exposure once; if a site is still short, the sweep correctly
   declares no-signal and keeps the incumbent — but then autogain never
   converges there. Mitigation if seen: raise `autotune_rounds`.
3. **Quiesce churn**: 40 pause/set/resume cycles per sweep is 5× the current
   rate; validated-safe for retunes, unproven at this cadence for gain sets —
   covered by validation gate 2.
