# Autotune (periodic RF recalibration) — design

**Date:** 2026-07-08 · **Status:** design capture (reception path just validated; feature to be built)
**Origin:** the RTL at 2.5 MSPS hears only LO±1.25 MHz of the 10 MHz Iridium band, and both the *optimal LO* (where ACARS/IDA is densest) and the *optimal gain* drift over time. Proven 2026-07-08: retuning LO 1622→1626 (IRA cluster) took the RTL from **0 decodes all night → 8 `bch_decoded` in 75 s** — the decode path works; we were just mis-tuned. See [[reference_freq_coverage_analysis]], [[project_near_dc_tagger_mask]].

## Goal
A periodic on-device routine that keeps the RTL tuned+gained for maximum real decode yield as satellite geometry/traffic shifts — without a human or the companion HydraSDR.

## Key insight: IRA is the calibration reference
IRA (ring-alert) frames are a **fixed, constant, strong** broadcast cluster at **1626.1–1626.3 MHz** (Iridium simplex allocation — does not move with satellites). That makes them the ideal calibration signal: unlike ACARS/IDA (sparse, pass-clustered, unmeasurable in a short window), IRA gives a **constant, gain-sensitive `bch_decoded` rate** to optimize against. Gain tuned on IRA transfers to the ACARS band (same antenna LNA, similar frame structure).

## Empirical findings (53 h HydraSDR, 2026-07-08) — revise the LO strategy
Ran `~/iridium_bits/hydra_cadence.py` + `hydra_predict.py` over 111,486 IDA frames / 53 h:
- **Best-LO drifts ~1.3 MHz median per 10-min window (≈ RTL half-BW), ~78% of windows shift >0.5 MHz** — but the drift is a **random walk with MEAN-REVERSION, not momentum.** Direction-persistence 38.6% (<50%); momentum-extrapolation MAE 3.15 MHz is *70% worse* than persistence (1.86); **mean-revert-to-global-median is the BEST predictor (1.46 MHz).** No exploitable periodicity (weak ~117 min peak only).
- **Design implication:** do NOT build predictive pre-tuning (hurts), and do NOT aggressively chase the instantaneous best-LO every ~10 min (you'd chase transient/noisy excursions that revert). **Anchor the LO to the stable long-term center (~1620.6, 54% IDA)** and re-scan only to track SLOW drift of that center — so `autotune_lo_interval_s` should be LONG (~hourly), not 600 s. The **IRA gain calibration is the high-value part**; the LO side is "find the stable center once, verify occasionally."
- **The 1620.6 optimum is LOCATION-SPECIFIC (this is why we can't hardcode it).** The band *structure* is global/fixed (IRA@1626, IDA 1618–1626), but the *density within it* is driven by (a) which beams are overhead and (b) **local/regional air-traffic** (ACARS follows where the aircraft are). Sydney's optimum ≠ another site's. Satellite geometry averages out over ~a day (global near-polar constellation), but the air-traffic density does NOT — so the per-location optimum persists and differs by site. **This is the core argument FOR autotune: discover the local optimum on deployment, don't ship a hardcoded LO.** Gain (IRA-calibrated) is likewise install-specific (local RFI + antenna) but stable per-install. So autotune = "discover the location/install-specific stable center + track slow drift + IRA-calibrate gain," NOT "chase satellites."
- **Caveats (53 h is under-powered for these):** periodicity (~31 orbital cycles, weak SNR), diurnal air-traffic (~2 cycles), and — untested — **ephemeris-conditioned** predictability (does best-LO track *which specific satellite/beam* is overhead? needs TLEs + weeks of passes). "Not predictable by naive extrapolation" is solid; "not predictable at all" is NOT established. Re-run the tools as capture accumulates; TLE-conditioned analysis is the next predictability probe. **DEFERRED until more data (user, 2026-07-08).**

## Recalibration period — split, informed by Iridium orbital mechanics
The frequency clusters and the optimal gain drift on **different clocks**, so use two intervals, not one:
- **LO / frequency-density re-scan → satellite-driven.** Iridium is ~780 km LEO, orbital period ~100 min, 66 sats/6 planes. The relevant timescale is NOT the period but the **satellite handoff cadence**: a ground point sees a sat for only ~7–10 min before handoff (~9 min typical), and each sat/beam uses different traffic channels — so the dense-ACARS LO shifts on a **~8–10 min** timescale (plus ~±38 kHz Doppler within a pass, small vs the 2.5 MHz window). Default `autotune_lo_interval_s` ≈ **600 s (10 min)**.
- **Gain re-calibration → RFI/thermal-driven, satellite-independent, slow.** The IRA gain sweep is expensive (minutes with proper dwell), and the optimum moves only with the RFI environment / temperature. Default `autotune_gain_interval_s` ≈ **3600 s (1 h)**. (Thrashing it every 10 min would consume the whole LO interval.)
- Both configurable; 0 = disabled. `autotune_interval_h` (single-clock) is superseded by this split.
- **Optional empirical calibration of the period itself:** we have 2 days of HydraSDR `.bits` — bin IDA density into ~10-min windows and measure how much the best-LO actually moves window-to-window to confirm/tune the ~600 s default against real data rather than orbital theory alone.

## The routine (LO re-scan every `autotune_lo_interval_s`; full gain-cal every `autotune_gain_interval_s`)
1. **Save** current LO/gain (restore on failure/abort).
2. **Gain calibration @ IRA:** hop to LO 1626.2. For each gain in `autotune_gain_set` (e.g. {8,15,25,35,44} dB), dwell `autotune_gain_dwell_s` (**configurable**, default e.g. 45 s), count `bch_decoded`+`bch_unknown` and `FRMDEC` frames + their SNR spread. Pick the gain that **maximizes decode rate** (see objective). Longer dwell → less noise in the estimate but slower recal; hence configurable.
3. **LO density scan (ACARS):** sweep candidate LOs across 1618–1626 (using the existing scanner, see [[project_freq_scanner_phase1]]), dwell `autotune_lo_dwell_s` each, rank by IDA/`bch_decoded` yield. Pick the densest ACARS LO. (Until the scanner's DMA-INT churn is fixed, this step may run manually/host-assisted.)
4. **Park** at the best ACARS LO with the calibrated gain. Log the chosen LO/gain + the calibration curve.

## Optimization objective (what "optimal" means)
**Maximize `bch_decoded` rate.** The gain→decode curve is an inverted-U:
- too low → real bursts below ADC noise / BCH threshold → few decodes (this is where "zero USB drops" sits — *clean but deaf*);
- optimal mid → bursts clear of noise, ADC not compressing → peak decodes (**expect some USB drops here — tolerated**);
- too high → 8-bit ADC saturates on strong RFI/signals → clip/distort → demod fails → decodes fall + heavy drops.

Therefore **do NOT minimize USB drops** — they are a *tolerated cost*, bounded only where they start reducing decodes (catastrophic case: SOFTWARE_AGC = 16k drops, 0 decodes). Secondary signals: `bch_unknown`>0 confirms demod is producing valid frames; keep `bch_failed` low *relative* to decoded (high = marginal SNR/clipping); want decoded-frame **SNR spread down to ~15–18 dB**, not just strong 26 dB frames, or we're too deaf for weak ACARS.

## Configurable parameters (NVS, mirror `tag_thr`/`dcmask` plumbing)
- `autotune_gain_dwell_s` — per-gain dwell during calibration (**explicit ask**; default ~45 s).
- `autotune_gain_set` — gains to try (or min/max/step).
- `autotune_lo_dwell_s` — per-LO dwell during density scan.
- `autotune_lo_interval_s` — LO/density re-scan period, default ~600 s (satellite handoff cadence; 0 = disabled).
- `autotune_gain_interval_s` — full gain re-calibration period, default ~3600 s (RFI/thermal drift; 0 = disabled).
- `autotune_ira_lo_hz` — IRA reference LO (default 1626.2 MHz; fixed allocation but keep tunable).
- Objective knobs: drop-tolerance ceiling, min-SNR-spread target.

## Measured gain curve (2026-07-08, LO 1626 IRA reference, 55 s dwell/gain)
First real gain calibration (proves "zero USB drops ≠ optimal decodes"):

| Gain dB | bch_decoded | bch_unknown | maxSNR | rb_full drops | note |
|---|---|---|---|---|---|
| 8.0 | 0 | 0 | — | 278 | deaf |
| 15.0 | 2 | 0 | 25.2 | 0 | clean but DEAF (zero-drop point) |
| **25.0** | **6** | **4** | **27.9** | 2662 | **PEAK** — 3× the decodes of 15 dB |
| 35.0 | 0 | 0 | 20.6 | 0 | saturating (SNR falling) |
| 44.0 | 1 | 2 | 20.5 | 2798 | saturated |

Shape = inverted-U. Optimum ~25 dB (accepts 2662 drops for 3× decodes); SNR degradation 27.9→20.5 above the peak is the 8-bit ADC compressing on the amplified signal. Chosen gain: 25.4 dB (nearest step). Real peak likely between the 22.9 and 28.0 steps — a refine pass there is where extra dwell earns its keep.

### Dwell adequacy — 55 s was NOT statistically sufficient
Counts were tiny (0/2/6/0/1); Poisson error ±√N ⇒ ~40% at N=6, ~70% at N=2. So the gross shape (deaf→peak→saturate) is real but adjacent-gain *ranking* is noisy — 6-vs-2 barely separates. The count RATE is capped by **worker throughput** (~1–2 processed/window), so even against constant/dense IRA we accumulate only ~6 in 55 s. **For a confident pick, dwell until counts ≥~30 per gain → minutes each (~3–5 min).** Hence `autotune_gain_dwell_s` should default **≫55 s** (e.g. 180–300 s), and a two-stage coarse(short)→refine(long) sweep is the efficient shape.

## Gain steps + search method
- **Possible gains (R828D, `librtlsdr.c:967`) — 29 discrete steps (dB):** 0.0, 0.9, 1.4, 2.7, 3.7, 7.7, 8.7, 12.5, 14.4, 15.7, 16.6, 19.7, 20.7, 22.9, 25.4, 28.0, 29.7, 32.8, 33.8, 36.4, 37.2, 38.6, 40.2, 42.1, 43.4, 43.9, 44.5, 48.0, 49.6. A requested `gain_dbx10` snaps to nearest. Sweep the REAL steps, not arbitrary values.
- **Search method — NOT binary search.** The objective (`bch_decoded` vs gain) is an inverted-U (unimodal), so binary search (monotonic-only) is wrong; the "correct" peak-finder is ternary / golden-section. BUT evaluations are expensive (reboot+dwell) and noisy (Poisson decode counts) — a noisy objective fools any divide-and-discard search. With only 29 steps, the robust approach is a **coarse subset sweep (~every 3rd step) with a long dwell (beat Poisson), then refine around the peak.** Ternary/golden-section only pays off if **live gain-apply** (no per-step reboot) is wired first, making a full 29-step sweep cheap.

## Open questions / notes
- **Non-disruptive scheduling:** each hop/gain change currently needs a reboot (LO/gain apply at detector-create); autotune reboots N times per cycle → do it on a low-traffic cadence, or wire live `hop`/live-gain-apply first (dcmask already got live-apply; gain/LO don't). Reboot churn risks the RTL wedge ([[feedback_crashloop_wedges_rtlsdr_tuner.md]]) — prefer live retune.
- **DC artifact at dense LO:** at LO~1620.6 the DC/LO spike lands in the IDA region; do NOT enable the near-DC mask there (real IDA co-located) — rely on EMA absorption.
- **Manual first:** ship a manual `autotune` command (one calibration pass, prints the curve + chosen LO/gain) before wiring the periodic timer, so it can be validated by hand.
- **Measurement channel:** serial STATUS `bch_decoded=` is reliable; HTTP `/diag` flaps (C6 SDIO). Autotune must read its own decode counters internally, not depend on HTTP.
