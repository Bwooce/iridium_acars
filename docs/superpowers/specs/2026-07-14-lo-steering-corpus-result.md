# LO steering — decisive corpus result (2026-07-14)

Fable's reframed test (`2026-07-13-autogain-v2-review.md` follow-on): bound the value
of *all* LO-steering schemes with one measurement on a fresh wideband corpus. Done.

## Corpus
HydraSDR RFOne on the Mac (built the whole capture toolchain from source — libhydrasdr
+ SoapyHydraSDR + arm64 iridium-sniffer, since the NUC was down). 45 min @ 10 MSPS,
center 1622 MHz → **37,517 frames, of which 7,427 IDA** (the ACARS-bearing duplex data),
spread 1618.05–1625.79 MHz. Parsed with iridium-toolkit (`-o line`).
Files: `scratchpad/corpus.bits` (17 MB), `corpus.parsed`.

## Result — 2.5 MHz window (the P4 RTL), fraction of IDA frames captured

| strategy | IDA captured | vs fixed |
|---|---|---|
| **Fixed park @ 1620.60 MHz** | **58.6%** | — |
| Causal steer (use prev 5-min bin's best window) | 45.7% | **−22% (worse)** |
| Causal steer (prev 15-min bin) | 44.9% | −23% (worse) |
| Oracle re-pick / 15 min (perfect foreknowledge) | 63.6% | +8.5% |
| Oracle re-pick / 5 min (perfect foreknowledge) | 73.8% | +25.9% |

## Verdict: **LO steering is UNSOUND — park the LO. Confirmed empirically.**
- A **causal** steerer (the only kind buildable) performs **worse than a static park** — it
  reaches −85% of the oracle's headroom. The best 5-min window is **anti-predictive** of
  the next: IDA occupancy is mean-reverting, so chasing it loses. The best any dynamic
  scheme can achieve is to *average back into the fixed park*; it cannot beat it.
- The oracle's +25.9% is a **mirage** — it needs foreknowledge no predictor has (Fable
  F4). At a realistic 15-min cadence even the oracle is only +8.5% (below worth-it).
- **What the specific center 1620.60 does NOT prove:** 45 min ≈ 4–5 passes of a ~100-min
  orbital cycle — too short to call any center *durable*. Two agreeing time-averaged
  estimates (2026-07-08 and today) are suggestive, not conclusive. The *time-averaged*
  best center is plausibly more stable than the *instantaneous* hotspot (which swings
  0.5→295 in 1 min, 2026-07-09), but that needs a multi-hour/multi-day duration-weighted
  corpus to establish — NOT claimed here.
- **What survives regardless of the center — the structural result:** a causal steerer
  underperforms a static window because the hotspot is *anti-predictive* bin-to-bin
  (mean-reversion, corroborated by the 2026-07-09 swings). "Chasing loses to sitting
  still" doesn't depend on which center → **"don't steer" holds; "park exactly at
  1620.6" is only a 45-min point estimate.**

## Actionable win for the deployed P4
The hourly LO-rescan (`autotune_run_lo_rescan`) does the harmful causal-chase — worse,
it scans by **all-burst density (IRA-dominated)**, so it wanders to 1626 (IDA≈0), as seen
all night (1626→1616→1623.5→1619.75→1618.5…). **It captures less IDA than a static park.**
→ **Disable the LO rescan and park LO at 1620.60 MHz.** Free improvement to IDA-frame
capture *and* a stable window for multi-frame SBD reassembly (a wandering LO breaks
reassembly — frames of one message land in different windows).

## Caveats
- Still reception-bound: this improves *IDA-frame capture*, but completed ACARS ≈ 0 is
  driven by SNR + multi-frame completion (see `project_r1_baseline_and_udp_telemetry`),
  so parking is an incremental win, not a fix.
- 58.6% is the ceiling for a 2.5 MHz window; the other ~41% of IDA is simply outside any
  single window — only wider instantaneous bandwidth captures it.
- FUTURE (Fable's crumb): LCW `handoff_resp`/IIP frames (1,816 in the corpus) announce
  where an *already-decoding* session moves — the only signal to *follow* (not predict)
  traffic. Not worth it at ACARS≈0, but quantifiable from this corpus if ever revisited.
