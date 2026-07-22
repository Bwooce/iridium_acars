# New-site commissioning: siting a P4 Iridium/ACARS receiver in a blind location

One procedure for standing up a receiver somewhere new, folding together the two
things a blind site needs to answer: **is this spot usable at all** (reception-
environment heuristic) and **where do I park the LO** (band strategy). Both are
backed by this project's data; see the companion docs for the derivations:
`2026-07-22-reception-environment-heuristic.md` and the 40 h analysis summarised in
the `reference_freq_coverage_analysis` memory.

## The two questions, and why they're separate

- **Environment** — near-zero decode at a new site has three very different causes:
  the band is quiet, the signal is weak (SNR), or there's *interference* (energy that
  isn't Iridium). The fixes differ (wait / re-aim / **move away from the emitter**),
  and they're indistinguishable from a raw decode count. The device now classifies
  this itself (`/status` → `reception.state`).
- **Frequency** — the RTL hears only LO ± 1.25 MHz of the 10 MHz Iridium band, and
  ACARS-bearing IDA traffic concentrates in a narrow, *time-stationary* hump. You park
  the LO at that hump's centre.

Do environment first: there's no point optimising the LO for a spot that's drowning in
interference or has no sky view.

## Step 1 — Is the spot usable? (reception heuristic)

Point the HC610 helix at **clear sky (zenith, vertical)**, bias-tee ON, then read
`/status` (browser dashboard shows a coloured banner; `curl` gives the JSON):

| `reception.state` | meaning | action |
|---|---|---|
| **good** | UW-lock + decode healthy | proceed to Step 2 |
| **marginal** | Iridium locks but frames too weak (SNR-limited, air-truth) | improve sky view / antenna; still workable |
| **interference** | lots tagged, almost nothing reaches BCH — energy present but not Iridium | **re-site**: clear sky, away from noise sources; recheck |
| **quiet** | few bursts (idle band or weak coverage) | wait a few minutes / check the feed is live |

Discriminator = `reception.uw_reach` (of the bursts the worker demod-attempted, the
fraction that reached BCH). Low with high `tagged_ema` = interference. The state uses
~20 s EMAs + an 8-window dwell so a single satellite pass can't flip it; give it ~30 s
to settle after any move.

**Interference sources to rule out (suburban context):** the HC610 pre-filters
out-of-band well (>60 dB <1570 MHz, >80 dB >1660 MHz), so cellular / GPS L1 / DME /
L-band radar are handled *at the antenna*. What it can't stop is (a) noise coupled
*downstream* of the antenna — coax pickup, the SDR's USB/host SMPS, PoE, the P4's own
supply; (b) poor helix orientation (RHCP needs sky-pointing; sideways picks up
terrestrial multipath that tags but won't demod); (c) in-band / filter-skirt emitters
it passes (Globalstar handset uplink 1610–1626.5, MSS/satphone terminals 1626.5–1660).
Gain does **not** fix interference — verified flat across 3.7/7.7/12.5 dB.

## Step 2 — Where to park the LO (band strategy)

**Park, don't steer.** The IDA hump is stationary on the hour-average even though the
instantaneous hotspot sweeps with each beam (~1 min); a causal steerer is anti-predictive
and even a perfect hourly oracle beats a fixed park by only ~5 points. Keep
`autotune_run_lo_rescan` **disabled**.

**Blind default:** start at **1620.5–1621.0 MHz** while the survey runs — the hump sits
in the lower-middle duplex band at every site (regulatory floor at 1618, simplex/no-ACARS
at 1626). The exact centre shifts a few hundred kHz by region, so measure it:

- **Preferred — HydraSDR wideband survey:** center 1622 MHz, ≥10 MSPS, parse with
  iridium-toolkit `-o line`, histogram IDA-frame frequency (field 3), pick the LO that
  maximises the ±1.25 MHz fraction (this is `~/iridium_capture/process_hour.py`'s
  `best_lo`). The distribution *shape* is reliable even with a poor antenna. Use ci16/cf32
  — **not cu8** (spectral-inversion bug mirrors frequencies; see the cu8 bug doc).
- **On-device only:** `POST /scan` (scanner.c) sweeps the LO in 1.25 MHz steps. Must
  **integrate over many sweeps / hours** — a single sweep lands on a near-random beam.
  Aggregate the density maps, park on the integrated peak. (Nit: the scan currently starts
  at 1616 MHz; the bottom ~1.8 MHz is legally empty — 1618 is a better start.)

**Survey window:** 24–72 h is enough (per-day centres converge to <0.7 MHz); reserve the
full ~10 days only if the 3-day centre is still visibly drifting. Then park via serial
`set lo_hz <hz>` + reboot (or scanner `hop` live) and confirm in `/status` (`lo_freq_hz`).

**Ceiling:** a 2.5 MHz window captures ~58–61% of IDA at best — ~40% is outside any single
window at once. No LO strategy recovers it; only wider instantaneous bandwidth or the
multi-receiver path does. Judge success by IDA/ACARS *rate*, not by chasing the ceiling.

## Step 3 — Keep it parked, re-check on drift

One-time determination at commissioning; then re-survey only on a trigger, not a schedule:

- **Trigger:** 7-day trailing-median IDA-frames/hour inside the current ±1.25 MHz window
  drops below **60% of the commissioning baseline** (7-day median rejects the normal ±13-pt
  hourly swing). Use IDA rate — not all-burst (IRA-contaminated) or completed-ACARS (too
  sparse to be stable).
- **Unconditionally** re-run Steps 1–2 on any antenna / site / hardware change.
- Optional: a quarterly wideband survey to catch seasonal/constellation drift; re-park only
  if the centre moves >0.3 MHz.

## Caveats

- The band-strategy numbers are from **one location** (suburban Sydney bench, 3–40 h). The
  band-edge structure is global; the exact peak is a regional/traffic inference until a
  second site is measured. Re-measure per site rather than trusting 1620.6.
- The reception thresholds ship as conservative defaults calibrated on this bench's
  good/marginal/interference anchors; `reception.uw_reach`/`decode_frac`/`fail_frac` are
  exported raw in `/status` so a new site can re-tune them.
