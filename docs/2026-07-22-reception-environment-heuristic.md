# Reception-environment heuristic + status display (design)

**Status:** design, not yet implemented. Origin: 2026-07-22, an outdoor antenna move
that flooded the tagger with bursts but decoded ~nothing — the funnel shape cleanly
distinguished *interference* from *weak signal*, which is worth surfacing so an operator
(or the device itself) can tell a bad location from a quiet one.

## Motivation

A new device in a new location is **blind**: near-zero decodes could mean quiet traffic,
weak signal (bad sky view / SNR), or RF interference — and the fix differs for each
(wait / re-aim / **move away from an emitter**). Today `/status` shows decode counts but
no classification. This session gave a textbook example: outdoors, **4872 bursts tagged,
~1% passed the prefilter (~49), only 2 reached BCH, 2 decoded** — i.e. the tagged energy
was *not Iridium* (failed the unique-word correlation). Indoors the same burst volume
reached BCH and decoded ~1-2%. That difference is diagnosable.

## The funnel (existing counters, worker_core1.h)

```
tagged (gone_bursts / freq_total)
  → prefilter: bursts_triage_rejected  vs  accepted        (width/duration/channel-SNR)
    → bursts_processed                                      (ran the worker pipeline)
      → demod_ok  (UW correlation found a valid Iridium unique word)   ← KEY GATE
        → reached BCH = bch_decoded + bch_unknown + bch_failed
          → decoded = bch_decoded (+ chase_recovered rescues)
```

`worker_core1.c:858` `if (!bres->demod_ok) return;` — bursts with no valid UW never
reach BCH. So **reached-BCH / processed** (or /prefilter-accepted) = the **UW-lock rate**,
and that's the interference discriminator.

## The three failure modes → the classifier

| state | signature | meaning / action |
|---|---|---|
| **QUIET** | tagged-rate LOW | few bursts — normal lull or weak coverage; just wait |
| **INTERFERENCE** | tagged-rate HIGH **and UW-lock rate LOW** (bursts pass prefilter but fail UW → not Iridium) | non-Iridium energy swamping the front end → **move the antenna / find the emitter**. (The 2026-07-22 outdoor spot: UW-lock ~4% of prefilter-passed vs decode ~0%.) |
| **MARGINAL** | UW-lock OK (bursts reach BCH) but **BCH-fail high / decode low**, chase carrying | real Iridium but SNR-limited (air-truth) → better sky view / antenna would help; not interference |
| **GOOD** | UW-lock + decode both healthy | fine — leave it |

The novel bit vs what we show now: **UW-lock rate separates INTERFERENCE from MARGINAL** —
both look like "low decode," but one says "move away from the noise" and the other says
"you need more signal." Today we can't tell them apart at a glance.

## Implementation sketch (small)

- Compute in `status_logger` (it already aggregates the windowed funnel: `bursts_processed`,
  `bursts_bch_*`, prefilter accept, gone_bursts). Derive per rolling window (~30-60 s, and a
  longer EMA to avoid flapping on a single pass):
  - `tagged_rate` (gone_bursts/s), `uw_lock_rate` = reached_BCH / max(1, processed)
    [or / prefilter-accepted], `decode_rate`, `bch_fail_frac` among reached.
- Classify into {quiet, interference, marginal, good} with hysteresis (don't flip on one
  window). Expose in `/status` as `"reception_state":"interference"` + the raw ratios, and
  render a one-line banner on the HTML dashboard ("⚠ INTERFERENCE — check antenna siting").
- Counters needed: reached-BCH is already cumulative (the `bch{}` block); `processed` /
  `demod_ok` may need a cumulative getter (small — mirror `worker_core1_get_bch_cumulative`).
  Prefer deriving from existing windowed `worker_stats` in status_logger to avoid new state.

## Calibration anchors (from this session — use to set thresholds)

- **GOOD/MARGINAL (indoors):** prefilter-passed bursts reached BCH; decode ~1-2%; chase
  carrying ~86% (marginal-SNR flavour).
- **INTERFERENCE (outdoors 2026-07-22):** 4872 tagged, ~49 prefilter-passed, 2 reached BCH,
  2 decoded — UW-lock ~4%, decode ~0.07%, at every gain (3.7/7.7/12.5 all flat → gain-
  independent, the interference tell).
- **QUIET:** overnight lulls — tagged-rate low, everything else scales down.

Gather a couple more labelled windows (a clean-indoor GOOD, the outdoor INTERFERENCE, a
QUIET lull) to fix the thresholds before trusting the auto-classification.

## Why it matters beyond diagnostics

This is the missing piece for **auto-siting a device in a new location** (see the
companion ACARS-band-strategy research): combined with the frequency scanner, a device
could report "interference here, try elsewhere" vs "weak but clean, park and wait" — turning
the blind-new-location problem into a guided one.

## Notes / risks

- Don't flap: use hysteresis + a multi-window EMA (a single satellite pass shouldn't flip
  the state).
- Interference can be intermittent (periodic emitter) — the state should reflect a window,
  and ideally log transitions so an operator sees "interference started HH:MM".
- Thresholds are site/antenna-relative; ship conservative defaults + make them tunable.
- Implement/deploy when the device is in a recoverable state (not mid-move, off-dock).
