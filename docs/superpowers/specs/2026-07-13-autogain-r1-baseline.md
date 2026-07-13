# Autogain R1 — first passive-telemetry baseline (2026-07-13)

Fable's v2 review (`2026-07-13-autogain-v2-review.md`) made R1 the do-first,
free step: measure the real decode rate before setting any Part-B constant, and
reconcile the ~20× contradiction in the source docs (54–66/min deployed vs
2.6–13/pass sim). Done — **from live telemetry, no firmware**.

## Method (reusable, no-flash)
The P4's `iot_log` UDP stream (STATUS lines, port **4210**) and `acars_push` JSON
(port **5005**) were redirected from the offline NUC (`192.168.1.206`) to this Mac
(`192.168.1.64`) via one `POST /config` (`out_host`, `iot_log_host`, `ota_url`;
reboots). A Mac-side listener (`tools`/scratch `rate_soak.py`) parses the STATUS
stream into a decode-rate time series. **NB: the device's NVS config now points
udp+OTA at the Mac** — revert to the NUC (or keep) at redeploy.

STATUS field semantics (verified from the raw stream, mixed and easy to get wrong):
- **per-interval** (sum them): `bch_dec`, `bch_unk`, `drops`, `bursts`
- **cumulative since boot** (take last−first): `lwda`, `lwda_bad`, `sbd`

## Baseline — 57.8 min, LO 1626 MHz (IRA region), autotune-set gain

| signal | total | rate | meaning |
|---|---|---|---|
| `bch_dec` | 925 | **16.0/min · 961/h** | all BCH-passing frames |
| `bch_unk` | 479 | 8.3/min · 498/h | BCH false positives (junk) |
| real (`bch_dec−unk`) | 446 | 7.7/min · 463/h | genuine Iridium (control + IDA) |
| `lwda` | 73 | 1.26/min | IDA/ACARS-carrying frames |
| `lwda_bad` | 43 | — | |
| `drops` | 15366 | 266/min | worker queue drops (flood) |
| completed ACARS (`:5005`) | **0** | 0 | none — matches operator ground truth |

- **Passes:** 7 detected in 58 min (~1 per 8 min). Per-minute `bch_dec` floor ≈ 8,
  in-pass peak **142/min**. Per-pass `bch_dec`: mean 120, sd 86, **CV = 0.72**.

## The three R1 conclusions

1. **The "20× contradiction" is apples-vs-oranges — resolved.** "54–66/min" is the
   **frame** rate during a pass peak (measured in-pass peak 142/min, sustained
   16/min) — NOT completed ACARS. The sim's "2.6–13/pass" is **completed ACARS**,
   which is **0** here (and per the operator, ~never anywhere). Two different
   quantities; no real contradiction. **The mission signal that actually exists is
   the frame rate, not completed ACARS.**

2. **B4 acceptance MUST change (Fable R5, now evidence-backed).** With completed
   ACARS ≈ 0 there is no ACARS yield to preserve, so the autogain objective and
   acceptance test must be defined on a **measurable frame proxy**: `bch_dec`
   frame-yield (+ `lwda` IDA-frame yield) at the operating LO. "Preserve ACARS
   yield" is untestable and must be struck.

3. **Part A's garbage trigger is NOT dead-on-arrival (Fable Important-2 ceiling
   concern).** Measured **Ḡ = bch_unk/bch_dec = 0.52**, well below the 0.85 ceiling
   where a `+0.15` step couldn't fire — headroom +0.33. Caveat: the STATUS stream
   carries `bch_unk` but not the pre-Chase `bch_failed` (F); the true Ḡ including F
   is higher and needs the on-device counter. But the visible junk fraction alone
   already refutes "trigger saturated at this LO."

## What this hands the design
- **CV = 0.72** (pass-to-pass) confirms Fable R5 quantitatively: an *unpaired* A/B
  gain comparison at n=5 passes/arm has 1σ ≈ 45% of the mean → can't see a 20–40%
  erosion. **Paired within-pass A/B is required.**
- Part B constants can now be seeded from real rates: ~7 passes/h, in-pass
  `bch_dec` ~30–140/min, quiet floor ~8/min. `MIN_COUNTS` on `bch_dec` (not ACARS)
  is reachable in ~1 pass; on `lwda` it needs many passes (1.26/min).

## Caveats / next
- **One LO only (1626, IRA region), one gain, ~1 h, 7 passes.** Not the ACARS-dense
  LO (~1620.6) — the `lwda` rate there is unmeasured and likely higher. Re-run the
  same soak at 1620.6 for the ACARS-transport comparison when the device is back.
- The soak ended because the device was **powered off for a cable** (intentional,
  not a crash). Method + listener are proven and reusable.
