# Demod-gap phase 0: where the bits die vs gr-iridium (2026-07-17)

**Question** (follow-up to `docs/2026-07-17-demod-diff-hydrasdr.md`):
for the ~120 IDA frames gr-iridium decodes CRC:OK from the HydraSDR
slice but WE fail, is the loss (A) a small number of bit errors per
BCH codeword — just over the hard-decision t=2 limit, fixable by
soft-decision — or (B) fundamentally noisy demod (8+ errors), fixable
only by upstream precision work?

**Verdict: A — and more than half the gap isn't even bit errors in the
payload.** The single biggest loss is our strict UW/LCW *acceptance
gates* discarding frames whose 312-bit data section is already fully
decodable by the existing hard BCH. The remaining data-section
failures sit predominantly at 3–4 errors in the worst codeword (t=2
misses by 1–2 bits). Q15 fixed-point precision contributes **nothing
measurable** (float64 stage-swap ablation: zero outcome changes), and
PLL α=0.2 is already optimal on this corpus. No precision or PLL work
is warranted.

## Method

Instrumented `tests/host/test_demod_diff_gri.c` (`--dumpbits`,
`--ida-only`) to emit every demodulated frame's raw pre-BCH bits +
per-bit soft metrics; `tests/scripts/demod_diff_biterr.py` then:

1. Reconstructs the TRUE transmitted 200-bit message of every gri IDA
   CRC:OK line from the iridium-parser pretty print (lossless for
   header fields + payload + CRC field + tail), re-encodes the 10
   ACCH BCH(31,20) codewords (poly 3545) and the frame's UW/LCW
   context, and compares against our raw demod bits per codeword.
   **Validation:** all 568/568 reconstructions pass the CRC-16
   residue check; all 448 frames we decode DA_OK show exactly 0 UW
   errors, 0 LCW distance, and ≤2 errors per data codeword — i.e. the
   truth model, index maps, and line-to-manifest alignment are exact.
2. LCW words use exact syndrome coset-leader tables (min distance to
   any codeword — exact for lcw1 whose truth is the constant ft=2
   codeword; a tight lower bound for lcw2/lcw3 where the pretty print
   is lossy — flagged UNVERIFIED as an absolute error count).
3. Chase analysis: per failed codeword, flip subsets of the L
   least-reliable bits, syndrome-decode (≤2), check against truth.

Reproduce (extraction dir per the earlier doc, `--tag-offset-ms 25.79`):

```
cmake --build tests/host/build-mac --target test_demod_diff_gri
tests/host/build-mac/test_demod_diff_gri <dir> --ida-only --dumpbits dump.txt
~/iridium_capture/.venv/bin/python tests/scripts/demod_diff_biterr.py \
    --parsed ACARS_SLICE_20260717_164657.parsed \
    --manifest <dir>/manifest.csv --dump dump.txt --chase-l 6
```

## Result 1 — failure decomposition (120 gold misses)

| cause | frames | notes |
|---|---|---|
| LCW gate only (UW + data clean) | 46 | data section fully decodable |
| UW+LCW gates only | 15 | data section fully decodable |
| UW gate only | 7 | data section fully decodable |
| LCW gate + data errors | 27 | |
| data errors only | 14 | |
| no demod frame (NO_UW) | 11 | true SNR floor |

**68/120 (57%) fail only on acceptance gates.** The classifier
requires a bit-exact 24-bit UW (`iridium_frame.c:373`, although
qpsk_demod deliberately accepted ≤2 symbol errors) and CLEAN divides
on lcw1/lcw3 + a clean lcw2 pad-guess (`classify_lw`) — no ECC repair,
mirroring iridium-toolkit's *default* mode. gri's own toolkit has a
`--harder` mode that repairs exactly these. Observed distances on the
109 demodulated misses: lcw1 {0:65, 1:37, 2:7}, lcw2 {0:54, 1:40,
2:13, 3:2}, lcw3 {0:23, **1:82**, 2:4} — nearly all 1–2 bits.

## Result 2 — error-per-codeword histogram (the A-vs-B discriminator)

41 frames have ≥1 uncorrectable data codeword (err > 2):

| worst codeword errors | frames | | uncorrectable cws total | count |
|---|---|---|---|---|
| 3 | 22 | | 3 errors | 61 |
| 4 | 9 | | 4 errors | 22 |
| 5 | 4 | | 5 errors | 6 |
| 6 | 3 | | 6 errors | 5 |
| 7 | 3 | | 7 errors | 4 |

Bad codewords per frame: 1 bad ×17, 2 bad ×14, ≥4 bad ×10. So ~31/41
are marginal (A); ~10 are deep failures (B-ish tail, likely residual
timing/phase transients — they are also the ≥5-error rows).

**Error anatomy:** error bits cluster as runs of exactly TWO
consecutive symbols (252 of 352 runs) — the signature of an isolated
DQPSK hard-decision slip at one symbol corrupting the differential
bits of itself and its successor. Consequently a bit's reliability is
NOT its own symbol's `|pll_out|` (the current `soft_bits` metric —
under it many error bits rank as high-confidence): it is
`min(|sym_i|, |sym_{i-1}|)`. With that pair-min metric, Chase-6 on
the failed codewords recovers the true codeword into the candidate
set for **20/41** frames (oracle; a CRC-16-arbited candidate search
approaches this) and picks it outright by soft cost for 9/41.

## Result 3 — stage-swap ablation: precision & PLL exonerated

Env-gated float64 replacements (host-only diagnostics, kept in tree):
`BP_FLOAT_RRC` (unquantised taps, float accumulate),
`BP_FLOAT_ROT` (all three Q15 rotations), `BP_FLOAT_INTERP`
(sub-sample interp/decim), `PLL_ALPHA` (loop-gain override).

| config | gold DA_OK |
|---|---|
| baseline (Q15) | 448/568 = 78.9% |
| float RRC | 448 |
| float rotations | 448 |
| float interp/decim | 448 |
| all three float | 448 (0 stage changes on 1393 IDA windows) |
| PLL α = 0.05 / 0.1 / 0.15 | 362 / 424 / 441 |
| PLL α = 0.25 / 0.3 / 0.4 | 449 / 447 / 436 |

The float paths verifiably engage (per-burst UW-SNR diagnostics shift)
yet change no outcome: at burst amplitudes ~500–1700 counts the Q15
truncation floor is orders below the channel noise at 5–9 dB SNR. The
UW correlator and CFO estimator already run float on host and target
(`CORR_USE_FLOAT_FFT=1`), and the PLL is float — the "Q15 vs gri
float" hypothesis from the previous doc is dead.

## Recommendation (ranked by expected DA_OK gain / effort)

1. **CRC-gated acceptance relaxation** (+66–68 gold frames, 448→~516
   = 90.8% of gri; ~+15% relative lw_da_valid, ≈ +33% on squared
   2-burst ACARS completion). Waive the exact-UW gate for frames
   qpsk_demod already direction-verified, and run
   `iridium_bch_repair1/2` on lcw1/lcw2/lcw3 (exactly iridium-toolkit
   `--harder`), letting the existing ida_decode chain (10× BCH +
   zero1 + CRC-16) arbitrate. False-accept probability of the full
   chain on junk ≈ 0.243¹⁰·⅛·2⁻¹⁶ ≈ 1e-12 per frame. Control-flow
   only — trivially PIE-compatible, runs only on frames that already
   failed classification (no hot-path cost). Effort: small.
2. **Symbol-domain Chase-2 BCH on the data codewords** (+9 easy /
   +15–20 with CRC-arbited candidate combos, on top of #1 → ~531–536
   = ~94% of gri). Key design input: reliability = pair-min of
   adjacent symbol magnitudes (or hypothesize quadrant slips of the
   weakest symbols directly — each hypothesis fixes 2 symbols' bits at
   once). Integer-only comparisons on existing int16 soft_bits;
   fires only after hard decode fails (~small ×41-frame cost). Effort:
   moderate. Beats gri's hard-decision toolkit at equal demod quality.
3. **Not worth building:** Q15→float precision work in RRC / rotate /
   interp (0 measured gain), PLL retune (α=0.2 already optimal, +1
   frame at 0.25), input re-scaling.

Residual after #1+#2: ~32 frames = 11 NO_UW + ~21 deep-error (≥5-bit)
frames — the genuine sensitivity floor on this capture.

## Implemented — recommendation #1 (CRC-gated acceptance relaxation)

Production change (device-portable, no host-only `#ifdef` around the
logic), `common/iridium_decoder/iridium_frame.{c,h}`:

- `iridium_frame_classify` refactored: the exact-24-bit-UW gate no
  longer early-returns. Strict dispatch (MS/TL/BC/LW/RA, clean divides)
  runs only when the UW is bit-exact — byte-identical behaviour for
  every strict-classifiable frame. When strict leaves the frame
  UNKNOWN, a **DA-only `classify_lw_harder` fallback** runs (default ON,
  toggled by `iridium_frame_classify_set_harder`): it skips the exact-UW
  gate (qpsk_demod already verified ≤2 symbol errors) and BCH-*repairs*
  the LCW words — `iridium_bch_repair1` on the t=1 lcw1 (poly 29),
  `iridium_bch_repair2` on lcw2 (poly 465, both pad guesses) and lcw3
  (poly 41) — mirroring iridium-toolkit `bitsparser.py:352-364`. It
  forwards ONLY `ft==2` (DA); ida_decode's 10×BCH + zero1 + CRC-16 is
  untouched and remains the sole arbiter. Runs only on strict-rejected
  frames → off the hot path; PIE-irrelevant (bit ops only).

Harness-verified on `test_demod_diff_gri` + the 568/825 ground truth:

- **A — yield:** 448 → **514 DA_OK (+66)** = 90.5% of gri's 568
  (predicted +66–68). The 2-frame shortfall vs the +68 upper bound is
  the deliberate t=1 `repair1` on lcw1 (repair2 there would exceed the
  BCH(7,3) design distance and risk miscorrection / false-accepts).
- **B — safety:** **0 new false-accepts** across all 825 gri
  CRC-FAIL IDA frames (was 0, still 0). The CRC arbiter rejects every
  repaired-but-wrong LCW. This is the load-bearing result.
- **C — no regression:** `--no-harder` reproduces **448 bit-identically**;
  full host suite **54/54 pass** (classifier corpus tests included).

Device A/B still required before ship (host gain necessary, not
sufficient — per `feedback_device_smoke_mandatory_after_dsp`).

## Prototyped — recommendation #2 (CRC-arbitrated Chase-2, task #16 ref)

Host prototype `tests/scripts/chase_crc_bch.py` — the exact reference a
device C port would mirror. Runs on OUR demod bits + int16 soft metrics
(no ground truth in the decision path); truth only *scores* the result.
Per frame that fails hard-decode after `--harder`: reliability =
pair-min of adjacent DQPSK symbol magnitudes; for each FAILED data
codeword take the L least-reliable positions, enumerate 2^L flip
patterns, syndrome-decode (≤2), collect distinct candidate messages;
combine candidates across failed codewords into whole-frame hypotheses;
accept iff a hypothesis passes the REAL frame CRC-16 + zero1/da_len gate
(ida_decode's DA_OK criterion). Bounded by a per-frame CRC-check cap.

### A / B / C by L (gold=568 on top of --harder's 514; safety=825)

| config | DA_OK | +chase | % of gri | gold→wrong | nak false-accept | max checks/frame |
|---|---|---|---|---|---|---|
| L=4 (uncapped) | 514 | **+9** = 523 | 92.1% | 0 | **0** | 8 |
| L=5, cap 256 | 514 | **+11** = 525 | 92.4% | 0 | **0** | 11 |
| L=5 (uncapped) | 514 | +12 = 526 | 92.6% | **1** | 0 | 3072 |
| L=6, cap 256 | 514 | +14 = 528 | 93.0% | 0 | **1** | 61 |
| L=6 (uncapped) | 514 | +16 = 530 | 93.3% | **2** | **1** | 15078 |

Cost (C): only ~11 safety-population frames enter chase at all; at the
L=5/cap-256 operating point they spend 339 CRC-16 evals TOTAL (expected
2^-16 collisions ≈ 0.005). Recovered-frame checks: median 1, max 11.
Trivially within any device budget (CRC-16 over 26 B ≈ a few hundred
cycles; a handful of frames per window, post-hard-fail only).

### Verdict: closer to +9 than +20 — real, safe, cheap, but modest.

The oracle's +20 (Phase-0 "truth-in-candidate-set at L=6") is **not
safely reachable**. Reaching those extra frames needs search depth at
which 16-bit CRC arbitration starts admitting WRONG CRC-valid frames —
visible as both gold miscorrections (L=5 uncapped: 1; L=6: 2) and a
safety-population false-accept (L=6, at *any* cap — an early collision
no cap removes). The `zero2==0` structural check (widening the arbiter
16→20 bits) did NOT help — the colliding candidates satisfy it too. **The
16-bit CRC is the binding constraint, not the chase search.**

**Chosen operating point: L=5, CRC-check cap = 256** → +11 (525/568 =
92.4% of gri), 0 gold-miscorrection, 0 safety false-accept. Ultra-safe
alternative: L=4 uncapped (+9, ≤8 checks). Do NOT run L=6.

**Task #16 recommendation:** worth porting only if the ACARS-completion
metric needs every point AND the port is cheap — it's +11 frames (+2.0%
absolute lw_da_valid on top of --harder, ≈ +4% on squared 2-burst
completion), not the +20 the oracle hinted. The large win was #1
(--harder, +66); Chase adds a safe, bounded increment. Device C shape:
hook in `ida_decode.c` right after the 10-block hard loop on the
`!out->ok` path only; integer-only (min() over existing int16
soft_bits + existing iridium_bch syndrome table + existing
crc16_ccitt_false); candidate scratch ≤ 10×2^L 20-bit ints; hard
per-frame CRC-check cap = the safety knob. UNVERIFIED on device and on a
single 180 s capture — the CRC-16 collision floor is probabilistic, so
the check cap (not L) is the load-bearing safety control on a larger
corpus.

## Caveats

- Single 180 s capture, bench antenna; per project rule, treat these
  as raw phase-0 numbers, not a durability claim. The *decomposition*
  (gate losses vs BCH margin) is structural and should generalise;
  the exact +66/+20 counts may not.
- lcw2/lcw3 "distance" is min-distance-to-code (exact as a distance;
  lower bound as an error count) — UNVERIFIED against per-line truth,
  which the pretty print doesn't carry for those words. lcw1/UW/data
  comparisons are exact-truth.
- Device front end (RTL 2.5 MSPS + rotate + direct_if_decim + tagger
  windows) differs from this harness's HydraSDR/scipy front end; gate
  relaxation gains apply to whatever the device demodulates, but the
  absolute counts on-device remain UNVERIFIED until a device A/B.
- gri parsed-line SNR is uncalibrated (see previous doc); stage table
  uses it only as a coarse stratifier.

## Files

- `tests/host/test_demod_diff_gri.c` — `--dumpbits <file>`,
  `--ida-only` (this work).
- `tests/scripts/demod_diff_biterr.py` — truth reconstruction +
  bit-error/Chase analysis (this work).
- `common/iridium_decoder/burst_pipeline.c`, `uw_correlator.c`,
  `qpsk_demod.c` — host-only env-gated ablation switches
  (`BP_FLOAT_ROT/INTERP/RRC`, `PLL_ALPHA`), `#ifndef ESP_PLATFORM`,
  default behaviour bit-identical (20/20 related host tests pass).
- Session outputs (scratchpad): `dump_baseline.txt`,
  `biterr_l6.csv`, `abl_base.csv`, `abl_allfloat.csv`.
