# Demod diff vs gr-iridium on the HydraSDR ACARS slice (2026-07-17)

**Question.** Our P4 decodes only ~34% of the LW.DA frames it demodulates
(lw_da_valid/lw_da = 610/1803 over the 9.5 h soak). Are the CRC failures
(a) SNR-edge — we fail the same weak bursts gr-iridium also fails — or
(b) a systematic demod gap — we fail bursts gr-iridium decodes cleanly
(a fixable bug in CFO/timing/UW/PLL/BCH)?

**Verdict: SNR-edge.** Our chain decodes **99.6% (244/245)** of the
gr-iridium CRC:OK IDA bursts whose true in-window SNR is ≥ 12 dB, and
the failure probability is a smooth monotonic function of true SNR
(19% at <7 dB → 100% at ≥16 dB, 50% point ≈ 8.6 dB). There is **no
failure cluster at good SNR and no stage-specific bug**. What the data
does show is a quantified *sensitivity delta at the threshold edge*:
gr-iridium's float demod still produces CRC:OK frames down to ~5–7 dB
in-window SNR where our Q15 chain's post-demod bit errors overwhelm the
BCH. That is worth a few dB and ~21% of gri's clean decodes on this
capture — an optimization opportunity (soft-decision/PLL/precision at
threshold), not a bug.

Equally important: **the 60–66% lwda_bad is air-truth, not our demod.**
gr-iridium itself gets CRC:OK on only 568/1393 = 40.8% of the IDA
frames it demodulates on this same capture (device: 33.8%). Of gri's
825 CRC-FAIL IDA demods, we demodulate 805 (97.6%) to the **same
outcome** (BCH+header OK, CRC fails) — the frames are genuinely
corrupt on air.

## Method

Ground truth: `~/iridium_capture/ACARS_SLICE_20260717_164657.ci16`
(HydraSDR RFOne, ci16_le, 10 MSPS, center 1622 MHz, 180 s) and the
gr-iridium/iridium-parser decode of that same file (`.parsed`,
6968 burst lines, 1393 IDA, 568 CRC:OK — every CRC:OK line in the file
is IDA).

1. `tests/scripts/demod_diff_extract.py` parses each burst line
   (time, abs freq, gri SNR, type, CRC verdict), seeks the memmapped
   ci16 (never loads the 7.2 GB), rotates by (freq − 1622 MHz) with
   exact-integer phase math, low-passes with gr-iridium's input filter
   (Kaiser, cutoff 20 kHz / transition 40 kHz / 40 dB — the same design
   `direct_if_dump.py` uses) and decimates 40× to 250 ksps int16, with
   gri's burst_downmix padding (1.64 ms pre, 16 ms post). No per-burst
   rescaling (original capture count units throughout).
2. `tests/host/test_demod_diff_gri` (CMake target) runs every window
   through the production `burst_pipeline_process_burst()` (D13 running
   normally, no force-start) → `iridium_frame_classify` → `ida_decode`,
   and attributes each burst to a stage:
   SHORT / NO_UW / NO_DEMOD / NO_DA (frame demodulated but classify ≠
   LW.DA) / DA_BCH / DA_HDR / DA_CRC / DA_OK. DA_OK == the device's
   `lw_da_valid` criterion.

Reproduce:

```
/usr/bin/python3 tests/scripts/demod_diff_extract.py \
    --parsed .../ACARS_SLICE_20260717_164657.parsed \
    --ci16   .../ACARS_SLICE_20260717_164657.ci16 \
    --out <dir> --tag-offset-ms 25.79
cmake --build tests/host/build-mac --target test_demod_diff_gri
tests/host/build-mac/test_demod_diff_gri <dir> [--csv]
```

### Timestamp calibration (important gotcha)

The parser's ms-since-start timestamps are offset **−25.79 ms** from
the ci16 file's sample clock — constant over the whole 180 s (measured
on 12 strong bursts spread across the file: mean +25.79 ms, stdev
0.02 ms; burst leading edge = tag + offset). Cause UNVERIFIED (most
plausibly samples dropped between stream start and file-write start in
the sniffer). Without the correction every window is pure noise —
`--tag-offset-ms` must be measured per capture. The three reassembled
ACARS in `.acars.jsonl` (81.2 s / 92.1 s / 178.5 s at 1618.752 /
1618.751 / 1624.776 MHz) match IDA CRC:OK lines at exactly those
times/freqs, validating the time-field interpretation (field 3 = ms
from file start).

## Results

Population: all 6968 gri burst lines extracted and run; decisive set =
568 IDA CRC:OK lines.

### By gr-iridium's SNR estimate (misleading — see below)

| gri SNR | n | ours DA_OK | ours % | dominant failure |
|---|---|---|---|---|
| <18 | 46 | 30 | 65.2% | NO_DA 10, DA_BCH 4 |
| 18–22 | 172 | 103 | 59.9% | NO_DA 57, DA_BCH 7 |
| 22–26 | 177 | 143 | 80.8% | NO_DA 28, DA_BCH 3 |
| 26–30 | 133 | 132 | 99.2% | NO_UW 1 |
| ≥30 | 40 | 40 | 100.0% | — |
| **total** | **568** | **448** | **78.9%** | |

The 18–26 dB dip looks non-monotonic — but gri's per-line SNR estimate
is on a different scale from the in-channel reality (its 22–26 dB rows
measure ~9–11 dB burst-RMS/noise-RMS in the extracted 250 ksps window;
cf. the known "ours ≈ gri − 4 dB" tagger-threshold scale mismatch, and
it varies by channel). Re-stratifying by **measured** in-window SNR
(burst RMS samples 500–2300 / trailing-noise RMS samples 3000–4300):

### By measured in-window SNR (the real curve)

| true SNR (dB) | n | ours DA_OK | ours % | failures |
|---|---|---|---|---|
| <7 | 37 | 7 | 18.9% | NO_DA 26, DA_BCH 1, NO_UW 3 |
| 7–8 | 56 | 17 | 30.4% | NO_DA 27, DA_BCH 8, NO_UW 4 |
| 8–9 | 53 | 30 | 56.6% | NO_DA 17, DA_BCH 4, NO_UW 2 |
| 9–10 | 75 | 59 | 78.7% | NO_DA 15, DA_BCH 1 |
| 10–12 | 102 | 91 | 89.2% | NO_DA 9, NO_UW 2 |
| 12–16 | 183 | 182 | 99.5% | NO_DA 1 |
| ≥16 | 62 | 62 | 100.0% | — |

Smooth, monotonic, saturating at 100% — the SNR-edge signature the
brief defined. Zero systematic failures at good SNR.

### Cross-checks

- **Same-frame proof:** all 448 of our DA_OK decodes match gri's
  `ctr` and `len` header fields exactly (0 mismatches) — we decode the
  same frames, not window neighbors.
- **Frequency band is not an independent factor:** the worst band
  (1624.5 MHz, 36.8% raw) has median true SNR 7.5 dB; controlling for
  true SNR ≥ 10 dB, every band scores 94–100%.
- **gri CRC-FAIL agreement:** of gri's 825 IDA demod-but-CRC-fail
  lines, ours: DA_CRC 805, NO_DA 14, DA_BCH 5, NO_UW 1 — 97.6% land at
  the identical stage, and we "rescue" 0 to clean CRC (gri rescues 0 of
  our set by construction). Both decoders agree these frames are
  corrupt on air.
- **Failure-stage anatomy of our misses:** NO_UW is only 11/568 —
  D13 start-find, coarse CFO, and the UW correlator lock essentially
  always. The dominant miss mode (NO_DA 95, DA_BCH 14) is a frame that
  demodulates (UW verified by qpsk_demod) whose LCW or payload BCH then
  fails — i.e. distributed bit errors at threshold SNR, not a
  stage-localized defect.
- **Aggregate consistency:** ours 448/1393 = 32.2% clean on gri's IDA
  windows vs gri 40.8% vs the device soak's 33.8% (different burst
  population — device tagger; corroborating, not identical).

## Implications

1. **No demod code bug to chase.** The remaining lever at the demod
   level is threshold sensitivity (~3–4 dB at the edge vs gri's float
   chain): candidates are soft-decision use ahead of/inside the BCH
   chain, PLL behaviour at low SNR, and Q15 quantization in the
   RRC/pre-rotate path. Expected yield if fully closed: +120 clean
   LW.DA per 568 (≈ +27% relative), which squares to ≈ +60% relative on
   2-burst ACARS completion.
2. **The ~34% lw_da_valid ratio is close to what the air offers**:
   gr-iridium on a HydraSDR gets 40.8% on the same spectrum. Treat
   ~41% as the practical ceiling for lw_da_valid/lw_da, not 100%.
3. gri's per-line SNR field is NOT calibrated to in-channel SNR —
   don't stratify by it without a scale check (it reads ~13 dB high on
   this capture and varies by channel).

## Caveats

- Host harness ≠ device front end: windows come from the HydraSDR
  10 MSPS capture through a scipy Kaiser LPF; the device runs an RTL
  dongle at 2.5 MSPS through `rotate_to_dc` + `direct_if_decim`, with
  tagger-placed windows. This experiment isolates
  `burst_pipeline_process_burst` + classify + BCH/CRC (the stages the
  lw_da_valid ratio measures); front-end deltas on-device can only add
  to the SNR-edge loss, in the same smooth way.
- The −25.79 ms tag offset was measured, not derived; its cause is
  UNVERIFIED. It is constant (stdev 0.02 ms), so alignment error does
  not contaminate the comparison.
- In-window SNR uses the trailing 5.2 ms of each window as the noise
  reference; occasional co-channel energy there adds ~±1 dB noise to
  the x-axis, not to the outcome.

## Files

- `tests/scripts/demod_diff_extract.py` — window extractor + manifest.
- `tests/host/test_demod_diff_gri.c` — harness (CMake target
  `test_demod_diff_gri`; measurement tool, not a regression gate).
- Session outputs (scratchpad, regenerate via commands above):
  per-burst CSV (`full.csv`), summary (`full_summary.txt`).
