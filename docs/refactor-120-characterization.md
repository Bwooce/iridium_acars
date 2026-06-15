# #120 refactor — characterization baseline

Before refactoring `uw_correlator` / `direct_if_decim` / `dsp_processor` to
take explicit context (no behavior change intended), these characterization
tests pin the *exact* current output so any drift is caught. Captured
2026-06-15 from commit `8a41b4f` (pre-#120).

## Host golden tests (run every CI build, `ctest`)

- **`test_uw_correlator_golden`** — runs the D13 + RRC + UW-correlator pipeline
  on the compiled-in ALBQ fixture and asserts EXACT: D13 burst_start, an FNV-1a
  checksum of the RRC-filtered sample buffer, UW offset, direction, and the
  CFO/SNR/peak float fields (tight relative epsilon). Baseline in the file's
  `G_*` `#define`s.
- **`test_direct_if_decim_golden`** — feeds a deterministic LCG int16 stream
  (no external fixture) through `direct_if_decim_process` and asserts the exact
  output sample count + FNV-1a checksum.

Both are registered in `tests/host/CMakeLists.txt` (`uw_correlator_golden`,
`direct_if_decim_golden`) and pass as part of the 25-test host suite.

Re-baselining: if the algorithm changes *intentionally*, run the test — it
prints a copy-pasteable `GOLDEN CAPTURE:` block — and update the `G_*` defines.

## Device characterization (`dsp_processor`)

`dsp_processor.c` is not host-buildable (esp-dsp deps), so its gate is the
device smoke test (`CONFIG_SMOKE_TEST_MODE=y` + `CONFIG_SMOKE_TEST_CORPUS=y`).
Baseline captured on ESP32-P4 v1.3 hardware, commit `8a41b4f`:

```
===== SMOKE_PASS =====
Result: bursts=14  strongest peak_bin=1040  snr=24.78 dB
corpus DC-window hit: peak_bin=1040 snr=24.78 dB
Perf: DSP/frame total=462 us (wind=61 fft=245 mag=42 detect=19 base=92), 585 frames
```

**Refactor gate:** after the `dsp_processor` change, rebuild smoke mode, flash,
and confirm `SMOKE_PASS` with the corpus DC-window hit at the same bin. The
burst count / exact SNRs are deterministic (synthetic IQ) and should match;
treat any divergence as a regression. This is also the `Smoke-verified:` trailer
the pre-push hook requires for DSP-path commits.

## Procedure for the refactor

1. Confirm host suite green (`ctest` → 25/25) and capture a fresh smoke baseline.
2. Refactor ONE module at a time; rebuild + run host golden tests after each.
3. After `dsp_processor`, run the device smoke and compare to the baseline above.
4. Only commit once all golden tests + smoke match bit-for-bit.
