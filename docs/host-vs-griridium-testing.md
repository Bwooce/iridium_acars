# Host pipeline vs gr-iridium DSP comparison

The host pipeline (`tests/host/test_pipeline_direct_if_albq`) and
gr-iridium decoder operate on the same Albuquerque corpus and dump
per-stage IQ to `/tmp/host_signals/` and `/tmp/signals/` respectively.
`tests/scripts/dsp_compare.py` is the single tool used to diff them
and to gate regressions against saved golden vectors.

## When to use which subcommand

| Mode | What it does | Typical use |
|---|---|---|
| `--mode stagewise` | Walks every per-stage host/gri dump pair for one burst and prints NMSE/phase/peak-freq/RMS with pass/fail status. | Day-to-day "did my DSP change regress anything?" check. |
| `--mode squared-fft` | Re-runs gr-iridium's CFO front-end (Blackman → z² → 4096-pt FFT) on the post-D13 dumps from each side and compares the top peaks. | Investigating CFO divergence specifically. |
| `--mode raw --left ... --right ...` | Compares any two cf32/.cfile files with the full metric set. | One-off file diffs; spot checks. |
| `--mode raw --synthesize-worker-test` | Runs the worker DSP chain (stage-1 FIR + 25/8 polyphase + decim 5) on a synthetic 25-ksym RRC burst and compares against `scipy.signal.resample_poly`. | DSP-design equivalence after worker chain changes. |

Burst alignment uses `/tmp/host_direct_if/manifest.csv` (the `gri_burst_id`
column maps each host `burst_idx` to the matching gr-iridium debug id).
Pass `--burst-id <gri_id>` and the tool resolves the host index, or
pass `--host-burst-idx <n>` for the inverse lookup.

## Metrics and thresholds

For each stage we report:

- **NMSE-dB** — `10·log10(||x-y||² / ||y||²)` after `scipy.signal.correlate` integer alignment. Literal definition: scale mismatches show up as a positive NMSE; same-shape signals at matched gain go to −∞.
- **phase_coh** — `|<x,y>| / (||x||·||y||)` in [0, 1]. Insensitive to overall gain; 1.0 = identical up to a complex scalar.
- **Δf_Hz** — peak FFT bin difference (host − gri), parabolic-interpolated on the magnitude spectrum.
- **rms / peak** — secondary diagnostics.
- **lag** — integer sample offset chosen by the correlator.
- **status** — `pass` / `fail` / `skip` against the per-stage thresholds.

Threshold values live in `THRESHOLDS_NMSE_DB`, `THRESHOLDS_PHASE`, and
`THRESHOLDS_PEAK_HZ` near the top of `dsp_compare.py`. They reflect
conventional gr-iridium-port engineering thresholds (channelizer −50 dB,
RRC −40 dB, etc.) — not values tuned to make the current run pass.

## Golden-vector regression

```
# Save (use after confirming a run is good):
python3 tests/scripts/dsp_compare.py --mode stagewise --burst-id 30 \
        --save-golden tests/fixtures/stagewise_burst30.npz

# Check (on every subsequent run):
python3 tests/scripts/dsp_compare.py --mode stagewise --burst-id 30 \
        --check-golden tests/fixtures/stagewise_burst30.npz
```

The npz contains one entry per dumped stage (`<key>_gri`, `<key>_host`,
plus the host-only `channelized_40k_host`). `--check-golden` compares
shape and computes NMSE per array. "PERFECT MATCH" means every array is
bit-identical to the reference; any deviation is reported as an NMSE
value.

Golden vectors live in `tests/fixtures/` (e.g. `stagewise_burst30.npz`).
Regenerate them after any pipeline change you trust by re-running with
`--save-golden`.

## Inputs

- Host dumps: produced by running
  `DUMP_BURST_IDX=<n> ./build-host/test_pipeline_direct_if_albq` (writes
  `/tmp/host_signals/*.cf32` for the selected burst, plus a manifest
  row at `/tmp/host_direct_if/manifest.csv`).
- gr-iridium dumps: produced by
  `python3 tests/scripts/grIridium_on_slices.py ... --debug-id <gri_id>`
  (writes `/tmp/signals/signal-*-<id>.cfile`).
