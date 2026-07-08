# Design: Configurable near-DC tagger mask + fine near-DC diagnostic

**Date:** 2026-07-08
**Status:** Approved (brainstorm), pending implementation plan
**Backlog item:** HANDOFF-2026-07-08 #3 — "Exclude the near-DC bins from the tagger burst mask"
**Companion memory:** `project_stale_drop_and_priority_model`, `project_tagger_threshold_scale`,
`project_heap_position_decode_bug`, `feedback_115_cfo_peak_filter_reverted`

## Problem

The bench sees a burst *flood* (~285.9k detections/window at 12 dB, 243.8k at 13 dB per
`/diag/histograms`) against `bch_total:10` — reception is SNR-limited and the tagger is swamped.
A chunk of that flood is **self-inflicted**: a detection peak near the LO (reported ~1621.9 MHz,
i.e. within ~100 kHz of baseband DC at LO 1622) that the **HydraSDR does not see** (confirmed a
prior session: same sky, 12-bit, 1621.9 reads as an ordinary neighbor). Because the HydraSDR is
clean, the peak is **tuner-internal** to the 8-bit RTL (DC offset / I-Q imbalance / IF spur),
not real RF.

A tuner-internal artifact sits at a **fixed baseband offset from the LO**, so in FFT-bin space it
stays put regardless of where the scanner retunes the LO — a static bin-space mask is robust
across all LOs and will not move when the scanner hops. Second-order effects (dongle unit, gain,
temperature) *could* shift it, which motivates configurability and a permanent drift monitor.

The coarse `/diag` freq histogram (40 bins × 62.5 kHz over ±1.25 MHz) cannot resolve where the
artifact actually sits at FFT-bin resolution (2048-pt FFT ⇒ 1220 Hz/bin, DC at bin 1024). Before
masking anything we must pin the exact bins so we do not suppress real signal.

## Goals

1. **Pin the artifact** at FFT-bin resolution and keep watching it over time (thermal/temporal drift).
2. **Suppress the self-inflicted flood at source** by excluding the artifact bins from *new-burst*
   declaration, with **zero risk to real IDA** (reference peaks are 1620.5–1621 MHz = −1.5…−1.0 MHz
   from LO, far from the −100 kHz…DC region).
3. **Live-tunable, no reflash** — the exclusion window is an NVS knob so we tune empirically on the
   bench, mirroring the existing `tag_thr` workflow.
4. **Do not regress decode.** Judge by BCH-ok/hour, not burst counts (see
   `feedback_115_cfo_peak_filter_reverted`: a host-green filter killed device decode).

## Non-goals

- Not touching `tag_thr` (a separate, complementary lever — HANDOFF #4).
- Not an RF/hardware notch (the artifact is internal, not on the antenna — HANDOFF #2 is separate).
- Not an adaptive/auto-locating mask in this change. The fine diagnostic + live NVS tuning does the
  locating by hand first; an auto-tune is a possible follow-up if drift data warrants it.
- No change to the shipping default behavior: the mask defaults to **disabled** (empty window).

## Architecture

Three isolated units, each independently testable:

### Unit A — fine near-DC diagnostic (`worker_core1.c`)

- New `static _Atomic uint32_t s_hist_dcfine[DCFINE_BINS]` where `DCFINE_BINS = 384`, spanning
  DC ±192 FFT bins (bin 1024 ± 192 ⇒ ±234.4 kHz at 1220.7 Hz/bin). This comfortably covers both a
  true-DC spike (bin 1024) and the observed −100 kHz cluster (bin ~942), with margin to watch drift.
- **Always compiled** (no build gate). Cost: ~1.5 KB internal `.bss`; one atomic increment per
  detection in the push path — same cost class as the existing `hist_freq_record`.
- Recorded at push time, next to `hist_freq_record`, keyed off the detection's `center_bin`
  (only detections whose `center_bin` falls in the DC ±192 window increment a bucket; others are
  ignored by this histogram).
- **Heap-position safety:** the PIE-critical buffers (`s_w_table`, `s_fft_scratch`,
  `baseline_history`) are heap-allocated and pinned early at boot by `fft_sc16_2048_init()`, guarded
  at runtime by `esp_ptr_in_dram()`. Heap layout is set by boot alloc *order*, not `.bss` size, so a
  standalone `.bss` array cannot push them. Device-smoke is the backstop.
- Exposed on `/diag/histograms` as a new field with kHz metadata mirroring the existing pattern:
  `dcfine[...]`, `dcfine_bin0_Hz` (= (1024 − 192 − 1024) × 1220.7 = −234375, i.e. offset of bin 0
  of this sub-histogram relative to DC), `dcfine_bin_Hz_width` (= 1220.7, reported as integer Hz).
  Human summaries/reports convert to kHz and call out the peak bin's kHz offset from DC.

### Unit B — configurable DC-exclusion window (`fft_burst_tagger.c` / `.h`)

- Tagger struct gains `int dc_mask_lo, dc_mask_hi` (signed bin offsets from DC = fft_size/2).
  Sentinel for **disabled**: `dc_mask_lo > dc_mask_hi` (e.g. init to `+1 / −1`), meaning no bins
  excluded — the default.
- New public setter `void fft_burst_tagger_set_dc_mask(fft_burst_tagger_t *t, int lo, int hi)`
  (offsets relative to DC; clamps to valid bin range; safe to call live to re-tune).
- A helper `apply_dc_static_mask(t)` sets `burst_mask[k] = 0` for
  `k in [DC + dc_mask_lo, DC + dc_mask_hi]` (clamped). It is invoked at the **end of**
  `rebuild_burst_mask()` and at every full mask-reset site (init and the reset paths that set the
  whole mask to 1), so the excluded bins can **never** spawn a new burst regardless of active-burst
  churn. This reuses the existing `burst_mask[]` machinery — the new-burst scan already does
  `if (!burst_mask[bin]) continue;` — so there is **zero new hot-loop cost**.
- Scope: the window blocks **new-burst declaration** only. Keep-alive of an existing burst that
  drifts into the window is a non-issue because nothing real lives there; simplest correct behavior
  is that these bins are simply never allowed to start a burst.

### Unit C — NVS plumbing (`app_config.c`, `dsp_processor.c`, `serial_cmd.c/.h`)

Mirror the `tag_thr` path exactly:
- `app_config.c`: load two int32 keys `dcmask_lo` / `dcmask_hi` via `nvs_get_i32_or` (add the helper
  if only the f32 variant exists) with defaults that encode **disabled** (`+1 / −1`); add
  `SET_FIELD_NUM(...)` setters and matching commit.
- `dsp_processor.c`: after `fft_burst_tagger_init(...)`, call
  `fft_burst_tagger_set_dc_mask(tagger, cfg.dcmask_lo, cfg.dcmask_hi)`.
- `serial_cmd.c/.h`: `set dcmask_lo <n>` / `set dcmask_hi <n>` / `get` support so the window is
  tunable over UART0 without a reflash. (A live re-tune re-calls the setter; the next
  `rebuild_burst_mask()` picks it up.)

## Data flow

```
RTL IQ (2.5 MSPS, P4 USB host)
  → dsp_processor: fft_burst_tagger_step()
      new-burst scan skips bins where burst_mask[bin]==0
      (active-burst masks ∪ DC-exclusion window)          ← Unit B
  → push to priority queue
      hist_freq_record(rel_freq)        (existing, coarse)
      hist_dcfine_record(center_bin)    (new, fine)        ← Unit A
  → /diag/histograms exposes coarse + dcfine (kHz metadata) ← Unit A
NVS (dcmask_lo/hi) → app_config → dsp_processor → set_dc_mask() ← Unit C
```

## Error handling / edge cases

- **Disabled default:** `dc_mask_lo=+1, dc_mask_hi=−1` ⇒ `apply_dc_static_mask` masks nothing;
  behavior is bit-identical to today. This is what ships until we bench-tune.
- **Out-of-range window:** setter clamps `DC+lo` and `DC+hi` to `[margin, N-margin)`; an inverted or
  empty window masks nothing.
- **Window overlapping real signal:** prevented by process — the fine diagnostic must confirm the
  chosen bins carry only the artifact (HydraSDR already confirms no real RF there); host tests assert
  a burst just outside the window is still emitted.
- **Diagnostic overflow:** `_Atomic uint32_t` cumulative since boot, same saturation semantics as the
  existing histograms (no decay; documented).

## Testing / validation gates

1. **Host unit tests** (`tests/host/test_fft_burst_tagger*.c` or a new case):
   - Burst injected at a bin inside the window ⇒ **not** emitted.
   - Burst injected just outside the window ⇒ emitted (no over-reach).
   - Window disabled (default) ⇒ output bit-identical to pre-change (regression guard).
   - `set_dc_mask` re-tune takes effect on the next step.
2. **Device-smoke RAW GOLDEN** (`OUTDIR=... scripts/smoke_run.sh raw`, `matched≥40`, ~61 healthy)
   — proves the always-on diag array did not disturb the PIE heap layout and decode is intact
   (default-disabled ⇒ fixture tone untouched). Then `... restore` to production. **Mandatory**
   (`feedback_device_smoke_mandatory_after_dsp`).
3. **Bench empirical loop** (live, no reflash after step 2):
   - Read `/diag/histograms.dcfine`, identify the artifact bins in kHz.
   - `set dcmask_lo/hi` to cover them; confirm the SNR-pushed flood and `worker_dropped` drop.
   - Confirm `bch_ok/h` (FRMDEC) does **not** regress over a dwell — decodes, not burst counts,
     are the acceptance criterion.
   - Record the tuned window + observed drift; a sane non-disabled default gets baked in a
     **follow-up** commit once validated.

## Commit shape (anticipated)

- Commit 1: Unit A (fine diagnostic + `/diag` field) — self-contained, smoke-verified.
- Commit 2: Unit B + C (configurable mask + NVS/serial plumbing, default disabled) + host tests —
  smoke-verified.
- Follow-up (separate, after bench tuning): set the validated default window.

Each firmware commit carries a `Smoke-verified:` trailer per the pre-push hook.
