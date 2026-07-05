# Iridium frequency scanner — Phase 1 design

**Date:** 2026-07-06
**Branch:** `freq-scanner` (off `t60-narrowband-priority` — depends on T60 `width_bins`)
**Status:** design approved, spec under review

## Problem

The RTL-SDR front end sees only ~2.5 MHz instantaneously (2.5 MSPS), but
Iridium user/SBD traffic — which carries aircraft ACARS — is spread across the
~10 MHz duplex sub-band (1616.0–1626.0 MHz, ~240 channels @ 41.667 kHz),
dynamically assigned per session and hopping. A single fixed LO can only watch
one 2.5 MHz slice. We want to find where narrowband (channel-shaped) traffic
concentrates and camp there.

This spec is **Phase 1**: the reusable core — runtime LO retune (no reboot) plus
a live narrowband-density map — driven and observed by hand over the serial
command interface. The autonomous scan→dwell→resume state machine is **Phase 2**
(separate spec) and builds on this.

Non-goal / honest caveat: this optimizes *where* we listen and *how long*, not
*how well we hear*. At the current bench antenna (0 real UW locks, broadband
interference dominant) it will find no real concentrations. Its value is the
mechanism — testable now against injected/known signals — ready for when the
antenna delivers decodable SNR.

## Approach

**Live retune, keep streaming (Approach A).** Retune the R820T2 tuner on the fly
(`rtlsdr_set_center_freq` → `r82xx_set_freq`, a fast control-transfer + register
write) while the USB bulk sample stream keeps running. This deliberately avoids
stopping/restarting the stream, which would re-exercise the USB
enumeration/stall path this codebase has fought hard to stabilize. The ~0.5 s
after a hop (PLL settle + tagger baseline re-prime) is treated as a dirty window
and its detections are discarded.

Fallback (Approach C) if a mid-stream control transfer glitches the bulk pipe:
add a one-buffer flush barrier in ingest so no half-old/half-new buffer is ever
analyzed. Decide from the hardware stability check (see Testing).

## Components

### 1. `scanner.c` / `scanner.h` (new, p4-usb-host/main)

Single-purpose module. Owns the hop primitive and the density map. Contains no
DSP / PIE code — it is control-plane only, so it stays out of the Core-1 worker
loop (and therefore clear of the deferred PIE-save smoke-gate hang).

- `esp_err_t scanner_hop(uint32_t hz, bool persist)`
  - Retune via a new `class_driver_retune(uint32_t hz)` accessor — `class_driver`
    owns the `rtldev` handle (used at `class_driver.c:276`), so the scanner does
    not reach into it directly. `class_driver_retune` returns an error if the SDR
    is not streaming yet; `scanner_hop` propagates it.
  - Update the running center used for burst-bin→Hz conversion so reported
    frequencies are correct at the new LO (see §4). NVS write only if `persist`.
  - `fft_burst_tagger_reset_baseline(tagger)` (new entry point, §3).
  - Record `settle_until = now + SETTLE_US` (~500 ms).
- `void scanner_scan(uint32_t start_hz, uint32_t stop_hz, uint32_t step_hz, uint32_t dwell_ms)`
  - For each center from start to stop by step: `scanner_hop`; sleep past
    `settle_until`; sleep `dwell_ms`; read-and-reset the density counters;
    record `{center_hz, narrowband_per_s, total_per_s, mean_snr}`.
  - Print a ranked table (by narrowband_per_s). Park on the hottest center and
    report it.
  - Runs in the caller's (serial_cmd) task context.
- `void scanner_print_last_map(void)` — reprint the last table.

Defaults: `SCAN_START=1616000000`, `SCAN_STOP=1626000000`, `SCAN_STEP=2500000`,
`SCAN_DWELL_MS=2000`, `SETTLE_US=500000`, `NARROWBAND_MAX_BINS=48` (matches
T60's `BURST_NARROW_MAX_BINS`).

### 2. Narrowband-density counters (in `dsp_processor.c`)

Tap `dispatch_gone_burst` — where the T60 `width_bins` is already in hand — to
increment atomic per-window accumulators:
- `_Atomic(uint32_t) acc_narrowband_bursts` (width ≤ `NARROWBAND_MAX_BINS`)
- `_Atomic(uint32_t) acc_all_bursts`
- `_Atomic(uint64_t) acc_snr_milli_sum` (Σ SNR×1000, for mean SNR)

Getter `dsp_processor_read_reset_density(dsp_processor_t*, out struct)` snapshots
and zeroes them. Same atomic/relaxed idiom as the existing diagnostic
accumulators (T48). Cheap; off the decode-critical path.

### 3. `fft_burst_tagger_reset_baseline(fft_burst_tagger_t*)` (new)

Thin entry point wrapping the existing init-time logic: `memset(baseline_history,
0)` + `history_primed = false`. After a hop the tagger re-learns the noise floor
over `HISTORY_SIZE` (512) steps (~0.42 s at 2.5 MSPS) and stays quiet until
primed — which is exactly the settle window we already discard.

### 4. Running-center update

`dsp_processor` converts burst bin → Hz using `FS_DETECT_HZ` and the LO center.
Today the LO is compile/NVS constant. Add a settable running center
(`dsp_processor_set_lo_hz`) that `scanner_hop` updates, so `rel_freq_hz` and any
reported absolute frequency track the new slice. Boot initializes it from config
as today.

### 5. Serial commands (`serial_cmd.c`)

Control plane is serial (httpd is unreliable per the SDIO-throttle note). Add to
the existing dispatch:
- `hop <hz> [save]` → `scanner_hop(hz, save)`; print confirmation + "settling".
- `scan [start stop step dwell_ms]` → `scanner_scan(...)` with defaults; prints
  the ranked map and the parked center.
- `map` → `scanner_print_last_map()`.

## Data flow

```
serial_cmd task ──hop/scan──> scanner ──class_driver_retune──> R820T2 (live)
                                  │
                                  ├─ fft_burst_tagger_reset_baseline (re-prime)
                                  ├─ dsp_processor_set_lo_hz (bin→Hz correct)
                                  └─ (dwell) dsp_processor_read_reset_density ──> ranked map
Core-1 ingest/worker: untouched; tagger keeps stepping, quiet until re-primed.
```

## Integration & safety

- Default behavior unchanged: no hopping unless commanded; boot tunes from NVS
  `lo_hz`. Scanning is fully opt-in.
- Approach A risk gate: watch the stream-stall watchdog and sample rate across
  hops; if a mid-stream control transfer glitches the bulk pipe, switch to the
  Approach C flush barrier.
- Dirty-window discard: detections before `settle_until` are ignored, so a hop
  never emits garbage bursts into the map or the worker.
- No worker/PIE code touched, so the deferred smoke-gate PIE-save hang
  (`project_pie_save_deadlock_smoke`) is not implicated by this change.

## Testing (mechanism only — real ACARS needs the antenna)

1. **Hop correctness:** `hop <hz>` retunes (confirm R82xx PLL log = hz + IF) and
   decoding resumes within ~0.5 s; reported burst frequencies reflect the new
   center.
2. **Positive control for the map:** inject the known smoke tone / a CW source at
   a known frequency; `scan` must rank that center highest in narrowband density.
   Without this, an all-flat or all-noise map means nothing.
3. **USB stability:** dozens of hops back-to-back — no stream stall, no
   re-enumeration, rate holds ~4.8 MB/s (via `/status` / serial STATUS lines).
4. **Baseline re-prime:** after a hop, no detections during settle, then normal
   detection resumes (not a burst of garbage).
5. **Regression:** fixed-tune decode + device-smoke GOLDEN unaffected (scanner is
   opt-in and control-plane).

## Out of scope (Phase 2)

Autonomous scan→dwell→resume state machine, hysteresis, periodic background
re-sweep, HTTP map display, SNR-weighted or decode-feedback density metrics.
