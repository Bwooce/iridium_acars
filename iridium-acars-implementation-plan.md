# Iridium ACARS Decoding Stack — Implementation Plan

Tracks the concrete implementation steps for the
[Iridium ACARS Decoding Stack Design](./iridium-acars-decoding-stack-design.md).

## Current status (2026-05-19)

The full receive chain works end-to-end on the ESP32-P4 against an
RTL-SDR v4. USB ingest → wideband detector → per-burst extract → DSP
chain → DQPSK demod → BCH → IDA/SBD → libacars all run cleanly at the
device's native 2.56 MSPS streaming rate with zero packet loss
(4.88 MB/s = 100.5% of real-time, Core 0 cycle headroom ~35%).

The wideband C front end (`fft_burst_tagger` + `direct_if_decim`) is
**numerically gr-iridium-equivalent on the host** (NMSE ≤ −17 dB,
phase coherence ≥ 0.993 across all stages on burst id=30), decoding
**59 frames on the ALBQ fixture** through the same `burst_pipeline`
the firmware runs.

The wideband path is now the active runtime front end on P4:
`ingest_core1` resamples 2.56 → 2.5 MSPS, `dsp_processor` runs
`fft_burst_tagger_step` against the 2.5 MSPS stream, and
`worker_core1` does the per-burst rotate + 10× decim + burst_pipeline
chain. The legacy polyphase channelizer files remain in-tree as
fallback / reference; orphan sections drop in the linker output.
Source-level cutover is done; on-target smoke + per-burst CPU
profiling (steps 2 of the forward plan) are next.

**Blocker for live RF**: 1620 MHz QFH antenna + LNA hardware not yet
acquired.

---

## Forward plan

One ordered sequence. Execute top-down; each step is gated on the
previous one passing.

### 1. Wire the wideband front end into the live P4 firmware

Source-level cutover landed (commit pending). The polyphase
channelizer + per-channel chain is no longer in the active data path:

- **1a — PSRAM placement (✅).** `fft_burst_tagger`'s 4 MB
  `baseline_history` is `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` in
  `dsp_processor_init`. Internal SRAM holds the 16 KB lookback +
  the ~120 KB hot working set inside the tagger itself.
- **1b — `dsp_processor` rewrite (✅).** Accumulates 2048-sample
  chunks across variable-length feed calls; runs `fft_burst_tagger_step`
  on each; converts `fbt_burst_t.center_bin → rel_freq_hz` and
  dispatches via the user callback.
- **1c — Queue payload (✅).** `detected_burst_t` extended to carry
  `rel_freq_hz`, `magnitude_db`, `noise_db`. Removed channelizer
  `channel` field. `peak_bin` retained for diagnostics.
- **1d — `worker_core1` rewrite (✅).** `signal_buffer_extract` of
  a wider window at 2.5 MSPS; absolute-phase Q15 rotation to DC
  (cosf/sinf per sample — see step 3 for the perf concern);
  `direct_if_decim_process` (10× to 250 ksps); `burst_pipeline_process_250khz`.
- **1e — Input-rate adapter (✅).** `resample_256_to_250` runs in
  `ingest_core1` after the int8→int16 conversion, before
  `signal_buffer_push`. signal_buffer and dsp_processor both see
  2.5 MSPS samples.
- **1f — Smoke test on target (next).** Flash the firmware,
  run the existing `CONFIG_SMOKE_TEST_RAW_IRIDIUM` against the ALBQ
  fixture, confirm tagger fires + decim runs + burst_pipeline sees
  expected content. Compare against host's 59/138 decode count as
  the upper bound.

### 2. On-target performance measurement

After cutover compiles and the smoke test produces expected frame
counts, measure actual CPU cost on the P4:

- Per-frame wall time for `fft_burst_tagger_step()` (expected ~18%
  of one core at N=2048 @ 2.5 MSPS per the 3.6.M feasibility analysis).
- Per-burst wall time for the `direct_if_decim_process()` +
  `burst_pipeline` chain.
- Ringbuffer drop rate / Core 0 headroom under the new front end.

Stop condition: real-time at 2.56 MSPS input with at least 30% of
one core idle. If we miss it, the next swap targets are listed in
step 3.

### 3. Tighten Q15 / SIMD where step 2 identifies hot spots

Likely candidates, by descending probability of being the bottleneck:

- **Absolute-phase rotation in direct_if_decim.** The host validation
  uses float64 cos/sin per sample to dodge the Q15 incremental-phasor
  magnitude decay (see `memory/feedback_q15_incremental_phasor_decays.md`).
  Per-sample cosf/sinf on P4 is feasible (scalar FPU) but slow.
  Replace with a table-driven cordic or periodically-renormalised Q15
  phasor.
- **fft_sc16_2048 internal kernel.** Currently a portable ANSI radix-2
  DIT (a direct N=2048 port of esp-dsp's
  `dsps_fft2r_sc16_ansi`). Swap for `dsps_fft2r_sc16_arp4` PIE-
  accelerated variant if step 2 shows the FFT loop dominates.
- **CFO FFT / RRC / UW correlator** (Phase 3.6 P2 candidates from the
  old plan). Floating-point hot paths in `burst_pipeline.c` that
  P4 will execute on scalar FPU. Convert to Q15 with PIE int16 SIMD
  only if profiling justifies it.

Each swap is validated by re-running `tests/scripts/dsp_compare.py`
against the gr-iridium reference and confirming NMSE / phase coherence
stay within Phase 3.6.M thresholds.

### 4. Reduce wideband tagger false positives

Current tagger emits ~138 burst tags on the ALBQ fixture vs gri's
~65. Closing the gap recovers the ~5 lost decodes vs path C (host
path C = 64/65; wideband path = 59/138). The cost is mostly wasted
worker cycles, not lost decodes, but it inflates queue pressure.

Approach: magnitude-sorted peak extraction per FFT step so
overlapping bursts collapse into one tag instead of N tags on
adjacent bins. Mirror gri's logic in `fft_burst_tagger_impl.cc`.

### 5. Live RF validation (Phase 4) — blocked on hardware

- Acquire 1620 MHz QFH antenna (Scan Iridium GO! recommended) +
  Nooelec SAWbird+ IR LNA.
- Connect; verify physical link integrity.
- Switch RTL-SDR from AGC to manual gain at ~35 dB.
- First live end-to-end Iridium ACARS decode.
- One-hour soak; compare frame rate vs upstream `iridium-toolkit`
  on the same recording.

### 6. Productisation (post-RF)

Sequenced after we have a real signal flowing through the stack:

- LCW sub-type body parsing (ISY, IIU, I36, IIP, IVO, IBC) for
  non-ACARS telemetry — only if it pays for itself in real-world
  data.
- ESP32-C6 Wi-Fi/Thread output path (MQTT or JSON-over-TCP); the
  C6 is already on the Nano carrier but unused.
- NVS-backed runtime config (LO, sample rate, station ID); avoids
  re-flash for field deployment.
- OTA firmware updates via `esp_https_ota` (needs partition layout
  change from factory-only to factory + ota_0).
- README with build/flash instructions and architecture overview.
- CI/CD for firmware builds.

---

## What's already done

Snapshot of completed milestones. Detail is in git history and commit
messages; this section only states what works today.

- **USB host + RTL-SDR v4 driver**: async multi-buffer transfer,
  AXI-GDMA push, 4.88 MB/s sustained (100.5% real-time, 0 drops at
  2.56 MSPS). R828D tuner properly detected and initialised
  (28.8 MHz XTAL, correct init array, multi-byte I2C). Software-only
  USB recovery via root-port-power cycling.
- **Dual-core pipeline**: USB ingest on Core 0, DSP+demod+decode on
  Core 1, 4 MB PSRAM lookback ringbuffer, queue between cores. Pre-
  allocated PSRAM stage buffers (no per-burst malloc).
- **DSP detector path (legacy/active)**: polyphase channelizer
  (M=64, L=1024-tap Hamming-windowed sinc) with PIE/SIMD-optimised
  Q14 path (D20 step 3c, all-phases asm via `esp.lp.setup`). Hits
  real-time at 2.56 MSPS with ~2% headroom on a 2048-sample frame.
  Per-channel detector: integer EMA baseline + 40× threshold (Q15
  weights), DSP/frame 407 µs in worst-case smoke; production ~300 µs.
- **DSP detector path (wideband, gr-iridium-equivalent, host-validated)**:
  `fft_burst_tagger` + `direct_if_decim` + `resample_256_to_250` +
  `fft_sc16_2048`. Compiles into P4 firmware (commit 1e386b7), not
  yet active.
- **Burst pipeline (`burst_pipeline.c`)**: D13 start-finder (Kaiser LP
  at 2.5 kHz, 28% threshold), CFO estimator (square-then-FFT,
  Blackman, 4096-pt), pre-rotate, RRC β=0.4 51-tap @ 10 sps, UW cross-
  correlator (1024-pt FFT, 271-sym sync reference), gri-aligned
  decim+demod. Multi-frame loop (Phase 3.6.H7) sweeps additional
  search starts after the first attempt.
- **DQPSK demod**: first-order PLL (α=0.2, β=0 — gri-equivalent), UW
  exact-match per rotation r∈{0..3} with complex-correlation fallback.
- **BCH(31,21)**: hard-decision block correction; soft-decision
  (Chase-2) implemented but not enabled by default.
- **Frame layer**: IDA decode (de-interleave + BCH), CRC-16-CCITT
  validation, SBD reassembler, libacars (vendored) for ACARS parsing
  with multi-segment reassembly via `la_acars_parse_and_reassemble`.
- **Host parity (Phase 3.6.H)**: path C = 64/65 decodes (98.5%) on
  ALBQ fixture; baseline for any future regression.
- **Host wideband-front-end equivalence (Phase 3.6.M)**: NMSE ≤−17 dB,
  phase coherence ≥ 0.993 across all stages vs gri (commit `de72f24`).
- **Comparison tooling**: `tests/scripts/dsp_compare.py` (single CLI,
  three modes, manifest-aware alignment, NPZ regression vectors).
- **Tests**: target smoke (synthetic + real-IQ) + 4 host tests
  including bit-level corpus comparison vs upstream gr-iridium. All
  green.

---

## Reference: pipeline vs gr-iridium

| Stage | gr-iridium | us | Status |
|---|---|---|---|
| SDR ingest | USRP/HackRF 6–12 MSPS float | RTL-SDR 2.56 MSPS uint8 → int16 | Hardware delta |
| Wideband detect | `fft_burst_tagger` (spectrogram peak + persistence + hysteresis) | Same (C port) — active on host, pending cutover on P4 | Numerically equivalent |
| Per-burst extract | Burst tags drive `burst_downmix` | Tag → rotate → 10× Kaiser decim → 250 ksps | Equivalent |
| Coarse freq | Channel selection | Tag freq → DC mix | Equivalent |
| Decimation | input_fir + decim to channel rate | gri-exact 279-tap Kaiser FIR, 10× | Equivalent |
| Start finder | `start_finder_fir`: Kaiser LP @ 2.5 kHz, ~182 taps, 28% threshold | Same scaled to rate (41 taps @ 50 ksps, same cutoff/fs ratio, 28%) | Equivalent |
| RRC | β=0.4, 51 taps @ 10 sps | Same | Equivalent |
| CFO | Square-then-FFT, 64 sym × 16× zero-pad, Blackman | Same: 280-sample × ~15× zero-pad → 4096-pt FFT, Blackman | Equivalent |
| Sub-sample timing | Sync-pos integer decim | Linear interpolation by `uw_res.correction` | Equivalent |
| DQPSK PLL | α=0.2, β=0 (first-order) | Same | Equivalent |
| UW check | Per-symbol soft sum, threshold ≤ 2 | Exact-match per rotation, complex-correlation fallback ≥ 0.6 | Different shape, equivalent strictness |
| BCH | Hard-decision | Hard-decision (Chase-2 available, off by default) | Equivalent |
| Frame parse | `iridium-parser.py` | `iridium_frame.c` + LCW classifier + IDA decode | Equivalent on IDA/SBD path |
| ACARS | `libacars` via iridium-toolkit | `libacars_idf` (vendored) with multi-segment reassembly | Equivalent |

---

## Hardware notes

### USB host VBUS on Waveshare ESP32-P4-Nano: not a GPIO

Verified against the Waveshare ESP32-P4-NANO schematic:

- The USB Type-A host port (J2) VBUS is fed through U2 (DIO7003HEST5
  load switch) from the always-on VCC_5V rail. U2's EN pin is held
  active by board-level pulls — there is **no GPIO that turns USB
  host VBUS on/off**.
- GPIO 45 is SD-card power enable on the Nano (drives Q1 AO3401
  P-MOSFET gate controlling SD1_VDD). Nothing to do with USB.
- GPIO 54 is just a breakout pin on header P1; no USB role.
- Same conclusion on the ESP32-P4-Pico: Picoblade P1 pin 1 (VBUS)
  is hardwired to VCC_5V, and GPIO 45 is also SD-card power.

**Implication:** do not drive any GPIO as a phantom VBUS_EN. An
earlier iteration drove GPIO 45/54 low under that mistaken
assumption — at best ineffective, at worst toggling SD-card power.

### Software-only USB recovery (despite no VBUS GPIO)

The ESP-IDF host stack provides a software path to recover stuck
devices even with VBUS hardwired-on:

- `usb_host_lib_set_root_port_power(false/true)` halts SOFs and
  re-evaluates attach state from the DWC OTG controller's perspective
  — equivalent to a USB cable unplug from the host's side. On boards
  with EN wired to a GPIO this would also drop physical VBUS; on the
  Nano it doesn't, but controller-side disconnect+reattach is enough
  to recover most stuck-device states without physical intervention.
- Firmware uses this in two places:
  1. **Boot**: install with `root_port_unpowered=true`, then explicit
     `set_root_port_power(true)` — forces fresh attach every reset.
  2. **Watchdog**: if no device enumerates within 6 s, cycle
     `power(false)` → 500 ms → `power(true)`, up to 3 attempts.

True hardware power-cycle of the attached USB device still requires
physical unplug. The software recovery covers the common cases that
previously needed physical intervention.

### Tuner gain mode

`librtlsdr.c` currently selects R820T2/R828D **automatic gain control**
(AGC) at boot. Acceptable for development without an antenna. For
live Iridium reception once Phase 4 hardware is installed, switch to
manual gain at ~35 dB and verify noise-floor stability via the
detector logs. AGC's reactive gain pumping shifts the constellation
and breaks the PLL phase tracker during bursts.

### esp-dsp 1.8.1 upstream bugs

Two issues in the bundled esp-dsp version that the worker works
around (worth filing upstream):

1. **`dsps_resampler_mr_init` rejects `samplerate_factor < 1`** but
   does not propagate the error — leaves the struct uninitialised,
   then `_exec()` faults at NULL+0x10. Workaround: bypass the `_mr_`
   wrapper and call the underlying `dsps_firmr_init_s16` /
   `dsps_firmr_s16` directly.
2. **`dsps_fird_s16_arp4.S` returns uninitialised register `a6`**
   instead of the output sample count. Observed as a constant -256
   across repeated calls. The function does write the output buffer
   correctly, it just doesn't report what it wrote. Workaround: pass
   the correct output length (`input/decim`, note that's the
   convention this API uses, vs `dsps_firmr_s16` which takes input
   length) and compute the count locally.

### Throughput optimisation: things tried that didn't help

Recorded so they're not retried:

- `aligned(64)` on the five hot DSP buffers regresses the EMA stage
  by ~170 µs/frame — five 8 KB buffers all at the same 64-byte
  alignment land in the same L1 D cache sets and conflict-thrash.
  Kept `aligned(16)`.
- `esp.vmul.f32` / `esp.vadd.f32` do not exist on ESP32-P4 — PIE on
  P4 covers s8/s16/s32 and complex variants but no generic f32
  vector arithmetic. Verified by enumerating `xesppie.S`.
- Shift-only detector threshold (`mag >> 5 > baseline`, 15 dB at 32×)
  produces too many priming-noise false positives. Settled on uint32
  multiply with 40× threshold.

---

## Deferred

Items that have been considered and explicitly deferred. Listed
only so they're not re-litigated.

- **gri-461 multi-frame edge case**: the one remaining burst out of
  65 not decoded by path C. Per-iter CFO inside the multi-frame loop
  (gri's `process_next_frame` model) recovers nothing on this corpus
  and regressed path A by 10/99 when last tried. Reconsider only if
  a new fixture exposes more sub-frames of this shape.
- **Option B fs=2.667 MHz channelizer rate change**: measured +0.5 dB
  mean recovery (much less than the +3 dB predicted, because Doppler
  ±40 kHz at 1.6 GHz keeps actual carriers off the nominal grid
  anyway). Substantial refactor for marginal benefit; deferred
  indefinitely. Measurement infrastructure (build_albq_raw_2667.py +
  fixture_albq_raw_2667.h) preserved for any future revisit.
- **Polyphase channelizer further optimisation**: the wideband path
  is the chosen front end. Channelizer remains as fallback only;
  no further tuning work on it.
- **Soft-decision BCH on by default**: Chase-2 decoder exists but
  costs 2–5× hard-decision and the wideband path delivers clean
  enough bursts that the soft margin doesn't pay. Revisit if real-RF
  data shows BCH at the SNR margin.
