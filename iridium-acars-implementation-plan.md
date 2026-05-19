# Iridium ACARS Decoding Stack - Implementation Plan

This document tracks the concrete implementation steps for the [Iridium ACARS Decoding Stack Design](./iridium-acars-decoding-stack-design.md).

## Current Status
- **Target:** Phase 3.6 (host parity with gr-iridium) → then Phase 4
  (Live RF validation).
- **Status:** Phases 0, 2, 3.1–3.5 COMPLETE. End-to-end DSP pipeline
  runs against a real RTL-SDR v4 at the device's full streaming rate
  with zero packet loss. Bursts flow through detect → extract →
  freq-centre → decimate → resample → demod → BCH cleanly. USB host
  recovers from stuck-device states without physical unplug.
- **Decode-rate gap:** on the shared 1.25 s ALBQ fixture gr-iridium
  decodes 65/65; the channelizer path (A) decodes 6/99 (channelizer
  losses dominate); the direct-IF host harness (path C — bypasses
  channelizer + resampler, feeds gr-iridium-equivalent baseband into
  the same downstream `burst_pipeline`) decodes **63/65 (97%)**. Path
  C is now at gr-iridium parity per Phase 3.6.H goal. The remaining 2
  are sub-frames of multi-frame bursts whose primary frames DO decode;
  closing them needs `handle_multiple_frames_per_burst` support in
  burst_pipeline (deferred as a feature, not a parity fix).
  Channelizer path A remains broken (8 dB SNR loss from per-channel
  filter rolloff) — Phase 3.6.P will substitute components one at a
  time, measuring decode rate and execution time at each swap to
  decide front-end strategy.
- **Throughput:** 4.88 MB/s = 100.5% of the 4.85 MB/s real-time
  target; rb_full_drops = 0; Core 0 cycle headroom ~35%.
- **Functional regression tests:** all green (target smoke + 4 host
  tests including bit-level corpus comparison vs upstream gr-iridium).
- **Blocker (Phase 4):** antenna + LNA hardware (Scan QFH +
  SAWbird+ IR).
- **Immediate Goal:** acquire the RF frontend; switch tuner to
  manual gain at ~35 dB; first successful real Iridium decode.

---

## Phase 0: Reference Corpus and Ground Truth (DONE)
- [x] Port FFT/Detection pipeline to ESP32-P4.
- [x] Optimize for PIE (Processor Instruction Extensions).
- [x] **Result:** 2048-pt `sc16` pipeline processes a frame in **0.93ms** (Limit: 1.024ms).

---

## Phase 2: ESP32-P4 USB Host Benchmarking (DONE)
- [x] Identify RTL-SDR v4 (VID 0xbda, PID 0x2838).
- [x] Reached bit-perfect **5.12 MB/s** (2.56 MSPS) using async driver.
- [x] **Validation:** Confirmed stable operation with Octal PSRAM (15.6 MB free).
- [x] **Tuner detection:** R828D probe @ I2C 0x74 returns 0x69 (R82XX_CHECK_VAL).
      Earlier iterations only probed R820T @ 0x34, which silently fell back to
      direct-sampling mode and produced false bursts at DC. The v4 specifically
      uses R828D — both probe paths are now enabled.

---

## Phase 3: Single-Board Prototype (Lite Node) (DONE)
Merging USB ingestion with the DSP detection pipeline in a dual-core architecture.

### Step 3.1: Async USB Driver Optimization (DONE)
- [x] Implement multi-buffer asynchronous USB transfers.
- [x] Implement a ring buffer between USB callback and DSP task.
- [x] Refactor `class_driver.c` to a state machine.

### Step 3.2: Dual-Core Pipeline Integration (DONE)
- [x] **Core 0 (Master):** USB high-speed ingestion + 2048-pt `sc16` FFT detector.
- [x] **Lookback Buffer:** 4MB Circular IQ buffer in PSRAM (~400ms capacity).
- [x] **Core 1 (Worker):** Handoff via thread-safe "Burst Queue".

### Step 3.3: Port Demodulator & Decoder (DONE)
- [x] Implement **Frequency Centering** (Complex Rotation) on Core 1.
- [x] Implement **FIR Decimation** (32x) and **Resampling** (5/8 = 1.6×, 80 kHz → 50 kHz) to 2 sps.
- [x] Port the **QPSK/DQPSK Demodulator** with PLL phase tracking.
- [x] Port the **BCH(31,21) Decoder** with Hard-Decision block correction.
- [x] **Live integration validated:** 177 bursts processed in 14 s of indoor
      USB streaming with no crashes. Stage 1 outputs `input/32` samples, Stage 2
      outputs `Stage1 × 5/8` — matches the design ratios exactly.
- *Note:* `BCH DECODE SUCCESS` was first observed against synthetic input;
  against live indoor RFI the bursts aren't real Iridium so DEMOD/BCH
  unsurprisingly fail. Re-validation against real Iridium signals is Phase 4.

### Step 3.4: USB Host Recovery (DONE)
Several issues uncovered during live integration that didn't surface in the
benchmark phase:

- [x] **Stuck-device recovery without physical unplug.** After an unclean
  ESP32 reset (panic, partial flash, etc.), the RTL-SDR could end up in a
  state where it stopped enumerating despite VBUS being applied. Solved with:
  install with `root_port_unpowered=true`, then explicit
  `usb_host_lib_set_root_port_power(true)` after install (forces fresh attach
  on every boot); plus a 6-second watchdog in `class_driver_task` that
  cycles root port power up to 3 times if no device enumerates.
- [x] **Defensive guards in worker.** Edge-bin guard (drops bursts at
  `peak_bin < 4`, `> 2043`, or DC bin 1024); normalized-frequency clamp
  before `dsps_cplx_gen` (was previously panicking with `Load access fault`
  on bin-0 false detections); zero-output guards on both DSP stages so
  empty buffers can't reach the demod.
- [x] **Pre-allocated stage buffers.** Replaced per-burst `malloc`/`free`
  in the worker with PSRAM buffers allocated once in `worker_core1_init`,
  with `DSP_PADDING_ELEMS` padding around the per-channel halves to keep
  `arp4` vector loads from running off the end.

---

## Phase 3.5: Integrated Throughput Optimization (IN PROGRESS)

When the tuner+DSP pipeline was integrated end-to-end against a live RTL-SDR
v4, sustained throughput dropped from the dry benchmark's 5.12 MB/s to
1.22 MB/s = 25% of real-time. Investigation found a chain of bugs and
contention sources. Real-time at 2.56 MSPS / int8 IQ = **4.85 MB/s** at the
current 16 KB transfer size (~3300 μs/cycle). Progress:

| Phase | Throughput | % of target | Notes |
|---|---|---|---|
| Pre-fix (broken PLL → PSRAM thrash from RFI) | 1.22 MB/s | 25% | |
| **PLL fix** (XTAL 28.8 MHz, I2C array memcpy, init array vs upstream) | 2.38 MB/s | 49% | RTL-SDR v4 finally locks |
| **Step 1** (sdkconfig: L2=256 KB, WDT cfg, malloc reserve, FreeRTOS trace) | 2.49 MB/s | 51% | |
| **Step 2** (`signal_buffer_push` → AXI-GDMA via `esp_async_memcpy`) | 2.82 MB/s | 58% | Architectural win — moves CPU off the AXI master bus |
| **Step 4b** (USB Host daemon + ISR pinned to Core 1) | 2.84 MB/s | 59% | DWC OTG ISR off Core 0's hot loop |
| **Step 4** (vectorise convert loop, 4× unrolled 32-bit loads) | 2.91 MB/s | 60% | |
| **Step 5** (convert + push moved to Core 1 via internal-SRAM ping-pong) | 3.20 MB/s | 66% | Core 0 cycle drops 4859 → 4374 μs; `consumer_waits=0` |
| **Step 6** (`CONFIG_COMPILER_OPTIMIZATION_PERF=y`, `-Og` → `-O2`) | 4.61 MB/s | 95% | Single biggest win in the arc. The whole prior path had been benchmarked under `-Og` (debug). Per-frame DSP 1050 → 645 μs; `rb_full_drops` 32% → 2.3% |
| **Step 7** (per-file `-O3 -funroll-loops` on dsp_processor.c, ingest_core1.c) | 4.61 MB/s | 95% | DSP/frame 645 → 570 μs; cycle 2581 → 2280 μs. Throughput device-capped (drops still 2.3%). |
| **Step 3a** (hand-rolled PIE Q15 windowing kernel) | 4.61 MB/s | 95% | Wind 92 → 16 μs (5.75×); DSP/frame 570 → 493 μs. Throughput still device-capped through drops. |
| **Step 6.5** (status logger offloaded to Core 1 task) | **4.88 MB/s** | **100.5%** | Drops collapsed 7/303 → 0/313. Per-second status formatting on Core 0 had been stalling the consumer ~5–10 ms/sec, filling the USB ringbuffer past 480 KB. Moving the printf work to a Core 1 task closes the last gap. |
| **Step 7a** (linear-write magnitude, fftshift at report only) | **4.88 MB/s** | **100.5%** | Throughput device-capped already; this is pure CPU headroom. DSP/frame 447 → 422 μs (−5.6%). The mag loop's prior `magnitudes[(i+N/2) % N] = …` pattern was two write-streams that triggered write-allocate cache evictions in L1 D. Linear write + apply fftshift only at the burst-callback boundary. Per-stage detail below. |
| **Step 7b** (eradicate floats — `magnitudes[]`/`baseline[]`/threshold all uint32) | **4.88 MB/s** | **100.5%** | DSP/frame 422 → 407 μs (−3.6%). Magnitudes hold `re² + im²` directly (no normalisation); detection threshold is integer 40× (within 0.02 dB of prior float setpoint); EMA is `b = (β·b + α·m) >> 15` with Q15 weights. Mag −24%, detect −30%, base +9% (uint64 mul). Net win plus integer substrate for any future PIE work. |
| **Step 7c** (PIE int magnitude — research only, no perf win) | **4.88 MB/s** | **100.5%** | Four PIE recipes tried (qacc 4-quadrant, fused mul+load, deinterleave-first, vmul.s32.s16xs16). Best kernel mag was 32 μs (−18% vs scalar 39 μs), but every variant traded the savings back in EMA-stage L1 D-cache pressure from the int32 scratch buffer (16–32 KB) evicting the EMA's working set. Scalar mag stays in production. The kernel is preserved in `dsp_mag_arp4.S` with documented findings: qacc int64-spaced layout, `vmul.s32.s16xs16` is Q15-shifted (lossy), Xhwlp / lp.setup with PIE store at body-end works fine on P4 (Espressif's ACE customisation). |

### Step 7a per-stage delta (synthetic-tone smoke)

| Stage | Step 6.5 | **Step 7a** | Δ |
|---|---|---|---|
| wind   | 16  | 15  | flat |
| fft    | 199 | 200 | flat (already PIE) |
| mag    | 74  | **51** | **−31%** |
| detect | 46  | 47  | flat |
| base   | 111 | 109 | flat |
| **DSP total/frame** | **447 μs** | **422 μs** | **−5.6%** |

### Step 7b — float eradication (uint32 magnitudes/baseline/threshold)

DSP path now runs on integer arithmetic end-to-end. `magnitudes[]` and
`baseline[]` changed from `float32` to `uint32_t`; magnitude stores
`re² + im²` directly with no normalisation; threshold is integer 40×
(matching the prior float setpoint within 0.02 dB); EMA runs as
`b = (β·b + α·m) >> 15` with Q15 weights.

| Stage | Step 6.5 (f32) | Step 7a (linear-mag, f32) | **Step 7b (int)** | Δ vs 7a |
|---|---|---|---|---|
| wind   | 16  | 15  | 15  | flat |
| fft    | 199 | 200 | 200 | flat (PIE Q15) |
| mag    | 74  | 51  | **39** | **−24%** |
| detect | 46  | 47  | **33** | **−30%** |
| base   | 111 | 109 | 119 | +9% (uint64 mul vs f32) |
| **DSP total/frame** | **447 μs** | **422 μs** | **407 μs** | **−3.6% / −9% cumulative** |

`mag` and `detect` got faster because integer arithmetic on RV32 has
no FPU pipeline stalls; `base` got marginally slower because the
fused EMA does two uint64 multiplies per element (no PIE int32 SIMD
on P4 yet — same constraint that blocked Step 3c f32-PIE). Net win
small but real, plus we now have an integer substrate that **enables
future PIE work on mag and EMA**.

Tried the cache-audit recommendation of `aligned(64)` but it
regressed the EMA stage by ~170 μs/frame from L1 D cache-set
conflicts (five 8 KB buffers all at the same 64-byte alignment land
in the same set offsets). Tried `magnitudes[i] >> 5 > baseline[i]`
shift-only threshold; at 32× (15 dB) it produced too many priming-
noise false positives. Settled on uint32 multiply with wrap-around
(40 × baseline overflows for baseline > 2^26 ≈ 67 M; realistic noise
levels stay well below 2^25 so wrap is not a concern).

Confirmed `esp.vmul.f32`/`esp.vadd.f32` do **not** exist on ESP32-P4
by enumerating IDF's `xesppie.S` (the PIE assembler's accepted-mnemonic
list). PIE on P4 covers s8/s16/s32 + complex; no generic f32 vector
arithmetic. So the f32-fast-path the cache audit hoped for is a
phantom. Step 7b's integer conversion was the only viable path to
meaningful PIE on the mag/EMA loops.

In production (real RTL-SDR thermal noise, not synthetic) DSP/frame
drops to **~300 μs** because the baseline EMA runs nearly every
frame and stays cache-warm; the smoke test's 407 μs is the worst-
case (false-positive noise spikes inflate the average).

### AGC vs manual gain — current setting and recommendation

`librtlsdr.c` currently calls `rtlsdr_set_tuner_gain_mode(rtldev, 0)`
which selects the R820T2/R828D **automatic gain control** (AGC).

**For Iridium burst reception, manual gain is generally preferable**:
- AGC adjusts gain dynamically. Between bursts (95% of time = noise
  only), AGC pushes gain up; when a burst arrives, the AGC has to
  react fast or the burst clips. Slow AGC = clipping mid-burst; fast
  AGC = gain pumping that shifts the constellation and breaks the
  PLL phase tracker.
- Predictable signal levels into the FFT detector mean the baseline
  EMA tracks a stable noise floor instead of an AGC-modulated one.
- 30–40 dB manual gain is typical for Iridium with QFH + LNA chain.

**For now: stay AGC.** Without the actual antenna+LNA installed we
can't measure the right manual setpoint. Once Phase 4 hardware is
installed, switch to manual at ~35 dB and verify noise-floor
stability via the `Worker-stages` and burst-detection logs.

### Is 4.88 MB/s "real 100.5%" or is the pipeline capable of more?

The 4.85 MB/s real-time target was computed for **2.56 MSPS × 2 bytes
× 1 (single-channel)** at the 16 KB transfer size. The device sends
4.88 MB/s = 313 transfers/sec × 16 KB = 100.6% of that target.

But this **does not** mean the pipeline is at its ceiling:

- The RTL-SDR's theoretical max at 2.56 MSPS is 5.12 MB/s with zero
  USB overhead. Actual 4.88 MB/s is the device's real output rate
  including USB framing. We can't see beyond 4.88 because that's
  what comes off the wire.
- Our Core 0 cycle currently runs ~1972 μs vs the 3300 μs/cycle
  budget at 4.85 MB/s — about **35% headroom**.
- Whether the pipeline could handle 3.2 MSPS (or higher) is **untested**.
  Bumping the sample rate would reveal the next bottleneck (USB
  ringbuffer depth? cache pressure on signal_buffer?).

**Honest reading:** "100.5% of nominal real-time" is real, but it's
limited by what the device sends, not by what the pipeline can
process. We have measurable CPU headroom for future feature work
(more DSP, multiple channels, higher sample rate) but the actual
ceiling remains undetermined until we test those modes.

The cache audit also recommended bumping `aligned(16) → aligned(64)` on
the five hot DSP buffers. Tried it; **regressed** the EMA stage by
~170 μs/frame — five 8 KB buffers all aligned to the same 64-byte
boundary land at the same L1 D cache-set offsets and conflict-thrash a
64-set 8-way L1 D. The linker's natural placement (16-byte-aligned at
varying mod-64 offsets) scatters them across sets and is empirically
faster. Kept aligned(16). Recorded as a comment in `dsp_processor.c`
so nobody re-tries this.

The cache audit also flagged that **`esp.vmul.f32` / `esp.vadd.f32` do
not exist on ESP32-P4** (verified by enumerating `xesppie.S` — PIE on
P4 covers s8/s16/s32 and complex variants but not generic f32 vector
arithmetic; the only f32-PIE kernels in esp-dsp are the FFT and
biquad, which are full-algorithm units, not vector primitives). This
means a "drop-in" PIE EMA that keeps `baseline[]` as f32 isn't
possible. Step 3c (PIE EMA) requires the same Q31 conversion path as
Step 3b — staying on the headroom-only list.

### Critical bugs found during 3.5 (each had silent failure modes)

- **`rtlsdr_read_array` / `rtlsdr_write_array` were single-byte-truncated.**
  The original port did `*array = data[0]`, copying only the first byte of
  every multi-byte I2C transaction and silently corrupting all R828D init
  writes. Fixed with `memcpy(array, data, len)`. This was the underlying
  cause of every prior "PLL won't lock" symptom.
- **`R828D_XTAL_FREQ` was 16 MHz** (DVB-T2 default) but RTL-SDR v4 shares
  the 28.8 MHz clock from the RTL2832U. Wrong divider arithmetic → no lock.
- **`r82xx_init_array` had 14 of 27 registers wrong vs upstream librtlsdr.**
- **PLL settle delays (`usleep_range`) were commented out.** PLL needs ~10 ms
  to lock before reading the lock bit.
- **Daemon task on Core 0 was preempting DSP feed** with ~300 ISR/s. Pinning
  to Core 1 measurably reduced jitter.

### Open work

- [ ] **Step 3 — PIE/Q15 baseline EMA** (next, biggest remaining lever).
  Audit (5-min grep against esp-dsp 1.8.1 sources, confirmed 2026-05-06):
  on RISC-V P4 the only `_arp4` (PIE-optimised) primitives are
  `fft2r_{sc16,fc32}`, `fft4r_fc32`, `fird_{s16,f32}`, `dotprod_{s16,f32}`,
  `biquad_{f32,sf32}`. **No PIE variants exist for windowing, magnitude,
  multi-rate FIR, complex generator, mulc, add, or bit-reverse.** Step 3
  therefore can't be done by function-call swap — it requires hand-rolled
  PIE intrinsics on a Q15-converted pipeline. Projected ~−800 μs/cycle →
  ~4.6 MB/s = 95% of target.
- [ ] **Step 2.5 — pre-Step-3 instrumentation.** DMA queue depth, L2 cache
  hit/miss counters, Core 1 busy %, synthetic burst injection, so we can
  attribute Step 3's effect to inner-loop time vs bus contention.
- [ ] **Functional regression tests** (new, blocking Step 3). Q15
  conversion will quietly change numerical output unless gated on a
  fixture-based comparison. Two layers:
  - Target-side smoke test: synthetic IQ → `signal_buffer` → DSP detector
    asserts the burst fires on the right FFT bin.
  - Host unit tests: `bch_decoder` + `qpsk_demod` (both essentially
    IDF-free) built with native gcc, run against the `test_corpus/`
    fixtures.
- [ ] **Step 9 — zero-copy USB pointer passing** (deferred). ~3-5% gain,
  ~2-3 days work; defer until we're closer to budget.

### Architecture changes already in place

- **Ping-pong ingest on Core 1** (`ingest_core1.c`): two cache-aligned
  internal-SRAM slots, per-slot `s_free`/`s_ready` semaphores. Class
  driver does USB read → dispatch → DSP feed; the ingest task on Core 1
  does convert (uint8 → int16 Q15) → `signal_buffer_push`. Slot ownership
  protocol: only `class_driver` takes/gives `s_free` — the ingest task
  must never `take(s_free)` (deadlock; was a bug during initial Step 5).
- **AXI-GDMA push** (`signal_buffer.c`): `esp_async_memcpy_install_gdma_axi`
  + per-buffer `esp_cache_msync(M2C|INVALIDATE)` for coherency. Removes
  CPU stalls on the AXI master.

### Deferred refactor (post-3.5)

When throughput is at-budget, move shared C files (`qpsk_demod`,
`bch_decoder`, `dsp_processor`, etc.) into a `common/` sibling directory
so future child-processor P4 boards can re-use them. Until then,
`p4-usb-host/main/` holds everything.

---

## DSP gaps vs gr-iridium (open)

The current pipeline is a minimal port; gr-iridium has stages we don't.
These gaps surface immediately on real-shape data — the smoke tests
`CONFIG_SMOKE_TEST_RAW_IRIDIUM` and `CONFIG_SMOKE_TEST_REAL_IRIDIUM`
both fail at the `qpsk_demod` step on the Albuquerque corpus because
of them. **No amount of "better data later" closes the gap** —
gr-iridium implemented these stages because real Iridium RF requires
them. Tracked in TODOs D7-D11.

### Side-by-side: us vs gr-iridium (May 2026)

Stage-by-stage comparison. "Equivalent" means same algorithm + same
parameters (within rate scaling). "Deviation" means different
algorithm or parameters that affect outcome.

| Stage | gr-iridium | us | Status |
|---|---|---|---|
| SDR ingest | USRP/HackRF 6-12 MSPS, float complex, 96 dB DR | RTL-SDR 2.56 MSPS, uint8 → int16, 48 dB DR | Hardware delta (not algorithmic) |
| Wideband channelizer | FFT overlap-save (decim ≥8), no window, rect bins. PFB ≤300 taps Kaiser 60 dB for decim <8 | Polyphase M=64, Hamming-windowed sinc, L=1024 taps (N=16), ~52 dB stopband, 64.6 dB adjacent rejection | Different architecture; we have TIGHTER per-channel filtering than their high-decim option |
| Burst detect | `fft_burst_tagger` (spectrogram peak + persistence + hysteresis on wideband) | Per-channel power threshold + EMA baseline + dedup | Functionally equivalent; theirs operates pre-channelization |
| Per-burst extract | Burst tags drive `burst_downmix` with whole-burst window | `signal_buffer_extract` from PSRAM ringbuffer | Equivalent |
| Coarse freq center | Channel selection only (no extra QPSK estimation) | Snap to nearest Iridium 41.667 kHz grid + channelizer bin → DC mix | Equivalent (#41 step 1) |
| Decimation | input_fir + decimate to channel rate | FIR decim 32× (2.56M → 80k) + polyphase resample 5/8 (→ 50k = 2 sps) | Equivalent |
| Start finder (D13) | `start_finder_fir`: Kaiser LP @ 2.5 kHz cutoff, ~182 taps, 28% threshold | Kaiser-windowed sinc LP, 41 taps at 50 ksps (same cutoff-to-fs ratio), 28% threshold | Equivalent (same Kaiser-design ratio scaled to rate) |
| RRC on burst | β=0.4, 51 taps at 10 sps (5.1 sym periods) | β=0.4, 51 taps at 10 sps (5.1 sym periods) | **Equivalent** (matches gr-iridium tap count exactly) |
| Sync reference | RC (RRC ⊛ RRC) of preamble + UW = 28 syms | Same — RC of preamble + UW = 28 syms | Equivalent |
| Sync search | FFT cross-correlation, 1024-pt at 10 sps | FFT cross-correlation, 1024-pt at 10 sps | Equivalent |
| CFO estimation | Square-then-FFT on 64 sym × 16× zero-pad, Blackman window | Same: 280-sample preamble+UW × ~15× zero-pad → 4096-pt FFT, Blackman | Equivalent |
| CFO correction | Multiply burst by exp(-j·CFO·n) | Same (worker pre-rotation) | Equivalent |
| Sub-sample timing | Implicit via sync_pos + integer decim from sps=10 sample grid | Linear interpolation by `uw_res.correction` (#46) at sps=10 | Equivalent (both at 10 sps now) |
| Symbol rate (internal) | 10 sps internal (250 kHz) | 10 sps internal (250 kHz) — worker resamples 80k → 250k then decim 5× to 50k before qpsk_demod | **Equivalent** |
| DQPSK PLL | First-order (alpha=0.2 = 1/5), phase only | First-order (alpha=0.2, beta=0) — beta=0 explicitly to match | **Equivalent** |
| UW check | Sum of \|demod-uw\| per symbol (cap diff=3 → 1); threshold ≤ 2; **no rotation trial** | Exact-match per symbol per rotation r ∈ {0..3}; min ≤ 2; complex-correlation fallback ≥ 0.6 | **Deviation in shape, not strictness**: theirs tolerates per-symbol noise (partial credit for 90° off); ours tolerates uniform rotation |
| BCH | Hard-decision (main path) | Hard-decision | Equivalent (theirs has soft-decision experimental, not in main pipeline) |
| Frame parser | `iridium-parser.py` — full classifier + LCW decode | `iridium_frame.c` + LCW classifier + IDA decode | Equivalent for IDA/SBD path |
| ACARS | `libacars` via iridium-toolkit | `libacars_idf` (vendored) | Equivalent |

**Outcome on shared ALBQ_RAW corpus**:
- gr-iridium reports SNR 19-25 dB on the 3 known bursts; decodes them
- Us: detects same bursts at SNR 16-21 dB (channelizer -1.4 to -2.8 dB delta), all bursts get to qpsk_demod, decode fails with `dl_diffs=6-8` (just above the threshold of 2)

All four algorithmic deviations from the original comparison have
been resolved (commits 80b89fe → 37b990b):
1. Start finder filter length — Kaiser-windowed sinc, 41 taps at same
   cutoff-to-fs ratio as gr-iridium's 182-tap filter at 250 ksps.
2. Symbol rate — worker now resamples to 250 kHz (10 sps) internal,
   matching gr-iridium's burst_downmix rate; final decim back to 50 kHz
   before qpsk_demod (which still expects 2 sps).
3. PLL beta — explicitly 0 (first-order, phase only) like gr-iridium's
   qpskFirstOrderPLL.
4. Same as 2.

After these changes the smoke result is still 0/5. The pipeline is
now algorithmically equivalent to gr-iridium on the stages that
matter; any remaining gap is signal-quality limited (we run from
RTL-SDR at 2.56 MSPS uint8 vs USRP/HackRF at 6-12 MSPS float),
imprecise channelizer for the Iridium grid, or downstream gates
(UW threshold, BCH error tolerance) that gr-iridium doesn't hit
because they get cleaner per-channel SNR from their wider-band SDR.

| # | Gap | Why it matters | Estimated effort | TODO |
|---|---|---|---|---|
| D7 | Polyphase channelizer | Single FFT detector picks SNR-ratio peak which lands BETWEEN concurrent bursts in the same 2.56 MHz subband. Per-channel streams give each Iridium TDMA slot its own chain. | 3-5 days, ~10-15% of one core | #26 |
| D8 | Fine carrier-freq estimation in worker | Bin-resolution centring leaves residual >> qpsk_demod PLL capture (1.25 kHz). 4th-power method on QPSK gives sub-Hz residual. | 1-2 days, ~1% per burst | #27 |
| D9 | Two-stage / wider-capture PLL | Defensive: when D8's residual estimate is itself noisy at low SNR, two-stage PLL (acquisition α=0.5 → tracking α=0.2) recovers gracefully. | 1 day | #28 |
| D10 | Symbol timing recovery (Gardner / O&M) | qpsk_demod currently sample-decimates by 2; sample-rate offset between SDR XO and Iridium symbol clock drifts the sampling instant. Real RF will surface this once D7-D9 are done. | 2-3 days, ~2-5% per burst | #29 |
| D11 | Soft-decision BCH (Chase-2) | gr-iridium uses soft inputs to BCH for ~1-2 dB SNR margin. Lower priority — payback only at low SNR. | 2-3 days, 2-5× current BCH cost | #30 |
| D12 | IDA CRC-16-CCITT validation | ida_decode extracts da_crc_reported but doesn't validate against payload. Real RF will produce corrupted bursts that should be filtered before SBD reassembly. | 0.5 day | #31 |
| D13 | Burst-edge detection (sub-frame) | dsp_processor reports start/length at FFT-frame granularity (0.8 ms). qpsk_demod's UW slide-search runs out of search space when the burst's actual start is several ms past the reported one. | 1-2 days | #32 |
| D14 | ACARS multi-segment reassembly | frame_decoder calls la_acars_parse (single-segment); messages split at the ACARS application layer don't reassemble. Switch to la_acars_parse_and_reassemble with a long-lived rtables ctx. | 0.5 day | #33 |
| D15 | LCW sub-type body parsing | We classify LW frames by ft (DA/VO/IP/SY/U3/U6) but only DA gets a full body decode. SY/IIU/I36/IBC carry useful telemetry (sat config, time, handoff metrics). Off the ACARS path. | 3-5 days | #34 |
| D16 | AGC / dynamic gain | RTL-SDR fixed at tuner-AGC-mode setting. Live antenna will see ±10 dB signal level swings; static gain produces saturation or undersampling at edges. | 1-2 days | #35 |
| D17 | ESP32-C6 Wi-Fi/Thread output | Currently decoded ACARS only goes to ESP_LOGI on serial. Production rooftop node needs Wi-Fi/Thread → MQTT or JSON-over-TCP. Design doc specifies the C6 (already on board) for this. | 5-7 days | #36 |
| D18 | NVS-backed runtime config | LO/sample rate/station ID hardcoded. Field deployment needs runtime-configurable values without re-flashing. | 2 days | #37 |
| D19 | OTA firmware updates | Single factory partition; field updates need physical USB. Switch to two_ota or factory+ota_0 layout for esp_https_ota. | 2 days | #38 |
| D20 | Channelizer-detector PIE/SIMD optimisation | Host-side correctness landed in pure float; on the P4 the malloc-free path will be borderline at 2.56 MSPS without the kind of PIE-assembly treatment dsp_window_arp4.S / dsp_mag_arp4.S got for the legacy detector. Detail in §"D20 — Channelizer-detector optimisation roadmap" below. | 5-8 days, expected ~10-15% one core after | (new) |

Combined CPU budget for the DSP gaps (D7-D11): ~20-25% of one P4 core.
Easily within the existing 60% Core 0 headroom.

**Per-stage SNR-gap measurement (May 2026 baseline)** — task #47

Comparing our channelizer's per-burst SNR_dB against gr-iridium's
reported SNR on the same raw uint8 SDR data (ALBQ_RAW_UINT8 fixture,
LO=1618.5 MHz):

| Burst | gr-iridium | ours | delta |
|---|---|---|---|
| ch 58 (-232891 Hz) | 25.10 dB | 21.30 dB | **-3.80 dB** |
| ch  0 (+17103 Hz)  | 20.46 dB | 18.24 dB | **-2.22 dB** |
| ch 56 (-316217 Hz) | 19.16 dB | **MISSED** (16 dB threshold) | — |

Mean delta over the 2 detected bursts: **-3.01 dB** at the channelizer
stage alone. The missed third burst lands close to bin edge between
ch 56 and ch 57 — strongly suggests 40 kHz / 41.667 kHz grid mismatch
is real and biting hardest at edge-aligned channels.

This 3 dB upstream loss propagates through the rest of the chain,
which is why the smoke test sees matched-filter SNRs of 7-9 dB on
bursts gr-iridium decoded at 19-25 dB. Run
`tests/host/test_snr_gap_measurement` to reproduce; it always passes
(diagnostic instrument) and the numbers are the gate.

**dB-recovery priority ranking** (where to look first):

| Source | Estimated recovery | Task |
|---|---|---|
| Channelizer Iridium-grid alignment | 3-4 dB | #41 (D7+) |
| Channel filter quality / freq-centering | 1-2 dB | #45 |
| Soft-decision BCH (only matters after demod works) | 2 dB | #30 (D11) |
| Symbol timing refinement / Gardner loop | 0.5-1 dB | #46 |

**Task #41 progress notes (May 2026)**:

*Step 1 (DONE)*: Worker-side Iridium-grid carrier snap. Computes the
nearest 41.666... kHz multiple from the channelizer's bin-centre
coarse offset and uses that as the mix-down frequency instead. Effect
on smoke corpus's strong burst: CFO estimate dropped from -0.715
rad/sym → -0.213 rad/sym; PLL residual now ~0 after pre-rotation.
Doesn't recover the upstream channelizer energy loss.

*Step 1 alt (TRIED + REVERTED)*: Widened the channelizer's prototype
filter cutoff (1.10× and 1.25× of 1/(2M)) to capture more of an
Iridium channel when it falls between our bins. Did recover ~0.9 dB
of mean SNR loss and rescued a previously-missed edge-aligned burst
— but broke the channelizer reference tests because their fixture
was computed with the original 1/(2M) cutoff. Widening moves AWAY
from gr-iridium parity by that test. Reverted; the path to recovery
is fs/M change, not filter widening.

*Step 2 (MEASURED + DEFERRED, task #48)*: Iridium-grid channelizer.
Three design options analysed; option B (fs=2.667 MHz) measured
empirically before commitment:

**Measurement result** (test_snr_gap_measurement, ALBQ_RAW vs
ALBQ_RAW_2667 fixtures — same source content, different sample rates):

| burst | baseline 2.56 MHz | option B 2.667 MHz |
|---|---|---|
| ch 58 (-232891 Hz) | -3.80 dB | -4.16 dB (slightly worse) |
| ch 56 (-316217 Hz) | MISSED  | -0.82 dB (recovered) |
| ch  0 (+17103 Hz)  | -2.22 dB | MISSED (regressed) |
| **mean over matched** | **-3.01 dB** | **-2.49 dB (+0.52 dB)** |

The +0.5 dB net gain is much smaller than the ~3 dB I predicted
because my analysis assumed bursts arrive on the nominal Iridium
41.667 kHz grid. In practice **Doppler shifts of ±40 kHz at 1.6 GHz
move actual carriers off the nominal grid** — so channelizer
grid-alignment doesn't uniformly help, it just reshuffles which
bursts are well-aligned vs poorly-aligned with our bins.

Option B's substantial refactor (SDR rate, FIR coefficient regen,
resampler retune, fixture regen for every host test) isn't justified
by 0.5 dB. **Deferred indefinitely**. The measurement infrastructure
(build_albq_raw_2667.py script + fixture_albq_raw_2667.h + the
side-by-side measurement test) is committed so any future revisit
can re-measure under different conditions (e.g. after AGC / D16 work
or with a sharper channelizer prototype filter design).

Original option design choices for reference:

| Option | fs | M | Channel | FFT | Effort | Notes |
|---|---|---|---|---|---|---|
| A | 2.0 MHz  | 48 | 41667 Hz (exact) | mixed-radix | ~1-2 weeks | needs new FFT; -0.56 MHz coverage |
| B | 2.667 MHz | 64 | 41671 Hz (~exact) | radix-2 (existing) | ~3-5 days | RTL-SDR quantises rate |
| C | 2.56 MHz | 64 × 2 | 20 kHz effective (half-bin) | radix-2 (existing) | ~2-3 days | 2× channelizer compute |

All three options give ~3 dB recovery on the measured channelizer
loss. Option B is the cleanest gr-iridium parity with minimal
architecture disruption; option C is the fallback if 2.667 MHz isn't
clean on our RTL-SDR v4.

**Filter widening attempted as an interim (REVERTED)**: a cheaper
"middle ground" — widen the channelizer's prototype filter cutoff
from 1/(2M) to ~1.1× — would recover some loss without changing
fs/M. Tried both 1.10× and 1.25× cutoffs. Both improved the SNR-gap
measurement but broke the channelizer reference tests (the per-cell
snapshot fixture was computed with the original 1/(2M) cutoff;
widening the filter changes all channel outputs and breaks bit-exact
comparison). One specific high-conf burst (ch 53 at 29.5 dB on
LO=1625.5 fixture) was lost with 1.25× because wider filter
admits more adjacent-channel energy, raising the relative noise
floor. The per-cell test could be regenerated if we commit to a new
filter, but that's a divergence from gr-iridium's reference filter
design — the proper Iridium-grid fix is option A/B/C above, not
ad-hoc cutoff tuning. Reverted to 1/(2M).

**Status snapshot (D7-D10), May 2026:**
- **D7** — polyphase channelizer landed (host correctness 9/9, target wiring in flight).
- **D8** — fine freq estimation in worker is in place; residual still
  reaches the demod at moderate magnitudes (omega_hat tracking from
  D9 catches the bulk of it).
- **D9** — second-order PLL with phase + frequency tracking shipped
  in `qpsk_demod` (`PLL_ALPHA=0.2`, `PLL_BETA=0.1`). Visibly tracks
  per-symbol omega in the smoke test (±0.2–0.6 rad/sym).
- **D10** — Gardner symbol-timing recovery is implemented
  (`sym_timing_correct_2sps`) but **not yet wired** into the worker:
  default textbook gains regressed the only decoding burst when added
  on top of correlator + pre-rotation. The module + host trace tool
  (`tests/host/test_sym_timing_trace.c`) remain for offline tuning.
  In its place we adopted gr-iridium's burst-mode approach: a one-shot
  UW cross-correlator (`uw_correlator_find`) gives both timing and
  direction in one pass, and the complex peak phase is used to
  pre-rotate the burst so the PLL starts already locked. First
  end-to-end demod success on raw RTL-SDR corpus came from this
  pipeline. Smoke test: 1 of 3 expected bursts decoded; remaining
  two have larger residual freq offsets (omega -0.44, +0.59) that
  the current PLL gains/initial phase can't acquire within the 12-
  symbol UW window.

### D20 — Channelizer-detector optimisation roadmap

The D7 channelizer + per-channel detector work is correctness-first: it
runs in pure scalar float complex on the host and target. That gets us
to 9/9 host tests passing across simulated PRBS and real-RF Albuquerque
fixtures at multiple LOs. It will **not** be fast enough on the P4 at
2.56 MSPS without further work. This TODO captures the optimisation
plan so we don't lose it; do **not** start before:
  1. Worker-chain integration is done (channelizer_detector wired into
     dsp_processor in place of the single-FFT path).
  2. We've measured actual hardware cost on the P4 with the live SDR
     feed — we want to know which stage is the bottleneck, not guess.

**Cost breakdown (estimates, scalar float on P4 @ 360 MHz):**

| Stage | Per-cycle cost | Cycles/s | Mflops |
|---|---|---|---|
| int16 → float complex | 64 muls + 64 adds | 40 000 | ~5 |
| Polyphase filter | 8 taps × 64 phases × cmplx mul-acc = 1024 flops | 40 000 | ~40 |
| 64-pt radix-2 FFT (hand-rolled) | ~600 flops | 40 000 | ~25 |
| Per-cycle power + percentile + threshold | ~250 ops | 40 000 | ~10 |
| **Total** | | | **~80** |

Scalar float on the P4 lands ~50-100 Mflops, so we'd be running at
~80% of one core just for the detector. Tight. PIE / esp-dsp can
bring this down 5-10×.

**Optimisation steps, in priority order:**

1. **Eliminate per-feed malloc (DONE).** `channelizer_detector_create()`
   pre-allocates `in_buf` (8192 cf32) + `out_buf` (128 × 64 cf32)
   once. Verified host-side; carry forward into target.

2. **Replace 64-pt FFT with esp-dsp PIE.** The hand-rolled radix-2
   `fft_64()` in `polyphase_channelizer.c` is portable scalar float.
   esp-dsp's `dsps_fft2r_fc32_aes3` (Anyfft P4) hits ~3-4× scalar.
   For an N=64 FFT this is borderline — fixed twiddles + small N
   means cache effects dominate. Measure first. Likely: use
   esp-dsp's sc16 FFT after converting to int16 cmplx (frees up
   PIE for the polyphase filter too).

3. **Polyphase filter as PIE kernel.** The inner loop is
   8-tap × M=64 channels × cmplx-FIR per cycle. The pattern is
   identical to the windowing kernel in `dsp_window_arp4.S` —
   8-lane Q15 multiply-accumulate. Expected 5-10× speedup over
   scalar float. This is the single biggest win.

4. **int16 fixed-point throughout the channelizer.** Float complex
   is convenient for prototyping but the P4 has no FPU SIMD; PIE
   only operates on int8/int16 vectors. Quantise the prototype
   filter taps to Q15, run the polyphase + FFT in int16 cmplx,
   convert to power as uint32. Same precision in practice (Iridium
   bursts are 12-15 dB SNR, well above quantisation noise floor).

5. **Cross-channel percentile in PIE.** Per-cycle 16th-of-64
   selection isn't a textbook PIE kernel but two passes of
   8-lane parallel-min (find min, mask it, find next min, etc.)
   to the 16th rank — ~16 PIE iterations. Or: skip percentile
   entirely and use a fixed noise-floor (calibrated at boot from
   a quiet 50 ms sample). Lower priority once 1-4 are done.

6. **Per-burst log10f → fixed-point.** SNR_dB is computed at
   burst-end with `log10f(power / floor)`. Once per burst, not
   hot-path. Leave as float.

**What we explicitly are NOT doing:**

- Threading the channelizer onto Core 1 — Core 1 is already used
  by the DQPSK demod / frame_decoder pipeline. Adding the
  channelizer there ruins the producer/consumer split.
- Reducing M from 64. 64 channels at 40 kHz × M = 2.56 MHz
  matches our channel-spacing target (Iridium uses 41.667 kHz
  spacing, so 40 kHz channels overlap somewhat — close enough).
  Going to M=32 (80 kHz channels) makes adjacent-channel rejection
  insufficient and breaks the "burst centre is ≤20 kHz from
  channel centre" guarantee that drives D8's PLL-friendliness.

**Stop condition:** detector keeps up at 2.56 MSPS with ≥40% Core 0
idle headroom for the rest of the pipeline.

**Progress, final D20 results on hardware:**

| Step | Status | DSP cost / frame (2048 samples) | Throughput |
|---|---|---|---|
| Baseline (first hardware run, float) | Reference | n/a (quiet log) | 320 ksps (12.5% RT) |
| D20 step 1: cached twiddles, mask-not-mod, N=8 unroll | ✅ done | 1681 µs | 1150 ksps (45% RT) |
| D20 step 2: `dsps_fft2r_fc32_arp4` for the 64-pt FFT | ✅ done | 1562 µs (–7%) | 1230 ksps (48% RT) |
| D20 step 3 (a): per-phase PIE asm kernel (xacc) | ✅ done | 1315 µs (–16%) | 1.45 MSPS (57% RT) |
| D20 step 3 (b): 2-copy delay line + unaligned PIE | ✅ done | 845 µs (–36%) | ~2.18 MSPS (85% RT) |
| **D20 step 3 (c): all-phases asm via esp.lp.setup** | ✅ **done** | **785 µs (–7%)** | **>2.56 MSPS (102% RT)** |

Net: ~10× speedup from the float baseline. Channelizer hits real-time
with ~2% headroom on a 2048-sample frame (785 µs DSP vs 800 µs budget).
Residual ~9% USB drops are scheduling jitter (not DSP starvation) on
the live SDR feed.

**Caveats / known correctness issues (as of D20 step 3 (c) landing):**

- **The smoke test (`SMOKE_TEST_RAW_IRIDIUM`) shows 0 frames reaching
  the classifier vs 3 expected — BUT this is a D7-level limitation,
  not a D20 regression.** Both the float path AND the int16+asm
  path produce the same "worker chain broken" diagnosis on the same
  fixture. The channelizer detects bursts correctly (18-23 of them,
  SNR 14-22 dB), the worker processes them, but demod produces no
  valid frames. Root cause: the channelizer's 40 kHz frequency
  quantisation leaves the PLL with up to ±20 kHz residual offset,
  16× outside the PLL's ~1.25 kHz capture range. **D8 (fine carrier
  frequency estimation) is the fix; D7 + D8 are a logical pair.**
- The int16/Q14 path's *output values* differ from the float path's
  in scale, saturation, and quantisation noise. A host-side
  comparison test (`tests/host/test_polyphase_int16.c`) flags this
  with bit-equality tolerances that Q14 cannot meet — the test
  reveals scale/saturation differences but does NOT indicate the
  asm path is broken. The on-target detector logic is ratio-based
  and EMA-learning so the differences are absorbed; both paths
  produce comparable burst counts and SNR in smoke-test mode.
- The Q14 quantisation has 1 bit of headroom; full-scale random IQ
  saturates the per-phase MAC sum after >>14. Production input from
  the SDR is far below full scale, so this only shows up in the
  synthetic host test. Consider Q13 (2 bits of headroom) if real
  inputs ever push near saturation.
- The unaligned-PIE cfg bit is set once at `polyphase_channelizer_create()`
  and assumed to persist; haven't tested behaviour across deep-sleep
  or other CSR-resetting events.
- **CHANNELIZER_USE_INT16_PATH=1 is the production default** — the
  asm path is functionally equivalent to the float path (both fail
  the smoke test identically) and ~2× faster. Flip to 0 to revert
  to the float path if a future bug is suspected.

**Learnings from D20** (worth carrying into D9 / further PIE work):

- The `arp4` suffix on esp-dsp's float FFT is **loop-overhead
  removal via esp.lp.setup**, NOT PIE vectorisation — P4 PIE is
  integer-only. Scalar float MAC (RV-32IMF fmadd.s) is competitive
  with scalar int16 MAC on P4; the PIE win only materialises with
  `esp.vmulas.s16.xacc` or `.qacc`.
- `qacc` (4 × int64 lanes) requires either `esp.srcmb.s16.qacc` for
  shift-saturate-to-vector, or `esp.st.qacc.l.{l,h}.128.ip` for
  memory drain — but the latter is ONLY valid inside an
  `esp.lp.setup` body (the assembler rejects it otherwise).
  For single-accumulator MACs `xacc` + `esp.srs.s.xacc` is much
  simpler (esp-dsp's `dsps_dotprod_s16_arp4` is the canonical
  example).
- `esp.srs.s.xacc rd, rsh` only accepts certain RV register
  encodings for `rd` / `rsh` — t1/t2 was rejected, t5/t6 works.
  Likely a 4-bit register field limited to x24..x31. The same
  restriction applies to `.xp` base registers (matrix-mult uses
  x24/x25 = s8/s9 specifically).
- `esp.vld.128.ip` post-increment immediates are limited to
  ±16 bytes. For larger strides use `.xp` with a register stride.
- Unaligned 128-bit PIE loads need the cfg-bit-1 enable (one
  `esp.movx.r.cfg` / `or` / `esp.movx.w.cfg` sequence; persists
  in the CSR thereafter).
- Per-iteration call overhead on RV-32 is ~20 cycles. For 64
  identical operations per cycle, folding the loop into the asm
  via `esp.lp.setup` is a real win even at the same per-iteration
  PIE-instruction cost.
- Cache scratch matters: at 4 KB delay-line + ~2 KB tap state +
  output buffers, we're comfortably in L1 D-cache. Larger
  working sets thrash the cache and can erase PIE wins (see
  `dsp_mag_arp4.S` for a case where the kernel itself was faster
  but the increased scratch caused a net regression).

**D20 step 3 — sub-task breakdown** (entry point:
`common/iridium_decoder/polyphase_mac_arp4.S`, scaffold + algorithm
spec + register map already in-tree; symbol stub-only):

1. **Int16 caller in C.** Add `polyphase_channelizer_process_int16`
   to `polyphase_channelizer.c`. Quantise the float prototype to Q15
   at create-time. Validate Q15 output against the float path on
   host (allow tolerance; bit-equal is not expected). Choose delay-
   line discipline (rotate writes vs head-rotated reads — currently
   recommending rotate writes so the asm reads linearly).
2. **Scalar C reference.** `polyphase_mac_arp4_ref` — pure C
   implementation of the same kernel, 4-phase parallel in the same
   layout the asm will use. Compare bit-for-bit against the
   straight 64-phase unrolled scalar. Becomes the host fallback +
   the asm validation baseline.
3. **PIE asm, incremental.**
   - 3a. 1-phase-per-qacc-round PIE (slower, simpler, bit-equal to
     reference; this is the "make it work" milestone).
   - 3b. 4-phase parallel via `vunzip.16` (the design described in
     the .S file header).
   - 3c. Wrap the 16 qacc rounds in `esp.lp.setup` for zero-overhead
     loop. End state should be `≤64 PIE ops` per channelizer cycle.
4. **Wire into `polyphase_channelizer_process_int16` on target only**
   (ifdef ESP_PLATFORM). Host build keeps scalar reference.
5. **Re-measure on hardware.** Compare to the 1562 µs/frame baseline
   above. Stop if real-time + 40% headroom achieved; otherwise
   continue to step 4 (int16 throughout) and step 5 (PIE percentile).

Combined effort estimate: D7-D14 ≈ 3 weeks of focused work to close the
"correctness on real RF" gap. D15-D19 ≈ 2 weeks of "deployment / nice-
to-haves". The first batch is what's required to actually decode ACARS
from raw IQ on real silicon.

Combined CPU budget: ~20% of one P4 core for D7+D8+D9+D10. Easily within
the existing 60% Core 0 headroom.

The host-side `test_demod_albq` regression passes 0/382 bit diffs vs
gr-iridium because `tests/scripts/build_albq_fixture.py` shifts each
burst PRECISELY to DC at fixture-build time using float math — i.e.,
it does the work D7+D8 would do for us at runtime. Real RF doesn't
get that pre-processing.

**Why we don't skip stages gr-iridium has:** gr-iridium implemented
each of these stages because real Iridium RF requires them. Multi-
burst-in-subband, off-bin carriers, sample-rate drift, channel
overlap, etc. are properties of the protocol/spec, not artefacts of
the test data. No antenna improvement closes the gap.

---

## Phase 3.6: Host-first parity, then measured P4 substitution (IN PROGRESS)

The D7–D14 work above tried to evolve a *single* pipeline toward gr-iridium
parity. After many rounds we are still at 3/64 decodes on the shared ALBQ
fixture while gr-iridium decodes 64/64. Mixing front-end choices
(polyphase channelizer for P4 fit) with downstream-correctness work makes
it hard to tell which deviation is costing decodes.

Phase 3.6 fixes that with a two-step strategy:

1. **Build a host pipeline that fully matches gr-iridium's algorithm**
   (decode rate ≥ 90% of gr-iridium on the corpus) before touching any
   P4-realistic component. The host is unconstrained — we use scipy,
   float64, wideband FFTs, anything that maps 1:1 onto gr-iridium.
2. **Substitute one component at a time with a P4-appropriate version**,
   measuring BOTH decode rate AND execution time on the host. A swap is
   acceptable only if decode rate degradation is within budget. Once a
   swap is accepted on the host, port to the P4 and confirm both numbers
   reproduce there.

This decouples correctness from performance: any decode regression after
a substitution is by definition caused by that substitution.

### Step 3.6.H — Host parity with gr-iridium

Goal: ≥ 58/64 decodes on the ALBQ fixture via the path-C harness
(`tests/host/test_pipeline_direct_if_albq.c`). Status: **59/65 decoded
(91% of gri's 65/65)** — goal met.

| # | Change | Target | Status |
|---|---|---|---|
| H1 | Path-C scaffold: rotate raw 2.5 MSPS → DC at gr-iridium-tagged offset, decimate 10× with scipy, feed `burst_pipeline_process_250khz` | baseline | ✅ 20/64 |
| H2 | gri-aligned uw_correlator constants (CORR_FFT_N 1024→2048; START_PRE_SAMPLES 2→25; START_LP_NTAPS 101→183; D13 search valid range only; SYNC_RRC_LEN 280→271; CFO_INPUT_N 280→256) | +N | ✅ 21/65 |
| H3 | Fix D13 cut-position formula — half_fir_size double-counted (our smooth[n] is input-centred, gri's filtered[k] is filtered-output-indexed) | +N | ✅ 23/65 |
| H4 | Float-precision matched filter alt path (CORR_USE_FLOAT_FFT) — rules out Q15 BFP as the primary divergence | research | ✅ |
| H5 | gri-aligned decimation FIR in direct_if_dump.py — scipy's default short Kaiser leaked adjacent-channel bursts into the squared-FFT CFO step, clamping ω to ±2π on 38 bursts | +30+ | ✅ 59/65 |
| H6 | Manifest-aware D13 bypass — sub-frames of multi-frame bursts have their RAW timestamp 8+ms after the burst envelope onset; D13 fires on the still-active prior frame at iq250[91] instead of locating the second frame's preamble. Force burst_start from the manifest's UW position for path C (deterministic at iq250[~250] given our Python pre-pad of 4096 raw samples) | +4 | ✅ **63/65 (97%)** |
| H7 | Last 2 misses (gri 391, 461) are sub-frames of multi-frame physical bursts. Path C rotates the whole window by the sub-frame's published freq, but gri's iter2 CFO correction was per-frame inside the same PDU. Eliminating these would require porting gr-iridium's `handle_multiple_frames_per_burst` loop into burst_pipeline_process_250khz — a feature, not a parity fix | (multi-frame feature) | deferred |

### Step 3.6.P — P4-realistic substitution with measured impact

Each step replaces ONE host component with a P4-appropriate version
and runs the same path-C harness for decode rate plus a new
per-burst wall-time measurement. Order is from least-risky to most-risky:

| # | Substitution | What we expect to lose / measure | P4 constraint |
|---|---|---|---|
| P1 | scipy `resample_poly` 10× decim → C int16 polyphase FIR decim (host) | float→Q15 rounding; ~0–2 decodes acceptable | First step needed for any embedded path |
| P2 | Floating-point downstream DSP hot paths (CFO FFT, RRC, UW correlator) → Q15 / fixed-point | ~0–2 decodes; full precision impact across pipeline | P4 has scalar FPU but PIE vector is integer-only — Q15 needed for SIMD |
| P3 | gr-iridium-equivalent **C** wideband `fft_burst_tagger` → produces same `(start_sample, freq_offset)` tags we currently parse from `iridium-extractor` stdout | 0 decodes (just replaces the Python detection step) | Still wideband; not yet P4-fit, but standalone |
| P4 | Wideband C tagger → **polyphase channelizer** (current path-A front end) | Largest expected drop (currently 17 decodes between path-C and path-A); the trade-off the whole effort is built to measure | Resolves the 8 dB SNR-loss measured at post-D13 in May; informs whether to widen channel BW, redesign filters, or keep direct-IF |
| P5 | Direct-IF rotate+FIR-decim on raw 2.5 MSPS → integer rotate+`firmr_s16` (esp-dsp) | ~0 decodes; just the existing P4 resampler in the new role | If P4 chosen over A, this is the P4 front end |
| P6 | Per-burst wall-time profiling on actual P4 hardware: confirm host substitution numbers reproduce; identify whichever step exceeds the per-burst CPU budget | actual perf numbers | Real platform — PSRAM/SRAM placement, cache behaviour, queue contention |

Each P-step writes one row to a results table in this plan:
`(step, decode rate vs prior, decode rate vs gr-iridium, host µs/burst,
P4 µs/burst, notes)`. The table is the artefact — anyone reading
the plan can see exactly how each substitution moved both numbers.

### Outcome shape

By end of 3.6 we have:
- A host pipeline at gr-iridium parity (≥58/64) that serves as a
  golden reference for any future change.
- A measured cost-vs-loss curve for every P4-specific substitution,
  so the front-end choice (channelizer vs direct-IF + tagger) is made
  on data not speculation.
- A regression test suite where the path-C run prints a single line:
  `D/N decoded` — degradation from substitution is one number, not
  a long log to read.

### Step 3.6.T — Comparison tooling (DONE)

Single unified DSP comparison tool: `tests/scripts/dsp_compare.py`.
Replaces four scattered scripts with one CLI exposing three modes
(`--mode stagewise`, `--mode squared-fft`, `--mode raw`) plus
`--save-golden` / `--check-golden` for regression vectors stored under
`tests/fixtures/*.npz`. All FFTs are numpy/scipy (no hand-rolled DFTs)
and burst alignment uses `/tmp/host_direct_if/manifest.csv`.

Per-stage thresholds (NMSE-dB, phase coherence, peak-frequency Hz) are
hard-coded in the script's `THRESHOLDS_*` dicts from the conventional
gr-iridium-port engineering targets — channelizer ≤ −50 dB NMSE, RRC
≤ −40 dB, etc. Not empirically tuned to pass the current run.

Reference: [docs/host-vs-griridium-testing.md](docs/host-vs-griridium-testing.md).

---

## Phase 4: Live RF Validation (NEXT — blocked on hardware)
*Requires: 1620 MHz QFH antenna (Scan Iridium GO! recommended) + Nooelec SAWbird+ IR.*

- [ ] Acquire RF frontend hardware.
- [ ] Connect RTL-SDR v4 + LNA + antenna; verify physical link integrity.
- [ ] Run the live decoding stack and verify real-world burst reception
      (expect detected bursts to land on Iridium channel grid spacings, not
      random RFI peaks).
- [ ] First successful end-to-end decode of an Iridium ACARS frame.
- [ ] Long-soak test: 1+ hour of capture, frame rate vs upstream
      `iridium-toolkit` on the same recording.

## Phase 5: Maintenance & Documentation (TODO)
- [ ] Add a comprehensive **README.md** with build instructions and architecture overview.
- [ ] Implement automated CI/CD for firmware builds.

---

## Hardware Notes

### USB Host VBUS on Waveshare ESP32-P4-Nano: not a GPIO

Verified against the Waveshare ESP32-P4-NANO schematic
(`files.waveshare.com/wiki/ESP32-P4-NANO/ESP32-P4-NANO-schematic.pdf`):

- The USB Type-A host port (J2) VBUS is fed through **U2 (DIO7003HEST5 load
  switch)** from the always-on **VCC_5V** rail. U2's EN pin is held active
  by board-level pulls — there is **no GPIO that turns USB host VBUS on/off**.
- **GPIO 45** is **SD-card power enable** on the Nano (drives Q1 AO3401
  P-MOSFET gate controlling `SD1_VDD`). It has nothing to do with USB.
- **GPIO 54** is just a breakout pin on header P1; no USB role.
- Same conclusion on the Waveshare ESP32-P4-Pico (verified separately):
  Picoblade P1 pin 1 (VBUS) is hardwired to VCC_5V, and GPIO 45 there is
  also SD-card power.

**Implication for firmware:** do not drive any GPIO as a phantom VBUS_EN on
these boards. An earlier iteration drove GPIO 45/54 low under that mistaken
assumption — at best ineffective, at worst actively toggling SD-card power.

### Software-only USB recovery (despite no VBUS GPIO)

Even though VBUS itself is hardwired-on, the ESP-IDF host stack does
provide a software path to recover stuck devices:

- `usb_host_lib_set_root_port_power(false/true)` halts SOFs and re-evaluates
  attach state from the DWC OTG controller's perspective, equivalent to a
  USB cable unplug from the host's side. On boards with EN wired to a GPIO
  this would also drop physical VBUS; on the Nano it doesn't, but the
  controller-side disconnect+reattach is enough to recover most stuck-device
  states without physical intervention.
- The firmware uses this in two places:
  1. Boot: install with `root_port_unpowered=true`, then explicit
     `set_root_port_power(true)` — forces a fresh attach sequence every reset.
  2. Watchdog: if no device enumerates within 6 s (e.g. device was already
     stuck before the firmware booted), cycle `power(false)` →500ms→
     `power(true)`, up to 3 attempts. Resets the counter on success so a
     future hot-disconnect can also trigger recovery.

True hardware power-cycle of the attached USB device still requires
physical unplug or unplug of the Nano's USB-C feed. The software recovery
covers the common cases that previously needed physical intervention.

### esp-dsp 1.8.1 upstream bugs found

Two issues in the bundled esp-dsp version that the worker has to work
around (worth filing upstream):

1. **`dsps_resampler_mr_init` rejects `samplerate_factor < 1`** but does
   not propagate the error — leaves the struct uninitialised, then
   `_exec()` later dereferences a garbage `filter` pointer and faults at
   NULL+0x10. Workaround: bypass the `_mr_` wrapper and call the
   underlying `dsps_firmr_init_s16` / `dsps_firmr_s16` directly, which
   handle interp/decim ratios in either direction.
2. **`dsps_fird_s16_arp4.S` returns uninitialised register `a6`** instead
   of the output sample count. Observed as a constant `-256` across
   repeated calls. The function does write the output buffer correctly,
   it just doesn't report what it wrote. Workaround: pass the correct
   output length (`input/decim` — note that's the convention this API
   uses, vs `dsps_firmr_s16` which takes input length) and compute the
   count locally.
