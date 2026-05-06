# Iridium ACARS Decoding Stack - Implementation Plan

This document tracks the concrete implementation steps for the [Iridium ACARS Decoding Stack Design](./iridium-acars-decoding-stack-design.md).

## Current Status
- **Target:** Phase 4 (Live RF validation), gated by Phase 3.5 (throughput).
- **Status:** Phases 0, 2, 3.1–3.4 COMPLETE. End-to-end DSP pipeline runs
  against a real RTL-SDR v4 with no crashes; bursts flow through
  detect → extract → freq-centre → decimate → resample → demod → BCH
  cleanly. USB host recovers from stuck-device states without physical
  unplug.
- **Phase 3.5 (in progress):** integrated throughput optimization. Currently
  3.20 MB/s = **66% of the 4.85 MB/s real-time target** with `rb_full_drops`
  ~32%. Steps 1, 2, 4, 4b, 5 done; Step 3 (PIE/Q15 baseline EMA) is the
  remaining lever projected to close most of the gap.
- **Blocker (Phase 4):** antenna + LNA hardware (Scan QFH + SAWbird+ IR).
- **Immediate Goal:** complete Step 3 + functional regression tests, then
  acquire the RF frontend.

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

### Step 7a per-stage delta (synthetic-tone smoke)

| Stage | Step 6.5 | **Step 7a** | Δ |
|---|---|---|---|
| wind   | 16  | 15  | flat |
| fft    | 199 | 200 | flat (already PIE) |
| mag    | 74  | **51** | **−31%** |
| detect | 46  | 47  | flat |
| base   | 111 | 109 | flat |
| **DSP total/frame** | **447 μs** | **422 μs** | **−5.6%** |

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
