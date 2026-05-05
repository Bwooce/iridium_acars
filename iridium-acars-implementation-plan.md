# Iridium ACARS Decoding Stack - Implementation Plan

This document tracks the concrete implementation steps for the [Iridium ACARS Decoding Stack Design](./iridium-acars-decoding-stack-design.md).

## Current Status
- **Target:** Phase 4 (Live RF validation).
- **Status:** Phases 0, 2, 3.1, 3.2, 3.3 are COMPLETE. End-to-end DSP pipeline
  runs against a real RTL-SDR v4 with no crashes; bursts flow through
  detect → extract → freq-centre → decimate → resample → demod → BCH cleanly.
  USB host recovers from stuck-device states without physical unplug.
- **Blocker:** Phase 4 needs antenna + LNA hardware (Scan QFH + SAWbird+ IR).
- **Immediate Goal:** Acquire the RF frontend; first real Iridium decode.

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
