# Iridium ACARS Decoding Stack - Implementation Plan

This document tracks the concrete implementation steps for the [Iridium ACARS Decoding Stack Design](./iridium-acars-decoding-stack-design.md).

## Current Status
- **Target:** Phase 3 (Single-Board Prototype)
- **Status:** Phase 3.1, 3.2, and 3.3 are COMPLETE.
- **Immediate Goal:** Final RF validation and ACARS packet assembly.

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
- [x] Implement **FIR Decimation** (32x) and **Resampling** (1.6x) to 50 kHz (2sps).
- [x] Port the **QPSK/DQPSK Demodulator** with PLL phase tracking.
- [x] Port the **BCH(31,21) Decoder** with Hard-Decision block correction.
- [x] **Milestone:** Verified "BCH DECODE SUCCESS" with live data bits.

---

## Phase 4: Maintenance & Documentation (TODO)
- [ ] Add a comprehensive **README.md** with build instructions and architecture overview.
- [ ] Implement automated CI/CD for firmware builds.

## Phase 1a: Lite Node RF Validation (Blocked)
*Requires: 1620 MHz QFH/Patch Antenna, Nooelec SAWbird+ IR.*
- [ ] Connect RTL-SDR v4 + LNA + Antenna.
- [ ] Run live decoding stack on P4 and verify real-world frame reception.

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
Phase 2's 5.12 MB/s success was achieved without any VBUS GPIO code, which
is consistent with USB host VBUS being always-on.

There is no software path to VBUS-cycle the host port on this hardware.
Power-cycling an attached USB device (e.g. RTL-SDR) requires physical
unplug/replug of the device or of the Nano's USB-C feed.
