# Iridium ACARS Decoding Stack — Pole-Mounted Design (Rev 5)

*See the [Implementation Plan](./iridium-acars-implementation-plan.md) for the active step-by-step execution guide.*

## 1. System Overview

This design captures Iridium L-band signals (1616–1626.5 MHz), demodulates the TDMA bursts, and extracts ACARS messages carried via Iridium Short Burst Data (SBD). The architecture centers on the ESP32-P4 High-Speed USB host capability and PIE (Processor Instruction Extensions) for real-time DSP.

The "Lite Node" variant (single-board, RTL-SDR v4) is the current primary implementation, achieving a full end-to-end baseband pipeline on a single Waveshare P4-Nano.

### Target Signals

| Parameter | Value |
|---|---|
| Band | 1616.0–1626.5 MHz (L-band) |
| Active bandwidth | ~8.5 MHz (channels 1–240) |
| Channel spacing | 41.667 kHz |
| Channel count | 240 simplex TDMA channels |
| Modulation | Burst DQPSK, ~25 ksym/s per channel |
| Burst duration | ~8.28 ms |
| ACARS transport | Iridium SBD (Short Burst Data) within IDA frames |

---

## 2. System Architecture (Lite Node)

```
                          ┌─────────────────┐
                          │  Iridium QFH    │
                          │  Antenna (RHCP) │
                          │  1616-1626.5MHz │
                          └───────┬─────────┘
                                  │ 
                          ┌───────▼─────────┐
                          │  LNA + SAW      │
                          │  Nooelec        │
                          │  SAWbird+ IR    │
                          └───────┬─────────┘
                                  │ 
                          ┌───────▼─────────┐
                          │   RTL-SDR v4    │
                          │   2.56 MSPS IQ  │
                          │   ~2.4 MHz BW   │
                          └───────┬─────────┘
                                  │ USB 2.0 HS (HUSB port)
          ┌───────────────────────▼──────────────────────────┐
          │            ESP32-P4 (Waveshare Nano)             │
          │                                                  │
          │  Core 0: Ingestion & Detection                   │
          │  ├─ USB HS Async DMA (8x16KB transfers)          │
          │  ├─ Lookback Ring Buf (4MB PSRAM)                │
          │  └─ FFT Detector (2048-pt sc16, ~930μs)          │
          │                                                  │
          │             handoff via Burst Queue              │
          │                        │                         │
          │  Core 1: Burst Decoding                          │
          │  ├─ Freq Centering (Complex Rotation)            │
          │  ├─ FIR Low-pass & Decimation (32x + 1.6x)       │
          │  ├─ QPSK Demodulator (PLL Phase Tracking)        │
          │  └─ BCH(31,21) Error Correction                  │
          │                                                  │
          └────────────────────────┬─────────────────────────┘
                                   │ 
                            Decoded ACARS
                            (JSON over WiFi)
```

---

## 3. Hardware Platform Details

### 3.1 ESP32-P4 Core Specifications

| Feature | Specification |
|---|---|
| CPU | Dual-core RISC-V, 360-400 MHz (HP cores) |
| SIMD | PIE: eight 128-bit vector registers |
| PSRAM support | 16 MB Octal SPI (200 MHz) on Nano board |
| USB (HUSB) | USB 2.0 OTG High-Speed (480 Mbps), integrated PHY |

---

## 4. Software Architecture: Dual-Core Pipeline

### 4.1 Porting Target and DSP Library

**esp-dsp** (Espressif's official DSP library) provides PIE-optimised implementations.

| esp-dsp Function | Use in This Project | Performance |
|---|---|---|
| `dsps_fft2r_sc16_arp4` | FFT energy detector (2048-point) | ~210 μs per FFT; ~930 μs total pipeline |
| `dsps_fird_s16_arp4` | FIR decimation filter (32x) | PIE-optimised |
| `dsps_cplx_gen` | Phasor for frequency centering | PIE-optimised |

### 4.2 Timing and Budget

At **2.56 MSPS**, the real-time budget for a 2048-sample frame is **800 μs**.
Our current measured performance for the detection pipeline (Core 0) is **~930 μs**.

**Status:** The system is technically over-budget for bit-perfect continuous 2.56 MSPS on a single core. The current implementation uses a **4MB Lookback Buffer** in PSRAM to mask this deficit. For sustained high-density traffic, the rate should be reduced to **2.0 MSPS** (budget: 1024 μs) or the FFT size reduced to 1024-pt.

---

## 11a. Lite Node Variant (RTL-SDR V4, Single Board)

This is the baseline implementation for the project.

### 11a.2 Performance Envelope

| Factor | Baseline (RTL-SDR v4) | Status |
|---|---|---|
| Sample rate | 2.56 MSPS (target) / 2.0 MSPS (fallback) | |
| USB data rate (target, 2.56 MSPS, int8 IQ) | **4.85 MB/s** real-time at 16 KB transfers | |
| USB data rate (dry benchmark, no DSP load) | 5.12 MB/s | achieved Phase 2 |
| **USB data rate (integrated, with DSP)** | **3.20 MB/s = 66% of target** | as of 2026-05-06, post-Step 5 |
| FFT detector | 2048-pt sc16, ~1250 frames/sec | |
| Core 0 budget (2.56) | 800 μs / FFT frame (930 μs actual) | **Tight** |
| Core 0 cycle (per 16 KB USB transfer) | 4374 μs (need ≤3300 for real-time) | **Over** |
| PSRAM Slack | ~400 ms @ 5.12 MB/s, ~32% drops at integrated rate | |

**Note on the "5.12 MB/s achieved" claim from Phase 2:** that was the dry
benchmark — USB streaming with no tuner-driving I2C, no DSP feed, and a
broken PLL that happened to make the buffer easier to drain. After
integrating tuner control and the live DSP pipeline, throughput dropped to
1.22 MB/s and has been climbing back through Phase 3.5 optimisation steps.
Current integrated number is 3.20 MB/s; Step 3 (PIE/Q15 baseline EMA) is
projected to close most of the remaining gap.

### 11a.3 Dual-Core Split
- **Core 0:** USB Ingestion (DMA) + Circular Buffering + FFT Energy Detection.
- **Core 1:** Burst Extraction + Complex Rotation + FIR filtering + QPSK Demod + BCH Decode.

This split ensures that detection latency (930 μs) does not block the time-critical USB DMA ingestion. The 4MB PSRAM buffer allows the detector to run slightly "behind" real-time without losing signal starts.

**Integration status (Phase 3 complete):** the full pipeline runs end-to-end against a live RTL-SDR v4 stream with no crashes. Stage 1 FIR (32× decimation) and Stage 2 polyphase resample (5/8) produce the expected sample counts (`input/32` and `input × 5/8` respectively). 177 bursts processed in 14 s of indoor testing with no panics. Real-RF validation against actual Iridium signals is pending antenna hardware (Phase 4).

### 11a.3a USB Stuck-Device Recovery (software-only)

VBUS to the host port is hardwired-on through U2 — no GPIO controls it. Despite this, software recovery from stuck-device states works without physical unplug:

- Boot with `usb_host_config_t.root_port_unpowered = true`, then call `usb_host_lib_set_root_port_power(true)` after install. This forces a fresh USB attach sequence on every reset.
- Recovery watchdog in the class driver task: if no device enumerates within 6 s, cycle root port power (`false → 500 ms → true`), up to 3 times.

The controller-side disconnect/reconnect is enough to recover most stuck states even though physical VBUS doesn't drop. The watchdog has been observed firing and successfully recovering an RTL-SDR that was left stuck by an unclean prior reset.

### 11a.4 Roof-Mount Install: Lightning & Surge

All electronics live in a single rooftop enclosure (LNA at the antenna, SDR + ESP32-P4 in the box). Output is via WiFi (ESP32-C6 companion) — no metallic data cable leaves the roof. The protection scheme is built around the enclosure as the single point of ground reference.

**RF chain placement:**

```
Antenna → SAWbird+ LNA (sacrificial) → ARRESTOR (at enclosure wall) → SDR → ESP32-P4
```

- **Coax arrestor:** gas-discharge-tube type (Polyphaser IS-50UX-MA, NexTek IS-LP-G-MA, or equivalent), mounted through the enclosure wall *after* the LNA, before the SDR.
- **LNA placement:** at the antenna (short coax) — accepts the LNA as a sacrificial component in a direct strike. Putting the arrestor before the LNA degrades system NF and is the wrong trade for amateur work.

**Bonding (the load-bearing part):**

- Enclosure body → ground rod via short, straight #6 AWG copper strap. No loops, no sharp bends.
- Antenna mast → same ground rod, separately bonded.
- Single-point ground discipline: enclosure, mast, and building electrical earth all meet at one busbar. Multiple separate grounds are worse than no ground.
- 1.8 m copper-clad ground rod, driven into damp earth, as direct a path from enclosure as possible.

**Power line protection:**

- DC surge suppressor at enclosure power entry (Phoenix Contact / Citel DIN-rail unit, or equivalent in-line TVS network).
- MOV-style mains protector on the AC outlet feeding the 12V supply at the building end.

**Cable discipline:**

- Antenna coax and ground strap perpendicular where they cross — never parallel runs.
- Drip loops below cable entries to the enclosure.
- Connector weatherproofing: self-amalgamating tape (Scotch 23) under vinyl tape on every outdoor joint.

**Annual inspection:** GDT cartridges degrade after firing. Polyphaser IS-series uses a serviceable replaceable cartridge.

**Cost:** ~$60 arrestor + ~$25 DC suppressor + ~$15 ground rod + heavy strap ≈ **$100** to insure a $500+ rooftop install.

---

## 12. Conclusion

The ESP32-P4 architecture for Iridium ACARS is firmware-complete and
runs at full real-time throughput. detect → extract → freq-centre →
decimate → resample → DQPSK → BCH runs end-to-end against a live
RTL-SDR v4. USB stuck-device recovery is fully software-driven via
root-port-power cycling.

**Throughput status (Phase 3.5 closed):** 4.88 MB/s = 100.5% of the
4.85 MB/s real-time target at 2.56 MSPS, with `rb_full_drops = 0`.
Core 0 cycle headroom ~35%. The arc went from 1.22 MB/s (broken PLL)
through 11 incremental steps to 4.88 MB/s — see
[`iridium-acars-implementation-plan.md`](./iridium-acars-implementation-plan.md)
for the per-step breakdown. Single biggest win: switching from
`-Og` to `-O2` (Step 6, 3.20 → 4.61 MB/s), which dwarfs every
architectural change. Recorded as a memory rule so we never measure
performance under `-Og` again.

**Functional regression tests (closed):** five test layers gate every
numerical change — target smoke (synthetic + real-corpus IQ), host
unit tests for `bch_decoder` and `qpsk_demod`, and a bit-level corpus
regression that compares qpsk_demod output to upstream gr-iridium
ground truth (currently 0/382 bit differences). Plus a synthetic
low-SNR (~10 dB) variant to catch demod-margin regressions invisible
on the high-SNR corpus.

The remaining work is **live RF validation** (Phase 4):
the indoor pipeline has only seen RFI and harmonics, not real Iridium
bursts. Decoding a real ACARS frame is gated on the antenna + LNA
hardware (Scan QFH + SAWbird+ IR) plus the lightning protection
described in §11a.4.
