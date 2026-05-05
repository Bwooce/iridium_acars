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

| Factor | Baseline (RTL-SDR v4) |
|---|---|
| Sample rate | 2.56 MSPS (Tested) / 2.0 MSPS (Safe) |
| USB data rate | 5.12 MB/s (Bit-perfect achieved) |
| FFT detector | 2048-pt sc16, ~1250 frames/sec |
| Core 0 budget (2.56) | 800 μs (930 μs actual — **Tight**) |
| Core 0 budget (2.0) | 1024 μs (930 μs actual — **Safe**) |
| PSRAM Slack | ~400ms @ 5.12 MB/s |

### 11a.3 Dual-Core Split
- **Core 0:** USB Ingestion (DMA) + Circular Buffering + FFT Energy Detection.
- **Core 1:** Burst Extraction + Complex Rotation + FIR filtering + QPSK Demod + BCH Decode.

This split ensures that detection latency (930 μs) does not block the time-critical USB DMA ingestion. The 4MB PSRAM buffer allows the detector to run slightly "behind" real-time without losing signal starts.

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

The ESP32-P4 architecture for Iridium ACARS is proven. By leveraging PIE-optimized fixed-point math and Octal PSRAM, we have successfully implemented a real-time baseband pipeline capable of sustained 5.12 MB/s ingestion and decoding. While the 2048-pt FFT pushes Core 0 to its limit at 2.56 MSPS, the Dual-Core offloading and PSRAM buffering provide the necessary stability for reliable frame recovery.
