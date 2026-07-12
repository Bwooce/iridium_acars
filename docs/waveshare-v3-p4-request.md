# ESP32-P4 v3 / 400 MHz board request — P4 PIE & silicon issues

**Date:** 2026-07-12
**Current board:** Waveshare ESP32-P4-NANO (P4NRW32 = **P4 silicon v1.3**, 32 MB PSRAM, C6-MINI-1 for Wi-Fi via esp_hosted/SDIO)
**Firmware:** real-time SDR DSP pipeline (Iridium L-band → ACARS), ESP-IDF v6.1, using the P4 **PIE 128-bit integer vector unit** heavily on **both** HP cores. We currently carry 7 IDF/component patches (`patches/0001-0007`) plus several in-app workarounds, most of them for PIE/silicon behaviour.

## TL;DR for the vendor
- **The ask is NOT primarily throughput.** A 400 MHz part over our current 360 MHz cap is only ~11% raw (~13–18% system headroom after the v3 HWLP-tax recovery) — that does **not** make dense-burst peaks sustainable, and our workload doesn't need it to (peak drops are ~99% benign control chatter; reception SNR, not compute, is the yield limiter).
- **The value of a v3 stepping is deleting workaround debt** — specifically two undocumented PIE silicon behaviours (S1, S2 below) that force an "early-alloc placement dance" and a "one-PIE-owner-per-core" architectural straitjacket, plus a documented HWLP tax (S4).
- **Three concrete asks** (see bottom): (1) rev **v3.1+**, (2) Espressif confirmation on S1/S2, (3) move the C6 reset off GPIO54.

---

## (S) Silicon — a v3 stepping plausibly removes these

| # | Issue | Evidence / repro | v3 outlook |
|---|---|---|---|
| **S1** | **PIE address-dependent silent corruption in HP-SRAM.** `esp.vld.128` reads of buffers placed mid-main-SRAM (~`0x4ff6xxxx`–`0x4ff7xxxx`) return silently-wrong vectors; the *same bytes* read correctly at other addresses and via scalar loads. No fault raised. Workaround: a boot-time "early-alloc dance" pinning every PIE buffer low + freezing struct sizes (growing a struct shifts buffers and re-triggers it). | Falsified all cache-sync theories via `esp_cache_msync` A/B; scalar access at the "broken" addresses passes. NOT in the public errata matrix. | Same class as the v1.x bus-matrix flaws reworked for v3 — **likely fixed, unverified. Please confirm.** |
| **S2** | **PIE hangs the core during the FreeRTOS coprocessor owner-swap.** An `esp.vst.128` Q-register store inside `rtos_save_pie_coproc` never retires → HP-WDT reset. **Five** independent software mitigations were falsified with build-matched disassembly (internal-RAM save area, trap-storm watchdog, fence/nop/alignment, strict-aligned CFG). Only avoidance: architecturally forbid two PIE-using tasks pinned to one core. | Deterministic repro; full falsification log retained. NOT in the errata matrix. | **Unknown — this is the single most important question for Espressif.** The one-PIE-owner-per-core constraint is our biggest architecture tax. |
| **S3** | **PIE returns garbage (silently) on RTCRAM (`0x5010xxxx`) and TCM.** A `MALLOC_CAP_INTERNAL` fallback into RTCRAM cratered decode 95%→6% with no fault. | Reproduced 2026-07-05. | May be architectural (PIE bus reach) rather than a bug — either way, **a bus-error instead of silent garbage would suffice; please document.** |
| **S4** | **HWLP (`esp.lp.setup`) hardware-loop state bugs** (`SOC_CPU_HAS_HWLOOP_STATE_BUG` + a spurious EXT_ILL reason bit + esp-dsp alignment sensitivity). Forced us to strip all hardware loops from esp-dsp's PIE kernels (patch `0002`): **~5–15% cost per kernel, +15–20% on the sc16 FFT.** | patches/README 0002; IDF `portasm.S` gates this to silicon rev ≤1. | **Documented fixed ≥ v3** → patch 0002 drops, that perf recovers. |
| **S5** | **Documented v1.3 errata we already work around:** MSPI-750/751 (PSRAM DMA alignment **+ the 360 MHz CPU cap**), DMA-767 (GDMA ch 0), APM-560 exposure. | `sdkconfig`: `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_360=y`, `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y`. | **MSPI-751 is fixed in v3.1 (not v3.0)** — so **spec v3.1+** to actually unlock 400 MHz. |

## (T) Toolchain / IDF — a new board does **not** fix these (same ESP-IDF)
- `dsps_fft2r_init_sc16(NULL, sz)` silently ignores its size argument (esp-dsp 1.8.1) — caller-owned twiddle table required.
- esp-dsp `arp4` PIE FFT kernels are non-bit-exact vs a textbook radix-2 in the small-N2 butterfly tail — any adoption needs a bit-exact diff harness (we built one).
- IDF P4 FreeRTOS port gaps: the **PIE CFG register is not context-switched at all**; coprocessor save areas are carved from task stacks (→ 128-bit PIE stores into PSRAM for PSRAM-stacked tasks, our patch `0003`).
- These patches travel with us to any board.

## (C) Our code / inherent to integer SIMD — clock helps linearly, a stepping changes nothing
- PIE is int16/Q15-only. The gri-parity float matched filter and spectral multiply stay scalar **by choice** — the Q15 version measurably lost borderline-SNR resolution. Plus Q15 incremental-phasor magnitude decay; xacc/qacc rules; `lp.setup` body constraints; the unaligned-`vld` CFG bit. The cost of doing fixed-point SIMD DSP anywhere.

---

## What 400 MHz alone buys (the blunt arithmetic)
Current clock is **360 MHz** (MSPI-751 cap). 400 MHz = **+11%** raw. Against measured load: Core-0 tagger 97% median → ~87%; Core-1 worker peaks 172% → ~155%. Adding back the v3 HWLP-tax recovery (~5–15% on PIE kernels ⇒ low single digits system-wide) gives a realistic **~13–18% total headroom**. That does **not** make dense-burst peaks sustainable — and it doesn't need to: the SNR-priority queue sheds correctly and the shed is ~99% benign control chatter. **Throughput is not the reason to swap boards.** Deleting the workaround debt (S1/S2/S4) is.

## Board-design flag for Waveshare
On v3 silicon **GPIO54 is reassigned NC → VDD_HP_1** (a power rail). Our deployed firmware drives **GPIO54 as the C6 hardware-reset line** (`wifi_link.c`, the anti-brick recovery for a wedged C6/SDIO link). **A v3 board must route the C6 reset to a different P4 GPIO**, or that recovery is lost. (Our firmware already carries a v3 warning at `usb_host_lib_main.c`.)

## Concrete asks
1. **P4 rev v3.1 or later** — the MSPI-751 fix (hence 400 MHz) is v3.1, not v3.0.
2. **Espressif confirmation** whether **S1** (PIE address-dependent HP-SRAM corruption) and **S2** (PIE coprocessor owner-swap hang) are fixed in v3 — neither is in the public errata matrix; we can supply deterministic repros.
3. **Move the C6 reset line off GPIO54** on the board layout.

---
*Key sources in-repo: `patches/README.md`; `common/iridium_decoder/{fft_sc16_2048.c,uw_correlator.c}`; `p4-usb-host/main/{worker_core1.c,wifi_link.c,usb_host_lib_main.c}`; commits `6f3d686`, `7033a48`, `cf929d2`, `cfd2739`, `53327e7`, `6be9d4e`; project notes on the PIE save-deadlock, heap-position corruption, and v1.3 errata audit.*
