# HydraSDR wideband + SPI-distributed processing — feasibility

> **Status:** feasibility-only, not on the active roadmap. Drafted
> 2026-06-10 from a throughput-analysis pass after the question "could
> we use a higher-bandwidth SDR like a HydraSDR, with more P4s connected
> over SPI to do the processing?"
>
> This doc is the **wideband-single-SDR companion** to
> [`multi-receiver-spi-aggregator-design.md`](./multi-receiver-spi-aggregator-design.md).
> That doc covers N independent **RTL-SDR receivers fanning frames IN**
> to an aggregator. This one covers the inverse: **one wideband SDR
> fanning IQ OUT** to worker P4s. They are different topologies with
> different binding constraints; read that doc first — most of the
> physical-layer, PDU, and aggregator analysis there is reused here and
> not repeated.
>
> Nothing here is a commitment to build. It exists to decide whether the
> HydraSDR path is worth pursuing and, if so, under what conditions.

## 1. Why this came up

A single ESP32-P4-NANO + RTL-SDR v4 sustains **one ~2.5 MHz subband**
of Iridium L-band with 0 drops (the documented 4.88 MB/s operating
point). Iridium's duplex + simplex allocation spans roughly **1616.0 –
1626.5 MHz ≈ 10.5 MHz** (240 × 41.667 kHz duplex channels plus the
simplex Ring-Alert / IBC band). So one P4 sees ~1/4 of the band and
hunts within it.

The motivation for a HydraSDR RFOne (AirSpy-lineage, up to 10 MSPS) is
to capture the **whole Iridium band coherently from one front end**,
rather than stitching together several RTL-SDRs. The open question is
whether the ESP32-P4 — which already can't process more than ~2.5 MSPS
on one chip — can be scaled with additional P4s over SPI to actually
*use* that wider capture.

## 2. The three ceilings (why one P4 can't just "go wider")

A wideband SDR exposes three *independent* limits on the P4. They are
reached in this order, and the binding one is the lowest:

| Ceiling | v1 estimate | Set by |
|---|---|---|
| USB transport (DWC OTG bulk-IN) | ~25–40 MB/s | USB host stack; **not** the constraint |
| Firmware ingest → PSRAM | ~8–15 MB/s | PSRAM contention (CPU AXI + USB AHB + AXI-GDMA) + the GDMA internal→PSRAM copy + the 64 KB internal-SRAM URB pool |
| **Full Iridium processing** | **~5 MB/s ≈ 2.5 MSPS** | **Core 0 tagger FFT — sized for 2.5 MSPS, over real-time budget beyond ~3 MSPS** |

Figures: USB transport is triangulated (no single published P4
benchmark); the ingest knee is estimated from the ingest path's
~10–14× PSRAM write-amplification × the measured ~185 MB/s effective
single-stream PSRAM bandwidth; the processing ceiling is anchored on
the measured Core 0 budget (cycle 1972 µs of 3300; DSP 407 µs/frame vs
the 800 µs/frame real-time threshold at 2.56 MSPS). See
[`AGENTS.md`](../AGENTS.md) throughput section and the per-stage notes
in `ingest_core1.c` / `dsp_processor.c`.

**Key consequence:** the board can *ingest* far more than it can
*process*. A HydraSDR at 5 MSPS (10 MB/s) is plausibly capturable to
PSRAM on v1, but the `FBT_FFT_SIZE = 2048` tagger and the worker
pipeline can only keep real-time over ~2.5 MSPS of it. The extra
bandwidth is wasted unless the processing is distributed.

## 3. HydraSDR data rates and the protocol port

HydraSDR/AirSpy streams **plain USB 2.0 HS BULK IN** (EP 0x81, 512 B
MPS — confirmed from the AirSpy firmware descriptor), so the existing
`esp_libusb` async-URB model is structurally the right driver. No
isochronous handling needed. But it is **not** a drop-in:

- **Sample format.** 12-bit **real** samples in 16-bit words (2 B/sample
  default; 1.5 B/sample with `pack=1`, which we'd leave off — PSRAM, not
  USB, is the wall). Rates: 2.5 / 5 / 10 MSPS → **5 / 10 / 20 MB/s** on
  the bus.
- **Real → complex front end.** Unlike the RTL-SDR (interleaved uint8
  IQ), AirSpy-class devices sample real-IF and the **host** library does
  the FIR/Hilbert decimation to complex IQ. That work **does not exist
  in this firmware** and is heavier than the current uint8→Q15 convert.
  The existing 256→250 resampler is RTL-specific (2.56→2.5 MSPS) and
  would not apply.
- **Command set.** It doesn't free-run on enumeration; it needs the
  libhydrasdr/libairspy vendor command set (`SET_SAMPLERATE`,
  `RECEIVER_MODE` to start streaming, gain, freq) ported behind the
  existing tuner/device abstraction — comparable in scope to the
  `librtlsdr.c` port already in the tree.

**Port cost estimate:** a `libhydrasdr` device-control layer + a new
real→complex decimating front end, roughly the size of the existing
librtlsdr work, *before* any throughput benefit appears.

## 4. The topology question: fan-OUT IQ vs fan-IN frames

This is the crux, and it's where the HydraSDR path diverges sharply
from the RTL multi-receiver doc.

```
            ┌─ Topology A: one wideband SDR, fan-OUT raw IQ over SPI ─┐
 HydraSDR ─USB─▶ [P4 #0: ingest + CHANNELIZE] ─SPI×N─▶ [P4 worker ×N: decode] ─▶ frames ─▶ aggregator
            └────────────────────────────────────────────────────────┘

            ┌─ Topology B: N cheap SDRs, fan-IN frames (the existing doc) ─┐
 RTL ×N ─USB─▶ [P4 worker ×N: ingest+decode] ─SPI×N(280 B PDUs)─▶ [aggregator] ─▶ msg_ring
            └──────────────────────────────────────────────────────────────┘
```

### 4.1 The SPI interconnect is the deciding factor

From the multi-receiver doc: a P4 SPI link runs **~40 MHz reliable for
PSRAM-backed transfers ≈ 5 MB/s per CS**. That number governs both
topologies and it is small relative to raw IQ:

- **Topology A (fan-out IQ):** each worker needs its ~2.5 MHz subband =
  **5 MB/s over SPI** — i.e. **one worker saturates one SPI link.** To
  cover the 10 MHz band you need ~4 workers = 4 high-rate SPI fan-out
  links off P4 #0. The P4 has only **2–3 free SPI peripherals** (the
  rest are committed to the C6 wireless co-processor / PSRAM), so one
  ingest node **cannot cleanly drive 4 saturated fan-out links.** This
  is the same wall the multi-receiver doc hit when it discarded "Option
  B-1: ship raw IQ" — shipping IQ over SPI doesn't scale.

- **Topology B (fan-in frames):** a decoded-frame PDU is ~280 B; even a
  100×-real-RF flood is <1 Mbps per link (see the multi-receiver doc's
  Option B-3). The heavy IQ **never crosses the wire** — each worker
  decodes its own RTL-SDR locally. SPI is trivially adequate.

### 4.2 Topology A also needs channelization CPU the ingest node lacks

Topology A's P4 #0 must **split the wideband capture into per-worker
subbands** before fan-out (you can't ship the full 20 MB/s wideband
stream over any one SPI link). That's a polyphase/FFT channelizer —
exactly the kind of per-sample DSP that Core 0 has **no spare cycles
for** (it's already at ~60% of budget just ingesting, and the prior
channelizer was removed in Phase 3.6.M). So Topology A needs *either*:

- a **dedicated channelizer P4** between the SDR host and the workers
  (now 1 ingest + 1 channelizer + N workers + 1 aggregator), or
- **v3.1 silicon** on the ingest node (400 MHz + USB-DMA-direct-to-PSRAM
  removing the GDMA copy and the internal-pool limit — see §5), which
  frees enough headroom to channelize while ingesting.

### 4.3 When is Topology A (HydraSDR) actually worth it?

On pure "more channels" grounds, **Topology B wins**: N cheap RTL-SDRs
fanning 280 B frames in is cheaper and far easier on the interconnect
than one HydraSDR fanning 5 MB/s IQ slices out. The HydraSDR only earns
its place when its **single coherent wideband capture** matters:

- **Seamless coverage / no seam gaps.** N independent RTL-SDRs have N
  independent LOs and clocks; subband edges have to overlap-with-margin
  and you still risk a burst landing in a seam. One HydraSDR covers the
  band continuously with one clock.
- **Phase coherence across the band** — required for any future
  direction-finding, cross-channel time-alignment, or coherent
  combining. N RTL-SDRs cannot do this (no shared clock/phase).
- **One RF front end / antenna chain** instead of N — simpler RF, one
  LNA/filter, less desensitization between receivers.

If none of those apply, prefer Topology B and don't buy a HydraSDR.

## 5. What v3.1 silicon changes

v3.1 (400 MHz cores; MSPI-750 / APM-560 fixed) materially improves the
HydraSDR case but does **not** remove the processing ceiling:

- **USB-DMA-direct-to-PSRAM becomes safe** (`CONFIG_USB_HOST_DWC_DMA_CAP_MEMORY_IN_PSRAM=y`)
  → eliminates the internal-SRAM URB pool limit **and** the GDMA
  internal→PSRAM copy (one whole bus master + one write-pass of the
  amplification factor gone). Biggest single ingest win.
- **GDMA INCR4/8/16 bursts** push effective PSRAM utilization toward the
  ~400 MB/s raw class rather than ~185 MB/s single-stream.
- **400 MHz (+11%)** gives the tagger modest headroom — still only
  ~3 MSPS-class real-time, **not** 5+. The Core 0 processing wall
  survives; distributing the work (Topology A or B) remains the only way
  past it.

Net: v3.1 makes a single P4 a *viable channelizer/ingest node* for
Topology A (it can ingest 10 MSPS and have cycles to channelize), which
on v1 it cannot. It does not let one P4 fully process the wide band.

## 6. Interconnect alternatives to SPI (if Topology A is pursued)

The user framing assumes SPI, and §4.1 shows SPI caps Topology A at
~1 worker per link. If coherent-wideband (Topology A) is genuinely
needed, the IQ-distribution leg likely wants a **faster interconnect**
than GPSPI:

- **OSPI / octal SPI** at higher clocks (the P4's PSRAM-grade MSPI runs
  at 200 MHz) — but those controllers are committed to PSRAM/flash.
- **The SDIO link** already used for the C6 (ESP-Hosted) runs ~50 MB/s —
  enough for a couple of subbands, but it's a host↔single-slave link,
  not a fan-out bus.
- **Parallel / I80 LCD-cam interfaces** can move tens of MB/s but are
  awkward as a multi-drop fabric.

None is a clean N-way fan-out. This is the strongest structural argument
that **a single-wideband-SDR + on-board distribution does not fit the
P4's interconnect model well**, and that the frame-fan-in topology
(Topology B) is the architecture the P4 actually wants — it keeps the
high-bandwidth IQ on-chip with its SDR and only ever ships tiny frames
between chips.

## 7. Recommendation

1. **Default to Topology B** ([`multi-receiver-spi-aggregator-design.md`](./multi-receiver-spi-aggregator-design.md)):
   N RTL-SDR-per-P4 workers fanning decoded-frame PDUs into an
   aggregator. It is cheaper, the interconnect is trivial, and it
   reuses the existing single-P4 firmware almost unchanged. For "see
   more of the band / decode more," this is the answer.

2. **Reserve the HydraSDR (Topology A) for coherence-driven goals** —
   seamless coverage, phase-coherent wideband, single RF chain. It is
   not a throughput win on its own and it fights the P4's interconnect.

3. **If Topology A is pursued, gate it on v3.1 silicon** for the
   ingest/channelizer node (400 MHz + USB-DMA-to-PSRAM), accept a
   dedicated channelizer node, and budget a **faster-than-GPSPI**
   IQ-distribution fabric — one ~2.5 MHz subband per high-speed link.

4. **The firmware port is real regardless of topology choice for
   HydraSDR:** a `libhydrasdr` command layer + a real→complex
   decimating front end (≈ the size of the librtlsdr port) must land
   before any HydraSDR build streams at all. Prove enumeration +
   streaming-start on the bench with the front end stubbed before
   committing to the wider architecture.

5. **Shared `iridium_decoder` component already supports both
   topologies** — the worker decode path is board-agnostic by design
   (`common/iridium_decoder/`). Neither topology needs the decoder
   forked; they differ only in the top-level binary (ingest/channelize
   vs decode vs aggregate), which is the same Kconfig-profile split the
   multi-receiver doc already calls for.

## 8. Open questions to resolve before any code

1. **Real measured Core 0 processing ceiling** — the ~2.5–3 MSPS wall is
   estimated from the FFT budget. Confirm with a bench sweep of
   `FBT_FFT_SIZE` / sample-rate against real-time before sizing any
   multi-board split.
2. **HydraSDR real→complex front-end cost on P4** — the host-side
   Hilbert/FIR decimation is unbudgeted. Prototype it on one P4 at
   2.5 MSPS and measure Core 1 occupancy before assuming a worker can
   also run it.
3. **Channelizer feasibility on v3.1** — does a 400 MHz P4 with
   USB-DMA-to-PSRAM actually have the cycles to split 10 MSPS into 4
   subbands while ingesting? Unproven; needs v3.1 hardware.
4. **IQ-distribution fabric** — §6 has no clean N-way fan-out answer.
   This is the single biggest unknown for Topology A and should be
   settled (or Topology A abandoned) before anything else.
5. **Cost/benefit vs Topology B** — quantify what coherent wideband buys
   for the actual decode mission (ACARS). If the answer is "nothing the
   frame-fan-in topology can't do," Topology A is a research curiosity,
   not a roadmap item.
