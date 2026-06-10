# HydraSDR wideband + SPI-distributed processing — feasibility

> **Status:** feasibility-only, not on the active roadmap. Drafted
> 2026-06-10 from a throughput-analysis pass after the question "could
> we use a higher-bandwidth SDR like a HydraSDR, with more P4s connected
> over SPI to do the processing?"
>
> This doc is the **wideband-single-SDR companion** to
> [`multi-receiver-spi-aggregator-design.md`](./multi-receiver-spi-aggregator-design.md).
> That doc covers N independent **RTL-SDR receivers fanning frames IN**
> to an aggregator (Topology B). This one adds **one wideband SDR fanning
> IQ OUT** (Topology A) and, after a follow-up question, **a hybrid where
> the ingest P4 processes some bursts locally and offloads the overflow
> to adjunct P4s** (Topology C, §4.4 — the most pragmatic of the three).
> Read the multi-receiver doc first; its physical-layer, PDU, and
> aggregator analysis is reused here and not repeated.
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

### 4.4 Topology C: hybrid local + adjunct worker offload (deepen one band)

A third option, raised after the fan-out/fan-in framing: **the ingest P4
processes what it can locally and offloads the overflow to one or more
adjunct P4s.** This is not about widening the band — it's about
*deepening* the processing of a single band that one P4's worker can't
keep up with.

**The motivation is measured, not hypothetical.** Under a bench-RF flood
the tagger emits ~140 bursts/sec but the per-burst worker (the
`burst_pipeline` UW-correlation + demod + BCH chain, ~50 ms wall) can
only drain ~20/sec — so `worker_core1.c:869-871` records
`worker_dropped=130/s indefinitely`. The worker, **not** ingest, is the
bottleneck for decode depth on one band. Adding worker capacity is
exactly what an adjunct P4 provides.

**Why the split point is ideal.** The split is at the **burst queue**
(post-tagger / post-decimation, pre-`burst_pipeline`):

```
 [ingest P4]  USB ─▶ resample ─▶ tagger ─▶ extract+decim ─┬─▶ LOCAL burst_pipeline ─▶ frame_decoder ─▶ msg_ring
                                                          │                                    ▲
                                              overflow ───┘                                    │ frames back
                                                          ▼ 8 KB burst PDU (SPI)               │ (280 B)
 [adjunct P4]                                   burst_pipeline (rotate/UW/demod/BCH) ───────────┘
```

- **What crosses the wire is the *decimated* 250 ksps burst window**, not
  raw IQ. A typical single-slot burst is ~8 ms × 250 ksps × 4 B ≈ **8 KB**
  (worst-case multi-frame up to ~250 KB, rare). The ingest P4 keeps the
  cheap stages (extract + the PIE-FIR 10× decim) and offloads the
  expensive stage (the UW correlator's multiple 2048-pt FFTs, which
  dominate the ~50 ms/burst).
- **Flow-controlled to adjunct capacity.** You ship only as fast as the
  adjunct's queue drains (~20 bursts/sec), so each adjunct SPI link
  carries ~20 × 8 KB = **~160 KB/s** — trivially within the ~5 MB/s SPI
  budget, and *self-throttling* (no risk of flooding the link). This is
  the decisive difference from Topology A's 5 MB/s continuous IQ stream.
- **The ingest P4 doubles as aggregator.** It already runs
  `frame_decoder` + `msg_ring` + `acars_push`. The adjunct returns the
  same 280 B post-BCH frame PDU as Topology B, which the ingest P4 feeds
  into its own `frame_decoder`. No separate aggregator node needed for a
  small cluster.
- **Offloading *reduces* the ingest P4's Core 1 load** — it sheds the
  expensive worker bursts it can't process anyway. Core 0 (~35% idle)
  hosts the SPI-master offload as a DMA-driven low-priority task; the
  burst is already in PSRAM, so shipping it adds ~160 KB/s of PSRAM read
  per adjunct (negligible vs the ~50–70 MB/s already flowing).

**QoS refinement (recommended).** The tagger gives an SNR per burst.
Process the **highest-SNR bursts locally** (most likely real) and ship
the marginal ones to adjuncts. Then even if an adjunct is slow or absent,
the high-value bursts are never the ones dropped — graceful degradation
instead of FIFO loss.

**Why the existing multi-receiver doc discarded this and why it's wrong
to here.** That doc's "Option B-2: ship tagged bursts" was rejected
because it assumed *one aggregator runs every receiver's worker*
(N × 150/s × 50 ms = doesn't fit). Topology C is the opposite shape: each
adjunct is a **1:1 (or 1:few) co-processor for one ingest node's
overflow**, not a central aggregator for N receivers. Per-adjunct load is
bounded by the adjunct's own throughput; you scale decode depth by adding
adjuncts (1 ingest + k adjuncts ≈ (k+1)× worker capacity on one band).

**This is the right answer to "process some locally, some on an adjunct."**
It is more SPI-friendly than Topology A (bursty 160 KB/s vs continuous
5 MB/s), needs **no channelizer** and **no second SDR**, attacks the
actual measured bottleneck (`worker_dropped`), and reuses the existing
single-P4 firmware almost verbatim — the adjunct *is* the current worker
pipeline behind an SPI-slave burst intake, and the ingest P4 is the
current firmware plus an overflow-shipping branch on the burst queue.

It composes with the others: Topology C deepens each band; run it under a
Topology B array to get both wider coverage *and* deeper per-band decode.

### 4.5 "Process all channels at once" — detection is the wall, not decode

The motivating goal for the HydraSDR is to **process the entire ~10 MHz
Iridium band in real-time from one coherent capture and hand off frames**.
The reason this is hard is *not* the frame handoff (Topology C makes that
trivial). It's that "process all channels" splits into two problems that
distribute very differently:

- **Detection (the tagger):** find bursts against the noise floor across
  the whole band. **Must run on the raw sample stream.** Cost scales with
  *bandwidth* — the full ~10 MHz is ~4× the 2.5 MHz one P4's Core 0
  already nearly fills (the tagger FFT + per-bin EMA baseline is the
  documented Core 0 real-time bottleneck). A wideband tagger over 10 MHz
  is **~4–5× one P4's tagger budget** — one chip cannot run it in
  real-time, on v1 *or* v3.1.
- **Decode (the worker):** turn a *detected* burst into a frame. Cost
  scales with *burst rate*, and a burst is ~1% of the stream. This is
  what Topology C ships as 8 KB PDUs and distributes linearly. **This part
  is solved.**

So the binding constraint is **wideband detection**, and it creates a
trap specific to a single wide SDR:

1. **You can't run the full-band tagger on the one ingest chip** — it's
   4–5× over Core 0 budget. (And on v1 you can't even *ingest* 10 MSPS:
   ~20 MB/s exceeds the ~8–15 MB/s firmware ingest ceiling. v3.1's
   USB-DMA-to-PSRAM is required just to capture the stream — see §5.)
2. **You can't distribute the detection without moving the firehose.**
   Splitting detection across N nodes means fanning the ~20 MB/s wideband
   stream (or its channelized subbands) out to them — and one ingest chip
   **cannot push 20 MB/s over its 2–3 free SPI links at ~5 MB/s each**
   (§4.1–4.2). Channelization is itself the same per-sample DSP the chip
   has no spare cycles for.

In short: **a single wide SDR funnels all the samples through one chip's
I/O — and that is exactly the chip that can neither run full-band
detection nor redistribute the stream fast enough to offload it.** The
per-burst decode handoff you asked about is the easy half; the half that
doesn't close is getting *detection* done across the whole band.

(Note also that one HydraSDR at its 10 MSPS max yields ~9 MHz usable —
most of the 10.5 MHz Iridium allocation, not quite all of it. A second
capture or a slightly narrower goal closes that, but it's a side issue
next to the detection wall.)

**What actually closes "process all channels in real-time":**

- **Split the SDR, not the stream (Topology B is the real answer).** Give
  each P4 its own RTL-SDR tuned to its own ~2.5 MHz slice. Each chip
  detects *and* decodes only the samples it captured — **no wideband data
  ever has to be redistributed.** Four RTL-SDRs on four P4s cover the band
  and each runs comfortably within its own budget. This is precisely why
  the P4's interconnect model "wants" the SDR split across the processing
  nodes. The coherence you'd get from one HydraSDR is the price.
- **Or use a bigger host for the wideband node.** If a single coherent
  capture is non-negotiable, the ingest+detect+distribute node should be
  something with real I/O bandwidth — a Linux SBC (Pi-class) that runs the
  wideband channelizer/tagger and fans *detected bursts* out over Ethernet
  to P4 decode workers (the workers stay useful as cheap Topology-C-style
  decode nodes). At that point the P4 is the *worker*, not the wideband
  brain.
- **Or accept a narrower coherent band.** A v3.1 P4 might ingest and
  detect ~2.5–3 MHz coherently from a HydraSDR (a slice, not the whole
  band) and deepen it with Topology C adjuncts. That buys HydraSDR
  coherence over a fraction of the band, not "all channels."

**Bottom line for the HydraSDR-everything goal:** the frame handoff is
fine; the wideband **burst detection** is what a P4 cluster can't do from
a single SDR, because detection needs every sample and the samples are
trapped on the one chip that can't process or redistribute them at full
band. To process all channels in real-time, either split the capture
across the processing nodes (N SDRs, Topology B) or put a more capable
host on the wideband front end.

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

The three topologies answer different goals — they are not competing for
the same job:

| Goal | Topology | Why |
|---|---|---|
| Decode **more of one band** (beat `worker_dropped`) | **C — local + adjunct offload** | Attacks the measured worker bottleneck; bursty 160 KB/s/adjunct SPI; one SDR; reuses current firmware |
| **All channels at once**, real-time (the stated goal) | **B — RTL-per-worker, fan frames in** | Detection must run on the samples; split the SDR so each P4 detects+decodes its own slice — no firehose to redistribute (§4.5) |
| **Coherent** wideband (DF, seamless, one RF chain) | **A — HydraSDR**, but **not on a P4 cluster** | One SDR funnels all samples through one chip that can't run full-band detection or fan the stream out (§4.5); needs a bigger wideband host |

1. **For "process some locally, some on an adjunct" → Topology C (§4.4).**
   This is the most pragmatic multi-P4 option and the only one that
   targets the *measured* limit (the worker, not ingest). Start here if
   the goal is decode depth on a band one P4 can't fully drain. It needs
   no HydraSDR and no channelizer — the adjunct is the current worker
   pipeline behind an SPI-slave burst intake; the ingest P4 is current
   firmware plus an overflow branch on the burst queue (+ optional
   SNR-priority QoS).

2. **For wider coverage cheaply → Topology B** ([`multi-receiver-spi-aggregator-design.md`](./multi-receiver-spi-aggregator-design.md)):
   N RTL-SDR-per-P4 workers fanning decoded-frame PDUs into an
   aggregator. Compose it with C (B widens, C deepens each band).

3. **Reserve the HydraSDR (Topology A) for coherence-driven goals** —
   seamless coverage, phase-coherent wideband, single RF chain. It is
   not a throughput win on its own and it fights the P4's interconnect.
   If pursued, gate it on v3.1 silicon for the ingest/channelizer node
   (400 MHz + USB-DMA-to-PSRAM), accept a dedicated channelizer node, and
   budget a **faster-than-GPSPI** IQ-distribution fabric — one ~2.5 MHz
   subband per high-speed link.

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

### Topology C (try first — it's the closest to shippable)

1. **SPI-slave burst intake on the adjunct** — the adjunct must accept an
   8 KB burst PDU over SPI and feed it into the existing `burst_pipeline`.
   Which core hosts the SPI-slave task (multi-receiver doc open risk #1
   applies)? On the adjunct Core 1 is the worker; the SPI-slave intake
   likely lives on Core 0. Needs a budget check.
2. **Overflow decision + SNR-priority on the ingest P4** — where on the
   burst-queue path does the keep-local-vs-ship choice go, and does the
   SNR-priority variant cost enough Core 0 to matter? Cheap on paper;
   confirm.
3. **Burst-PDU shape and worst case** — fix the wire format (descriptor +
   decimated 250 ksps window). Typical ~8 KB; cap or fragment the rare
   multi-frame burst (up to ~250 KB) so it can't stall the SPI link.
4. **Round-trip latency budget** — burst out + decode + frame back adds
   an SPI hop to the ~60–200 ms worker latency. Fine for ACARS;
   measure it.

### Topologies A / B

5. **Real measured Core 0 processing ceiling** — the ~2.5–3 MSPS wall is
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
