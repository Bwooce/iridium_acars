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

### 4.1 The SPI interconnect — fast enough with quad/octal, but it's not the real wall

The multi-receiver doc cited **~40 MHz reliable ≈ 5 MB/s per CS**. That is
a conservative **single-line** figure and it is the wrong number to plan
the wideband case around. GPSPI2/3 on the P4 support 1/2/4/8-bit modes:

| SPI mode @ 80 MHz | Raw | Sustained (after DMA/txn overhead) |
|---|---|---|
| Single-line | 10 MB/s | ~7–9 MB/s |
| Quad (4-line) | 40 MB/s | ~25–35 MB/s |
| Octal (8-line) | 80 MB/s | ~50–60 MB/s |

(Caveats: slave-side clock limits, signal integrity at 80 MHz over real
wiring, and DMA pipelining — bench-validate before relying on the top of
the range.) And SPI is a **shared bus with per-slave CS**, so one octal
bus carries its aggregate (~50–60 MB/s) across several workers — you do
**not** need one peripheral per worker. So:

- **Topology A fan-out is bandwidth-feasible after all.** Four ~2.5 MHz
  subbands = ~20 MB/s of channelized distribution fits on a single octal
  bus (4 CS lines), or you can broadcast the whole ~20 MB/s wideband
  stream to all workers on one bus and let each pick its slice. The "2–3
  SPI peripherals can't drive 4 links" objection was wrong — it assumed
  single-line point-to-point links, not a shared octal bus.

- **Topology B (fan-in frames)** is, as before, trivial — 280 B PDUs, the
  heavy IQ never crosses the wire.

**So the link is not the binding constraint.** Correcting that moves the
wall to where it actually is — the *compute* (§4.2, §4.5): someone still
has to channelize/detect 10 MHz of samples, and faster SPI does nothing
for that.

### 4.2 The real wall: channelization/detection CPU, not the link

With fast SPI (§4.1) the data *can* move; the question is who does the
per-sample DSP to turn 10 MHz of samples into per-channel bursts. There
are two shapes, and each has a compute cost the link speed doesn't touch:

- **Centralized channelize + fan-out.** One node runs a polyphase/FFT
  channelizer, splitting all ~4 subbands in one pass (~1.2× one tagger —
  efficient), and fans the subbands out over an octal bus. But that node
  is *also* the USB ingest node, and on v1 Core 0 is already ~60% busy
  just ingesting (and can't ingest 20 MB/s at all — §2). It needs
  **v3.1** (400 MHz + USB-DMA-to-PSRAM frees the ingest cost) to have any
  hope of ingest + channelize on one chip, or a **dedicated channelizer
  node** between the SDR host and the workers.
- **Broadcast + per-worker DDC.** Skip central channelization: broadcast
  the full ~20 MB/s wideband stream to all workers on one octal bus; each
  worker digitally down-converts and decimates *its own* ~2.5 MHz slice,
  then runs its normal tagger + worker. The catch: every worker now pays
  a **DDC over the full 10 MSPS input** (≈ tagger-scale work, done ×4
  redundantly across the cluster) *plus* SPI-ingesting 20 MB/s — roughly
  **2× a normal worker P4's load**. Whether that fits is a **v3.1 per-chip
  budget question**, not a link question.

Either way the binding constraint is **compute**: the full-band
channelize/detect is ~4–5× one P4's tagger budget (§4.5), and the only
way to distribute it without one chip running all of it is the broadcast
shape — which trades the central bottleneck for redundant per-worker DDC.
Faster SPI is what makes that trade *possible*; it doesn't make it *free*.

### 4.3 When is Topology A (HydraSDR) actually worth it?

On pure "more channels" grounds, **Topology B wins**: N cheap RTL-SDRs
fanning 280 B frames in is cheaper and avoids paying the full-band
channelize/detect cost entirely — each chip only ever touches its own
slice. One HydraSDR has to channelize/detect the whole band somewhere
(§4.2), even though octal SPI can carry the data. The HydraSDR only earns
that extra cost when its **single coherent wideband capture** matters:

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
  adjunct's queue drains (~20 bursts/sec), so each adjunct link carries
  ~20 × 8 KB = **~160 KB/s** — a rounding error on any SPI mode, and
  *self-throttling* (no risk of flooding the link). It needs no wideband
  channelization and no continuous high-rate stream — the cheapest
  inter-chip shape of all three topologies.
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
It is gentler on the link than Topology A (bursty, self-throttled
160 KB/s/adjunct vs continuous wideband), needs **no channelizer** and
**no second SDR**, attacks the
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

So the binding constraint is **wideband detection compute** (not the
link — §4.1 fixed that). Detection needs every sample, and the full-band
channelize/detect is ~4–5× one P4's tagger budget. With fast SPI there
are now two ways to distribute it (§4.2), each gated on compute:

1. **Centralized channelize + octal-bus fan-out.** Needs **v3.1** (one
   chip can't ingest 20 MB/s *and* channelize on v1). Plausible on v3.1
   if ingest+channelize fits one chip — a bench question.
2. **Broadcast the wideband stream + per-worker DDC.** The octal bus can
   carry it; each worker DDCs its own slice. Costs each worker ~2× a
   normal load (full-rate DDC + SPI-ingest + tagger + worker) — also a
   v3.1 per-chip budget question.

So this is **no longer a flat "no."** It's "plausible on v3.1, gated on
whether the channelization compute fits per-chip" — bench-measurable, not
ruled out. What stays true: **on v1 it doesn't close** (can't even ingest
20 MB/s; no Core 0 cycles to channelize), and **even on v3.1 someone pays
the ~4–5× full-band detection cost** — you're trading a central
bottleneck for redundant per-worker DDC, and the cluster has to be sized
for it.

(Note also that one HydraSDR at its 10 MSPS max yields ~9 MHz usable —
most of the 10.5 MHz Iridium allocation, not quite all of it. A second
capture or a slightly narrower goal closes that.)

**Three ways to "process all channels in real-time," cheapest-first:**

- **Split the SDR, not the stream (Topology B — still the simplest).** N
  RTL-SDRs, each P4 detects *and* decodes its own ~2.5 MHz slice; **no
  wideband data is ever redistributed** and no chip pays the full-band
  detection cost. Cheapest and lowest-risk; the price is losing the
  HydraSDR's coherent single capture.
- **HydraSDR + v3.1 broadcast cluster (now on the table thanks to octal
  SPI).** One HydraSDR → v3.1 ingest node broadcasts the wideband stream
  over an octal bus → v3.1 workers each DDC + detect + decode their slice.
  Keeps coherence; costs ~2× per-worker load and v3.1 hardware; needs the
  `libhydrasdr` port (§3). The viability hinges on the per-chip DDC budget
  — the open question to settle on the bench.
- **Or a bigger wideband host.** If even the v3.1 cluster doesn't budget
  out, a Linux SBC (Pi-class) runs the channelizer/tagger and fans
  *detected bursts* to P4 decode workers (Topology-C style) over Ethernet.
  P4s stay useful as cheap decode nodes; the SBC is the wideband brain.

**Bottom line for the HydraSDR-everything goal:** the frame handoff is
the easy half. Faster SPI removes the link objection, so the remaining
question is purely **compute** — can a v3.1 cluster absorb the ~4–5×
full-band detection cost (centralized on one chip, or as redundant
per-worker DDC off a broadcast bus)? On v1, no. On v3.1, plausibly yes
but the cluster must be sized for it and it needs the HydraSDR port —
worth a bench prototype rather than a dismissal.

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

## 6. Interconnect options (Topology A's broadcast bus)

§4.1 corrects the earlier "~5 MB/s per link" figure: **octal GPSPI at
80 MHz sustains ~50–60 MB/s on one shared bus**, which carries the
~20 MB/s wideband broadcast (or ~4 channelized subbands) with headroom.
So the interconnect is **adequate**, not the wall. Options, best-first:

- **Octal GPSPI broadcast bus** (recommended for Topology A): one MOSI
  octal bus, all workers clocked in parallel, each reads the full stream
  and picks its slice. One shared bus, not N point-to-point links — the
  "2–3 free peripherals" count is irrelevant for a broadcast.
- **SDIO** (the link already used for the C6, ~50 MB/s) — fine for a
  point-to-point hop but it's host↔single-slave, not a fan-out bus.
- **Parallel / I80 LCD-cam** interfaces move tens of MB/s but are awkward
  as a multi-drop fabric.

The earlier claim that "no clean N-way fan-out exists" was wrong — an
octal broadcast bus *is* one. The structural argument now is narrower and
**compute-side**: the bus can move the samples, but someone still pays
the ~4–5× full-band channelize/detect (§4.2). Topology B remains the
architecture that *avoids* paying it at all (each chip only ever touches
its own slice); Topology A *pays* it (centralized or as redundant
per-worker DDC) in exchange for coherence.

## 7. Recommendation

The three topologies answer different goals — they are not competing for
the same job:

| Goal | Topology | Why |
|---|---|---|
| Decode **more of one band** (beat `worker_dropped`) | **C — local + adjunct offload** | Attacks the measured worker bottleneck; bursty 160 KB/s/adjunct SPI; one SDR; reuses current firmware |
| **All channels at once**, real-time, **cheapest** | **B — RTL-per-worker, fan frames in** | No firehose to redistribute; no chip pays full-band detection; lowest risk (§4.5). Price: no coherent capture |
| **All channels** *coherently*, real-time | **A — HydraSDR + v3.1 broadcast cluster** | Octal SPI carries the broadcast; v3.1 workers DDC their own slice. Viable iff the ~2× per-chip DDC budget fits — bench it (§4.2, §4.5). Needs `libhydrasdr` port |
| Coherent wideband if the v3.1 cluster won't budget out | **Bigger wideband host (Linux SBC) + P4 decode workers** | SBC runs channelizer/tagger, fans detected bursts to P4s; P4 is the worker, not the brain |

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

3. **The HydraSDR (Topology A) is for coherence-driven goals** — seamless
   coverage, phase-coherent wideband, single RF chain. With octal SPI the
   interconnect is *adequate* (§4.1/§6), so it's no longer ruled out — but
   it's gated on **v3.1 silicon** and on the **per-chip channelize/DDC
   budget** fitting (§4.2/§4.5), plus the `libhydrasdr` port. Bench the
   per-chip DDC cost before committing — that's the make-or-break number,
   not the link.

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
