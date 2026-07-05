# Wideband front-end design sketch — exploiting a wide SDR (e.g. HydraSDR)

Status: **design sketch, not a plan of record.** Written 2026-07-05 at the tail
of the memory-placement/PSRAM investigation. Its main job is to (a) capture what
the new "chunked PIE on PSRAM works" knowledge actually unlocks, and (b) be
brutally honest about what the P4 can and cannot do, so nobody starts building
the infeasible version.

## 1. Motivation

Today the receiver processes **one 2.56 MHz sub-band** of the Iridium downlink:
RTL-SDR at 2.56 MSPS → resample to 2.5 MSPS → one FFT burst tagger → one worker.
That covers ~2.5 MHz / 41.667 kHz ≈ **~60 of the ~240 Iridium FDMA channels**.

The Iridium downlink spans **~1616.0–1626.5 MHz (~10.5 MHz)**. A wide SDR
(HydraSDR/Airspy-class, up to ~10 MSPS) could in principle capture the **whole
band at once** — ~4× the channels, ~4× the burst opportunities — which is the
real reason to want a better radio's *bandwidth* (its better *SNR* is a separate,
simpler win — see §3 Tier 0).

## 2. The hard reality: this is compute-bound, not bandwidth-bound

Before any architecture, the wall: **the P4 cannot decode the full 10 MHz band
in real time.** Measured on the live bench, at **one** 2.56 MHz sub-band:

- Core 0 (ingest + FFT tagger): **dsp_cap ≈ 77%**
- Core 1 (worker: channelize/rotate/RRC/UW-correlate/demod/BCH): **frequently
  >100%, peaks ~494%**, already dropping bursts during storms (`worker_dropped`)

Both the tagger and the worker scale ~linearly with the sample rate / channel
count. 4× the band ⇒ ~4× that load, on the same **2 cores at a 360 MHz cap**
(MSPI-751 errata forbids going faster — see `project_p4_errata_status`). There is
no version of "decode all 240 channels in real time on this P4." Full-band
real-time is **out of scope permanently**, not just "for now."

So the design is about the *feasible slice*, and about where a wide SDR helps
without pretending the compute exists.

## 3. What is actually feasible — three tiers

### Tier 0 — HydraSDR at the SAME 2.56 MSPS, for SNR (no front-end change)
The current bench bottleneck for *live decode* is **antenna/SNR**, not bandwidth
(proven this session: decode chain verified at 95% on 20 dB fixture, 0 live
decodes at 12–13 dB). A HydraSDR's better front-end/LNA at the *existing*
narrowband rate is the **highest-value, lowest-effort** "use a better radio" move
— it needs **zero** new DSP, just the driver. This is the pragmatic decode win
and should be tried first. Everything below is only worth it once SNR is solved.

### Tier 1 — modest wideband (~2 sub-bands, ~5 MSPS), real-time
Roughly 2× the current load. Core 0 tagger 77%→~150% and Core 1 worker already
saturated ⇒ **does not fit today**. It becomes *conceivable* only after reclaiming
CPU headroom:
- T50 (PIE-vectorise the tagger window multiply — banked) buys some Core 0.
- Moving the worker's channel-select/rotate to run over PSRAM in chunks (now
  known-safe, see §5) avoids internal-copy staging but doesn't cut the core math.
- Realistically needs the worker parallelised or a second worker — and Core 1 has
  no headroom, so this implies restructuring, not tuning.
Verdict: a stretch goal, gated on real CPU wins, not a near-term target.

### Tier 2 — wideband **capture**, offline decode (the realistic wideband win)
Don't decode the wide band on-device at all. **Capture** the raw wide IQ to SD (or
stream to a host) at the SDR's native rate, and decode the full band **offline**
on a real machine (the host `iridium-extractor`/gr-iridium chain, which the whole
DSP port is already validated against). The P4's job shrinks to a high-rate
DMA→SD (or DMA→USB/net) pump — which is bandwidth-bound, not compute-bound, and
is exactly what T48/T49a were building toward. This is the version that genuinely
exploits a HydraSDR's bandwidth on this hardware.

## 4. Channelizer architecture (for Tier 1, and the on-device part of any real-time wideband)

The standard structure to split a wide band into N narrow sub-bands efficiently is
a **polyphase FFT filterbank** (weighted overlap-add): one prototype low-pass FIR
run polyphase, feeding an N-point FFT whose bins ARE the decimated sub-band
outputs. Cost is ~one FIR + one small FFT per input block for ALL N channels —
far cheaper than N independent mixers+decimators.

Per sub-band the existing pipeline is reused unchanged: each channelizer output is
a **2.5 MSPS** stream (the gr-iridium grid — see `resample_256_to_250`'s rationale)
handed to a tagger+worker instance. So the wideband front-end is purely additive:
`wide IQ → polyphase channelizer → N × (existing 2.5 MSPS tagger/worker)`.

Relationship to existing design notes: this is the D7/D8 "channelizer" concept
referenced in `project_d7_d8_coupling` — the 40 kHz channel quantisation and the
need for D8 fine-frequency estimation per sub-band apply here directly.

## 5. Why the PSRAM-PIE knowledge is the enabler

The blocker that would have made this look impossible: internal SRAM is
silicon-capped and already full (USB pool must be internal — APM-560/MSPI errata
forbid USB-OTG DMA to PSRAM; tagger + PIE FFT scratch consume the rest). A
channelizer's wide input buffer and per-sub-band scratch **cannot** all be
internal.

This session proved (rotate_to_dc A/B, 2026-07-05) that **chunked PIE vector ops
run correctly on PSRAM-resident data** — the "PIE mis-services PSRAM" rule is only
about *large sustained* in-place transfers (multi-KB FFT/FIR scratch), not the
small per-chunk `vld/vst` a channelizer's polyphase inner loop does. So the
channelizer can keep its wide IQ and sub-band buffers in **PSRAM** and PIE-process
them in chunks — which is what makes a wideband front-end architecturally viable
on the tight internal budget at all. (See `project_heap_position_decode_bug`
2026-07-05 update for the exact rule.)

Constraints that still bind (from `project_p4_errata_status`):
- Any **DMA** touching PSRAM must be 4-byte-aligned base+length (MSPI-750).
- No **second AHB master** into PSRAM concurrently (APM-560) — so the SDR ingest
  DMA and the AXI-GDMA (signal_buffer) sharing must stay as-is; adding a new PSRAM
  DMA master needs an APM audit.
- 360 MHz CPU cap stays (MSPI-751) — no clocking out of the compute wall.

## 6. What we throw away / defer (explicit scope cuts)

- **Full-band (10 MHz / ~240-channel) real-time decode** — permanently infeasible
  on this P4 (§2). Not "later"; never, on this silicon.
- **On-device Tier 1 real-time wideband** — deferred until there's a real Core-1
  CPU win (T50 + worker parallelisation). Not a near-term target.
- **HydraSDR driver / librtlsdr-equivalent** — out of scope of this sketch; the
  radio-bring-up is its own task (and Tier 0 needs only this, nothing here).
- **Simplex / higher-rate frame handling** at the wide edges — deferred.
- **Any implementation** — this is a sketch. No code, no fixtures (we have no
  wideband test vector and no HydraSDR on the bench).
- **The T49b tile fusion** — parked (`wip/t49b-tile-fuse`); orthogonal to this.

## 7. Recommendation

1. **If the goal is more live decodes:** Tier 0 (HydraSDR at 2.56 MSPS for SNR) —
   no wideband, no new DSP, targets the actual bottleneck.
2. **If the goal is genuinely capturing the whole band:** Tier 2 (wideband capture
   → offline decode) — the only version that both fits the P4 and exploits the
   bandwidth. Build the high-rate capture pump (T48/T49a groundwork already helps),
   decode on host.
3. **On-device wideband decode (Tier 1)** is a research stretch, not an engineering
   task, until Core-1 has headroom.

The one-line takeaway: **the PSRAM-PIE result removes the *memory-architecture*
objection to wideband, but the P4's *compute* budget remains the wall — so the
realistic wideband play is capture-and-offline-decode, not on-device full-band.**
