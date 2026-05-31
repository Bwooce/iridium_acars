# Multi-receiver SPI-aggregator design

**Status:** design doc, not implemented. Drafted 2026-05-31 in response
to design review #119 (the reviewer flagged that the existing
implementation plan covers C6↔P4 SDIO but no doc covered a future
N-P4 fan-in architecture, even though `iridium-acars-implementation-plan.md`
documents two single-P4 parallelism efforts as **BLOCKED** by
SRAM/L2 contention — see "BLOCKED: Two-worker burst pool" and
"BLOCKED: Two-worker parallel resample" sections in the implementation
plan).

This doc bounds the design space and identifies the gating refactor
(#120) so a future scaling effort isn't started on shifting sand.
Nothing here is a commitment to build — it's a sketch detailed enough
to refute or confirm whether the single-P4 architecture can be
extended at all.

## The problem

A single ESP32-P4-NANO running our current 12-fix firmware sustains
one 2.56 MHz subband of Iridium L-band reception (1616.0–1626.0 MHz
duplex). Real-world deployment goals will eventually want:

- **Wider band coverage** — pulling 4× or 8× the spectrum so we catch
  whichever channel a real burst lands on
- **Higher decode rate** — driven by either better antennas (more bursts
  per second) or wider coverage (same)

Per the implementation plan, the single P4 can't be parallelised
further. The two-worker burst-pool and parallel-resample efforts are
both documented as BLOCKED:

| Attempted split | Status | Why it broke |
|---|---|---|
| Two-worker burst pool | BLOCKED | Each worker instance needs ~34 KB internal SRAM for PIE buffers (UW correlator FFT scratch, RRC tap tables, decim state). Largest contiguous DMA-INT free block after init is ~31 KB. Per memory `project_heap_position_decode_bug`, PIE buffers in mid-RAM corrupt silently. |
| Two-worker parallel resample | BLOCKED | Validated bit-exact in isolation, then collapsed Core 0's FFT throughput from 263 → 443 µs/step via L2 cache contention. Can't shrink L2 to recover heap without 13% USB throughput regression. |

So scaling requires **more P4 chips**, fanned in to an aggregator. The
question is: what crosses the inter-chip boundary, and where?

## The boundary options

The pipeline has six natural split points:

```
USB ingest ─▶ resample 2.56→2.5 MSPS ─▶ tagger ─▶ per-burst worker ─▶ frame_decoder ─▶ acars_push
   (1)             (2)                    (3)         (4)                 (5)              (6)
```

Each split point trades off:
- **Bandwidth across the wire**: lower-rate output = easier link
- **State to ship**: small struct vs raw IQ buffer
- **CPU saved on aggregator vs distributed**: shifts the work, doesn't
  eliminate it
- **Recovery semantics**: how each P4 handles aggregator failure

### Option B-1: Split at (1) — ship raw 2.56 MSPS IQ

Each worker P4 pulls its USB stream and forwards uint8 IQ to the
aggregator over SPI. Aggregator runs ALL DSP for ALL receivers.

- **Per-link bandwidth**: 2.56 MSPS × 2 B/sample = 5.12 MB/s = 40 Mbps per
  worker. P4 SPI master is ~40 MHz max for reliable PSRAM access — 1
  worker per aggregator and you're already saturated.
- **Aggregator load**: N receivers × full DSP load. At N=4 that's a
  4-core machine; well beyond a single P4. Defeats the point.

Discarded: the boundary is too rich for an aggregator to keep up.

### Option B-2: Split at (3) — ship tagged bursts (post-resample, pre-worker)

Each worker P4 runs ingest + resample + tagger. Aggregator receives
per-burst PDUs containing the windowed IQ slab + tagger metadata
(start_sample, center_bin, peak_snr, magnitude_db), runs worker
(matched filter + UW + QPSK + BCH + classify) + frame_decoder for
all receivers.

- **Per-link bandwidth**: bursts are 8 ms at 250 ksps × 4 B = 8 KB each.
  At bench-RF burst-rate (~140/sec false-positives + ~10/sec real),
  ~150 bursts × 8 KB = 1.2 MB/s = 9.6 Mbps per worker. Comfortable
  on SPI. Real-RF will be higher but still well under saturation.
- **Aggregator load**: per-burst worker is ~50 ms wall time on P4 at
  current PIE-int16 path (see `worker_core1.c` per-burst timing).
  Aggregator running 4 workers' bursts at 150/s × 4 = 600/s × 50 ms =
  30 s of work per second — **doesn't fit**. Would need a worker pool
  on the aggregator, which is the same blocker as on a single P4.
- **Better aggregator placement: per-receiver worker on the worker P4
  itself, ship only DECODED FRAMES (post-BCH) to the aggregator.**

Discarded in this form, but motivates Option B-3.

### Option B-3 (RECOMMENDED): Split at (5) — ship decoded frames (post-BCH, pre-frame_decoder)

Each worker P4 runs **ingest + resample + tagger + worker + BCH**.
That's the entire numerically-heavy DSP. It ships ONLY successfully-
demodulated (UW-locked, BCH-decoded) frames to the aggregator as
small fixed-shape PDUs:

```c
struct iridium_frame_pdu {
    uint64_t timestamp_us;           // worker_emit_frame's burst start
    uint32_t source_p4_id;           // receiver identifier
    int32_t  rel_freq_hz;            // burst center vs receiver LO
    int16_t  peak_bin;
    float    peak_snr_db;
    uint8_t  direction;              // 0 = DL, 1 = UL
    uint8_t  bch_e1;                 // 0..2 corrected, or 0xFF if Chase-2 rescued
    uint8_t  bch_e2;
    uint8_t  reserved;
    uint16_t n_bits;
    uint8_t  bits[256];              // padded to a clean size; actual length in n_bits
};                                    // total ~280 B per PDU
```

- **Per-link bandwidth**: at bench RF the BCH-passing rate is ~7/h on a
  bad antenna; at real RF expect 60-600/h depending on antenna quality.
  Even at 1/s × 280 B = 280 B/s = 2.2 Kbps per worker — utterly trivial.
  Even a 100×-real-RF stress scenario is <1 Mbps.
- **Aggregator load**: the aggregator runs frame_decoder
  (iridium_frame_classify + ims_decode + ida_lcw_decode + sbd_reassembler
  + libacars). All of those are pure-CPU and small. Per-frame cost is
  ms-scale; even at 100 frames/sec the aggregator runs at <1% CPU.
- **Aggregator can be ANY P4** (no SDR attached), or even a more
  capable SoC. It owns: msg_ring, acars_push, sd_log, http_server.

This is the **right shape**. The boundary is small (1 PDU = 280 B),
the bandwidth is trivial, the aggregator cost is bounded, and the
distributed cost matches what one P4 already does.

## Physical layer

ESP32-P4 has both AXI and AHB SPI controllers. For aggregator-side
flexibility:

- **Aggregator: SPI master**, polled or interrupt-driven on each
  worker's CS line. One CS per worker (P4 has 4 SPI peripherals if
  bypassing the C6 wireless co-proc; in practice 2–3 are usually free).
- **Worker: SPI slave**, queues PDUs into a small ring buffer and
  asserts a per-worker INT line (or uses SPI flow-control) when the
  ring has data.
- **Wire rate**: 10 MHz SPI is conservative and safe; gives ~1.25 MB/s
  per CS. Plenty for 280 B PDUs.
- **Topology**: star (each worker has its own CS+INT pair to the
  aggregator). No bus arbitration needed.

Cabling on the bench would be jumper wires per receiver. Production
deployment could use a small backplane PCB. None of that is in scope
for this doc.

## Worker P4 changes from current single-P4 firmware

After #120 lands (refactor singletons → explicit context), the
worker firmware becomes a self-contained "ingest → demod → frame
PDU" pipeline. Required additions:

1. **SPI-slave PDU transmitter** — short FreeRTOS task pinned to Core 0.
   - Pulls completed frames from a per-worker output queue (PSRAM,
     size ~32 PDUs ≈ 9 KB).
   - When SPI master clocks in a frame-fetch command, writes the next
     PDU + advances the ring.
   - Asserts INT line when ring transitions non-empty.
2. **Replace local frame_decoder call** at `worker_core1.c:492` with a
   call to the SPI-output queue's `enqueue(pdu)` function.
   - The worker still does iridium_frame_classify (#111) to decide
     whether the frame is "real" vs UNKNOWN — only known-type frames
     get PDU'd. This preserves bandwidth savings against BCH false-
     positives.
3. **No SD logging on worker P4.** SD is the aggregator's job (only
   real frames land on SD, and only the aggregator sees the deduplicated
   stream after reassembly).
4. **No httpd on worker P4 in normal operation.** Optional debug
   httpd left compiled but bound to LAN only — for the worker's local
   /diag/dsp_health / /diag/histograms endpoints to remain useful for
   debugging. Aggregator's httpd is the operator-facing one.

Storage: the worker's PSRAM frees up significantly with no
frame_decoder, libacars, sd_capture, or acars_push. PSRAM is ~28 MB
available after pipeline allocations on the current single-P4 design;
removing those frees ~5 MB.

## Aggregator P4 changes

The aggregator is much closer to the current single-P4 firmware MINUS
the front-end DSP. Required:

1. **SPI-master poll loop** per worker CS, ~100 Hz default. Reads PDUs
   into a per-worker input queue.
2. **Frame ingest task** — pulls PDUs from each worker's input queue,
   calls `iridium_frame_classify(pdu.bits, pdu.n_bits, pdu.direction,
   &classified)` followed by the same MS/TL/BC/LW/RA dispatch that
   `frame_decoder.c::process_one` does today.
3. **Cross-receiver deduplication** — the same real Iridium burst can
   land in two receivers' bands if their LOs overlap (likely if we
   spread them to cover the 10 MHz duplex band with ~5 MHz subband
   margins). De-dupe by `(timestamp_us ± 5 ms, peak_bin window,
   bits Hamming distance < 4)`. Likely a 100-line addition.
4. **All current top-level firmware functions** — msg_ring, acars_push,
   sd_log, http_server, /diag endpoints, /status. Mostly unchanged
   except that decode source is N input queues instead of one.
5. **wifi_link + health_wdt** — same.
6. **Receiver health surfaced via /status** — per-worker SPI link
   liveness, last-seen-PDU timestamp, PDUs/sec.

## Latency

End-to-end "burst arrives at antenna → frame ready in msg_ring":
- Worker side (unchanged from current single-P4): tagger detection
  + 50 ms worker processing ≈ 60-200 ms.
- SPI hop: 280 B / 1.25 MB/s ≈ 224 µs per PDU. Polling latency
  adds up to 10 ms at 100 Hz poll. Total: 1-10 ms.
- Aggregator classify + libacars: 1-5 ms.

Total: 70-220 ms, dominated by the worker. Acceptable for ACARS
(which is non-realtime); for any future low-latency overlay (e.g.
voice) this would need to change but is not in scope here.

## What this doc decides

1. **Boundary**: at decoded-frame PDU level (Option B-3). Not raw IQ
   (B-1) and not tagged bursts (B-2). The recommended PDU layout is
   sketched above (~280 B).
2. **Aggregator runs**: classifier + libacars + msg_ring + acars_push +
   sd_log + http_server + dedupe. NOT DSP.
3. **Worker runs**: ingest + resample + tagger + per-burst worker + BCH
   + (#111 classify-or-drop). Ships PDUs only when classify returns a
   known type.
4. **Gating refactor**: #120 (uw_correlator / direct_if_decim /
   dsp_processor → explicit context) is required first. Until that
   lands, even a single-P4 worker can't be lifted into a new top-level
   binary cleanly.
5. **Not in scope here**: PCB design, mechanical, cable harnesses,
   power distribution. Those follow once the firmware proof-of-concept
   shows the SPI link carries traffic reliably.

## Open risks worth surfacing before any code

1. **Worker P4 SPI slave throughput under stress.** A real-RF flood
   (100 PDUs/s) is well under SPI capacity, but the worker's Core 0
   is already crowded by class_driver consumer + ingest dispatch +
   httpd. Adding an SPI-slave ISR may need to live on Core 1 —
   conflict with worker's prio-4 occupancy (memory
   `project_core1_cpu_budget_scheduling`). Need budget audit before
   we know which core hosts the SPI slave task.
2. **Frame deduplication accuracy.** The proposed `(timestamp_us ±
   5 ms, peak_bin window, Hamming < 4)` triple needs validation on a
   real dual-receiver capture. Until two physical setups exist, this
   is unprovable.
3. **SD card singular-aggregator.** If the aggregator dies, ALL
   workers' decodes are lost (workers don't have local fallback). A
   "spool to flash, replay on aggregator reconnect" feature might be
   worth adding to the worker — adds ~1 MB flash spool, ~50 LOC.
4. **OTA update**: each worker needs its own OTA. Currently the
   single-P4 firmware doesn't differentiate worker/aggregator builds.
   That's a build-system addition (two Kconfig profiles) before any
   field deployment.

## Next steps if this proceeds

1. Land #120 (refactor singletons to explicit context). Mandatory.
2. Build a minimum-viable two-P4 bench setup using existing P4-NANO
   boards + SPI jumpers. Worker firmware = current minus
   frame_decoder/SD/httpd; aggregator firmware = current minus
   ingest/resample/tagger/worker/BCH.
3. Validate frame PDU rate and end-to-end decode-to-msg_ring latency
   with the existing CORPUS smoke fixture forced through each.
4. Production firmware: split build profiles (`p4-worker`,
   `p4-aggregator`) via Kconfig. Single source tree, two binaries.
5. PCB / cabling — out of scope here, follows from step 3-4.
