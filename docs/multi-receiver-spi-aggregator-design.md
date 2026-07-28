# Multi-unit receiver architecture — one main aggregator + N child receiver boards

**Status:** design doc, not implemented (the Part II SPI split has a
partial implementation — see `multi-receiver-implementation-plan.md`,
phases 1/2/4/5 done, phase 3 SPI transport DRAFT).

**Retitle / supersession note (2026-07-28):** this doc was originally
"Multi-receiver SPI-aggregator design" (#119, drafted 2026-05-31) and
covered ONE scenario: N Iridium worker P4s fanning decoded-frame PDUs
into an aggregator over SPI. The scope is now generalized to a
**multi-unit, multi-BAND receiver architecture**: one MAIN board plus N
physically-separate CHILD boards that are heterogeneous across bands
and frequencies (several Iridium children at different LO parks for
wider L-band coverage, plus VDL2 / POA / future Inmarsat-Aero
children). The original SPI/Iridium content is retained verbatim as
**Part II** — it remains the correct design for the *co-located*
special case (child SoCs on one PCB sharing a wideband front end, per
`hydrasdr-wideband-spi-feasibility-2026-06-10.md`), and its boundary
analysis (Option B-3: ship small decoded-frame PDUs, not IQ) is the
foundation Part I builds on. Where Part I contradicts Part II
(band-less PDU, one-band-per-boot assumptions, SPI-only transport),
**Part I wins**. The filename is kept for link stability.

Original #119 context: the single-P4 parallelism efforts are **BLOCKED**
by SRAM/L2 contention ("BLOCKED: Two-worker burst pool" and "BLOCKED:
Two-worker parallel resample" in `iridium-acars-implementation-plan.md`),
so scaling means more P4 boards. Nothing here is a commitment to build.

---

# Part I — Generalized architecture (2026-07-28): main + heterogeneous child boards

## I.1 Why the generalization is nearly free now

Two things landed since the 2026-05-31 draft that change the shape of
the problem:

1. **The band-mode overhaul made bands first-class.** A receiver's
   entire personality is now data: `app_config` NVS key `band`
   (`band_id_t`, `common/band_pipeline/band_profile.h`) + `lo_hz`, with
   per-band tunables in band-namespaced NVS keys (gain, bias-tee,
   tag_thr — `app_config.c` phase-2 namespacing), resolved once at boot
   by `band_runtime_resolve()` (`band_select.c`) into
   `{band, band_profile_t, band_pipeline_t}`. `worker_core1.c` and
   `frame_decoder.c` dispatch off that bundle; `band_decode_stats_get()`
   gives band-agnostic decode counters. **A child board is therefore
   just the existing single-band firmware with `band=X, lo=Y` in NVS.**
   No child-specific firmware exists or is needed.
2. **The distributed PDU path half-exists.** `frame_pdu.{c,h}`
   (pack/unpack + PSRAM output queue), `frame_link.c` (framed
   `[magic][ver][flags][PDU][crc16]` wire format, host-tested),
   `aggregator_ingest.c` (queue → `frame_decoder_push`, per-source
   liveness table), and the WORKER/COMBINED role gate in
   `worker_core1.c` (~L988) already implement the child→main hop —
   currently over an in-process queue (COMBINED_LOOPBACK, smoke-passing)
   or the draft SPI transport (HW-pending). The generalization mostly
   swaps the transport and widens the PDU.

Plus the supporting pieces: SNTP wall-clock (`net_time.c`), the
airframes.io feeder with its reusable `push_target_t` UDP delivery
struct (`acars_push.c`), and per-band `/status` decode funnels.

## I.2 Topology and roles

```
                      ┌────────────────────── MAIN (aggregator) ─────────────────────┐
                      │ ingest (net + optional local RX) → per-band frame_decoder    │
                      │ → dedupe → msg_ring / sd_log / http /status+/messages rollup │
                      │ → ONE upstream feed (airframes.io via acars_push)            │
                      └──────▲──────────▲──────────▲──────────▲──────────▲───────────┘
                        IP   │          │          │          │          │  IP (Ethernet or WiFi)
              ┌──────────────┴─┐ ┌──────┴───────┐ ┌┴─────────┐ ┌────────┴─┐ ┌──────────────┐
              │ child: iridium │ │ child:iridium│ │child:vdl2│ │child:poa │ │child:inmarsat│
              │ lo=1618.75 MHz │ │ lo=1621.25   │ │136.8125  │ │131.550   │ │~1545 (future)│
              └────────────────┘ └──────────────┘ └──────────┘ └──────────┘ └──────────────┘
              each child = P4-NANO + RTL-SDR + its own antenna, running the SAME firmware,
              pinned to one band/LO by NVS
```

- **Child**: a complete standalone unit (P4-NANO + RTL-SDR v4 + the
  band's antenna) running today's firmware with `band` + `lo_hz` set. It
  runs its band's FULL pipeline — ingest → tagger → band_pipeline demod
  → (band L2 or not, see §I.4) — and ships results to the main over IP.
  It keeps its own httpd for debug/commissioning, but does NOT feed
  airframes and does not need SD.
- **Main**: owns aggregation, cross-child dedupe, the unified
  `/status`+`/messages` view, SD archive, and the SINGLE upstream
  airframes.io feed (one station identity for the whole cluster). The
  main can be a P4-NANO with no SDR at all — per Part II the
  aggregation work is ms-scale per frame — or a "combined" unit that
  also receives (the COMBINED role generalizes to this).
- **Configuration of a child** uses only existing mechanisms: serial
  `set band vdl2` / `set lo 136812500` (serial_cmd NVS interface), or
  `POST /config`, or pre-provisioned NVS. The per-band NVS namespacing
  means a child re-purposed from Iridium to VDL2 keeps sane per-band
  gain/bias/threshold defaults (the "gain footgun" fix).

**Homogeneous fleet (the "wider Iridium" goal):** Iridium spans
1616.0–1626.5 MHz ≈ 10.5 MHz; one RTL child hears LO ±1.25 MHz
(2.5 MSPS Path A). A fixed park at 1620.6 MHz covers ~58.6% of observed
traffic (freq-coverage analysis; park-don't-steer). Four children
parked at ~1617.25 / 1619.75 / 1622.25 / 1624.75 MHz tile the whole
band with no steering, dissolving the LO-steering problem entirely —
this is the multi-receiver fix the demod-ceiling analysis called for
(the real gap is BANDWIDTH, not demod).

**Heterogeneous fleet:** add a VDL2 child (LO 136.8125 MHz — all VDL2
channels in one window), a POA child (131.550 MHz AM-MSK, §I.8), and
later an Inmarsat-Aero child (~1545–1555 MHz GEO patch,
`2026-07-24-inmarsat-aero-band-proposal.md`). Each band's antenna is
physically distinct anyway, which is exactly why these are separate
boards and not a soft-switch on one board. Children may even be
different P4 silicon revisions (`2026-07-28-p4-cpu-revision-variants.md`)
— the wire contract, not the binary, is what's shared.

## I.3 Transport — two regimes, reconciled

There are two legitimate transports, for two different physical
arrangements. They carry the SAME logical payload (§I.4) and feed the
same `aggregator_ingest`-style funnel on the main.

| | **SPI `frame_link` (Part II)** | **Network (Ethernet/WiFi)** |
|---|---|---|
| Physical fit | child SoCs co-located on ONE PCB / backplane, typically sharing a wideband front end (HydraSDR Topology A/C) | physically-separate child BOARDS, each with its own SDR + antenna, anywhere on the LAN |
| Status | draft code (`frame_link.c`), host-tested framing, **HW-pending, GPIO pins unverified** | building blocks live: `acars_push.c` UDP delivery (`push_target_t`), C6 WiFi via SDIO; P4-NANO has an unused 100M Ethernet PHY (IP101GRI, RMII — `p4-nano-board-schematic-summary.md`) |
| Latency / rate | µs-scale, MB/s-scale | ms-scale, still 3+ orders above the ~kbps PDU rate |
| Reach / fault domain | cm; shared power/reset domain | building-scale; each child fails independently |
| When to use | wideband-single-SDR channelizer clusters; dense multi-P4 boards | **the default for this architecture** — heterogeneous bands never share an RF front end, so board separation is inherent |

**Recommendation: network-federated is the primary regime.** A POA
child and an Iridium child have nothing to share but IP; Ethernet (or
the existing C6 WiFi) is already provisioned per board; and the PDU
rate (Part II: ≤ Kbps per child even in stress scenarios) is trivial
for UDP on a LAN. The SPI `frame_link` remains the co-located special
case and is NOT discarded: its wire format (`[magic][ver][flags]
[payload][crc16]`) is transport-agnostic and should simply be carried
inside a UDP datagram — one encoder (`frame_link_encode`/`decode`),
two transports, and the CRC+magic+version check does double duty as
the datagram sanity filter. UDP loss semantics match the existing
philosophy (drop-don't-stall, `acars_push` model); a lost PDU is
air-truth-equivalent loss and the rate is so low that even a naive
resend-on-nack would be cheap if ever needed.

Practical notes:
- **Firmware Ethernet support does not exist yet** (WiFi via
  esp_hosted/C6 is the only active netif; even `uart_log` AUTO's
  network check is WiFi-only). Children can federate over WiFi TODAY;
  bringing up `esp_eth`+IP101GRI on the main (wired backhaul, frees
  WiFi entirely) is a self-contained work item.
- esp_hosted drops outbound **multicast**, so discovery/transport must
  be unicast (§I.6) — same constraint the iot_log/mDNS work hit.

## I.4 The payload: what crosses the child→main boundary

Part II's Option B-3 analysis still holds — ship post-demod frames,
never IQ. But there are two candidate levels, and the answer is
per-band:

**(a) Band-tagged PHY frame PDU (decode-at-main).** Child stops after
demod+FEC (Iridium: BCH; VDL2: nothing — RS runs in L2); main runs the
band's L2/L3 (`frame_decoder.c`: classify + IDA/LCW + `sbd_reassembler`
+ libacars, or `vdl2_l2_feed`).
- Pro: **cross-child reassembly.** This is decisive for overlapping
  Iridium parks: a multi-fragment SBD chain can hop channels
  mid-conversation, so fragment 1 can land in child A's passband and
  the continuation in child B's. Only a main-side `sbd_reassembler`
  sees both. (The 2026-07-17 "loss = continuation not opener" finding
  is exactly this failure mode within one window.)
- Pro: central logic — one place for libacars, speculative-DA (#24),
  best-effort salvage, per-frame-type stats.
- Con: main must run every band's L2 concurrently (§I.5), and the PDU
  must carry enough context (absolute freq, band, epoch time).

**(b) Decoded ACARS record (decode-at-child).** Child runs its full
existing pipeline including L2 and ships the `acars_msg_t`-equivalent
(reg/label/text/…) as NDJSON — essentially pointing the existing
`out_host` debug push (or an airframes-schema `push_target_t`) at the
main instead of at airframes.
- Pro: zero new child code (it is literally today's standalone firmware
  with `out_host=<main>`); main needs only a JSON ingest + dedupe; band
  L2s stay where they are.
- Con: forecloses cross-child SBD reassembly and main-side rescue
  (#24); duplicates libacars state per child (harmless); loses PHY
  metadata unless the schema carries it.

**Decision: level (a) — band-tagged PHY PDU — is the target contract**,
because the homogeneous-Iridium case (the first real deployment) needs
cross-child reassembly, and VDL2/POA fit the same contract trivially
(their "PHY frame" is the AVLC/ACARS bit vector the child's demod
already emits at the `worker_emit_frame_*` seam). Level (b) is the
**Phase-1 stepping stone** (§I.7) because it works with zero firmware
change.

**The generalized PDU.** `iridium_frame_pdu_t` (`frame_pdu.h`) is
Iridium-shaped in three ways that must change:

```c
// band_frame_pdu_t sketch (v2 of the frame_link payload; ver byte bumps)
typedef struct {
    uint64_t epoch_us;      // WALL-CLOCK capture time (net_time), not esp_timer —
                            // boot-relative timestamps cannot be aggregated
    uint32_t source_id;     // child id (STA-MAC low 4 B, as today)
    uint8_t  band;          // band_id_t — THE new field; drives main-side dispatch
    uint8_t  direction;     // band-defined (Iridium DL/UL; 0 for VDL2)
    uint8_t  flags;         // CHASE (Iridium), SPECULATIVE/UNKNOWN (#24, §I.4a), …
    uint8_t  fec_meta;      // band-defined (Iridium: e1/e2 packed; VDL2: RS state)
    uint32_t freq_hz;       // ABSOLUTE burst frequency (child LO + rel_freq) —
                            //   rel_freq_hz/peak_bin are meaningless across
                            //   children at different LOs; dedupe needs Hz
    float    snr_db;
    uint16_t n_bits;
    uint8_t  bits[];        // variable/max-sized: Iridium ≤512 bits fits today's
                            //   64 B; a VDL2 AVLC burst can be several kbit —
                            //   FRAME_PDU_MAX_BITS=512 is an Iridium constant,
                            //   the v2 wire format must be length-prefixed
} band_frame_pdu_t;
```

Main-side, `aggregator_ingest.c` grows a band dispatch: today it
unconditionally calls `frame_decoder_push()` (the Iridium entry). v2
routes on `pdu.band` — Iridium → `frame_decoder_push`, VDL2 →
`vdl2_l2_feed`-path, per `band_select_pipeline()`-style mapping. Soft
bits stay child-side in v2 (as today: "the PDU wire format carries
hard bits only, so Chase-2 is inert on the aggregator path");
revisit only if #24 measurement says main-side soft decode pays.

### I.4a The `real_known` ship-gate and speculative decode (#24)

`worker_core1.c` ~L988 ships an Iridium PDU only `if (real_known)`
(BCH-OK AND classify != UNKNOWN) — the #111 false-positive filter.
Memory `project_distributed_pdu_gate_verified` established this is
outcome-equivalent to STANDALONE **today** (same classify on same
bits). But in the aggregator model it forecloses two things:
1. **Speculative-DA rescue (#24):** main-side rescue of BCH-OK-UNKNOWN
   frames requires those frames to cross the link. v2 should add an
   NVS-gated "ship UNKNOWN" mode (default off) that forwards
   BCH-OK-UNKNOWN frames flagged `SPECULATIVE` — bandwidth is a
   non-issue (Part II margins are ~1000×), the gate exists purely as a
   false-positive filter, and the main can apply the Tier-1/Tier-2
   arbitration centrally.
2. **VDL2 on the distributed roles:** the `worker_emit_frame_vdl2` sink
   currently *discards* frames on WORKER/COMBINED ("ships Iridium PDUs
   only", worker_core1.c ~L1038) — a placeholder, not a design. The
   band-tagged PDU makes VDL2 shipping first-class.

## I.5 Multi-band main: the one-band-per-boot invariant breaks

This is the largest real supersession the generalization causes. The
band framework deliberately resolves ONE band per boot
(`band_runtime_resolve()` once at init; `frame_decoder.c` boot-time
`s_band_vdl2` branch; the airframes feeder config is "global — the
device runs exactly ONE band per boot", `2026-07-27-airframes-feeder-plan.md`
§0). A heterogeneous main violates that on the AGGREGATOR role only:

- `frame_decoder` on the main must host **per-band L2 state
  concurrently** (Iridium classify/IDA/SBD-reassembler AND
  `vdl2_l2`/AVLC), dispatched per-PDU by `pdu.band` rather than
  resolved once. The DSP front end is untouched (children own it), so
  none of the PIE/heap-position constraints apply — this is plain
  C on Core 0/1 at trivial rates.
- `acars_push`'s airframes formatter is already band-aware per message
  (`af_msg_t.band`, `AF_BAND_VDL2`/`AF_BAND_IRIDIUM`) but selects the
  band from the boot-global `cfg->band` and feeds ONE port. Multi-band
  main needs per-message band → per-message target port (VDL2→5552,
  Iridium→5590, POA→5550) — a small change since `push_target_t` is
  already a struct; instantiate one per upstream port.
- `/status` on the main: the per-band decode funnel
  (`band_decode_stats_get`) becomes per-(child, band) — fold into the
  existing `pdu_link.sources[]` table (`aggregator_ingest.c`), which
  already tracks per-source counts/liveness; add `band`, `lo_hz`,
  `last_epoch_us`, and per-band message counters per source.

Note: WORKER-role children need none of this — each child stays
strictly one-band-per-boot, which is what keeps the child firmware
identical to today's standalone build.

## I.6 Dedup, time, and identity

- **Time:** children stamp PDUs with **wall-clock epoch** via
  `net_time.c` (SNTP, already in-tree; started on STA-got-IP). The
  existing PDU's `timestamp_us` is boot-relative `esp_timer` time —
  useless across boards. Rule (matches `net_time.h` doctrine): a child
  that has not synced yet ships `epoch_us=0` and the main timestamps on
  arrival (flagged); never ship a boot-relative value as if it were
  epoch. LAN SNTP gives ~ms agreement, far tighter than the ±5 ms
  dedupe window needs.
- **Dedupe at the main** (needed once two children's coverage
  overlaps — adjacent Iridium parks share edge channels; an aircraft
  simulcasting on VDL2 vs POA does not happen in practice, media
  selection picks one):
  - PHY level (v2 ingest): key on `(epoch_us ±5 ms, freq_hz within a
    channel width, bits Hamming < 4)` — Part II's triple with
    `peak_bin` (child-local, LO-relative, meaningless cross-child)
    replaced by **absolute `freq_hz`**.
  - ACARS level: libacars reassembly already drops
    `LA_REASM_DUPLICATE`, and airframes dedups server-side across
    stations — so PHY-level dedupe is a stats-hygiene feature, not a
    correctness gate. Ship without it; add when two-park data exists
    (Part II open-risk #2 said the same).
- **Identity:** `source_id` = STA-MAC low-4B (existing
  `frame_pdu_source_id()`); the CLUSTER has one airframes station
  identity (`af_id` on the main). Children's `station_id` stays a
  debug label.

## I.7 Config / ops / fleet lifecycle

- **Provisioning a child** = today's commissioning flow: flash standard
  firmware, `set band …` / `set lo …` / WiFi creds via serial_cmd or
  `/config`, plus two new NVS keys: role (or simply
  `pdu_host`/`pdu_port` — a set pdu_host IS the worker role, mirroring
  the `out_host`-empty-disables idiom) pointing at the main.
- **Discovery: none — children push.** Static push targets, unicast
  (esp_hosted multicast is broken anyway). The main learns its fleet
  passively from `source_id`s exactly as `aggregator_ingest.c
  note_source()` does today, with `AGG_MAX_SOURCES` sized for the fleet.
  A periodic child **heartbeat datagram** (same wire format, a
  status-PDU type carrying band/lo/gain/decode-funnel/uptime, ~1/10 s)
  turns the `sources[]` `age_ms` into a real liveness verdict and gives
  the per-child `/status` rollup WITHOUT the main HTTP-polling N
  children (avoids stacked-poller pathologies; children remain
  individually curl-able for deep debug).
- **OTA:** per-child, existing `ota_url` pull. One shared binary for
  all P4 children (band is NVS, not build config) — the fleet property
  that makes N boards operable. AGGREGATOR remains a build role until
  proven worth folding into NVS.
- **Failure domains:** main down ⇒ children keep decoding; Part II's
  open-risk #3 (spool-and-replay on the child) applies unchanged and
  is more attractive here since UDP tells the child nothing — a tiny
  flash/PSRAM spool with replay-on-heartbeat-ack would close it.
  Child down ⇒ coverage hole only; main `/status` surfaces it via
  heartbeat age.

## I.8 POA child (forward reference)

A POA ("Plain Old ACARS", AM-MSK 2400 bps) child on **131.550 MHz**
(the SITA Asia-Pacific primary — the one live POA channel over
Australia) is the canonical heterogeneous-VHF example: same VHF
antenna class as VDL2, a small new MSK `band_pipeline_t` (no FEC; L2
is essentially "hand bits to libacars"), and — decisively — POA and
VDL2 are >2.5 MHz apart, so **one dongle cannot do both**
(`2026-07-22-vhf-acars-feasibility.md`): a second VHF child is the
architecturally-honest way to add POA, not time-sharing. A separate
task is being filed for the POA band itself; this doc only reserves
its place in the fleet (`band_id_t` slot, PDU band tag, airframes port
5550).

## I.9 Phased path (what exists vs what's new)

| Phase | Deliverable | Already built | New work |
|---|---|---|---|
| 0 (today) | Single standalone unit, any one band | everything | — |
| 1 | **2-unit network federation, decode-at-child**: Iridium child with `out_host` → main; main ingests NDJSON, rolls up `/messages`, single airframes feed | band framework, `acars_push`/`push_target_t`, SNTP, airframes formatter | main-side JSON ingest + rollup; child `af_on` stays off |
| 2 | **Band-tagged PDU over UDP (decode-at-main)**: v2 `band_frame_pdu_t` + `frame_link` framing in UDP; `aggregator_ingest` band dispatch; absolute `freq_hz`; epoch stamping | `frame_pdu`/`frame_link` wire code + tests, `aggregator_ingest`, WORKER role gate, COMBINED smoke | PDU v2 (band, epoch, freq, var-length), UDP tx/rx tasks, per-band L2 dispatch on main, heartbeat status-PDU |
| 3 | **Homogeneous Iridium fleet**: 3–4 children tiling 1616–1626.5 | phase 2 + per-band NVS config | park plan, cross-child dedupe + SBD-reassembly validation on real two-park data, `sources[]` rollup UI |
| 4 | **Heterogeneous fleet**: + VDL2 child (+ POA child when the band lands) | VDL2 pipeline is first-class | un-stub `worker_emit_frame_vdl2` distributed path; multi-band main (§I.5); per-band airframes ports |
| 5 | Inmarsat-Aero child | band framework slot | the Aero demod itself (measure-first per its proposal) — the CLUSTER absorbs it as just another `band=` value |
| ∥ | Co-located SPI variant (Part II) | draft `frame_link` SPI | proceeds independently if/when the wideband-HydraSDR channelizer (Topology A/C) is pursued; adopts the same v2 payload |

Sequencing rationale: Phase 1 is a zero-firmware-change proof of the
operational model; Phase 2 is where the real (small) contract work
lands; Phases 3/4 are then fleet provisioning plus the specific gaps
called out in §I.4a/§I.5, each independently testable.

## I.10 What Part II got right / wrong, in hindsight

Still correct and load-bearing: the split-point analysis (B-3: ship
decoded frames, never IQ or tagged bursts); bandwidth/CPU envelopes;
"aggregator can be any P4"; the role/Kconfig structure and
COMBINED_LOOPBACK validation trick; open risks 2–4. Superseded by
Part I: SPI as the *assumed* transport (now the co-located special
case); the band-less `iridium_frame_pdu_t` (needs band tag, epoch
time, absolute freq, variable length); `peak_bin` in the dedupe key
(child-local — use absolute Hz); the boot-relative `timestamp_us`
(needs `net_time` epoch); the implicit one-band-per-boot aggregator
(§I.5); the unconditional `real_known` gate as permanent policy
(§I.4a); "no httpd on worker" (children keep httpd — it is the
commissioning/debug surface and costs nothing at PDU rates).

---

# Part II — The co-located SPI instance (original #119 design, 2026-05-31)

> **Scope note (2026-07-28):** everything below is the original
> SPI-centric, Iridium-only design, kept as the concrete co-located
> instance of Part I (one PCB, shared wideband front end — see
> `hydrasdr-wideband-spi-feasibility-2026-06-10.md` Topologies A/C).
> Read it for the boundary analysis and the SPI physical layer; read
> Part I for transport choice, the band-tagged PDU, and everything
> multi-band. Where they disagree, Part I wins (§I.10 lists the
> deltas).

This part bounds the design space and identifies the gating refactor
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
