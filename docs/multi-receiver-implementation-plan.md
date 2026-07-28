# Multi-receiver worker/aggregator — implementation plan

Implements the #119 design (`multi-receiver-spi-aggregator-design.md`,
Option B-3): N worker P4s each run the full single-instance front end
and ship ~280 B decoded-frame PDUs over SPI to one aggregator P4 that
runs the classifier + libacars + dedupe + outputs.

> **Scope note (2026-07-28):** the design doc has been generalized to a
> multi-unit, multi-band architecture (its Part I): one main + N
> physically-separate child boards, heterogeneous across bands
> (Iridium LO parks / VDL2 / POA / Inmarsat), federated over the
> NETWORK by default, with a band-tagged v2 PDU. This plan remains the
> implementation record for the co-located SPI instance (design doc
> Part II) and for the role/PDU/ingest infrastructure phases 1/2/4/5,
> which Part I reuses. New work items (PDU v2, UDP transport, per-band
> aggregator dispatch) are scoped in the design doc §I.9, not here.

**Scope decision (2026-06-16):** worker/aggregator SPI split, built now,
validated on a single board (software loopback + optional GPSPI2↔GPSPI3
jumper loopback); the second board gets wired later.

**Key simplification:** each chip is single-instance, so the pipeline
singletons stay singletons. The full #120 context refactor (esp. the
uw_correlator PIE-buffer relocation) is NOT required and is explicitly
out of scope — we avoid the heap-position-corruption risk entirely. The
split needs role-gated wiring + a frame-output redirect, not reentrancy.

## Roles, one codebase

`CONFIG_DEVICE_ROLE` choice (Kconfig): `WORKER` (default, == today's
behaviour plus SPI-out) or `AGGREGATOR`. `app_main` branches on role.
Shared code (DSP, decoders, transport, wifi, http) compiles for both;
role-specific top-level wiring is gated.

A third validation-only value, `COMBINED_LOOPBACK`, runs both halves in
one binary on one board with the SPI transport replaced by an in-process
queue — lets us validate the PDU pipeline + dedupe end-to-end without a
second board or SPI wiring.

## Phases

1. **Role infrastructure** — `CONFIG_DEVICE_ROLE` choice; role-gated
   `app_main`; build both roles green. No behaviour change for WORKER
   (it's today's firmware). *Deliverable:* both binaries build + boot.
2. **Frame PDU + worker output queue** — define `iridium_frame_pdu_t`
   (#119 layout), pack/unpack, and a PSRAM output ring (~32 PDUs).
   Worker: at `worker_core1.c` frame-emit, keep the #111 classify-or-drop,
   then enqueue a PDU instead of (WORKER role) / in addition to
   (COMBINED) the local frame_decoder call. *Deliverable:* worker fills
   the ring; unit-tested pack/unpack on host.
3. **SPI transport** — DRAFT (`frame_link.{c,h}`, commit d12781e).
   Aggregator = SPI master (SPI2), worker = SPI slave (SPI3) serving PDUs
   from the ring and asserting a handshake GPIO. Fixed-size wire frame
   `[magic][ver][flags][PDU][crc16]` (CRC-16/CCITT). Pins are
   Kconfig-configurable (`FRAME_LINK_*_GPIO`); defaults avoid C6/SDIO
   (6,14-19,54) and SD (39-45) but MUST be checked against board headers.
   COMBINED keeps the in-process shim (no SPI). *Validated so far:* host
   `test_frame_link` (framing + CRC, every-single-byte corruption rejected)
   + clean builds in all four configs. *NOT hardware-validated:* the SPI
   master/slave path and `frame_link_loopback_selftest()` need a second P4
   or a GPSPI2↔GPSPI3 jumper (CONFIG_FRAME_LINK_LOOPBACK_SELFTEST runs the
   one-board test at boot). This is the ready-to-bring-up draft.
4. **Aggregator frame ingest + dedupe** — DONE (`aggregator_ingest.{c,h}`,
   commit d57625d). A Core-1 prio-4 task pops the frame_pdu queue, unpacks
   bits, and calls `frame_decoder_push` (which already does the
   MS/TL/BC/LW/RA dispatch). Wired for COMBINED (in-process) and AGGREGATOR
   (idle until the Phase-3 SPI source). Cross-receiver dedupe (timestamp
   ±5 ms, peak_bin window, bits Hamming < 4) is DEFERRED to Phase 3: with a
   single worker there are no duplicate sightings, so it is a no-op today
   and needs the multi-worker SPI transport to exercise.
   *Validated:* COMBINED_LOOPBACK + RAW_IRIDIUM smoke = SMOKE_PASS. Note the
   COMBINED gate is a bounded (<=90 s) positive-delivery assertion, NOT the
   STANDALONE fixed-window count: the worker ships only known-type frames
   (#111 drops UNKNOWN) and is not real-time on this fixture (~140 s full
   backlog), so the gate asserts that >= floor frames actually traversed
   pack→queue→ingest→unpack→frame_decoder (5 delivered, clean BCH
   blocks=10/10 errs=0 crc=OK → serialization is bit-exact).
5. **Health + observability** — DONE for the single-board surface
   (commit 5687f8c). `/status` now carries a `pdu_link` object (gated to
   non-STANDALONE roles): `forwarded`, `queue_depth`, `dropped`, and a
   per-source `sources[]` table (`id`/`count`/`age_ms`) fed by
   `aggregator_ingest`'s per-source tracking. STANDALONE `/status` is
   unchanged. Verified on-device (COMBINED): valid JSON, block present.
   *Deferred to Phase 3:* turning `age_ms` into a liveness verdict
   (worker-silent alerting) and surfacing N>1 workers — both need real
   multi-worker SPI traffic to exercise.

## Validation strategy (single board)

- **Host unit tests:** PDU pack/unpack round-trip; dedupe logic.
- **COMBINED_LOOPBACK device smoke:** drive the RAW_IRIDIUM fixture through
  the worker front end → PDU queue → aggregator ingest → frame_decoder, and
  assert that >= floor frames are delivered + classified through the PDU
  path (bounded <=90 s; UNKNOWN frames are not shipped so the STANDALONE
  fixed-window count doesn't transfer). Clean BCH/CRC on the delivered
  frames proves the split is behaviour-preserving without a second board.
- **SPI driver:** GPSPI2↔GPSPI3 jumper loopback test (one board) for the
  real master/slave path; full two-board test deferred until the second
  P4 is wired.

## Out of scope (this effort)

- uw_correlator / worker_core1 / ingest_core1 / signal_buffer context
  refactor (not needed for single-instance-per-chip).
- Production backplane PCB (bench jumpers only).
- Low-latency / voice overlays.
