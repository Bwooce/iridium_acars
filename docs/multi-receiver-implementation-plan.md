# Multi-receiver worker/aggregator — implementation plan

Implements the #119 design (`multi-receiver-spi-aggregator-design.md`,
Option B-3): N worker P4s each run the full single-instance front end
and ship ~280 B decoded-frame PDUs over SPI to one aggregator P4 that
runs the classifier + libacars + dedupe + outputs.

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
3. **SPI transport** — `frame_link.{c,h}`: aggregator = SPI master
   (`esp_driver_spi`, per-worker CS + INT), worker = SPI slave (serves
   PDUs from the ring, asserts INT). Plus the COMBINED in-process shim.
   *Validation:* COMBINED loopback on one board; optional GPSPI2(master)↔
   GPSPI3(slave) jumper loopback to exercise the real SPI driver.
4. **Aggregator frame ingest + dedupe** — master poll → per-worker input
   queue → `iridium_frame_classify` → the existing `frame_decoder`
   MS/TL/BC/LW/RA dispatch; cross-receiver dedupe (timestamp ±5 ms,
   peak_bin window, bits Hamming < 4). *Deliverable:* COMBINED run decodes
   the RAW_IRIDIUM corpus through the PDU path with matched=baseline.
5. **Health + observability** — aggregator `/status` shows per-worker
   link liveness / last-PDU / PDUs-per-sec; worker keeps a LAN-only debug
   httpd. *Deliverable:* operator can see receiver health.

## Validation strategy (single board)

- **Host unit tests:** PDU pack/unpack round-trip; dedupe logic.
- **COMBINED_LOOPBACK device smoke:** drive the RAW_IRIDIUM fixture through
  the worker front end → PDU queue → aggregator ingest → frame_decoder,
  and assert the classified count matches the current direct-path baseline
  (GOLDEN matched=4). This proves the split is behaviour-preserving without
  a second board.
- **SPI driver:** GPSPI2↔GPSPI3 jumper loopback test (one board) for the
  real master/slave path; full two-board test deferred until the second
  P4 is wired.

## Out of scope (this effort)

- uw_correlator / worker_core1 / ingest_core1 / signal_buffer context
  refactor (not needed for single-instance-per-chip).
- Production backplane PCB (bench jumpers only).
- Low-latency / voice overlays.
