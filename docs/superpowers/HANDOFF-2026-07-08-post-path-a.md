# HANDOFF 2026-07-08 — post-Path-A, post-P1.5-triage

Supersedes HANDOFF-2026-07-06-freq-scanner.md for current state. Read this first
to restart. Companion memory: `project_path_a_native_2500`,
`project_stale_drop_and_priority_model`, `project_first_acars_ground_truth`.

## TL;DR — where we are
- **`main` @ `cad71e8`, pushed, in sync.** Single clean branch (all experiment/
  scanner/P1.5b branches deleted; their work is in main or superseded).
- **Path A is the architecture:** RTL-SDR sampled natively at **2.5 MSPS, no
  resample**; **int8 signal_buffer** (3.36 s window); one PIE owner per core (no
  wedge). Proven to stream 4.6–4.9 MB/s clean, 0 USB drops, and to **decode OTA**
  (15 crc=OK + 73 LW.DA in one satellite pass at LO 1622).
- **THE binding constraint is RECEPTION**, not code. Bench is SNR-limited (8-bit
  RTL ADC under heavy narrowband RFI); decodes are pass-clustered ~17/h at best.
  The pipeline is proven end-to-end — we need a strong pass / better RF, not more
  DSP.
- Bench currently runs a 6 h instrumented dwell at **LO 1622** (see below).

## What landed this session (main, newest first)
| commit | what |
|---|---|
| cad71e8 | docs: P1.5 triage-redesign design doc |
| 5d8581d | **P1.5 burst_prefilter triage** — reject noise before demod (width/duration/channel-SNR gates + evict-stale queue) |
| f0c76f5 | **min-based DSP perf check** (mean was preemption-inflated); wind assertion dropped (smoke-only PIE artifact) |
| efe41c6 | ring-capacity single-source-of-truth + int8 window-defeat fix |
| b4f8c70 | **int8 signal_buffer** — 2× window (3.36 s), half DMA, bit-exact |
| ecebe39 | drop-SNR instrumentation (snr= on stale-burst drops) |
| 4c2c029 | **TASK_WDT-reboot fix** — frame_decoder prio 4→6 + unsubscribe |
| a048bc4 | STATUS logs listening band (`lo=/band=`) |
| 59b0ae6 | quiet per-burst UW-no-match log (INFO→DEBUG) |
| 2052d93 / c8fb095 / cf929d2 | Path A foundation (native 2.5, 2.5 fixtures, PIE tagger mag/EMA) |

## Mental models / key findings (don't re-derive)
- **Stale-drop / priority model** (`project_stale_drop_and_priority_model`): the
  worker queue holds descriptor POINTERS into the shared 3.36 s sample ring, not
  copies. Under the ~1400-burst/window RFI flood the worker can't pop fast enough;
  low-priority descriptors' samples get overwritten → "stale burst — drop". These
  drops are **BENIGN** — they shed the weak tail (drop-SNR all <16 dB = BCH-fail
  junk). Guard = watch the drop `snr=`; a STRONG drop = a real lost decode (0 so
  far). `burst_priority` = narrowband-first (−1000 if width>48 bins), then SNR.
- **Frequency-prior is a NO-GO** for flood reduction. `/diag/histograms` (491k
  detections) shows RFI **densest INSIDE the ACARS band** (63% in 1620.75–1622)
  with a sharp near-LO peak (~1621.9, bins 17–19) that does NOT match real IDA
  (reference peaks 1620.5–1621) → a local RTL DC/LO artifact, in-band, untouchable
  by a band prior. **CONFIRMED local (2026-07-08): the HydraSDR (same sky, 12-bit)
  shows 1621.9 as ordinary (2127, == neighbors) — no spike.** It's our RTL
  DC-offset spike (LO=1622; peak just below LO = near baseband DC) past #113 DC
  removal — a chunk of our "flood" is self-inflicted. Real flood levers = **exclude
  near-DC bins from the tagger burst mask** (kills the peak at source) + **raise
  `tag_thr`** (residual broadband).
- **Path A dissolved the PIE wedge** (one owner/core by construction). Old R1/R3/
  pie-retier branches are dead.
- **IMS = dead/off-mission**: 0 IMS frames received in 14 h; planes use SBD-data
  (LW.DA→IDA→SBD→ACARS), never paging. Skip.
- **SD logging works** (LDO#4 bring-up done); `sd_log_emit` lazy-mounts on first
  decoded ACARS — idle only because ~0 complete ACARS decoded at the bench.
- **Device-smoke is mandatory** — this session it caught a boot crash-loop (P1.5
  port reintroduced an EXT_RAM_BSS_ATTR-without-esp_attr.h bug → 24 KB internal
  .bss → broke the 144 KB boot DMA reserve) that 46/46 host tests missed.

## Bench / device state + how to check
- Dwell script: `scratchpad/dwell.py` (LO 1622, logs to `/tmp/smoke_pathA2/
  dwell_1622.{log,runlog}`, 10-min heartbeats). Serial via pyserial w/ DTR asserted
  (bare `cat` gets EOF on the CH343). Stop with `pkill -f "[d]well.py"` (bracket
  trick — plain `pkill -f dwell.py` self-matches the shell and kills the wrong job).
- Live health: `curl http://192.168.1.235/status` and `/messages` and
  `/diag/histograms` (freq/SNR/BCH occupancy). STATUS line now carries `lo=/band=`.
- Smoke: `OUTDIR=/tmp/smoke_pathA2 scripts/smoke_run.sh raw` (GOLDEN `matched≥40`,
  ~61 = healthy) then `... restore` to return to production. sdkconfig is
  smoke-tainted after; restore rebuilds prod. RAW smoke is GREEN as of f0c76f5.
- Dev loop: `scripts/build.sh` then `scripts/flash.sh` (SEPARATE invocations).
  Flash port = CH343 1a86:55d3 (ACM index shuffles; detect by vendor).

## REMAINING BACKLOG (prioritized)
**Reception (the gate — everything else is downstream):**
1. Confirm live ACARS decode on a strong pass (pipeline proven; needs RF/pass).
2. RTL RX-chain SNR: 1.6 GHz front-end filter, gain 35→45 dB experiment, confirm
   bias-tee/antenna. 8-bit-ADC-under-RFI is the ceiling.

**Flood / noise (tagger-side — the real levers, per the histogram):**
3. **Exclude the near-DC bins from the tagger burst mask** (~1621.9, bins 17–19)
   — CONFIRMED RTL DC/LO spike (HydraSDR clean at 1621.9, so it's local, not RF);
   ~66k detections/window in 3 bins. Most targeted win, self-inflicted flood.
4. **Raise `tag_thr`** — flood is high across ALL bins; a threshold bump cuts
   everywhere (cheap, one value). Judge by FRMDEC bch_ok/h, not burst counts.
5. Verify P1.5 prefilter's per-burst FFT didn't worsen worker throughput (A/B).
6. **Frequency prior: DROPPED** (histogram no-go — RFI is in-band).

**Frequency scanner (make it useful — from the merge agent's report):**
7. Unblock automated `scan` — DMA-INT churn; reinstate control-URB reuse (the
   p3-dma-reclaim fix was reverted at 9638816); reconcile pool/reserve on Path A.
8. Fix `xfer_mutex` per-CT-not-per-sequence race (AGC CT interleaves in a retune).
9. Density-map LO selection: SNR-weighted ranking + non-uniform dwell.
10. CW positive-control device test (never run); re-prime-after-hop bench check.

**ACARS content (libacars task 10 — scoped, `2026-07-08-libacars-best-effort-decode.md`):**
11. Host path: ADS-C (cheap) then CPDLC (heavy ASN.1, ~500–700 KB). Design+tests exist.
12. Firmware surfacing plumbing (flag-gated): detect + `la_proto_tree_format_text`
    the CPDLC/ADS-C child, add to msg_ring/sd_log/HTTP.
13. Device ADS-C enable (cheap); DEFER device CPDLC (oceanic-only, reception-gated).
14. **Our-chain raw salvage** — raw label/sublabel+hex on broken fragments
    (37–49% of captures) — higher on-device value than CPDLC.

**Smoke / PIE / test:**
15. Smoke-only `wind` PIE artifact (~2 ms window_multiply in smoke vs ~76 µs prod);
    re-add the wind perf-assertion once understood.
16. device-smoke GOLDEN PIE-save deadlock (`rtos_save_pie_coproc`) — gate deferred.

**Networking / hardware / multi-receiver:**
17. esp_hosted multicast TX broken (mDNS/iot-log; HTTP `/status` workaround live).
18. c6_forwarder → SDIO rewrite (wired to dead GPIOs; P4↔C6 is SDIO-only).
19. Multi-receiver Phase 3 SPI (`frame_link`) hardware bring-up; GPIO pins unverified.

**Review backlog / housekeeping:**
20. 2026-07-04 review open items T13–T50 — notably T14 OTA-auth (security),
    T18/T19 (physical USB-unplug testing), T48–T50 (throughput/PIE).
21. Whole-branch review (freq-scanner history) if desired.

**Cheapest / highest-leverage next:** #4 (`tag_thr`), #3 (near-LO notch), #2 (RF
front-end) — because reception is what actually gates a real ACARS decode.
