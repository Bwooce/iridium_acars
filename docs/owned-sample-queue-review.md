# Owned-sample priority queue — design review (MEASURED & REJECTED)

**Date:** 2026-07-12
**Status:** **Rejected on current evidence.** Design preserved here for reopening if the RF picture changes (a data-heavy scenario or a different site). Reopen criterion at the bottom.

## The problem this would solve
The burst priority queue (`worker_core1.c`, `s_pq[BURST_PQ_CAP=64]`) holds **descriptors** pointing into the shared 16 MB sample ring (`signal_buffer`, ~3.3 s at 2.5 MSPS). Under a dense flood the worker (~13 demods/s) can't keep up, and an *admitted* burst's samples get overwritten ("lapped") before the worker demods it — a **stale loss**.

Measured over a 58-min midday flood (via the burst-drop SNR histogram, `/diag/reassembler` → `burst_drops`):
- **stale (ring-lap) losses ≈ 2283 bursts, ALL ≥16 dB** (16-20:1872, 20-24:344, ≥24:67).
- **priority-evicted losses ≈ 597, also ≥16 dB** (a uniform high-SNR flood fills the queue with strong bursts and drops strong newcomers too).
- ⇒ ~50 decodable-SNR bursts/min shed during floods, while the worker produced only ~156 valid IDA frames.

## The idea
Make the queue **own its bursts' samples** (copy the window in at admission) so a queued burst is immune to ring-lap and survives until demodded — deferring demod from the flood peak into the idle trough. This dissolves the stale-loss class and makes the queue itself the peak→trough backlog.

## Verdict: NO-GO as specified
1. **Wrong core.** Copy-at-admission runs in the tagger callback on **Core 0** — the 97%-busy core during exactly these floods. Copying ~126 KB/admission at tens/s injects 5–15 MB/s of PSRAM+L2 work into the saturated core, ~95% of it wasted. Structural, not tunable.
2. **Budget doesn't cover it.** ~3 MB free PSRAM (the ring **cannot grow** — 20/24 MB OOM the usbring). Raw int8 windows ⇒ ~16 single / 3–4 multi-frame bursts. Against ~50 shed/min that's a lifeboat, not "ownership."
3. **~0 message yield.** Through the flood `sbd_complete` stayed **0**; the traffic is ~99% single-frame control chatter (maint/hndof), <1% data; prior stale-recovery yield ~2–4%. This is a **frames/coverage** lever (a real ~4× *frame* multiplier), but a **~0 messages** lever — and messages are the mission. Reception + multi-frame completion remain the bottleneck.

## The reduced-scope design, if it's ever built (the "decimated lifeboat")
The one shape that isn't dominated:
- **Store the decimated 250 ksps window, not raw ring samples.** The worker's first act (`wb_extract_decim`) already produces this int16 250 ksps buffer — **5× smaller** than the raw int8 window. So a "rescue" = run that existing stage early and stash its output. This front-loads work that had to happen anyway, and lets `burst_prefilter` run at rescue time so junk never occupies a slot.
- **Pool:** 12 slots × 128 KB = **1.5 MB**, one static boot-time `MALLOC_CAP_SPIRAM` alloc (64-B aligned, never `.bss` — DMA-INT/stream-kill rule), bitmap allocator. A slot = 32k complex 250 ksps samples = one gri-capped multi-frame burst. Descriptor queue stays 64; ownership is a **parallel array** `s_pq_owned[64]` (`detected_burst_t` must NOT grow — PIE placement hazard).
- **Copy on the WORKER (Core 1), between bursts — the hybrid, not at admission.** At most one rescue per pop loop, targeting the highest-priority *endangered* entry (aged > 50% of ring span AND ≥16 dB AND a slot free). Cost ≈ one `wb_extract_decim` (~5–10 ms) at ~0.83/s ⇒ **~0.7% of worker capacity**; PSRAM traffic ~0.13 MB/s. Copy-at-admission is strictly dominated (50–100× more copies, wrong core).
- **Draining:** worker pops owned entries in the trough like any other, strongest-first; owned-entry pop skips the `signal_buffer_burst_valid` guards + extract/decim (samples are private). Two-phase rescue with a generation check + post-read `burst_valid` re-check (torn-read hazard). `pq_insert`'s EVICT-STALE-FIRST skips owned entries; lowest-priority eviction frees the slot.
- **Kill-switch:** NVS `rescue_en` (default off) for an A/B soak judged on `bch_decoded/h`.
- **Cut line:** all in `worker_core1.c` + new `burst_store.{c,h}`; `signal_buffer.{c,h}` untouched.

## Why it's rejected without even measuring
The gate was two measurements: **M1** (is there reachable trough idle to drain into?) and **M2** (is the shed >90% control chatter?). **M2 is already answered by existing telemetry** — over the 58-min soak, of the 156 valid data-channel frames decoded, **139/139 standalone frames filtered as non-SBD control, `sbd_complete=0`, `acars_decoded=0`** (~100% chatter, well past the 90% stop threshold). M1 only matters if M2 passes; it doesn't. So the deciding measurement says **don't build** without any new instrumentation.

## Better use of the same effort (ranked)
1. **Reception** (antenna / LO siting) — the actual yield limiter (53% of frames fail the CRC gate = marginal SNR).
2. **Queue-protects-multi-frame policy** — don't evict an in-progress LW.DA/SBD chain member; a cheap `worker_core1.c` PQ policy change, targets the rare data directly.
3. **Task C** (dirty-continuation admission) — `dirty_cont` counter is measuring its size (currently ~0).

**Note:** compute is *not* a lever here — the FFTs are already PIE-accelerated and the 76%-of-budget UW correlator is scalar-float by a settled gri-parity choice (see `docs/waveshare-v3-p4-request.md` Q1).

## Reopen criterion
Reopen only if a soak shows the shed cohort is <90% control chatter (i.e. real data bursts are being shed) — then run M1 (per-window stale count vs worker idle, mostly offline log analysis) to size the recoverable yield, and build the decimated-lifeboat slice above.
