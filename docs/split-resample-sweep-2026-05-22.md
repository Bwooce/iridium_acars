# Split-resample sweep — 2026-05-22

Empirical sweep of `ingest_core1_set_split_pct()` after the worker-pool
architecture landed (commit `aad9735`). Goal: characterise the
parallelism gain vs Core 0 contention cost across split ratios, on
both the RAW_IRIDIUM fixture and LIVE_SDR steady-state.

**Headline:** at today's Core 0 load (~80% with the tagger),
parallelising any non-trivial portion of the resample onto Core 0
costs more in FFT slowdown (cache contention) than it saves in
Core 1 throughput. **Default to split=0 until Core 0 has headroom.**

## RAW_IRIDIUM fixture — decode + per-frame DSP cost

Decode (matched/BCH/recall/BER) is **bit-identical at all split
values** — process_explicit is correct, the closed-form
n_emits-on-mid is correct, and the persist-state handoff between
Worker B and the next chunk is correct.

| split | DSP total | wind | FFT | mag | detect | base | matched | BCH | recall | BER |
|---|---|---|---|---|---|---|---|---|---|---|
| 0   | 574 | 77 | 258 | 50 | 107 | 73  | 61 | 29 | 93.8% | 1.91% |
| 12  | 676 | 76 | 349 | 53 | 110 | 78  | 61 | 29 | 93.8% | 1.91% |
| 25  | 762 | 77 | 428 | 54 | 111 | 85  | 61 | 29 | 93.8% | 1.91% |
| 50  | 915 | 76 | 576 | 55 | 121 | 79  | 61 | 29 | 93.8% | 1.91% |

`detect` and `base` also slow slightly with split, consistent with
generalised L2 cache pressure on Core 0 from Worker A's PSRAM I/O.

## LIVE_SDR steady-state — throughput

Steady-state with no antenna signal (pure noise) so the tagger
detects 0 bursts. USB ingest rate-throttled by the consumer chain.

| split | USB rate_inst | feed_calls/s | push | resample | DSP cap | FFT | rb_full_drops/s |
|---|---|---|---|---|---|---|---|
| 0   | 4.57 MB/s | 293 | 2976 | 2701 | 79.9% | 263 | 39 |
| 12  | 4.45 MB/s | 285 | 2542 | 2328 | 87.6% | 341 | 56 |

Core 1 work drops (push 2976 → 2542 µs, −15%) — Worker B carries
less. Core 0 work rises (FFT 263 → 341 µs, +30%; DSP cap 79.9% →
87.6%, +7.7pp). The two effects cancel: throughput is essentially
flat (4.57 → 4.45 MB/s) and **drops actually increase** (39 → 56
rb_full_drops/s, +43%). The Core 0 FFT slowdown is non-linear in
Worker A's share — even 12% costs disproportionately.

## Interpretation

The polyphase resampler's per-MAC PSRAM accesses (input chunk
reads) evict the tagger's working set from L2 when both run on the
same core. Adding any amount of Worker A traffic to Core 0
substantially slows the FFT.

The math budget that *would* let split help (Core 0 spare ≥
Worker A's share × resample cost) doesn't close because the FFT
slowdown is the dominant cost, not Worker A's compute. We can't
fix the slowdown without partitioning the cache (not a thing on
P4) or moving the FFT off Core 0.

## What this implies for the upstream backlog

The split-resample architecture is correct, validated, and ready
to use — but it's blocked behind reducing Core 0 load. Specifically:

- **Opportunity #5 (PIE-FFT swap)** — ~~would cut tagger FFT~~
  TURNS OUT TO BE ALREADY DONE (2026-05-22). The 263 µs/step we
  measure IS the PIE-optimised FFT. No further speedup available
  from that lever. See updated entry in
  `optimization-opportunities-2026-05-22.md`.
- **Opportunity #1 (drop to 2.0 MSPS)** — deletes the resampler
  entirely, making the split architecture moot.

Until something like Opp #1 lands, `s_split_pct = 0` is the
correct default.

## Deeper investigation closed out (2026-05-22 follow-up)

Investigated whether the FFT slowdown at split>0 (263 → 443 µs)
was caused by Worker A's PSRAM writes to s_resamp evicting tagger
L2 cache lines. Test: redirect Worker A's output to an internal-
SRAM scratch (`WORKER_OUT_TO_INTERNAL_SCRATCH=1` in
`ingest_core1.c`) and re-measure at split=25.

**Result: FFT cost UNCHANGED (536 µs, slightly worse than 443 µs
with PSRAM writes).** So PSRAM writes are NOT the L2 evictor. The
slowdown is from something more fundamental:

- Worker A's PSRAM **reads** of `s_conv` input (one fetch per MAC
  tap, ~4000 outputs × 9 taps × 2 bytes = 72 KB PSRAM traffic
  per dispatch via L2)
- Inter-core L2 line invalidation pattern when both cores are
  writing to PSRAM concurrently
- PIE coprocessor internal shared state we don't visibility into

None of these have a clean fix today. The split-resample
architecture stays gated behind reducing Core 0 tagger load via
some structural change (Opp #1 the most concrete).

## Alternative considered: split burst_worker instead

Could we split the burst-decode worker (worker_core1) into two
unpinned tasks instead, getting parallelism elsewhere? Memory cost
analysis (2026-05-22):

- Each new instance needs ~34 KB internal SRAM (s_chunk_iq 16 KB
  + decim_scr_in_i/q 16 KB + small).
- Plus the `uw_correlator` module has static-global FFT scratches
  (~32-50 KB internal) that would need to become per-instance —
  bigger API refactor.
- Total per new worker: ~50–70 KB internal SRAM (after API rework).

But `post-ingest_core1` heap state today: INT free=68 KB,
**largest contiguous block = 31 KB**. The 34 KB chunk_iq alone
exceeds the largest block. **Doesn't fit without freeing more
internal SRAM first** — and we already tried (peaks-PSRAM broke
decode, L2 reduction hurt USB).

Net: this alternative is also blocked. Both parallelism levers
require the same prerequisite work (Opp #1 to delete the
resampler, or a different way to free internal SRAM that we
haven't found).

## How to re-run

```c
// At runtime, from any task (e.g. via NVS-loaded config or a
// debug command):
ingest_core1_set_split_pct(25);   // 25% to Core 0
```

Changes take effect on the next dispatch. Run a smoke for ~40 s
and compare `DSP/frame total/fft` (from `SMOKE: Perf check` block)
and `Ingest/dispatch push` against the table above. The numbers
should reproduce within ±5 µs on the same fixture/board.
