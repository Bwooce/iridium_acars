# Optimization Opportunities — 2026-05-22

Snapshot of the next batch of perf opportunities after the session of
2026-05-21/22 (queue → 1024 in PSRAM, s_extract_buf removal, PIE FFT
swap, float-only UW magsearch, internal-SRAM moves, resampler
specialised + -O3 + PIE asm). Ranked by expected impact × confidence ×
feasibility on the ESP32-P4 platform.

Current state at time of writing:

| metric | value |
|---|---|
| RAW_IRIDIUM smoke avg burst | 84.5 ms (was 150 ms session-start) |
| RAW_IRIDIUM smoke decode | 29 BCH, 61 matched, 93.8% recall, 1.91% BER |
| LIVE_SDR sustained throughput | 4.25 MB/s (was 0.85 MB/s, 5.0× faster) |
| LIVE_SDR rb_full_drops | ~13% (RTL emits 5.12 MB/s) |
| Resample per dispatch | 2.67 ms (was 17.9 ms) |
| Host bch/qpsk/pipeline ctest | 20/20 pass |

### 2026-05-23 update — post D17-redo (esp_wifi_remote + esp_hosted landed)

Adding `esp_wifi_remote` + `esp_hosted` (commit `bba7817`) shifted code
in flash and added ~30 KB of always-on runtime. Then `IRAM_ATTR`-ed
the inner DSP kernels (commit `354f46f`) to mitigate the layout hit.
Re-measured on the same RAW_IRIDIUM fixture + LIVE_SDR run:

| metric | pre-D17-redo | post-D17-redo |
|---|---|---|
| RAW_IRIDIUM matched | 61 | **61** (same) |
| RAW_IRIDIUM recall | 93.8% | **93.8%** (same) |
| LIVE_SDR USB sustained | 4.25 MB/s | 3.58 MB/s |
| DSP wind/frame | ~16 µs | 76 µs |
| DSP fft/frame | ~256 µs | 256 µs |
| DSP mag/frame | ~46 µs | 46 µs |
| DSP detect/frame | ~33 µs | 94-177 µs |
| Resample per dispatch | 2.67 ms | **3.79 ms** |
| sbpush per dispatch | n/a | 0.19 ms |
| convert per dispatch | n/a | 0.33 ms |

The biggest change is resample (+1.1 ms/dispatch). At 225 dispatches/sec
in live mode that's 250 ms/sec of extra Core 1 work — and resample is
**85 % of one core** outright. **`resample_256_to_250_process_explicit`
is now the dominant real-time lever** (was the throughput lever back at
2.67 ms; now it's the throughput lever AND the only obviously-attackable
one).

Smoke perf bars rebaselined in `smoke_test.c` to current numbers + ~30 %
headroom (commit landing alongside this doc update). The old bars (wind
25 µs, detect 50 µs, push 130 µs) reflected the pre-Wi-Fi code layout
and no longer reflect a useful regression gate.

### 2026-05-24 update — task #74 instrumented (verbose status logger)

`CONFIG_STATUS_LOG_VERBOSE=y` exposed per-iteration cycle breakdown.
At 3.7 MB/s sustained, the consumer loop in `class_driver_task` looks
like:

```
Cycle (Core0): iter=239 (239/s)  handle_events=262 us/iter (6.2%)
                                 take_converted=1599 us/feed (38%)
Cycle (Core0 us avg): read=218  feed=2068
```

So per ~4.2 ms cycle: handle_events 6%, read 5%, take_converted 38%,
feed (DSP) 49%. The previous-iteration ingest is what's keeping Core
0 blocked in `take_converted`. Ingest cost per dispatch
(`convert+resample+sbpush`) is **4.3 ms**, dominated by **resample
at 3.8 ms** — exactly Optimization Opportunity #1 below. That sets
the throughput ceiling at min(Core0_cycle, Core1_ingest) =
16 KB / 4.3 ms = **3.72 MB/s**. Matches the measured 3.6.

**Conclusion:** pool size + USB host stack are NOT the throttle.
Resample is the throttle. Either rewrite it (opp #1 alt: PIE
inner-MAC, bigger) or eliminate it by dropping SDR rate to 2.0 MSPS
(opp #1 below, much smaller). Drop-to-2.0 is the next likely win.

---

## 1. Drop RTL-SDR rate from 2.56 → 2.0 MSPS (gri-aligned)

**What:** Change `FS_IN_HZ` in `dsp_processor.h:20` from `2560000` to
`2000000`. gr-iridium ships `sample_rate=2000000`; librtlsdr
explicitly warns "sample loss is to be expected for rates > 2400000".
The 125/128 resampler becomes 250/200 — drop the resampler entirely
(or retune to 2.5 MSPS native via R820T xtal divider 48.18).

**Impact:** USB byte rate 5.12 → 4.0 MB/s (-22%). Eliminates resampler
(~2.67 ms × ~125 dispatches/s = 333 ms/s of Core 1 freed). RTL stops
dropping at the source. Ringbuf drops should evaporate.

**Effort:** ~2 hours. Also retune `burst_post_len` (40000 → 32000 at
2 MSPS) and `direct_if_decim`'s decim (10× → 8× at FS_DETECT_HZ=2 MSPS).

**Risk:** Low — gri runs at this rate. Validate by re-running ALBQ
smoke (fixture stays at 2.5 MSPS; live-rate change is orthogonal).

## 2. PIE-accelerate the burst-tagger EMA + magnitude loops

**What:** `compute_magnitude_shifted()` and `update_baseline_ema()` in
`fft_burst_tagger.c:197/380` are the per-step hotspots after the FFT.
Both are perfectly vectorisable: mag² is 8-lane int16×int16→int32
reduction; EMA is `int32_t baseline_sum[k] -= old[k]; baseline_sum[k]
+= mag[k]` — pure SIMD add/sub on 2048 int32. esp-dsp doesn't have a
direct int32 add helper, but `esp.vmulas.s32.qacc` / `esp.vld.128`
sequences are straightforward (same pattern as `resample_arp4.S`).
2048 ops ÷ 8 lanes = ~256 vector iters/loop.

**Impact:** Tagger is at 81% of Core 0 cap. PIE the inner arithmetic +
ensure the active history slot is in INTERNAL SRAM → expect 2-3×
tagger speedup, Core 0 utilization 81% → ~30-40%, unblocks Core 0
second worker.

**Effort:** 1-2 days (hand-roll 2 PIE asm helpers, validate against
scalar with diff-harness pattern from `pie_fft_diff_test.c`).

**Risk:** Medium. Bit-exact validation; bin-count consistency vs ALBQ
smoke (must keep tagger output identical or recall drops).

## 3. ~~Move `baseline_history` access pattern out of PSRAM~~ — TRIED, REVERTED

**Status (2026-05-22):** Rewrote `update_baseline_ema` to use an
8 KB internal-SRAM scratch (bulk PSRAM→SRAM memcpy in, in-SRAM
EMA, bulk SRAM→PSRAM memcpy out). Measured `base` cost 73 → 99
µs/step (+36% WORSE). The fused interleaved per-bin loop is
already L2-prefetch-friendly at -O3; the split-into-three-passes
rewrite added memcpy setup overhead without saving bandwidth.

**Don't retry this** unless you're attacking it from a different
angle (e.g. background DMA prefetch of the next slot while CPU
operates on the current, which is more complex).

**See also:** s_conv → internal SRAM (would help L2 contention
when split-resample's Worker A runs on Core 0, DSP/frame 574 →
543 µs at split=0) — tried, also reverted because the extra 64 KB
of internal SRAM consumed pushed total free below the ~115 KB
threshold the mystery downstream allocation needs (silently
regressed decode matched 61 → 44, recall 93.8 → 67.7%, same
pattern as the worker-stack bug). Blocked behind finding/
relocating that victim allocation.

## 4. Conditional multi-frame iteration

**What:** `burst_pipeline.c:457` always iterates until window
exhausted, regardless of frame 1 outcome. Retry loop is ~3.3× per
burst × 18.3 ms = 60 ms/burst, but multi-frame contributes only ~4
BCH frames out of 29 (per task #79 attribution). Gate frames 2+ on:
`frame[0] BCH-clean OR (frame[0] corrected AND remaining_window >
2×MIN_FRAME)`.

**Impact:** Estimated 40-50 ms/burst on the average (most bursts are
single-frame). Per-burst time 84.5 → ~40 ms.

**Effort:** ~3 hours. Sweep "frame_1_ok_required" flag against ALBQ
smoke; expect 27-28 BCH frames vs current 29 (acceptable).

**Risk:** Low — gate is configurable; ALBQ corpus tells you the cost
exactly.

## 5. ~~Retry `dsps_fft2r_sc16_arp4` swap in the tagger~~ — **ALREADY DONE**

**Status correction (2026-05-22):** Already landed; this doc was
stale. `fft_sc16_2048.c` on ESP_PLATFORM calls `dsps_fft2r_sc16`
which on P4 resolves to `dsps_fft2r_sc16_arp4` (verified via
linker map: `dsps_fft2r_sc16_arp4_` is the actual symbol called
from `fft_sc16_2048.c.obj`). The known traps from
`feedback_pie_fft_swap_needs_validation.md` are mitigated in-source:

- Caller-owned twiddle table (`s_w_table`) passed to
  `dsps_fft2r_init_sc16` — sidesteps the size-arg-ignored bug
- Internal-SRAM scratch (`s_fft_scratch`) — PIE can't service
  PSRAM addresses on `vld.128.ip`
- ANSI bit-reversal at the end (no PIE bit-rev exists for sc16)

**Measured:** Tagger FFT at 263 µs/step (verified 2026-05-22 from
`SMOKE: Perf check`), well below this doc's earlier 610 µs
estimate. The ~6.7× win over the pre-PIE 1.77 ms baseline is
already baked into the current 80% Core 0 cap number.

**Implication:** there is no further sc16 PIE FFT speedup to
unlock. Before claiming "Tagger FFT is the next lever" in future
docs, re-check the linker map.

## 6. ~~Swap UW matched-filter to radix-4~~ — NOT APPLICABLE

**Status (2026-05-22):** `dsps_fft4r_fc32_*` requires N to be a
power of 4 (rejects with `if ((log2N & 0x01) != 0)` at
`dsps_fft4r_fc32_ansi.c:112`). Our `CORR_FFT_N = 2048` has
log2N=11 (odd), so radix-4 init returns ESP_ERR_DSP_INVALID_LENGTH
and the FFT no-ops. Pure radix-4 N values are 1024, 4096, 16384.

To use radix-4 here we'd have to double the FFT size to 4096,
which doubles the work — defeats the purpose. Stay with radix-2
PIE.

(If a future change moves the matched-filter to a power-of-4 N
for some other reason, the radix-4 swap becomes worth ~25% of
the FFT cost.)

## 7. Q15 absolute-phase phasor table for rotate-to-DC

**What:** Per memory `feedback_q15_incremental_phasor_decays.md`, you
switched to per-sample float `cosf/sinf` to avoid Q15 incremental
decay. But that's a scalar float trig call per sample × ~40 K
samples/burst. A precomputed sin/cos LUT at 2.5 MSPS phase resolution
(or 16 K entries with linear interp) keeps absolute-phase semantics,
eliminates trig, and trivially vectorises the IQ complex-mul into PIE
fixed-point.

**Impact:** Rotate cost likely 3-6 ms/burst now. PIE Q15 complex-mul
with table lookup: target ~0.5 ms.

**Effort:** 1 day (LUT generator + lookup + 1 PIE asm helper for cplx
mul).

**Risk:** Low. Validate against existing host stagewise NMSE/phase-
coherence harness.

## 8. SNR-gate + tagger threshold relaxation (after #1-#3)

**What:** Tagger is at 14 dB threshold because the worker can't keep
up. After #1-#3 land, lower to 10 dB to recover the missing 4 BCH
frames (host runs 92% recall at 10 dB vs 88% at 14). Two changes
coupled — don't lower threshold without first having Core 0 / worker
headroom.

**Impact:** +2-4 BCH frames on ALBQ; gri-alignment-correct.

**Effort:** Trivial (one #define), blocked on #1-#3 landing.

**Risk:** Needs smoke recall check.

---

## Cross-cutting constraint discovered 2026-05-23: PIE position-dependent corruption

While attempting pipelined-tagger (FFT step N+1 ‖ post-FFT step N
on Core 1), a latent silicon bug surfaced: PIE asm
`esp.vld.128.ip` produces SILENTLY WRONG output for the polyphase
coefficient buffer at certain HP-SRAM addresses on ESP32-P4 v1.3.
Decode collapses matched=61 → 44 (recall 93.8% → 67.7%).

- Not in any of the seven named v1.3 errata.
- Three theory tests done (L1 d-cache writeback, invalidate,
  non-cacheable alias) — all failed to fix or were rejected by
  the silicon (PIE faults on the 0x40000000 alias).
- Workaround: alloc `s_coeffs_pp` FIRST in boot via the new
  `resample_256_to_250_alloc_coeffs()` API. TLSF places it at
  0x4ff3e330 (small-RAM region), deterministic across builds.

Implication for the opportunities below: **any future change that
grows pre-resample internal-SRAM consumption must verify
s_coeffs_pp still lands at a working address.** The alloc-first
workaround eliminates the risk for the resampler's own coeffs,
but if a similar pattern surfaces for another PIE-asm buffer
(e.g., FFT twiddles, channelizer coefficients), the same workaround
applies: alloc FIRST.

See `project_heap_position_decode_bug.md`, `project_p4_errata_status.md`.

---

## Non-opportunities (deprioritised)

- **Resampler closer to 0.5 ms floor:** Current 2.67 ms is dominated
  by per-sample delay-line shift in C and the IQ deinterleave/store.
  Floor is ~1.2 ms, not 0.5. With #1 (drop resampler) this is moot.
- **BCH soft-decision:** Chase-2 adds ~2× BCH time for ~2-3 frames;
  net loss against #4.
- **Pipeline split (Stage A/B across cores):** Worse than two-worker
  parallel (67 ms/burst limited).
- **PLL convergence shortcut:** ~5 µs/burst total, not a target.

---

## References

- gr-iridium `examples/rtl-sdr.conf` — 2.0 MSPS reference rate
- gr-iridium `fft_burst_tagger_impl.cc:215-280` — volk-vectorised EMA
- esp-dsp `modules/fft/float/dsps_fft4r_fc32_arp4.S` — radix-4 PIE FFT
- esp-dsp `modules/fft/fixed/dsps_fft2r_sc16_arp4.S` — sc16 PIE FFT
- `pie_fft_diff_test.c` — proven bit-exact PIE-vs-scalar harness
- Memory notes: `feedback_pie_fft_swap_needs_validation.md`,
  `feedback_q15_incremental_phasor_decays.md`,
  `project_tagger_postpad_coupled_to_multiframe.md`,
  `feedback_usb_pool_size_not_throttle.md`

## Key file paths

- `common/iridium_decoder/fft_burst_tagger.c` (EMA/mag loops, PIE candidate)
- `common/iridium_decoder/fft_sc16_2048.c` (already PIE; sc16 swap reference)
- `common/iridium_decoder/uw_correlator.c:444` (fft4r candidate)
- `common/iridium_decoder/burst_pipeline.c:457` (multi-frame gate)
- `p4-usb-host/main/dsp_processor.h:20` (FS_IN_HZ)
- `p4-usb-host/main/dsp_processor.c:201` (baseline_history PSRAM alloc)
