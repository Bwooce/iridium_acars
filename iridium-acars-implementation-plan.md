# Iridium ACARS Decoding Stack — Implementation Plan

Tracks the concrete implementation steps for the
[Iridium ACARS Decoding Stack Design](./iridium-acars-decoding-stack-design.md).

## Current status (2026-05-22)

**Bit-correctness is solved; the wideband C decoder produces clean
frames on both host and P4.** Focus has moved to performance —
specifically, closing the gap to real-time on a single ESP32-P4.

Smoke run against the ALBQ fixture (72 tagged bursts) — current numbers:

| | gri (ground truth) | P4 RAW_IRIDIUM smoke |
|---|---|---|
| Bursts tagged | 65 | 72 |
| Frames decoded by device | — | 71 |
| Matched vs gri (within ±125 k samples / ±20 kHz) | — | **61 (recall 93.8%, precision 85.9%)** |
| BCH outcomes — clean / corrected / failed / skipped | — | 0 / **29** / 26 / 16 |
| Raw BER over matched bursts | 0% | **1.91%** |

Host bch/qpsk/burst-pipeline regression suite: **20/20 ctest pass**.

Bit-correctness milestones landed in May 2026:

- **Task #71** (47% BER mystery) was a reference-frame mismatch in
  the GOLDEN comparator, not a decode bug. Real host BER on the
  same fixture was 3.18% all along; the BER number had been
  comparing pre- and post-BCH-decoded bit streams that didn't
  line up at the same symbol boundary.
- **Task #76**: extended `SYNC_SEARCH_LEN` from 840 → 2520 samples
  in `burst_pipeline.c::try_decode_frame` to compensate for our
  ~14k-sample wider tagger window. Host BER 3.18 → 1.33%, P4 BER
  2.54 → 1.76%.
- **Task #79**: implemented gri's `handle_multiple_frames_per_burst`
  multi-frame decode loop. P4 BCH-corrected frames 25 → 29; host
  recall 84.6 → 96.9%.
- **gri-alignment audit (May 2026)** removed several test-tailoring
  workarounds (4-rotation UW search, 0.6 correlation fallback,
  6 dB SNR filter); verified D13, RRC, PLL math match gri exactly.
  See memory note `project_gri_alignment_audit_2026_05.md`.

### Performance status

Sustained per-burst processing time on the ALBQ smoke corpus
(2026-05-22, after the session-long round of perf wins):

| metric | value |
|---|---|
| avg per-burst worker time | **103.4 ms** |
| sustained burst throughput | ~9.7 bursts/sec |
| **gap to single-channel real-time (~11 ms/burst)** | **~9× too slow** |

Per-burst stage breakdown (avg µs):

| stage | µs |
|---|---|
| extract (just L2 invalidate, task #64 landed) | 17 |
| decim (chunked PIE FIR, reads circular_buf directly) | 34 591 |
| pipeline | 58 116 |
| ↳ D13 start-finder | 1 343 |
| ↳ CFO estimator (N=4096 scalar float FFT) | 5 700 |
| ↳ peak-phase prerot | 1 949 |
| ↳ RRC matched filter | 11 870 |
| ↳ first try_decode_frame call | 22 320 |
| ↳ retry try_decode_frame calls | 18 300 × ~2.3/burst |
| bch + log + queue (Chase-2 BCH, IDA/SBD/libacars dispatch) | 3 735 |

Session wins (this round, May 21-22):

- **PIE float FFT for UW matched filter** (commit `da8163a`):
  ~5 ms/burst saved on the three N=2048 FFTs inside
  `uw_correlator_find`.
- **Float-only UW magnitude search** (commit `6531c2d`): removed
  per-iteration soft-double arithmetic in the magnitude-search
  loops; -15 ms/burst (the single biggest perf win this session).
- **UW correlator buffers → internal SRAM** (commit `40ff0c9`).
- **CFO FFT scratch → internal SRAM** (commit `3b38d67`).
- **Eliminate `s_extract_buf`** (commit `a5553f0`, task #64):
  decim chunk loop reads directly from circular_buf via
  `signal_buffer_read_chunk`; the per-burst PSRAM-write extract
  stage drops from 7.5 ms to 17 µs.
- **PIE FFT bit-exact diff harness** (commit `7033a48`, task #67):
  on-target validation that `dsps_fft2r_fc32_arp4` matches the
  scalar reference within float epsilon. Disproved the
  long-standing "PIE FFT broke UW" claim in memory; the earlier
  regression was a wrapping bug, not the FFT math.

Net session perf: ~150 → 103.4 ms/burst (-31%), no decode
regression at any commit.

### What's NOT yet real-time

USB throughput (task #74) is closed — we're at 4.57 MB/s sustained
(was 0.85 MB/s). That's 89% of the 5.12 MB/s target. The remaining
gap is structural: ~85 ms/burst worker time vs ~11 ms target for
single-channel real-time. The dominant remaining cost is the
matched-filter + multi-frame retry loop on Core 1
(~63 ms/burst, intrinsically serial within one burst).

Approaches considered for the worker gap, with current verdicts
(updated 2026-05-23):

| approach | viability | notes |
|---|---|---|
| Tighten tagger window (task #70) | rejected | Sweep shows every clip costs recall; multi-frame loop needs the 16 ms post-pad. Memory note `project_tagger_postpad_coupled_to_multiframe.md`. |
| PIE FFT for CFO (N=4096) | retried, deferred | esp-dsp's single-instance init forced manual twiddle handling; PIE result had subtle float divergence that caused -1 BCH frame regression. Needs bit-exact diff harness like #67 before retry. |
| Chunked RRC PIE FIR | failed twice | Static-BSS variant broke boot (PSRAM DMA pool reserve), heap-alloc variant produced corrupt output (streaming FIR semantics quirk in arp4). Worth another look. |
| Two-worker burst pool | blocked (memory) | Each instance needs ~34 KB internal SRAM. Largest contiguous block after init = 31 KB. Doesn't fit without further freeing. |
| Two-worker parallel resample | architecture built, blocked (L2 contention) | Within-chunk split implemented and validated bit-exact (commit `aad9735`). At split>0, FFT cost on Core 0 jumps 263→443 µs. Default split_pct=0 ships. See `docs/split-resample-sweep-2026-05-22.md`. |
| Pipelined tagger (FFT step N+1 ‖ post-FFT step N) | attempted, exposed silicon bug | Refactor surfaced a latent P4 v1.3 PIE position-dependent corruption bug (matched 61→44 on heap shift). Workaround landed (`resample_256_to_250_alloc_coeffs` early in boot, commit `cfd2739`). Pipelining itself parked. Memory note `project_heap_position_decode_bug.md`. |
| Conditional multi-frame iteration | tried, reverted | Saved ~40 ms/burst but cost matched 61→56 and BCH 29→25 — quality regression. Quality-must-not-be-compromised policy. |
| Drop input to 2.0 MSPS (task #1 in opt doc) | available, deferred | Deletes the resampler entirely, closes the 11% USB gap. Trade-off: ~22% less spectrum captured. Worth it only if real-RF reveals the gap is binding. |

See `Forward plan` below for the concrete next steps.

---

## Forward plan

Bit-correctness is done. USB ingest is now at **4.57 MB/s** on
LIVE_SDR (was 0.85 MB/s — task #74 closed via the resample +
coeffs-internal + PSRAM-stacks work). The remaining gap is **~11%
short of true realtime** (5.12 MB/s at 2.56 MSPS) and the existing
~85 ms/burst decode budget. The plan is sequenced so each step is
testable in isolation and the next one is informed by what the
previous one measured.

### 1. ~~Grow the PSRAM ring buffer~~ — DONE

Burst queue is now 1024 entries × 28 B in PSRAM. Plenty of slack
for transient overloads; `qmax` stays low even at peak fixture
rate.

### 2. ~~Restore USB SDR throughput~~ — DONE (LIVE_SDR ≈ 4.57 MB/s)

Resample-and-related work brought the consumer-side pipeline from
~0.85 MB/s to 4.57 MB/s, well within the "above 4.5 MB/s no drops"
acceptance criterion. The 11% gap to true 5.12 MB/s is from the
remaining ~2.7 ms resample on Core 1.

### 3. Live RF validation (Phase 4)

Highest leverage next step — actual antenna signal is what
validates the whole stack end-to-end:

- Acquire 1620 MHz QFH antenna + Nooelec SAWbird+ IR LNA.
- Switch RTL-SDR from AGC to manual gain at ~35 dB (task #62
  also has bias-tee + manual-gain NVS config).
- First live end-to-end Iridium ACARS decode.
- One-hour soak; compare frame rate vs upstream
  `iridium-toolkit` on the same recording.

This validates the assumed burst rate, gives real-input dynamics
data (signal_buffer overruns? worker queue depth?), and shows
whether the 89% realtime ceiling is binding in practice or
whether typical traffic is well below saturation anyway.

### 4. ~~Conditional multi-frame decode~~ — TRIED, REVERTED

Tested in this session: gating frames 2+ on first-frame outcome
saved ~40 ms/burst but cost matched=61→56, BCH=29→25 — quality
regression, reverted. User policy: quality must not be
compromised. Not retrying without a different gate criterion.

### 5. ~~Two-worker parallel resample~~ — BLOCKED (L2 contention)

Architecture built and validated bit-exact, but split>0 collapses
FFT cost on Core 0 (263 → 443 µs at split=25, ~+71%) due to
L2 cache contention that isn't fixable today. Investigated:

- s_resamp PSRAM writes → NOT the evictor (verified by redirecting
  to internal SRAM — FFT still slow).
- L2 cache size 256→128 KB → frees enough heap but USB throughput
  regresses 13%.
- s_conv → internal SRAM → would help but needs heap headroom we
  don't have.

Closed out 2026-05-22; see `docs/split-resample-sweep-2026-05-22.md`.

### 6. ~~Two-worker burst decode pool~~ — BLOCKED (internal SRAM)

Per-instance memory: ~34 KB internal SRAM for chunk_iq + decim
scratches alone, plus a uw_correlator API refactor to make its
static-global FFT scratches per-instance. Largest contiguous
internal SRAM block after init: 31 KB. **Doesn't fit** without
freeing more internal SRAM first — and the two ways tried (peaks-
PSRAM, L2 reduction) both regressed something else.

### 6.5 P4 v1.3 PIE position-dependent corruption — WORKAROUND LANDED

Discovered 2026-05-23 while attempting pipelined tagger. PIE asm
`esp.vld.128.ip` produces silently wrong output when
`s_coeffs_pp` lands at certain HP-SRAM addresses (e.g.,
0x4ff737e0). Decode collapses matched=61→44 with no other
symptom. Not in the official ESP32-P4 v1.3 errata list; fits the
pattern of "silent bus/address corruption on v1.3 silicon"
alongside MSPI-750, APM-560, and IDF #18235.

Workaround in commit `cfd2739`: new `resample_256_to_250_alloc_coeffs()`
called as FIRST internal-SRAM consumer in boot, deterministically
placing the 4 KB buffer at 0x4ff3e330 (small-RAM region). Future
heap-growing changes are now position-independent for the
resampler. See memory note `project_heap_position_decode_bug.md`
and the errata audit `project_p4_errata_status.md`.

The pipelined-tagger refactor itself is parked — it's blocked
behind a clean way to extract mag/detect/EMA into a separately
dispatched function. The expected gain (~14% Core 0 freed,
DSP/frame 548→470 µs) is real but the refactor is non-trivial.

### 7. RRC scratch boot-time alloc (task #58 follow-up)

The RRC FIR currently lazy-allocates per-burst and silently
falls back to PSRAM (PIE disabled) due to internal-heap
fragmentation at hot-path time. Pre-allocating at worker
init — before heavy USB allocs fragment the heap — should
recover ~5-8 ms/burst.

Previous attempts both failed (BSS variant broke boot via
PSRAM DMA pool reserve; heap-alloc chunked variant produced
corrupt output, likely an arp4 streaming-FIR quirk). Needs
diagnostic work to identify the streaming-state issue, then
the heap-alloc pre-allocation pattern can land.

### 8. Live-SDR smoke watchdog (task #72)

Wire the `usb_host_lib_set_root_port_power` cycle into the smoke
loop as a startup watchdog: if no device enumerates within 6 s,
cycle the root port and retry up to 3 times. Recovers from the
stuck-enumeration state that LIVE_SDR currently hits on re-flash.

### 9. Drop input rate to 2.0 MSPS (task #1 in opt doc)

Last-resort structural change if Phase 4 reveals 4.57 MB/s isn't
enough for typical-traffic operation. Eliminates the resampler
entirely, deletes the 2.7 ms/dispatch Core 1 cost. gr-iridium
ships at 2.0 MSPS by default so we'd be standards-aligned.
Trade-off: ~22% less of the Iridium spectrum captured per receive
window. Skip unless real-RF data shows the gap matters.

### 9. Productisation (post-RF)

Sequenced after we have a real signal flowing through the stack:

- LCW sub-type body parsing (ISY, IIU, I36, IIP, IVO, IBC) —
  IBC done (sv_id, beam_id parsed, task #34); IRA done
  (location parsing); IMS header done; TL deferred (task #80).
- ESP32-C6 Wi-Fi/Thread output path — see
  `docs/c6-companion-firmware-design.md` for the design.
- NVS-backed runtime config (LO, sample rate, station ID).
- OTA firmware updates via `esp_https_ota`.
- README with build/flash instructions and architecture overview.
- CI/CD for firmware builds.

---

## What's already done

Snapshot of completed milestones. Detail is in git history and commit
messages; this section only states what works today.

- **USB host + RTL-SDR v4 driver**: async multi-buffer transfer,
  AXI-GDMA push, 4.88 MB/s sustained (100.5% real-time, 0 drops at
  2.56 MSPS). R828D tuner properly detected and initialised
  (28.8 MHz XTAL, correct init array, multi-byte I2C). Software-only
  USB recovery via root-port-power cycling.
- **Dual-core pipeline**: USB ingest on Core 0, DSP+demod+decode on
  Core 1, 4 MB PSRAM lookback ringbuffer, queue between cores. Pre-
  allocated PSRAM stage buffers (no per-burst malloc).
- **DSP detector path (legacy/active)**: polyphase channelizer
  (M=64, L=1024-tap Hamming-windowed sinc) with PIE/SIMD-optimised
  Q14 path (D20 step 3c, all-phases asm via `esp.lp.setup`). Hits
  real-time at 2.56 MSPS with ~2% headroom on a 2048-sample frame.
  Per-channel detector: integer EMA baseline + 40× threshold (Q15
  weights), DSP/frame 407 µs in worst-case smoke; production ~300 µs.
- **DSP detector path (wideband, on-target)**:
  `fft_burst_tagger` + `direct_if_decim` + `resample_256_to_250` +
  `fft_sc16_2048`. Active runtime front end on P4. End-to-end bit
  output now matches gri: 1.91% raw BER on the P4 smoke corpus,
  1.33% on host. FIR design gri-aligned since `8862232`. Worker
  reads circular_buf directly via `signal_buffer_read_chunk` (no
  PSRAM extract intermediate, task #64).
- **Burst pipeline (`burst_pipeline.c`)**: D13 start-finder (Kaiser LP
  at 2.5 kHz, 28% threshold), CFO estimator (square-then-FFT,
  Blackman, 4096-pt scalar), pre-rotate, RRC β=0.4 51-tap @ 10 sps,
  UW cross-correlator (2048-pt PIE float FFT, 271-sym sync
  reference), gri-aligned decim+demod. Multi-frame loop sweeps
  additional search starts after the first attempt (task #79;
  contributes ~4 BCH frames on the smoke corpus). Bit output
  matches gri at 1.91% raw BER (P4) / 1.33% (host).
- **DQPSK demod**: first-order PLL (α=0.2, β=0). PLL math, DQPSK
  symbol map, and quadrant→bit decoding all verified byte-identical
  to gri's source (`iridium_qpsk_demod_impl.cc`). UW correlation
  works — first 24 bits match exactly on every matched burst.
- **BCH(31,21)**: hard-decision block correction; soft-decision
  (Chase-2) implemented but not enabled by default. With upstream
  bits now correct (task #71 resolved), the BCH-corrected counter
  is a meaningful decode-quality signal — 29 frames on the ALBQ
  smoke corpus, 27-29 across recent perf-tuning commits.
- **Frame layer**: IDA decode (de-interleave + BCH), CRC-16-CCITT
  validation, SBD reassembler, libacars (vendored) for ACARS parsing
  with multi-segment reassembly via `la_acars_parse_and_reassemble`.
- **Comparison tooling**: `tests/scripts/dsp_compare.py` (single CLI,
  three modes, manifest-aware alignment, NPZ regression vectors).
  Per-stage NMSE / phase coherence dumps for any matched burst.
- **GOLDEN bit-level compare**:
  `tests/fixtures/fixture_albq_golden_bits.h` (auto-generated from
  gri's RAW: output), with the device-side compare in
  `worker_core1.c::golden_compare_burst` (commit `4ca6af9`) and the
  host-side compare in `test_pipeline_wideband_albq.c` (commit
  `a32c705`). Same tolerance window, same buckets — output is
  directly comparable across host and P4.
- **Tests**: target smoke (synthetic + real-IQ + LIVE_SDR) + 20 host
  ctest cases. All ctest green. `test_pipeline_wideband_albq` runs
  the full pipeline + GOLDEN compare; it always "passes" because
  it's a measurement test, but the BER it reports is the metric we
  watch.

---

## Reference: pipeline vs gr-iridium

| Stage | gr-iridium | us | Status |
|---|---|---|---|
| SDR ingest | USRP/HackRF 6–12 MSPS float | RTL-SDR 2.56 MSPS uint8 → int16 | Hardware delta |
| Wideband detect | `fft_burst_tagger` (spectrogram peak + persistence + hysteresis) | Same (C port) | Window ~16% longer than gri; task #70 sweep confirmed clipping costs recall — multi-frame loop consumes the trailing tail. Memory note `project_tagger_postpad_coupled_to_multiframe.md`. |
| Per-burst extract | Burst tags drive `burst_downmix` | Tag → rotate → 10× Kaiser decim → 250 ksps | Equivalent in design |
| Coarse freq | Channel selection | Tag freq → DC mix | Equivalent |
| Decimation | `firdes.low_pass_2(1, 2.5e6, 20e3, 40e3, 40)` = 141 Kaiser taps | Same (141 design + 3 zero-pad to 144 for PIE) | **Equivalent (after `8862232`)** |
| Start finder | `start_finder_fir`: Kaiser LP @ 2.5 kHz, ~182 taps, 28% threshold | Same scaled to rate (41 taps @ 50 ksps, same cutoff/fs ratio, 28%) | Equivalent in design |
| RRC | β=0.4, 51 taps @ 10 sps | Same | Equivalent in design |
| CFO | Square-then-FFT, 64 sym × 16× zero-pad, Blackman | Same: 280-sample × ~15× zero-pad → 4096-pt FFT, Blackman | Equivalent in design |
| Sub-sample timing | Sync-pos integer decim | Linear interpolation by `uw_res.correction` | Equivalent in design (was suspect for #71 but root cause was elsewhere — comparator misalignment, since resolved) |
| Decimate-by-10 | `decimate(in, sps=10)` → picks samples 0, 10, 20, … | Same effective pick via `POST_CORR_DECIM=5` + `samples_2sps[i*4+0]` | Equivalent in design |
| Power-decay truncation | Stops symbols at 3 consecutive samples < max/8 | **Missing — emits full 382 bits always** | Task #73 (deferred; doesn't affect bit-correctness inside gri's frame range, only output length) |
| DQPSK PLL | α=0.2, β=0 (first-order) | Same — math verified byte-identical | Equivalent |
| UW check | Per-symbol soft sum, threshold ≤ 2 | Exact-match per rotation, complex-correlation fallback ≥ 0.6 | Different shape, equivalent strictness; UW alignment is correct in both (first 24 bits always match gri) |
| DQPSK decode | `decode_deqpsk` with mapping {0, 2, 3, 1} | Same `DQPSK_MAP[]` | Equivalent (verified) |
| Bit emission | `(s>>1)&1`, `s&1` per symbol | Same | Equivalent (verified) |
| **End-to-end bits vs gri** | — | **1.91% raw BER (P4 smoke), 1.33% (host)** | OK — task #71 resolved (was a comparator misalignment, not a decode bug) |
| BCH | Hard-decision | Hard-decision (Chase-2 available, off by default) | Equivalent in design. With clean bits now landing, the per-burst BCH-corrected counter is a meaningful decode-quality signal (29 frames on the ALBQ smoke corpus). |
| Frame parse | `iridium-parser.py` | `iridium_frame.c` + LCW classifier + IDA decode | Equivalent on IDA/SBD path |
| ACARS | `libacars` via iridium-toolkit | `libacars_idf` (vendored) with multi-segment reassembly | Equivalent |

---

## Hardware notes

### USB host VBUS on Waveshare ESP32-P4-Nano: not a GPIO

Verified against the Waveshare ESP32-P4-NANO schematic (summary at
`docs/p4-nano-board-schematic-summary.md`):

- The USB Type-A host port (H3) VBUS is driven by U2 (DIO7003HEST5
  load switch) from the always-on VCC_5V rail. U2's `EN` pin is
  pulled to VCC_5V via R2 = 10K with **no GPIO routing**. Two NC
  pads (R11, R12) exist for a future hardware mod but are
  unpopulated as shipped.
- LED1 (the USB overcurrent indicator) is wired only to U2's `FLG`
  output, not back to any P4 GPIO — so the firmware can't even
  detect overcurrent in software.
- GPIO 45 is SD-card power enable on the Nano (drives Q1 AO3401
  P-MOSFET gate controlling SD1_VDD). Nothing to do with USB.
- GPIO 54 controls C6_CHIP_PU (the C6 companion's enable line) via
  0R R39; no USB role.

**Implication:** `usb_host_lib_set_root_port_power(false)` only
flips a bit inside the P4's USB-OTG controller. It cannot cut
physical VBUS to the SDR on this board. The software fallback path
below still works for most stuck-device cases.

### Software-only USB recovery (despite no VBUS GPIO)

The ESP-IDF host stack provides a software path to recover stuck
devices even with VBUS hardwired-on:

- `usb_host_lib_set_root_port_power(false/true)` halts SOFs and
  re-evaluates attach state from the DWC OTG controller's perspective
  — equivalent to a USB cable unplug from the host's side. On boards
  with EN wired to a GPIO this would also drop physical VBUS; on the
  Nano it doesn't, but controller-side disconnect+reattach is enough
  to recover most stuck-device states without physical intervention.
- Firmware uses this in two places:
  1. **Boot**: install with `root_port_unpowered=true`, then explicit
     `set_root_port_power(true)` — forces fresh attach every reset.
  2. **Watchdog**: if no device enumerates within 6 s, cycle
     `power(false)` → 500 ms → `power(true)`, up to 3 attempts.

True hardware power-cycle of the attached USB device still requires
physical unplug. The software recovery covers the common cases that
previously needed physical intervention.

### Tuner gain mode

`librtlsdr.c` currently selects R820T2/R828D **automatic gain control**
(AGC) at boot. Acceptable for development without an antenna. For
live Iridium reception once Phase 4 hardware is installed, switch to
manual gain at ~35 dB and verify noise-floor stability via the
detector logs. AGC's reactive gain pumping shifts the constellation
and breaks the PLL phase tracker during bursts.

### Internal SRAM accounting (P4 has 768 KB L2MEM, not 283 KB)

**Physical:**

| Region | Address | Size | Linker-visible |
|---|---|---|---|
| L2MEM (HP) | 0x4FF00000 | 768 KiB | 435 KiB (`sram_low` 179 + `sram_high` 256) |
| TCM | (HP CPU-local) | 8 KiB | — (no DMA) |
| LP SRAM | 0x50108000 | 32 KiB | 32 KiB |
| HP SPM | 0x30100000 | 8 KiB | 8 KiB |

**L2MEM (768 KiB) breakdown** (per `esp-idf/components/esp_system/ld/esp32p4/memory.ld.in:30-43`):

| Region | Address range | Size | Knob |
|---|---|---|---|
| `sram_low` (firmware IRAM + DRAM low) | `0x4FF00000` .. `0x4FF2CBD0` | **179 KiB** | — |
| 2nd-stage bootloader `iram_loader_seg` | `0x4FF2CBD0` .. `0x4FF40000` | **77 KiB** | Hard-coded constant in `memory.ld.in`; no Kconfig knob. Stuck. |
| `sram_high` (firmware DRAM high + heap) | `0x4FF40000` .. `0x4FF80000` | **256 KiB** | Computed as `0x80000 - CACHE_L2_CACHE_SIZE` — shrinks if cache grows |
| L2 cache backing | `0x4FF80000` .. `0x4FFC0000` | **256 KiB** | `CONFIG_CACHE_L2_CACHE_256KB=y` (choice of 128 / 256 / 512 KiB) |
| **Total** | | **768 KiB** | |

Firmware static layout (from `p4-usb-host.map`, lives inside `sram_low` + `sram_high`):

| Section | Size | Notes |
|---|---|---|
| `.iram0.text` | 65.6 KiB | Code in SRAM for hot paths |
| `.dram0.data` | 13.0 KiB | Initialised globals |
| `.dram0.bss` + `.dram1.bss` | **184.7 KiB** | See `docs/p4-bss-audit.md` for per-symbol breakdown — ~150 KiB firmware-owned + ~35 KiB library |
| Flash-mapped `.flash.text` | 228 KiB | No SRAM cost; cached on demand via L2 |

Heap (what IDF heap manager exposes): **283 KiB total** = RETENT_RAM 65 + RAM small 18 + RAM large 162 + RTCRAM 31 + SPM 7. Of those, only the 162 KiB "RAM large" chunk is contiguous enough to back a useful `SPIRAM_MALLOC_RESERVE_INTERNAL`.

**Knobs to reclaim SRAM:**

- **`.bss` migrations** (audit's recommended wins): ~140 KiB. See `docs/p4-bss-audit.md`. **The actionable lever.**
- L2 cache 256 → 128 KiB: +128 KiB SRAM but DSP working-set thrash risk, especially after `.bss`-to-PSRAM moves increase cache pressure. Probably stay at 256.
- 2nd-stage bootloader `iram_loader_seg` (77 KiB): hard-coded, would need custom bootloader. Not worth it.
- FreeRTOS trace facility / runtime stats: ~500 B saving, real diagnostic loss.
- Memory protection / ULP reserve / RTC reserve: 0 or marginal.

Flash-mapped (no SRAM cost): `.flash.text` 228 KiB. Mapped via L2 cache, evicts on pressure.

**Of the 184.7 KiB `.bss`, ~150 KiB is firmware-owned and movable to PSRAM** via `EXT_RAM_BSS_ATTR`. The audit identifies the top 35 firmware symbols ≥100 B and classifies each as `CAN_MOVE`, `MUST_STAY_INTERNAL` (PIE-asm bound), or library-out-of-scope. Recommended first three wins reclaim ~140 KiB; see `docs/p4-bss-audit.md`.

**Live demand on DMA-capable internal SRAM (LIVE_SDR build):**

| Consumer | Size | DMA-required? |
|---|---|---|
| `ingest_core1` `s_raw[2]` | 32 KB | yes (USB DWC OTG-HS DMA target) |
| `worker_core1` `s_chunk_iq` | 16 KB | yes (PIE FIR `esp.vld.128.ip` can't read PSRAM) |
| `worker_core1` `s_decim_scr_*` (4 buffers) | ~32 KB | yes (PIE FIR) |
| `uw_correlator` RRC + start-finder PIE state | ~32 KB | yes (PIE FIR coeffs) |
| `fft_sc16_2048` twiddle | 8 KB | yes (FFT inner loop) |
| `fft_burst_tagger` working .bss | ~30 KB | yes (fed by `fft_sc16_2048`) |
| `esp_libusb` async transfer pool | 8 × 8 KB = 64 KB | yes (USB DMA) |

**Demand: ~214 KB DMA-capable. Reserve: 144 KB. Free pre-stream: 70 KB.**
The gap closed once `s_conv` (64 KB) moved to PSRAM (commit 88112bc).
After applying the audit's first three wins, the heap manager will see
roughly +140 KiB more DMA-capable headroom, comfortably accommodating
the original 8 × 16 KB transfer pool plus margin.

Levers for future tightening (descending impact):

1. **Apply the `.bss` audit's three wins** — reclaim ~140 KiB by
   annotating `syn_ra` (bch_decoder), six FFT scratch buffers
   (uw_correlator), and the float twiddle/window precomputes
   (uw_correlator) with `EXT_RAM_BSS_ATTR`.
2. **Convert read-only-after-init tables to `static const`** — the
   bit-reversal, twiddle, and window tables in `uw_correlator.c`
   are all computable at compile time. Moving them to
   `.flash.rodata` frees SRAM entirely (no PSRAM space used). One
   evening's refactor.
3. **Drop L2 cache from 256 KB back to 128 KB** — saves 64 KB but
   risks re-introducing the cache thrashing the bigger cache was
   added to fix. Measure first.

### esp-dsp 1.8.1 upstream bugs

Two issues in the bundled esp-dsp version that the worker works
around (worth filing upstream):

1. **`dsps_resampler_mr_init` rejects `samplerate_factor < 1`** but
   does not propagate the error — leaves the struct uninitialised,
   then `_exec()` faults at NULL+0x10. Workaround: bypass the `_mr_`
   wrapper and call the underlying `dsps_firmr_init_s16` /
   `dsps_firmr_s16` directly.
2. **`dsps_fird_s16_arp4.S` returns uninitialised register `a6`**
   instead of the output sample count. Observed as a constant -256
   across repeated calls. The function does write the output buffer
   correctly, it just doesn't report what it wrote. Workaround: pass
   the correct output length (`input/decim`, note that's the
   convention this API uses, vs `dsps_firmr_s16` which takes input
   length) and compute the count locally.

### Throughput optimisation: things tried that didn't help

Recorded so they're not retried:

- `aligned(64)` on the five hot DSP buffers regresses the EMA stage
  by ~170 µs/frame — five 8 KB buffers all at the same 64-byte
  alignment land in the same L1 D cache sets and conflict-thrash.
  Kept `aligned(16)`.
- `esp.vmul.f32` / `esp.vadd.f32` do not exist on ESP32-P4 — PIE on
  P4 covers s8/s16/s32 and complex variants but no generic f32
  vector arithmetic. Verified by enumerating `xesppie.S`.
- Shift-only detector threshold (`mag >> 5 > baseline`, 15 dB at 32×)
  produces too many priming-noise false positives. Settled on uint32
  multiply with 40× threshold.

---

## Deferred

Items that have been considered and explicitly deferred. Listed
only so they're not re-litigated.

- **gri-461 multi-frame edge case**: the one remaining burst out of
  65 not decoded by path C. Per-iter CFO inside the multi-frame loop
  (gri's `process_next_frame` model) recovers nothing on this corpus
  and regressed path A by 10/99 when last tried. Reconsider only if
  a new fixture exposes more sub-frames of this shape.
- **Option B fs=2.667 MHz channelizer rate change**: measured +0.5 dB
  mean recovery (much less than the +3 dB predicted, because Doppler
  ±40 kHz at 1.6 GHz keeps actual carriers off the nominal grid
  anyway). Substantial refactor for marginal benefit; deferred
  indefinitely. Measurement infrastructure (build_albq_raw_2667.py +
  fixture_albq_raw_2667.h) preserved for any future revisit.
- **Polyphase channelizer further optimisation**: the wideband path
  is the chosen front end. Channelizer remains as fallback only;
  no further tuning work on it.
- **Soft-decision BCH on by default**: Chase-2 decoder exists but
  costs 2–5× hard-decision and the wideband path delivers clean
  enough bursts that the soft margin doesn't pay. Revisit if real-RF
  data shows BCH at the SNR margin.
