# Iridium ACARS Decoding Stack — Implementation Plan

Tracks the concrete implementation steps for the
[Iridium ACARS Decoding Stack Design](./iridium-acars-decoding-stack-design.md).

## Current status (2026-05-20)

**The wideband C decoder runs end-to-end on both host and P4 but
emits the wrong bits.** First bit-wise comparison against gr-iridium
ground truth (commit `a32c705`, host-side GOLDEN compare wired into
`test_pipeline_wideband_albq`) shows:

| | gri (ground truth) | host wideband | P4 RAW_IRIDIUM smoke |
|---|---|---|---|
| Bursts tagged | 99 | 133 | 72 |
| Demod produced bits | 65 | 62 | 56 |
| Matched (±125 k samples / ±20 kHz vs gri) | — | 25 | 22 |
| **Raw BER over matched bursts** | 0% | **46.92%** | **47.89%** |
| Per-burst histogram (exact / close <5% / partial <25% / divergent ≥25%) | — | 0/0/2/23 | 0/0/1/21 |

Host and P4 are the **same broken** — they run the same shared C
code, so the P4 is not a regression vs host. Both diverge from gri
past the unique-word in essentially every matched burst. The PLL
locks on the UW (first 24 bits match exactly in every BITDUMP row),
then the post-UW bit stream is near-random vs gri.

**What earlier "59-decoded" / "62-decoded" numbers actually measured.**
Both counted `demod_ok` returns from `qpsk_demod_process()`, which is
set whenever the UW correlator passes. No prior measurement checked
whether the emitted bits matched gri. The new GOLDEN compare is the
first one that does.

**Confirmed gr-iridium-equivalent (numerical, per-sample).**

- `direct_if_decim` Kaiser LPF design (after commit `8862232` fixed
  the transition-width mistake): 141 taps at trans=40 kHz matches
  gri's `firdes.low_pass_2`.
- Per-sample `07b_post_rotate_cut_250k.cf32` matches gri's
  `signal-filtered-deci-cut-start-shift-rrc-rotate-cut-390.cfile`
  to within ±5° phase diff through symbol 170+ on burst 390.
- DQPSK_MAP, hard-decision quadrant→symbol mapping, and bit-pair
  emission are byte-identical to gri's `decode_deqpsk` /
  `map_symbols_to_bits` (verified in `iridium_qpsk_demod_impl.cc`).
- PLL math: ours `phi_hat *= exp(-j·α·angle)` = gri's
  `phiHat *= conj(pow(phiHatT, α))`. Same α = 0.2, BETA = 0.

**Known gri-vs-us behavioural differences (not yet fixed).**

- gri's `demod_qpsk` truncates the symbol stream when 3 consecutive
  symbols have `|sample| < max/8` — that's why gri's `n_bits` varies
  per burst (24, 142, 294, 336, 382) while ours is always 382. We
  emit the full 382 regardless of burst length.
- Tagger window length: our PDU runs ~16% longer than gri's for the
  same physical burst (burst 390: ours 106490 vs gri 92160 input
  samples). Task #70.
- Per-burst (large-freq-offset bursts like gri_id=70 at −342 kHz):
  qpsk_demod input shows modulation collapse around symbol 60 —
  signal magnitude stays high but phase parks in `(-,-)` quadrant.
  Does not happen on small-offset bursts (gri_id=390 at +17 kHz).

**Open root cause.** Even when the upstream signal matches gri
per-sample (burst 390), the bits diverge ~46% vs gri. Same input,
nominally same PLL math, different bits. Hypothesis: a sub-sample
sampling-position skew between our decimate(by-10) and gri's, or a
state-history difference in the PLL initial conditions. Task #71.

**Blocker for live RF**: bit-correctness — chasing a live signal is
pointless until the wideband path produces correct bits vs gri on the
fixture.

---

## Forward plan

Bit-correctness first. Performance, productisation, and live RF are
all downstream of the wideband decoder actually producing the right
bits. Sequence is now strictly:

### 1. Get host wideband bits matching gri (task #71) — IN PROGRESS

Until host BER drops from 46.92% to single digits on the ALBQ
fixture, nothing else matters. The investigation infrastructure is
in place:

- `tests/host/test_pipeline_wideband_albq.c` runs the full host
  pipeline + GOLDEN compare (commit `a32c705`).
- `QPSK_DUMP=<path>` env var drops a per-symbol CSV of the first
  UW-locked burst from `qpsk_demod_process` (commit `ba56cee`).
- `tests/scripts/dsp_compare.py --mode stagewise --burst-id <gri_id>`
  reports per-stage NMSE / phase coherence vs gri's `/tmp/signals/`
  dumps.

Working hypotheses, ordered by probability:

1. **Sub-sample sampling-position skew in our decimate-by-10.** Our
   pipeline picks samples `0, 10, 20, …` of the 250-ksps stream;
   gri does the same. Per-sample post-rotate-cut data matches gri
   to ±5° phase diff on burst 390, yet bits diverge 46%. Suggests
   the symbol-center alignment between us and gri is off by a
   different sub-sample fraction even though the samples we pick
   are the same. Possible culprit: our linear `interp_frac`
   interpolation in `burst_pipeline.c:186-199` introduces a tiny
   phase distortion gri doesn't.
2. **Missing power-decay frame truncation.** gri's `demod_qpsk`
   (gr-iridium/lib/iridium_qpsk_demod_impl.cc:216-225) stops
   emitting symbols when 3 consecutive symbols fall below `max/8`.
   We emit the full 382 bits regardless. This doesn't reduce BER
   inside gri's range but explains the length mismatch and lets
   downstream stages run on clean data only.
3. **Per-burst pipeline collapse on large-freq-offset bursts.**
   gri_id=70 (freq_off = −342 kHz) shows post-symbol-60 modulation
   collapse in the QPSK dump. gri_id=390 (freq_off = +17 kHz)
   doesn't. Probably a numerical edge in `direct_if_decim`'s
   rotate-to-DC step at large offsets, but lower priority than
   #1 because #1 affects all bursts.

Each fix is validated by re-running the GOLDEN compare and watching
BER fall.

### 2. Pick up the tagger-window divergence (task #70)

Once bits match in gri's range, the ~14 k-sample tagger-window
overshoot becomes visible as extra trailing garbage bits. Fixing the
tagger to terminate the burst window where gri does will further
tighten the comparison.

### 3. Live-SDR smoke recovery

The `SMOKE_TEST_LIVE_SDR` variant runs against a USB-attached SDR.
It currently crashes the SDR enumeration when re-flashed, because
**USB-A host VBUS on this board is not GPIO-controllable** (the EN
pin of U2 DIO7003HEST5 is hard-pulled to VCC_5V — see
`docs/p4-nano-board-schematic-summary.md`). The software fallback is
`usb_host_lib_set_root_port_power(false/true)` which halts SOFs and
re-triggers attach without dropping physical VBUS; that's enough to
recover the device-side state in most stuck-device cases. Wire that
into the smoke as a startup watchdog.

### 4. On-target performance measurement (deferred)

Only meaningful once bits are correct. Targets and methodology
unchanged from the pre-discovery plan: per-frame `fft_burst_tagger_step`
cost, per-burst `direct_if_decim` + `burst_pipeline` cost, drop rate
under sustained 2.56 MSPS input.

### 5. Tighten Q15 / SIMD where step 4 identifies hot spots (deferred)

Same candidate list as before, deferred until bit-correctness is in
place. Each swap will be validated by re-running both the stagewise
NMSE check AND the host GOLDEN bit compare.

### 6. Live RF validation (Phase 4) — blocked on bits + hardware

- Acquire 1620 MHz QFH antenna + Nooelec SAWbird+ IR LNA.
- Switch RTL-SDR from AGC to manual gain at ~35 dB.
- First live end-to-end Iridium ACARS decode.
- One-hour soak; compare frame rate vs upstream `iridium-toolkit`
  on the same recording.

### 7. Productisation (post-RF)

Sequenced after we have a real signal flowing through the stack:

- LCW sub-type body parsing (ISY, IIU, I36, IIP, IVO, IBC).
- ESP32-C6 Wi-Fi/Thread output path (MQTT or JSON-over-TCP).
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
  `fft_sc16_2048`. Active runtime front end on P4. Produces the
  expected per-stage shape but the end-to-end bit output diverges
  46.92% (host) / 47.89% (P4) from gri (see Current status). FIR
  design now gri-aligned after `8862232`.
- **Burst pipeline (`burst_pipeline.c`)**: D13 start-finder (Kaiser LP
  at 2.5 kHz, 28% threshold), CFO estimator (square-then-FFT,
  Blackman, 4096-pt), pre-rotate, RRC β=0.4 51-tap @ 10 sps, UW cross-
  correlator (1024-pt FFT, 271-sym sync reference), gri-aligned
  decim+demod. Multi-frame loop sweeps additional search starts after
  the first attempt. **Per-sample output matches gri to ±5° phase
  diff at the rotate-cut stage** on burst 390; bit output still
  diverges (root cause active investigation, task #71).
- **DQPSK demod**: first-order PLL (α=0.2, β=0). PLL math, DQPSK
  symbol map, and quadrant→bit decoding all verified byte-identical
  to gri's source (`iridium_qpsk_demod_impl.cc`). UW correlation
  works — first 24 bits match exactly on every matched burst.
- **BCH(31,21)**: hard-decision block correction; soft-decision
  (Chase-2) implemented but not enabled by default. The "bch=2/2
  corrected" counters in golden compare are mostly false-positive
  matches against random data — BCH only protects the 64-bit LCW,
  and random bits land within 3 hamming distance of valid codewords
  often enough to skew the count. Don't trust BCH-corrected as a
  decode-quality signal until the upstream bits are right.
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
| Wideband detect | `fft_burst_tagger` (spectrogram peak + persistence + hysteresis) | Same (C port) | Window ~16% longer than gri (task #70) |
| Per-burst extract | Burst tags drive `burst_downmix` | Tag → rotate → 10× Kaiser decim → 250 ksps | Equivalent in design |
| Coarse freq | Channel selection | Tag freq → DC mix | Equivalent |
| Decimation | `firdes.low_pass_2(1, 2.5e6, 20e3, 40e3, 40)` = 141 Kaiser taps | Same (141 design + 3 zero-pad to 144 for PIE) | **Equivalent (after `8862232`)** |
| Start finder | `start_finder_fir`: Kaiser LP @ 2.5 kHz, ~182 taps, 28% threshold | Same scaled to rate (41 taps @ 50 ksps, same cutoff/fs ratio, 28%) | Equivalent in design |
| RRC | β=0.4, 51 taps @ 10 sps | Same | Equivalent in design |
| CFO | Square-then-FFT, 64 sym × 16× zero-pad, Blackman | Same: 280-sample × ~15× zero-pad → 4096-pt FFT, Blackman | Equivalent in design |
| Sub-sample timing | Sync-pos integer decim | Linear interpolation by `uw_res.correction` | **Diverges (suspect for #71)** |
| Decimate-by-10 | `decimate(in, sps=10)` → picks samples 0, 10, 20, … | Same effective pick via `POST_CORR_DECIM=5` + `samples_2sps[i*4+0]` | Equivalent in design |
| Power-decay truncation | Stops symbols at 3 consecutive samples < max/8 | **Missing — emits full 382 bits always** | Diverges (task #71 sub-item) |
| DQPSK PLL | α=0.2, β=0 (first-order) | Same — math verified byte-identical | Equivalent |
| UW check | Per-symbol soft sum, threshold ≤ 2 | Exact-match per rotation, complex-correlation fallback ≥ 0.6 | Different shape, equivalent strictness; UW alignment is correct in both (first 24 bits always match gri) |
| DQPSK decode | `decode_deqpsk` with mapping {0, 2, 3, 1} | Same `DQPSK_MAP[]` | Equivalent (verified) |
| Bit emission | `(s>>1)&1`, `s&1` per symbol | Same | Equivalent (verified) |
| **End-to-end bits vs gri** | — | **46.92% raw BER (host), 47.89% (P4)** | **Broken** — task #71 |
| BCH | Hard-decision | Hard-decision (Chase-2 available, off by default) | Equivalent in design; "BCH-corrected" counts are dominated by false-positive matches against random data |
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
