# AGENTS.md

Notes for AI assistants (Claude, Gemini, Copilot, etc.) and humans new to
this repo. Project-level conventions live here; longer architectural and
status docs are in [`iridium-acars-decoding-stack-design.md`](./iridium-acars-decoding-stack-design.md)
and [`iridium-acars-implementation-plan.md`](./iridium-acars-implementation-plan.md).

This file replaces the older convention of separate `CLAUDE.md` / `GEMINI.md`
files: one source of truth, all assistants read it.

## What this project is

ESP32-P4 firmware that captures Iridium L-band signals via an RTL-SDR v4,
detects bursts in real time, and decodes ACARS frames carried by Iridium
Short Burst Data. Active code lives under
[`p4-usb-host/`](./p4-usb-host/).

The implementation plan is the source of truth for what's done and what's
next. Read it before changing anything substantial.

## Build and flash

ESP-IDF v6.1 is checked out at `./esp-idf/`. Three helper scripts wrap the
common workflow:

```sh
# Build (uses direct `ninja -C build` for fast incremental builds;
#  falls back to `idf.py build` on first build / when reconfigure is needed).
./scripts/build.sh

# Flash + reset. Auto-detects the first /dev/ttyACM* if no port given.
./scripts/flash.sh                # auto port
./scripts/flash.sh /dev/ttyACM0   # explicit

# Non-interactive serial monitor — reads N seconds of output to stdout
# (grep/awk-pipeable). Use this from automation; idf.py monitor is for humans.
./scripts/monitor.sh <seconds> [/dev/ttyACM0]
```

Build and flash are intentionally split into separate scripts so each can
be authorised independently — don't chain them with `&&` in a single Bash
call.

If you need raw `idf.py` (e.g. `menuconfig`, `clean`, interactive monitor):
`source esp-idf/export.sh` then `cd p4-usb-host`.

## Reading the runtime diagnostics

The firmware logs a status block once per second. Interpretation guide:

```
USB:           rate_inst=X.XX MB/s rate_avg=...   feed_calls=N (avg_per_call us)
USB-XFR:       completed=N short=N (fill=%)       rb_full_drops=N status_err=N
Cycle (Core0): read=us feed=us                    (consumer-loop costs)
Ingest (Core1): convert=us push=us                dispatches=N consumer_waits=N
DSP:           N frames, total=us/frame           [wind fft mag detect base]
Worker:        queued/dropped/processed/skipped   qmax  avg_burst_us
Worker-stages: extract/freq/fir/resamp/demod/bch  (us per processed burst)
```

- **rate_inst** is the throughput over the last 1 s window (use this, not
  rate_avg, which includes startup and is misleading).
- **rb_full_drops > 0** means the DSP task can't drain the USB ringbuffer
  fast enough — samples are being lost, not just delayed.
- **fill=100% short=0** means the device is sending full-rate; any throughput
  shortfall is host-side.
- **DSP per-frame > 800 μs** is over real-time budget at 2.56 MSPS.
- **Worker dropped/queued ratio** > 0 means burst queue is overflowing.
- **Cycle (Core0) feed** dominates the loop. Read+feed = the per-transfer
  budget; at 16 KB transfers, target real-time is 4.85 MB/s ≈ 3300 μs/cycle.
- **Ingest consumer_waits > 0** means Core 0 stalled because Core 1's ingest
  task hadn't finished converting+pushing the previous slot in time —
  back-pressure from Core 1, not the DSP feed.

## Things to know before changing code

These are bugs and conventions discovered during integration that aren't
obvious from reading the code:

- **`rtlsdr_read_array` / `rtlsdr_write_array` were single-byte-broken.**
  The original port did `*array = data[0]`, copying only the first byte of
  every multi-byte I2C transaction and silently corrupting all R828D init
  writes. Now uses `memcpy(array, data, len)`. If you see "wrong PLL
  divider" or "tuner won't lock" symptoms after touching the I2C path, this
  is the bug to check.
- **RTL-SDR v4 (R828D) XTAL is 28.8 MHz.** Unlike some R828D implementations
  that use 16 MHz (DVB-T2 default), the Blog v4 shares the 28.8 MHz clock
  from the RTL2832U. Setting this incorrectly in `tuner_r82xx.h` prevents
  PLL lock.
- **r82xx_init_array must match upstream.** The vendored copy had 14 of 27
  registers different from the canonical librtlsdr v4 init array — many
  with values that compile-time looked plausible but don't actually lock
  the PLL. When updating any tuner table, diff against
  `librtlsdr/tuner_r82xx.c` line-by-line.
- **Tuner PLL needs settle delays.** The R82XX PLL requires ~10 ms to lock.
  Our port needs explicit `esp_rom_delay_us()` calls between setting
  dividers and checking the lock bit, as the original `usleep` calls were
  missing/commented.
- **DSP buffers need 32-byte padding.** The `arp4` (PIE) assembly kernels in `esp-dsp` have a vector look-ahead bug. All processing buffers must be padded by at least 16 `int16_t` elements (`DSP_PADDING_ELEMS`) to avoid `CHIP_LP_WDT_RESET` or memory faults.
- **`dsps_fird_s16`** takes its `len` as the *output* length (input/decim),
  not the input length. Sibling `dsps_firmr_s16` takes input length. Don't
  confuse them.
- **`dsps_fird_s16_arp4` (P4 PIE asm) returns garbage** — `mv a0, a6` where
  `a6` is never written. The function writes the output buffer correctly,
  it just doesn't report the count. Compute the count locally.
- **`dsps_resampler_mr_init` rejects `samplerate_factor < 1`** without
  propagating the error. If you need to downsample, use the lower-level
  `dsps_firmr_init_s16` directly with explicit `interp/decim`.
- **`dsps_cplx_gen_init(..., NULL, ...)`** mallocs ~2 KB internally per call.
  Pre-allocate the LUT and reuse the generator across bursts via
  `dsps_cplx_gen_freq_set`.
- **No GPIO controls USB host VBUS** on either Waveshare ESP32-P4-Nano or
  ESP32-P4-Pico. VBUS is hardwired-on through the U2 (DIO7003HEST5) load
  switch. Earlier code that drove GPIO 45/54 as a phantom VBUS_EN was wrong
  on both boards (45 is SD card power, 54 is a header pin). Software-only
  recovery from a stuck device is via `usb_host_lib_set_root_port_power()`.
- **PSRAM contention is the real ceiling, not CPU.** Both AXI master (CPU
  loads/stores to PSRAM) and AHB master (USB DWC OTG) contend on the bus
  matrix. The biggest single perf win came from moving `signal_buffer_push`
  off the CPU and onto AXI-GDMA via `esp_async_memcpy_install_gdma_axi` —
  not from inner-loop tuning.
- **Daemon task is pinned to Core 1 deliberately.** `usb_host_install`
  registers the DWC OTG ISR on whichever core executed it; pinning the
  daemon to Core 1 keeps ~300 ISR/s off Core 0's hot DSP loop. Don't move
  it back to Core 0 to "balance" things — that's the inversion of what we
  measured.
- **Convert + push run on Core 1 via ping-pong** (`ingest_core1.c`). The
  consumer (`class_driver`) only does USB read → dispatch → DSP feed.
  Slot ownership is tracked via `s_free[i]` and `s_ready[i]` binary
  semaphores; only `class_driver` takes/gives `s_free`. The ingest task
  must never call `xSemaphoreTake(s_free[...])` — that deadlocks. (Was a
  bug during Step 5 implementation.)
- **DSP pipeline is currently float32 with `_ansi` esp-dsp variants.**
  `dsp_processor.c` calls `dsps_mulc_f32`/`dsps_add_f32` which on P4 only
  have ANSI scalar fallbacks (no `_arp4` PIE variant for f32). The
  windowing and magnitude loops are hand-written scalar. The big PIE wins
  in esp-dsp are on the `*_s16` / `*_sc16` (Q15) kernels, so getting
  meaningful SIMD speedup requires converting the whole pipeline to Q15
  fixed-point, not just slotting in a different function call. This is
  what Step 3 (in the implementation plan) is.

## Throughput status — DONE (Phase 3.5 closed 2026-05-06)

The optimisation arc is complete. We're at the RTL-SDR v4's actual
streaming rate with zero packet loss.

  - Throughput: **4.88 MB/s = 100.5% of the 4.85 MB/s real-time target**
  - rb_full_drops: **0** (steady state; was 32% pre-fix)
  - Core 0 cycle headroom: ~35% (cycle 1972 μs vs 3300 μs budget)
  - Functional regression: bit-perfect against gr-iridium ground truth

| Phase | Throughput | % of target | Drops |
|---|---|---|---|
| Pre-fix (broken PLL) | 1.22 MB/s | 25% | thrashing |
| PLL fix | 2.38 MB/s | 49% | high |
| Step 1 (sdkconfig L2=256 KB, WDT, malloc reserve) | 2.49 MB/s | 51% | high |
| Step 2 (AXI-GDMA `signal_buffer_push`) | 2.82 MB/s | 58% | high |
| Step 4b (USB daemon + ISR Core 1) | 2.84 MB/s | 59% | high |
| Step 4 (convert loop vectorise) | 2.91 MB/s | 60% | high |
| Step 5 (ping-pong convert+push to Core 1) | 3.20 MB/s | 66% | 32% |
| Step 6 (`-Og` → `-O2`) | 4.61 MB/s | 95% | 2.3% |
| Step 7 (per-file `-O3` on hot files) | 4.61 MB/s | 95% | 2.3% |
| Step 3a (hand-rolled PIE Q15 windowing kernel) | 4.61 MB/s | 95% | 2.3% |
| Step 6.5 (status logger to Core 1 task) | 4.88 MB/s | 100.5% | 0 |
| Step 7a (linear-write magnitude + cache hygiene) | 4.88 MB/s | 100.5% | 0 |
| Step 7b (eradicate floats from dsp_processor) | 4.88 MB/s | 100.5% | 0 |
| Step 7c (PIE int magnitude — research, no perf win) | 4.88 MB/s | 100.5% | 0 |

DSP/frame progression: 447 → 422 → 407 μs (Steps 6.5 → 7a → 7b).
Step 7c left it at 407 (scalar mag stays in production after four PIE
recipes were investigated; details in `p4-usb-host/main/dsp_mag_arp4.S`).

Throughput is device-capped at 4.88 MB/s with zero drops. Further
DSP-frame headroom is **pure feature runway**, not a throughput lever
— useful when we add channels, bump sample rate, or stack more DSP.

The "Step 6 = biggest single win" lesson lives in
`memory/feedback_o2_for_optimization.md`: the prior arc had been measured
at `-Og` the whole time; flipping to `-O2` gave more than all five
architectural changes combined. Memorialised so future-me checks the
optimisation level before any throughput tuning.

**Step 7c outcome (closed):** PIE int magnitude squared investigated
thoroughly. Four variants tried: qacc 4-quadrant, fused mul+load,
deinterleave-first, `vmul.s32.s16xs16`. Best kernel mag stage 39→32 μs
(−18% on the kernel itself); but every variant traded the savings back
in EMA-stage L1 D-cache pressure from the int32 scratch buffer evicting
the EMA's working set. Scalar mag stays in production. Working PIE
kernel preserved in `dsp_mag_arp4.S` with discovered semantics: qacc
int64-spaced layout (8 lanes, low32 holds value, high32 = 0), 
`vmul.s32.s16xs16` is Q15-shifted output (lossy), Xhwlp / `lp.setup`
with PIE store at body-end works fine on P4 (Espressif's ACE
customisation handles the AndesCore D45 caveat). Real reference value
for the next person investigating P4 PIE.

**Headroom-only work, paused (not blocking anything today):**
- PIE EMA. Blocked at the silicon ISA level — P4 has no s32×s32 vector
  multiply, no horizontal pair-add for s32, no f32 vector ops in PIE.
  Would need a complete reformulation (e.g., 16-bit baseline at the
  cost of dynamic range). Future ESP32 chip with RVV would unblock.
- Single-memcpy USB via shared URB/ingest buffers (~2–4 days). True
  zero-copy needs forking `espressif__usb` (~2–3 weeks). Useful for
  higher-rate / multi-channel modes; not useful at our current
  100.5% / 0-drop steady state.
- Worker-side PIE kernels (FIR, freq-shift, resampler). Burst
  extraction is within budget. Would need new per-stage host tests
  before touching.
- `esp.vmul.s16.ld.incp` fused mul+load in `dsp_window_arp4.S` —
  ~6 μs/frame saving on a stage already at 16 μs. Documented inline.

**Cross-board sharing:** `bch_decoder.{c,h}` and `qpsk_demod.{c,h}` live
in `common/iridium_decoder/` as a standalone IDF component. The host
firmware pulls them in via `EXTRA_COMPONENT_DIRS
${CMAKE_CURRENT_LIST_DIR}/../common` in its top-level `CMakeLists.txt`
plus `PRIV_REQUIRES iridium_decoder` in `main/CMakeLists.txt`. A future
worker-board P4 firmware (sibling directory of `p4-usb-host/`) will
share the same component the same way.

Other DSP files (`dsp_processor.c`, `signal_buffer.c`, `worker_core1.c`,
`ingest_core1.c`) are board-specific in their current form and stay in
`p4-usb-host/main/` until the worker-board variant exists and the actual
sharing requirements are clear. Don't pre-emptively move them.

## What I (any assistant) should and shouldn't do

- Don't add `Co-Authored-By` / AI attribution lines to git commits — see
  the user's global rule.
- Default to running `./scripts/build.sh` for builds; the direct `ninja`
  path is materially faster for the iterate-loop and avoids re-sourcing IDF.
- Use the per-stage diagnostic logs as primary evidence when reasoning about
  performance, not the running-average rate.
- For new perf work, check whether the relevant esp-dsp primitive has an
  `_arp4` variant (PIE-optimised on P4). If not, the `_ansi` fallback is
  scalar C and won't be much faster than a hand-rolled loop. Sometimes the
  bigger win is removing per-burst allocations rather than vectorising the
  inner loop.

  **Definitive list of `_arp4` (PIE) kernels in esp-dsp 1.8.1 and HEAD master
  @33a5e1c** (audit run 2026-05-06; identical kernel set in both): 13 kernels
  total —
   - FFT: `dsps_fft2r_{sc16,fc32}_arp4`, `dsps_fft4r_fc32_arp4`
   - FIR (single-rate): `dsps_fird_{s16,f32}_arp4`
   - IIR: `dsps_biquad_{f32,sf32}_arp4` (sf32 = stereo float)
   - Dot product: `dsps_dotprod_{s16,f32}_arp4`, `dsps_dotprode_f32_arp4`
   - Matrix-vector mul: `dspm_mult_{s16,f32}_arp4`, `dspm_mult_ex_f32_arp4`

  **Everything else** — windowing, multi-rate FIR (`firmr`), complex
  generator, scalar arithmetic (`mulc`, `add`, `addc`, `sub`),
  sqrt/abs/min/max, convolution, bit-reversal, conjugate — has only
  `_ansi` (scalar C) on P4. The Xtensa LX6/LX7 backends are much wider
  because they've had ~7 years to mature; the P4 PIE backend is ~21 months
  old (since esp-dsp 1.5.1, 2024-08).

  **ESP32-S31** (RISC-V dual-core 320 MHz with MMU) shares the P4 PIE
  backend — the platform headers gate `_arp4_enabled` on
  `CONFIG_IDF_TARGET_ESP32P4 || CONFIG_IDF_TARGET_ESP32S31`. New PIE
  kernels in future esp-dsp releases will benefit both chips, but at
  HEAD master no new ones have landed.

  **Don't assume there's a PIE variant just because the function exists.**
  When in doubt: `grep -rh -E "dsp[ms]_[a-z_0-9]*_arp4" managed_components/ -o | sort -u`.

## File organisation

```
iridium_acars/                   # repo root
  esp-idf/                       # ESP-IDF v6.1 (vendored)
  scripts/                       # repo-level dev-loop tools
    build.sh                     # ninja-direct, idf.py fallback
    flash.sh                     # idf.py flash with port auto-detect
    monitor.sh                   # non-interactive serial monitor
  common/                        # shared IDF components (cross-board)
    iridium_decoder/             # BCH + DQPSK demod
      bch_decoder.{c,h}
      qpsk_demod.{c,h}
      CMakeLists.txt
  p4-usb-host/                   # host-board P4 firmware (this app)
    main/                        # board-specific sources
      class_driver.c             # USB host client task, periodic stats
      dsp_processor.c/h          # FFT + burst detection (Core 0)
      esp_libusb.c/h             # async USB streaming + transfer stats
      worker_core1.c/h           # extract → freq → resample (Core 1)
      ingest_core1.c/h           # ping-pong USB ingest (Core 1)
      signal_buffer.c/h          # 4MB PSRAM lookback ring (AXI-GDMA)
      librtlsdr.c                # RTL-SDR control (R820T + R828D probes)
      smoke_test.c/h             # target-side functional+perf regression
      Kconfig.projbuild          # STATUS_LOG_VERBOSE / smoke test / DIAG_TASK_DUMP toggles
    CMakeLists.txt               # adds ../common to EXTRA_COMPONENT_DIRS
    sdkconfig.defaults           # PSRAM Octal, USB host bias, -O2, etc.
  tests/
    host/                        # native gcc unit tests
      test_bch.c                 # synthetic BCH(31,21) codewords
      test_qpsk.c                # synthetic UW + shape checks
      test_demod_corpus.c        # bit-level vs upstream gr-iridium truth
      CMakeLists.txt             # native build, not an IDF project
    fixtures/                    # generated C arrays from test_corpus
      fixture_corpus_2sps.h      # decimated burst @ 50 ksps int16
      fixture_corpus_uint8.h     # resampled burst @ 2.56 MSPS uint8
      fixture_ground_truth.h     # gr-iridium expected bits
    scripts/build_fixtures.py    # rebuilds the headers from test_corpus
  test_corpus/                   # canonical IQ + ground-truth artefacts
  librtlsdr/                     # vendored librtlsdr (reference)
  gr-iridium/, iridium-toolkit/, # upstream reference impls (read-only)
  iridium-sniffer/, libacars/
  iridium-acars-decoding-stack-design.md
  iridium-acars-implementation-plan.md
```

(The previous nested `esp32p4-usb-host-benchmark/` parent has been
flattened — `p4-usb-host`, `common`, `tests`, `scripts`, `librtlsdr`
are now repo-root-level. Two stale benchmark dirs (`esp32-rtl-sdr/`,
`esp32p4-dsp-harness/`) were removed once the optimisation arc closed
and they were no longer referenced. Future P4 boards (worker variant)
will live as siblings of `p4-usb-host/` and share `common/iridium_decoder/`
via the same `EXTRA_COMPONENT_DIRS ../common` line in their top-level
CMakeLists.)
