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
[`esp32p4-usb-host-benchmark/p4-usb-host/`](./esp32p4-usb-host-benchmark/p4-usb-host/).

The implementation plan is the source of truth for what's done and what's
next. Read it before changing anything substantial.

## Build and flash

ESP-IDF v6.1 is checked out at `./esp-idf/`. Two helper scripts wrap the
common workflow:

```sh
# Build (uses direct `ninja -C build` for fast incremental builds;
#  falls back to `idf.py build` on first build / when reconfigure is needed).
./esp32p4-usb-host-benchmark/scripts/build.sh

# Flash + reset. Auto-detects the first /dev/ttyACM* if no port given.
./esp32p4-usb-host-benchmark/scripts/flash.sh                # auto port
./esp32p4-usb-host-benchmark/scripts/flash.sh /dev/ttyACM0   # explicit
```

If you need raw `idf.py` (e.g. for `monitor`, `menuconfig`, `clean`):
`source esp-idf/export.sh` then `cd esp32p4-usb-host-benchmark/p4-usb-host`.

## Reading the runtime diagnostics

The firmware logs a four-line status block once per second. Interpretation
guide:

```
USB:     rate_inst=X.XX MB/s rate_avg=...   feed_calls=N (avg_per_call us)
USB-XFR: completed=N short=N (fill=%)       rb_full_drops=N status_err=N
DSP:     N frames, total=us/frame           [wind fft mag detect base]
Worker:  queued/dropped/processed/skipped   qmax  avg_burst_us
Worker-stages: extract/freq/fir/resamp/demod/bch (us per processed burst)
```

- **rate_inst** is the throughput over the last 1 s window (use this, not
  rate_avg, which includes startup and is misleading).
- **rb_full_drops > 0** means the DSP task can't drain the USB ringbuffer
  fast enough — samples are being lost, not just delayed.
- **fill=100% short=0** means the device is sending full-rate; any throughput
  shortfall is host-side.
- **DSP per-frame > 800 μs** is over real-time budget at 2.56 MSPS.
- **Worker dropped/queued ratio** > 0 means burst queue is overflowing.

## Things to know before changing code

These are bugs and conventions discovered during integration that aren't
obvious from reading the code:

- **RTL-SDR v4 (R828D) XTAL is 28.8 MHz.** Unlike some R828D implementations that use 16 MHz, the Blog v4 shares the 28.8 MHz clock from the RTL2832U. Setting this incorrectly in `tuner_r82xx.h` prevents PLL lock.
- **Tuner PLL needs settle delays.** The R82XX PLL requires ~10ms to lock. Our port needs explicit `esp_rom_delay_us()` calls between setting dividers and checking the lock bit, as the original `usleep` calls were missing/commented.
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

## File organisation

```
esp-idf/                         # ESP-IDF v6.1 (vendored)
esp32p4-usb-host-benchmark/
  scripts/
    build.sh                     # ninja-direct, idf.py fallback
    flash.sh                     # idf.py flash with port auto-detect
  p4-usb-host/
    main/                        # firmware sources
      class_driver.c             # USB host client task, periodic stats
      dsp_processor.c/h          # FFT + burst detection (Core 0)
      esp_libusb.c/h             # async USB streaming + transfer stats
      worker_core1.c/h           # extract → demod → BCH (Core 1)
      librtlsdr.c                # RTL-SDR control (R820T + R828D probes)
    sdkconfig.defaults           # PSRAM Octal, USB host bias, etc.
esp32p4-dsp-harness/             # earlier offline DSP harness (Phase 0)
gr-iridium/, iridium-toolkit/,   # upstream reference impls (read-only)
  iridium-sniffer/, libacars/
test_corpus/                     # canonical IQ + ground-truth artefacts
iridium-acars-decoding-stack-design.md
iridium-acars-implementation-plan.md
```
