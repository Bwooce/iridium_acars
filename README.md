# Iridium ACARS Decoder on ESP32-P4

Single-board firmware that captures Iridium L-band signals via an
RTL-SDR v4 USB SDR, detects bursts in real time, and decodes the ACARS
frames carried by Iridium Short Burst Data. Runs on a Waveshare
ESP32-P4-Nano or P4-Pico board.

## Status

| Aspect | State |
|---|---|
| **USB throughput** | **~4.48 MB/s** sustained at 2.56 MSPS (= ~87% of the 5.12 MB/s real-time target). `rb_full_drops` ~0 after the 2026-05-25 USB-pool fixes. |
| **Bit-correctness** | Wideband path matches gr-iridium: **1.91% raw BER** on the P4 ALBQ smoke corpus, **1.33%** on host. 29 BCH-corrected frames / 61 matched / 93.8% recall. |
| **Decode budget** | ~85 ms/burst on Core 1 vs ~11 ms target for single-channel real-time (~8× too slow). Reception is the practical limit today, not the worker. |
| End-to-end DSP | Detect → extract → freq-shift → decimate → resample → DQPSK → BCH → IDA → libacars all working. |
| On-device capture | SD raw-IQ capture (continuous + burst modes) over HTTP, with download / list / delete / FIFO-evict tooling. |
| USB host stack | Recovers stuck-device states without physical unplug; class driver no longer subscribed to TASK_WDT (was reboot-storming on stalls). |
| Functional regression tests | 20 host ctests green + on-target smoke (synthetic + real-IQ + LIVE_SDR). |
| **Blocker** | **Antenna + LNA hardware** (Scan Iridium GO! QFH + Nooelec SAWbird+ IR + lightning protection). Until installed, only RFI is in the air; 30-min bench soak (2026-05-24) saw 14 real bursts at 12-13 dB SNR — below the host's reliable decode threshold. |

## Quick start

```sh
# Clone with submodules (esp-idf is vendored)
git clone --recurse-submodules https://github.com/Bwooce/iridium_acars.git
cd iridium_acars

# Build the firmware
./scripts/build.sh

# Flash to the board (auto-detects /dev/ttyACM*)
./scripts/flash.sh

# Watch live diagnostics for 30 seconds
./scripts/monitor.sh 30 /dev/ttyACM0

# Run the host-side regression tests (no board needed)
cd tests/host && mkdir -p build && cd build && cmake .. && make
./test_bch && ./test_qpsk && ./test_demod_corpus && ./test_demod_low_snr && ./test_demod_albq
```

The build/flash scripts re-source the vendored ESP-IDF v6.1 in `esp-idf/`
— they ignore any system IDF on `$PATH`.

**Run build and flash as separate commands.** The split lets you
authorise each independently and re-flash without re-building.

## Hardware

- **Compute:** Waveshare ESP32-P4-Nano or ESP32-P4-Pico (dual-core
  RISC-V at 360 MHz, with PIE 128-bit SIMD). Connect via the
  USB-C debug port for power and serial.
- **SDR:** RTL-SDR Blog v4 (R828D tuner, 28.8 MHz XTAL). Connect via
  the high-speed USB OTG port (Picoblade or Type-A depending on board
  variant).
- **RF frontend (for Phase 4 — not yet acquired):**
  - Scan Iridium GO! passive QFH antenna (1620 MHz, RHCP)
  - Nooelec SAWbird+ IR LNA (1620 MHz centre, ~60 MHz BW)
  - GDT lightning arrestor (Polyphaser IS-50UX-MA or equivalent)

The current SAWbird+ IR is centred on Iridium; if you also want to
decode Inmarsat Aero (Phase 5), see
[`inmarsat-acars-feasibility.md`](./inmarsat-acars-feasibility.md) — a
SAWbird+ iO swap is required.

## Project layout

```
iridium_acars/
├── esp-idf/                       ESP-IDF v6.1 (vendored)
├── scripts/                       dev-loop tools
│   ├── build.sh                   ninja-direct, idf.py fallback
│   ├── flash.sh                   idf.py flash with port auto-detect
│   └── monitor.sh                 non-interactive serial monitor
├── common/iridium_decoder/        cross-board shared IDF component
│   ├── bch_decoder.{c,h}          BCH(31,21) t=2
│   └── qpsk_demod.{c,h}           DQPSK + PLL phase tracking
├── p4-usb-host/                   host-board P4 firmware
│   ├── main/                      board-specific sources
│   │   ├── class_driver.c         USB host client task
│   │   ├── dsp_processor.c        FFT detector (Core 0)
│   │   ├── ingest_core1.c         ping-pong USB ingest (Core 1)
│   │   ├── signal_buffer.c        4 MB PSRAM lookback ring
│   │   ├── worker_core1.c         extract → resample → demod (Core 1)
│   │   ├── librtlsdr.c            RTL-SDR control
│   │   ├── status_logger.c        per-second status block (Core 1)
│   │   ├── smoke_test.c           target-side functional+perf regression
│   │   ├── dsp_window_arp4.S      hand-rolled PIE Q15 windowing kernel
│   │   └── dsp_mag_arp4.S         PIE int magnitude kernel (research only)
│   ├── CMakeLists.txt
│   └── sdkconfig.defaults
├── tests/
│   ├── host/                      native gcc unit tests
│   │   ├── test_bch.c             synthetic BCH(31,21) codewords
│   │   ├── test_qpsk.c            synthetic UW + shape checks
│   │   ├── test_demod_corpus.c    synthetic PRBS-15 vs gr-iridium ground truth
│   │   ├── test_demod_low_snr.c   ~10 dB SNR variant for margin checks
│   │   └── test_demod_albq.c      real-RF Albuquerque burst vs gr-iridium
│   ├── fixtures/                  generated C arrays from test_corpus
│   └── scripts/build_fixtures.py  rebuilds fixture headers
├── test_corpus/                   synthetic PRBS-15 corpus from gr-iridium upstream
├── test_data/
│   └── iridium_downlink_2022-03-17_albuquerque/
│                                   real-RF capture (USRP B210, 12 MSPS, 1.25s,
│                                   CC0). Used by tests/host/test_demod_albq.c.
│                                   Raw cf32 stored zstd-compressed (~45 MB);
│                                   derive.sh decompresses + runs gr-iridium
│                                   extractor + iridium-toolkit parser.
├── librtlsdr/                     vendored upstream (reference)
├── gr-iridium/                    upstream (reference)
├── iridium-toolkit/               upstream (reference)
├── iridium-sniffer/               upstream (reference)
├── libacars/                      upstream (reference)
├── AGENTS.md                      project conventions for AI assistants & humans new to repo
├── iridium-acars-decoding-stack-design.md       architecture / design
├── iridium-acars-implementation-plan.md         per-phase progress + history
└── inmarsat-acars-feasibility.md                Phase 5 extension scope
```

## Reading the runtime diagnostics

By default the firmware emits a single short health line per second
plus an extra warning line only when an anomaly counter is nonzero:

```
STATUS: rate=4.88 MB/s steps=1252 bursts=3 processed=0 drops=0
STATUS-ERR: rb_full_drops=N status_err=N resubmit_err=N worker_dropped=N last_err=0xNN  # only when something's wrong
```

Plus burst/decode events at their source (always logged):

```
BURST DETECTED! Frame:N Bin:N SNR:Y dB
Worker processing burst: ...
DEMOD SUCCESS: DL frame (N bits)
BCH DECODE SUCCESS! Errors: e1, e2
Block1 Data: <bits>
```

To get the full per-second telemetry block (USB / USB-XFR / USB-RB /
Cycle / Ingest / DSP / Worker / Worker-stages), enable
`CONFIG_STATUS_LOG_VERBOSE=y` via `idf.py menuconfig` → "Iridium ACARS
app options" → "Emit the full 8-line per-second status block". Useful
while tuning throughput; off by default to keep long unattended
captures readable.

Healthy operational signs:
- `rate=` ~4.4-4.6 MB/s stable (= the device's actual streaming rate; RTL emits 5.12 MB/s at 2.56 MSPS)
- `drops=0`, no `STATUS-ERR:` line
- `processed > 0` lines appear once real Iridium bursts are detected

When something's wrong:
- `STATUS-ERR: rb_full_drops > 0` → consumer is falling behind (samples lost, not just delayed)
- `STATUS-ERR: status_err / resubmit_err` → USB host issues
- `STATUS-ERR: worker_dropped > 0` → burst queue overflow

## Capturing real bursts (when the antenna is hooked up)

Background-log the serial port with `serial_logger.sh` — auto-detects the
P4 port by USB vendor:product, auto-reconnects across reflashes / USB
re-enumeration, optionally rotates the log:

```sh
./scripts/serial_logger.sh &                  # default log: /tmp/p4_serial.log
./scripts/serial_logger.sh status             # is it alive?
./scripts/serial_logger.sh stop               # kill the instance
```

For one-shot N-second captures use `./scripts/monitor.sh <seconds>` instead.

Or pipe through a filtering wrapper that only writes interesting events
(`BURST DETECTED`, `Worker: ... processed=N`, USB errors, panics) to
keep the log small over long captures.

## How the optimization arc went

The throughput journey (25% baseline → peak ~100% on the early
synthetic load → today's ~87% sustained on live SDR, after the
wideband bit-correct decode path landed) is documented in
[`iridium-acars-implementation-plan.md`](./iridium-acars-implementation-plan.md).
Some highlights worth keeping front of mind:

- **Single biggest win** was Step 6: switching `CONFIG_COMPILER_OPTIMIZATION_DEBUG=y` (`-Og`) → `CONFIG_COMPILER_OPTIMIZATION_PERF=y` (`-O2`). Jumped throughput 3.20 → 4.61 MB/s. Bigger than all five prior architectural changes combined. **Always profile at `-O2` or higher** — IDF's default is `-Og`, which makes scalar DSP code 3-4× slower.
- **Final 5% was logging**, not DSP. The per-second status block on Core 0 was stalling the consumer for ~5-10 ms every second; moving it to a Core 1 task closed the last gap.
- **PIE int-vector multiplication** has subtle gotchas. We documented the qacc int64-spaced lane layout, the `vmul.s32.s16xs16` Q15-shift quirk, and the lp.setup PIE-store-at-body-end ACE behavior in `p4-usb-host/main/dsp_mag_arp4.S` for the next person investigating P4 PIE.

## Documentation map

- **[`AGENTS.md`](./AGENTS.md)** — conventions, gotchas, build/flash workflow. Read first if you're new (or you're an AI assistant working in the repo).
- **[`iridium-acars-decoding-stack-design.md`](./iridium-acars-decoding-stack-design.md)** — architecture, signal characteristics, link budget, lightning protection.
- **[`iridium-acars-implementation-plan.md`](./iridium-acars-implementation-plan.md)** — phase-by-phase progress; full per-step throughput history.
- **[`inmarsat-acars-feasibility.md`](./inmarsat-acars-feasibility.md)** — feasibility report for adding Inmarsat Classic Aero decoding (deferred).

## License

[MIT](./LICENSE). The vendored upstreams (gr-iridium, iridium-toolkit,
libacars, etc.) keep their respective licenses; see each subdirectory.

## Acknowledgements

- [gr-iridium](https://github.com/muccc/gr-iridium) and
  [iridium-toolkit](https://github.com/muccc/iridium-toolkit) — the
  reference implementations we validate against.
- [Espressif esp-idf, esp-dsp, esp-nn, esp-dl](https://github.com/espressif)
  — IDF runtime and the PIE assembly examples that informed the
  hand-rolled kernels.
- [libacars](https://github.com/szpajder/libacars) — for the
  application-layer decoding once we get there.
