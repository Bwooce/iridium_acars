# Code review — optimisations, deadlocks, inefficiencies; ESP32-P4 v1 → v3 assessment

Date: 2026-06-10. Scope: active firmware (`p4-usb-host/main/`,
`common/iridium_decoder/`) plus build configuration. Method: full-file
review of the hot ingest/DSP path, the worker/decode path, and the
network/IO/config path, plus primary-source research on ESP32-P4
silicon revisions. Findings marked **[verified]** were independently
re-traced against the source during synthesis; the rest were
single-pass reviewed.

The headline: the measured-architecture engineering in this codebase
is genuinely strong (see "Done well" at the end), and nothing found
here contradicts the documented throughput status (4.88 MB/s, 0
drops). The significant findings are *correctness under stress* —
two latent data-corruption/deadlock windows in the ingest path, a
71-minute uptime bug in SBD reassembly, and a heap-overrun violation
of the project's own DSP padding rule.

---

## 1. High-severity findings

### H1. [verified] Slot-rotation deadlock when a USB read fails with a slot in flight
`p4-usb-host/main/class_driver.c:429-433` + `ingest_core1.c:502-523`

The failed-read branch releases the slot it just acquired but leaves
`prev_dsp_slot` outstanding and the rotation pointer advanced:

- Iter N: acquire slot A (takes `s_free[A]`), dispatch, `prev_dsp_slot = A`.
- Iter N+1: acquire slot B, `esp_libusb_read_stream` fails (empty
  ringbuffer — dongle hiccup / unplug / quiet window), release B.
  Rotation pointer now points back at A.
- Iter N+2: `acquire_raw` blocks on `xSemaphoreTake(s_free[A],
  portMAX_DELAY)`. `s_free[A]` can only be given by class_driver
  itself, after `take_converted(A)` — code it can no longer reach.

Permanent wedge; only the health_wdt reboot (~30–90 s) recovers. The
deadlock arms exactly when the stream pauses, which is consistent
with the #105/#106 stall-forensics breadcrumbs (`"acquire_raw"` in
`k_class_stage_name`). It is masked in steady state because
`usb_host_client_handle_events(hdl, 10)` blocks 10 *ticks* (100 ms,
see L4) so the ring is rarely empty.

**Fix (keeps the protocol):** in the failure branch, drain
`prev_dsp_slot` first (take_converted → feed → release), then release
the just-acquired slot. Alternative: an `ingest_core1_unacquire()`
that rewinds `s_next_acquire_slot`.

### H2. [verified] CPU-memcpy DMA fallback skips cache writeback — later invalidate can discard the samples
`p4-usb-host/main/signal_buffer.c:296-300` vs `:357-368`

The #126E recovery path (`esp_async_memcpy` submit failed, no wrap)
does a plain `memcpy` into the PSRAM ring **through the write-back L2
cache** and gives `s_dma_done` — but never calls
`esp_cache_msync(..., DIR_C2M)`. The worker's
`signal_buffer_extract`/`invalidate_range` later perform
`M2C | INVALIDATE` over burst regions, which *discards dirty lines*.
If the fallback-written lines are still dirty at that point, the
burst silently reads pre-write PSRAM contents. The comments document
this path firing regularly under DMA-INT pressure ("100% recovery
observed in 33 min soak"), so the window is exercised, bounded only
by natural L2 eviction racing burst extraction.

**Fix:** after the fallback memcpy add
`esp_cache_msync(dst_base + head_bytes, aligned_bytes,
ESP_CACHE_MSYNC_FLAG_DIR_C2M);` — both address and length are
64-aligned by the #125 invariant.

### H3. [verified] 32-bit timestamp truncation kills SBD reassembly after 71.6 minutes of uptime
`p4-usb-host/main/frame_decoder.c:498` (`item.timestamp_us =
(uint32_t)esp_timer_get_time();`), `frame_queue.h:46`, vs the 64-bit
tick at `frame_decoder.c:405-409`.

Sessions are fed `last_update_us < 2^32` (zero-extended truncated
timestamps) while `sbd_reassembler_tick` runs with the full 64-bit
timer. Once uptime exceeds 2^32 µs (~71.6 min), `now -
last_update_us` exceeds `SBD_TIMEOUT_US` for every session, so every
multi-frame SBD session is expired by the 1 Hz tick before its
continuation arrives — multi-block ACARS reassembly silently stops.
The same truncated value feeds libacars `rx_time` (`:179-180`),
wrapping its reassembly timestamps too.

**Fix:** make `frame_queue_item_t.timestamp_us` a `uint64_t` and drop
the cast (the item is ~2 KB; 4 bytes is free).

### H4. [verified] FIR scratch buffers violate the project's own `DSP_PADDING_ELEMS` rule
`common/iridium_decoder/uw_correlator.c:1939` and
`p4-usb-host/main/worker_core1.c:814-815`

AGENTS.md: all buffers fed to `_arp4` PIE kernels need ≥16 `int16_t`
of trailing padding (vector look-ahead bug → `CHIP_LP_WDT_RESET` /
memory faults). The RRC scratch (`bytes = fir_len * sizeof(int16_t)`,
4 buffers, **reallocated at varying sizes per burst** so the adjacent
heap block changes) and the decim scratch
(`DECIM_CHUNK_IN`-exact, 4 buffers) are sized exactly, with no
padding, and both feed `dsps_fird_s16_arp4`. The decim *delay line*
(`direct_if_decim.h`) and the D13 buffers do carry the padding —
these eight allocations are the stragglers.

**Fix:** `+ 16 * sizeof(int16_t)` on all eight allocations (both the
INTERNAL and the PSRAM-fallback variants in uw_correlator).

---

## 2. Medium-severity findings

### Ingest / DSP path

- **M1. `s_carry` overflow in the defensive clamp** —
  `signal_buffer.c:209-215, 311-315`. If the
  `aligned_bytes > ALIGN_SCRATCH_MAX_BYTES` clamp ever fires,
  `new_carry_count` can far exceed the 64-byte `s_carry`, and the
  `(uint8_t)` cast truncates it: the guard corrupts BSS in exactly
  the contract-violation case it exists to catch. Clamp
  `new_carry_count` to `ALIGN_COMPLEX - 1`, drop the excess, count it.
- **M2. Unchecked `esp_cache_msync` in extract/invalidate** —
  `signal_buffer.c:357-368, 390-402`. M2C+INVALIDATE requires
  cache-line-aligned address *and* size; burst start/length from the
  tagger aren't guaranteed multiples of 16 complex. On
  `ESP_ERR_INVALID_ARG` the call no-ops silently and the worker reads
  stale lines. Round address down / end up to 64 B (the whole ring is
  module-owned) and check the return.
- **M3. `ESP_LOGI` escaped back onto Core 0's hot loop** —
  `dsp_processor.c:300-312` formats and emits the `fbt:` line
  synchronously inside `dsp_processor_get_stage_stats()`, called from
  class_driver's 1 Hz snapshot — the exact work the status_logger
  offload was built to remove. Move the fields into
  `dsp_stage_stats_t` and emit from the Core 1 logger.
- **M4. `action_start_stream` ignores init failures** —
  `class_driver.c:194-199`. `signal_buffer_init` /
  `worker_core1_init` / `ingest_core1_init` returns are dropped; a
  partial `ingest_core1_init` failure (`ingest_core1.c:407-409` has
  no cleanup) leaves NULL semaphores → later crash or a silently dead
  pipeline. Check returns; clean up partial allocations.
- **M5. Shutdown use-after-free** — `usb_host_lib_main.c:287-293`
  calls `vTaskDelete(class_driver_task_hdl)` after the task already
  did `vTaskDelete(NULL)` (`class_driver.c:559-560`). One side should
  own deletion.

### Worker / decode path

- **M6. `try_decode_frame` destroys the shared burst buffer in
  place** — `burst_pipeline.c:255-277` with the retry loop at
  `:491-502`. Sub-sample interpolation and 5:1 decimation overwrite
  `adj_burst[uw .. uw+1910)`; when `qpsk_demod_process` rejects the
  frame (the designed noise gate), the retry windows at
  `retry_start = 655, 1310, …` then correlate against mutilated
  samples — silently lowering multi-frame/retry recall. Do interp +
  decim into a dedicated ~7.6 KB static buffer; the pre-rotation can
  stay in place.
- **M7. Per-frame `malloc` + unchecked NULL** —
  `qpsk_demod.c:294-295`. `out->bits` / `out->soft_bits` are heap
  allocated per decoded frame (~3.5×/burst) and `out->bits` is
  written without a NULL check — an OOM under PSRAM fragmentation
  crashes the worker. Sizes are statically bounded
  (`QPSK_MAX_SYMBOLS`): use static or caller-provided buffers; at
  minimum check for NULL.
- **M8. Frame decoder drains ≤100 frames/s against a 63-deep queue**
  — `frame_decoder.c:410-417` processes one item per 10 ms tick.
  Under the documented 140-bursts/s bench-noise load with multi-frame
  bursts, `frame_queue` fills within seconds and drops. Drain a small
  batch (e.g. 8) per wake; the loop already feeds the TWDT.
- **M9. Soft-double trig in the rotate renorm path** —
  `rotate_to_dc.c` (renorm every 128 samples uses
  `cos(double)`/`sin(double)`; the `:51-55` "negligible" comment
  assumes hardware trig the P4 doesn't have — doubles are soft-float).
  ~625 double-trig pairs per 40 k-sample window; ~10 k for a worst-case
  multi-frame burst — plausibly tens of ms on the hottest worker
  stage. Plain `sinf/cosf` is *not* safe (phase reaches ~2e6 rad;
  float ULP there ~0.25 rad): keep phase as a Q32
  fraction-of-a-turn accumulator (exact modular wrap), reduce to
  [0,1), then one `sinf/cosf` pair per renorm. Also hoist the
  per-chunk `cos(phase_step)` doubles in
  `rotate_to_dc_q15_simd_arp4_at` (worker calls it ~157×/burst with
  the same `phase_step`) to once per burst.
- **M10. UW search range silently truncated 1909 → 1778** —
  `uw_correlator.c:1353-1355`. A 2048-pt FFT with a 271-tap reference
  yields 1778 alias-free lags; `SYNC_SEARCH_LEN = 1909`
  (`burst_pipeline.c:191`) was chosen for a guarantee the correlator
  cannot deliver. The 655-step retry usually rescues missed UWs at
  ~18 ms each. Either document and set `SYNC_SEARCH_LEN = 1778`, or
  (probably not worth it) go to a 4096-pt correlation FFT.
- **M11. Worker stats counters: non-atomic cross-core RMW + 64-bit
  tears** — `worker_core1.c:50-96, 911-948`. `volatile` RMW isn't
  atomic on RV32 and `worker_core1_get_stats` read-then-zeroes from
  Core 0. `frame_decoder.c` already does this correctly with
  `_Atomic` + relaxed ops — mirror that (`atomic_exchange` for the
  reset). Diagnostic-only impact.

### Network / IO / config path

- **M12. [verified] 5-second `vTaskDelay` inside the Wi-Fi event
  handler** — `wifi_link.c:60-67`. Runs on the shared `sys_evt`
  task: every disconnect freezes *all* default-event-loop delivery
  (IP events, esp-hosted internals) for 5 s; repeated disconnects
  serialize and can overflow the event queue. Replace with a one-shot
  `esp_timer` that calls `esp_wifi_connect()`.
- **M13. [verified] `/config` POST: 256-byte body buffer + unbounded
  timeout retry** — `http_server.c:467-477`. (a) A URL-encoded form
  with long PSK / OTA URL exceeds 256 B; truncated credentials are
  saved to NVS and the device reboots into Wi-Fi it can't join —
  recovery needs physical access. Use a ~1 KB buffer and reject
  `content_len >= sizeof(body)` with 413. (b) `continue` on
  `HTTPD_SOCK_ERR_TIMEOUT` retries forever; a trickling client pins
  the single httpd serve task and the whole HTTP server (status, OTA,
  capture control) goes dark. Cap retries; stop at `content_len`.
- **M14. SD lazy-mount holds `s_log_mu` across a possible
  format, with the global TWDT relaxed to 180 s** —
  `sd_log.c:218-250, 361-372, 386-391`. `.format_if_mount_failed =
  true` on the lazy path can reformat a glitchy-mount card on the
  first decoded message, while HTTP handlers block on the mutex and
  `esp_task_wdt_reconfigure(180s)` removes watchdog protection from
  *every* watched task system-wide for the window. Make format an
  explicit operator action; prefer `esp_task_wdt_delete/add` on just
  the writer task.
- **M15. `sd_capture_stop()` blocks httpd up to 60 s** —
  `sd_capture.c:458-486`. Return 202 after setting
  `CAP_STATE_STOPPING` and let the client poll `/capture/status`.
- **M16. `/tasks` writes unbounded `vTaskList` output into a fixed
  4 KB buffer** — `http_server.c:948-963`. ~25 tasks already; growth
  silently corrupts PSRAM heap. Size by
  `uxTaskGetNumberOfTasks()`, or use `uxTaskGetSystemState` with
  bounded `snprintf`.
- **M17. Unescaped strings in JSON outputs** — `acars_push.c:116-142`,
  `sd_log.c:151-168`, `http_server.c` status/ota emitters. `msg_num`
  and `flight_id` come from over-the-air frames; SSID / `out_host` /
  `ota_url` / error strings are also emitted raw. A `"` in any of
  them breaks the UDP feed line / NDJSON log / `/status` for every
  client. Route all string fields through the existing
  `json_escape()` (and hoist it to a shared header — it's
  copy-pasted three times, acknowledged at `sd_log.c:107-109`).
- **M18. `captive_dns` busy-spins on persistent `recvfrom` error** —
  `captive_dns.c:89-91`. `n < 0` takes the same `continue` as a short
  packet: tight loop at prio 3, no affinity — can starve idle (WDT)
  or burn Core 1 cycles. Delay + socket recreate after N errors.

---

## 3. Low-severity / cleanup

- **L1.** `signal_buffer.c:227-229` — after a `s_dma_done` timeout the
  code proceeds to overwrite `s_align_scratch` while the previous DMA
  may still be reading it (and a late give un-gates the *next* push
  early). Documented as never-fired; a counter-gated channel reset
  would close it properly.
- **L2.** Torn 64-bit reads of cross-core `volatile uint64_t`
  diagnostics on RV32 (`class_driver.c:65-66`, `esp_libusb.c:39-48`,
  `dsp_processor.c:114`, `ingest_core1.c:595-611`,
  `msg_ring.c:72-75`, `sd_log.c:399`). All diagnostic-only;
  `_Atomic`/read-twice if you want them clean.
- **L3.** Tick/ms confusion: `usb_host_client_handle_events(hdl, 10)`
  is 10 ticks = 100 ms at the default 100 Hz tick
  (`class_driver.c:324`; comments say "10 ms"), `vTaskDelay(10)` in
  `usb_host_lib_main.c:107,284`. Use `pdMS_TO_TICKS`. Interacts with
  H1: the long block currently keeps the ring non-empty.
- **L4.** Odd-byte USB read would permanently swap I/Q downstream
  (`class_driver.c:386` accepts any `n_read`; `ingest_core1.c:253`
  floors). One-line guard + log-once.
- **L5.** `esp_libusb.c` robustness nits: `:91` stores the address of
  a stack parameter in `transfer->context` (dangling; should be
  `driver_obj`); `:417` string-descriptor loop reads one UTF-16 unit
  past `wData` and never NUL-terminates; unchecked
  `calloc`/`transfer_alloc`; `start_stream` mid-loop failure leaks
  the ringbuffer and earlier URBs; hardcoded `4 * 1024 * 1024`
  duplicated at `:310`/`:401`.
- **L6.** `status_logger.c:182-212` STATUS-ERR gates on cumulative
  never-reset counters — with the documented residual ~110 split-RX
  errors/min the WARN line is permanent. Warn on deltas instead.
- **L7.** `burst_pipeline.c:380-387` DC-removal `int32` accumulators
  have only ~4.6 % headroom at `WB_DECIM_MAX` full-scale; any bump of
  `WB_MAX_BURST_SAMPLES` overflows silently (UB). Use `int64_t`.
- **L8.** `fft_burst_tagger.c:488-499` bubble-sorts up to 2048 peaks
  (O(n²) with int64 keys) inside the per-step budget when a wideband
  interferer lights up the band; only `FBT_MAX_BURSTS` winners are
  consumed. Partial top-K selection. Also `:522-526` uses double
  `log10` (soft-float) where `log10f` is free.
- **L9.** `bch_decoder.c:24-27, 90-96` — zeroed-BSS LUT means a
  pre-init `bch_decode_block` call returns "success, 0 errors" on
  garbage (sentinel is `errs = -1` but zeroed state is `errs = 0`).
  Add an init guard or lazy-init.
- **L10.** `freq_estimator.c:117` — 4 KB of float arrays on the
  caller's stack (host-only today, instant overflow if ever wired to
  a small task stack); comments still say N=256, code is N=512.
- **L11.** `resample_256_to_250.c:334-335` — `+0x7fff` before `>>15`
  is round-up-biased (~+1 LSB DC); if it intentionally mirrors the
  PIE asm for bit-exactness, document that, else use `0x4000`.
- **L12.** Stale comments that will mislead the next tuning pass:
  `sd_capture.c:355-358` says worker is prio 3 (it's 4; the
  above-worker placement itself is deliberate per
  `worker_core1.c:861-863`); `http_server.c:1347` says class_driver
  is 3 (it's 6); `worker_core1.c:758` says worker is prio 5 (it's 4);
  `frame_decoder.c:35` says queue ≈ 27 KB (it's ~132 KB PSRAM);
  `qpsk_demod.c:106-111` understates locals (5.1 KB vs "3.8 KB").
- **L13.** Dead code: `q15_freq_shift_inplace`
  (`burst_pipeline.c:81-100`) has no callers.
- **L14.** `smoke_test.c` is compiled into production builds (only
  the *call* is Kconfig-gated); its 8 KB
  `s_per_bin_max_snr[2048]` float array lands in internal-SRAM BSS —
  the resource everything else rations. Gate the translation unit.
  Similarly `http_server.c:607-609`'s 9 KB static `s_snap` belongs in
  PSRAM (`EXT_RAM_BSS_ATTR`).
- **L15.** `acars_push.c:53-54,181-187` — blocking `getaddrinfo` per
  message with no backoff on a dead DNS server; rate-limit
  re-resolution. `ota_runner.c` task is `tskNO_AFFINITY` at prio 5 —
  pin to Core 0 so it can't preempt worker/frame_decoder (4) on
  Core 1.
- **L16.** `frame_decoder.c:36` `DECODER_STACK 6144` — verify
  high-water; libacars can recurse into ARINC-622/CPDLC ASN.1
  decoders, which are stack-hungry.

---

## 4. ESP32-P4 v1 → v3: what changes (researched 2026-06-10, primary sources)

Sources: ESP32-P4 Series SoC Errata v1.2 (2026-04-20), ESP32-P4 Chip
Revision v3.x User Guide (2026-03), Espressif v3.x announcement,
ESP-IDF Kconfig/CMake/linker sources, Espressif PCN202600801. Full
citations in the research notes; key registry rows reproduced below.

**The revision ladder is v0.x → v1.0 → v1.3 → v3.0 → v3.1 → v3.2 (no
v2.x), and the one to target is v3.1, not v3.0.** v3.0 fixes the v1.x
errata this project never tripped (RMT-176, I2C-308) but *introduces*
all three MSPI/PSRAM-DMA errata — MSPI-749 (boot-time AXI read
fault), **MSPI-750** (PSRAM unaligned-DMA stale reads — the exact
USB-DMA-into-PSRAM hazard this project's sdkconfig hardens against),
and MSPI-751 (write-then-read staleness via DMA *or cache* at certain
AXI:MSPI clock ratios). All three plus **APM-560** (which affects
v1.x too) are fixed in **v3.1**. Espressif's own esp-usb
USB-buffers-in-PSRAM CI pins `CONFIG_ESP32P4_REV_MIN_301`.

What v3.1 buys this firmware specifically:

1. **400 MHz HP cores** (vs 360) — ~11 % more CPU on both cores; the
   rev3+ IDF default is `ESP_DEFAULT_CPU_FREQ_MHZ_400`. Worker-stage
   and DSP-frame budgets scale directly; PSRAM-bound stages won't
   (PSRAM stays 200 MHz octal — no speed-grade change in v3.x).
2. **Errata freedom**: MSPI-750/751 and APM-560 fixed →
   `CONFIG_USB_HOST_DWC_DMA_CAP_MEMORY_IN_PSRAM` becomes genuinely
   safe, unblocking the paused "single-memcpy USB / shared
   URB-ingest buffers" work (AGENTS.md headroom item) without forking
   `espressif__usb` defensively. The MSPI-749 boot workaround
   (`P4_REV3_MSPI_CRASH_AFTER_POWER_UP_WORKAROUND`) auto-disables on
   v3.1+ builds.
3. **New PIE ISA + Zb/Zc* extensions**: rev3+ builds use
   `-march=…_xespv` (newer PIE version) vs `_xespv2p1`, plus Zb
   bit-manipulation and the Zc* code-size extensions (v1.x had a
   ZCMP hardware bug). The hand-written `.S` kernels
   (`dsp_window_arp4.S` notes aside, `resample_arp4.S`,
   `rotate_to_dc_arp4.S`, `dsp_mag_arp4.S`) **must be re-validated
   against the new PIE ISA** — the qacc layout / `vmul.s32.s16xs16`
   semantics documented in `dsp_mag_arp4.S` were discovered on v1
   silicon. Re-run the PIE-vs-ANSI diff tests (`pie_fft_diff_test.c`,
   host ctest) first thing on v3 hardware. Whether the "PIE EMA
   blocked at the ISA level" item (no s32×s32 vector multiply)
   unblocks depends on the new ISA's instruction list — check the
   xespv spec when it lands in the toolchain docs; don't assume.
4. **AXI-GDMA improvements**: INCR4/8/16 burst support, better
   arbitration and error logging — directly relevant to the
   `signal_buffer_push` async-memcpy ring and possibly to the
   stash-alloc pressure that motivated `patches/0001` (re-test
   whether the patch is still needed on the v3.1 + new-IDF stack).
5. **CLIC interrupt latency improvements** — marginal gains for the
   ~300/s DWC OTG ISR path on Core 1.

Migration costs, not just wins:

- **One image cannot serve both revisions.** Different ROM linker
  scripts, different `-march`, different L2MEM mapping (cached region
  now starts at the *bottom* of L2MEM; `SRAM_START = 0x4FF00000 +
  L2_CACHE_SIZE` on rev3+), different eFuse tables. The OTA story
  needs two artifacts if v1 boards stay fielded — and a guard so a
  v3 image is never pushed to a v1 board (the bootloader's
  `REV_MAX_FULL=199` check is the backstop, but the OTA server/url
  config should distinguish them first).
- **IDF ≥ v5.5.3 or ≥ v6.0 required** for v3.x; the vendored v6.1
  checkout is already new enough. Flip
  `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=n` + `REV_MIN_301` in a second
  sdkconfig (e.g. `sdkconfig.defaults.v3`).
- **Boards, not chips, are the unit of adoption**: Waveshare
  P4-Nano/Pico ship dates with v3.1 silicon aren't published;
  check the chip marking (v3.1 = `XGXX`, v3.0 = `XFXX`) or eFuse on
  received units. Espressif's v1.3 demand-collection deadline is
  2026-06-30 (PCN202600801), so distributor stock should roll to
  v3.1 through H2 2026. v3.x needed PCB changes (pin 54 becomes
  VDD_HP_1 + DCDC passives), so existing v1 boards cannot be
  chip-swapped; expect new board revisions.
- **Re-measure, don't assume**: the documented perf ceiling is PSRAM
  bus contention, and PSRAM speed is unchanged. The 11 % CPU bump
  helps the CPU-bound worker stages; the AXI/MSPI fabric changes
  (MSPI-751's overlap-detection logic, GDMA arbitration) could move
  the contention picture either way. Re-run the full per-stage
  diagnostic suite before porting any conclusions from the v1
  optimisation arc.

Recommended prep that costs nothing today:

- Keep `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y` exactly as is for v1
  builds (already correct in `sdkconfig.defaults:2-3`).
- Add the v3 sdkconfig overlay + a CI build of it once a v3.1 board
  is on the bench, gating: 400 MHz, REV_MIN_301, and (as an
  experiment branch) `USB_HOST_DWC_DMA_CAP_MEMORY_IN_PSRAM=y` for the
  zero-copy USB work.
- Tag every hand-rolled PIE `.S` file with a "validated on: v1
  (xespv2p1)" comment so the v3 re-validation has a checklist.

---

## 5. Done well (worth preserving as conventions)

- **Measured-architecture honesty**: negative results are kept and
  documented in place (the `-O2` lesson, the PIE magnitude
  investigation with kernel preserved in `dsp_mag_arp4.S`, the
  internal-SRAM `s_conv` regression numbers, the
  `WORKER_OUT_TO_INTERNAL_SCRATCH` experiment). This review found
  *zero* cases where a documented measured choice looked wrong.
- **`frame_queue` SPSC ring** is textbook-correct lock-free code
  (acquire/release pairs, drop-on-full, atomic stats), and
  `frame_decoder.c`'s `_Atomic` discipline is the model the worker
  counters (M11) should copy.
- **Hot-path isolation of the message fan-out**: decoder → msg_ring /
  acars_push / sd_log are all non-blocking drop-on-full; no network,
  flash, or SD I/O can back-pressure the decode path.
- **ISR hygiene**: `dma_done_cb` is exactly right (`IRAM_ATTR`,
  `FromISR` give, woken-flag returned).
- **Memory-placement engineering**: PSRAM task stacks for
  latency-tolerant tasks, eager boot-time DMA-INT pre-allocation,
  codegenned const FFT tables into flash .rodata, the documented
  64-byte msync alignment derivation with its measured 46 %-BER
  failure story, and `_Static_assert` guards on config hazards.
- **NVS-write discipline**: every flash commit runs on a short-lived
  internal-SRAM-stack task with the HTTP response flushed first — the
  cache-disabled-window footgun is consistently avoided.
