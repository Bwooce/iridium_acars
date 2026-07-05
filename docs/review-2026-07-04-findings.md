# Code review findings — 2026-07-04

Whole-project bug + optimisation review (5 parallel readers over DSP, frame
decode, USB/ingest, worker/storage, network/HTTP). Host suite green (27/27);
device healthy at 4.88 MB/s, decode drought is the poor bench antenna.

Priority order, unique task IDs. `[ ]` = open, `[~]` = in progress, `[x]` = done.
Full per-file notes: `.claude/jobs/*/tmp/findings_{dsp,decode,usb,worker,net}.md`.

**Fix status (2026-07-04):**
- Batch 1 — T1–T4 DONE, pushed (commits 1aca876/e6db4b9/933ab38/97c54ea). T1/T3
  host-verified with new regression tests; T2/T4 source-verified + device build;
  on-device RAW_IRIDIUM SMOKE_PASS. T1/T2 (DSP-path) carry `Smoke-verified:` trailers.
- Batch 2 — T2t, T5, T6, T7, T8 DONE, pushed. T2t adds the host regression for the
  T2 ring-index invariant (29/29 ctest, teeth-proven); T5/T6 http/push output safety;
  T7 frame_link SPI-DMA size; T8 esp_libusb control-transfer mutex. On-device: batch-2
  RAW_IRIDIUM SMOKE_PASS (T2t signal_buffer, carries the trailer) + normal firmware
  live-stable at 4.88 MB/s, drops=0, no xfer_mutex timeout/panic (validates T8).
- Note: the batch-1 firmware left on the bench was mistakenly the smoke image (a
  restore-to-normal step ran in the wrong cwd and silently failed); corrected in batch 2
  — device now runs the real normal streaming firmware.
- Batch 3 — T9, T10, T11, T12, T4t DONE, pushed. T9 keeps uw_correlator direction/SNR
  in float (the int64 bridge was UB: `sum_dl_f≈2.63e20 > INT64_MAX`, independently
  confirmed by instrumenting the pre-fix code); the `uw_correlator_golden` characterization
  value was a snapshot of that UB output and was corrected 27.05→11.44 dB (the true
  correlation peak-to-sidelobe ratio, hand-verified). **NB: CORR_USE_FLOAT_FFT=1 on
  device, so device SNR reporting for strong bursts was UB-inflated and is now correct.**
  T10 tagger baseline_sum overflow clamp, T11 resample scalar saturation, T12 SBD crc_ok
  gate + da_len payload, T4t sd_capture framing host regression. On-device: batch-3
  RAW_IRIDIUM SMOKE_PASS (DSP 520 µs/frame, corrected SNRs still clear the 10 dB gate) +
  normal fw live-stable 4.88 MB/s. Four DSP-path commits carry `Smoke-verified:` trailers.
- Batch 4 — T17, T22, T23 DONE, pushed. T17 bounds burst_pipeline pre-rotation to the
  consumed frame span (≈gri's frame_size, saving ~12-14k Q15 MACs/call on hard bursts;
  a 4-bit BER wobble on one marginal burst from removing compounded truncating rotations,
  no decode-classification change — host golden + device smoke both confirm). T22 moves
  the ~2.1 KB frame_queue_item_t off the decoder-task stack (single-consumer static).
  T23 hardens ida_decode partial-decode to write blocks at true positions (latent bug,
  no live consumer; all-10 path byte-identical). On-device: batch-4 RAW_IRIDIUM SMOKE_PASS
  (LW.SY/DA/IP/IBC decoding, DSP 514 µs) + normal fw live-stable 4.88 MB/s. Three DSP-path
  commits carry `Smoke-verified:` trailers.
- Batch 5 — T51, T52, T53, T54 DONE, pushed. T51 demotes the per-burst worker log to
  DEBUG; T52 drops the per-call stats mutex on the sd_capture ingest path (IDF 64-bit
  atomics = interrupt-safe global spinlock, cheaper than the FreeRTOS mutex — NOT
  lock-free, but safe); T53 removes the dead ~1.8 KB tail memset + copies only bits[0..n_bits)
  in frame_queue push/pop (verified no consumer reads past n_bits; bits[] is the last
  struct field); T54 investigated the async-memcpy split-RX stash and documented the IDF
  driver root cause (unconditional stash alloc, no safe config knob — comment-only, the
  T2 CPU-memcpy recovery stays the mitigation). On-device: batch-5 RAW_IRIDIUM SMOKE_PASS
  (DSP 513 µs, LW.SY/DA/IP/U3/IBC decode) + normal fw live-stable 4.88 MB/s, no atomic/panic.
- Batch 6 — T13, T21 DONE, pushed. T13 makes the WiFi STA_DISCONNECTED reconnect
  non-blocking (one-shot esp_timer instead of a 5 s vTaskDelay in the shared event-loop
  handler); T21 realigns the SD file position after a short fwrite (fseek back to the
  512-byte boundary + re-queue the uncommitted bytes) to avoid the FATFS/SDMMC EIO cliff.
  Neither is DSP-path (no smoke trailer). On-device: boots clean, WiFi associates + IP,
  stream stable 4.88 MB/s. Follow-up: T21b — the CAP_STATE_STOPPING drain loop has the
  same short-write gap (lower risk, runs once before fclose).
- Batch 7 (2026-07-05) — the perf-decoupling + PIE-placement session. DONE + pushed:
  **T48** (split class_driver into usb_pump/dsp_feed tasks, commit fbcf30e); **T49a**
  (zero-copy usbring replacing the IDF ringbuffer + s_raw deletion, commit 473b81b);
  **T50 harness** (bit-exact golden gate for the window multiply, 3bf50a3 — PIE kernel
  itself still banked). **T55 (NEW, the headline): PIE-FFT-scratch RTCRAM-spill decode
  bug** — uw_correlator's fc32 FFT scratch was lazily spilling to RTCRAM under DRAM
  pressure, silently cratering clean-signal decode to ~6% recall (mistaken for antenna
  for months); fixed by early-alloc DRAM pinning + esp_ptr_in_dram guard, and the RAW
  smoke re-gated on GOLDEN-matched recall (was `classified>=5`, which had been calibrated
  to the corrupted baseline). Recall 6%→95% (commit 473b81b). See
  `project_heap_position_decode_bug` + `docs/perf-decoupling-design-2026-07-04.md`.
  **T57 (NEW): rotate_to_dc PSRAM-stack PIE scratch — VERIFIED FINE** (A/B: internal vs
  PSRAM gave identical decode; the "PIE mis-services PSRAM" rule is only for large
  sustained transfers, not small per-chunk vld/vst); corrected a false "ROT_SIMD_DIAG"
  validation comment (ab5d58d). Also: full PIE-buffer placement audit (in memory);
  resampler proven chunk-continuable (test on wip/t49b-tile-fuse); wideband front-end
  design sketch (`docs/wideband-frontend-design-2026-07-05.md` — full-band real-time is
  compute-infeasible on P4; capture→offline is the realistic play).
- **T56 DONE (53327e7): pinned the 3 lazy PIE FIR delay lines** (D13-LP / RRC / decim
  in uw_correlator + direct_if_decim) early in the boot dance + backfill esp_ptr_in_dram
  guards on s_coeffs_pp/s_fft_scratch. Fixed the residual ~3-frame layout-lottery (proven:
  the 4 KB T49b tile decoded 59 without the pins, 62 with — end-to-end confirmation).
- PARKED: **T49b** (convert/resample SRAM tile fuse) on branch wip/t49b-tile-fuse —
  correct + bit-exact, but decode-neutral needs a >=8 KB tile which doesn't fit the
  silicon-locked DMA-INT budget (USB pool can't move to PSRAM: APM-560/MSPI errata).
  Non-bottleneck. **T49c** (resample writes into signal_buffer scratch) — not started.
- Batch 8 (2026-07-05) — P3 low-severity sweep, DONE + pushed (db20f7a DSP-path,
  d67a452 non-DSP). ~19 fixes, all proven decode-NEUTRAL (RAW GOLDEN bit-identical
  to baseline via A/B). **T45's WB_PRE_PAD 288->320 BACKED OUT** — it degraded decode
  (BER 1.39%->2.42%, exact 44->43, div 1->3) recovering only harmless safety-padding
  samples; caught via the GOLDEN bucket/BER breakdown (matched count alone hid it).
  Partial/skip: T25 (host-only, device matches vendored esp-dsp), T30 (3/4; static
  scratch left to avoid a PSRAM-stall regression), T46 (already fixed).
- Batch 9 (2026-07-05) — T43 + T44 follow-ups landed: 2906fe6 (T43 dead sync bulk path
  removed, non-DSP, build-verified), 8a19ce3 (T44 64-bit head, Smoke-verified RAW
  matched=62/65 exact=44 BER=1.39% — behavior-neutral on the corpus). Pushed b6ddddc..8a19ce3.
- Remaining open: T14 (OTA auth — the notable security gap), T15/T16 (frame_link, HW not
  brought up), T18–T20 (USB hot-unplug/teardown — need physical unplug, not on this bench),
  T49c + T50 kernel, T21b (minor SD drain-loop follow-up).

**Live-device verification (2026-07-04, device on LAN at 192.168.1.235, build 5e18864):**
- **T2 wrap-desync — PROVEN via fault injection.** Built with CONFIG_FAULT_INJECT=y,
  armed `POST /debug/fault_inject?site=dma_submit_wrap&count=15`. All 15 fired the
  fix's recovery branch (`wrap=1, wrap_first_submitted=0 — CPU memcpy fallback (both
  segments)`); counters ended `stash_fails=15 recoveries=15 audio_dropped=0` — every
  wrap-path DMA failure CPU-recovered with zero drop and zero desync, stream stable
  at 4.88 MB/s. Strongest T2 verification (the injection hits the wrap sub-case the
  fix rewrote, which natural failures rarely reach). Production fw (fault-inject off)
  restored afterward.
- **T5/T6 HTTP output — path confirmed.** `POST /debug/inject` → `GET /messages`
  returned well-formed JSON with all escaped fields correct; write→ring→serve path
  and the snprintf clamp are sound. Caveat: /debug/inject uses fixed benign text, so
  the adversarial escaping (quotes/control bytes) is not stressed — that needs a real
  RF decode (none, poor antenna).

**ESP-IDF upgrade (2026-07-04): master snapshot `v6.1-dev-4427-gc00874869b` (2026-04-30)
→ `release/v6.1` branch @ `v6.1-dev-5215-g0d92878008` (2026-06-08, 788 commits, clean
superset).** IDF is a gitignored local checkout (not repo-tracked), so this is an
environment change; `patches/0001` (async_memcpy descriptors → PSRAM) re-applies cleanly.
Full cycle passed: clean rebuild (our code warning-clean), 32 MB PSRAM inits, host suite
30/30, RAW_IRIDIUM SMOKE_PASS (DSP 515 µs, decode distribution identical to old IDF),
production fw live-stable 4.86–5.01 MB/s, drops=0, then multi-hour soak. Config drift
fixed: `sdkconfig.defaults` stale `SPIRAM_TYPE_OCTAL` → `SPIRAM_MODE_HEX` (P4 default;
old symbol removed in v6.1). Rollback point: IDF commit `c00874869b`.

Legend: **bug** / **sec** (security) / **perf**. "verified" = main agent
re-read the source and confirmed the defect.

## P0 — fix first (verified, real decode/data impact)

- [x] **T1** `ibc_decode.c:93` — bug, verified. FIXED (branch, host-verified). IBC body parsed WITHOUT the
  pair-swap the classifier applied (`iridium_frame.c:397` `classify_bc(swapped)`).
  Siblings ida/ira/ims/tl all re-swap `frame->bits`; ibc does raw `memcpy`.
  Every clean BC frame decodes in wrong orientation → header_ok false or
  silently-wrong sv_id/beam_id/iri_time. No host test covers ibc. Fix: copy +
  pair-swap the 6+256 post-UW bits first, mirror `ida_decode.c:118-124`. Add a test.
- [x] **T2** `signal_buffer.c:333` — bug, verified. FIXED (branch, pending device smoke). Wrap-path async-memcpy submit
  failure drops the chunk but does NOT advance `head`, while the tagger's
  cumulative sample index keeps advancing → `start_sample_idx % total_cap`
  mapping skews ~4 ms permanently, no re-sync. Device shows `audio_dropped=4`/29 min.
  Fix: CPU-memcpy-recover the wrap path like the simple path, or advance head.
- [x] **T3** `aggregator_ingest.c:67` (+ `frame_pdu.c:91`, `frame_link` decode) — FIXED (branch, host-verified).
  bug/sec, verified. Wire `n_bits` unclamped; `s_bits01` is 512 B but
  `frame_decoder_push` accepts 2048 → ~1.5 KB OOB read on a CRC-valid hostile PDU.
  Fix: reject `n_bits > FRAME_PDU_MAX_BITS` in `frame_link_decode` and before push.
- [x] **T4** `sd_capture.c:562,575,422` — bug, verified. FIXED (branch, pending device smoke). (a) burst records use
  `xStreamBufferSend(...,0)` whose partial write desyncs the hdr+IQ framing on any
  SD stall; (b) file is `_IONBF` so the "periodic flush" `fflush` at :243/:291 are
  no-ops and there's no `fsync` → power-loss loses whole file. Fix: atomic
  space-check + drop whole burst; add throttled `fsync`.
- [x] **T55** `uw_correlator.c:477` — bug, DONE (473b81b), the biggest decode finding.
  The fc32 PIE FFT scratch (16 KB) is lazy-allocated on the worker's first burst with
  `MALLOC_CAP_INTERNAL`; under DRAM pressure it silently spilled to RTCRAM (0x5010_xxxx),
  where the PIE vector unit mis-decodes → clean-signal RAW-smoke recall cratered to ~6%
  (matched 4/65) and was mistaken for antenna/RF for months. Fix: `uw_correlator_prealloc_pie_fft()`
  pins it in DRAM from the boot dance + an `esp_ptr_in_dram` guard fails loudly on a
  non-DRAM placement. ALSO re-gated the RAW smoke on GOLDEN-matched (≥40) instead of
  `classified≥5` (which counted UNKNOWN/BCH false positives and had been calibrated to the
  corrupted baseline — it passed the bug for months). Recall 6.2%→95.4%, deterministic.

## Follow-up test coverage (tracked 2026-07-04)

- [x] **T2t** signal_buffer wrap-desync (T2) has NO automated regression — device-only
  code. Add a SMOKE_TEST sub-test that arms `FI_SITE_DMA_SUBMIT_WRAP`, pushes across a
  ring wrap, and asserts a marker sample is still retrievable at its cumulative index
  (no desync). Optionally a host model test of the index arithmetic. Rides the pre-push
  smoke gate already enforced for signal_buffer. **Highest-value gap** — T2 is the one
  decode-affecting bug and self-masks as "bad antenna/RF".
- [x] **T4t** sd_capture atomic-record framing (T4) has no automated regression. Factor
  the whole-record space-check decision into a pure helper and host-test it; framing/fsync
  themselves are device-only (consider a device capture-mode assertion).

## P1 — High (verified)

- [x] **T5** `http_server.c:723` — sec, verified. `snprintf` truncation returns
  would-be length, passed unclamped to `httpd_resp_send_chunk` → sends adjacent
  httpd stack to LAN client (RF-crafted escape-heavy ACARS txt). Same pattern:
  `status_get` pdu_link splice `:298` (size_t wrap → stack overflow, non-STANDALONE),
  `sd_list_get:1137`, latent `diag_histograms:328`. Fix: clamp len to buf-1.
- [x] **T6** `acars_push.c:150`, `http_server.c:728/744/233` — sec. RF-decoded
  `mode`/`block_id`/`label`/`station_id` emitted unescaped → JSON injection into
  every push consumer / /messages client. Fix: escape/isprint-filter.
- [x] **T7** `frame_link.c:229,266` — bug (hardware-only). `FRAME_LINK_FRAME_SIZE`=98
  not a multiple of 4 → SPI DMA rx rejected or trailing CRC word corrupted; rx
  buffers (`:214,251`) not cache-line padded on P4. Fix before hw bring-up: pad to
  mult of 64, `heap_caps_aligned_calloc(64,...)`.
- [x] **T8** `librtlsdr.c` + `esp_libusb.c:135` — bug. No serialisation of control
  transfers despite `class_driver.h`'s claim; AGC gain-change racing class task can
  free in-flight transfer (UAF) / two tasks in handle_events. Works by luck today;
  an HTTP retune endpoint detonates it. Fix: mutex around all control/bulk transfers.

## P2 — Medium

- [x] **T9** `uw_correlator.c:1411` — bug, verified. float→int64 magnitude bridge
  overflows int64 (UB) on strong burst; RISC-V saturates both dirs → tie → always UL.
  Fix: direction-pick + SNR in float/double.
- [x] **T10** `fft_burst_tagger.c:597` — bug, verified. `baseline_sum` int32 can
  overflow under coherent strong carrier in un-primed first 512 steps (code says
  "needs int64"). Fix: widen or clamp per-bin mag².
- [x] **T11** `resample_256_to_250.c:346` — bug, verified. scalar MAC store
  `(int16)(acc>>15)` no saturation, PIE path saturates → host/device divergence at
  full-scale, breaks bit-exact parity gate. Fix: saturate scalar store.
- [x] **T12** `sbd_reassembler.c` + `ida_decode.c:183` + `frame_decoder.c:332` — bug.
  Session gate checks `ida.ok && header_ok` but NOT `crc_ok` (BCH false-positives
  pollute reassembly); multi-IDA-frame packets unrecoverable (da_cont/da_ctr unused);
  payload_len fixed 20/22 not da_len → filler bytes leak to ACARS parser.
- [x] **T13** `wifi_link.c:60` — bug. 5 s `vTaskDelay` in disconnect handler blocks
  shared default event loop during AP flaps. Fix: esp_timer for reconnect.
- [ ] **T14** `ota_runner.c:70` — sec. No image authenticity (no cert, project_name
  only logged) + unauthenticated LAN/open-AP POST /ota. Fix: reject project_name
  mismatch; token-gate /ota + /sd/format.
- [ ] **T15** `frame_link.c:126` — bug. SPI slave ISR callbacks call flash-resident
  `gpio_set_level` → panic during OTA cache-disabled window. Fix: IRAM gpio or gpio_ll.
- [ ] **T16** `frame_link.c:356` — bug. Loopback selftest passes primary handshake
  GPIO to slave instead of `..._LB_HANDSHAKE_GPIO`; passes only via fixed 5 ms delay,
  never exercises handshake.
- [x] **T17** `burst_pipeline.c:227` — perf, verified. Pre-rotation rotates entire
  remaining burst (~16k cplx) per try, only ~1911 consumed; ~1-2 ms/burst × 20 retries.
  Also truncating `>>15` (no rounding) accumulates bias per retry. Fix: bound rotate.
- [ ] **T18** `class_driver.c:114` — bug. `rtldev` TOCTOU UAF: AGC uses it unlocked
  while `action_close_dev` NULLs it on hot-unplug.
- [ ] **T19** `librtlsdr.c:1478` — bug. `rtlsdr_close()` fully commented out; nothing
  freed on DEV_GONE, teardown can't complete → replug-requires-reboot. Document or fix.
- [ ] **T20** `librtlsdr.c:1346` — bug. `rtlsdr_open` panics via `ESP_ERROR_CHECK` on
  transient open/claim failure (boot-loop vs retry); `driver_obj` calloc unchecked.
- [x] **T21** `sd_capture.c:186` — bug. Partial short `fwrite` leaves file position
  mid-sector → reintroduces the known EIO cliff. Fix: fseek back to 512 boundary.
- [x] **T22** `frame_decoder.c:386` — bug. DECODER_STACK=6144 tight vs ~2.1 KB stack
  `frame_queue_item_t` + process_one + libacars + vsnprintf(256). Fix: static item or 8-12 KB.
- [x] **T23** `ida_decode.c:141` — robustness. Partial-decode compacts skipped blocks
  left → misaligned bitstream for positional consumers. Fix: zeros at true pos + ok-mask.

## P3 — Low

- [x] **T24** `burst_pipeline.c:411` — DC removal int16 wrap near rails; use q15_saturate.
- [~] **T25** `direct_if_decim.c:203` — host acc starts 0 vs device 0x7fff round const +
  unsaturated stores → ~1 LSB host/device parity drift; drops `n_in%10` remainder.
- [x] **T26** `resample_256_to_250.c:136` — silent malloc-fail leaves coeffs unwritten.
- [x] **T27** `fft_sc16_2048.c:62` — alloc-fail path no ESP_LOGE → silent no-op tagger.
- [x] **T28** `fft_burst_tagger.c:340` — flush() doesn't drain in-flight helper / reset
  pipe_in_flight (latent; helper disabled).
- [x] **T29** `sym_timing.c:159` — unclamped negative v drives strobe_idx backwards (module unwired).
- [~] **T30** `uw_correlator.c` — PIE-FFT partial-alloc leak (:480), silent no-op on init
  fail emits confident wrong result (:494), non-reentrant static scratch (:1313),
  r*r+i*i UB at -32768 (:1665).
- [x] **T31** `bch_decoder.c:49` — lazy init sets inited=true before syn_ra filled →
  cross-core reader accepts garbage as 0-errs. Fix: set flag last.
- [x] **T32** `iridium_bch.c:10` — bits_to_u32 truncates n_bits>32; all-zero always passes
  (matches upstream; feeds BC/RA false-positive stats).
- [x] **T33** `sbd_reassembler.c:225` — noise msg_cnt (≤255) opens sessions dying only by
  5 s timeout → 8-slot table saturates, drops real multi-frame; session match ignores type (:70).
- [x] **T34** `tuner_r82xx.c:603` — usleep_range → esp_rom_delay_us busy-spins ≤2×10 ms in
  PLL lock; latent, a runtime retune stalls Core 0. Fix: vTaskDelay.
- [x] **T35** `frame_decoder.c:135` — get_rolling_rates double-counts most recent hour
  (24 h figure up to 2× inflated, spans up to 25 h).
- [x] **T36** `sd_log.c:157` — esc buffer 2× assumption vs 6× `\u00XX` → silent truncation ~519 B.
- [x] **T37** `sd_log.c:485` — s_stats.log_path/log_open written without s_stats_mu (torn read);
  fflush/fsync unchecked (:261), messages_written increments pre-commit.
- [x] **T38** `worker_core1.c:619` — burst_valid TOCTOU: producer advances head during
  multi-ms read, marginal bursts partially overwritten. Fix: re-validate after read.
- [x] **T39** `status_logger.c:122` — dsp_pct/worker_pct divide by window_us without zero
  guard → inf → int cast UB.
- [x] **T40** `app_config.c:170` — gain_mode from NVS not range-validated before enum cast.
- [x] **T41** `serial_cmd.c:296` — uart driver init return codes ignored.
- [x] **T42** `http_server.c:1233` — capture_start body >128 B left unread → keep-alive
  desync. Fix: return 413.
- [x] **T43** `esp_libusb.c:158` — response_buf calloc unchecked (fixed earlier); MPS hardcode
  60 (:85) + the whole dead sync bulk path (rtlsdr_read_sync + esp_libusb_bulk_transfer,
  zero callers) removed in 2906fe6. Build-verified; streaming sanity via the production reflash.
- [x] **T44** `signal_buffer.c` — burst_valid aliased past one ring lap (~420 ms). Fixed in
  8a19ce3: 64-bit monotonic s_head_total (seqlock-published), tagger's cumulative index carried
  un-truncated, absolute-space staleness check. Smoke-verified RAW matched=62/65 exact=44 BER=1.39%
  (behavior-neutral on the corpus; only the stale-beyond-one-lap case changes). head_total advances
  at the single head exit site → T2 index invariant holds for both. DMA-timeout double-give (:241)
  was the earlier partial; never fired.
- [x] **T45** `worker_core1.c:713` — final decim chunk 8 mod 10 (make WB_PRE_PAD 320);
  gold static_assert (:138); non-atomic volatile hist RMW (:79).
- [x] **T46** `frame_queue.c` — full 2064 B memcpy per push/pop regardless of n_bits (~5× waste).
- [x] **T47** `ira_decode.c`/`ibc_decode.c` — missing `frame->type` assertion (ida/ims have it).

## OPT — optimisation / throughput

- [x] **T48** `class_driver.c` — DONE (fbcf30e). Split into usb_pump (prio 7, event-only)
  + dsp_feed (prio 6) Core-0 tasks; slot protocol moved verbatim; latency/structure fix
  (Core 0 saturates ~8 MB/s so no MB/s gain at 4.88, as designed). Smoke-verified.
- [~] **T49** `esp_libusb.c` + `ingest_core1.c` — SPLIT: **T49a DONE** (473b81b) zero-copy
  usbring replacing the IDF ringbuffer (consumer copy removed, s_raw deleted, +32 KB DMA-INT).
  **T49b DONE** (ed4ef38) convert→resample 4 KB SRAM-tile fuse — decode-neutral
  (unblocked by T56 FIR pinning + HTTP-buf-to-PSRAM reclaim); saves ~19.5 MB/s PSRAM. **T49c** (resample→
  signal_buffer scratch) not started.
- [~] **T50** `fft_burst_tagger.c:364` — window_multiply → PIE 8-lane. **Harness DONE**
  (3bf50a3, bit-exact golden gate); **PIE kernel banked** (device-attended, ~6-10% of DSP
  frame). Resample chunk-continuity proven via a sibling model test.
- [x] **T56** `uw_correlator.c` + `direct_if_decim.c` — DONE (53327e7). Pin the 3 lazy PIE FIR
  delay lines (D13-LP `s_start_lp_fir`, RRC `s_rrc_fir_i/q`, decim `s_decim.fir_dsp_i/q`)
  in the boot early-alloc dance + backfill esp_ptr_in_dram guards on s_coeffs_pp/s_fft_scratch.
  Fixes the residual ~3-frame heap-layout decode lottery left after T55.
- [x] **T57** `rotate_to_dc.c` — DONE (ab5d58d, doc-only). Verified the every-burst PIE
  rotate on worker_task's PSRAM stack is decode-neutral (A/B internal vs PSRAM = identical
  matched=62); corrected a comment claiming a non-existent `ROT_SIMD_DIAG` validation.
- [x] **T51** `worker_core1.c:591` — per-burst ESP_LOGI in dequeue hot path steals worker
  CPU during burst storms.
- [x] **T52** `sd_capture.c:504` — takes s_stats_mu every USB-ingest call to read cap/target;
  use _Atomic.
- [x] **T53** `frame_decoder.c:499` — three 2 KB copies + unconditional memset of unused
  bits tail per frame.
- [x] **T54** `signal_buffer.c:153` — root-cause why async-memcpy takes split-RX path when
  src/dst/len all 64-aligned; eliminating it makes stash_fails structurally zero (T2/DMA-INT).

## Enhancements (post-review, from live bench work)

- [ ] **T58** `http_server.c` config form + `config_post` — expose **gain_mode / gain_dbx10**
  (and likely **tag_thr**) in the web `/config` UI. Today they are serial-only
  (`serial_cmd.c`: `set gain_mode|gain_dbx10|tag_thr`), so switching to SOFTWARE_AGC or
  trimming front-end gain requires a UART session. Motivation (2026-07-05 bench): a better
  antenna + external LNA on hardware TUNER_AGC overdrove the front end — detect stage
  saturated (dsp_cap 98%), USB ring dropping ~21/s, worker fell behind so bursts went stale
  and burst_valid dropped them → 0 decodes despite SNR now reaching 31 dB. Fix was
  `set gain_mode 2` (SOFTWARE_AGC, auto-reduces gain on saturation) over serial. Making that
  a UI setting means operators can do it without serial. Add the field to the args struct +
  HTML form + `form_field` parse (mirror the `bias_tee` checkbox), pick a sensible widget
  (dropdown TUNER_AGC/MANUAL/SOFTWARE_AGC + a manual-gain number box). Gate on confirming the
  SOFTWARE_AGC approach actually recovers decodes with the better antenna.
  NOTE (2026-07-05): SOFTWARE_AGC did NOT help — it only reduces gain on ADC near-clipping
  (peak_dev > 110), and this overload is not clipping-driven. Real bottleneck is the detector
  (see T59).

- [ ] **T59** graceful overload handling (detector congestion collapse). Observed 2026-07-05
  with a better antenna+LNA: the fft_burst_tagger (dsp_feed) saturates at dsp_cap=98%, falls
  behind real-time, and by the time it dispatches a burst the ring has overwritten that
  burst's samples → signal_buffer_burst_valid rejects it → worker skips 100% of bursts
  (processed=0) while sitting at worker_cap=0%. Maximum work, zero output — a livelock. The
  worker is NOT the bottleneck (idle, spare capacity); the detector starving it of fresh work
  is. Treatments, most-impactful first:
  (1) Load-shed at the detector to keep it real-time — adaptively raise the effective
      threshold / cap peaks or active-bursts per tile when dsp_cap is high, so the tagger
      stays real-time and emits FRESH bursts. (Static tag_thr raise is the manual version but
      it's boot-only today — dsp_processor.c:210 reads it once at create.)
  (2) Freshness-first queue — the burst queue (worker_core1.c) is FIFO and xQueueSend drops
      the NEWEST when full (~line 1007); under overload that's backwards. Prefer newest
      (drop-oldest / LIFO) so worker effort lands on decodable bursts.
      PREREQUISITE — capture-position timestamps: today the PDU/message timestamp is stamped
      at DECODE time (worker_core1.c:605 pdu.timestamp_us = esp_timer_get_time(); same at
      frame_decoder.c:523). LIFO would then hand out timestamps in processing order → a burst
      captured earlier but decoded later gets a LATER stamp = corrupted ordering. Fix: derive
      the timestamp from burst.start_sample_idx (the T44 64-bit cumulative sample index):
      timestamp_us = stream_start_epoch_us + start_sample_idx * 1e6 / FS_DETECT_HZ, anchored
      by the esp_timer value captured at stream start. Then any processing order preserves
      correct capture-time stamps; also removes the current decode-latency jitter (tens of ms,
      worse under load) even in FIFO. Downstream consumers that assume in-order ARRIVAL need a
      small reorder buffer, but the timestamp VALUES stay correct.
  (3) Reject already-stale bursts before enqueue — cheap early-out in dispatch.
  Also make tag_thr live-reloadable (re-read cfg per tile or on a config-changed flag) so (1)
  and manual threshold tuning don't need a reboot.

  STATUS 2026-07-05 (mostly landed): shipped (2) as an **SNR priority queue** (146bd56:
  bounded 64-slot buffer, evict-weakest, decode-strongest-first) + **capture-position
  timestamps** (its prerequisite) + a **band-occupancy freq histogram** (25166e0). Live
  result: turned the 0-decode congestion collapse into real decoding — worker_cap 0→65-215%,
  stale drops 9921→23, and under extreme bench RFI (~646/s) the PQ sheds **99.4% at insert**
  with the worker never overwhelmed (drops=0). Fixed a load-dependent crash the PQ exposed —
  the 1536 B ISR stack overflowed under the cross-core IPC churn (963fd5a → 4096; see
  [[feedback_isr_stack_overflow_under_load]]). REMAINING GAP: the priority KEY is raw SNR, and
  at this bench the loudest signal (28-31 dB) is broadband interference that doesn't decode, so
  the PQ feeds the worker interference and sheds the real (12-24 dB) Iridium — see T60. The
  detector-side load-shed (1) and live tag_thr (T58) are still open.

- [ ] **T60** smarter priority key (separate real Iridium from broadband interference). The
  T59 PQ ranks by peak_snr_db, but "loudest ≠ most decodable" when the RF floor has strong
  broadband RFI (bench 2026-07-05: 28-31 dB interference dominates; real Iridium sits at
  12-24 dB and gets shed; ~0.7% of the strongest bursts decode). Candidate keys, cheapest
  first: (a) **narrowband-ness** — Iridium channels are ~41.67 kHz; broadband RFI is wide.
  Plumb the tagger's burst bin-width (fft_burst_tagger tracks active bins) into
  detected_burst_t and prefer narrow bursts. Strongest single discriminator. (b) **duration
  match** — real bursts ~8-90 ms simplex (≤250 ms multi-frame duplex); score by fit to that
  window using existing length_samples. (c) **adaptive SNR window** — learn the SNR band where
  BCH actually passes and prioritise it (feedback loop). Combine into a score, keep SNR as a
  tie-breaker. Does NOT touch detection (still gri-aligned) — only decode-budget allocation.

- [ ] **T61** dynamic decode depth (go deeper when load is low). The worker is NOT saturated
  now (worker_cap 0-68%, drops=0) — it has spare capacity but the PQ feeds it few decodable
  bursts. When the burst rate is low, spend the spare budget on DEEPER decode (chase-decode,
  wider UW search, more retries) to catch marginal real bursts; when high, stay shallow to keep
  up. Gate the per-burst effort on current queue depth / recent load. Complements T60 (fix
  WHAT we decode first, then HOW deep). Lower priority than T60 — selection, not capacity, is
  today's limiter.
