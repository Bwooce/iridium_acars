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
- Remaining open: T4t (sd_capture framing test) + T9–T54.

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

## Follow-up test coverage (tracked 2026-07-04)

- [x] **T2t** signal_buffer wrap-desync (T2) has NO automated regression — device-only
  code. Add a SMOKE_TEST sub-test that arms `FI_SITE_DMA_SUBMIT_WRAP`, pushes across a
  ring wrap, and asserts a marker sample is still retrievable at its cumulative index
  (no desync). Optionally a host model test of the index arithmetic. Rides the pre-push
  smoke gate already enforced for signal_buffer. **Highest-value gap** — T2 is the one
  decode-affecting bug and self-masks as "bad antenna/RF".
- [ ] **T4t** sd_capture atomic-record framing (T4) has no automated regression. Factor
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

- [ ] **T9** `uw_correlator.c:1411` — bug, verified. float→int64 magnitude bridge
  overflows int64 (UB) on strong burst; RISC-V saturates both dirs → tie → always UL.
  Fix: direction-pick + SNR in float/double.
- [ ] **T10** `fft_burst_tagger.c:597` — bug, verified. `baseline_sum` int32 can
  overflow under coherent strong carrier in un-primed first 512 steps (code says
  "needs int64"). Fix: widen or clamp per-bin mag².
- [ ] **T11** `resample_256_to_250.c:346` — bug, verified. scalar MAC store
  `(int16)(acc>>15)` no saturation, PIE path saturates → host/device divergence at
  full-scale, breaks bit-exact parity gate. Fix: saturate scalar store.
- [ ] **T12** `sbd_reassembler.c` + `ida_decode.c:183` + `frame_decoder.c:332` — bug.
  Session gate checks `ida.ok && header_ok` but NOT `crc_ok` (BCH false-positives
  pollute reassembly); multi-IDA-frame packets unrecoverable (da_cont/da_ctr unused);
  payload_len fixed 20/22 not da_len → filler bytes leak to ACARS parser.
- [ ] **T13** `wifi_link.c:60` — bug. 5 s `vTaskDelay` in disconnect handler blocks
  shared default event loop during AP flaps. Fix: esp_timer for reconnect.
- [ ] **T14** `ota_runner.c:70` — sec. No image authenticity (no cert, project_name
  only logged) + unauthenticated LAN/open-AP POST /ota. Fix: reject project_name
  mismatch; token-gate /ota + /sd/format.
- [ ] **T15** `frame_link.c:126` — bug. SPI slave ISR callbacks call flash-resident
  `gpio_set_level` → panic during OTA cache-disabled window. Fix: IRAM gpio or gpio_ll.
- [ ] **T16** `frame_link.c:356` — bug. Loopback selftest passes primary handshake
  GPIO to slave instead of `..._LB_HANDSHAKE_GPIO`; passes only via fixed 5 ms delay,
  never exercises handshake.
- [ ] **T17** `burst_pipeline.c:227` — perf, verified. Pre-rotation rotates entire
  remaining burst (~16k cplx) per try, only ~1911 consumed; ~1-2 ms/burst × 20 retries.
  Also truncating `>>15` (no rounding) accumulates bias per retry. Fix: bound rotate.
- [ ] **T18** `class_driver.c:114` — bug. `rtldev` TOCTOU UAF: AGC uses it unlocked
  while `action_close_dev` NULLs it on hot-unplug.
- [ ] **T19** `librtlsdr.c:1478` — bug. `rtlsdr_close()` fully commented out; nothing
  freed on DEV_GONE, teardown can't complete → replug-requires-reboot. Document or fix.
- [ ] **T20** `librtlsdr.c:1346` — bug. `rtlsdr_open` panics via `ESP_ERROR_CHECK` on
  transient open/claim failure (boot-loop vs retry); `driver_obj` calloc unchecked.
- [ ] **T21** `sd_capture.c:186` — bug. Partial short `fwrite` leaves file position
  mid-sector → reintroduces the known EIO cliff. Fix: fseek back to 512 boundary.
- [ ] **T22** `frame_decoder.c:386` — bug. DECODER_STACK=6144 tight vs ~2.1 KB stack
  `frame_queue_item_t` + process_one + libacars + vsnprintf(256). Fix: static item or 8-12 KB.
- [ ] **T23** `ida_decode.c:141` — robustness. Partial-decode compacts skipped blocks
  left → misaligned bitstream for positional consumers. Fix: zeros at true pos + ok-mask.

## P3 — Low

- [ ] **T24** `burst_pipeline.c:411` — DC removal int16 wrap near rails; use q15_saturate.
- [ ] **T25** `direct_if_decim.c:203` — host acc starts 0 vs device 0x7fff round const +
  unsaturated stores → ~1 LSB host/device parity drift; drops `n_in%10` remainder.
- [ ] **T26** `resample_256_to_250.c:136` — silent malloc-fail leaves coeffs unwritten.
- [ ] **T27** `fft_sc16_2048.c:62` — alloc-fail path no ESP_LOGE → silent no-op tagger.
- [ ] **T28** `fft_burst_tagger.c:340` — flush() doesn't drain in-flight helper / reset
  pipe_in_flight (latent; helper disabled).
- [ ] **T29** `sym_timing.c:159` — unclamped negative v drives strobe_idx backwards (module unwired).
- [ ] **T30** `uw_correlator.c` — PIE-FFT partial-alloc leak (:480), silent no-op on init
  fail emits confident wrong result (:494), non-reentrant static scratch (:1313),
  r*r+i*i UB at -32768 (:1665).
- [ ] **T31** `bch_decoder.c:49` — lazy init sets inited=true before syn_ra filled →
  cross-core reader accepts garbage as 0-errs. Fix: set flag last.
- [ ] **T32** `iridium_bch.c:10` — bits_to_u32 truncates n_bits>32; all-zero always passes
  (matches upstream; feeds BC/RA false-positive stats).
- [ ] **T33** `sbd_reassembler.c:225` — noise msg_cnt (≤255) opens sessions dying only by
  5 s timeout → 8-slot table saturates, drops real multi-frame; session match ignores type (:70).
- [ ] **T34** `tuner_r82xx.c:603` — usleep_range → esp_rom_delay_us busy-spins ≤2×10 ms in
  PLL lock; latent, a runtime retune stalls Core 0. Fix: vTaskDelay.
- [ ] **T35** `frame_decoder.c:135` — get_rolling_rates double-counts most recent hour
  (24 h figure up to 2× inflated, spans up to 25 h).
- [ ] **T36** `sd_log.c:157` — esc buffer 2× assumption vs 6× `\u00XX` → silent truncation ~519 B.
- [ ] **T37** `sd_log.c:485` — s_stats.log_path/log_open written without s_stats_mu (torn read);
  fflush/fsync unchecked (:261), messages_written increments pre-commit.
- [ ] **T38** `worker_core1.c:619` — burst_valid TOCTOU: producer advances head during
  multi-ms read, marginal bursts partially overwritten. Fix: re-validate after read.
- [ ] **T39** `status_logger.c:122` — dsp_pct/worker_pct divide by window_us without zero
  guard → inf → int cast UB.
- [ ] **T40** `app_config.c:170` — gain_mode from NVS not range-validated before enum cast.
- [ ] **T41** `serial_cmd.c:296` — uart driver init return codes ignored.
- [ ] **T42** `http_server.c:1233` — capture_start body >128 B left unread → keep-alive
  desync. Fix: return 413.
- [ ] **T43** `esp_libusb.c:158` — response_buf calloc unchecked; MPS hardcoded 64 (:85, HS=512).
- [ ] **T44** `signal_buffer.c:359` — burst_valid aliases past one ring lap (~420 ms); needs
  64-bit cumulative head. DMA-timeout double-give semaphore (:241, never fired).
- [ ] **T45** `worker_core1.c:713` — final decim chunk 8 mod 10 (make WB_PRE_PAD 320);
  gold static_assert (:138); non-atomic volatile hist RMW (:79).
- [ ] **T46** `frame_queue.c` — full 2064 B memcpy per push/pop regardless of n_bits (~5× waste).
- [ ] **T47** `ira_decode.c`/`ibc_decode.c` — missing `frame->type` assertion (ida/ims have it).

## OPT — optimisation / throughput

- [ ] **T48** `class_driver.c:343` — the ~5 MB/s ceiling is structural: lock-stepped
  handle_events→drain 16 KB→blocking dsp_feed. Levers: drain multi-block per feed,
  32-64 KB blocks, move feed off read path.
- [ ] **T49** `esp_libusb.c:216,414` + `ingest_core1.c:235` — every byte crosses PSRAM
  3-4× (~19 MB/s avoidable). Remove consumer copy (RingbufferReceiveUpTo returns ptr),
  fuse convert→resample via SRAM staging, PIE-vectorise convert.
- [ ] **T50** `fft_burst_tagger.c:364` — window_multiply 2048 scalar Q15 muls/step → PIE 8-lane.
- [ ] **T51** `worker_core1.c:591` — per-burst ESP_LOGI in dequeue hot path steals worker
  CPU during burst storms.
- [ ] **T52** `sd_capture.c:504` — takes s_stats_mu every USB-ingest call to read cap/target;
  use _Atomic.
- [ ] **T53** `frame_decoder.c:499` — three 2 KB copies + unconditional memset of unused
  bits tail per frame.
- [ ] **T54** `signal_buffer.c:153` — root-cause why async-memcpy takes split-RX path when
  src/dst/len all 64-aligned; eliminating it makes stash_fails structurally zero (T2/DMA-INT).
