# DMA-capable Internal-SRAM Budget Audit — unblocking the frequency scanner

Date: 2026-07-09
Scope: READ-ONLY code audit. No code changed, no hardware touched.
Goal: find how to free / stop-churning the `MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL`
heap so a periodic LO retune ("hop"/scan) runs without wedging the tuner.

> **Platform note:** on the ESP32-P4, the internal HP-SRAM heap region carries
> both `MALLOC_CAP_INTERNAL` and `MALLOC_CAP_DMA`. Therefore **every**
> `MALLOC_CAP_INTERNAL` allocation draws from the same pool that
> `MALLOC_CAP_DMA` reports as `dma_free`/`dma_largest`. A buffer does not need
> the explicit `MALLOC_CAP_DMA` flag to consume the DMA-INT budget — plain
> `MALLOC_CAP_INTERNAL` does too. This is why the tagger struct and the PIE
> scratch (all flagged only `INTERNAL`) still starve the USB pool.

---

## TL;DR — two of the task's premises are already resolved by the code

1. **Lever #1 as framed (align the USB RX buffers to kill the stash) is
   infeasible and will free nothing.** The failing `stash` allocation does
   **not** come from the USB RX pool. It comes from the AXI `esp_async_memcpy`
   path in `signal_buffer_push()`. The pool buffers are already 64-B aligned
   (`usb_private.c:33` ALIGN_UP to cache line; #125 invariant), and the IDF
   driver `mcp_gdma_memcpy()` calloc's the ~128 B stash **unconditionally,
   before it checks alignment** — so alignment cannot remove it. See §Lever B.

2. **Lever #2 as framed (move the ~96 KB ingest raw/conv to PSRAM) is already
   done.** T49a removed `s_raw[]` (2×16 KB DMA-INT) and T49b removed `s_conv[]`;
   the current ingest internal footprint is a single 4 KB `s_tile`
   (`ingest_core1.c:45`) that is **PIE-locked to DRAM** and cannot move. The
   "~96 KB" figure in project memory is historical — that reclaim is already
   banked (it now lives in PSRAM as `s_resamp[]`, `ingest_core1.c:566`).

3. **The hop does not allocate DMA-INT on its happy path.** Pause parks URBs
   (no free), resume re-submits the same objects (no alloc), and control
   transfers reuse one pre-allocated ctrl URB. The `XFER_DESC_LIST` is
   allocated **per-pipe at device open**, not per control-submit. So a *single*
   hop works today (matches the operator note "manual hop works"). The scanner
   blocker is **sustained hopping colliding with continuous stash churn keeping
   `dma_largest` pinned near 0**, not a one-shot 8 KB allocation.

**Bottom line:** almost every internal buffer is locked (USB pool = throughput,
PIE scratch = DRAM-position correctness, tagger = decode-quality). The only
concretely-reclaimable steady-state block is **sd_capture's 8 KB writer-buf**
(§Lever A). The definitive churn fix is the **async_memcpy bypass** (§Lever B).

---

## A. Allocation table — DMA-INT consumers (our code)

All sizes are the requested bytes; actual heap cost rounds up to alignment
(16 or 64 B) plus TLSF block header (~8–12 B). "DMA-INT?" = does it consume the
`MALLOC_CAP_DMA|INTERNAL` pool (Y for any INTERNAL alloc on P4).

| # | Site (file:line) | Size | Purpose | True HW-DMA target? (who writes) | Move candidate? |
|---|---|---|---|---|---|
| 1 | `esp_libusb.c:343` (`ASYNC_TRANSFER_COUNT`×`_SIZE`, defs `esp_libusb.h:65-66`) | **6 × 8 KB = 48 KB** | USB bulk stream transfer pool | **Y** — USB-DWC OTG DMA writes RX samples here (`stream_transfer_cb` reads `transfer->data_buffer`, `esp_libusb.c:233`) | **N** — throughput-critical; history 8×16→4×8 (0.85 MB/s)→6×8. Shrinking reintroduces `rb_full_drops`. |
| 2 | `esp_libusb.c:87` via `usb_host_transfer_alloc(CTRL_XFER_BUF_SIZE…)` (def `esp_libusb.h:23`, =8+256=264 B) | ~264 B → cache-aligned | Pre-allocated control URB `data_buffer` (tuner/demod reg I/O) | **Y** — DWC DMA for EP0 control (`usb_private.c:38` DATA_BUFFER_CAPS = DMA\|INTERNAL) | **N** — one-time, tiny, reused every hop; essential to the retune path. |
| 3 | `fft_burst_tagger.c:393` `heap_caps_calloc(1, sizeof(*t), INTERNAL\|8BIT)` | **~66 KB** (`sizeof(fft_burst_tagger_t)`; smoke log "66672") | Burst-tagger state (per-bin `accum[]`, `bursts[64]`, EMA) | **N** — CPU-only, hot accum written every FFT step (~730/s) | **N (locked)** — AUDIT 2026-05-22: PSRAM spill silently regressed decode; forced INTERNAL to fail loudly. Decode-quality-locked, not merely perf. |
| 4 | `worker_core1.c:1197` `s_chunk_iq` (16 B aln, `DECIM_CHUNK_IN`×2×2 = 4000×4) | **16000 B** | Per-chunk IQ scratch for rotate-to-DC + decim | **N** — CPU-filled by `signal_buffer_read_chunk` (CPU/AXI to it) then read by PIE `rotate_to_dc_q15_simd_at` (`worker_core1.c:896`) | **N (PIE-locked)** — PIE MAC reads it; PIE garbles on non-DRAM (PSRAM). Must stay internal DRAM. |
| 5 | `worker_core1.c:1198-1199` `s_decim_scr_in_i/_q` (`(4000+16)`×2) | **2 × 8032 B ≈ 16 KB** | FIR decim input scratch (I/Q) | **N** — CPU/PIE; `dsps_fird_s16_arp4` PIE kernel reads/writes (`worker_core1.c` §1186 comment) | **N (PIE-locked)** — PIE arp4 kernel; DRAM-required. |
| 6 | `worker_core1.c:1200-1201` `s_decim_scr_out_i/_q` (`(400+16)`×2) | **2 × 832 B ≈ 1.6 KB** | FIR decim output scratch (I/Q) | **N** — PIE kernel writes | **N (PIE-locked)** |
| 7 | `ingest_core1.c:45` `s_tile` (64 B aln, `INGEST_TILE_COMPLEX`×2×2 = 1024×4) | **4096 B** | Convert+resample staging tile (T49b) | **N** — CPU convert writes; PIE `resample_arp4.S` MAC reads | **N (PIE-locked)** — explicit `esp_ptr_in_dram` guard at `ingest_core1.c:57`; refuses non-DRAM. |
| 8 | `fft_sc16_2048.c:59` `s_w_table` (16 B aln, `N`×2 = 2048×2) | **4096 B** | FFT twiddle table (N=2048) | **N** — CPU init writes; PIE-adjacent FFT reads | **N (PIE-locked)** — `esp_ptr_in_dram` guard at :80. |
| 9 | `fft_sc16_2048.c:61` `s_fft_scratch` (16 B aln, `2N`×2) | **8192 B** | FFT in-place scratch | **N** — CPU/PIE FFT | **N (PIE-locked)** — same guard. |
| 10 | `resample_256_to_250.c:224` `s_coeffs_pp` (16 B aln, `RS25_PADDED_TAPS`×`RS25_INTERP`×2 = 16×125×2) | **4000 B** | Polyphase resampler coeffs | **N** — CPU init; PIE asm reads | **N (PIE-locked)** — needs magic address 0x4ff7e300; `esp_ptr_in_dram` guard at :236. THE early-alloc-dance anchor. |
| 11 | `sd_capture.c:412` `s_writer_buf` (64 B aln, `WRITER_RECV_CHUNK`) | **8192 B** | SDMMC fwrite bounce buffer | **Y** — SDMMC host DMA reads it during `fwrite` (PSRAM source → ENOSPC, must be DMA-INT; `sd_capture.c:407`) | **Y (conditional)** — **eager at boot** but capture is normally idle; a lazy path already exists (`sd_capture_alloc_writer_buf`, `sd_capture.c:456`). See §Lever A. |
| 12 | `dsp_processor.c:314` `heap_caps_calloc(1, sizeof(*p), INTERNAL)` | small (~1–2 KB, `sizeof(dsp_processor_t)`) | DSP handle (hot accum fields) | **N** — CPU-only | **N (perf)** — comment: perf-only placement, holds no PIE buffers. Minor; not worth moving. |
| 13 | `frame_link.c:222/223/260/261/384-386` `heap_caps_aligned_alloc(64, FRAME_LINK_XFER_SIZE, DMA)` | 3–6 × `FRAME_LINK_XFER_SIZE` | SPI inter-P4 frame link TX/RX/status | **Y** — SPI DMA | **N/A for single-P4 SDR** — role-gated to `CONFIG_DEVICE_ROLE_WORKER`/`AGGREGATOR` (`usb_host_lib_main.c:303,311`). Not allocated in the single-SDR scanner role. |

### Not in the DMA-INT budget (already in PSRAM — do not re-propose)
- `ingest_core1.c:566` `s_resamp[2]` = 2×40 KB — **PSRAM** (`MALLOC_CAP_SPIRAM|DMA`). AXI-GDMA reads from it into `signal_buffer`. This IS the ex-`s_conv`/`s_raw` reclaim (T49a/T49b).
- `worker_core1.c:1196` `s_decim_buf` — PSRAM.
- `dsp_processor.c:291` `baseline_history` 4 MB — PSRAM.
- `fft_burst_tagger.c:490/496/498` `fft_buf_alt`, `staged_new`, `staged_gone` — **PSRAM** (the header's "~120 KB internal total" is STALE; only the 66 KB struct is internal now).
- `signal_buffer.c:221` `circular_buf` 16 MB, `:282` `s_align_scratch` 32 KB — PSRAM.
- All task stacks (resample workers, sd_capture, worker) — deliberately PSRAM (`MALLOC_CAP_SPIRAM`) to avoid fragmenting internal SRAM.
- IDF/USB-host internals: `hcd_dwc.c:1061` frame_list 4 KB (one-time at HCD init), per-pipe `XFER_DESC_LIST` (INTERNAL, per-pipe not per-hop), esp_hosted C6-SDIO DMA buffers (live only with Wi-Fi up). These are real DMA-INT consumers but out of our direct control.

---

## B. Total DMA-INT budget & the boot NO_MEM

**Our hot-path SDR consumers (steady state, capture idle, Wi-Fi up):**

| Block | KB |
|---|---|
| USB stream pool (#1) | 48.0 |
| Tagger struct (#3) | ~66.0 |
| Worker PIE scratch (#4-6) | ~33.6 |
| FFT tables+scratch (#8,9) | 12.0 |
| Ingest tile (#7) | 4.0 |
| RS25 coeffs (#10) | 4.0 |
| sd_capture writer-buf (#11) | 8.0 |
| ctrl URB + dsp handle (#2,12) | ~1.5 |
| **Subtotal (our code)** | **~177 KB** |

Plus USB-host stack (frame_list 4 KB + pipe desc lists) and esp_hosted C6 DMA
buffers, which push the live DMA-INT working set higher.

**Boot NO_MEM explained.** The log
`transfer_alloc #1 failed: NO_MEM, DMA-internal free=8KB largest=7KB (needed 8)`
is the **streaming pool** transfer (`ASYNC_TRANSFER_SIZE`=8 KB, `esp_libusb.c:343`),
**not** a control transfer — the "needed 8" is 8 KB, not the 264 B ctrl URB. It
fails because by the time `submit_stream_transfers()` runs (at stream start),
the tagger's 66 KB (`dsp_processor_init`) and the rest have already consumed the
pool: the `:405` sd_capture comment records the trajectory — **139 KB largest at
`sd_capture_init` (early boot) → only ~2–3 KB largest after USB+DSP init**. The
pool therefore under-fills (fewer than 6 transfers get allocated), and the
steady-state `dma_largest≈0` leaves no slack for the per-push stash.

**The stash churn (root cause, `signal_buffer.c:247` block comment / T54).**
`signal_buffer_push()` → `esp_async_memcpy()` → IDF `mcp_gdma_memcpy()`
unconditionally `heap_caps_calloc()`s a 2×cache-line (~128 B) stash from
`MALLOC_CAP_DMA|INTERNAL` on **every** call, **before** computing whether a
head/tail split is even needed (our 64-aligned transfers need none). With
`dma_largest≈0` that ~128 B alloc fails ~1/sec → `stash_fails` → the #126E
CPU-memcpy fallback recovers losslessly (`audio_dropped=0`) but burns Core 0 and
keeps the pool thrashing. Alignment cannot fix this — the driver allocates
before checking alignment, and there is no public API to hand it a persistent
stash (documented in the `signal_buffer.c:247` comment).

**Shortfall:** we need `dma_largest ≥ ~8 KB` sustained — enough to (a) let the
stash's ~128 B alloc always succeed (stop the churn), and (b) cover any
transient the hop's control-transfer flurry induces. Today it sits at ~0–2 KB.

---

## C. Ranked levers

### Lever A — make sd_capture's 8 KB writer-buf lazy  (RECOMMENDED FIRST)
- **Frees:** 8 KB DMA-INT in steady state (capture is normally off), lifting
  `dma_largest` from ~0 to ~8 KB — exactly the shortfall.
- **Sketch:** stop the eager `heap_caps_aligned_alloc` at `sd_capture.c:412`;
  instead call the **already-present** `sd_capture_alloc_writer_buf()`
  (`sd_capture.c:456`) on `POST /capture/start`, and free on `/capture/stop`.
  Capture is a deliberate operator action; a scan and a raw-capture are
  mutually exclusive workflows.
- **Tension (must handle):** the `:405` comment placed it early precisely so it
  wouldn't fail later when `dma_largest` is only 2–3 KB. Deferring means the
  writer-buf alloc at `/capture/start` faces a tight pool. Mitigation: on
  `/capture/start`, pause the stream (the retune path already has
  `esp_libusb_pause_stream`) or accept that capture during a scan needs the
  scan stopped. Deferring writer-buf **helps** the tagger's 66 KB contiguous
  alloc (more contiguity at DSP init), so it does not regress boot.
- **PIE risk:** none — `s_writer_buf` is not PIE-touched, and it is allocated
  *after* the early-alloc dance regardless.
- **Errata risk:** none.
- **Smoke validation:** full device-smoke (RAW/FRAME/REAL) to confirm decode
  unchanged; manual `/capture/start` + `/capture/stop` to confirm capture still
  works; check `stash_fails` rate drops after the 8 KB is reclaimed.

### Lever B — bypass esp_async_memcpy in signal_buffer (ROOT-CAUSE, definitive)
- **Frees:** ~0 KB, but **eliminates the failing ~128 B/push stash allocation
  entirely** and stops the `dma_largest` oscillation — the true fix for
  *sustained* hopping.
- **Sketch:** replace `esp_async_memcpy_install_gdma_axi` +
  `esp_async_memcpy()` (`signal_buffer.c:265`, `signal_buffer_push`) with a
  hand-driven `gdma_link_list` on our own AXI-GDMA channel (own channel setup,
  link-list construction, explicit `esp_cache_msync`). This is the escape the
  `signal_buffer.c:247` comment names ("materially larger and riskier… not
  attempted here"). Our src/dst/len are already 64-aligned (#125), so no split
  is ever needed — we simply never allocate a stash.
- **PIE risk:** none — signal_buffer is not on the PIE path and holds no PIE
  buffers; no internal-SRAM buffer relocates, so the early-alloc dance is
  untouched.
- **Errata risk:** none (AXI-GDMA, not USB-DWC).
- **Effort/risk:** medium-large rewrite of the DMA push path; needs a bit-exact
  correctness harness (the existing CPU-memcpy fallback is a ready oracle).
- **Smoke validation:** full device-smoke + a sustained-hop soak (drive
  repeated `class_driver_retune`) watching `stash_fails` (should go to 0) and
  `worker[dropped=]`.

### Lever C — re-enable USB DWC DMA-from-PSRAM  (LAST RESORT, constraint #2)
- **Frees:** ~48 KB (moves the whole stream pool #1 + ctrl URB #2 to PSRAM via
  `DATA_BUFFER_CAPS`, `usb_private.c:22`).
- **Errata risk (blocking):** `CONFIG_USB_HOST_DWC_DMA_CAP_MEMORY_IN_PSRAM` is
  deliberately **disabled** for hardware-errata hardening of USB-DWC
  DMA-from-PSRAM. Re-enabling reintroduces that exposure and may hurt USB
  throughput/latency (extra PSRAM contention on the hot RX path).
- **PIE risk:** none.
- **Recommendation:** do **not** use unless A+B prove insufficient; if ever
  used, gate behind a soak test for USB stalls and revisit the errata.

### Non-levers (documented so they aren't re-tried)
- Aligning/padding USB RX pool buffers (task Lever #1): **won't remove the
  stash** — wrong root cause (§B). Already 64-aligned.
- Moving ingest raw/conv to PSRAM (task Lever #2): **already done** (T49a/T49b).
- Moving `s_conv` again: spent (now `s_tile`, PIE-locked).
- Shrinking the USB pool below 48 KB: reintroduces `rb_full_drops`.
- Moving the tagger 66 KB or any PIE scratch to PSRAM: breaks decode
  (quality-locked / PIE-DRAM-locked).

---

## D. Recommended sequence to unblock the scanner

**Necessary vs optional:** For a *single* hop, nothing is needed — it works
today. For a reliable *scan* (repeated hops under live streaming), the churn
must stop or be given headroom.

1. **Lever A first (necessary, low-risk, ~1–2 h).** Defer sd_capture's 8 KB to
   the lazy `/capture/start` path. This alone lifts `dma_largest` to ~8 KB,
   which should let the per-push stash alloc succeed (churn stops failing) and
   cover the hop's transients. This is very likely sufficient to unblock the
   scanner on its own. Validate with device-smoke + a short hop soak; if
   `stash_fails` stops climbing during a scan, ship it.
2. **Lever B if A leaves residual churn (definitive, medium-large).** Only if
   the soak still shows `stash_fails` or `worker[dropped=]` under sustained
   hopping. Removes the root cause permanently. Gate on a bit-exact harness
   (CPU fallback as oracle) + full device-smoke + long hop soak.
3. **Lever C only if A+B insufficient (last resort).** Accept the USB-DWC
   errata exposure knowingly; soak for USB stalls.

**Mandatory gate for all of the above:** any DSP-path or DMA-path commit
requires device-smoke (`.githooks/pre-push` enforces `Smoke-verified:`), because
host-suite green does not predict device decode.

---

### Appendix — key evidence cites
- Stash root cause & "alignment can't fix it": `signal_buffer.c:225-270` (T54 block comment).
- USB pool defs & boot NO_MEM site: `esp_libusb.h:65-66`, `esp_libusb.c:343-354`.
- Hop path is alloc-free: `esp_libusb.c:442-490` (pause parks, resume re-submits), `:150-176` (ctrl URB reused).
- Tagger forced-INTERNAL, decode-quality lock: `fft_burst_tagger.c:387-405`; other tagger buffers now PSRAM: `:448,490,496,498`.
- PIE-DRAM guards (why scratch can't move): `ingest_core1.c:57`, `fft_sc16_2048.c:80`, `resample_256_to_250.c:236`.
- sd_capture eager alloc + lazy alt + contiguity tension: `sd_capture.c:405-421,456`.
- Ingest reclaim already done: `ingest_core1.c` T49a/T49b comments (`:20-27,540-560`).
