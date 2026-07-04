# Perf / decoupling design — T48, T49, T50 (2026-07-04)

Design + task breakdown for the three deferred throughput findings from
`docs/review-2026-07-04-findings.md`. This is a **proposal document**, not a
merge: nothing here is implemented. Every claim below was re-verified against
the source on 2026-07-04 (commit 716727e); where the original finding's line
numbers or magnitudes were wrong, the correction is stated explicitly.

Context that bounds all of this work: the firmware is currently
**device-capped at 4.88 MB/s with 0 drops** (the RTL-SDR v4's actual streaming
rate — AGENTS.md "Throughput status"). None of these changes will move the
throughput number on the current dongle. They are *headroom* work: they matter
when the input rate rises (HydraSDR, multi-channel, higher sample rates) and
they buy Core-1 CPU back for the worker today. Wins must therefore be
quantified as **PSRAM bus bandwidth removed**, **Core-0/Core-1 µs removed**,
and **DMA-INT bytes freed** — not as MB/s gained, which is unmeasurable until
a faster source exists.

---

## 1. Verified pipeline map — where every byte is copied

Data path (all line numbers re-read 2026-07-04):

```
RTL-SDR bulk-IN (USB DWC OTG, AHB master)
  └─ DMA → transfer->data_buffer            8 KB URBs × 8, DMA-INT SRAM
       (esp_libusb.h:50-51 ASYNC_TRANSFER_COUNT=8, ASYNC_TRANSFER_SIZE=8 KB)
  [C1] stream_transfer_cb: xRingbufferSend  esp_libusb.c:273
       → 4 MB PSRAM byte ringbuffer (esp_libusb.c:396, STREAM_RINGBUF_BYTES)
  [C2] esp_libusb_read_stream: memcpy       esp_libusb.c:471-474
       ring item → s_raw[slot] (16 KB, DMA-INT!  ingest_core1.c:393-394)
       called from class_driver.c:419 with out_block_size = 16 KB (:300)
  [C3] ingest_task convert uint8→int16 Q15  ingest_core1.c:233-244
       s_raw (internal) → s_conv[slot] (PSRAM, ingest_core1.c:391-392)
  [C4] resample 125/128 polyphase           ingest_core1.c:271-276
       reads s_conv (PSRAM) → writes s_resamp[slot] (PSRAM, :399-400)
  [C5] signal_buffer_push staging memcpy    signal_buffer.c:284-287
       s_resamp (PSRAM) → s_align_scratch (PSRAM, 64 KB, :206-207)
  [C6] AXI-GDMA async memcpy                signal_buffer.c:306/317-325
       s_align_scratch (PSRAM) → circular_buf 4 MB PSRAM lookback ring
  [C7] dsp_processor_feed accum memcpy      dsp_processor.c:262-264
       s_resamp (PSRAM) → p->accum (internal SRAM, 8 KB)
  └─ fft_burst_tagger_step (Core 0)         fft_burst_tagger.c:684
       window (:364) → fft_sc16_2048 → mag (:395) → detect → EMA (:615)
  └─ worker_core1 (Core 1) reads bursts back out of circular_buf
       via signal_buffer_read_chunk (worker_core1.c:722) — burst-rate only
```

### PSRAM bus traffic at steady state (4.88 MB/s uint8 ingress)

uint8 stream = 4.88 MB/s; after uint8→int16 = 9.77 MB/s; after 125/128
resample = 9.54 MB/s. Each row is a PSRAM bus transaction stream (R = read,
W = write; GDMA traffic contends on the same PSRAM bus as CPU AXI traffic —
AGENTS.md: "PSRAM contention is the real ceiling, not CPU"):

| # | Site | Dir | MB/s | Avoidable? |
|---|------|-----|------|-----------|
| C1 | cb → stream ringbuf | W | 4.88 | No — this IS the elastic buffer |
| C2 | ring → s_raw memcpy | R | 4.88 | **Yes — T49a (pointer consume)** |
| C3 | convert → s_conv | W | 9.77 | **Yes — T49b (SRAM tile fuse)** |
| C4a | resample reads s_conv | R | 9.77 | **Yes — T49b** |
| C4b | resample writes s_resamp | W | 9.54 | No (some PSRAM sink needed) |
| C5a | push reads s_resamp | R | 9.54 | **Yes — T49c (write into scratch)** |
| C5b | push writes s_align_scratch | W | 9.54 | **Yes — T49c** |
| C6 | GDMA scratch→ring | R+W | 19.08 | No — the lookback ring is the product |
| C7 | feed reads s_resamp | R | 9.54 | No (tagger input must come from somewhere; already lands in internal accum) |

**Total ≈ 96.5 MB/s of PSRAM bus traffic to move a 4.88 MB/s stream.**

### Correction to the T49 finding

The finding said "every byte crosses PSRAM 3-4× (~19 MB/s avoidable)". Both
numbers are **understated**: counting reads and writes separately, the stream
generates ~96.5 MB/s of PSRAM transactions (~20× the ingress rate, ~10
transactions per sample), and the avoidable slice (C2 + C3 + C4a + C5a + C5b)
is **≈ 43.5 MB/s**, i.e. ~45 % of all stream-related PSRAM traffic. The
finding's cited lines also drifted: `esp_libusb.c:216` is the *control*
transfer path (irrelevant); the real stream copies are `esp_libusb.c:273`
(producer) and `esp_libusb.c:471-474` (consumer). `ingest_core1.c:235`
(convert loop, actually :233-244) was correct.

### Correction/confirmation of the T48 finding

`class_driver.c:343` is confirmed as the `while (1)` consumer loop, and it is
lock-stepped as claimed: one `usb_host_client_handle_events` (:357) → one
≤16 KB ring drain (:419) → one synchronous `dsp_processor_feed` (:465) per
iteration. Two nuances the finding missed:

1. Convert+resample+signal_buffer_push are **already off this loop** (Core 1
   ping-pong, ingest_core1.c) — the only heavy work left inline is the tagger
   feed (~1.6-2.0 ms per 16 KB block: 3.9 FFT frames × 407-515 µs/frame).
2. URB completion callbacks run **inside handle_events in this same task**, so
   while feed runs, no URBs are recycled. At 4.88 MB/s the 8×8 KB = 64 KB URB
   pool covers ~13 ms and never runs dry. At ~20 MB/s (HydraSDR-class) the
   pool covers only 3.2 ms while a proportionally larger feed runs — the
   lock-step becomes the real ceiling. Structural, exactly as claimed.

Ceiling arithmetic (honest version): the tagger feed costs ~58 % of Core 0 at
4.88 MB/s and scales linearly with input rate, so even with perfect
decoupling **Core 0 saturates at ≈ 8 MB/s** with today's per-frame DSP cost.
T48 removes the lock-step (URB starvation, latency spikes); T50 and the FFT
are what move the Core-0 saturation point. The 5 MB/s figure is the RTL-SDR's
rate, not a port or host ceiling — the actual host ceiling is unmeasured, and
a cheap probe exists (run the RTL-SDR at 3.2 MSPS = 6.4 MB/s; see §5
measurement plan).

### T50 confirmation

`fft_burst_tagger.c:364-374` confirmed: `window_multiply` does 2048 iterations
× 2 scalar Q15 multiplies + shifts (4096 muls) per FFT step, on internal-SRAM
buffers. The old hand-rolled PIE window kernel (AGENTS.md Step 3a,
`dsp_window_arp4.S`) was **removed** with the Phase 3.6.M cutover of the old
dsp_processor — the tree today has no PIE window kernel, only
`resample_arp4.S` and `rotate_to_dc_arp4.S` as reference patterns
(common/iridium_decoder/).

---

## 2. Per-finding design

### T49a — remove the consumer-side copy (zero-copy ring drain)

**Blocker discovered:** the "just use the pointer from
`xRingbufferReceiveUpTo`" lever in the finding does not survive contact with
the IDF ringbuffer: **byte buffers allow only ONE outstanding retrieval**
(`esp-idf/components/esp_ringbuf/ringbuf.c:512` — "Byte buffers do not allow
multiple retrievals before return"). The current pipeline holds two slots in
flight (ping-pong), and zero-copy requires holding the ring item across the
Core-1 convert. One outstanding item serialises acquire-next against
convert-done — tolerable (the take_converted handshake already roughly
serialises there, and the eliminated memcpy pays for it), but fragile, and it
couples the drain granularity to the wrap position.

**Design: replace `dev->ringbuf` with a small custom SPSC byte ring** (PSRAM,
power-of-two size, cached head/tail, producer = `stream_transfer_cb`, consumer
= class/feeder task):

- API: `usbring_write(buf, n)` (producer, memcpy + release-store of head);
  `usbring_peek(&ptr, &n_contig)` (consumer, returns pointer + contiguous
  length up to wrap); `usbring_consume(n)` (advance tail after Core 1 signals
  conversion done). Multiple peeks before consume are fine by construction;
  consume order is FIFO which matches the slot protocol.
- The pure index/wrap logic lives in a header (mirroring
  `signal_buffer_ring.h`) so it gets a **host unit test** (sibling of
  `test_signal_buffer_index.c`) — including a randomised producer/consumer
  interleaving model test.
- `s_raw[]` (2 × 16 KB **DMA-INT**, ingest_core1.c:393-394) is deleted. Its
  "MUST be DMA-capable" comment (ingest_core1.c:361-362) is **stale**: no DMA
  engine touches s_raw in the current architecture — the USB DMA target is the
  URB `data_buffer`, and s_raw is only ever a memcpy destination and CPU-read
  source (convert loop + `sd_capture_write`). Both readers work identically
  from a ring pointer.
- `ingest_core1_dispatch` carries `(ptr, bytes)` instead of `(slot, bytes)`;
  the convert loop reads the ring region directly (PSRAM read — this replaces
  the s_raw internal read, see cost note below); ingest signals done, and the
  consumer task calls `usbring_consume`. The `s_free`/`s_ready` semaphore
  protocol keeps its shape (slot = in-flight dispatch record, not a buffer).
- Cache-coherency note: producer (Core 0 CPU) writes and consumer (Core 1 CPU)
  reads the PSRAM ring through the shared L2 — no `esp_cache_msync` needed for
  CPU↔CPU sharing (unlike the GDMA paths in signal_buffer). Needs the
  `__sync_synchronize()` publish barrier the codebase already uses
  (ingest_core1.c:137).

**Win:** −4.88 MB/s PSRAM reads (C2 as a distinct pass; the convert's input
read still happens but is fused into C3's loop, which previously read internal
s_raw — net PSRAM read delta ≈ 0 for convert, but the separate 16 KB
memcpy pass and its Core-0 wall time, measured as part of `cycle_read_us`,
disappear entirely — ~200-400 µs/16 ms cycle of Core-0 time back).
**+32 KB DMA-INT freed** (the entire s_raw allocation) — directly relaxes
constraint #1 (pre-stream free ≥ 62 KB) and funds T49b's tile.
**Risk:** medium — custom concurrency primitive; mitigated by host model test
+ the fact that misuse deadlocks loudly (health_wdt reboots, stall diag names
the stage). Heap-position: no PIE buffers involved, but deleting s_raw shifts
every later allocation → mandatory device smoke (which is mandatory anyway).

### T49b — fuse convert→resample through an internal-SRAM tile

Today convert writes 9.77 MB/s into PSRAM `s_conv` and the resampler
immediately reads all of it back (C3 + C4a ≈ 19.5 MB/s round trip), because a
full 32 KB s_conv per slot didn't fit internal SRAM (the attempt cost L2 128 KB
and regressed −13 % throughput — ingest_core1.c:385-390).

**Design:** don't move the whole buffer — **tile it**. Convert and resample
in chunks of ~2-4 K complex samples through a single small internal-SRAM
staging tile (8-16 KB, `MALLOC_CAP_INTERNAL`), looping inside `ingest_task`:

```
for each tile of ≤ TILE_COMPLEX from ring region:
    convert uint8→int16 into s_tile (internal)
    resample_256_to_250_process_explicit(..., s_tile, tile_n, out_psram, ...)
```

The resampler is explicitly chunk-continuable — its delay-line/phase state
(`s_persist_*`, ingest_core1.c:33-36) already carries across calls, and
`test_resample_split.c` already proves bit-exactness across arbitrary chunk
splits. The 9-tap PIE MAC (`resample_arp4.S`) then reads its input from
internal SRAM instead of PSRAM, which attacks the dominant ingest cost
(resample ≈ 2.5 ms of the ~16 ms dispatch cycle, PSRAM-read-latency-bound in
the MAC inner loop).

- `s_conv[]` (2 × 32 KB PSRAM) is deleted; AGC prefix scan (ingest_core1.c:
  221-231) moves to the first tile.
- While split_pct is 0 (production default since the 2026-05-23 sweep) the
  Worker-A/B split path can keep the old behaviour or be tiled later; the
  design does not need to solve it now (it's disabled), but must not break its
  compile.

**Win:** −19.5 MB/s PSRAM (C3+C4a) and a real Core-1 CPU cut: even a
conservative 15-25 % reduction of the resample stage is ~400-600 µs per 16 ms
cycle of Core 1 handed back to the worker (ingest currently eats 62-75 % of
Core 1; worker starvation is the documented failure mode).
**Risk:** low-medium. Output must be **bit-exact** vs today (same resampler,
same state walk, different input staging) — mechanically checkable on host.
Heap-position: the tile is a **PIE-read buffer** (resample_arp4.S input) →
constraint #2 applies: allocate it in the early-alloc dance
(class_driver.c:199-200, next to `resample_256_to_250_alloc_coeffs`), log its
address, and keep struct sizes elsewhere baseline-equivalent. DMA-INT budget:
8-16 KB internal, fully funded by T49a's +32 KB; boot-log gate
(`LIBUSB: Pre-stream DMA-internal heap: free=`) must stay ≥ 62 KB.

### T49c — resample writes directly into signal_buffer's aligned scratch

Today the resampler writes `s_resamp` (PSRAM) and `signal_buffer_push`
memcpys it into `s_align_scratch` (PSRAM→PSRAM, C5a+C5b ≈ 19.1 MB/s,
signal_buffer.c:284-287) purely to establish the 64-byte-aligned DMA source
built by #125.

**Design:** an "acquire/commit" producer API on signal_buffer:

- `int16_t *signal_buffer_acquire_scratch(int slot, size_t *carry_offset)` —
  returns a per-slot 64-aligned PSRAM scratch pointer plus the byte offset the
  producer must start writing at (= current carry length, 0..15 complex).
  Needs **2 scratches** (one per in-flight slot) since dsp_feed still reads
  slot N's output while ingest fills slot N+1 (2 × 64 KB PSRAM — trivial).
- Resampler writes its output at `scratch + carry_offset`.
- `signal_buffer_commit(slot, n_complex)` — prepends the carry into the gap,
  computes the aligned length + new carry, and runs the existing submit/wrap/
  CPU-fallback machinery **unchanged** from the scratch.
- `dsp_processor_feed` and the sd-capture burst path read from the same
  scratch (replacing `s_resamp`), so `s_resamp[]` (2 × 32 KB PSRAM) is
  deleted.

**Win:** −19.1 MB/s PSRAM.
**Risk:** **highest of the three** — this reshapes the carry/alignment
invariants of the T2/T125-hardened wrap logic, whose failure mode
(occasional garbage window / index desync) self-masks as "bad RF". Gates
exist and are strong: `test_signal_buffer_index` host test, the **T2t
fault-injection smoke sub-test** (arms `FI_SITE_DMA_SUBMIT_WRAP` across a ring
wrap and asserts index integrity — proven live 2026-07-04), and the mandatory
4-variant device smoke. Do this last, after T49b settles the producer side.

### T48 — decouple the USB pump from the DSP feed

**Design: split `class_driver_task` into two Core-0 tasks.**

- **`usb_pump` (existing class task, prio 6, Core 0):** keeps
  `usb_host_client_handle_events` (URB completion → ring write → resubmit),
  enumeration actions (`action_open_dev`/`start_stream`/`close_dev`),
  root-port recovery, the no-device idle path, and the 1 Hz snapshot/stall
  watchdog (reading shared counters). Its loop becomes purely
  event-driven — no stream-path calls at all.
- **`dsp_feed` task (new, prio 5, Core 0):** runs the stream cycle —
  `usbring_peek` (post-T49a) or `acquire_raw`/`read_stream` (pre-T49a) →
  `sd_capture_write` tap → `ingest_core1_dispatch` → `take_converted(prev)` →
  `dsp_processor_feed` → `release(prev)`. Blocks on ring-empty (peek with
  timeout), so it consumes zero CPU when idle. The slot-ownership protocol
  (the #105/#106 deadlock minefield) moves **verbatim** — same acquire/
  dispatch/take/release sequence, same drain-in-flight-slot-on-empty branch
  (class_driver.c:486-495), now in one task instead of interleaved with USB
  events.
- **Priorities:** pump (6) > feeder (5) on Core 0 keeps URB service latency
  bounded — the feeder's ~2 ms feed can no longer delay completions; the pump
  preempts it. Feeder must stay **above** nothing else hot on Core 0 (Core 0
  is otherwise webserver/wifi via esp_hosted on Core 0? — no: httpd runs at
  prio 5 on Core 0; feeder at 5 round-robins with httpd, which is the current
  de-facto behaviour of feed-in-class anyway, and the existing rule "USB
  consumer must outrank httpd" is preserved by giving the feeder prio 6 and
  the pump prio 7 if measurement shows httpd interference — decide from the
  soak, both orderings keep pump > feeder).
- **Core 1 is untouched:** ingest (8) / worker (4) / frame_decoder (4) budget
  and the daemon-on-Core-1 ISR placement (usb_host_lib_main.c:337-350) are
  unchanged. The design deliberately does NOT move any feed work to Core 1 —
  Core 1 has no headroom (memory: fbt_pipe's 24 % was the difference between
  worker running and dying).
- **Multi-block drains + larger blocks:** once the feeder is decoupled, drain
  size is a policy knob. Post-T49a there is no s_raw to size — `usbring_peek`
  naturally returns up to the contiguous region (32-64 KB typical), and
  `dsp_processor_feed` already accepts arbitrary lengths (dsp_processor.c:
  246-273 accumulates into 2048-sample chunks). Cap dispatches at
  INGEST_SLOT-equivalent (≤ 16 K complex) to respect `ALIGN_SCRATCH_MAX_BYTES`
  (signal_buffer.c:69) and the tagger's per-call latency.
- **Watchdog/diagnostics migration:** `s_class_stage` breadcrumbs split
  (pump stages vs feeder stages); `bytes_window`/stall watchdog counters
  become shared atomics read by the pump's 1 Hz snapshot. health_wdt
  (wifi_link.c) semantics unchanged — it watches `usb.completed` from outside.

**Win:** URB-service latency decoupled from feed (removes the structural
ceiling term; at today's rates the visible effect is fewer producer-side
`rb_full_drops` spikes during tagger bursts — measurable in soak). Enables the
32-64 KB drain lever. No CPU is saved (same work, different task), so state
plainly: **T48 is a latency/structure fix, not a throughput win at 4.88 MB/s.**
**Risk:** medium — task split around a protocol with a deadlock history; but
the protocol itself is unmodified and every historical failure mode has a
named detector (stall diag stages, health_wdt, take_converted slow-wait
logging).

**Prototype note:** deliberately NOT prototyped in this pass. class_driver /
ingest are device-only code — a host build proves nothing about them, and an
un-smoked device build of a task-split around the #105/#106 protocol is
exactly the kind of "compiles ≠ works" artefact this project's gates exist to
reject. The precise task/queue/priority spec above is the deliverable.

### T50 — PIE 8-lane window multiply

**Design:** hand-rolled PIE kernel `fbt_window_arp4.S` (pattern:
`resample_arp4.S` / `rotate_to_dc_arp4.S`), plus a **precomputed duplicated
window table** `w2[2N]` = {w[0],w[0],w[1],w[1],…} (8 KB) so the kernel is a
pure elementwise Q15 multiply over 4096 int16 lanes — no shuffle needed:

```
512 iterations: vld 8×int16 input, vld 8×int16 w2, Q15 mul, vst 8×int16
```

- Semantics must be **bit-exact with the scalar** `(int32)a*(int32)w >> 15`
  (truncation, not rounding). Memory notes warn `esp.vmul.s32.s16xs16` is a
  lossy Q15-shifted output — whether its truncation matches `>>15` exactly is
  precisely what the golden harness must decide **before** the swap. The
  resample kernel proves scalar/PIE bit-exactness is achievable on this ISA
  when the rounding constant is controlled (resample_256_to_250.c:344-345).
- Both fft_buf targets are already PIE-touched today (fft_sc16_2048 bounces
  them through its internal scratch), so output placement is unchanged.
- Scalar `window_multiply` **stays** as the host implementation and the
  device fallback (`#if` on ESP_PLATFORM + runtime kill-switch for A/B).
- Expected win: window stage from ~40-60 µs/step scalar (4096 muls + pack, to
  be measured — the per-stage `wind` timer already exists,
  fft_burst_tagger.c:766) down to ~8-12 µs → **~30-50 µs/step ≈ 6-10 % of the
  ~500 µs DSP frame**, Core 0. Modest; honest.

**Constraint #2 (heap-position) applies twice:** `w2` is a new PIE-read
buffer and the kernel may need a small aligned scratch. Both must join the
early-alloc dance (class_driver.c:199) and log addresses; `fft_burst_tagger_t`
struct size must stay baseline-equivalent (allocate w2 out-of-struct, like
`staged_new`/`staged_gone`, fft_burst_tagger.c:157-166). DSP buffers need the
16-element PIE look-ahead padding (AGENTS.md, DSP_PADDING_ELEMS).

**Constraint #3 (bit-exact gate) applies:** see §4 — the golden-fixture
harness is a hard prerequisite and its skeleton already exists
(`pie_fft_diff_test.c`, task #67 pattern).

---

## 3. Decoupling architecture summary (T48 + T49 combined end-state)

```
Core 0                                Core 1
──────                                ──────
usb_pump (prio 6)                     daemon (prio 4, USB ISR home)
  handle_events → cb:                 ingest_core1 (prio 8)
    usbring_write (C1, only copy        loop tiles over ring region:
    before the lookback ring)             convert → s_tile (internal)
  enumeration/recovery/1 Hz snapshot      resample s_tile → sb scratch
                                          (PIE MAC, internal input)
dsp_feed (prio 5)                       signal_buffer_commit (GDMA → ring)
  usbring_peek → dispatch ─────────►    signal s_ready
  take_converted(prev) ◄────────────  worker_core1 (prio 4)
  dsp_processor_feed (tagger)           reads bursts from lookback ring
  usbring_consume / release           frame_decoder (prio 4), sd_capture (5)
```

Copies per byte, end-state: **C1 (URB→ring), C4b' (resample→sb scratch),
C6 (GDMA scratch→lookback ring), C7 (scratch→tagger accum)** ≈ 53 MB/s PSRAM
traffic vs ~96.5 MB/s today (−45 %), plus 32 KB DMA-INT freed, minus 8-16 KB
internal for the tile (net +16-24 KB internal headroom).

---

## 4. Verification plan per change

Shared gates that exist today:
- **Host suite:** `cmake -S tests/host -B build_host && cmake --build
  build_host && ctest --test-dir build_host` (30 tests, incl. bit-exact
  resample-split, tagger-vs-manifest, wideband pipeline vs gr-iridium).
- **Device smoke (MANDATORY for any DSP-path commit):** 4 variants
  (RAW_IRIDIUM / REAL_IRIDIUM / CORPUS / FRAME_DECODER), enforced by
  `.githooks/pre-push` `Smoke-verified:` trailer. Real-RF decode is NOT a
  usable gate (bench antenna ~0 decodes) — parity + smoke carry correctness.
- **Boot-log budget gate:** `LIBUSB: Pre-stream DMA-internal heap: free=`
  ≥ 62 KB (esp_libusb.c:415-422).
- **Soak:** serial_logger.sh multi-hour, drops=0, health_wdt silent.

| Task | Existing gate | New harness required first |
|------|--------------|---------------------------|
| T49a SPSC ring | device build + smoke + soak + boot-log budget | **Host unit test of ring index/wrap logic** (header-only core, sibling of test_signal_buffer_index) incl. randomised interleaving model. Verify with a positive control (a deliberately broken variant must FAIL). |
| T49b tile fuse | `test_resample_split` pattern + smoke | **Extend test_resample_split**: same input via tile-sized chunks vs monolithic must be bit-exact (the state-carry machinery is already proven chunk-invariant; the new test pins the exact tile size + AGC-prefix move). |
| T49c sb scratch API | test_signal_buffer_index (host), **T2t fault-injection smoke** (FI_SITE_DMA_SUBMIT_WRAP across wrap), device smoke, soak | Host-side: extend the signal_buffer index model test with the acquire/commit carry-offset arithmetic (pure function — extract it as a header helper so the host can test it). |
| T48 task split | device build, device smoke, **stall-diag/soak** (health_wdt, take_converted slow-waits, rb_full_drops trend) | None new, but add a smoke assertion: rb_full_drops == 0 post-grace AND all stall counters 0 over the smoke window. No bit-exact surface — data path bytes are untouched. |
| T50 window PIE | host suite unaffected (scalar stays) + device smoke | **Golden-fixture bit-exact harness — HARD PREREQUISITE.** Device-side sub-test (pie_fft_diff_test.c pattern, runs under CONFIG smoke): (1) positive control — scalar-vs-scalar must report 0 diff; (2) PIE-vs-scalar over (a) the corpus fixture's real IQ, (b) full-scale ±32767 extremes, (c) 10⁴ random Q15 vectors; assert **max |diff| == 0** (bit-exact, not epsilon — the tagger threshold compare is integer). Any nonzero diff = swap rejected, no tuning. Constants trace to the scalar reference, never adjusted to make the harness pass (no test-fitting). |

Rule restated for the implementer: **T50 (and any future PIE swap) may not be
committed until its harness exists, its positive control passes, and the
PIE-vs-scalar diff is exactly zero on all three vector classes.** History:
fft2r_sc16 swap 58→0, fc32 swap 58→3, #115 host-green/device-dead.

---

## 5. Measurement plan (before/after evidence)

1. **Baseline capture (before any change):** 10-minute production soak log →
   per-stage table: `Cycle read/feed`, `Ingest convert/resample/sbpush`,
   `DSP [wind fft mag detect base]`, worker %, rb_full_drops, DMA-INT free.
   All at -O2 (already the build default; re-confirm in sdkconfig before
   trusting numbers).
2. **Host-ceiling probe (T48 claim-check):** set RTL-SDR to 3.2 MSPS
   (6.4 MB/s) via app_config; record rate_inst / drops / cycle margins. This
   is the only way to substantiate any ">4.88 MB/s" claim on current
   hardware. Expect real sample loss from the dongle itself at 3.2 MSPS —
   use rate/drops trend, not decode, as the metric.
3. After each task: same 10-minute soak + diff the table. Success criteria:
   T49a `cycle_read_us` ≈ 0 and DMA-INT free +32 KB; T49b `resample_us` −15 %
   or better; T49c `sbpush_us` shrinks by the memcpy share; T48 producer
   `rb_max_used` p99 down, drops 0; T50 `wind` µs −30 or better.

---

## 6. Model-tiering recommendation

Rule applied: *cheapest model whose mistakes the gates mechanically catch;
Opus reserved for trust-bearing judgment / ungated work.*

| Task | Model | Why the gate makes it safe (or doesn't) |
|------|-------|------------------------------------------|
| Baseline + 3.2 MSPS measurement runs (§5) | **Haiku 4.5** | Pure procedure: flash, soak, grep counters into a table. The numbers are self-evidencing; a botched run is visibly absent data. Needs only allowlisted single commands (background-agent-safe). |
| T49a: SPSC ring host unit test (written BEFORE impl) | **Fable 5** | The test is itself a gate — a wrong test green-lights a wrong ring. Positive/negative controls are specified (§4) which mechanises most of it, but interleaving-model design needs judgment above Haiku. |
| T49a: ring impl + class/ingest rewiring | **Sonnet 5** | Gated by the (pre-existing) host model test + device smoke + soak + boot-log budget. Residual risk is rare-interleaving races that gates may miss — needs a model strong enough to reason about memory ordering; not Haiku. |
| T49b: tile fuse | **Fable 5** | Strongest gate of the batch: bit-exact host test (extended test_resample_split) catches any staging mistake mechanically, and device smoke catches placement effects. Mistakes cannot hide. |
| T49c: signal_buffer acquire/commit | **Sonnet 5** impl, **Opus 4.8 reviews the diff** | Good gates (index host test, T2t fault-injection smoke, device smoke) but the failure mode that slips through — an occasional wrap-window corruption — self-masks as bad RF. That residual ungated risk is exactly what the tiering rule sends to Opus: not the typing, the adjudication. |
| T48: pump/feeder split | **Sonnet 5** | Protocol moved verbatim; gates = build + smoke + instrumented soak with named detectors for every historical failure (stall stages, health_wdt, slow-wait counters). Deadlock bugs manifest loudly in soak. Priority-tuning decision (feeder vs httpd) is made from soak data per the spec, not invented. |
| T50: golden-fixture harness (written BEFORE kernel) | **Fable 5** | Trust-bearing-ish: a wrong harness is a false gate. But §4 pins the design (positive control, three vector classes, exact-zero criterion) so it's mostly mechanical; the pie_fft_diff_test.c skeleton exists as a template. |
| T50: PIE asm kernel | **Sonnet 5** | Mistakes are mechanically caught by the bit-exact harness + smoke (exact-zero criterion leaves no wiggle room). But P4 PIE asm has documented ISA traps (qacc layout, lp.setup, look-ahead padding) — below Sonnet, the iteration count explodes even with a perfect gate. |
| Sequencing / integration adjudication, any gate-failure post-mortem | **Opus 4.8** | No mechanical gate exists for "is this negative result real or a harness artefact" (memory: correctness fixes invalidate prior negatives; filters need positive controls). Judgment work. |

Record this table in project memory as the current tiering; revisit when the
gates change (e.g. once T50's harness exists, future PIE kernels can drop a
tier for the kernel-writing step).

---

## 7. Sequencing and parallelism

Three independent chains; items within a chain are ordered:

- **Chain A (structure):** A1 T49a host ring test → A2 T49a impl →
  A3 T48 split (benefits from s_raw removal; can technically precede T49a but
  would then need throwaway 32 KB DMA-INT slot growth for multi-block drains —
  don't).
- **Chain B (Core-1 bandwidth):** B1 test_resample_split tile extension →
  B2 T49b tile fuse → B3 T49c acquire/commit (touches the same producer
  hand-off B2 just reshaped; keep serial, and last overall — highest residual
  risk, and its win is pure PSRAM bandwidth with no CPU urgency).
- **Chain C (Core-0 DSP):** C1 T50 golden harness → C2 T50 PIE kernel.

A, B, C are mutually independent and parallelisable across worktrees, EXCEPT:
all device-smoke runs serialise on the single bench device, and concurrent
agents on a shared tree must follow the pathspec-commit rules. Measurement
task §5.1-5.2 precedes everything (baseline is the before-picture every claim
diffs against).

Suggested order if serialised: §5 baseline → A1 → A2 → C1 → B1 → B2 → A3 →
C2 → B3 → §5 re-measure.
