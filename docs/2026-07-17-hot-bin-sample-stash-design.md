# Hot-bin sample stash — design (2026-07-17)

**Status:** DESIGN — not implemented.
**Goal:** make open-IDA-chain continuations immune to ring-lapping under worker
saturation, by copying their raw sample windows out of the 3.36 s signal ring into a
small dedicated PSRAM stash at detect time, and falling back to that copy when the
pop-time stale guard (`signal_buffer_burst_valid`) fails.

**Prior art (read first):**
- `docs/2026-07-15-a6-continuation-priority-boost-spec.md` — A6 hot-bin table (queue-priority boost; this design reuses its table as the single source of truth).
- `docs/2026-07-15-p4-reassembly-design-review.md` — FRAG_GAP=700 ms rationale; first-retransmit gap clusters 0.36–0.54 s (0x7608 continuations: 0.36 s), so the 700 ms hot window already covers the first retransmit.
- `docs/2026-07-15-triage-acars-continuation-review.md` — prefilter SNR-gate hot exemption (already merged; interacts, see §8.6).
- Stale-drop comment block, `p4-usb-host/main/worker_core1.c:217-226` ("if the stale histogram shows real mass at ≥16 dB, a backlog pays off") and `worker_core1.h:60-62` ("recoverable by a sample backlog / an owned-sample queue") — this feature IS that owned-sample backlog, scoped to hot bins only.

**Problem evidence:** 9.5 h soak — six 0x7608 ACARS chains salvaged as opener-only
partials; opener decodes, continuation lost every time. A6 (+100 queue priority,
merged) cannot help once the ring has lapped the continuation's samples: stale-drop
is ring-age, not priority. Confirmation instrumentation (`hot.cont_stale`/`cont_pri`
in `/diag/reassembler`) is deployed; this design assumes `cont_stale` is the dominant
term. **UNVERIFIED until the soak counters land — if `cont_pri` dominates instead,
build nothing here and fix PQ eviction policy.**

---

## 1. Verified codebase facts (all file:line checked 2026-07-17)

### Ring
- `signal_buffer.h:23,32` — 16 MB PSRAM ring, **int8 IQ, 2 B/complex** (NOT 4 B —
  the RTL ADC is 8-bit; int16→int8 narrow is bit-exact lossless, re-expanded
  `int16 = int8<<8` at read). `SIGNAL_BUF_CAPACITY_COMPLEX = 8 Mi complex` ≈ 3.36 s
  at 2.5 MSPS. **Cannot be grown** (`signal_buffer.h:16-22`: +4 MB OOMs the 4 MB
  usbring alloc — PSRAM is at its ceiling; see §5 headroom risk).
- Writer: `ingest_task`, **Core 1, prio 8** (`ingest_core1.c:641-642`), via
  AXI-GDMA (`signal_buffer.c:84-88`) — DMA writes **bypass L2**, hence readers must
  `signal_buffer_invalidate_range()` first (`signal_buffer.h:37-41`, ~1 ms / 2.5 MB).
- `signal_buffer_burst_valid(start_abs64, len)` (`signal_buffer.c:557-578`): true iff
  window fully produced AND `head_total - start ≤ capacity`. `head_total` is 64-bit
  seqlock-guarded (`signal_buffer.c:58-82`).
- `signal_buffer_read_chunk(ring_off, len, int16* dst)` (`signal_buffer.c:620-652`):
  int8→int16 expansion, handles wrap.

### Burst descriptor
`detected_burst_t` (`dsp_processor.h:44-60`): `start_sample_idx` (uint64 absolute),
`length_samples` (uint32), `rel_freq_hz`, `peak_snr_db`, `magnitude_db`, `noise_db`,
`peak_bin` (packed bin|width, `BURST_PEAK_BIN`/`BURST_WIDTH_BINS`). 28 B; comment
warns growing it perturbs `s_pq[]`/PIE state — **we do not grow it**.

### Producer / consumer topology (corrects the brief's premise)
- The burst-detect callback is `worker_core1_push_burst`, registered at
  `class_driver.c:446`, fired from `dsp_processor_feed` inside **`dsp_feed` task,
  Core 0, prio 6** (`class_driver.c:488-490`; pump=7 → feed=6, `usb_host_lib_main.c:66`).
  It is **NOT** the prio-8 ingest task — ingest (Core 1, prio 8) only pushes samples.
  So a producer-side copy stalls the tagger/feed loop on Core 0, not USB ingest
  directly; the 4 MB usbring (~0.8 s at 5 MB/s) buffers upstream of it.
- Worker: `worker_task`, **Core 1, prio 4** (`worker_core1.c:1450-1451`).
- `frame_decoder`: **Core 0, prio 6** (`frame_decoder.c:69,83`) — publishes/clears
  hot bins at `frame_decoder.c:566-579` (wall-clock `esp_timer`, not RF time).
- PQ: `BURST_PQ_CAP 64` (`worker_core1.c:71`), mutex `s_pq_lock` + counting sem;
  insert `worker_core1.c:250` (evict-stale-first, then lowest-priority), pop
  `worker_core1.c:314`. Push-side permanent-stale reject: `worker_core1.c:1486-1502`.
  Pop-side stale guard + drop path: `worker_core1.c:1104-1143`. Mid-read re-check
  (T38): `worker_core1.c:1205-1224`.
- Hot table: `hot_bin_table.{c,h}` — 4 entries (== `IDA_REASM_MAX_SESSIONS`,
  `ida_reassembler.h:42`), deadband 4 bins, TTL `HOT_BIN_TTL_MS = 700 ms`
  (`worker_core1.c:121`, == `IDA_REASM_FRAG_GAP_US`), single-writer
  (frame_decoder), lock-free acquire/release readers, 32-bit atomics only.
  `clear_all` on LO retune: `scanner.c:35,44`.

### Extraction geometry
- `WB_PRE_PAD_SAMPLES 288`, `WB_MAX_BURST_SAMPLES 625000` (250 ms),
  `WB_EXTRACT_SAFETY 1024` (`worker_core1.c:790-793`).
- Worker read envelope: `check_start = start_sample_idx − 288`,
  `check_len = length + 288` (`worker_core1.c:1116-1117`); `ext_len = floor80(min(len,
  625000)) + 288` (`worker_core1.c:1144-1169`).
- Consumption: `wb_extract_decim(ext_start, ext_len, phase_step, burst, sd_tap)`
  (`worker_core1.c:1032-1072`) loops `signal_buffer_read_chunk` in
  `DECIM_CHUNK_IN=4000`-sample chunks into internal-SRAM `s_chunk_iq`, rotates,
  decimates into `s_decim_buf`. **This is the single choke point to shim.**
- Tagger single-frame descriptor length ≈ 30 ms = 75 k complex (`worker_core1.c:753`:
  pre 4096 + frame ≤ ~20.7 k + post `burst_post_len` 40000); multi-frame 50–250 ms.
  gri caps at `max_burst_len = 0.09 s` (225 ms) (`worker_core1.c:755`).

---

## 2. Architecture

```
dsp_feed (Core 0, prio 6)                         worker_task (Core 1, prio 4)
  worker_core1_push_burst(b)                        pop → burst_valid FAIL
    ├─ push-side stale reject (exists)                ├─ sample_stash_lookup(check_start)
    ├─ hot_bin_match(bin)? ──yes──┐                   │     hit → burst_src_t{stash}
    │                             ▼                   │           wb_extract_decim(src)
    │                    stash_try_copy(b)            │           release(FREE)  [stash_hits]
    │                      claim slot (CAS)           │     miss → drop (today)  [stash_misses]
    │                      signal_buffer_copy_raw     └─ burst_valid OK → ring path (today);
    │                      re-validate → commit             opportunistic stash consume-free
    └─ pq_insert (exists, A6 boost)
frame_decoder (Core 0, prio 6)
  worker_core1_hot_publish(bin)  ──►  NEW: stash-sweep of s_pq for bin matches
                                       (closes the "hot published after the
                                        continuation was already queued" gap)
```

New module `p4-usb-host/main/sample_stash.{c,h}` — dependency-free (no ESP includes,
time injected as `now_ms`, ring access injected), same pattern as `hot_bin_table.c`,
so it is host-testable byte-for-byte. Device glue lives in `worker_core1.c` +
one new `signal_buffer` API.

Single source of truth: the stash **keys its gating on the existing `s_hot` table**
(`hot_bin_table_match`); it never maintains its own notion of "open chain". A stale
positive match (chain completed between detect and clear) wastes one slot — accepted.

---

## 3. Memory & bandwidth arithmetic

**Stash format = ring format (int8 IQ, 2 B/complex).** Reuses the lossless narrow;
halves the brief's ×4 estimate; the reader shim does the same `<<8` expansion as
`signal_buffer_read_chunk`, so downstream sees bit-identical int16.

**Slot capacity:** `STASH_SLOT_COMPLEX = 104000` complex (41.6 ms) — covers
pre-pad 288 + a full single-frame descriptor (~75 k) with margin, and a 2-frame
burst (~96 k). Chosen as a multiple of 80 (LCM of decim 10 and 16-complex cache
alignment, same rule as `safe_len`). Longer (multi-frame) hot bursts are stashed
truncated (§7.2). Slot bytes = 208 000 B (203.1 KiB).

**Slot count:** `STASH_SLOTS = 5`. Demand model: ≤4 concurrent chains
(`IDA_REASM_MAX_SESSIONS`), each expecting ≤1 continuation per 700 ms hot window,
first retransmit at 0.36–0.54 s also inside the window → ≤2 stash-worthy events per
chain per window, but slots are freed on consume/expiry and the worker (with A6
boost) pops hot bursts first, so steady-state occupancy is ~open-chain count.
5 slots + SNR-ranked eviction (§6) absorbs junk co-channel captures.
**Total = 1 040 000 B ≈ 0.99 MiB PSRAM** (≤ 1 MB budget).

**Copy cost per stash (worst case, full slot):**
- L2 invalidate of the ring range: 208 KB × (1 ms / 2.5 MB) ≈ **84 µs** (`signal_buffer.h:40`).
- memcpy PSRAM→PSRAM 208 KB: read+write ≈ 416 KB traffic. Project datapoints:
  extract-stage 2.5 MB PSRAM write took 7.5 ms ≈ 330 MB/s write-side
  (`worker_core1.c:1399-1404` history); GDMA push ~800 µs/cycle. Assuming a
  conservative 100 MB/s effective CPU round-trip: **≈ 2.1 ms**. UNVERIFIED — must be
  measured on-device (counter `stash_copy_us_max`, §9).
- C2M writeback of the slot (§6 ordering): 208 KB ≈ **84 µs**.
- **Total ≤ ~2.3 ms per stash, worst case.**

**Producer-context budget:** dsp_feed (Core 0, prio 6) sits downstream of the 4 MB
usbring ≈ 0.8 s of buffering; a 2.3 ms stall ≈ 12 KB of ring occupancy (0.3% of
headroom). With the rate cap (§6, token bucket 8 copies/s): ≤ 1.9% Core-0 CPU and
≤ 3.4 MB/s transient PSRAM traffic, vs the steady 5 MB/s ingest write. Acceptable.

**PSRAM headroom risk (the real constraint):** `signal_buffer.h:16-22` measured that
+4 MB of ring OOMs the usbring; total PSRAM ≈ 16 (ring) + 4 (usbring) + ~12 (stacks/
http) of 32 MB. Whether ~1 MiB more fits is **UNVERIFIED**. Mitigation is structural:
heap-allocate the stash **late** (after `esp_libusb_start_stream`, next to
`sd_capture_alloc_writer_buf()`, `class_driver.c:462`) with `MALLOC_CAP_SPIRAM`
(never DMA/internal — zero DMA-INT impact), and degrade gracefully: alloc failure →
feature self-disables, one log line + `stash_alloc_failed` flag in /diag. NVS knob
`stash_slots` (0 = off) lets us shrink to 3–4 slots (~610–813 KiB) if the bench shows
tight headroom. No `EXT_RAM_BSS_ATTR` static — link-time reservation would push the
OOM onto the usbring unconditionally.

---

## 4. THE critical decision: copy context

| Option | Verdict | Reasoning |
|---|---|---|
| **(a) Inline memcpy in `worker_core1_push_burst` (dsp_feed, Core 0 prio 6)** | **CHOSEN** | ≤2.3 ms worst, rate-capped 8/s; producer is buffered by 0.8 s of usbring; zero new tasks/queues; copy happens at the earliest possible instant (maximum ring freshness ≈ minimum tear risk); no lock held during copy. The brief's "producer runs at prio 8 and must never stall USB ingest" premise doesn't hold — prio-8 ingest is the sample pusher, not the burst callback (§1). |
| (b) Dedicated copier task (prio 5–7, Core ?) | REJECT | On Core 1 it competes with the exact saturation we're evading and violates the Core-1 budget (memory: any prio ≥5 risks starving worker); on Core 0 it buys nothing over inline (same core, same bandwidth) while adding a request queue, slot-reservation handshake, and copy latency during which the ring keeps lapping. |
| (c) Copy inside `pq_insert_locked` | REJECT | Runs under `s_pq_lock`; the worker's pop blocks on that mutex → adds ms-scale worker latency and a priority-inversion window. Copy must complete **before** taking the lock. |
| (d) `esp_async_memcpy` (AXI-GDMA) | REJECT (v1) | Channel is owned by `signal_buffer_push`'s choreography; sharing reintroduces the DMA-INT "stash buffer" alloc-failure class (`signal_buffer.c:454` irony noted) and completion-ordering complexity. Revisit only if measured copy cost exceeds budget. |

---

## 5. Data structures (`sample_stash.h`)

```c
// Lifecycle: FREE → COPYING → READY → (READING → FREE | COPYING via reclaim/evict).
// 32-bit atomics only (RV32 has no lock-free 64-bit); the 64-bit start index is
// split lo/hi and is PLAIN data — guarded by slot ownership (see §6).
typedef enum { STASH_FREE = 0, STASH_COPYING, STASH_READY, STASH_READING } stash_state_t;

typedef struct {
    _Atomic uint32_t state;      // stash_state_t; the only atomic per slot
    // Plain metadata: written only by the context that won the CAS into COPYING;
    // read only after winning the CAS into READING (then re-verified) — never
    // concurrently mutated with a reader attached.
    uint32_t start_lo, start_hi; // absolute complex index of buf[0] (= check_start)
    uint32_t len_complex;        // samples actually copied (≤ slot_complex)
    uint32_t bin;                // detect-FFT bin (BURST_PEAK_BIN space)
    uint32_t expiry_ms;          // reclaim deadline, caller's ms clock
    float    snr_db;             // eviction ranking
    uint8_t  truncated;          // source window exceeded slot_complex
    int8_t  *buf;                // slot_complex × 2 B, PSRAM, 64-B aligned
} stash_slot_t;

typedef struct {
    stash_slot_t     slot[SAMPLE_STASH_SLOTS_MAX]; // compile ceiling 8
    uint32_t         n_slots;                      // runtime (NVS stash_slots)
    uint32_t         slot_complex;                 // 104000
    _Atomic bool     enabled;
    // counters — all _Atomic uint32_t, cumulative since boot (§9)
    _Atomic uint32_t copies, hits, misses, overflow, dup, torn,
                     truncated_n, evicted, expired_reclaims;
    _Atomic uint32_t copy_us_max;                  // relaxed max-update
} sample_stash_t;
```

API (all time injected; ring access injected on host):
```c
void          sample_stash_init(sample_stash_t*, uint32_t n_slots, uint32_t slot_complex);
// producer side (two-phase so the module never touches the ring itself):
stash_slot_t *sample_stash_claim (sample_stash_t*, uint32_t start_lo, uint32_t start_hi,
                                  uint32_t bin, float snr_db, uint32_t now_ms, uint32_t ttl_ms);
void          sample_stash_commit(sample_stash_t*, stash_slot_t*, uint32_t len_copied,
                                  bool truncated, uint32_t copy_us);   // COPYING→READY
void          sample_stash_abort (sample_stash_t*, stash_slot_t*);      // COPYING→FREE (torn++)
// consumer side:
stash_slot_t *sample_stash_lookup (sample_stash_t*, uint32_t start_lo, uint32_t start_hi);
void          sample_stash_release(sample_stash_t*, stash_slot_t*);     // READING→FREE
void          sample_stash_clear_all(sample_stash_t*);                  // LO retune
```

New `signal_buffer` API (`signal_buffer.c`, next to `read_chunk` at :620):
```c
// Copy up to len complex samples starting at absolute index start_abs into dst
// (ring int8 format). Clamps to what has been produced (head_total). Performs the
// M2C invalidate internally. Returns samples copied; 0 if the start is already
// lapped. Caller MUST re-check signal_buffer_burst_valid(start_abs, copied)
// afterwards to detect a mid-copy lap (same rationale as the worker's T38 re-check,
// worker_core1.c:1205-1224).
uint32_t signal_buffer_copy_raw(uint64_t start_abs, uint32_t len, int8_t *dst);
```

---

## 6. Concurrency protocol

**Contexts.** Stash writers: (W1) dsp_feed via `push_burst` (Core 0, prio 6);
(W2) frame_decoder via the publish-time PQ sweep (§7.3; Core 0, prio 6 — same core,
but FreeRTOS time-slices equal-priority tasks, so W1/W2 interleave arbitrarily).
Reader: worker_task (Core 1, prio 4), plus diag readers (counters only).
⇒ **multi-writer, single-reader**: all slot acquisition is CAS-based; no mutex.

**Claim (writer):**
1. Dedupe scan: any slot with `state ∈ {COPYING, READY}` whose `start_lo/hi` match →
   return NULL (`dup++`). (Benign race: two writers can pass the scan simultaneously
   and double-stash one burst into two slots — wastes a slot, reader matches the
   first READY; accepted.)
2. Claim order: (i) `CAS(FREE→COPYING)`; else (ii) expired READY
   (`now ≥ expiry_ms`): `CAS(READY→COPYING)` (`expired_reclaims++`); else
   (iii) evict: lowest-`snr_db` READY with `snr_db < new.snr_db`:
   `CAS(READY→COPYING)` (`evicted++`); else `overflow++`, return NULL.
   Every transition out of READY is a CAS, so a concurrent reader's
   `CAS(READY→READING)` and a writer's reclaim resolve to exactly one winner.
3. Winner owns the slot: plain-store metadata, copy samples into `buf`.
4. Publish: `esp_cache_msync(buf, len*2, C2M)` **then**
   `atomic_store_explicit(&state, READY, memory_order_release)`.
   The release pairs with the reader's acquire load: a reader that observes READY
   observes the metadata and (post its own M2C invalidate) the sample bytes.
   (P4's L2 is believed shared across HP cores, which would make the msync pair
   redundant for CPU↔CPU — **UNVERIFIED**; the explicit C2M-writeback (producer) /
   M2C-invalidate (reader) pair costs ~84 µs each and removes the assumption. Keep it.)

**Lookup (reader, on stale pop):**
1. For each slot: `load(state, acquire)`; skip unless READY; compare `start_lo/hi`
   with the burst's `check_start`.
2. Match → `CAS(READY→READING, acq_rel)`. On CAS failure (writer reclaimed it first)
   continue scanning. On success **re-verify** `start_lo/hi` (defeats ABA: the slot
   may have been reclaimed+republished for a different burst between step 1 and the
   CAS); mismatch → `store(READY, release)`, continue.
3. Owner: `esp_cache_msync(buf, len*2, M2C|INVALIDATE)`, consume samples
   (§7.2), then `store(FREE, release)` (`hits++`).
4. No match → `misses++` *only if* `hot_bin_match(bin)` (keeps the counter meaning
   "hot continuation that the stash failed to save").

**Expiry/TTL:** `STASH_TTL_MS = 8000`. Rationale: worst worker drain = 64-deep PQ ×
~103 ms ≈ 6.6 s; ring lap = 3.36 s; TTL must exceed both so a stashed burst survives
any pop latency the PQ can produce (A6 boost pops hot bursts far sooner in practice).
Expiry is lazy — only writers reclaim; a reader that finds an expired-but-READY slot
still uses it (the data is valid; expiry only authorises reclaim).

**Flood guard (producer):** the tagger has historically emitted junk floods
(~730/s, memory: frozen-baseline latch). Stash gating, in order, all cheap:
1. `hot_bin_match(BURST_PEAK_BIN, now_ms)` — ≤4 bins × 700 ms windows.
2. Narrowband: `BURST_WIDTH_BINS ≤ DSP_NARROWBAND_MAX_BINS (48)` (`dsp_processor.h:141`)
   — broadband RFI junk never stashes.
3. Token bucket: ≤ 8 copies/s (refill 8/s, burst 4) — hard PSRAM-bandwidth ceiling
   regardless of table state. Rejections → `overflow++`.
   (No duration gate: the P1 squelch force-closes real bursts short —
   `worker_core1.c:92-113` history — a min-length gate would eat exactly the
   marginal continuations we're saving.)

---

## 7. Integration points (file:line)

### 7.1 Producer hook — `worker_core1.c:1466` `worker_core1_push_burst`
After the push-side permanent-stale reject (`:1486-1502`), **before**
`xSemaphoreTake(s_pq_lock)` (`:1504`):
```c
uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
if (stash_enabled && hot_bin_match(BURST_PEAK_BIN(burst), now_ms))
    stash_try_copy(burst, now_ms);            // §7.4 glue; never holds s_pq_lock
```

### 7.2 Consumer fallback — pop-time stale guard `worker_core1.c:1104-1143`
Restructure the read path around a source shim so `wb_extract_decim`
(`:1032`) is agnostic of where bytes come from:
```c
typedef struct {                  // worker-local, built per burst
    const int8_t *base;           // NULL → ring (today's path)
    uint64_t      start_abs;      // absolute index of base[0]
    uint32_t      len;            // valid complex samples in base
} burst_src_t;
static void burst_src_read(const burst_src_t *s, uint64_t abs_idx,
                           uint32_t n, int16_t *dst);
// ring:  signal_buffer_read_chunk((uint32_t)abs_idx, n, dst)         (:1051 today)
// stash: expand int8<<8 from base[(abs_idx - start_abs)*2], identical loop to
//        signal_buffer.c:628-635 — bit-exact same int16 stream.
```
Fallback flow replacing the unconditional drop at `:1118-1143`:
```c
if (!signal_buffer_burst_valid(check_start, check_len)) {
    stash_slot_t *sl = sample_stash_lookup(&s_stash, lo(check_start), hi(check_start));
    if (!sl) { /* existing drop path unchanged, incl. cont_stale counter */ }
    else {
        // clamp exactly as the ring path does, plus the stash length:
        safe_len = min(burst.length_samples, WB_MAX_BURST_SAMPLES,
                       sl->len_complex - WB_PRE_PAD_SAMPLES); safe_len -= safe_len % 80;
        ext_len  = safe_len + WB_PRE_PAD_SAMPLES;
        burst_src_t src = {sl->buf, check_start, sl->len_complex};
        // msync M2C on sl->buf (§6), NO signal_buffer_invalidate_range, NO T38
        // mid-read re-check (stash is owned — it cannot be lapped mid-read),
        // then the normal wb_extract_decim → prefilter → pipeline path.
        sample_stash_release(&s_stash, sl);   // after wb_extract_decim returns
    }
}
```
`sd_tap` stays true — SD capture records the same bytes the worker decoded.
Opportunistic recycle: when the **ring** path decodes a burst successfully, look up
and free a matching stash entry (`CAS READY→READING; store FREE`) so consumed
duplicates don't pin slots until TTL. The T38 mid-read failure (`:1217`) keeps
today's drop in v1 (rare; a stash retry there is a cheap follow-up).

### 7.3 Publish-time PQ sweep — closes the ordering gap
The hot bin is published only when the **opener decodes**
(`frame_decoder.c:566-579`), which under saturation is ≥1 s after the continuation
was *detected* — so the detect-time hook (§7.1) misses continuations already sitting
in the PQ un-stashed. In `worker_core1_hot_publish` (`worker_core1.c:185-189`):
take `s_pq_lock`, copy out (28 B each) descriptors whose `BURST_PEAK_BIN` is within
`HOT_BIN_DEADBAND` of the published bin and that are still
`signal_buffer_burst_valid`, release the lock, then `stash_try_copy` each (same
gates/budget). Runs in frame_decoder (Core 0, prio 6) — writer W2, safe per §6.
Cost: ≤64-entry scan under the mutex (~µs) + ≤2 copies typically.
A burst popped between snapshot and copy is a harmless duplicate stash.

### 7.4 Device glue `stash_try_copy` (worker_core1.c, producer contexts)
```c
claim → signal_buffer_copy_raw(check_start, min(check_len, slot_complex), sl->buf)
      → copied==0 ? abort (torn++) :
        !signal_buffer_burst_valid(check_start, copied) ? abort (torn++)  // mid-copy lap
      : commit(len=copied, truncated=(check_len > slot_complex), copy_us)
```

### 7.5 LO retune — `scanner.c:35,44`
Add `sample_stash_clear_all()` beside `worker_core1_hot_clear_all()` (bins AND the
sample windows are LO-relative; stale stashes would decode a different channel).
Also call from `worker_core1_hot_clear_all()` itself so all three call sites stay
in lockstep.

### 7.6 Init / alloc order
`sample_stash_init` (metadata) in `worker_core1_init` (`worker_core1.c:1354`);
slot buffers heap-allocated (MALLOC_CAP_SPIRAM, 64-B aligned) **after** stream
start, at `class_driver.c:462` (`sd_capture_alloc_writer_buf` site) so ring + usbring
+ URB pool take their slices first. Alloc failure → `enabled=false`, WARN once.
Build: add `sample_stash.c` to `p4-usb-host/main/CMakeLists.txt`.

---

## 8. Failure modes

| Mode | Handling |
|---|---|
| 8.1 Slots exhausted in a dense pass | Claim order FREE → expired → evict-weaker-SNR; else `overflow++` and the burst simply keeps today's behaviour (priority boost only). Never blocks. |
| 8.2 Burst longer than slot | Stash first `slot_complex` samples from `check_start` (`truncated=1`, `truncated_n++`); worker clamps `ext_len` (§7.2). A 0x7608 continuation is a single frame (~21 k complex) — always fits; only multi-frame co-channel bursts truncate, and their leading frames still decode. |
| 8.3 Hot match at detect, chain completes before pop | Wasted copy; slot freed by opportunistic recycle or TTL. Bounded by the 700 ms window. |
| 8.4 Double-stash | Dedupe by exact `start_sample_idx` (start_lo/hi) at claim; cross-writer race can still double-stash once — wasted slot, correct behaviour. |
| 8.5 Mid-copy ring lap (producer racing ingest) | `signal_buffer_copy_raw` + post-copy `burst_valid` re-check → `abort`, `torn++`. Same pattern as the worker's T38 guard. |
| 8.6 Interaction with A6 + prefilter | Same `s_hot` table gates boost (pq), stash (this design), and the prefilter SNR exemption (`worker_core1.c:1252-1290`) — one source of truth, no new table. A stashed burst still passes through `burst_prefilter` with the hot SNR exemption, so the two merged fixes compose. |
| 8.7 PSRAM alloc failure at boot | Feature self-disables; `stash_alloc_failed` visible in /diag; zero effect on existing pipeline. |
| 8.8 Copy-cost overrun (bandwidth estimate wrong) | Token bucket caps damage at 8 × 2.3 ms = 1.9% Core-0; `copy_us_max` counter proves/disproves the estimate on-device before default-ON. |

---

## 9. Counters & diagnostics

Extend `worker_hot_stats_t` (`worker_core1.h:115-137`) with:
`stash_enabled, stash_copies, stash_hits, stash_misses, stash_overflow, stash_dup,
stash_torn, stash_truncated, stash_evicted, stash_copy_us_max`.
Export in `/diag/reassembler`'s `hot{}` object (`http_server.c:2653-2657` format,
values at `:2680-2685`). **Note: `body[1500]` (`http_server.c:2640`) must grow to
2048 — snprintf currently clamps silently.**
Success metric: `stash_hits > 0` **and** `ida.parts_completed[2]` advancing while
`hot.cont_stale` stops advancing. `stash_misses` ≈ residual loss (never-detected or
pre-publish continuations).

---

## 10. Host-test plan (`tests/host/`)

Pattern: `test_hot_bin_table` (`tests/host/CMakeLists.txt:199-203`) — dependency-free
module, injected clock, plain-C CHECK harness.

1. **`test_sample_stash.c`** (new): FSM lifecycle (claim/commit/abort/lookup/release);
   claim policy order (free → expired → evict-by-SNR, never evict stronger);
   dedupe; truncation clamp; TTL lazy reclaim; ABA re-verify (simulate: reader loads
   READY+metadata, writer reclaims+republishes different start, reader CAS succeeds,
   re-verify must bounce); torn-copy abort via injected ring-reader returning short/0;
   counters exact; `clear_all`; multi-writer claim (interleaved W1/W2 claim sequences
   must never both own one slot).
2. **Bit-exactness of the read shim:** stash a window from a fixture ring, read via
   `burst_src_read`, compare int16-for-int16 against `signal_buffer_read_chunk` on
   the same range (host builds `signal_buffer.c` already for pipeline tests) —
   including a ring-wrap source window.
3. **Fallback integration:** host pipeline harness (test_pipeline_* fixtures): feed a
   real burst, force `head_total` past one capacity (simulated lap), verify the
   stale path with a pre-stashed window decodes the identical frame bits as the
   un-lapped ring path (gri golden fixture, per cross-validation policy).
4. **Regression guard:** with stash disabled, byte-identical behaviour to today
   (existing suite must stay green untouched).

Device: `/debug/inject` is write-path only (can't exercise this), so on-target proof
is counter-driven soak (§11). Device smoke is MANDATORY before push (DSP-path
commit; `.githooks/pre-push` Smoke-verified trailer).

---

## 11. Rollout

| Stage | Content | Gate |
|---|---|---|
| 0 | Land `sample_stash.{c,h}` + host tests + shim refactor of `wb_extract_decim` (behaviour identical, stash **compiled in, default OFF**) | Host suite green + device GOLDEN smoke unchanged (shim refactor touches the decode path) |
| 1 | NVS knob `stash_en` (0/1, default **0**) + `stash_slots` (default 5) via the serial_cmd NVS interface; /diag counters wired | Smoke + 1 h bench: `copy_us_max` ≤ 3 ms, no `rb_full_drops` regression, usbring alloc still succeeds, free-PSRAM logged |
| 2 | `set stash_en 1` on the bench unit; multi-day soak spanning full pass cycles (per the no-premature-conclusions rule: verdict only on the full dataset) | `stash_hits>0`, `parts_completed[2]` advancing, `hot.cont_stale` flat, pk_wk not degraded |
| 3 | Flip default ON (mirroring A6's default-ON once proven) | Soak verdict + human sign-off |

---

## 12. Open questions (for the human)

1. **PSRAM headroom:** is ~1.0 MiB actually free after usbring/stacks on the current
   build? Needs one boot-time `heap_caps_get_free_size(MALLOC_CAP_SPIRAM)` log line
   before Stage 1. If tight, is 3 slots (~610 KiB) an acceptable first cut?
2. **Copy bandwidth:** the ≤2.3 ms/copy figure rests on a 100 MB/s PSRAM round-trip
   inferred from write-side datapoints — measure `copy_us_max` on the bench under
   live streaming before enabling by default. UNVERIFIED.
3. **Soak counters first:** do the deployed `hot.cont_stale`/`cont_pri` numbers from
   the 36 h capture confirm stale (ring-lap) dominates? If `cont_pri` dominates,
   this design is the wrong lever.
4. **Publish-time PQ sweep (§7.3) in v1 or v1.1?** It adds the second writer context
   and the `hot_publish` PQ scan; but without it, any continuation detected before
   its opener decodes (the common case at ≥1 s worker lag) is never stashed at
   detect time. My recommendation: v1 — it is where most of the win lives.
5. **Slot geometry:** 5 × 104 k complex (41.6 ms) vs 6 × 80 k (32 ms) — do soak logs
   show hot-channel continuation bursts ever exceeding ~75 k samples (i.e. is
   2-frame coverage worth the bigger slot)? UNVERIFIED distribution.
6. **L2 coherency across HP cores:** design keeps the explicit msync pair (~170 µs
   total) regardless; confirm with ESP-IDF docs whether it can be dropped later.
