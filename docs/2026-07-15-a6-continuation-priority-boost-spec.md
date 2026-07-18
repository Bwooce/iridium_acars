# A6 — Continuation-priority boost: implementation spec

Date: 2026-07-15 (AEST)
Status: SPEC — no production code changed. Companion to
`docs/2026-07-15-p4-reassembly-design-review.md` (§3.5/§5-A6 sized this lever; A3 results
table therein is the quantitative basis).

Scope of change (all files):
- `p4-usb-host/main/worker_core1.c` — hot-bin table (owner), `burst_priority()` boost,
  new counters.
- `p4-usb-host/main/worker_core1.h` — publish/clear/stats API.
- `p4-usb-host/main/frame_decoder.c` — publish/refresh/clear calls around
  `ida_reassembler_feed_ex()` (~line 529).
- `p4-usb-host/main/http_server.c` / `status_logger.c` — surface counters (existing
  `/diag` + `frame_decoder_get_reasm_stats` / `worker_core1_get_stats` idioms).
- NO changes to `common/iridium_decoder/*` (the reassembler is untouched).

---

## 1. Problem and expected gain

A 2-burst LW.DA (IDA) chain needs BOTH bursts: opener (`da_ctr=0, da_cont=1`) and
continuation (`da_ctr=1, da_cont=0`). The worker (Core 1) fully decodes only ~13 bursts/s
(~76 ms each — comment at worker_core1.c:52); under flood, bursts it can't reach go STALE
when the 3.36 s sample ring (`SIGNAL_BUF_CAPACITY_COMPLEX`) laps their samples
(pop-side guard at worker_core1.c:1007, insert-side evict-stale-first at :168-181).
Priority today is `peak_snr_db` with a −1000 wideband penalty (`burst_priority()`,
worker_core1.c:111-116). The continuation competes on raw SNR and drops independently at
probability p, so chain completion ≈ (1−p)².

The host drop-model (`dropmodel.py`, arm 5, 45-min corpus — review §A3) shows that
exempting continuations from drop while openers still drop moves completion to ≈ (1−p):
at p = 0.3, 0x7608 chain coverage 49.7% → 71.1% = **+43% relative**. This is the highest-
value firmware lever for ACARS yield identified this cycle.

**Evidence caveat (do not over-claim):** arm-5 sizing rests on a 45-min bench corpus with
n = 3 retransmit pairs; the 36 h re-run (review action A5) is pending. Treat +43% as an
estimate of the ceiling, not a commitment. The mechanism (protecting the second burst
converts a squared loss into a linear one) is robust; the magnitude at the device's real
p̂ is what the 36 h data and the on-device counters (§8) will settle.

**Ceiling:** the OPENER cannot be protected — a burst is not known to be an opener until
after demod + BCH + IDA header parse on Core 0. (1−p) is the hard ceiling of this design.

---

## 2. Architecture — the cross-core feedback loop

```
Core 0                                                Core 1
======                                                ======
dsp_feed (prio 6)                                     worker_core1 task (worker_task,
  fft_burst_tagger_step                                 created at worker_core1.c:1299)
    └─ callback worker_core1_push_burst (:1315)          │
         pq_insert_locked (:154)  ── s_pq_lock ──►  pq_extract_max_locked (:207)
         [reads HOT TABLE]                          [reads HOT TABLE]
                                                         │ demod ~76 ms
                                                         ▼
                                                    frame_decoder_push (:892)
                                                         │ frame_queue (64 slots, PSRAM)
frame_decoder task (decoder_task, prio 6,                ▼
  DECODER_CORE 0, frame_decoder.c:53, :693)  ◄───────────┘
  process_one → IR_FRAME_LW/IR_LW_DA (:481)
    ida_reassembler_feed_ex (:529)
      rc==0  → chain OPEN/still open ─► WRITE HOT TABLE (publish/refresh)
      rc==1  → chain complete       ─► CLEAR matching entry
      rc==-1 → orphan/overflow      ─► no-op
                    │
                    ▼
        HOT TABLE (4 × {bin, expiry}, BSS in worker_core1.c)
        single writer: decoder task (Core 0)
        readers: tagger callback (Core 0, dsp_feed ctx) + worker (Core 1)
```

The loop: worker demods the opener → decoder task opens the chain → decoder publishes the
opener's detect-FFT bin + TTL → the worker's priority function boosts any queued/incoming
burst within ±4 bins → the continuation is popped next and demodded before its ring
samples lapse, and is never the eviction minimum while the entry is live.

### 2.1 Latency budget

Publish latency (opener RF end → hot entry live):

| stage | latency |
|---|---|
| tagger gone event (FBT_BURST_POST_LEN 40000 samples) | 16 ms |
| opener wait in PQ | 0 (idle) … ~1–3 s (flood; bounded by its own 3.36 s ring life) |
| opener demod | ~76 ms |
| frame_queue → decoder task (prio 6, one-frame-per-wake + taskYIELD, :651) | ~1–10 ms |
| feed → publish (same call stack) | µs |

Continuation arrival: opener + 90–180 ms (clean, TDMA-quantised; review §2.1) or
+450–540 ms (head-of-line ARQ retransmit; review §2.2).

Two orderings, both covered:
- **Moderate load** (opener pops within ~1 slot): hot entry live ~100–200 ms after the
  opener's RF end — often before the continuation's gone event, so the continuation is
  boosted already at `pq_insert_locked` (survives full-queue eviction) and wins the very
  next `pq_extract_max_locked`.
- **Overload** (opener queued ~1–2 s): the continuation is usually already sitting in the
  PQ at base priority when the entry lands. That is fine: priority is re-evaluated on
  EVERY insert-when-full and every extract, so the boost takes effect at the next pop.
  The binding constraint is the continuation's own ring life: captured at opener+0.09 s,
  its samples survive until opener+~3.45 s. The boost is effective as long as
  (opener pop time + 76 ms demod + decode + publish + one worker wake + 76 ms) <
  continuation capture + 3.36 s — i.e. opener pops by roughly +3 s. Beyond that the
  opener itself is at/over its own ring life, so the chain was unprotectable anyway.
- **Extreme overload / failure mode:** if the opener is itself stale-dropped or pops so
  late that the continuation's ring life is exhausted, no entry is ever published (or it
  is published too late) and the chain is lost exactly as today. A6 degrades to the
  status quo — it never makes things worse. This is the (1−p) opener-drop ceiling in
  operation. Residual risk inside the window: the continuation can be EVICTED at base
  priority during the ~100–200 ms before the entry lands (needs 64 stronger narrowband
  bursts); accepted, not mitigated (see §10).

---

## 3. Data structures

All in `worker_core1.c` file scope, plain BSS (**32 bytes total** — no heap, no DMA-INT,
no PSRAM, no PIE interaction):

```c
// A6 hot-bin table. Written ONLY by the frame_decoder task (Core 0); read by
// burst_priority() from the tagger callback (Core 0, dsp_feed context) and the
// worker task (Core 1). Entry is two 32-bit relaxed/release-acquire atomics —
// same pattern as s_drop_stale_snr (worker_core1.c:131). An entry is live iff
// expiry_ms != 0 && now_ms < expiry_ms.
#define HOT_BIN_ENTRIES IDA_REASM_MAX_SESSIONS          // 4
#define HOT_BIN_DEADBAND 4   // bins; == floor(IDA_REASM_FREQ_DEADBAND_HZ /
                             // (FS_DETECT_HZ / FFT_SIZE)) = floor(5000 / 1220.7)
#define HOT_BIN_TTL_US IDA_REASM_FRAG_GAP_US            // 700 ms — see §7.1
#define HOT_BOOST 100.0f                                 // see §5

typedef struct {
    _Atomic uint32_t bin;       // detect-FFT bin, 0..FFT_SIZE-1 (DC-centred)
    _Atomic uint32_t expiry_ms; // esp_timer_get_time()/1000 deadline; 0 = empty
} hot_bin_entry_t;
static hot_bin_entry_t s_hot_bins[HOT_BIN_ENTRIES];
```

Sizing: `HOT_BIN_ENTRIES == IDA_REASM_MAX_SESSIONS == 4` — the table mirrors the
reassembler's session capacity by construction; measured open-chain concurrency is ~0.04
(review §2.4), so 4 slots are already ~100× headroom.

`expiry_ms` is a 32-bit millisecond clock (wraps at 49.7 days of uptime; on wrap the
worst case is one bogus ≤700 ms boost window — bounded-harm, not worth 64-bit atomics,
which are NOT lock-free on RV32 and would drag libatomic locking into the tagger
callback).

---

## 4. Concurrency protocol

Single writer (decoder task, Core 0), two readers (tagger callback on Core 0 under
`s_pq_lock`; worker task on Core 1 under `s_pq_lock`). The writer does NOT take
`s_pq_lock` — the table is lock-free with respect to the PQ.

Writer (publish/refresh, decoder task only):
```c
// 1. reuse: entry with |bin - new_bin| <= HOT_BIN_DEADBAND and still live → refresh
// 2. else: first entry with expiry_ms == 0 or expired
// 3. else (all 4 live — ~never, concurrency 0.04): overwrite soonest-expiring
atomic_store_explicit(&e->expiry_ms, 0, memory_order_release);   // close window
atomic_store_explicit(&e->bin, new_bin, memory_order_relaxed);
atomic_store_explicit(&e->expiry_ms, now_ms + HOT_BIN_TTL_US/1000,
                      memory_order_release);                     // open window
```

Writer (clear-on-complete): find live entry within deadband of the completing frame's
bin; `atomic_store_explicit(&e->expiry_ms, 0, memory_order_release)`.

Reader (inside `burst_priority_at`, see §5):
```c
uint32_t exp = atomic_load_explicit(&e->expiry_ms, memory_order_acquire);
if (exp != 0 && now_ms < exp) {
    uint32_t hbin = atomic_load_explicit(&e->bin, memory_order_relaxed);
    if ((uint32_t)abs((int)hbin - (int)BURST_PEAK_BIN(b)) <= HOT_BIN_DEADBAND) → boost
}
```

Ordering argument: the release store to `expiry_ms` after the relaxed `bin` store
guarantees an acquire reader that sees the new expiry also sees the new bin. The only
racy interleaving (reader between the invalidate and the re-arm) makes the entry
momentarily invisible — a missed boost for one scoring pass, self-healing at the next
pop. There is NO interleaving that pairs a live expiry with a torn/foreign bin. Even if
ordering were dropped entirely, the worst case is a ≤700 ms boost at a wrong bin: a
priority perturbation, never a correctness violation (demod/classify/CRC gate everything
downstream). This is deliberately the same "relaxed atomics, benign diagnostic-grade
races" doctrine already used for `s_drop_stale_snr`/`s_bursts_queued`
(worker_core1.c:128-131, 222-227). No seqlock is needed because no reader ever requires
a consistent multi-field snapshot.

`now_ms` is snapshotted ONCE per `pq_insert_locked` / `pq_extract_max_locked` call
(`esp_timer_get_time()/1000`) and threaded through, so the O(BURST_PQ_CAP=64) priority
scans do not call the timer 64×.

---

## 5. Exact insertion points and the new priority formula

### 5.1 `worker_core1.c`

- **`burst_priority()` (line 111-116)** becomes `burst_priority_at(const
  detected_burst_t *b, uint32_t now_ms)`:

```c
static inline float burst_priority_at(const detected_burst_t *b, uint32_t now_ms)
{
    float p = b->peak_snr_db;
    if ((int)BURST_WIDTH_BINS(b) > BURST_NARROW_MAX_BINS) p -= 1000.0f;
    if (hot_bin_match(BURST_PEAK_BIN(b), now_ms)) p += HOT_BOOST; // A6
    return p;
}
```

  Magnitude proof (`HOT_BOOST = 100`): live SNR range is ~0–45 dB.
  - boosted narrow: ≥ 100 → outranks every unboosted narrow (≤ ~45) regardless of SNR gap ✓
  - boosted WIDE junk that happens to sit near a hot bin: −1000 + snr + 100 ∈ [−900, −855]
    → still below every unboosted narrow (≥ 0): the wideband penalty semantics are
    preserved — junk still loses ✓
  - two boosted narrow bursts: SNR remains the tie-breaker ✓

- **`pq_insert_locked()` (line 154)** and **`pq_extract_max_locked()` (line 207)**:
  compute `uint32_t now_ms` once at entry, replace the three `burst_priority(...)` calls
  (lines 185, 187, 193) and the two in extract (210, 212) with
  `burst_priority_at(..., now_ms)`. The evict-stale-first pass (:168-181) is untouched —
  a stale continuation is unrecoverable and its slot should still be reclaimed.

- New file-scope: `s_hot_bins[]`, `hot_bin_match()`, and the public API:

```c
// worker_core1.h
void worker_core1_hot_publish(int bin, uint64_t now_us);  // open/refresh, TTL-managed
void worker_core1_hot_clear(int bin);                     // clear-on-complete
void worker_core1_hot_clear_all(void);                    // LO retune hook
void worker_core1_get_hot_stats(worker_hot_stats_t *out); // §8 counters
```

  `worker_core1_hot_publish` takes `now_us` from the CALLER'S `esp_timer_get_time()` (the
  decoder task already reads it each wake, frame_decoder.c:630) — NOT from the frame's
  RF-derived `timestamp_us`, which lags wall-clock by the full queueing delay and would
  under-size the TTL exactly when it matters (overload).

### 5.2 `frame_decoder.c` — write-side hooks (all in `process_one`, LW.DA arm, :481-562)

The frame's detect bin is `it->peak_bin & 0xFFFF`. **Masking is mandatory:** the
STANDALONE path passes the PACKED bin|width int (worker_core1.c:894 passes
`wctx->burst->peak_bin` verbatim; see the pack comment at dsp_processor.h:53-58), while
the aggregator path passes the already-truncated int16 (aggregator_ingest.c:77). The mask
yields the plain bin (0..2047) in both roles.

```c
int rc_reasm = ida_reassembler_feed_ex(...);              // existing call, :529
int det_bin  = it->peak_bin & 0xFFFF;                     // BURST_PEAK_BIN semantics
if (rc_reasm == 0) {
    // Chain opened (opener) OR continuation merged, chain still open:
    // both mean "a continuation is expected within FRAG_GAP" — publish/refresh.
    worker_core1_hot_publish(det_bin, (uint64_t)esp_timer_get_time());
} else if (rc_reasm == 1 && !(ida.da_ctr == 0 && ida.da_cont == 0)) {
    // Multi-burst chain completed (NOT the standalone fast path, which never
    // published): stop boosting this channel now rather than at TTL expiry.
    worker_core1_hot_clear(det_bin);
}
// rc_reasm == -1 (orphan / table full / overflow): no hot-table action.
```

Reaped/expired chains (`ida_salvage_drain`, :177/:520/:636) need **no hook**: with
`HOT_BIN_TTL_US == IDA_REASM_FRAG_GAP_US`, the hot entry expires at exactly the moment
`find_matching_session()` (ida_reassembler.c:37) would stop accepting a continuation, and
strictly before the reaper fires at `IDA_REASM_SESSION_TIMEOUT_US = 1 s`. The TTL is a
faithful shadow of chain acceptability — this is the invariant that lets the table stay
decoupled from the session table (sessions do not store a bin, and MUST NOT need to:
`common/` stays untouched).

### 5.3 LO retune hook

Detect bins are LO-relative on BOTH sides of the loop, so the comparison is
LO-independent in steady state — but across a live retune (scanner Phase 1 / hourly
autotune LO rescan) a surviving entry points at a now-meaningless bin. Call
`worker_core1_hot_clear_all()` wherever the retune path already calls
`dsp_processor_reset_tagger_baseline()` (scanner/autotune). Cost of omission is bounded
at one ≤700 ms misplaced boost per retune, but the hook is one line — do it.

---

## 6. bin ↔ Hz decision: compare in BIN space, no conversion at all

Facts from the code:
- The tagger emits `center_bin` (0..2047, DC-centred post-FFT-shift; fft_burst_tagger.h:92)
  and `dsp_processor.c:140-141` derives
  `rel_freq_hz = (center_bin − FFT_SIZE/2) × FS_DETECT_HZ / FFT_SIZE`
  (bin width = 2 500 000 / 2048 = **1220.703 Hz**).
- The burst descriptor carries the bin packed in `peak_bin` (`BURST_PEAK_BIN(b)`,
  dsp_processor.h:62).
- **The frame's `freq_hz` is 0 through BOTH decode paths** — worker_core1.c:893 and
  aggregator_ingest.c:76 pass `0u` — so `it->freq_hz` at frame_decoder.c:532 is always 0.
  (Consequence: the reassembler's ±5 kHz deadband is currently VACUOUS in production —
  every frame trivially matches every session on frequency. See §10, Discovered issue.)

Therefore the only frequency truth that survives end-to-end is the detect bin itself, and
it is the SAME quantity on the writer (frame's `peak_bin`) and the reader (candidate
burst's `peak_bin`). **Decision: store and compare raw bins.** ±5 kHz ≡
`HOT_BIN_DEADBAND = floor(5000 / 1220.703) = 4` bins.

Why not Hz on either side:
- bin→Hz in the worker: a float multiply + LO lookup per entry per scored burst, ×64
  slots per scan — pure cost, zero information gained (both values came from the same
  quantiser).
- Hz→bin on the writer: requires the current LO (runtime-settable via POST /tune,
  dsp_processor.h:36) to be threaded into frame_decoder; a retune between opener decode
  and continuation scoring would silently skew the window. Bin-space comparison is
  immune to the LO VALUE by construction (both bins are LO-relative offsets), needing
  only the clear-all hook at the retune EVENT (§5.3).

Alignment check: same-carrier bursts jitter by ~±1–2 bins in the tagger's peak estimate
(the ida.py reference used ±260 Hz on gr-iridium's FINE estimate; our detect-bin
quantisation alone is 1220 Hz). ±4 bins (= ±4883 Hz) covers this with margin while
staying at ~1/8.5 of the 41.667 kHz channel grid — adjacent channels cannot be confused.
On-device validation of the actual opener↔continuation bin spread is measurement M2 (§10).

---

## 7. Edge cases

1. **Stale entries:** TTL-expired entries are dead at read time (`now_ms >= expiry_ms`);
   no reaper needed. Slots are recycled by the writer's find-free scan.
2. **`da_ctr` wrap (chains > 8 fragments):** the reassembler inherits upstream ida.py's
   behaviour (wrap mis-reads as a new chain — documented at ida_reassembler.c:110-115).
   A6 is agnostic: each merge refreshes the TTL, so a long chain keeps its boost; a
   wrap-split chain just re-publishes on the "new" opener. No special case.
3. **Chain completion:** cleared eagerly (§5.2) using the COMPLETING frame's bin, matched
   within the same ±4-bin deadband (the completing fragment's bin may jitter 1–2 bins off
   the stored opener bin). If the match fails (jitter > deadband — then the reassembler
   itself wouldn't have matched either), TTL cleans up within 700 ms.
4. **Standalone frames (`ctr=0, cont=0`, the majority):** `rc==1` via the fast path at
   ida_reassembler.c:101 — never published, and §5.2 explicitly excludes them from the
   clear call so they cannot erase a live entry for a genuine concurrent chain on the
   same channel.
5. **Orphans / overflow (`rc==-1`):** no publish (no open chain to protect). The
   overflow-kills-chain path (ida_reassembler.c:148) leaves a live entry to TTL out —
   ≤700 ms of harmless residual boost; not worth plumbing a signal out of `common/`.
6. **Table full (4 concurrent chains):** overwrite the soonest-expiring entry. Expected
   concurrency 0.04 (review §2.4) — effectively unreachable.
7. **Extreme overload:** opener delayed/stale-dropped → entry late/never → chain lost as
   today; A6 strictly ⊇ status-quo behaviour. Ceiling (1−p) (§1, §2.1).
8. **Boost starvation of non-hot bursts:** bounded: ≤ 4 channels × ≤700 ms (refreshed
   only by real merges). A boosted burst still pays the normal triage fast-pass
   (P1.5a/b), so junk that sneaks into a hot window costs ~1/40 of a full decode.
9. **Invariants preserved:** no new task, no PIE state, no PSRAM/DMA-INT allocation
   (32 B BSS); decoder task's one-frame-per-wake + taskYIELD discipline
   (frame_decoder.c:641-651) untouched; wideband −1000 penalty semantics proven dominant
   in §5.1; `s_pq_lock` acquisition order unchanged (writer never takes it).

---

## 8. Diagnostics

New counters in worker_core1.c (relaxed `_Atomic uint32_t`, same idiom as :228-238):

| counter | increments when |
|---|---|
| `s_hot_published` | publish/refresh call (decoder task) |
| `s_hot_cleared` | clear-on-complete found a live entry |
| `s_hot_boost_pops` | `pq_extract_max_locked` winner was boosted (count ONCE per pop, in `worker_task` after extract — not per comparison) |
| `s_hot_boost_inserts` | `pq_insert_locked` admitted/evicted-for a boosted newcomer |

Surface: extend `worker_core1_get_stats()` (drained by status_logger's 1 Hz line — add
`hot=pub/pop` to the WORKER1 status line) and the `/diag` JSON in http_server.c alongside
the existing drop histograms (`worker_core1_get_drop_snr`, worker_core1.c:146).

Success metrics to watch before/after (all already exported):
- `s_drop_stale_snr` 16–20 dB+ buckets (worker_core1.c:131, /diag): stale-drop mass at
  real-signal SNR should FALL for continuation-bearing channels.
- `ida parts_completed[2]` vs `parts_expired[1]` (frame_decoder_get_reasm_stats,
  frame_decoder.c:809-812; status_logger:399): 2-fragment completion ratio should RISE;
  opener-only expiries (`parts_expired[1]`) are the unprotectable-opener residue and
  should NOT change (a change there means something else moved — investigate).
- `s_salvage_ok` / `SALVAGE` iot_log lines: truncated-chain salvage should FALL as chains
  complete instead.
- `s_hot_boost_pops / s_hot_published`: the loop's hit rate — if ~0 under flood, the
  publish is landing too late (measure M1).

---

## 9. Phased, testable plan

**Phase 0 — host unit test (first).** New `tests/host/test_hot_bin_table.c`: publish/
refresh/expiry/deadband/clear semantics, the §4 writer sequence under interleaved reads,
and the §5.1 formula ranking table (boosted-narrow > unboosted-narrow > boosted-wide >
unboosted-wide; SNR tiebreak). Pure C, no FreeRTOS — build the table module host-side the
way worker helpers already are. The end-to-end GAIN is already host-proven by
`dropmodel.py` arm 5; do NOT re-tune the model to this implementation (no test-fitting).

**Phase 1 — firmware, boost OFF-able.** Implement §3-§5 behind an NVS-gated runtime flag
(`hot_boost`, default ON, settable via serial_cmd/app_config like `best_effort_decode`) so
an A/B soak needs no reflash. Zero behaviour change when disabled (`hot_bin_match` returns
false without touching atomics).

**Phase 2 — device smoke (MANDATORY — this is a DSP-path commit; pre-push hook enforces
the `Smoke-verified:` trailer).** Expectations: GOLDEN corpus decodes all 62 with counts
unchanged — the corpus never fills the PQ, so the boost can only reorder pops, never shed
(any count change = a bug, fix the code not the fixture). CORPUS/REAL passes unchanged.
Watch for the known CORPUS harness-RNG flake before blaming the change.

**Phase 3 — bench A/B soak.** ≥ 2 h ON vs OFF (flag from Phase 1) under live antenna;
compare §8 metrics. Relay interim numbers as RAW points only — verdict waits for the full
window (no premature conclusions from partial data), ideally aligned with the 36 h corpus
landing so the arm-5 sizing is re-validated on the same air.

**Phase 4 — measurements M1/M2 (§10) fed back into constants** (TTL, deadband) if the
on-device numbers disagree with the corpus-derived ones.

---

## 10. Risks and open questions (measure on-device)

- **M1 — opener→publish lead time.** Log `esp_timer_get_time() − it->timestamp_us` at
  each `worker_core1_hot_publish` (the full RF-to-publish loop latency, since
  `timestamp_us` is RF-sample-derived — worker_core1.c:867-872). If the p90 under flood
  exceeds ~2.5 s, the boost mostly lands after the continuation's ring life and the lever
  under-delivers; that would motivate the (out-of-scope) escalation of publishing
  earlier, e.g. at demod time before classification.
- **M2 — opener↔continuation bin spread.** Count boost-window hits by |Δbin| (0..4) at
  `hot_bin_match`; if real continuations cluster at Δbin > 2, ±4 may be tight against
  tagger jitter and needs re-derivation. Host-side cross-check possible against the
  45-min corpus if detect-bin (not just Hz) is recoverable from the capture.
- **Evidence thinness (§1):** +43% is a 45-min, n=3-retransmit-pair estimate; the 36 h
  corpus re-run is the fix. Ship Phase 1 behind the flag regardless — the mechanism is
  sound and the downside is bounded (§7.8).
- **Continuation evicted before publish lands** (§2.1 residual): unprotectable within
  this design without opener-independent chain inference; quantify indirectly as
  `parts_expired[1]`-adjacent mass that persists with the boost ON.
- **Discovered issue (companion fix, NOT part of A6):** the reassembler's
  `IDA_REASM_FREQ_DEADBAND_HZ` check is vacuous in production because both decode paths
  pass `freq_hz = 0` (worker_core1.c:893, aggregator_ingest.c:76) — every session matches
  every frame on frequency, and the review's false-merge hazard analysis (§2.3) assumed a
  WORKING deadband. Recommend a follow-up that passes
  `(uint32_t)(int32_t)llroundf((BURST_PEAK_BIN(b) − 1024) × 1220.703f) + <bias>` (or,
  cheaper and consistent with this spec, changes the reassembler key to bin space) so the
  air-interface false-merge protection is real. Keep it a SEPARATE commit: making the
  deadband real could in principle reject chains that today match, so it needs its own
  smoke + soak.
- **Aggregator role:** the hot table lives in worker_core1.c, which does not run on the
  AGGREGATOR-role board; `worker_core1_hot_publish` from frame_decoder must be a safe
  no-op there (weak stub or role-#ifdef, matching how frame_decoder already splits at
  worker_core1.c:873-896). Multi-receiver protection (aggregator → per-worker feedback
  over frame_link) is explicitly out of scope.
