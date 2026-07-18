# Multi-fragment reassembly bug review — why `acars_decoded` has never incremented

2026-07-15/16. Code review only; no production code changed. Evidence: source at working tree
(incl. today's uncommitted Task #6/A6 changes), the on-device counter snapshot
`~/iridium_session_backup/p4_pre_a6_reasm.json`, a live read-only `GET /diag/reassembler`
(2026-07-16), and the 36 h HydraSDR ground-truth corpus (`~/iridium_capture/ida_lines_archive.txt`,
55 k parsed `IDA:` lines).

---

## 1. VERDICT

**Yes — there is a deterministic bug, and it is a clock-domain mix, not a decode or routing bug.**

**The bug:** `frame_decoder.c`'s decoder task reaps IDA reassembly sessions on the **wall clock**
while the sessions are timestamped on the **RF-arrival clock**:

- Sessions are stamped with `it->timestamp_us` = `cap_us` = *RF arrival time of the burst*
  (`worker_core1.c:941-942`: `signal_buffer_stream_epoch_us() + start_sample_idx·1e6/FS_DETECT_HZ`),
  stored into `s->last_time_us` (`ida_reassembler.c:128, 154`).
- The 1 Hz idle tick reaps with **`esp_timer_get_time()`** (`frame_decoder.c:667` → `:673`
  `ida_salvage_drain(now)`), and `ida_reassembler_reap()` expires any session with
  `now_us − last_time_us > IDA_REASM_SESSION_TIMEOUT_US` (1 s) (`ida_reassembler.c:49-50`).

The difference between the two clocks is exactly the **pipeline decode lag** (tagger close + priority-queue
wait + ~0.5 s/burst worker decode + frame queue). So the *effective* reassembly window is

```
effective window = SESSION_TIMEOUT (1 s) − decode_lag
```

which is **≤ 0 whenever the pipeline runs ≥ 1 s behind RF** — the device's permanent operating point
(the same snapshot shows **10 086 stale-dropped bursts at ≥ 16 dB**, i.e. bursts routinely age past the
3.36 s ring before the worker reaches them; lag ≥ 1 s is the norm, and even the theoretical best case is
~0.7 s: burst close ~0.2 s + decode ~0.5 s). Under lag > 1 s, **every open chain is reaped at the first
1 Hz tick after its opener is fed**. A chain can only survive if its continuation is fed in the gap
between opener-feed and that tick — i.e. only if the worker decodes the two fragments **back-to-back**
(Δfeed ≥ ~0.5 s at one burst per ~0.5 s, so survival ≤ ~50 % even then), and with **probability 0** if
even one other burst is decoded in between (Δfeed ≥ 1 s ≥ tick period → a tick always lands in the
window). Introduced 2026-07-12 by the salvage wiring (`46dc971`, Task B3), which moved expiry out of
`feed()` (where it used the fed frame's RF clock, consistently) into the wall-clock tick.

### Why the mandatory device smoke never caught it

`smoke_test.c:359-360` pushes the real 2-fragment A62001 fixture with the **fixture's own capture
timestamps (~1.783 × 10¹⁵ µs — HydraSDR wall epoch)**, which are astronomically *ahead* of the device's
`esp_timer` uptime. `ida_reassembler_reap()`'s backwards-clock guard
(`now_us > s->last_time_us`, `ida_reassembler.c:49`) is therefore **false for every session during
smoke — the wall-clock tick reap is a structural no-op in the smoke test**, the exact code path that
kills every live chain. Host tests (`test_acars_tail_real.c` via `acars_tail_feed()`) never call the
tick/reap at all. This is precisely the "what the host test does NOT exercise" gap.

### Trace: a valid 2-burst A62001 `_d` (37 B, msg 1/1) on the live device

1. Both bursts RF-received, tagged as two separate bursts (~90 ms apart, same channel), inserted into
   the worker SNR-priority queue. (First hazard, known: either may be stale-dropped/evicted; correlated,
   not independent — see §2.6.)
2. Worker decodes the **opener** (`da_ctr=0, da_cont=1, da_len=20, CRC OK`) after lag `L` (≈ 1–3 s under
   load) and pushes it with `timestamp_us = cap_us` (RF arrival), `freq_hz=0`, packed `peak_bin`.
3. `process_one` (`frame_decoder.c:500-533`): classify → `ida_decode` → `clean = ok && header_ok &&
   crc_ok` = **true** (real openers are `len>0 CRC:OK` — see §2.1) → `ida_salvage_drain(it->timestamp_us)`
   (RF clock, harmless) → `ida_reassembler_feed_ex(...)` opens a session: `next_ctr=1`,
   `last_time_us = arr₀` (`ida_reassembler.c:116-134`).
4. Within ≤ 1 s of wall time the decoder task's 1 Hz tick fires (`frame_decoder.c:667-673`):
   `now_wall − arr₀ = L + δ > 1 s` → **the session is reaped** (`ida_reassembler.c:49-50`),
   `cnt_expired++`, `parts_expired[1]++`, and `salvage_emit()` classifies the 20 lone opener bytes:
   7608, prehdr 0x26/7, `0x10` sub-header declares body 25 > 8 available → `truncated` →
   **the exact `PARTIAL 7608 DL f1 msg1/1 trunc` rows seen in `/messages`** (`frame_decoder.c:142-195`,
   `sbd_reassembler.c:314-418`).
5. The worker decodes the **continuation** (`da_ctr=1, da_cont=0, da_len=17, CRC OK`) one-to-many bursts
   later → fed → `find_matching_session()` finds nothing → **`cnt_orphan++`, silently dropped**
   (`ida_reassembler.c:138-141`). A `ctr>0` fragment can never open a session, so the chain is dead.
6. `sbd_reassembler_feed` is never called for the chain; `try_acars` never runs; `acars_decoded` stays 0.

**On-device signature (pre-A6 snapshot, one session):** `opened=31, merged=3, completed=1, orphan=34,
expired=30` with `parts_expired=[0,28,2,…]` — 28 of 31 chains died holding only their opener, orphan
count ≈ opener count, and only 3 fragments ever merged. Live re-check 2026-07-16: `opened=3, merged=1,
completed=0, orphan=2, expired=3` — same shape. `dirty_cont=0, dirty_emit=0`: the dirty path has
**never fired once**.

**Historical completeness** (why *"months"* of zero, not just since 07-12): every real ACARS envelope is
≥ 25 B ⇒ always ≥ 2 LW.DA bursts ⇒
- Before `3033190`/`ba0bf69` (**2026-07-06**): no IDA reassembler existed — each fragment went alone
  into `sbd_reassembler_feed`, whose `0x10` length check (`sbd_reassembler.c:194-197`) rejects the
  truncated envelope → **structurally zero, by design**, for the entire pre-07-06 period.
- 07-06 → 07-12: expiry ran *inside* `feed()` on the fed frame's RF clock (consistent), so decode *delay*
  was harmless; the blockers were correlated partner-loss (§2.6), ~50 % decode-order reversal → orphan
  (priority queue, not FIFO), and FRAG_GAP=280 ms orphaning every ARQ retransmit (retransmits cluster at
  0.36–0.54 s — deterministic for the retransmit path; fixed to 700 ms only in today's uncommitted diff).
  Expected true-pair completions over those 6 days: ~0–1. Zero ACARS is unremarkable here.
- 07-12 → now: the wall-clock reap (this bug) collapses the window to ≤ 0 under load ⇒ near-absolute.

---

## 2. Suspect-by-suspect

### 2.1 Dirty-flag routing (was "strongest") — **EXONERATED**

Two independent measurements kill the premise that continuations always fail their own CRC:

- **Device:** `dirty_cont = 0` in both the multi-hour pre-A6 snapshot and the live counters — not a
  single clean-demod/CRC-failed continuation has *ever* been seen, so the dirty→salvage route
  (`frame_decoder.c:570-580`) has never once executed (`dirty_emit = 0`).
- **Ground truth (36 h corpus, 55 k IDA lines):** of chain fragments, `opener: 874 CRC:OK / 1 CRC:no`,
  `middle: 627 / 3`, `final: 902 / 5 / 1 len=0` → **≥ 99.4 % of real continuations arrive
  `frag_crc_ok=true`**. (The big `crc=---` population — 31 294 lines — is `da_len=0` *standalone*
  frames, mostly 0x7605 ring alerts, which `ida_decode.c:259` correctly marks `crc_ok=false` and the
  gate drops; upstream `ida.py`'s ingest regex requires `CRC:OK` and ignores them identically. They
  account for most of `lw_da_gate_rejected=1096/1730` and are not ACARS-relevant.)

The observed `PARTIAL 7608 DL` rows are **not** dirty completions (those would be tagged
`PARTIAL DIRTY …`, `frame_decoder.c:185-188`); they are **timed-out salvage** rows (`… f1 msg1/1 trunc`)
— i.e. evidence for the §1 bug, not for this one. Answer to the design question: routing
dirty-but-complete chains to display-only PARTIAL is *not* too aggressive in practice (population is
empty); optionally a dirty chain whose ACARS layer then passes `la` CRC could be emitted flagged, but it
is irrelevant to the zero.

### 2.2 Freq-key change (Task #6, uncommitted, deployed today) — **PLAUSIBLE-SAFE, not the cause; needs one counter to close**

Correct on inspection: `peak_bin` is genuinely packed `bin | width<<16` (`dsp_processor.h:53-64`), the
mask matches, `uint64` intermediate avoids overflow, bin pitch 2.5 MHz/2048 = 1220.7 Hz so ±5 kHz = ±4.1
bins ≫ same-channel bin jitter over 90 ms (Doppler ≈ Hz-scale). Cannot be the historical cause
(freq key was constant 0 = always-match until today). Residual risk: no counter currently distinguishes
"orphan because ctr matched but freq key missed" from other orphans — add the split (§4) before trusting
it under Doppler. Note the pre-change behaviour had the opposite defect: zero frequency discrimination →
concurrent chains could cross-merge into franken-payloads that then fail SBD/ACARS parsing — a plausible
explanation for why even the rare pre-A6 completion (`completed=1`) produced no ACARS.

### 2.3 IDA→SBD handoff — **EXONERATED**

The merged 37 B fixture envelope walks `sbd_reassembler_feed` cleanly: type 7608, `0x26` → prehdr 7,
`msg_cnt = prehdr[3] = 1`, `0x10` sub-header len 25 = available body, `msg_cnt==1 && msg_no==1` →
`cnt_single`, emit (`sbd_reassembler.c:126-243`) — proven byte-exact by `test_acars_tail_real.c` AND
end-to-end on device by the smoke FRAME_DECODER corpus (which requires `acars_decoded ≥ 2` through the
real `frame_decoder_push` → classify → BCH → reassembler → SBD → libacars path, `smoke_test.c:264-441`).
The large `sbd.filtered` (561 of 567 fed in the snapshot) is fully accounted for by *standalone* ≤ 20 B
frames (0x7605 etc. classify UNKNOWN; lone 7608 mailbox frames fail the 0x10 length check) — it is not
eating completed chains (`completed=1` that session; ≤ 1 of the 561).

### 2.4 `try_acars` gating — **EXONERATED**

Identical logic to host `acars_tail.c` (SOH + 0x03/8-byte strip, ≥ 8 B gates, same libacars call and
COMPLETE/SKIPPED acceptance); device smoke proves it increments `s_acars_decoded` on-device when fed
both fragments. No best-effort flag touches this path.

### 2.5 `da_ctr`/opener matching — **EXONERATED as a code bug**

`feed_ex` mirrors upstream `ida.py` exactly (opener `ctr==0&&cont` → `next_ctr=1`; continuation must
equal `next_ctr`; mod-8 advance; >8-fragment wrap mis-read matches upstream by intent). The
`parts_completed[2]/[3] > 0` with zero ACARS is *not* chains completing "as something else" — it is (a)
non-7608 chains (0600 hello etc.) completing and legitimately not being ACARS, and (b) pre-freq-key
cross-merges (§2.2). One real orphan-inflation nit (already in the 07-15 design review): a retransmit of
an *already-merged* ctr counts as `orphan` rather than `dupe`, muddying the counter.

### 2.6 Worker partner-loss (context, not a code bug in this path) — **CONFIRMED contributor**

`stale = [.., 16-20 dB: 7166, 20-24: 2279, ≥24: 641]` in one session: the premise "even p = 0.5 gives
25 %" fails twice — measured p at ACARS-relevant SNR is far higher, and the two fragments' fates are
**correlated** (both sit in the same congested queue window; the evict-stale-first scan ages them
together), so (1−p)² is not the right model. This is what A6 (hot-bin boost) attacks. But even when both
fragments *do* decode, §1 then requires them back-to-back within one tick gap — which is why A6 alone
cannot fix the zero. (Live A6 counters already show the next-layer hazard: `pf_rej_hot_snr=8` —
continuations in hot bins being rejected by the *prefilter* before the queue ever sees them.)

---

## 3. What fraction of continuations could be clean? (deliverable 3)

≥ 99.4 % (corpus, §2.1); device has seen 0 dirty continuations ever. The dirty-routing design is sound
and irrelevant to the zero.

## 4. Cheap on-device confirmation (deliverable 4)

Existing counters already separate two of the three states:

- **chain never completed:** `ida.expired` + `parts_expired[]` (dominant today) and `ida.orphan`;
- **completed but dirty → salvage:** `dirty_emit` (= 0 forever ⇒ this path is NOT the answer);
- **completed clean:** `cnt_completed − dirty_emit`.

Add three one-line counters (diagnostic only, no behaviour change):

1. **`ida.reap_lag`** — in the *tick* drain only: count reaps where the chain is NOT stale in RF time,
   i.e. `(rf_newest_seen − s->last_time_us) < SESSION_TIMEOUT` at reap. If `reap_lag ≈ expired`, the §1
   clock bug is confirmed live in one session, zero risk. (rf_newest_seen = max `it->timestamp_us` fed.)
2. **`ida.orphan_freq`** — orphans where a session matched uplink+ctr+time but failed only the freq
   deadband (closes §2.2).
3. **`chain_sbd_fed / chain_sbd_rejected`** — on the `rc_reasm==1 && !dirty` path, count
   `sbd_reassembler_feed` calls and its −1 returns, so "completed-clean but SBD-rejected" (the §2.2
   cross-merge signature) is no longer hidden inside the standalone-dominated `sbd.filtered`.

## 5. The fix (deliverable 5)

**Primary — one clock for reassembly expiry.** In `frame_decoder.c`:

- Track `s_rf_now = max(s_rf_now, it->timestamp_us)` in `process_one`, plus `s_wall_at_rf_now =
  esp_timer_get_time()` when it advances.
- The 1 Hz tick drains with the **extrapolated RF clock**:
  `drain_now = s_rf_now + (esp_timer_get_time() − s_wall_at_rf_now)` — chains now expire after 1 s of
  *RF-time* inactivity (and still expire when the stream stalls entirely), never because the pipeline is
  lagging. Apply the same `drain_now` to `sbd_reassembler_tick` (same latent mix, 5 s scale) and keep
  `ida_salvage_drain(it->timestamp_us)` at the feed site as-is.

Risk: **low** — all state is owned by the single decoder task; no ISR/multi-core interaction; salvage
still fires (just at the correct age); PARTIAL emission timing shifts by the lag, nothing else. Corner
cases: smoke-fixture future timestamps still work (extrapolation is monotonic); RF-timestamp regressions
across a stream restart are already guarded by reap's `now_us > last_time_us`.

**Also commit** (already uncommitted-deployed): FRAG_GAP 280 → 700 ms (kills the deterministic
ARQ-retransmit orphaning) and the peak_bin freq key (with counter #2 above).

**Validation:**

- **Host test that exercises the device orchestration gap:** feed opener at RF t₀; call
  `ida_reassembler_reap` with `now = t₀ + 2.5 s` (simulating the tick under 2 s lag — current wiring);
  feed continuation at t₀ + 90 ms → today this orphans (demonstrates the bug); with the drain-clock
  policy under test (reap driven by extrapolated-RF now) the chain completes. Plus a stall case: no
  feeds for > 1 s of extrapolated time → salvage still fires.
- **Device smoke, unmasked:** add a FRAME_DECODER corpus variant that pushes the opener with
  `timestamp_us` rebased to `esp_timer_get_time() − 2 s`, sleeps 1.5 s (guaranteeing a tick), then
  pushes the continuation at opener+90 ms. Fails on current firmware (orphan), passes with the fix.
  Keeps the existing back-to-back variant as-is.
- **Live:** within one session, expect `ida.merged/completed` and `parts_completed[2]` to rise,
  `parts_expired[1]/opened` to fall, `reap_lag ≈ 0`, and — with A6 landing continuations back-to-back —
  the first `acars_decoded > 0`.

---

### One-line summary

The reassembly window is measured with two different clocks: sessions live on RF-arrival time but the
1 Hz reaper uses wall time, so under the P4's permanent ≥ 1 s decode lag every open chain is reaped
before its continuation can arrive (`frame_decoder.c:667/673` vs `worker_core1.c:941`,
`ida_reassembler.c:49-50`); the device smoke can't see it because fixture timestamps are in the far
future, which disables the reaper entirely. Fix: drive all reassembler expiry off the (extrapolated)
RF clock; confirm with a `reap_lag` counter.
