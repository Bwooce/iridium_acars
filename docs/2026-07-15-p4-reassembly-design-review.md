# P4 IDA reassembly design review — FRAG_GAP, partial-decode queue, drop-model spec

Date: 2026-07-15 (AEST)
Scope: `common/iridium_decoder/ida_reassembler.c/.h`, its driver in
`p4-usb-host/main/frame_decoder.c` (feed at ~529, salvage drain at ~177/520/636), and the
proposed hole-tolerant "partial-decode queue". Review only — no production code changed.

Evidence base (all read-only, reproducible from the scratchpad):
- 45-min HydraSDR bench corpus: `corpus.parsed` / `corpus_frags.tsv` (1115 data-bearing IDA
  fragments, 100 chain openers, 162 continuations, 78 assembled multi-fragment chains).
- Validated A/B harness `drive_reasm.py` (subclasses the real iridium-toolkit
  `ReassembleIDA`; reproduces the reference exactly: 78 assembled / 182 frag / 25 broken /
  260 dupes).
- Fresh strict-reassembly re-simulation + false-merge hazard scan (`gap_analysis.py`, this
  review) and the retransmission census (`retrans_analysis.py`).
- 36 h fragment log `ida_frags.tsv` still accumulating via `process_hour.py` (132 rows so
  far — too small to lean on yet).

---

## 1. Executive summary

| Question | Verdict |
|---|---|
| FRAG_GAP 280 ms → 700 ms | **KEEP 700 ms** — but the stated rationale is wrong and must be corrected. 700 ms buys nothing on clean chains (per-hop gaps never exceed 180 ms); its real value is that it silently captures the first ARQ retransmit (0.36–0.54 s) of a head-of-line fragment the worker dropped. Fix the header comment; no code change. |
| SESSION_TIMEOUT / slot count | **KEEP 1 s and 4 slots.** Measured concurrency ≈ 0.04 open chains; multi-match rate 0. |
| Hole-tolerant partial-decode queue | **DON'T BUILD (now).** Its benefit ceiling is the *natural* retransmit rate of ACARS-chain continuations — measured ≈ 3% (n = 3 pairs in 45 min, thin) — and the strict design at 700 ms already captures the dominant retransmit pattern for free. The established lever, protecting open-chain continuations from stale-drop, addresses 100% of compute-drops and is strictly dominant: at p = 0.3 it lifts 2-burst completion 0.49 → 0.70 (+43%) vs ≲ +3% for hole-tolerance. |
| Decisive measurement | **RUN IT** (host-only, ~1 day of work): Bernoulli drop-model over the un-de-duped corpus, strict-280 vs strict-700 vs hole-tolerant, plus a "continuations never dropped" arm that bounds the protection lever. Go/no-go threshold in §4. The same harness re-runs free on the 36 h corpus when it lands. |

Bottom line: the reassembler is not where the ACARS is being lost. Spend the firmware
effort on the worker's stale-drop policy (protect open-chain continuations), keep the
700 ms window as a free retransmit net, and let the drop-model + 36 h data decide whether
hole-tolerance ever earns its complexity.

---

## 2. Task A — FRAG_GAP = 700 ms verdict

### 2.1 The 280 ms value was NOT clipping legitimate chains — the justification conflated span with hop

`find_matching_session()` gates on `now_us - s->last_time_us`, i.e. the **per-hop** gap
between consecutive accepted fragments, not the chain's first→last span. Re-simulating the
strict algorithm over `corpus_frags.tsv`:

```
per-hop gap: median 90 ms, p90 90 ms, p99 180 ms, max 180 ms
chain span : median 180 ms, max 450 ms
assembled  : 78 at 280 ms = 78 at 700 ms = 78 at 1000 ms   (identical)
```

Hops are quantised to 1–2 Iridium TDMA frames (90 ms). The 0.36–0.45 s figures are total
*spans* of 3–5-fragment chains, which a per-hop gate never sees. So on clean data 280 ms
dropped **zero** chains, and the "~3% clipping" rationale currently written into
`ida_reassembler.h` (lines 56–63) is factually wrong. A second part of that rationale is
also wrong: "worse under P4 timing jitter" — `frame_decoder_push()` stamps
`item.timestamp_us` from the **burst sample position**, so worker/queue latency does not
stretch observed inter-fragment gaps at all (only genuinely out-of-order *processing*
matters, see §3.4).

### 2.2 …but 700 ms is still the right value, for a different reason: retransmit capture

The retransmission census on the same corpus (`retrans_analysis.py`): 190 same-channel
duplicate pairs; first-retransmit gap clusters hard at **0.36–0.54 s** across every type
(0x7605 median 0.36 s, 0x0512 0.45 s, 0x0605 0.54 s, 0x7608 chain continuations 0.36 s).

Now consider the P4 compute-drop regime. If the worker stale-drops continuation `ctr=k`,
the session's `next_ctr` stays `k` and `last_time_us` stays at the previous accepted
fragment. When the network retransmits (go-back-N style, resending from the un-ACKed frame
onward), the retransmitted `ctr=k` arrives with hop gap ≈ 0.36 + 0.09 ≈ **0.45–0.54 s**
from the last accepted fragment:

- at FRAG_GAP = 280 ms → orphan-dropped, chain dead;
- at FRAG_GAP = 700 ms → `ctr == next_ctr` matches, chain **completes in strict order,
  zero new code**.

So the 280→700 ms change is precisely the cheap, safe fraction of the hole-tolerance
proposal — head-of-line retransmit fill — implemented as a constant. It should be kept and
its comment rewritten to say this. (A 1 s window would also catch second retransmits at
~0.72–0.9 s, but ties the gap to SESSION_TIMEOUT and creates a reap race at the boundary;
not worth it — see 2.4.)

### 2.3 False-merge risk at 700 ms: measured zero, bounded negligible

A false merge needs an unrelated continuation with **exactly** `ctr == next_ctr`, same
link direction, within ±5 kHz, inside the window, while a chain is open. Scanning both
fragment logs for same-ctr different-payload pairs within ±5 kHz:

```
W=280 ms: 0 pairs    W=700 ms: 0 pairs    W=1000 ms: 0 pairs   (0.0/h in 45 min + 0.5 h)
```

Analytic bound: continuation rate 0.066/s across the whole 2.5 MHz passband → in a ±5 kHz
slice ≈ 2.6×10⁻⁴/s, × ~1/7 ctr-match ≈ 4×10⁻⁵/s hazard per open chain. With ~2.2 chains/min
each exposed ≤ 0.7 s, expected false merges ≈ **10⁻³ per 45 min**. Even a 10× busier
overhead pass leaves this in the noise, and the SBD/ACARS CRC-16 above would reject the
corrupt merge anyway (cost: one lost chain, which was already at risk). Caveat: bench
corpus is RF-quiet; re-check the hazard scan on the 36 h log (action A5).

### 2.4 SESSION_TIMEOUT and slot count

- **SESSION_TIMEOUT = 1 s: keep.** The retransmit-capture path needs the session alive at
  hop-gap 0.45–0.54 s; 1 s covers it with ~2× margin while keeping salvage latency low.
  Raising it would only serve *second* retransmits (rare) and delays salvage/PARTIAL emit.
  Invariant to preserve: `FRAG_GAP < SESSION_TIMEOUT` (a fragment must not be acceptable
  to a session the reaper considers dead).
- **4 slots: keep.** cnt_opened ≈ 0.04/s × ≤1 s lifetime → expected concurrency ≈ 0.04;
  P(4 simultaneous) is astronomically small, and `find_matching_session` observed 0
  multi-candidate events. `reap-before-feed` already frees slots ahead of need.
- **Optional (cheap, useful for §4):** count retransmit dupes separately from true
  orphans — in the orphan path, if a session on-frequency holds `s->next_ctr > ida->da_ctr`
  (already merged this ctr), bump a new `cnt_dupe` instead of `cnt_orphan`. This gives the
  on-device measurement of how often the 700 ms net actually catches/sees retransmits.

**Verdict: 700 ms is correct; correct the comment (A1), keep 1 s / 4 slots, add cnt_dupe.**

---

## 3. Task B — partial-decode queue (hole-tolerant, out-of-order, retransmit-fill)

### 3.1 The benefit ceiling is set by the *real link's* ARQ, not ours

The retransmissions we observe exist because the **real** recipient (aircraft/gateway)
NAKed a frame. Our eavesdropping receiver's compute-drops are invisible to that link and
uncorrelated with its ARQ. Therefore a hole in *our* chain gets filled only when the real
link *happened* to retransmit that fragment anyway. Measured base rate for the traffic
that matters: **3 retransmitted 0x7608 chain-continuation pairs in 45 min**, against
~100 chain continuations → natural retransmit probability ≈ **3% per fragment**
(n = 3 — very thin; the 36 h log exists to fix this).

Ceiling math at per-burst drop p and natural-retransmit rate r ≈ 0.03: hole-tolerance
turns a 2-burst chain's completion from (1−p)² into ≈ (1−p)(1−p(1−r·(1−p))) — at p = 0.3
that is 0.49 → 0.494. The established lever (protect open-chain continuations from
stale-drop) turns it into ≈ (1−p) = 0.70. **+1% vs +43%.** Hole-tolerance is not the
right lever for the compute-drop problem; it is a residual-polish lever at best.

Two further discounts on the ceiling:
- **Drops are bursty.** Stale-drops happen during floods (worker at 242%); 0.36 s later
  the flood is usually still in progress, so the retransmit faces a similar drop
  probability. The independent-p model above is optimistic for hole-tolerance.
- **Strict-700 already banks the common case.** Go-back-N-style retransmission (resend
  from the missing frame onward) completes under the existing strict matcher (§2.2).
  Hole-tolerance adds value only for selective-repeat patterns — a *later* fragment
  arrives first and must be buffered while the earlier hole waits. Whether Iridium's
  LAPDm-derived ARQ ever does selective repeat on this bearer is unknown; the drop-model
  (§4) answers it empirically without us needing the spec.

### 3.2 A/B evidence already in hand

`drive_reasm.py` (validated against the pristine reference) showed out-of-order slotting +
windows up to 280 s recovers **zero** extra chains on clean data (78 → 78, 0 slot
conflicts). This review's re-run confirms: holes absorb 5 orphans into open chains but
complete nothing extra. On clean data every assemblable chain completes in-order; the ~25
broken are fragments never received. The queue's benefit is strictly conditional on
(receiver loses fragment) ∧ (fragment reappears) — the bench has neither; the P4 under
flood has the first and *rarely* the second.

### 3.3 P4-specific design risks (were we to build it)

- **Memory: trivial.** Per-slot fragment store: 8 ctr-slots × 24 B + validity bitmap +
  per-slot timestamps ≈ ~230 B/session over the current 320 B buffer; ×4 sessions ≈ +1 KB
  BSS. Not a constraint.
- **Compute: trivial** at ≤ 0.07 continuations/s. The decoder task's one-frame-per-wake
  yield discipline (frame_decoder.c:640) is unaffected.
- **mod-8 ctr wrap ambiguity: real.** Strict matching tolerates wrap only for > 8-fragment
  chains (upstream-matching behaviour, documented at ida_reassembler.c:110). A
  hole-tolerant matcher that accepts *any* not-yet-filled ctr within the window converts
  wrap into aliasing: a late fragment of chain N and an early fragment of chain N+1 on the
  same channel become indistinguishable. Needs "accept only ctr in a bounded forward
  window from the highest filled slot" logic — more state, more edge cases.
- **Retransmit-vs-new-chain ambiguity at ctr=0: real.** A retransmitted opener (same ctr=0,
  same payload) vs a new chain opener (same ctr=0, different payload) on the same channel
  within the window must be disambiguated by payload compare; getting it wrong either
  duplicates or destroys a chain. Strict + slot-full drop currently sidesteps this
  (`find_free_session` just opens a second slot).
- **Salvage-path interaction: the expensive part.** `ida_reassembler_reap()` /
  `salvage_emit()` / `sbd_salvage_parse()` all assume a **contiguous byte prefix**. A
  holey buffer salvages as fragments-with-gaps; sbd_salvage_parse cannot walk past a hole
  (lengths/sub-headers become unaligned garbage). Either salvage only the contiguous
  prefix before the first hole (loses the buffered later fragments — most of the queue's
  point) or teach the whole PARTIAL path about gap maps (touches frame_decoder.c salvage,
  msg_ring text format, Task B4/C contracts). This is where the complexity actually lands.
- **Dirty-flag (Task C) interaction:** per-chain `dirty` must become per-slot so a
  retransmitted clean copy can *clear* a dirty slot — otherwise the queue fills holes but
  still emits PARTIAL. Small but easy to get wrong.

### 3.4 One cheap sibling worth measuring: reorder tolerance (not hole tolerance)

If the worker's queue-priority/stale-drop model can ever *process* burst B before an
earlier burst A (RF-timestamped), the reassembler sees time go backwards and
`find_matching_session` rejects (`now_us < s->last_time_us`, ida_reassembler.c:36), or an
opener arrives after its continuation → orphan. That is a *pipeline reorder* failure, not
an RF one, and it is fixable with a tiny K-deep timestamp-sort holding buffer at the
decoder — no protocol ambiguity at all. Whether it happens is a single counter away:
log/count `now_us < last_time_us` rejections and opener-after-continuation orphans
(action A4). If nonzero under flood, fix ordering, not the reassembler.

### 3.5 Recommendation

**DON'T BUILD** the hole-tolerant queue now. **MEASURE FIRST** (§4) — the go/no-go is
cheap and reuses the validated harness. **BUILD instead** the established lever: worker
stale-drop protection for open-chain continuations (when the reassembler holds an open
session, pin bursts within ±5 kHz of it for ~1 s against stale-drop; the completion math
in §3.1 says this is worth an order of magnitude more). That protection design is separate
work; note it interacts *positively* with FRAG_GAP=700 ms — a protected-but-delayed
continuation still matches because timestamps are RF-derived.

---

## 4. Task C — the decisive drop-model measurement

Goal: empirically bound what hole-tolerance recovers under the actual failure mode
(compute-drop) with the actual retransmit supply (kept in-stream), vs what
continuation-protection recovers, on the same data.

### 4.1 Harness

Extend `drive_reasm.py` (scratchpad; already validated to reproduce the reference
78/182/25/260 exactly). Add a drop stage **at the input line level, upstream of the
reference's consecutive-frame de-dupe** — this is the critical detail: dropping the
original before the reassembler sees it means the retransmit the reference would have
discarded as one of its 260 dupes now arrives as a first-sighting and is *available* to
fill the hole. Dropping after de-dupe would assume away the entire effect.

- Drop model: i.i.d. Bernoulli per IDA frame with p ∈ {0.1, 0.2, 0.3, 0.4, 0.5};
  ≥ 20 seeds per point. Add one **bursty variant** (drop in 0.5 s on/off gates at the same
  average p) since device drops are flood-correlated — this directly tests the "retransmit
  arrives mid-flood" discount of §3.1.
- Arms per (p, seed):
  1. strict, per-hop 280 ms (old firmware);
  2. strict, per-hop 700 ms (deployed — includes free head-of-line retransmit capture);
  3. hole-tolerant OOO, window 700 ms, timeout 1 s (the firmware-realistic queue);
  4. hole-tolerant OOO, timeout 2 s (upper bound: catches second retransmits);
  5. strict 700 ms with **continuations exempt from dropping** (openers still drop) —
     this is the simulation of the stale-drop-protection lever, on identical data.
- Ground truth: the p = 0 assembled set (payload bytes). Per arm count:
  - multi-fragment completions, total and 0x7608-only (the ACARS carrier);
  - **false merges** = completed payloads not present in ground truth;
  - salvage quality: broken chains' contiguous-prefix length (feeds the PARTIAL path).
- Report mean ± 95% CI per (arm, p). Runtime is seconds per pass; the whole grid is
  minutes.

### 4.2 Inputs

- Now: `corpus.parsed` (45 min, 78 ground-truth chains, 37 multi-frag 0x7608 openers,
  3 retransmit pairs — **underpowered for the hole-fill effect**, fine for the protection
  arm and false-merge measurement).
- When the 36 h run completes: re-run on the accumulated capture. `ida_frags.tsv` persists
  per-fragment rows (`process_hour.py` deletes raw .parsed), so either (a) feed the TSV
  through a thin adapter emitting synthetic parsed lines, or (b) patch `worker.sh` **now**
  to also archive IDA: lines from each hour's .parsed before deletion (few MB/day; do this
  — it preserves the exact reference-compatible input). Power gate: require ≥ 30 observed
  0x7608 chain-continuation retransmit pairs before treating the hole-fill delta as
  measured rather than anecdotal; at the bench rate (~3/45 min raw, but ACARS-sparse
  hours exist) 36 h should yield O(50–100) if the 45-min rate holds — flag if it doesn't.

### 4.3 Go/no-go for firmware hole-tolerance

BUILD the partial-decode queue only if ALL of:
1. At the device-measured drop rate p̂ (from worker `dropped=` vs processed counters
   during representative flood hours; use p = 0.2 and 0.4 brackets if p̂ is noisy), arm 3
   recovers **≥ 10% more multi-fragment 0x7608 completions than arm 2** (relative), with
   the bursty drop model, and the gain is outside the 95% CI;
2. False merges < 1% of completions in arm 3 at all p;
3. Arm 3's gain is **≥ 25% of arm 5's gain** on the same (p, seed) grid — i.e.
   hole-tolerance must deliver at least a quarter of what stale-drop protection delivers
   before it can justify riding alongside it (if protection ships, the residual drops
   hole-tolerance could fix shrink further);
4. The ≥ 30-retransmit-pair power gate (§4.2) is met.

Expected outcome, stated for the record: arms 2 and 3 differ by ~r ≈ 3% (within noise),
arm 5 dominates everything, and the answer stays DON'T-BUILD. If the 36 h data shows a
much higher retransmit rate at night/under different beam geometry, the model updates
honestly.

### 4.4 On-device companions (no redesign, counters only)

- `cnt_dupe` (§2.4): how often the 700 ms net sees retransmits in production.
- Reorder counters (§3.4): backwards-time rejects + opener-after-continuation orphans.
- Existing `parts_expired[]` histogram already gives the broken-chain shape; watch it
  before/after any stale-drop-protection deployment.

---

## 5. Ordered actions

1. **A1 (doc-only, now):** Rewrite the FRAG_GAP comment in `ida_reassembler.h:56-63` —
   remove the "clips legit chains / P4 jitter" rationale (measured false: per-hop max
   180 ms; timestamps are RF-derived), replace with the retransmit-capture rationale
   (first ARQ retransmit at 0.36–0.54 s; window must stay < SESSION_TIMEOUT). Value stays
   700 ms.
2. **A2 (scratchpad, now):** Patch `worker.sh`/`process_hour.py` to archive each hour's
   IDA: lines before deleting .parsed, so the 36 h corpus can drive the reference-exact
   harness (§4.2). Cheap, and the data is otherwise lost as it rolls.
3. **A3 (host, ~1 day):** Implement and run the §4 drop-model grid on `corpus.parsed`;
   publish the arm-2 vs arm-3 vs arm-5 table. This also produces the first quantitative
   sizing of the stale-drop-protection win before any firmware work.
4. **A4 (firmware, small):** Add `cnt_dupe` + reorder counters (§2.4, §3.4) to
   `ida_reassembler.c` / `frame_decoder.c` stats; surface in
   `frame_decoder_get_reasm_stats`. Device-smoke gate applies (DSP-adjacent counters only,
   but follow the mandatory-smoke rule).
5. **A5 (analysis, when 36 h lands):** Re-run the false-merge hazard scan and the
   retransmission census on the full capture; check the ≥ 30-pair power gate; re-run A3 on
   the archived hours. Apply §4.3 go/no-go.
6. **A6 (firmware, the real lever — separate design):** Open-chain continuation
   protection in the worker stale-drop policy (pin ±5 kHz of any open IDA session for
   ~1 s), sized by A3's arm 5. This, not the reassembler, is where completion ≈ (1−p)²
   becomes ≈ (1−p).

## UPDATE 2026-07-15: the §2.3 false-merge deadband was VACUOUS, now fixed (Task #6)

While speccing A6 it was found (VERIFIED) that both decode paths passed `freq_hz=0` into
`frame_decoder_push`, so the reassembler's ±5 kHz deadband was `abs(0-0)<5000` = always true —
NO frequency discrimination. §2.3's false-merge analysis assumed a working deadband, so real
risk was higher (bounded: CRC-16 rejects corrupt merges → lost chain, not corrupt output).
FIXED in `frame_decoder.c`: the reassembler key is now derived from `peak_bin`
(`reasm_freq_key_hz`, ~1220.7 Hz/bin, ±5 kHz ≈ ±4 bins vs ~34-bin channel spacing). Host-proof
the fix loses ZERO legitimate chains: strict-700 completions are identical at deadband ∞/5k/260 Hz
on both corpus (73/73/73) and 5 h of the 36 h archive (106/106/106) — same-chain fragments are
same-frequency. Deployed via OTA; device healthy. §2.3's analysis now applies as written.

## A3 RESULTS (run 2026-07-15, `dropmodel.py`, 20 seeds, corpus.parsed)

Ground truth (strict-700, p=0): 73 unique multi-frag chains, 31 are 0x7608. Metric = coverage
(% of the 31 ground-truth 0x7608 chains recovered). Confirms every §1/§3 prediction:

Bernoulli drops — 0x7608 coverage %:
```
 p    strict-280  strict-700  ooo-700/1s  ooo-700/2s  strict-700+PROTECT
0.1     78.2        78.9        78.9        78.9        89.4
0.2     62.3        62.9        62.9        62.9        80.2
0.3     48.7        49.7        49.7        49.7        71.1
0.4     33.9        34.7        34.7        34.7        59.5
0.5     22.6        23.5        23.5        23.5        48.7
```
(bursty drops: same ordering, protection still dominates at every p.)

Findings:
- **out-of-order (arms 3,4) ≡ strict-700 (arm 2) to 3 decimals — recovers ZERO extra.** Go/no-go
  criterion #1 (arm3 ≥10% over arm2) FAILS immediately (+0%). **DON'T-BUILD confirmed empirically.**
- strict-700 vs strict-280: +0.7–1.0 pts (the free head-of-line retransmit capture) — small, real.
- **continuation-protection (arm 5) dominates: at p=0.3, 71.1% vs 49.7% = +43% relative** — exactly
  the §3.1 prediction (0.49→0.70). Every p shows the same order-of-magnitude advantage. **A6 is
  the lever; sizing confirmed.**
- Caveat: n=3 retransmit pairs (power gate ≥30 unmet), so the ooo=+0 is underpowered — but the
  mechanism (go-back-N caught by strict-700; selective-repeat ~absent) makes it robust. 36 h log
  (A5) will confirm the retransmit rate; the DON'T-BUILD verdict is already clear.

## Evidence-thinness register

- 0x7608 retransmit rate: **n = 3 pairs** — the single most load-bearing small number in
  this review; everything in §3.1 scales with it. The 36 h corpus is the fix.
- False-merge hazard: measured on RF-quiet bench data only; analytic bound says negligible
  even at 10× density, but re-verify on 36 h (A5).
- Go-back-N vs selective-repeat ARQ: inferred from 3 pairs' pattern + LAPDm lineage, not
  from spec; the drop-model measures the consequence without needing the truth.
- `ida_frags.tsv` (36 h run) currently holds only ~0.5 h of fragments; nothing in this
  review leans on it yet (per the no-premature-conclusions rule).
