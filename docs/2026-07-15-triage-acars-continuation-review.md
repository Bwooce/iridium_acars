# Triage/prefilter vs 0x7608 continuations — code review

Date: 2026-07-15 (AEST). Status: CODE REVIEW + HYPOTHESIS — nothing here is measured on
device yet; §5 specs the measurement. No production code changed.

Question under review: **is the triage fast-pass (burst_prefilter) rejecting valid 0x7608
continuation fragments, preventing multi-fragment IDA chains from completing into ACARS?**

Files examined:
- `common/iridium_decoder/burst_prefilter.c` / `.h` (the P1.5b fast-pass — the only triage
  in the live path)
- `p4-usb-host/main/worker_core1.c` (PQ + orchestration; prefilter call at :1187-1199)
- `common/iridium_decoder/burst_pipeline.c` (post-triage pipeline; `burst_pipeline_triage`
  at :562 is the DEAD P1.5a path — no production caller, only worker comment :1121-1128 and
  host tests reference it)
- `common/iridium_decoder/ida_decode.{h,c}`, `ida_reassembler.h` (fragment physics)
- `tests/host/test_burst_prefilter.c` (the "recall-safe" evidence)
- `docs/2026-07-15-a6-continuation-priority-boost-spec.md`, `hot_bin_table.{h,c}`

---

## 1. VERDICT

**Plausible contributor — via the channel-SNR gate (gate 2), NOT the duration gate.
The task's central premise — that a short final fragment is physically shorter on air and
falls under `MIN_FRAME_LENGTH` — is FALSE for IDA frames; the duration gate cannot be the
mechanism. But the SNR gate re-applies the 14 dB detection threshold with a *different,
never-cross-calibrated estimator*, so any real burst detected marginally above the tagger
threshold faces a second, ~independent coin-flip at the same nominal level. A chain needs
BOTH bursts to pass, so this loss channel is squared for chains — exactly the
`(1−p)²` structure already identified for stale-drops — and it sits AFTER the PQ pop, so
A6 cannot bypass it.**

Confidence: mechanism confirmed in code; magnitude unknown (zero on-device evidence either
way — there is no per-gate reject breakdown, see §5). Treat as "must measure", not "must fix".

### Why the duration gate is exonerated (the premise is wrong)

An IDA (LW.DA / 0x7608-bearing) frame is **fixed-length on air regardless of payload
bytes**. Proof from the project's own decoder:

- `ida_decode.c:12-14` — `UW_BITS 24`, `LCW_BITS 46`, `DATA_BITS_TOTAL 312`.
- `ida_decode.c:114` — `if (frame->n_bits < UW_BITS + LCW_BITS + DATA_BITS_TOTAL) return -1;`
  A frame shorter than the full 382 bits is *invalid input*, not a valid short fragment.
- `ida_decode.h:54-63` — `da_len` (5 bits, 0..24) is a *byte count inside* the fixed
  312-bit data section; the CRC sits at fixed positions `bits[180..196]`
  (`ida_decode.c:227-228`). A continuation carrying 3 payload bytes is padded; the
  transmitted burst is identical in symbol count to a full one.

382 bits = **191 symbols** post-preamble (matching `MAX_FRAME_LEN_NORMAL_10SPS = 191 × 10`
in `burst_pipeline.c:290`), ≈ 207 symbols ≈ **8.3 ms** with the 16-symbol DL preamble.
The duration gate requires 80 symbols (3.2 ms) of active envelope — a real continuation
clears it with a ~2.4× margin. "Short final fragment" is a *payload*-level fact
(`da_len` small), not an RF-duration fact. The `PARTIAL 7608 DL` entries in `/messages`
are salvage output of chains missing a fragment (and demod truncations), not evidence of
short bursts on air.

---

## 2. Gate-by-gate analysis

All three gates run in `burst_prefilter()` (`burst_prefilter.c:87-264`) on the decimated
250 ksps, DC-centred window; short-circuit order: width → duration → SNR.

### Gate 0 — spectral width (`width_ok`, :99-106)

- **Threshold:** `width_bins <= PF_MAX_WIDTH_BINS = 100` tagger-FFT bins (:62); one Iridium
  channel ≈ 34 bins; `width_bins <= 0` (unmeasured) always passes.
- **Continuation risk: none.** A continuation is a normal ~1-channel burst (~34 bins),
  3× under the limit. Reject requires >3 channels of contiguous above-threshold width —
  broadband RFI, not a frame.
- **Grounding:** the *measurement* (tagger `width_bins`) is gri-derived, but gri has **no
  width gate anywhere** — this gate is project-original. It is, however, so conservative
  (100 vs 34, and 2× the PQ's own `BURST_NARROW_MAX_BINS = 48` demotion line,
  worker_core1.c:90) that it is recall-safe for any single-channel burst by construction.

### Gate 1 — active-envelope duration (`dur_ok`, :108-170) — the prime suspect, ACQUITTED

- **Threshold:** `active_len >= PF_MIN_FRAME_SAMPLES = 800` samples at 250 ksps
  = `MIN_FRAME_LENGTH_SIMPLEX (80 symbols) × 10 sps` (:36-37). `active_len` counts
  boxcar-10-smoothed power samples above `sqrt(floor_env × peak_env)` (geometric-mean /
  half-power-in-dB crossing, :149) anywhere in the window (non-contiguous).
- **Does a continuation clear it?** Yes, structurally: every IDA fragment is ~2 070
  active samples (§1) vs the 800 needed. For it to fail, <3.2 ms of the burst would have
  to be inside the extraction window — but the window is `start .. last_active + 16 ms
  post-pad` (tagger gone-event), so a mid-burst truncation cannot occur by construction.
  The known historical hazard (P1 squelch force-closing real bursts short — memory note
  `project_tagger_frozen_baseline_latch`) shortens `length_samples`, but the pop-side
  guards (`< 128` raw at worker_core1.c:1046, `n_250k <= 64` at :1167) catch degenerate
  cases before the prefilter, and a squelch-clipped window still contains whatever burst
  span preceded the close plus 16 ms pad.
- **Recall-safe for continuation-length bursts?** Yes — *because continuation-length ==
  full-frame length for IDA*. The 80-symbol bound would matter only for a hypothetical
  frame class shorter than SIMPLEX, which does not exist in this traffic.
- **Grounding:** genuine gri counterpart (`burst_downmix_impl.cc:502` drops
  `burst_size − start < min_frame_length`), and using the SIMPLEX minimum (80) instead of
  NORMAL (131) is the conservative choice. The *statistic* differs (gri: window length
  from `start_finder`; ours: above-half-power sample count), but the direction of every
  difference is permissive (noise samples above threshold also count toward `active_len`).
- **Residual edge case (not continuation-specific):** a strong CW/carrier co-located in
  the window lifts `floor_env` toward `peak_env`, pushing `thr_env` up; a real burst
  riding on a continuous interferer could under-count `active_len`. Bench near-DC
  artifact scenario; flag, don't fix.

### Gate 2 — integrated in-band channel SNR (`snr_ok`, :172-259) — THE SUSPECT

- **Threshold:** `channel_snr_db >= PF_THRESH_DB = 14.0` (:68), where
  `channel_snr = p_in / (median_oob_bin × 329)`: one 2048-pt FFT (8.192 ms — almost
  exactly one 8.3 ms burst) on the highest-energy segment, in-band = DC ± 164 bins
  (±20 kHz = gri `burst_width` 40 kHz), noise floor = median of the 1 719 out-of-band
  bin powers.
- **Mechanism against continuations:** every burst reaching the prefilter was already
  detected by the tagger at ≥ its threshold (default 14 dB, `dsp_processor.c:52`). Gate 2
  re-tests *nominally the same condition* with a **different estimator** (median-OOB-bin
  noise floor over one 8.2 ms segment vs the tagger's rolling per-bin baseline at 2.5 MSPS
  resolution; integrated 329-bin power vs peak-bin excess). Two estimators of the same
  quantity at the same threshold disagree on marginal inputs: a burst the tagger scored
  14–17 dB can measure 12–13.9 dB here and be dropped. For a 2-burst chain whose opener
  passes at 18 dB, a continuation at tagger-15 dB is exactly the population at risk.
  The stale-drop histograms already show real mass at 16–20 dB
  (`s_drop_stale_snr`, memory note: "30k+ strong 16-20dB bursts"), so the marginal band
  is populated on this bench.
- **Is "recall-safe" proven for continuations?** No. The positive control
  (`test_burst_prefilter.c:253-262`, "zero false-rejects") quantifies over **bursts the
  full pipeline decodes on the ALBQ fixture** (~62 bursts, 50-min bench corpus,
  bench SNR 12-24 dB). That is (a) one corpus, (b) n≈62, (c) conditioned on decodability
  *and* on being in the fixture — it cannot bound the false-reject rate of the marginal
  14-17 dB class, and nobody has checked whether the fixture's decoded set contains ANY
  multi-fragment IDA continuation at marginal SNR. "Recall-safe" is an existence proof on
  strong bursts, not a proof over the continuation population.
- **Grounding audit:** the *constants* trace to gri (40 kHz `iridium-extractor:126`;
  18 dB `iridium-extractor:130`; −4 dB domain scale from the memory note) — so this is
  not test-fitting. **But the gate itself has no gri counterpart: gr-iridium never
  re-gates on SNR at decode time** — `burst_downmix` decodes everything the tagger
  tagged. And the "ours ≈ gri − 4 dB" calibration was measured for the *tagger's*
  statistic; transplanting the number onto a third statistic (median-OOB integrated SNR)
  is an unvalidated transfer. Header's "same threshold the tagger applied" (:67,
  burst_prefilter.h:44-46) is only nominally true.
- **Threshold-coupling bug (latent):** `PF_THRESH_DB` is compile-time 14.0, but the
  device tagger threshold is NVS-overridable (`tag_thr`, `dsp_processor.c:51-52,281-282`).
  Set `tag_thr` = 12 for an experiment and the prefilter silently rejects the entire
  12-14 dB band the tagger was asked to admit — the prefilter becomes the binding
  detection threshold with no log line saying so.

### Systematic biases of gate 2 (direction check)

Permissive (good): `p_in` includes in-band noise (+0.17 dB at 14 dB true SNR); the decim
FIR's transition band attenuates the outermost OOB bins, biasing the median floor low.
Restrictive (bad): the best-8.192 ms-segment can clip burst edges; RRC sidelobes of the
burst itself land OOB and lift the median; a neighbour burst inside the window lifts it
further. Net bias: **unknown — this is precisely the cross-calibration measurement M-C
(§5) that has never been done.**

---

## 3. Ordering: triage vs PQ vs A6 — does triage undercut A6?

The order in `worker_task` (worker_core1.c:1021-1243) is unambiguous:

```
tagger callback (Core 0)
  → worker_core1_push_burst (:1374)      [stale-reject at push :1394-1410]
  → pq_insert_locked (:202)              [A6 boost affects eviction/admission :206-251]
  → ... queue wait ...
  → pq_extract_max_locked (:261)         [A6 boost decides WHICH burst pops :133-139]
  → length guard (:1046), stale guard (:1065)
  → extract + rotate + 10× decim (:1134-1144, ~ms of work already spent)
  → burst_prefilter (:1187-1199)         ← TRIAGE RUNS HERE, POST-POP
  → burst_pipeline_process_burst (:1217) [the ~94%-cost retry pipeline]
```

**Triage runs strictly AFTER the pop** (and after extraction/decim). Consequences:

1. **A6 is partially undercut, not wasted.** A6's target loss channel is the *stale-drop*
   (ring laps the continuation while it queues) — that happens **before** the pop, and A6
   genuinely fixes it. But A6 hands its boosted continuation directly to the prefilter:
   if gate 2 rejects it, the boost bought an extraction+decim (~ms) and a rejection.
   The A6 spec even codifies this (§7.8: "A boosted burst still pays the normal triage
   fast-pass") — written as a *feature* (junk protection) without noticing it applies to
   the real continuation too. So the post-A6 loss model for a continuation is:
   `P(lost) = P(stale despite boost) + P(prefilter reject) + P(demod fail)`, and the
   middle term is invisible today.
2. **No wasted-pop compute concern in reverse:** triage-after-pop is the *right* place
   for a compute filter (it must see samples). The problem is not the position but that
   the gate-2 verdict ignores chain state that the system already possesses (the A6 hot
   table) at the exact moment it matters.
3. `burst_pipeline.c` adds **no additional triage**: `pipeline_head` bails only on
   `adj_n < 280` (:516) and the demod/UW/BCH stages are the decode itself, not gating.
   The old P1.5a `burst_pipeline_triage` (:562-589) has no production caller.

---

## 4. Chain-level arithmetic (why a small per-burst rate matters)

Let `q` = P(gate-2 false-reject | real burst in the marginal band). Openers of decoded
chains are conditioned on having passed (that's how the chain opened), so the *observable*
damage is concentrated on continuations: every rejected continuation ≈ one
`parts_expired[1]` increment and one dead chain — indistinguishable in today's counters
from a stale-drop loss. With `acars_decoded = 0` and `parts_expired[1]` dominant, even
q ≈ 0.1 in the marginal band is material once A6 removes the stale-drop term, because
gate 2 then becomes the **largest remaining unprotected serial term**. Conversely if
q ≈ 0, A6's soak numbers will show it and triage is exonerated. Either way the counter in
§5 is the cheapest possible discriminator.

---

## 5. On-device measurement (confirm/refute before touching thresholds)

**M-A — hot-bin-matched prefilter rejects (the direct test).** A nonzero rate = triage IS
eating expected continuations.

- **Where:** `worker_core1.c`, the reject branch at :1193-1198 (between
  `s_bursts_prefilter_rejected++` and `continue`).
- **What:** reuse the A6 table — it is live precisely during the 700 ms window
  (`HOT_BIN_TTL_MS` = `IDA_REASM_FRAG_GAP_US`, :121) in which a continuation is expected
  on that ±4-bin channel:

```c
// A6-shadow: prefilter rejected a burst on a channel with an OPEN IDA chain —
// candidate continuation killed by triage. Per-gate attribution.
if (hot_bin_match(BURST_PEAK_BIN(&burst), (uint32_t)(esp_timer_get_time() / 1000))) {
    atomic_fetch_add_explicit(&s_pf_rej_hot, 1, memory_order_relaxed);
    if      (!pf.width_ok) atomic_fetch_add_explicit(&s_pf_rej_hot_width, 1, memory_order_relaxed);
    else if (!pf.dur_ok)   atomic_fetch_add_explicit(&s_pf_rej_hot_dur,   1, memory_order_relaxed);
    else                   atomic_fetch_add_explicit(&s_pf_rej_hot_snr,   1, memory_order_relaxed);
    // one throttled log line with the evidence:
    ESP_LOGW(TAG, "PF-REJ-HOT bin=%u snr_tag=%.1f pf_snr=%.1f active=%d",
             (unsigned)BURST_PEAK_BIN(&burst), (double)burst.peak_snr_db,
             (double)pf.channel_snr_db, pf.active_len);
}
```

  Four relaxed `_Atomic uint32_t` at file scope (same idiom as :326-332); surface via
  `worker_core1_get_stats()` → status_logger WORKER1 line + `/diag` JSON (both already
  carry `triage_rej`, status_logger.c:320-328, http_server.c:269). Cost: one
  `hot_bin_table_match` (4-entry scan) per *rejection* — nanoseconds. Requires A6
  Phase 1 to be flashed (the table exists in the working tree already;
  `worker_core1_init` :1264 initialises it).
- **Interpretation:** `s_pf_rej_hot_snr > 0` over a soak = confirmed, and the log line
  gives the (tagger SNR, prefilter SNR) pair for each kill. All-zero over a multi-hour
  flood soak with chains opening (`hot published > 0`) = triage exonerated for
  continuations; look elsewhere (pure stale/evict, or demod).
- **Caveat:** hot-bin match ≠ certainty the reject was the continuation (junk on a hot
  channel within 700 ms also counts). The logged `pf_snr` vs `snr_tag` and `active_len`
  disambiguate: a real continuation shows `snr_tag ≥ 14`, `active` ≈ 2000; junk shows
  short `active` or wide width.

**M-B (free, no A6 dependency) — per-gate reject breakdown for ALL rejects** (three more
counters at the same site, no hot-bin condition) plus a coarse 1 dB histogram of
`pf.channel_snr_db` for rejects with `snr_ok == false`. A pile-up at 12-13.9 dB is the
signature of marginal-real-burst shaving; junk rejects spread broadly.

**M-C (cross-calibration) — accepted-burst estimator offset:** for every ACCEPTED burst,
record `(int)(burst.peak_snr_db - pf.channel_snr_db)` into a small histogram. Mean and
spread quantify the tagger↔prefilter statistic offset — the number the 14 dB transplant
silently assumed was 0 ± 0. Host-side twin: run `burst_prefilter` over the 45-min corpus'
gri-decoded continuations (the review doc's 162 continuations / 78 chains,
`docs/2026-07-15-p4-reassembly-design-review.md`) and count false-rejects — that is the
recall test the fixture never ran.

---

## 6. Recommendations IF implicated (low-risk, in preference order)

1. **Hot-bin triage exemption (the A6-completing fix).** At the reject branch, if
   `hot_bin_match(...)` and the failing gate is SNR (width/duration rejects stand — junk
   protection intact), ESCALATE anyway (treat as accept). One `if`, no threshold moves.
   - *Compute risk:* bounded by design: ≤ 4 channels × ≤ 700 ms TTL windows, refreshed
     only by real merges; worst case a few extra full decodes (~76 ms each) per open
     chain — the exact spend A6 already argued for. Under flood a junk storm inside a hot
     window costs N × 76 ms for its duration; acceptable given windows are rare
     (open-chain concurrency ≈ 0.04, A6 review §2.4).
   - *False-accept risk:* nil downstream (UW/demod/BCH/classify/CRC all still gate);
     worst case wasted compute.
   - *Validate:* device smoke (GOLDEN 62/62 unchanged — corpus never opens hot windows
     under no-flood, so behaviour is identical), then a bench A/B soak with the flag
     pattern from A6 Phase 1; watch `parts_completed[2]` vs `parts_expired[1]`,
     `s_pf_rej_hot_*`, `acars_decoded`, and worker `cap=%` for the compute cost.
2. **Margin the SNR gate relative to the tagger, don't re-test at parity:** set the gate
   to `min(PF_THRESH_DB, runtime_tag_thr) − 2 dB` (2 dB ≈ generous estimator-disagreement
   allowance; make it a named constant derived from the M-C measured offset once known).
   Also fixes the `tag_thr` coupling bug. Global (helps marginal openers too), but costs
   compute across the board under flood — size with M-B's histogram first.
   *Validate:* same as (1) plus `triage_rej` rate and `cap=%` before/after.
3. **Do NOT touch the duration or width gates** — no mechanism (§2), and loosening them
   re-admits the documented impulse flood.
4. **Docs:** correct `burst_prefilter.h`'s "recall-safe — never reject a real burst"
   claim to what the test actually proves (zero false-rejects *on the ALBQ decoded set*),
   and annotate gate 2 as having no gri decode-time counterpart. The project's
   no-test-fitting rule cuts both ways: the constants aren't fitted, but the recall claim
   is currently wider than its evidence.

## 7. Where the evidence is thin (explicit)

- No on-device measurement exists yet: `s_bursts_prefilter_rejected` is aggregate, no
  per-gate split, no SNR-at-rejection record, no hot-bin correlation. Everything in §1-§4
  is code-derived mechanism, not observed rate.
- The claim "continuations are weaker than openers" is unverified (same satellite, same
  TX power; only fading/scintillation over the 90-540 ms gap differentiates them). Gate 2
  endangers *marginal* bursts of either role; the chain math (§4) is what makes
  continuations the visible victims.
- The estimator-offset direction (§2, gate 2 biases) is argued, not measured — M-C decides.
- The 45-min corpus / n=3 retransmit-pair caveats from the A6 spec apply to any yield
  prediction here too.

## 8. Bottom line

The triage prefilter cannot reject a 0x7608 continuation for being short — IDA fragments
are fixed-length 191-symbol frames (`ida_decode.c:114`), and the duration gate passes them
with 2.4× margin. It CAN reject one for being marginal: gate 2 re-applies the 14 dB
detection threshold with a second, never-cross-calibrated estimator, after the PQ pop —
i.e., after A6 has spent its boost. One four-counter patch at `worker_core1.c:1193`
(reusing the A6 hot-bin table) settles empirically whether this is a real loss channel;
if it is, the one-line hot-bin exemption converts A6 from "protects continuations from
stale-drop" into "protects continuations, period."
