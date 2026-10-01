# POA 4-channel decoder optimization — measure-first plan

Date: 2026-08-03
Branch context: `feat/poa-onband` (task #36). Relates to
`docs/2026-08-01-poa-onband-plan.md`, memory `project_poa_onband_implementation`.

Status: PLANNING ONLY. No firmware/build/sdkconfig changes are proposed to be
made as part of writing this doc. This document is the deliverable.

---

## 0. TL;DR / key recommendations

1. **The dominant decoder cost is soft-float double, confirmed as fact (not
   inference).** `poa_decoder.c.obj` (current `-O2` build) pulls in undefined
   `__adddf3 __subdf3 __muldf3 __divdf3 __gedf2 __extendsfdf2 __fixdfsi
   __floatsidf` and a double-complex `cexp`. The ESP32-P4 is RV32IMAFC —
   **single-precision FPU only, no `D` extension** — so every `double` op in
   `demod_msk()` is a library call, executed **per 12.5 kHz envelope sample per
   channel**, plus one soft-double `cexp` per sample. This is the hog.

2. **The highest-leverage fix is NOT PIE.** It is converting the decoder's
   per-sample path to **single precision throughout**, with the NCO replaced by
   a **fixed-point phase accumulator + sin/cos table** (no `cexp`, no `sinf`
   call). Arithmetic below projects the 4-channel total from ~206 to ~54
   cyc/complex-sample against a 144 budget — i.e. **4ch fits with margin and no
   PIE at all**. PIE-vectorizing the matched filter is explicitly *not* worth
   doing (§6).

3. **The 206/164 cyc numbers were measured while overloaded and are suspect.**
   They may conflate (a) a possible build difference between the 2ch and 4ch
   runs — `po_chans` has no runtime setter, so the two counts likely came from
   two different binaries — and (b) higher-priority `usb_pump` preemption landing
   inside the `esp_timer` window that defines `dsp_pct`. **Phase 1 re-measures
   cleanly on the contention-free POA smoke bench before any code changes.**

4. **Separate the two gating questions** (this is the cleanest framing):
   - The **decoder single-precision rewrite** is worth doing on its own merit at
     2ch/3ch for Core-0 headroom (HTTP reachability, SD, cross-core feed
     stability). Low-risk local change. Do NOT gate it on "is 4ch justified".
   - **Expanding the channel set to 4** is what gets gated on the 2ch soak
     occupancy data (all saved POA IQ decodes exactly one frame, on 131.550;
     ch1-3 have never produced a frame). See §7.

---

## 1. How the decoder is driven (data flow)

```
class_driver dsp_feed_task (Core 0, prio = pump_prio-1)
  -> dsp_processor_feed()                       [dsp_processor.c:475]
       if (p->poa_fe) poa_frontend_feed(...)    [dsp_processor.c:488-490]
  -> poa_frontend_feed(fe, iq, nsamp)           [poa_frontend.c:209]
       per whole rtlMult(=200)-sample block:
         consume_block()                        [poa_frontend.c:188]
           deinterleave int16 I/Q -> xr/xi (internal DRAM)
           per channel: poa_mix_q15_arp4(...)   [PIE, poa_mix_arp4.S]   <-- MIX (solved)
             envelope out[n][m] = sqrtf(Dre^2+Dim^2)
         flush_out() unconditionally at feed end [poa_frontend.c:239]
  -> poa_decoder_feed(dec, chn, audio, len)     [poa_decoder.c:325]     <-- DECODER (this plan)
       demod_msk(d, ch, len)                    [poa_decoder.c:252]
         putbit() -> decode_acars() state machine
```

Two rate domains:
- **Input / mix domain: 2.5 MSPS complex.** Budget = 360 MHz / 2.5 MSPS =
  **144 cyc/complex-sample total, all stages, all channels.**
- **Envelope / decoder domain: `POA_INTRATE` = 12 500 Hz** (rtlMult = fs/INTRATE
  = 200). The decoder runs 1 envelope sample per 200 input samples per channel.
  So each decoder envelope-sample cycle is amortized over 200 input samples:
  `cyc_per_input_sample = cyc_per_envelope_sample / 200`.

Note (fixed cost, minor): `flush_out()` is called unconditionally at every
`poa_frontend_feed` (line 239), and typical feeds are ~8000 complex samples =>
~40 envelope samples. So the OUTBUF=1024 batching never engages; the decoder is
entered `nch` times per feed with `len ≈ 40`. Per-call fixed overhead is tiny
(a few pointer stores) but worth noting for Phase 1 attribution.

---

## 2. Where the decoder spends time (per-sample characterization)

### 2.1 `demod_msk()` — the hot per-envelope-sample loop  [poa_decoder.c:252-298]

This loop runs **once per envelope sample (12.5 kHz) per channel**. Every line
below that touches a `double` is a soft-float library call on the P4:

| line | operation | type | cost class |
|---|---|---|---|
| 258 | `s = 1800.0/INTRATE*2*M_PI + ch->MskDf` | double add | soft `__adddf3` |
| 259-260 | `p += s; if (p>=2pi) p-=2pi` | double add/cmp | soft `__adddf3 __gedf2` |
| 262-263 | `in * cexp(-p*I)`, `in` promoted to double | **double-complex `cexp`** + `__extendsfdf2` + complex mul | **DOMINANT — libm transcendental in double** |
| 264, 274 | `idx = (idx+1) % FLEN`, `(j+idx) % FLEN` | int mod by 11 (non-pow2) | minor (`rem`) |
| 266-267 | `MskClk += s; if (MskClk >= 3*M_PI/2 - s/2)` | double/float mix + cmp | soft double |
| 278 | `MskLvlSum += (double)lvl*lvl/4.0` | double mul/add/div | soft double |
| 281-289 | `dphi` selection | double | soft double |
| 293 | `MskDf = PLLC*MskDf + (1-PLLC)*PLLG*dphi` | double mul/add | soft double |

**Per-bit (not per-sample) work**, inside the `if (MskClk >= ...)` branch which
fires ~ once per `INTRATE/2400 ≈ 5.2` envelope samples:
- 273-274: `for j in [0,FLEN=11): v += s_h[o] * ch->inb[(j+idx)%FLEN]` — an
  **11-tap complex×real float MAC**. Single-precision floats (`s_h` is `float`,
  `inb` is `float complex`). ~11 float MACs + address arithmetic.
- 276-277: `cabsf(v)`, `v /= lvl+1e-8f` — one sqrt + complex divide (float).

`s_h[FLENO=133]` matched filter is built once (float) in
`poa_decoder_create()` [309-315]; not on the hot path.

### 2.2 `decode_acars()` / `putbit()` — L2 state machine  [poa_decoder.c:175-249]

Runs once per **bit** (2400 bit/s per channel), not per sample. Pure integer
byte/state bookkeeping (`numbits[]`, shifts, compares). Negligible.

### 2.3 `process_block()` + `fixprerr`/`fixdberr` + CRC  [poa_decoder.c:71-165]

Runs once per **completed ACARS block** (rare — seconds apart, only on a live
frame). `update_crc` table-driven; `fixprerr` recursion is depth ≤ MAXPERR=3.
Not a throughput concern. One `log10f` per block at line 229 (fine, per §
`feedback_prefer_int16_over_float` rule 6). **Leave as-is.**

### 2.4 Cost model (order-of-magnitude, to be replaced by Phase 1 measurement)

The soft-double `cexp` + ~6-8 soft-double add/mul/div dominate. Rough
per-envelope-sample cost is in the low thousands of cycles/channel; over 200
input samples that is the reported ~41 cyc/input-sample/channel scalar. The
**per-bit** 11-tap MAC is ~11 float MACs / 5.2 samples ≈ 2 float-MAC/sample,
i.e. ~0.3 cyc/input-sample/channel — two orders of magnitude below the NCO.
**Conclusion: kill the doubles; the MAC is noise.**

---

## 3. Measured baseline and why it is suspect

| config | dsp% | drops | rate | derived cyc/complex-sample |
|---|---|---|---|---|
| 2ch | ~28% | 0 | 4.77 MB/s (full stream) | ~40 total (~20/ch) |
| 4ch | ~99% | ~40% rb_full | 3.25 MB/s (over budget) | ~206 "measured"; drop-corrected ~238 (~59/ch) |

Linear extrapolation of the clean 2ch point predicts 4ch ≈ 80 cyc/sample (≈22%
dsp). Measured is ~3× higher per channel. That gap is too large for D-cache or
PSRAM read bandwidth (the mix working set — 4×3 weight tables of ~200 int16 +
xr/xi + envelope out — is a few KB, fits L1). The likely real causes, in
priority order for Phase 1:

1. **Build drift between the 2ch and 4ch runs.** `po_chans` has **no runtime
   setter** (http_server.c / serial_cmd.c never write the NVS key;
   app_config.c only reads it — see `project_poa_onband_implementation`
   "REMAINING"). So a 2ch run required editing `DEFAULT_POA_CHANS` + reflashing.
   **The 2ch and 4ch numbers plausibly came from two different binaries.** If so,
   the "nonlinearity" may be partly or wholly an artifact. *Resolving this is the
   single highest-value item in Phase 1.*
2. **Preemption contamination of `dsp_pct`.** `dsp_pct` derives from
   `s_feed_dsp_total_time_us`, an `esp_timer_get_time()` **wall-clock** delta
   around `dsp_processor_feed()` (class_driver.c:1008-1022). `usb_pump` is a
   *higher-priority task on the same core* (Core 0). Under 4ch saturation the ring
   backs up and usb_pump's drain/resubmit work is scheduled **inside** the timed
   feed window, inflating the attributed time. NB: swapping to the CPU cycle
   counter (`dsp_get_cpu_cycle_count`) does **not** fix this — `mcycle` also
   advances while preempted. The only clean fix is to measure where nothing else
   competes: the smoke bench (§4).

The build is confirmed `-O2` (`CONFIG_COMPILER_OPTIMIZATION_PERF=y`), so the
numbers are NOT an `-Og` artifact (ruling out the `feedback_o2_for_optimization`
trap). Phase 1 must nonetheless state the optimization level next to every
number it reports.

---

## 4. Phase 1 — clean, contention-free re-measurement (GATE for everything else)

**Goal:** obtain the *true unthrottled* per-channel, per-stage cost, and resolve
the 2ch-vs-4ch nonlinearity, BEFORE deciding whether any decoder work is needed.

**Method: instrument the existing POA smoke bench.** `smoke_test_run_poa()`
(smoke_test.c:821-866) already runs the *real* `poa_frontend_create` (PIE Q15
mix) + `poa_decoder` on an embedded IQ slice with **no USB, no ingest, no ring,
no drops, and no competing task**. This is the clean bench that removes both
suspect factors from §3 at once.

Concrete changes (measurement-only, on a throwaway/local branch — not for merge):
- In `smoke_test.c`, wrap the mix and the decoder separately with
  `dsp_get_cpu_cycle_count()` deltas. Because the mix and decoder are called from
  inside `poa_frontend_feed`/`consume_block`, the cleanest split is a **compile-
  time instrumentation hook**: add optional `#if CONFIG_SMOKE_TEST_POA` cycle
  accumulators around `poa_mix_q15_arp4()` (poa_frontend.c:200) and around
  `demod_msk()` (poa_decoder.c:330), summed into file-scope counters, divided by
  the total complex-sample count fed. Report `cyc/complex-sample` for MIX and for
  DECODER, per channel, at the end of the run.
- **Keep the int8->int16 conversion loop (smoke_test.c:849) OUTSIDE the timed
  region** — it is fixture marshalling, not decoder cost.
- Run the bench at **nch = 1, 2, 3, 4** (the create call at smoke_test.c:826
  takes an explicit channel count — vary it). Same binary, only the count
  changes. This directly answers: *is per-channel decoder cost constant?* If
  cyc/sample/ch is flat across 1..4, the "nonlinearity" was build drift +
  preemption (§3 causes 1-2) and does not exist in the compute itself.

**Deliverables of Phase 1 (write into this doc's addendum):**
- MIX cyc/complex-sample/ch (expect ~10, PIE, already solved).
- DECODER cyc/complex-sample/ch at -O2 (this is the number that decides §5).
- Whether the cost is linear in nch (resolves the nonlinearity).
- A drop-corrected read of the *live* 4ch number for cross-check
  (`0.99 × 360e6 / (0.60 × 2.5e6)` etc.), explicitly labelled as preemption-
  contaminated and therefore an upper bound.

**Decision gate:** if the clean decoder cost + mix already fits 144 at the target
channel count, **stop — no decoder optimization needed.** Otherwise proceed to
Phase 2. (Given §2, the decoder almost certainly does NOT fit 4ch as-is, but it
may already fit 3ch cleanly — Phase 1 tells us which channel count is the real
ceiling.)

**Risk:** low. Measurement-only, on the smoke path, off by default. No production
code path changes.

---

## 5. Phase 2 — single-precision decoder rewrite (the real fix)

**This is the whole optimization.** Convert `demod_msk()`'s per-sample path from
soft-double to single precision, and replace the NCO.

### 5.1 The change
- Change the per-sample state to single precision / fixed point:
  `MskPhi, MskDf, MskLvlSum` (poa_decoder.c:39-40) `double -> float`;
  `MskClk` already `float`. Downstream `s`, `p`, `dphi` (258, 281-289) `-> float`.
- **Replace `cexp(-p*I)` (line 263) with a fixed-point phase-accumulator NCO:**
  a Q32 phase accumulator advanced by a precomputed Q32 increment per sample
  (`s`); use the top ~10 bits to index a **1024-entry cos/sin table** (built once
  in `poa_decoder_create`, single-precision float, alongside `s_h`). This removes
  the libm transcendental *and* the double add/compare of the phase wrap
  (the accumulator wraps for free at 2^32). No `sinf`/`cosf` per-sample call.
- Keep `s_h` / `inb` / the 11-tap MAC / `cabsf` in single-precision float
  (they already are). No change needed there.
- PLL update (line 293) in single precision.
- Optional cleanup (not required for the win): replace `% FLEN` (264, 274) with a
  compare-and-wrap. Cheap; do only if Phase 1 attribution shows it matters.

### 5.2 Why this is the right altitude (arithmetic)
Per envelope sample the soft-double `cexp` + ~8 soft-double ops (~thousands of
cyc) collapse to a fixed-point add + table lookup + a handful of `fmul.s/fadd.s`
(tens of cyc). Decoder projected from ~41 to ~1-3 cyc/input-sample/channel.

At 4 channels: `mix ~42 + decoder ~12 ≈ 54 cyc/complex-sample` vs **144 budget**
— large margin, **no PIE**. Even generous error bars leave 3ch and 4ch
comfortably inside budget.

### 5.3 Validation gates (MANDATORY — the numerics change, unlike the Q15 mix)
The Q15 mix was bit-identical-by-construction, so its weak smoke gate was OK. A
double->float NCO is **not** bit-identical. Therefore:

- **Host bit-exact/behavioral gate:** `tests/host/test_poa_decoder` and
  `test_poa_frontend` must still decode JQ0404. Because bits may differ in the
  last ulp, assert not just that the text appears but that the block's **`err`
  and `crc_fixed` are unchanged** from the current (pre-change) baseline. Capture
  the current baseline values first (they should be `err==0, crc_fixed==false`
  for a clean golden — confirm before pinning).
- **Golden smoke gate hole to fix:** the current on-device gate is
  `strstr(s,"JQ0404")||strstr(s,"VH-VGD")` with `s_poa_smk_hit>=1`
  (smoke_test.c:818,857). **This passes even on a degraded demod that only
  decoded because `fixprerr`/`fixdberr` repaired it** — the exact circular-golden
  failure mode from `project_heap_position_decode_bug` (a gate calibrated to a
  corrupted baseline hid the RTCRAM bug for months). **Tighten the POA smoke gate
  to additionally require `b->err == 0 && b->crc_fixed == false`**, after first
  confirming from a current smoke log that today's golden actually achieves that
  (pin the real baseline, not a repaired one). This tightening is itself a small,
  worthwhile change independent of the rewrite.
- **Cross-validate vs oracle** (`~/dev/vdl2-tools/acarsdec`) per
  `project_cross_validate_against_gr_iridium` if any doubt about numeric drift.
- **On-device smoke MANDATORY after this DSP-path commit**
  (`feedback_device_smoke_mandatory_after_dsp`): `scripts/smoke_run.sh poa` must
  emit `SMOKE_PASS` with the tightened gate. Pre-push trailer.

**Risk:** medium. MSK PLL is sensitive to phase-tracking precision; single float
has ~24-bit mantissa which is ample for a 12.5 kHz NCO over a ~1 s frame, but the
tightened golden gate is what proves it. If the tightened gate regresses, the
Q32 phase accumulator (which does NOT accumulate float rounding — it is exact
integer wrap) is the safer NCO than a float phase variable; prefer it from the
start for exactly this reason.

**PIE footguns:** none — Phase 2 introduces no PIE. (The `cos/sin` table is
scalar-indexed, not vector-loaded.)

---

## 6. Phase 3 — PIE the matched filter: EXPLICITLY NOT RECOMMENDED

Stated with arithmetic so nobody re-opens it: the FLEN=11 complex×real MAC
(poa_decoder.c:273-274) runs **per bit (2400 bit/s), not per sample**. Cost ≈
2400 × 11 × (few cyc) per channel ≈ 0.3 cyc per input complex-sample per channel
— two orders of magnitude below the NCO, and already single-precision. PIE cannot
meaningfully accelerate an 11-tap (padded-to-16) dot product that runs at bit
rate; the vld/setup overhead would likely *exceed* the scalar MAC. After Phase 2
the decoder is ~12 cyc/sample at 4ch against a 144 budget — there is nothing to
optimize. **Do not do Phase 3.** If Phase 1/2 somehow disagree, re-measure before
touching this.

---

## 7. Over-engineering flags and cheaper alternatives

- **4 channels is not data-justified.** All saved POA IQ (~11.5 min analyzed)
  decodes exactly one frame, on 131.550; ch1-3 have produced zero. The 6 h
  coverage-run IQ was not saved. **Gate the channel-set expansion to 4 on the
  ongoing 2ch soak occupancy data** — not on this optimization. (The optimization
  itself is justified independently at 2ch/3ch for Core-0 headroom; see §0.4.)

- **Per-channel envelope squelch — keep as a CONTINGENCY, not a phase.** Since
  3/4 channels are ~always idle, gating the expensive decoder on a near-free
  per-block envelope-energy threshold (the mix already computes `|D|`) would cut
  decoder cost ~4× at 4ch. BUT: the MSK PLL must be locked *before* the two SYN
  bytes arrive, so un-squelching on envelope rise means replaying a short
  envelope history to re-acquire — meaningfully more complex than the numeric
  substitution, and it carries missed-opener acquisition risk. After Phase 2 the
  decoder is ~12 cyc/sample at 4ch, so squelch buys nothing we need. **Do not
  build it unless post-Phase-2 measurement unexpectedly still shows a problem.**

- **Do NOT** add a parallel "optimized" path alongside the float reference in
  production (`feedback_prefer_int16_over_float` rule 5): commit to the
  single-precision path once the golden gate proves it; keep the double float
  only as a host reference if needed for the bit-exact diff, then delete.

- **Do NOT** move the decoder to a second core as the first lever. It is
  available if Phase 1 shocks us, but it adds a cross-core queue + the
  single-PIE-owner-per-core hazards; the single-precision rewrite is far cheaper
  and keeps the whole chain on the existing Core-0 `dsp_feed` task.

---

## 8. PIE ownership note (as requested — verify + state)

The POA channelizer's PIE mix (`poa_mix_q15_arp4`) already runs on the **Core-0
`dsp_feed` task**, invoked synchronously from `consume_block` (poa_frontend.c:200).
The decoder runs in the **same task** (called from the same `poa_frontend_feed`
=> `flush_out` => `poa_decoder_feed`, all synchronous, same stack). Therefore, if
Phase 2 ever *did* add PIE to the decoder (it should not — §6), it would be the
**same PIE owner on the same core** as the mix — no second active PIE owner is
introduced, so the trap-storm / lazy-save-deadlock hazard of
`project_p4_one_pie_owner_per_core` / `project_pie_save_deadlock_smoke` does not
arise. Any decoder PIE buffer would still need the `project_heap_position_decode_bug`
early-alloc dance (`heap_caps_aligned_alloc(16, MALLOC_CAP_INTERNAL)` +
`esp_ptr_in_dram` guard + zero-pad + read-ahead). **Since Phase 2 uses no PIE,
this is documented for completeness only.**

---

## 9. Phase summary

| phase | what | validation gate | risk | PIE? |
|---|---|---|---|---|
| 1 | Instrument POA smoke bench; measure MIX vs DECODER cyc/sample/ch at nch=1..4 on the contention-free slice; resolve 2ch-vs-4ch nonlinearity (build drift + preemption); state -O2. | Bench prints per-stage cyc/sample; decision gate on whether 144 budget already met at target nch. | low (measurement only) | no |
| 2 | Single-precision decoder rewrite: `double->float` per-sample state; replace `cexp` NCO with Q32 phase accumulator + 1024-entry cos/sin LUT. | Host `test_poa_decoder`/`test_poa_frontend` decode JQ0404 with **unchanged `err`/`crc_fixed`**; TIGHTEN on-device POA smoke gate to require `err==0 && !crc_fixed`; `scripts/smoke_run.sh poa` => SMOKE_PASS; device-smoke trailer. | medium (PLL precision — mitigated by exact-integer Q32 NCO + tightened gate) | no |
| 3 | PIE the 11-tap matched filter. | — | — | **NOT RECOMMENDED** (runs at bit rate; ~0.3 cyc/sample; §6) |

**Bottom line:** Phase 1 is the real gate. The projected fix (Phase 2, single
precision, no PIE) closes 4ch to ~54/144 cyc/sample with large margin, is a
low-risk local change worth doing at 2ch/3ch regardless, and the only genuinely
new requirement is a **tightened golden smoke gate** so the numeric change is
validated on an unrepaired decode. Reach for PIE / a second core / squelch only
if Phase 1 contradicts the arithmetic — it should not.
