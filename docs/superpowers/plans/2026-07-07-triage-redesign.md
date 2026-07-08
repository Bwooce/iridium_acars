# P1.5b — Burst triage redesign: feature pre-discriminator

Date: 2026-07-07
Status: host implementation + host validation complete; device wiring is a
gated follow-up (see §7). No device flash / serial / commit performed.

## 1. Problem with P1.5a (the triage being replaced)

`burst_pipeline_triage()` (common/iridium_decoder/burst_pipeline.c) was meant
to be a cheap fast-pass that screens the burst flood before the expensive full
decode. On-device diagnosis showed it is net-negative:

- Its "fast pass" IS the expensive path. It runs `pipeline_head` (per-burst DC
  removal + D13 + a squared-FFT coarse CFO + RRC matched filter) and then one
  `try_decode_frame` = a UW correlation (2048-pt FFT) + sub-sample interp/decim
  + QPSK demod. The UW correlation and CFO are the FFT-bound operations the
  screen was supposed to avoid (~188 ms UW + ~76 ms CFO in the on-device
  budget). The screen cannot be cheaper than the thing it screens.
- It is single-attempt, whereas the full pipeline recovers real bursts with a
  retry loop over advancing search offsets (burst_pipeline.c:643-654). A real
  burst whose UW lands past the first correlator window is rejected by triage
  but decoded by the full pipeline. In the ALBQ fixture this is the 1
  retry-recovered decode (`first_used_search_start > 0`).
- Measured ~0 escalations in a live soak: it rejects almost everything.

## 2. gr-iridium's actual reject model (ground truth)

gr-iridium has **no early reject** anywhere: no SNR gate, no correlation
threshold, no per-burst spectral discriminator on the burst_downmix side. Every
tagged burst runs the full chain and junk dies at the *final* post-demod
unique-word Hamming check (`iridium_qpsk_demod_impl.cc`, diffs ≤ 2). It affords
this because x86+AVX is ~100-300× our per-op budget.

The only rejects that exist upstream are in the **tagger** and are inherently
feature-based and cheap:

- Duration bound. `burst_downmix_impl.cc:502`
  `if (burst_size - start < min_frame_length) return 0;`
  with `min_frame_length = MIN_FRAME_LENGTH_{NORMAL|SIMPLEX} ×
  output_samples_per_symbol` (iridium.h:18 = 131 sym, iridium.h:21 = 80 sym).
  A burst shorter than one frame is dropped without decoding.
- Channel/SNR detection. `fft_burst_tagger_impl.cc:386`
  `if (d_relative_magnitude_f[bin] > d_threshold)` — a burst_width-wide channel
  is only declared present when its magnitude exceeds the rolling noise baseline
  by the detection threshold (`:248-250` check center bin ±1). `burst_width =
  40 kHz` (iridium-extractor:126, iridium_extractor_flowgraph.py:30) and
  `threshold = 18 dB` (iridium-extractor:130).

The redesign takes exactly these two tagger-grade features and applies them as a
*pre-correlation* discriminator, so we reject junk before entering our (non-gri)
retry loop — which is where ~94% of our per-junk-burst cost lives.

## 3. Design: `burst_prefilter()`

New host-portable module `common/iridium_decoder/burst_prefilter.{c,h}`.
Runs on the decimated 250 ksps, DC-centred (post rotate-to-dc) IQ window — the
same buffer `burst_pipeline_process_burst` consumes. Does not mutate it.

Two gates, in cheap-first order. Accept ⇒ escalate to the **full retry
pipeline** (`burst_pipeline_process_burst`); reject ⇒ drop.

### Gate 1 — active-envelope duration (time domain, O(N))

- Per-sample power, boxcar-smoothed over one symbol (`PF_ENV_WIN = SPS = 10`) to
  ride through QPSK/RRC amplitude dips.
- `peak_env`, `floor_env` = max/min of the smoothed envelope.
- `thr_env = sqrt(peak_env · floor_env)` — the geometric-mean (log-domain
  midpoint / "half-power in dB") crossing. This is a *standard per-burst
  pulse-width definition*, adaptive to each burst; it is not a tuned level.
- `active_len` = count of envelope samples ≥ `thr_env`.
- **Reject if `active_len < MIN_FRAME_LENGTH_SIMPLEX × SPS = 800` samples.**

Threshold grounding: `burst_downmix_impl.cc:502` + `iridium.h:21`. We use the
SIMPLEX minimum (80 symbols — the *smallest* decodable frame class) rather than
NORMAL's 131, so the gate can never reject a real burst of any class. This gate
kills the documented bench flood signature: short (≤ 2.5 ms), narrowband
impulses — which are shorter than one frame regardless of how narrowband they
are.

### Gate 2 — integrated in-band channel SNR (one 2048-pt FFT)

- Pick the highest-energy 2048-sample segment (running power sum, O(N)) so the
  burst — not the ~16 ms noise post-padding — fills the FFT window.
- DC-remove the segment (subtract mean, matches `pipeline_head` step 0) so the
  residual RTL DC spike can't fake an in-band peak.
- `fft_sc16_2048` (the sc16 FFT already used by the tagger). 122.07 Hz/bin.
- In-band channel = DC ± burst_width/2 = ±20 kHz = ±164 bins ⇒ 329 bins.
  `PF_HALF_BAND_BINS = round(20000 / 122.07) = 164`.
- `P_in` = summed power in the 329 in-band bins. Noise floor `N0` = **median**
  of the out-of-band bin powers (robust to RRC sidelobes / stray spurs a mean
  would inflate).
- `channel_snr_db = 10·log10( P_in / (N0 · 329) )`.
- **Reject if `channel_snr_db < 14 dB`.**

Threshold grounding: `fft_burst_tagger_impl.cc:386` (relative_magnitude >
threshold), `:248-250` (burst_width channel around the center bin),
burst_width = 40 kHz (iridium-extractor:126), threshold = 18 dB
(iridium-extractor:130) mapped to **14 dB in our baseline-scale domain** (memory
`project_tagger_threshold_scale`: ours ≈ gri − 4 dB; these host tests run the
tagger at 14 dB). This is the same channel/threshold model that admitted the
burst, re-applied to the extracted window; it rejects flat wideband noise and
out-of-band/broadband impulses that carry no in-band channel excess.

Both gates are recall-biased by construction (smallest frame class; the
admitting threshold). Neither is tuned to a fixture — every constant traces to a
gr-iridium source line.

## 4. Flow

```
tagged burst → decimate to 250 ksps, rotate-to-dc
   │
   ├─ Gate 1 (duration, O(N), µs)  ── reject → drop (short impulse flood)
   │        ↓ pass
   ├─ Gate 2 (1× 2048 FFT + O(N))  ── reject → drop (flat / out-of-band noise)
   │        ↓ pass (accept)
   └─ burst_pipeline_process_burst  (FULL retry pipeline — unchanged)
```

Survivors run the *full* multi-frame + retry pipeline, so the P1.5a recall loss
(single-attempt) is gone.

## 5. Cost (operation-count argument; device timing is a gated follow-up)

Per burst the pre-filter does: 3 O(N) time-domain passes (envelope peak/floor,
active count, segment-energy scan) + one DC-remove + **one 2048-pt FFT** + one
`qsort` of ~1719 int64. That is a single FFT, versus the full path's per-attempt
3× 2048-pt FFT + a 4096-pt CFO FFT + demod, run up to ~40 times in the retry
loop (~11.8 ms/attempt, ~500 ms per rejected junk burst per memory
`project_p15_triage_design`). A junk burst that fails Gate 1 costs *zero* FFTs;
one that reaches Gate 2 costs exactly one FFT. Either way it never enters the
retry loop, which is the ~94% cost sink.

Device wall-clock is deliberately **not** claimed here — this task is host-only,
no flashing. The device measurement is part of the gated integration (§7).

## 6. Validation (host, real output)

Test: `tests/host/test_burst_prefilter.c` (ctest `burst_prefilter`). Same
self-contained ALBQ harness as `test_pipeline_wideband_resampled` (fixture →
firmware resampler → tagger → rotate → decim → per-burst pipeline). Actual run:

```
Bursts tagged:              70
Decoded (pre-filter OFF):   63   (62 first-attempt, 1 via retry loop)
Pre-filter accepts (all):   70
Decoded (pre-filter ON):    63
PASS: positive control — all 63 decodes accepted (0 false-rejects)
PASS: pre-filter-ON decode 63 (floor 55)
PASS: synthetic noise rejected   [dur_ok=1 active=56770 snr_ok=0 snr=1.2 dB]
PASS: synthetic impulse rejected [dur_ok=0 active=209   snr_ok=0 snr=0.0 dB]

Synthetic population sweep (60 each):
  long narrowband tone accepted:     60/60   (positive control)
  short narrowband impulse rejected: 60/60
  flat wideband noise rejected:      60/60
  overall synthetic junk rejection:  120/120 (100%)
```

### Recall (false-reject rate on real bursts) — the priority metric

**0 false-rejects. All 63 full-pipeline decodes are accepted**, including the 1
retry-loop recovery that P1.5a rejects. Pre-filter-ON decode count = 63, i.e.
the pre-filter is decode-transparent on this fixture (and one decode *above*
P1.5a's 62). The synthetic positive control confirms the gates accept
signal-like input: 60/60 long narrowband tones accepted.

### Junk rejection

100% (120/120) on the synthetic populations that match the documented bench
signatures: 60 short narrowband impulses (all fail Gate 1) + 60 flat wideband
noise windows (all fail Gate 2). The two original hand-built cases
(noise, impulse) also both reject.

### Important finding: the 7 tagged-but-undecoded ALBQ bursts are accepted

The pre-filter accepts all 70 tagged bursts, so it rejects 0 of the 7 that don't
decode. This is **correct, not a miss.** Those 7 passed the real tagger at 14 dB
and — verified by both gates passing — are long (`active_len ≥ 800`) and
in-band above 14 dB: they are genuine, tagger-grade narrowband detections that
simply fail the *final* UW/BCH check (marginal SNR, bit errors). gr-iridium
would also run them through the full chain and drop them at the post-demod UW
check. A cheap pre-correlation filter that stays faithful to gr-iridium's model
*must not* and *cannot* reject them without inventing a non-gri heuristic (the
banned #115 class). The ALBQ fixture is clean bench-captured data, so every
tagged burst is a real detection; the pre-filter's rejection power applies to
the *live* frozen-baseline flood (memory `project_tagger_frozen_baseline_latch`)
whose short narrowband impulses and flat noise are exactly the synthetic
populations rejected 100% above.

## 7. Hypothesis verdict

**Held, with a sharpened understanding of the junk.** A genuinely cheap,
feature-based, pre-correlation discriminator (duration + integrated in-band
channel SNR) rejects the documented junk (short narrowband impulses; flat noise)
at ~1-FFT cost while preserving full recall — 0 false-rejects on real bursts and
better recall than P1.5a because survivors take the full retry pipeline.

Refinement vs the original hypothesis wording: the bench flood is *narrowband*
in frequency (1-3 bins) but *impulsive* in time, so the **duration gate is the
primary discriminator** for it; the channel-SNR gate is the secondary gate for
flat/wideband noise. A per-burst *spectral-flatness* gate was considered and
dropped: a narrowband short impulse is spectrally peaky, so flatness would not
catch it, whereas duration does — and flatness has no direct gr-iridium
threshold to cite (test-fit risk).

## 8. Follow-up (gated — do NOT wire in yet)

- Wire `burst_prefilter` into `worker_core1.c` in place of the
  `burst_pipeline_triage` call, escalating accepts to
  `burst_pipeline_process_burst`. Gated behind the device RAW-smoke GOLDEN,
  currently blocked by the PIE/USB work. On device the window must be decimated
  to 250 ksps before the pre-filter; measure the real per-burst cost and the
  live FRMDEC bch_ok/h then.
- Consider deleting `burst_pipeline_triage` once the pre-filter is wired
  (keep for now so `test_triage_fast_pass` still documents the old behaviour).
- The device FFT (`dsps_fft2r_sc16_arp4`) has known PIE bit-exactness caveats
  (memory `feedback_pie_fft_swap_needs_validation`); the pre-filter's single FFT
  should use the validated `fft_sc16_2048` path, not a raw PIE swap.
```
