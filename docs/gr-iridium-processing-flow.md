# gr-iridium processing flow (reference notes)

These are working notes captured while wiring up path-C of our host
pipeline and chasing stage-by-stage divergence from gr-iridium. The
goal here is not to duplicate the upstream code, but to capture the
sequence, the per-stage data layout, and (most importantly) the
non-obvious invariants that bit us when we tried to mirror it.

All line numbers reference `gr-iridium/lib/burst_downmix_impl.cc` and
`gr-iridium/lib/fft_burst_tagger_impl.cc` as of the version checked
into our submodule.

## Numeric constants reference

Every numeric constant in our port should match gr-iridium's exactly.
Below is the inventory we built up while reaching 97% path-C parity.
"gri source" gives the formula or literal in upstream; "ours" is the
matching constant in `common/iridium_decoder/uw_correlator.c` or
`tests/scripts/direct_if_dump.py`.

| Constant | gri source | gri value | Our name | Notes |
|---|---|---|---|---|
| Input sample rate | `--rate` arg | 2.5 MSPS | `FS_RAW` | divisible-by-100k requirement |
| Output sample rate (burst rate) | hardcoded | 250 ksps | – | sps=10 at 25 ksym/s |
| FFT-burst-tagger FFT size | `2**round(log2(fs/1000))` | 2048 | – | ~1 ms windows |
| burst_pre_len | `2 * fft_size` | 4096 raw | `PRE_SAMPLES_RAW` | ~1.64 ms |
| burst_post_len | `fs * 16e-3` | 40000 raw | `POST_SAMPLES_RAW` | 16 ms |
| Input FIR filter | `firdes.low_pass_2(1, 2.5e6, 20e3, 40e3, 40dB)` | ~279 taps Kaiser β≈5.5 | scipy `kaiserord(40, 2*20e3/fs)` + `firwin(..., window=('kaiser', β))` | **Critical** — scipy `resample_poly`'s default short window leaks adjacent bursts, breaks CFO |
| D13 LP filter | `firdes.low_pass_2(1, 250e3, 2.5e3, 5e3, 60dB)` | 183 taps Kaiser β=5.65 | `START_LP_NTAPS=183`, `START_LP_KAISER_BETA=5.65` | Smooths the |x|² envelope |
| D13 threshold fraction | `0.28 * max(filtered_mag)` | 0.28 | `START_THRESHOLD_FRAC=0.28f` | – |
| D13 pre-start back-off | `0.1e-3 * 250e3` | 25 samples | `START_PRE_SAMPLES=25` | Keep ahead of preamble |
| D13 search depth | `0.007 * burst_sample_rate` | 1750 samples | `SEARCH_DEPTH_SAMPLES=1750` in burst_pipeline.c | 7 ms |
| Squared-FFT CFO input | `pow(2, int(log(sps*(PREAMBLE_SHORT+10))/log(2)))` | 256 samples | `CFO_INPUT_N=256` | preamble + 10 UW syms rounded DOWN |
| Squared-FFT CFO zero-pad factor | `d_fft_over_size_facor` | 16 | – | input × 16 = 4096-pt FFT |
| Squared-FFT CFO FFT size | `cfo_input × 16` | 4096 | `CFO_FFT_N=4096` | – |
| Squared-FFT CFO window | `WIN_BLACKMAN` | 3-term Blackman | `s_cfo_window_full_f` | Standard formula |
| RRC pulse β | flowgraph constant | 0.4 | `RRC_BETA=0.4f` | – |
| RRC tap count | `rcosfilter(51, ...)` | 51 | `RRC_NTAPS=51` | – |
| Sync reference length | `sync_word*sps - (sps-1)` after `erase()` | **271** samples | `SYNC_RRC_LEN = SYNC_LENGTH*sps - (sps-1) = 271` | NOT 280; the trailing 9 zeros are erased before RC filter |
| d_sync_search_len | `(PREAMBLE_LONG + UW_LEN + 8) * sps` | 840 | – | Uses LONG (=64), not SHORT |
| Matched-filter FFT size | `next_pow2(sync_search + sync_word - 1)` | **2048** | `CORR_FFT_N=2048` | next_pow2(840+280-1)=2048; NOT 1024 |
| UW correlator peak format | `corr_offset = argmax` | raw IFFT index | downstream uses `peak_k = idx - (L-1)` | gri uses `corr_offset` directly and computes `preamble_offset = corr_offset - L + 1` |
| Max frame length (normal) | `MAX_FRAME_LENGTH_NORMAL * sps` | 1910 | `MAX_FRAME_LEN_NORMAL_10SPS = 191*10` | 191 = MAX_FRAME_LENGTH_NORMAL |
| Max frame length (simplex) | `MAX_FRAME_LENGTH_SIMPLEX * sps` | 4440 | not yet wired into burst_pipeline | freq > SIMPLEX_FREQUENCY_MIN (1626 MHz) |
| SIMPLEX freq threshold | constant | 1.626 GHz | `SIMPLEX_FREQUENCY_MIN` | DL simplex/paging band cutoff |
| Pre-rotation order | rotate FULL frame, then trim by uw_start (lines 692-711) | – | mirrored in burst_pipeline.c step 6a/6b | We add sub-sample interp as 6b (gri leaves this for downstream Python QPSK demod) |

## Block diagram

```
                          raw IF cf32 stream @ input_sample_rate (2.5 MSPS)
                                          │
                                          ▼
            ┌─────────────────────────────────────────────────────┐
            │ fft_burst_tagger                                     │
            │   N-point FFT (N = 2048 for fs=2.5M; ~1 ms window)   │
            │   per-bin power vs rolling noise floor               │
            │   contiguous-bin burst tracker w/ hysteresis         │
            │   emits stream tag "new_burst" with metadata         │
            │   stream output delayed by burst_pre_len samples     │
            └─────────────────────────────────────────────────────┘
                                          │
                                          ▼
            ┌─────────────────────────────────────────────────────┐
            │ tagged_burst_to_pdu                                  │
            │   collects samples from tag.offset onward            │
            │   first sample of PDU == raw stream sample at b.start │
            │   PDU length = (b.stop - b.start) + burst_post_len   │
            │   emits pmt PDU on "cpdus" port                      │
            └─────────────────────────────────────────────────────┘
                                          │
                                          ▼
            ┌─────────────────────────────────────────────────────┐
            │ burst_downmix.handle_burst (one call per burst PDU)  │
            │   1. rotate by -relative_frequency  (line 804)       │
            │   2. update center_frequency += rf * fs (line 808)   │
            │   3. input_fir low-pass + decim 10× (line 830)       │
            │      sample_rate /= decimation (line 832)            │
            │      timestamp += ntaps/2 * 1e9/fs (line 827)        │
            │   4. write signal-filtered-deci-{id}.cfile (line 838)│
            │   5. start_finder: |x|² → LPF → 28% threshold      │
            │      → `start` index (lines 852-880)                 │
            │   6. write signal-filtered-deci-cut-start-{id}       │
            │      (line 884) from frame[start:end]                │
            └─────────────────────────────────────────────────────┘
                                          │
                                          ▼
            ┌─────────────────────────────────────────────────────┐
            │ burst_downmix.process_next_frame  (iterated per PDU  │
            │   if handle_multiple_frames_per_burst, lines 890-905)│
            │   7. squared-FFT CFO (line 520-528)                  │
            │      256 samples × Blackman → 4096-pt FFT × 16       │
            │      std::max_element on |X|² → center_offset        │
            │   8. rotate burst by -center_offset (line 571-574)   │
            │      center_frequency += center_offset * fs (line 575)│
            │      write signal-filtered-deci-cut-start-shift-{id} │
            │   9. RRC matched filter (line 593-599)               │
            │      write ...rrc-{id}.cfile                         │
            │  10. UW cross-correlation (FFT-based, N=2048 corr_fft │
            │      sync_ref is the 271-sample post-erase RC-shaped │
            │      reversed-conjugated preamble+UW pattern)        │
            │      → corr_offset → preamble_offset → uw_start      │
            │  11. pre-rotate FULL frame_size by conj(corr/|corr|) │
            │      → write ...rrc-rotate-{id}.cfile (frame_size)   │
            │  12. trim front by uw_start; write ...rotate-cut-{id}│
            │      (frame_size - uw_start samples)                 │
            │  13. timestamp += start * 1e9/fs (line 720)          │
            │      where start = start_finder_offset + uw_start    │
            │  14. publish pdu_meta { center_frequency, timestamp, │
            │      sample_rate, id, uw_start } on "burst_handled"  │
            │  15. if handle_multiple_frames_per_burst AND          │
            │      remaining > min_frame_length:                   │
            │        start += handled_samples; sub_id++; goto 7    │
            └─────────────────────────────────────────────────────┘
                                          │
                                          ▼
                              QPSK demodulator → BCH → frame parser
                              (one RAW line emitted per sub_id)
```

## fft_burst_tagger output metadata

Each new-burst tag carries:

| Key | Value | Notes |
|---|---|---|
| `id` | uint64_t burst counter | starts at 0, increments by 10 per detected burst |
| `relative_frequency` | float, (-0.5, +0.5) | = `(center_bin - fft_size/2) / fft_size` |
| `center_frequency` | double Hz | the LO/center of the input stream (NOT the burst freq) |
| `magnitude` | float | rolling power estimate at the burst bin |
| `noise` | float | rolling noise floor estimate |
| `sample_rate` | float | input_sample_rate (2.5 MSPS) |
| `timestamp` | uint64_t ns | time of **b.start**, NOT detection time |

`b.start = d_index - d_burst_pre_len` (line 306). `d_burst_pre_len = 2 * fft_size`
(= 4096 samples at fft_size=2048). So `b.start` is positioned `pre_len` samples
BEFORE the FFT detection point — that's the lead-in noise band before the
envelope rise. The PDU samples include this lead-in.

The tag is added at stream position `b.start + d_burst_pre_len = d_index` in
the DELAYED output stream (line 452). Because the stream output is delayed
by `pre_len` (line 555: `memcpy(out, in - d_burst_pre_len, ...)`), tag.offset
in the output stream corresponds to actual sample `d_index - pre_len = b.start`
in the original input.

This means **tagged_burst_to_pdu's PDU has its first sample at b.start in
the original raw IF stream.**

## burst_downmix timestamp adjustments

Three places the timestamp is incremented after entering burst_downmix:

1. **Inside fft_burst_tagger**: the published timestamp is computed
   from `b.start`, not from `d_index`. So it points to the start of
   the pre-pad, not the burst.

2. **Inside handle_burst, line 827**: after input_fir + decimation,
   ```cpp
   timestamp += d_input_fir.ntaps() / 2 * 1e9 / (int)sample_rate;
   ```
   compensates for the input_fir's group delay (~55 µs at 2.5 MSPS
   with ~278-tap Kaiser LPF).

3. **Inside process_next_frame, line 720**:
   ```cpp
   timestamp += start * 1e9 / (int)sample_rate;
   ```
   shifts the timestamp by the start_finder + UW correlator offsets
   (in 250 ksps samples). `start = start_finder_cut + uw_start`,
   typically ~400-700 250 ksps samples (~1.6-2.8 ms).

So the **RAW-line timestamp** that iridium_frame_printer prints is:
```
RAW_timestamp = time(b.start) + input_fir_delay + (start_finder_cut + uw_start) / sample_rate
              ≈ time(b.start) + 55 µs + ~1.6-2.8 ms
```

That is **NOT b.start** — it is shifted forward by the start_finder
+ UW alignment. The position printed is essentially "the time when
the UW symbol stream starts in the raw input stream".

For external code trying to reconstruct gri's burst window (path C of
our host pipeline), this means:
- `start_sample = RAW_timestamp × FS_RAW` does NOT give b.start.
- It gives a position somewhere INSIDE the burst, ~1.5-3 ms past b.start.
- To capture from b.start you need to look BACK by that amount.

## Frequency: relative_frequency, center_frequency, abs_freq

- **Input**: `center_frequency = LO` (the input stream's LO, e.g. 1618.5 MHz).
- After line 808 in handle_burst (post initial rotation):
  `center_frequency = LO + relative_frequency × fs`.
- After line 575 in process_next_frame (post squared-FFT CFO):
  `center_frequency = LO + relative_frequency × fs + center_offset × fs`.

The published `center_frequency` (printed as `abs_freq` in the RAW line)
is **the carrier's estimated absolute frequency in Hz after both the
tagger-bin rotation AND the squared-FFT CFO refinement**.

For a host pipeline rotating by `-(abs_freq - LO)`, that produces a
signal whose residual carrier is at the squared-FFT-refinement error
(typically ~0 Hz). It is NOT the same as gri's `signal-filtered-deci-{id}.cfile`,
which is post-initial-rotation but PRE-CFO — that file's spectrum is
offset from DC by the CFO-correction amount (~±200 Hz typically).

## Debug file layout (when --debug-id is used)

Files written into `/tmp/signals/` for the targeted burst id:

| File | Stage | Sample rate | Length |
|---|---|---|---|
| `signal-{id}.cfile` | raw burst PDU input (post fft_burst_tagger) | 2.5 MSPS | (b.stop - b.start + post_len) |
| `signal-filtered-deci-{id}.cfile` | post initial rotation + input_fir + decim | 250 ksps | input_len / 10 |
| `signal-mag-{id}.f32` | |x|² before start_finder LPF | 250 ksps | first ~7 ms only |
| `signal-mag-filter-{id}.f32` | post start_finder LPF (envelope) | 250 ksps | ditto |
| `signal-filtered-deci-cut-start-{id}.cfile` | post start_finder trim | 250 ksps | filtered_len - start_finder_cut |
| `signal-filtered-deci-cut-start-shift-{id}.cfile` | post squared-FFT CFO rotation | 250 ksps | same as above |
| `signal-filtered-deci-cut-start-shift-rrc-{id}.cfile` | post RRC matched filter | 250 ksps | same |
| `signal-filtered-deci-cut-start-shift-rrc-rotate-{id}.cfile` | post UW pre-rotation | 250 ksps | frame_size |
| `signal-filtered-deci-cut-start-shift-rrc-rotate-cut-{id}.cfile` | post uw_start trim | 250 ksps | frame_size - uw_start |

## Important pitfalls (the ones we hit)

### 1. RAW-line burst ID ≠ enumeration order

The `I:NNNNNNNNNNN` field in the RAW line is gri's internal `burst id`,
incremented by 10 per detected burst. **It is NOT the index of the
decoded-burst list.** When matching debug files (`signal-{id}.cfile`)
to a decoded burst, use the `I:` field, not the position in the RAW
output.

We initially indexed bursts by enumeration order in iridium-extractor
stdout, then compared path-C burst 0 (timestamp 418 ms, freq +17 kHz)
against `signal-30.cfile` (burst id=30 = timestamp 435 ms, freq +225 kHz)
and got bogus spectra. The two were entirely different bursts.

### 2. RAW timestamp is not b.start

See "burst_downmix timestamp adjustments" above. The RAW timestamp is
positioned at the UW symbol start, not the PDU start.

### 3. relative_frequency is in cycles/sample, not Hz

`relative_frequency = (center_bin - fft_size/2) / fft_size`. To get
Hz, multiply by `sample_rate`. The published `center_frequency` is
in Hz, but `relative_frequency` is normalized.

### 4. center_frequency is cumulative

It starts as the LO, then gets incremented by rotations. The published
value reflects the final cumulative state.

### 5. fft_burst_tagger output stream is delayed

The stream output is delayed by `burst_pre_len` samples (line 555).
This is why the tag at `tag.offset = d_index` actually points at raw
sample `d_index - pre_len = b.start` in the original input.

If you cross-correlate `signal-{id}.cfile` against the raw input
stream, the first sample of `signal-{id}.cfile` is at absolute raw
sample `b.start` (not `tag.offset`).

### 6. Decimation FIR quality is load-bearing

gri uses `firdes.low_pass_2(1, 2.5MHz, 20kHz, 40kHz, 40dB)` — about
279 Kaiser taps — for the 10× decimation from 2.5 MSPS to 250 ksps.
scipy.signal.resample_poly's DEFAULT internal window is a short
Kaiser (~21 taps) that gives only ~15 dB stopband at the passband
edge. In a 2.5 MHz window typical of Iridium reception, an adjacent
Iridium burst at ±42 kHz leaks through at -15 dB after decimation,
which is plenty to dominate the squared-FFT CFO step of the target
burst.

Build the FIR explicitly with `scipy.signal.kaiserord(40, 2*20e3/fs)`
+ `firwin(ntaps, 20e3, window=('kaiser', beta), fs=fs)` and pass it
as the `window=` argument to `resample_poly`. On the ALBQ fixture
this single change took path C from 21/65 to 59/65 decodes.

### 7. D13 cut-position formula depends on FIR index convention

gri's `start_finder` writes the filtered envelope to
`d_magnitude_filtered_f[k]`, where index `k=0` corresponds to INPUT
position `k + half_fir_size`. Their formula is:
```cpp
start = filtered_start + half_fir_size - pre_start_samples;
```
which converts a filtered-output index back to an input position
(`+ half_fir_size`) and then backs off by `pre_start_samples`.

If your port uses a CENTRED LP convolution where `smooth[n]` already
represents the smoothed envelope value at input position `n` (no
output-index conversion needed), the formula degenerates to:
```c
start = first_crossing - pre_start_samples;
```
i.e. **drop the `+ half_fir_size` term**. Including it double-counts
the FIR group delay by `half_fir = 91` samples (= 9 symbols at sps=10),
putting `adj_burst[0]` past the preamble's first symbol. The matched
filter then can't find the sync because it starts in negative-index
territory and falls back to a spurious data-region peak.

### 8. The sync reference length is 271 samples, NOT 280

`generate_sync_word()` lines 244-260 build the padded sync word in
two steps:
1. For each of the 28 sync symbols, push the symbol then push
   `sps - 1 = 9` zeros → 28 × 10 = 280 elements.
2. `erase(end - sps + 1, end)` — strip the trailing 9 zeros from
   the LAST symbol → 271 elements.

The padded sync is then RC-filtered in valid mode and reverse-conjugated.
The final reference stored in `d_dl_preamble_reversed_conj` has
**271 samples**, with the last symbol's RC trailing tail truncated.

Using a 280-sample reference (the naive interpretation of
`SYNC_LENGTH × sps`) carries 9 extra samples of the last symbol's RC
tail that gri doesn't have, biasing the matched-filter peak position.

### 9. The squared-FFT CFO input is 256 samples, NOT 280

gri's `d_cfo_est_fft_size`:
```cpp
pow(2, int(log(sps * (PREAMBLE_LENGTH_SHORT + 10)) / log(2)))
  = pow(2, int(log(260) / log(2)))
  = pow(2, 8)
  = 256
```
That is: preamble (16 syms) + **10 UW symbols** (not 12), rounded
DOWN to the next power of 2 → 256. The 256-sample Blackman window
shape and the squared signal's spectral content both differ from the
naive `SYNC_LENGTH × sps = 280` choice.

### 10. The matched-filter FFT needs `next_pow2(840 + 280 - 1) = 2048`

`d_sync_search_len = (PREAMBLE_LENGTH_LONG + UW_LENGTH + 8) * sps`
uses `PREAMBLE_LENGTH_LONG = 64`, giving 840 at sps=10. With
`sync_word_len = 271` (post-erase) the corr FFT size is
`next_pow2(840 + 271 - 1) = next_pow2(1110) = 2048`. Using LENGTH_SHORT
(=16) here gives `next_pow2(360 + 280 - 1) = 1024`, which aliases the
linear convolution and produces spurious peaks in the data portion of
the burst.

### 11. burst_downmix iterates frames per PDU (`handle_multiple_frames_per_burst`)

After a successful frame decode, `handle_burst` advances `start` by
the consumed sample count (line 902) and calls `process_next_frame`
again with a new `sub_id`. Each iteration runs its own CFO, matched
filter, and demod from the new `start`. Frames within one PDU are
published as consecutive IDs (`sub_id++`).

When matching gri's published RAW lines to a per-burst host pipeline,
you may see two RAW lines whose timestamps fall within the same
physical burst's PDU window: those are sibling frames from one PDU.
Our port currently processes one frame per call; the second frame
manifests as a "no-demod" failure on path C (the matched filter
finds the FIRST frame, not the SECOND). Fix requires porting the
iterative loop into burst_pipeline_process_250khz.

### 12. Q15 incremental phasor decays magnitude — gri uses float32

A naïve Q15 rotator with an incremental phasor:
```c
int16_t pr = 32767, pi = 0;
for (k = 0; k < n; k++) {
    out[k] = in[k] * (pr + j*pi);
    // advance: p ← p · (cs_q + j·ss_q)
    pr_new = (pr*cs_q - pi*ss_q) >> 15;
    pi_new = (pr*ss_q + pi*cs_q) >> 15;
}
```
loses **0.012% magnitude per sample** because the `>>15` truncates ~1
LSB of the unit-magnitude phasor every step, and the quantised
(cs_q, ss_q) themselves have |z| slightly < 1. Over a 44k-sample
burst window the phasor magnitude collapses:
`|phasor| → 0.99988^44096 ≈ 2e-7`.

This manifests as a ~3 dB AMPLITUDE divergence at every downstream
stage of the wideband chain, even though the SHAPE of the signal
(phase coherence) stays high. Easy to mistake for some other DSP
issue.

gr-iridium uses volk's float32 rotator (`d_r.rotateN()`), which
keeps |phasor| ≈ 1.0 across any practical burst length. To match
gr-iridium on the host, **compute the phasor from absolute phase
each sample** (`cosf(k·dphi)`, `sinf(k·dphi)`) — no per-step
multiplication, no cumulative quantisation. Two trig ops per sample
are negligible vs the FFT cost.

On P4 we'll need either periodic Q15 phasor renormalisation OR
per-sample cosf/sinf — TBD under Phase 3.6.P. For the
`q15_freq_shift_inplace` in burst_pipeline.c (which applies a CFO
correction over a ≤17 ms window = ≤4250 samples at 250 ksps,
shorter than the wideband window so the decay is tolerable for
that use), the incremental phasor is currently still in use.

### 13. Pre-rotation: rotate full frame_size first, THEN trim by uw_start

In `process_next_frame` (lines 692-694, 710-711) the order is:
1. Compute uw_start from the FFT correlation peak
2. Rotate the FULL `frame_size` samples of `d_tmp_a` into `d_tmp_b`
3. THEN trim front by `uw_start` (`frame_size -= uw_start`, `start += uw_start`)

The "rotate" debug dump (`...rrc-rotate-{id}.cfile`) is written
BEFORE the trim, so its first sample is at the burst-frame start
(NOT at the UW). The "rotate-cut" dump (`...rrc-rotate-cut-{id}.cfile`)
is written AFTER the trim, so its first sample is at the UW.

A naive port that rotates only from the UW onward (= `src = adj_burst
+ uw_offset`) produces a "rotate" dump that doesn't align with gri's
window boundaries, making stagewise comparison misleading.
