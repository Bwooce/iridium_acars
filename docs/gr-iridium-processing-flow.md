# gr-iridium processing flow (reference notes)

These are working notes captured while wiring up path-C of our host
pipeline and chasing stage-by-stage divergence from gr-iridium. The
goal here is not to duplicate the upstream code, but to capture the
sequence, the per-stage data layout, and (most importantly) the
non-obvious invariants that bit us when we tried to mirror it.

All line numbers reference `gr-iridium/lib/burst_downmix_impl.cc` and
`gr-iridium/lib/fft_burst_tagger_impl.cc` as of the version checked
into our submodule.

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
            │ burst_downmix.process_next_frame                     │
            │   7. squared-FFT CFO (line 520-528)                  │
            │      256 samples × Blackman → 4096-pt FFT × 16       │
            │      std::max_element on |X|² → center_offset        │
            │   8. rotate burst by -center_offset (line 571-574)   │
            │      center_frequency += center_offset * fs (line 575)│
            │      write signal-filtered-deci-cut-start-shift-{id} │
            │   9. RRC matched filter (line 593-599)               │
            │      write ...rrc-{id}.cfile                         │
            │  10. UW cross-correlation (FFT-based)                │
            │      → corr_offset → preamble_offset → uw_start      │
            │  11. pre-rotate burst by conj(corr_result)/|corr|    │
            │      → align phase; write ...rrc-rotate-{id}.cfile   │
            │  12. trim front by uw_start; write ...rotate-cut-{id}│
            │  13. timestamp += start * 1e9/fs (line 720)          │
            │      where start = start_finder_offset + uw_start    │
            │  14. publish pdu_meta { center_frequency, timestamp, │
            │      sample_rate, id, uw_start } on "burst_handled"  │
            └─────────────────────────────────────────────────────┘
                                          │
                                          ▼
                              QPSK demodulator → BCH → frame parser
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
