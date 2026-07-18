# P4 full-chain vs gr-iridium at equal (2.5 MHz) bandwidth — 2026-07-17

## Question

Does the ESP32-P4's FULL receive chain — OUR fft_burst_tagger +
rotate-to-DC + 10× decim + burst_prefilter + burst_pipeline +
classify/ida_decode + IDA/SBD reassemblers + libacars — recover the same
IDA/ACARS yield gr-iridium does when both see the SAME 2.5 MSPS input?
(The prior demod-diff only tested per-burst demod on gri-tagged windows;
this run includes our tagger and the whole production decode tail.)

## Method

Harness: `tests/host/test_fullchain_slice.c` (build:
`cmake --build tests/host/build-mac --target test_fullchain_slice`), a
host build of the production chain:

    fft_burst_tagger (thr 10 dB = DEFAULT_TAGGER_THRESHOLD_DB, pre 4096,
      post 40000, width 32 — dsp_processor.c Path-A params)
    → per burst: rotate_to_dc + direct_if_decim (2.5 MSPS → 250 ksps)
    → burst_prefilter (production P1.5b gate, PF_THRESH_DB 14, incl. a
      serial emulation of the A6 hot-bin SNR exemption)
    → burst_pipeline_process_burst (D13/CFO/RRC/UW/PLL/demod, multi-frame)
    → iridium_frame_classify → ida_decode (clean gate: bch+hdr+crc, as
      frame_decoder.c; best_effort_decode=OFF, the firmware default)
    → ida_reassembler → sbd_reassembler → libacars (acars_tail_feed),
      with real RF-sample-derived timestamps driving the 700 ms/1 s
      IDA chain gap/expiry logic.

Inputs: two 2.5 MSPS int16 ci16 slices decimated from a real HydraSDR
capture to the P4's exact Path-A bandwidth, each holding exactly one
gri-decodable ACARS:

- `~/iridium_capture/slice25_vhvwn_off.ci16` — 3.0 s, center 1,619.5 MHz
- `~/iridium_capture/slice25_vh8vb_off.ci16` — 2.62 s, center 1,624.0 MHz

gr-iridium baseline re-verified on this machine from the same files
(`iridium-sniffer --format ci16 -r 2500000` → `iridium-parser.py` →
`reassembler.py -m acars`): VH-VWN slice = 4 IDA / 1 ACARS; VH-8VB slice
= 7 IDA / 1 ACARS — matching the provided baselines exactly.

## Results

| metric                        | VH-VWN slice: gri | VH-VWN slice: P4 | VH-8VB slice: gri | VH-8VB slice: P4 |
|-------------------------------|------------------:|-----------------:|------------------:|-----------------:|
| bursts tagged                 | 29                | 1059 ¹           | 48                | 992 ¹            |
| prefilter rejects (all junk)  | —                 | 660 (all dur)    | —                 | 616 (all dur)    |
| frames demodulated            | 25 lines          | 25               | 31 lines          | 31               |
| **IDA (LW.DA) frames**        | **4**             | **4** (3 clean + 1 crc-fail ²) | **7** | **7** (5 clean + 2 crc-fail ²) |
| SBD messages                  | 1                 | 1                | 1                 | 1                |
| **ACARS reassembled**         | **1 (VH-VWN)**    | **1 (VH-VWN)** ³ | **1 (VH-8VB)**    | **1 (VH-8VB)** ³ |

¹ Tag counts are not comparable: our tagger ran the firmware-default
10 dB threshold (≈ gri − 4 dB scale ⇒ far looser than gri's 18 dB) and
the slices' band edges are full of decimation-roll-off noise tags. All
excess tags died at the prefilter duration gate or in the pipeline; they
cost nothing here (host has no compute budget).

² The crc-fail LW.DA frames are the same maintenance/handoff-payload DA
frames gri also lists as IDA lines (its IDA count doesn't require the DA
payload CRC). Apples-to-apples on total LW.DA: 4/4 and 7/7.

³ Both ACARS needed a real 2-fragment IDA chain (cont=1,ctr=0 →
ctr=1) → 0x7608 SBD → libacars. Field-exact vs gri's JSON: reg VH-VWN
label `_d` block X / reg VH-8VB label `_d` block Y, mode 2, DL, crc OK,
empty text.

Controls:

- `PF=0` (prefilter off): identical IDA/ACARS on both slices (only +2/+1
  junk/marginal demods). The prefilter cost zero yield on these slices —
  no SNR-gate rejects at all fired (bursts here are strong, 16–27 dB),
  so the known continuation-eating channel-SNR gate was not exercised.
- Hot-bin rescue count: 0 (never needed).
- Harness exits PASS on `expect_substr` VH-VWN / VH-8VB respectively.

## Verdict

**The P4 full chain matches gr-iridium at equal bandwidth on both
slices: same 4/7 IDA frames, same 1+1 ACARS, field-exact.** There is no
front-end (tagger) gap and no demod gap on this material — every frame
gri demodulated, our chain demodulated (25/25 and 31/31), including both
2-fragment ACARS chains through the full reassembly tail with real
timestamps.

Attribution note: since the DSP/decode chain is at parity here, this is
consistent with the live-device 0-ACARS finding being **not** a decode
bug but reception/compute-throughput (worker stale-drops under load —
see memory `p4_acars_decode_is_reception_not_bug`): the host harness has
unlimited compute, the device does not.

## Caveats / UNVERIFIED

- These bursts are strong (16–27 dB). Parity here does NOT bound the
  known ~3–4 dB demod-edge gap at threshold SNR; a weak-burst corpus
  would be needed for that.
- Device NVS `tag_thr` on the live unit is UNVERIFIED (harness used the
  firmware default 10 dB; `TAG_THR` env overrides). At 10 dB the tagger
  over-tags band-edge junk ~30×; harmless on host, but on-device this
  tag load is worker pressure.
- Non-DA classification is not 1:1 (VH-VWN slice: gri saw 3 IBC, our
  chain classed those among 17 unk — BCH margin on weak IBC frames;
  VH-8VB slice classed BC 9 vs gri 10). Does not affect the IDA/ACARS
  path measured here.
- The A6 hot-bin exemption was emulated serially in-harness (device runs
  it cross-core with wall-clock TTL); it never triggered, so untested by
  this run.
- Slice sample amplitudes are small (|IQ| mostly < a few hundred
  counts); int16 quantisation of the slice prep did not visibly hurt
  (parity achieved at SCALE=1).
