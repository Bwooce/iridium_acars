# Iridium Downlink — Albuquerque, 2022-03-17

Wideband recording of the full Iridium L-band downlink band, captured
at a USRP B210 in Albuquerque, NM (35.14, -106.51) on 2022-03-17 at
21:49:12 UTC. Released under CC0.

This is the first **public** real-RF capture in this repo with
sufficient bandwidth to contain ACARS-bearing IDA frames (the duplex
band in 1616-1626 MHz). The pre-existing `test_corpus/` is a
synthetic PRBS-15 fixture from gr-iridium upstream; it is useful as a
DSP correctness check but does not exercise the SBD/IDA/ACARS
framing layer above the BCH decoder.

## Capture parameters

| Field | Value |
|---|---|
| Sample format | cf32 little-endian (interleaved float32 I/Q, 8 bytes/sample) |
| Sample rate | 12,000,000 Hz |
| Center freq | 1,621,500,000 Hz |
| Coverage | 1615.5 - 1627.5 MHz (full Iridium downlink) |
| Hardware | USRP B210 |
| Location | 35.14°N, -106.51°W (Albuquerque, NM) |
| Datetime (UTC) | 2022-03-17T21:49:12Z |
| File size (uncompressed) | 120,000,000 bytes (115 MB) |
| File size (zstd) | ~45 MB — what's actually committed |
| Sample count | 15,000,000 (= 1.25 s of capture) |
| License | CC0 |

A 1.25 s slice at 12 MHz BW captures roughly 14 Iridium TDMA frames
(90 ms per L-band frame), with up to ~12 simultaneous channels
visible — so on the order of 100+ bursts in the file. How many of
those carry IDA → SBD → ACARS payloads (vs IRA ring alerts, IBC
broadcasts, etc.) is what `gr-iridium`'s extractor + parser tells us.

## Files in this directory

- `iridium_cf32.sigmf-data.zst` — zstd-compressed raw recording (45 MB).
  `derive.sh` transparently decompresses to `iridium_cf32.sigmf-data`
  on first use; the decompressed file is gitignored.
- `iridium_cf32.sigmf-meta` — SigMF v1.0.0 metadata (JSON).
- `derivations/` — all artefacts we produce *from* the raw recording:
  - `iridium.bits` — gr-iridium iridium-extractor output (bursts +
    demodulated bits, one line per detected frame).
  - `iridium.parsed` — iridium-toolkit `iridium-parser.py` output:
    classifies each line as IRA / IDA / IBC / etc., decodes payloads.
  - `acars_frames.txt` — extracted ACARS payloads if any are present.
  - `iridium_uint8_2p56MHz.bin` — resampled to 2.56 MSPS uint8 IQ
    (RTL-SDR convention) for our P4 pipeline. Built by
    `tests/scripts/resample_for_p4.py`.

## Reproducing the derivations

```sh
# From the repo root:
./test_data/iridium_downlink_2022-03-17_albuquerque/derive.sh
```

The script runs `iridium-extractor` over the cf32, then
`iridium-parser.py` to classify the bursts, then our resampler to
produce the uint8 P4 fixture. All outputs land in
`derivations/`. The script is idempotent and skips steps whose
output already exists.

## Provenance

The user (bruce@fitzsimons.org) downloaded this from a public source
on 2026-05-06; metadata as supplied above. The CC0 license permits
redistribution as a regression fixture in this repo.
