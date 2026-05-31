# tests/fixtures/external/ — externally-sourced IQ fixtures

This directory tracks IQ data from sources outside the project. Data
that's already in the repo (the CC0-licensed ALBQ recording — see
**Layer B — wideband** below) is the primary fixture source. This
directory adds:

- **Documentation of the fixture taxonomy** so future additions land in
  the right slot.
- **A manifest schema for user-collected wideband captures** that drop
  into `~/iq_cache/wideband/` (Bruce's future 10 MSPS collection).
- **Optional scripts for the published Mendeley Iridium dataset** —
  available if we ever want per-satellite / per-beam horizontal
  diversity. Not needed for the immediate goal: the in-repo ALBQ
  wideband already gives us 82 bursts across 7 burst types from a
  real-RF capture.

## Fixture taxonomy

Iridium IQ data sits at one of two layers, depending on where it was
captured. Tests at different layers exercise different DSP code, so the
fixture infra deliberately keeps these separate.

### Layer A — burst-level fixtures

Already-extracted bursts at the channelizer output (~25–250 ksps,
1024–2048 complex samples per burst). One file = one burst.

These feed into `burst_pipeline_process_burst()` directly. They cover:
D13 envelope start_finder, `cfo_fine_estimate`, RRC, UW correlator,
PLL, QPSK demod, BCH, deinterleave, classify.

They do **NOT** cover: USB ingest, signal_buffer, dsp_processor /
fft_burst_tagger. Those want layer B.

Burst-level sources currently in use:

| Source | Bursts | Burst types | Notes |
|---|---|---|---|
| **In-repo ALBQ slice** | ~82 | ISY×28 IDA×20 RAW×11 IBC×11 IIU×7 I36×4 IRI×1 | `tests/fixtures/fixture_albq_*.h` — derived from the in-repo wideband; covers all the variants the production pipeline classifies. |

Optional / deferred (alphabetical):

| Source | Scale & format | What it would add | Why deferred |
|---|---|---|---|
| **Oligeri/Sciancalepore Mendeley** ([DOI 10.17632/xcxspv8c2r.2](https://data.mendeley.com/datasets/xcxspv8c2r/2)) | 4.67 GB zip → 14 GB text. 3.8 M IRA packets. **~110 complex samples per packet at post-PLL symbol rate** (25 ksps, gr-iridium's PHASE_B_FS) — *not* raw wideband despite the paper's framing | Cross-validation of our BCH(31,21) decoder + IRA payload parser against gr-iridium's output across 3.8 M ground-truthed bursts | Per-burst layer means we cannot exercise burst_pipeline / FBT / DC removal / D13 / CFO / UW / PLL / sym_timing. Useful only for the post-PLL pipeline tail. `tests/scripts/mendeley_decode_validate.py` runs the differential-decode → BCH → classify path but **the exact frame alignment requires more iridium-toolkit-source reverse-engineering** to reach high decode rate; current dry-run hits 14% bch_ok / 1% sat_id-match, signalling the frame-structure assumption ([UW]+[LCW]+[Block1]+[Block2]) is incomplete. |
| **Oxford SatIQ — Watch This Space** ([Zenodo 8220494](https://zenodo.org/record/8220494)) | 135 GB compressed → 67 TB uncompressed. 1.7 M IRA messages as numpy .npy files, **25 MS/s** per-burst | Highest-rate per-burst dataset publicly available; ideal for fingerprinting research. Code at [ssloxford/SatIQ](https://github.com/ssloxford/SatIQ) | Massive — far exceeds what's needed for pipeline regression testing. Per-burst layer (same as Mendeley). |
| **Oxford SatIQ — 3 locations** ([UK](https://doi.org/10.7910/DVN/P5FUAW), [Germany](https://doi.org/10.7910/DVN/RXWV1M), [Switzerland](https://doi.org/10.7910/DVN/OSSJ68) on Harvard Dataverse) | Three geographic captures, full size each unknown but on the order of the Watch This Space set | Geographic diversity (channel statistics vary with antenna location); useful if we ever want to debug a propagation-channel-related issue | Same per-burst layer; storage cost > usefulness for our regression goals. |

### Layer B sources not yet public (but worth asking)

A June 2026 survey of all major repositories (Zenodo, figshare, Harvard Dataverse, SigMF/IQEngine, sigidwiki, GitHub, Kaggle, Internet Archive, Reddit r/RTLSDR, GNU Radio Discourse) found **no publicly downloadable wideband (≥4 MHz) Iridium L-band IQ captures** other than our own ALBQ recording.

One promising lead:

| Source | Format | Why interesting | Status |
|---|---|---|---|
| **alphafox02/iridium-sniffer benchmark recording** ([github.com/alphafox02/iridium-sniffer](https://github.com/alphafox02/iridium-sniffer)) | cf32, 10 MHz BW, 1622 MHz centre, USRP B210, 60 s | Exactly the format and bandwidth we need; referenced in the README but not published | Not released. Contact author (@cemaxecuter on X / GitHub issue) to request it. |

Suggested request text for a GitHub issue: *"We're building a wideband Iridium ACARS/SBD decoder (ESP32-P4, open-source) and are looking for a second wideband test fixture to complement our ALBQ USRP B210 capture. Would you be willing to share the 60-second cf32/10 MHz/1622 MHz B210 recording you reference in the README?"*

### Layer B — wideband fixtures

Raw SDR output at the radio sample rate (typically 2.4–12 MSPS),
covering the full Iridium band (1616–1626 MHz) or a sub-band slice.
Multiple bursts within one capture, plus noise between bursts.

These feed into the full pipeline starting from `dsp_processor_feed`
(or earlier through `signal_buffer` if testing the ingest path). They
cover everything Layer A does, **plus** the FBT tagger, DC removal,
and pipeline timing under realistic burst arrival statistics.

Wideband sources currently in use:

| Source | Sample rate | Bandwidth | Notes |
|---|---|---|---|
| **In-repo ALBQ raw** | 12 MSPS native; 2.56 MSPS resampled for P4 | Full 12 MHz Iridium band | `test_data/iridium_downlink_2022-03-17_albuquerque/iridium_cf32.sigmf-data.zst` (45 MB committed; decompresses to 115 MB via `derive.sh`). This is the BASIS for `fixture_albq_raw.h`, `fixture_albq_stripe_*.h`, the `RAW_IRIDIUM` smoke variant, and the Layer-A ALBQ burst slices. CC0. |
| **(future) user-captured 10 MHz** | up to 10 MSPS | Whole Iridium band 1616–1626 | RESERVED. When Bruce captures these, drop them into `~/iq_cache/wideband/` and add a manifest entry per the schema below. |

## Cache layout (`~/iq_cache/`)

The cache directory is **never committed**. It holds large source files
that are external to the repo. Default layout:

```
~/iq_cache/
├── mendeley_iridium.zip                # 4.67 GB (only if you ran fetch_mendeley_iridium.py)
├── mendeley_extracted/                 # decompressed text files
│   ├── 1208-1009_20_parsed.txt         # 7.5 GB
│   └── 1109-0910_20_parsed.txt         # 6.7 GB
└── wideband/                           # FUTURE — user wideband captures
    ├── manifest.json                   # see schema below
    ├── 2026-mm-dd_1620MHz_10MSPS.cf32
    └── ...
```

Override the cache root with `IQ_CACHE_ROOT=/path/to/somewhere`.

## Wideband manifest schema

For future wideband captures, drop a `manifest.json` into
`~/iq_cache/wideband/` so test scripts can discover them:

```json
{
  "captures": [
    {
      "filename": "2026-06-15_1620MHz_10MSPS_rooftop.cf32",
      "sample_rate_hz": 10000000,
      "center_freq_hz": 1620000000,
      "format": "cf32",                  // or ci16, cu8, cs8
      "duration_s": 60.0,
      "antenna": "L-band patch + LNA",
      "location": "Sydney NSW rooftop",
      "notes": "Clear sky, ~20 dB elevation min",
      "expected_decodes_min": 10         // optional: regression floor
    }
  ]
}
```

`test_pipeline_wideband_user.c` (to be written when fixtures exist) will
iterate over this manifest and run each capture through the full
pipeline.

## Sigil

If a test wants external data:

```c
// Skip cleanly if the external fixture isn't present (CI without cache).
const char *path = getenv("IRIDIUM_EXTERNAL_DATA");
if (!path) { puts("SKIP: external data not available"); return 0; }
```

CI runs without external data; developers with cached data get richer
coverage.
