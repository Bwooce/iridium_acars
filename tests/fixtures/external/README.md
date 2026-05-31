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

Optional / deferred:

| Source | What it would add | Why deferred |
|---|---|---|
| **Oligeri/Sciancalepore Mendeley** ([DOI 10.17632/xcxspv8c2r.2](https://data.mendeley.com/datasets/xcxspv8c2r/2)) | 3.8 M IRA packets across 66 sats × 2 months in Doha QA — *horizontal* diversity (sat ID, beam ID, Doppler, time of day) for ONE burst type | Our ALBQ has *vertical* diversity (7 burst types in one capture). The Mendeley set's win is per-sat/beam variation, which only matters if we want to debug a specific sat-ID-dependent regression or do per-beam SNR statistics. 4.67 GB download + per-packet format (already burst-extracted at unclear sample rate) — the cost is real. Fetcher + extractor + skeleton test are committed; populate the cache + run the build script if/when needed. |

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
