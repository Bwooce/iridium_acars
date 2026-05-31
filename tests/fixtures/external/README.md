# tests/fixtures/external/ — externally-sourced IQ fixtures

This directory tracks IQ data from sources outside the project (published
datasets, user-collected captures) that is too large to commit but is
useful for regression testing. The data files themselves live OUTSIDE
the repo (in `~/iq_cache/` by default); only the **scripts that fetch
them** and the **small derived fixtures** (≤ ~1 MB each) live here.

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

Burst-level sources currently usable:

| Source | Bursts | Sat coverage | Notes |
|---|---|---|---|
| **Oligeri/Sciancalepore Mendeley** ([10.17632/xcxspv8c2r](https://data.mendeley.com/datasets/xcxspv8c2r/2)) | ~3.8 M | 66-sat Iridium NEXT, Doha QA, Aug–Oct 2020 | All IRA-DL (ring alert downlink). Massive variety in SNR / Doppler / sub-burst position. Fetcher: `fetch_mendeley_iridium.py`. |
| **In-repo ALBQ slice** | ~100 | Single Albuquerque recording | `tests/fixtures/fixture_albq_*.h` — already shipped, used by `test_pipeline_wideband_albq` etc. |

### Layer B — wideband fixtures

Raw SDR output at the radio sample rate (typically 2.4–10 MSPS),
covering the full Iridium band (1616–1626 MHz) or a sub-band slice.
Multiple bursts within one capture, plus noise between bursts.

These feed into the full pipeline starting from `dsp_processor_feed`
(or earlier through `signal_buffer` if testing the ingest path). They
cover everything Layer A does, **plus** the FBT tagger, DC removal,
and pipeline timing under realistic burst arrival statistics.

Wideband sources currently usable:

| Source | Sample rate | Bandwidth | Notes |
|---|---|---|---|
| **In-repo ALBQ raw** | 2.56 MSPS | ~2.56 MHz @ 1625.27 MHz | `fixture_albq_raw.h`, `fixture_albq_raw_high.h`, `fixture_albq_raw_2667.h`. The basis for the `RAW_IRIDIUM` smoke variant. |
| **(future) user-captured 10 MHz** | up to 10 MSPS | Whole Iridium band 1616–1626 | RESERVED. When Bruce captures these, drop them into `~/iq_cache/wideband/` and add a manifest entry under `wideband/manifest.json` per the schema below. |

## Cache layout (`~/iq_cache/`)

The cache directory is **never committed**. It holds the large source
files plus extracted derivatives. Default layout:

```
~/iq_cache/
├── mendeley_iridium.zip                # 4.67 GB, raw download
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

For the future wideband captures, drop a `manifest.json` into
`~/iq_cache/wideband/` so the fetcher / test scripts can discover them:

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
