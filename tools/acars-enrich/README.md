# acars-enrich

Host-side sidecar that decodes the **text payload** of ACARS messages captured by
the P4 SDR, using [`@airframes/acars-decoder`](https://github.com/airframesio/acars-decoder-typescript)
— the same content decoder that powers airframes.io and AcarsHub.

This is **not** an RF/frame decoder and it does **not** run on the device. The
ESP32-P4 firmware handles RF → frame → ACARS (with the vendored `libacars` C
library for the ARINC applications). This tool runs on the Mac and adds the
*human-readable content* layer on top of the raw `txt` field — position reports,
progress reports, CPDLC/ADS-C, weather, fuel/ETA, etc.

It is deliberately isolated: its own `package.json`, `node_modules/` is
gitignored, and it is **outside** the firmware build and the host-test CI.

## Requirements

Node.js >= 18 (tested on the repo's installed Node).

## Install

```
cd tools/acars-enrich
npm install
```

## Use

Input is the device NDJSON produced by `scripts/pull_acars_logs.sh` (consolidated
at `~/iridium_capture/acars_consolidated.ndjson`) or any NDJSON with `label` +
`txt` (also accepts `text`) fields.

```
# Enriched NDJSON on stdout (adds a `decode` block per record):
node enrich.js ~/iridium_capture/acars_consolidated.ndjson > enriched.ndjson

# Human-readable, decoded messages only:
node enrich.js ~/iridium_capture/acars_consolidated.ndjson --pretty --only-decoded

# From a pipe:
cat foo.ndjson | node enrich.js
```

A coverage summary (full / partial / none, by label) is always printed to
**stderr**, so it never pollutes the NDJSON on stdout.

### Flags

- `--pretty` — human-readable table instead of NDJSON.
- `--only-decoded` — skip records the library couldn't decode (level `none`).

## What "decoded" means

Each record gets a `decode` block:

```json
"decode": {
  "level": "full" | "partial" | "none",
  "decoder": "<plugin name>",
  "description": "Position Report",
  "raw": { "position": {"latitude": -33.83, "longitude": 151.16}, "altitude": 2400, ... },
  "items": [ {"label": "Altitude", "value": "2400 feet"}, ... ]
}
```

Coverage is plugin-based and partial by design (the library documents the label
formats the community has reverse-engineered). Empty-payload records (many `_d`
downlink acks, empty `Q0`) correctly decode to `none`.

## Refresh the corpus

```
bash ../../scripts/pull_acars_logs.sh    # read-only pull from the device
```
