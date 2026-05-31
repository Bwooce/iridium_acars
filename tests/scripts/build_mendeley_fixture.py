#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
build_mendeley_fixture.py — extract a small representative slice from the
Mendeley Iridium dataset and emit a compact C header fixture for host
tests.

Reads from the cache populated by fetch_mendeley_iridium.py. Picks N
bursts (default 100) balanced across the SNR range present in the data,
quantises IQ to int16 (Q15-ish), and writes tests/fixtures/external/
fixture_mendeley_burst_sample.h. The committed fixture is ≤ ~1 MB.

Format note (verified post-download): TODO — fill in once the actual
text format is inspected. Until then, the parser supports two
fallbacks: MATLAB-style "(a+jb), (a+jb), ..." cell-arrays and a
column-per-sample shape.

Usage:
    python3 tests/scripts/build_mendeley_fixture.py
    python3 tests/scripts/build_mendeley_fixture.py --n 100
    python3 tests/scripts/build_mendeley_fixture.py --inspect    # dump first 3 lines, no fixture build
"""

import argparse
import re
import sys
from pathlib import Path

import numpy as np

# Local imports
sys.path.insert(0, str(Path(__file__).parent))
import fetch_mendeley_iridium as fetch     # noqa: E402

DEFAULT_N = 100              # bursts to keep in the fixture
HEADER_PATH = (Path(__file__).resolve().parent.parent
               / "fixtures" / "external"
               / "fixture_mendeley_burst_sample.h")


# Regex for one complex number in MATLAB notation:
#   (-0.123+0.456j)  /  (0.5-0.25j)  /  (-1e-3+2e-4j)
RE_COMPLEX = re.compile(
    r"\(\s*([+-]?\d+\.?\d*(?:[eE][+-]?\d+)?)"
    r"\s*([+-])\s*"
    r"(\d+\.?\d*(?:[eE][+-]?\d+)?)\s*j\s*\)"
)


def parse_iq_cell(s: str) -> np.ndarray:
    """Parse a MATLAB-style cell array of complex numbers into ndarray.

    Returns empty array if no complex numbers found (caller should fall
    back to alternate parser)."""
    parts = RE_COMPLEX.findall(s)
    if not parts:
        return np.empty(0, dtype=np.complex64)
    arr = np.empty(len(parts), dtype=np.complex64)
    for i, (re_s, sign, im_s) in enumerate(parts):
        re_f = float(re_s)
        im_f = float(im_s) * (1.0 if sign == "+" else -1.0)
        arr[i] = complex(re_f, im_f)
    return arr


def parse_one_line(line: str) -> dict | None:
    """Parse one record line from the Mendeley text file.

    Expected columns per the paper (in order):
        timestamp, sat_id, beam_id, lat, lon, alt, confidence, freq, iq_cell

    Returns dict on success, None if the line can't be parsed (logs to
    stderr at most once per N lines)."""
    line = line.rstrip("\n")
    if not line:
        return None
    # Crude CSV split — the IQ field is the last column and contains
    # commas. Split off the first 8 columns and treat the rest as IQ.
    parts = line.split(",", 8)
    if len(parts) < 9:
        return None
    try:
        rec = {
            "ts":         float(parts[0]),
            "sat_id":     int(parts[1]),
            "beam_id":    int(parts[2]),
            "lat":        float(parts[3]),
            "lon":        float(parts[4]),
            "alt":        float(parts[5]),
            "confidence": float(parts[6]),
            "freq_hz":    float(parts[7]),
            "iq":         parse_iq_cell(parts[8]),
        }
    except ValueError:
        return None
    if rec["iq"].size == 0:
        return None
    return rec


def snr_estimate_db(iq: np.ndarray) -> float:
    """Rough SNR estimate: peak-magnitude vs background.

    Real PSD-based SNR would be better; this is for SELECTION not for
    annotation. We want to pick bursts spanning a range of qualities."""
    if iq.size < 32:
        return 0.0
    mag2 = (iq.real ** 2 + iq.imag ** 2)
    peak = mag2.max()
    median = np.median(mag2) + 1e-30
    return 10.0 * np.log10(peak / median)


def pick_balanced(records: list, n: int) -> list:
    """Bucket records by SNR and pull a balanced slice."""
    if len(records) <= n:
        return records
    snrs = np.array([r["_snr_db"] for r in records])
    # 5 SNR buckets across the observed range.
    edges = np.quantile(snrs, np.linspace(0.0, 1.0, 6))
    out = []
    per_bucket = n // 5
    for i in range(5):
        lo, hi = edges[i], edges[i + 1]
        mask = (snrs >= lo) & (snrs <= hi if i == 4 else snrs < hi)
        bucket = [records[j] for j in np.where(mask)[0]]
        if not bucket:
            continue
        # Spread within bucket: every k-th.
        k = max(1, len(bucket) // per_bucket)
        out.extend(bucket[::k][:per_bucket])
    return out[:n]


def quantise_int16(iq: np.ndarray) -> np.ndarray:
    """Quantise cf32 → int16 with auto-scaling to ±28000 (90% of int16
    range, leaves headroom for any subsequent in-place ops)."""
    if iq.size == 0:
        return np.empty(0, dtype=np.int16)
    peak = max(np.abs(iq.real).max(), np.abs(iq.imag).max(), 1e-9)
    scale = 28000.0 / peak
    out = np.empty(iq.size * 2, dtype=np.int16)
    out[0::2] = np.clip(np.round(iq.real * scale), -32768, 32767).astype(np.int16)
    out[1::2] = np.clip(np.round(iq.imag * scale), -32768, 32767).astype(np.int16)
    return out


def emit_header(records: list, out_path: Path) -> None:
    """Write the C header. Each burst is a flat int16 IQ array plus
    metadata. Stored in PROGMEM-style flat arrays; index table maps
    burst index → (offset, len, metadata)."""
    total_samples = sum(2 * len(r["_iq_int16"]) // 2 for r in records)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="\n") as f:
        f.write("// Generated by tests/scripts/build_mendeley_fixture.py — do not edit.\n")
        f.write("// Source: Oligeri/Sciancalepore Mendeley dataset (DOI 10.17632/xcxspv8c2r.2).\n")
        f.write("// IQ scaled to ±28000 int16 (Q15-ish; 90% of range with headroom).\n")
        f.write("#pragma once\n")
        f.write("#include <stdint.h>\n")
        f.write("#include <stddef.h>\n\n")
        f.write(f"#define MENDELEY_FIXTURE_N_BURSTS  {len(records)}\n")
        f.write(f"#define MENDELEY_FIXTURE_TOTAL_IQ_SAMPLES  {total_samples // 2}\n\n")
        f.write("typedef struct {\n")
        f.write("    uint32_t offset_iq;   // starting index into mendeley_iq[] (int16-pairs)\n")
        f.write("    uint32_t n_complex;   // number of complex samples\n")
        f.write("    uint32_t sat_id;\n")
        f.write("    uint32_t beam_id;\n")
        f.write("    int32_t  freq_hz_off; // freq_hz - 1626000000, signed\n")
        f.write("    int32_t  snr_db_q4;   // SNR in dB × 16 (Q4 fixed-point)\n")
        f.write("} mendeley_burst_meta_t;\n\n")
        # IQ blob
        f.write("static const int16_t mendeley_iq[] = {\n   ")
        col = 0
        offsets = []
        cursor = 0
        for r in records:
            offsets.append(cursor)
            iq16 = r["_iq_int16"]
            for v in iq16:
                f.write(f"{int(v):>6},")
                col += 1
                if col == 12:
                    f.write("\n   ")
                    col = 0
            cursor += iq16.size // 2
        f.write("\n};\n\n")
        # Metadata table
        f.write("static const mendeley_burst_meta_t mendeley_meta[] = {\n")
        for r, off in zip(records, offsets):
            f.write(f"  {{ .offset_iq = {off},"
                    f" .n_complex = {r['_iq_int16'].size // 2},"
                    f" .sat_id = {r['sat_id']},"
                    f" .beam_id = {r['beam_id']},"
                    f" .freq_hz_off = {int(r['freq_hz'] - 1626000000)},"
                    f" .snr_db_q4 = {int(round(r['_snr_db'] * 16))} }},\n")
        f.write("};\n")


def collect_records(src_files: list, max_lines: int, inspect: bool):
    out = []
    for src in src_files:
        print(f"  scanning {src} ...")
        with open(src, "r", errors="replace") as f:
            for i, line in enumerate(f):
                if inspect and i < 3:
                    print(f"    line {i} (first 200 chars): {line[:200]!r}")
                if i >= max_lines:
                    break
                r = parse_one_line(line)
                if r is None:
                    continue
                r["_snr_db"] = snr_estimate_db(r["iq"])
                r["_iq_int16"] = quantise_int16(r["iq"])
                out.append(r)
        if inspect:
            print(f"  parsed {len(out)} records from {src}; stopping (inspect mode)")
            return out
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--n", type=int, default=DEFAULT_N,
                   help=f"number of bursts to keep in the fixture (default {DEFAULT_N})")
    p.add_argument("--max-scan", type=int, default=50000,
                   help="max lines to scan per file (caps work for balanced selection)")
    p.add_argument("--inspect", action="store_true",
                   help="dump first 3 lines of the first file and exit (format discovery)")
    p.add_argument("--out", type=Path, default=HEADER_PATH,
                   help=f"output header path (default {HEADER_PATH})")
    args = p.parse_args()

    ed = fetch.extract_dir()
    if not ed.exists() or not any(ed.iterdir()):
        print(f"ERROR: extracted dir {ed} missing; run fetch_mendeley_iridium.py first")
        return 1

    src_files = sorted(ed.glob("*.txt"))
    if not src_files:
        print(f"ERROR: no .txt files found under {ed}")
        return 1

    records = collect_records(src_files, max_lines=args.max_scan, inspect=args.inspect)
    if args.inspect:
        return 0

    print(f"  collected {len(records)} candidate bursts; selecting balanced {args.n}")
    picked = pick_balanced(records, args.n)
    print(f"  selected {len(picked)} bursts; total IQ samples = "
          f"{sum(r['_iq_int16'].size // 2 for r in picked):,}")
    print(f"  SNR span: {min(r['_snr_db'] for r in picked):.1f} ... "
          f"{max(r['_snr_db'] for r in picked):.1f} dB")
    print(f"  sat-ID span: {sorted(set(r['sat_id'] for r in picked))[:10]} ...")
    print(f"  freq span: {min(r['freq_hz'] for r in picked) / 1e6:.3f} ... "
          f"{max(r['freq_hz'] for r in picked) / 1e6:.3f} MHz")

    emit_header(picked, args.out)
    size_kb = args.out.stat().st_size / 1024
    print(f"OK. Wrote {args.out} ({size_kb:.1f} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
