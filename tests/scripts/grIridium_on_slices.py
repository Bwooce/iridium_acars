#!/usr/bin/env python3
"""
Run gr-iridium's iridium-extractor on each uint8 fixture our host
tests / on-target smoke use. Reports per-slice decode count so we
can A/B against our pipeline on the same input.

For each fixture (e.g. fixture_albq_raw.h):
  1. Parse the C header to extract ALBQ_RAW_UINT8 (uint8 IQ array)
  2. Save as binary .cu8 file (= what iridium-extractor -f cu8 wants)
  3. Run iridium-extractor with the fixture's LO + 2.56 MSPS
  4. Report frame counts from the .bits / parsed output

Outputs land in /tmp/gr_iridium_slices/.
"""
import argparse
import re
import struct
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
FIXTURES = REPO / "tests/fixtures"
OUT_DIR = Path("/tmp/gr_iridium_slices")
OUT_DIR.mkdir(exist_ok=True)

# (header file, array name, length define, LO macro, LO value, expected_bursts)
# Each LO is read from the header at runtime.
SLICES = [
    ("fixture_albq_raw.h",     "ALBQ_RAW_UINT8",     "ALBQ_RAW_LO_HZ"),
    ("fixture_albq_stripe_0.h", "ALBQ_STRIPE_0_UINT8", "ALBQ_STRIPE_0_UINT8_LO_HZ"),
    ("fixture_albq_stripe_1.h", "ALBQ_STRIPE_1_UINT8", "ALBQ_STRIPE_1_UINT8_LO_HZ"),
    ("fixture_albq_stripe_2.h", "ALBQ_STRIPE_2_UINT8", "ALBQ_STRIPE_2_UINT8_LO_HZ"),
    ("fixture_albq_stripe_3.h", "ALBQ_STRIPE_3_UINT8", "ALBQ_STRIPE_3_UINT8_LO_HZ"),
    ("fixture_albq_stripe_4.h", "ALBQ_STRIPE_4_UINT8", "ALBQ_STRIPE_4_UINT8_LO_HZ"),
    ("fixture_albq_stripe_5.h", "ALBQ_STRIPE_5_UINT8", "ALBQ_STRIPE_5_UINT8_LO_HZ"),
    ("fixture_albq_stripe_6.h", "ALBQ_STRIPE_6_UINT8", "ALBQ_STRIPE_6_UINT8_LO_HZ"),
    ("fixture_albq_stripe_7.h", "ALBQ_STRIPE_7_UINT8", "ALBQ_STRIPE_7_UINT8_LO_HZ"),
]

SAMPLE_RATE_HZ = 2_560_000


def parse_uint8_array(header_path: Path, array_name: str) -> bytes:
    """Parse a uint8_t array out of a C header. The header has lines like:
        const uint8_t ALBQ_RAW_UINT8[49152] = {
          0xab, 0xcd, ...,
        };
    We grab everything between '= {' after the array name and the closing '};'."""
    text = header_path.read_text()
    # Find the array declaration line.
    pat = re.compile(rf"\b{re.escape(array_name)}\s*\[\s*\d+\s*\]\s*=\s*\{{(.*?)\}}\s*;",
                     re.DOTALL)
    m = pat.search(text)
    if not m:
        raise RuntimeError(f"array {array_name} not found in {header_path}")
    body = m.group(1)
    # Strip /* ... */ and // ... comments
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.DOTALL)
    body = re.sub(r"//[^\n]*", "", body)
    bytes_out = bytearray()
    for tok in body.replace(",", " ").split():
        if tok.startswith("0x") or tok.startswith("0X"):
            bytes_out.append(int(tok, 16))
        elif tok.isdigit():
            bytes_out.append(int(tok))
        else:
            # Skip anything else (shouldn't happen for clean fixtures).
            continue
    return bytes(bytes_out)


def parse_macro(header_path: Path, name: str) -> int:
    """Pull an integer #define out of a C header."""
    text = header_path.read_text()
    m = re.search(rf"#define\s+{re.escape(name)}\s+([\dxX_a-fA-F]+)u?", text)
    if not m:
        raise RuntimeError(f"#define {name} not found in {header_path}")
    val = m.group(1).rstrip("uU")
    return int(val, 0)


def run_iridium_extractor(cu8_path: Path, center_hz: int) -> dict:
    """Run iridium-extractor on a .cu8 file. iridium-extractor requires
    sample_rate divisible by 100000; 2.56 MSPS isn't. We resample the
    .cu8 to 2.5 MSPS (factor 125/128) using scipy before running."""
    import numpy as np
    from scipy.signal import resample_poly
    cu8 = np.fromfile(cu8_path, dtype=np.uint8)
    iq = (cu8.astype(np.float32) - 128.0) / 128.0
    cf = iq[0::2] + 1j * iq[1::2]
    cf_25 = resample_poly(cf, up=125, down=128).astype(np.complex64)
    cf32_path = cu8_path.with_suffix(".cf32")
    cf_25.tofile(cf32_path)

    cmd = [
        "iridium-extractor",
        "-c", str(center_hz),
        "-r", "2500000",
        "-f", "cf32_le",
        "--offline",
        str(cf32_path),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    stdout = res.stdout
    stderr = res.stderr
    # iridium-extractor writes RAW: lines for each decoded burst to stdout.
    bits_lines = [l for l in stdout.splitlines() if l.startswith("RAW:")]
    return {
        "rc": res.returncode,
        "n_bits": len(bits_lines),
        "stdout": stdout,
        "stderr": stderr,
        "bits": bits_lines,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true", help="dump stdout/stderr")
    ap.add_argument("--only", help="only process this header (e.g. fixture_albq_raw.h)")
    args = ap.parse_args()

    total_bits = 0
    print(f"{'slice':40s} {'LO MHz':>9s} {'bytes':>7s} {'decoded':>8s}")
    print("-" * 70)

    for header_name, array_name, lo_macro in SLICES:
        if args.only and args.only != header_name:
            continue
        header = FIXTURES / header_name
        if not header.exists():
            print(f"{header_name:40s} -- MISSING --")
            continue

        try:
            data = parse_uint8_array(header, array_name)
            lo_hz = parse_macro(header, lo_macro)
        except RuntimeError as e:
            print(f"{header_name:40s} parse error: {e}")
            continue

        cu8 = OUT_DIR / (header.stem + ".cu8")
        cu8.write_bytes(data)

        result = run_iridium_extractor(cu8, lo_hz)
        print(f"{header_name:40s} {lo_hz/1e6:>9.2f} {len(data):>7d} {result['n_bits']:>8d}")
        total_bits += result['n_bits']

        if args.verbose and result["n_bits"] > 0:
            print("  bits lines:")
            for line in result["bits"]:
                print(f"    {line[:120]}")

    print("-" * 70)
    print(f"Total decoded across all slices: {total_bits}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
