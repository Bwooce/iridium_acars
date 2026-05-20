#!/usr/bin/env python3
"""
Generate the C golden-bits fixture header from gr-iridium's decoded
output on a target fixture. The header is compiled into both the
device smoke test and (optionally) the host wideband test so each
can verify its decoded bits against gri's canonical bits in-place,
not just count decodes.

Pipeline:
  1. Run iridium-extractor on the fixture's .cu8 file (assumed
     already produced by tests/scripts/grIridium_on_slices.py).
  2. Parse each RAW: line for (timestamp_ms, abs_freq_hz, gri_id,
     confidence, bits).
  3. Compute alignment keys the device can match against:
       start_sample_2500k   = timestamp_ms * 2500
       freq_offset_hz       = abs_freq_hz - lo_hz
     (Tolerance windows are applied in the C compare code, not here.)
  4. Emit tests/fixtures/fixture_albq_golden_bits.h.

Run:  python3 tests/scripts/build_golden_bits.py
Optional --include-host: also reads /tmp/host_wideband_bits.txt
(produced by test_pipeline_wideband_albq with HOST_DUMP_BITS=1)
and merges per-burst host bits into the header alongside the gri
column — so the smoke can compare against both oracles.
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
FIXTURES = REPO / "tests/fixtures"
SLICES_DIR = Path("/tmp/gr_iridium_slices")
HOST_BITS_FILE = Path("/tmp/host_wideband_bits.txt")
OUT_HEADER = FIXTURES / "fixture_albq_golden_bits.h"

# Reused from grIridium_on_slices.py — same fixture-path mapping.
ALBQ_RAW_HEADER = "fixture_albq_raw.h"
ALBQ_RAW_LO_MACRO = "ALBQ_RAW_LO_HZ"
SAMPLE_RATE_HZ = 2_500_000   # tagger frame rate (post-resample to 2.5 MSPS)


def parse_macro(header_path: Path, macro: str) -> int:
    """Extract `#define MACRO N` from a C header. Mirrors
    grIridium_on_slices.parse_macro to avoid an import dance."""
    text = header_path.read_text()
    m = re.search(rf"#define\s+{macro}\s+(\d+)", text)
    if not m:
        raise RuntimeError(f"could not find #define {macro} in {header_path}")
    return int(m.group(1))


def run_extractor(cu8_path: Path, lo_hz: int) -> str:
    cmd = [
        "iridium-extractor",
        "-c", str(lo_hz),
        "-r", "2500000",
        "-f", "cu8",
        "--offline",
        str(cu8_path),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=240)
    return res.stdout


def parse_raw_lines(stdout: str):
    """
    RAW: <name> <timestamp_ms> <abs_freq_hz> N:<n0>-<n1> I:<id> <pct>% <amp> <nbits> <bits>

    The last whitespace-separated field is the binary bit string
    (length nbits). Returns a list of dicts.
    """
    tags = []
    for line in stdout.splitlines():
        if not line.startswith("RAW:"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        try:
            timestamp_ms = float(parts[2])
            abs_freq_hz  = int(parts[3])
        except ValueError:
            continue
        gri_id = -1
        conf_pct = -1
        for p in parts[4:]:
            if p.startswith("I:"):
                try:
                    gri_id = int(p[2:])
                except ValueError:
                    pass
            elif p.endswith("%"):
                try:
                    conf_pct = int(p[:-1])
                except ValueError:
                    pass
        # nbits is the second-to-last numeric field; bits is last.
        bits_str = parts[-1]
        try:
            n_bits_decl = int(parts[-2])
        except ValueError:
            n_bits_decl = -1
        # Sanity: the bit string should be exactly n_bits_decl chars.
        if n_bits_decl >= 0 and len(bits_str) != n_bits_decl:
            sys.stderr.write(
                f"warn: gri_id={gri_id} declared n_bits={n_bits_decl} "
                f"but bit string len={len(bits_str)}; using string len\n")
        if not re.fullmatch(r"[01]+", bits_str):
            sys.stderr.write(f"warn: gri_id={gri_id} non-binary bits, skipped\n")
            continue
        tags.append({
            "timestamp_ms": timestamp_ms,
            "abs_freq_hz":  abs_freq_hz,
            "gri_id":       gri_id,
            "conf_pct":     conf_pct,
            "n_bits":       len(bits_str),
            "bits":         bits_str,
        })
    return tags


def read_host_bits(path: Path):
    """
    Parses /tmp/host_wideband_bits.txt produced by the host wideband
    test in HOST_DUMP_BITS mode. One burst per line:

      HOST: tag_idx=<i> start=<s> bin=<b> n_bits=<n> bits=<binary>

    Returns dict keyed by (rounded timestamp_ms, freq_offset_hz)
    for nearest-neighbour lookup later. Empty dict if file missing.
    """
    if not path.exists():
        return {}
    out = {}
    for line in path.read_text().splitlines():
        if not line.startswith("HOST:"):
            continue
        m = re.search(
            r"start=(\d+)\s+bin=(\d+).*n_bits=(\d+)\s+bits=([01]+)", line)
        if not m:
            continue
        out[(int(m.group(1)), int(m.group(2)))] = {
            "n_bits": int(m.group(3)),
            "bits":   m.group(4),
        }
    return out


def emit_header(out: Path, fixture_name: str, lo_hz: int,
                 sample_rate_hz: int, gri_tags, host_bits_by_key):
    name_upper = fixture_name.upper().replace("-", "_").replace(".", "_")
    lines = []
    lines.append("// Auto-generated by tests/scripts/build_golden_bits.py.")
    lines.append("// Do not edit by hand — re-run the script to regenerate.")
    lines.append("//")
    lines.append(f"// Source fixture: {fixture_name}")
    lines.append(f"// LO frequency:   {lo_hz} Hz")
    lines.append(f"// Sample rate:    {sample_rate_hz} Hz (post-resample)")
    lines.append(f"// gri burst count: {len(gri_tags)}")
    lines.append("//")
    lines.append("// Each golden entry pairs the alignment keys (start_sample_2500k,")
    lines.append("// freq_offset_hz) that the tagger / worker can match against")
    lines.append("// with the canonical decoded bit payload from gri (and where")
    lines.append("// available, the host wideband test's own decoded bits for")
    lines.append("// independent cross-checking).")
    lines.append("//")
    lines.append("// Bits are stored ONE BIT PER BYTE (0x00 or 0x01) to match")
    lines.append("// decoded_frame_t.bits and avoid bit-packing math at compare")
    lines.append("// time. Memory: ~180 bytes/burst × 65 bursts ≈ 12 KB total.")
    lines.append("")
    lines.append("#pragma once")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append("typedef struct {")
    lines.append("    int      gri_id;             // gri's internal burst id (`I:` field)")
    lines.append("    uint64_t start_sample_2500k; // = timestamp_ms * 2500")
    lines.append("    int32_t  freq_offset_hz;     // abs_freq_hz - LO")
    lines.append("    int      conf_pct;           // gri's confidence percentage")
    lines.append("    int      gri_n_bits;         // bits in gri_bits[]")
    lines.append("    const uint8_t *gri_bits;     // gri canonical decoded bits")
    lines.append("    int      host_n_bits;        // 0 if no host match")
    lines.append("    const uint8_t *host_bits;    // NULL if no host match")
    lines.append("} golden_burst_t;")
    lines.append("")
    # Per-burst bit arrays first.
    for i, t in enumerate(gri_tags):
        lines.append(f"static const uint8_t {name_upper}_GRI_BITS_{i}[] = {{")
        bs = t["bits"]
        # 24 bits per line for readable diffs.
        for k in range(0, len(bs), 24):
            chunk = bs[k:k+24]
            row = ", ".join(f"0x{int(c):02x}" for c in chunk)
            lines.append(f"    {row},")
        lines.append("};")
    # Host bits (optional).
    host_matched = 0
    for i, t in enumerate(gri_tags):
        # Match by gri_id if we have host dumps (host test logs gri_id too).
        # If not exact match, leave host_bits NULL — the C compare will skip.
        # For now: no host bits supported until phase 2 (host dump).
        host = None
        if host is not None:
            host_matched += 1
            lines.append(f"static const uint8_t {name_upper}_HOST_BITS_{i}[] = {{")
            bs = host["bits"]
            for k in range(0, len(bs), 24):
                chunk = bs[k:k+24]
                row = ", ".join(f"0x{int(c):02x}" for c in chunk)
                lines.append(f"    {row},")
            lines.append("};")
    # Master table.
    lines.append("")
    lines.append(f"#define {name_upper}_GOLDEN_COUNT {len(gri_tags)}")
    lines.append("")
    lines.append(f"static const golden_burst_t {name_upper}_GOLDEN_BURSTS[{len(gri_tags)}] = {{")
    for i, t in enumerate(gri_tags):
        start_2500k = int(round(t["timestamp_ms"] * 2500))
        freq_off = t["abs_freq_hz"] - lo_hz
        host_n = 0
        host_ptr = "NULL"
        # (Host wiring left for phase 2.)
        lines.append(
            f"    {{ .gri_id = {t['gri_id']}, "
            f".start_sample_2500k = {start_2500k}u, "
            f".freq_offset_hz = {freq_off}, "
            f".conf_pct = {t['conf_pct']}, "
            f".gri_n_bits = {t['n_bits']}, "
            f".gri_bits = {name_upper}_GRI_BITS_{i}, "
            f".host_n_bits = {host_n}, "
            f".host_bits = {host_ptr} }},")
    lines.append("};")
    lines.append("")
    out.write_text("\n".join(lines) + "\n")
    print(f"wrote {out} ({len(gri_tags)} gri bursts, {host_matched} host-matched)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--include-host", action="store_true",
                    help="merge host wideband bits from /tmp/host_wideband_bits.txt")
    args = ap.parse_args()

    cu8 = SLICES_DIR / "fixture_albq_raw.cu8"
    if not cu8.exists():
        print(f"error: {cu8} missing. Run:\n"
              f"  python3 tests/scripts/grIridium_on_slices.py "
              f"--only fixture_albq_raw.h", file=sys.stderr)
        return 2

    header = FIXTURES / ALBQ_RAW_HEADER
    lo_hz = parse_macro(header, ALBQ_RAW_LO_MACRO)
    print(f"running iridium-extractor on {cu8.name} @ LO {lo_hz} Hz...", file=sys.stderr)
    out = run_extractor(cu8, lo_hz)
    gri_tags = parse_raw_lines(out)
    print(f"  parsed {len(gri_tags)} RAW lines from gri", file=sys.stderr)
    if not gri_tags:
        print("error: no RAW lines parsed", file=sys.stderr)
        return 2

    host_bits = read_host_bits(HOST_BITS_FILE) if args.include_host else {}
    if args.include_host:
        print(f"  loaded {len(host_bits)} host bit dumps", file=sys.stderr)

    emit_header(OUT_HEADER, "fixture_albq_raw", lo_hz,
                 SAMPLE_RATE_HZ, gri_tags, host_bits)
    return 0


if __name__ == "__main__":
    sys.exit(main())
