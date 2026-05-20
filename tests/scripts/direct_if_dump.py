#!/usr/bin/env python3
"""Generate per-burst 250 ksps baseband cf32 dumps for the host
direct-IF test.

This mirrors gr-iridium's burst_downmix front end: for every burst
that iridium-extractor decoded from the fixture, we:

  1. Read its RAW line to get (timestamp_ms, abs_freq_hz).
  2. Compute start_sample at 2.5 MSPS and freq_offset = abs - LO.
  3. Slice the raw 2.5 MSPS IQ around the burst (1.6 ms pre, 16 ms
     post — same padding gr-iridium uses).
  4. Rotate the slice by -freq_offset so the carrier sits at DC.
  5. Low-pass + decimate 10x to 250 ksps (matches gr-iridium's
     burst_sample_rate).
  6. Save as interleaved cf32 to /tmp/host_direct_if/burst_<idx>.cf32.

Also writes a manifest CSV so the C test (test_pipeline_direct_if_albq)
knows which files to read and what metadata to print.

Why this exists: our host pipeline goes
    polyphase channelizer (40 ch * 40 kHz) -> resampler -> burst_pipeline
which loses 8 dB of SNR at the post-D13 stage and decodes 3/65 vs
gr-iridium's 65/65. This script lets us run the SAME burst_pipeline
on gr-iridium-equivalent baseband input, isolating whether the gap
is the front end (channelizer) or downstream (D13/CFO/RRC/UW/PLL).

Usage:
  python3 tests/scripts/direct_if_dump.py
  ./build-host/test_pipeline_direct_if_albq
"""
import csv
import re
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.signal import resample_poly, firwin, kaiserord

REPO = Path(__file__).resolve().parent.parent.parent
FIXTURES = REPO / "tests/fixtures"
OUT_DIR = Path("/tmp/host_direct_if")
OUT_DIR.mkdir(exist_ok=True)

FIXTURE_HDR = FIXTURES / "fixture_albq_raw.h"
ARRAY_NAME  = "ALBQ_RAW_UINT8"
LO_MACRO    = "ALBQ_RAW_LO_HZ"

# After resample_poly 125/128 the cu8 fixture is at 2.5 MSPS.
FS_RAW    = 2_500_000
FS_BB     = 250_000
DECIM     = FS_RAW // FS_BB   # = 10
# gr-iridium burst_downmix padding (from iridium_extractor_flowgraph.py):
#   burst_pre_len  = 2 * fft_size = 2 * 2048 = 4096 samples (~1.64 ms)
#   burst_post_len = input_sample_rate * 16e-3 = 40000 samples (16 ms)
#
# IMPORTANT: the RAW-line timestamp is NOT b.start. It is offset from
# b.start by (start_finder_cut + uw_start)/sample_rate — see
# burst_downmix_impl.cc:720. That offset varies per burst (typically
# 0.4-1.5 ms = 1000-3700 raw samples). PRE_SAMPLES_RAW must be larger
# than the typical offset so D13 can find the envelope rise within
# our window. 4096 samples (= gri's pre_len) is the gri-aligned
# choice; the per-burst offset adds variance D13 has to absorb.
PRE_SAMPLES_RAW  = 4096
POST_SAMPLES_RAW = 40000


def parse_uint8_array(header_path: Path, array_name: str) -> bytes:
    text = header_path.read_text()
    pat = re.compile(rf"\b{re.escape(array_name)}\s*\[\s*\d+\s*\]\s*=\s*\{{(.*?)\}}\s*;",
                     re.DOTALL)
    m = pat.search(text)
    if not m:
        raise RuntimeError(f"array {array_name} not found in {header_path}")
    body = m.group(1)
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.DOTALL)
    body = re.sub(r"//[^\n]*", "", body)
    bytes_out = bytearray()
    for tok in body.replace(",", " ").split():
        if tok.startswith("0x") or tok.startswith("0X"):
            bytes_out.append(int(tok, 16))
    return bytes(bytes_out)


def parse_macro(header_path: Path, name: str) -> int:
    text = header_path.read_text()
    m = re.search(rf"#define\s+{re.escape(name)}\s+([\dxX_a-fA-F]+)u?", text)
    if not m:
        raise RuntimeError(f"#define {name} not found in {header_path}")
    return int(m.group(1).rstrip("uU"), 0)


def load_raw_cf32() -> np.ndarray:
    """Return raw fixture as complex64 at 2.5 MSPS (post 125/128 resample)."""
    cu8 = np.frombuffer(parse_uint8_array(FIXTURE_HDR, ARRAY_NAME),
                        dtype=np.uint8)
    iq = (cu8.astype(np.float32) - 128.0) / 128.0
    cf = iq[0::2] + 1j * iq[1::2]
    cf_25 = resample_poly(cf, up=125, down=128).astype(np.complex64)
    return cf_25


def parse_raw_lines(stdout: str):
    """RAW: name timestamp_ms abs_freq_hz N:noise I:burst_id pct amp nbits bits

    The `I:` field is gr-iridium's internal burst ID — increments by 10
    per detection. This is what `iridium-extractor --debug-id N` filters
    on; debug-dump files are named `signal-<I>.cfile`. We MUST record it
    or stagewise compare will pair our bursts with the wrong gri files
    (path-C burst-index order ≠ gri-internal-id order).
    """
    tags = []
    for line in stdout.splitlines():
        if not line.startswith("RAW:"):
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            timestamp_ms = float(parts[2])
            abs_freq_hz  = int(parts[3])
        except ValueError:
            continue
        confidence_pct = None
        gri_burst_id   = -1
        for p in parts[4:]:
            if p.endswith("%"):
                try:
                    confidence_pct = int(p[:-1])
                except ValueError:
                    pass
            elif p.startswith("I:"):
                try:
                    gri_burst_id = int(p[2:])
                except ValueError:
                    pass
        tags.append({
            "timestamp_ms":   timestamp_ms,
            "abs_freq_hz":    abs_freq_hz,
            "confidence_pct": confidence_pct,
            "gri_burst_id":   gri_burst_id,
        })
    return tags


def run_iridium_extractor(cf32_path: Path, lo_hz: int) -> str:
    cmd = [
        "iridium-extractor",
        "-c", str(lo_hz),
        "-r", "2500000",
        "-f", "cf32_le",
        "--offline",
        str(cf32_path),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    return res.stdout


def direct_if_one_burst(raw_cf: np.ndarray, lo_hz: int, tag: dict) -> np.ndarray:
    """Rotate + decimate one tagged burst to 250 ksps baseband."""
    start_sample = int(round(tag["timestamp_ms"] * 1e-3 * FS_RAW))
    freq_offset  = tag["abs_freq_hz"] - lo_hz
    begin = max(0, start_sample - PRE_SAMPLES_RAW)
    end   = min(len(raw_cf), start_sample + POST_SAMPLES_RAW)
    slc   = raw_cf[begin:end]

    # Phase ramp aligned to the slice — the rotation must reference
    # *absolute* sample index, so the carrier estimate downstream
    # is what gr-iridium would see for the *aligned* burst.
    n_idx = np.arange(begin, end, dtype=np.float64)
    rot = np.exp(-2j * np.pi * freq_offset * n_idx / FS_RAW).astype(np.complex64)
    shifted = (slc * rot).astype(np.complex64)

    # 10x decimation with gr-iridium's EXACT input filter
    # (iridium_extractor_flowgraph.py:517):
    #   firdes.low_pass_2(gain=1, fs=2.5e6, cutoff=20kHz,
    #                     transition_width=40kHz, attenuation_dB=40)
    # → Kaiser β ≈ 3.4, 141 taps. scipy.signal.resample_poly defaults to
    # ~21-tap window — 40 dB at this cutoff/transition needs ~141 taps,
    # so the default leaks adjacent-burst content (>60 kHz away) by
    # only ~10-15 dB. On bursts where another Iridium burst lives near
    # our window, that leakage dominates the squared-FFT CFO step and
    # pushes omega to the ±2π clamp. Match gri exactly here.
    cutoff_hz   = 20_000           # gri's `cutoff_freq` arg
    trans_hz    = 40_000           # gri's `transition_width` arg
    atten_db    = 40
    ntaps, beta = kaiserord(atten_db, 2 * trans_hz / FS_RAW)
    if ntaps % 2 == 0: ntaps += 1
    fir         = firwin(ntaps, cutoff_hz, window=('kaiser', beta), fs=FS_RAW)
    bb = resample_poly(shifted, up=1, down=DECIM, window=fir).astype(np.complex64)
    return bb


def save_cf32(path: Path, x: np.ndarray):
    """Save complex64 as interleaved float32 IQ."""
    flat = np.empty(2 * len(x), dtype=np.float32)
    flat[0::2] = x.real
    flat[1::2] = x.imag
    flat.tofile(path)


def main():
    if not FIXTURE_HDR.exists():
        print(f"missing {FIXTURE_HDR}", file=sys.stderr)
        return 1

    lo_hz   = parse_macro(FIXTURE_HDR, LO_MACRO)
    raw_cf  = load_raw_cf32()
    print(f"loaded fixture: {len(raw_cf)} cf32 samples at {FS_RAW/1e6:.2f} MSPS, "
          f"LO={lo_hz/1e6:.2f} MHz", file=sys.stderr)

    # Write the cu8 used by iridium-extractor (cached if exists)
    cf32_dump = OUT_DIR / "fixture_albq_raw_2500k.cf32"
    save_cf32(cf32_dump, raw_cf)

    stdout = run_iridium_extractor(cf32_dump, lo_hz)
    tags   = parse_raw_lines(stdout)
    print(f"gr-iridium decoded {len(tags)} bursts", file=sys.stderr)

    # Per-burst direct-IF dump + manifest
    manifest_path = OUT_DIR / "manifest.csv"
    with open(manifest_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["burst_idx", "gri_burst_id", "timestamp_ms",
                    "abs_freq_hz", "freq_offset_hz", "n_samples_250k",
                    "confidence_pct", "filename"])
        for i, tag in enumerate(tags):
            bb   = direct_if_one_burst(raw_cf, lo_hz, tag)
            fname = f"burst_{i:03d}.cf32"
            save_cf32(OUT_DIR / fname, bb)
            w.writerow([i,
                        tag["gri_burst_id"],
                        f"{tag['timestamp_ms']:.4f}",
                        tag["abs_freq_hz"],
                        tag["abs_freq_hz"] - lo_hz,
                        len(bb),
                        tag["confidence_pct"] if tag["confidence_pct"] is not None else -1,
                        fname])
    print(f"wrote {len(tags)} burst cf32 files + {manifest_path}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
