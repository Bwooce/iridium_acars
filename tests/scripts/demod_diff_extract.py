#!/usr/bin/env python3
"""demod_diff_extract.py — per-burst window extractor for the
gr-iridium demod-diff harness (tests/host/test_demod_diff_gri.c).

Purpose: quantify WHERE our on-device demod chain loses frames that
gr-iridium decodes, using a wideband HydraSDR capture as ground truth.

For every burst line in an iridium-parser `-o line` file we:
  1. Parse (time_ms, abs_freq_hz, snr_db, type, crc_ok).
  2. Seek the raw ci16 capture (int16 interleaved IQ) at
     start_sample = time_ms*1e-3*FS — the file is memmapped, never
     loaded whole (7.2 GB).
  3. Slice [start - PRE_RAW, start + POST_RAW) — the same 1.64 ms /
     16 ms padding gr-iridium's burst_downmix uses and the same
     geometry tests/scripts/direct_if_dump.py produces, so after
     decimation the gri-tagged position lands at iq250[~410] just
     like the ALBQ path-C fixtures.
  4. Rotate by -(abs_freq - center) with EXACT integer phase math
     (absolute sample index * f_off mod fs — float64 phase would lose
     precision at sample indices ~1.8e9).
  5. LPF + decimate to 250 ksps with gr-iridium's input filter
     (low_pass_2 cutoff=20 kHz, transition=40 kHz, 40 dB — the same
     Kaiser design direct_if_dump.py uses, re-derived at this fs).
  6. Save interleaved int16 (sc16) in the ORIGINAL capture count
     units (no per-burst rescaling — that would erase the SNR
     differences the downstream chain depends on).

Also writes manifest.csv for the C harness.

Usage:
  /usr/bin/python3 tests/scripts/demod_diff_extract.py \
      --parsed <file.parsed> --ci16 <file.ci16> --out <dir> \
      [--fs 10000000] [--center 1622000000] [--types IDA,ISY,...]

The default python3 on this Mac lacks numpy; /usr/bin/python3 has
numpy+scipy.
"""
import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np
from scipy.signal import firwin, kaiserord, resample_poly

# gr-iridium burst_downmix padding, scaled to the raw sample rate at
# extraction time (see direct_if_dump.py for the derivation):
#   pre  = 1.6384 ms  (= 4096 samples at 2.5 MSPS = 16384 at 10 MSPS)
#   post = 16 ms      (= 160000 samples at 10 MSPS)
# After decimation to 250 ksps: pre = 409.6 samples, total ~4410.
PRE_S = 4096 / 2.5e6  # 1.6384 ms
POST_S = 16e-3
FS_BB = 250_000

LINE_RE = re.compile(
    r"^([A-Z0-9]{3}):\s+\S+\s+(\d+\.\d+)\s+(\d+)\s+(\d+)%\s+"
    r"(-?[\d.]+)\|(-?[\d.]+)\|(-?[\d.]+)"
)
IDA_HDR_RE = re.compile(r"cont=(\d) \d ctr=([01]{3}) [01]{3} len=(\d+)")


def parse_line(line):
    m = LINE_RE.match(line)
    if not m:
        return None
    typ = m.group(1)
    ent = {
        "type": typ,
        "time_ms": float(m.group(2)),
        "abs_freq_hz": int(m.group(3)),
        "conf_pct": int(m.group(4)),
        "level_db": float(m.group(5)),
        "noise_db": float(m.group(6)),
        "snr_db": float(m.group(7)),
        "gri_crc_ok": 1 if "CRC:OK" in line else 0,
        "gri_cont": -1,
        "gri_ctr": -1,
        "gri_len": -1,
    }
    if typ == "IDA":
        h = IDA_HDR_RE.search(line)
        if h:
            ent["gri_cont"] = int(h.group(1))
            ent["gri_ctr"] = int(h.group(2), 2)
            ent["gri_len"] = int(h.group(3))
    return ent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--parsed", required=True)
    ap.add_argument("--ci16", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--fs", type=int, default=10_000_000)
    ap.add_argument("--center", type=int, default=1_622_000_000)
    ap.add_argument("--tag-offset-ms", type=float, default=0.0,
                    help="constant correction added to every parsed "
                    "timestamp before seeking the ci16. The parser's "
                    "sample clock can be skewed from the raw file "
                    "(e.g. samples dropped before the recorder "
                    "started). Measure it by locating a few strong "
                    "bursts' leading edges vs their tags; for "
                    "ACARS_SLICE_20260717_164657 it is +25.79 ms "
                    "(constant over the whole 180 s, stdev 0.02 ms).")
    ap.add_argument("--types", default="",
                    help="comma-separated type filter (default: all)")
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    fs = args.fs
    decim = fs // FS_BB
    assert fs % FS_BB == 0, "fs must be an integer multiple of 250 kHz"
    pre_raw = int(round(PRE_S * fs))
    post_raw = int(round(POST_S * fs))

    # gr-iridium input filter: firdes.low_pass_2(1, fs, 20e3, 40e3, 40 dB)
    ntaps, beta = kaiserord(40, 2 * 40_000 / fs)
    if ntaps % 2 == 0:
        ntaps += 1
    fir = firwin(ntaps, 20_000, window=("kaiser", beta), fs=fs)

    typefilter = set(t for t in args.types.split(",") if t)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    mm = np.memmap(args.ci16, dtype=np.int16, mode="r")
    n_total = len(mm) // 2
    print(f"capture: {n_total} complex samples = {n_total/fs:.1f} s, "
          f"fs={fs/1e6:.1f} MSPS, decim={decim}, fir={ntaps} taps",
          file=sys.stderr)

    entries = []
    with open(args.parsed) as fh:
        for line in fh:
            ent = parse_line(line)
            if ent is None:
                continue
            if typefilter and ent["type"] not in typefilter:
                continue
            entries.append(ent)
            if args.limit and len(entries) >= args.limit:
                break
    print(f"parsed {len(entries)} burst lines", file=sys.stderr)

    clip_bursts = 0
    manifest = out_dir / "manifest.csv"
    with open(manifest, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["idx", "type", "time_ms", "abs_freq_hz", "off_hz",
                    "snr_db", "conf_pct", "gri_crc_ok", "gri_cont",
                    "gri_ctr", "gri_len", "n_complex", "filename"])
        for i, ent in enumerate(entries):
            start = int(round((ent["time_ms"] + args.tag_offset_ms) * 1e-3 * fs))
            f_off = ent["abs_freq_hz"] - args.center
            begin = max(0, start - pre_raw)
            end = min(n_total, start + post_raw)
            if end - begin < decim * 64 or abs(f_off) > fs // 2 - 60_000:
                w.writerow([i, ent["type"], f"{ent['time_ms']:.4f}",
                            ent["abs_freq_hz"], f_off,
                            f"{ent['snr_db']:.2f}", ent["conf_pct"],
                            ent["gri_crc_ok"], ent["gri_cont"],
                            ent["gri_ctr"], ent["gri_len"], 0, "SKIP"])
                continue
            raw = np.asarray(mm[2 * begin:2 * end], dtype=np.float32)
            cf = raw[0::2] + 1j * raw[1::2]  # original int16 count units

            # Exact-integer phase: frac(n*f_off/fs) with int64 math.
            n_idx = np.arange(begin, end, dtype=np.int64)
            ph_num = (n_idx * np.int64(f_off)) % np.int64(fs)
            rot = np.exp(-2j * np.pi * (ph_num.astype(np.float64) / fs))
            shifted = (cf * rot).astype(np.complex64)

            bb = resample_poly(shifted, up=1, down=decim,
                               window=fir).astype(np.complex64)

            flat = np.empty(2 * len(bb), dtype=np.float32)
            flat[0::2] = bb.real
            flat[1::2] = bb.imag
            n_clip = int(np.count_nonzero(np.abs(flat) > 32767.0))
            if n_clip:
                clip_bursts += 1
            sc16 = np.clip(np.rint(flat), -32768, 32767).astype(np.int16)
            fname = f"burst_{i:05d}.sc16"
            sc16.tofile(out_dir / fname)
            w.writerow([i, ent["type"], f"{ent['time_ms']:.4f}",
                        ent["abs_freq_hz"], f_off,
                        f"{ent['snr_db']:.2f}", ent["conf_pct"],
                        ent["gri_crc_ok"], ent["gri_cont"],
                        ent["gri_ctr"], ent["gri_len"], len(bb), fname])
            if (i + 1) % 500 == 0:
                print(f"  {i+1}/{len(entries)}", file=sys.stderr)

    print(f"wrote {len(entries)} windows + {manifest} "
          f"(clipped bursts: {clip_bursts})", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
