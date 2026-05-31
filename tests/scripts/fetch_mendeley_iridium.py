#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
fetch_mendeley_iridium.py — pull and stage the Oligeri/Sciancalepore
Iridium dataset (10.17632/xcxspv8c2r v2) into the local IQ cache.

Dataset characteristics (per the paper at PMC9868370):
  - Single zip: 4.67 GB
  - Decompresses to two text files:
      1208-1009_20_parsed.txt   (7.5 GB)
      1109-0910_20_parsed.txt   (6.7 GB)
  - Each line = one IRA (Ring Alert) packet:
      timestamp, sat_id, beam_id, lat, lon, alt, confidence, freq, IQ-cell
  - IQ-cell is MATLAB cell-array syntax: {'(a1+jb1), (a2+jb2), ...'}
    Approximately 2000 complex samples per packet at the channelizer
    output rate (already burst-extracted by gr-iridium).

Usage:
    python3 tests/scripts/fetch_mendeley_iridium.py            # download + decompress
    python3 tests/scripts/fetch_mendeley_iridium.py --check    # report cache status, no I/O
    IQ_CACHE_ROOT=/elsewhere python3 tests/scripts/fetch_mendeley_iridium.py
"""

import argparse
import hashlib
import os
import sys
import time
import urllib.request
import zipfile
from pathlib import Path

DATASET_DOI = "10.17632/xcxspv8c2r.2"
ZIP_URL = ("https://data.mendeley.com/public-files/datasets/xcxspv8c2r/"
           "files/7d39bf61-d67a-4327-b76d-ec7a61e6cd01/file_downloaded")
ZIP_SHA256 = "60b1001686067eb351d1e369a05360449e707062012432091460b09632b369e5"
ZIP_SIZE = 4674824309   # 4.67 GB, exact

DEFAULT_CACHE = Path.home() / "iq_cache"


def cache_root() -> Path:
    env = os.environ.get("IQ_CACHE_ROOT")
    return Path(env) if env else DEFAULT_CACHE


def zip_path() -> Path:
    return cache_root() / "mendeley_iridium.zip"


def extract_dir() -> Path:
    return cache_root() / "mendeley_extracted"


def fmt_bytes(n: int) -> str:
    for unit in "B", "KB", "MB", "GB":
        if n < 1024:
            return f"{n:.1f} {unit}"
        n /= 1024
    return f"{n:.1f} TB"


def report_status() -> None:
    root = cache_root()
    zp = zip_path()
    ed = extract_dir()
    print(f"cache root:      {root}")
    print(f"zip ({fmt_bytes(ZIP_SIZE)}): "
          f"{'present' if zp.exists() else 'absent'}"
          f"{' (' + fmt_bytes(zp.stat().st_size) + ')' if zp.exists() else ''}")
    if ed.exists():
        for f in sorted(ed.iterdir()):
            print(f"extracted:       {f.name}  ({fmt_bytes(f.stat().st_size)})")
    else:
        print(f"extracted dir:   {ed} (absent)")


def verify_zip(path: Path) -> bool:
    """Stream-hash the zip; True iff sha256 matches the published value."""
    print(f"  verifying sha256 of {path} ...")
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            chunk = f.read(1 << 20)
            if not chunk:
                break
            h.update(chunk)
    got = h.hexdigest()
    if got != ZIP_SHA256:
        print(f"  HASH MISMATCH: got {got}, expected {ZIP_SHA256}")
        return False
    print("  hash OK")
    return True


def download_zip() -> None:
    """Resume-capable download. Streams in 1 MB chunks with progress."""
    zp = zip_path()
    zp.parent.mkdir(parents=True, exist_ok=True)
    if zp.exists() and zp.stat().st_size == ZIP_SIZE:
        print(f"  zip already present at full size ({fmt_bytes(ZIP_SIZE)})")
        return
    start = zp.stat().st_size if zp.exists() else 0
    if start > ZIP_SIZE:
        print("  cached zip is larger than expected — re-downloading")
        zp.unlink()
        start = 0
    headers = {"Range": f"bytes={start}-"} if start else {}
    print(f"  GET {ZIP_URL}")
    print(f"  resume from {fmt_bytes(start)} / {fmt_bytes(ZIP_SIZE)}")
    req = urllib.request.Request(ZIP_URL, headers=headers)
    t0 = time.time()
    written = start
    with urllib.request.urlopen(req, timeout=60) as resp, \
            open(zp, "ab" if start else "wb") as out:
        while True:
            chunk = resp.read(1 << 20)
            if not chunk:
                break
            out.write(chunk)
            written += len(chunk)
            dt = max(time.time() - t0, 0.001)
            rate_mbps = (written - start) * 8 / 1e6 / dt
            print(f"\r  {fmt_bytes(written)} / {fmt_bytes(ZIP_SIZE)}"
                  f"  ({100.0 * written / ZIP_SIZE:.1f}%, {rate_mbps:.1f} Mbps)",
                  end="", flush=True)
    print()


def extract_zip() -> None:
    """Decompress the zip into extract_dir/ idempotently."""
    zp = zip_path()
    ed = extract_dir()
    if ed.exists() and any(ed.iterdir()):
        print(f"  already extracted in {ed}")
        return
    ed.mkdir(parents=True, exist_ok=True)
    print(f"  unzipping {zp} -> {ed}")
    with zipfile.ZipFile(zp, "r") as z:
        z.extractall(ed)
    for f in sorted(ed.iterdir()):
        print(f"  extracted: {f.name}  ({fmt_bytes(f.stat().st_size)})")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--check", action="store_true",
                   help="report cache status only; no download or extract")
    p.add_argument("--skip-verify", action="store_true",
                   help="skip sha256 verification after download (faster but unsafe)")
    args = p.parse_args()

    if args.check:
        report_status()
        return 0

    cache_root().mkdir(parents=True, exist_ok=True)
    download_zip()

    if not args.skip_verify:
        if not verify_zip(zip_path()):
            print("ERROR: zip failed verification; refusing to extract")
            return 1

    extract_zip()
    report_status()
    print()
    print(f"OK. Next step: python3 tests/scripts/build_mendeley_fixture.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
