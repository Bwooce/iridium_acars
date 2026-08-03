#!/usr/bin/env python3
# Regenerate the POA golden-replay smoke fixture (CONFIG_SMOKE_TEST_POA).
#
# Extracts a short slice of the HydraSDR POA golden capture around the
# JQ0404/VH-VGD burst and downscales int16 -> int8 (top 8 bits, matching the
# RTL cu8 precision the device actually feeds). The device smoke upscales each
# int8 back to int16 (<<8) before the channelizer, so this is byte-faithful to
# a real 8-bit-origin sample stream.
#
# The .bin is intentionally NOT committed (tests/fixtures/*.bin is gitignored,
# like ~/iridium_capture/*_ref/ goldens). Generate it locally before building
# the POA smoke:  python3 tests/fixtures/make_poa_slice.py
#
# Source capture (out-of-git, ~5.6 GB):
#   ~/iridium_capture/poa_ref/poa_2500k_130800_10min_s16.raw
# Slice: start=35,400,000 complex samples, count=500,000 (0.20 s @ 2.5 MSPS).
# Verified minimal window that still decodes JQ0404 (host test_poa_frontend
# geometry + the acarsdec oracle).

import os
import sys
import numpy as np

SRC   = os.path.expanduser("~/iridium_capture/poa_ref/poa_2500k_130800_10min_s16.raw")
OUT   = os.path.join(os.path.dirname(__file__), "poa_jq0404_slice_i8.bin")
START = 35_400_000   # complex samples
COUNT = 500_000      # complex samples (0.20 s @ 2.5 MSPS)

def main():
    if not os.path.exists(SRC):
        sys.exit(f"source golden not found: {SRC}\n"
                 "(the 5.6 GB HydraSDR POA capture lives out-of-git under "
                 "~/iridium_capture/poa_ref/)")
    # int16 interleaved I/Q; 2 int16 per complex sample.
    off = START * 2
    n   = COUNT * 2
    a   = np.fromfile(SRC, dtype=np.int16, count=n, offset=off * 2)  # offset in bytes
    if a.size != n:
        sys.exit(f"short read: got {a.size} int16, want {n}")
    i8 = (a >> 8).astype(np.int8)  # top 8 bits == device cu8 precision
    i8.tofile(OUT)
    print(f"wrote {OUT}: {i8.size} bytes int8 ({i8.size // 2} complex, "
          f"{COUNT / 2_500_000:.3f} s), absmax={int(np.abs(i8).max())}")

if __name__ == "__main__":
    main()
