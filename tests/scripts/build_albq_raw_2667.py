#!/usr/bin/env python3
"""
Generate a 2.667 MSPS variant of the Albuquerque raw-mode fixture
(fixture_albq_raw_2667.h) for testing task #48 / D7+ step 2: SDR
sample-rate change to 2.667 MHz so the M=64 channelizer's bins align
with Iridium's 41.667 kHz channel grid.

Reads the same 12 MSPS cf32 source as build_albq_fixture.py
(test_data/iridium_downlink_2022-03-17_albuquerque/iridium_cf32.sigmf-data;
decompress with derive.sh if not present) and emits a uint8 IQ
fixture at 2.667 MSPS using a clean rational resample (up=2, down=9
gives 12e6 × 2/9 = 2_666_666.67 Hz).

Same burst-list metadata as fixture_albq_raw.h so test_snr_gap_measurement
can compare per-burst SNR_dB against gr-iridium ground truth at the
new sample rate.
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import resample_poly

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
SRC_DIR = REPO_ROOT / "test_data" / "iridium_downlink_2022-03-17_albuquerque"
FIXTURE_DIR = REPO_ROOT / "tests" / "fixtures"

DATA_PATH = SRC_DIR / "iridium_cf32.sigmf-data"
META_PATH = SRC_DIR / "iridium_cf32.sigmf-meta"
BITS_PATH = SRC_DIR / "derivations" / "iridium.bits"

# Output rate for option B (task #48). 12e6 × 2/9 = 2_666_666.67 Hz.
TARGET_RATE = 2_666_667        # integer label; actual is 2_666_666.67

# Time window must match build_albq_fixture.py's raw-mode window so the
# burst content is identical at the new rate. That window is
# [burst_time - 1.0, burst_time + 8.6] ms around the highest-SNR DL
# burst (at 1151.8839 ms), giving 1150.88-1160.48 ms = 9.6 ms.
WIN_PRE_MS = 1.0
WIN_POST_MS = 8.6
ANCHOR_BURST_MS = 1151.8839     # same anchor as build_albq_fixture.py

# At 2.667 MHz: 9.6 ms = 25600 complex = 51200 bytes. Bigger than the
# 2.56 fixture's 49152 because the same time span needs more samples
# at the higher rate.
TARGET_BYTES = 0     # 0 = no truncation; sized to the actual data length

# Same LO as fixture_albq_raw.h so the burst list is identical.
LO_HZ = 1_618_500_000


def hex_array(bytes_obj, varname, per_line=16):
    lines = [f"const unsigned int {varname}_LEN = {len(bytes_obj)};",
             f"const uint8_t {varname}[{len(bytes_obj)}] = {{"]
    for i in range(0, len(bytes_obj), per_line):
        chunk = bytes_obj[i:i + per_line]
        hexstr = ",".join(f"0x{b:02x}" for b in chunk)
        lines.append(f"  {hexstr},")
    lines.append("};")
    return "\n".join(lines)


def main():
    if not DATA_PATH.exists():
        sys.exit(f"missing {DATA_PATH}; run derive.sh to decompress the .zst")

    meta = json.loads(META_PATH.read_text())
    src_rate = float(meta["global"]["core:sample_rate"])     # 12_000_000
    center_freq = float(meta["captures"][0]["core:frequency"])   # 1_621_500_000
    print(f"source: {src_rate/1e6:g} MSPS @ {center_freq/1e6:g} MHz")

    cf32 = np.fromfile(DATA_PATH, dtype=np.complex64)
    print(f"loaded {len(cf32)} cf32 samples = {len(cf32)/src_rate:.3f} s")

    # Match the time window of fixture_albq_raw.h. That file embeds
    # 49152 uint8 samples at 2.56 MSPS = 9.6 ms in the window
    # [anchor_burst - 1.0, anchor_burst + 8.6] ms.
    t_start_ms = ANCHOR_BURST_MS - WIN_PRE_MS
    t_end_ms = ANCHOR_BURST_MS + WIN_POST_MS
    s_start = int(round(t_start_ms / 1000.0 * src_rate))
    s_end = int(round(t_end_ms / 1000.0 * src_rate))
    slice_cf32 = cf32[s_start:s_end].astype(np.complex64)
    print(f"time window {t_start_ms:.1f}-{t_end_ms:.1f} ms "
          f"= {len(slice_cf32)} src samples")

    # Frequency-shift cf32 to bring the desired LO to baseband.
    shift = center_freq - LO_HZ
    n = np.arange(len(slice_cf32), dtype=np.float64)
    phasor = np.exp(-2j * np.pi * (-shift) / src_rate * n).astype(np.complex64)
    shifted = slice_cf32 * phasor

    # Resample 12 MSPS → 2.6667 MSPS with rational ratio 2/9.
    up, down = 2, 9
    dec = resample_poly(shifted, up=up, down=down).astype(np.complex64)
    print(f"resample up={up} down={down} → {len(dec)} samples @ {src_rate*up/down/1e6:.4f} MSPS")

    # Quantise to uint8 IQ (RTL-SDR convention).
    scale = 100.0 / max(0.5, float(np.max(np.abs(dec))))
    re = np.clip(np.real(dec) * scale + 128.5, 0, 255).astype(np.uint8)
    im = np.clip(np.imag(dec) * scale + 128.5, 0, 255).astype(np.uint8)
    bytes_iq = np.zeros(2 * len(dec), dtype=np.uint8)
    bytes_iq[0::2] = re
    bytes_iq[1::2] = im

    if TARGET_BYTES > 0:
        if len(bytes_iq) >= TARGET_BYTES:
            bytes_iq = bytes_iq[:TARGET_BYTES]
        else:
            pad = np.full(TARGET_BYTES - len(bytes_iq), 128, dtype=np.uint8)
            bytes_iq = np.concatenate([bytes_iq, pad])
    # Otherwise keep all samples (= 9.6 ms × 2.667 MSPS ≈ 51200 bytes)

    # Load the same burst list as fixture_albq_raw.h (gr-iridium ground
    # truth). At the new rate, channel = round(rel_hz / 41666.67) mod 64.
    bursts = []
    for line in BITS_PATH.read_text().splitlines():
        if not line.startswith("RAW:"):
            continue
        toks = line.split()
        try:
            time_ms = float(toks[2])
            freq_hz = int(toks[3])
            snr = float(toks[4].split(":")[1].split("+")[0])
            conf = int(toks[6].rstrip("%"))
        except (ValueError, IndexError):
            continue
        if not (t_start_ms <= time_ms <= t_end_ms):
            continue
        if abs(freq_hz - LO_HZ) > 1_280_000:
            continue
        bursts.append({"time_ms": time_ms, "freq_hz": freq_hz,
                       "snr": snr, "conf": conf})
    bursts.sort(key=lambda b: -b["snr"])
    bursts = bursts[:8]
    print(f"bursts in subband: {len(bursts)}")

    # Channel mapping at the NEW rate: spacing = 2666667/64 ≈ 41667 Hz
    # (exact Iridium grid match within RTL-SDR rate quantisation).
    out_path = FIXTURE_DIR / "fixture_albq_raw_2667.h"
    chan_spacing = 41666.67
    lines = [
        "// Auto-generated by tests/scripts/build_albq_raw_2667.py — do not edit by hand.",
        f"// Albuquerque cf32 @ LO={LO_HZ/1e6:.3f} MHz, resampled 12 → 2.6667 MSPS",
        f"// (clean rational ratio 2/9: 12e6 × 2/9 = 2_666_666.67 Hz).",
        "// Used by tests/host/test_snr_gap_measurement to validate that",
        "// channelizer Iridium-grid alignment (task #48 option B) recovers",
        "// the ~3 dB loss measured at 2.56 MSPS.",
        "#pragma once",
        "#include <stdint.h>",
        '#include "channelizer_burst_ref.h"',
        "",
        f"#define ALBQ_RAW_2667_LO_HZ           {LO_HZ}u",
        f"#define ALBQ_RAW_2667_SAMPLE_RATE_HZ  {TARGET_RATE}u",
        f"#define ALBQ_RAW_2667_EXPECTED_BURSTS {len(bursts)}",
        "",
        f"// Bursts gr-iridium detected in this fixture's subband, sorted by SNR desc.",
        f"// channel = round(rel_hz / 41666.67) mod 64 (Iridium grid match).",
        f"static const channelizer_burst_ref_t ALBQ_RAW_2667_BURSTS[] = {{",
    ]
    for b in bursts:
        rel_hz = b["freq_hz"] - LO_HZ
        ch = round(rel_hz / chan_spacing)
        if ch < 0:
            ch += 64
        lines.append(
            f"    {{ {rel_hz:+d}, {ch}u, {b['conf']}u, {b['snr']:.2f}f, "
            f"{b['time_ms']:.4f}f }},"
        )
    lines.append("};")
    lines.append("")
    lines.append(hex_array(bytes_iq, "ALBQ_RAW_2667_UINT8"))
    out_path.write_text("\n".join(lines) + "\n")
    print(f"wrote {out_path} ({len(bytes_iq)} uint8 samples)")


if __name__ == "__main__":
    main()
