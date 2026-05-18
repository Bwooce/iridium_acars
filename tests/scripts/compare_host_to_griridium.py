"""
Compare the host pipeline test's burst detections against gr-iridium's
decoded-frame ground truth on the same ALBQ 1-sec fixture.

Produces a per-burst summary:
  - was the burst detected by host?
  - what did host's pipeline do with it (no-decode, no-UW, OK)?
  - what did gr-iridium produce (always OK, per its decoded set)?

This is the step-by-step diagnostic the user asked for: shows exactly
which gr-iridium-decodable bursts the host pipeline is losing, and at
which stage.
"""
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
LO_HZ = 1_618_500_000      # ALBQ_RAW_LO_HZ
CHANNEL_HZ = 40_000        # 2.56 MHz / 64 channels
M = 64


def gr_iridium_decoded():
    """Run gr-iridium on the slice; return list of (freq_hz, t_ms, snr_db)."""
    p = subprocess.run(
        ["python3", str(REPO / "tests/scripts/grIridium_on_slices.py"),
         "--only", "fixture_albq_raw.h", "--verbose"],
        capture_output=True, text=True, check=False, timeout=240)
    decoded = []
    for line in p.stdout.splitlines():
        # Format: "    RAW: fixture_albq_raw <t_ms> <freq_hz> N:<noise>-<sig> ..."
        m = re.match(
            r"\s*RAW:\s+\S+\s+(\d+\.\d+)\s+(\d+)\s+N:([\d.]+)-([\d.]+)", line)
        if not m:
            continue
        t_ms = float(m.group(1))
        freq = int(m.group(2))
        snr_lo = float(m.group(3))
        snr_hi = float(m.group(4))
        # gr-iridium reports N:lo-hi which we read as the noise floor
        # range; the signal SNR is (hi - lo). Convert for readability.
        decoded.append({
            "freq_hz": freq,
            "t_ms":    t_ms,
            "snr":     snr_hi - snr_lo,
        })
    return decoded


def host_bursts():
    """Run host pipeline test; return list of dicts per detected burst."""
    p = subprocess.run(
        [str(REPO / "build-host-ubsan/test_worker_pipeline_albq")],
        capture_output=True, text=True, check=False, timeout=60)
    bursts = []
    # Two output sections we need:
    #   1. "  burst N: ch=K (rel +X kHz) snr=Y dB len=Z" — detection
    #   2. "  K Y.Z Y2.Z ... qpsk-result" — per-burst pipeline
    detect_re = re.compile(
        r"^\s+burst\s+(\d+):\s+ch=(\d+)\s+\(rel\s+([+-]?\d+)\s+kHz\)\s+"
        r"snr=([\d.]+)\s+dB\s+len=(\d+)")
    # Pipeline result line uses fixed-width columns:
    #   "  62  20.6     7.6       UL       ω_pre=-0.611 ω_post=+0.005 654      no-UW"
    pipeline_re = re.compile(
        r"^\s+(\d+)\s+([\d.]+)\s+([\d.]+)\s+(\S+)\s+"
        r"ω_pre=([+-][\d.]+)\s+ω_post=([+-][\d.]+)\s+(\d+)\s+(\S+)")
    detections = {}
    pipeline_idx = 0
    for line in p.stdout.splitlines():
        m = detect_re.match(line)
        if m:
            i = int(m.group(1))
            detections[i] = {
                "ch":        int(m.group(2)),
                "rel_khz":   int(m.group(3)),
                "input_snr": float(m.group(4)),
                "len":       int(m.group(5)),
            }
            continue
        m = pipeline_re.match(line)
        if m:
            ch = int(m.group(1))
            # Match by order — pipeline lines come in same order as detections
            if pipeline_idx < len(detections):
                detections[pipeline_idx].update({
                    "uw_snr":  float(m.group(3)),
                    "uw_dir":  m.group(4),
                    "omega":   float(m.group(5)),
                    "uw_off":  int(m.group(6)),
                    "result":  m.group(7),
                })
                pipeline_idx += 1
    return list(detections.values())


def freq_to_channel(freq_hz):
    """Convert absolute frequency to (signed_channel, unsigned_channel)."""
    rel_hz = freq_hz - LO_HZ
    signed = round(rel_hz / CHANNEL_HZ)
    unsigned = signed if signed >= 0 else (M + signed)
    return signed, unsigned


def main():
    print("Running gr-iridium on fixture_albq_raw.h ...", file=sys.stderr)
    gri = gr_iridium_decoded()
    print(f"gr-iridium decoded {len(gri)} frames", file=sys.stderr)

    print("Running host pipeline test ...", file=sys.stderr)
    host = host_bursts()
    print(f"host detected {len(host)} bursts", file=sys.stderr)

    # Map each gr-iridium-decoded frame to its channelizer channel.
    gri_by_channel = {}
    for g in gri:
        signed, unsigned = freq_to_channel(g["freq_hz"])
        gri_by_channel.setdefault(unsigned, []).append(g)

    # Build host's channel distribution.
    host_by_channel = {}
    for h in host:
        host_by_channel.setdefault(h["ch"], []).append(h)

    # Aggregate per-channel.
    all_channels = sorted(set(gri_by_channel.keys()) | set(host_by_channel.keys()))
    print()
    print(f"{'ch':>3} {'rel_kHz':>9}  {'gr-iridium':>10}  {'host det':>9}  "
          f"{'host OK':>8}  {'host no-UW':>11}  {'host no-decode':>15}")
    print("-" * 80)
    host_ok_total = 0
    host_nouw_total = 0
    host_nodec_total = 0
    for ch in all_channels:
        gri_count = len(gri_by_channel.get(ch, []))
        host_list = host_by_channel.get(ch, [])
        host_det = len(host_list)
        host_ok = sum(1 for h in host_list if h.get("result") == "OK")
        host_nouw = sum(1 for h in host_list if h.get("result") == "no-UW")
        host_nodec = sum(1 for h in host_list if h.get("result") == "no-decode")
        host_ok_total += host_ok
        host_nouw_total += host_nouw
        host_nodec_total += host_nodec
        signed = ch if ch <= M / 2 else ch - M
        rel_khz = signed * CHANNEL_HZ // 1000
        # Highlight channels where gr-iridium decoded but host didn't
        marker = ""
        if gri_count > 0 and host_ok == 0:
            marker = "  ← gr-iridium decoded, host got nothing"
        print(f"{ch:>3} {rel_khz:>+8} k {gri_count:>10}  {host_det:>9}  "
              f"{host_ok:>8}  {host_nouw:>11}  {host_nodec:>15}{marker}")

    print()
    print(f"Totals: gr-iridium decoded {len(gri)}  |  host detected {len(host)}  "
          f"OK={host_ok_total} no-UW={host_nouw_total} no-decode={host_nodec_total}")


if __name__ == "__main__":
    main()
