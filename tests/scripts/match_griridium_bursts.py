"""
Match each gr-iridium-decoded burst to host's nearest detected burst,
classifying every gr-iridium burst as one of:

  MISSED       host detected nothing within (Δt < 5 ms, Δch ≤ 1)
  DETECTED     host detected but qpsk_demod gave no-decode or no-UW
  DECODED      host detected AND qpsk_demod returned OK

Reports per-burst, then totals.

Tight tolerance (5 ms, ±1 channel) because gr-iridium's wideband
fft_burst_tagger and our polyphase channelizer can disagree slightly
on burst start time and which 40 kHz bin a burst lands in.
"""
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent.parent
LO_HZ = 1_618_500_000
FS_HZ = 2_560_000              # input sample rate
CHANNEL_HZ = 40_000            # = FS / M
M = 64
TIME_TOL_SAMPLES = int(0.005 * FS_HZ)   # 5 ms = 12 800 samples
CHANNEL_TOL = 1                # ±1 channel


def gri_decoded():
    """Run gr-iridium, return list of (t_samples, freq_hz, channel_unsigned)."""
    p = subprocess.run(
        ["python3", str(REPO / "tests/scripts/grIridium_on_slices.py"),
         "--only", "fixture_albq_raw.h", "--verbose"],
        capture_output=True, text=True, check=False, timeout=240)
    out = []
    for line in p.stdout.splitlines():
        m = re.match(r"\s*RAW:\s+\S+\s+(\d+\.\d+)\s+(\d+)\s", line)
        if not m:
            continue
        t_ms = float(m.group(1))
        freq = int(m.group(2))
        t_samp = int(t_ms * 1e-3 * FS_HZ)
        rel = freq - LO_HZ
        signed = round(rel / CHANNEL_HZ)
        unsigned = signed if signed >= 0 else (M + signed)
        out.append({"t_samp": t_samp, "freq_hz": freq, "channel": unsigned,
                    "signed_ch": signed})
    return out


def host_bursts():
    """Run host pipeline; return list of detections with result."""
    p = subprocess.run(
        [str(REPO / "build-host-ubsan/test_worker_pipeline_albq")],
        capture_output=True, text=True, check=False, timeout=120)
    detections = []
    detect_re = re.compile(
        r"^\s+burst\s+(\d+):\s+ch=(\d+)\s+\(rel\s+([+-]?\d+)\s+kHz\)\s+"
        r"snr=([\d.]+)\s+dB\s+len=(\d+)\s+start=(\d+)")
    pipeline_re = re.compile(
        r"^\s+(\d+)\s+([\d.]+)\s+([\d.]+)\s+(\S+)\s+"
        r"ω_pre=([+-][\d.]+)\s+ω_post=([+-][\d.]+)\s+(\d+)\s+(\S+)")
    pipeline_idx = 0
    for line in p.stdout.splitlines():
        m = detect_re.match(line)
        if m:
            detections.append({
                "ch":     int(m.group(2)),
                "snr":    float(m.group(4)),
                "len":    int(m.group(5)),
                "start":  int(m.group(6)),
            })
            continue
        m = pipeline_re.match(line)
        if m:
            if pipeline_idx < len(detections):
                detections[pipeline_idx]["uw_snr"] = float(m.group(3))
                detections[pipeline_idx]["dir"]    = m.group(4)
                detections[pipeline_idx]["omega"]  = float(m.group(5))
                detections[pipeline_idx]["result"] = m.group(7)
                pipeline_idx += 1
    return detections


def channel_dist(a, b):
    """Modular distance between two channel indices on a circular M=64 grid."""
    d = abs(a - b) % M
    return min(d, M - d)


def best_match(gri, host_list):
    """Find host burst that best matches `gri`. Prefers (in order):
      1. result=OK matches (host actually decoded the burst)
      2. result=no-UW (host's uw_correlator found a peak, qpsk failed)
      3. result=no-decode w/ uw_snr>0 (matched filter ran, low SNR)
      4. anything else with non-zero uw_snr
    Within each tier, picks lowest time delta.
    Hosts entries with uw_snr=0 mean the matched filter returned
    UNKNOWN (no SNR ≥6 dB peak) — treat those as "host didn't really
    detect" and skip them entirely."""
    candidates = []
    for h in host_list:
        dt = abs(h["start"] - gri["t_samp"])
        dch = channel_dist(h["ch"], gri["channel"])
        if dt > TIME_TOL_SAMPLES or dch > CHANNEL_TOL:
            continue
        if h.get("uw_snr", 0.0) < 1e-3:
            continue       # host emitted but matched filter rejected
        candidates.append((dt, dch, h))
    if not candidates:
        return None, (TIME_TOL_SAMPLES + 1, CHANNEL_TOL + 1)

    def tier(h):
        r = h.get("result", "")
        if r == "OK":         return 0
        if r == "no-UW":      return 1
        if r == "no-decode":  return 2
        return 3
    candidates.sort(key=lambda c: (tier(c[2]), c[0], c[1]))
    dt, dch, h = candidates[0]
    return h, (dt, dch)


def main():
    print("Running gr-iridium ...", file=sys.stderr)
    gri = gri_decoded()
    print(f"gr-iridium decoded {len(gri)}", file=sys.stderr)

    print("Running host pipeline ...", file=sys.stderr)
    host = host_bursts()
    print(f"host detected {len(host)}", file=sys.stderr)

    missed = []
    detected_failed = []
    decoded = []
    for g in gri:
        h, score = best_match(g, host)
        if h is None:
            missed.append(g)
        elif h.get("result") == "OK":
            decoded.append((g, h, score))
        else:
            detected_failed.append((g, h, score))

    print()
    print(f"{'idx':>3} {'gri_t(ms)':>10} {'gri_freq':>10} {'gri_ch':>6} "
          f"{'  host_match':<25} {'result':<12}")
    print("-" * 80)
    for i, g in enumerate(gri):
        h, score = best_match(g, host)
        signed = g["signed_ch"] if g["signed_ch"] >= 0 else g["signed_ch"]
        if h is None:
            host_desc = "(none within tol)"
            result = "MISSED"
        else:
            host_desc = (f"ch={h['ch']} Δt={score[0]:>6}smp Δch={score[1]} "
                         f"SNR={h.get('uw_snr',0):.1f}")
            result = h.get("result", "?")
            if result == "OK":
                result = "DECODED"
            elif result == "no-UW":
                result = "DET (no-UW)"
            elif result == "no-decode":
                result = "DET (no-UW)"
            else:
                result = f"DET ({result})"
        t_ms = g["t_samp"] * 1000.0 / FS_HZ
        print(f"{i:>3} {t_ms:>10.1f} {g['freq_hz']:>10} {g['channel']:>6} "
              f"{host_desc:<25} {result:<12}")

    print()
    print(f"Totals on {len(gri)} gr-iridium decoded bursts:")
    print(f"  MISSED  (no host detection within ±{TIME_TOL_SAMPLES}smp / "
          f"±{CHANNEL_TOL}ch): {len(missed)}")
    print(f"  DETECTED, demod failed: {len(detected_failed)}")
    print(f"  DECODED:                 {len(decoded)}")


if __name__ == "__main__":
    main()
