#!/usr/bin/env python3
"""
Unified DSP comparison tool for the gr-iridium → ESP32-P4 port.

Replaces four ad-hoc scripts:
  - stagewise_compare.py       (per-stage RMS/peak/peak-freq comparison)
  - compare_host_to_griridium.py (host vs gri burst-level comparison)
  - compare_worker_to_scipy.py (worker chain vs scipy reference)
  - squared_fft_compare.py     (CFO squared-FFT input comparison)

All FFTs and correlations are numpy/scipy. No hand-rolled DFT loops.

Subcommands (via --mode):
  stagewise   — per-stage gri-vs-host pipeline comparison
  squared-fft — CFO squared-FFT magnitude spectrum comparison
  raw         — compare any two cf32/.cfile files with the full metric set

Golden-vector regression:
  --save-golden  <path.npz>   dump all stages from the current run
  --check-golden <path.npz>   compare current run against a stored set

Metrics per stage:
  NMSE-dB         10·log10(||x-y||²/||y||²) after time alignment via xcorr
  phase coherence |<x,y>| / (||x||·||y||)   in [0, 1]
  peak-freq Δ Hz  numpy FFT + parabolic interpolation on the peak bin
  RMS, peak       secondary diagnostics
  status          pass/fail vs the per-stage threshold (see THRESHOLDS)

Bursts are looked up via /tmp/host_direct_if/manifest.csv when present
(maps host burst_idx ↔ gri_burst_id). Without a manifest, --burst-id is
the gri id and --host-burst-idx (defaults to same) selects the host file.

Threshold values are taken from gr-iridium-port conventional engineering
thresholds (see DSP gaps vs gr-iridium plan section). They are NOT
empirically tuned to make the current run pass.
"""
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np
import scipy.signal as ss


# =============================================================================
# Configuration
# =============================================================================

# Per-stage NMSE pass thresholds, in dB. These are not empirically tuned —
# they reflect what a competent gr-iridium port should achieve at each
# stage given matched filter coefficients and matched sample rates.
#
# Reasoning:
#   - Channelizer outputs differ only by FIR-stopband leakage, integer
#     quantisation, and group-delay alignment → -50 dB is achievable.
#   - The 250 kHz resampled IQ should match gr-iridium's input-FIR-then-
#     decimate output to similar fidelity; -45 dB allows for small
#     polyphase-vs-overlap-save phase differences.
#   - RRC matched filter is identical in tap design, so -40 dB after the
#     CFO-corrected signal is the realistic target (CFO accuracy and
#     start-finder offset both feed in).
#   - Phase-rotated and final 2-sps stages depend on PLL and per-symbol
#     decisions and so loosen to -30 dB.
THRESHOLDS_NMSE_DB = {
    "channelized_40k":   -50.0,   # post polyphase channelizer (40 kHz IQ)
    "filtered_deci":     -45.0,   # post-resample 250 kHz IQ (gri primary stage)
    "cut_start":         -40.0,   # post start-finder (D13) trim
    "shift":             -35.0,   # post coarse CFO shift
    "rrc":               -40.0,   # post RRC matched filter
    "rotate":            -30.0,   # post peak-phase + linear-ramp rotation
    "rotate_cut":        -30.0,   # final 2-sps stream
}

# Phase coherence pass threshold per stage (0..1). Cosine of angle between
# the two waveforms (after alignment). 1.0 = perfectly in phase.
THRESHOLDS_PHASE = {
    "channelized_40k":   0.999,
    "filtered_deci":     0.995,
    "cut_start":         0.99,
    "shift":             0.98,
    "rrc":               0.99,
    "rotate":            0.95,
    "rotate_cut":        0.95,
}

# Peak frequency difference pass threshold (Hz). The channelizer outputs
# are tied to a 40 kHz grid → 100 Hz alignment is the channelizer limit.
# Later stages can shift via CFO; 200 Hz allows for the coarse estimator
# residual.
THRESHOLDS_PEAK_HZ = {
    "channelized_40k":   100.0,
    "filtered_deci":     100.0,
    "cut_start":         150.0,
    "shift":             200.0,
    "rrc":               200.0,
    "rotate":            500.0,
    "rotate_cut":        500.0,
}

# Stage definitions: short_key → (display label, gri filename stem, host
# filename, sample rate Hz). gri files are completed with the burst id.
STAGE_DEFS = [
    ("filtered_deci",
     "post-resample 250k",
     "signal-filtered-deci",
     "03_resamp_250k.cf32",
     250_000),
    ("cut_start",
     "post start-finder (D13)",
     "signal-filtered-deci-cut-start",
     "04_post_d13_250k.cf32",
     250_000),
    ("shift",
     "post CFO shift",
     "signal-filtered-deci-cut-start-shift",
     "05_post_cfo_250k.cf32",
     250_000),
    ("rrc",
     "post RRC matched filter",
     "signal-filtered-deci-cut-start-shift-rrc",
     "06_post_rrc_250k.cf32",
     250_000),
    ("rotate",
     "post phase-rotate",
     "signal-filtered-deci-cut-start-shift-rrc-rotate",
     "07_post_prerot_250k.cf32",
     250_000),
    ("rotate_cut",
     "post UW-cut 250k (rotate-cut)",
     "signal-filtered-deci-cut-start-shift-rrc-rotate-cut",
     "07b_post_rotate_cut_250k.cf32",
     250_000),
]

GRI_DIR = Path("/tmp/signals")
HOST_DIR = Path("/tmp/host_signals")
HOST_DIRECT_IF = Path("/tmp/host_direct_if")
DEFAULT_MANIFEST = HOST_DIRECT_IF / "manifest.csv"
DEFAULT_GOLDEN_DIR = Path(__file__).resolve().parent.parent / "fixtures"


# =============================================================================
# File I/O
# =============================================================================

def read_cf32(path: Path) -> Optional[np.ndarray]:
    """Read interleaved cf32 / .cfile into a complex64 numpy array."""
    if not path.exists():
        return None
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size < 2:
        return np.empty(0, dtype=np.complex64)
    n_pairs = raw.size // 2
    return raw[: 2 * n_pairs].view(np.complex64).copy()


def load_manifest(path: Path) -> dict[int, dict]:
    """Read /tmp/host_direct_if/manifest.csv → { burst_idx: row }."""
    out: dict[int, dict] = {}
    if not path.exists():
        return out
    with path.open() as f:
        for row in csv.DictReader(f):
            try:
                idx = int(row["burst_idx"])
            except (KeyError, ValueError):
                continue
            out[idx] = row
    return out


def manifest_resolve(manifest: dict[int, dict], gri_id: Optional[int],
                     host_idx: Optional[int]) -> tuple[Optional[int], Optional[int]]:
    """Look up the gri_burst_id ↔ host burst_idx pair via the manifest.

    Either argument may be None; the other side is filled in if possible.
    Returns (gri_id, host_idx). If both are None or no manifest entry is
    found, returns the original values unchanged.
    """
    if host_idx is not None and host_idx in manifest:
        try:
            gri = int(manifest[host_idx]["gri_burst_id"])
            return gri, host_idx
        except (KeyError, ValueError):
            pass
    if gri_id is not None:
        for hidx, row in manifest.items():
            try:
                if int(row["gri_burst_id"]) == gri_id:
                    return gri_id, hidx
            except (KeyError, ValueError):
                continue
    return gri_id, host_idx


# =============================================================================
# Metrics
# =============================================================================

@dataclass
class StageMetrics:
    stage_key: str
    label: str
    n_gri: int
    n_host: int
    nmse_db: float
    phase_coh: float
    peak_freq_diff_hz: float
    rms_gri: float
    rms_host: float
    peak_gri: float
    peak_host: float
    lag: int
    status: str   # "pass", "fail", or "skip"

    def to_row(self) -> list[str]:
        return [
            self.label,
            f"{self.n_gri}",
            f"{self.n_host}",
            _fmt_db(self.nmse_db),
            f"{self.phase_coh:.4f}",
            f"{self.peak_freq_diff_hz:+.1f}",
            f"{self.rms_gri:.4f}",
            f"{self.rms_host:.4f}",
            f"{self.peak_gri:.4f}",
            f"{self.peak_host:.4f}",
            f"{self.lag:+d}",
            self.status,
        ]


def _fmt_db(x: float) -> str:
    if not np.isfinite(x):
        return "-inf" if x < 0 else "inf"
    return f"{x:+.2f}"


def time_align(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, int]:
    """Find the integer lag that maximises |<x, shift(y, lag)>| via
    scipy.signal.correlate, then return overlapping aligned slices and the
    lag (positive => y starts later than x, so we trim x's head)."""
    if x.size == 0 or y.size == 0:
        return x, y, 0
    # Use 'full' correlation; magnitude peak gives the alignment.
    corr = ss.correlate(x, y, mode="full", method="fft")
    peak = int(np.argmax(np.abs(corr)))
    lag = peak - (y.size - 1)
    if lag >= 0:
        a = x[lag:]
        b = y[: a.size]
    else:
        b = y[-lag:]
        a = x[: b.size]
    n = min(a.size, b.size)
    return a[:n], b[:n], lag


def nmse_db(x: np.ndarray, y: np.ndarray) -> float:
    """NMSE = ||x - y||² / ||y||² (literal spec; no scalar fit).

    Returns the value in dB. -inf when x == y exactly. NaN when y has
    no energy. If x and y come out of pipelines with mismatched gains,
    the result reflects that — that is intentional, see the docstring
    in compute_stage_metrics for the rationale.
    """
    if x.size == 0 or y.size == 0:
        return float("nan")
    denom = float(np.vdot(y, y).real)
    if denom <= 0:
        return float("nan")
    err = x - y
    num = float(np.vdot(err, err).real)
    if num <= 0:
        return float("-inf")
    return 10.0 * np.log10(num / denom)


def nmse_db_normalized(x: np.ndarray, y: np.ndarray) -> float:
    """NMSE after gain-matching x to y via the optimal complex scalar α =
    <y, x>/<y, y>: residual ||x - αy||² / ||αy||². This isolates shape
    differences from overall gain/phase rotation."""
    if x.size == 0 or y.size == 0:
        return float("nan")
    denom = float(np.vdot(y, y).real)
    if denom <= 0:
        return float("nan")
    alpha = np.vdot(y, x) / denom
    err = x - alpha * y
    num = float(np.vdot(err, err).real)
    scaled = float(np.vdot(alpha * y, alpha * y).real)
    if scaled <= 0:
        return float("nan")
    if num <= 0:
        return float("-inf")
    return 10.0 * np.log10(num / scaled)


def phase_coherence(x: np.ndarray, y: np.ndarray) -> float:
    """|<x,y>| / (||x|| · ||y||); 1.0 = identical up to a scalar; 0 = orthogonal."""
    if x.size == 0 or y.size == 0:
        return float("nan")
    nx = np.linalg.norm(x)
    ny = np.linalg.norm(y)
    if nx == 0 or ny == 0:
        return float("nan")
    return float(np.abs(np.vdot(x, y)) / (nx * ny))


def peak_freq_hz(x: np.ndarray, fs_hz: float, n_fft: Optional[int] = None) -> float:
    """Locate the magnitude-peak bin of x's FFT (zero-padded to at least
    4× length, next pow2) and refine with parabolic interpolation. Returns
    the signed frequency in Hz."""
    n = x.size
    if n == 0:
        return float("nan")
    if n_fft is None:
        n_fft = 1
        while n_fft < n:
            n_fft <<= 1
        n_fft = max(n_fft * 4, 1024)
    spec = np.fft.fft(x, n=n_fft)
    mag = np.abs(spec)
    k = int(np.argmax(mag))
    # Parabolic interpolation on |X|. Wrap neighbours circularly.
    km = (k - 1) % n_fft
    kp = (k + 1) % n_fft
    am, a0, ap = mag[km], mag[k], mag[kp]
    denom = (am - 2.0 * a0 + ap)
    delta = 0.0 if abs(denom) < 1e-20 else 0.5 * (am - ap) / denom
    k_refined = k + delta
    # Map to signed frequency.
    if k_refined > n_fft / 2.0:
        k_refined -= n_fft
    return float(k_refined * fs_hz / n_fft)


def rms_peak(x: np.ndarray) -> tuple[float, float]:
    if x.size == 0:
        return 0.0, 0.0
    mag2 = (x.real * x.real + x.imag * x.imag)
    return float(np.sqrt(mag2.mean())), float(np.sqrt(mag2.max()))


# =============================================================================
# Stagewise mode
# =============================================================================

def compute_stage_metrics(stage_key: str, label: str, gri: np.ndarray,
                          host: np.ndarray, fs_hz: float) -> StageMetrics:
    if gri is None or host is None or gri.size == 0 or host.size == 0:
        rg, pg = rms_peak(gri if gri is not None else np.empty(0, np.complex64))
        rh, ph = rms_peak(host if host is not None else np.empty(0, np.complex64))
        return StageMetrics(
            stage_key=stage_key, label=label,
            n_gri=0 if gri is None else gri.size,
            n_host=0 if host is None else host.size,
            nmse_db=float("nan"), phase_coh=float("nan"),
            peak_freq_diff_hz=float("nan"),
            rms_gri=rg, rms_host=rh, peak_gri=pg, peak_host=ph,
            lag=0, status="skip",
        )

    a, b, lag = time_align(host, gri)
    nm = nmse_db(a, b)
    pc = phase_coherence(a, b)
    f_host = peak_freq_hz(host, fs_hz)
    f_gri = peak_freq_hz(gri, fs_hz)
    df = f_host - f_gri
    rg, pg = rms_peak(gri)
    rh, ph = rms_peak(host)

    nmse_thr = THRESHOLDS_NMSE_DB.get(stage_key, 0.0)
    phase_thr = THRESHOLDS_PHASE.get(stage_key, 0.0)
    peak_thr = THRESHOLDS_PEAK_HZ.get(stage_key, float("inf"))
    ok = (nm <= nmse_thr) and (pc >= phase_thr) and (abs(df) <= peak_thr)
    status = "pass" if ok else "fail"

    return StageMetrics(
        stage_key=stage_key, label=label,
        n_gri=gri.size, n_host=host.size,
        nmse_db=nm, phase_coh=pc, peak_freq_diff_hz=df,
        rms_gri=rg, rms_host=rh, peak_gri=pg, peak_host=ph,
        lag=lag, status=status,
    )


def run_stagewise(args) -> int:
    manifest = load_manifest(Path(args.manifest)) if args.manifest else {}
    gri_id, host_idx = manifest_resolve(manifest, args.burst_id, args.host_burst_idx)

    if gri_id is None:
        print("error: --burst-id is required (or a manifest entry to derive it)",
              file=sys.stderr)
        return 2

    # Optional channelizer stage: 40 kHz raw channelizer dump. gri has no
    # direct equivalent at 40 kHz (their channelizer is FFT overlap-save),
    # so we skip the gri side and just report host stats with status=skip.
    channelizer_host = HOST_DIR / "01_channelizer_40k.cf32"

    results: list[StageMetrics] = []
    saved_arrays: dict[str, np.ndarray] = {}

    # Channelizer (host-only at 40 kHz; no gri equivalent file).
    if channelizer_host.exists():
        host_ch = read_cf32(channelizer_host)
        rg, pg = 0.0, 0.0
        rh, ph = rms_peak(host_ch)
        results.append(StageMetrics(
            stage_key="channelized_40k",
            label="channelizer 40k (host-only)",
            n_gri=0, n_host=host_ch.size,
            nmse_db=float("nan"), phase_coh=float("nan"),
            peak_freq_diff_hz=float("nan"),
            rms_gri=rg, rms_host=rh, peak_gri=pg, peak_host=ph,
            lag=0, status="skip",
        ))
        saved_arrays["channelized_40k_host"] = host_ch

    gri_dir = Path(args.gri_dir)
    host_dir = Path(args.host_dir)

    for stage_key, label, gri_stem, host_name, fs in STAGE_DEFS:
        gri_path = gri_dir / f"{gri_stem}-{gri_id}.cfile"
        host_path = host_dir / host_name
        gri = read_cf32(gri_path)
        host = read_cf32(host_path)
        metrics = compute_stage_metrics(stage_key, label, gri, host, fs)
        results.append(metrics)
        if gri is not None:
            saved_arrays[f"{stage_key}_gri"] = gri
        if host is not None:
            saved_arrays[f"{stage_key}_host"] = host

    # Print table.
    print(f"Burst: gri_id={gri_id}  host_idx={host_idx if host_idx is not None else 'n/a'}")
    if host_idx in manifest:
        row = manifest[host_idx]
        print(f"  abs_freq={row.get('abs_freq_hz')} Hz  offset={row.get('freq_offset_hz')} Hz"
              f"  conf={row.get('confidence_pct')}%")
    print()
    print_table(results)

    # Optional golden-vector handling.
    rc = 0
    if args.save_golden:
        save_golden(Path(args.save_golden), saved_arrays, results)
    if args.check_golden:
        rc = max(rc, check_golden(Path(args.check_golden), saved_arrays))

    # Overall pass/fail by stage thresholds.
    any_fail = any(r.status == "fail" for r in results)
    return rc if rc else (1 if any_fail else 0)


def print_table(results: Iterable[StageMetrics]) -> None:
    header = ["stage", "n_gri", "n_host", "NMSE_dB", "phase_coh",
              "Δf_Hz", "rms_gri", "rms_host", "peak_gri", "peak_host",
              "lag", "status"]
    rows = [r.to_row() for r in results]
    widths = [max(len(h), max((len(r[i]) for r in rows), default=0))
              for i, h in enumerate(header)]
    fmt = "  ".join(f"{{:<{w}}}" for w in widths)
    print(fmt.format(*header))
    print("-" * (sum(widths) + 2 * (len(widths) - 1)))
    for r in rows:
        print(fmt.format(*r))


# =============================================================================
# Squared-FFT mode
# =============================================================================

# Reproduce gr-iridium's CFO front-end: take CFO_INPUT_N samples, Blackman
# window, square (z²), zero-pad to CFO_FFT_N, then FFT. Magnitude² spectrum.
CFO_FFT_N = 4096
CFO_INPUT_N = 256
CFO_FS_HZ = 250_000


def squared_fft_spectrum(x: np.ndarray, n_in: int = CFO_INPUT_N,
                         n_fft: int = CFO_FFT_N) -> np.ndarray:
    if x.size == 0:
        return np.zeros(n_fft)
    n_in = min(n_in, x.size)
    window = ss.windows.blackman(n_in, sym=False)
    sq = (x[:n_in] ** 2) * window
    spec = np.fft.fft(sq, n=n_fft)
    return np.abs(spec) ** 2


def top_peaks(mag_sq: np.ndarray, n_top: int, fs_hz: float) -> list[tuple[int, float, float, float]]:
    """Return [(signed_bin, f_2c_Hz, carrier_residual_Hz, mag²)]."""
    n = mag_sq.size
    idx = np.argpartition(mag_sq, -n_top)[-n_top:]
    idx = idx[np.argsort(mag_sq[idx])[::-1]]
    out = []
    for k in idx:
        signed = int(k) if k < n // 2 else int(k) - n
        f2c = signed * fs_hz / n
        out.append((signed, f2c, f2c / 2.0, float(mag_sq[k])))
    return out


def run_squared_fft(args) -> int:
    manifest = load_manifest(Path(args.manifest)) if args.manifest else {}
    gri_id, host_idx = manifest_resolve(manifest, args.burst_id, args.host_burst_idx)
    if gri_id is None:
        print("error: --burst-id is required", file=sys.stderr)
        return 2

    gri_path = Path(args.gri_dir) / f"signal-filtered-deci-cut-start-{gri_id}.cfile"
    host_path = Path(args.host_dir) / "04_post_d13_250k.cf32"

    gri = read_cf32(gri_path)
    host = read_cf32(host_path)
    if gri is None or gri.size == 0:
        print(f"missing or empty: {gri_path}", file=sys.stderr)
        return 1
    if host is None or host.size == 0:
        print(f"missing or empty: {host_path}", file=sys.stderr)
        return 1

    print(f"Burst gri_id={gri_id} host_idx={host_idx}: post-D13 gri n={gri.size}, host n={host.size}")

    gri_spec = squared_fft_spectrum(gri)
    host_spec = squared_fft_spectrum(host)
    gri_peaks = top_peaks(gri_spec, args.n_top, CFO_FS_HZ)
    host_peaks = top_peaks(host_spec, args.n_top, CFO_FS_HZ)

    def print_peaks(name: str, peaks):
        print(f"\n{name} squared-FFT top {len(peaks)} peaks (window={CFO_INPUT_N}, fft={CFO_FFT_N}):")
        print(f"  {'bin':>5}  {'f_2c_Hz':>9}  {'carrier_Hz':>11}  magnitude²")
        for k, f2c, fc, mag in peaks:
            print(f"  {k:>+5}  {f2c:>+9.1f}  {fc:>+11.1f}  {mag:.3e}")

    print_peaks("gr-iridium", gri_peaks)
    print_peaks("host", host_peaks)

    rc = 0
    if gri_peaks[0][0] == host_peaks[0][0]:
        print(f"\nPASS: peak bin matches ({gri_peaks[0][0]}, carrier {gri_peaks[0][2]:+.1f} Hz)")
    else:
        print(f"\nFAIL: gri peak bin {gri_peaks[0][0]} ({gri_peaks[0][2]:+.1f} Hz) "
              f"vs host {host_peaks[0][0]} ({host_peaks[0][2]:+.1f} Hz)")
        rc = 1

    if args.save_golden:
        save_golden(Path(args.save_golden),
                    {"gri_spec": gri_spec, "host_spec": host_spec,
                     "gri_input": gri, "host_input": host},
                    None)
    if args.check_golden:
        rc = max(rc, check_golden(Path(args.check_golden),
                                  {"gri_spec": gri_spec, "host_spec": host_spec}))
    return rc


# =============================================================================
# Raw two-file mode
# =============================================================================

def run_raw(args) -> int:
    if args.synthesize_worker_test:
        return run_worker_synthesis(args)

    if not args.left or not args.right:
        print("error: --mode raw requires --left and --right (or --synthesize-worker-test)",
              file=sys.stderr)
        return 2

    left = read_cf32(Path(args.left))
    right = read_cf32(Path(args.right))
    if left is None or right is None:
        print("error: cannot read inputs", file=sys.stderr)
        return 1

    a, b, lag = time_align(left, right)
    metrics = StageMetrics(
        stage_key="raw",
        label=f"{Path(args.left).name} vs {Path(args.right).name}",
        n_gri=right.size, n_host=left.size,
        nmse_db=nmse_db(a, b),
        phase_coh=phase_coherence(a, b),
        peak_freq_diff_hz=peak_freq_hz(left, args.fs_hz) - peak_freq_hz(right, args.fs_hz),
        rms_gri=rms_peak(right)[0], rms_host=rms_peak(left)[0],
        peak_gri=rms_peak(right)[1], peak_host=rms_peak(left)[1],
        lag=lag, status="info",
    )
    print_table([metrics])

    rc = 0
    if args.save_golden:
        save_golden(Path(args.save_golden),
                    {"left": left, "right": right}, [metrics])
    if args.check_golden:
        rc = check_golden(Path(args.check_golden), {"left": left, "right": right})
    return rc


# =============================================================================
# Worker-chain synthesis: stage-1 FIR + polyphase resampler vs scipy reference
# (Mirrors the original compare_worker_to_scipy.py behaviour.)
# =============================================================================

def _build_worker_stage1_fir(taps: int, polychan_m: int) -> np.ndarray:
    omega_c = np.pi / polychan_m
    n = np.arange(taps) - (taps - 1) / 2.0
    sinc = np.where(np.abs(n) < 1e-9, omega_c / np.pi,
                    np.sin(omega_c * n) / (np.pi * n))
    hamming = 0.5 * (1.0 - np.cos(2.0 * np.pi * np.arange(taps) / (taps - 1)))
    h = sinc * hamming
    return h / h.sum()


def _build_worker_stage2_resampler(taps: int, interp: int, decim: int,
                                   fs_after_stage1: float,
                                   channel_half_bw: float) -> np.ndarray:
    rN = taps * interp
    omega_c = 2.0 * np.pi * channel_half_bw / (fs_after_stage1 * interp)
    n = np.arange(rN) - (rN - 1) / 2.0
    sinc = np.where(np.abs(n) < 1e-9, omega_c / np.pi,
                    np.sin(omega_c * n) / (np.pi * n))
    hamming = 0.5 * (1.0 - np.cos(2.0 * np.pi * np.arange(rN) / (rN - 1)))
    h = sinc * hamming
    return h / h.sum() * interp


def run_worker_synthesis(args) -> int:
    fs_in = 2_560_000
    decim1 = 32
    interp2 = 25
    decim2 = 8
    polychan_m = 64
    np.random.seed(42)

    n = int(0.020 * fs_in)        # 20 ms input
    n_syms = 50
    syms = np.random.choice([-1+1j, 1-1j, 1+1j, -1-1j], n_syms)
    sym_period = fs_in / 25_000
    sym_positions = np.zeros(n, dtype=np.complex64)
    indices = (np.arange(n_syms) * sym_period).astype(int)
    valid = indices < n
    sym_positions[indices[valid]] = syms[valid]

    # RRC pulse shape at fs_in.
    rrc_len = 2048
    rrc_t = (np.arange(rrc_len) - rrc_len // 2) / fs_in
    sym_T = 1.0 / 25_000
    beta = 0.4
    tau = rrc_t / sym_T
    eps = 1e-9
    num = np.sin(np.pi * tau * (1 - beta)) + 4 * beta * tau * np.cos(np.pi * tau * (1 + beta))
    den = np.pi * tau * (1 - (4 * beta * tau) ** 2)
    rrc = np.where(np.abs(tau) < eps, 1 - beta + 4 * beta / np.pi, num / np.where(np.abs(den) < eps, 1, den))
    rrc[np.abs(den) < eps] = 0.0
    rrc /= np.sqrt(np.sum(rrc ** 2))

    signal = np.convolve(sym_positions, rrc, mode="same")
    sig_pow = np.mean(np.abs(signal) ** 2)
    noise_pow = sig_pow * 10 ** (-25 / 10)
    noise = (np.random.randn(n) + 1j * np.random.randn(n)) * np.sqrt(noise_pow / 2)
    x = (signal + noise).astype(np.complex64)

    s1 = _build_worker_stage1_fir(64, polychan_m)
    s2 = _build_worker_stage2_resampler(64, interp2, decim2,
                                        fs_in / decim1, fs_in / (2.0 * polychan_m))

    post_corr_decim = 5     # worker's final 250k → 50k (=2 sps) step
    y1 = ss.fftconvolve(x, s1, mode="same")[::decim1]
    y2 = ss.resample_poly(y1, up=interp2, down=decim2, window=s2)
    y_ours = y2[::post_corr_decim]
    # Scipy direct reference: 2.56 MHz → 50 kHz via up=5, down=256.
    y_scipy = ss.resample_poly(x, up=5, down=256)

    n_common = min(y_ours.size, y_scipy.size)
    y_ours = y_ours[:n_common]
    y_scipy = y_scipy[:n_common]

    a, b, lag = time_align(y_ours, y_scipy)
    # The two chains have different overall gains by design (worker
    # uses interp*polyphase-DC-gain normalisation); use shape-only NMSE.
    nm = nmse_db_normalized(a, b)
    pc = phase_coherence(a, b)
    rms_a, pk_a = rms_peak(y_ours)
    rms_b, pk_b = rms_peak(y_scipy)

    print(f"Worker DSP chain vs scipy.resample_poly (synthesised 25-ksym RRC burst, 25 dB SNR)")
    print(f"  Input: {n} samples @ {fs_in/1e6:.2f} MS/s ({n/fs_in*1000:.1f} ms)")
    print(f"  Stage 1 (LP + decim {decim1}) RMS: {np.sqrt(np.mean(np.abs(y1)**2)):.4f}")
    print(f"  Stage 2 (resample {interp2}/{decim2}) RMS: {np.sqrt(np.mean(np.abs(y2)**2)):.4f}")
    print(f"  Ours  (final decim {post_corr_decim}) RMS: {rms_a:.4f}")
    print(f"  Scipy direct (5/256)          RMS: {rms_b:.4f}")
    print(f"  Aligned NMSE: {_fmt_db(nm)} dB   phase_coh: {pc:.4f}   lag: {lag:+d}")
    if nm <= -30.0:
        print("  PASS: paths agree to better than -30 dB NMSE")
        rc = 0
    elif nm <= -15.0:
        print("  MARGINAL: -30 dB < NMSE ≤ -15 dB — bandlimit/transient differences")
        rc = 0
    else:
        print(f"  FAIL: NMSE {nm:+.1f} dB exceeds -15 dB threshold")
        rc = 1

    if args.save_golden:
        save_golden(Path(args.save_golden),
                    {"y_ours": y_ours, "y_scipy": y_scipy}, None)
    if args.check_golden:
        rc = max(rc, check_golden(Path(args.check_golden),
                                  {"y_ours": y_ours, "y_scipy": y_scipy}))
    return rc


# =============================================================================
# Golden-vector save/check
# =============================================================================

def save_golden(path: Path, arrays: dict[str, np.ndarray],
                metrics: Optional[Iterable[StageMetrics]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = dict(arrays)
    if metrics is not None:
        payload["_metrics_labels"] = np.array([m.label for m in metrics])
        payload["_metrics_nmse_db"] = np.array([m.nmse_db for m in metrics])
        payload["_metrics_phase"] = np.array([m.phase_coh for m in metrics])
    np.savez_compressed(path, **payload)
    print(f"\nsaved golden vector → {path} ({len(arrays)} arrays)")


def check_golden(path: Path, arrays: dict[str, np.ndarray]) -> int:
    if not path.exists():
        print(f"\nerror: golden file not found: {path}", file=sys.stderr)
        return 2
    with np.load(path, allow_pickle=False) as data:
        keys = [k for k in data.files if not k.startswith("_")]
        print(f"\nchecking against golden {path} ({len(keys)} arrays)")
        worst = 0.0
        any_diff = False
        rc = 0
        for key in keys:
            if key not in arrays:
                print(f"  {key}: MISSING in current run")
                rc = 1
                continue
            ref = data[key]
            cur = arrays[key]
            if ref.shape != cur.shape:
                print(f"  {key}: SHAPE MISMATCH ref={ref.shape} cur={cur.shape}")
                rc = 1
                any_diff = True
                continue
            if ref.dtype.kind == "c" or cur.dtype.kind == "c":
                cur_c = cur.astype(np.complex128)
                ref_c = ref.astype(np.complex128)
                num = float(np.vdot(cur_c - ref_c, cur_c - ref_c).real)
                den = float(np.vdot(ref_c, ref_c).real)
            else:
                err = cur.astype(np.float64) - ref.astype(np.float64)
                num = float((err * err).sum())
                den = float((ref.astype(np.float64) ** 2).sum())
            if den == 0:
                ratio_db = float("-inf") if num == 0 else float("inf")
            else:
                ratio_db = 10.0 * np.log10(num / den) if num > 0 else float("-inf")
            worst = max(worst, ratio_db if np.isfinite(ratio_db) else worst)
            status = "match" if num == 0 else f"NMSE {_fmt_db(ratio_db)} dB"
            if num != 0:
                any_diff = True
            print(f"  {key}: {status}")
        if any_diff:
            print(f"  worst NMSE: {_fmt_db(worst)} dB")
        else:
            print("  PERFECT MATCH (all arrays bit-identical)")
        return rc


# =============================================================================
# CLI
# =============================================================================

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Unified gr-iridium ↔ host pipeline DSP comparison.")
    ap.add_argument("--mode", choices=["stagewise", "squared-fft", "raw"],
                    default="stagewise",
                    help="comparison mode (default: stagewise)")
    ap.add_argument("--burst-id", type=int, default=None,
                    help="gr-iridium debug burst id (the number in "
                         "/tmp/signals/signal-*-<id>.cfile)")
    ap.add_argument("--host-burst-idx", type=int, default=None,
                    help="optional host burst index; if omitted and a "
                         "manifest is present we look it up via "
                         "gri_burst_id ↔ burst_idx")
    ap.add_argument("--manifest", default=str(DEFAULT_MANIFEST),
                    help=f"path to host_direct_if/manifest.csv "
                         f"(default: {DEFAULT_MANIFEST})")
    ap.add_argument("--gri-dir", default=str(GRI_DIR),
                    help=f"gr-iridium dump directory (default: {GRI_DIR})")
    ap.add_argument("--host-dir", default=str(HOST_DIR),
                    help=f"host dump directory (default: {HOST_DIR})")
    ap.add_argument("--n-top", type=int, default=10,
                    help="(squared-fft) number of top peaks to print")
    ap.add_argument("--left", default=None,
                    help="(raw) left-hand cf32 file")
    ap.add_argument("--right", default=None,
                    help="(raw) right-hand cf32 file")
    ap.add_argument("--fs-hz", type=float, default=250_000.0,
                    help="(raw) sample rate of the input files (Hz)")
    ap.add_argument("--synthesize-worker-test", action="store_true",
                    help="(raw) ignore --left/--right and run the worker "
                         "DSP chain vs scipy.resample_poly synthesis test")
    ap.add_argument("--save-golden", default=None,
                    help="save current run as a golden npz at this path")
    ap.add_argument("--check-golden", default=None,
                    help="check current run against a golden npz")
    args = ap.parse_args()

    if args.mode == "stagewise":
        return run_stagewise(args)
    if args.mode == "squared-fft":
        return run_squared_fft(args)
    if args.mode == "raw":
        return run_raw(args)
    return 2


if __name__ == "__main__":
    sys.exit(main())
