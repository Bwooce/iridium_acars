#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
mendeley_cfo_analyze.py — read the CSV from mendeley_cfo_characterize.py
and derive a data-driven threshold for #115's CFO peak-confidence filter.

Outputs:
  - Percentile tables for peak/second, peak/mean, peak/median
  - Per-satellite breakdown (mean ± stddev of ratios)
  - Per-beam breakdown
  - A threshold recommendation that admits ≥ 99% of legitimate IRA-DL
    bursts (the target false-negative rate for the filter)
  - Optional PNG histograms if matplotlib is installed

Usage:
    python3 tests/scripts/mendeley_cfo_analyze.py \\
        < ~/iq_cache/cfo_stats.csv

    # With histograms:
    python3 tests/scripts/mendeley_cfo_analyze.py \\
        --plot-dir docs/115_cfo_analysis \\
        < ~/iq_cache/cfo_stats.csv

Result: a markdown report on stdout suitable for committing to
docs/115_cfo_distribution_analysis.md (or similar).
"""

import argparse
import csv
import sys
from collections import defaultdict


def parse_csv(stream):
    """Return list of dicts (one per burst). Streams stdin to avoid
    loading all 3.8 M into memory unnecessarily, but does collect into
    a list at the end since the analyser is non-streaming."""
    reader = csv.DictReader(stream)
    rows = []
    for r in reader:
        try:
            rows.append({
                "ts":      float(r["ts"]),
                "sat_id":  int(r["sat_id"]),
                "beam_id": int(r["beam_id"]),
                "freq_hz": int(r["freq_hz"]),
                "n":       int(r["n_complex"]),
                "p":       float(r["peak_mag"]),
                "s2":      float(r["second_mag"]),
                "pm":      float(r["mean_mag"]),
                "pmd":     float(r["median_mag"]),
                "p_s":     float(r["peak_over_second"]),
                "p_mn":    float(r["peak_over_mean"]),
                "p_md":    float(r["peak_over_median"]),
            })
        except (KeyError, ValueError):
            continue
    return rows


def percentiles(values, ps):
    """Lightweight percentile without numpy dep (so analyzer runs even
    on stripped-down Pythons). Sorts in place is ok — values is local."""
    if not values:
        return [float("nan")] * len(ps)
    values = sorted(values)
    n = len(values)
    out = []
    for p in ps:
        idx = max(0, min(n - 1, int(p * (n - 1) / 100.0)))
        out.append(values[idx])
    return out


def format_pcts(label, values):
    ps = [0.1, 1.0, 5.0, 25.0, 50.0, 75.0, 95.0, 99.0, 99.9]
    vals = percentiles(values, ps)
    cells = "  ".join(f"{v:>10.3f}" for v in vals)
    headers = "  ".join(f"{p:>10}" for p in [f"p{p}" for p in ps])
    return f"\n## {label}\n\n  {headers}\n  {cells}\n"


def threshold_recommendation(values, accept_pct=99.0):
    """Pick a threshold that admits at least accept_pct% of legitimate
    bursts. The threshold is the (100-accept_pct)-percentile — bursts
    BELOW this value are rejected."""
    if not values:
        return float("nan")
    keep_idx = (100 - accept_pct) / 100.0
    vals = sorted(values)
    n = len(vals)
    return vals[max(0, int(keep_idx * (n - 1)))]


def per_group_summary(rows, key, ratio_key, top_n=20):
    """Mean+stddev per group. Returns list of (group_id, n, mean, std)."""
    groups = defaultdict(list)
    for r in rows:
        groups[r[key]].append(r[ratio_key])
    summary = []
    for g, vals in groups.items():
        if not vals:
            continue
        m = sum(vals) / len(vals)
        v = sum((x - m) ** 2 for x in vals) / len(vals) if len(vals) > 1 else 0.0
        summary.append((g, len(vals), m, v ** 0.5))
    summary.sort(key=lambda t: -t[1])  # by count, descending
    return summary[:top_n]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--plot-dir", default=None,
                   help="if given, write histogram PNGs to this dir "
                        "(requires matplotlib)")
    p.add_argument("--accept-pct", type=float, default=99.0,
                   help="acceptance rate the threshold must achieve (default 99)")
    args = p.parse_args()

    rows = parse_csv(sys.stdin)
    print(f"# CFO peak-distribution analysis on {len(rows):,} Mendeley bursts\n")
    if not rows:
        print("FAIL: no rows parsed from stdin")
        return 1

    print(f"Source: Oligeri/Sciancalepore Iridium dataset "
          f"(DOI 10.17632/xcxspv8c2r.2)")
    print(f"All bursts are IRA-DL (Ring Alert downlink), per dataset spec.\n")

    # Overall distributions for each ratio
    for label, key in (
        ("peak / second-peak (the original #115 statistic, threshold was 4×)",
         "p_s"),
        ("peak / mean (excluding main lobe)", "p_mn"),
        ("peak / median (excluding main lobe)", "p_md"),
    ):
        vals = [r[key] for r in rows if r[key] < float("inf")]
        print(format_pcts(label, vals))
        rec = threshold_recommendation(vals, args.accept_pct)
        print(f"  Recommended threshold (admit ≥ {args.accept_pct}% bursts): "
              f"≥ {rec:.3f}\n")

    # Per-satellite breakdown
    print(f"\n## Per-satellite peak/median (top 20 by count)\n")
    print(f"  {'sat_id':>8}  {'n':>8}  {'mean':>10}  {'stddev':>10}")
    for sat, n, m, s in per_group_summary(rows, "sat_id", "p_md", top_n=20):
        print(f"  {sat:>8}  {n:>8}  {m:>10.2f}  {s:>10.2f}")

    print(f"\n## Per-beam peak/median (top 20 by count)\n")
    print(f"  {'beam_id':>8}  {'n':>8}  {'mean':>10}  {'stddev':>10}")
    for beam, n, m, s in per_group_summary(rows, "beam_id", "p_md", top_n=20):
        print(f"  {beam:>8}  {n:>8}  {m:>10.2f}  {s:>10.2f}")

    # Optional plot
    if args.plot_dir:
        try:
            import os
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            os.makedirs(args.plot_dir, exist_ok=True)
            for label, key in (("peak_over_second", "p_s"),
                               ("peak_over_mean", "p_mn"),
                               ("peak_over_median", "p_md")):
                vals = [r[key] for r in rows if r[key] < float("inf")]
                plt.figure(figsize=(8, 4))
                plt.hist(vals, bins=200, log=True)
                plt.xlabel(label)
                plt.ylabel("burst count (log)")
                plt.title(f"{label} distribution ({len(vals):,} bursts)")
                plt.axvline(threshold_recommendation(vals, args.accept_pct),
                            color="red", linestyle="--",
                            label=f"{args.accept_pct}% threshold")
                plt.legend()
                out = os.path.join(args.plot_dir, f"{label}.png")
                plt.savefig(out, dpi=120, bbox_inches="tight")
                plt.close()
                print(f"\n  wrote {out}")
        except ImportError:
            print("\n(matplotlib not installed; skipping plots)", file=sys.stderr)

    print(f"\n## Recommendation for #115 re-apply\n")
    print(f"Use peak/median as the statistic (more robust than peak/2nd-peak\n"
          f"because the 2nd-best bin is itself a noisy estimate). Set the\n"
          f"threshold to the {args.accept_pct}-percentile peak/median value\n"
          f"reported above. This rejects truly noise-dominated CFO estimates\n"
          f"while admitting ≥ {args.accept_pct}% of legitimate IRA bursts.\n"
          f"\nValidate by re-running the smoke tests (RAW + REAL) with the\n"
          f"new threshold in cfo_fine_estimate; baseline counts must hold.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
