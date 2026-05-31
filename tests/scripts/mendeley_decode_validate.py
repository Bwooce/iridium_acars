#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
mendeley_decode_validate.py — DEFERRED / NON-FUNCTIONAL.

This script was designed to process Mendeley Iridium IRA bursts through
iridium-toolkit's bit-level decoder and validate against ground-truth
metadata. The architecture is sound (3-way deinterleave + 3 × BCH(31,21)
with poly 1207 per iridium-toolkit; UW pattern matches uw_dl from
gr-iridium burst_downmix_impl.cc), but the BIT EXTRACTION from
Mendeley's IQ samples does not produce decoder-compatible bits despite
brute-forcing 12 reasonable convention combinations (IQ vs QI bit order
× as-is / differential-encode / differential-decode × skip-UW vs no-skip).

Top variant achieved 35/60 BCH-blocks-OK / 1/20 sat_id matches across
20 test records — close to random-baseline for a t=2 BCH(31,21) code
(50% per block) and effectively zero ground-truth agreement. Fixing
requires more iridium-toolkit / gr-iridium source inspection (or
authors of the Mendeley dataset publishing their exact extraction
format) than the value justifies for our pipeline-regression needs.

Status 2026-05-31: SHELVED. The ALBQ wideband recording
(test_data/iridium_downlink_2022-03-17_albuquerque/) already provides
all the ground truth our pipeline regression actually needs:
  - 82 bursts across 7 burst types (vs Mendeley's 100% IRA-DL)
  - gr-iridium-decoded bits in derivations/iridium.bits
  - Parsed messages in derivations/iridium.parsed
  - Already used by smoke RAW_IRIDIUM (3 real decodes baseline).

Keep the file as documentation of what was tried and a starting point
if a future maintainer wants to revisit (e.g. with format clarification
from Oligeri/Sciancalepore — task #131 captures the broader 'find
better-documented Iridium IQ datasets' followup).

Usage (still runnable for testing the structure, just don't expect
meaningful sat_id matches):
    python3 tests/scripts/mendeley_decode_validate.py \\
        > ~/iq_cache/decode_stats.csv
"""

import argparse
import re
import sys
import time
from pathlib import Path

import numpy as np

# Bring iridium-toolkit into the path so we can use its bch module
# directly (saves re-implementing the BCH poly + syndrome table).
REPO = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO / "iridium-toolkit"))
sys.path.insert(0, str(Path(__file__).parent))

import bch          # noqa: E402  — iridium-toolkit/bch.py
import fetch_mendeley_iridium as fetch     # noqa: E402

# IRA constants from iridium-toolkit/bitsparser.py:
IRIDIUM_ACCESS = "001100000011000011110011"     # downlink UW (24 bits = 12 symbols, BPSK)
RINGALERT_BCH_POLY = 1207
N_UW_SYMBOLS = 12   # = 24 bits

RE_COMPLEX = re.compile(
    r"\(\s*([+-]?\d+\.?\d*(?:[eE][+-]?\d+)?)"
    r"\s*([+-])\s*"
    r"(\d+\.?\d*(?:[eE][+-]?\d+)?)\s*j\s*\)"
)


def parse_iq(text: str) -> np.ndarray:
    parts = RE_COMPLEX.findall(text)
    if not parts:
        return np.empty(0, dtype=np.complex64)
    re_arr = np.array([float(p[0]) for p in parts], dtype=np.float32)
    im_arr = np.array([float(p[2]) * (1 if p[1] == "+" else -1) for p in parts],
                      dtype=np.float32)
    return re_arr + 1j * im_arr


def iq_to_dqpsk_bits(iq: np.ndarray) -> str:
    """Replicate iridium-toolkit's IQ-to-bits flow.

    gr-iridium emits *raw* (not differentially-decoded) DQPSK symbols.
    iridium-toolkit takes the demodulated bit pairs and runs
    `de_dqpsk(bits)` which:
      1. Re-interprets each bit pair as a Gray-coded symbol index
         imap = [0, 1, 3, 2]
      2. Cumulatively sums mod 4 to undo the differential encoding

    Since we have IQ (not bit pairs), we extract the bit pairs first
    from the absolute phase — each symbol's phase lands in one of 4
    π/2 sectors centred at ±π/4, ±3π/4. The mapping back to
    bit-pairs matches iridium-toolkit's gr-iridium-compatible
    ordering.
    """
    if iq.size < 2:
        return ""
    ang = np.angle(iq)
    # Sector: 0 ∈ [π/4 .. 3π/4), 1 ∈ [3π/4 .. -3π/4) (wrap),
    #         2 ∈ [-3π/4 .. -π/4), 3 ∈ [-π/4 .. π/4)
    # i.e. sector index = ((angle + π/4) // (π/2)) mod 4, with reverse to
    # match iridium-toolkit's I,Q -> bit pair ordering. (The sign of I
    # and Q gives the bit pair directly under standard QPSK Gray.)
    bits = []
    for z in iq:
        i, q = z.real, z.imag
        # Standard Gray: (sign_I, sign_Q) -> (bit0, bit1)
        # I>0, Q>0 -> 00; I<0, Q>0 -> 01; I<0, Q<0 -> 11; I>0, Q<0 -> 10
        b0 = "1" if i < 0 else "0"
        b1 = "1" if q < 0 else "0"
        bits.append(b0)
        bits.append(b1)
    return "".join(bits)


def de_dqpsk_str(bits: str) -> list:
    """Replicate iridium-toolkit's de_dqpsk: bit-pair string -> list of
    differentially-undone symbol indices in {0,1,2,3}."""
    imap = [0, 1, 3, 2]
    symbols = []
    for x in range(0, len(bits) - 1, 2):
        symbols.append(imap[int(bits[x]) * 2 + int(bits[x + 1])])
    for c in range(1, len(symbols)):
        symbols[c] = (symbols[c - 1] + symbols[c]) % 4
    return symbols


def symbols_to_bpsk_bits(symbols: list) -> str:
    """After de_dqpsk, each symbol in {0,1,2,3} represents a BPSK-pair.
    Map back to a flat bit string.

    iridium-toolkit treats the symbol index as 2 bits: lower bit, upper bit.
    """
    bits = []
    for s in symbols:
        bits.append(str(s >> 1))
        bits.append(str(s & 1))
    return "".join(bits)


def de_interleave3(group: str) -> tuple:
    """Direct port of iridium-toolkit's de_interleave3."""
    # Pair-swap then 3-way demux from the END.
    symbols = [group[z + 1] + group[z] for z in range(0, len(group), 2)]
    third = ''.join([symbols[x] for x in range(len(symbols) - 3, -1, -3)])
    second = ''.join([symbols[x] for x in range(len(symbols) - 2, -1, -3)])
    first = ''.join([symbols[x] for x in range(len(symbols) - 1, -1, -3)])
    return (first, second, third)


def decode_ira_header(data_bits: str) -> dict:
    """Run the IRA-specific path: 3-way deinterleave the first 96 bits,
    BCH-decode each 32-bit block, concatenate the 21-bit data parts to
    form the 63-bit bitstream_bch, parse sat_id + beam_id from it.

    Returns dict with: bch1_ok, bch1_errs, bch2_ok, bch2_errs,
    bch3_ok, bch3_errs, sat_id_decoded, beam_id_decoded (when all 3
    blocks decode)."""
    out = {
        "bch1_ok": False, "bch1_errs": -1,
        "bch2_ok": False, "bch2_errs": -1,
        "bch3_ok": False, "bch3_errs": -1,
    }
    if len(data_bits) < 96:
        return out

    blocks = de_interleave3(data_bits[:96])
    bitstream_bch = ""
    for i, blk in enumerate(blocks, 1):
        if len(blk) < 31:
            return out
        # Each block: 31 BCH bits + 1 parity bit; decode the first 31.
        errs, data, _ = bch.bch_repair(RINGALERT_BCH_POLY, blk[:31])
        out[f"bch{i}_ok"] = (errs >= 0)
        out[f"bch{i}_errs"] = errs
        if errs < 0:
            return out
        # Parity check (last bit + count): see bitsparser.py:1241
        bch_corrected = data + _
        parity_ok = (bch_corrected + blk[31]).count('1') % 2 == 0
        if not parity_ok:
            out[f"bch{i}_ok"] = False
            return out
        bitstream_bch += data

    # IRA payload (per IridiumRAMessage in bitsparser.py:1560-1561):
    #   bits [0:7]   = ra_sat (sat_id)
    #   bits [7:13]  = ra_cell (beam_id)
    out["sat_id_decoded"] = int(bitstream_bch[0:7], 2)
    out["beam_id_decoded"] = int(bitstream_bch[7:13], 2)
    return out


def decode_burst(iq: np.ndarray) -> dict:
    """Full per-burst decode chain mirroring iridium-toolkit's IRA path."""
    raw_bits = iq_to_dqpsk_bits(iq)
    if len(raw_bits) < 24 + 96:
        return {"too_short": True}
    # iridium-toolkit's pipeline:
    #   bitstream_raw = (raw bits with UW prepended)
    #   data = bitstream_raw[len(iridium_access):]   # strip UW
    #   for RA: descrambled = de_interleave3(data[:3*32])
    #   Then BCH on each de-interleaved block.
    #
    # We DO NOT need to run de_dqpsk on the BIT representation because
    # the IQ already encodes the symbols absolutely (each IQ sample =
    # one symbol). What's needed is to *differentially undo* the symbol
    # sequence, then re-flatten to bits.
    symbols = de_dqpsk_str(raw_bits)
    flat_bits = symbols_to_bpsk_bits(symbols)
    # Strip the 24-bit UW (12 symbols × 2 bits)
    data = flat_bits[24:]
    return decode_ira_header(data)


def stream_records(src_files, max_n):
    seen = 0
    for src in src_files:
        with open(src, "r", buffering=1 << 20, errors="replace") as f:
            for line in f:
                if seen >= max_n:
                    return
                parts = line.rstrip("\n").split("\t")
                if len(parts) < 10:
                    continue
                try:
                    meta = {
                        "ts":      float(parts[0]),
                        "sat_id":  int(parts[2]),
                        "beam_id": int(parts[3]),
                        "freq_hz": float(parts[8]),
                    }
                except ValueError:
                    continue
                yield meta, parts[9]
                seen += 1


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--files", nargs="+", default=None)
    p.add_argument("--max", type=int, default=10**9)
    p.add_argument("--progress-every", type=int, default=10000)
    args = p.parse_args()

    ed = fetch.extract_dir()
    if not ed.exists() or not any(ed.iterdir()):
        sys.stderr.write(f"ERROR: extracted dir {ed} missing\n")
        return 1
    src_files = ([ed / f for f in args.files] if args.files
                 else sorted(ed.glob("*.txt")))
    for f in src_files:
        if not f.exists():
            sys.stderr.write(f"ERROR: {f} not found\n")
            return 1

    cols = ("ts", "sat_truth", "beam_truth", "freq_truth",
            "bch1_ok", "bch2_ok", "bch3_ok",
            "bch_errs_total",
            "sat_dec", "beam_dec",
            "sat_match", "beam_match")
    print(",".join(cols))

    n_proc = n_too_short = n_all_bch_ok = n_sat_match = n_beam_match = 0
    t0 = time.time()
    for meta, iq_str in stream_records(src_files, args.max):
        iq = parse_iq(iq_str)
        r = decode_burst(iq)
        if r.get("too_short"):
            n_too_short += 1
            continue
        bch_ok = r["bch1_ok"] and r["bch2_ok"] and r["bch3_ok"]
        bch_errs = sum(max(0, r.get(f"bch{i}_errs", 0)) for i in (1, 2, 3))
        if bch_ok:
            n_all_bch_ok += 1
        sat_dec = r.get("sat_id_decoded", "")
        beam_dec = r.get("beam_id_decoded", "")
        sat_match = bch_ok and sat_dec == meta["sat_id"]
        beam_match = bch_ok and beam_dec == meta["beam_id"]
        if sat_match:
            n_sat_match += 1
        if beam_match:
            n_beam_match += 1
        print(",".join((
            f"{meta['ts']:.3f}",
            str(meta["sat_id"]), str(meta["beam_id"]),
            str(int(meta["freq_hz"])),
            "1" if r["bch1_ok"] else "0",
            "1" if r["bch2_ok"] else "0",
            "1" if r["bch3_ok"] else "0",
            str(bch_errs),
            str(sat_dec), str(beam_dec),
            "1" if sat_match else "0",
            "1" if beam_match else "0",
        )))
        n_proc += 1
        if n_proc % args.progress_every == 0:
            dt = time.time() - t0
            sys.stderr.write(f"  {n_proc:>10}: rate={n_proc/dt:.0f}/s "
                             f"all-bch-ok={100*n_all_bch_ok/n_proc:.1f}% "
                             f"sat_match={100*n_sat_match/n_proc:.1f}% "
                             f"beam_match={100*n_beam_match/n_proc:.1f}%\n")
            sys.stderr.flush()

    dt = time.time() - t0
    sys.stderr.write(f"\nDONE: {n_proc} processed in {dt:.1f}s "
                     f"({n_proc/dt:.0f}/s), {n_too_short} too short\n")
    if n_proc:
        sys.stderr.write(f"  all-3-BCH-OK:  {n_all_bch_ok}/{n_proc} "
                         f"({100*n_all_bch_ok/n_proc:.2f}%)\n")
        sys.stderr.write(f"  sat_id match:  {n_sat_match}/{n_proc} "
                         f"({100*n_sat_match/n_proc:.2f}%)\n")
        sys.stderr.write(f"  beam_id match: {n_beam_match}/{n_proc} "
                         f"({100*n_beam_match/n_proc:.2f}%)\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
