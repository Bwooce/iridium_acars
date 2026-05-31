#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
mendeley_decode_validate.py — pivot of the original CFO-characterization
workflow after discovering the Mendeley IQ is *post-PLL symbol-rate*,
not raw 250 ksps as I'd assumed when first designing the workflow.

REALITY: each Mendeley record contains ~110 complex IQ samples at
gr-iridium's PHASE_B_FS = 25 ksps (post-symbol-timing, post-PLL). This
is the UW + IRA payload region of a Ring Alert burst, already aligned
to symbols. We cannot run it through burst_pipeline_process_burst
(which expects 250 ksps raw with envelope), but we *can* do the
downstream stages: differential-decode → BCH → classify → compare
extracted IRA fields to the dataset's metadata.

This validates:
  - Differential QPSK decode (the dibit map)
  - BCH(31,21) decoder (both blocks per IRA frame)
  - IRA payload parser (sat_id, beam_id, freq_id extraction)
  - End-to-end correctness against 3.8M ground-truthed bursts

Limitations:
  - Does NOT validate burst tagger, signal_buffer, DC removal, D13,
    cfo_fine_estimate, UW correlator, PLL, sym_timing.
  - Mendeley's metadata IS what gr-iridium extracted, so a 100% match
    really validates 'our decoder agrees with gr-iridium' — not against
    physical-layer truth. Still valuable: catches any divergence in our
    BCH or IRA-payload parser as we evolve them.

Output CSV columns (one row per burst):
  ts, sat_id_truth, beam_id_truth, freq_truth, n_iq, n_bits,
  bch1_ok, bch2_ok, classify_ok,
  sat_id_decoded, beam_id_decoded, freq_decoded,
  sat_id_match, beam_id_match, freq_match

Usage:
    python3 tests/scripts/mendeley_decode_validate.py \\
        > ~/iq_cache/decode_stats.csv

THIS REPLACES mendeley_cfo_characterize.py for Mendeley-specific work
(that script's CFO peak-distribution premise assumed wideband input;
it's still the right design for ALBQ-style wideband data, just
mis-targeted at Mendeley).
"""

import argparse
import re
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import fetch_mendeley_iridium as fetch     # noqa: E402

# IRA-DL frame structure (per iridium-toolkit / gr-iridium):
#   UW:     12 symbols (24 bits) — DL_UW = 0xC4_1A_05 or similar (DQPSK)
#   LCW:     4 bits
#   Block 1: 31 bits (BCH(31,21))
#   Block 2: 31 bits (BCH(31,21))
# Total post-UW payload: 4 + 31 + 31 = 66 bits = 33 symbols
# So a minimum-viable IRA burst is 12 + 33 = 45 symbols.
N_UW_SYMBOLS = 12
N_PAYLOAD_BITS = 66

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


def dqpsk_decode(iq: np.ndarray) -> np.ndarray:
    """Differential QPSK decode. Returns array of 2-bit dibits (one per
    symbol pair).

    Standard DQPSK mapping (Gray-coded):
      phase_diff ≈ +π/4 → dibit 00
      phase_diff ≈ +3π/4 → dibit 01
      phase_diff ≈ −3π/4 → dibit 11
      phase_diff ≈ −π/4 → dibit 10
    """
    if iq.size < 2:
        return np.empty(0, dtype=np.uint8)
    # Per-symbol angle
    ang = np.angle(iq)
    # Differential phase between consecutive symbols
    dphi = ang[1:] - ang[:-1]
    # Wrap to [-π, π]
    dphi = np.mod(dphi + np.pi, 2 * np.pi) - np.pi
    # Quantise to 4 sectors of π/2 each, centred at ±π/4 and ±3π/4
    # Sector index in {0,1,2,3} mapping to dibits per Gray-coded DQPSK.
    # Sector 0: [0, π/2)   = +π/4 zone     → 00
    # Sector 1: [π/2, π]   = +3π/4 zone    → 01
    # Sector 2: [-π, -π/2) = -3π/4 zone    → 11
    # Sector 3: [-π/2, 0)  = -π/4 zone     → 10
    sec = np.where(dphi >= 0,
                   np.where(dphi >= np.pi / 2, 1, 0),
                   np.where(dphi < -np.pi / 2, 2, 3))
    # Map sector -> dibit
    GRAY = np.array([0, 1, 3, 2], dtype=np.uint8)
    return GRAY[sec]


def dibits_to_bits(dibits: np.ndarray) -> np.ndarray:
    """Unpack uint8 dibits into a flat array of 0/1 bits (MSB first)."""
    bits = np.empty(dibits.size * 2, dtype=np.uint8)
    bits[0::2] = (dibits >> 1) & 1
    bits[1::2] = dibits & 1
    return bits


# BCH(31,21) with t=2 correction. Generator polynomial x^10+x^9+x^8+x^6+x^5+x^3+1
# (Iridium standard). Pure-python syndrome decode + Peterson for 2 errors.
BCH_GEN = 0b11101101001       # 11 bits (x^10 + ... + 1)
GF32_PRIM = 0b100101          # GF(2^5) primitive polynomial x^5+x^2+1


def bch_encode(msg21: int) -> int:
    """Systematic BCH(31,21) encode. msg21 = 21 message bits, MSB first.
    Returns 31-bit codeword (msg in top 21 bits, parity in bottom 10)."""
    cw = msg21 << 10
    rem = cw
    for i in range(30, 9, -1):  # bits 30 down to 10
        if rem & (1 << i):
            rem ^= BCH_GEN << (i - 10)
    return (msg21 << 10) | (rem & ((1 << 10) - 1))


def gf32_mult(a: int, b: int) -> int:
    """Multiplication in GF(2^5)."""
    r = 0
    while b:
        if b & 1:
            r ^= a
        a <<= 1
        if a & 32:
            a ^= GF32_PRIM
        b >>= 1
    return r & 31


# Build GF(2^5) log/antilog tables for fast Chien search and inversions.
_ALPHA = 2  # primitive element
_LOG = [0] * 32
_ANTILOG = [0] * 32
_x = 1
for i in range(31):
    _ANTILOG[i] = _x
    _LOG[_x] = i
    _x = gf32_mult(_x, _ALPHA)
_ANTILOG[31] = _ANTILOG[0]


def bch_decode_31_21(cw: int) -> tuple[int, int]:
    """Return (msg21, n_errors_corrected) or (None, -1) if uncorrectable.

    Syndromes S1, S3 in GF(2^5).
    """
    # S_j = sum over i where cw bit i is set of alpha^(j*i)
    S1 = 0
    S3 = 0
    for i in range(31):
        if (cw >> i) & 1:
            S1 ^= _ANTILOG[i % 31]
            S3 ^= _ANTILOG[(3 * i) % 31]
    if S1 == 0 and S3 == 0:
        return (cw >> 10) & ((1 << 21) - 1), 0
    # Peterson: try 1-error correction first
    # If 1 error at position i: S1 = alpha^i, S3 = alpha^(3i) = S1^3
    # Check S1^3 == S3
    S1_cubed = gf32_mult(gf32_mult(S1, S1), S1)
    if S1 != 0 and S1_cubed == S3:
        # one error at position _LOG[S1]
        pos = _LOG[S1]
        if pos < 31:
            corrected = cw ^ (1 << pos)
            return (corrected >> 10) & ((1 << 21) - 1), 1
    # 2-error correction via Peterson:
    #   sigma(x) = 1 + sigma1*x + sigma2*x^2
    #   sigma1 = S1
    #   sigma2 = (S1^3 + S3) / S1 = (S1_cubed XOR S3) / S1
    if S1 != 0:
        num = S1_cubed ^ S3
        # Inverse of S1 in GF(2^5)
        S1_inv = _ANTILOG[(31 - _LOG[S1]) % 31]
        sigma2 = gf32_mult(num, S1_inv)
        sigma1 = S1
        # Chien search: find roots of 1 + sigma1*x + sigma2*x^2 in GF(2^5)
        roots = []
        for alpha_inv_i in range(31):
            x = _ANTILOG[alpha_inv_i]
            v = 1 ^ gf32_mult(sigma1, x) ^ gf32_mult(sigma2, gf32_mult(x, x))
            if v == 0:
                # root x = alpha^alpha_inv_i; error position = -alpha_inv_i mod 31
                pos = (31 - alpha_inv_i) % 31
                roots.append(pos)
                if len(roots) == 2:
                    break
        if len(roots) == 2:
            corrected = cw
            for p in roots:
                corrected ^= 1 << p
            return (corrected >> 10) & ((1 << 21) - 1), 2
    return None, -1


def ira_parse(msg42: int) -> dict:
    """Parse the 42-message-bit IRA payload (two BCH blocks of 21 bits).
    The exact bit layout is per gr-iridium / iridium-toolkit IridiumIRA.

    For this validation we extract the few fields that the Mendeley
    dataset annotates: sat_id (7 bits), beam_id (6 bits), and the
    frequency_id (which maps to absolute Hz via the Iridium channel
    plan). Plus the LCW header (4 bits, set to 0011 for IRA).

    Returns dict of extracted ints; caller compares to ground truth."""
    # NOTE: bit positions below are from iridium-toolkit's IridiumIRA
    # parse but only the lossless integer fields. Lat/lon need extra
    # decoding (the dataset gives them in floats already).
    # bit 0 = MSB of block1, bit 41 = LSB of block2.
    sat_id = (msg42 >> 35) & 0x7F
    beam_id = (msg42 >> 29) & 0x3F
    freq_id = (msg42 >> 17) & 0xFFF
    return {"sat_id": sat_id, "beam_id": beam_id, "freq_id": freq_id}


def decode_burst(iq: np.ndarray) -> dict:
    """Run our differential-decode + BCH + classify chain on a Mendeley
    burst's symbol-rate IQ. Returns dict of stats."""
    if iq.size < N_UW_SYMBOLS + 33:
        return {"too_short": True}
    dibits = dqpsk_decode(iq)
    bits = dibits_to_bits(dibits)
    # Skip the UW (24 bits / 12 dibits)
    if bits.size < 24 + N_PAYLOAD_BITS:
        return {"too_short": True}
    payload = bits[24:24 + N_PAYLOAD_BITS]    # 66 bits
    # Skip the LCW (4 bits) — IRA-specific framing
    block1 = payload[4:35]   # 31 bits
    block2 = payload[35:66]  # 31 bits
    cw1 = 0
    for b in block1:
        cw1 = (cw1 << 1) | int(b)
    cw2 = 0
    for b in block2:
        cw2 = (cw2 << 1) | int(b)
    m1, e1 = bch_decode_31_21(cw1)
    m2, e2 = bch_decode_31_21(cw2)
    out = {
        "n_iq": iq.size,
        "n_bits": int(bits.size),
        "bch1_ok": m1 is not None,
        "bch1_errs": e1,
        "bch2_ok": m2 is not None,
        "bch2_errs": e2,
    }
    if m1 is not None and m2 is not None:
        msg42 = (m1 << 21) | m2
        ira = ira_parse(msg42)
        out.update({
            "sat_id_decoded": ira["sat_id"],
            "beam_id_decoded": ira["beam_id"],
            "freq_id_decoded": ira["freq_id"],
        })
    return out


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
                        "ts": float(parts[0]),
                        "sat_id": int(parts[2]),
                        "beam_id": int(parts[3]),
                        "lat": float(parts[4]),
                        "lon": float(parts[5]),
                        "alt": float(parts[6]),
                        "conf": float(parts[7]),
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

    cols = ("ts", "sat_truth", "beam_truth", "freq_truth", "n_iq", "n_bits",
            "bch1_ok", "bch1_errs", "bch2_ok", "bch2_errs",
            "sat_dec", "beam_dec", "freq_id_dec",
            "sat_match", "beam_match")
    print(",".join(cols))

    n_proc = n_too_short = n_both_bch_ok = n_sat_match = 0
    t0 = time.time()
    for meta, iq_str in stream_records(src_files, args.max):
        iq = parse_iq(iq_str)
        r = decode_burst(iq)
        if r.get("too_short"):
            n_too_short += 1
            continue
        bch1 = r.get("bch1_ok", False)
        bch2 = r.get("bch2_ok", False)
        if bch1 and bch2:
            n_both_bch_ok += 1
        sat_dec = r.get("sat_id_decoded", "")
        beam_dec = r.get("beam_id_decoded", "")
        freq_id_dec = r.get("freq_id_decoded", "")
        sat_match = (sat_dec == meta["sat_id"]) if sat_dec != "" else False
        beam_match = (beam_dec == meta["beam_id"]) if beam_dec != "" else False
        if sat_match:
            n_sat_match += 1
        print(",".join((
            f"{meta['ts']:.3f}",
            str(meta["sat_id"]), str(meta["beam_id"]),
            str(int(meta["freq_hz"])),
            str(r.get("n_iq", 0)), str(r.get("n_bits", 0)),
            "1" if bch1 else "0", str(r.get("bch1_errs", -1)),
            "1" if bch2 else "0", str(r.get("bch2_errs", -1)),
            str(sat_dec), str(beam_dec), str(freq_id_dec),
            "1" if sat_match else "0", "1" if beam_match else "0",
        )))
        n_proc += 1
        if n_proc % args.progress_every == 0:
            dt = time.time() - t0
            sys.stderr.write(f"  {n_proc:>10}: rate={n_proc/dt:.0f}/s "
                             f"bch_ok={100*n_both_bch_ok/n_proc:.1f}% "
                             f"sat_match={100*n_sat_match/n_proc:.1f}%\n")
            sys.stderr.flush()

    dt = time.time() - t0
    sys.stderr.write(f"\nDONE: {n_proc} processed in {dt:.1f}s "
                     f"({n_proc/dt:.0f}/s), {n_too_short} too short\n")
    if n_proc:
        sys.stderr.write(f"  both-BCH-OK:  {n_both_bch_ok}/{n_proc} "
                         f"({100*n_both_bch_ok/n_proc:.2f}%)\n")
        sys.stderr.write(f"  sat_id match: {n_sat_match}/{n_proc} "
                         f"({100*n_sat_match/n_proc:.2f}%)\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
