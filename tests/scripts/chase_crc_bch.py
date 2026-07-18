#!/usr/bin/env python3
"""chase_crc_bch.py — CRC-arbitrated Chase-2 soft-decision BCH prototype
for the LW.DA data section (task #16 reference; phase-0 follow-on to
demod_diff_biterr.py).

This is the REFERENCE the ESP32 C port would mirror. It runs entirely
on OUR demodulated bits + int16 soft metrics — NO ground truth in the
decision path. Ground truth is used only to *score* the result
(recovered-correct vs the safety population).

Decode model (mirrors ida_decode.c exactly for the hard path, then adds
the chase fallback):
  1. Data section = frame bits[70:382] -> pair_swap -> de-interleave ->
     10 BCH(31,20) codewords (poly 3545).
  2. Hard-decode each: syndrome==0 -> 0 err; coset-leader weight<=2 ->
     correct; else the codeword FAILED.
  3. If all 10 decode AND header(zero1==0) AND CRC-16==0 AND da_len>0 ->
     DA_OK already (not the chase population).
  4. Otherwise CHASE the FAILED codewords only:
       reliability(bit) = pair-MIN of adjacent DQPSK symbol magnitudes
       (a hard slip at symbol s corrupts the differential bits of s and
        s+1, so a bit is only as reliable as the weaker of its two
        parent symbols);
       take the L least-reliable positions of that codeword, enumerate
       the 2^L flip patterns, hard-syndrome-decode each (<=2), collect
       the DISTINCT candidate 20-bit messages.
  5. Combine candidates across the failed codewords into whole-frame
     message hypotheses; run each through the REAL frame CRC-16 (+ the
     same header/da_len gate ida_decode uses). Accept the frame iff a
     hypothesis passes. CRC-16 is the arbiter (same 1e-12 safety
     principle as --harder). Bounded by a per-frame CRC-check cap.

Device-C shape: hooks into ida_decode.c right after the 10-block hard
loop, on the !out->ok path only (post-hard-fail). Everything is
integer: reliability is a min() over existing int16 soft_bits, the
syndrome decode is the existing iridium_bch table, the CRC is the
existing crc16_ccitt_false. No float, no allocation beyond a small
candidate scratch (<= 10 * (2^L capped) 20-bit ints).

Usage:
  chase_crc_bch.py --parsed <.parsed> --manifest <manifest.csv> \
      --dump <dumpbits.txt> [--L 4,5,6] [--max-crc 20000]
"""
import argparse
import csv
import itertools
import sys
from collections import Counter

# reuse the validated truth reconstruction + primitives
sys.path.insert(0, __file__.rsplit("/", 1)[0])
from demod_diff_biterr import (          # noqa: E402
    LINE_RE, reconstruct_truth, ndivide, bch_encode, build_syndrome_table,
    bits_to_int, pair_swap, data_section_to_codewords, crc16_ccitt_false,
    LCW_TBL, UW_DL, UW_UL,
)

ACCH_POLY = 3545
SYN = None            # syndrome -> (weight, leader) for poly 3545, n=31
SYN29 = None          # lcw1 (7 bits)
SYN465 = None         # lcw2 (14 bits)
SYN41 = None          # lcw3 (26 bits)
LCW1_DA = None        # ft=2 lcw1 codeword


# ---- hard BCH decode of one 31-bit codeword (ida_decode semantics) ----
def hard_decode_cw(word):
    """Return (message_20bit, n_err) or (None, -1) if uncorrectable."""
    s = ndivide(ACCH_POLY, word, 31)
    if s == 0:
        return word >> 11, 0
    w, leader = SYN[s]
    if w <= 2:
        return (word ^ leader) >> 11, w
    return None, -1


# ---- LCW classification (strict clean-divide OR harder repair, DA) ----
def repair(poly, word, n, tmax):
    s = ndivide(poly, word, n)
    if s == 0:
        return word, 0
    tab = {3545: SYN, 465: SYN465, 41: SYN41, 29: SYN29}[poly]
    w, leader = tab[s]
    if w <= tmax:
        return word ^ leader, w
    return None, -1


def classify_da(bits):
    """Mirror iridium_frame.c strict+harder, DA-only. Returns True iff
    the frame classifies as LW.DA (ft==2)."""
    if len(bits) < 70:
        return False
    p = bits[24:70]
    sw = pair_swap(p)
    perm = [sw[LCW_TBL[k]] for k in range(46)]
    l1 = bits_to_int(perm[0:7])
    l3 = bits_to_int(perm[20:46])
    l2a = bits_to_int(perm[7:20] + [0])
    l2b = bits_to_int(perm[7:20] + [1])
    # strict: clean divides on l1/l3, l2 pad-guess clean via repair2==0.
    # harder: repair1 on l1 (t=1), repair2 on l3, repair2 on either l2 pad.
    r1, _ = repair(29, l1, 7, 1)
    if r1 is None:
        return False
    r3, _ = repair(41, l3, 26, 2)
    if r3 is None:
        return False
    ok2 = False
    for l2 in (l2a, l2b):
        rr, _ = repair(465, l2, 14, 2)
        if rr is not None:
            ok2 = True
            break
    if not ok2:
        return False
    ft = (r1 >> 4) & 0b111  # top 3 bits of the 7-bit lcw1 message
    return ft == 2


# ---- frame CRC/header gate (ida_decode DA_OK criterion) ---------------
STRICT_ZERO2 = False   # extra arbiter bit: bits[196:200] == 0


def frame_da_ok(msgs):
    """msgs = 10 ints (20-bit). Returns True iff the assembled 200-bit
    message satisfies ida_decode's DA_OK: header_ok(zero1==0) &&
    da_len>0 && CRC-16 residue == 0. With STRICT_ZERO2, also require the
    4-bit zero2 tail (bits[196:200]) to be 0 — iridium-toolkit enforces
    this (bitsparser `zero2 != 0` -> error); our ida_decode currently
    does not, but the device C could add it to widen the CRC arbiter's
    effective safety margin (16 -> 20 bits) for the chase path."""
    b = []
    for m in msgs:
        b += [(m >> (19 - k)) & 1 for k in range(20)]
    zero1 = (b[17] << 2) | (b[18] << 1) | b[19]
    if zero1 != 0:
        return False
    da_len = bits_to_int(b[11:16])
    if da_len == 0:
        return False
    if STRICT_ZERO2 and bits_to_int(b[196:200]) != 0:
        return False
    stream = b[0:20] + [0] * 12 + b[20:196]
    by = [bits_to_int(stream[i:i + 8]) for i in range(0, 208, 8)]
    return crc16_ccitt_false(by) == 0


# ---- reliability (pair-min of adjacent symbol magnitudes) -------------
def frame_conf(soft):
    """symbol magnitude per bit -> pair-min reliability per frame bit."""
    n = len(soft)
    mag = [abs(soft[2 * (p // 2)]) for p in range(n)]  # symbol mag
    conf = [0] * n
    for p in range(n):
        s = p // 2
        c = mag[2 * s]
        if s > 0:
            c = min(c, mag[2 * (s - 1)])
        conf[p] = c
    return conf


CW_IDX_MAP = None   # codeword bit -> frame bit index (bits[70:382] domain)


def chase_candidates(word, conf_cw, L):
    """Distinct candidate 20-bit messages from Chase over the L
    least-reliable positions of a failed codeword."""
    order = sorted(range(31), key=lambda k: conf_cw[k])[:L]
    cands = set()
    for mset in range(1 << L):
        w = word
        for j in range(L):
            if (mset >> j) & 1:
                w ^= 1 << (30 - order[j])
        msg, e = hard_decode_cw(w)
        if msg is not None:
            cands.add(msg)
    return cands


def chase_frame(bits, soft, L, max_crc):
    """Full decode-realistic chase. Returns dict:
        status: 'da_ok' | 'recovered' | 'fail'
        msgs:   assembled 10x20 message on success (else None)
        checks: CRC checks spent
        prod:   product of candidate counts (pre-cap)
    """
    if len(bits) < 382:
        return {"status": "fail", "msgs": None, "checks": 0, "prod": 0}
    rcw = [bits_to_int(cw) for cw in data_section_to_codewords(bits[70:382])]
    msgs = [None] * 10
    failed = []
    for i in range(10):
        m, e = hard_decode_cw(rcw[i])
        msgs[i] = m
        if m is None:
            failed.append(i)
    if not failed:
        return {"status": "da_ok" if frame_da_ok(msgs) else "fail",
                "msgs": msgs, "checks": 0, "prod": 0}

    # reliability, mapped to each failed codeword's 31 bits
    conf = frame_conf(soft)
    # CW_IDX_MAP maps codeword bit k -> index into bits[70:382]; add 70
    cand_lists = []
    for i in failed:
        conf_cw = [conf[70 + CW_IDX_MAP[i][k]] for k in range(31)]
        c = chase_candidates(rcw[i], conf_cw, L)
        if not c:
            return {"status": "fail", "msgs": None, "checks": 0,
                    "prod": 0}   # a failed cw with no candidate -> dead
        cand_lists.append(c)

    prod = 1
    for c in cand_lists:
        prod *= len(c)
    checks = 0
    for combo in itertools.product(*cand_lists):
        if checks >= max_crc:
            break
        for idx, i in enumerate(failed):
            msgs[i] = combo[idx]
        checks += 1
        if frame_da_ok(msgs):
            return {"status": "recovered", "msgs": list(msgs),
                    "checks": checks, "prod": prod}
    return {"status": "fail", "msgs": None, "checks": checks, "prod": prod}


def main():
    global SYN, SYN29, SYN465, SYN41, LCW1_DA, CW_IDX_MAP
    ap = argparse.ArgumentParser()
    ap.add_argument("--parsed", required=True)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--dump", required=True)
    ap.add_argument("--L", default="4,5,6")
    # Default = the safe operating point found in phase-0: a tight
    # per-frame CRC-check cap bounds the 2^-16 collision floor so
    # aggressive L can't admit a wrong CRC-valid frame. L=5 + cap 256
    # gives +11 with 0 false-accepts; L=6 is unsafe at ANY cap (an early
    # collision on one safety-population frame).
    ap.add_argument("--max-crc", type=int, default=256)
    ap.add_argument("--strict-zero2", action="store_true",
                    help="also require the 4-bit zero2 tail == 0 "
                         "(widens the CRC arbiter margin 16 -> 20 bits)")
    args = ap.parse_args()
    Ls = [int(x) for x in args.L.split(",")]
    global STRICT_ZERO2
    STRICT_ZERO2 = args.strict_zero2

    SYN = build_syndrome_table(3545, 31)
    SYN29 = build_syndrome_table(29, 7)
    SYN465 = build_syndrome_table(465, 14)
    SYN41 = build_syndrome_table(41, 26)
    LCW1_DA = bch_encode(29, 0b010, 3, 7)
    CW_IDX_MAP = data_section_to_codewords(list(range(312)))

    # manifest count (enumeration sanity)
    with open(args.manifest) as fh:
        n_man = sum(1 for _ in csv.DictReader(fh))

    # truth for gold rows (score recovered-correct)
    truth = {}
    i = -1
    with open(args.parsed) as fh:
        for line in fh:
            if not LINE_RE.match(line):
                continue
            i += 1
            if line.startswith("IDA:") and "CRC:OK" in line:
                t = reconstruct_truth(line)
                if t is not None:
                    truth[i] = [bits_to_int(t[j * 20:(j + 1) * 20])
                                for j in range(10)]
    assert i + 1 == n_man, f"{i+1} != {n_man}"

    # dump: per-burst frames
    bursts = {}
    with open(args.dump) as fh:
        for line in fh:
            f = line.rstrip("\n").split(",")
            if f[0] == "H":
                bursts[int(f[1])] = {"gold": f[3] == "1",
                                     "stage": f[2], "frames": []}
            elif f[0] == "B":
                bits = [int(c) for c in f[5]]
                soft = [int(x) for x in f[6].split(";")] if f[6] else None
                bursts[int(f[1])]["frames"].append((bits, soft))

    # pick the best DA-classified frame per burst (max hard blocks_ok);
    # mirrors the harness best-rank selection.
    def best_da_frame(b):
        best = None
        best_ok = -1
        for bits, soft in b["frames"]:
            if soft is None or not classify_da(bits) or len(bits) < 382:
                continue
            rcw = data_section_to_codewords(bits[70:382])
            nok = sum(1 for cw in rcw
                      if hard_decode_cw(bits_to_int(cw))[0] is not None)
            if nok > best_ok:
                best_ok = nok
                best = (bits, soft)
        return best

    print(f"manifest rows {n_man}, gold truth {len(truth)}", file=sys.stderr)

    for L in Ls:
        gold_daok = gold_recov = 0
        nak_daok = nak_recov = 0           # nak_daok should already be 0
        checks_hist = Counter()
        prod_hist = Counter()
        gold_wrong = 0                     # recovered but != truth (unsafe!)
        nak_checks_total = 0               # CRC checks spent over safety pop
        nak_chased = 0                     # nak frames that entered chase
        for idx, b in sorted(bursts.items()):
            fr = best_da_frame(b)
            if fr is None:
                continue
            bits, soft = fr
            r = chase_frame(bits, soft, L, args.max_crc)
            if b["gold"]:
                if r["status"] == "da_ok":
                    gold_daok += 1
                elif r["status"] == "recovered":
                    gold_recov += 1
                    checks_hist[r["checks"]] += 1
                    prod_hist[min(r["prod"], 10**9)] += 1
                    if idx in truth and r["msgs"] != truth[idx]:
                        gold_wrong += 1
            else:
                if r["status"] == "da_ok":
                    nak_daok += 1
                elif r["status"] == "recovered":
                    nak_recov += 1
                if r["checks"] > 0:
                    nak_chased += 1
                    nak_checks_total += r["checks"]

        base = gold_daok
        print(f"\n===== L={L}  (max CRC checks/frame = {args.max_crc}) =====")
        print(f"A yield (gold=568): hard DA_OK {base}  + chase-recovered "
              f"{gold_recov}  = {base + gold_recov}  "
              f"({100.0*(base+gold_recov)/568:.1f}% of gri)")
        print(f"    recovered-but-mismatched-truth (must be 0): {gold_wrong}")
        print(f"B safety (gri CRC-FAIL=825): hard-DA_OK {nak_daok}  "
              f"chase-false-accepts {nak_recov}  <-- MUST be 0")
        exp_fa = nak_checks_total / 65536.0
        print(f"    safety-population chase: {nak_chased} frames, "
              f"{nak_checks_total} CRC checks total -> expected CRC-16 "
              f"collisions ~{exp_fa:.2f} (2^-16 floor)")
        if checks_hist:
            cs = sorted(checks_hist)
            tot = sum(k * v for k, v in checks_hist.items())
            mx = max(checks_hist)
            med = sorted(k for k in checks_hist for _ in range(checks_hist[k]))
            print(f"C cost (recovered frames): CRC-checks "
                  f"min {cs[0]} median {med[len(med)//2]} max {mx} "
                  f"total {tot}")
            print(f"    checks histogram: "
                  f"{dict(sorted(checks_hist.items()))}")


if __name__ == "__main__":
    sys.exit(main())
