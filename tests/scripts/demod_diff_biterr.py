#!/usr/bin/env python3
"""demod_diff_biterr.py — phase-0 bit-error localisation for the demod
gap vs gr-iridium (companion to tests/host/test_demod_diff_gri.c
--dumpbits mode).

For every gri IDA CRC:OK burst, reconstructs the TRUE transmitted
frame content from the iridium-parser line (the pretty print is
lossless for the 200-bit post-BCH message: header fields + payload
bytes + CRC field + zero2 tail), re-encodes the 10 ACCH BCH(31,20)
codewords (poly 3545), and compares them against OUR raw demodulated
bits (pre-BCH) from the --dumpbits file. Answers, with numbers:

  * how many bit errors per codeword our failed frames carry
    (the "just over the t=2 limit" vs "fundamentally noisy" verdict);
  * where NO_DA losses actually happen (UW exact-match gate, LCW BCH,
    truncated demod, or data-section errors);
  * whether the erroring bits are the LOW-CONFIDENCE soft symbols,
    i.e. what a Chase-2 / soft-decision BCH would recover.

Truth caveat: the data-section comparison is exact (validated by the
CRC residue on every reconstructed message). The LCW words lcw2/lcw3
have no per-line truth (pretty print is lossy there); we report
min-distance-to-any-codeword from a syndrome coset-leader table —
exact as a distance, a lower bound as an error count. lcw1 truth is
exact (ft=2 for every DA frame).

Usage:
  demod_diff_biterr.py --parsed <.parsed> --manifest <manifest.csv> \
      --dump <dumpbits.txt> [--out-csv <per-burst.csv>] [--chase-l 5]
"""
import argparse
import csv
import re
import sys
from collections import Counter, defaultdict
from itertools import combinations

LINE_RE = re.compile(
    r"^([A-Z0-9]{3}):\s+\S+\s+(\d+\.\d+)\s+(\d+)\s+(\d+)%\s+"
    r"(-?[\d.]+)\|(-?[\d.]+)\|(-?[\d.]+)"
)
# IridiumDAMessage.pretty(): bits[0:3] cont=bits[3] bits[4] ctr=bits[5:8]
# bits[8:11] len=%02d 0:bits[16:20] [payload] crc/res CRC:OK bits[196:200]
IDA_BODY_RE = re.compile(
    r" ([01]{3}) cont=([01]) ([01]) ctr=([01]{3}) ([01]{3}) len=(\d+)"
    r" 0:([01]{4}) \[([0-9a-f.!]*)\]\s+"
    r"(?:([0-9a-f]{4})/([0-9a-f]{4}) CRC:(OK|no)|---)\s+([01]{4})"
)

ACCH_POLY = 3545
CW_N = 31
CW_K = 20

UW_DL = [0,0,1,1,0,0,0,0,0,0,1,1,0,0,0,0,1,1,1,1,0,0,1,1]
UW_UL = [1,1,0,0,1,1,0,0,0,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0]

# iridium_frame.c LCW_TBL (0-based)
LCW_TBL = [39,38,35,34,31,30,27,26,23,22,19,18,15,14,11,10,7,6,3,2,
           40,37,36,33,32,29,28,25,24,21,20,17,16,13,12,9,8,5,4,1,0,
           45,44,43,42,41]


def bl(x):
    return x.bit_length()


def ndivide(poly, word, n_bits):
    """iridium_bch_ndivide on an int word of n_bits."""
    num = word
    if num == 0:
        return 0
    num_len = bl(num)
    shift = num_len - bl(poly)
    pw = 1 << (num_len - 1)
    while shift >= 0:
        if num >= pw:
            num ^= poly << shift
        pw >>= 1
        shift -= 1
    return num


def bch_encode(poly, msg, k, n):
    """systematic encode: msg (int, k bits) -> codeword (int, n bits)."""
    word = msg << (n - k)
    rem = ndivide(poly, word, n)
    return word | rem


def build_syndrome_table(poly, n, max_w=6):
    """syndrome -> (coset leader weight, leader pattern int). Exact
    min-distance-to-code for any received word (covering radius of these
    codes is < max_w; assert full coverage)."""
    tab = {0: (0, 0)}
    for w in range(1, max_w + 1):
        for pos in combinations(range(n), w):
            e = 0
            for p in pos:
                e |= 1 << p
            s = ndivide(poly, e, n)
            if s not in tab:
                tab[s] = (w, e)
        if len(tab) == 1 << (bl(poly) - 1):
            break
    return tab


def bits_to_int(bits):
    v = 0
    for b in bits:
        v = (v << 1) | (b & 1)
    return v


def pair_swap(seq):
    out = list(seq)
    for i in range(0, len(out) - 1, 2):
        out[i], out[i + 1] = out[i + 1], out[i]
    return out


def de_interleave(seq):
    """ida_decode.c de_interleave: returns (odd, even)."""
    n_sym = len(seq) // 2
    odd, even = [], []
    for s in range(n_sym - 1, -1, -2):
        odd += [seq[2 * s + 1], seq[2 * s]]
    for s in range(n_sym - 2, -1, -2):
        even += [seq[2 * s + 1], seq[2 * s]]
    return odd, even


def data_section_to_codewords(seq312):
    """Mirror ida_decode.c: 312 'items' (bits or indices) -> list of 10
    31-item codewords."""
    seq = pair_swap(seq312)
    cws = []
    for chunk in (seq[0:124], seq[124:248]):
        b1, b2 = de_interleave(chunk)
        cat = b1 + b2
        cws += [cat[93:124], cat[31:62], cat[62:93], cat[0:31]]
    b1, b2 = de_interleave(seq[248:312])
    cws += [b2[1:32], b1[1:32]]
    return cws


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def reconstruct_truth(line):
    """Parse an IDA CRC:OK pretty line -> 200-bit message (list of 0/1)
    or None. Validates via the CRC residue; brute-forces the one byte
    the pretty print can silently omit (da_ta[da_len] when the
    all-zero check starts at da_len+1)."""
    m = IDA_BODY_RE.search(line)
    if not m or m.group(11) != "OK":
        return None
    f1, cont, f1b, ctr, f2, dlen, z16, pl, crcr, _, _, z2 = m.groups()
    da_len = int(dlen)
    bits = [int(c) for c in f1] + [int(cont), int(f1b)] + \
           [int(c) for c in ctr] + [int(c) for c in f2]
    bits += [(da_len >> (4 - k)) & 1 for k in range(5)]
    bits += [int(c) for c in z16]
    payload = [int(x, 16) for x in pl.replace("!", ".").split(".") if x]
    pay20 = (payload + [0] * 20)[:20]
    crc_field = int(crcr, 16)

    def build(pay):
        b = list(bits)
        for by in pay:
            b += [(by >> (7 - k)) & 1 for k in range(8)]
        b += [(crc_field >> (15 - k)) & 1 for k in range(16)]
        b += [int(c) for c in z2]
        return b

    def crc_ok(b):
        # crcstream = bits[0:20] + '0'*12 + bits[20:196] -> 26 bytes
        stream = b[0:20] + [0] * 12 + b[20:196]
        by = [bits_to_int(stream[i:i + 8]) for i in range(0, 208, 8)]
        return crc16_ccitt_false(by) == 0

    b = build(pay20)
    if crc_ok(b):
        return b
    # brute-force the possibly-omitted byte at index da_len
    if da_len < 20 and len(payload) <= da_len:
        for v in range(1, 256):
            pay = list(pay20)
            pay[da_len] = v
            b = build(pay)
            if crc_ok(b):
                return b
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--parsed", required=True)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--dump", required=True)
    ap.add_argument("--out-csv", default=None)
    ap.add_argument("--chase-l", type=int, default=5)
    args = ap.parse_args()

    # --- manifest: idx -> row --------------------------------------
    man = {}
    with open(args.manifest) as fh:
        for r in csv.DictReader(fh):
            man[int(r["idx"])] = r

    # --- parsed lines, same enumeration as demod_diff_extract.py ---
    truth = {}       # idx -> 200-bit msg
    truth_fail = 0
    i = -1
    with open(args.parsed) as fh:
        for line in fh:
            if not LINE_RE.match(line):
                continue
            i += 1
            if "CRC:OK" not in line or not line.startswith("IDA:"):
                continue
            t = reconstruct_truth(line)
            if t is None:
                truth_fail += 1
                continue
            truth[i] = t
    # sanity: enumeration must match the manifest
    assert i + 1 == len(man), f"line count {i+1} != manifest {len(man)}"
    print(f"truth reconstructed for {len(truth)} IDA CRC:OK lines "
          f"({truth_fail} failed CRC-validation)", file=sys.stderr)

    # truth codewords per idx
    truth_cw = {}
    for idx, msg in truth.items():
        truth_cw[idx] = [bch_encode(ACCH_POLY, bits_to_int(msg[i*20:(i+1)*20]),
                                    CW_K, CW_N) for i in range(10)]

    # --- syndrome tables --------------------------------------------
    syn3545 = build_syndrome_table(3545, 31)   # data cw min-dist + decode
    syn465 = build_syndrome_table(465, 14)     # lcw2 (13 + pad bit)
    syn41 = build_syndrome_table(41, 26)       # lcw3
    lcw1_true = bch_encode(29, 0b010, 3, 7)    # ft=2 (DA)

    # index maps: codeword bit position -> raw frame bit index
    cw_idx_map = data_section_to_codewords(list(range(70, 382)))
    lcw_idx_map = [24 + (LCW_TBL[i] ^ 1) for i in range(46)]  # ^1 = pair swap

    # --- dump file ---------------------------------------------------
    bursts = {}   # idx -> dict(stage=..., frames=[(bits,soft,dir)])
    with open(args.dump) as fh:
        for line in fh:
            f = line.rstrip("\n").split(",")
            if f[0] == "H":
                bursts[int(f[1])] = {"stage": f[2], "gold": f[3] == "1",
                                     "snr": float(f[4]), "frames": []}
            elif f[0] == "B":
                idx = int(f[1])
                bits = [int(c) for c in f[5]]
                soft = [int(x) for x in f[6].split(";")] if f[6] else None
                bursts[idx]["frames"].append((bits, soft, int(f[4])))

    # --- per-burst analysis ------------------------------------------
    def analyse_frame(bits, soft, direction, tcws):
        """Return dict of error metrics for one demod frame vs truth."""
        n = len(bits)
        uw_pat = UW_UL if direction == 1 else UW_DL
        uw_err = sum(1 for k in range(min(24, n)) if bits[k] != uw_pat[k])
        res = {"n_bits": n, "uw_err": uw_err, "trunc": n < 382}
        # LCW (needs 70 bits)
        if n >= 70:
            sw = pair_swap(bits[24:70])
            perm = [sw[LCW_TBL[k]] for k in range(46)]
            l1 = bits_to_int(perm[0:7])
            l3 = bits_to_int(perm[20:46])
            l2a = bits_to_int(perm[7:20] + [0])
            l2b = bits_to_int(perm[7:20] + [1])
            res["lcw1_err"] = bin(l1 ^ lcw1_true).count("1")
            res["lcw2_d"] = min(syn465[ndivide(465, l2a, 14)][0],
                                syn465[ndivide(465, l2b, 14)][0])
            res["lcw3_d"] = syn41[ndivide(41, l3, 26)][0]
        # data codewords
        if n >= 382:
            rcw = data_section_to_codewords(bits[70:382])
            errs = []
            for i in range(10):
                r = bits_to_int(rcw[i])
                errs.append(bin(r ^ tcws[i]).count("1"))
            res["cw_err"] = errs
        return res

    def frame_score(r):
        s = (r["uw_err"] + r.get("lcw1_err", 9) + r.get("lcw2_d", 9)
             + r.get("lcw3_d", 9))
        s += sum(r["cw_err"]) if "cw_err" in r else 999
        return s

    chase_L = args.chase_l

    def chase_cw(bits, soft, cw_i, tcw):
        """Chase test on data codeword cw_i: flip subsets of the L
        least-reliable of its 31 bits, syndrome-decode (<=2), collect
        candidates. Returns (truth_in_candidates, soft_best_is_truth,
        min_L_covering_errors)."""
        pos = cw_idx_map[cw_i]              # frame bit index per cw bit
        rbits = [bits[p] for p in pos]
        # DQPSK reliability: qpsk_demod assigns |pll_out(sym)| to both
        # bits of a symbol, but a hard-decision slip at symbol s
        # corrupts the DIFFERENTIALLY decoded bits of symbols s AND
        # s+1 — so a bit's reliability is the WEAKER of its own symbol
        # and the previous one.
        def rel(p):
            s = p // 2
            c = abs(soft[2 * s])
            if s > 0:
                c = min(c, abs(soft[2 * (s - 1)]))
            return c
        conf = [rel(p) for p in pos]
        r = bits_to_int(rbits)
        err = r ^ tcw
        err_pos = [k for k in range(31) if (err >> (30 - k)) & 1]
        order = sorted(range(31), key=lambda k: conf[k])   # least reliable first
        rank = {k: i for i, k in enumerate(order)}
        minL = 1 + max(rank[k] for k in err_pos) if err_pos else 0
        low = order[:chase_L]
        cands = {}
        for mset in range(1 << chase_L):
            w = r
            for j in range(chase_L):
                if (mset >> j) & 1:
                    w ^= 1 << (30 - low[j])
            s = ndivide(3545, w, 31)
            lw, le = syn3545.get(s, (99, 0))
            if lw <= 2:
                c = w ^ le
                # soft metric: sum |conf| over bits flipped from received
                d = c ^ r
                cost = sum(conf[k] for k in range(31) if (d >> (30 - k)) & 1)
                if c not in cands or cost < cands[c]:
                    cands[c] = cost
        in_cand = tcw in cands
        best = min(cands, key=cands.get) if cands else None
        return in_cand, best == tcw, minL

    rows = []
    for idx, b in sorted(bursts.items()):
        if not b["gold"] or idx not in truth_cw:
            continue
        tcws = truth_cw[idx]
        frames = b["frames"]
        row = {"idx": idx, "stage": b["stage"], "snr": b["snr"],
               "n_frames": len(frames)}
        best = None
        for bits, soft, d in frames:
            r = analyse_frame(bits, soft, d, tcws)
            if best is None or frame_score(r) < frame_score(best[0]):
                best = (r, bits, soft, d)
        if best:
            r, bits, soft, d = best
            row.update(r)
            if "cw_err" in r and soft is not None:
                ch = [chase_cw(bits, soft, i, tcws[i])
                      for i in range(10)]
                row["chase_oracle"] = all(c[0] for i, c in enumerate(ch)
                                          if r["cw_err"][i] > 2)
                row["chase_soft"] = all(c[1] for i, c in enumerate(ch)
                                        if r["cw_err"][i] > 2)
                row["minL_worst"] = max((c[2] for i, c in enumerate(ch)
                                         if r["cw_err"][i] > 2), default=0)
        rows.append(row)

    # ------------------------- report --------------------------------
    gold_stages = Counter(r["stage"] for r in rows)
    print("\n== gold bursts analysed:", len(rows), dict(gold_stages))

    fails = [r for r in rows if r["stage"] != "DA_OK"]
    demod = [r for r in fails if "n_bits" in r]     # got at least 1 frame
    print(f"failed gold bursts: {len(fails)} (with demod frame: {len(demod)})")

    # failure cause decomposition
    def classify_fail(r):
        if "n_bits" not in r:
            return "no_frame"
        if r.get("trunc"):
            return "truncated"
        cw = r.get("cw_err", [])
        data_bad = sum(1 for e in cw if e > 2)
        uw_bad = r["uw_err"] > 0
        # classify_lw accepts lcw1/lcw3 only on CLEAN divide and lcw2
        # only when one of the two pad guesses divides cleanly — so ANY
        # nonzero distance on any LCW word is a classification failure.
        lcw_bad = (r.get("lcw1_err", 0) > 0 or r.get("lcw2_d", 0) > 0
                   or r.get("lcw3_d", 0) > 0)
        if data_bad == 0 and not uw_bad and not lcw_bad:
            return "none?!"
        parts = []
        if uw_bad:
            parts.append("UW")
        if lcw_bad:
            parts.append("LCW")
        if data_bad:
            parts.append("DATA")
        return "+".join(parts)

    causes = Counter(classify_fail(r) for r in fails)
    print("\n== failure cause decomposition (gold, ours != DA_OK) ==")
    for k, v in causes.most_common():
        print(f"  {k:12s} {v}")

    # error-per-codeword histogram over failed frames' bad codewords
    full = [r for r in fails if "cw_err" in r]
    worst = Counter()
    all_bad = Counter()
    nbad_per_frame = Counter()
    for r in full:
        bad = [e for e in r["cw_err"] if e > 2]
        nbad_per_frame[len(bad)] += 1
        for e in bad:
            all_bad[e] += 1
        if bad:
            worst[max(bad)] += 1
    print("\n== failed-frame worst-codeword error histogram ==")
    for e in sorted(worst):
        print(f"  worst={e:2d} errors: {worst[e]} frames")
    print("== all uncorrectable codewords (err>2) histogram ==")
    for e in sorted(all_bad):
        print(f"  {e:2d} errors: {all_bad[e]} codewords")
    print("== uncorrectable codewords per failed frame ==")
    for k in sorted(nbad_per_frame):
        print(f"  {k:2d} bad cws: {nbad_per_frame[k]} frames")

    # chase recoverability
    ch_frames = [r for r in full
                 if any(e > 2 for e in r["cw_err"]) and "chase_oracle" in r]
    orc = sum(1 for r in ch_frames if r["chase_oracle"])
    sft = sum(1 for r in ch_frames if r["chase_soft"])
    print(f"\n== Chase-{chase_L} (data cws only) on {len(ch_frames)} "
          f"data-failed frames: oracle-recoverable {orc}, "
          f"soft-pick-recoverable {sft}")
    minl = Counter(r.get("minL_worst") for r in ch_frames)
    print("   minimal per-cw flip-set size L needed (worst cw per frame):")
    for k in sorted(x for x in minl if x is not None):
        print(f"     L>={k}: {minl[k]} frames")

    # UW/LCW-only quick wins
    uwlcw_only = [r for r in fails if classify_fail(r) in
                  ("UW", "LCW", "UW+LCW")]
    print(f"\n== frames failing ONLY on UW/LCW (data all correctable): "
          f"{len(uwlcw_only)}")
    uw_hist = Counter(r["uw_err"] for r in uwlcw_only)
    print("   uw_err histogram:", dict(sorted(uw_hist.items())))

    if args.out_csv:
        keys = ["idx", "stage", "snr", "n_frames", "n_bits", "trunc",
                "uw_err", "lcw1_err", "lcw2_d", "lcw3_d", "cw_err",
                "chase_oracle", "chase_soft", "minL_worst"]
        with open(args.out_csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(keys)
            for r in rows:
                w.writerow([r.get(k, "") for k in keys])
        print(f"wrote {args.out_csv}", file=sys.stderr)


if __name__ == "__main__":
    sys.exit(main())
