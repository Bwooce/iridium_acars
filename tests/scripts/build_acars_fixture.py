#!/usr/bin/env python3
"""
Build a host/device-test fixture from the 2026-07-06 HydraSDR ACARS
milestone capture (~/iridium_bits/acars-milestone-20260706.{parsed,txt}).

Ground truth: acars-milestone-20260706.txt is iridium-toolkit's
`reassembler.py -m acars` pretty-printed output over the milestone
capture -- 7 ACARS "demand mode" pings from REG A62001, an
independent reference (not produced by any code in this repo).
acars-milestone-20260706.parsed is the underlying frame-level (IDA:/
RAW:/...) trace that reference was derived from.

Finding (see docs/superpowers/plans/2026-07-06-acars-smoke-test.md and
this script's derivation below): EVERY one of the 7 reference ACARS
messages needs its SBD envelope split across TWO physical LW.DA bursts
(iridium-toolkit's own stats confirm this: "46 valid packets assembled
from 178 fragments (1:3.87)" -- i.e. this capture's SBD/ACARS traffic
routinely needs multiple over-the-air fragments per message). A single
LW.DA burst caps at 24 payload bytes (5-bit da_len field); these 37-
byte demand-mode envelopes need 2 bursts chained via the da_cont/
da_ctr header fields -- a lower, separate layer from the SBD envelope's
own (msgno/msgcnt) multi-frame chaining that sbd_reassembler.c already
implemented. See common/iridium_decoder/ida_reassembler.{h,c} (added
alongside this script) for the production-code fix this necessitated.

This script:
  1. Parses every CRC:OK "IDA:" line out of the milestone .parsed file
     (same regex iridium-toolkit's ida.py filter() uses).
  2. Re-implements ida.py's ReassembleIDA.process() fragment-chaining
     algorithm (frequency deadband, ctr sequencing, time window) to
     assemble complete multi-fragment byte streams, while remembering
     which raw IDA: lines contributed to each.
  3. Decodes each assembled stream's SBD+ACARS fields using
     iridium-toolkit's OWN ReassembleIDASBDACARS class directly (not a
     reimplementation) so the "ground truth" fields below come from
     the reference toolchain, not our code.
  4. Matches decoded messages against acars-milestone-20260706.txt by
     (REG, ACK, block_id) to confirm each fixture message really is
     one of the 7 independently-referenced ones.
  5. Emits tests/fixtures/fixture_acars_frames.h: for each matched
     message, its constituent LW.DA fragments (da_cont, da_ctr,
     payload bytes, freq_hz, timestamp_us) in over-the-air order, plus
     the expected ACARS fields to assert against.

Usage: tests/scripts/build_acars_fixture.py [--max-messages N]
"""
from __future__ import annotations
import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
FIXTURE_DIR = REPO_ROOT / "tests" / "fixtures"
IRIDIUM_TOOLKIT_DIR = REPO_ROOT / "iridium-toolkit"

MILESTONE_DIR = Path("/home/bruce/iridium_bits")
PARSED_PATH = MILESTONE_DIR / "acars-milestone-20260706.parsed"
REFERENCE_TXT = MILESTONE_DIR / "acars-milestone-20260706.txt"

IDA_LINE_RE = re.compile(
    r'^IDA: (\S+) (\S+) (\d+)\s+(\d+)%\s+(\S+)\s+(\d+)\s+(UL|DL)\s+(.*)$'
)
IDA_BODY_RE = re.compile(
    r'.* cont=(\d) (\d) ctr=(\d+) \d+ len=(\d+) 0:0000 \[([0-9a-f.!]*)\]\s+\S+\s+CRC:OK'
)
REF_LINE_RE = re.compile(
    r'^(\S+) \[hdr: (\S+)\] Dir:(\S+) Mode:(\S+) REG:(\S+)\s+ACK:(\S+) '
    r'Label:\S+ \((.*?)\) bID:(\S+)'
)

FREQ_DEADBAND_HZ = 260  # matches iridiumtk/reassembler/ida.py
FRAG_GAP_S = 0.280      # matches ida.py: time[-1] <= m.time <= time[-1]+280ms


class Fragment:
    __slots__ = ("lineno", "time_s", "freq_hz", "cont", "ctr", "length", "payload")

    def __init__(self, lineno, time_s, freq_hz, cont, ctr, length, payload):
        self.lineno = lineno
        self.time_s = time_s
        self.freq_hz = freq_hz
        self.cont = cont
        self.ctr = ctr
        self.length = length
        self.payload = payload  # bytes


def parse_ida_lines(path: Path) -> list[Fragment]:
    out = []
    with path.open() as f:
        for lineno, line in enumerate(f, 1):
            if not line.startswith("IDA:"):
                continue
            m = IDA_LINE_RE.match(line)
            if not m:
                continue
            name, mstime_s, freq, _conf, _level, _symbols, _uldl, rest = m.groups()
            bm = IDA_BODY_RE.match(rest)
            if not bm:
                continue
            f1, _f2, ctr_bin, length, hexdata = bm.groups()
            cont = (f1 == '1')
            ctr = int(ctr_bin, 2)
            # name like p-1783307384-e000: ftype='p', starttime='1783307384'.
            parts = name.split('-')
            if len(parts) < 2:
                continue
            starttime = float(parts[1])
            abs_time = starttime + float(mstime_s) / 1000.0
            hexbytes = hexdata.replace('.', '').replace('!', '')
            try:
                payload = bytes.fromhex(hexbytes)
            except ValueError:
                continue
            out.append(Fragment(lineno, abs_time, int(freq), cont, ctr, int(length), payload))
    return out


class AssembledMessage:
    def __init__(self, fragments: list[Fragment]):
        self.fragments = fragments  # in over-the-air order

    @property
    def payload(self) -> bytes:
        return b''.join(fr.payload for fr in self.fragments)


def assemble_chains(fragments: list[Fragment]) -> list[AssembledMessage]:
    """Re-implements iridiumtk/reassembler/ida.py's
    ReassembleIDA.process() fragment-chaining, restricted to the
    subset relevant to producing complete multi-fragment byte streams
    (we don't need its de-dupe / broken-fragment stats for this)."""
    assembled = []
    # open chains: list of dicts {freq, times:[Fragment,...], ctr}
    open_chains: list[dict] = []
    for fr in fragments:
        matched = False
        for i, chain in enumerate(open_chains):
            last = chain["frags"][-1]
            if (abs(fr.freq_hz - chain["freq_hz"]) < FREQ_DEADBAND_HZ and
                    last.time_s <= fr.time_s <= last.time_s + FRAG_GAP_S and
                    (chain["ctr"] + 1) % 8 == fr.ctr):
                chain["frags"].append(fr)
                chain["ctr"] = fr.ctr
                if fr.cont:
                    pass  # still open
                else:
                    assembled.append(AssembledMessage(chain["frags"]))
                    open_chains.pop(i)
                matched = True
                break
        if matched:
            continue
        if fr.ctr == 0 and not fr.cont:
            # standalone single-fragment frame -- not interesting for
            # this fixture (no chaining to demonstrate), skip.
            continue
        elif fr.ctr == 0 and fr.cont:
            open_chains.append({"freq_hz": fr.freq_hz, "frags": [fr], "ctr": 0})
        # ctr>0 with no match: orphan, drop (matches upstream).
        # expire stale chains (>1s since their last fragment).
        open_chains = [c for c in open_chains if fr.time_s - c["frags"][-1].time_s < 1.0]
    return assembled


def decode_acars_fields(payload: bytes, time_s: float):
    """Decode SBD+ACARS fields using iridium-toolkit's own classes
    (not a reimplementation) so the resulting golden fields trace to
    the reference toolchain, per the no-circular-goldens rule."""
    sys.path.insert(0, str(IRIDIUM_TOOLKIT_DIR))
    import iridiumtk.config as cfgmod

    class Cfg:
        args = []
        station = None

    cfgmod.config = Cfg()
    from iridiumtk.reassembler.sbd import ReassembleIDASBDACARS

    import io
    r = ReassembleIDASBDACARS()
    r.outfile = io.StringIO()  # consume_l2 pretty-prints; we only want the fields
    q = r.process_l2((payload, time_s, False, 50.0, 1622000000))
    if q is None:
        return None
    r.consume_l2(q)  # populates q.mode/f_reg/ack/label/b_id/hdr/txt/errors
    # consume_l2 returns early (without setting q.errors) for non-ACARS
    # SBD content (q.data[0] != 1, or empty/too-short payloads) -- not
    # every assembled IDA packet in this capture is an ACARS message.
    if not hasattr(q, "errors") or q.errors:
        return None
    return {
        "mode": q.mode.decode('latin-1'),
        "reg": q.f_reg.decode('latin-1').lstrip('.'),
        "ack": q.ack.decode('latin-1'),
        "label": q.label,  # raw bytes, e.g. b'_\x7f'
        "block_id": q.b_id.decode('latin-1'),
        "hdr": q.hdr.hex(),
        "txt": q.txt.decode('latin-1'),
        "timestamp": q.timestamp,
    }


def parse_reference_txt(path: Path) -> list[dict]:
    out = []
    for line in path.read_text().splitlines():
        m = REF_LINE_RE.match(line)
        if not m:
            continue
        ts, hdr, direction, mode, reg, ack, label_desc, bid = m.groups()
        out.append({
            "timestamp_iso": ts,
            "hdr": hdr,
            "direction": direction,
            "mode": mode,
            "reg": reg,
            "ack": ack,
            "label_desc": label_desc,
            "block_id": bid,
        })
    return out


def emit_fixture(matches: list[dict], out_path: Path):
    lines = [
        "// Auto-generated by tests/scripts/build_acars_fixture.py — do not edit by hand.",
        "//",
        "// Ground truth: /home/bruce/iridium_bits/acars-milestone-20260706.{parsed,txt}",
        "// (2026-07-06 HydraSDR capture, REG A62001). The expected_* fields below",
        "// come from acars-milestone-20260706.txt, iridium-toolkit's",
        "// `reassembler.py -m acars` output over that capture — an independent",
        "// reference, not derived from this repo's own decode code.",
        "//",
        "// Each message's fragments[] are the REAL, verified-CRC-OK LW.DA burst",
        "// payloads (post-BCH bytes, exactly as iridium-toolkit's ida.py parsed",
        "// them from the capture) in over-the-air order. Every message in this",
        "// capture needs >1 fragment: a single LW.DA burst caps at 24 payload",
        "// bytes (da_len is a 5-bit field) but these ACARS/SBD envelopes run",
        "// 25-37 bytes, so the transmitter split them across consecutive LW.DA",
        "// time slots chained via da_cont/da_ctr — see",
        "// common/iridium_decoder/ida_reassembler.h.",
        "//",
        "// NOTE: this fixture is built at the post-ida_decode() level (raw",
        "// pre-BCH symbol bits for these specific over-the-air bursts are not",
        "// available -- the live capture that produced them has since rotated",
        "// past this time window; only the milestone's derived frame-level",
        "// trace survives). test_acars_tail_real.c therefore exercises",
        "// ida_reassembler_feed() -> sbd_reassembler_feed() -> libacars against",
        "// real captured content, while iridium_frame_classify()/ida_decode()'s",
        "// own BCH path is covered separately by the Albuquerque corpus tests",
        "// (test_iridium_frame_corpus, test_ida_decode_corpus).",
        "#pragma once",
        "#include <stdint.h>",
        "#include <stdbool.h>",
        "",
        "typedef struct {",
        "    uint8_t  da_cont;",
        "    uint8_t  da_ctr;",
        "    const uint8_t *payload;",
        "    uint8_t  payload_len;",
        "    uint32_t freq_hz;",
        "    uint64_t timestamp_us;",
        "} acars_fixture_fragment_t;",
        "",
        "typedef struct {",
        "    const char *description;",
        "    const acars_fixture_fragment_t *fragments;",
        "    int n_fragments;",
        "    bool uplink;",
        "    const char *expected_reg;",
        "    char expected_mode;",
        "    uint8_t expected_label[2];",
        "    char expected_block_id;",
        "    char expected_ack;",
        "    const char *expected_txt;",
        "    const char *expected_timestamp_iso;",
        "} acars_fixture_message_t;",
        "",
    ]
    for i, m in enumerate(matches):
        frags = m["fragments"]
        lines.append(f"static const uint8_t ACARS_FIX_{i}_F0_PAYLOAD[] = "
                     f"{{ {', '.join(f'0x{b:02x}' for b in frags[0].payload)} }};")
        for j in range(1, len(frags)):
            lines.append(f"static const uint8_t ACARS_FIX_{i}_F{j}_PAYLOAD[] = "
                         f"{{ {', '.join(f'0x{b:02x}' for b in frags[j].payload)} }};")
        lines.append(f"static const acars_fixture_fragment_t ACARS_FIX_{i}_FRAGMENTS[] = {{")
        for j, fr in enumerate(frags):
            t_us = int(round(fr.time_s * 1e6))
            lines.append(
                f"    {{ {1 if fr.cont else 0}, {fr.ctr}u, ACARS_FIX_{i}_F{j}_PAYLOAD, "
                f"{len(fr.payload)}u, {fr.freq_hz}u, {t_us}ULL }},"
            )
        lines.append("};")
        lines.append("")
    lines.append(f"#define ACARS_FIXTURE_NUM_MESSAGES {len(matches)}")
    lines.append("static const acars_fixture_message_t ACARS_FIXTURE_MESSAGES[ACARS_FIXTURE_NUM_MESSAGES] = {")
    for i, m in enumerate(matches):
        dec = m["decoded"]
        ref = m["reference"]
        label = dec["label"]
        lines.append(
            f'    {{ "{ref["timestamp_iso"]} REG:{ref["reg"]} ACK:{ref["ack"]} bID:{ref["block_id"]}", '
            f'ACARS_FIX_{i}_FRAGMENTS, {len(m["fragments"])}, false, '
            f'"{dec["reg"]}", \'{dec["mode"]}\', {{ 0x{label[0]:02x}u, 0x{label[1]:02x}u }}, '
            f"'{dec['block_id']}', '{dec['ack']}', "
            f'"{dec["txt"]}", "{ref["timestamp_iso"]}" }},'
        )
    lines.append("};")
    out_path.write_text("\n".join(lines) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-messages", type=int, default=2,
                    help="cap fixture size (all 7 reference messages need "
                         "the same 2-fragment reassembly; 2 is a "
                         "representative, fast-to-test sample)")
    args = ap.parse_args()

    if not PARSED_PATH.exists() or not REFERENCE_TXT.exists():
        print(f"ERROR: milestone ground truth not found at {MILESTONE_DIR} "
              "(read-only; see docs/superpowers/plans/2026-07-06-acars-smoke-test.md)",
              file=sys.stderr)
        return 1

    fragments = parse_ida_lines(PARSED_PATH)
    print(f"parsed {len(fragments)} CRC:OK IDA fragments", file=sys.stderr)

    assembled = assemble_chains(fragments)
    print(f"assembled {len(assembled)} multi-fragment IDA packets", file=sys.stderr)

    reference = parse_reference_txt(REFERENCE_TXT)
    print(f"parsed {len(reference)} reference ACARS lines from {REFERENCE_TXT.name}",
          file=sys.stderr)

    matches = []
    for am in assembled:
        dec = decode_acars_fields(am.payload, am.fragments[-1].time_s)
        if dec is None:
            continue
        # Match against the reference by (REG, ACK, block_id, hdr) --
        # unique per message in this capture.
        for ref in reference:
            if (ref["reg"] == dec["reg"] and ref["ack"] == dec["ack"] and
                    ref["block_id"] == dec["block_id"] and ref["hdr"] == dec["hdr"]):
                matches.append({"fragments": am.fragments, "decoded": dec, "reference": ref})
                break
        if len(matches) >= args.max_messages:
            break

    print(f"matched {len(matches)} assembled packets against the reference "
          f"(requested max {args.max_messages})", file=sys.stderr)
    if len(matches) < 2:
        print("ERROR: need at least 2 matched messages for the fixture", file=sys.stderr)
        return 1

    for m in matches:
        print(f"  fixture message: {m['reference']['timestamp_iso']} "
              f"REG={m['decoded']['reg']} ACK={m['decoded']['ack']} "
              f"bID={m['decoded']['block_id']} "
              f"fragments={len(m['fragments'])} "
              f"total_bytes={sum(len(fr.payload) for fr in m['fragments'])}",
              file=sys.stderr)

    FIXTURE_DIR.mkdir(parents=True, exist_ok=True)
    out_path = FIXTURE_DIR / "fixture_acars_frames.h"
    emit_fixture(matches, out_path)
    print(f"wrote {out_path}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
