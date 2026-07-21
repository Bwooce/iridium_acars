# RFC: soft-decision (Chase-2) BCH decoding for gr-iridium / iridium-toolkit

**Status:** draft RFC, not yet filed. Target: discussion/issue on `muccc/gr-iridium`
+ `muccc/iridium-toolkit` (cross-repo — it's a design change, not a drop-in patch).
**Origin:** ESP32-P4 Iridium receiver; soft-BCH has been in production there since
2026-05-30 and does the bulk of decoding at marginal SNR.
**Confidence upstream adopts:** medium (needs a wire-format change; but the safety
data is strong and the air-truth headroom is real).

---

## Motivation

gr-iridium's QPSK demod emits **hard** symbol decisions plus a single aggregate
confidence % (`iridium_qpsk_demod_impl.cc:239`), and iridium-toolkit's BCH is
**hard** syndrome repair (t≤2) plus a hard-retry `--harder` mode. There is **no
soft-decision path anywhere** in either repo (verified by exhaustive grep,
2026-07-21).

Yet the residual decode failures sit exactly where soft decoding helps: our
demod-gap analysis (`docs/2026-07-17-demod-gap-phase0.md`) shows the frames that
fail hard-BCH cluster at **3–4 bit errors in the worst codeword** — just past the
t=2 hard-correction limit, i.e. classic Chase territory. And gr-iridium itself
decoded only ~**41%** of the air-truth burst set in that comparison, so upstream
has the same headroom to recover, not just us.

**Soft-decision Chase-2 BCH turns the demod's per-bit reliabilities into recovered
frames that hard decoding throws away.**

## What it is

Chase-2 (Chase, IEEE-TIT 1972): from the per-bit soft metrics, take the K
least-reliable bit positions, try all 2^K flip patterns, hard-decode each, and
pick the codeword with the smallest reliability-weighted (soft) distance to the
received word. K=3 → 8 trials → recovers up to 3 errors; K=4 → 16 → up to 4. Cheap
(a handful of hard syndrome-decodes per block).

For the IDA/SBD payload we layer **CRC-16 arbitration** on top: a Chase candidate
is accepted only if it also passes the frame's own CRC, which is what keeps the
false-accept rate near zero (below).

## Evidence from the P4 deployment

1. **Load-bearing at marginal SNR.** The worker's K=3 Chase on the BCH(31,21)
   frame blocks (`common/iridium_decoder/bch_decoder.c:146` `bch_decode_block_soft`,
   always-on) recovers ~**86%** of all decoded frames on live bench signal
   (measured 2026-07-21: `chase_recovered` 132 of 151 decodes). Turn it off and
   decode craters ~7×. This is real recovery, not a marginal tweak.

2. **Safe with CRC arbitration.** The IDA-layer Chase (`common/iridium_decoder/
   ida_chase.c`, reference prototype `tests/scripts/chase_crc_bch.py`) at
   **L=5 / CRC-check cap 256** recovered **+11 frames with 0 false accepts and 0
   ground-truth mismatches** over a corpus of **568 gold + 825 CRC-fail** frames.
   Key safety finding: **the CRC-check cap, not the Chase order L, is the safety
   knob** — L=6 (more trials) admitted one wrong-but-CRC-valid frame, so bound the
   number of CRC checks, not just L. This is the concrete parameterization to
   recommend upstream.

## The blocker (why it's an RFC, not a patch)

gr-iridium's RAW output carries **hard bits only** — there's no field for per-bit
reliabilities. Chase-2 needs those soft metrics. So this requires:

- **gr-iridium (extractor):** the QPSK demod must expose per-symbol/per-bit soft
  values (e.g. the pre-slicer soft outputs). Either extend the `RAW:` line with an
  optional soft-metric field, or add a new output mode/tag that carries them.
  Backward-compatible: default off; hard `RAW:` unchanged.
- **iridium-toolkit (parser):** add a Chase-2 stage in the BCH repair path
  (`bch.py` / `bitsparser.py`) that consumes the soft field when present, gated
  behind a flag (e.g. `--chase K` / `--chase-cap N`), CRC-arbitrated for payloads.

That's a coordinated two-repo change, hence: propose as an RFC/discussion with the
evidence, agree the wire format, then implement.

## Interface sketch (for discussion)

- RAW line today: `RAW: <name> <ts> <freq> N:<snr> I:<...> <conf>% <...> <len> <bits>`.
- Option A: append an optional trailing soft field, e.g. ` SOFT:<base64 int8 per
  bit>` — ignored by current parsers, consumed by a Chase-aware one.
- Option B: a distinct tag (`RWS:`?) for soft-bearing bursts.
- Reliability encoding: signed magnitude per bit (sign = hard decision, |value| =
  reliability), as we use internally (`bch_decode_block_soft` soft distance metric).

## Safety guidance to include

- Soft/multi-trial decode can "correct" noise into a valid-looking codeword — the
  danger scales with trials. **Require a downstream check before accepting**:
  CRC (payload) or classify-to-known-type (per-frame). Count Chase-recovered
  frames separately from hard decodes.
- Recommend **L=5, CRC-cap 256** as the validated safe point; expose both; warn
  that raising the cap (not L) is what raises false-accept risk.
- Our validation harness pattern (gold + CRC-fail corpus, count false accepts +
  truth mismatches) is worth replicating on their air-truth set before enabling by
  default.

## Caveats (be honest in the filing)

- Our 86% figure is **bench-marginal-SNR-specific**; gri users at higher SNR will
  see a smaller (but nonzero) gain — the air-truth ceiling is the fair estimate of
  upstream headroom, not our bench number.
- Cross-repo format change = real coordination cost; may land as "interesting,
  later" rather than merged quickly.
- Depends on nothing P4-specific — it's pure algorithm + a wire-format field.

## References

- P4 impl: `common/iridium_decoder/bch_decoder.c` (`bch_decode_block_soft`),
  `common/iridium_decoder/ida_chase.c`, `tests/scripts/chase_crc_bch.py`.
- Analysis: `docs/2026-07-17-demod-gap-phase0.md` (error-count distribution,
  ~41% air-truth).
- Chase, D., "A class of algorithms for decoding block codes with channel
  measurement information," IEEE Trans. Inf. Theory, 1972.
- Companion report: `docs/2026-07-21-gr-iridium-cu8-spectral-inversion-bug.md`.

## Pre-filing checklist

- [ ] Re-confirm gri/toolkit still have no soft path at the version you file against.
- [ ] Reproduce the +11/0-false-accept result on a clean upstream checkout's corpus
      (or at least describe our corpus precisely).
- [ ] Draft the wire-format proposal concretely (Option A vs B) before opening.
- [ ] File the cu8-inversion bug first (it's a prerequisite for anyone reproducing
      soft-decode gains from cu8 captures).
