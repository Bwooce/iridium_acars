# gr-iridium `-f cu8` spectral-inversion bug — upstream report draft

**Status:** draft, not yet filed. Target: `muccc/gr-iridium` (issue + small PR).
**Found:** 2026-07-21, during ESP32-P4 P4-vs-gr-iridium decode comparison.
**Verified:** compile-level mechanism check + byte-identical-sample A/B (below).
**Confidence:** high for the offline `-f cu8` failure; the "live rtl_sdr is safe"
claim is inferred — confirm before filing (see checklist).

---

## Summary

`iridium-extractor -f cu8 <file>` **spectrally inverts** the input on every
little-endian host (x86, ARM64, Raspberry Pi — i.e. essentially all machines).
Burst *detection* is unaffected (magnitude-based), so the extractor reports a
healthy burst count and high confidence %, but **every demodulated payload is
I/Q-swapped**, so downstream `iridium-parser` decodes **0 frames** — silently,
with no error or warning. The classic `rtl_sdr → .cu8 file → offline decode`
workflow is totally broken and looks like it's working.

## Root cause

`lib/iuchar_to_complex_impl.cc` (around lines 48-56) builds its uint8→`gr_complex`
lookup table under:

```cpp
#ifdef BOOST_LITTLE_ENDIAN
    ... // I -> real, Q -> imag  (correct on LE)
#else
    ... // I -> imag, Q -> real  (I/Q swapped)
#endif
```

…but the file includes **no boost endian header** (only `config.h` /
`io_signature.h`). The block was copied from gr-osmosdr, which *did* include the
header. So `BOOST_LITTLE_ENDIAN` is undefined, the `#else` (big-endian) branch
compiles on a little-endian host, the first byte (I) is written to `imag` and the
second (Q) to `real` → I/Q swap ≡ spectral inversion (mirror about DC/LO).

It's silent because (a) the fft burst tagger triggers on magnitude, invariant
under inversion, and (b) the 24-symbol Unique Word is composed of identical bit
pairs (`00`/`11`) and is itself inversion-invariant, so the UW still matches and
the extractor emits confident `RAW:` bursts — while the payload bit-pairs are all
swapped.

HEAD checked: `a09b254` (2025-03-30). Possibly the cause of open issue **#48**
("Weird Patterns in I/Q values") — check for overlap.

## Reproduction (A/B on byte-identical samples)

Take one sample buffer, extract it two ways:

```
# cf32 input (correct path):
iridium-extractor -f cf32_le -c <LO> -r 2500000 <file.cf32> | grep '^RAW:' > a.bits
iridium-parser.py -o line --uw-ec a.bits           # -> ~59 typed frames

# cu8 input (same underlying samples, converted 1:1):
iridium-extractor -f cu8 -c <LO> -r 2500000 <file.cu8> | grep '^RAW:' > b.bits
iridium-parser.py -o line --uw-ec b.bits           # -> 0 typed frames
sed 's/^RAW:/RWA:/' b.bits | iridium-parser.py -o line --uw-ec /dev/stdin  # -> ~59 typed
```

Observed (byte-identical ALBQ fixture, 66 bursts both ways):
- cf32 → **59** typed frames as `RAW:` (correct).
- cu8  → **0** as `RAW:`, **59** as `RWA:` (inverted; `RWA` = "no swap" happens to
  compensate the inversion).

So only the cu8 path inverts, and relabeling `RAW:`→`RWA:` is a *workaround* that
compensates (it does NOT diagnose the parser — the parser is correct). Other
corpora confirm the cf32 pairing is self-consistent: `derivations/iridium.bits`
(cf32) → 73 typed as `RAW:`, 1 as `RWA:`.

## Proposed fix (~5 lines)

Determine endianness correctly instead of relying on the (unincluded) boost macro:

- C++17/20: `if constexpr (std::endian::native == std::endian::little) { ... }`, or
- build the LUT explicitly per byte (no compile-time endian branch at all), or
- `#include <boost/predef/other/endian.h>` and use `BOOST_ENDIAN_LITTLE_BYTE`.

Include the A/B repro in the PR.

## Scope

- **Affected:** `iridium-extractor -f cu8 <file>` on any little-endian host — the
  mainstream offline rtl_sdr-file workflow.
- **NOT affected:** `-f cf32` input (upstream test fixtures use it → the test
  suite never caught this).
- **Probably NOT affected:** live `rtl_sdr` capture — the live path uses the
  gr-osmosdr source block, which does its own uint8→complex conversion *with* the
  endian header. **CONFIRM before filing** (verify the live flowgraph's input
  block, don't assert).

## Corollary — burst frequencies are mirrored

Even with the RWA decode workaround, cu8-extracted burst **frequencies are
mirrored about the LO** (inversion flips the sign of the frequency offset). Decode
*counts* are valid; any *per-frequency* analysis from cu8 output is wrong. (This
affected the sign of our 2026-07-19/20 P4 cu8 frequency observations; the
HydraSDR/cf32-based analyses are unaffected.)

## Optional companion: parser hardening (separate small suggestion)

`iridium-parser` silently commits to a symbol order (`bitsparser.py:72`,
`self.swapped = (m.group(1) != "RWA")`). Combined with this bug the user gets
plausible `RAW:` lines and zero decoded frames with no hint. Suggest: if >K
messages parse but ~0 upgrade to typed frames, emit a stderr warning ("0 frames
from N bursts — input may be spectrally inverted / symbol-swapped; try RWA"), or
auto-probe `symbol_reverse` on a prefix. Frame this inside the main bug report.

## Pre-filing checklist

- [ ] Confirm live `rtl_sdr` path is unaffected (inspect the live input block).
- [ ] Check issue #48 for overlap; link or fold in.
- [ ] Re-run the A/B on a clean upstream checkout (not our gri-tools build) to rule
      out a local build quirk.
- [ ] Decide: one PR (fix) + issue, or issue first.
