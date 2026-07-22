# VDL Mode 2 implementation plan (136.975 MHz D8PSK → libacars)

> Status: plan + foundation IMPLEMENTED (Phase V0, this branch). 2026-07-22.
> Decision basis: `2026-07-22-vhf-acars-feasibility.md` (VDL2-first for Sydney,
> skip POA). This doc is the build contract for the component agents.

## TL;DR

One firmware, an NVS band soft-switch (`band=iridium|vdl2`, default iridium,
reboot to apply). The shared front end — USB ingest → `signal_buffer` →
`fft_burst_tagger` → worker extract/rotate/decimate — is reused unchanged;
only its *constants* are now per-band (`band_profile`). The worker dispatches
each decimated burst through a new `band_pipeline_t` interface; Iridium's
existing `burst_pipeline` is the first implementer (bit-identity proven by
host test), and VDL2 plugs in as `vdl2_pipeline` = **D8PSK demod** →
**RS(255,249)** → **AVLC deframe** → **libacars** — the same
`la_acars_parse_and_reassemble()` call the Iridium SBD path already makes in
`p4-usb-host/main/frame_decoder.c` (`try_acars()`). Every DSP/decode stage is
host-built and cross-validated against **dumpvdl2** on shared IQ captures,
the same discipline this repo used against gr-iridium.

## Phase V0 — foundation scaffolding (DONE on this branch)

What exists now (all built + host-tested; Iridium behaviour bit-identical):

- **Band soft-switch**: `app_config` field `band` (u8 = `band_id_t`, NVS key
  `"band"`, default 0 = iridium; out-of-range clamps to iridium). Set over
  serial: `set band vdl2` / `get band` (`serial_cmd.c`). When NVS `lo_hz` was
  never set, the LO default now comes from the band profile (Iridium's
  profile LO == `IRIDIUM_CENTER_FREQ_HZ`, `_Static_assert`ed in
  `app_config.c`); an explicitly-set `lo_hz` always wins.
- **`common/band_pipeline/band_profile.{h,c}`** — per-band front-end
  parameters: default LO, detect-path fs, tagger pre/post pads, burst width,
  default threshold. `dsp_processor_create()` (`p4-usb-host/main/
  dsp_processor.c`) now configures `fft_burst_tagger_init()` from the
  profile; the Iridium values are pinned to the legacy constants by
  `_Static_assert`s there and by `tests/host/test_band_profile.c` (which also
  pins the float threshold). The VDL2 profile parks the LO at
  **136.8125 MHz** — half a channel off the 25 kHz grid so no channel sits at
  DC (memory: never centre the downconvert on a signal) — putting all of
  136.650–136.975 within ±162.5 kHz; fs stays 2.5 MSPS so ingest is
  untouched. VDL2 width = 16 bins ≈ 19.5 kHz (D8PSK 10.5 kBd, RRC α≈0.6
  occupies ≈16.8 kHz of a 25 kHz channel). VDL2 numbers are marked
  PROVISIONAL pending V1 calibration.
- **`common/band_pipeline/band_pipeline.h`** — the `band_pipeline_t`
  interface (see §Interface below), designed from the worker's real call
  site.
- **`common/iridium_decoder/iridium_band_pipeline.{h,c}`** — Iridium's
  implementation: a zero-processing type-mapping shim over the exact
  `burst_prefilter()` + `burst_pipeline_process_burst()` calls
  `worker_core1.c` used to make directly. Bit-identity is proven by
  `tests/host/test_band_pipeline_iridium.c`: the ALBQ fixture end-to-end,
  every tagged burst run through both paths on identical buffer copies,
  asserting identical prefilter verdicts (including per-gate booleans) and
  byte-identical bits/soft_bits/direction per frame (67 bursts, 61 decoded
  frames).
- **`common/vdl2_decoder/vdl2_pipeline.{h,c}`** — the VDL2 vtable stub:
  accept-all prefilter, `process_burst` counts bursts
  (`vdl2_pipeline_bursts_seen()`) and returns 0 frames. Conformance test:
  `tests/host/test_vdl2_pipeline_stub.c`. A `band=vdl2` device boots, tags
  VHF bursts, and counts them — plumbing provable before any demod exists.
- **`p4-usb-host/main/band_select.{h,c}`** — NVS band id → concrete
  pipeline (the only file that references every band implementation, keeping
  `common/band_pipeline` dependency-free). `worker_core1_init()` resolves
  `s_band_pipeline` once at boot.

Deliberately **not** done in V0: no HTTP endpoint for band (adding a route
requires the `max_uri_handlers` bump — see memory note; serial + NVS is
enough until the demod exists), no runtime fs changes (both bands are
2.5 MSPS; `worker_core1.c`'s compile-time `FS_DETECT_HZ` buffer sizing is
unchanged and would need revisiting only for a future band at a different
rate).

## The `band_pipeline_t` interface (frozen for the component agents)

Derived from the worker's actual per-burst sequence in
`p4-usb-host/main/worker_core1.c` `worker_task()`: pop burst → stale guards →
`wb_extract_decim()` (ring extract → `rotate_to_dc` at the tagger's
`rel_freq_hz` → `direct_if_decim` 10× to 250 ksps) → **[interface]**
prefilter → process_burst → per-frame emit callback.

```c
// common/band_pipeline/band_pipeline.h  (authoritative copy in the header)

typedef struct {
    uint8_t *bits;      // demodulated hard bits (1/byte); CALLBACK OWNS (free())
    int      n_bits;
    int16_t *soft_bits; // per-bit soft metric, sign=hard decision; may be NULL; callback owns
    int      direction; // band-defined enum (Iridium: ir_direction_t)
    bool     demod_ok;  // false = diagnostic callback (bits still owned by callback)
    float    snr_db;
    const void *band_detail; // band-specific result, valid during the callback
} band_frame_t;

typedef void (*band_frame_cb_t)(band_frame_t *f, void *ctx);

typedef struct {
    bool accept, width_ok, dur_ok, snr_ok;   // per-gate verdicts (worker's
} band_prefilter_verdict_t;                  // continuation rescue reads them)

typedef struct {
    const char *name;
    bool (*prefilter)(const int16_t *iq250, int n_complex, int width_bins,
                      band_prefilter_verdict_t *v);   // NULL = accept all
    int  (*process_burst)(int16_t *iq250, int n_complex,
                          band_frame_cb_t cb, void *ctx); // returns #demod-ok frames
} band_pipeline_t;
```

Input contract: interleaved int16 IQ at **250 ksps** (= FS_DETECT/10), burst
already rotated to DC (channel centre). `process_burst` may mutate the buffer
in place (Iridium's does). At 10.5 kBd this is 23.81 samples/symbol — the
VDL2 demod decimates further internally (see V2).

Split of responsibilities (mirrors Iridium exactly): the *pipeline* is
DSP → bit vector; FEC + framing + application dispatch live in the *emit
sink* downstream. Iridium's sink is `worker_emit_frame()` in
`worker_core1.c` (BCH(31,21) + `iridium_frame_classify` +
`frame_decoder_push`). VDL2's sink (V3) runs RS + AVLC + ACARS extraction —
cheap byte work, planned for the frame_decoder task (Core 0) so the worker
stays DSP-only.

## Component breakdown (build-in-isolation contracts)

All new modules go in `common/vdl2_decoder/`, pure C, no ESP-IDF runtime
deps, host-tested first (the `iridium_decoder` pattern: same sources compile
on host and target). **No PIE asm anywhere in VDL2** — at 10.5 kBd the whole
chain is ≪1% of a core, so we opt out of the two standing PIE hazards
(heap-position early-alloc, one-PIE-owner-per-core — AGENTS.md) entirely.
Scalar C only; internal buffers are ordinary heap/static.

### C1 — D8PSK demod (`vdl2_demod.{h,c}`) — the hard part

```c
typedef struct {
    uint8_t *bits;       // descrambled bits, Gray-decoded, MSB-first per symbol; malloc'd
    int16_t *soft_bits;  // per-bit reliability, same convention as qpsk_demod.h; malloc'd
    int      n_bits;
    float    cfo_hz;     // residual carrier offset (diagnostics/cross-val)
    int      sync_offset;// sample index (at demod rate) of training-sequence start
    float    evm_rms;    // demod quality (cross-val gate)
} vdl2_demod_result_t;

// iq250: the band_pipeline input (250 ksps, rotated to channel centre).
// Returns false if no training-sequence lock (caller drops the burst).
bool vdl2_demod_burst(const int16_t *iq250, int n_complex,
                      vdl2_demod_result_t *out);
```

Internal stages (each dumpable for stage-wise cross-validation, the
`burst_pipeline_set_dump_once()` idiom):

1. **Resample 250 k → 42 k (4 sps)**: polyphase multi-rate FIR — reuse
   `common/iridium_decoder/firmr_s16.{c,h}` (the host-proven
   `dsps_firmr_s16`-equivalent already compiled for target; see
   `resample_256_to_250.c` for the established usage pattern). 250/42 =
   interp 21 / decim 125.
2. **Coarse CFO**: the squared-FFT trick generalises to 8-PSK as the
   **8th-power** spectrum (signal^8 collapses modulation); the Q15 FFT is
   already there (`fft_sc16_2048.c`). NOTE: ^8 on Q15 needs staged
   renormalisation — model on `uw_correlator_estimate_cfo()`'s ^2
   implementation in `uw_correlator.c`, don't copy it blindly.
3. **RRC matched filter** α≈0.6 (ICAO Annex 10 Vol III / ARINC 631 value —
   confirm against dumpvdl2's filter during V1): new tap set, applied via
   the same FIR helpers as (1). Iridium's RRC application site in
   `burst_pipeline.c` is the pattern.
4. **Training-sequence sync**: correlate the known 16-symbol VDL2
   synchronisation sequence (extract the exact symbol values from dumpvdl2 —
   see §Reference — during V1, embed as a table like
   `uw_correlator_tables.c`). Replaces Iridium's UW correlator; same
   matched-filter + parabolic-peak-interp structure (`uw_correlator.c` is
   the reference implementation to mirror, not to call).
5. **Timing recovery**: Gardner is modulation-independent — reuse
   `common/iridium_decoder/sym_timing.{c,h}` as-is at 2 sps (it already
   serves `qpsk_demod.c`).
6. **Carrier tracking + slicing**: generalise `qpsk_demod.c`'s 2nd-order
   PLL from 4-ary to 8-ary decision-directed (new code in vdl2_demod.c;
   qpsk_demod.c stays untouched). D8PSK is *differential* — decode the
   phase delta between successive symbols, Gray-map to 3 bits.
7. **Descrambler**: the VDL2 self-synchronising scrambler (15-stage LFSR;
   take polynomial + preload from dumpvdl2 in V1). Trivial, but bit-exact
   fixtures mandatory — an off-by-one here looks like RF noise downstream.

Host gates (V2 exit criteria): on the V1 capture corpus, ≥90% of the bursts
dumpvdl2 decodes must sync-lock, and the descrambled bitstream must be
bit-identical to dumpvdl2's pre-RS bitstream on strong bursts (dumpvdl2's
debug output / a patched dump hook provides the reference — same approach as
gr-iridium's `--dump` files, `tests/scripts/stagewise_compare.py` precedent;
heed the alignment pitfalls memory note).

### C2 — RS(255,249) decoder (`rs_255_249.{h,c}`) — being built separately

Frozen plug-in contract (the RS agent builds against this; vdl2_pipeline
consumes it):

```c
// Shortened RS(255,249) over GF(2^8) (field/generator per ARINC 631 —
// extract poly constants from dumpvdl2's rs implementation and cite them
// in the header). Corrects <= 3 symbol errors per block.
//
// block: n_data message bytes followed immediately by 6 parity bytes
// (n_data <= 249). Corrects IN PLACE. Returns the number of corrected
// symbols (0..3) or -1 if uncorrectable (block left unmodified — the
// no-modify-on-fail rule ida_chase.c established).
int  rs_255_249_decode(uint8_t *block, int n_data);

// Encoder — for synthetic test fixtures only (ida_encode.c precedent:
// lets host tests drive real payloads through the production decode path).
void rs_255_249_encode(const uint8_t *data, int n_data, uint8_t parity_out[6]);
```

Test bar (mirrors `tests/host/test_bch_syndrome.c` / `test_ida_chase.c`):
exhaustive 0/1/2/3-symbol-error recovery on encoded blocks, >3-error
must-fail (or misdecode counted honestly), no-modify-on-fail, plus parity
against dumpvdl2 on captured blocks. Register in `tests/host/CMakeLists.txt`
with `-Werror` (new-code policy from V0).

Where it plugs in: `vdl2_pipeline.c`'s L2 feed (V3) — after the demod
bitstream is packed to bytes and de-interleaved (VDL2 interleaves bytes
across RS blocks; interleaver geometry from the burst-header length field —
extracted in V1), each block goes through `rs_255_249_decode` before AVLC.

### C3 — AVLC/HDLC deframe (`avlc.{h,c}`)

```c
typedef struct {
    uint32_t dst_addr, src_addr; // 27-bit AVLC addresses + type bits, packed
    bool     cr;                 // command/response
    uint8_t  control;            // HDLC control byte (I/S/U)
    const uint8_t *info;         // borrowed pointer into the deframe buffer
    int      info_len;
    bool     fcs_ok;             // CRC-16 FCS verdict
} avlc_frame_t;

typedef void (*avlc_frame_cb_t)(const avlc_frame_t *f, void *ctx);

// bits: RS-corrected bit vector (1/byte, demod order). Walks 0x7E flag
// boundaries, removes bit-stuffing (drop 0 after five 1s), packs bytes
// LSB-first, verifies the frame FCS, fires cb per frame (fcs_ok=false
// frames fire too — counted, not parsed). Returns frames emitted.
int avlc_deframe(const uint8_t *bits, int n_bits, avlc_frame_cb_t cb, void *ctx);
```

FCS: AVLC uses the ISO/HDLC CRC-16 (reflected, init 0xFFFF, final XOR —
X.25 FCS), which is NOT the CRC-16/CCITT-FALSE already in
`common/iridium_decoder/crc16.{c,h}` — extend that module with the
reflected variant (one new table/function beside the existing one; both
host-tested in `test_crc16`-style vectors) rather than adding a new module.

Payload dispatch (V3, in the emit sink not in avlc.c): information fields
are either ACARS-over-AVLC (feeds libacars) or ATN/X.25 (count + discard
for now — same "count what you don't decode" convention as
`frame_decoder.c`'s IMS/paging handling). The discriminator byte comes from
dumpvdl2's avlc dispatch logic (V1 extraction task).

### C4 — integration & emit path (V3, firmware side)

- `vdl2_pipeline.c` `process_burst` = `vdl2_demod_burst()` → fire ONE
  `band_frame_t` per locked burst with the descrambled bit vector
  (`direction=0`, `band_detail`=`vdl2_demod_result_t*`). This mirrors
  Iridium exactly (pipeline emits PHY bits; FEC downstream).
- Worker side: `worker_emit_frame()` in `worker_core1.c` is Iridium's sink.
  V3 adds the band branch at ONE site — `worker_core1_init` picks the emit
  callback alongside `s_band_pipeline` (iridium → existing
  `worker_emit_frame`; vdl2 → a thin forwarder that pushes the bit vector
  to the frame_decoder task via the existing `frame_decoder_push()` /
  `frame_queue.c` infrastructure, which is already band-agnostic bits +
  metadata).
- frame_decoder task (Core 0, `frame_decoder.c`): band=vdl2 routes popped
  bit vectors to `vdl2_l2_feed()` (bytes → de-interleave → C2 RS → C3 AVLC
  → ACARS payloads) instead of `iridium_frame_classify`. The ACARS payload
  then enters **the same call the SBD path uses today**:
  `la_acars_parse_and_reassemble(acars_buf, acars_len, dir, s_reasm_ctx,
  rx_time)` — see `try_acars()` in `frame_decoder.c` (~line 401). Factor
  the libacars invocation + `acars_msg_t` emission out of `try_acars` into
  a payload-level helper both paths share, so `msg_ring`/`acars_push`/
  `http_server`/`sd_log`/airframes feed carry over with zero changes.
- Status/diag: extend `/status` with `vdl2.bursts_seen / synced / rs_ok /
  avlc_ok / acars` counters (the `worker_hot_stats_t` getter pattern). If a
  new HTTP route is ever added instead, bump `max_uri_handlers` (memory
  note) — prefer extending `/status` JSON.

## Cross-validation plan (dumpvdl2 = the gr-iridium of this band)

dumpvdl2 (GPL-3, same author as the vendored libacars) is the reference.
Discipline identical to the gri playbook (`AGENTS.md`, memory:
cross-validate-against-gr-iridium, don't-fit-tests-to-data):

1. **V1 step 0**: clone dumpvdl2 read-only at repo root beside `gr-iridium/`
   (it is NOT currently in the tree). Build on the Mac. Extract-and-cite
   constants (sync symbols, scrambler poly/preload, RRC α, RS field/
   generator polys, interleaver, FCS params) into the module headers with
   file references — every numeric traces to the reference or the spec
   (ICAO Annex 10 Vol III / ARINC 631), never to a fixture sweep.
2. **Shared IQ captures**: VHF antenna on the P4's RTL-SDR, LO 136.8125 MHz,
   2.5 MSPS, SD dark-slice burst capture (`sd_capture.c`; the daytime-capture
   procedure memory applies — mount SD first, card-pull retrieval). Feed the
   SAME file to (a) dumpvdl2 on the Mac (raw-file input mode) → ground-truth
   frame inventory + intermediate dumps, and (b) the host pipeline harness.
   Sydney note: 136.975 CSC is busy hub traffic — a 10-minute daytime
   capture should carry dozens–hundreds of frames.
3. **Per-stage gates** (host ctest, fixture-embedded like
   `fixture_albq_raw.h` via `tests/scripts/build_fixtures.py`):
   - tagger: burst inventory recall vs dumpvdl2's decoded-frame times
     (`test_tagger_vs_manifest.c` precedent) — validates the PROVISIONAL
     profile numbers (width/pads/threshold), which get corrected from
     evidence, not tuned to pass;
   - demod: descrambled-bit identity on strong bursts; sync rate ≥90% of
     reference on the corpus;
   - RS: block-level parity on captured blocks + exhaustive synthetic;
   - AVLC: frame-level identity (addresses, control, FCS verdicts, info
     bytes) vs dumpvdl2's frame log;
   - end-to-end: ACARS-message identity (reg/label/text) vs dumpvdl2's
     ACARS output, with a decode-count floor registered in ctest
     (`pipeline_wideband_resampled` floor precedent).
4. **On-device**: live A/B — P4 (band=vdl2) vs dumpvdl2+RTL on the Mac from
   the same antenna/splitter, frames/hour by aircraft reg (the P4-vs-gri
   live methodology, memory: demod-at-reference-ceiling). Device smoke
   remains MANDATORY after any DSP-path commit (memory note) — the Iridium
   RAW_IRIDIUM smoke must still pass on every VDL2-adjacent change since
   the worker/tagger code is shared.

## Phases & sequencing (independent agents)

| Phase | What | Depends on | Agent-parallel? |
|---|---|---|---|
| V0 | Scaffolding (band switch, profiles, band_pipeline, iridium adapter, vdl2 stub) | — | DONE (this branch) |
| V1 | dumpvdl2 checkout, constant extraction, VHF antenna, IQ capture corpus + fixtures, tagger-profile calibration | antenna on hand | one agent + bench time |
| V2 | C1 D8PSK demod, host-first, stagewise vs dumpvdl2 | V1 fixtures | yes — the demod agent |
| V2b | C2 RS(255,249) | interface frozen HERE (no V1 dep for synthetic tests) | yes — already underway separately |
| V2c | C3 AVLC + crc16 extension | interface frozen HERE; V1 fixtures for parity | yes |
| V3 | Wire C1–C3 into vdl2_pipeline + frame_decoder vdl2 branch + shared libacars helper + /status counters; end-to-end host floor test | V2, V2b, V2c | integration agent |
| V4 | Device bring-up: band=vdl2 soak, live A/B vs dumpvdl2, threshold/width finalisation, airframes.io VDL2 feed | V3 | bench |

Estimated effort unchanged from the feasibility doc: ~4–8 weeks, dominated by
V2.

## Risks / open questions

- **Non-integer sps at 250 k** (23.81): resolved by the internal 21/125
  polyphase to 4 sps; alternative (change the worker's shared decim per
  band) rejected — it would fork the band-agnostic front half.
- **Tagger on CSMA bursts**: VDL2 bursts arrive back-to-back on ONE channel
  (unlike Iridium's frequency spread); the gone-event post-pad may merge
  consecutive transmissions into one window. The multi-frame iteration in
  `process_burst` (Iridium already handles multi-frame bursts —
  `burst_pipeline_process_burst`'s loop) is the model; V1 capture data
  decides whether the profile post-pad needs shrinking.
- **A6 hot-bin / prefilter couplings**: worker continuation-boost logic
  (`hot_bin_table`) is Iridium-reassembly-specific but harmless under vdl2
  (no publisher → never hot). Leave as-is; do not gate it per band.
- **Iridium regression** remains the #1 risk on every shared-file touch:
  `test_band_pipeline_iridium` (host) + the device RAW_IRIDIUM smoke are the
  standing gates.
- **ATN/X.25 traffic**: a large share of AU VDL2 (CPDLC) is ATN, not ACARS.
  V3 counts it; decoding ATN (CLNP/COTP + CPDLC via libacars' ASN.1 path,
  already host-buildable — `test_libacars_best_effort`) is a natural V5 but
  OUT of this plan's scope.
- **GPL-3 hygiene**: dumpvdl2 is reference + ground-truth tooling. Constants
  and protocol facts are extracted with citations; code is re-implemented in
  this repo's style (the gr-iridium precedent).
