# Plan — ACARS smoke test from HydraSDR ground truth (2026-07-06)

Goal: a regression gate that proves the device (and host) pipeline decodes
**real ACARS** — using the ground truth we now possess (`~/iridium_bits/`,
first 7 ACARS + 28 SBD messages captured 2026-07-06; see memory
`project_first_acars_ground_truth`). Two phases: frame-level now, IQ-level
once a raw capture exists.

## Phase A — FRAME_DECODER smoke with real ACARS frames (unblocked today)

The FRAME_DECODER smoke variant pushes frame bits through
`frame_decoder_push` (`smoke_test.c:270` corpus mechanism) — no worker DSP,
so it does NOT trip the PIE-save deadlock that hangs GOLDEN. That makes this
the one smoke variant we can extend honestly before P2 lands.

1. **Extract the ACARS-bearing frames.** From `hydrasdr-*.bits` /
   `snap.parsed`, identify the IDA frames composing at least 2 assembled
   ACARS messages (filter parsed IDA lines by the reassembler's kept set —
   REG A62001 time windows; take the FULL SBD fragment sequence per message,
   in order). Convert to our corpus bit format the same way
   `tests/scripts/build_albq_fixture.py` built `fixture_albq_frames_corpus.h`
   from gr-iridium parser output (same parser, same field layout — extend
   that script or add `build_acars_fixture.py` alongside it).
2. **Host gate first**: new ctest `test_acars_tail_real` — feed the real
   frames through `iridium_frame_classify → ida_decode → acars_tail_feed`
   (the glue merged at 61d260b) and assert the EXACT ACARS fields/text that
   the reference toolchain produced (independent golden — satisfies the
   no-circular-goldens rule: expected values come from iridium-toolkit +
   libacars-reference, not our code).
3. **Device smoke**: add the same frames as smoke-corpus entries tagged
   ACARS; extend the FRAME_DECODER variant verdict to require ≥1
   `FRMDEC: ACARS:` line with `crc=OK` and matching REG/label, and the
   `/messages` + UDP push counters to tick. Update `scripts/smoke_run.sh`
   verdict parsing and the `Smoke-verified:` trailer format
   (`FRAME=PASS+ACARS`).
4. **Gotchas** (from project memory): smoke↔normal firmware rebuild
   (`CONFIG_SMOKE_TEST_*` taint in sdkconfig); `smoke_test.c` IS on the
   pre-push DSP list → the commit needs a smoke trailer (honest: run the
   FRAME variant itself); direction flag (these are DL frames); SBD
   session-timeout semantics are fine at smoke push rate; libacars digit
   `msg_num_seq` quirk (documented in `test_acars_tail_synthetic.c`).

Effort: Sonnet-tier (extraction script + host test are gated by
exact-golden assertions; smoke_test.c edit is small). Device verification
by main session. ~half a day.

## Phase B — IQ-level ACARS smoke (true end-to-end; needs raw IQ)

1. **Bounded raw capture**: rerun `iridium-extractor` with
   `--raw-capture` for a 20–30 min window (disk is fine: 686 GB free;
   ~40 MB/s → ≤72 GB, delete after cutting). At the observed ~8 ACARS/h
   expect 2–4 messages; the live decode stream timestamps them.
2. **Cut + retune the burst**: locate each ACARS burst (timestamp + freq
   from the parsed output), cut a ±0.5 s window from the raw file, mix to
   center the burst and resample 10 MS/s → 2.5 MS/s (scipy; same approach
   as `direct_if_dump.py` / `build_albq_raw_2667.py`). Store as a sigmf
   fixture in `test_data/` (small — seconds of IQ).
3. **Host gate**: extend the pipeline tests (or `decode_burst_capture` with
   a cf32 input path) to run tagger→worker→classify→tail on that fixture
   and assert the reference ACARS text — the first full IQ→ACARS proof of
   OUR DSP chain.
4. **Device smoke**: add as a REAL_IRIDIUM-style stripe asserting the same
   on-device. NOTE: dense IQ replay is the PIE-deadlock trigger — this
   variant lands only after P2. Sequencing: Phase B host gate can land
   before P2; the device variant is gated on it.
5. Also valuable reversed: once P1 lands and the P4 parks at ~1621 MHz,
   `sd_capture` continuous mode during an ACARS interval gives device-RF
   raw IQ for the same fixtures (2.5 MS/s native, no resample needed) —
   cross-validating the P4's own front end, which the HydraSDR capture
   cannot.

## Ordering vs current work

Phase A can start now (independent of P1/P2/P3 — touches tests/scripts,
tests/host, smoke_test.c only). Phase B raw capture can run tonight
unattended; the cut/fixture tooling is Sonnet-tier; the device IQ variant
waits for P2. Both phases' host gates slot into the existing 37-test ctest
suite.
