// band_profile — per-band RF/detection parameter profiles (VHF/VDL2
// foundation, docs/2026-07-22-vhf-acars-feasibility.md §Architecture).
//
// One firmware, an NVS band soft-switch (app_config `band`, u8, default
// iridium). Everything the front end needs that used to be an
// Iridium-baked compile-time constant (LO default, detect-path sample
// rate, fft_burst_tagger window pads / burst width / threshold) lives
// here as data, selected once at boot. The IRIDIUM profile is REQUIRED
// to be bit-identical to the historical constants — dsp_processor.c
// carries _Static_asserts pinning the integer fields against the legacy
// #defines, and tests/host/test_band_profile.c pins all of them
// (including the float threshold) on the host.
//
// Pure C, no ESP-IDF/FreeRTOS deps: compiles on host for unit tests
// (same discipline as band_health_core.h / decode_survey_core.h).

#pragma once

#include <stdint.h>

typedef enum {
    BAND_IRIDIUM = 0, // 1616–1626.5 MHz, DQPSK 25 ksym/s (current default)
    BAND_VDL2    = 1, // 136.650–136.975 MHz, D8PSK 10.5 kBd (VHF Data Link Mode 2)
    BAND_COUNT
} band_id_t;

// --- Iridium profile constants -------------------------------------------
// Single source of truth for the values that used to live as #defines in
// p4-usb-host/main/dsp_processor.{c,h}. Kept as macros (not just struct
// initialisers) so dsp_processor.c can _Static_assert them against the
// legacy defines — a compile-time proof that band=iridium is bit-identical.
#define BAND_IRIDIUM_LO_HZ 1626000000u         // IRIDIUM_CENTER_FREQ_HZ (dsp_processor.h)
#define BAND_IRIDIUM_FS_HZ 2500000u            // FS_DETECT_HZ (dsp_processor.h)
#define BAND_IRIDIUM_FBT_PRE_LEN 4096          // FBT_BURST_PRE_LEN = 2 × FBT_FFT_SIZE
#define BAND_IRIDIUM_FBT_POST_LEN 40000        // FBT_BURST_POST_LEN = fs × 16 ms (gri default)
#define BAND_IRIDIUM_FBT_WIDTH_BINS 32         // FBT_BURST_WIDTH ≈ half a 41.667 kHz channel
#define BAND_IRIDIUM_TAG_THR_DB 14.0f          // FBT_THRESHOLD_DB (gri-18dB parity on our scale)

// --- VDL2 profile constants (PRINCIPLED, UNCALIBRATED) ---------------------
// Derived from the VDL2 physical layer (ICAO Annex 10 Vol III / DO-224:
// 25 kHz channels, D8PSK 10.5 kBd, raised-cosine α=0.6) and dumpvdl2's
// demodulator behaviour — NOT from measurement. No wideband VHF capture
// exists yet: the golden fixture is a single 25 kHz baseband channel,
// not a 2.5 MSPS wideband stream, so the tagger has never actually seen
// a VDL2 burst. Every tagger value below is a derivation awaiting live
// calibration at V4 bring-up (docs/2026-07-22-vdl2-device-bringup.md
// §tagger-calibration). If band=vdl2 shows 0 bursts on-device, suspect
// THESE numbers before the demod.
//
// LO: all VDL2 channels sit on the 25 kHz grid in 136.650–136.975 MHz
// (≈350 kHz span, fits ONE 2.5 MHz window). Park half-a-channel off the
// grid (…+12.5 kHz) so no channel lands at DC — the DC-removal /
// near-DC path would eat a channel centred on the LO (memory:
// feedback_dont_center_downconvert_on_burst). 136.8125 MHz puts the
// worldwide Common Signalling Channel 136.975 at +162.5 kHz and
// 136.650 at −162.5 kHz, both comfortably inside ±1.25 MHz.
#define BAND_VDL2_LO_HZ 136812500u
// Front-end rate UNCHANGED from Iridium: the ingest → signal_buffer →
// tagger chain keeps running at 2.5 MSPS (Path A native rate), the
// vdl2_pipeline decimates further per burst. This is what makes the
// band switch software-only.
#define BAND_VDL2_FS_HZ 2500000u
// Pre pad (input samples before the detecting FFT step, i.e. how far
// the recorded burst start reaches back): 3 FFT steps = 6144 samples =
// 2.46 ms, one full VDL2 TX preamble — ramp-up (DO-224 caps transmitter
// power stabilisation at ~1 ms) + the 16-symbol sync sequence
// (16 / 10500 Bd = 1.52 ms). Iridium uses 2 steps (gri default) on the
// assumption that the hard QPSK burst edge crosses threshold within
// ≤2 FFT steps; VDL2's ramp is slow, so on a weak burst the threshold
// crossing can land a step later — the extra step guarantees the whole
// preamble (which vdl2_demod's sync search needs) is still inside the
// window. Cost is 2048 extra samples per extracted burst: negligible.
// UNCALIBRATED — verify at V4 that live bursts sync near the window
// start, not at its edge.
#define BAND_VDL2_FBT_PRE_LEN 6144
// Post pad (hangover: a burst closes when no bin exceeds threshold for
// post_len samples; the gone window ends at last_active + post_len):
// 10000 samples = 4.0 ms ≈ 42 symbols ≈ 4.9 FFT steps. A VDL2
// transmission is continuous D8PSK — unlike Iridium there is no
// intra-burst frame gap to bridge, so the post pad only needs to (a)
// ride through per-FFT-step threshold flicker on marginal bursts and
// (b) cover the last-active quantisation at the frame tail. Iridium's
// 16 ms (gri default) exists to hold multi-frame Iridium sequences in
// one window; for VDL2 that would instead merge back-to-back CSMA
// transmissions from different stations into one window and spend the
// tagger's 90 ms force-close budget (FBT_MAX_BURST_LEN, compile-time)
// on dead air. 4 ms closes each transmission promptly while tolerating
// ~5 consecutive below-threshold FFT steps mid-burst. UNCALIBRATED —
// if live bursts split mid-frame (sync count >> phy_ok with truncated
// frames), raise this first.
#define BAND_VDL2_FBT_POST_LEN 10000
// Burst width: gri semantics = the occupied bandwidth of one channel,
// in FFT bins — the span integrated by the detection statistic
// (Iridium: 40 kHz / 1220.7 Hz = 32 bins). VDL2 D8PSK occupies
// (1 + α) × 10.5 kBd = 1.6 × 10.5 = 16.8 kHz of its 25 kHz channel;
// at 2.5 MSPS / 2048-pt FFT (1220.7 Hz/bin) that is 13.8 → 14 bins
// (17.1 kHz). Wider (the previous 16-bin guess) integrates noise-only
// bins into the detection statistic and edges toward the adjacent
// 25 kHz channel for no signal gain. UNCALIBRATED.
#define BAND_VDL2_FBT_WIDTH_BINS 14
// Detection threshold: keep the Iridium operating point. The threshold
// is relative to the per-bin EMA noise floor, so its meaning is
// band-independent, and 14 dB (our ENBW scale ≈ gri 16.4 dB) is the
// proven-permissive value on this hardware — live Iridium data showed
// marginal sub-threshold-margin decodes are productive, and dumpvdl2
// itself has NO energy gate at all (it demods its channel
// continuously), so err low. Only live VDL2 SNR statistics can justify
// moving this. UNCALIBRATED.
#define BAND_VDL2_TAG_THR_DB 14.0f

// Per-band front-end profile. All fields are consumed at boot/create
// time (dsp_processor_create, app_config_init defaults) — nothing here
// is hot-path data.
typedef struct {
    band_id_t   id;
    const char *name;          // NVS/CLI token: "iridium", "vdl2"
    uint32_t    default_lo_hz; // tuner LO default when NVS lo_hz is unset
    uint32_t    detect_fs_hz;  // detect-path sample rate (signal_buffer/tagger domain)
    // fft_burst_tagger_init() window parameters (input samples at
    // detect_fs_hz / tagger-FFT bins):
    int   fbt_pre_len;
    int   fbt_post_len;
    int   fbt_width_bins;
    float tagger_threshold_db; // default when NVS tag_thr is unset/invalid
} band_profile_t;

// Profile lookup. Out-of-range ids (including stale/corrupt NVS bytes)
// return the IRIDIUM profile — the safe default. Never returns NULL.
const band_profile_t *band_profile_get(band_id_t id);

// Parse an NVS/CLI token ("iridium" / "vdl2", case-sensitive). Unknown
// or NULL input returns BAND_IRIDIUM (the default band).
band_id_t band_profile_from_str(const char *s);
