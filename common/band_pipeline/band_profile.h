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

// --- VDL2 profile constants (PROVISIONAL) ---------------------------------
// The VDL2 demod does not exist yet; these numbers position the shared
// front end (tagger window + LO park) and will be validated against
// dumpvdl2 on shared IQ captures before the demod lands (implementation
// plan docs/2026-07-22-vdl2-implementation-plan.md §cross-validation).
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
// Tagger pads: pre = 2×FFT (same rationale as gri — enough lookback for
// the training sequence + ramp-up); post = 16 ms of input samples. A
// VDL2 burst can be much longer than an Iridium frame (up to ~hundreds
// of ms of AVLC frames per CSMA transmission); the tagger's gone-event
// already handles long bursts via last_active tracking, the post pad
// only sets the trailing slack. PROVISIONAL until validated vs dumpvdl2.
#define BAND_VDL2_FBT_PRE_LEN 4096
#define BAND_VDL2_FBT_POST_LEN 40000
// Burst width: gri semantics = the bin span integrated when scoring a
// candidate channel. D8PSK 10.5 kBd with RRC α=0.6 occupies ≈16.8 kHz
// in a 25 kHz channel; at 2.5 MSPS / 2048-pt FFT (1220.7 Hz/bin) that
// is ≈14 bins. 16 bins ≈ 19.5 kHz covers the occupied bandwidth without
// bleeding into the adjacent 25 kHz channel. PROVISIONAL.
#define BAND_VDL2_FBT_WIDTH_BINS 16
// Detection threshold: start at the Iridium-parity value; VDL2 SNR
// statistics on the bench will move this. PROVISIONAL.
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
