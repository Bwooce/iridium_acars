// poa_decoder — plain VHF ACARS (POA) demodulator + L2, ported from acarsdec
// (Thierry Leconte, GPLv2 — see poa_decoder.c / poa_syndrome.h headers).
//
// Input: one channel's 12.5 kHz REAL AM-envelope audio (the channelizer's
// per-channel mix -> integrate-dump -> |.| output). Output: decoded, parity-
// stripped ACARS blocks via a callback, after MSK bit recovery + the ACARS
// state machine + parity/CRC error correction. Continuous cross-block state
// (no burst framing) — this is why POA is the CHANNELIZED front end, not the
// burst tagger (docs/2026-08-01-poa-onband-plan.md §1).
//
// Pure C11 + libm + standard library — no ESP-IDF, no pthreads (acarsdec's
// block-queue thread is inlined synchronously), so tests/host links it
// directly and cross-validates byte-for-byte against acarsdec (the oracle).
// Float math (POA is low-rate; §3 of the plan quantifies it fits easily).
//
// THREADING: one poa_decoder_t is single-threaded; feed one channel at a time.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define POA_MAX_CHANNELS 8
#define POA_TXT_MAX      250

// One decoded ACARS block (7-bit chars, parity stripped, CRC verified/repaired).
typedef struct {
    int           chn;              // channel index this block came in on
    int           len;             // number of bytes in txt
    int           err;             // residual parity errors after repair (0 = clean)
    bool          crc_fixed;       // CRC was repaired (fixprerr/fixdberr) vs clean
    float         level_db;        // mean bit level, dB
    unsigned char crc[2];          // received ACARS CRC-16 (after ETX; append + DEL 0x7f for libacars)
    unsigned char txt[POA_TXT_MAX]; // block bytes (mode..ETX), 7-bit, no SOH
} poa_block_t;

// Called for every block that passes (or is repaired to) parity+CRC.
typedef void (*poa_block_cb)(const poa_block_t *blk, void *user);

typedef struct poa_decoder poa_decoder_t;

// nchannels <= POA_MAX_CHANNELS. cb fires synchronously from poa_decoder_feed.
poa_decoder_t *poa_decoder_create(int nchannels, poa_block_cb cb, void *user);

// Feed `len` real audio samples (12.5 kHz) for channel `chn`.
void poa_decoder_feed(poa_decoder_t *d, int chn, const float *audio, int len);

void poa_decoder_destroy(poa_decoder_t *d);

// Per-channel demod-activity telemetry (counters since the last reset). Lets a
// live operator see, per channel: whether the MSK demod is finding ACARS
// structure (sync = SYN locks, unlikely from noise), how far frames get
// (blk_start), and the outcome (delivered vs crc_fail) — the readout the
// channelized POA path otherwise lacks (no tagger/SNR).
typedef struct {
    uint32_t sync;      // SYN sync locks (WSYN -> SYN2)
    uint32_t blk_start; // SOH block starts (SOH1 -> TXT)
    uint32_t delivered; // blocks emitted (clean or CRC-fixed)
    uint32_t crc_fail;  // blocks reaching CRC/parity but dropped
} poa_chan_dstats_t;

// Copy up to max_ch channels' counters into out[]; if reset, zero them after.
void poa_decoder_get_stats(poa_decoder_t *d, poa_chan_dstats_t *out, int max_ch, int reset);
