// poa_mod — synthetic POA (plain VHF ACARS) modulator, for host test fixtures
// and the CONFIG_SMOKE_TEST_POA on-device smoke ONLY (the vdl2_mod.c analogue;
// nothing on the device transmits). Deterministic, seedable — same inputs give
// the same bytes, so round-trip tests are reproducible.
//
// This is the exact INVERSE of acarsdec's demod (msk.c + acars.c), which the
// in-tree poa_decoder/poa_frontend port is bit-faithful to. It builds a known
// ACARS block, wraps it in the ARINC-618 air framing (pre-key + SYN SYN SOH +
// block + CRC), MSK-modulates it as a 2400 bps continuous-phase signal on an
// 1800 Hz audio sub-carrier, then AM-modulates that onto a chosen VHF channel
// as 2.5 MSPS complex int16 IQ — precisely the waveform the channelizer front
// end expects. Feeding the output through poa_frontend -> poa_decoder must
// recover the exact block bytes with err == 0 and crc_fixed == false: that
// round-trip IS the correctness proof.
//
// SIGNAL MODEL
//   ACARS is 2400 bps MSK (a CPFSK with modulation index 0.5) on an 1800 Hz
//   sub-carrier, so the two tones are 1800 +/- 600 = 1200 / 2400 Hz and the
//   phase advances by exactly +/-pi/2 per bit. The sub-carrier amplitude-
//   modulates the VHF carrier (env(t) = 1 + m*msk(t), m < 1 so env > 0); the
//   channelizer's per-channel mix + integrate-dump + |.| recovers env(t), the
//   1800 Hz sub-carrier survives (DC maps outside the matched-filter passband),
//   and the MSK demod recovers the bits. See poa_mod.c for the bit<->phase
//   encoding derivation (it inverts msk.c's alternating-axis, MskS&2-polarity
//   decision) and the strobe-parity alignment.
//
// SNR CONVENTION
//   poa_mod_add_awgn adds i.i.d. complex Gaussian noise (sigma per I and per Q
//   component). SNR here is WIDEBAND: signal power / noise power measured over
//   the whole fs band, where signal power = mean(|IQ|^2) of the clean burst and
//   noise power = 2*sigma^2. The decoder sees ~23 dB more than this: the 200:1
//   integrate-dump (rtlMult = fs/12500 = 200) sums the carrier coherently and
//   the noise incoherently (10*log10(200) ~ 23 dB of processing gain), so a
//   negative wideband SNR still decodes cleanly. Use poa_mod_sigma_for_snr to
//   turn a target wideband SNR into a sigma.
//
// Pure C11 + libm — no ESP-IDF deps, host-compiles like the rest of
// common/poa_decoder.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// One ACARS block to modulate. Fields map 1:1 onto the on-air block the decoder
// recovers in poa_block_t.txt (parity stripped): mode, 7-char registration,
// technical-ack, 2-char label, block id, STX, text, ETX/ETB. The modulator adds
// the odd-parity bit to every char and the trailing CRC-16.
typedef struct {
    char        mode;        // ACARS mode char (e.g. '2')
    char        reg[8];      // registration/address, up to 7 chars (NUL-pad ok)
    char        ack;         // technical ack; 0x15 (NAK) = "no ack"
    char        label[3];    // 2-char label (e.g. "H1"); label[1] may be 0x7f (_d)
    char        block_id;    // '0'..'9' = downlink; a letter = uplink
    const char *text;        // block text (post-STX); may be NULL/empty for uplink
    int         text_len;    // bytes in text (if < 0, strlen(text) is used)
    int         final_block; // 1 -> terminate with ETX (last/only), 0 -> ETB
} poa_mod_msg_t;

typedef struct {
    uint32_t fs_hz;       // IQ sample rate; must be a multiple of 12500 (2500000)
    uint32_t lo_hz;       // capture centre frequency (channelizer LO)
    uint32_t chan_hz;     // absolute channel frequency of this burst
    int      prekey_bits; // MSK pre-key bits before SYN (PLL/bit-clock lock)
    int      postkey_bits;// trailing bits to flush the last byte + settle
    double   amp;         // carrier amplitude (int16 LSB); keep amp*(1+depth) well under 32767
    double   mod_depth;   // AM modulation depth m in env = 1 + m*msk, 0 < m < 1
    int      axis_parity; // 0..3: constellation pre-rotation (quarter turns).
                          // MEASURED INERT for the current decoder — the PLL
                          // absorbs any common rotation, so all four values
                          // decode identically (see poa_mod.c). Kept as a
                          // documented lever; leave 0.
} poa_mod_params_t;

// fs 2500000, lo 130800000, chan 131550000, prekey 192, postkey 24,
// amp 1000 (clipping-free sweep), depth 0.9, axis_parity 0.
void poa_mod_params_default(poa_mod_params_t *p);

// Build the raw on-air block bytes (mode..ETX/ETB) WITH the odd-parity bit set,
// i.e. exactly what the decoder holds in txt[] before it strips the high bit.
// Returns the block length, or -1 on bad args / overflow. Handy for tests to
// derive the expected decoded text (mask each byte & 0x7f).
int poa_mod_build_block(const poa_mod_msg_t *msg, uint8_t *out, int max_out);

// Modulate one block into interleaved int16 IQ at params->fs_hz.
// phase_ref is the GLOBAL complex-sample index of out_iq[0] in a continuously
// fed stream (the carrier and bit grid are phase-referenced to it, so multiple
// bursts fed back-to-back stay coherent with the decoder's continuous VCO).
// Pass the running total of complex samples already fed. The returned sample
// count is always a multiple of 6250 (= 6 bit periods, an exact integer number
// of samples at 2.5 MSPS), so phase_ref stays grid-aligned across bursts.
// Returns the complex sample count written, or -1 on bad args / capacity.
int poa_mod_block(const poa_mod_msg_t *msg, const poa_mod_params_t *p,
                  int64_t phase_ref, int16_t *out_iq, int max_complex);

// Mean |x|^2 over the interleaved IQ buffer (the wideband signal power).
double poa_mod_signal_power(const int16_t *iq, int n_complex);

// sigma (per I/Q component) that yields the target wideband SNR (dB) for a
// burst of the given signal power. sigma = sqrt(signal_power / (2*10^(snr/10))).
double poa_mod_sigma_for_snr(double signal_power, double snr_db);

// Add i.i.d. complex Gaussian noise (sigma per component) in place, clamped to
// int16. Deterministic in seed. Returns the number of clamped (clipped)
// int16 samples (should be ~0 for a well-scaled burst).
int poa_mod_add_awgn(int16_t *iq, int n_complex, double sigma, uint32_t seed);

#ifdef __cplusplus
}
#endif
