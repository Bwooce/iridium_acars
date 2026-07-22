// vdl2_demod — VDL Mode 2 D8PSK burst demodulator (plan C1,
// docs/2026-07-22-vdl2-implementation-plan.md).
//
// Algorithm: a burst-mode port of dumpvdl2's phase-domain demodulator
// (the cross-validation reference for this band, the way gr-iridium is
// for Iridium). Stages:
//
//   1. Polyphase resample 250 ksps -> 105 ksps (interp 21 / decim 50,
//      Kaiser-windowed-sinc Q15 taps via firmr_s16) so the symbol rate
//      divides the sample rate exactly: SPS = 105000 / 10500 = 10,
//      the same samples-per-symbol dumpvdl2 uses (dumpvdl2.h: SPS 10).
//      The resampler lowpass doubles as the channel filter (fc ~9 kHz,
//      144 taps/phase since V4 — the sharp transition/stopband is
//      worth ~5 dB of demod EVM on the real capture vs the V2 48-tap
//      prototype; dumpvdl2 uses a 2-pole Chebyshev at 8 kHz —
//      demod.c: INP_LPF_CUTOFF_FREQ — before its own decimation).
//      Deliberately NOT an RRC matched filter: the VDL2 TX pulse is
//      the full raised cosine, so flat passband = zero ISI; the
//      root/root alternative measurably regressed the golden capture
//      (A/B table at the resampler in vdl2_demod.c).
//   2. Training-sequence (preamble) search on the per-sample PHASE
//      trajectory: the 16-symbol VDL2 sync sequence is matched as a
//      vector of expected cumulative phases, mean-removed (absolute
//      phase drops out) and linear-regression-detrended (the slope IS
//      the carrier-frequency-offset estimate, so no separate coarse-CFO
//      stage is needed). Exact port of dumpvdl2 demod.c got_sync():
//      attempt every SYNC_SKIP=3 samples, accept the first local
//      minimum of the squared phase error below SYNC_THRESHOLD=4.0,
//      parabolic-vertex interpolation for the symbol-clock origin.
//      NOTE vs the implementation plan's C1 sketch: the plan proposed
//      8th-power coarse CFO + Gardner + an 8-ary PLL; dumpvdl2 uses
//      none of those (the preamble regression supplies both CFO and the
//      symbol strobe; the clock free-runs — at ±5 ppm over the longest
//      legal frame the drift is < T/10). We follow the reference, which
//      keeps the validation story one-to-one.
//   3. Differential 8-PSK slicing at symbol strobes (every SPS samples
//      from the sync vertex): dphi = phi[k] - phi[k-1] - dphi_cfo,
//      idx = round(dphi / (pi/4)) mod 8, bits = graycode[idx]
//      (demod.c:223 graycode = {0,1,3,2,6,7,5,4}), 3 bits/symbol
//      MSB-first (demod.c:274 bitstream_append_msbfirst). V4 delta vs
//      the reference: strobes land on the FRACTIONAL symbol instant
//      from the sync parabola (cubic interpolation of the filtered
//      stream) instead of dumpvdl2's whole-sample T/10 grid.
//   4. Descramble: 15-stage additive LFSR, polynomial x^15 + x + 1,
//      initial value 0x6959, applied to every bit from the first
//      post-preamble symbol on (decode.c:50 LFSR_IV,
//      bitstream.c:100-102 bitstream_descramble).
//   5. Burst header decode: 25 bits = 3 reserved (transmitted 0) +
//      17-bit transmission length (bit-reversed on air, decode.c:222)
//      + 5 parity bits of a (25,20) block code (H matrix + syndrome
//      table: decode.c:55-96). Single-bit errors corrected; frames
//      with post-correction nonzero reserved bits or implausible
//      length rejected (decode.c MAX_FRAME_LENGTH 0x3FFF /
//      MAX_FRAME_LENGTH_CORRECTED 0x1FFF). The header tells us how
//      many bits the burst carries, so the output is trimmed to the
//      frame and the caller can resume scanning after it (CSMA
//      back-to-back transmissions).
//
// Output = the descrambled PHY bitstream starting at the reserved
// symbol (header included), exactly dumpvdl2's post-descramble
// bitstream. Byte packing, RS(255,249) de-interleave/decode
// (common/vdl2/rs_vdl2.h) and AVLC deframing are downstream (plan
// C2/C3) — this module is DSP -> bits only, mirroring the Iridium
// split.
//
// All dumpvdl2 references are to the 2026-07 tree of
// https://github.com/szpajder/dumpvdl2 (src/ paths). Symbol rate
// 10500 Bd, 3 bits/symbol: dumpvdl2.h SYMBOL_RATE / BPS.
//
// Pure C, no ESP-IDF deps, no PIE (plan: VDL2 is scalar-only, so the
// PIE placement/ownership landmines don't apply). Host and target
// compile the same source.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VDL2_SYMBOL_RATE_HZ 10500 // dumpvdl2.h: SYMBOL_RATE
#define VDL2_BPS            3     // dumpvdl2.h: BPS (bits per symbol)
#define VDL2_PREAMBLE_SYMS  16    // dumpvdl2.h: PREAMBLE_SYMS
#define VDL2_SPS            10    // samples/symbol at 105 ksps (dumpvdl2.h: SPS)
#define VDL2_FS_IN_HZ       250000 // band_pipeline input contract
#define VDL2_FS_DEMOD_HZ    105000 // internal rate = SYMBOL_RATE * SPS
#define VDL2_RESAMP_INTERP  21     // 250000 * 21 / 50 = 105000
#define VDL2_RESAMP_DECIM   50

// Header layout (dumpvdl2.h: TRLEN, HDRFECLEN, HEADER_LEN).
#define VDL2_HDR_RESERVED_BITS 3
#define VDL2_HDR_TRLEN_BITS    17
#define VDL2_HDR_FEC_BITS      5
#define VDL2_HDR_BITS          (VDL2_HDR_RESERVED_BITS + VDL2_HDR_TRLEN_BITS + VDL2_HDR_FEC_BITS) // 25

// Scrambler (decode.c: LFSR_IV; bitstream.c: bitstream_descramble).
#define VDL2_LFSR_IV 0x6959u

// Longest plausible transmission length in bits (decode.c:
// MAX_FRAME_LENGTH / MAX_FRAME_LENGTH_CORRECTED — the tighter limit
// applies when the header needed FEC correction).
#define VDL2_MAX_FRAME_BITS           0x3FFF
#define VDL2_MAX_FRAME_BITS_CORRECTED 0x1FFF

typedef struct {
    // Descrambled hard bits (one per byte), starting at the reserved
    // symbol: bits[0..24] = burst header, bits[25..] = data + RS FEC
    // as transmitted. malloc'd; caller owns (free()).
    uint8_t *bits;
    // Per-bit soft metric: sign = hard decision (bit 0 -> positive),
    // magnitude = confidence from the differential-phase distance to
    // the sliced constellation point. malloc'd; caller owns. Note the
    // soft value reflects the SCRAMBLED channel bit's confidence; the
    // descrambler flips hard values but confidence is unaffected.
    int16_t *soft_bits;
    int      n_bits; // bits actually demodulated (<= n_bits_needed)
    // Total bits this frame carries per the decoded header:
    // 25 + 8*(data octets + RS FEC octets). n_bits < n_bits_needed
    // means the burst window ended before the frame did (truncated).
    int n_bits_needed;
    bool complete; // n_bits == n_bits_needed

    // Diagnostics / cross-validation hooks.
    float    cfo_hz;      // carrier offset from the preamble regression
    int      sync_offset; // sample index (105 ksps domain) of the sync
                          // vertex = last preamble symbol strobe
    float    evm_rms;     // RMS differential-phase error, radians
    float    snr_db;      // ~ -20*log10(evm_rms): phase-noise-derived
                          // SNR proxy (sigma_phi ~ 1/sqrt(SNR))
    uint32_t datalen_bits; // header transmission-length field (payload
                           // bits, EXCLUDING header and RS FEC octets)
    int hdr_synd_weight;   // 0 = clean header, 1/2 = corrected bits
    // Consumed input (250 ksps domain) through the end of this frame —
    // the caller's cursor for scanning the remainder of the window for
    // a back-to-back transmission.
    int consumed_complex_250k;
} vdl2_demod_result_t;

// Demodulate the FIRST decodable VDL2 burst found in the window.
// iq250: interleaved int16 IQ at 250 ksps, channel centred at DC
// (band_pipeline input contract). Returns false if no preamble locked
// anywhere in the window or every lock failed header validation
// (*out is zeroed apart from diagnostics; nothing to free). On true,
// out->bits / out->soft_bits are malloc'd and owned by the caller.
bool vdl2_demod_burst(const int16_t *iq250, int n_complex,
                      vdl2_demod_result_t *out);

// ---- shared protocol helpers (also used by the test modulator and,
// ---- later, the L2 feed) ----

// Body length in bits that follows the 25-bit header for a given
// transmission length: 8 * (data octets + RS FEC octets), FEC octets
// per dumpvdl2 decode.c get_fec_octetcount(): last partial RS block
// gets 0/2/4/6 parity octets for <3 / <31 / <68 / >=68 data octets;
// full RS(255,249) blocks always carry 6. Returns -1 if datalen_bits
// is 0 or exceeds VDL2_MAX_FRAME_BITS.
int vdl2_burst_body_bits(uint32_t datalen_bits);

// (25,20) header block code. Encode: build the 25-bit transmitted
// header word (MSB = first bit on air) for a transmission length:
// [3 reserved zeros][reverse(datalen,17)][5 parity]. Parity per the H
// matrix in dumpvdl2 decode.c:55-61.
uint32_t vdl2_hdr_encode(uint32_t datalen_bits);

// Decode: correct *hdr (25-bit word) in place via the syndrome table
// (decode.c:63-96), returns the syndrome weight (0 = clean, >0 =
// corrected) or -1 if the corrected word still has nonzero reserved
// bits (reject). On success *datalen_bits gets the bit-reversed
// length field.
int vdl2_hdr_decode(uint32_t *hdr, uint32_t *datalen_bits);

// Additive scrambler/descrambler (same operation both directions):
// XOR bits[0..n) with the LFSR keystream, advancing *lfsr. Feedback
// bit = (lfsr ^ lfsr>>14) & 1, shifted in at stage 14
// (bitstream.c:100-102). Init *lfsr = VDL2_LFSR_IV at the first bit
// after the preamble.
void vdl2_scramble(uint8_t *bits, int n_bits, uint16_t *lfsr);

// Cumulative preamble phase table (radians), dumpvdl2 demod.c:107-124.
// Index i = phase after preamble symbol i relative to symbol 0's.
extern const float vdl2_preamble_phase[VDL2_PREAMBLE_SYMS];

// D8PSK Gray map: 3 transmitted bits for differential phase index
// 0..7 (units of pi/4), dumpvdl2 demod.c:223.
extern const uint8_t vdl2_graycode[8];

// Kaiser-windowed-sinc polyphase lowpass designer (Q15, firmr_s16
// layout coeffs[tap * interp + phase], per-phase DC gain normalised to
// 1.0). fc is in cycles per VIRTUAL sample (= f_cut_hz / (fs_in_hz *
// interp)). Exposed for the demod's own init and for host harnesses
// that need other ratios (e.g. 48 ksps reference captures -> 250 k).
// coeffs must hold delay_size * interp entries. Design pass runs in
// double on a small heap scratch; returns false on alloc failure
// (coeffs zeroed).
bool vdl2_lpf_design_q15(int16_t *coeffs, int delay_size, int interp,
                         double fc_cycles_per_vsample, double beta);

// Root-raised-cosine polyphase prototype designer (Q15, firmr_s16
// layout, per-phase DC gain 1.0 — same conventions as
// vdl2_lpf_design_q15). vsamples_per_symbol = symbol period in VIRTUAL
// samples (= fs_in_hz * interp / symbol_rate). Prototype spans
// delay_size input samples, truncated (no window — windowing would
// detune the shape); size the span so the truncated tail is negligible
// (RRC alpha 0.6 is ~0.15 % of peak at 4 T). Returns false on
// scratch-alloc failure (coeffs zeroed).
//
// NOT used by the production demod: the VDL2 transmitter applies the
// FULL raised cosine (ICAO Annex 10 Vol III, alpha 0.6), so an RX-side
// RRC "matched filter" un-Nyquists the cascade — measured on the
// sigidwiki golden capture it DROPPED RS-clean frames 46 -> 21 (see
// the resampler note in vdl2_demod.c). Kept (and unit-covered in
// test_vdl2_demod) for V4 live-TX probing, where a real over-the-air
// transmitter — not a capture already narrowed by the recording
// receiver's front end — can re-arbitrate the shape question.
bool vdl2_rrc_design_q15(int16_t *coeffs, int delay_size, int interp,
                         double vsamples_per_symbol, double alpha);

#ifdef __cplusplus
}
#endif
