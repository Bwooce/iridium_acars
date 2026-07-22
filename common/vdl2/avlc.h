// avlc — VDL Mode 2 AVLC (Aviation VHF Link Control) link-layer deframer
// (ISO/IEC 13239 HDLC subset per ICAO Annex 10 Vol III / ARINC 631).
// Pure C, integer-only, no ESP dependencies; buildable on host and target.
//
// Position in the VDL2 receive chain (plan §C3,
// docs/2026-07-22-vdl2-implementation-plan.md):
//
//   D8PSK demod -> descrambled bitstream -> de-interleave -> RS(255,249)
//   (rs_vdl2.c) -> [THIS MODULE] flag delimiting + bit-unstuffing + FCS
//   -> per-frame callback -> ACARS payloads to la_acars_parse_and_reassemble()
//   (the same libacars call frame_decoder.c's try_acars() makes for Iridium
//   SBD payloads).
//
// INPUT: the RS-corrected octet stream of ONE burst, with a bit count.
// dumpvdl2 reassembles corrected RS blocks into a bitstream LSB-first per
// octet and truncates it to the burst header's transmission-length in BITS
// (src/decode.c:326-342, v2.6.0 3f583da) — the length is usually not a
// multiple of 8 because of bit stuffing, hence n_bits, not n_octets, here.
// Bit i of the stream is (octets[i >> 3] >> (i & 7)) & 1, i.e. the standard
// HDLC LSB-first octet serialisation (dumpvdl2 bitstream_append_lsbfirst /
// bitstream_read_lsbfirst, src/bitstream.c:58-81).
//
// WHAT IT DOES (each step's reference in avlc.c):
//   1. walks 0x7E flag boundaries and removes stuffed 0-bits (a 0 following
//      five consecutive 1s), aborting on invalid sequences (>= 7 ones, or a
//      destuffed frame that does not end on an octet boundary) exactly as
//      dumpvdl2's bitstream_copy_next_frame does;
//   2. packs each frame's destuffed bits into octets LSB-first;
//   3. verifies the 16-bit HDLC FCS (CRC-16/X-25; crc16_x25_raw residue ==
//      CRC16_X25_GOOD_RESIDUE, see common/iridium_decoder/crc16.h);
//   4. parses the 4+4-octet AVLC address fields + 1-octet link control
//      field, and classifies the frame: ACARS-bearing I frame (info field
//      starts 0xFF 0xFF 0x01), other I frame (ATN/X.25 — counted, not
//      decoded), supervisory, unnumbered (XID etc.), or invalid;
//   5. fires the callback once per frame. Bad-FCS / too-short frames fire
//      too with only kind/raw fields valid — "count what you don't decode",
//      the frame_decoder.c IMS/paging convention. Only AVLC_KIND_ACARS
//      frames should be forwarded to libacars.
//
// THREADING: not reentrant (one static destuff buffer). Call from a single
// task/thread — the target caller is the Core-0 frame_decoder task (plan
// §C4), same single-consumer model as the Iridium L2 path.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Shortest parseable frame: 4 dst + 4 src + 1 control + 2 FCS octets
// (dumpvdl2 src/avlc.c:39 MIN_AVLC_LEN 11).
#define AVLC_MIN_FRAME_OCTETS 11

// Largest destuffed frame we accept. A whole burst is capped upstream at
// 0x3FFF bits (dumpvdl2 src/decode.c:45 MAX_FRAME_LENGTH) ≈ 2047 octets,
// and a single frame can never exceed its burst, so 2048 covers everything
// a spec-conformant burst can carry. Longer "frames" (only possible on
// garbage input) abort the burst like an invalid stuffing sequence.
#define AVLC_MAX_FRAME_OCTETS 2048

// AVLC address type (3-bit field). Values from dumpvdl2 src/avlc.c:102-105;
// 0, 2, 3, 6 are reserved.
#define AVLC_ADDRTYPE_AIRCRAFT 1
#define AVLC_ADDRTYPE_GS_ADM   4 // ground station (administrative)
#define AVLC_ADDRTYPE_GS_DEL   5 // ground station (delegated)
#define AVLC_ADDRTYPE_ALL      7 // broadcast, "all stations"

typedef enum {
    AVLC_KIND_ACARS = 0,   // I frame carrying ACARS: acars/acars_len valid,
                           // forward to libacars
    AVLC_KIND_X25,         // I frame carrying ATN/X.25 — count + discard
                           // (decoding ATN is plan phase V5, out of scope)
    AVLC_KIND_SUPERVISORY, // S frame (RR/RNR/REJ/SREJ) — link management
    AVLC_KIND_UNNUMBERED,  // U frame (XID/TEST/DISC/...) — link management
    AVLC_KIND_BAD_FCS,     // FCS mismatch: only kind/raw/raw_len valid
    AVLC_KIND_TOO_SHORT,   // destuffed frame < AVLC_MIN_FRAME_OCTETS: ditto
} avlc_frame_kind_t;

// One deframed AVLC frame. All pointers borrow the deframer's internal
// buffer — valid only for the duration of the callback; copy anything you
// keep (the band_pipeline "borrowed during cb" convention).
typedef struct {
    avlc_frame_kind_t kind;
    bool fcs_ok;    // true iff the CRC-16/X-25 FCS verified
    // The destuffed frame octets INCLUDING the 2 trailing FCS octets —
    // always valid, any kind (diagnostics / hex dumps / cross-validation).
    const uint8_t *raw;
    int raw_len;
    // ---- fields below are valid only when fcs_ok ----
    uint32_t dst_addr, src_addr; // 24-bit ICAO/station specific addresses
    uint8_t  dst_type, src_type; // 3-bit AVLC_ADDRTYPE_*
    bool response;      // C/R bit (carried in the src address field):
                        // false = command, true = response
    bool src_on_ground; // source Air/Ground status bit (carried in the DST
                        // address field per ARINC 631 — dumpvdl2
                        // src/avlc.c:420): false = airborne
    uint8_t control;    // the raw link control octet (I/S/U formats)
    // Information field: octets after the control field, FCS excluded.
    // Non-NULL (possibly len 0) for I and U frames and for S frames that
    // carry trailing octets; NULL when there is none.
    const uint8_t *info;
    int info_len;
    // AVLC_KIND_ACARS only: the ACARS block (mode char onward) after the
    // 3-octet 0xFF 0xFF 0x01 ACARS-over-AVLC discriminator — pass THIS to
    // la_acars_parse_and_reassemble(), not info.
    const uint8_t *acars;
    int acars_len;
} avlc_frame_t;

typedef void (*avlc_frame_cb_t)(const avlc_frame_t *f, void *ctx);

// Deframe one burst's RS-corrected octet stream (bit i = LSB-first within
// octets[i/8], see header comment). Fires cb (may be NULL to just count)
// once per frame found, including bad-FCS/too-short ones. Returns the
// number of frames emitted; stops early (frames so far still emitted and
// counted) on an invalid bit-stuffing sequence, mirroring dumpvdl2's
// whole-burst abort (src/decode.c:367-370).
int avlc_deframe_octets(const uint8_t *octets, int n_bits,
                        avlc_frame_cb_t cb, void *ctx);

// Identical, but the input is unpacked hard bits, one bit per byte (LSB of
// each byte), in the same stream order — the band_pipeline_t bit-vector
// form (band_frame_t.bits) for callers upstream of octet packing.
int avlc_deframe(const uint8_t *bits, int n_bits,
                 avlc_frame_cb_t cb, void *ctx);

#ifdef __cplusplus
}
#endif
