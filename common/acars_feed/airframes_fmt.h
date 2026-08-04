// airframes_fmt — format one decoded ACARS message into airframes.io's
// wire JSON, for BOTH bands:
//
//   AF_BAND_VDL2    -> dumpvdl2 2.6.0 native JSON ({"vdl2":{...}}), the
//                      shape airframes.io's vdlm2 ingest expects.
//   AF_BAND_IRIDIUM -> iridium-toolkit reassembler.py -m acars -a json
//                      ({"app":{"name":"iridium-toolkit",...},...}).
//
// Pure C11 + standard library only — no ESP-IDF, no FreeRTOS, no device
// headers — so tests/host links this file directly (vdl2_l2.c pattern)
// and the byte-exact wire fixtures in test_airframes_fmt.c gate every
// change to the emitted shape. The two schemas deliberately live in ONE
// module: they share the ACARS field set and the JSON escaping rules,
// and keeping them side by side makes the per-band asymmetries (see
// af_msg_t notes below) visible instead of drifting apart in two files.
//
// THREADING: stateless and reentrant — everything lives on the caller's
// stack / in the caller's output buffer.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { AF_BAND_IRIDIUM = 0, AF_BAND_VDL2 = 1, AF_BAND_POA = 2 } af_band_t;

// One decoded ACARS message, band-agnostic. Fixed-size char arrays are
// NUL-terminated; an empty string (or 0 for single-char fields) means
// "not present" and the corresponding JSON field is OMITTED, never
// emitted empty. Pointer fields may be NULL with the same meaning.
typedef struct {
    af_band_t   band;
    int64_t     epoch_us;      // wall-clock microseconds since 1970; 0 => time not synced
    bool        time_valid;    // false => omit/zero timestamp fields
    bool        uplink;        // true = ground->air (link_direction "uplink")
    bool        crc_ok;
    bool        err;
    bool        more;          // ACARS "more to come" bit (block_end = !more)
    char        mode;          // ACARS mode byte
    char        label[3];      // 2 chars + NUL
    char        block_id;      // single char (may be 0)
    char        msg_num[5];    // up to 4 chars + NUL
    char        msg_num_seq;   // single char (may be 0)
    char        flight_id[7];  // up to 6 chars + NUL
    char        reg[8];        // libacars raw reg, MAY be dot-prefixed e.g. ".F-GCBG"
    char        ack;           // ack byte; 0x15 (NAK) is emitted as "!"
    const char *txt;           // NUL-terminated payload text (may be NULL/empty)
    uint32_t    freq_hz;       // absolute channel freq; 0 => omit
    float       sig_level_dbfs;// signal level; used for VDL2 sig_level
    bool        sig_level_valid;
    char        src_addr[7];   // VDL2 AVLC source hex addr, e.g. "390826" (empty => omit avlc src)
    char        dst_addr[7];   // VDL2 AVLC dest hex addr
    const char *src_type;      // e.g. "Aircraft" / "Ground station" (may be NULL)
    const char *dst_type;
    const char *src_status;    // e.g. "Airborne" / "On ground" (may be NULL)
    const char *station_id;    // feeder station ident or UUID (may be NULL => omit)
} af_msg_t;

// Writes a single-line JSON object (NO trailing newline) into out[0..cap).
// Returns bytes written (excluding NUL), or 0 on truncation/error. On a
// 0 return the buffer contents are unspecified — the caller must not
// transmit them.
//
// Band asymmetries baked into the wire schemas (pinned by the host test):
//   - reg: VDL2 emits it VERBATIM (dumpvdl2 keeps libacars's leading
//     '.'); Iridium's "tail" strips ALL leading dots (iridium-toolkit
//     reassembler.py behaviour).
//   - label: Iridium maps "_\x7f" (underscore + DEL, the general-response
//     label) to "_d"; VDL2 emits the raw label.
//   - ack: both map the NAK byte 0x15 to "!".
//   - freq / sig_level exist only in the VDL2 schema.
size_t airframes_format(char *out, size_t cap, const af_msg_t *m);
