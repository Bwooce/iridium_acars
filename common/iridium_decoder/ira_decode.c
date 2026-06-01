// IRA (Iridium Ring Alert) decoder. See ira_decode.h.
//
// Direct port of iridium-toolkit/bitsparser.py:IridiumRAMessage init
// (line 1553) restricted to the 63-bit fixed header. Paging blocks
// (12 × 42 bits after the header) are not parsed -- they carry TMSI
// addresses for paging which aren't user-visible.

#include "ira_decode.h"
#include "iridium_bch.h"
#include <string.h>
#include <math.h>

#define UW_BITS 24
#define RA_HEAD 96 // 3 × 32-bit interleaved codewords

// Same de_interleave3 as in iridium_frame.c -- kept local rather than
// exporting since the classifier was inlined for header-detection speed.
// 96 bits → three 32-bit outputs (high-symbol-first ordering).
static void de_interleave3(const uint8_t *in, size_t n_in,
                           uint8_t *first_out, uint8_t *second_out,
                           uint8_t *third_out)
{
    int n_sym     = (int)(n_in / 2);
    int third_idx = 0, second_idx = 0, first_idx = 0;
    for (int s = n_sym - 3; s >= 0; s -= 3) {
        third_out[third_idx++] = in[2 * s + 1] & 1;
        third_out[third_idx++] = in[2 * s + 0] & 1;
    }
    for (int s = n_sym - 2; s >= 0; s -= 3) {
        second_out[second_idx++] = in[2 * s + 1] & 1;
        second_out[second_idx++] = in[2 * s + 0] & 1;
    }
    for (int s = n_sym - 1; s >= 0; s -= 3) {
        first_out[first_idx++] = in[2 * s + 1] & 1;
        first_out[first_idx++] = in[2 * s + 0] & 1;
    }
}

static uint32_t pick_bits(const uint8_t *bits, int start, int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++) {
        v = (v << 1) | (bits[start + i] & 1u);
    }
    return v;
}

// Sign-extend a 12-bit two's-complement value: sign at bit position
// `sign_idx`, magnitude at `mag_start..mag_start+10`. Matches the
// iridium-toolkit formula: int(bits[mag], 2) - bit(sign)*(1<<11).
static int sign_extend_12(const uint8_t *bits, int sign_idx,
                          int mag_start, int mag_n)
{
    int mag  = (int)pick_bits(bits, mag_start, mag_n);
    int sign = bits[sign_idx] & 1;
    return mag - (sign << 11);
}

int ira_decode(const iridium_frame_t *frame, ira_decoded_t *out)
{
    if (!frame || !out) return -1;
    memset(out, 0, sizeof(*out));

    if (frame->n_bits < UW_BITS + RA_HEAD) return -1;
    const uint8_t *p = frame->bits + UW_BITS;

    // Same 2-bit-pair swap iridium_frame_classify does. RA classification
    // already passed in `swapped` form, but the bits handed to us in
    // frame->bits are the original (UN-swapped) demod output. Swap the
    // first 96 bits to match the classifier's view.
    uint8_t swapped[RA_HEAD];
    for (int i = 0; i + 1 < RA_HEAD; i += 2) {
        swapped[i + 0] = p[i + 1] & 1;
        swapped[i + 1] = p[i + 0] & 1;
    }

    // De-interleave into three 32-bit codewords and BCH-repair each.
    uint8_t cw1[32], cw2[32], cw3[32];
    de_interleave3(swapped, RA_HEAD, cw1, cw2, cw3);
    int e1 = iridium_bch_repair2(1207u, cw1, 31);
    int e2 = iridium_bch_repair2(1207u, cw2, 31);
    int e3 = iridium_bch_repair2(1207u, cw3, 31);
    if (e1 < 0 || e2 < 0 || e3 < 0) {
        out->bch_ok = false;
        return 0;
    }
    out->bch_ok = true;

    // Concatenate the three data parts (21 + 21 + 21 = 63 bits).
    uint8_t hdr[63];
    memcpy(hdr + 0, cw1, 21);
    memcpy(hdr + 21, cw2, 21);
    memcpy(hdr + 42, cw3, 21);

    // Field extraction matches iridium-toolkit IridiumRAMessage:
    out->sv_id    = (int)pick_bits(hdr, 0, 7);
    out->beam_id  = (int)pick_bits(hdr, 7, 6);
    out->pos_x    = sign_extend_12(hdr, 13, 14, 11);
    out->pos_y    = sign_extend_12(hdr, 25, 26, 11);
    out->pos_z    = sign_extend_12(hdr, 37, 38, 11);
    out->ra_int   = (int)pick_bits(hdr, 49, 7);
    out->ra_ts    = (int)pick_bits(hdr, 56, 1);
    out->ra_eip   = (int)pick_bits(hdr, 57, 1);
    out->ra_bc_sb = (int)pick_bits(hdr, 58, 5);

    // Convert (x, y, z) — in 4-km LSB units — to geocentric lat/lon/alt.
    // iridium-toolkit's formula:
    //   ra_lat = atan2(z, sqrt(x² + y²)) * 180/π
    //   ra_lon = atan2(y, x) * 180/π
    //   ra_alt = sqrt(x² + y² + z²) * 4   (in km)
    // Geocentric latitude (not geodetic) — off by up to 0.2° at high lat.
    double dx = (double)out->pos_x;
    double dy = (double)out->pos_y;
    double dz = (double)out->pos_z;
    double r2 = dx * dx + dy * dy + dz * dz;
    if (r2 > 0.0) {
        out->lat_deg = (float)(atan2(dz, sqrt(dx * dx + dy * dy)) * 180.0 / M_PI);
        out->lon_deg = (float)(atan2(dy, dx) * 180.0 / M_PI);
        out->alt_km  = (float)(sqrt(r2) * 4.0);
    } else {
        out->lat_deg = 0.0f;
        out->lon_deg = 0.0f;
        out->alt_km  = 0.0f;
    }
    return 0;
}
