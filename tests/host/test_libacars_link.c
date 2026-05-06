/*
 * Host-side smoke test: prove that libacars's core ACARS parser entry
 * points are reachable from a vendored compile of the libacars_idf
 * sources. We're not testing parse correctness here — that's upstream
 * libacars's job — we're testing that the minimum-source-set we
 * compile in common/libacars_idf/ actually links.
 *
 * Failure modes this test catches:
 *   - missing source file in the SRCS list (undefined symbol at link)
 *   - mis-configured config.h (compile-time error)
 *   - regressed function signature on libacars upgrade
 *
 * Runtime behaviour: la_acars_parse(NULL, 0, ...) returns NULL
 * immediately (early-out for buf==NULL); we just check that.
 *
 * NOTE: libacars's util.c calls _exit(1) on calloc/realloc failure;
 * since we never allocate here that path is dead. The la_proto_node
 * trees we DO create (one for the "valid" path below) are leaked —
 * acceptable for a single-shot link test.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h>

#include <libacars/libacars.h>   /* la_msg_dir, la_proto_node */
#include <libacars/acars.h>      /* la_acars_parse, la_acars_extract_sublabel_and_mfi */
#include <libacars/crc.h>        /* la_crc16_ccitt */

// --- helper: walk a la_proto_node tree to find the la_acars_msg payload.
extern la_type_descriptor const la_DEF_acars_message;
static la_acars_msg *find_acars_msg(la_proto_node *node) {
    while (node) {
        if (node->td == &la_DEF_acars_message && node->data) {
            return (la_acars_msg *)node->data;
        }
        node = node->next;
    }
    return NULL;
}

int main(void) {
    int failures = 0;

    /* 1. NULL buffer should return NULL without parsing. */
    {
        la_proto_node *node = la_acars_parse(NULL, 0, LA_MSG_DIR_UNKNOWN);
        if (node != NULL) {
            fprintf(stderr, "FAIL: la_acars_parse(NULL, 0, ...) returned %p (expected NULL)\n",
                    (void *)node);
            failures++;
        }
    }

    /* 2. la_acars_extract_sublabel_and_mfi rejects NULL inputs. */
    {
        char sublabel[3] = { 0 }, mfi[3] = { 0 };
        int r = la_acars_extract_sublabel_and_mfi(NULL, LA_MSG_DIR_AIR2GND,
                NULL, 0, sublabel, mfi);
        if (r != -1) {
            fprintf(stderr, "FAIL: la_acars_extract_sublabel_and_mfi(NULL,...) = %d (expected -1)\n", r);
            failures++;
        }
    }

    /* 3. Too-short buffer should produce an err=true acars node, not crash.
     *    We use a fake 4-byte buffer; LA_ACARS_PREAMBLE_LEN is 16, so this
     *    triggers the "Preamble too short" path inside acars.c. */
    {
        uint8_t fake[4] = { 0x42, 0x42, 0x42, 0x7f };
        la_proto_node *node = la_acars_parse(fake, 4, LA_MSG_DIR_AIR2GND);
        if (node == NULL) {
            fprintf(stderr, "FAIL: la_acars_parse on too-short buf returned NULL "
                    "(expected node with err=true)\n");
            failures++;
        }
        /* Don't free — see file-header note. */
    }

    /* 4. Canonical ACARS frame: build a known frame in memory, compute the
     *    CRC with libacars's own la_crc16_ccitt, parse it, verify the
     *    library extracts the expected mode/label/block_id/msg_num/
     *    flight_id/text/crc_ok. This catches regressions in upstream
     *    libacars's parser logic and confirms the field-extraction layer
     *    works (not just the entry-point symbols).
     *
     *    Frame layout (NOT including the leading SOH 0x01 — la_acars_parse
     *    expects it pre-stripped — but DOES include the trailing DEL 0x7f):
     *
     *      [0]   mode      '2'         downlink
     *      [1-7] address   "ABCDEFG"   7-byte aircraft registration field
     *      [8]   ack       0x15 (NAK)  no-ack indicator
     *      [9-10] label    "H1"        hopcount-based label
     *      [11]  block_id  '5'   (digit '0'-'9' = downlink — libacars uses
     *                              the block_id range to discriminate
     *                              direction; non-digit means uplink)
     *      [12]  STX       0x02
     *      [13-16] msg_num "M001"
     *      [17-22] flight_id "FLT123"
     *      [23-...] text   "DOWNLINK MSG OK"
     *      [N]   ETX       0x03
     *      [N+1..N+2] CRC  computed over [0..N] (mode through ETX)
     *      [N+3] DEL       0x7f
     */
    {
        uint8_t buf[64];
        size_t i = 0;

        buf[i++] = '2';
        memcpy(buf + i, "ABCDEFG", 7); i += 7;
        buf[i++] = 0x15;                /* ACK byte; 0x15 = NAK */
        buf[i++] = 'H';
        buf[i++] = '1';
        buf[i++] = '5';                 /* block_id = digit -> downlink */
        buf[i++] = 0x02;                /* STX */
        memcpy(buf + i, "M001", 4);   i += 4;
        memcpy(buf + i, "FLT123", 6); i += 6;
        const char *txt = "DOWNLINK MSG OK";
        memcpy(buf + i, txt, strlen(txt));
        i += strlen(txt);
        buf[i++] = 0x03;                /* ETX */

        /* CRC covers from start of buf through ETX inclusive.
         * la_crc16_ccitt's init value is 0; matches what libacars's
         * own internal acars-frame validator uses. */
        uint16_t crc = la_crc16_ccitt(buf, i, 0);
        buf[i++] = (uint8_t)(crc & 0xff);
        buf[i++] = (uint8_t)((crc >> 8) & 0xff);
        buf[i++] = 0x7f;                /* DEL terminator */

        la_proto_node *node = la_acars_parse(buf, i, LA_MSG_DIR_AIR2GND);
        if (node == NULL) {
            fprintf(stderr, "FAIL canonical-frame: la_acars_parse returned NULL\n");
            failures++;
        } else {
            la_acars_msg *m = find_acars_msg(node);
            if (m == NULL) {
                fprintf(stderr, "FAIL canonical-frame: no la_acars_msg in tree\n");
                failures++;
            } else {
                if (!m->crc_ok) {
                    fprintf(stderr, "FAIL canonical-frame: crc_ok=false (expected true)\n");
                    failures++;
                }
                if (m->err) {
                    fprintf(stderr, "FAIL canonical-frame: err=true (expected false)\n");
                    failures++;
                }
                if (m->mode != '2') {
                    fprintf(stderr, "FAIL canonical-frame: mode='%c' (expected '2')\n", m->mode);
                    failures++;
                }
                if (memcmp(m->label, "H1", 2) != 0) {
                    fprintf(stderr, "FAIL canonical-frame: label='%.2s' (expected 'H1')\n", m->label);
                    failures++;
                }
                if (m->block_id != '5') {
                    fprintf(stderr, "FAIL canonical-frame: block_id='%c' (expected '5')\n", m->block_id);
                    failures++;
                }
                /* msg_num is 3 chars + msg_num_seq char (libacars splits "M00"
                 * + sequence "1" — see acars.c:400). */
                if (memcmp(m->msg_num, "M00", 3) != 0 || m->msg_num_seq != '1') {
                    fprintf(stderr, "FAIL canonical-frame: msg_num='%.3s%c' (expected 'M001')\n",
                            m->msg_num, m->msg_num_seq);
                    failures++;
                }
                if (memcmp(m->flight_id, "FLT123", 6) != 0) {
                    fprintf(stderr, "FAIL canonical-frame: flight_id='%.6s' (expected 'FLT123')\n",
                            m->flight_id);
                    failures++;
                }
                if (m->txt == NULL || strcmp(m->txt, txt) != 0) {
                    fprintf(stderr, "FAIL canonical-frame: txt='%s' (expected '%s')\n",
                            m->txt ? m->txt : "(null)", txt);
                    failures++;
                }
            }
            /* Don't free — see file-header note. */
        }
    }

    /* 5. Bad-CRC frame: same canonical frame but with the CRC bytes flipped.
     *    Should still parse the structure (extract label, msg_num, etc.)
     *    but mark crc_ok=false. Verifies graceful handling of a corrupted
     *    frame, which we'll see plenty of in real RF.
     */
    {
        uint8_t buf[64];
        size_t i = 0;
        buf[i++] = '2';
        memcpy(buf + i, "BADCRC ", 7); i += 7;
        buf[i++] = 0x15;
        memcpy(buf + i, "H1", 2);     i += 2;
        buf[i++] = 'A';
        buf[i++] = 0x02;
        memcpy(buf + i, "X007", 4);   i += 4;
        memcpy(buf + i, "BADCRC", 6); i += 6;
        const char *txt = "BAD CRC";
        memcpy(buf + i, txt, strlen(txt));
        i += strlen(txt);
        buf[i++] = 0x03;
        /* Deliberately wrong CRC */
        buf[i++] = 0xFF;
        buf[i++] = 0xFF;
        buf[i++] = 0x7f;

        la_proto_node *node = la_acars_parse(buf, i, LA_MSG_DIR_AIR2GND);
        la_acars_msg *m = (node != NULL) ? find_acars_msg(node) : NULL;
        if (m == NULL) {
            fprintf(stderr, "FAIL bad-crc: no la_acars_msg returned\n");
            failures++;
        } else {
            if (m->crc_ok) {
                fprintf(stderr, "FAIL bad-crc: crc_ok=true (expected false)\n");
                failures++;
            }
            /* The library should still extract label/msg_num/flight_id even
             * with a bad CRC, since the structure is recoverable. */
            if (memcmp(m->label, "H1", 2) != 0) {
                fprintf(stderr, "FAIL bad-crc: label='%.2s' (expected 'H1')\n", m->label);
                failures++;
            }
        }
    }

    if (failures == 0) {
        printf("PASS test_libacars_link\n");
        return 0;
    }
    fprintf(stderr, "FAIL test_libacars_link: %d failure(s)\n", failures);
    return 1;
}
