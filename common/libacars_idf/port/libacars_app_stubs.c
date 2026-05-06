/*
 * libacars_idf — application-decoder stubs.
 *
 * libacars's acars.c hard-references four sub-decoder entry points:
 *
 *   la_arinc_parse                  (arinc.c)
 *   la_miam_parse_and_reassemble    (miam.c)
 *   la_ohma_parse_and_reassemble    (ohma.c)
 *   la_media_adv_parse              (media-adv.c)
 *
 * For our use case (Iridium SBD-carried ACARS plain text frames) we
 * only want the core ACARS frame parser — preamble / CRC / label /
 * message-number / text extraction. The application sub-decoders pull
 * in heavy dependencies (~242 ASN.1 source files for FANS-1/A CPDLC,
 * zlib/jansson/libxml2 for OHMA & MIAM-Core), so we don't compile them
 * on the embedded target.
 *
 * Without these stubs the libacars_idf archive would have unresolved
 * references to those four symbols when acars.c is linked into the
 * firmware. Returning NULL from each stub leaves the parsed acars.txt
 * field untouched and skips the application-decoder branch in
 * la_acars_parse_and_reassemble — exactly the behaviour we want.
 *
 * If the host project later wants to enable any of these sub-decoders,
 * delete the relevant stub and add the upstream .c (plus its transitive
 * deps) to libacars_idf/CMakeLists.txt.
 *
 * NOTE: we deliberately avoid editing the upstream .c files in
 * libacars/libacars/ per the project policy on vendored upstream
 * sources; this file is the adapter layer.
 */

#include <stddef.h>     /* NULL */
#include <sys/time.h>   /* struct timeval */

#include <libacars/libacars.h>     /* la_proto_node, la_msg_dir */
#include <libacars/reassembly.h>   /* la_reasm_ctx */
#include <libacars/arinc.h>        /* la_arinc_parse prototype */
#include <libacars/miam.h>         /* la_miam_parse_and_reassemble */
#include <libacars/ohma.h>         /* la_ohma_parse_and_reassemble */
#include <libacars/media-adv.h>    /* la_media_adv_parse */

la_proto_node *la_arinc_parse(char const *txt, la_msg_dir msg_dir) {
    (void)txt;
    (void)msg_dir;
    return NULL;
}

la_proto_node *la_miam_parse_and_reassemble(char const *reg, char const *txt,
        la_reasm_ctx *rtables, struct timeval rx_time) {
    (void)reg;
    (void)txt;
    (void)rtables;
    (void)rx_time;
    return NULL;
}

la_proto_node *la_ohma_parse_and_reassemble(char const *reg, char const *txt,
        la_reasm_ctx *rtables, struct timeval rx_time) {
    (void)reg;
    (void)txt;
    (void)rtables;
    (void)rx_time;
    return NULL;
}

la_proto_node *la_media_adv_parse(char const *txt) {
    (void)txt;
    return NULL;
}
