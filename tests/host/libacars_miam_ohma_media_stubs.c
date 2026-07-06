/*
 * Stubs for the three application decoders acars.c hard-references that
 * this test target deliberately does NOT compile (MIAM/OHMA/media-adv --
 * they'd pull in zlib/jansson/libxml2, see the CMakeLists.txt comment
 * above test_libacars_best_effort). Modeled on
 * common/libacars_idf/port/libacars_app_stubs.c, but WITHOUT its
 * la_arinc_parse stub: test_libacars_best_effort compiles the real
 * arinc.c so acars.c's label "H1" dispatch
 * (acars.c:130-194, la_acars_apps_parse_and_reassemble) reaches the real
 * ARINC-622 -> CPDLC/ADS-C chain for the §6c ZK-NNC acceptance test.
 */

#include <stddef.h>   /* NULL */
#include <sys/time.h> /* struct timeval */

#include <libacars/libacars.h>   /* la_proto_node, la_msg_dir */
#include <libacars/reassembly.h> /* la_reasm_ctx */
#include <libacars/miam.h>       /* la_miam_parse_and_reassemble */
#include <libacars/ohma.h>       /* la_ohma_parse_and_reassemble */
#include <libacars/media-adv.h>  /* la_media_adv_parse */

la_proto_node *la_miam_parse_and_reassemble(char const *reg, char const *txt,
                                            la_reasm_ctx *rtables, struct timeval rx_time)
{
    (void)reg;
    (void)txt;
    (void)rtables;
    (void)rx_time;
    return NULL;
}

la_proto_node *la_ohma_parse_and_reassemble(char const *reg, char const *txt,
                                            la_reasm_ctx *rtables, struct timeval rx_time)
{
    (void)reg;
    (void)txt;
    (void)rtables;
    (void)rx_time;
    return NULL;
}

la_proto_node *la_media_adv_parse(char const *txt)
{
    (void)txt;
    return NULL;
}
