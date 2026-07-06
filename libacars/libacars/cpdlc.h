/*
 *  This file is a part of libacars
 *
 *  Copyright (c) 2018-2023 Tomasz Lemiech <szpajder@gmail.com>
 */

#ifndef LA_CPDLC_H
#define LA_CPDLC_H 1

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>                          // size_t
#include <libacars/libacars.h>              // la_type_descriptor, la_proto_node
#include <libacars/vstring.h>               // la_vstring
#include <libacars/asn1/asn_application.h>  // asn_TYPE_descriptor_t

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	asn_TYPE_descriptor_t *asn_type;
	void *data;
	bool err;
	// best_effort_decode (design doc
	// docs/superpowers/plans/2026-07-07-libacars-best-effort-decode.md §4):
	// true whenever err == false but the decode was not a clean, fully-
	// consumed success. trailing_junk distinguishes the two ways that can
	// happen -- do NOT conflate them, they carry very different trust
	// levels (design §4's flagged policy trap):
	//   partial && !trailing_junk : uPER decode desynced mid-message
	//     (RC_FAIL). The tree is a genuine partial parse -- fields before
	//     the desync point are real, fields after are calloc'd zero /
	//     plausible-looking garbage (asn1-util.c §3 contract).
	//   partial &&  trailing_junk : uPER decode completed successfully
	//     (RC_OK) but didn't consume the whole buffer. The whole tree is
	//     a normal, fully-valid decode; only the ignored trailing bytes
	//     are suspect.
	// consumed_bits is only meaningful when partial == true; see cpdlc.c
	// for how it's derived from asn_dec_rval_t.consumed (byte-granular,
	// not bit-exact -- see per_decoder.c / patches/README.md #0004).
	bool partial;
	bool trailing_junk;
	size_t consumed_bits;   // meaningful iff partial == true
	size_t total_bits;      // input length in bits, for the banner's "of %zu"
	// reserved for future use
	void (*reserved0)(void);
} la_cpdlc_msg;

// cpdlc.c
extern la_type_descriptor const la_DEF_cpdlc_message;
la_proto_node *la_cpdlc_parse(uint8_t const *buf, int len, la_msg_dir msg_dir);
void la_cpdlc_format_text(la_vstring *vstr, void const *data, int indent);
void la_cpdlc_format_json(la_vstring *vstr, void const *data);
void la_cpdlc_destroy(void *data);
la_proto_node *la_proto_tree_find_cpdlc(la_proto_node *root);

#ifdef __cplusplus
}
#endif

#endif // !LA_CPDLC_H
