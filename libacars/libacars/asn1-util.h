/*
 *  This file is a part of libacars
 *
 *  Copyright (c) 2018-2023 Tomasz Lemiech <szpajder@gmail.com>
 */

#ifndef LA_ASN1_UTIL_H
#define LA_ASN1_UTIL_H 1
#include <stddef.h>                         // size_t
#include <stdint.h>                         // uint8_t
#include <libacars/asn1/asn_application.h>  // asn_TYPE_descriptor_t
#include <libacars/asn1/asn_codecs.h>       // asn_dec_rval_t
#include <libacars/vstring.h>               // la_vstring

// Parameters to the formatter function
typedef struct {
	la_vstring *vstr;
	char const *label;
	asn_TYPE_descriptor_t *td;
	void const *sptr;
	int indent;
} la_asn1_formatter_params;

// Formatter function prototype
typedef void (*la_asn1_formatter_func)(la_asn1_formatter_params);

typedef struct {
	asn_TYPE_descriptor_t *type;
	la_asn1_formatter_func format;
	char const *label;
} la_asn1_formatter;

#define LA_ASN1_FORMATTER_FUNC(x) \
	void x(la_asn1_formatter_params p)

// asn1-util.c
//
// rval_out (design docs/superpowers/plans/2026-07-07-libacars-best-effort-decode.md
// §4) is optional (NULL is fine, matching the pre-existing caller): when
// non-NULL, the raw asn_dec_rval_t from uper_decode_complete() is copied
// into *rval_out regardless of the return value, so a best-effort caller
// can distinguish RC_FAIL (desync) from RC_OK-with-trailing-bytes -- the
// return value alone (-1 vs a positive byte count) already implies this,
// but rval_out->code makes it explicit and gives access to .consumed
// even in the RC_FAIL case, now that per_decoder.c's consumed-on-fail fix
// makes it meaningful.
int la_asn1_decode_as(asn_TYPE_descriptor_t *td, void **struct_ptr, uint8_t const *buf, int size,
		asn_dec_rval_t *rval_out);
void la_asn1_output(la_asn1_formatter_params p, la_asn1_formatter const *asn1_formatter_table,
		size_t asn1_formatter_table_len, bool dump_unknown_types);
#endif // !LA_ASN1_UTIL_H
