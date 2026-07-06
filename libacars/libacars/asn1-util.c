/*
 *  This file is a part of libacars
 *
 *  Copyright (c) 2018-2023 Tomasz Lemiech <szpajder@gmail.com>
 */

#include <stdint.h>
#include <search.h>                         // lfind()
#include <libacars/asn1/asn_application.h>  // asn_TYPE_descriptor_t
#include <libacars/asn1-util.h>             // la_asn1_formatter
#include <libacars/macros.h>                // LA_ISPRINTF, la_debug_print
#include <libacars/vstring.h>               // la_vstring
#include "config.h"                         // LFIND_NMEMB_SIZE_SIZE_T, LFIND_NMEMB_SIZE_UINT

static int la_compare_fmtr(void const *k, void const *m) {
	la_asn1_formatter const *memb = m;
	return(k == memb->type ? 0 : 1);
}

int la_asn1_decode_as(asn_TYPE_descriptor_t *td, void **struct_ptr, uint8_t const *buf, int size,
		asn_dec_rval_t *rval_out) {
	asn_dec_rval_t rval;
	// Passing NULL here (as the pre-best-effort code did) makes
	// uper_decode() fall back to ASN__DEFAULT_STACK_MAX (30000 bytes,
	// asn_internal.h) as the recursion-depth guard
	// (ASN__STACK_OVERFLOW_CHECK). Under ASan (HOST_TESTS_ASAN, needed
	// for the best-effort bit-flip fuzz -- design
	// docs/superpowers/plans/2026-07-07-libacars-best-effort-decode.md
	// §6b), every stack frame in the recursive constr_SEQUENCE/
	// constr_CHOICE decoders is inflated by redzone instrumentation, and
	// that budget is exhausted by ordinary, VALID, intact CPDLC messages
	// (observed: SOUCAYA/MSTEC7X/MELCAYA all silently RC_FAIL under
	// ASan, RC_OK without it -- no sanitizer report, just the guard
	// firing early). Disable the check here (max_stack_size = 0, "0
	// disables stack bounds checking" per asn_codecs.h) rather than
	// raise it to an arbitrary larger number: this ASN.1 module's
	// grammar (single FANS CPDLC/ADS-C schema) has small, SCHEMA-bounded
	// nesting and repetition limits (eg. the uplink/downlink msg element
	// SEQUENCE OF caps at 4 elements) -- there is no attacker-controlled
	// unbounded-recursion vector here for the guard to defend against,
	// unlike a generic ASN.1 decoder accepting arbitrary schemas. This
	// path is host-only today (common/libacars_idf stubs it out on
	// device -- device enablement is a later phase); revisit this
	// decision against the embedded target's real stack budget when
	// that phase wires it up.
	asn_codec_ctx_t codec_ctx = { .max_stack_size = 0 };
	rval = uper_decode_complete(&codec_ctx, td, struct_ptr, buf, size);
	if(rval_out != NULL) {
		*rval_out = rval;
	}
	if(rval.code != RC_OK) {
		la_debug_print(D_ERROR, "uper_decode_complete failed: %d\n", rval.code);
		return -1;
	}
	if(rval.consumed < (size_t)size) {
		la_debug_print(D_ERROR, "uper_decode_complete left %zd unparsed octets\n", (size_t)size - rval.consumed);
		return (int)((size_t)size - rval.consumed);
	}
#ifdef DEBUG
	if(Debug >= D_VERBOSE) {
		asn_fprint(stderr, td, *struct_ptr, 1);
	}
#endif
	return 0;
}

void la_asn1_output(la_asn1_formatter_params p, la_asn1_formatter const *asn1_formatter_table,
		size_t asn1_formatter_table_len, bool dump_unknown_types) {
	if(p.td == NULL || p.sptr == NULL) return;
#if defined LFIND_NMEMB_SIZE_SIZE_T
	size_t table_len = asn1_formatter_table_len;
#elif defined LFIND_NMEMB_SIZE_UINT
	unsigned int table_len = (unsigned int)asn1_formatter_table_len;
#endif
	la_asn1_formatter *formatter = lfind(p.td, asn1_formatter_table, &table_len,
			sizeof(la_asn1_formatter), &la_compare_fmtr);
	if(formatter != NULL) {
		// NULL formatting routine is allowed - it means the type should be silently omitted
		if(formatter->format != NULL) {
			p.label = formatter->label;
			(*formatter->format)(p);
		}
	} else if(dump_unknown_types) {
		LA_ISPRINTF(p.vstr, p.indent, "-- Formatter for type %s not found, ASN.1 dump follows:\n", p.td->name);
		LA_ISPRINTF(p.vstr, p.indent, "%s", "");    // asn_sprintf does not indent the first line
		asn_sprintf(p.vstr, p.td, p.sptr, p.indent+1);
		LA_EOL(p.vstr);
		LA_ISPRINTF(p.vstr, p.indent, "%s", "-- ASN.1 dump end\n");
	}
}
