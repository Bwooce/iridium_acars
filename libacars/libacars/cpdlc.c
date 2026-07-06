/*
 *  This file is a part of libacars
 *
 *  Copyright (c) 2018-2023 Tomasz Lemiech <szpajder@gmail.com>
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <libacars/asn1/FANSATCDownlinkMessage.h>   // asn_DEF_FANSATCDownlinkMessage
#include <libacars/asn1/FANSATCUplinkMessage.h>     // asn_DEF_FANSATCUplinkMessage
#include <libacars/asn1/asn_application.h>          // asn_sprintf()
#include <libacars/macros.h>                        // la_assert
#include <libacars/asn1-util.h>                     // la_asn1_decode_as()
#include <libacars/asn1-format-cpdlc.h>             // la_asn1_output_cpdlc_as_*()
#include <libacars/cpdlc.h>                         // la_cpdlc_msg
#include <libacars/libacars.h>                      // la_proto_node, la_config_get_bool, la_proto_tree_find_protocol
#include <libacars/macros.h>                        // la_debug_print
#include <libacars/util.h>                          // LA_XFREE
#include <libacars/vstring.h>                       // la_vstring, la_vstring_append_sprintf()
#include <libacars/json.h>                          // la_json_append_bool()

la_proto_node *la_cpdlc_parse(uint8_t const *buf, int len, la_msg_dir msg_dir) {
	if(buf == NULL)
		return NULL;

	la_proto_node *node = la_proto_node_new();
	LA_NEW(la_cpdlc_msg, msg);
	node->data = msg;
	node->td = &la_DEF_cpdlc_message;

	if(msg_dir == LA_MSG_DIR_GND2AIR) {
		msg->asn_type = &asn_DEF_FANSATCUplinkMessage;
	} else if(msg_dir == LA_MSG_DIR_AIR2GND) {
		msg->asn_type = &asn_DEF_FANSATCDownlinkMessage;
	}
	la_assert(msg->asn_type != NULL);
	if(len == 0) {
		// empty payload is not an error
		la_debug_print(D_INFO, "Empty CPDLC message, decoding skipped\n");
		return node;
	}

	la_debug_print(D_INFO, "Decoding as %s, len: %d\n", msg->asn_type->name, len);
	msg->total_bits = (size_t)len * 8;
	asn_dec_rval_t rval;
	int ret = la_asn1_decode_as(msg->asn_type, &msg->data, buf, len, &rval);
	if(ret == 0) {
		// Clean, fully-consumed success -- same as always.
		msg->err = false;
		return node;
	}

	bool best_effort_decode = false;
	(void)la_config_get_bool("best_effort_decode", &best_effort_decode);
	// design §4: OFF -> exactly current behaviour (err on any nonzero
	// return, whether RC_FAIL or RC_OK-with-trailing-bytes). ON and we
	// got at least one successfully-decoded bit -> keep the tree and
	// tag it PARTIAL instead of discarding it. consumed == 0 means the
	// decoder never got anywhere (eg. empty/garbage input) -- err as
	// today regardless of the flag.
	if(best_effort_decode && rval.consumed > 0) {
		msg->err = false;
		msg->partial = true;
		// rval.code == RC_OK here means uper_decode_complete() built a
		// complete, valid structure and simply didn't consume the whole
		// buffer (trailing junk) -- NOT the same thing as a mid-message
		// desync (RC_FAIL). Conflating the two is the policy trap design
		// §4 calls out: trailing-junk trust is "whole message is good,
		// ignore the tail"; desync trust is "only the prefix is good,
		// everything after is uPER's silent post-corruption garbage".
		msg->trailing_junk = (rval.code == RC_OK);
		// asn_dec_rval_t.consumed is byte-granular (per_decoder.c rounds
		// pd.moved's bit-exact value up to a byte count -- see
		// patches/README.md #0004); express it in bits for the banner,
		// documented as such rather than claimed bit-exact.
		msg->consumed_bits = (size_t)rval.consumed * 8;
	} else {
		msg->err = true;
	}
	return node;
}

void la_cpdlc_format_text(la_vstring *vstr, void const *data, int indent) {
	la_assert(vstr);
	la_assert(data);
	la_assert(indent >= 0);

	la_cpdlc_msg const *msg = data;
	if(msg->err == true) {
		LA_ISPRINTF(vstr, indent, "-- Unparseable FANS-1/A message\n");
		return;
	}
	if(msg->partial == true) {
		// design §4: distinct wording for the two partial-success cases --
		// do not conflate a benign trailing-bytes tail with a mid-message
		// desync (see cpdlc.h for the full trust-level explanation).
		if(msg->trailing_junk) {
			LA_ISPRINTF(vstr, indent,
					"-- NOTE: message decoded OK, %zu trailing bit(s) beyond bit %zu of %zu were ignored -- display only\n",
					msg->total_bits - msg->consumed_bits, msg->consumed_bits, msg->total_bits);
		} else {
			LA_ISPRINTF(vstr, indent,
					"-- WARNING: PARTIAL/UNTRUSTED decode (desync after bit %zu of %zu) -- display only\n",
					msg->consumed_bits, msg->total_bits);
		}
	}
	if(msg->asn_type != NULL) {
		if(msg->data != NULL) {
			bool dump_asn1 = false;
			(void)la_config_get_bool("dump_asn1", &dump_asn1);
			if(dump_asn1 == true) {
				LA_ISPRINTF(vstr, indent, "ASN.1 dump:\n");
				// asn_fprint does not indent the first line
				LA_ISPRINTF(vstr, indent + 1, "");
				asn_sprintf(vstr, msg->asn_type, msg->data, indent + 2);
				LA_EOL(vstr);
			}
			la_asn1_output_cpdlc_as_text((la_asn1_formatter_params){
					.vstr = vstr,
					.td = msg->asn_type,
					.sptr = msg->data,
					.indent = indent
					});
		} else {
			LA_ISPRINTF(vstr, indent, "-- <empty PDU>\n");
		}
	}
}

void la_cpdlc_format_json(la_vstring *vstr, void const *data) {
	la_assert(vstr);
	la_assert(data);

	la_cpdlc_msg const *msg = data;
	la_json_append_bool(vstr, "err", msg->err);
	if(msg->err == true) {
		return;
	}
	// design §4: "partial"/"consumed_bits" sit next to "err". trailing_junk
	// is exposed too -- a consumer must not treat a trailing-bytes partial
	// the same as a desync partial (see cpdlc.h).
	// NOTE for the upstream PR: the "partial" key is emitted
	// unconditionally (always false when the flag is OFF), so flag-OFF
	// JSON output gains one constant key vs upstream 2.2.1. Emitting it
	// only when true would keep flag-OFF output identical -- upstream's
	// call whether schema stability or output stability matters more.
	la_json_append_bool(vstr, "partial", msg->partial);
	if(msg->partial == true) {
		la_json_append_bool(vstr, "trailing_junk", msg->trailing_junk);
		la_json_append_int64(vstr, "consumed_bits", (int64_t)msg->consumed_bits);
		la_json_append_int64(vstr, "total_bits", (int64_t)msg->total_bits);
	}
	if(msg->asn_type != NULL) {
		if(msg->data != NULL) {
			la_asn1_output_cpdlc_as_json((la_asn1_formatter_params){
					.vstr = vstr,
					.td = msg->asn_type,
					.sptr = msg->data,
					});
		}
	}
}

void la_cpdlc_destroy(void *data) {
	if(data == NULL) {
		return;
	}
	la_cpdlc_msg *msg = data;
	if(msg->asn_type != NULL) {
		msg->asn_type->free_struct(msg->asn_type, msg->data, 0);
	}
	LA_XFREE(data);
}

la_type_descriptor const la_DEF_cpdlc_message = {
	.format_text = la_cpdlc_format_text,
	.format_json = la_cpdlc_format_json,
	.json_key = "cpdlc",
	.destroy = la_cpdlc_destroy
};

la_proto_node *la_proto_tree_find_cpdlc(la_proto_node *root) {
	return la_proto_tree_find_protocol(root, &la_DEF_cpdlc_message);
}
