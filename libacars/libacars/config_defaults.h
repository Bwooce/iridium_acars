/*
 *  This file is a part of libacars
 *
 *  Copyright (c) 2018-2023 Tomasz Lemiech <szpajder@gmail.com>
 */

/* Default libacars configuration settings */

{
// Output raw dump of ASN.1 structure for all ASN.1-encoded messages?

	LA_CONFIG_SETTING_BOOLEAN("dump_asn1", false),

// Try to decode ACARS applications in fragmented messages, when
// reassembly is in progress?
// If reassembly is disabled, this setting has no effect (ie. applications
// are always decoded).
//
	LA_CONFIG_SETTING_BOOLEAN("decode_fragments", false),

// Radio link type (bearer) used to transmit ACARS messages processed by
// libacars. Must be set correctly when message reassembly is used (it
// determines timer values for the reassembly process). Default is 1 (VHF).
// See acars.h for full list of supported values.

	LA_CONFIG_SETTING_INTEGER("acars_bearer", 1),

// Pretty-print XML in ACARS and MIAM Core payloads?

	LA_CONFIG_SETTING_BOOLEAN("prettify_xml", false),

// Pretty-print JSON?

	LA_CONFIG_SETTING_BOOLEAN("prettify_json", false),

// Render display-only, hard-tagged PARTIAL output for CPDLC (ASN.1 uPER)
// and ADS-C messages that fail to decode completely, instead of
// discarding everything? uPER fails silently -- post-desync fields
// decode as plausible-looking garbage -- so this must never feed
// automation; see docs/superpowers/plans/2026-07-07-libacars-best-effort-decode.md
// for the safety contract. Default OFF preserves today's behaviour
// exactly.

	LA_CONFIG_SETTING_BOOLEAN("best_effort_decode", false)
};
