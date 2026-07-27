#!/usr/bin/env node
// Host-side sidecar: enrich ACARS captured by the P4 SDR with the decoded text
// payload, using @airframes/acars-decoder (the same content decoder that powers
// airframes.io / AcarsHub). This runs on the Mac ONLY — it is not part of the
// firmware build or the host-test CI. It reads the device's NDJSON (the field
// layout produced by scripts/pull_acars_logs.sh / the /messages export), decodes
// each message's `txt`, and re-emits the record with an added `decode` block.
//
// Usage:
//   node enrich.js [FILE.ndjson]        # FILE or stdin -> enriched NDJSON on stdout
//   cat foo.ndjson | node enrich.js
//   node enrich.js FILE.ndjson --pretty # human-readable table instead of NDJSON
//   node enrich.js FILE.ndjson --only-decoded
//
// A coverage summary (decoded / partial / none, by label) is always printed to
// stderr so it never pollutes the NDJSON on stdout.

import { createInterface } from 'node:readline';
import { createReadStream } from 'node:fs';
import { MessageDecoder } from '@airframes/acars-decoder';

const args = process.argv.slice(2);
const flags = new Set(args.filter((a) => a.startsWith('--')));
const files = args.filter((a) => !a.startsWith('--'));
const pretty = flags.has('--pretty');
const onlyDecoded = flags.has('--only-decoded');

const decoder = new MessageDecoder();

const input = files.length
  ? createReadStream(files[0], 'utf8')
  : process.stdin;

const rl = createInterface({ input, crlfDelay: Infinity });

// Coverage tally, keyed by label -> { none, partial, full }.
const cov = new Map();
let total = 0;
let withText = 0;

function tally(label, level) {
  const t = cov.get(label) ?? { none: 0, partial: 0, full: 0 };
  t[level] = (t[level] ?? 0) + 1;
  cov.set(label, t);
}

for await (const line of rl) {
  const trimmed = line.trim();
  if (!trimmed) continue;

  let rec;
  try {
    rec = JSON.parse(trimmed);
  } catch {
    process.stderr.write(`skip (bad JSON): ${trimmed.slice(0, 80)}\n`);
    continue;
  }
  total += 1;

  const label = rec.label ?? '';
  // Device NDJSON uses `txt`; accept `text`/`message` too for other sources.
  const text = rec.txt ?? rec.text ?? rec.message ?? '';
  if (text.trim()) withText += 1;

  let result;
  try {
    result = decoder.decode({ label, text, sublabel: rec.sublabel });
  } catch (e) {
    result = { decoded: false, decoder: { decodeLevel: 'none' }, error: String(e) };
  }

  const level = result?.decoder?.decodeLevel ?? 'none';
  tally(label, level);

  if (onlyDecoded && level === 'none') continue;

  if (pretty) {
    const tag = level === 'full' ? '✓' : level === 'partial' ? '~' : ' ';
    const who = rec.flight ?? rec.tail ?? '';
    const desc = result?.formatted?.description ?? '';
    process.stdout.write(`[${tag}] ${label.padEnd(3)} ${String(who).padEnd(8)} ${desc}\n`);
    if (result?.raw?.position) {
      const p = result.raw.position;
      process.stdout.write(`        pos ${p.latitude?.toFixed(4)}, ${p.longitude?.toFixed(4)}\n`);
    }
    for (const it of result?.formatted?.items ?? []) {
      process.stdout.write(`        ${it.label}: ${it.value}\n`);
    }
    if (text.trim() && level !== 'none') {
      // Show the raw payload for context.
      process.stdout.write(`        raw: ${text}\n`);
    }
  } else {
    rec.decode = {
      level,
      decoder: result?.decoder?.name,
      description: result?.formatted?.description,
      raw: result?.raw,
      items: result?.formatted?.items,
    };
    process.stdout.write(JSON.stringify(rec) + '\n');
  }
}

// Summary to stderr.
let full = 0;
let partial = 0;
let none = 0;
const rows = [];
for (const [label, t] of [...cov.entries()].sort()) {
  full += t.full;
  partial += t.partial;
  none += t.none;
  rows.push(
    `  ${label.padEnd(4)} full=${t.full} partial=${t.partial} none=${t.none}`,
  );
}
process.stderr.write(
  `\n=== acars-enrich coverage ===\n` +
    `records=${total} with_text=${withText}\n` +
    `decoded: full=${full} partial=${partial} none=${none}` +
    (total ? ` (${((100 * (full + partial)) / total).toFixed(0)}% any)\n` : '\n') +
    rows.join('\n') +
    '\n',
);
