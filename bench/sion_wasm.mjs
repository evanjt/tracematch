// Run the Sion corpus end-to-end through the WASM module.
//
// Prerequisites:
//   cd tracematch-wasm && wasm-pack build --target nodejs --release --out-dir pkg-node
//   node bench/sion_wasm.mjs
//
// Compares apples-to-apples with `cargo run --release --example sion_corpus_report`
// — same data, same configs, same operations, but executing the WASM module
// instead of the native Rust library. Use this to see the WASM-vs-native gap
// without browser overhead (no message passing, no JSON re-serialisation
// across worker boundaries, no JS event loop scheduling).

import { readdir, readFile } from 'node:fs/promises';
import { performance } from 'node:perf_hooks';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import { createRequire } from 'node:module';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const require = createRequire(import.meta.url);

const pkgPath = path.join(__dirname, '..', 'tracematch-wasm', 'pkg-node');
let wasm;
try {
  wasm = require(path.join(pkgPath, 'tracematch_wasm.js'));
} catch (e) {
  console.error(`Failed to load WASM from ${pkgPath}`);
  console.error('Build it first with:');
  console.error('  cd tracematch-wasm && wasm-pack build --target nodejs --release --out-dir pkg-node');
  process.exit(1);
}

const SION_DIR = path.join(__dirname, '..', 'sionrunning');

function parseGpx(content) {
  const points = [];
  for (const line of content.split('\n')) {
    if (!line.includes('<trkpt')) continue;
    const latMatch = line.match(/lat="([^"]+)"/);
    const lonMatch = line.match(/lon="([^"]+)"/);
    if (latMatch && lonMatch) {
      points.push({
        latitude: parseFloat(latMatch[1]),
        longitude: parseFloat(lonMatch[1]),
      });
    }
  }
  return points;
}

function fmtMs(ms) {
  return ms >= 1000 ? `${(ms / 1000).toFixed(2)} s` : `${Math.round(ms)} ms`;
}

console.log('============================================================');
console.log('  Sion corpus WASM report');
console.log('============================================================\n');

// --- 1. Load GPX -----------------------------------------------------
const tLoad = performance.now();
const files = (await readdir(SION_DIR)).filter((f) => f.endsWith('.gpx'));
const tracks = [];
let totalPoints = 0;
let skippedShort = 0;
for (const f of files) {
  const content = await readFile(path.join(SION_DIR, f), 'utf8');
  const points = parseGpx(content);
  if (points.length < 50) {
    skippedShort++;
    continue;
  }
  totalPoints += points.length;
  tracks.push({ id: path.basename(f, '.gpx'), points });
}
const dLoad = performance.now() - tLoad;
console.log(`## Load`);
console.log(`  Files loaded:        ${tracks.length}`);
console.log(`  Files skipped (<50): ${skippedShort}`);
console.log(`  Total GPS points:    ${totalPoints} (${Math.round(totalPoints / tracks.length)} avg/track)`);
console.log(`  Time:                ${fmtMs(dLoad)}\n`);

// --- 2. Signatures ---------------------------------------------------
const tSig = performance.now();
const signatures = [];
const sectionTracks = [];
const sportTypes = {};
for (const t of tracks) {
  const sig = wasm.createSignature(t.id, JSON.stringify(t.points), '{}');
  if (sig) {
    signatures.push(sig);
    sectionTracks.push([t.id, t.points]);
    sportTypes[t.id] = 'Run';
  }
}
const dSig = performance.now() - tSig;
console.log(`## Signatures`);
console.log(`  Signatures built:    ${signatures.length} (from ${tracks.length} tracks)`);
console.log(`  Time:                ${fmtMs(dSig)}\n`);

// --- 3. Route grouping ----------------------------------------------
const tGrp = performance.now();
const sigJson = JSON.stringify(signatures);
const groups = wasm.groupRoutes(sigJson, '{}');
const dGrp = performance.now() - tGrp;

const sizes = groups.map((g) => g.activityIds.length).sort((a, b) => b - a);
const singletons = sizes.filter((n) => n === 1).length;
const nonSingleton = sizes.filter((n) => n > 1);
const medianNonSingleton = nonSingleton.length
  ? nonSingleton.sort((a, b) => a - b)[Math.floor(nonSingleton.length / 2)]
  : 0;

console.log(`## Route grouping`);
console.log(`  Groups:              ${groups.length}`);
console.log(`  Singletons:          ${singletons} (${Math.round((100 * singletons) / Math.max(groups.length, 1))}%)`);
console.log(`  Multi-member groups: ${groups.length - singletons}`);
console.log(`  Largest group:       ${sizes[0] ?? 0} activities`);
console.log(`  Median multi-group:  ${medianNonSingleton} activities`);
console.log(`  Time:                ${fmtMs(dGrp)}\n`);

console.log('  Top 5 groups by size:');
for (const g of [...groups].sort((a, b) => b.activityIds.length - a.activityIds.length).slice(0, 5)) {
  console.log(`    ${String(g.activityIds.length).padStart(4)}  ${g.activityIds[0]}`);
}
console.log();

// --- 4. Sections -----------------------------------------------------
const sectionConfig = JSON.stringify({
  proximityThreshold: 50,
  minSectionLength: 200,
  maxSectionLength: 200_000,
  minActivities: 3,
  clusterTolerance: 80,
  samplePoints: 50,
  detectionMode: 'discovery',
  includePotentials: true,
  scalePresets: [
    { name: 'short', minLength: 100, maxLength: 500, minActivities: 2 },
    { name: 'medium', minLength: 500, maxLength: 2000, minActivities: 2 },
    { name: 'long', minLength: 2000, maxLength: 5000, minActivities: 3 },
    { name: 'extra_long', minLength: 5000, maxLength: 50000, minActivities: 3 },
  ],
  preserveHierarchy: false,
});

const tSec = performance.now();
const sections = wasm.detectSections(
  JSON.stringify(sectionTracks),
  JSON.stringify(sportTypes),
  JSON.stringify(groups),
  sectionConfig,
);
const dSec = performance.now() - tSec;

const sortedSections = [...sections].sort((a, b) => b.activityIds.length - a.activityIds.length);
const totalVisits = sections.reduce((sum, s) => sum + s.activityIds.length, 0);
const visitCounts = sections.map((s) => s.activityIds.length).sort((a, b) => a - b);
const medianVisits = visitCounts.length ? visitCounts[Math.floor(visitCounts.length / 2)] : 0;

console.log(`## Section detection`);
console.log(`  Sections:            ${sections.length}`);
console.log(`  Total visits:        ${totalVisits}`);
console.log(`  Median visits:       ${medianVisits}`);
console.log(`  Largest section:     ${sortedSections[0]?.activityIds.length ?? 0} visits, ${Math.round(sortedSections[0]?.distanceMeters ?? 0)} m`);
console.log(`  Time:                ${fmtMs(dSec)}\n`);

console.log('  Top 5 sections by visits:');
for (const s of sortedSections.slice(0, 5)) {
  console.log(`    ${String(s.activityIds.length).padStart(4)}  ${String(Math.round(s.distanceMeters)).padStart(6)} m  ${s.id}`);
}
console.log();

// --- 5. Totals -------------------------------------------------------
const dTotal = dLoad + dSig + dGrp + dSec;
console.log(`## Totals`);
console.log(`  Load + signatures:   ${fmtMs(dLoad + dSig)}`);
console.log(`  Route grouping:      ${fmtMs(dGrp)}`);
console.log(`  Section detection:   ${fmtMs(dSec)}`);
console.log(`  ─────────────────────────────`);
console.log(`  End-to-end:          ${fmtMs(dTotal)}`);
