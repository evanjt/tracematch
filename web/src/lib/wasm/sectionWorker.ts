/**
 * Web Worker for off-main-thread analysis.
 *
 * Handles the entire analysis pipeline (signatures, grouping, section
 * detection) so the main thread stays responsive throughout. Each step
 * posts progress updates back to the main thread.
 *
 * The worker imports the wasm-bindgen JS glue directly from
 * `./pkg/tracematch_wasm.js`. The pkg directory is gitignored and
 * populated at container start (see docker-compose.yml).
 */

import type {
  BoundaryRecord,
  FrequentSection,
  GpsPoint,
  RouteGroup,
  RouteSignature,
} from './types';
import init, {
  createSignature,
  groupRoutesWithProgress,
  detectSectionsUnified,
} from './pkg/tracematch_wasm.js';

// Map raw phase strings from the Rust library to human-readable labels
// for the progress overlay. Falls back to the raw string when unknown.
//
// Grouping is the only phase that reports progress. The unified
// detector's batch entry point emits nothing, so detection is
// indeterminate rather than counted.
const PHASE_LABELS: Record<string, string> = {
  comparing_pairs: 'Comparing route pairs',
};

function labelFor(phase: string): string {
  return PHASE_LABELS[phase] ?? phase;
}

let wasmReady: Promise<void> | null = null;

async function ensureWasm(): Promise<void> {
  if (wasmReady) return wasmReady;
  wasmReady = init().then(() => undefined);
  return wasmReady;
}

export type WorkerRequest = AnalyseRequest;

export type AnalyseRequest = {
  type: 'analyse';
  requestId: number;
  traces: { id: string; points: GpsPoint[]; sportType: string }[];
  sectionConfig: string;
  tunables: string;
};

export type WorkerResponse =
  | { type: 'analyse:progress'; requestId: number; phase: string; current: number; total: number }
  | {
      type: 'analyse:ok';
      requestId: number;
      signatures: RouteSignature[];
      groups: RouteGroup[];
      sections: FrequentSection[];
      boundaries: BoundaryRecord[];
    }
  | { type: 'analyse:err'; requestId: number; message: string };

function progress(requestId: number, phase: string, current: number, total: number) {
  self.postMessage({ type: 'analyse:progress', requestId, phase, current, total } satisfies WorkerResponse);
}

async function handleAnalyse(req: AnalyseRequest) {
  try {
    console.time('[worker] wasm-init');
    progress(req.requestId, 'Initializing WASM engine', 0, 0);
    await ensureWasm();
    console.timeEnd('[worker] wasm-init');
    const traces = req.traces;

    console.time('[worker] signatures');
    const signatures: RouteSignature[] = [];
    const tracks: [string, GpsPoint[]][] = [];
    const sportTypes: Record<string, string> = {};
    const batchSize = 50;

    for (let i = 0; i < traces.length; i += batchSize) {
      const end = Math.min(i + batchSize, traces.length);
      for (let j = i; j < end; j++) {
        const t = traces[j];
        const sig = createSignature(t.id, JSON.stringify(t.points), '{}');
        if (sig) {
          signatures.push(sig);
          tracks.push([t.id, t.points]);
          sportTypes[t.id] = t.sportType ?? 'Ride';
        }
      }
      progress(req.requestId, 'Creating signatures', end, traces.length);
    }
    console.timeEnd('[worker] signatures');
    console.log(`[worker] ${signatures.length} signatures from ${traces.length} traces`);

    // Emit a transition status BEFORE the (potentially multi-second)
    // JSON.stringify so the progress bar doesn't sit at 100% of the
    // signatures phase while we prepare data for the next WASM call.
    progress(req.requestId, 'Serializing routes for grouping', 0, 0);
    console.time('[worker] grouping');
    const sigJson = JSON.stringify(signatures);
    console.log(`[worker] signatures JSON: ${(sigJson.length / 1024 / 1024).toFixed(1)} MB`);
    progress(req.requestId, 'Preparing route comparison', 0, 0);
    const groups = groupRoutesWithProgress(sigJson, '{}', (phase: string, current: number, total: number) => {
      progress(req.requestId, labelFor(phase), current, total);
    });
    console.timeEnd('[worker] grouping');
    console.log(`[worker] ${groups.length} groups`);

    let sections: FrequentSection[] = [];
    let boundaries: BoundaryRecord[] = [];
    if (tracks.length >= 3) {
      progress(req.requestId, 'Serializing tracks for section detection', 0, 0);
      console.time('[worker] sections');
      const tracksJson = JSON.stringify(tracks);
      const sportTypesJson = JSON.stringify(sportTypes);
      console.log(`[worker] tracks JSON: ${(tracksJson.length / 1024 / 1024).toFixed(1)} MB`);

      // The unified detector reports no progress, so this sits
      // indeterminate until it returns. Seconds are empty: the GPX
      // parser reads no <time>, and the lift veto falls back to
      // geometry when the stream is absent.
      progress(req.requestId, 'Detecting sections', 0, 0);
      const detection = detectSectionsUnified(
        tracksJson,
        '[]',
        sportTypesJson,
        req.sectionConfig,
        req.tunables,
      );
      sections = detection.sections;
      boundaries = detection.boundaries;
      console.timeEnd('[worker] sections');
      console.log(`[worker] ${sections.length} sections, ${boundaries.length} boundaries`);
    }

    progress(req.requestId, 'Finalizing results', 0, 0);
    self.postMessage({
      type: 'analyse:ok',
      requestId: req.requestId,
      signatures,
      groups,
      sections,
      boundaries,
    } satisfies WorkerResponse);
  } catch (err) {
    const msg = err instanceof Error ? err.message : String(err);
    self.postMessage({ type: 'analyse:err', requestId: req.requestId, message: msg });
  }
}

self.onmessage = async (event: MessageEvent<WorkerRequest>) => {
  const req = event.data;
  if (req?.type === 'analyse') {
    await handleAnalyse(req);
  }
};
