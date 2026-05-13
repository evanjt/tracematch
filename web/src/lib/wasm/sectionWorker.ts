/**
 * Web Worker for off-main-thread analysis.
 *
 * Handles the entire analysis pipeline (signatures, grouping, section
 * detection) so the main thread stays responsive throughout. Each step
 * posts progress updates back to the main thread.
 *
 * The worker imports the wasm-bindgen JS glue directly from
 * `./pkg/tracematch_wasm.js`. `wasm-bindgen-rayon` needs `import.meta.url`
 * to resolve to a real path so it can spawn its rayon worker pool —
 * the previous blob-URL trick broke that. The pkg directory is
 * gitignored and populated at container start (see docker-compose.yml).
 */

import type { FrequentSection, GpsPoint, RouteGroup, RouteSignature } from './types';
// @ts-expect-error — generated at build time, may not exist when this file is type-checked in CI
import init, {
  createSignature as wasmCreateSignature,
  groupRoutes as wasmGroupRoutes,
  detectSections as wasmDetectSections,
  // initThreadPool is only present when the parallel feature is enabled.
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  initThreadPool,
} from './pkg/tracematch_wasm.js';

let wasmReady: Promise<void> | null = null;

async function ensureWasm(): Promise<void> {
  if (wasmReady) return wasmReady;
  wasmReady = (async () => {
    await init();
    // Spin up the rayon thread pool if available. Falls back silently
    // when the WASM was built without the `parallel` feature (or when
    // SharedArrayBuffer is unavailable because COOP/COEP headers are
    // missing on the host).
    if (typeof initThreadPool === 'function') {
      try {
        const threads = Math.max(1, (self as unknown as { navigator?: { hardwareConcurrency?: number } }).navigator?.hardwareConcurrency ?? 4);
        await initThreadPool(threads);
        console.log(`[worker] rayon thread pool initialised with ${threads} threads`);
      } catch (err) {
        console.warn('[worker] initThreadPool failed — falling back to single-threaded WASM', err);
      }
    } else {
      console.log('[worker] WASM built without parallel feature — single-threaded execution');
    }
  })();
  return wasmReady;
}

// --- Message types ---

export type WorkerRequest = AnalyseRequest;

export type AnalyseRequest = {
  type: 'analyse';
  requestId: number;
  traces: { id: string; points: GpsPoint[]; sportType: string }[];
  sectionConfig: string;
};

export type WorkerResponse =
  | { type: 'analyse:progress'; requestId: number; phase: string; current: number; total: number }
  | {
      type: 'analyse:ok';
      requestId: number;
      signatures: RouteSignature[];
      groups: RouteGroup[];
      sections: FrequentSection[];
    }
  | { type: 'analyse:err'; requestId: number; message: string };

function progress(requestId: number, phase: string, current: number, total: number) {
  self.postMessage({ type: 'analyse:progress', requestId, phase, current, total } satisfies WorkerResponse);
}

async function handleAnalyse(req: AnalyseRequest) {
  try {
    console.time('[worker] wasm-init');
    await ensureWasm();
    console.timeEnd('[worker] wasm-init');
    const traces = req.traces;

    // Phase 1: Create signatures
    console.time('[worker] signatures');
    const signatures: RouteSignature[] = [];
    const tracks: [string, GpsPoint[]][] = [];
    const sportTypes: Record<string, string> = {};
    const batchSize = 50;

    for (let i = 0; i < traces.length; i += batchSize) {
      const end = Math.min(i + batchSize, traces.length);
      for (let j = i; j < end; j++) {
        const t = traces[j];
        const sig = wasmCreateSignature(t.id, JSON.stringify(t.points), '{}');
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

    // Phase 2: Group routes
    console.time('[worker] grouping');
    progress(req.requestId, 'Grouping routes', 0, 1);
    console.time('[worker] grouping:serialize');
    const sigJson = JSON.stringify(signatures);
    console.timeEnd('[worker] grouping:serialize');
    console.log(`[worker] signatures JSON: ${(sigJson.length / 1024 / 1024).toFixed(1)} MB`);
    console.time('[worker] grouping:wasm');
    const groups = wasmGroupRoutes(sigJson, '{}');
    console.timeEnd('[worker] grouping:wasm');
    console.timeEnd('[worker] grouping');
    console.log(`[worker] ${groups.length} groups`);
    progress(req.requestId, 'Grouping routes', 1, 1);

    // Phase 3: Detect sections
    let sections: FrequentSection[] = [];
    if (tracks.length >= 3) {
      console.time('[worker] sections');
      progress(req.requestId, 'Detecting sections', 0, 1);
      console.time('[worker] sections:serialize');
      const tracksJson = JSON.stringify(tracks);
      const sportTypesJson = JSON.stringify(sportTypes);
      const groupsJson = JSON.stringify(groups);
      console.timeEnd('[worker] sections:serialize');
      console.log(`[worker] tracks JSON: ${(tracksJson.length / 1024 / 1024).toFixed(1)} MB`);
      console.time('[worker] sections:wasm');
      sections = wasmDetectSections(tracksJson, sportTypesJson, groupsJson, req.sectionConfig);
      console.timeEnd('[worker] sections:wasm');
      console.timeEnd('[worker] sections');
      console.log(`[worker] ${sections.length} sections detected`);
      progress(req.requestId, 'Detecting sections', 1, 1);
    }

    self.postMessage({
      type: 'analyse:ok',
      requestId: req.requestId,
      signatures,
      groups,
      sections,
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
