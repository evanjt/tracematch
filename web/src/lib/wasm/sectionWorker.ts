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

import type { FrequentSection, GpsPoint, RouteGroup, RouteSignature } from './types';
import init, { createSignature, groupRoutes, detectSections } from './pkg/tracematch_wasm.js';

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
    await ensureWasm();
    const traces = req.traces;

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

    progress(req.requestId, 'Grouping routes', 0, 1);
    const groups = groupRoutes(JSON.stringify(signatures), '{}');
    progress(req.requestId, 'Grouping routes', 1, 1);

    let sections: FrequentSection[] = [];
    if (tracks.length >= 3) {
      progress(req.requestId, 'Detecting sections', 0, 1);
      sections = detectSections(
        JSON.stringify(tracks),
        JSON.stringify(sportTypes),
        JSON.stringify(groups),
        req.sectionConfig,
      );
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
