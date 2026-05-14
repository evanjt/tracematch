import type { GpsPoint, RouteSignature, RouteGroup, FrequentSection } from './types';
import type { WorkerResponse } from './sectionWorker';

// ---------------------------------------------------------------------------
// Off-main-thread analysis via Web Worker
// ---------------------------------------------------------------------------
//
// All WASM work runs inside the worker (see sectionWorker.ts). The worker
// loads its own WASM instance, so the main thread never needs a wasm-bindgen
// module loaded.

let _worker: Worker | null = null;
let _nextRequestId = 1;

type PendingAnalyse = {
  resolve: (result: { signatures: RouteSignature[]; groups: RouteGroup[]; sections: FrequentSection[] }) => void;
  reject: (err: Error) => void;
  onProgress?: (phase: string, current: number, total: number) => void;
};

const _pendingAnalyse = new Map<number, PendingAnalyse>();

function getWorker(): Worker {
  if (_worker) return _worker;
  _worker = new Worker(new URL('./sectionWorker.ts', import.meta.url), {
    type: 'module',
  });
  _worker.onmessage = (event: MessageEvent<WorkerResponse>) => {
    const msg = event.data;

    if (msg.type === 'analyse:progress') {
      const handler = _pendingAnalyse.get(msg.requestId);
      handler?.onProgress?.(msg.phase, msg.current, msg.total);
      return;
    }

    if (msg.type === 'analyse:ok') {
      const handler = _pendingAnalyse.get(msg.requestId);
      _pendingAnalyse.delete(msg.requestId);
      handler?.resolve({ signatures: msg.signatures, groups: msg.groups, sections: msg.sections });
      return;
    }

    if (msg.type === 'analyse:err') {
      const handler = _pendingAnalyse.get(msg.requestId);
      _pendingAnalyse.delete(msg.requestId);
      handler?.reject(new Error(msg.message));
      return;
    }
  };
  _worker.onerror = (event) => {
    const err = new Error(`Worker error: ${event.message}`);
    for (const handler of _pendingAnalyse.values()) handler.reject(err);
    _pendingAnalyse.clear();
    _worker = null;
  };
  return _worker;
}

export interface AnalysisTrace {
  id: string;
  points: GpsPoint[];
  sportType: string;
}

export function runAnalysisAsync(
  traces: AnalysisTrace[],
  sectionConfig: string,
  onProgress?: (phase: string, current: number, total: number) => void,
  detectionMode?: 'density' | 'flow',
): Promise<{ signatures: RouteSignature[]; groups: RouteGroup[]; sections: FrequentSection[] }> {
  return new Promise((resolve, reject) => {
    const worker = getWorker();
    const requestId = _nextRequestId++;
    _pendingAnalyse.set(requestId, { resolve, reject, onProgress });
    worker.postMessage({
      type: 'analyse',
      requestId,
      traces,
      sectionConfig,
      detectionMode,
    });
  });
}

export function terminateSectionWorker(): void {
  _worker?.terminate();
  _worker = null;
  const err = new Error('Worker terminated');
  for (const handler of _pendingAnalyse.values()) handler.reject(err);
  _pendingAnalyse.clear();
}
