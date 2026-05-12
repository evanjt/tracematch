import { get as idbGet, set as idbSet, del as idbDel, keys as idbKeys } from 'idb-keyval';
import type { GpsPoint, RouteSignature, RouteGroup, FrequentSection } from '$lib/wasm/types';

export interface StoredTrace {
  id: string;
  name: string;
  fileName: string;
  points: GpsPoint[];
  distance: number;
  sportType: string;
  addedAt: number;
}

export interface AnalysisResult {
  signatures: RouteSignature[];
  groups: RouteGroup[];
  sections: FrequentSection[];
  analyzedAt: number;
}

const TRACES_PREFIX = 'trace:';
const ANALYSIS_KEY = 'analysis';

class TraceStore {
  traces = $state<StoredTrace[]>([]);
  analysis = $state<AnalysisResult | null>(null);
  loading = $state(false);

  async load() {
    this.loading = true;
    try {
      const allKeys = await idbKeys();
      const traceKeys = allKeys.filter(
        (k) => typeof k === 'string' && k.startsWith(TRACES_PREFIX)
      );
      const loaded: StoredTrace[] = [];
      for (const key of traceKeys) {
        const trace = await idbGet(key);
        if (trace) loaded.push(trace);
      }
      loaded.sort((a, b) => b.addedAt - a.addedAt);
      this.traces = loaded;

      const saved = await idbGet(ANALYSIS_KEY);
      if (saved) this.analysis = saved;
    } finally {
      this.loading = false;
    }
  }

  async addTrace(trace: StoredTrace) {
    await idbSet(TRACES_PREFIX + trace.id, trace);
    this.traces = [trace, ...this.traces];
    this.analysis = null;
    await idbDel(ANALYSIS_KEY);
  }

  async removeTrace(id: string) {
    await idbDel(TRACES_PREFIX + id);
    this.traces = this.traces.filter((t) => t.id !== id);
    this.analysis = null;
    await idbDel(ANALYSIS_KEY);
  }

  async clearAll() {
    const allKeys = await idbKeys();
    for (const key of allKeys) {
      if (typeof key === 'string' && key.startsWith(TRACES_PREFIX)) {
        await idbDel(key);
      }
    }
    await idbDel(ANALYSIS_KEY);
    this.traces = [];
    this.analysis = null;
  }

  async saveAnalysis(result: AnalysisResult) {
    this.analysis = result;
    await idbSet(ANALYSIS_KEY, result);
  }
}

export const traceStore = new TraceStore();
