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
  loadProgress = $state<{ current: number; total: number } | null>(null);

  async load() {
    this.loading = true;
    this.loadProgress = null;
    try {
      console.time('[tracematch] idb:keys');
      const allKeys = await idbKeys();
      const traceKeys = allKeys.filter(
        (k) => typeof k === 'string' && k.startsWith(TRACES_PREFIX)
      );
      console.timeEnd('[tracematch] idb:keys');

      const total = traceKeys.length;
      this.loadProgress = { current: 0, total };

      console.time('[tracematch] idb:load-traces');
      const loaded: StoredTrace[] = [];
      const batchSize = 50;
      for (let i = 0; i < traceKeys.length; i += batchSize) {
        const batch = traceKeys.slice(i, i + batchSize);
        const results = await Promise.all(batch.map((k) => idbGet(k)));
        for (const trace of results) {
          if (trace) loaded.push(trace);
        }
        this.loadProgress = { current: Math.min(i + batchSize, total), total };
      }
      console.timeEnd('[tracematch] idb:load-traces');
      console.log(`[tracematch] Loaded ${loaded.length} traces from IndexedDB`);

      loaded.sort((a, b) => b.addedAt - a.addedAt);
      this.traces = loaded;

      console.time('[tracematch] idb:load-analysis');
      const saved = await idbGet(ANALYSIS_KEY);
      if (saved) this.analysis = saved;
      console.timeEnd('[tracematch] idb:load-analysis');
    } finally {
      this.loading = false;
      this.loadProgress = null;
    }
  }

  async addTrace(trace: StoredTrace) {
    await idbSet(TRACES_PREFIX + trace.id, trace);
    this.traces = [trace, ...this.traces];
    this.analysis = null;
    await idbDel(ANALYSIS_KEY);
  }

  async addTraces(newTraces: StoredTrace[]) {
    await Promise.all(newTraces.map((t) => idbSet(TRACES_PREFIX + t.id, t)));
    this.traces = [...newTraces, ...this.traces];
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
