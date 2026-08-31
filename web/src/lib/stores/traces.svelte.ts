import { get as idbGet, set as idbSet, del as idbDel, keys as idbKeys } from 'idb-keyval';
import { traceId } from '$lib/traceId';
import type {
  BoundaryRecord,
  FrequentSection,
  GpsPoint,
  RouteGroup,
  RouteSignature,
} from '$lib/wasm/types';

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
  boundaries: BoundaryRecord[];
  analyzedAt: number;
  schemaVersion: number;
}

// A cached catalogue carries no detector identity of its own, so a run
// from a removed detector is indistinguishable from a current one on the
// next visit. Stamp it, and discard anything that does not match.
export const ANALYSIS_SCHEMA_VERSION = 2;

// Traces stored before the id came from the points carry a random UUID, so
// the same file read twice sits in the store twice. Version 1 re-keys them
// onto their content id and folds whatever that reveals as a duplicate.
export const TRACE_SCHEMA_VERSION = 1;

const TRACES_PREFIX = 'trace:';
const ANALYSIS_KEY = 'analysis';
const TRACE_VERSION_KEY = 'traceSchemaVersion';

/// An id already derived from something stable: its own content, or the
/// archive path GeoLife addresses a trajectory by. Nothing to re-key.
const STABLE_ID_PREFIXES = ['gpx:', 'geolife:'];

function hasStableId(trace: StoredTrace): boolean {
  return STABLE_ID_PREFIXES.some((prefix) => trace.id.startsWith(prefix));
}

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
      let loaded: StoredTrace[] = [];
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

      if ((await idbGet(TRACE_VERSION_KEY)) !== TRACE_SCHEMA_VERSION) {
        loaded = await reidentify(loaded);
      }

      loaded.sort((a, b) => b.addedAt - a.addedAt);
      this.traces = loaded;

      console.time('[tracematch] idb:load-analysis');
      const saved = await idbGet(ANALYSIS_KEY);
      if (saved?.schemaVersion === ANALYSIS_SCHEMA_VERSION) {
        this.analysis = saved;
      } else if (saved) {
        // Produced by a detector that no longer exists.
        await idbDel(ANALYSIS_KEY);
      }
      console.timeEnd('[tracematch] idb:load-analysis');
    } finally {
      this.loading = false;
      this.loadProgress = null;
    }
  }

  async addTrace(trace: StoredTrace) {
    await this.addTraces([trace]);
  }

  // Ids come off the points, so a file read twice arrives under the key it
  // already holds. The incoming record replaces it rather than sitting
  // beside it: two copies of one ride read as two visits to the support
  // floors, which is enough to lift a section on one outing's ground.
  async addTraces(newTraces: StoredTrace[]) {
    const incoming = new Map<string, StoredTrace>();
    for (const trace of newTraces) incoming.set(trace.id, trace);
    const arriving = [...incoming.values()];

    await Promise.all(arriving.map((t) => idbSet(TRACES_PREFIX + t.id, t)));
    this.traces = [...arriving, ...this.traces.filter((t) => !incoming.has(t.id))];
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

/**
 * Take every stored trace onto an id derived from its own points.
 *
 * A duplicate the random ids let in is folded here, keeping the earliest
 * import so the result does not depend on the order IndexedDB hands the
 * keys back. Any catalogue cached over the old ids is dropped, since its
 * activity references no longer name anything in the store. Stamping the
 * version last means an interrupted run re-runs rather than half-applies.
 */
async function reidentify(loaded: StoredTrace[]): Promise<StoredTrace[]> {
  const byId = new Map<string, StoredTrace>();
  const oldKeys = new Set<string>();
  let moved = false;

  for (const trace of loaded) {
    const id = hasStableId(trace) ? trace.id : traceId(trace.points);
    oldKeys.add(TRACES_PREFIX + trace.id);
    if (id !== trace.id) moved = true;

    const held = byId.get(id);
    if (held && held.addedAt <= trace.addedAt) {
      moved = true;
      continue;
    }
    if (held) moved = true;
    byId.set(id, { ...trace, id });
  }

  const kept = [...byId.values()];
  await Promise.all(kept.map((t) => idbSet(TRACES_PREFIX + t.id, t)));
  const live = new Set(kept.map((t) => TRACES_PREFIX + t.id));
  await Promise.all([...oldKeys].filter((k) => !live.has(k)).map((k) => idbDel(k)));

  if (moved) await idbDel(ANALYSIS_KEY);
  await idbSet(TRACE_VERSION_KEY, TRACE_SCHEMA_VERSION);
  return kept;
}

export const traceStore = new TraceStore();
