import type { GpsPoint, RouteSignature, MatchResult, RouteGroup, FrequentSection, SectionMatch } from './types';

// eslint-disable-next-line @typescript-eslint/no-explicit-any
let wasm: any = null;

export async function initWasm() {
  if (wasm) return;

  // Fetch the JS glue as text, then import via blob URL to bypass Vite's
  // restriction on importing JS from the public directory in dev mode.
  const jsResponse = await fetch('/wasm/tracematch_wasm.js');
  const jsText = await jsResponse.text();
  const blob = new Blob([jsText], { type: 'application/javascript' });
  const blobUrl = URL.createObjectURL(blob);
  const mod = await import(/* @vite-ignore */ blobUrl);
  URL.revokeObjectURL(blobUrl);

  await mod.default('/wasm/tracematch_wasm_bg.wasm');
  wasm = mod;
}

function w() {
  if (!wasm) throw new Error('WASM not initialized — call initWasm() first');
  return wasm;
}

export function createSignature(
  activityId: string,
  points: GpsPoint[],
  config = '{}'
): RouteSignature | null {
  return w().createSignature(activityId, JSON.stringify(points), config) as RouteSignature | null;
}

export function compareRoutes(
  sig1: RouteSignature,
  sig2: RouteSignature,
  config = '{}'
): MatchResult | null {
  return w().compareRoutes(JSON.stringify(sig1), JSON.stringify(sig2), config) as MatchResult | null;
}

export function groupRoutes(
  signatures: RouteSignature[],
  config = '{}'
): RouteGroup[] {
  return w().groupRoutes(JSON.stringify(signatures), config) as RouteGroup[];
}

export function detectSections(
  tracks: [string, GpsPoint[]][],
  sportTypes: Record<string, string>,
  groups: RouteGroup[],
  config = '{}'
): FrequentSection[] {
  return w().detectSections(
    JSON.stringify(tracks),
    JSON.stringify(sportTypes),
    JSON.stringify(groups),
    config
  ) as FrequentSection[];
}

export function findSectionsInRoute(
  route: GpsPoint[],
  sections: FrequentSection[],
  config = '{}'
): SectionMatch[] {
  return w().findSectionsInRoute(
    JSON.stringify(route),
    JSON.stringify(sections),
    config
  ) as SectionMatch[];
}

// ---------------------------------------------------------------------------
// Off-main-thread detection via Web Worker
// ---------------------------------------------------------------------------
//
// `detectSections` above runs synchronously and blocks the main thread for
// the entire detection duration — at 400 tracks that's seconds to minutes
// depending on geographic spread. `detectSectionsAsync` offloads the call
// to a worker so the UI stays responsive.
//
// The worker re-loads WASM on first call; subsequent calls reuse the
// existing module. There's intentionally no shared global state between
// the worker and the main thread, so every invocation passes the full
// input (tracks/groups/sportTypes) over postMessage. For ~400 tracks
// that's a few MB — fast enough that the structured-clone copy is
// negligible compared to the detection itself.

let _worker: Worker | null = null;
let _nextRequestId = 1;
const _pending = new Map<
  number,
  { resolve: (sections: FrequentSection[]) => void; reject: (err: Error) => void }
>();

function getWorker(): Worker {
  if (_worker) return _worker;
  _worker = new Worker(new URL('./sectionWorker.ts', import.meta.url), {
    type: 'module',
  });
  _worker.onmessage = (event: MessageEvent) => {
    const msg = event.data as
      | { type: 'detect:ok'; requestId: number; sections: FrequentSection[] }
      | { type: 'detect:err'; requestId: number; message: string };
    const handler = _pending.get(msg.requestId);
    if (!handler) return;
    _pending.delete(msg.requestId);
    if (msg.type === 'detect:ok') {
      handler.resolve(msg.sections);
    } else {
      handler.reject(new Error(msg.message));
    }
  };
  _worker.onerror = (event) => {
    // Reject all pending requests if the worker dies — leaving them
    // hanging would freeze the UI even though the worker existed
    // specifically to keep the UI responsive.
    const err = new Error(`Section worker error: ${event.message}`);
    for (const handler of _pending.values()) {
      handler.reject(err);
    }
    _pending.clear();
    _worker = null;
  };
  return _worker;
}

export function detectSectionsAsync(
  tracks: [string, GpsPoint[]][],
  sportTypes: Record<string, string>,
  groups: RouteGroup[],
  config = '{}',
): Promise<FrequentSection[]> {
  return new Promise((resolve, reject) => {
    const worker = getWorker();
    const requestId = _nextRequestId++;
    _pending.set(requestId, { resolve, reject });
    worker.postMessage({
      type: 'detect',
      requestId,
      tracks,
      sportTypes,
      groups,
      config,
    });
  });
}

/** Tear down the worker — useful for tests or on unmount. */
export function terminateSectionWorker(): void {
  _worker?.terminate();
  _worker = null;
  // Reject any in-flight requests so promises don't hang.
  const err = new Error('Section worker terminated');
  for (const handler of _pending.values()) {
    handler.reject(err);
  }
  _pending.clear();
}
