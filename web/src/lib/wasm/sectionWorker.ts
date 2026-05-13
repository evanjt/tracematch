/**
 * Web Worker for off-main-thread section detection.
 *
 * The WASM detectSections call can take seconds to minutes on large
 * datasets and blocks the main thread completely while running. Moving
 * it into a dedicated worker keeps the UI responsive (panning the map,
 * clicking around, scrolling) during long detection runs.
 *
 * The worker re-initialises WASM independently of the main thread —
 * each worker has its own WebAssembly.Module instance. This is fine for
 * the synchronous, fire-and-forget detection use case but means we
 * can't share state with the main thread; each detection request must
 * include the full input data.
 *
 * Note on parallelism: this worker is still single-threaded internally.
 * Enabling rayon inside the WASM module would require
 * `wasm-bindgen-rayon` + SharedArrayBuffer + COOP/COEP headers, which
 * is a substantial separate change. The Phase 3 hierarchical grid
 * pre-filter is the bigger win for typical user data and works without
 * any threading changes.
 */

import type { FrequentSection, GpsPoint, RouteGroup } from './types';

type WasmModule = {
  default: (input: string) => Promise<unknown>;
  detectSections: (
    tracks_json: string,
    sport_types_json: string,
    groups_json: string,
    config_json: string,
  ) => FrequentSection[];
};

let wasm: WasmModule | null = null;

async function loadWasm(): Promise<WasmModule> {
  if (wasm) return wasm;
  // The worker fetches the same WASM bundle the main thread uses. We
  // resolve URLs relative to `self.location` so the worker works from
  // any base path the host page is served from.
  const jsResponse = await fetch(new URL('/wasm/tracematch_wasm.js', self.location.href));
  const jsText = await jsResponse.text();
  const blob = new Blob([jsText], { type: 'application/javascript' });
  const blobUrl = URL.createObjectURL(blob);
  const mod = (await import(/* @vite-ignore */ blobUrl)) as WasmModule;
  URL.revokeObjectURL(blobUrl);
  await mod.default(new URL('/wasm/tracematch_wasm_bg.wasm', self.location.href).toString());
  wasm = mod;
  return mod;
}

export type DetectRequest = {
  type: 'detect';
  requestId: number;
  tracks: [string, GpsPoint[]][];
  sportTypes: Record<string, string>;
  groups: RouteGroup[];
  config: string; // JSON-stringified SectionConfig
};

export type DetectResponse =
  | { type: 'detect:ok'; requestId: number; sections: FrequentSection[] }
  | { type: 'detect:err'; requestId: number; message: string };

self.onmessage = async (event: MessageEvent<DetectRequest>) => {
  const req = event.data;
  if (req?.type !== 'detect') return;

  try {
    const mod = await loadWasm();
    const sections = mod.detectSections(
      JSON.stringify(req.tracks),
      JSON.stringify(req.sportTypes),
      JSON.stringify(req.groups),
      req.config,
    );
    const ok: DetectResponse = {
      type: 'detect:ok',
      requestId: req.requestId,
      sections,
    };
    self.postMessage(ok);
  } catch (err) {
    const msg = err instanceof Error ? err.message : String(err);
    const errResp: DetectResponse = {
      type: 'detect:err',
      requestId: req.requestId,
      message: msg,
    };
    self.postMessage(errResp);
  }
};
