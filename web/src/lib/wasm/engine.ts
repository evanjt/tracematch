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
