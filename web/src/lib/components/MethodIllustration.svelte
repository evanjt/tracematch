<script lang="ts">
  import { onMount } from 'svelte';
  import type { FrequentSection } from '../wasm/types';

  let {
    mode = 'corridor',
    proximity = 150,
    minTracks = 3,
    minRoutes = 3,
    minSectionLength = 200,
  }: {
    mode: 'density' | 'flow' | 'corridor';
    proximity?: number;
    minTracks?: number;
    minRoutes?: number;
    minSectionLength?: number;
  } = $props();

  const REF_LAT = 46.22;
  const REF_LNG = 7.36;
  const LNG_SPAN = 0.04;
  const LAT_SPAN = 0.02;

  function svgToGps(x: number, y: number) {
    return { latitude: REF_LAT - (y / 200) * LAT_SPAN, longitude: REF_LNG + (x / 400) * LNG_SPAN };
  }
  function gpsToSvg(lat: number, lng: number): [number, number] {
    return [(lng - REF_LNG) / LNG_SPAN * 400, (REF_LAT - lat) / LAT_SPAN * 200];
  }

  // 5 base traces (one per route group), each duplicated 10x with jitter
  // to simulate realistic traffic volume for all detection methods.
  type Trace = { pts: [number, number][]; route: number };

  const BASE_TRACES: { pts: [number, number][]; route: number }[] = [
    { pts: [[15,105],[55,90],[110,78],[165,72],[220,70],[275,72],[330,78],[385,88]], route: 0 },
    { pts: [[16,106],[56,91],[111,79],[165,73],[185,52],[200,30],[210,15]], route: 1 },
    { pts: [[200,190],[220,170],[245,145],[270,115],[285,90],[330,79],[385,89]], route: 2 },
    { pts: [[165,73],[195,65],[225,63],[255,65],[275,73],[260,88],[225,92],[195,88],[165,73]], route: 3 },
    { pts: [[16,106],[56,91],[110,79],[130,100],[145,130],[155,160],[160,190]], route: 4 },
  ];

  // Generate 50 traces (10 copies per base with slight jitter)
  const traces: Trace[] = [];
  for (let rep = 0; rep < 10; rep++) {
    const jx = (rep - 5) * 0.8;
    const jy = (rep - 5) * 0.4;
    for (const base of BASE_TRACES) {
      traces.push({
        pts: base.pts.map(([x, y]) => [x + jx, y + jy] as [number, number]),
        route: base.route,
      });
    }
  }

  // Densify: interpolate points every ~3px for realistic GPS density
  function densify(pts: [number, number][]): { latitude: number; longitude: number }[] {
    const out: { latitude: number; longitude: number }[] = [svgToGps(pts[0][0], pts[0][1])];
    for (let i = 1; i < pts.length; i++) {
      const [x0, y0] = pts[i - 1];
      const [x1, y1] = pts[i];
      const dist = Math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2);
      const steps = Math.ceil(dist / 3);
      for (let s = 1; s <= steps; s++) {
        const t = s / steps;
        out.push(svgToGps(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t));
      }
    }
    return out;
  }

  let wasm: typeof import('../wasm/pkg/tracematch_wasm.js') | null = $state(null);

  onMount(async () => {
    const mod = await import('../wasm/pkg/tracematch_wasm.js');
    try { await mod.default(); } catch { /* already initialized */ }
    wasm = mod;
  });

  function buildInputs() {
    const tracks: [string, { latitude: number; longitude: number }[]][] = traces.map((t, i) => [
      `t${i}`, densify(t.pts),
    ]);
    const sportTypes: Record<string, string> = {};
    for (let i = 0; i < traces.length; i++) sportTypes[`t${i}`] = 'Run';
    return { tracks, sportTypes };
  }

  function buildConfig(overrides?: Partial<{ proximity: number; minTracks: number; minRoutes: number }>) {
    const p = overrides?.proximity ?? proximity;
    const mt = overrides?.minTracks ?? minTracks;
    const mr = overrides?.minRoutes ?? minRoutes;
    return JSON.stringify({
      proximityThreshold: p,
      minSectionLength,
      maxSectionLength: 200000,
      minActivities: mt,
      clusterTolerance: 80,
      samplePoints: 50,
      detectionMode: 'discovery',
      includePotentials: false,
      scalePresets: [
        { name: 'short', minLength: 50, maxLength: 500, minActivities: Math.max(mt, 2) },
        { name: 'medium', minLength: 500, maxLength: 2000, minActivities: Math.max(mt, 2) },
        { name: 'long', minLength: 2000, maxLength: 50000, minActivities: Math.max(mt, 2) },
      ],
      preserveHierarchy: true,
      jaccardThreshold: 0.5,
      minRoutes: mr,
      enableDensitySplits: false,
      mergeDistanceMultiplier: 4.0,
      minCellVisits: 3,
      divergenceThreshold: 0.10,
      minCorridorTracks: mt,
    });
  }

  const highlights = $derived.by(() => {
    if (!wasm) return [];

    const { tracks, sportTypes } = buildInputs();
    const tracksJson = JSON.stringify(tracks);
    const sportTypesJson = JSON.stringify(sportTypes);

    let sections: FrequentSection[] = [];
    try {
      if (mode === 'corridor') {
        sections = wasm.detectSectionsCorridor(tracksJson, sportTypesJson, buildConfig());
      } else if (mode === 'flow') {
        // Flow graph needs fine cells to detect junctions on small datasets.
        // Use proximity/3 as the effective proximity so cell_size stays small.
        const flowProx = Math.max(25, Math.round(proximity / 3));
        sections = wasm.detectSectionsFlowGraph(tracksJson, sportTypesJson, buildConfig({ proximity: flowProx }));
      } else {
        const sigs: any[] = [];
        for (const [id, pts] of tracks) {
          const sig = wasm.createSignature(id, JSON.stringify(pts), '{}');
          if (sig) sigs.push(sig);
        }
        const groups = wasm.groupRoutes(JSON.stringify(sigs), '{}');
        sections = wasm.detectSectionsWithProgress(
          tracksJson, sportTypesJson, JSON.stringify(groups), buildConfig(), () => {},
        );
      }
    } catch (e) {
      console.warn('[MethodIllustration] detection error:', e);
      return [];
    }

    return sections.map(s =>
      s.polyline.map(p => gpsToSvg(p.latitude, p.longitude).join(',')).join(' ')
    );
  });

  // Only show the 5 base traces (not the 50 jittered copies)
  const displayTraces = BASE_TRACES.map(t => t.pts.map(p => p.join(',')).join(' '));

  // Scale bar: 400px = 3076m, so 100px ~ 769m ~ 750m
  const SCALE_BAR_PX = 100;
  const SCALE_BAR_M = 750;
</script>

<svg viewBox="0 0 400 210" class="method-illustration" xmlns="http://www.w3.org/2000/svg">
  {#each displayTraces as points}
    <polyline {points} fill="none" stroke="rgba(255,255,255,0.18)" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" />
  {/each}
  {#each highlights as points}
    <polyline {points} fill="none" stroke="#FC4C02" stroke-width="3" stroke-linecap="round" stroke-linejoin="round" opacity="0.9" />
  {/each}

  <!-- Scale bar -->
  <line x1={400 - SCALE_BAR_PX - 10} y1="205" x2={400 - 10} y2="205" stroke="rgba(255,255,255,0.5)" stroke-width="1.5" />
  <line x1={400 - SCALE_BAR_PX - 10} y1="202" x2={400 - SCALE_BAR_PX - 10} y2="208" stroke="rgba(255,255,255,0.5)" stroke-width="1" />
  <line x1={400 - 10} y1="202" x2={400 - 10} y2="208" stroke="rgba(255,255,255,0.5)" stroke-width="1" />
  <text x={400 - SCALE_BAR_PX / 2 - 10} y="202" text-anchor="middle" fill="rgba(255,255,255,0.5)" font-size="8">{SCALE_BAR_M}m</text>
</svg>

<style>
  .method-illustration {
    width: 100%;
    height: 190px;
    background: rgba(0,0,0,0.3);
    border-radius: 6px;
    margin: 8px 0;
  }
</style>
