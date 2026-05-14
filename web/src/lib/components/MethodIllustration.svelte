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

  // SVG (0-400, 0-200) mapped to GPS coordinates.
  // ~3km east-west, ~1.5km north-south around Sion, Switzerland.
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

  // 14 traces in 4 route groups. Coordinates are SVG space.
  type Trace = { pts: [number, number][]; route: number };
  const traces: Trace[] = [
    { pts: [[15,105],[55,90],[110,78],[165,72],[220,70],[275,72],[330,78],[385,88]], route: 0 },
    { pts: [[18,107],[58,92],[113,80],[168,74],[223,72],[278,74],[333,80],[388,90]], route: 0 },
    { pts: [[12,103],[52,88],[107,76],[162,70],[217,68],[272,70],[327,76],[382,86]], route: 0 },
    { pts: [[16,109],[56,94],[111,82],[166,76],[221,74],[276,76],[331,82],[386,92]], route: 0 },
    { pts: [[14,101],[54,86],[109,74],[164,68],[219,66],[274,68],[329,74],[384,84]], route: 0 },
    { pts: [[16,106],[56,91],[111,79],[165,73],[185,52],[200,30],[210,15]], route: 1 },
    { pts: [[13,104],[53,89],[108,77],[162,71],[182,50],[197,28],[207,13]], route: 1 },
    { pts: [[19,108],[59,93],[114,81],[168,75],[188,54],[203,32],[213,17]], route: 1 },
    { pts: [[200,190],[220,170],[245,145],[270,115],[285,90],[330,79],[385,89]], route: 2 },
    { pts: [[203,192],[223,172],[248,147],[273,117],[288,92],[333,81],[388,91]], route: 2 },
    { pts: [[175,182],[195,162],[222,138],[252,108],[272,88],[308,76],[368,86]], route: 2 },
    { pts: [[165,73],[195,65],[225,63],[255,65],[275,73],[260,88],[225,92],[195,88],[165,73]], route: 3 },
    { pts: [[168,75],[198,67],[228,65],[258,67],[278,75],[263,90],[228,94],[198,90],[168,75]], route: 3 },
    { pts: [[162,71],[192,63],[222,61],[252,63],[272,71],[257,86],[222,90],[192,86],[162,71]], route: 3 },
  ];

  // WASM module loaded lazily
  let wasm: typeof import('../wasm/pkg/tracematch_wasm.js') | null = $state(null);

  onMount(async () => {
    const mod = await import('../wasm/pkg/tracematch_wasm.js');
    try { await mod.default(); } catch { /* already initialized */ }
    wasm = mod;
  });

  // Build WASM inputs from illustration traces
  function buildInputs() {
    const tracks: [string, { latitude: number; longitude: number }[]][] = traces.map((t, i) => [
      `trace_${i}`,
      t.pts.map(([x, y]) => svgToGps(x, y)),
    ]);
    const sportTypes: Record<string, string> = {};
    for (let i = 0; i < traces.length; i++) sportTypes[`trace_${i}`] = 'Run';
    return { tracks, sportTypes };
  }

  function buildConfig() {
    return JSON.stringify({
      proximityThreshold: proximity,
      minSectionLength,
      maxSectionLength: 200000,
      minActivities: minTracks,
      clusterTolerance: 80,
      samplePoints: 50,
      detectionMode: 'discovery',
      includePotentials: false,
      scalePresets: [
        { name: 'short', minLength: 100, maxLength: 500, minActivities: Math.max(minTracks, 2) },
        { name: 'medium', minLength: 500, maxLength: 2000, minActivities: Math.max(minTracks, 2) },
        { name: 'long', minLength: 2000, maxLength: 50000, minActivities: Math.max(minTracks, 2) },
      ],
      preserveHierarchy: true,
      jaccardThreshold: 0.5,
      minRoutes,
      enableDensitySplits: false,
      mergeDistanceMultiplier: 4.0,
      minCellVisits: 5,
      divergenceThreshold: 0.15,
      minCorridorTracks: minTracks,
    });
  }

  // Run real detection and convert results to SVG polylines
  const highlights = $derived.by(() => {
    if (!wasm) return [];

    const { tracks, sportTypes } = buildInputs();
    const tracksJson = JSON.stringify(tracks);
    const sportTypesJson = JSON.stringify(sportTypes);
    const configJson = buildConfig();

    let sections: FrequentSection[] = [];
    try {
      if (mode === 'corridor') {
        sections = wasm.detectSectionsCorridor(tracksJson, sportTypesJson, configJson);
      } else if (mode === 'flow') {
        sections = wasm.detectSectionsFlowGraph(tracksJson, sportTypesJson, configJson);
      } else {
        // Density grid needs route groups. Build signatures and group first.
        const sigs: any[] = [];
        for (const [id, pts] of tracks) {
          const sig = wasm.createSignature(id, JSON.stringify(pts), '{}');
          if (sig) sigs.push(sig);
        }
        const groups = wasm.groupRoutes(JSON.stringify(sigs), '{}');
        const groupsJson = JSON.stringify(groups);
        sections = wasm.detectSectionsWithProgress(
          tracksJson, sportTypesJson, groupsJson, configJson,
          () => {},
        );
      }
    } catch (e) {
      console.warn('[MethodIllustration] detection error:', e);
      return [];
    }

    // Convert section polylines from GPS back to SVG
    return sections.map(s =>
      s.polyline.map(p => gpsToSvg(p.latitude, p.longitude).join(',')).join(' ')
    );
  });

  const traceStrings = traces.map(t => t.pts.map(p => p.join(',')).join(' '));
</script>

<svg viewBox="0 0 400 200" class="method-illustration" xmlns="http://www.w3.org/2000/svg">
  {#each traceStrings as points}
    <polyline {points} fill="none" stroke="rgba(255,255,255,0.15)" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" />
  {/each}
  {#each highlights as points}
    <polyline {points} fill="none" stroke="#FC4C02" stroke-width="3" stroke-linecap="round" stroke-linejoin="round" opacity="0.9" />
  {/each}
</svg>

<style>
  .method-illustration {
    width: 100%;
    height: 180px;
    background: rgba(0,0,0,0.3);
    border-radius: 6px;
    margin: 8px 0;
  }
</style>
