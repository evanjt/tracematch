<script lang="ts">
  let {
    mode = 'corridor',
    proximity = 150,
    minTracks = 3,
    minRoutes = 3,
  }: {
    mode: 'density' | 'flow' | 'corridor';
    proximity?: number;
    minTracks?: number;
    minRoutes?: number;
  } = $props();

  // 15 traces on a 400x200 canvas. Organized into 4 route groups.
  // Main east-west corridor, a north branch, a south branch, and a loop.
  type Trace = { pts: [number, number][]; route: number };

  const traces: Trace[] = [
    // Route group 0: main east-west corridor (5 traces, slight variation)
    { pts: [[20,110],[60,95],[110,80],[160,72],[210,70],[260,72],[310,78],[370,90]], route: 0 },
    { pts: [[25,115],[65,98],[115,82],[165,75],[215,72],[265,75],[315,80],[375,88]], route: 0 },
    { pts: [[18,108],[55,92],[105,78],[155,70],[205,68],[255,70],[305,76],[365,85]], route: 0 },
    { pts: [[22,112],[62,96],[112,81],[162,74],[212,71],[262,74],[312,79],[372,92]], route: 0 },
    { pts: [[20,106],[58,90],[108,76],[158,68],[208,66],[258,68],[308,74],[368,82]], route: 0 },

    // Route group 1: shares west half, branches north (3 traces)
    { pts: [[22,112],[60,96],[110,80],[155,72],[180,55],[200,35],[215,20]], route: 1 },
    { pts: [[18,108],[58,94],[108,78],[152,70],[175,52],[195,32],[210,18]], route: 1 },
    { pts: [[25,114],[63,98],[113,82],[158,74],[182,58],[202,38],[218,22]], route: 1 },

    // Route group 2: shares east half, comes from south (3 traces)
    { pts: [[180,185],[200,165],[225,140],[255,110],[275,90],[310,78],[370,88]], route: 2 },
    { pts: [[185,188],[205,168],[228,142],[258,112],[278,92],[312,80],[372,90]], route: 2 },
    { pts: [[175,182],[195,162],[222,138],[252,108],[272,88],[308,76],[368,86]], route: 2 },

    // Route group 3: short local loop in center (3 traces)
    { pts: [[140,110],[160,95],[190,90],[210,95],[220,110],[200,125],[170,125],[140,110]], route: 3 },
    { pts: [[142,112],[162,97],[192,92],[212,97],[222,112],[202,127],[172,127],[142,112]], route: 3 },
    { pts: [[138,108],[158,93],[188,88],[208,93],[218,108],[198,123],[168,123],[138,108]], route: 3 },
  ];

  // Scale proximity from meters (real-world) to illustration grid cells.
  // The illustration is 400px wide representing roughly 3km of terrain.
  // So 1px ~ 7.5m. Cell size = proximity / 7.5.
  const SCALE = 7.5;

  function rasterise(pts: [number, number][], cellSize: number): Set<string> {
    const cells = new Set<string>();
    for (const [x, y] of pts) {
      const cx = Math.floor(x / cellSize);
      const cy = Math.floor(y / cellSize);
      cells.add(`${cx},${cy}`);
    }
    // Fill gaps between consecutive points
    for (let i = 0; i < pts.length - 1; i++) {
      const [x0, y0] = pts[i];
      const [x1, y1] = pts[i + 1];
      const steps = Math.ceil(Math.max(Math.abs(x1 - x0), Math.abs(y1 - y0)) / (cellSize * 0.5));
      for (let s = 0; s <= steps; s++) {
        const t = s / Math.max(steps, 1);
        const cx = Math.floor((x0 + (x1 - x0) * t) / cellSize);
        const cy = Math.floor((y0 + (y1 - y0) * t) / cellSize);
        cells.add(`${cx},${cy}`);
      }
    }
    return cells;
  }

  // Compute corridor highlights: hot cells where >= minTracks traces overlap
  function computeCorridor(cellSize: number, threshold: number): string[] {
    const cellCounts = new Map<string, Set<number>>();
    traces.forEach((t, i) => {
      for (const key of rasterise(t.pts, cellSize)) {
        if (!cellCounts.has(key)) cellCounts.set(key, new Set());
        cellCounts.get(key)!.add(i);
      }
    });

    const hotCells = new Set<string>();
    for (const [key, ids] of cellCounts) {
      if (ids.size >= threshold) hotCells.add(key);
    }

    // For each trace, find segments that pass through hot cells
    const segments: string[] = [];
    for (const t of traces) {
      let run: [number, number][] = [];
      for (const [x, y] of t.pts) {
        const cx = Math.floor(x / cellSize);
        const cy = Math.floor(y / cellSize);
        if (hotCells.has(`${cx},${cy}`)) {
          run.push([x, y]);
        } else {
          if (run.length >= 2) segments.push(run.map(p => p.join(',')).join(' '));
          run = [];
        }
      }
      if (run.length >= 2) segments.push(run.map(p => p.join(',')).join(' '));
    }
    return segments;
  }

  // Compute density grid highlights: hot cells where >= minRoutes route groups overlap
  function computeDensity(cellSize: number, threshold: number): string[] {
    const cellRoutes = new Map<string, Set<number>>();
    for (const t of traces) {
      for (const key of rasterise(t.pts, cellSize)) {
        if (!cellRoutes.has(key)) cellRoutes.set(key, new Set());
        cellRoutes.get(key)!.add(t.route);
      }
    }

    const hotCells = new Set<string>();
    for (const [key, routes] of cellRoutes) {
      if (routes.size >= threshold) hotCells.add(key);
    }

    const segments: string[] = [];
    for (const t of traces) {
      let run: [number, number][] = [];
      for (const [x, y] of t.pts) {
        const cx = Math.floor(x / cellSize);
        const cy = Math.floor(y / cellSize);
        if (hotCells.has(`${cx},${cy}`)) {
          run.push([x, y]);
        } else {
          if (run.length >= 2) segments.push(run.map(p => p.join(',')).join(' '));
          run = [];
        }
      }
      if (run.length >= 2) segments.push(run.map(p => p.join(',')).join(' '));
    }
    return segments;
  }

  // Compute flow graph highlights: find junction cells and short edges between them
  function computeFlow(cellSize: number, threshold: number): string[] {
    const cellCounts = new Map<string, Set<number>>();
    traces.forEach((t, i) => {
      for (const key of rasterise(t.pts, cellSize)) {
        if (!cellCounts.has(key)) cellCounts.set(key, new Set());
        cellCounts.get(key)!.add(i);
      }
    });

    // Track transitions between cells per trace
    const exits = new Map<string, Map<string, Set<number>>>();
    traces.forEach((t, ti) => {
      const cells: string[] = [];
      for (const [x, y] of t.pts) {
        const key = `${Math.floor(x / cellSize)},${Math.floor(y / cellSize)}`;
        if (cells.length === 0 || cells[cells.length - 1] !== key) cells.push(key);
      }
      for (let i = 0; i < cells.length - 1; i++) {
        if (!exits.has(cells[i])) exits.set(cells[i], new Map());
        const m = exits.get(cells[i])!;
        if (!m.has(cells[i + 1])) m.set(cells[i + 1], new Set());
        m.get(cells[i + 1])!.add(ti);
      }
    });

    // Junctions: cells with 3+ distinct exit directions with >= 2 traces each
    const junctions = new Set<string>();
    for (const [cell, exitMap] of exits) {
      const count = cellCounts.get(cell)?.size ?? 0;
      if (count < threshold) continue;
      let sigDirs = 0;
      for (const ids of exitMap.values()) {
        if (ids.size >= 2) sigDirs++;
      }
      if (sigDirs >= 3) junctions.add(cell);
    }

    // For each trace, find short segments between junctions
    const segments: string[] = [];
    for (const t of traces) {
      const cells: { key: string; pt: [number, number] }[] = [];
      for (const [x, y] of t.pts) {
        const key = `${Math.floor(x / cellSize)},${Math.floor(y / cellSize)}`;
        if (cells.length === 0 || cells[cells.length - 1].key !== key) {
          cells.push({ key, pt: [x, y] });
        }
      }
      let run: [number, number][] = [];
      let inEdge = false;
      for (const { key, pt } of cells) {
        if (junctions.has(key)) {
          if (inEdge && run.length >= 2) {
            run.push(pt);
            segments.push(run.map(p => p.join(',')).join(' '));
          }
          run = [pt];
          inEdge = true;
        } else if (inEdge) {
          run.push(pt);
        }
      }
    }
    return segments;
  }

  const cellSize = $derived(proximity / SCALE);

  const highlights = $derived(
    mode === 'corridor' ? computeCorridor(cellSize, minTracks) :
    mode === 'density' ? computeDensity(cellSize, minRoutes) :
    computeFlow(cellSize, minTracks)
  );

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
