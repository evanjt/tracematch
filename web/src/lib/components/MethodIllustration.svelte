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

  type Trace = { pts: [number, number][]; route: number };

  // 14 traces on a 400x200 canvas, organized into 4 route groups.
  // Traces within the same area are kept within ~8px of each other
  // so they share grid cells at typical cell sizes (15-25px).
  const traces: Trace[] = [
    // Route 0: main east-west corridor (5 traces, tight spread)
    { pts: [[15,105],[55,90],[110,78],[165,72],[220,70],[275,72],[330,78],[385,88]], route: 0 },
    { pts: [[18,107],[58,92],[113,80],[168,74],[223,72],[278,74],[333,80],[388,90]], route: 0 },
    { pts: [[12,103],[52,88],[107,76],[162,70],[217,68],[272,70],[327,76],[382,86]], route: 0 },
    { pts: [[16,109],[56,94],[111,82],[166,76],[221,74],[276,76],[331,82],[386,92]], route: 0 },
    { pts: [[14,101],[54,86],[109,74],[164,68],[219,66],[274,68],[329,74],[384,84]], route: 0 },

    // Route 1: shares west half with route 0, branches north at x~165
    { pts: [[16,106],[56,91],[111,79],[165,73],[185,52],[200,30],[210,15]], route: 1 },
    { pts: [[13,104],[53,89],[108,77],[162,71],[182,50],[197,28],[207,13]], route: 1 },
    { pts: [[19,108],[59,93],[114,81],[168,75],[188,54],[203,32],[213,17]], route: 1 },

    // Route 2: comes from south, joins route 0 at x~275
    { pts: [[200,190],[220,170],[245,145],[270,115],[285,90],[330,79],[385,89]], route: 2 },
    { pts: [[203,192],[223,172],[248,147],[273,117],[288,92],[333,81],[388,91]], route: 2 },

    // Route 3: local loop overlapping route 0 in center (x~165-275)
    { pts: [[165,73],[195,65],[225,63],[255,65],[275,73],[260,88],[225,92],[195,88],[165,73]], route: 3 },
    { pts: [[168,75],[198,67],[228,65],[258,67],[278,75],[263,90],[228,94],[198,90],[168,75]], route: 3 },
    { pts: [[162,71],[192,63],[222,61],[252,63],[272,71],[257,86],[222,90],[192,86],[162,71]], route: 3 },
  ];

  // 1px ~ 7.5m. Cell size = proximity / 7.5.
  const SCALE = 7.5;

  function rasterise(pts: [number, number][], cellSize: number): Set<string> {
    const cells = new Set<string>();
    for (let i = 0; i < pts.length; i++) {
      const [x, y] = pts[i];
      cells.add(`${Math.floor(x / cellSize)},${Math.floor(y / cellSize)}`);
      if (i < pts.length - 1) {
        const [x1, y1] = pts[i + 1];
        const dist = Math.max(Math.abs(x1 - x), Math.abs(y1 - y));
        const steps = Math.ceil(dist / (cellSize * 0.4));
        for (let s = 1; s < steps; s++) {
          const t = s / steps;
          const ix = Math.floor((x + (x1 - x) * t) / cellSize);
          const iy = Math.floor((y + (y1 - y) * t) / cellSize);
          cells.add(`${ix},${iy}`);
        }
      }
    }
    return cells;
  }

  function traceSegments(
    hotCells: Set<string>,
    cellSize: number,
  ): string[] {
    const segments: string[] = [];
    for (const t of traces) {
      let run: [number, number][] = [];
      for (const [x, y] of t.pts) {
        const key = `${Math.floor(x / cellSize)},${Math.floor(y / cellSize)}`;
        if (hotCells.has(key)) {
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

  function computeCorridor(cellSize: number, threshold: number): string[] {
    const cellCounts = new Map<string, Set<number>>();
    traces.forEach((t, i) => {
      for (const key of rasterise(t.pts, cellSize)) {
        if (!cellCounts.has(key)) cellCounts.set(key, new Set());
        cellCounts.get(key)!.add(i);
      }
    });
    const hot = new Set<string>();
    for (const [key, ids] of cellCounts) {
      if (ids.size >= threshold) hot.add(key);
    }
    return traceSegments(hot, cellSize);
  }

  function computeDensity(cellSize: number, threshold: number): string[] {
    const cellRoutes = new Map<string, Set<number>>();
    for (const t of traces) {
      for (const key of rasterise(t.pts, cellSize)) {
        if (!cellRoutes.has(key)) cellRoutes.set(key, new Set());
        cellRoutes.get(key)!.add(t.route);
      }
    }
    const hot = new Set<string>();
    for (const [key, routes] of cellRoutes) {
      if (routes.size >= threshold) hot.add(key);
    }
    return traceSegments(hot, cellSize);
  }

  function computeFlow(cellSize: number, threshold: number): string[] {
    // Count traces per cell
    const cellCounts = new Map<string, Set<number>>();
    traces.forEach((t, i) => {
      for (const key of rasterise(t.pts, cellSize)) {
        if (!cellCounts.has(key)) cellCounts.set(key, new Set());
        cellCounts.get(key)!.add(i);
      }
    });

    // Track transitions
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

    // Junctions: cells with 3+ exit directions each having 2+ traces
    const junctions = new Set<string>();
    for (const [cell, exitMap] of exits) {
      const count = cellCounts.get(cell)?.size ?? 0;
      if (count < threshold) continue;
      let sigDirs = 0;
      for (const ids of exitMap.values()) {
        if (ids.size >= 2) sigDirs++;
      }
      if (sigDirs >= 2) junctions.add(cell);
    }

    // Trace edges between junctions
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
          if (inEdge && run.length >= 1) {
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
