<script lang="ts">
  import { onMount } from 'svelte';
  import { traceStore, type StoredTrace } from '$lib/stores/traces.svelte';
  import { parseGpx } from '$lib/parsers/gpx';
  import { runAnalysisAsync } from '$lib/wasm/engine';
  import type { RouteGroup, FrequentSection, GpsPoint } from '$lib/wasm/types';

  // --- Sport color system ---
  const SPORT_COLORS: Record<string, string> = {
    Ride: '#3B82F6', Run: '#10B981', Walk: '#8B5CF6', Swim: '#06B6D4'
  };
  const DEFAULT_SPORT_COLOR = '#64748b';
  const SECTION_COLOR = '#FC4C02';

  function sportColor(sportType: string | undefined): string {
    return SPORT_COLORS[sportType ?? ''] ?? DEFAULT_SPORT_COLOR;
  }

  function getRouteStyle(sportType: string, activityCount: number, maxCount: number) {
    const color = SPORT_COLORS[sportType] ?? DEFAULT_SPORT_COLOR;
    const ratio = activityCount / Math.max(maxCount, 1);
    return { color, weight: 1.5 + ratio * 3.5, opacity: 0.3 + ratio * 0.55 };
  }

  // --- Core state ---
  let wasmReady = $state(false);
  let analysing = $state(false);
  let analysisProgress = $state<{ phase: string; current: number; total: number } | null>(null);
  let overallProgress = $state(0);
  let error = $state<string | null>(null);

  // Each analysis pipeline step gets an equal slice of the overall 0-100
  // bar. Within a step, sub-progress (current/total) fills its slice.
  // Transitions ("Serializing...", "Preparing...") snap to the start of the
  // next step so the bar advances cleanly between phases.
  const ANALYSIS_STEPS = [
    'Creating signatures',
    'Comparing route pairs',
    'Building spatial indices',
    'Finding section overlaps',
    'Post-processing sections',
    'Saving results'
  ] as const;
  const TOTAL_STEPS = ANALYSIS_STEPS.length;
  // Maps every phase string (including transitions and the init phase) to
  // the step index it belongs to. Step index = 0 for everything before
  // grouping; transitions hop to the index of the upcoming real step.
  // The "Finding dense regions" / "Assembling sections" labels come from
  // the density-grid section detection in Rust (re-labelled in
  // sectionWorker.ts via PHASE_LABELS).
  const PHASE_TO_STEP: Record<string, number> = {
    'Preparing': 0,
    'Initializing WASM engine': 0,
    'Creating signatures': 0,
    'Serializing routes for grouping': 1,
    'Preparing route comparison': 1,
    'Comparing route pairs': 1,
    'Serializing tracks for section detection': 2,
    'Preparing section detection': 2,
    'Finding dense regions': 2,
    'Assembling sections': 3,
    'Post-processing sections': 4,
    'Finalizing results': 5,
    'Saving results': 5
  };
  let sectionError = $state<string | null>(null);
  let dragOver = $state(false);
  let mapDragOver = $state(false);
  let mapContainer: HTMLDivElement;
  let renderProgress = $state<{ current: number; total: number } | null>(null);
  let importProgress = $state<{ current: number; total: number } | null>(null);

  // Section detection parameters
  let proximityThreshold = $state(50);
  let minSectionLength = $state(200);
  let minActivities = $state(3);
  let minRoutes = $state(3);
  let settingsCollapsed = $state(true);

  // Layer visibility
  let showTraces = $state(true);
  let showRoutes = $state(false);
  let showSections = $state(false);
  let sportFilters = $state(new Set(['Ride', 'Run', 'Walk', 'Swim', 'Other']));

  // Sidebar state
  let showAllRoutes = $state(false);
  let showAllSections = $state(false);

  // Hover state
  let highlightedRouteGroupId = $state<string | null>(null);
  let highlightedSectionId = $state<string | null>(null);

  // Theme state
  let themeMode = $state<'system' | 'light' | 'dark'>(
    typeof localStorage !== 'undefined' ? (localStorage.getItem('theme') as any) || 'system' : 'system'
  );
  let resolvedDark = $state(false);

  // Map internals
  let map: L.Map | null = null;
  let tileLayer: L.TileLayer | null = null;
  let traceLayerGroup: L.LayerGroup | null = null;
  let routeLayerGroup: L.LayerGroup | null = null;
  let sectionLayerGroup: L.LayerGroup | null = null;
  let L: typeof import('leaflet') | null = null;
  let hasFitOnce = false;
  let lastAnalyzedAt: number | null = null;

  // Polyline maps
  let tracePolylines = new Map<string, { polyline: L.Polyline; sportType: string }>();
  let routePolylines = new Map<string, { polyline: L.Polyline; groupId: string; sportType: string }>();
  let portionOverlays: L.Polyline[] = [];

  // --- Derived data ---
  let sportBreakdown = $derived.by(() => {
    const counts: Record<string, { traces: number; routes: number }> = {};
    for (const t of traceStore.traces) {
      const s = t.sportType || 'Other';
      if (!counts[s]) counts[s] = { traces: 0, routes: 0 };
      counts[s].traces++;
    }
    const analysis = traceStore.analysis;
    if (analysis) {
      for (const g of analysis.groups) {
        const s = g.sportType || 'Other';
        if (!counts[s]) counts[s] = { traces: 0, routes: 0 };
        counts[s].routes++;
      }
    }
    return counts;
  });

  let sortedRoutes = $derived.by(() => {
    const analysis = traceStore.analysis;
    if (!analysis) return [];
    return [...analysis.groups].sort((a, b) => b.activityIds.length - a.activityIds.length);
  });

  let sortedSections = $derived.by(() => {
    const analysis = traceStore.analysis;
    if (!analysis) return [];
    return [...analysis.sections].sort((a, b) => b.visitCount - a.visitCount);
  });

  let maxRouteCount = $derived.by(() => {
    if (sortedRoutes.length === 0) return 1;
    return sortedRoutes[0].activityIds.length;
  });

  let hasAnalysis = $derived(traceStore.analysis !== null);

  // --- Helpers ---
  function traceNameById(id: string): string {
    return traceStore.traces.find((t) => t.id === id)?.name ?? id.slice(0, 8);
  }

  function traceSportById(id: string): string {
    return traceStore.traces.find((t) => t.id === id)?.sportType ?? 'Other';
  }

  function traceDistanceById(id: string): number {
    return traceStore.traces.find((t) => t.id === id)?.distance ?? 0;
  }

  function toggleSportFilter(sport: string) {
    const next = new Set(sportFilters);
    if (next.has(sport)) next.delete(sport);
    else next.add(sport);
    sportFilters = next;
  }

  function isTraceVisible(trace: StoredTrace): boolean {
    const s = trace.sportType || 'Other';
    return sportFilters.has(s);
  }

  function isGroupVisible(group: RouteGroup): boolean {
    const s = group.sportType || 'Other';
    return sportFilters.has(s);
  }

  function zoomToGroup(group: RouteGroup) {
    if (!map || !L) return;
    const trace = traceStore.traces.find((t) => t.id === group.representativeId);
    if (!trace) return;
    const latlngs = trace.points.map((p) => L!.latLng(p.latitude, p.longitude));
    if (latlngs.length > 0) {
      map.fitBounds(L.latLngBounds(latlngs), { padding: [60, 60] });
    }
  }

  function zoomToSection(section: FrequentSection) {
    if (!map || !L) return;
    const latlngs = section.polyline.map((p) => L!.latLng(p.latitude, p.longitude));
    if (latlngs.length > 0) {
      map.fitBounds(L.latLngBounds(latlngs), { padding: [60, 60] });
    }
  }

  function routeIndexLabel(group: RouteGroup): string {
    const idx = traceStore.analysis?.groups.indexOf(group) ?? -1;
    return group.customName || `Route ${idx + 1}`;
  }

  function sectionIndexLabel(section: FrequentSection): string {
    const idx = traceStore.analysis?.sections.indexOf(section) ?? -1;
    return section.name || `Section ${idx + 1}`;
  }

  // --- CSS-class hover helpers ---
  function getOverlayPane(): Element | null {
    if (!map) return null;
    return map.getPane('overlayPane') ?? null;
  }

  function clearHighlight() {
    const pane = getOverlayPane();
    if (pane) pane.classList.remove('dimmed');
    // Remove .hl from everything
    document.querySelectorAll('.leaflet-overlay-pane path.hl').forEach((el) => el.classList.remove('hl'));
    // Clear portion overlays
    for (const ol of portionOverlays) ol.remove();
    portionOverlays = [];
  }

  function highlightPolylines(polylines: L.Polyline[]) {
    const pane = getOverlayPane();
    if (!pane) return;
    pane.classList.add('dimmed');
    for (const pl of polylines) {
      pl.getElement()?.classList.add('hl');
    }
  }

  function onRouteHover(group: RouteGroup) {
    highlightedRouteGroupId = group.groupId;
    clearHighlight();
    // Highlight the route representative polyline
    const entry = routePolylines.get(group.groupId);
    if (entry) highlightPolylines([entry.polyline]);
    // Also highlight matching trace polylines
    const tracePls: L.Polyline[] = [];
    for (const id of group.activityIds) {
      const te = tracePolylines.get(id);
      if (te) tracePls.push(te.polyline);
    }
    if (tracePls.length > 0) highlightPolylines(tracePls);
  }

  function onRouteLeave() {
    highlightedRouteGroupId = null;
    clearHighlight();
  }

  function onSectionHover(section: FrequentSection) {
    highlightedSectionId = section.id;
    clearHighlight();
    // Highlight the section's portion traces on map
    const pane = getOverlayPane();
    if (!pane) return;
    pane.classList.add('dimmed');

    // Highlight section polyline layers (they are in sectionLayerGroup)
    // All section layers get .hl — we rely on them being in the section layer group
    // For now, highlight all matching trace polylines + create portion overlays
    for (const portion of section.activityPortions) {
      const trace = traceStore.traces.find((t) => t.id === portion.activityId);
      if (!trace || !L || !traceLayerGroup) continue;
      const pts = trace.points.slice(portion.startIndex, portion.endIndex + 1);
      if (pts.length < 2) continue;
      const latlngs = pts.map((pt) => L!.latLng(pt.latitude, pt.longitude));
      const ol = L.polyline(latlngs, {
        color: SECTION_COLOR,
        weight: 5,
        opacity: 1,
        className: 'hl'
      }).addTo(traceLayerGroup);
      ol.bringToFront();
      portionOverlays.push(ol);
    }

    // Also highlight the section's own polyline elements
    sectionLayerGroup?.eachLayer((layer: any) => {
      layer.getElement?.()?.classList.add('hl');
    });
  }

  function onSectionLeave() {
    highlightedSectionId = null;
    clearHighlight();
  }

  // --- Map sync functions ---
  let _renderVersion = 0;

  async function syncTracesToMap() {
    if (!map || !L || !traceLayerGroup) return;
    const thisRender = ++_renderVersion;
    const traces = traceStore.traces;
    const currentIds = new Set(traces.map((t) => t.id));

    // Remove polylines for traces that no longer exist
    for (const [id, entry] of tracePolylines) {
      if (!currentIds.has(id)) {
        entry.polyline.remove();
        tracePolylines.delete(id);
      }
    }

    // Find traces that need rendering
    const toAdd = traces.filter((t) => !tracePolylines.has(t.id));
    if (toAdd.length === 0) return;

    const total = toAdd.length;
    if (total > 30) renderProgress = { current: 0, total };

    const allLatLngs: L.LatLng[] = [];
    const batchSize = 30;

    for (let i = 0; i < toAdd.length; i += batchSize) {
      if (_renderVersion !== thisRender) return;
      const end = Math.min(i + batchSize, toAdd.length);
      for (let j = i; j < end; j++) {
        const trace = toAdd[j];
        const latlngs = trace.points.map((p) => L!.latLng(p.latitude, p.longitude));
        if (latlngs.length === 0) continue;

        const color = sportColor(trace.sportType);
        const pl = L.polyline(latlngs, { color, weight: 2, opacity: 0.5 })
          .bindPopup(
            `<b>${trace.name}</b><br>` +
              `${(trace.distance / 1000).toFixed(1)} km &middot; ${trace.points.length} pts` +
              `<br>${trace.sportType || 'Unknown'}`
          )
          .addTo(traceLayerGroup!);
        tracePolylines.set(trace.id, { polyline: pl, sportType: trace.sportType || 'Other' });
        if (isTraceVisible(trace)) {
          allLatLngs.push(...latlngs);
        }
      }

      // Fit bounds after first batch for quick initial view
      if (i === 0 && !hasFitOnce && allLatLngs.length > 0) {
        hasFitOnce = true;
        map!.fitBounds(L!.latLngBounds(allLatLngs), { padding: [40, 40] });
      }

      if (total > 30) renderProgress = { current: end, total };
      if (i + batchSize < toAdd.length) {
        await new Promise((r) => setTimeout(r, 0));
      }
    }

    renderProgress = null;
  }

  function syncRoutesToMap() {
    if (!map || !L || !routeLayerGroup) return;
    routeLayerGroup.clearLayers();
    routePolylines.clear();

    const analysis = traceStore.analysis;
    if (!analysis) return;

    for (const group of analysis.groups) {
      const trace = traceStore.traces.find((t) => t.id === group.representativeId);
      if (!trace) continue;
      const latlngs = trace.points.map((p) => L!.latLng(p.latitude, p.longitude));
      if (latlngs.length === 0) continue;

      const style = getRouteStyle(group.sportType, group.activityIds.length, maxRouteCount);
      const idx = analysis.groups.indexOf(group);
      const pl = L.polyline(latlngs, style)
        .bindPopup(
          `<b>${group.customName || `Route ${idx + 1}`}</b><br>` +
            `${group.activityIds.length} activities &middot; ${group.sportType}` +
            `<br>${(trace.distance / 1000).toFixed(1)} km`
        )
        .on('mouseover', () => onRouteHover(group))
        .on('mouseout', () => onRouteLeave())
        .addTo(routeLayerGroup!);
      routePolylines.set(group.groupId, { polyline: pl, groupId: group.groupId, sportType: group.sportType });
    }
  }

  function syncSections() {
    if (!map || !L || !sectionLayerGroup) return;
    sectionLayerGroup.clearLayers();

    const analysis = traceStore.analysis;
    if (!analysis?.sections) return;

    for (const section of analysis.sections) {
      const latlngs = section.polyline.map((p: GpsPoint) =>
        L!.latLng(p.latitude, p.longitude)
      );
      if (latlngs.length < 2) continue;

      L.polyline(latlngs, { color: '#ffffff', weight: 8, opacity: 0.8 }).addTo(sectionLayerGroup!);

      const idx = analysis.sections.indexOf(section);
      L.polyline(latlngs, { color: SECTION_COLOR, weight: 5, opacity: 0.9 })
        .bindPopup(
          `<b>${section.name || `Section ${idx + 1}`}</b><br>` +
            `${(section.distanceMeters / 1000).toFixed(1)} km<br>` +
            `${section.visitCount} traversals`
        )
        .on('mouseover', () => onSectionHover(section))
        .on('mouseout', () => onSectionLeave())
        .addTo(sectionLayerGroup!);

      const markerIcon = L.divIcon({
        className: 'section-marker',
        html: '<div style="width:10px;height:10px;border-radius:50%;background:#FC4C02;border:2px solid white;"></div>',
        iconSize: [14, 14],
        iconAnchor: [7, 7]
      });
      L.marker(latlngs[0], { icon: markerIcon }).addTo(sectionLayerGroup!);
      L.marker(latlngs[latlngs.length - 1], { icon: markerIcon }).addTo(sectionLayerGroup!);
    }
  }

  function updateLayerVisibility() {
    if (!map || !traceLayerGroup || !routeLayerGroup || !sectionLayerGroup) return;
    if (showTraces) traceLayerGroup.addTo(map);
    else traceLayerGroup.remove();
    if (showRoutes) routeLayerGroup.addTo(map);
    else routeLayerGroup.remove();
    if (showSections) sectionLayerGroup.addTo(map);
    else sectionLayerGroup.remove();
  }

  function applyTraceFilters() {
    for (const [id, entry] of tracePolylines) {
      const s = entry.sportType === 'Other' ? 'Other' : entry.sportType;
      const visible = sportFilters.has(s);
      entry.polyline.setStyle({ opacity: visible ? 0.5 : 0, weight: visible ? 2 : 0 });
    }
  }

  function applyRouteFilters() {
    for (const [, entry] of routePolylines) {
      const s = entry.sportType === 'Other' ? 'Other' : entry.sportType;
      const visible = sportFilters.has(s);
      entry.polyline.setStyle({ opacity: visible ? undefined : 0, weight: visible ? undefined : 0 });
    }
  }

  // --- Lifecycle ---
  onMount(async () => {
    console.time('[tracematch] startup:total');
    console.time('[tracematch] startup:idb-load');
    await traceStore.load();
    console.timeEnd('[tracematch] startup:idb-load');
    // WASM is loaded lazily inside the analysis worker on first run.
    wasmReady = true;
    console.timeEnd('[tracematch] startup:total');
    console.log(`[tracematch] ${traceStore.traces.length} traces loaded, analysis: ${traceStore.analysis ? 'yes' : 'no'}`);

    const mq = window.matchMedia('(prefers-color-scheme: dark)');
    mq.addEventListener('change', () => { if (themeMode === 'system') resolvedDark = mq.matches; });
  });

  // Theme effect
  $effect(() => {
    if (typeof window === 'undefined') return;
    const systemDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
    resolvedDark = themeMode === 'system' ? systemDark : themeMode === 'dark';
    document.documentElement.setAttribute('data-theme', resolvedDark ? 'dark' : 'light');
    localStorage.setItem('theme', themeMode);
  });

  // Initialize map
  $effect(() => {
    if (!mapContainer || typeof window === 'undefined') return;
    if (map) return;

    import('leaflet').then((leaflet) => {
      import('leaflet/dist/leaflet.css');
      L = leaflet;
      map = L.map(mapContainer).setView([30, 0], 2);
      const tileUrl = resolvedDark
        ? 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png'
        : 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';
      tileLayer = L.tileLayer(tileUrl, {
        attribution: '&copy; OpenStreetMap contributors &copy; CARTO',
        maxZoom: 19
      }).addTo(map);
      traceLayerGroup = L.layerGroup().addTo(map);
      routeLayerGroup = L.layerGroup();
      sectionLayerGroup = L.layerGroup();

      void syncTracesToMap();
      updateLayerVisibility();
    });
  });

  // Swap tiles when theme changes
  $effect(() => {
    const dark = resolvedDark;
    if (!map || !L || !tileLayer) return;
    const newUrl = dark
      ? 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png'
      : 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png';
    tileLayer.setUrl(newUrl);
  });

  // React to trace list changes
  $effect(() => {
    traceStore.traces;
    if (!map || !L) return;
    void syncTracesToMap();
  });

  // React to analysis changes
  $effect(() => {
    const analysis = traceStore.analysis;
    if (!map || !L) return;
    const analyzedAt = analysis?.analyzedAt ?? null;
    if (analyzedAt !== lastAnalyzedAt) {
      lastAnalyzedAt = analyzedAt;
      syncRoutesToMap();
      syncSections();
      if (analysis) {
        showTraces = false;
        showRoutes = true;
        showSections = true;
      }
      updateLayerVisibility();
    }
  });

  // Derive overall analysis progress from the current sub-step progress.
  // Unknown phases (no entry in PHASE_TO_STEP) keep the bar where it was
  // rather than snapping back to zero.
  $effect(() => {
    if (!analysisProgress) {
      overallProgress = 0;
      return;
    }
    const idx = PHASE_TO_STEP[analysisProgress.phase];
    if (idx === undefined) return;
    const sub = analysisProgress.total > 0
      ? Math.min(analysisProgress.current / analysisProgress.total, 1)
      : 0;
    overallProgress = ((idx + sub) / TOTAL_STEPS) * 100;
  });

  // React to layer toggles
  $effect(() => {
    showTraces; showRoutes; showSections;
    updateLayerVisibility();
  });

  // React to sport filter changes
  $effect(() => {
    sportFilters;
    if (!map || !L) return;
    applyTraceFilters();
    applyRouteFilters();
  });

  // --- File handling ---
  async function handleFiles(files: FileList | File[]) {
    error = null;
    const fileArray = Array.from(files);
    const total = fileArray.length;
    if (total === 0) return;

    // Show progress IMMEDIATELY so the user gets feedback the moment
    // they confirm the file picker — before any parsing/IDB work runs.
    importProgress = { current: 0, total };
    // Yield once so Svelte renders the overlay before we start the
    // synchronous-ish parse loop.
    await new Promise((r) => setTimeout(r, 0));

    console.time('[tracematch] import:total');

    // Read + parse files in parallel within each batch. Sequential
    // `await file.text()` over 400+ files was the dominant cost.
    // Promise.all lets the browser overlap the disk reads while we
    // bulk-insert each batch.
    const batchSize = 50;
    let completed = 0;

    for (let i = 0; i < fileArray.length; i += batchSize) {
      const end = Math.min(i + batchSize, fileArray.length);
      const slice = fileArray.slice(i, end);

      const perFile = await Promise.all(
        slice.map(async (file): Promise<{ err: string | null; traces: StoredTrace[] }> => {
          if (!file.name.endsWith('.gpx')) {
            completed++;
            importProgress = { current: completed, total };
            return { err: `Skipped ${file.name} — only GPX files are supported currently`, traces: [] };
          }
          try {
            const text = await file.text();
            const parsed = parseGpx(text);
            const traces: StoredTrace[] = parsed.map((trace) => ({
              id: crypto.randomUUID(),
              name: trace.name,
              fileName: file.name,
              points: trace.points,
              distance: trace.distance,
              sportType: trace.sportType,
              addedAt: Date.now()
            }));
            completed++;
            importProgress = { current: completed, total };
            return { err: null, traces };
          } catch (e) {
            completed++;
            importProgress = { current: completed, total };
            return { err: `Failed to parse ${file.name}: ${e}`, traces: [] };
          }
        })
      );

      const batch: StoredTrace[] = [];
      for (const r of perFile) {
        if (r.err) error = r.err;
        if (r.traces.length > 0) batch.push(...r.traces);
      }
      if (batch.length > 0) {
        await traceStore.addTraces(batch);
      }
      await new Promise((r) => setTimeout(r, 0));
    }

    console.timeEnd('[tracematch] import:total');
    console.log(`[tracematch] imported ${total} files, ${traceStore.traces.length} traces total`);

    importProgress = null;
  }

  function handleDrop(e: DragEvent) {
    e.preventDefault();
    dragOver = false;
    if (e.dataTransfer?.files) handleFiles(e.dataTransfer.files);
  }

  function handleDragOver(e: DragEvent) {
    e.preventDefault();
    dragOver = true;
  }

  function handleFileInput(e: Event) {
    const input = e.target as HTMLInputElement;
    if (input.files) handleFiles(input.files);
  }

  // --- Analysis ---
  async function runAnalysis() {
    if (!wasmReady || traceStore.traces.length < 2) return;
    analysing = true;
    error = null;
    sectionError = null;
    analysisProgress = { phase: 'Preparing', current: 0, total: traceStore.traces.length };

    try {
      const sectionConfig = JSON.stringify({
        proximityThreshold,
        minSectionLength,
        maxSectionLength: 5000,
        minActivities,
        clusterTolerance: 80,
        samplePoints: 50,
        detectionMode: 'discovery',
        includePotentials: true,
        scalePresets: [
          { name: 'short', minLength: 100, maxLength: 500, minActivities: 2 },
          { name: 'medium', minLength: 500, maxLength: 2000, minActivities: 2 },
          { name: 'long', minLength: 2000, maxLength: 5000, minActivities: 3 }
        ],
        preserveHierarchy: true,
        minRoutes
      });

      const traces = traceStore.traces.map((t) => ({
        id: t.id,
        points: t.points.map((p) => ({ latitude: p.latitude, longitude: p.longitude, elevation: p.elevation })),
        sportType: t.sportType ?? 'Ride'
      }));

      const result = await runAnalysisAsync(traces, sectionConfig, (phase, current, total) => {
        analysisProgress = { phase, current, total };
      });

      if (result.sections.length === 0 && traceStore.traces.length >= 3) {
        sectionError = 'No sections detected with current settings.';
      }

      analysisProgress = { phase: 'Saving results', current: 0, total: 1 };
      await traceStore.saveAnalysis({
        signatures: result.signatures,
        groups: result.groups,
        sections: result.sections,
        analyzedAt: Date.now()
      });
      // Show 100% briefly so the user sees a completed bar before it
      // disappears, instead of jumping from ~83% back to nothing.
      analysisProgress = { phase: 'Saving results', current: 1, total: 1 };
      await new Promise((r) => setTimeout(r, 250));
    } catch (e) {
      error = `Analysis failed: ${e}`;
    } finally {
      analysing = false;
      analysisProgress = null;
    }
  }
</script>

<svelte:head>
  <title>tracematch — GPS route analysis</title>
  <link
    rel="stylesheet"
    href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    crossorigin=""
  />
</svelte:head>

<div class="app">
  <header>
    <h1>tracematch</h1>
    <button class="theme-toggle" onclick={() => {
      themeMode = themeMode === 'system' ? 'light' : themeMode === 'light' ? 'dark' : 'system';
    }} title="Theme: {themeMode}">
      {#if themeMode === 'system'}
        <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="4"/><path d="M12 2v2m0 16v2M4.93 4.93l1.41 1.41m11.32 11.32l1.41 1.41M2 12h2m16 0h2M6.34 17.66l-1.41 1.41m12.73-12.73l1.41-1.41"/></svg>
      {:else if themeMode === 'light'}
        <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="5"/><path d="M12 1v2m0 18v2M4.22 4.22l1.42 1.42m12.72 12.72l1.42 1.42M1 12h2m18 0h2M4.22 19.78l1.42-1.42M18.36 5.64l1.42-1.42"/></svg>
      {:else}
        <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2"><path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"/></svg>
      {/if}
    </button>
    <div
      class="header-drop"
      class:drag-over={dragOver}
      ondrop={handleDrop}
      ondragover={handleDragOver}
      ondragleave={() => (dragOver = false)}
      role="button"
      tabindex="0"
    >
      <label class="header-drop-label">
        <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/></svg>
        <span>Add GPX files</span>
        <input type="file" accept=".gpx" multiple onchange={handleFileInput} />
      </label>
    </div>
    <div class="header-spacer"></div>
    {#if traceStore.traces.length > 0}
      <span class="header-count">{traceStore.traces.length} traces</span>
      <button
        class="header-clear"
        onclick={() => {
          if (confirm(`Clear all ${traceStore.traces.length} traces and analysis?`)) {
            void traceStore.clearAll();
          }
        }}
        title="Clear all traces and analysis"
      >
        <svg viewBox="0 0 24 24" width="14" height="14" fill="none" stroke="currentColor" stroke-width="2"><polyline points="3 6 5 6 21 6"/><path d="M19 6l-2 14a2 2 0 0 1-2 2H9a2 2 0 0 1-2-2L5 6"/><path d="M10 11v6M14 11v6"/><path d="M8 6V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/></svg>
        <span>Clear all</span>
      </button>
    {/if}
  </header>

  <div class="content">
    <div class="sidebar">
      {#if error}
        <div class="error">{error}</div>
      {/if}

      {#if hasAnalysis}
        <!-- Post-analysis sidebar -->
        {@const analysis = traceStore.analysis!}

        <!-- Summary bar -->
        <div class="summary-bar">
          {traceStore.traces.length} traces &middot; {analysis.groups.length} routes &middot; {analysis.sections.length} sections
        </div>

        <!-- Sport breakdown / filter -->
        <div class="sport-breakdown">
          {#each Object.entries(sportBreakdown) as [sport, counts]}
            {@const active = sportFilters.has(sport)}
            <button
              class="sport-row"
              class:inactive={!active}
              onclick={() => toggleSportFilter(sport)}
            >
              <span class="sport-dot" style="background: {sportColor(sport)}"></span>
              <span class="sport-name">{sport}</span>
              <span class="sport-stat">{counts.traces} traces</span>
              {#if counts.routes > 0}
                <span class="sport-stat">{counts.routes} routes</span>
              {/if}
            </button>
          {/each}
        </div>

        <!-- Top Routes -->
        {#if sortedRoutes.length > 0}
          <div class="result-section">
            <div class="section-title-row">
              <h2>Routes</h2>
              <span class="section-count">{analysis.groups.length}</span>
            </div>
            {#each (showAllRoutes ? sortedRoutes : sortedRoutes.slice(0, 5)) as group, i (group.groupId)}
              {@const isHl = highlightedRouteGroupId === group.groupId}
              <button
                class="list-row"
                class:highlighted={isHl}
                onmouseenter={() => onRouteHover(group)}
                onmouseleave={() => onRouteLeave()}
                onclick={() => zoomToGroup(group)}
              >
                <span class="color-swatch" style="background: {sportColor(group.sportType)}"></span>
                <span class="row-name">{routeIndexLabel(group)}</span>
                <span class="row-count">{group.activityIds.length}&times;</span>
                <span class="row-dist">{(traceDistanceById(group.representativeId) / 1000).toFixed(1)} km</span>
              </button>
            {/each}
            {#if sortedRoutes.length > 5}
              <button class="show-all-btn" onclick={() => showAllRoutes = !showAllRoutes}>
                {showAllRoutes ? 'Show less' : `Show all ${sortedRoutes.length}`}
              </button>
            {/if}
          </div>
        {/if}

        <!-- Top Sections -->
        {#if sectionError}
          <div class="error">{sectionError}</div>
        {/if}

        {#if sortedSections.length > 0}
          <div class="result-section">
            <div class="section-title-row">
              <h2>Sections</h2>
              <span class="section-count">{analysis.sections.length}</span>
            </div>
            {#each (showAllSections ? sortedSections : sortedSections.slice(0, 5)) as section (section.id)}
              {@const isHl = highlightedSectionId === section.id}
              <button
                class="list-row"
                class:highlighted={isHl}
                onmouseenter={() => onSectionHover(section)}
                onmouseleave={() => onSectionLeave()}
                onclick={() => zoomToSection(section)}
              >
                <span class="color-swatch" style="background: {SECTION_COLOR}"></span>
                <span class="row-name">{sectionIndexLabel(section)}</span>
                <span class="row-count">{section.visitCount}&times;</span>
                <span class="row-dist">{(section.distanceMeters / 1000).toFixed(1)} km</span>
              </button>
            {/each}
            {#if sortedSections.length > 5}
              <button class="show-all-btn" onclick={() => showAllSections = !showAllSections}>
                {showAllSections ? 'Show less' : `Show all ${sortedSections.length}`}
              </button>
            {/if}
          </div>
        {:else if traceStore.traces.length >= 3 && !sectionError}
          <div class="result-section">
            <div class="section-title-row"><h2>Sections</h2></div>
            <p class="no-results">No frequent sections detected.</p>
          </div>
        {/if}

        <!-- Settings + re-analyse -->
        <div class="result-section">
          <button class="panel-toggle" onclick={() => settingsCollapsed = !settingsCollapsed}>
            <span class="panel-chevron">{settingsCollapsed ? '▸' : '▾'}</span>
            <h2>Settings</h2>
          </button>
          {#if !settingsCollapsed}
            <div class="settings-panel">
              <label class="slider-row">
                <span class="slider-label">Proximity threshold</span>
                <span class="slider-value">{proximityThreshold} m</span>
                <input type="range" min="10" max="150" step="5" bind:value={proximityThreshold} />
                <span class="slider-hint">Max distance between tracks to overlap</span>
              </label>
              <label class="slider-row">
                <span class="slider-label">Min section length</span>
                <span class="slider-value">{minSectionLength} m</span>
                <input type="range" min="50" max="2000" step="50" bind:value={minSectionLength} />
                <span class="slider-hint">Shortest section to detect</span>
              </label>
              <label class="slider-row">
                <span class="slider-label">Min activities</span>
                <span class="slider-value">{minActivities}</span>
                <input type="range" min="2" max="10" step="1" bind:value={minActivities} />
                <span class="slider-hint">Activities needed to form a section</span>
              </label>
              <label class="slider-row">
                <span class="slider-label">Min distinct routes</span>
                <span class="slider-value">{minRoutes}</span>
                <input type="range" min="2" max="6" step="1" bind:value={minRoutes} />
                <span class="slider-hint">How many different routes must overlap</span>
              </label>
            </div>
          {/if}
        </div>
        <button class="btn-analyse" onclick={runAnalysis} disabled={analysing || !wasmReady}>
          {#if analysing && analysisProgress}
            <div class="progress-content">
              <span>{analysisProgress.phase}</span>
              {#if analysisProgress.total > 1}
                <span class="progress-count">{analysisProgress.current}/{analysisProgress.total}</span>
              {/if}
            </div>
            <div class="progress-bar progress-bar-step">
              <div class="progress-fill" style="width: {analysisProgress.total > 1 ? Math.min(analysisProgress.current / analysisProgress.total * 100, 100) : 50}%"></div>
            </div>
            <div class="progress-overall-row">
              <span>Overall</span>
              <span class="progress-count">{Math.round(overallProgress)}%</span>
            </div>
            <div class="progress-bar progress-bar-overall">
              <div class="progress-fill" style="width: {overallProgress}%"></div>
            </div>
          {:else if analysing}
            Analysing...
          {:else}
            Re-analyse
          {/if}
        </button>

      {:else}
        <!-- Pre-analysis sidebar -->
        {#if traceStore.traces.length > 0}
          <div class="summary-bar">
            {traceStore.traces.length} traces loaded
          </div>

          <!-- Sport breakdown -->
          <div class="sport-breakdown">
            {#each Object.entries(sportBreakdown) as [sport, counts]}
              <div class="sport-row">
                <span class="sport-dot" style="background: {sportColor(sport)}"></span>
                <span class="sport-name">{sport}</span>
                <span class="sport-stat">{counts.traces} traces</span>
              </div>
            {/each}
          </div>
        {/if}

        <!-- File drop zone -->
        <div
          class="drop-zone"
          class:drag-over={dragOver}
          ondrop={handleDrop}
          ondragover={handleDragOver}
          ondragleave={() => (dragOver = false)}
          role="button"
          tabindex="0"
        >
          <svg viewBox="0 0 24 24" width="24" height="24" fill="none" stroke="currentColor" stroke-width="1.5"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/><polyline points="17 8 12 3 7 8"/><line x1="12" y1="3" x2="12" y2="15"/></svg>
          <span>Drop GPX files or click to browse</span>
          <input type="file" accept=".gpx" multiple onchange={handleFileInput} />
        </div>

        {#if traceStore.traces.length >= 2}
          <div class="result-section">
            <button class="panel-toggle" onclick={() => settingsCollapsed = !settingsCollapsed}>
              <span class="panel-chevron">{settingsCollapsed ? '▸' : '▾'}</span>
              <h2>Settings</h2>
            </button>
            {#if !settingsCollapsed}
              <div class="settings-panel">
                <label class="slider-row">
                  <span class="slider-label">Proximity threshold</span>
                  <span class="slider-value">{proximityThreshold} m</span>
                  <input type="range" min="10" max="150" step="5" bind:value={proximityThreshold} />
                  <span class="slider-hint">Max distance between tracks to overlap</span>
                </label>
                <label class="slider-row">
                  <span class="slider-label">Min section length</span>
                  <span class="slider-value">{minSectionLength} m</span>
                  <input type="range" min="50" max="2000" step="50" bind:value={minSectionLength} />
                  <span class="slider-hint">Shortest section to detect</span>
                </label>
                <label class="slider-row">
                  <span class="slider-label">Min activities</span>
                  <span class="slider-value">{minActivities}</span>
                  <input type="range" min="2" max="10" step="1" bind:value={minActivities} />
                  <span class="slider-hint">Activities needed to form a section</span>
                </label>
                <label class="slider-row">
                  <span class="slider-label">Min distinct routes</span>
                  <span class="slider-value">{minRoutes}</span>
                  <input type="range" min="2" max="6" step="1" bind:value={minRoutes} />
                  <span class="slider-hint">How many different routes must overlap</span>
                </label>
              </div>
            {/if}
          </div>
          <button class="btn-analyse" onclick={runAnalysis} disabled={analysing || !wasmReady}>
            {#if analysing && analysisProgress}
              <div class="progress-content">
                <span>{analysisProgress.phase}</span>
                {#if analysisProgress.total > 1}
                  <span class="progress-count">{analysisProgress.current}/{analysisProgress.total}</span>
                {/if}
              </div>
              <div class="progress-bar progress-bar-step">
                <div class="progress-fill" style="width: {analysisProgress.total > 1 ? Math.min(analysisProgress.current / analysisProgress.total * 100, 100) : 50}%"></div>
              </div>
              <div class="progress-overall-row">
                <span>Overall</span>
                <span class="progress-count">{Math.round(overallProgress)}%</span>
              </div>
              <div class="progress-bar progress-bar-overall">
                <div class="progress-fill" style="width: {overallProgress}%"></div>
              </div>
            {:else if analysing}
              Analysing...
            {:else}
              Analyse routes
            {/if}
          </button>
        {/if}
      {/if}

      <footer class="sidebar-footer">
        <p>
          Powered by <a href="https://github.com/evanjt/tracematch" target="_blank">tracematch</a>
          — all computation runs locally in your browser via WebAssembly. No data leaves your device.
        </p>
      </footer>
    </div>

    <div
      class="map-panel"
      ondrop={(e) => {
        e.preventDefault();
        mapDragOver = false;
        if (e.dataTransfer?.files) handleFiles(e.dataTransfer.files);
      }}
      ondragover={(e) => {
        e.preventDefault();
        mapDragOver = true;
      }}
      ondragleave={() => (mapDragOver = false)}
      role="region"
    >
      <div class="map-container" bind:this={mapContainer}></div>

      <!-- Layer control bar (top-right) -->
      {#if traceStore.traces.length > 0}
        <div class="layer-control">
          <button
            class="layer-btn"
            class:active={showTraces}
            onclick={() => showTraces = !showTraces}
            title="Traces"
          >
            <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2"><path d="M22 12h-4l-3 9L9 3l-3 9H2"/></svg>
          </button>
          <button
            class="layer-btn"
            class:active={showRoutes}
            onclick={() => showRoutes = !showRoutes}
            title="Routes"
          >
            <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><path d="M2 12h20"/><path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z"/></svg>
          </button>
          <button
            class="layer-btn"
            class:active={showSections}
            onclick={() => showSections = !showSections}
            title="Sections"
          >
            <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2"><path d="M4 15s1-1 4-1 5 2 8 2 4-1 4-1V3s-1 1-4 1-5-2-8-2-4 1-4 1z"/><line x1="4" y1="22" x2="4" y2="15"/></svg>
          </button>
          <div class="layer-divider"></div>
          {#each [['Ride', SPORT_COLORS.Ride], ['Run', SPORT_COLORS.Run], ['Walk', SPORT_COLORS.Walk], ['Swim', SPORT_COLORS.Swim], ['Other', DEFAULT_SPORT_COLOR]] as [sport, color]}
            <button
              class="sport-filter-dot"
              class:active={sportFilters.has(sport)}
              style="--dot-color: {color}"
              onclick={() => toggleSportFilter(sport)}
              title="{sport}"
            ></button>
          {/each}
        </div>
      {/if}

      <!-- Compact legend (bottom-left) -->
      {#if traceStore.traces.length > 0}
        <div class="compact-legend">
          <div class="legend-grid">
            <span class="legend-line" style="background: {SPORT_COLORS.Ride}"></span><span class="legend-text">Ride</span>
            <span class="legend-line" style="background: {SPORT_COLORS.Run}"></span><span class="legend-text">Run</span>
            <span class="legend-line" style="background: {SPORT_COLORS.Walk}"></span><span class="legend-text">Walk</span>
            <span class="legend-line" style="background: {SPORT_COLORS.Swim}"></span><span class="legend-text">Swim</span>
            {#if hasAnalysis}
              <span class="legend-line" style="background: {SECTION_COLOR}"></span><span class="legend-text">Section</span>
            {/if}
          </div>
        </div>
      {/if}

      {#if mapDragOver}
        <div class="map-drop-overlay">Drop GPX files here</div>
      {:else if traceStore.loading && traceStore.loadProgress}
        <div class="map-overlay">
          <div class="overlay-status">
            Loading traces {traceStore.loadProgress.current}/{traceStore.loadProgress.total}
            <div class="overlay-bar">
              <div class="overlay-fill" style="width: {traceStore.loadProgress.current / traceStore.loadProgress.total * 100}%"></div>
            </div>
          </div>
        </div>
      {:else if importProgress}
        <div class="map-overlay">
          <div class="overlay-status">
            Importing {importProgress.current}/{importProgress.total} files
            <div class="overlay-bar">
              <div class="overlay-fill" style="width: {importProgress.current / importProgress.total * 100}%"></div>
            </div>
          </div>
        </div>
      {:else if renderProgress}
        <div class="map-overlay">
          <div class="overlay-status">
            Rendering {renderProgress.current}/{renderProgress.total} traces
            <div class="overlay-bar">
              <div class="overlay-fill" style="width: {renderProgress.current / renderProgress.total * 100}%"></div>
            </div>
          </div>
        </div>
      {:else if !wasmReady && !error}
        <div class="map-overlay">Loading WASM engine...</div>
      {:else if traceStore.traces.length === 0}
        <div class="map-overlay">Drop GPX files anywhere or use the sidebar</div>
      {/if}
    </div>
  </div>
</div>

<style>
  /* --- CSS-class hover system --- */
  :global(.leaflet-overlay-pane.dimmed path) {
    opacity: 0.06 !important;
    transition: opacity 0.12s;
  }
  :global(.leaflet-overlay-pane.dimmed path.hl) {
    opacity: 1 !important;
  }

  /* --- Layout --- */
  .app {
    height: 100vh;
    display: flex;
    flex-direction: column;
  }

  header {
    padding: 8px 16px;
    background: var(--card);
    border-bottom: 1px solid var(--border);
    display: flex;
    align-items: center;
    gap: 12px;
  }

  header h1 {
    margin: 0;
    font-size: 18px;
    font-weight: 700;
    color: var(--text);
    flex-shrink: 0;
  }

  .theme-toggle {
    background: none;
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 4px 6px;
    cursor: pointer;
    color: var(--text-muted);
    display: flex;
    align-items: center;
    transition: all 0.15s;
  }
  .theme-toggle:hover {
    color: var(--primary);
    border-color: var(--primary);
    background: var(--primary-subtle);
  }

  .header-drop {
    border: 1.5px dashed var(--border);
    border-radius: var(--radius);
    padding: 5px 14px;
    transition: all 0.15s;
    flex-shrink: 0;
  }
  .header-drop:hover,
  .header-drop.drag-over {
    border-color: var(--primary);
    background: color-mix(in srgb, var(--primary) 8%, transparent);
  }

  .header-drop-label {
    display: flex;
    align-items: center;
    gap: 6px;
    cursor: pointer;
    font-size: 13px;
    color: var(--text-muted);
    font-weight: 500;
  }
  .header-drop-label:hover { color: var(--primary); }
  .header-drop-label input { display: none; }

  .header-spacer { flex: 1; }

  .header-count {
    font-size: 12px;
    color: var(--text-muted);
    flex-shrink: 0;
  }

  .header-clear {
    display: flex;
    align-items: center;
    gap: 4px;
    background: none;
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 4px 8px;
    cursor: pointer;
    color: var(--text-muted);
    font-size: 12px;
    transition: all 0.15s;
    flex-shrink: 0;
  }
  .header-clear:hover {
    color: #ef4444;
    border-color: #ef4444;
    background: color-mix(in srgb, #ef4444 8%, transparent);
  }

  .content {
    flex: 1;
    display: flex;
    overflow: hidden;
  }

  /* --- Sidebar --- */
  .sidebar {
    width: 340px;
    background: var(--bg-elevated);
    border-right: 1px solid var(--border);
    display: flex;
    flex-direction: column;
    overflow-y: auto;
    padding: 12px 14px;
    gap: 0;
  }

  .error {
    background: color-mix(in srgb, #b91c1c 10%, var(--card));
    color: #ef4444;
    padding: 8px 12px;
    border-radius: 6px;
    font-size: 13px;
    word-break: break-word;
    margin-bottom: 8px;
  }

  .summary-bar {
    font-size: 13px;
    color: var(--text-muted);
    padding: 8px 0;
    border-bottom: 1px solid var(--border);
    margin-bottom: 8px;
    font-weight: 500;
  }

  /* --- Sport breakdown --- */
  .sport-breakdown {
    display: flex;
    flex-direction: column;
    gap: 1px;
    margin-bottom: 12px;
  }

  .sport-row {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 5px 6px;
    border-radius: 6px;
    font-size: 12px;
    color: var(--text);
    background: none;
    border: none;
    cursor: pointer;
    font: inherit;
    text-align: left;
    width: 100%;
    transition: opacity 0.12s;
  }
  .sport-row:hover { background: var(--highlight); }
  .sport-row.inactive { opacity: 0.35; }

  .sport-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    flex-shrink: 0;
  }

  .sport-name {
    font-weight: 500;
    min-width: 40px;
  }

  .sport-stat {
    font-size: 11px;
    color: var(--text-muted);
    margin-left: auto;
  }
  .sport-stat + .sport-stat {
    margin-left: 8px;
  }

  /* --- Result sections --- */
  .result-section {
    margin-bottom: 12px;
  }

  .section-title-row {
    display: flex;
    align-items: center;
    gap: 6px;
    margin-bottom: 6px;
  }

  .section-title-row h2 {
    margin: 0;
    font-size: 12px;
    font-weight: 600;
    color: var(--text-muted);
    text-transform: uppercase;
    letter-spacing: 0.5px;
  }

  .section-count {
    font-size: 11px;
    color: var(--text-dim);
    background: var(--highlight);
    padding: 1px 6px;
    border-radius: 8px;
  }

  /* --- List rows (routes + sections) --- */
  .list-row {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 6px 8px;
    border-radius: 6px;
    background: none;
    border: none;
    cursor: pointer;
    font: inherit;
    color: var(--text);
    font-size: 13px;
    text-align: left;
    width: 100%;
    transition: background 0.1s;
  }
  .list-row:hover,
  .list-row.highlighted {
    background: var(--highlight);
  }

  .color-swatch {
    width: 14px;
    height: 4px;
    border-radius: 2px;
    flex-shrink: 0;
  }

  .row-name {
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    flex: 1;
    min-width: 0;
  }

  .row-count {
    font-size: 12px;
    color: var(--text-muted);
    font-variant-numeric: tabular-nums;
    flex-shrink: 0;
  }

  .row-dist {
    font-size: 11px;
    color: var(--text-dim);
    flex-shrink: 0;
    font-variant-numeric: tabular-nums;
  }

  .show-all-btn {
    background: none;
    border: none;
    cursor: pointer;
    font: inherit;
    font-size: 12px;
    color: var(--primary);
    padding: 4px 8px;
    width: 100%;
    text-align: left;
  }
  .show-all-btn:hover { text-decoration: underline; }

  .no-results {
    font-size: 13px;
    color: var(--text-muted);
    margin: 0;
    font-style: italic;
  }

  /* --- Drop zone (pre-analysis) --- */
  .drop-zone {
    border: 1.5px dashed var(--border);
    border-radius: var(--radius);
    padding: 24px;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 8px;
    color: var(--text-muted);
    font-size: 13px;
    cursor: pointer;
    transition: all 0.15s;
    margin-bottom: 12px;
    position: relative;
  }
  .drop-zone:hover, .drop-zone.drag-over {
    border-color: var(--primary);
    background: color-mix(in srgb, var(--primary) 6%, transparent);
    color: var(--primary);
  }
  .drop-zone input {
    position: absolute;
    inset: 0;
    opacity: 0;
    cursor: pointer;
  }

  /* --- Settings + analyse --- */
  .panel-toggle {
    display: flex;
    align-items: center;
    gap: 6px;
    background: none;
    border: none;
    cursor: pointer;
    padding: 4px 0;
    margin-bottom: 6px;
    width: 100%;
    text-align: left;
    font: inherit;
    color: var(--text);
  }
  .panel-toggle:hover h2 { color: var(--text); }

  .panel-toggle h2 {
    margin: 0;
    font-size: 12px;
    font-weight: 600;
    color: var(--text-muted);
    text-transform: uppercase;
    letter-spacing: 0.5px;
  }

  .panel-chevron {
    font-size: 11px;
    color: var(--text-muted);
    width: 12px;
  }

  .settings-panel {
    display: flex;
    flex-direction: column;
    gap: 10px;
    margin-bottom: 8px;
  }

  .slider-row {
    display: grid;
    grid-template-columns: 1fr auto;
    grid-template-rows: auto auto auto;
    gap: 2px 8px;
    font-size: 12px;
    cursor: default;
  }
  .slider-label { color: var(--text); font-weight: 500; }
  .slider-value { color: var(--text-muted); text-align: right; font-variant-numeric: tabular-nums; }
  .slider-row input[type='range'] {
    grid-column: 1 / -1;
    width: 100%;
    height: 4px;
    accent-color: var(--primary);
    cursor: pointer;
  }
  .slider-hint {
    grid-column: 1 / -1;
    font-size: 11px;
    color: var(--text-muted);
    opacity: 0.7;
  }

  .btn-analyse {
    padding: 12px;
    background: var(--primary);
    color: white;
    border: none;
    border-radius: var(--radius);
    font-size: 15px;
    font-weight: 600;
    cursor: pointer;
    transition: background 0.15s;
    overflow: hidden;
    position: relative;
    margin-bottom: 12px;
  }
  .btn-analyse:hover:not(:disabled) { background: var(--primary-dark); }
  .btn-analyse:disabled { opacity: 0.6; cursor: not-allowed; }

  .progress-content {
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 13px;
    margin-bottom: 6px;
  }
  .progress-count { opacity: 0.8; font-size: 12px; font-variant-numeric: tabular-nums; }
  .progress-bar {
    height: 3px;
    background: rgba(255, 255, 255, 0.3);
    border-radius: 2px;
    overflow: hidden;
  }
  .progress-bar-step .progress-fill {
    background: rgba(255, 255, 255, 0.6);
  }
  .progress-overall-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 11px;
    margin-top: 8px;
    margin-bottom: 4px;
    opacity: 0.85;
  }
  .progress-bar-overall {
    height: 5px;
  }
  .progress-fill {
    height: 100%;
    background: white;
    border-radius: 2px;
    transition: width 0.2s ease-out;
  }

  .sidebar-footer {
    margin-top: auto;
    padding-top: 16px;
    border-top: 1px solid var(--border);
    font-size: 12px;
    color: var(--text-muted);
    line-height: 1.5;
  }
  .sidebar-footer a { color: var(--primary); }

  /* --- Map panel --- */
  .map-panel {
    flex: 1;
    position: relative;
  }

  .map-container {
    width: 100%;
    height: 100%;
  }

  /* --- Layer control (top-right) --- */
  .layer-control {
    position: absolute;
    top: 10px;
    right: 10px;
    z-index: 1000;
    display: flex;
    flex-direction: column;
    gap: 2px;
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 4px;
    box-shadow: var(--shadow-lg);
  }

  .layer-btn {
    width: 34px;
    height: 34px;
    display: flex;
    align-items: center;
    justify-content: center;
    border: none;
    border-radius: 6px;
    cursor: pointer;
    background: transparent;
    color: var(--text-muted);
    transition: all 0.12s;
  }
  .layer-btn:hover { background: var(--highlight); color: var(--text); }
  .layer-btn.active { background: var(--primary-subtle); color: var(--primary); }

  .layer-divider {
    height: 1px;
    background: var(--border);
    margin: 2px 4px;
  }

  .sport-filter-dot {
    width: 34px;
    height: 20px;
    display: flex;
    align-items: center;
    justify-content: center;
    border: none;
    border-radius: 4px;
    cursor: pointer;
    background: transparent;
    padding: 0;
    transition: opacity 0.12s;
  }
  .sport-filter-dot::after {
    content: '';
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: var(--dot-color);
    opacity: 0.3;
    transition: opacity 0.12s;
  }
  .sport-filter-dot.active::after { opacity: 1; }
  .sport-filter-dot:hover::after { opacity: 0.8; }

  /* --- Compact legend (bottom-left) --- */
  .compact-legend {
    position: absolute;
    bottom: 24px;
    left: 10px;
    z-index: 1000;
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 6px 10px;
    box-shadow: var(--shadow);
  }

  .legend-grid {
    display: grid;
    grid-template-columns: 16px auto 16px auto;
    gap: 3px 6px;
    align-items: center;
  }

  .legend-line {
    width: 14px;
    height: 3px;
    border-radius: 2px;
  }

  .legend-text {
    font-size: 11px;
    color: var(--text-muted);
    white-space: nowrap;
  }

  /* --- Map overlays --- */
  .map-overlay {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    background: var(--card);
    padding: 16px 24px;
    border-radius: var(--radius);
    font-size: 14px;
    color: var(--text-muted);
    pointer-events: none;
    z-index: 1000;
    box-shadow: var(--shadow-lg);
  }

  .overlay-status {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 8px;
    min-width: 180px;
  }

  .overlay-bar {
    width: 100%;
    height: 3px;
    background: var(--border);
    border-radius: 2px;
    overflow: hidden;
  }

  .overlay-fill {
    height: 100%;
    background: var(--primary);
    border-radius: 2px;
    transition: width 0.15s ease-out;
  }

  .map-drop-overlay {
    position: absolute;
    inset: 0;
    background: color-mix(in srgb, var(--primary) 8%, transparent);
    border: 3px dashed var(--primary);
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 18px;
    font-weight: 600;
    color: var(--primary);
    z-index: 1000;
    pointer-events: none;
  }
</style>
