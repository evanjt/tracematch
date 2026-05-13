<script lang="ts">
  import { onMount } from 'svelte';
  import { traceStore, type StoredTrace } from '$lib/stores/traces.svelte';
  import { parseGpx } from '$lib/parsers/gpx';
  import {
    initWasm,
    createSignature,
    groupRoutes,
    detectSections
  } from '$lib/wasm/engine';
  import type { RouteSignature, RouteGroup, FrequentSection, GpsPoint, SectionPortion } from '$lib/wasm/types';

  let wasmReady = $state(false);
  let analysing = $state(false);
  let analysisProgress = $state<{ phase: string; current: number; total: number } | null>(null);

  // Section detection parameters
  let proximityThreshold = $state(50);
  let minSectionLength = $state(200);
  let minActivities = $state(3);
  let settingsCollapsed = $state(true);
  let error = $state<string | null>(null);
  let sectionError = $state<string | null>(null);
  let dragOver = $state(false);
  let mapDragOver = $state(false);
  let mapContainer: HTMLDivElement;
  let map: L.Map | null = null;
  let traceLayerGroup: L.LayerGroup | null = null;
  let sectionLayerGroup: L.LayerGroup | null = null;
  let hiddenTraces = $state(new Set<string>());
  let expandedSections = $state(new Set<string>());
  let tracesCollapsed = $state(true);
  let groupsCollapsed = $state(true);
  let sectionsCollapsed = $state(true);
  let highlightedTrace = $state<string | null>(null);
  let highlightedGroup = $state<{ ids: string[]; portions?: SectionPortion[] } | null>(null);
  let portionOverlays: L.Polyline[] = [];
  let hasFitOnce = false;
  let lastAnalyzedAt: number | null = null;
  let tracePolylines = new Map<string, { polyline: L.Polyline; baseStyle: { color: string; weight: number; opacity: number; dashArray?: string } }>();
  let sectionLayers: { activityIds: string[]; layers: L.Layer[] }[] = [];
  let L: typeof import('leaflet') | null = null;

  // Theme state
  let themeMode = $state<'system' | 'light' | 'dark'>(
    typeof localStorage !== 'undefined' ? (localStorage.getItem('theme') as any) || 'system' : 'system'
  );
  let resolvedDark = $state(false);
  let tileLayer: L.TileLayer | null = null;

  // Display strategy state
  let visibleGroups = $state(new Set<string>());
  let showUngrouped = $state(true);

  const GROUP_COLORS = [
    '#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4',
    '#42d4f4', '#f032e6', '#bfef45', '#fabed4', '#469990',
    '#dcbeff', '#9A6324', '#800000', '#aaffc3', '#808000'
  ];

  const TRACE_GRAY = '#64748b';

  function toggleSectionExpand(id: string) {
    const next = new Set(expandedSections);
    if (next.has(id)) next.delete(id);
    else next.add(id);
    expandedSections = next;
  }

  function zoomToTrace(traceId: string) {
    const trace = traceStore.traces.find((t) => t.id === traceId);
    if (!trace || !map || !L) return;
    const latlngs = trace.points.map((p) => L!.latLng(p.latitude, p.longitude));
    if (latlngs.length > 0) {
      map.fitBounds(L.latLngBounds(latlngs), { padding: [60, 60] });
    }
    if (hiddenTraces.has(traceId)) toggleTrace(traceId);
  }

  function zoomToSection(section: FrequentSection) {
    if (!map || !L) return;
    const latlngs = section.polyline.map((p) => L!.latLng(p.latitude, p.longitude));
    if (latlngs.length > 0) {
      map.fitBounds(L.latLngBounds(latlngs), { padding: [60, 60] });
    }
  }

  function traceDistanceById(id: string): number {
    return traceStore.traces.find((t) => t.id === id)?.distance ?? 0;
  }

  function groupForTraceId(id: string): { index: number; group: RouteGroup } | null {
    const analysis = traceStore.analysis;
    if (!analysis) return null;
    for (let i = 0; i < analysis.groups.length; i++) {
      if (analysis.groups[i].activityIds.includes(id)) {
        return { index: i, group: analysis.groups[i] };
      }
    }
    return null;
  }

  function sectionsForTraceId(id: string): { index: number; section: FrequentSection }[] {
    const analysis = traceStore.analysis;
    if (!analysis) return [];
    const result: { index: number; section: FrequentSection }[] = [];
    for (let i = 0; i < analysis.sections.length; i++) {
      if (analysis.sections[i].activityIds.includes(id)) {
        result.push({ index: i, section: analysis.sections[i] });
      }
    }
    return result;
  }

  function toggleTrace(id: string) {
    const next = new Set(hiddenTraces);
    if (next.has(id)) next.delete(id);
    else next.add(id);
    hiddenTraces = next;
  }

  function toggleGroup(activityIds: string[]) {
    const allHidden = activityIds.every((id) => hiddenTraces.has(id));
    const next = new Set(hiddenTraces);
    for (const id of activityIds) {
      if (allHidden) next.delete(id);
      else next.add(id);
    }
    hiddenTraces = next;
  }

  function traceNameById(id: string): string {
    return traceStore.traces.find((t) => t.id === id)?.name ?? id.slice(0, 8);
  }

  function getTraceStyle(traceId: string): { color: string; weight: number; opacity: number; dashArray?: string } {
    const analysis = traceStore.analysis;
    if (!analysis?.groups) {
      return { color: TRACE_GRAY, weight: 2, opacity: 0.6 };
    }
    for (let i = 0; i < analysis.groups.length; i++) {
      if (analysis.groups[i].activityIds.includes(traceId)) {
        const groupId = analysis.groups[i].groupId;
        if (!visibleGroups.has(groupId)) {
          return { color: GROUP_COLORS[i % GROUP_COLORS.length], weight: 3, opacity: 0 };
        }
        const isRepresentative = analysis.groups[i].representativeId === traceId;
        if (isRepresentative) {
          return {
            color: GROUP_COLORS[i % GROUP_COLORS.length],
            weight: 3,
            opacity: 0.75
          };
        }
        return {
          color: GROUP_COLORS[i % GROUP_COLORS.length],
          weight: 3,
          opacity: 0
        };
      }
    }
    // Ungrouped trace
    if (showUngrouped) {
      return { color: TRACE_GRAY, weight: 2, opacity: 0.4, dashArray: '6 4' };
    }
    return { color: TRACE_GRAY, weight: 2, opacity: 0 };
  }

  onMount(async () => {
    await traceStore.load();
    try {
      await initWasm();
      wasmReady = true;
    } catch (e) {
      error = `Failed to load WASM: ${e}`;
    }

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

  // Sync traces to map — incremental add/remove, no full rebuild
  function syncTracesToMap() {
    if (!map || !L || !traceLayerGroup) return;
    const traces = traceStore.traces;
    const currentIds = new Set(traces.map((t) => t.id));

    // Remove polylines for traces that no longer exist
    for (const [id, entry] of tracePolylines) {
      if (!currentIds.has(id)) {
        entry.polyline.remove();
        tracePolylines.delete(id);
      }
    }

    // Add polylines for new traces
    let addedAny = false;
    const allLatLngs: L.LatLng[] = [];
    for (const trace of traces) {
      if (tracePolylines.has(trace.id)) {
        if (!hiddenTraces.has(trace.id)) {
          const latlngs = trace.points.map((p) => L!.latLng(p.latitude, p.longitude));
          allLatLngs.push(...latlngs);
        }
        continue;
      }
      const latlngs = trace.points.map((p) => L!.latLng(p.latitude, p.longitude));
      if (latlngs.length === 0) continue;

      const traceId = trace.id;
      const baseStyle = getTraceStyle(traceId);
      const pl = L.polyline(latlngs, baseStyle)
        .bindPopup(
          `<b>${trace.name}</b><br>` +
            `${(trace.distance / 1000).toFixed(1)} km &middot; ${trace.points.length} pts`
        )
        .on('mouseover', () => { highlightedTrace = traceId; })
        .on('mouseout', () => { if (highlightedTrace === traceId) highlightedTrace = null; })
        .addTo(traceLayerGroup!);
      tracePolylines.set(traceId, { polyline: pl, baseStyle });
      addedAny = true;
      if (!hiddenTraces.has(trace.id)) {
        allLatLngs.push(...latlngs);
      }
    }

    // Fit bounds only on first batch load
    if (addedAny && !hasFitOnce && allLatLngs.length > 0) {
      hasFitOnce = true;
      map.fitBounds(L.latLngBounds(allLatLngs), { padding: [40, 40] });
    }
  }

  // Apply styles to existing polylines (used when analysis changes or traces hidden/shown)
  function applyStyles() {
    if (!traceLayerGroup) return;
    for (const [id, entry] of tracePolylines) {
      const newStyle = getTraceStyle(id);
      entry.baseStyle = newStyle;
      entry.polyline.setStyle(newStyle);
      if (hiddenTraces.has(id)) {
        entry.polyline.setStyle({ opacity: 0, weight: 0 });
      }
    }
  }

  // Rebuild section layers (only when analysis changes)
  function syncSections() {
    if (!map || !L || !sectionLayerGroup) return;
    sectionLayerGroup.clearLayers();
    sectionLayers = [];

    const analysis = traceStore.analysis;
    if (!analysis?.sections) return;

    for (const section of analysis.sections) {
      const latlngs = section.polyline.map((p: GpsPoint) =>
        L!.latLng(p.latitude, p.longitude)
      );
      if (latlngs.length < 2) continue;

      const sLayers: L.Layer[] = [];

      const border = L.polyline(latlngs, { color: '#ffffff', weight: 8, opacity: 0.8 }).addTo(sectionLayerGroup!);
      sLayers.push(border);

      const line = L.polyline(latlngs, { color: '#FC4C02', weight: 5, opacity: 0.9 })
        .bindPopup(
          `<b>${section.name || 'Detected section'}</b><br>` +
            `${(section.distanceMeters / 1000).toFixed(1)} km<br>` +
            `${section.activityIds.length} activities`
        )
        .addTo(sectionLayerGroup!);
      sLayers.push(line);

      // Only start and end markers
      const markerIcon = L.divIcon({
        className: 'section-marker',
        html: '<div style="width:10px;height:10px;border-radius:50%;background:#FC4C02;border:2px solid white;"></div>',
        iconSize: [14, 14],
        iconAnchor: [7, 7]
      });
      const m1 = L.marker(latlngs[0], { icon: markerIcon }).addTo(sectionLayerGroup!);
      const m2 = L.marker(latlngs[latlngs.length - 1], { icon: markerIcon }).addTo(sectionLayerGroup!);
      sLayers.push(m1, m2);

      sectionLayers.push({ activityIds: section.activityIds, layers: sLayers });
    }
  }

  // Initialize map
  $effect(() => {
    if (!mapContainer || typeof window === 'undefined') return;
    if (map) return; // Already initialized

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
      sectionLayerGroup = L.layerGroup().addTo(map);

      // Initial sync if traces were already loaded
      syncTracesToMap();
      applyStyles();
      syncSections();
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

  // React to trace list changes — incremental sync
  $effect(() => {
    traceStore.traces;
    if (!map || !L) return;
    syncTracesToMap();
  });

  // React to analysis changes — restyle traces, rebuild sections, populate visibleGroups, collapse traces
  $effect(() => {
    const analysis = traceStore.analysis;
    if (!map || !L) return;
    const analyzedAt = analysis?.analyzedAt ?? null;
    if (analyzedAt !== lastAnalyzedAt) {
      lastAnalyzedAt = analyzedAt;
      applyStyles();
      syncSections();
    }
  });

  // Populate visibleGroups when analysis completes, and auto-collapse traces panel
  $effect(() => {
    const analysis = traceStore.analysis;
    if (!analysis) return;
    visibleGroups = new Set(analysis.groups.map(g => g.groupId));
    tracesCollapsed = true;
  });

  // React to hidden traces — just restyle
  $effect(() => {
    hiddenTraces;
    if (!map || !L) return;
    applyStyles();
  });

  // React to visibleGroups / showUngrouped changes — restyle traces and update section overlays
  $effect(() => {
    visibleGroups;
    showUngrouped;
    if (!map || !L) return;
    applyStyles();

    // Update section overlay visibility based on whether their activities' groups are visible
    const analysis = traceStore.analysis;
    if (!analysis) return;
    for (const sl of sectionLayers) {
      const anyActivityVisible = sl.activityIds.some((actId) => {
        const g = groupForTraceId(actId);
        if (g) return visibleGroups.has(analysis.groups[g.index].groupId);
        return showUngrouped;
      });
      for (const layer of sl.layers) {
        const el = (layer as L.Polyline | L.Marker).getElement?.() as HTMLElement | undefined;
        if (el) {
          el.style.opacity = anyActivityVisible ? '' : '0';
        }
      }
    }
  });

  // Highlight effect — update styles on existing polylines
  $effect(() => {
    const ht = highlightedTrace;
    const hg = highlightedGroup;
    const activeIds = hg?.ids ?? (ht ? [ht] : null);
    const portions = hg?.portions ?? null;

    // Clear previous portion overlays
    for (const ol of portionOverlays) ol.remove();
    portionOverlays = [];

    for (const [id, entry] of tracePolylines) {
      if (hiddenTraces.has(id)) {
        entry.polyline.setStyle({ opacity: 0, weight: 0 });
        continue;
      }
      if (activeIds === null) {
        // No hover — revert to display strategy
        entry.polyline.setStyle(entry.baseStyle);
      } else if (activeIds.includes(id)) {
        // Check if the trace's group is visible
        const g = groupForTraceId(id);
        const analysis = traceStore.analysis;
        const groupVisible = g && analysis ? visibleGroups.has(analysis.groups[g.index].groupId) : showUngrouped;
        if (!groupVisible) {
          entry.polyline.setStyle({ opacity: 0, weight: 0 });
          continue;
        }
        if (portions) {
          entry.polyline.setStyle({ ...entry.baseStyle, opacity: 0.65, weight: 2.5 });
          const trace = traceStore.traces.find((t) => t.id === id);
          if (trace && L) {
            const matching = portions.filter((p) => p.activityId === id);
            for (const p of matching) {
              const pts = trace.points.slice(p.startIndex, p.endIndex + 1);
              if (pts.length < 2) continue;
              const latlngs = pts.map((pt) => L!.latLng(pt.latitude, pt.longitude));
              const ol = L.polyline(latlngs, {
                color: entry.baseStyle.color,
                weight: 5,
                opacity: 1
              }).addTo(traceLayerGroup!);
              ol.bringToFront();
              portionOverlays.push(ol);
            }
          }
        } else {
          entry.polyline.setStyle({ ...entry.baseStyle, weight: 5, opacity: 1 });
          entry.polyline.bringToFront();
        }
      } else {
        entry.polyline.setStyle({ ...entry.baseStyle, opacity: 0.05, weight: 1.5 });
      }
    }

    for (const sl of sectionLayers) {
      const relevant = activeIds === null || activeIds.some((id) => sl.activityIds.includes(id));
      for (const layer of sl.layers) {
        const el = (layer as L.Polyline | L.Marker).getElement?.() as HTMLElement | undefined;
        if (el) {
          el.style.opacity = relevant ? '' : '0.1';
        }
      }
    }
  });

  async function handleFiles(files: FileList | File[]) {
    error = null;
    const batch: StoredTrace[] = [];
    for (const file of files) {
      if (!file.name.endsWith('.gpx')) {
        error = `Skipped ${file.name} — only GPX files are supported currently`;
        continue;
      }
      try {
        const text = await file.text();
        const parsed = parseGpx(text);
        for (const trace of parsed) {
          batch.push({
            id: crypto.randomUUID(),
            name: trace.name,
            fileName: file.name,
            points: trace.points,
            distance: trace.distance,
            sportType: trace.sportType,
            addedAt: Date.now()
          });
        }
      } catch (e) {
        error = `Failed to parse ${file.name}: ${e}`;
      }
    }
    if (batch.length > 0) {
      await traceStore.addTraces(batch);
    }
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

  async function runAnalysis() {
    if (!wasmReady || traceStore.traces.length < 2) return;
    analysing = true;
    error = null;
    sectionError = null;
    analysisProgress = { phase: 'Creating signatures', current: 0, total: traceStore.traces.length };

    try {
      const signatures: RouteSignature[] = [];
      const tracks: [string, GpsPoint[]][] = [];
      const sportTypes: Record<string, string> = {};
      const traces = traceStore.traces;
      const batchSize = 50;

      for (let i = 0; i < traces.length; i += batchSize) {
        const end = Math.min(i + batchSize, traces.length);
        for (let j = i; j < end; j++) {
          const trace = traces[j];
          const sig = createSignature(trace.id, trace.points);
          if (sig) {
            signatures.push(sig);
            tracks.push([trace.id, trace.points]);
            sportTypes[trace.id] = trace.sportType ?? 'Ride';
          }
        }
        analysisProgress = { phase: 'Creating signatures', current: end, total: traces.length };
        await new Promise((r) => setTimeout(r, 0));
      }

      console.log(
        `[tracematch] Created ${signatures.length} signatures from ${traces.length} traces`
      );

      analysisProgress = { phase: 'Grouping routes', current: 0, total: 1 };
      await new Promise((r) => setTimeout(r, 0));
      const groups = groupRoutes(signatures);
      console.log(`[tracematch] Grouped into ${groups.length} route groups`);

      let sections: FrequentSection[] = [];
      if (tracks.length >= 3) {
        analysisProgress = { phase: 'Detecting sections', current: 0, total: 1 };
        await new Promise((r) => setTimeout(r, 0));
        console.log(`[tracematch] Detecting sections across ${tracks.length} tracks...`);
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
            preserveHierarchy: true
          });
          sections = detectSections(tracks, sportTypes, groups, sectionConfig);
          console.log(`[tracematch] Found ${sections.length} sections`);
        } catch (e) {
          console.error('[tracematch] Section detection failed:', e);
          sectionError = `Section detection failed: ${e}`;
        }
      }

      analysisProgress = { phase: 'Saving results', current: 0, total: 1 };
      await traceStore.saveAnalysis({
        signatures,
        groups,
        sections,
        analyzedAt: Date.now()
      });
    } catch (e) {
      error = `Analysis failed: ${e}`;
      console.error('[tracematch] Analysis error:', e);
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
    {/if}
  </header>

  <div class="content">
    <div class="sidebar">
      {#if error}
        <div class="error">{error}</div>
      {/if}

      {#if traceStore.traces.length > 0}
        <div class="result-section">
          <button class="panel-toggle" onclick={() => tracesCollapsed = !tracesCollapsed}>
            <span class="panel-chevron">{tracesCollapsed ? '▸' : '▾'}</span>
            <h2>Traces ({traceStore.traces.length})</h2>
            <!-- svelte-ignore a11y_no_static_element_interactions -->
            <span class="panel-action" onclick={(e) => { e.stopPropagation(); traceStore.clearAll(); }}>Clear</span>
          </button>
          {#if !tracesCollapsed}
            {#each traceStore.traces as trace (trace.id)}
              {@const traceGroup = groupForTraceId(trace.id)}
              {@const traceSections = sectionsForTraceId(trace.id)}
              {@const dotColor = traceGroup ? GROUP_COLORS[traceGroup.index % GROUP_COLORS.length] : TRACE_GRAY}
              <div
                class="group-activity"
                class:highlighted={highlightedTrace === trace.id}
                onmouseenter={() => highlightedTrace = trace.id}
                onmouseleave={() => highlightedTrace = null}
              >
                <span class="color-dot sm" style="background: {dotColor}"></span>
                <span class="activity-name">{trace.name}</span>
                {#if traceGroup}
                  <span class="assignment-tag" style="border-color: {dotColor}">G{traceGroup.index + 1}</span>
                {/if}
                {#each traceSections as ts}
                  <span class="assignment-tag section-tag">S{ts.index + 1}</span>
                {/each}
                <span class="activity-dist">{(trace.distance / 1000).toFixed(1)} km</span>
                <button
                  class="btn-remove-sm"
                  onclick={() => traceStore.removeTrace(trace.id)}
                  title="Remove trace"
                >&times;</button>
              </div>
            {/each}
          {/if}
        </div>
      {/if}

      {#if traceStore.analysis}
        <div class="results">
          <div class="result-section">
            <button class="panel-toggle" onclick={() => groupsCollapsed = !groupsCollapsed}>
              <span class="panel-chevron">{groupsCollapsed ? '▸' : '▾'}</span>
              <h2>Route groups ({traceStore.analysis.groups.length})</h2>
            </button>
            {#if !groupsCollapsed}
            {#each traceStore.analysis.groups as group, i}
              {@const groupColor = GROUP_COLORS[i % GROUP_COLORS.length]}
              {@const allHidden = group.activityIds.every((id) => hiddenTraces.has(id))}
              <div
                class="group-card"
                onmouseenter={() => highlightedGroup = { ids: group.activityIds }}
                onmouseleave={() => highlightedGroup = null}
              >
                <div class="group-header">
                  <span class="color-dot" style="background: {groupColor}"></span>
                  <span class="group-title">Group {i + 1}</span>
                  <span class="group-count">{group.activityIds.length} activities</span>
                  <button
                    class="btn-vis"
                    class:is-hidden={allHidden}
                    onclick={() => toggleGroup(group.activityIds)}
                    title={allHidden ? 'Show group' : 'Hide group'}
                  >
                    {#if allHidden}
                      <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2"><path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94"/><path d="M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19"/><line x1="1" y1="1" x2="23" y2="23"/></svg>
                    {:else}
                      <svg viewBox="0 0 24 24" width="16" height="16" fill="none" stroke="currentColor" stroke-width="2"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/><circle cx="12" cy="12" r="3"/></svg>
                    {/if}
                  </button>
                </div>
                <div class="group-activities">
                  {#each group.activityIds as actId}
                    {@const isHidden = hiddenTraces.has(actId)}
                    <div
                      class="group-activity"
                      class:dimmed={isHidden}
                      class:highlighted={highlightedTrace === actId}
                      onmouseenter={() => highlightedTrace = actId}
                      onmouseleave={() => highlightedTrace = null}
                    >
                      <span class="color-line" style="background: {groupColor}"></span>
                      <span class="activity-name">{traceNameById(actId)}</span>
                      <button
                        class="btn-vis sm"
                        class:is-hidden={isHidden}
                        onclick={() => toggleTrace(actId)}
                        title={isHidden ? 'Show on map' : 'Hide from map'}
                      >
                        {#if isHidden}
                          <svg viewBox="0 0 24 24" width="14" height="14" fill="none" stroke="currentColor" stroke-width="2"><path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94"/><path d="M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19"/><line x1="1" y1="1" x2="23" y2="23"/></svg>
                        {:else}
                          <svg viewBox="0 0 24 24" width="14" height="14" fill="none" stroke="currentColor" stroke-width="2"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/><circle cx="12" cy="12" r="3"/></svg>
                        {/if}
                      </button>
                    </div>
                  {/each}
                </div>
              </div>
            {/each}
            {/if}
          </div>

          {#if sectionError}
            <div class="error">{sectionError}</div>
          {/if}

          {#if traceStore.analysis.sections.length > 0}
            <div class="result-section">
              <button class="panel-toggle" onclick={() => sectionsCollapsed = !sectionsCollapsed}>
                <span class="panel-chevron">{sectionsCollapsed ? '▸' : '▾'}</span>
                <h2>Sections ({traceStore.analysis.sections.length})</h2>
              </button>
              {#if !sectionsCollapsed}
              {#each traceStore.analysis.sections as section, si}
                {@const isExpanded = expandedSections.has(section.id)}
                <div
                  class="section-card"
                  class:expanded={isExpanded}
                  onclick={() => { toggleSectionExpand(section.id); if (!isExpanded) zoomToSection(section); }}
                  onmouseenter={() => highlightedGroup = { ids: section.activityIds, portions: section.activityPortions }}
                  onmouseleave={() => highlightedGroup = null}
                  role="button"
                  tabindex="0"
                >
                  <div class="section-header">
                    <span class="section-dot"></span>
                    <span class="section-name">{section.name || `Section ${si + 1}`}</span>
                    <span class="section-chevron">{isExpanded ? '▾' : '▸'}</span>
                  </div>
                  <div class="section-summary">
                    {(section.distanceMeters / 1000).toFixed(1)} km &middot;
                    {section.activityIds.length} activities &middot;
                    {section.visitCount} traversals
                  </div>
                  {#if isExpanded}
                    <!-- svelte-ignore a11y_no_static_element_interactions -->
                    <div class="section-details" onclick={(e) => e.stopPropagation()}>
                      <div class="section-stats">
                        <div class="stat-row">
                          <span class="stat-label">Avg spread</span>
                          <span class="stat-value">{section.averageSpread.toFixed(1)} m</span>
                        </div>
                        <div class="stat-row">
                          <span class="stat-label">Observations</span>
                          <span class="stat-value">{section.observationCount}</span>
                        </div>
                        <div class="stat-row">
                          <span class="stat-label">Quality</span>
                          <span class="stat-value">{(section.confidence * 100).toFixed(0)}%</span>
                        </div>
                      </div>
                      <div class="section-activities-header">Activities</div>
                      <div class="section-activity-list">
                        {#each section.activityIds as actId}
                          <button
                            class="section-activity-row"
                            class:highlighted={highlightedTrace === actId}
                            onclick={() => zoomToTrace(actId)}
                            onmouseenter={() => highlightedTrace = actId}
                            onmouseleave={() => highlightedTrace = null}
                            title="Zoom to this activity"
                          >
                            <span class="activity-name">{traceNameById(actId)}</span>
                            <span class="activity-dist"
                              >{(traceDistanceById(actId) / 1000).toFixed(1)} km</span
                            >
                            <span class="zoom-icon">&#8599;</span>
                          </button>
                        {/each}
                      </div>
                    </div>
                  {/if}
                </div>
              {/each}
              {/if}
            </div>
          {:else if traceStore.traces.length >= 3 && !sectionError}
            <div class="result-section">
              <h2>Sections</h2>
              <p class="no-results">No frequent sections detected in these traces.</p>
            </div>
          {:else if traceStore.traces.length < 3}
            <div class="result-section">
              <h2>Sections</h2>
              <p class="no-results">Add at least 3 traces to detect sections.</p>
            </div>
          {/if}
        </div>
      {/if}

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
            <div class="progress-bar">
              <div class="progress-fill" style="width: {analysisProgress.total > 1 ? (analysisProgress.current / analysisProgress.total * 100) : 50}%"></div>
            </div>
          {:else if analysing}
            Analysing...
          {:else if traceStore.analysis}
            Re-analyse
          {:else}
            Analyse routes
          {/if}
        </button>
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
      {#if traceStore.analysis && traceStore.analysis.groups.length > 0}
        <div class="map-legend">
          <div class="legend-title">Routes</div>
          {#each traceStore.analysis.groups as group, i}
            {@const color = GROUP_COLORS[i % GROUP_COLORS.length]}
            {@const isVisible = visibleGroups.has(group.groupId)}
            <label class="legend-entry" class:dimmed={!isVisible}
              onmouseenter={() => highlightedGroup = { ids: group.activityIds }}
              onmouseleave={() => highlightedGroup = null}>
              <input type="checkbox" checked={isVisible}
                onchange={() => {
                  const next = new Set(visibleGroups);
                  if (next.has(group.groupId)) next.delete(group.groupId);
                  else next.add(group.groupId);
                  visibleGroups = next;
                }} />
              <span class="legend-swatch" style="background: {color}"></span>
              <span class="legend-label">{group.customName || `Route ${i + 1}`}</span>
              <span class="legend-count">{group.activityIds.length}</span>
            </label>
          {/each}
          {#if traceStore.traces.length > traceStore.analysis.groups.flatMap(g => g.activityIds).length}
            <label class="legend-entry" class:dimmed={!showUngrouped}>
              <input type="checkbox" checked={showUngrouped}
                onchange={() => showUngrouped = !showUngrouped} />
              <span class="legend-swatch ungrouped"></span>
              <span class="legend-label">Ungrouped</span>
            </label>
          {/if}
        </div>
      {/if}
      {#if mapDragOver}
        <div class="map-drop-overlay">Drop GPX files here</div>
      {/if}
      {#if !wasmReady && !error && !mapDragOver}
        <div class="map-overlay">Loading WASM engine...</div>
      {/if}
      {#if traceStore.traces.length === 0 && !mapDragOver}
        <div class="map-overlay">Drop GPX files anywhere or use the sidebar</div>
      {/if}
    </div>
  </div>
</div>

<style>
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

  .header-drop-label:hover {
    color: var(--primary);
  }

  .header-drop-label input {
    display: none;
  }

  .header-spacer {
    flex: 1;
  }

  .header-count {
    font-size: 12px;
    color: var(--text-muted);
    flex-shrink: 0;
  }

  .content {
    flex: 1;
    display: flex;
    overflow: hidden;
  }

  .sidebar {
    width: 360px;
    background: var(--bg-elevated);
    border-right: 1px solid var(--border);
    display: flex;
    flex-direction: column;
    overflow-y: auto;
    padding: 16px;
    gap: 0;
  }

  .error {
    background: color-mix(in srgb, #b91c1c 10%, var(--card));
    color: #ef4444;
    padding: 8px 12px;
    border-radius: 6px;
    font-size: 13px;
    word-break: break-word;
  }

  .panel-toggle h2 {
    margin: 0;
    font-size: 13px;
    font-weight: 600;
    color: var(--text-muted);
    text-transform: uppercase;
    letter-spacing: 0.5px;
  }

  .result-section {
    margin-bottom: 12px;
  }

  .result-section > h2 {
    margin: 0 0 6px 0;
    font-size: 13px;
    font-weight: 600;
    color: var(--text-muted);
    text-transform: uppercase;
    letter-spacing: 0.5px;
  }

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

  .panel-toggle:hover h2 {
    color: var(--text);
  }

  .panel-chevron {
    font-size: 11px;
    color: var(--text-muted);
    width: 12px;
  }

  .group-card {
    background: var(--bg);
    border-radius: 8px;
    padding: 10px 12px;
    margin-bottom: 8px;
    box-shadow: var(--shadow);
  }

  .group-card:hover {
    background: var(--card-hover);
  }

  .group-header {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-bottom: 6px;
  }

  .group-title {
    font-size: 13px;
    font-weight: 600;
  }

  .group-count {
    font-size: 12px;
    color: var(--text-muted);
    margin-left: auto;
  }

  .group-activities {
    display: flex;
    flex-direction: column;
    gap: 3px;
    padding-left: 18px;
  }

  .group-activity {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 12px;
    color: var(--text-muted);
  }

  .group-activity.dimmed {
    opacity: 0.4;
  }

  .group-activity.highlighted {
    background: var(--highlight);
    border-radius: 4px;
    margin: 0 -4px;
    padding: 1px 4px;
  }

  .btn-vis {
    background: none;
    border: none;
    cursor: pointer;
    padding: 3px 5px;
    color: var(--text-muted);
    line-height: 1;
    border-radius: 4px;
    margin-left: auto;
    display: flex;
    align-items: center;
  }

  .btn-vis:hover {
    background: var(--highlight);
    color: var(--text);
  }

  .btn-vis.is-hidden {
    color: var(--border);
  }

  .btn-vis.is-hidden:hover {
    color: var(--text-muted);
    background: var(--highlight);
  }

  .btn-vis.sm {
    padding: 2px 4px;
  }

  .color-dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    flex-shrink: 0;
  }

  .color-dot.sm {
    width: 8px;
    height: 8px;
  }

  .color-line {
    width: 12px;
    height: 3px;
    border-radius: 2px;
    flex-shrink: 0;
  }

  .section-card {
    background: var(--bg);
    border-radius: 8px;
    padding: 10px 12px;
    margin-bottom: 8px;
    cursor: pointer;
    transition: background 0.1s;
    box-shadow: var(--shadow);
  }

  .section-card:hover {
    background: var(--card-hover);
  }

  .section-card.expanded {
    background: var(--card-hover);
  }

  .section-header {
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .section-card:hover .section-name {
    color: var(--section-orange);
  }

  .section-dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: var(--section-orange);
    flex-shrink: 0;
  }

  .section-name {
    font-weight: 500;
    font-size: 13px;
    transition: color 0.1s;
  }

  .section-chevron {
    margin-left: auto;
    font-size: 12px;
    color: var(--text-muted);
  }

  .section-summary {
    font-size: 12px;
    color: var(--text-muted);
    padding-left: 18px;
    margin-top: 2px;
  }

  .section-details {
    margin-top: 8px;
    padding-top: 8px;
    border-top: 1px solid var(--border);
  }

  .section-stats {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 4px 12px;
    margin-bottom: 10px;
  }

  .stat-row {
    display: flex;
    justify-content: space-between;
    font-size: 12px;
  }

  .stat-label {
    color: var(--text-muted);
  }

  .stat-value {
    font-weight: 500;
    color: var(--text);
  }

  .section-activities-header {
    font-size: 11px;
    font-weight: 600;
    color: var(--text-muted);
    text-transform: uppercase;
    letter-spacing: 0.5px;
    margin-bottom: 4px;
  }

  .section-activity-list {
    display: flex;
    flex-direction: column;
    gap: 2px;
  }

  .section-activity-row {
    display: flex;
    align-items: center;
    gap: 6px;
    padding: 4px 6px;
    border-radius: 4px;
    background: none;
    border: none;
    cursor: pointer;
    font: inherit;
    color: var(--text-muted);
    font-size: 12px;
    text-align: left;
    width: 100%;
  }

  .section-activity-row:hover,
  .section-activity-row.highlighted {
    background: var(--highlight);
    color: var(--text);
  }

  .section-activity-row .activity-dist {
    color: var(--text-muted);
    margin-left: auto;
    font-size: 11px;
    flex-shrink: 0;
  }

  .zoom-icon {
    color: var(--text-muted);
    font-size: 11px;
    flex-shrink: 0;
    opacity: 0.5;
  }

  .section-activity-row:hover .zoom-icon {
    color: var(--section-orange);
    opacity: 1;
  }

  .no-results {
    font-size: 13px;
    color: var(--text-muted);
    margin: 0;
    font-style: italic;
  }

  .panel-action {
    margin-left: auto;
    font-size: 11px;
    color: #ef4444;
    padding: 2px 8px;
    border-radius: 3px;
  }

  .panel-action:hover {
    background: color-mix(in srgb, #ef4444 10%, var(--card));
  }

  .assignment-tag {
    font-size: 10px;
    padding: 1px 5px;
    border-radius: 3px;
    border: 1px solid var(--border);
    color: var(--text-muted);
    background: var(--card);
    flex-shrink: 0;
  }

  .assignment-tag.section-tag {
    border-color: var(--section-orange);
    color: var(--section-orange);
  }

  .btn-remove-sm {
    background: none;
    border: none;
    color: var(--border);
    font-size: 14px;
    cursor: pointer;
    padding: 0 4px;
    line-height: 1;
    flex-shrink: 0;
  }

  .btn-remove-sm:hover {
    color: #e11d48;
  }

  .activity-name {
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    max-width: 250px;
  }

  .activity-dist {
    font-size: 11px;
    color: var(--text-muted);
    flex-shrink: 0;
    margin-left: auto;
  }

  .settings-panel {
    display: flex;
    flex-direction: column;
    gap: 10px;
  }

  .slider-row {
    display: grid;
    grid-template-columns: 1fr auto;
    grid-template-rows: auto auto auto;
    gap: 2px 8px;
    font-size: 12px;
    cursor: default;
  }

  .slider-label {
    color: var(--text);
    font-weight: 500;
  }

  .slider-value {
    color: var(--text-muted);
    text-align: right;
    font-variant-numeric: tabular-nums;
  }

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
  }

  .btn-analyse:hover:not(:disabled) {
    background: var(--primary-dark);
  }

  .btn-analyse:disabled {
    opacity: 0.6;
    cursor: not-allowed;
  }

  .progress-content {
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 13px;
    margin-bottom: 6px;
  }

  .progress-count {
    opacity: 0.8;
    font-size: 12px;
    font-variant-numeric: tabular-nums;
  }

  .progress-bar {
    height: 3px;
    background: rgba(255, 255, 255, 0.3);
    border-radius: 2px;
    overflow: hidden;
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

  .sidebar-footer a {
    color: var(--primary);
  }

  .map-panel {
    flex: 1;
    position: relative;
  }

  .map-container {
    width: 100%;
    height: 100%;
  }

  .map-legend {
    position: absolute;
    bottom: 24px;
    right: 12px;
    z-index: 1000;
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 10px 12px;
    box-shadow: var(--shadow-lg);
    max-height: 300px;
    overflow-y: auto;
    min-width: 140px;
  }
  .legend-title {
    font-size: 11px;
    font-weight: 600;
    color: var(--text-dim);
    text-transform: uppercase;
    letter-spacing: 0.5px;
    margin-bottom: 6px;
  }
  .legend-entry {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 12px;
    color: var(--text);
    cursor: pointer;
    padding: 2px 0;
  }
  .legend-entry.dimmed {
    opacity: 0.4;
  }
  .legend-entry input[type="checkbox"] {
    width: 14px;
    height: 14px;
    accent-color: var(--primary);
    margin: 0;
    cursor: pointer;
  }
  .legend-swatch {
    width: 16px;
    height: 4px;
    border-radius: 2px;
    flex-shrink: 0;
  }
  .legend-swatch.ungrouped {
    background: #64748b;
    border: 1px dashed #64748b;
    height: 2px;
  }
  .legend-label {
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .legend-count {
    margin-left: auto;
    color: var(--text-dim);
    font-size: 11px;
  }

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
