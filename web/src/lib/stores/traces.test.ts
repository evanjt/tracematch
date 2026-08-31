import 'fake-indexeddb/auto';
import { get as idbGet, keys as idbKeys, set as idbSet } from 'idb-keyval';
import { beforeEach, describe, expect, it } from 'vitest';
import { traceId } from '$lib/traceId';
import { ANALYSIS_SCHEMA_VERSION, TRACE_SCHEMA_VERSION, traceStore } from './traces.svelte';
import type { StoredTrace } from './traces.svelte';
import type { GpsPoint } from '$lib/wasm/types';

function line(n: number, lat = 46.0): GpsPoint[] {
  return Array.from({ length: n }, (_, i) => ({
    latitude: lat + i * 0.0001,
    longitude: 7.0,
    elevation: 1000 + i,
  }));
}

function trace(id: string, points: GpsPoint[], name = 'Ride'): StoredTrace {
  return {
    id,
    name,
    fileName: `${name}.gpx`,
    points,
    distance: 1000,
    sportType: 'Ride',
    addedAt: 1,
  };
}

async function wipe() {
  for (const key of await idbKeys()) await import('idb-keyval').then((m) => m.del(key));
  traceStore.traces = [];
  traceStore.analysis = null;
}

describe('the trace store', () => {
  beforeEach(wipe);

  it('collapses a re-import onto the one trace it already holds', async () => {
    const points = line(40);
    await traceStore.addTraces([trace(traceId(points), points)]);
    await traceStore.addTraces([trace(traceId(points), points, 'Ride renamed')]);

    expect(traceStore.traces).toHaveLength(1);
    const stored = (await idbKeys()).filter(
      (k) => typeof k === 'string' && k.startsWith('trace:')
    );
    expect(stored).toHaveLength(1);
  });

  it('keeps the newer record when a re-import replaces one', async () => {
    const points = line(40);
    await traceStore.addTraces([trace(traceId(points), points, 'first')]);
    await traceStore.addTraces([trace(traceId(points), points, 'second')]);

    expect(traceStore.traces).toHaveLength(1);
    expect(traceStore.traces[0].name).toBe('second');
    expect((await idbGet(`trace:${traceId(points)}`)).name).toBe('second');
  });

  it('re-keys a trace a previous build stored under a random id', async () => {
    const points = line(40);
    await idbSet('trace:9d1f0c3e-uuid', trace('9d1f0c3e-uuid', points));

    await traceStore.load();

    expect(traceStore.traces.map((t) => t.id)).toEqual([traceId(points)]);
    expect(await idbGet('trace:9d1f0c3e-uuid')).toBeUndefined();
    expect(await idbGet(`trace:${traceId(points)}`)).toBeDefined();
  });

  it('folds the duplicates a random id let in', async () => {
    const points = line(40);
    await idbSet('trace:uuid-a', trace('uuid-a', points, 'first import'));
    await idbSet('trace:uuid-b', trace('uuid-b', points, 'second import'));

    await traceStore.load();

    expect(traceStore.traces).toHaveLength(1);
    const stored = (await idbKeys()).filter(
      (k) => typeof k === 'string' && k.startsWith('trace:')
    );
    expect(stored).toEqual([`trace:${traceId(points)}`]);
  });

  it('leaves a GeoLife trace on the stable key it already had', async () => {
    const points = line(40);
    await idbSet('trace:geolife:001/Trajectory/x.plt', trace('geolife:001/Trajectory/x.plt', points));

    await traceStore.load();

    expect(traceStore.traces.map((t) => t.id)).toEqual(['geolife:001/Trajectory/x.plt']);
  });

  it('drops a catalogue cut over the ids it just replaced', async () => {
    const points = line(40);
    await idbSet('trace:uuid-a', trace('uuid-a', points));
    await idbSet('analysis', {
      signatures: [],
      groups: [],
      sections: [],
      boundaries: [],
      analyzedAt: 1,
      schemaVersion: ANALYSIS_SCHEMA_VERSION,
    });

    await traceStore.load();

    expect(traceStore.analysis).toBeNull();
    expect(await idbGet('analysis')).toBeUndefined();
  });

  it('runs once, so a store already on the current version is left alone', async () => {
    const points = line(40);
    await idbSet(`trace:${traceId(points)}`, trace(traceId(points), points));
    await idbSet('traceSchemaVersion', TRACE_SCHEMA_VERSION);
    await idbSet('analysis', {
      signatures: [],
      groups: [],
      sections: [],
      boundaries: [],
      analyzedAt: 1,
      schemaVersion: ANALYSIS_SCHEMA_VERSION,
    });

    await traceStore.load();

    expect(traceStore.traces).toHaveLength(1);
    expect(traceStore.analysis).not.toBeNull();
  });

  it('stamps the version so the second load has nothing to do', async () => {
    const points = line(40);
    await idbSet('trace:uuid-a', trace('uuid-a', points));

    await traceStore.load();
    expect(await idbGet('traceSchemaVersion')).toBe(TRACE_SCHEMA_VERSION);

    await idbSet('analysis', {
      signatures: [],
      groups: [],
      sections: [],
      boundaries: [],
      analyzedAt: 1,
      schemaVersion: ANALYSIS_SCHEMA_VERSION,
    });
    await traceStore.load();
    expect(traceStore.analysis).not.toBeNull();
  });
});
