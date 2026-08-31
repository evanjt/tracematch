import { describe, expect, it } from 'vitest';
import { traceId } from './traceId';
import type { GpsPoint } from '$lib/wasm/types';

function line(n: number, lat = 46.0): GpsPoint[] {
  return Array.from({ length: n }, (_, i) => ({
    latitude: lat + i * 0.0001,
    longitude: 7.0,
    elevation: 1000 + i,
  }));
}

describe('traceId', () => {
  it('gives one ride one id however many times the file is read', () => {
    expect(traceId(line(50))).toBe(traceId(line(50)));
  });

  it('prefixes the id so its origin reads off the key', () => {
    expect(traceId(line(50))).toMatch(/^gpx:[0-9a-f]{16}$/);
  });

  it('separates two rides that differ by one metre', () => {
    const moved = line(50);
    moved[25] = { ...moved[25], longitude: 7.00001 };
    expect(traceId(moved)).not.toBe(traceId(line(50)));
  });

  it('separates a ride from the same ride with a point dropped', () => {
    expect(traceId(line(49))).not.toBe(traceId(line(50)));
  });

  it('separates a ride from the same ground with no elevation', () => {
    const flat = line(50).map(({ latitude, longitude }) => ({ latitude, longitude }));
    expect(traceId(flat)).not.toBe(traceId(line(50)));
  });

  it('separates an out-and-back from its reverse', () => {
    expect(traceId(line(50).slice().reverse())).not.toBe(traceId(line(50)));
  });

  it('reads identity off the geometry alone, so a renamed export dedups', () => {
    // The id takes points only: a trace renamed between two exports is the
    // same ride and must collapse onto one activity.
    expect(traceId(line(50))).toBe(traceId(line(50).map((p) => ({ ...p }))));
  });

  it('still names a trace whose points did not parse', () => {
    expect(traceId([])).toMatch(/^gpx:[0-9a-f]{16}$/);
    expect(traceId([])).not.toBe(traceId(line(1)));
  });
});
