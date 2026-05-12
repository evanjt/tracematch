import { gpx } from '@tmcw/togeojson';
import type { GpsPoint } from '$lib/wasm/types';

export interface ParsedTrace {
  name: string;
  points: GpsPoint[];
  distance: number;
  sportType: string;
}

function inferSport(name: string): string {
  const lower = name.toLowerCase();
  if (lower.includes('run')) return 'Run';
  if (lower.includes('walk') || lower.includes('hike')) return 'Walk';
  if (lower.includes('swim')) return 'Swim';
  return 'Ride';
}

export function parseGpx(xmlString: string): ParsedTrace[] {
  const dom = new DOMParser().parseFromString(xmlString, 'text/xml');
  const geoJson = gpx(dom);
  const traces: ParsedTrace[] = [];

  for (const feature of geoJson.features) {
    if (feature.geometry.type !== 'LineString' && feature.geometry.type !== 'MultiLineString') {
      continue;
    }

    const name = feature.properties?.name || 'Unnamed trace';
    const coords =
      feature.geometry.type === 'LineString'
        ? feature.geometry.coordinates
        : feature.geometry.coordinates.flat();

    const points: GpsPoint[] = coords.map((c: number[]) => ({
      latitude: c[1],
      longitude: c[0],
      elevation: c[2] != null ? c[2] : undefined
    }));

    if (points.length < 2) continue;

    let distance = 0;
    for (let i = 1; i < points.length; i++) {
      distance += haversine(points[i - 1], points[i]);
    }

    traces.push({ name, points, distance, sportType: inferSport(name) });
  }

  return traces;
}

function haversine(a: GpsPoint, b: GpsPoint): number {
  const R = 6371000;
  const dLat = ((b.latitude - a.latitude) * Math.PI) / 180;
  const dLon = ((b.longitude - a.longitude) * Math.PI) / 180;
  const lat1 = (a.latitude * Math.PI) / 180;
  const lat2 = (b.latitude * Math.PI) / 180;
  const h =
    Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
  return R * 2 * Math.atan2(Math.sqrt(h), Math.sqrt(1 - h));
}
