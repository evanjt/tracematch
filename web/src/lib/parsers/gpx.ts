import type { GpsPoint } from '$lib/wasm/types';

export interface ParsedTrace {
  name: string;
  points: GpsPoint[];
  distance: number;
  sportType: string;
}

const SPORT_MAP: Record<string, string> = {
  running: 'Run',
  run: 'Run',
  trail_running: 'Run',
  '9': 'Run',
  '10': 'Run',
  cycling: 'Ride',
  biking: 'Ride',
  ride: 'Ride',
  '1': 'Ride',
  '2': 'Ride',
  '3': 'Ride',
  walking: 'Walk',
  walk: 'Walk',
  hiking: 'Walk',
  hike: 'Walk',
  '6': 'Walk',
  '11': 'Walk',
  swimming: 'Swim',
  swim: 'Swim',
  '5': 'Swim',
  '12': 'Swim'
};

function inferSport(name: string): string {
  const lower = name.toLowerCase();
  if (lower.includes('run')) return 'Run';
  if (lower.includes('walk') || lower.includes('hike')) return 'Walk';
  if (lower.includes('swim')) return 'Swim';
  return 'Ride';
}

// Pre-compiled regexes (constructing them in the parse hot loop costs measurable time over 400+ files).
const TRK_OPEN_RE = /<trk(\s|>)/;
const TRK_CLOSE_RE = /<\/trk>/;
const NAME_RE = /<name[^>]*>([\s\S]*?)<\/name>/;
const TYPE_RE = /<type[^>]*>([\s\S]*?)<\/type>/;
// Captures lat, lon, optional ele. Tolerant of attribute order and any
// quote style. We don't bother with full XML validity — this is
// strictly the format we know users export.
const TRKPT_RE = /<trkpt\s+[^>]*?lat\s*=\s*["']([\-\d.]+)["'][^>]*?lon\s*=\s*["']([\-\d.]+)["'][^>]*?>([\s\S]*?)<\/trkpt>|<trkpt\s+[^>]*?lon\s*=\s*["']([\-\d.]+)["'][^>]*?lat\s*=\s*["']([\-\d.]+)["'][^>]*?>([\s\S]*?)<\/trkpt>|<trkpt\s+[^>]*?lat\s*=\s*["']([\-\d.]+)["'][^>]*?lon\s*=\s*["']([\-\d.]+)["'][^>]*?\/>|<trkpt\s+[^>]*?lon\s*=\s*["']([\-\d.]+)["'][^>]*?lat\s*=\s*["']([\-\d.]+)["'][^>]*?\/>/g;
const ELE_RE = /<ele[^>]*>([\-\d.]+)<\/ele>/;

/**
 * Fast regex-based GPX parser. Skips the DOMParser + @tmcw/togeojson
 * round-trip which dominated import time on large datasets (Sion's 428
 * runs went from ~3 minutes to a few seconds). Handles the structure
 * intervals.icu / Garmin / Strava export: one or more `<trk>` blocks
 * with `<name>`, optional `<type>`, and any number of `<trkseg>` /
 * `<trkpt>` children.
 */
export function parseGpx(xmlString: string): ParsedTrace[] {
  const traces: ParsedTrace[] = [];

  // Split into <trk>...</trk> blocks. We iterate manually rather than
  // using a single split() so we can skip non-trk content cheaply.
  let cursor = 0;
  while (cursor < xmlString.length) {
    const openMatch = TRK_OPEN_RE.exec(xmlString.slice(cursor));
    if (!openMatch) break;
    const trkStart = cursor + (openMatch.index ?? 0);
    const closeOffset = xmlString.indexOf('</trk>', trkStart);
    if (closeOffset === -1) break;
    const trkEnd = closeOffset + '</trk>'.length;
    const trkBlock = xmlString.slice(trkStart, trkEnd);
    cursor = trkEnd;

    const nameMatch = NAME_RE.exec(trkBlock);
    const name = nameMatch ? nameMatch[1].trim() : 'Unnamed trace';

    const typeMatch = TYPE_RE.exec(trkBlock);
    const rawType = typeMatch ? typeMatch[1].trim().toLowerCase() : '';
    const sportType = SPORT_MAP[rawType] ?? inferSport(name);

    const points: GpsPoint[] = [];
    let distance = 0;
    let prevLat = 0;
    let prevLng = 0;
    let hasPrev = false;

    TRKPT_RE.lastIndex = 0;
    let m: RegExpExecArray | null;
    while ((m = TRKPT_RE.exec(trkBlock)) !== null) {
      let lat: number, lng: number, body: string | undefined;
      if (m[1] !== undefined) {
        lat = parseFloat(m[1]);
        lng = parseFloat(m[2]);
        body = m[3];
      } else if (m[4] !== undefined) {
        lng = parseFloat(m[4]);
        lat = parseFloat(m[5]);
        body = m[6];
      } else if (m[7] !== undefined) {
        lat = parseFloat(m[7]);
        lng = parseFloat(m[8]);
        body = undefined;
      } else {
        lng = parseFloat(m[9]);
        lat = parseFloat(m[10]);
        body = undefined;
      }

      let elevation: number | undefined;
      if (body) {
        const eleMatch = ELE_RE.exec(body);
        if (eleMatch) elevation = parseFloat(eleMatch[1]);
      }

      points.push({ latitude: lat, longitude: lng, elevation });

      // Inline haversine — cheap relative to regex matching, no function call overhead.
      if (hasPrev) {
        const dLat = ((lat - prevLat) * Math.PI) / 180;
        const dLon = ((lng - prevLng) * Math.PI) / 180;
        const a =
          Math.sin(dLat / 2) ** 2 +
          Math.cos((prevLat * Math.PI) / 180) *
            Math.cos((lat * Math.PI) / 180) *
            Math.sin(dLon / 2) ** 2;
        distance += 6371000 * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
      }
      prevLat = lat;
      prevLng = lng;
      hasPrev = true;
    }

    if (points.length >= 2) {
      traces.push({ name, points, distance, sportType });
    }
  }

  // Fallback: if no <trk> blocks but the file has <trkpt> at top level
  // (some apps export this way), parse all trkpts as a single track.
  if (traces.length === 0 && TRK_CLOSE_RE.exec(xmlString) === null && TRKPT_RE.exec(xmlString)) {
    const points: GpsPoint[] = [];
    let distance = 0;
    let prevLat = 0;
    let prevLng = 0;
    let hasPrev = false;
    TRKPT_RE.lastIndex = 0;
    let m: RegExpExecArray | null;
    while ((m = TRKPT_RE.exec(xmlString)) !== null) {
      let lat: number, lng: number;
      if (m[1] !== undefined) {
        lat = parseFloat(m[1]);
        lng = parseFloat(m[2]);
      } else if (m[4] !== undefined) {
        lng = parseFloat(m[4]);
        lat = parseFloat(m[5]);
      } else if (m[7] !== undefined) {
        lat = parseFloat(m[7]);
        lng = parseFloat(m[8]);
      } else {
        lng = parseFloat(m[9]);
        lat = parseFloat(m[10]);
      }
      points.push({ latitude: lat, longitude: lng });
      if (hasPrev) {
        const dLat = ((lat - prevLat) * Math.PI) / 180;
        const dLon = ((lng - prevLng) * Math.PI) / 180;
        const a =
          Math.sin(dLat / 2) ** 2 +
          Math.cos((prevLat * Math.PI) / 180) *
            Math.cos((lat * Math.PI) / 180) *
            Math.sin(dLon / 2) ** 2;
        distance += 6371000 * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
      }
      prevLat = lat;
      prevLng = lng;
      hasPrev = true;
    }
    if (points.length >= 2) {
      traces.push({ name: 'Unnamed trace', points, distance, sportType: 'Ride' });
    }
  }

  return traces;
}
