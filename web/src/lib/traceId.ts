import type { GpsPoint } from '$lib/wasm/types';

/**
 * A trace's id, derived from the ground it covers.
 *
 * The importer used to mint a fresh UUID per `<trk>` block, so the same file
 * read twice became two unrelated activities. The support floors count
 * activities, so a duplicate reads as an independent visit and can lift a
 * section over `minActivities` on one outing's ground. Taking the id from the
 * points instead makes a re-import replace rather than add, which is how the
 * GeoLife path has always behaved with its stable archive path.
 *
 * Geometry alone, not the name or the file it arrived in: the same ride
 * exported twice under two names is one ride. Order counts, so an
 * out-and-back is not its own reverse. Elevation counts, so a trace
 * re-exported with its profile is a different record from the flat one.
 */
export function traceId(points: GpsPoint[]): string {
  // Two independent FNV-1a streams over the same bytes, concatenated for
  // 64 bits. Not a cryptographic hash: this is identity, not integrity, and
  // 64 bits leaves a corpus of tens of thousands far clear of a collision.
  let a = 0x811c9dc5;
  let b = 0x7fffffff;
  const feed = (s: string) => {
    for (let i = 0; i < s.length; i++) {
      const c = s.charCodeAt(i);
      a = Math.imul(a ^ c, 0x01000193);
      b = Math.imul(b ^ c, 0x01000193);
    }
  };
  for (const p of points) {
    feed(String(p.latitude));
    feed(',');
    feed(String(p.longitude));
    feed(',');
    // An absent elevation is its own value, distinct from any number, so a
    // flat re-export never lands on the profiled trace's key.
    feed(p.elevation === undefined || p.elevation === null ? '~' : String(p.elevation));
    feed(';');
  }
  const hex = (n: number) => (n >>> 0).toString(16).padStart(8, '0');
  return `gpx:${hex(a)}${hex(b)}`;
}
