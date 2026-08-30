/**
 * Stream individual GeoLife trajectories out of the published archive
 * without downloading it.
 *
 * The archive is a 352 MB zip. Its central directory is 2.3 MB and sits
 * at the end, so one range request yields an index of all 18,670
 * trajectories, and each trajectory is then a second range request over
 * its own few kilobytes. A slider over "how many activities" therefore
 * controls the download, not just what is kept: the newest 200 cost
 * about 5 MB.
 *
 * The mirror is used because download.microsoft.com sends no
 * `Access-Control-Allow-Origin`, so a browser cannot read it. Nothing is
 * redistributed here: the bytes go straight from the host to the
 * visitor. If the mirror disappears, `fetchIndex` throws and the caller
 * falls back to dropping the zip in by hand.
 *
 * Dataset: Microsoft GeoLife GPS Trajectories 1.3 (Zheng et al.),
 * 182 users, April 2007 to August 2012.
 */

export const GEOLIFE_URL = 'https://ndownloader.figshare.com/files/45571758';

/** One trajectory's location inside the archive. */
export interface GeoLifeEntry {
  /** Path inside the zip, e.g. `Data/000/Trajectory/20081023025304.plt`. */
  path: string;
  /** The `000`-style user directory. */
  user: string;
  /** Start time parsed from the filename stem, epoch milliseconds. */
  startedAt: number;
  /** Byte offset of the local file header. */
  offset: number;
  /** Compressed size, so the caller can total a selection up front. */
  compressedSize: number;
  /** 0 = stored, 8 = deflate. Nothing else appears in this archive. */
  method: number;
}

export interface GeoLifeTrack {
  entry: GeoLifeEntry;
  points: { latitude: number; longitude: number; elevation?: number }[];
}

async function range(url: string, start: number, end: number): Promise<Uint8Array> {
  const res = await fetch(url, { headers: { Range: `bytes=${start}-${end}` } });
  if (!res.ok) throw new Error(`range request failed: HTTP ${res.status}`);
  return new Uint8Array(await res.arrayBuffer());
}

async function totalSize(url: string): Promise<number> {
  // Getting the archive's length from a browser is fiddlier than it
  // looks, and all three obvious routes are closed. `Content-Range` is
  // unreadable: the host redirects to storage that sends no
  // `Access-Control-Expose-Headers`, so only the CORS-safelisted
  // response headers survive, and `Content-Range` is not one of them.
  // `HEAD` fails outright. A suffix range (`bytes=-70000`) fails too.
  //
  // `Content-Length` IS safelisted. So ask for the whole file, read the
  // length off the headers, and abort before any of the body arrives.
  // fetch resolves as soon as the headers land, so this costs one round
  // trip and no payload.
  const controller = new AbortController();
  try {
    const res = await fetch(url, { headers: { Range: 'bytes=0-' }, signal: controller.signal });
    if (!res.ok) throw new Error(`size probe failed: HTTP ${res.status}`);
    const total = Number(res.headers.get('content-length'));
    if (!Number.isFinite(total) || total <= 0) throw new Error('server reports no length');
    return total;
  } finally {
    controller.abort();
  }
}

/** `20081023025304` -> epoch ms. Returns 0 when the stem is not a date. */
function stemToEpoch(stem: string): number {
  if (!/^\d{14}$/.test(stem)) return 0;
  const n = (a: number, b: number) => Number(stem.slice(a, b));
  return Date.UTC(n(0, 4), n(4, 6) - 1, n(6, 8), n(8, 10), n(10, 12), n(12, 14));
}

/**
 * Read the archive's central directory and return every trajectory,
 * newest first. Downloads about 2.3 MB and no trajectory data.
 */
export async function fetchIndex(url: string = GEOLIFE_URL): Promise<GeoLifeEntry[]> {
  const size = await totalSize(url);

  // The end-of-central-directory record is within the last 64 KB unless
  // the archive carries a comment, which this one does not.
  const tailLen = Math.min(size, 70_000);
  const tail = await range(url, size - tailLen, size - 1);

  let eocd = -1;
  for (let i = tail.length - 22; i >= 0; i--) {
    if (tail[i] === 0x50 && tail[i + 1] === 0x4b && tail[i + 2] === 0x05 && tail[i + 3] === 0x06) {
      eocd = i;
      break;
    }
  }
  if (eocd === -1) throw new Error('not a zip archive: no end-of-central-directory record');

  const view = new DataView(tail.buffer, tail.byteOffset, tail.byteLength);
  const cdSize = view.getUint32(eocd + 12, true);
  const cdOffset = view.getUint32(eocd + 16, true);

  const cd = await range(url, cdOffset, cdOffset + cdSize - 1);
  const cdView = new DataView(cd.buffer, cd.byteOffset, cd.byteLength);
  const decoder = new TextDecoder();

  const entries: GeoLifeEntry[] = [];
  let off = 0;
  while (off + 46 <= cd.length && cdView.getUint32(off, true) === 0x02014b50) {
    const method = cdView.getUint16(off + 10, true);
    const compressedSize = cdView.getUint32(off + 20, true);
    const nameLen = cdView.getUint16(off + 28, true);
    const extraLen = cdView.getUint16(off + 30, true);
    const commentLen = cdView.getUint16(off + 32, true);
    const localOffset = cdView.getUint32(off + 42, true);
    const path = decoder.decode(cd.subarray(off + 46, off + 46 + nameLen));

    if (path.toLowerCase().endsWith('.plt')) {
      const segments = path.split('/');
      const stem = segments[segments.length - 1].replace(/\.plt$/i, '');
      entries.push({
        path,
        user: segments.length > 1 ? segments[1] : '?',
        startedAt: stemToEpoch(stem),
        offset: localOffset,
        compressedSize,
        method,
      });
    }
    off += 46 + nameLen + extraLen + commentLen;
  }

  if (entries.length === 0) throw new Error('archive holds no .plt trajectories');
  entries.sort((a, b) => b.startedAt - a.startedAt);
  return entries;
}

/** Total bytes a selection will pull, so the slider can show the cost. */
export function downloadBytes(entries: GeoLifeEntry[]): number {
  return entries.reduce((n, e) => n + e.compressedSize, 0);
}

async function inflateRaw(data: Uint8Array): Promise<Uint8Array> {
  const stream = new Blob([data as BlobPart]).stream().pipeThrough(new DecompressionStream('deflate-raw'));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

/**
 * A GeoLife `.plt`: six header lines, then
 * `lat,lng,0,altitude_feet,days_since_1899,date,time` per point.
 */
export function parsePlt(text: string): { latitude: number; longitude: number; elevation?: number }[] {
  const points: { latitude: number; longitude: number; elevation?: number }[] = [];
  const lines = text.split('\n');
  for (let i = 6; i < lines.length; i++) {
    const line = lines[i];
    if (!line) continue;
    const parts = line.split(',');
    if (parts.length < 4) continue;
    const latitude = Number(parts[0]);
    const longitude = Number(parts[1]);
    if (!Number.isFinite(latitude) || !Number.isFinite(longitude)) continue;
    // Altitude is in feet, and -777 is the dataset's "unknown".
    const feet = Number(parts[3]);
    const elevation = Number.isFinite(feet) && feet !== -777 ? feet * 0.3048 : undefined;
    points.push({ latitude, longitude, elevation });
  }
  return points;
}

/** Fetch and decode one trajectory. Two range requests, a few KB. */
export async function fetchTrack(entry: GeoLifeEntry, url: string = GEOLIFE_URL): Promise<GeoLifeTrack> {
  // The local header repeats the name and extra lengths, and they can
  // differ from the central directory's, so read it rather than assume.
  const header = await range(url, entry.offset, entry.offset + 29);
  const hv = new DataView(header.buffer, header.byteOffset, header.byteLength);
  const nameLen = hv.getUint16(26, true);
  const extraLen = hv.getUint16(28, true);

  const start = entry.offset + 30 + nameLen + extraLen;
  const raw = await range(url, start, start + entry.compressedSize - 1);
  const bytes = entry.method === 0 ? raw : await inflateRaw(raw);
  return { entry, points: parsePlt(new TextDecoder().decode(bytes)) };
}

/**
 * Fetch `count` trajectories, newest first, a few at a time.
 *
 * `onProgress` fires after each one so the caller can show a count. A
 * trajectory that fails to decode is skipped rather than failing the
 * batch: one bad entry should not lose the other 199.
 */
export async function fetchNewest(
  entries: GeoLifeEntry[],
  count: number,
  onProgress?: (done: number, total: number) => void,
  url: string = GEOLIFE_URL,
): Promise<GeoLifeTrack[]> {
  const wanted = entries.slice(0, Math.max(0, count));
  const tracks: GeoLifeTrack[] = [];
  const concurrency = 6;

  for (let i = 0; i < wanted.length; i += concurrency) {
    const batch = wanted.slice(i, i + concurrency);
    const settled = await Promise.allSettled(batch.map((e) => fetchTrack(e, url)));
    for (const r of settled) {
      if (r.status === 'fulfilled' && r.value.points.length > 1) tracks.push(r.value);
    }
    onProgress?.(Math.min(i + concurrency, wanted.length), wanted.length);
  }
  return tracks;
}

/** Great-circle length of a track in metres, matching the GPX parser. */
export function trackDistance(points: { latitude: number; longitude: number }[]): number {
  let total = 0;
  for (let i = 1; i < points.length; i++) {
    const a1 = (points[i - 1].latitude * Math.PI) / 180;
    const a2 = (points[i].latitude * Math.PI) / 180;
    const dLat = a2 - a1;
    const dLng = ((points[i].longitude - points[i - 1].longitude) * Math.PI) / 180;
    const h =
      Math.sin(dLat / 2) ** 2 + Math.cos(a1) * Math.cos(a2) * Math.sin(dLng / 2) ** 2;
    total += 6371000 * 2 * Math.atan2(Math.sqrt(h), Math.sqrt(1 - h));
  }
  return total;
}
