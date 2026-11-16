"""Spatial report of a section-dump move: where each changed section's line
moved and why each ride joined or left, as a self-contained HTML page with
inline SVG plots. A scratch measurement over a private corpus, not a gate.

Run, from the tracematch crate root:

    python3 tests/section_spatial.py BEFORE.jsonl AFTER-renumbered.jsonl \
        CORPUS_DIR OUT_DIR

BEFORE and AFTER are `section_dump.rs` dumps sharing one id space (renumber
AFTER with `section_compare.py` first). CORPUS_DIR holds the GPX files the
dumps were folded from, named `<activity id>_<name>.gpx` (or `<id>__<name>.gpx`
when the id holds an underscore, as GeoLife ids do). Set SECTION_PASSES_FEATURES,
SECTION_PASSES_TEST and SECTION_PASSES_CWD to probe through another test target
(`section_passes_geolife` with `public-corpus`) or another checkout. The verdict on each
ride comes from the detector itself: `section_passes.rs` is run over probe
lines (the line as drawn, the line trimmed at either end, the line with a
wider lateral tolerance, each half) and the first probe a ride passes says how
far short or how far beside the line it ran. The geometry beside it (nearest
track point to each end, widest station offset) is measured here. Only full
rides of a section count: a pass is a run of track points within GROUND_TOL_M
of the line, bridging TRACK_GAP_POINTS, with every STATION_M station of the
line within GROUND_TOL_M of the run.
"""
import html
import json
import math
import os
import re
import subprocess
import sys
from collections import Counter, defaultdict

GROUND_TOL_M = 50.0
STATION_M = GROUND_TOL_M / 5.0
TRACK_GAP_POINTS = 3
WINDOW_POINTS = 200          # track points read either side of a row
CORRIDOR_M = 300.0           # how far from the line a track is drawn
TRIMS = [10, 25, 50, 75, 100, 150, 200, 300, 400, 600, 800, 1200, 1600]
LATERALS = list(range(5, 205, 5))
GENERIC = {
    'morning', 'afternoon', 'lunch', 'evening', 'night', 'run', 'ride', 'walk',
    'walking', 'running', 'cycling', 'hike', 'hiking', 'swim', 'swimming',
    'virtual', 'workout', 'training', 'easy', 'long', 'recovery', 'and', 'the',
    'to', 'de', 'la', 'le', 'du', 'des', 'a', 'in', 'with', 'from', 'w', 'et',
}

R = 6371000.0


def hav(a, b):
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    d = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * R * math.asin(math.sqrt(d))


class Proj:
    """Equirectangular metres about a reference point."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.kx = math.radians(1) * R * math.cos(math.radians(lat0))
        self.ky = math.radians(1) * R

    def xy(self, p):
        return ((p[1] - self.lon0) * self.kx, (p[0] - self.lat0) * self.ky)


def seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def cumulative(pts):
    c = [0.0]
    for k in range(1, len(pts)):
        c.append(c[-1] + hav(pts[k - 1], pts[k]))
    return c


def cut(pts, lo_m, hi_m):
    return [p for p, d in zip(pts, cumulative(pts)) if lo_m <= d <= hi_m]


class Line:
    def __init__(self, pts, proj):
        self.pts = pts
        self.xy = [proj.xy(p) for p in pts]
        self.cum = cumulative(pts)
        self.length = self.cum[-1]
        self.stations = []
        for k in range(len(pts)):
            if k + 1 == len(pts) or not self.stations or self.cum[k] - self.cum[self.stations[-1]] >= STATION_M:
                self.stations.append(k)
        self.cell = 100.0
        self.grid = defaultdict(list)
        for i in range(len(self.xy) - 1):
            (x1, y1), (x2, y2) = self.xy[i], self.xy[i + 1]
            for cx in range(int(min(x1, x2) // self.cell) - 1, int(max(x1, x2) // self.cell) + 2):
                for cy in range(int(min(y1, y2) // self.cell) - 1, int(max(y1, y2) // self.cell) + 2):
                    self.grid[(cx, cy)].append(i)

    def dist(self, q):
        cx, cy = int(q[0] // self.cell), int(q[1] // self.cell)
        best = float('inf')
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for i in self.grid.get((cx + dx, cy + dy), ()):
                    d = seg_dist(q, self.xy[i], self.xy[i + 1])
                    if d < best:
                        best = d
        return best


def load_gpx(path):
    text = open(path, encoding='utf-8', errors='replace').read()
    pts = []
    for m in re.finditer(r'<trkpt\s+lat="([^"]+)"\s+lon="([^"]+)"(.*?)(?=<trkpt|</trkseg)', text, re.S):
        ele = re.search(r'<ele>([^<]+)</ele>', m.group(3))
        pts.append((float(m.group(1)), float(m.group(2)), float(ele.group(1)) if ele else None))
    return pts


def corpus_index(corpus):
    names, paths = {}, {}
    for f in sorted(os.listdir(corpus)):
        if not f.lower().endswith('.gpx'):
            continue
        stem = f[:-4]
        # `<id>__<name>` keeps an id that itself holds an underscore whole.
        aid = stem.split('__')[0] if '__' in stem else stem.split('_')[0]
        names[aid] = stem[len(aid) + 1:].lstrip('_') if '_' in stem else ''
        paths[aid] = os.path.join(corpus, f)
    return names, paths


def ground_runs(flags):
    """Runs of true in flags, bridging up to TRACK_GAP_POINTS false."""
    runs, start, gap, last = [], None, 0, 0
    for i, on in enumerate(flags):
        if on:
            if start is None:
                start = i
            gap = 0
            last = i
        elif start is not None:
            gap += 1
            if gap > TRACK_GAP_POINTS:
                runs.append((start, last + 1))
                start, gap = None, 0
    if start is not None:
        runs.append((start, last + 1))
    return runs


def geometry(line, proj, track, lo, hi):
    """How the track between lo and hi (widened by WINDOW_POINTS) sits on the line.

    The nearest track point to each end of the line, the widest station
    offset over the best ground run, the metres of line no run point comes
    within tolerance of, and the window's xy for drawing."""
    lo = max(0, lo - WINDOW_POINTS)
    hi = min(len(track), hi + WINDOW_POINTS)
    xy = [proj.xy(p) for p in track[lo:hi]]
    d = [line.dist(q) for q in xy]
    runs = ground_runs([x <= GROUND_TOL_M for x in d])
    start_near = min((math.hypot(line.xy[0][0] - x, line.xy[0][1] - y) for x, y in xy), default=float('nan'))
    end_near = min((math.hypot(line.xy[-1][0] - x, line.xy[-1][1] - y) for x, y in xy), default=float('nan'))
    best = None
    for s, e in runs:
        run = xy[s:e]
        offs = [min(math.hypot(line.xy[k][0] - x, line.xy[k][1] - y) for x, y in run) for k in line.stations]
        covered = [i for i, o in enumerate(offs) if o <= GROUND_TOL_M]
        if not covered:
            continue
        holes = sum(STATION_M for o in offs[covered[0]:covered[-1] + 1] if o > GROUND_TOL_M)
        cand = dict(widest=max(offs[covered[0]:covered[-1] + 1]), holes=holes,
                    first_m=line.cum[line.stations[covered[0]]], last_m=line.cum[line.stations[covered[-1]]],
                    score=len(covered), run=(lo + s, lo + e))
        if best is None or cand['score'] > best['score']:
            best = cand
    if best is None:
        best = dict(widest=float('nan'), holes=line.length, first_m=float('nan'), last_m=float('nan'), score=0, run=None)
    best.update(xy=xy, window=(lo, hi), runs=len(runs), start_near=start_near, end_near=end_near)
    return best


def line_move(before, after, proj):
    lb, la = Line(before['polyline'], proj), Line(after['polyline'], proj)
    if before['line_hash'] == after['line_hash']:
        kind = 'same'
    elif before['polyline'] == after['polyline'][::-1]:
        kind = 'reversed'
    else:
        kind = 'redrawn'
    rev = kind == 'reversed'
    a_first, a_last = (after['last'], after['first']) if rev else (after['first'], after['last'])
    start_gap = hav(before['first'], a_first)
    end_gap = hav(before['last'], a_last)
    widest = max(max(la.dist(q) for q in lb.xy), max(lb.dist(q) for q in la.xy))
    dl = after['distance_m'] - before['distance_m']
    if kind == 'same':
        how = 'unchanged: the same points on the same ground'
    elif kind == 'reversed':
        how = 'the same points, now drawn in the opposite direction, so its start and end swap'
    else:
        how = []
        if abs(dl) > 0.5:
            how.append(('extended' if dl > 0 else 'trimmed') + f' by {abs(dl):.0f} m at the ' + ('start' if start_gap > end_gap else 'end'))
        if widest > GROUND_TOL_M:
            how.append(f'shifted up to {widest:.0f} m sideways')
        how = ', '.join(how) or 'moved within GPS tolerance'
    return dict(kind=kind, start_gap=start_gap, end_gap=end_gap, widest=widest, how=how, lb=lb, la=la)


def place_name(members, names):
    c = Counter()
    for m in members:
        n = names.get(m, '')
        words = [w for w in re.split(r'[^\w]+', n.lower()) if w and w not in GENERIC and not w.isdigit() and len(w) > 2]
        for w in set(words):
            c[w] += 1
    if not c:
        return 'none (the member activity names carry no place word)'
    return ', '.join(f'{w} ({n} of {len(members)})' for w, n in c.most_common(3))


def ground(before, tracks):
    pts = before['polyline']
    straight = hav(pts[0], pts[-1])
    ratio = straight / before['distance_m'] if before['distance_m'] else 0
    dy = (pts[-1][0] - pts[0][0])
    dx = (pts[-1][1] - pts[0][1]) * math.cos(math.radians(pts[0][0]))
    bearing = (math.degrees(math.atan2(dx, dy)) + 360) % 360
    compass = 'N NE E SE S SW W NW'.split()[int((bearing + 22.5) // 45) % 8]
    shape = 'a closed loop' if straight <= GROUND_TOL_M else ('a straight run' if ratio > 0.9 else ('a winding course' if ratio < 0.5 else 'a gently curving course'))
    climb = ''
    rep = tracks.get(before['rep'])
    if rep and before['rep_range']:
        s, e = before['rep_range']
        eles = [p[2] for p in rep[s:e + 1] if p[2] is not None]
        if len(eles) > 2:
            up = sum(max(0, b - a) for a, b in zip(eles, eles[1:]))
            down = sum(max(0, a - b) for a, b in zip(eles, eles[1:]))
            climb = f', {up:.0f} m up and {down:.0f} m down along the representative trace (altitude {eles[0]:.0f} to {eles[-1]:.0f} m)'
    return f'{shape} heading {compass} ({bearing:.0f} deg), {straight:.0f} m end to end for {before["distance_m"]:.0f} m of line{climb}'


# ---------------------------------------------------------------------------
# The detector probe
# ---------------------------------------------------------------------------

def row_directions(rows, passes):
    """The direction verdict of each pin row against the fresh passes.

    rows are (start, end, direction) pin rows of one member; passes are the
    detector's (start, end, direction, metres) passes over the stored line. A
    row is judged by the pass its indices overlap most, and the verdict is
    'agree' or 'differ' on that pass's direction, or None when no pass overlaps
    it. Extent is not judged here: the pin keeps rows over the pre-clip line.
    """
    verdicts = []
    for start, end, direction in rows:
        best = None
        for fs, fe, fd, _ in passes:
            if fe < start or fs > end:
                continue
            overlap = min(end, fe) - max(start, fs)
            if best is None or overlap > best[0]:
                best = (overlap, fd)
        if best is None:
            verdicts.append(None)
        else:
            verdicts.append('agree' if best[1] == direction else 'differ')
    return verdicts


def direction_line(verdicts):
    differ = verdicts.count('differ')
    agree = verdicts.count('agree')
    unmatched = verdicts.count(None)
    return (f'direction against the fresh pass count: {differ} of {len(verdicts)} member rows differ '
            f'({agree} agree, {unmatched} with no overlapping pass)')


class Probe:
    def __init__(self):
        self.reqs = []
        self.ans = {}

    def ask(self, key, act, line, lateral=0.0, sl=None):
        self.reqs.append(dict(key=key, activity=act, line=line, lateral_m=float(lateral), slice=list(sl) if sl else None))

    def run(self, out_dir):
        req = os.path.join(out_dir, 'probe-requests.jsonl')
        ans = os.path.join(out_dir, 'probe-answers.jsonl')
        with open(req, 'w') as f:
            for r in self.reqs:
                f.write(json.dumps(r) + '\n')
        env = dict(os.environ, SECTION_PASSES_IN=req, SECTION_PASSES_OUT=ans)
        # The public corpus answers through `section_passes_geolife` with
        # `public-corpus`; another checkout answers for another detector pin.
        features = os.environ.get('SECTION_PASSES_FEATURES', 'real-corpus')
        target = os.environ.get('SECTION_PASSES_TEST', 'section_passes')
        cwd = os.environ.get('SECTION_PASSES_CWD') or None
        subprocess.run(['cargo', 'test', '--release', '--features', features, '--test', target, '--', '--nocapture'],
                       env=env, cwd=cwd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for l in open(ans):
            r = json.loads(l)
            self.ans[(r['key'], r['activity'])] = r['passes']

    def get(self, key, act):
        return self.ans.get((key, act))


def variant_keys(sid, L, tag):
    """The probe lines for one failing row, in the order they are read."""
    keys = [(f'{sid}:full:{tag}', 'full', None)]
    for d in TRIMS:
        if d < L / 2:
            keys.append((f'{sid}:ts{d}:{tag}', 'ts', d))
            keys.append((f'{sid}:te{d}:{tag}', 'te', d))
            keys.append((f'{sid}:tb{d}:{tag}', 'tb', d))
    for lat in LATERALS:
        keys.append((f'{sid}:lat{lat}:{tag}', 'lat', lat))
    keys.append((f'{sid}:h1:{tag}', 'h1', None))
    keys.append((f'{sid}:h2:{tag}', 'h2', None))
    return keys


def ask_variants(probe, sid, act, line, L, sl):
    tag = f'sl{sl[0]}-{sl[1]}'
    for key, kind, d in variant_keys(sid, L, tag):
        if kind == 'full':
            probe.ask(key, act, line, 0.0, sl)
        elif kind == 'ts':
            probe.ask(key, act, cut(line, d, L + 1), 0.0, sl)
        elif kind == 'te':
            probe.ask(key, act, cut(line, -1, L - d), 0.0, sl)
        elif kind == 'tb':
            probe.ask(key, act, cut(line, d, L - d), 0.0, sl)
        elif kind == 'lat':
            probe.ask(key, act, line, d, sl)
        elif kind == 'h1':
            probe.ask(key, act, cut(line, -1, L / 2), 0.0, sl)
        elif kind == 'h2':
            probe.ask(key, act, cut(line, L / 2, L + 1), 0.0, sl)


def read_variants(probe, sid, act, L, sl):
    """The detector's explanation of a row that does not pass the line whole."""
    tag = f'sl{sl[0]}-{sl[1]}'
    got = {}
    for key, kind, d in variant_keys(sid, L, tag):
        p = probe.get(key, act)
        if p:
            got.setdefault(kind, d)
    if 'full' in got:
        return dict(kind='alone', text='passes the line whole on its own stretch of track')
    ts, te, tb, lat = got.get('ts'), got.get('te'), got.get('tb'), got.get('lat')
    if ts is not None and te is not None:
        return dict(kind='both', ts=ts, te=te, lat=lat, text=f'counts once the line gives up {ts} m at the start or {te} m at the end')
    if ts is not None:
        return dict(kind='start', d=ts, lat=lat, text=f'counts once the line starts {ts} m later: it misses the first {ts} m')
    if te is not None:
        return dict(kind='end', d=te, lat=lat, text=f'counts once the line ends {te} m earlier: it misses the last {te} m')
    if tb is not None:
        return dict(kind='both', d=tb, lat=lat, text=f'counts once the line gives up {tb} m at each end: it misses both ends')
    if lat is not None:
        return dict(kind='beside', lat=lat, text=f'counts only with {lat} m more lateral tolerance: it runs up to about {GROUND_TOL_M + lat:.0f} m beside the line somewhere between the ends')
    if 'h1' in got and 'h2' in got:
        return dict(kind='split', text='covers each half of the line but never in one pass: it turns or leaves the line in the middle')
    if 'h1' in got:
        return dict(kind='half', text='covers only the first half of the line')
    if 'h2' in got:
        return dict(kind='half', text='covers only the second half of the line')
    return dict(kind='none', text='no pass over either half of the line, even 200 m beside it: other ground')


# ---------------------------------------------------------------------------
# The report
# ---------------------------------------------------------------------------

def svg_plot(mv, left, joined, lost_rows, gained_rows, width=760):
    lb, la = mv['lb'], mv['la']
    xs = [x for x, _ in lb.xy + la.xy]
    ys = [y for _, y in lb.xy + la.xy]
    minx, maxx, miny, maxy = min(xs) - CORRIDOR_M, max(xs) + CORRIDOR_M, min(ys) - CORRIDOR_M, max(ys) + CORRIDOR_M
    w_m, h_m = maxx - minx, maxy - miny
    scale = (width - 20) / max(w_m, 1)
    height = max(280, min(900, int(h_m * scale) + 60))
    scale = min(scale, (height - 60) / max(h_m, 1))
    ox = (width - w_m * scale) / 2
    oy = (height - 50 - h_m * scale) / 2

    def P(q):
        return ((q[0] - minx) * scale + ox, height - 50 - oy - (q[1] - miny) * scale)

    def path(xy, colour, w, dash='', op=0.9):
        d, prev = [], None
        for q in xy:
            if minx <= q[0] <= maxx and miny <= q[1] <= maxy:
                x, y = P(q)
                d.append(f'{"L" if prev else "M"}{x:.1f},{y:.1f}')
                prev = q
            else:
                prev = None
        if not d:
            return ''
        return f'<path d="{" ".join(d)}" fill="none" stroke="{colour}" stroke-width="{w}" stroke-opacity="{op}" stroke-dasharray="{dash}" stroke-linejoin="round" stroke-linecap="round"/>'

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}" style="background:#fbfbf8;border:1px solid #ddd">']
    for r in lost_rows:
        out.append(path(r['xy'], '#e69500', 1.6, '4 3', 0.85))
    for r in gained_rows:
        out.append(path(r['xy'], '#00a3a3', 1.6, '4 3', 0.85))
    for r in left:
        out.append(path(r['xy'], '#d62728', 1.3, '', 0.7))
    for r in joined:
        out.append(path(r['xy'], '#2ca02c', 1.3, '', 0.7))
    out.append(path(lb.xy, '#555555', 6, '', 0.45))
    out.append(path(la.xy, '#1f5fbf', 2.2, '', 1.0))
    x, y = P(la.xy[0])
    out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="#1f5fbf"/>')
    x, y = P(la.xy[-1])
    out.append(f'<rect x="{x - 4:.1f}" y="{y - 4:.1f}" width="8" height="8" fill="#1f5fbf"/>')
    # 50 m tolerance band at both ends of the new line
    for q in (la.xy[0], la.xy[-1]):
        x, y = P(q)
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{GROUND_TOL_M * scale:.1f}" fill="none" stroke="#1f5fbf" stroke-dasharray="2 3" stroke-opacity=".7"/>')
    bar = 100 if w_m < 1500 else (500 if w_m < 6000 else 1000)
    x0, y0 = 20, height - 20
    out.append(f'<line x1="{x0}" y1="{y0}" x2="{x0 + bar * scale:.1f}" y2="{y0}" stroke="#222" stroke-width="2"/>')
    out.append(f'<text x="{x0}" y="{y0 - 6}" font-size="12" fill="#222">{bar} m</text>')
    out.append(f'<text x="{width - 10}" y="{height - 8}" font-size="11" fill="#444" text-anchor="end">north up, {w_m:.0f} m across, dashed circles are the 50 m end tolerance</text>')
    out.append('</svg>')
    return '\n'.join(out)


def fmt_m(x):
    return 'n/a' if x is None or (isinstance(x, float) and math.isnan(x)) else f'{x:.0f} m'


def ride_table(rows, label, with_probe=True):
    if not rows:
        return ''
    out = [f'<h4>{label} ({len({r["aid"] for r in rows})} rides, {len(rows)} passes)</h4>',
           '<table><tr><th>activity</th><th>name</th><th>row (track indices)</th><th>nearest to line start</th><th>nearest to line end</th><th>widest offset between</th><th>line left uncovered</th>' + ('<th>the detector says</th>' if with_probe else '') + '</tr>']
    for r in rows:
        probe = f'<td>{html.escape(r["probe"]["text"])}</td>' if with_probe else ''
        out.append(f'<tr><td>{html.escape(r["aid"])}</td><td>{html.escape(r["name"])}</td><td>{r["row"][1]}..{r["row"][2]} {r["row"][3]}</td>'
                   f'<td>{fmt_m(r["start_near"])}</td><td>{fmt_m(r["end_near"])}</td><td>{fmt_m(r["widest"])}</td><td>{fmt_m(r["holes"])}</td>{probe}</tr>')
    out.append('</table>')
    return '\n'.join(out)


def main():
    before_path, after_path, corpus, out_dir = sys.argv[1:5]
    os.makedirs(out_dir, exist_ok=True)
    before = [r for r in (json.loads(l) for l in open(before_path)) if 'id' in r]
    after = [r for r in (json.loads(l) for l in open(after_path)) if 'id' in r]
    ai = {r['id']: r for r in after}
    names, paths = corpus_index(corpus)
    tracks = {}

    def track(aid):
        if aid not in tracks:
            tracks[aid] = load_gpx(paths[aid]) if aid in paths else []
        return tracks[aid]

    changed = []
    for b in before:
        a = ai.get(b['id'])
        if a is None or b['members'] != a['members'] or b['visit_count'] != a['visit_count'] or b['line_hash'] != a['line_hash'] or abs(b['distance_m'] - a['distance_m']) > 0.5:
            changed.append((b, a))

    # Pass one: every member against the line, then the variants for each
    # row the pin does not keep.
    probe = Probe()
    plan = []
    for b, a in changed:
        cur = a if a is not None else b
        sid, line, L = b['id'], cur['polyline'], cur['distance_m']
        bm, am = set(b['members']), (set(a['members']) if a is not None else set())
        rows_b, rows_a = defaultdict(list), defaultdict(list)
        for p in b['portions']:
            rows_b[p[0]].append(p)
        if a is not None:
            for p in a['portions']:
                rows_a[p[0]].append(p)
        for m in sorted(bm | am):
            probe.ask(f'{sid}:full', m, line)
        left_rows = [(m, p) for m in sorted(bm - am) for p in rows_b[m]]
        lost_rows = []
        for m in sorted(bm & am):
            if len(rows_b[m]) == len(rows_a[m]):
                continue
            for p in rows_b[m]:
                if not any(not (q[2] < p[1] or q[1] > p[2]) for q in rows_a[m]):
                    lost_rows.append((m, p))
        gained_rows = []
        for m in sorted(bm & am):
            if len(rows_b[m]) == len(rows_a[m]):
                continue
            for q in rows_a[m]:
                if not any(not (q[2] < p[1] or q[1] > p[2]) for p in rows_b[m]):
                    gained_rows.append((m, q))
        for m, p in left_rows + lost_rows:
            ask_variants(probe, sid, m, line, L, (max(0, p[1] - WINDOW_POINTS), p[2] + WINDOW_POINTS))
        plan.append((b, a, rows_b, rows_a, left_rows, lost_rows, gained_rows))
    probe.run(out_dir)

    # Pass two: the detector's verdict on the line as drawn must reproduce
    # the pin's rows, or the explanations below are not the detector's.
    agree = disagree = 0
    disagreements = []
    direction = []
    for b, a, rows_b, rows_a, *_ in plan:
        if a is None:
            continue
        for m in sorted(set(b['members']) | set(a['members'])):
            passes = probe.get(f'{b["id"]}:full', m) or []
            got = [(p[0], p[1], p[2]) for p in passes]
            exp = [(p[1], p[2], p[3]) for p in rows_a.get(m, [])]
            if got == exp:
                agree += 1
            else:
                disagree += 1
                disagreements.append((b['id'], m, exp, got))
            direction += row_directions(exp, passes)

    sections = []
    for b, a, rows_b, rows_a, left_rows, lost_rows, gained_rows in plan:
        cur = a if a is not None else b
        sid, L = b['id'], cur['distance_m']
        mid = cur['polyline'][len(cur['polyline']) // 2]
        proj = Proj(mid[0], mid[1])
        mv = line_move(b, cur, proj)
        line_now = mv['la']
        bm, am = set(b['members']), (set(a['members']) if a is not None else set())

        def describe(m, p, with_variants):
            g = geometry(line_now, proj, track(m), p[1], p[2])
            g.update(aid=m, name=names.get(m, ''), row=p)
            if with_variants:
                g['probe'] = read_variants(probe, sid, m, L, (max(0, p[1] - WINDOW_POINTS), p[2] + WINDOW_POINTS))
            else:
                passes = probe.get(f'{sid}:full', m) or []
                g['probe'] = dict(kind='whole', text='runs the line whole' if any(q[0] == p[1] and q[1] == p[2] for q in passes) else 'runs the line whole (the pin row and this pass differ in indices)')
            return g

        left = [describe(m, p, True) for m, p in left_rows]
        joined = [describe(m, p, False) for m in sorted(am - bm) for p in rows_a[m]]
        lost = [describe(m, p, True) for m, p in lost_rows]
        gained = [describe(m, p, False) for m, p in gained_rows]
        whole_now = {m for m in bm | am if probe.get(f'{sid}:full', m)}

        # Verdict against the full-ride rule
        if a is None:
            n_whole = len(whole_now)
            verdict = 'correction' if n_whole < 2 else 'needs a look'
            why = f'{n_whole} of {len(bm)} former members run the aligned line whole under the detector\'s bar, and the creating floor is at least 2'
        else:
            bad_left = [r for r in left if r['probe']['kind'] == 'alone']
            bad_lost = [r for r in lost if r['probe']['kind'] == 'alone']
            bad_joined = [m for m in am - bm if m not in whole_now]
            if not (bad_left or bad_lost or bad_joined):
                verdict = 'correction'
                bits = []
                if left:
                    kinds = Counter(r['probe']['kind'] for r in left)
                    bits.append('every ride that left misses an end or runs beside the line (' + ', '.join(f'{k} {v}' for k, v in kinds.most_common()) + ')')
                if joined:
                    bits.append(f'every ride that joined runs the line whole')
                if lost:
                    kinds = Counter(r['probe']['kind'] for r in lost)
                    bits.append('every lost visit is a pass that does not run the line whole (' + ', '.join(f'{k} {v}' for k, v in kinds.most_common()) + ')')
                if gained:
                    bits.append('every gained visit runs the line whole')
                why = '; '.join(bits) or 'only the line moved'
            else:
                verdict = 'needs a look'
                bits = []
                if bad_left:
                    bits.append(f'{len(bad_left)} rides left although their stretch of track passes the line whole on its own')
                if bad_lost:
                    bits.append(f'{len(bad_lost)} visits lost although that stretch passes the line whole on its own: the pass before it swallowed it')
                if bad_joined:
                    bits.append(f'{len(bad_joined)} rides joined without passing the line whole')
                why = '; '.join(bits)

        sections.append(dict(
            id=sid, b=b, a=a, mv=mv, left=left, joined=joined, lost=lost, gained=gained, verdict=verdict, why=why,
            place=place_name(b['members'], names), ground=ground(b, tracks),
            moved=len(bm - am) + len(am - bm), visits=((a['visit_count'] if a is not None else 0) - b['visit_count']),
        ))

    sections.sort(key=lambda s: (-s['moved'], -abs(s['visits']), int(s['id'].rsplit('_', 1)[1])))
    write_html(sections, before, after, out_dir, agree, disagree, disagreements)
    write_summary(sections, before, after, out_dir, agree, disagree, disagreements, direction)


def write_summary(sections, before, after, out_dir, agree, disagree, disagreements, direction):
    reasons, lost_reasons = Counter(), Counter()
    shortfalls = []
    for s in sections:
        for r in s['left']:
            reasons[r['probe']['kind']] += 1
            if r['probe']['kind'] in ('start', 'end'):
                shortfalls.append(r['probe']['d'])
        for r in s['lost']:
            lost_reasons[r['probe']['kind']] += 1
    with open(os.path.join(out_dir, 'summary.txt'), 'w') as f:
        f.write(f'{len(before)} sections before, {len(after)} after, {len(sections)} changed\n')
        f.write(f'probe against the pin rows: {agree} members agree, {disagree} differ\n')
        f.write(direction_line(direction) + '\n')
        for d in disagreements:
            f.write(f'  differs: {d}\n')
        f.write('\nid | length | line | members b->a (+joined/-left) | visits b->a | verdict | why\n')
        for s in sections:
            b, a = s['b'], s['a']
            am = len(a['members']) if a else '-'
            av = a['visit_count'] if a else '-'
            f.write(f"{s['id']} | {b['distance_m']:.0f} m | {s['mv']['kind'] if a else 'dropped'} | {len(b['members'])}->{am} (+{len({c['aid'] for c in s['joined']})}/-{len({c['aid'] for c in s['left']})}) | {b['visit_count']}->{av} | {s['verdict']} | {s['why']}\n")
        n_corr = sum(s['verdict'] == 'correction' for s in sections)
        f.write(f'\nverdicts: {n_corr} correction, {len(sections) - n_corr} needs a look: ' + ', '.join(s['id'] for s in sections if s['verdict'] != 'correction') + '\n')
        f.write(f"rides left (kept sections): {sum(len({c['aid'] for c in s['left']}) for s in sections if s['a'])}, joined: {sum(len({c['aid'] for c in s['joined']}) for s in sections)}\n")
        f.write('why rows left, by the detector: ' + ', '.join(f'{k} {v}' for k, v in reasons.most_common()) + '\n')
        if shortfalls:
            shortfalls.sort()
            f.write(f'shortfall at one end (first trim that counts, m): min {shortfalls[0]}, median {shortfalls[len(shortfalls) // 2]}, max {shortfalls[-1]}, n {len(shortfalls)}\n')
            f.write('  distribution: ' + ', '.join(f'{k} m: {v}' for k, v in sorted(Counter(shortfalls).items())) + '\n')
        besides = sorted(r['probe']['lat'] for s in sections for r in s['left'] if r['probe']['kind'] == 'beside')
        if besides:
            f.write(f'extra lateral tolerance that would count the rides running beside: ' + ', '.join(f'{k} m: {v}' for k, v in sorted(Counter(besides).items())) + '\n')
        f.write('why visits were lost on kept members: ' + ', '.join(f'{k} {v}' for k, v in lost_reasons.most_common()) + '\n')


def write_html(sections, before, after, out_dir, agree, disagree, disagreements):
    css = '''body{font-family:system-ui,sans-serif;max-width:1100px;margin:2em auto;padding:0 1em;color:#222;line-height:1.45}
table{border-collapse:collapse;font-size:13px;margin:.5em 0}th,td{border:1px solid #ccc;padding:3px 7px;text-align:left;vertical-align:top}
th{background:#f0f0ea}.sec{border-top:2px solid #ccc;margin-top:3em;padding-top:1em}.legend span{display:inline-block;margin-right:1.2em}
.sw{display:inline-block;width:28px;height:0;border-top:4px solid;margin-right:5px;vertical-align:middle}
.correction{color:#2ca02c}.needs{color:#b8860b}small{color:#666}code{background:#f3f3ee;padding:0 3px}'''
    h = [f'<!doctype html><html><head><meta charset="utf-8"><title>Section move, spatial report</title><style>{css}</style></head><body>']
    h.append('<h1>Where each changed section moved, and why each ride joined or left</h1>')
    h.append(f'<p>{len(before)} sections before, {len(after)} after, {len(sections)} with a changed line, membership or visit count. Each section below shows its line before and now drawn over each other, '
             'with the rides that joined and left, and a table that says for each ride how close its track comes to each end of the line and what the detector itself answers.</p>')
    h.append('<p><b>The bar.</b> Only a full ride of the section counts. A pass is a run of track points within 50 m of the line, bridging up to 3 points, and it counts when every 10 m station of the line '
             'lies within 50 m of that run, so it reaches both ends and runs between them. The column <i>the detector says</i> is not a re-implementation: the detector was asked, through its public pass count, '
             'about the line as drawn, the line trimmed by 10 to 1600 m at the start, the end or both, the line with 5 to 200 m more lateral tolerance, and each half. The first probe a ride passes is what it says. '
             'Over the line as drawn the probe reproduces the pin\'s member rows for '
             f'{agree} of {agree + disagree} members; the {disagree} that differ are listed at the end.</p>')
    h.append('<p><b>How to read the ride tables.</b> <i>Nearest to line start</i> and <i>end</i> are the closest the ride\'s track comes to each end of the line, within 200 track points of the row; '
             'more than 50 m means the ride never reaches that end within tolerance. <i>Widest offset between</i> is the worst station offset along the stretch the ride does cover, and <i>line left uncovered</i> '
             'is how many metres of the line between its first and last covered station no run point comes within 50 m of.</p>')
    h.append('<p class="legend"><span><i class="sw" style="border-color:#555;opacity:.5"></i>line before</span><span><i class="sw" style="border-color:#1f5fbf"></i>line now (dot start, square end)</span>'
             '<span><i class="sw" style="border-color:#d62728"></i>ride that left</span><span><i class="sw" style="border-color:#2ca02c"></i>ride that joined</span>'
             '<span><i class="sw" style="border-color:#e69500;border-top-style:dashed"></i>visit lost on a kept member</span><span><i class="sw" style="border-color:#00a3a3;border-top-style:dashed"></i>visit gained on a kept member</span></p>')
    h.append('<h2>Summary, sorted by rides moved</h2>')
    h.append('<table><tr><th>#</th><th>section</th><th>place words</th><th>length</th><th>line</th><th>members</th><th>joined</th><th>left</th><th>visits</th><th>verdict</th><th>why</th></tr>')
    for i, s in enumerate(sections, 1):
        b, a = s['b'], s['a']
        line = s['mv']['kind'] if a else 'dropped'
        members = f"{len(b['members'])} to {len(a['members'])}" if a else f"{len(b['members'])} to none"
        visits = f"{b['visit_count']} to {a['visit_count']}" if a else f"{b['visit_count']} to none"
        cls = 'correction' if s['verdict'] == 'correction' else 'needs'
        h.append(f'<tr><td>{i}</td><td><a href="#{s["id"]}">{s["id"]}</a></td><td>{html.escape(s["place"])}</td><td>{b["distance_m"]:.0f} m</td><td>{line}</td><td>{members}</td><td>{len({c["aid"] for c in s["joined"]})}</td><td>{len({c["aid"] for c in s["left"]})}</td><td>{visits}</td><td class="{cls}">{s["verdict"]}</td><td><small>{html.escape(s["why"])}</small></td></tr>')
    h.append('</table>')
    for i, s in enumerate(sections, 1):
        b, a, mv = s['b'], s['a'], s['mv']
        h.append(f'<div class="sec" id="{s["id"]}"><h2>{i}. {s["id"]}, {b["distance_m"]:.0f} m' + (' (dropped)' if a is None else '') + '</h2>')
        h.append('<h3>Where it is</h3>')
        h.append(f'<p>Start {b["first"][0]:.5f}, {b["first"][1]:.5f}; end {b["last"][0]:.5f}, {b["last"][1]:.5f}. Place words from the member activity names: {html.escape(s["place"])}. '
                 f'The ground: {html.escape(s["ground"])}. Drawn from {b["rep"]}' + (f' points {b["rep_range"][0]}..{b["rep_range"][1]}' if b['rep_range'] else '') + '.</p>')
        h.append('<h3>How the line moved</h3>')
        if a is None:
            h.append(f'<p>Dropped. The line is the same one the alignment drew at the earlier pin; at the pin only one ride runs it whole, so against its creating floor it is a <code>DrawnPopulation</code> record and no section. '
                     f'The {len(b["members"])} former members were counted on the line before the alignment, and are checked against the aligned line below.</p>')
        else:
            same_rep = (a['rep'], a['rep_range']) == (b['rep'], b['rep_range'])
            h.append(f'<p>{mv["how"][0].upper() + mv["how"][1:]}. Start moved {mv["start_gap"]:.0f} m, end moved {mv["end_gap"]:.0f} m, widest separation between the two lines {mv["widest"]:.0f} m. '
                     + (f'Now drawn from {a["rep"]} points {a["rep_range"][0]}..{a["rep_range"][1]}' + (', the same trace and range as before' if same_rep else f' (was {b["rep"]} {b["rep_range"]})') + '. ' if a['rep_range'] else '')
                     + f'Length {b["distance_m"]:.0f} m to {a["distance_m"]:.0f} m. Members {len(b["members"])} to {len(a["members"])}, visits {b["visit_count"]} to {a["visit_count"]}.</p>')
        h.append(svg_plot(mv, s['left'], s['joined'], s['lost'], s['gained']))
        h.append('<h3>Rides</h3>')
        h.append(ride_table(s['left'], 'Left' if a is not None else 'Former members'))
        h.append(ride_table(s['joined'], 'Joined'))
        h.append(ride_table(s['lost'], 'Visits lost on kept members'))
        h.append(ride_table(s['gained'], 'Visits gained on kept members'))
        if a is not None and not (s['left'] or s['joined'] or s['lost'] or s['gained']):
            h.append('<p>No ride and no visit moved; only the line did.</p>')
        cls = 'correction' if s['verdict'] == 'correction' else 'needs'
        h.append(f'<h3>Verdict</h3><p class="{cls}"><b>{s["verdict"][0].upper() + s["verdict"][1:]}</b>: {html.escape(s["why"])}.</p></div>')
    if disagreements:
        h.append('<div class="sec"><h2>Where the probe and the pin rows differ</h2><p>The probe counts over the line as the pin dump holds it. These members read differently in the fold.</p><table><tr><th>section</th><th>activity</th><th>pin rows</th><th>probe rows</th></tr>')
        for sid, m, exp, got in disagreements:
            h.append(f'<tr><td>{sid}</td><td>{m}</td><td>{html.escape(str(exp))}</td><td>{html.escape(str(got))}</td></tr>')
        h.append('</table></div>')
    h.append('</body></html>')
    with open(os.path.join(out_dir, 'report.html'), 'w') as f:
        f.write('\n'.join(h))


if __name__ == '__main__':
    main()
