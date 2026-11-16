# Renumber a later section dump to an earlier dump's ids. Ids are positional
# (sec_<sport>_<n>), so a dropped section shifts every later id down by one and
# the two dumps cannot be compared by id alone. A later section takes the id of
# the earlier section of the same sport whose line it continues: one of the two
# lines has at least half of its points within GROUND_TOL_M of the other, the
# best overlap first. A line with no such partner takes the id of an earlier
# line with the same hash or the reversed polyline, else a fresh id past the
# earlier numbering. Member sets are not compared, since they change with every
# ride the detector adds or drops. Each pairing with its overlap, each fresh id
# and each dropped line is listed on stderr.
# Run: python3 -I tests/section_renumber.py before.jsonl after.jsonl > after-renumbered.jsonl
import json, math, sys

GROUND_TOL_M = 50.0
PAIR_MIN_OVERLAP = 0.5
R = 6371000.0


def read(path):
    lines = [json.loads(l) for l in open(path) if l.strip()]
    if lines and 'digest' in lines[0] and 'id' not in lines[0]:
        return lines[0], lines[1:]
    return None, lines


def seg_dist(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    px, py = p[0] - ax, p[1] - ay
    l2 = dx * dx + dy * dy
    if l2 == 0:
        return math.hypot(px, py)
    t = max(0.0, min(1.0, (px * dx + py * dy) / l2))
    return math.hypot(px - t * dx, py - t * dy)


def cell(v):
    return math.floor(v / GROUND_TOL_M)


class Shape:
    """A polyline in local metres, its segments binned in GROUND_TOL_M cells."""

    def __init__(self, record, origin):
        lat0, lon0 = origin
        kx = math.radians(1) * R * math.cos(math.radians(lat0))
        ky = math.radians(1) * R
        self.xy = [((lon - lon0) * kx, (lat - lat0) * ky) for lat, lon in record['polyline']]
        self.box = None
        if self.xy:
            xs, ys = [p[0] for p in self.xy], [p[1] for p in self.xy]
            self.box = (min(xs), min(ys), max(xs), max(ys))
        self.grid = {}
        segments = list(zip(self.xy, self.xy[1:])) or [(p, p) for p in self.xy]
        for p, q in segments:
            for cx in range(cell(min(p[0], q[0])), cell(max(p[0], q[0])) + 1):
                for cy in range(cell(min(p[1], q[1])), cell(max(p[1], q[1])) + 1):
                    self.grid.setdefault((cx, cy), []).append((p, q))

    def within(self, z):
        """Whether z lies within GROUND_TOL_M of this polyline."""
        cx, cy = cell(z[0]), cell(z[1])
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for p, q in self.grid.get((cx + dx, cy + dy), ()):
                    if seg_dist(z, p, q) <= GROUND_TOL_M:
                        return True
        return False


def share(src, dst):
    """Fraction of src's vertices within GROUND_TOL_M of dst."""
    if not src.xy:
        return 0.0
    return sum(dst.within(z) for z in src.xy) / len(src.xy)


def apart(a, b):
    """Whether the two boxes are further apart than GROUND_TOL_M on either axis."""
    return (a[0] - GROUND_TOL_M > b[2] or b[0] - GROUND_TOL_M > a[2]
            or a[1] - GROUND_TOL_M > b[3] or b[1] - GROUND_TOL_M > a[3])


def overlap(a, b):
    """The larger of the two shares: at least this much of one line lies on the other."""
    if a.box is None or b.box is None or apart(a.box, b.box):
        return 0.0
    return max(share(a, b), share(b, a))


def origin_of(records):
    points = [p for r in records for p in r['polyline']]
    if not points:
        return 0.0, 0.0
    return sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points)


def pair(before, after):
    """Ids for the later records, and the stderr lines that say how each was carried."""
    origin = origin_of(before + after)
    shapes_b = [Shape(r, origin) for r in before]
    shapes_a = [Shape(r, origin) for r in after]

    candidates = []
    for i, r in enumerate(after):
        for j, x in enumerate(before):
            if x['sport'] != r['sport']:
                continue
            o = overlap(shapes_a[i], shapes_b[j])
            if o >= PAIR_MIN_OVERLAP:
                candidates.append((o, i, j))
    candidates.sort(key=lambda c: (-c[0], c[1], c[2]))

    carried, taken, notes = {}, set(), []
    for o, i, j in candidates:
        if i not in carried and j not in taken:
            carried[i] = j
            taken.add(j)
            notes.append(f"{after[i]['id']} -> {before[j]['id']} by line overlap {o:.2f}")

    by_line = {r['line_hash']: j for j, r in enumerate(before)}
    for i, r in enumerate(after):
        if i in carried:
            continue
        j = by_line.get(r['line_hash'])
        if j is None:
            j = next((k for k, x in enumerate(before) if x['polyline'] == r['polyline'][::-1]), None)
        if j is not None and j not in taken:
            carried[i] = j
            taken.add(j)
            o = overlap(shapes_a[i], shapes_b[j])
            notes.append(f"{r['id']} -> {before[j]['id']} by line overlap {o:.2f}")

    ids = [None] * len(after)
    for i, j in carried.items():
        ids[i] = before[j]['id']
    fresh = max((int(x['id'].rsplit('_', 1)[1]) for x in before), default=0)
    for i, r in enumerate(after):
        if i in carried:
            continue
        fresh += 1
        prefix = r['id'].rsplit('_', 1)[0]
        ids[i] = f"{prefix}_{fresh}"
        notes.append(f"{r['id']} -> {ids[i]} (new line, {len(r['members'])} members)")
    for j, x in enumerate(before):
        if j not in taken:
            notes.append(f"{x['id']} dropped ({len(x['members'])} members, {x['distance_m']:.0f} m)")
    return [dict(r, id=ids[i]) for i, r in enumerate(after)], notes


def main(argv):
    _, b = read(argv[1])
    a_header, a = read(argv[2])
    if a_header is not None:
        print(json.dumps(a_header))
    records, notes = pair(b, a)
    for line in notes:
        print(line, file=sys.stderr)
    for r in records:
        print(json.dumps(r))


if __name__ == '__main__':
    main(sys.argv)
