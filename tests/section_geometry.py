"""Where each line of a section-dump move went. For a kept line that was
redrawn: how far each end moved and whether it was held, trimmed, extended or
shifted, whether the line reversed, and how far the new line strays from the
old. For a dropped line: which later line covers its ground and how many of its
members are there. For a new line: which earlier line it lies on. The
end-to-end distance over the length (the fold ratio) says how far a line
doubles back. A scratch measurement beside `section_compare.py`, not a gate.

Run: python3 -I tests/section_geometry.py BEFORE.jsonl AFTER-renumbered.jsonl LABEL
"""
import json, math, sys
from collections import defaultdict

TOL = 50.0
R = 6371000.0

def read(p):
    rows = [json.loads(l) for l in open(p) if l.strip()]
    return {r['id']: r for r in rows if 'id' in r}

def proj(lat0, lon0):
    kx = math.radians(1) * R * math.cos(math.radians(lat0)); ky = math.radians(1) * R
    return lambda p: ((p[1] - lon0) * kx, (p[0] - lat0) * ky)

def dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def seg_d(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]; px, py = p[0]-a[0], p[1]-a[1]
    l2 = dx*dx+dy*dy
    t = 0 if l2 == 0 else max(0, min(1, (px*dx+py*dy)/l2))
    return math.hypot(px-t*dx, py-t*dy)

def near_frac(pts, line):
    """Fraction of pts within TOL of polyline line (brute force, lines are short)."""
    if not pts or not line: return 0.0
    n = 0
    segs = list(zip(line, line[1:])) or [(line[0], line[0])]
    for p in pts:
        if min(seg_d(p, a, b) for a, b in segs) <= TOL: n += 1
    return n / len(pts)

def xy(rec, pr):
    return [pr(p) for p in rec['polyline']]

def end_moves(b, a):
    pr = proj(*b['polyline'][0])
    B, A = xy(b, pr), xy(a, pr)
    same = dist(B[0], A[0]) + dist(B[-1], A[-1])
    rev = dist(B[0], A[-1]) + dist(B[-1], A[0])
    reversed_ = rev < same
    if reversed_: A = A[::-1]
    s, e = dist(B[0], A[0]), dist(B[-1], A[-1])
    # does the new start lie on the old line (trim) or beyond it (extension)?
    segsB = list(zip(B, B[1:]))
    def on_old(p): return min(seg_d(p, x, y) for x, y in segsB) <= TOL
    segsA = list(zip(A, A[1:]))
    def on_new(p): return min(seg_d(p, x, y) for x, y in segsA) <= TOL
    def end_kind(moved, new_pt, old_pt):
        if moved <= TOL: return 'held'
        if on_old(new_pt): return 'trimmed'
        if on_new(old_pt): return 'extended'
        return 'shifted'
    widest = max(min(seg_d(p, x, y) for x, y in segsB) for p in A)
    return reversed_, s, e, end_kind(s, A[0], B[0]), end_kind(e, A[-1], B[-1]), widest

def fold_ratio(rec):
    pr = proj(*rec['polyline'][0]); P = xy(rec, pr)
    return dist(P[0], P[-1]) / max(rec['distance_m'], 1)

def main(before, after, label):
    b, a = read(before), read(after)
    print(f"### {label}: {len(b)} before, {len(a)} after")
    common = [i for i in b if i in a]
    red = []
    for i in common:
        if b[i]['line_hash'] == a[i]['line_hash']: continue
        rv, s, e, ks, ke, w = end_moves(b[i], a[i])
        red.append((i, b[i]['distance_m'], a[i]['distance_m'], rv, s, e, ks, ke, w, len(b[i]['members']), len(a[i]['members']), fold_ratio(b[i]), fold_ratio(a[i]), b[i]['rep'], a[i]['rep']))
    print(f"redrawn or reversed: {len(red)}")
    kinds = defaultdict(int)
    for r in red: kinds[(r[6], r[7])] += 1
    print("end kinds (start, end):", dict(kinds))
    print("reversed among them:", sum(1 for r in red if r[3]))
    print("new representative:", sum(1 for r in red if r[13] != r[14]))
    print("widest offset of new line from old > 50 m:", [(r[0], round(r[8])) for r in red if r[8] > TOL])
    print("\nid | len b->a | rev | start moved m (kind) | end moved m (kind) | widest off old | members b->a | fold b->a | rep b->a")
    for r in sorted(red, key=lambda r: -(abs(r[1]-r[2]))):
        print(f"{r[0]} | {r[1]:.0f}->{r[2]:.0f} | {'R' if r[3] else '-'} | {r[4]:.0f} ({r[6]}) | {r[5]:.0f} ({r[7]}) | {r[8]:.0f} | {r[9]}->{r[10]} | {r[11]:.2f}->{r[12]:.2f} | {r[13]}->{r[14]}")
    dropped = [i for i in b if i not in a]; added = [i for i in a if i not in b]
    print(f"\ndropped {len(dropped)}: which after-line covers the old ground (fraction of old points within 50 m), and former members found there")
    for i in dropped:
        pr = proj(*b[i]['polyline'][0]); B = xy(b[i], pr)
        best = []
        for j in a:
            if abs(a[j]['first'][0]-b[i]['first'][0]) > 0.2 or abs(a[j]['first'][1]-b[i]['first'][1]) > 0.2: continue
            f = near_frac(B, xy(a[j], pr))
            if f >= 0.3:
                mem = len(set(b[i]['members']) & set(a[j]['members']))
                best.append((f, j, a[j]['distance_m'], mem, len(a[j]['members']), j in added))
        best.sort(reverse=True)
        print(f"  {i} ({b[i]['distance_m']:.0f} m, {len(b[i]['members'])} members, fold {fold_ratio(b[i]):.2f}, rep {b[i]['rep']}): " + ("; ".join(f"{j}{'(new)' if nw else ''} {l:.0f} m covers {f:.2f}, holds {m}/{len(b[i]['members'])} former members (of {n})" for f, j, l, m, n, nw in best[:3]) or "no after-line covers 30 % of it"))
    print(f"\nadded {len(added)}: which before-line it lies on (fraction of new points within 50 m), and where its members came from")
    for j in added:
        pr = proj(*a[j]['polyline'][0]); A = xy(a[j], pr)
        best = []
        for i in b:
            if abs(a[j]['first'][0]-b[i]['first'][0]) > 0.2 or abs(a[j]['first'][1]-b[i]['first'][1]) > 0.2: continue
            f = near_frac(A, xy(b[i], pr))
            if f >= 0.3:
                mem = len(set(a[j]['members']) & set(b[i]['members']))
                best.append((f, i, b[i]['distance_m'], mem, len(b[i]['members']), i in dropped))
        best.sort(reverse=True)
        print(f"  {j} ({a[j]['distance_m']:.0f} m, {len(a[j]['members'])} members, fold {fold_ratio(a[j]):.2f}, rep {a[j]['rep']}): " + ("; ".join(f"on {i}{'(dropped)' if dr else ''} {l:.0f} m, {f:.2f} of its points, {m}/{len(a[j]['members'])} members were its (of {n})" for f, i, l, m, n, dr in best[:3]) or "on no before-line's ground: new ground"))

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
