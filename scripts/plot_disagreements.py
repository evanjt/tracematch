#!/usr/bin/env python3
"""Draw stored member rows that a fresh strict cut over the stored line does
not reproduce: the line (blue), the ride around the row (grey), the stored
row (red) and the fresh cut's first pass (green, dashed)."""
import json, math, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def xy(points, lat0, lon0):
    k = 111_320.0
    return [(p[1]-lon0)*k*math.cos(math.radians(lat0)) for p in points], [(p[0]-lat0)*k for p in points]

paths = sys.argv[1:-1]; out = sys.argv[-1]
os.makedirs(out, exist_ok=True)
rows = []
for p in paths:
    rows += json.load(open(p))
print(len(rows), "disagreements")
per = 10
for page in range(0, len(rows), per):
    chunk = rows[page:page+per]
    fig, axes = plt.subplots(2, 5, figsize=(18, 7.2))
    for ax in axes.flat: ax.axis("off")
    for i, d in enumerate(chunk):
        ax = axes.flat[i]; ax.axis("on")
        lat0, lon0 = d["line"][0]
        for pts, c, w, z, ls in ((d["ride_pts"], "#bbbbbb", 1, 1, "-"), (d["line"], "#1f77b4", 2.5, 2, "-"), (d["stored_pts"], "#d62728", 1.3, 3, "-"), (d["fresh_pts"], "#2ca02c", 1.3, 4, "--")):
            x, y = xy(pts, lat0, lon0); ax.plot(x, y, color=c, linewidth=w, zorder=z, linestyle=ls)
        lx, ly = xy(d["line"], lat0, lon0); ax.plot(lx[0], ly[0], "o", color="#1f77b4", markersize=5)
        ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
        ax.set_title(f"#{page+i+1} {d['corpus'][:12]} {d['kind']} sec {d['section_m']:.0f} m\nstored {d['stored']} fresh {d['fresh'][:2]}\nst50 {d['stored_station_share_50']:.2f} lat50 {d['stored_lateral_median_m']:.0f}", fontsize=7)
    fig.tight_layout()
    f = os.path.join(out, f"dis-{page//per+1:02d}.png"); fig.savefig(f, dpi=110); plt.close(fig); print(f)
