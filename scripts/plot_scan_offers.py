#!/usr/bin/env python3
"""Draw the sampled scan offers: the section line, the ride around the pass
and the offered pass itself, one panel each, as PNG grids for a by-eye check.

Usage: plot_scan_offers.py OFFERS.json... --out DIR [--per-image N]
Reads the JSON `scan_miss_rate` writes and draws only offers carrying geometry.
"""
import argparse
import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def local_xy(points, lat0, lon0):
    k = 111_320.0
    return (
        [(p[1] - lon0) * k * math.cos(math.radians(lat0)) for p in points],
        [(p[0] - lat0) * k for p in points],
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("offers", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--per-image", type=int, default=10)
    ap.add_argument("--only", help="comma-separated panel ids to draw")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    sampled = []
    for path in args.offers:
        for o in json.load(open(path)):
            if o.get("geometry"):
                sampled.append(o)
    print(f"{len(sampled)} offers with geometry")
    cols = 5
    rows = math.ceil(args.per_image / cols)
    for page in range(0, len(sampled), args.per_image):
        chunk = sampled[page : page + args.per_image]
        fig, axes = plt.subplots(rows, cols, figsize=(cols * 3.6, rows * 3.6))
        for ax in axes.flat:
            ax.axis("off")
        for i, o in enumerate(chunk):
            ax = axes.flat[i]
            ax.axis("on")
            g = o["geometry"]
            lat0, lon0 = g["line"][0]
            for pts, colour, width, z in (
                (g["ride"], "#bbbbbb", 1.0, 1),
                (g["line"], "#1f77b4", 2.5, 2),
                (g["pass"], "#d62728", 1.3, 3),
            ):
                x, y = local_xy(pts, lat0, lon0)
                ax.plot(x, y, color=colour, linewidth=width, zorder=z)
            lx, ly = local_xy(g["line"], lat0, lon0)
            ax.plot(lx[0], ly[0], "o", color="#1f77b4", markersize=5, zorder=4)
            ax.set_aspect("equal")
            ax.set_xticks([])
            ax.set_yticks([])
            n = page + i + 1
            ax.set_title(
                f"#{n} {o['corpus'][:14]} {o['ride_sport'][:5]}>{o['section_sport'][:5]}\n"
                f"cov {o['coverage']:.2f} lat50 {o['lateral_median_m']:.0f} "
                f"lat90 {o['lateral_p90_m']:.0f} held {o['member_pass_share']:.2f} "
                f"{'strict' if o['strict_finds'] else ''} {'CAND' if o['candidate'] else ''}",
                fontsize=7,
            )
        fig.tight_layout()
        out = os.path.join(args.out, f"offers-{page // args.per_image + 1:02d}.png")
        fig.savefig(out, dpi=110)
        plt.close(fig)
        print(out)


if __name__ == "__main__":
    main()
