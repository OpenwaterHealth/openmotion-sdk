"""Plot a thermal_soak.py run: temperatures and image statistics against time.

usage: thermal_soak_plot.py <run_dir> [--png-every N]

Writes <run_dir>/soak_<side>.png per sensor module:
  1. camera die temperature (sensor telemetry), all cameras
  2. lit-row mean of each imaged camera's composites
  3. speckle contrast K (lit rows, central ROI, minus the pedestal)
  4. dark level from the scheduled dark exposures' rows (when they landed)
  5. console temperatures
With --png-every N, also exports every Nth saved composite of each camera as
a lossless 16-bit PNG (raw values) plus an 8-bit contrast-stretched preview
into <run_dir>/png/.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


def read_csv(path):
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


def plot(run: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    comp = read_csv(run / "composites.csv")
    cam_t = read_csv(run / "camera_telemetry.csv")
    con_t = read_csv(run / "console_telemetry.csv")
    sides = sorted({r["side"] for r in comp} | {r["side"] for r in cam_t})
    outs = []
    for side in sides:
        fig, ax = plt.subplots(5, 1, figsize=(14, 17), sharex=True)
        colors = plt.cm.tab10(np.arange(8))
        by_cam = defaultdict(list)
        for r in cam_t:
            if r["side"] == side:
                by_cam[int(r["cam"])].append((num(r["t_s"]) / 3600, num(r["tpm_avg_c"])))
        for c, pts in sorted(by_cam.items()):
            t, v = np.array(pts).T
            ax[0].plot(t, v, color=colors[c], label=f"cam{c}")
        ax[0].set_ylabel("die temp (°C)")
        ax[0].legend(ncol=8, fontsize=8)
        comp_cam = defaultdict(list)
        for r in comp:
            if r["side"] == side:
                comp_cam[int(r["cam"])].append(r)
        for c, rows in sorted(comp_cam.items()):
            t = np.array([num(r["t_s"]) for r in rows]) / 3600
            ax[1].plot(t, [num(r["lit_mean"]) for r in rows], ".", ms=2, color=colors[c], label=f"cam{c}")
            ax[2].plot(t, [num(r["K"]) for r in rows], ".", ms=2, color=colors[c])
            dm = np.array([num(r["dark_mean"]) for r in rows])
            ok = np.isfinite(dm)
            ax[3].plot(t[ok], dm[ok], "o", ms=3, color=colors[c])
        ax[1].set_ylabel("lit mean (DN)")
        ax[1].legend(ncol=8, fontsize=8, markerscale=4)
        ax[2].set_ylabel("speckle K")
        ax[3].set_ylabel("dark rows mean (DN)")
        if con_t:
            t = np.array([num(r["t_s"]) for r in con_t]) / 3600
            for k in ("t1_c", "t2_c", "t3_c"):
                ax[4].plot(t, [num(r.get(k)) for r in con_t], label=k)
            ax[4].legend(fontsize=8)
        ax[4].set_ylabel("console temps (°C)")
        ax[4].set_xlabel("hours since start")
        info = {}
        if (run / "run.json").exists():
            info = json.loads((run / "run.json").read_text())
        fig.suptitle(f"Thermal soak {run.name}, {side} module - timing {info.get('args', {}).get('timing', '?')}, "
                     f"STRIDE {info.get('stride', '?')} ({info.get('image_period_s', '?')} s/image)")
        fig.tight_layout()
        p = run / f"soak_{side}.png"
        fig.savefig(p, dpi=90)
        plt.close(fig)
        outs.append(p)
    return outs


def export_png(run: Path, every: int):
    from PIL import Image
    dst = run / "png"
    n = 0
    for cam_dir in sorted((run / "images").glob("*/cam*")):
        files = sorted(cam_dir.glob("*.npz"))
        for f in files[::max(every, 1)]:
            z = np.load(f)
            img = z["image"]
            d = dst / cam_dir.parent.name / cam_dir.name
            d.mkdir(parents=True, exist_ok=True)
            Image.fromarray(img.astype(np.uint16)).save(d / (f.stem + "_raw16.png"))
            lo, hi = np.percentile(img, [1, 99.5])
            prev = np.clip((img.astype(np.float64) - lo) / max(hi - lo, 1) * 255, 0, 255).astype(np.uint8)
            Image.fromarray(prev).save(d / (f.stem + "_preview8.png"))
            n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--png-every", type=int, default=0)
    a = ap.parse_args()
    run = Path(a.run)
    for p in plot(run):
        print("wrote", p)
    if a.png_every:
        print("exported", export_png(run, a.png_every), "composites to", run / "png")


if __name__ == "__main__":
    main()
