"""Analyze single_exposure.py runs against stride composites of the same
cameras (full_frame_1hz.py output).

Dark reference for each lit frame = the dark phase before it and the dark
phase after it, linearly interpolated in time per pixel (the storage-node
dark pattern drifts as the sensor warms). Per camera it reports:
  - temporal noise (consecutive darks),
  - residual after subtracting the reference from held-out dark frames,
  - laser signal, speckle contrast K and saturation of the dark-subtracted
    lit frames, and consecutive-frame correlation,
  - the same signal / K / noise for the composite (pedestal and noise from
    a --no-laser composite when given, else pedestal 128).

usage:
  single_exposure_analyze.py <run_dir> <composite_dir> [--dark <composite_dark_dir>] [--json out.json]
<run_dir> is one camera's folder (holds phases.json) or a multi-camera run
(holds cam<N>/ folders); composite files are matched by "_cam<N>_".
"""
import argparse
import json
from pathlib import Path

import numpy as np

ROI = (slice(160, 1120), slice(240, 1680))   # central region, away from the edges
BANDS = [(0, 320), (320, 640), (640, 960), (960, 1280)]


def analyze_single(cam_dir):
    m = json.loads((cam_dir / "phases.json").read_text())
    frames = [f for f in m["frames"] if "file" in f]
    load = lambda f: np.load(cam_dir / f"{f['file']}.npy").astype(np.float64)  # noqa: E731
    phases = []
    for f in frames:
        if not phases or phases[-1][0] != f["phase"]:
            phases.append((f["phase"], []))
        phases[-1][1].append(f)
    lit_idx = [i for i, (p, _) in enumerate(phases) if p.startswith("L")]
    if not lit_idx or lit_idx[0] == 0 or lit_idx[-1] + 1 >= len(phases):
        return {"error": f"need dark / lit / dark phases, got {[p for p, _ in phases]}"}
    before = phases[lit_idx[0] - 1][1]
    after = phases[lit_idx[-1] + 1][1]
    lit = [f for i in lit_idx for f in phases[i][1]]

    def ref_at(t, drop=None):
        b = [f for f in before if f is not drop] or before
        a = [f for f in after if f is not drop] or after
        Db, Da = np.mean([load(f) for f in b], 0), np.mean([load(f) for f in a], 0)
        tb, ta = np.mean([f["t_first"] for f in b]), np.mean([f["t_first"] for f in a])
        w = np.clip((t - tb) / (ta - tb), 0, 1)
        return (1 - w) * Db + w * Da

    r = {"darks_before": len(before), "lit": len(lit), "darks_after": len(after)}
    d = [load(f) for f in before + after]
    tn = [np.std(d[i + 1] - d[i]) / np.sqrt(2) for i in range(len(before) - 1)]
    r["temporal_noise"] = float(np.median(tn)) if tn else float("nan")
    r["dark_pattern_by_band"] = [round(float(load(before[-1])[lo:hi].std()), 1) for lo, hi in BANDS]
    res = []
    for f in (before[len(before) // 2], after[len(after) // 2]):
        if len(before) > 1 and len(after) > 1:
            res.append(float((load(f) - ref_at(f["t_first"], drop=f)).std()))
    r["residual"] = float(np.mean(res)) if res else float("nan")
    S = [load(f) - ref_at(f["t_first"]) for f in lit]
    r["signal"] = float(np.mean([x[ROI].mean() for x in S]))
    r["K"] = float(np.mean([x[ROI].std() / x[ROI].mean() for x in S]))
    r["sat"] = float(np.mean([np.mean(load(f) >= 1023) for f in lit]))
    r["r_consecutive"] = float(np.mean([np.corrcoef(S[i][ROI].ravel(), S[i + 1][ROI].ravel())[0, 1]
                                         for i in range(len(S) - 1)])) if len(S) > 1 else float("nan")
    np.save(cam_dir / "single_dark_subtracted_mean.npy", np.mean(S, 0).astype(np.float32))
    return r


def analyze_composite(comp_dir, cam, dark_dir=None):
    files = sorted(Path(comp_dir).glob(f"*_cam{cam}_[0-9][0-9][0-9].npy"))
    if not files:
        return {}
    pedestal, noise = 128.0, float("nan")
    if dark_dir:
        dk = [np.load(p).astype(np.float64) for p in sorted(Path(dark_dir).glob(f"*_cam{cam}_[0-9][0-9][0-9].npy"))]
        if dk:
            pedestal = float(dk[0].mean())
            noise = float(np.std(dk[1] - dk[0]) / np.sqrt(2)) if len(dk) > 1 else float(dk[0].std())
    C = [np.load(p).astype(np.float64) for p in files]
    X = [c - pedestal for c in C]
    return {"frames": len(C), "signal": float(np.mean([x[ROI].mean() for x in X])),
            "K": float(np.mean([x[ROI].std() / x[ROI].mean() for x in X])),
            "sat": float(np.mean([np.mean(c >= 1023) for c in C])), "noise": noise}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("composite")
    ap.add_argument("--dark", default=None)
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    run = Path(a.run)
    if (run / "phases.json").exists():
        m = json.loads((run / "phases.json").read_text())
        cam = m.get("cam", m["args"]["cam"])       # older single-camera runs kept it in args
        cams = [(cam[0] if isinstance(cam, list) else cam, run)]
    else:
        cams = sorted(((int(p.name[3:]), p) for p in run.glob("cam[0-9]") if (p / "phases.json").exists()))
    rows = {}
    print(f"{'cam':>3} | {'single: noise':>13} {'resid':>6} {'signal':>6} {'K':>6} {'sat%':>5} {'r1s':>5} | "
          f"{'composite: noise':>16} {'signal':>6} {'K':>6} {'sat%':>5} | dark pattern by row band")
    for cam, cdir in cams:
        s = analyze_single(cdir)
        c = analyze_composite(a.composite, cam, a.dark) if cam is not None else {}
        rows[cam] = {"single": s, "composite": c}
        if "error" in s:
            print(f"{cam:>3} | {s['error']}")
            continue
        cs = (f"{c.get('noise', float('nan')):16.1f} {c.get('signal', float('nan')):6.0f} "
              f"{c.get('K', float('nan')):6.3f} {100 * c.get('sat', float('nan')):5.2f}") if c else f"{'n/a':>36}"
        print(f"{cam:>3} | {s['temporal_noise']:13.1f} {s['residual']:6.1f} {s['signal']:6.0f} {s['K']:6.3f} "
              f"{100 * s['sat']:5.2f} {s['r_consecutive']:5.2f} | {cs} | {s['dark_pattern_by_band']}")
    if a.json:
        Path(a.json).write_text(json.dumps(rows, indent=1))


if __name__ == "__main__":
    main()
