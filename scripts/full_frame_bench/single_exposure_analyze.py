"""Analyze a single_exposure.py run against a stride composite of the same
camera (full_frame_1hz.py output).

Dark reference for each lit frame = the warm dark phase before it and the dark
phase after it, linearly interpolated in time per pixel (the storage-node
dark pattern drifts slowly as the sensor warms). Reports:
  - temporal noise (consecutive warm darks),
  - residual after subtracting the reference from held-out dark frames,
  - speckle contrast K and pixel correlation of dark-subtracted lit frames
    vs the composite (and composite vs composite as the baseline).

usage: single_exposure_analyze.py <single_run_dir> <composite_dir> [pedestal]
"""
import json
import sys
from pathlib import Path

import numpy as np

run, comp = Path(sys.argv[1]), Path(sys.argv[2])
comp_pedestal = float(sys.argv[3]) if len(sys.argv) > 3 else 128.0
m = json.loads((run / "phases.json").read_text())
frames = [f for f in m["frames"] if "file" in f]
load = lambda f: np.load(run / f"{f['file']}.npy").astype(np.float64)  # noqa: E731

phases = []
for f in frames:
    if not phases or phases[-1][0] != f["phase"]:
        phases.append((f["phase"], []))
    phases[-1][1].append(f)
lit_idx = [i for i, (p, _) in enumerate(phases) if p.startswith("L")]
before = phases[lit_idx[0] - 1][1]
after = phases[lit_idx[-1] + 1][1]
lit = [f for i in lit_idx for f in phases[i][1]]


def ref_at(t, drop=None):
    b = [f for f in before if f is not drop]
    a = [f for f in after if f is not drop]
    Db, Da = np.mean([load(f) for f in b], 0), np.mean([load(f) for f in a], 0)
    tb, ta = np.mean([f["t_first"] for f in b]), np.mean([f["t_first"] for f in a])
    w = np.clip((t - tb) / (ta - tb), 0, 1)
    return (1 - w) * Db + w * Da


ROI = (slice(160, 1120), slice(240, 1680))   # central region, away from the edges
bands = [(0, 320), (320, 640), (640, 960), (960, 1280)]

print(f"run {run.name}: {len(before)} darks before, {len(lit)} lit, {len(after)} darks after")
d = [load(f) for f in before]
tn = [np.std(d[i + 1] - d[i]) / np.sqrt(2) for i in range(len(d) - 1)]
print(f"temporal noise (consecutive warm darks): {np.median(tn):.2f} DN/frame")
print("dark pattern std by row band (last warm dark):",
      [round(float(d[-1][lo:hi].std()), 1) for lo, hi in bands])

# held-out darks: subtract the interpolated reference built without them
res = []
for f in (before[len(before) // 2], after[len(after) // 2]):
    r = load(f) - ref_at(f["t_first"], drop=f)
    res.append((f["file"], r.std(), [round(float(r[lo:hi].std()), 2) for lo, hi in bands]))
for name, s, b in res:
    print(f"held-out dark {name}: residual std {s:.2f} DN, by row band {b}")

# lit frames, dark-subtracted
S = [load(f) - ref_at(f["t_first"]) for f in lit]
K = [x[ROI].std() / x[ROI].mean() for x in S]
mu = np.mean([x[ROI].mean() for x in S])
print(f"lit (dark-subtracted) mean {mu:.1f} DN, K {np.mean(K):.4f} +- {np.std(K):.4f}")
cons = [np.corrcoef(S[i][ROI].ravel(), S[i + 1][ROI].ravel())[0, 1] for i in range(len(S) - 1)]
print(f"single vs next single (1 s apart): r {np.mean(cons):.4f}")
Sav = np.mean(S, 0)
print(f"sat pixels in lit frames: {np.mean([np.mean(load(f) >= 1023) for f in lit]):.4%}")

# composite comparison
C = [np.load(p).astype(np.float64) - comp_pedestal for p in sorted(comp.glob("*_[0-9][0-9][0-9].npy"))]
Kc = [x[ROI].std() / x[ROI].mean() for x in C]
print(f"composite ({len(C)} frames): mean {np.mean([x[ROI].mean() for x in C]):.1f} DN, K {np.mean(Kc):.4f}")
if len(C) > 1:
    print(f"composite vs composite: r {np.corrcoef(C[0][ROI].ravel(), C[1][ROI].ravel())[0, 1]:.4f}")
print(f"single vs composite: r {np.mean([np.corrcoef(x[ROI].ravel(), C[0][ROI].ravel())[0, 1] for x in S]):.4f}"
      f"   (single averaged over {len(S)}: r {np.corrcoef(Sav[ROI].ravel(), C[0][ROI].ravel())[0, 1]:.4f})")
# noise-limited expectation: speckle std vs added noise
sp = np.mean([x[ROI].std() for x in S])
print(f"speckle std in single lit {sp:.1f} DN; noise share of variance "
      f"{(np.median(tn) ** 2 * 2) / sp ** 2:.2%} (temporal x2 incl. reference)")
np.save(run / "single_dark_subtracted_mean.npy", Sav.astype(np.float32))
