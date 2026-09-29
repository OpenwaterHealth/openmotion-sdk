"""Assemble stride-composite full frames: frames whose first line%stride runs 0..N-1."""
import sys, numpy as np
from pathlib import Path
from PIL import Image
run = Path(sys.argv[1]); N = int(sys.argv[2])
d = np.load(run / "lines.npz"); ln, px, fc, t, fl = d["line"], d["px"], d["fc"], d["t"], d["flags"]
order = np.argsort(t, kind="stable"); ln, px, fc, t, fl = ln[order], px[order], fc[order], t[order], fl[order]
# split into frames by frame_cnt change
frames = []; cur = None
for i in range(len(ln)):
    if cur is None or fc[i] != cur["fc"]:
        cur = {"fc": int(fc[i]), "idx": []}; frames.append(cur)
    cur["idx"].append(i)
for f in frames:
    ls = ln[f["idx"]]; f["phase"] = int(ls.min() % N); f["n"] = len(ls); f["t"] = float(t[f["idx"][0]])
    f["ok"] = bool(np.all(ls % N == f["phase"]))
print("frames:", len(frames), "lines/frame:", sorted(set(f["n"] for f in frames)), "phase-consistent:", all(f["ok"] for f in frames),
      "overrun flags:", int((fl & 1).sum()))
print("phase sequence (first 45):", [f["phase"] for f in frames[:45]])
imgs = []; k = 0
while k < len(frames):
    if frames[k]["phase"] != 0: k += 1; continue
    grp = frames[k:k+N]
    if len(grp) < N or [g["phase"] for g in grp] != list(range(N)): k += 1; continue
    img = np.zeros((1280, 1920), np.uint16); have = np.zeros(1280, bool)
    for g in grp:
        img[ln[g["idx"]]] = px[g["idx"]]; have[ln[g["idx"]]] = True
    imgs.append((grp[0]["t"], grp[-1]["t"], have.sum(), img)); k += N
for j, (t0, t1, cov, img) in enumerate(imgs):
    print(f"composite {j}: coverage {cov}/1280  t={t0:.3f}..{t1:.3f}s ({t1 - t0:.3f}s)  mean {img.mean():.1f} std {img.std():.1f}")
    np.save(run / f"composite_{j}.npy", img)
    lo, hi = np.percentile(img, [0.5, 99.8]); v = np.clip((img.astype(float) - lo) / max(hi - lo, 1) * 255, 0, 255).astype(np.uint8)
    Image.fromarray(v).save(run / f"composite_{j}_view.png")
if len(imgs) > 1:
    print("composite period(s):", np.round(np.diff([i[0] for i in imgs]), 3).tolist())
