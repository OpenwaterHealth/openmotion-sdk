import sys, numpy as np
from pathlib import Path
from PIL import Image
run = Path(sys.argv[1]); d = np.load(run / "lines.npz")
fcs = sorted(set(d["fc"].tolist()))
done = []
for fc in fcs:
    sel = d["fc"] == fc
    ln = d["line"][sel]; px = d["px"][sel]
    if len(set(ln.tolist())) != 1280:
        continue
    img = np.zeros((1280, 1920), np.uint16); img[ln] = px
    np.save(run / f"frame_fc{fc}.npy", img)
    lo, hi = np.percentile(img, [0.5, 99.8])
    v = np.clip((img.astype(float) - lo) / max(hi - lo, 1) * 255, 0, 255).astype(np.uint8)
    Image.fromarray(v).save(run / f"frame_fc{fc}_view.png")
    Image.fromarray(img).save(run / f"frame_fc{fc}_raw16.png")
    done.append(fc)
    print(f"fc={fc}: complete; mean {img.mean():.1f} min {img.min()} max {img.max()} p1/p99 {np.percentile(img,1):.0f}/{np.percentile(img,99):.0f}; stretch {lo:.0f}-{hi:.0f}")
print("complete frames:", done)
