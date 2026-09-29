"""1 Hz full-frame camera images (stride composite, camera-fpga map v3).

Streams complete 1920x1280 RAW10 frames from one camera, one per second, at
production image quality: 40 Hz laser-synced frames, every 40th line per
frame, phase advancing per frame (see omotion.ImageCapture
capture_composite_frames and epic OpenwaterHealth/openmotion-bloodflow-app#480).

Output per frame (all lossless except the preview): <side>_cam<N>_<k>_raw16.png
(16-bit PNG, raw 10-bit values unscaled), <..>.npy (uint16), <..>_preview8.png
(8-bit contrast-stretched, viewing only), plus meta.json.

Needs: sensor firmware with drip-scan image mode (sensor-fw feature/99) and
the stride bitstream (camera-fpga map v3) in the sensor's flash. Close the
app first (USB access is exclusive).

    python scripts/full_frame_1hz.py --side left --cam 0 --frames 10 --out ff_out
    python scripts/full_frame_1hz.py --side left --cam 0 --frames 30 --live --no-laser
"""
import argparse
import json
import logging
import time
from pathlib import Path

import numpy as np

from omotion import MotionInterface
from omotion.ImageCapture import capture_composite_frames


def _preview(img: np.ndarray) -> np.ndarray:
    lo, hi = np.percentile(img, [0.5, 99.8])
    return np.clip((img.astype(np.float32) - lo) / max(hi - lo, 1.0) * 255.0,
                   0, 255).astype(np.uint8)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--side", choices=("left", "right"), default="left")
    ap.add_argument("--cam", type=int, nargs="+", default=[0],
                    help="camera index/indices 0-7 (several = concurrent)")
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--out", default="full_frame_out")
    ap.add_argument("--no-laser", action="store_true", help="TA trigger off (dark / ambient)")
    ap.add_argument("--no-fpga-load", action="store_true",
                    help="skip the forced FPGA SRAM load (already loaded this power cycle)")
    ap.add_argument("--live", action="store_true", help="show each frame in a window")
    ap.add_argument("--laser-delay", type=int, default=None,
                    help="LaserPulseDelayUsec override (us after FSIN); default = production")
    a = ap.parse_args()

    logging.basicConfig(level=logging.WARNING)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image
    except ImportError:                       # pragma: no cover
        Image = None

    fig = im = None
    if a.live:
        import matplotlib.pyplot as plt
        plt.ion()
        fig, ax = plt.subplots(figsize=(9.6, 6.6))
        im = ax.imshow(np.zeros((1280, 1920), np.uint8), cmap="gray", vmin=0, vmax=255)
        ax.set_axis_off()

    iface = MotionInterface(data_dir=str(out / "sdk"))
    iface.start(wait=True, wait_timeout=3.0)
    iface.wait_for_ready(console=True, sensors=1, timeout=20)
    sensor = getattr(iface, a.side)
    if sensor is None or sensor.uart is None:
        print(f"{a.side} sensor not connected")
        return 1
    laser = not a.no_laser
    if laser and not iface.apply_laser_power():
        print("apply_laser_power failed")
        return 1

    meta = []
    t0 = time.monotonic()

    def on_frame(f):
        k = sum(1 for m in meta if m["cam"] == f.cam_id)
        stem = f"{a.side}_cam{f.cam_id}_{k:03d}"
        np.save(out / f"{stem}.npy", f.image)
        if Image is not None:
            Image.fromarray(f.image).save(out / f"{stem}_raw16.png")
            # 8-bit contrast-stretched copy for eyeballing ONLY -- not data.
            Image.fromarray(_preview(f.image)).save(out / f"{stem}_preview8.png")
        meta.append({"file": stem, "cam": f.cam_id, "t_s": round(f.t_last - t0, 3),
                     "frame_cnts": f.frame_cnts, "lines": f.lines,
                     "overrun": f.overrun, "mean": float(f.image.mean()),
                     "std": float(f.image.std())})
        print(f"[{f.t_last - t0:7.3f}s] cam{f.cam_id} frame {k}: {f.lines} lines from "
              f"{len(f.frame_cnts)} exposures, mean {f.image.mean():.1f}"
              f"{'  OVERRUN' if f.overrun else ''}", flush=True)
        if im is not None and f.cam_id == a.cam[0]:
            im.set_data(_preview(f.image))
            fig.canvas.draw_idle()
            fig.canvas.flush_events()

    try:
        capture_composite_frames(sensor, iface.console, a.cam, n_frames=a.frames,
                                 laser=laser, load_fpga=not a.no_fpga_load,
                                 on_frame=on_frame, laser_delay_us=a.laser_delay)
    finally:
        (out / "meta.json").write_text(json.dumps(
            {"side": a.side, "cam": a.cam, "laser": laser, "laser_delay_us": a.laser_delay,
             "formats": {
                 "*_raw16.png": "lossless 16-bit greyscale PNG of the raw sensor "
                                "values (10-bit, 0..1023, unscaled)",
                 "*.npy": "same data, uint16 1280x1920 (lossless)",
                 "*_preview8.png": "8-bit contrast-stretched preview for viewing only",
             },
             "frames": meta},
            indent=2))
        iface.stop()
    for c in a.cam:
        ts = [m["t_s"] for m in meta if m["cam"] == c]
        if len(ts) > 1:
            dt = np.diff(ts)
            print(f"cam{c}: {len(ts)} frames, period {dt.mean():.3f} s "
                  f"(min {dt.min():.3f}, max {dt.max():.3f})")
    return 0 if len(meta) == a.frames * len(a.cam) else 2


if __name__ == "__main__":
    raise SystemExit(main())
