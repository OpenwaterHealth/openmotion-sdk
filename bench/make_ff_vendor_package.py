#!/usr/bin/env python3
"""Build the vendor data package of full-frame raw images through the warm-up dip.

From the ff_dip_capture steps of 2026-09-30 (FFDIP_01 = cams 1+2, FFDIP_02 =
cams 7+8), export for each requested camera:
  * every composite whose exposures centre within the first --early-s seconds
    after camera power-on (descent, trough, recovery), plus --n-plateau frames
    spread evenly over the plateau window;
  * each frame as a 16-bit grayscale PNG holding the raw 10-bit values unscaled;
  * per-row metadata in one NumPy file per camera: row exposure time since
    power-on, FPGA exposure counter, and lit / dark / partly-lit class;
  * an index CSV (time, die temperatures, dark-corrected mean, % of plateau);
plus each run's camera telemetry, source-off windows and photodiode log on the
same time base, the analysis figures, and a README.

Time zero is camera power-on, reconstructed from the soak log: the session
powers the scan mask, waits 0.5 s, then loads the FPGA of each camera in turn
(~10.5 s each); power-on = first load end - load time - 0.5 s.

Usage:
  python bench/make_ff_vendor_package.py --out C:/Users/openwater/Projects/vendor_packages
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import re
import shutil
import sys
import zipfile
from pathlib import Path

import numpy as np
import pandas as pd
from PIL import Image

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import analyze_ff_dip as A  # noqa: E402

RUNS = {  # step dir -> cameras to export (0-based)
    "ff_dip_out/FFDIP_01_cams12": [1],
    "ff_dip_out/FFDIP_02_cams78": [6, 7],
}
PLATEAU = (600.0, 1500.0)          # s after imaging start, as in analyze_ff_dip
NAME = "OX02C1B_fullframe_warmup_dip_2026-09-30"


def parse():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--early-s", type=float, default=420.0, help="export every frame up to this long after power-on")
    ap.add_argument("--n-plateau", type=int, default=20, help="plateau frames per camera")
    ap.add_argument("--no-zip", action="store_true")
    return ap.parse_args()


def power_on_epoch(step: Path, start: float) -> float:
    ev = pd.read_csv(step / "soak" / "events.csv")
    first = ev[ev.event == "open"].iloc[0]
    load_s = float(re.search(r"loaded in ([0-9.]+) s", first.detail).group(1))
    return start + float(first.t_s) - load_s - 0.5


def main():
    a = parse()
    pkg = a.out / NAME
    if pkg.exists():
        shutil.rmtree(pkg)
    (pkg / "figures").mkdir(parents=True)
    (pkg / "run_logs").mkdir()
    index_rows, summary = [], []
    for run_rel, cams in RUNS.items():
        step = (HERE / run_rel).resolve()
        run = step.name.split("_cams")[0]
        meta, start, darks, pd_log, tel = A.load_step(step)
        t0 = meta["t0_epoch"]
        p_on = power_on_epoch(step, start)
        factor, _ = A.source_factor(pd_log, darks, t0, PLATEAU)
        runinfo = json.loads((step / "soak" / "run.json").read_text())
        # run-level logs on the power-on time base
        tl = tel.copy()
        tl.insert(0, "t_since_power_on_s", (start + tl.t_s - p_on).round(3))
        tl["camera"] = tl.cam + 1
        tl.drop(columns=["t_s", "wall_time", "side", "cam"], errors="ignore").to_csv(
            pkg / "run_logs" / f"{run}_camera_telemetry.csv", index=False)
        dk = darks.copy()
        dk["off_s"] = (dk.off_epoch - p_on).round(3)
        dk["on_s"] = np.where(dk.on_epoch > dk.off_epoch + 1e5, np.nan, (dk.on_epoch - p_on).round(3))
        dk[["kind", "off_s", "on_s"]].rename(columns={"off_s": "source_off_s_since_power_on",
                                                       "on_s": "source_on_s_since_power_on"}).to_csv(
            pkg / "run_logs" / f"{run}_source_off_windows.csv", index=False)
        if pd_log is not None:
            pl = pd_log.assign(t_since_power_on_s=(pd_log.epoch_s - p_on).round(3))
            pl[["t_since_power_on_s", "power_w"]].rename(columns={"power_w": "photodiode_power_w"}).to_csv(
                pkg / "run_logs" / f"{run}_photodiode.csv", index=False)
        for cam in cams:
            files = sorted(str(f) for f in (step / "soak" / "images" / "left" / f"cam{cam}").glob("*.npz"))
            tr, ped, pedestal, ref, lag = A.analyze_cam(files, start, t0, darks, factor)
            sm = tr.set_index("t_s").lit_dc.rolling(3, center=True, min_periods=1).median()
            plat = float(sm[(sm.index >= PLATEAU[0]) & (sm.index <= PLATEAU[1])].median())
            body = sm[(sm.index > 0) & (sm.index <= 420)]
            t_min = float(body.idxmin())
            tr = tr.assign(t_pon=tr.t_s + (t0 - p_on))
            early = tr[tr.t_pon <= a.early_s]
            plat_all = tr[tr.t_s.between(*PLATEAU)]
            pick = np.linspace(0, len(plat_all) - 1, min(a.n_plateau, len(plat_all))).round().astype(int)
            sel = pd.concat([early, plat_all.iloc[np.unique(pick)]]).drop_duplicates("file")
            tc = tel[tel.cam == cam]
            tel_ep = (start + tc.t_s).to_numpy()
            cdir = pkg / f"cam{cam + 1}"
            (cdir / "frames").mkdir(parents=True)
            row_time, row_exp, row_cls, names = [], [], [], []
            for k, r in enumerate(sel.itertuples()):
                z = np.load(r.file)
                img = z["image"].astype(np.uint16)
                t_rows = start + z["row_t_s"] - lag
                rm = img[:, A.COL_LO:A.COL_HI].astype(np.float32).mean(axis=1)
                cls = A.classify(rm, ref).astype(np.uint8)
                name = f"cam{cam + 1}_{k:03d}_t{r.t_pon:07.1f}s.png"
                Image.fromarray(img).save(cdir / "frames" / name, compress_level=6)
                names.append(name)
                row_time.append((t_rows - p_on).astype(np.float32))
                row_exp.append(z["row_fc"].astype(np.int16))
                row_cls.append(cls)
                ep = p_on + r.t_pon

                def tel_at(col):
                    if len(tc) == 0 or ep < tel_ep.min() or ep > tel_ep.max():
                        return ""
                    return round(float(np.interp(ep, tel_ep, tc[col].to_numpy())), 2)
                index_rows.append({
                    "camera": cam + 1, "run": run, "file": f"cam{cam + 1}/frames/{name}",
                    "set": "early" if r.t_pon <= a.early_s else "plateau",
                    "t_since_power_on_s": round(r.t_pon, 2),
                    "first_row_s": round(float(np.min(row_time[-1])), 2),
                    "last_row_s": round(float(np.max(row_time[-1])), 2),
                    "n_exposures": int(len(np.unique(z["row_fc"]))),
                    "die_temp_tpm0_c": tel_at("tpm0_c"), "die_temp_tpm1_c": tel_at("tpm1_c"),
                    "die_temp_avg_c": tel_at("tpm_avg_c"),
                    "pedestal_dn": round(float(np.median(pedestal(t_rows))), 2),
                    "lit_mean_minus_pedestal_dn": round(r.lit_dc, 3),
                    "pct_of_plateau": round(100 * r.lit_dc / plat, 2),
                    "source_correction_factor": round(r.source_factor, 4),
                    "n_lit_rows": r.n_lit_rows, "n_dark_rows": r.n_dark_rows, "n_partial_rows": r.n_partial_rows,
                })
            np.savez_compressed(cdir / f"cam{cam + 1}_rows.npz", frame_file=np.array(names),
                                row_time_s_since_power_on=np.stack(row_time),
                                row_exposure_counter=np.stack(row_exp), row_class=np.stack(row_cls))
            # every composite of the run (not only exported frames), for the full curve
            tr.assign(camera=cam + 1, t_since_power_on_s=tr.t_pon.round(2),
                      pct_of_plateau=(100 * tr.lit_dc / plat).round(2))[
                ["camera", "t_since_power_on_s", "lit_dc", "pct_of_plateau", "source_factor"]].rename(
                columns={"lit_dc": "lit_mean_minus_pedestal_dn", "source_factor": "source_correction_factor"}).to_csv(
                cdir / f"cam{cam + 1}_all_frames_curve.csv", index=False)
            fig = step / "analysis" / f"ff_dip_cam{cam + 1}.png"
            if fig.exists():
                shutil.copy(fig, pkg / "figures" / f"cam{cam + 1}_dip_and_loss_map.png")
            summary.append({"camera": cam + 1, "run": run, "gain": runinfo["gains"][cam],
                            "plateau_dn": round(plat, 1), "dip_pct": round(100 * (1 - body.min() / plat), 1),
                            "trough_s_since_power_on": round(t_min + (t0 - p_on)),
                            "imaging_start_s": round(t0 - p_on, 1), "usb_lag_s": round(lag, 2),
                            "pedestal_dn": round(float(ped.pedestal.median()), 2),
                            "frames_early": int((sel.t_pon <= a.early_s).sum()),
                            "frames_plateau": int((sel.t_pon > a.early_s).sum())})
            print(f"cam {cam + 1}: {len(sel)} frames exported, dip {summary[-1]['dip_pct']}% "
                  f"@ {summary[-1]['trough_s_since_power_on']} s after power-on", flush=True)
    pd.DataFrame(index_rows).to_csv(pkg / "index.csv", index=False)
    shutil.copy(HERE / "ff_vendor_package_README.md", pkg / "README.md")   # vendor-facing description
    pd.DataFrame(summary).to_csv(pkg / "summary.csv", index=False)
    print(json.dumps(summary, indent=1))
    if not a.no_zip:
        zpath = a.out / f"{NAME}.zip"
        with zipfile.ZipFile(zpath, "w") as zf:
            for f in sorted(pkg.rglob("*")):
                if f.is_file():
                    comp = zipfile.ZIP_STORED if f.suffix in (".png", ".npz") else zipfile.ZIP_DEFLATED
                    zf.write(f, f.relative_to(a.out), compress_type=comp)
        print(f"zip {zpath} {zpath.stat().st_size / 1e6:.0f} MB")


if __name__ == "__main__":
    main()
