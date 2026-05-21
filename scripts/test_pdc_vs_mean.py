#!/usr/bin/env python3
"""End-to-end correlation test: per-frame PDC vs per-camera image mean.

Runs a full ScanWorkflow scan for ``--duration`` seconds (default 600 = 10 min).
- Subscribes to ConsoleTelemetryPoller.add_pdc_listener for per-frame PDC.
- Subscribes to ScanWorkflow's on_uncorrected_fn for per-camera-per-frame Sample
  (which carries the pedestal-subtracted mean computed from the histogram).
- After the scan, joins the two streams on time, computes Pearson correlation
  per camera, and writes:
    - ``pdc_vs_mean_<ts>_timeseries.png`` — PDC and per-camera means over time
    - ``pdc_vs_mean_<ts>_scatter.png``   — scatter plot per camera
    - ``pdc_vs_mean_<ts>_corr.csv``      — correlation coefficient per camera

Run from openmotion-sdk:

    python scripts/test_pdc_vs_mean.py --duration 600 --subject pdc_corr_test
"""
from __future__ import annotations

import argparse
import datetime
import logging
import threading
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt
import numpy as np

from omotion.ConsoleTelemetry import PdcSample
from omotion.MotionInterface import MotionInterface
from omotion.ScanWorkflow import ScanRequest, ConfigureRequest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=600,
                        help="Scan duration in seconds (default 600 = 10 min)")
    parser.add_argument("--subject", type=str, default="pdc_corr_test")
    parser.add_argument("--data-dir", type=Path, default=Path("scan_data"))
    parser.add_argument("--left-mask", type=lambda s: int(s, 0), default=0xFF)
    parser.add_argument("--right-mask", type=lambda s: int(s, 0), default=0xFF)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    log = logging.getLogger("test_pdc_vs_mean")

    iface = MotionInterface()
    iface.start(wait=False)

    log.info("Waiting for console + sensors (up to 15 s)…")
    iface.wait_for_ready(console=True, sensors=2, timeout=15.0)
    if not iface.console.is_connected():
        log.error("Console not connected — aborting")
        iface.stop()
        return 2
    if not iface.connected_sensors():
        log.error("No sensors connected — aborting")
        iface.stop()
        return 2
    log.info("Console + %d sensor(s) connected: %s.",
             len(iface.connected_sensors()),
             [s.side for s in iface.connected_sensors()])

    # Power on the cameras explicitly so they're READY for FPGA programming.
    # (Skipping configure_workflow's power_off_unused_cameras=True branch,
    # which can leave the device in a weird state per user guidance.)
    log.info("Enabling camera power…")
    for sensor in iface.connected_sensors():
        mask = args.left_mask if sensor.side == "left" else args.right_mask
        if mask:
            ok = sensor.enable_camera_power(mask & 0xFF)
            log.info("  %s enable_camera_power(0x%02X) -> %s", sensor.side, mask, ok)
    time.sleep(0.5)  # let the cameras settle to READY

    # Configure cameras — programs FPGAs and prepares sensors. This is the
    # step the bloodflow-app calls before every scan.
    log.info("Configuring cameras (masks L=0x%02X R=0x%02X)…", args.left_mask, args.right_mask)
    cfg_done = threading.Event()
    cfg_result_box: list = [None]

    def on_cfg_complete(r) -> None:
        cfg_result_box[0] = r
        cfg_done.set()

    cfg_req = ConfigureRequest(
        left_camera_mask=args.left_mask,
        right_camera_mask=args.right_mask,
        power_off_unused_cameras=False,
    )
    started = iface.start_configure_camera_sensors(
        cfg_req,
        on_log_fn=lambda msg: log.info("config: %s", msg),
        on_complete_fn=on_cfg_complete,
    )
    if not started:
        log.error("Configure refused to start")
        iface.stop()
        return 2
    if not cfg_done.wait(timeout=120.0):
        log.error("Configure timed out")
        iface.stop()
        return 2
    cfg_r = cfg_result_box[0]
    if cfg_r is None or not cfg_r.ok:
        log.error("Configure failed: %s", getattr(cfg_r, "error", "?"))
        iface.stop()
        return 2
    log.info("Configure complete.")

    # ---- Data collection ----
    pdc_lock = threading.Lock()
    pdc_samples: list[PdcSample] = []

    def on_pdc(s: PdcSample) -> None:
        with pdc_lock:
            pdc_samples.append(s)

    iface.console.telemetry.add_pdc_listener(on_pdc)

    mean_lock = threading.Lock()
    # mean_samples[(side, cam_id)] = list of (timestamp_s, absolute_frame_id, mean, contrast)
    mean_samples: dict[tuple[str, int], list[tuple[float, int, float, float]]] = {}

    def on_uncorrected(sample) -> None:
        # `sample` is a science-pipeline Sample (see MotionProcessing.py)
        if sample.is_corrected:
            return
        with mean_lock:
            mean_samples.setdefault((sample.side, sample.cam_id), []).append(
                (sample.timestamp_s, sample.absolute_frame_id,
                 float(sample.mean), float(sample.contrast))
            )

    args.data_dir.mkdir(parents=True, exist_ok=True)

    req = ScanRequest(
        subject_id=args.subject,
        duration_sec=args.duration,
        left_camera_mask=args.left_mask,
        right_camera_mask=args.right_mask,
        data_dir=str(args.data_dir),
        disable_laser=False,
        write_raw_csv=False,           # we don't need raw histograms — we have on_uncorrected_fn
        write_corrected_csv=True,
        write_telemetry_csv=True,
        reduced_mode=False,
    )

    scan_done = threading.Event()
    scan_result_box: list = [None]

    def on_complete(result) -> None:
        scan_result_box[0] = result
        scan_done.set()

    log.info("Starting %ds scan (subject=%s, masks L=0x%02X R=0x%02X)…",
             args.duration, args.subject, args.left_mask, args.right_mask)
    t_start = time.time()
    ok = iface.start_scan(
        req,
        on_uncorrected_fn=on_uncorrected,
        on_complete_fn=on_complete,
        on_log_fn=lambda msg: log.info("scan: %s", msg),
    )
    if not ok:
        log.error("start_scan refused")
        iface.console.telemetry.remove_pdc_listener(on_pdc)
        iface.stop()
        return 3

    log.info("Scan running. Waiting for completion (timeout %ds + 30s margin)…", args.duration)
    if not scan_done.wait(timeout=args.duration + 30):
        log.error("Scan did not complete in time")
        iface.console.telemetry.remove_pdc_listener(on_pdc)
        iface.stop()
        return 4

    scan_elapsed = time.time() - t_start
    iface.console.telemetry.remove_pdc_listener(on_pdc)

    result = scan_result_box[0]
    log.info("Scan complete in %.1fs. ok=%s left=%s right=%s telemetry=%s",
             scan_elapsed, result.ok if result else "?",
             getattr(result, "left_path", "?"), getattr(result, "right_path", "?"),
             getattr(result, "telemetry_path", "?"))

    with pdc_lock:
        pdc_snap = list(pdc_samples)
    with mean_lock:
        mean_snap = {k: list(v) for k, v in mean_samples.items()}

    log.info("Collected: %d PDC samples, %d cameras with mean data",
             len(pdc_snap), len(mean_snap))
    for k, v in mean_snap.items():
        log.info("  %s cam %d: %d mean samples", k[0], k[1], len(v))

    if not pdc_snap or not mean_snap:
        log.error("Insufficient data for correlation")
        iface.stop()
        return 5

    # ---- Analysis ----
    # Build a single time-aligned series. The host_recv_timestamp on PdcSample
    # is the wall-clock time the SDK received the drain response; sample.timestamp_s
    # is the science-pipeline's frame timestamp (parsed from the histogram). Both
    # are wall clock; align by interpolating PDC onto each camera's mean timestamps.
    pdc_t = np.array([s.host_recv_timestamp for s in pdc_snap])
    pdc_v = np.array([s.pdc_mA for s in pdc_snap])
    # Normalize PDC times to 0 = first PDC sample (matches what the science
    # pipeline does for its sample.timestamp_s).
    pdc_t0 = pdc_t.min()
    pdc_t -= pdc_t0

    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_prefix = args.data_dir / f"pdc_vs_mean_{ts}"

    # ---- Plot 1: time series ----
    cam_keys = sorted(mean_snap.keys())
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    axes[0].plot(pdc_t, pdc_v, color="black", lw=0.6)
    axes[0].set_ylabel("PDC (mA)")
    axes[0].set_title(f"Per-frame PDC vs camera image means — {args.duration}s scan")
    axes[0].grid(True, alpha=0.3)

    for (side, cam_id), samples in cam_keys and [(k, mean_snap[k]) for k in cam_keys] or []:
        if not samples:
            continue
        t = np.array([s[0] for s in samples])
        m = np.array([s[2] for s in samples])
        # Normalize to the same origin as PDC
        # (mean timestamps from science pipeline are already 0-based per scan)
        axes[1].plot(t, m, lw=0.5, label=f"{side[0].upper()}{cam_id}")
    axes[1].set_xlabel("Time since scan start (s)")
    axes[1].set_ylabel("Image mean (pedestal-subtracted)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(fontsize=7, ncol=4, loc="upper right")
    fig.tight_layout()
    fig.savefig(f"{out_prefix}_timeseries.png", dpi=120)
    plt.close(fig)
    log.info("Wrote %s_timeseries.png", out_prefix)

    # ---- Plot 2: scatter per camera ----
    n_cams = len(cam_keys)
    n_cols = 4
    n_rows = (n_cams + n_cols - 1) // n_cols
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(4 * n_cols, 3 * n_rows),
                             squeeze=False)
    corr_rows: list[tuple[str, int, float, int]] = []
    for idx, (side, cam_id) in enumerate(cam_keys):
        r, c = divmod(idx, n_cols)
        ax = axes[r][c]
        samples = mean_snap[(side, cam_id)]
        t_m = np.array([s[0] for s in samples])
        m = np.array([s[2] for s in samples])
        # Interpolate PDC onto each camera's mean timestamps.
        # (Both time series share a wall-clock origin within ~ms.)
        pdc_interp = np.interp(t_m, pdc_t, pdc_v, left=np.nan, right=np.nan)
        # Drop rows where interp returned NaN (samples outside PDC time range)
        valid = ~np.isnan(pdc_interp)
        m_v = m[valid]
        p_v = pdc_interp[valid]
        if len(m_v) >= 10:
            corr = float(np.corrcoef(p_v, m_v)[0, 1])
        else:
            corr = float("nan")
        corr_rows.append((side, cam_id, corr, len(m_v)))
        ax.scatter(p_v, m_v, s=2, alpha=0.3)
        ax.set_xlabel("PDC (mA)")
        ax.set_ylabel("Image mean")
        ax.set_title(f"{side[0].upper()}{cam_id}  r={corr:+.3f}  n={len(m_v)}")
        ax.grid(True, alpha=0.3)
    # Hide unused axes
    for idx in range(n_cams, n_rows * n_cols):
        r, c = divmod(idx, n_cols)
        axes[r][c].set_visible(False)
    fig.suptitle(f"Per-camera correlation: image mean vs per-frame PDC", y=1.00)
    fig.tight_layout()
    fig.savefig(f"{out_prefix}_scatter.png", dpi=120)
    plt.close(fig)
    log.info("Wrote %s_scatter.png", out_prefix)

    # ---- CSV of correlation coefficients ----
    corr_csv = f"{out_prefix}_corr.csv"
    with open(corr_csv, "w", encoding="utf-8") as fh:
        fh.write("side,cam_id,pearson_r,n_samples\n")
        for side, cam_id, r, n in corr_rows:
            fh.write(f"{side},{cam_id},{r:.6f},{n}\n")
    log.info("Wrote %s", corr_csv)

    print()
    print("=" * 70)
    print(f"CORRELATION RESULTS — {args.duration}s scan, {len(pdc_snap)} PDC samples")
    print("=" * 70)
    print(f"{'Camera':<10}{'pearson r':>12}{'n samples':>12}")
    print("-" * 34)
    for side, cam_id, r, n in corr_rows:
        print(f"{side[0].upper()}{cam_id:<9}{r:>+12.4f}{n:>12d}")
    print("=" * 70)
    print(f"Outputs in {args.data_dir}/")
    print(f"  pdc_vs_mean_{ts}_timeseries.png")
    print(f"  pdc_vs_mean_{ts}_scatter.png")
    print(f"  pdc_vs_mean_{ts}_corr.csv")

    iface.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
