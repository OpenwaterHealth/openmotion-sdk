#!/usr/bin/env python3
"""
illumination_sweep.py

Sweeps a uniform illumination source's control voltage (Keysight E36300
channel 3, 0-10 V) while the source's supply rail (channel 2) is held at a
fixed 24 V / 2 A constant-voltage output, and records the resulting optical
power on a Thorlabs power meter (PM160 / PM100-series) *and* the mean
intensity seen by each of the 8 cameras on the connected OpenMOTION sensor
module (treated as 8 more photodiodes) at each step.

Wiring assumed:
    PSU channel 2 -> illumination source supply rail (24 V, 2 A CV)
    PSU channel 3 -> illumination source control input (0-10 V)
    Thorlabs meter -> optical output of the illumination source
    Sensor module  -> USB, cameras exposed to the same illumination source

Requires both bench drivers in this directory:
    bench/keysight-psu/keysight_psu.py
    bench/thorlabs-pm100/read_thorlabs_powermeter.py
and the omotion SDK (this repo) for the sensor module.

Usage
-----
    python bench/illumination_sweep.py
        Run the default 0 -> 10 V sweep in 0.5 V steps, 1 s dwell per step,
        show a plot at the end.

    python bench/illumination_sweep.py --csv sweep_out.csv --plot sweep_out.png
        Also save the raw per-step data and the plot to disk.

    python bench/illumination_sweep.py --psu-resource USB0::... --meter-resource USB0::...
        Pass explicit VISA resource strings if auto-discovery finds more than
        one matching instrument on the bus.

    python bench/illumination_sweep.py --no-sensor
        Skip the sensor module and only sweep the Thorlabs meter (original
        behavior).
"""
from __future__ import annotations

import argparse
import csv as csv_module
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "keysight-psu"))
sys.path.insert(0, str(Path(__file__).resolve().parent / "thorlabs-pm100"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import pyvisa

from keysight_psu import KeysightE36300, PSUError

SUPPLY_CHANNEL = 2
CONTROL_CHANNEL = 3
CAMERA_MASK = 0xFF
N_CAMERAS = 8
CONFIGURE_TIMEOUT_S = 240.0


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--supply-voltage", type=float, default=24.0, help="Channel 2 CV setpoint. Default: 24.0 V.")
    parser.add_argument("--supply-current-limit", type=float, default=2.0, help="Channel 2 current limit. Default: 2.0 A.")
    parser.add_argument("--control-current-limit", type=float, default=0.5, help="Channel 3 current limit. Default: 0.5 A.")
    parser.add_argument("--start", type=float, default=0.0, help="Control sweep start volts. Default: 0.0.")
    parser.add_argument("--stop", type=float, default=10.0, help="Control sweep stop volts (inclusive). Default: 10.0.")
    parser.add_argument("--step", type=float, default=0.5, help="Control sweep step volts. Default: 0.5.")
    parser.add_argument("--dwell", type=float, default=1.0, help="Seconds to sample the meter at each step. Default: 1.0.")
    parser.add_argument("--sample-interval", type=float, default=0.05, help="Seconds between meter samples within a dwell. Default: 0.05.")
    parser.add_argument("--settle", type=float, default=0.1, help="Seconds to wait after stepping voltage before sampling. Default: 0.1.")
    parser.add_argument("--psu-resource", default=None, help="VISA resource string for the PSU. Auto-discovered if omitted.")
    parser.add_argument("--meter-resource", default=None, help="VISA resource string for the Thorlabs meter. Auto-discovered if omitted.")
    parser.add_argument("--csv", type=Path, default=None, help="Optional path to write per-step results as CSV.")
    parser.add_argument("--plot", type=Path, default=None, help="Optional path to save the plot as an image (in addition to showing it).")
    parser.add_argument("--no-show", action="store_true", help="Don't open an interactive plot window (useful when --plot is set and running headless).")
    parser.add_argument("--no-sensor", action="store_true", help="Skip the OpenMOTION sensor module; sweep the Thorlabs meter only.")
    parser.add_argument("--sensor-camera-plot", type=Path, default=None, help="Optional path to save the per-camera intensity plot as an image.")
    return parser.parse_args()


def _prime_vendored_libusb() -> None:
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
        from omotion.usb_backend import get_libusb1_backend

        get_libusb1_backend()
    except Exception as exc:
        print(f"[*] Could not prime vendored libusb ({exc}); falling back to system libusb-1.0.dll if any is on PATH.")


def _connect_meter(rm: pyvisa.ResourceManager, resource: str | None):
    from read_thorlabs_powermeter import THORLABS_VID, find_thorlabs_resource

    resources = list(rm.list_resources())
    resource_str = resource or find_thorlabs_resource(resources)
    if not resource_str:
        raise RuntimeError(
            f"No Thorlabs (VID {THORLABS_VID:#06x}) meter auto-detected among {resources}; pass --meter-resource."
        )
    inst = rm.open_resource(resource_str)
    inst.timeout = 3000
    inst.read_termination = "\n"
    inst.write_termination = "\n"
    idn = inst.query("*IDN?").strip()
    unit = inst.query("SENS:POW:DC:UNIT?").strip()
    print(f"[+] Meter: {idn} ({resource_str}), unit={unit}")
    return inst, unit


def sample_meter(inst, duration_s: float, interval_s: float) -> float:
    """Sample MEAS:POW? repeatedly for `duration_s` seconds and return the mean."""
    readings = []
    t_end = time.monotonic() + duration_s
    while True:
        readings.append(float(inst.query("MEAS:POW?")))
        if time.monotonic() >= t_end:
            break
        time.sleep(interval_s)
    return statistics.mean(readings)


def frange_inclusive(start: float, stop: float, step: float):
    n_steps = round((stop - start) / step)
    for i in range(n_steps + 1):
        yield round(start + i * step, 10)


def connect_sensor():
    """Start MotionInterface, wait for one sensor module, power + configure
    all 8 cameras (mask 0xFF), and return (iface, sensor)."""
    from omotion import MotionInterface
    from omotion.ScanWorkflow import ConfigureRequest

    iface = MotionInterface(data_dir="illum_sweep_out")
    iface.start()
    if not iface.wait_for_ready(console=False, sensors=1, timeout=15):
        iface.stop()
        raise RuntimeError("No sensor module connected within 15 s.")

    sensors = iface.connected_sensors()
    sensor = sensors[0]
    side = "left" if sensor is iface.left else "right"
    print(f"[+] Sensor module connected on {side}")

    sensor.enable_camera_power(CAMERA_MASK)
    left_mask = CAMERA_MASK if side == "left" else 0x00
    right_mask = CAMERA_MASK if side == "right" else 0x00

    done = threading.Event()
    box: dict = {}

    def _on_complete(result) -> None:
        box["result"] = result
        done.set()

    accepted = iface.start_configure_camera_sensors(
        ConfigureRequest(left_camera_mask=left_mask, right_camera_mask=right_mask,
                          power_off_unused_cameras=False),
        on_complete_fn=_on_complete,
    )
    if not accepted:
        iface.stop()
        raise RuntimeError("Camera configure refused (already running?).")
    if not done.wait(CONFIGURE_TIMEOUT_S):
        iface.stop()
        raise RuntimeError(f"Camera configure timed out after {CONFIGURE_TIMEOUT_S:.0f}s")
    result = box.get("result")
    if result is None or not result.ok:
        iface.stop()
        raise RuntimeError(f"Camera configure failed: {result.error if result else 'no result'}")
    print(f"[+] Cameras configured (mask 0x{CAMERA_MASK:02X})")

    return iface, sensor


def read_camera_intensities(sensor) -> list[float]:
    """Capture one histogram per camera (real sensor data, test_pattern_id=4)
    and return the mean pixel intensity (0-1023) for each of the 8 cameras."""
    bins = np.arange(1024, dtype=np.float64)
    intensities = []
    for camera_id in range(N_CAMERAS):
        result = sensor.get_camera_histogram(camera_id, test_pattern_id=4, auto_upload=False)
        if result is None:
            intensities.append(float("nan"))
            continue
        integers, _hidden = result
        histo = np.asarray(integers, dtype=np.float64)
        total = histo.sum()
        intensities.append(float((bins * histo).sum() / total) if total > 0 else float("nan"))
    return intensities


def main() -> int:
    args = parse_cli()
    _prime_vendored_libusb()

    print("[*] Connecting to PSU ...")
    psu = KeysightE36300.connect(resource_name=args.psu_resource)
    print(f"[+] PSU: {psu}")

    rm = pyvisa.ResourceManager("@py")
    meter, unit = _connect_meter(rm, args.meter_resource)

    sensor_iface = None
    sensor = None
    if not args.no_sensor:
        print("[*] Connecting to sensor module ...")
        sensor_iface, sensor = connect_sensor()

    voltages: list[float] = []
    powers: list[float] = []
    camera_intensities: list[list[float]] = []  # one row of 8 per step

    try:
        # Supply rail: fixed 24 V / 2 A constant voltage.
        psu.set_current_limit(SUPPLY_CHANNEL, args.supply_current_limit)
        psu.set_voltage(SUPPLY_CHANNEL, args.supply_voltage)

        # Control line: start at 0 V.
        psu.set_current_limit(CONTROL_CHANNEL, args.control_current_limit)
        psu.set_voltage(CONTROL_CHANNEL, 0.0)

        psu.set_output(SUPPLY_CHANNEL, True)
        psu.set_output(CONTROL_CHANNEL, True)
        print(
            f"[*] Outputs on: ch{SUPPLY_CHANNEL}={args.supply_voltage}V/{args.supply_current_limit}A (CV), "
            f"ch{CONTROL_CHANNEL}=0V control"
        )

        for setpoint in frange_inclusive(args.start, args.stop, args.step):
            psu.set_voltage(CONTROL_CHANNEL, setpoint)
            time.sleep(args.settle)
            actual_v = psu.measure_voltage(CONTROL_CHANNEL)
            power = sample_meter(meter, args.dwell, args.sample_interval)
            voltages.append(actual_v)
            powers.append(power)

            if sensor is not None:
                cam_vals = read_camera_intensities(sensor)
                camera_intensities.append(cam_vals)
                cam_str = " ".join(f"{v:6.1f}" for v in cam_vals)
                print(f"  ctrl={setpoint:5.2f} V (meas {actual_v:5.3f} V) -> {power:.6e} {unit} | cams: {cam_str}")
            else:
                print(f"  ctrl={setpoint:5.2f} V (meas {actual_v:5.3f} V) -> {power:.6e} {unit}")

    finally:
        print("[*] Ramping control to 0 V and disabling outputs ...")
        try:
            psu.set_voltage(CONTROL_CHANNEL, 0.0)
            psu.set_output(CONTROL_CHANNEL, False)
            psu.set_output(SUPPLY_CHANNEL, False)
        except PSUError as exc:
            print(f"[!] Error while shutting down PSU outputs: {exc}")
        psu.close()
        meter.close()
        if sensor is not None:
            try:
                sensor.disable_camera_power(CAMERA_MASK)
            except Exception as exc:
                print(f"[!] Error while powering off cameras: {exc}")
        if sensor_iface is not None:
            sensor_iface.stop()

    if args.csv:
        with args.csv.open("w", newline="") as f:
            writer = csv_module.writer(f)
            header = ["control_voltage_v", "power", "unit"]
            if camera_intensities:
                header += [f"cam{i}_mean_intensity" for i in range(N_CAMERAS)]
            writer.writerow(header)
            for i, (v, p) in enumerate(zip(voltages, powers)):
                row = [f"{v:.4f}", p, unit]
                if camera_intensities:
                    row += [f"{x:.4f}" for x in camera_intensities[i]]
                writer.writerow(row)
        print(f"[+] Wrote {args.csv}")

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(voltages, powers, marker="o")
    ax.set_xlabel("Control voltage (V)")
    ax.set_ylabel(f"Illumination ({unit}, log scale)")
    ax.set_yscale("log")
    ax.set_title("Illumination vs. control voltage (Thorlabs)")
    ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()

    if args.plot:
        fig.savefig(args.plot, dpi=150)
        print(f"[+] Saved plot to {args.plot}")

    if camera_intensities:
        cam_array = np.array(camera_intensities)  # (n_steps, 8)
        fig2, ax2 = plt.subplots(figsize=(8, 5))
        for cam_id in range(N_CAMERAS):
            ax2.plot(voltages, cam_array[:, cam_id], marker="o", label=f"cam {cam_id + 1}")
        ax2.set_xlabel("Control voltage (V)")
        ax2.set_ylabel("Mean pixel intensity (0-1023, log scale)")
        ax2.set_yscale("log")
        ax2.set_title("Per-camera mean intensity vs. control voltage")
        ax2.grid(True, which="both", alpha=0.3)
        ax2.legend(ncol=4, fontsize=8)
        fig2.tight_layout()

        if args.sensor_camera_plot:
            fig2.savefig(args.sensor_camera_plot, dpi=150)
            print(f"[+] Saved per-camera plot to {args.sensor_camera_plot}")

    if not args.no_show:
        plt.show()

    return 0


if __name__ == "__main__":
    sys.exit(main())
