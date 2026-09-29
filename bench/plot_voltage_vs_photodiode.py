#!/usr/bin/env python3
"""Pop up a matplotlib window: commanded control voltage (reconstructed from
the logged dark-window on/off times, since drift_scan.py only logs the
toggle events, not a continuous voltage readback) vs. the Thorlabs photodiode
reading, both over the full 30-minute drift scan."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

parser = argparse.ArgumentParser()
parser.add_argument("--data-dir", type=Path, default=Path("bench/drift_scan_out"))
parser.add_argument("--subject-id", default="DRIFT30")
args = parser.parse_args()

meta = json.loads((args.data_dir / f"{args.subject_id}_drift_meta.json").read_text())
thorlabs = pd.read_csv(args.data_dir / f"{args.subject_id}_thorlabs.csv")

v_ctrl = meta["control_voltage"]
duration = meta["duration_sec"]

# Reconstruct the commanded step function from actual logged toggle times.
t_v = [0.0]
v_v = [v_ctrl]
for ev in meta["dark_events"]:
    off_t, on_t = ev["elapsed_off_sec"], ev["elapsed_on_sec"]
    t_v += [off_t, off_t, on_t, on_t]
    v_v += [v_ctrl, 0.0, 0.0, v_ctrl]
t_v.append(duration)
v_v.append(v_ctrl)

fig, ax1 = plt.subplots(figsize=(13, 5.5))
ax1.plot([t / 60 for t in t_v], v_v, color="#2a78d6", lw=1.3, label="Commanded control voltage")
ax1.set_xlabel("Time (min)")
ax1.set_ylabel("Control voltage (V)", color="#2a78d6")
ax1.tick_params(axis="y", labelcolor="#2a78d6")
ax1.set_ylim(-0.3, v_ctrl + 0.5)
ax1.grid(True, alpha=0.3)

ax2 = ax1.twinx()
ax2.plot(thorlabs["elapsed_s"] / 60, thorlabs["power"], color="#e34948", lw=0.6, alpha=0.85, label="Photodiode power")
ax2.set_ylabel("Photodiode power (W)", color="#e34948")
ax2.tick_params(axis="y", labelcolor="#e34948")

fig.suptitle(f"Commanded control voltage vs. Thorlabs photodiode -- {args.subject_id}")
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right")
fig.tight_layout()
plt.show()
