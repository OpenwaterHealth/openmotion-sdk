# Full-frame bench tools (2026-09-29 session)

These are the experiment scripts behind the numbers in `docs/FullFrameImages.md`
and the checkpoint comments on epic OpenwaterHealth/openmotion-bloodflow-app#480.
They are bench tools, not product code:
- Each one drives the rig directly and restores production registers on exit.
- Close the app first.
- Keep `DEBUG_FLAG_USB_PRINTF` off (it wedges COMM during image streaming).

For normal captures use `scripts/full_frame_1hz.py`.

| Script | What it does | Example (reproduces) |
|---|---|---|
| `ff_run.py` | One camera: bring-up in image mode, arbitrary timing/mode (`--mode trigger\|freerun`, `--hts/--vts/--expo/--maxexpo`), optional `--stride`, `--testpattern`, `--laser-delay`, `--fsin-off`. Saves every line to `lines.npz` and prints per-frame coverage. | Noise vs row time: `--mode freerun --hts 8000 --vts 1400 --expo 4 --collect-s 1.4` |
| `ratescan.py` | Sensor-only frame-rate scan across `hts:vts[:expo[:maxexpo]]` combos (reads the sensor's own frame counter `0x4610`). | Minimum VTS: `--mode freerun 432:1378 432:1360` |
| `cycles.py` | Assemble stride composites from an `ff_run.py --raw-collect --stride N` run (`cycles.py <run_dir> N`). | Phase sequence / coverage check |
| `assemble.py` | Assemble complete single-exposure frames from an `ff_run.py` run. | |
| `histocheck.py` | Production histogram path, laser off vs on, per-camera mean level (`histocheck.py <left\|right> 0xFF`). | "Does any camera see the laser?" |
| `laserdiag.py` | Console laser/safety/TEC/PDC readout while the TA trigger runs. | |

Output directories and the SDK log land in the current directory.
