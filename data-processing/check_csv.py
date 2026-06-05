import argparse

import numpy as np
import pandas as pd

# Run this script with:
# set PYTHONPATH=%cd%;%PYTHONPATH%
# python data-processing/check_csv.py

CSV_FILE = "histogram.csv"
EXPECTED_SUM = 2457606
MAX_FRAME_ID = 255
EXPECTED_HZ = 40.0
EXPECTED_DT_MS = 1000.0 / EXPECTED_HZ  # 25.0 ms
TIMESTAMP_TOLERANCE = 0.5  # flag intervals deviating >50% from expected


def parse_args():
    parser = argparse.ArgumentParser(description="Check histogram CSV integrity")
    parser.add_argument(
        "--csv", type=str, required=True, help="Path to input CSV file to check"
    )
    return parser.parse_args()


def check_csv_integrity(csv_path):
    df = pd.read_csv(csv_path)

    df["frame_id"] = df["frame_id"].astype(int)
    df["cam_id"] = df["cam_id"].astype(int)
    df["sum"] = df["sum"].astype(int)

    errors_found = False
    error_counts = {
        "bad_sum": 0,
        "frame_id_skipped": 0,
        "bad_frame_cam_count": 0,
        "bad_timestamp_intervals": 0,
    }
    expected_fids = []

    # --- Add frame cycle index based on rollover detection ---
    frame_cycles = []
    last_frame_id = None
    cycle = 0
    for fid in df["frame_id"]:
        if last_frame_id is not None and fid < last_frame_id:
            cycle += 1
        frame_cycles.append(cycle)
        last_frame_id = fid
    df["frame_cycle"] = frame_cycles

    # --- Constraint 1: Check sum column ---
    bad_sums = df[df["sum"] != EXPECTED_SUM]
    if not bad_sums.empty:
        print(f"[ERROR] {len(bad_sums)} rows have incorrect 'sum' values.")
        error_counts["bad_sum"] += len(bad_sums)
        errors_found = True

    # --- Constraint 2 + 3: Verify frame_id sequencing and cam count ---
    grouped = df.groupby(["frame_cycle", "frame_id"], sort=True)
    expected_fid = None
    expected_cam_count = None

    for (cycle, fid), group in grouped:
        row_idx = group.index.min()  # First row index of this frame group
        cam_count = len(group)

        if expected_cam_count is None:
            expected_cam_count = cam_count
            print(
                f"[INFO] Setting expected cam_id count per frame to {expected_cam_count}"
            )

        if cam_count != expected_cam_count:
            print(
                f"[WARN] Row {row_idx}: frame_id {fid} (cycle {cycle}) has {cam_count} cam_ids, expected {expected_cam_count}"
            )
            error_counts["bad_frame_cam_count"] += 1
            errors_found = True

        if expected_fid is None:
            expected_fid = fid
        elif fid != expected_fid:
            print(
                f"[WARN] Row {row_idx}: frame_id skipped — expected {expected_fid}, got {fid} (cycle {cycle})"
            )
            num_skipped = (fid - expected_fid) % 256
            error_counts["frame_id_skipped"] += num_skipped
            expected_fids.append((str(cycle), str(expected_fid)))
            expected_fid = fid

        expected_fid = (expected_fid + 1) % 256

    # --- Constraint 4: Timestamp regularity (should be 40 Hz / 25 ms) ---
    if "timestamp_s" in df.columns:
        frame_groups = df.groupby(["frame_cycle", "frame_id"])["timestamp_s"].first()
        frame_groups = frame_groups.sort_index()
        frame_keys = list(frame_groups.index)
        frame_ts = frame_groups.values

        if len(frame_ts) >= 2:
            dt = np.diff(frame_ts) * 1000.0  # ms
            pos_mask = dt > 0
            pos_dt = dt[pos_mask]
            pos_indices = np.where(pos_mask)[0]

            if len(pos_dt) > 0:
                lo = EXPECTED_DT_MS * (1.0 - TIMESTAMP_TOLERANCE)
                hi = EXPECTED_DT_MS * (1.0 + TIMESTAMP_TOLERANCE)
                bad_mask = (pos_dt < lo) | (pos_dt > hi)
                bad_count = int(bad_mask.sum())
                error_counts["bad_timestamp_intervals"] = bad_count

                mean_dt = float(pos_dt.mean())
                std_dt = float(pos_dt.std())
                min_dt = float(pos_dt.min())
                max_dt = float(pos_dt.max())
                actual_hz = 1000.0 / mean_dt if mean_dt > 0 else 0

                total_span = frame_ts[-1] - frame_ts[0]
                print(
                    f"\n[INFO] Timestamp analysis "
                    f"({len(pos_dt)} frame intervals, {total_span:.2f}s total):"
                )
                print(f"  Expected: {EXPECTED_DT_MS:.1f}ms ({EXPECTED_HZ:.0f} Hz)")
                print(f"  Actual:   {mean_dt:.3f}ms ({actual_hz:.1f} Hz)")
                print(f"  Std dev:  {std_dt:.3f}ms")
                print(f"  Min:      {min_dt:.3f}ms")
                print(f"  Max:      {max_dt:.3f}ms")
                print(
                    f"  Out-of-tolerance intervals "
                    f"(>{TIMESTAMP_TOLERANCE*100:.0f}% off): "
                    f"{bad_count}/{len(pos_dt)}"
                )

                if bad_count > 0:
                    errors_found = True
                    bad_dts = pos_dt[bad_mask]
                    bad_pos_indices = np.where(bad_mask)[0]
                    print(f"\n  All {bad_count} bad interval(s):")
                    print(
                        f"  {'#':>4}  {'Frame A':>12}  {'Frame B':>12}  "
                        f"{'t_A (s)':>12}  {'t_B (s)':>12}  "
                        f"{'dt (ms)':>10}  {'Fault'}"
                    )
                    print(f"  {'-'*4}  {'-'*12}  {'-'*12}  {'-'*12}  {'-'*12}  {'-'*10}  {'-'*20}")
                    for i in range(bad_count):
                        orig_idx = pos_indices[bad_pos_indices[i]]
                        cyc_a, fid_a = frame_keys[orig_idx]
                        cyc_b, fid_b = frame_keys[orig_idx + 1]
                        ts_a = frame_ts[orig_idx]
                        ts_b = frame_ts[orig_idx + 1]
                        interval = bad_dts[i]

                        if interval > hi:
                            fault = f"GAP {interval:.1f}ms (expected {EXPECTED_DT_MS:.0f}ms)"
                        else:
                            fault = f"SHORT {interval:.1f}ms (expected {EXPECTED_DT_MS:.0f}ms)"

                        frame_a_str = f"c{cyc_a}:f{fid_a}"
                        frame_b_str = f"c{cyc_b}:f{fid_b}"
                        print(
                            f"  {i+1:>4}  {frame_a_str:>12}  {frame_b_str:>12}  "
                            f"{ts_a:>12.3f}  {ts_b:>12.3f}  "
                            f"{interval:>10.3f}  {fault}"
                        )
            else:
                print("\n[WARN] No positive timestamp intervals found.")
        else:
            print("\n[INFO] Fewer than 2 frames — skipping timestamp analysis.")
    else:
        print("\n[INFO] No 'timestamp_s' column — skipping timestamp analysis.")

    # --- Post-repair invariant checks (quality column) ---
    if "quality" in df.columns:
        print("\n[INFO] Post-repair invariant checks:")
        valid_qualities = {"ok", "ts_corrected", "nan_filled"}
        invalid = df[~df["quality"].isin(valid_qualities)]
        if not invalid.empty:
            print(f"  [ERROR] {len(invalid)} rows with invalid quality values")
            errors_found = True
        else:
            print(f"  Quality values: {dict(df['quality'].value_counts())}")

        # Check monotonic timestamps
        if "timestamp_s" in df.columns:
            ts = df["timestamp_s"].values
            non_mono = np.sum(np.diff(ts) < 0)
            if non_mono > 0:
                print(f"  [ERROR] {non_mono} non-monotonic timestamp transitions")
                errors_found = True
            else:
                print("  Timestamps: monotonic non-decreasing")

    # --- Summary ---
    print("\n[INFO] Histogram count per cam_id:")
    value_counts = df["cam_id"].value_counts().sort_index()
    print(value_counts)

    print("\n[INFO] Error type counts:")
    for k, v in error_counts.items():
        print(f"  {k}: {v}")

    # Print percentage of skipped frame_ids out of the average value of histogram counts
    if error_counts["frame_id_skipped"] > 0:
        skipped_percentage = (
            error_counts["frame_id_skipped"]
            / (value_counts.iloc[0] + error_counts["frame_id_skipped"])
        ) * 100
        print(f"[INFO] Percentage of skipped frame_ids: {skipped_percentage:.2f}%")

    # if expected_fids:
    #     print("\n[INFO] Expected frame_ids that were skipped:")
    #     print(expected_fids)

    if not errors_found:
        print("\n[PASS] CSV passed all integrity checks.")
    else:
        print("\n[FAIL] One or more checks failed.")


if __name__ == "__main__":
    args = parse_args()
    check_csv_integrity(args.csv)
