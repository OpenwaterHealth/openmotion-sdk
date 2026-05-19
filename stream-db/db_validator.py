"""
db_validator.py - Validate imported database content against scan CSV files.

Usage
-----
python db_validator.py ow98NSF5 scan_data data/sqlite.db
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from contextlib import ExitStack
from pathlib import Path
from typing import Dict, Iterator, List, Optional

# Make the parent omotion-sdk repo importable when this script is run
# standalone from inside stream-db/ (issue #92 — scan_db.py moved into
# the omotion package).
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from importer import (
    ScanGroup,
    build_raw_row,
    build_session_meta,
    discover_scan_groups,
    iter_frame_groups,
    read_notes,
)
from omotion import ScanDatabase


def main() -> int:
    args = parse_args()
    scan_dir = Path(args.scan_dir)
    db_path = Path(args.db_path)

    if not scan_dir.exists():
        raise FileNotFoundError(f"Scan directory does not exist: {scan_dir}")
    if not db_path.exists():
        raise FileNotFoundError(f"Database does not exist: {db_path}")

    scan_groups = discover_scan_groups(args.session_label, scan_dir)
    if not scan_groups:
        raise FileNotFoundError(
            f"No scan files found for label '{args.session_label}' in {scan_dir}"
        )

    with ScanDatabase(db_path=db_path) as db:
        failures: List[str] = []
        validated_sessions = 0
        validated_rows = 0

        for group in scan_groups:
            session_failures, row_count = validate_scan_group(db, group)
            failures.extend(session_failures)
            if not session_failures:
                validated_sessions += 1
                validated_rows += row_count

    if failures:
        print("Validation failed.")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print(
        f"Validation succeeded for label {args.session_label}: "
        f"{validated_sessions} session(s), {validated_rows} raw frame row(s) matched."
    )
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate imported SQLite data against scan CSV source files."
    )
    parser.add_argument(
        "session_label",
        help="Session label to validate, for example ow98NSF5.",
    )
    parser.add_argument(
        "scan_dir",
        help="Directory containing scan data files.",
    )
    parser.add_argument(
        "db_path",
        help="SQLite database file to validate.",
    )
    return parser.parse_args()


def validate_scan_group(db: ScanDatabase, group: ScanGroup) -> tuple[List[str], int]:
    failures: List[str] = []
    expected_session_label = f"{group.label}_{group.timestamp_key}"
    session = db.get_session_by_label(expected_session_label)
    if session is None:
        return ([f"Missing session '{expected_session_label}' in database."], 0)

    expected_notes = read_notes(group.notes_path)
    expected_meta = build_session_meta(group)
    expected_start = group.scan_datetime.timestamp()

    if session["session_notes"] != expected_notes:
        failures.append(
            f"Session '{expected_session_label}' notes mismatch."
        )

    if session["session_meta"] != expected_meta:
        failures.append(
            f"Session '{expected_session_label}' metadata mismatch."
        )

    if float(session["session_start"]) != float(expected_start):
        failures.append(
            f"Session '{expected_session_label}' start time mismatch: "
            f"db={session['session_start']} csv={expected_start}"
        )

    db_rows = list(db.iter_raw_frames(int(session["id"])))
    expected_rows = list(iter_expected_rows(int(session["id"]), group))

    if len(db_rows) != len(expected_rows):
        failures.append(
            f"Session '{expected_session_label}' raw row count mismatch: "
            f"db={len(db_rows)} csv={len(expected_rows)}"
        )
        return failures, 0

    for index, (db_row, expected_row) in enumerate(zip(db_rows, expected_rows), start=1):
        mismatch = compare_raw_rows(db_row, expected_row)
        if mismatch is not None:
            failures.append(
                f"Session '{expected_session_label}' row {index} mismatch: {mismatch}"
            )
            return failures, 0

    return failures, len(expected_rows)


def iter_expected_rows(session_id: int, group: ScanGroup) -> Iterator[Dict[str, object]]:
    with ExitStack() as stack:
        frame_group_iters: Dict[str, Iterator[List[Dict[str, object]]]] = {}
        for side in ("left", "right"):
            sensor = group.sensor_files.get(side)
            if sensor is None:
                continue
            handle = stack.enter_context(
                sensor.path.open("r", encoding="utf-8", newline="")
            )
            reader = csv.DictReader(handle)
            frame_group_iters[side] = iter_frame_groups(session_id, sensor.side, reader)

        while frame_group_iters:
            exhausted_sides: List[str] = []
            for side in ("left", "right"):
                frame_iter = frame_group_iters.get(side)
                if frame_iter is None:
                    continue
                try:
                    for row in next(frame_iter):
                        yield row
                except StopIteration:
                    exhausted_sides.append(side)

            for side in exhausted_sides:
                frame_group_iters.pop(side, None)


def compare_raw_rows(
    db_row: Dict[str, object],
    expected_row: Dict[str, object],
) -> Optional[str]:
    fields = (
        "session_id",
        "side",
        "cam_id",
        "frame_id",
        "timestamp_s",
        "temp",
        "tcm",
        "tcl",
        "pdc",
    )
    for field in fields:
        if db_row[field] != expected_row[field]:
            return f"{field}: db={db_row[field]!r} csv={expected_row[field]!r}"

    db_sum = db_row["sum"]
    expected_sum = expected_row["sum_counts"]
    if db_sum != expected_sum:
        return f"sum: db={db_sum!r} csv={expected_sum!r}"

    if db_row["hist"] != expected_row["hist"]:
        db_len = len(db_row["hist"]) if isinstance(db_row["hist"], (bytes, bytearray)) else -1
        csv_len = len(expected_row["hist"]) if isinstance(expected_row["hist"], (bytes, bytearray)) else -1
        return f"hist blob mismatch: db_len={db_len} csv_len={csv_len}"

    return None


if __name__ == "__main__":
    sys.exit(main())
