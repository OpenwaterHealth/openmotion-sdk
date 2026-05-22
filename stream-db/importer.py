"""
importer.py - Import scan_data files into the SQLite scan database.

Usage
-----
python importer.py ow98NSF5 scan_data
python importer.py ow98NSF5 scan_data --compress-raw-hist

This importer groups files by:
    scan_<label>_<YYYYMMDD>_<HHMMSS>

Each label + timestamp combination is treated as one scan session. For each
session, the importer:

1. Reads the notes file into ``sessions.session_notes``.
2. Builds ``session_meta`` describing the available sensor modules and masks.
3. Reads the next frame group from each available side file.
4. Writes those frame groups into ``session_raw`` in left/right scan order.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import os
import struct
import sys
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional

# Make the parent omotion-sdk repo importable when this script is run
# standalone from inside stream-db/ (issue #92 — scan_db.py moved into
# the omotion package).
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from omotion import ScanDatabase


FILENAME_RE = (
    r"^scan_"
    r"(?P<label>[^_]+)_"
    r"(?P<date>\d{8})_"
    r"(?P<time>\d{6})"
    r"(?:_(?P<side>left|right)_mask(?P<mask>[0-9A-Fa-f]+))?"
    r"\.(?P<ext>csv|txt)$"
)

HISTOGRAM_WIDTH = 1024
HISTOGRAM_COLUMNS = [str(index) for index in range(HISTOGRAM_WIDTH)]


@dataclass
class SensorFile:
    side: str
    mask_hex: str
    path: Path


@dataclass
class ScanGroup:
    label: str
    timestamp_key: str
    scan_datetime: dt.datetime
    notes_path: Optional[Path]
    sensor_files: Dict[str, SensorFile]


def main() -> None:
    args = parse_args()
    scan_dir = Path(args.scan_dir)

    if not scan_dir.exists():
        raise FileNotFoundError(f"Scan directory does not exist: {scan_dir}")

    groups = discover_scan_groups(args.session_label, scan_dir)
    if not groups:
        raise FileNotFoundError(
            f"No scan files found for label '{args.session_label}' in {scan_dir}"
        )

    with ScanDatabase(
        db_path=args.db_path,
        compress_raw_hist=True if args.compress_raw_hist else None,
    ) as db:
        for group in groups:
            session_id = import_scan_group(db, group)
            print(
                f"Imported session {session_id}: "
                f"{group.label}_{group.timestamp_key} "
                f"with {len(group.sensor_files)} sensor file(s)"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Import scan_<label>_* files into the SQLite database."
    )
    parser.add_argument(
        "session_label",
        help="Session label to import, for example ow98NSF5.",
    )
    parser.add_argument(
        "scan_dir",
        help="Directory containing scan data files.",
    )
    parser.add_argument(
        "--db-path",
        default=None,
        help="Optional target SQLite database path. Defaults to data/sqlite.db.",
    )
    parser.add_argument(
        "--compress-raw-hist",
        action="store_true",
        help="When creating a new database, compress session_raw.hist blobs.",
    )
    return parser.parse_args()


def discover_scan_groups(session_label: str, scan_dir: Path) -> List[ScanGroup]:
    import re

    grouped: Dict[str, Dict[str, object]] = {}
    for path in sorted(scan_dir.glob(f"scan_{session_label}_*")):
        if not path.is_file():
            continue

        match = re.match(FILENAME_RE, path.name)
        if not match:
            continue

        label = match.group("label")
        if label != session_label:
            continue

        timestamp_key = f"{match.group('date')}_{match.group('time')}"
        scan_datetime = dt.datetime.strptime(timestamp_key, "%Y%m%d_%H%M%S")
        group = grouped.setdefault(
            timestamp_key,
            {
                "label": label,
                "timestamp_key": timestamp_key,
                "scan_datetime": scan_datetime,
                "notes_path": None,
                "sensor_files": {},
            },
        )

        side = match.group("side")
        ext = match.group("ext")
        if ext == "txt":
            group["notes_path"] = path
            continue

        if side is None:
            continue

        sensor_files = group["sensor_files"]
        assert isinstance(sensor_files, dict)
        sensor_files[side] = SensorFile(
            side=side,
            mask_hex=match.group("mask"),
            path=path,
        )

    scans = []
    for data in grouped.values():
        sensor_files = data["sensor_files"]
        assert isinstance(sensor_files, dict)
        if not sensor_files:
            continue
        scans.append(
            ScanGroup(
                label=data["label"],  # type: ignore[arg-type]
                timestamp_key=data["timestamp_key"],  # type: ignore[arg-type]
                scan_datetime=data["scan_datetime"],  # type: ignore[arg-type]
                notes_path=data["notes_path"],  # type: ignore[arg-type]
                sensor_files=sensor_files,  # type: ignore[arg-type]
            )
        )

    scans.sort(key=lambda group: group.scan_datetime)
    return scans


def import_scan_group(db: ScanDatabase, group: ScanGroup) -> int:
    session_label = f"{group.label}_{group.timestamp_key}"
    notes = read_notes(group.notes_path)
    session_meta = build_session_meta(group)

    session_id = db.create_session(
        session_label=session_label,
        session_start=group.scan_datetime.timestamp(),
        session_notes=notes,
        session_meta=session_meta,
    )

    import_interleaved_raw_frames(db, session_id, group)

    return session_id


def read_notes(notes_path: Optional[Path]) -> str:
    if notes_path is None:
        return ""
    return notes_path.read_text(encoding="utf-8").strip()


def build_session_meta(group: ScanGroup) -> Dict[str, object]:
    sensors: Dict[str, object] = {}
    for side, sensor in sorted(group.sensor_files.items()):
        sensors[side] = {
            "mask_hex": sensor.mask_hex.lower(),
            "mask_int": int(sensor.mask_hex, 16),
            "enabled_cameras": decode_camera_mask(sensor.mask_hex),
            "file_name": sensor.path.name,
        }

    return {
        "source_label": group.label,
        "scan_timestamp": group.timestamp_key,
        "scan_datetime_iso": group.scan_datetime.isoformat(),
        "sensor_modules": sensors,
    }


def decode_camera_mask(mask_hex: str) -> List[int]:
    mask_value = int(mask_hex, 16)
    return [camera_id for camera_id in range(8) if mask_value & (1 << camera_id)]


def import_interleaved_raw_frames(
    db: ScanDatabase,
    session_id: int,
    group: ScanGroup,
) -> None:
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
            batch_rows: List[Dict[str, object]] = []
            exhausted_sides: List[str] = []
            for side in ("left", "right"):
                frame_iter = frame_group_iters.get(side)
                if frame_iter is None:
                    continue
                try:
                    batch_rows.extend(next(frame_iter))
                except StopIteration:
                    exhausted_sides.append(side)

            for side in exhausted_sides:
                frame_group_iters.pop(side, None)

            if batch_rows:
                db.insert_raw_frames(batch_rows)


def iter_frame_groups(
    session_id: int,
    side: str,
    reader: csv.DictReader,
) -> Iterator[List[Dict[str, object]]]:
    pending_record: Optional[dict[str, str]] = None

    while True:
        current_record = pending_record
        if current_record is None:
            try:
                current_record = next(reader)
            except StopIteration:
                return

        current_frame_id = int(current_record["frame_id"])
        frame_rows = [build_raw_row(session_id, side, current_record)]
        pending_record = None

        for next_record in reader:
            next_frame_id = int(next_record["frame_id"])
            if next_frame_id != current_frame_id:
                pending_record = next_record
                break
            frame_rows.append(build_raw_row(session_id, side, next_record))

        yield frame_rows


def build_raw_row(
    session_id: int,
    side: str,
    record: dict[str, str],
) -> Dict[str, object]:
    histogram = [int(record[column]) for column in HISTOGRAM_COLUMNS]
    return {
        "session_id": session_id,
        "side": side,
        "cam_id": int(record["cam_id"]),
        "frame_id": int(record["frame_id"]),
        "timestamp_s": float(record["timestamp_s"]),
        "hist": pack_histogram(histogram),
        "temp": float(record["temperature"]),
        "sum_counts": int(record["sum"]),
        "tcm": float(record.get("tcm", 0) or 0),
        "tcl": float(record.get("tcl", 0) or 0),
        "pdc": float(record.get("pdc", 0) or 0),
    }


def pack_histogram(histogram: Iterable[int]) -> bytes:
    bins = list(histogram)
    if len(bins) != HISTOGRAM_WIDTH:
        raise ValueError(
            f"Expected {HISTOGRAM_WIDTH} histogram bins, got {len(bins)}"
        )
    return struct.pack(f"<{HISTOGRAM_WIDTH}I", *bins)


if __name__ == "__main__":
    main()
