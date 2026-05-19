"""
sensor_module_simulator.py - Replay sensor module CSV data at 40 Hz.

The simulator:

1. reads left/right camera histogram CSV files frame-by-frame,
2. displays the active eight histogram streams live,
3. batches raw frame writes to SQLite every N frame groups, and
4. closes the session when playback finishes.

Usage
-----
python sensor_module_simulator.py ow98NSF5 scan_data
python sensor_module_simulator.py ow98NSF5 scan_data --timestamp-key 20260407_152533
python sensor_module_simulator.py ow98NSF5 scan_data --headless --max-frame-groups 20
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Full, Queue
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, TextIO, Tuple

# Make the parent omotion-sdk repo importable when this script is run
# standalone from inside stream-db/ (issue #92 — scan_db.py moved into
# the omotion package).
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from importer import (
    HISTOGRAM_COLUMNS,
    ScanGroup,
    build_session_meta,
    discover_scan_groups,
    pack_histogram,
)
from omotion import ScanDatabase

if TYPE_CHECKING:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
else:
    try:
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
    except ModuleNotFoundError:
        pg = None
        QtCore = None
        QtGui = None
        QtWidgets = None


DEFAULT_PLAYBACK_RATE_HZ = 40.0
DEFAULT_BATCH_FRAME_GROUPS = 600
DEFAULT_PRELOAD_FRAME_GROUPS = 240
DEFAULT_UI_QUEUE_FRAME_GROUPS = 240
DEFAULT_UI_START_BUFFER_FRAMES = 4
MAX_VISIBLE_HISTOGRAMS = 8
PLOT_PENS = [
    "#0d3b66",
    "#2a9d8f",
    "#f4a261",
    "#e76f51",
    "#6d597a",
    "#355070",
    "#bc6c25",
    "#8d99ae",
]


@dataclass(frozen=True)
class CameraStream:
    side: str
    cam_id: int

    @property
    def label(self) -> str:
        return f"{self.side} cam {self.cam_id}"


@dataclass
class CameraFrame:
    side: str
    cam_id: int
    frame_id: int
    timestamp_s: float
    histogram: List[int]
    temperature: Optional[float]
    sum_counts: Optional[int]
    tcm: float
    tcl: float
    pdc: float

    def to_db_row(self, session_id: int) -> Dict[str, object]:
        return {
            "session_id": session_id,
            "side": self.side,
            "cam_id": self.cam_id,
            "frame_id": self.frame_id,
            "timestamp_s": self.timestamp_s,
            "hist": pack_histogram(self.histogram),
            "temp": self.temperature,
            "sum_counts": self.sum_counts,
            "tcm": self.tcm,
            "tcl": self.tcl,
            "pdc": self.pdc,
        }


@dataclass
class PlaybackFrameGroup:
    frame_id: int
    timestamp_s: float
    frames: Dict[CameraStream, CameraFrame]


@dataclass
class PreparedFrameGroup:
    playback_group: PlaybackFrameGroup
    db_rows: List[Dict[str, object]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay left/right sensor CSV files at 40 Hz and write batches to SQLite."
    )
    parser.add_argument("session_label", help="Scan label, for example ow98NSF5.")
    parser.add_argument("scan_dir", help="Directory containing scan CSV files.")
    parser.add_argument(
        "--timestamp-key",
        default=None,
        help="Optional scan timestamp key in YYYYMMDD_HHMMSS format. Defaults to the latest matching scan.",
    )
    parser.add_argument(
        "--db-path",
        default=None,
        help="Optional target SQLite database path. Defaults to data/sqlite.db.",
    )
    parser.add_argument(
        "--playback-rate-hz",
        type=float,
        default=DEFAULT_PLAYBACK_RATE_HZ,
        help=f"Playback rate in Hz. Default: {DEFAULT_PLAYBACK_RATE_HZ}.",
    )
    parser.add_argument(
        "--batch-frames",
        type=int,
        default=DEFAULT_BATCH_FRAME_GROUPS,
        help=f"Number of frame groups to buffer before each DB write. Default: {DEFAULT_BATCH_FRAME_GROUPS}.",
    )
    parser.add_argument(
        "--preload-frame-groups",
        type=int,
        default=DEFAULT_PRELOAD_FRAME_GROUPS,
        help=(
            "Number of decoded frame groups to keep queued ahead of playback. "
            f"Default: {DEFAULT_PRELOAD_FRAME_GROUPS}."
        ),
    )
    parser.add_argument(
        "--ui-queue-frame-groups",
        type=int,
        default=DEFAULT_UI_QUEUE_FRAME_GROUPS,
        help=(
            "Maximum number of frame groups queued for visualization. "
            f"Default: {DEFAULT_UI_QUEUE_FRAME_GROUPS}."
        ),
    )
    parser.add_argument(
        "--ui-start-buffer-frames",
        type=int,
        default=DEFAULT_UI_START_BUFFER_FRAMES,
        help=(
            "How many frame groups to buffer before the UI starts rendering. "
            f"Default: {DEFAULT_UI_START_BUFFER_FRAMES}."
        ),
    )
    parser.add_argument(
        "--no-compress-raw-hist",
        action="store_false",
        dest="compress_raw_hist",
        help="Disable compressed storage for session_raw.hist blobs.",
    )
    parser.set_defaults(compress_raw_hist=True)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run the replay and DB pipeline without opening the live histogram viewer.",
    )
    parser.add_argument(
        "--max-frame-groups",
        type=int,
        default=None,
        help="Optional playback limit for validation runs.",
    )
    return parser.parse_args()


class CsvPlaybackSource:
    def __init__(self, group: ScanGroup) -> None:
        self.group = group
        self.streams = self._discover_streams()
        if len(self.streams) > MAX_VISIBLE_HISTOGRAMS:
            raise ValueError(
                f"Expected at most {MAX_VISIBLE_HISTOGRAMS} active histogram streams, "
                f"found {len(self.streams)}: {[stream.label for stream in self.streams]}"
            )

        self._files: list[TextIO] = []
        self._readers: dict[str, csv.DictReader] = {}
        self._pending: dict[str, Optional[dict[str, str]]] = {"left": None, "right": None}
        for side in ("left", "right"):
            sensor = group.sensor_files.get(side)
            if sensor is None:
                continue
            handle = sensor.path.open("r", encoding="utf-8", newline="")
            self._files.append(handle)
            self._readers[side] = csv.DictReader(handle)

    def _discover_streams(self) -> list[CameraStream]:
        streams: list[CameraStream] = []
        for side in ("left", "right"):
            sensor = self.group.sensor_files.get(side)
            if sensor is None:
                continue
            camera_ids = build_session_meta(self.group)["sensor_modules"][side]["enabled_cameras"]
            streams.extend(CameraStream(side=side, cam_id=int(cam_id)) for cam_id in camera_ids)
        return streams

    def close(self) -> None:
        while self._files:
            self._files.pop().close()

    def __enter__(self) -> "CsvPlaybackSource":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def next_group(self) -> Optional[PlaybackFrameGroup]:
        grouped_frames: dict[CameraStream, CameraFrame] = {}
        frame_id: Optional[int] = None
        timestamp_s: Optional[float] = None
        any_frames = False

        for side in ("left", "right"):
            reader = self._readers.get(side)
            if reader is None:
                continue

            current_group = self._read_next_side_group(side, reader)
            if not current_group:
                continue

            any_frames = True
            group_frame_id = current_group[0].frame_id
            if frame_id is None:
                frame_id = group_frame_id
            elif frame_id != group_frame_id:
                raise ValueError(
                    f"Mismatched frame ids between sides: expected {frame_id}, got {group_frame_id} for {side}"
                )

            if timestamp_s is None:
                timestamp_s = current_group[0].timestamp_s

            for frame in current_group:
                grouped_frames[CameraStream(frame.side, frame.cam_id)] = frame

        if not any_frames or frame_id is None or timestamp_s is None:
            return None

        return PlaybackFrameGroup(
            frame_id=frame_id,
            timestamp_s=timestamp_s,
            frames=grouped_frames,
        )

    def _read_next_side_group(
        self,
        side: str,
        reader: csv.DictReader,
    ) -> list[CameraFrame]:
        record = self._pending[side]
        if record is None:
            try:
                record = next(reader)
            except StopIteration:
                return []

        frame_id = int(record["frame_id"])
        rows = [self._record_to_frame(side, record)]
        self._pending[side] = None

        for next_record in reader:
            next_frame_id = int(next_record["frame_id"])
            if next_frame_id != frame_id:
                self._pending[side] = next_record
                break
            rows.append(self._record_to_frame(side, next_record))

        return rows

    def _record_to_frame(self, side: str, record: dict[str, str]) -> CameraFrame:
        return CameraFrame(
            side=side,
            cam_id=int(record["cam_id"]),
            frame_id=int(record["frame_id"]),
            timestamp_s=float(record["timestamp_s"]),
            histogram=[int(record[column]) for column in HISTOGRAM_COLUMNS],
            temperature=_optional_float(record.get("temperature")),
            sum_counts=_optional_int(record.get("sum")),
            tcm=float(record.get("tcm", 0) or 0),
            tcl=float(record.get("tcl", 0) or 0),
            pdc=float(record.get("pdc", 0) or 0),
        )


class SessionBatchWriter:
    def __init__(
        self,
        *,
        group: ScanGroup,
        db_path: Optional[str],
        compress_raw_hist: bool,
        playback_rate_hz: float,
        batch_frames: int,
        preload_frame_groups: int,
    ) -> None:
        self._group = group
        self._db_path = db_path
        self._compress_raw_hist = compress_raw_hist
        self._playback_rate_hz = playback_rate_hz
        self._batch_frames = batch_frames
        self._preload_frame_groups = preload_frame_groups
        self._queue: Queue[Tuple[str, object]] = Queue()
        self._thread = threading.Thread(target=self._run, name="session-batch-writer", daemon=True)
        self._ready = threading.Event()
        self._finished = threading.Event()
        self._session_id: Optional[int] = None
        self._error: Optional[BaseException] = None
        self._rows_written = 0
        self._batches_written = 0

    @property
    def session_id(self) -> int:
        if self._session_id is None:
            raise RuntimeError("Session has not been created yet.")
        return self._session_id

    @property
    def rows_written(self) -> int:
        return self._rows_written

    @property
    def batches_written(self) -> int:
        return self._batches_written

    def start(self) -> None:
        self._thread.start()
        self._ready.wait()
        self.raise_if_error()

    def submit_rows(self, rows: List[Dict[str, object]]) -> None:
        if not rows:
            return
        self.raise_if_error()
        self._queue.put(("rows", rows))

    def finish(self, session_end: float) -> None:
        self.raise_if_error()
        self._queue.put(("finish", session_end))
        self._finished.wait()
        self.raise_if_error()

    def raise_if_error(self) -> None:
        if self._error is not None:
            raise RuntimeError(str(self._error)) from self._error

    def _run(self) -> None:
        try:
            with ScanDatabase(
                db_path=self._db_path,
                compress_raw_hist=True if self._compress_raw_hist else None,
            ) as db:
                session_meta = build_session_meta(self._group)
                session_meta["simulator"] = {
                    "source": "sensor_module_simulator.py",
                    "playback_rate_hz": self._playback_rate_hz,
                    "batch_frame_groups": self._batch_frames,
                    "preload_frame_groups": self._preload_frame_groups,
                }
                self._session_id = db.create_session(
                    session_label=f"sim_{self._group.label}_{self._group.timestamp_key}",
                    session_start=self._group.scan_datetime.timestamp(),
                    session_notes="Simulated CSV playback at fixed rate.",
                    session_meta=session_meta,
                )
                self._ready.set()

                while True:
                    message_type, payload = self._queue.get()
                    if message_type == "rows":
                        rows = payload
                        assert isinstance(rows, list)
                        db.insert_raw_frames(rows)
                        self._rows_written += len(rows)
                        self._batches_written += 1
                    elif message_type == "finish":
                        session_end = float(payload)
                        db.close_session(self.session_id, session_end)
                        break
        except BaseException as exc:
            self._error = exc
            self._ready.set()
        finally:
            self._finished.set()


class FrameGroupPreloader:
    def __init__(
        self,
        *,
        source: CsvPlaybackSource,
        session_id_provider,
        max_queue_size: int,
    ) -> None:
        if max_queue_size <= 0:
            raise ValueError("max_queue_size must be greater than zero")
        self._source = source
        self._session_id_provider = session_id_provider
        self._queue: Queue[Optional[PreparedFrameGroup]] = Queue(maxsize=max_queue_size)
        self._error: Optional[BaseException] = None
        self._stop_requested = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="frame-group-preloader",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def get_next(self) -> Optional[PreparedFrameGroup]:
        item = self._queue.get()
        if item is None:
            self.raise_if_error()
            return None
        self.raise_if_error()
        return item

    @property
    def buffered_frame_groups(self) -> int:
        return self._queue.qsize()

    def stop(self) -> None:
        self._stop_requested.set()
        self._thread.join(timeout=2.0)

    def raise_if_error(self) -> None:
        if self._error is not None:
            raise RuntimeError(str(self._error)) from self._error

    def _run(self) -> None:
        try:
            while not self._stop_requested.is_set():
                playback_group = self._source.next_group()
                if playback_group is None:
                    break

                prepared_group = PreparedFrameGroup(
                    playback_group=playback_group,
                    db_rows=[
                        frame.to_db_row(self._session_id_provider())
                        for frame in playback_group.frames.values()
                    ],
                )

                while not self._stop_requested.is_set():
                    try:
                        self._queue.put(prepared_group, timeout=0.1)
                        break
                    except Full:
                        continue
        except BaseException as exc:
            self._error = exc
        finally:
            while True:
                try:
                    self._queue.put(None, timeout=0.1)
                    break
                except Full:
                    if self._stop_requested.is_set():
                        try:
                            self._queue.get_nowait()
                        except Empty:
                            pass
                    continue


class PlaybackPipeline:
    def __init__(
        self,
        *,
        group: ScanGroup,
        db_path: Optional[str],
        playback_rate_hz: float,
        batch_frames: int,
        preload_frame_groups: int,
        compress_raw_hist: bool,
        max_frame_groups: Optional[int],
    ) -> None:
        if playback_rate_hz <= 0:
            raise ValueError("playback_rate_hz must be greater than zero")
        if batch_frames <= 0:
            raise ValueError("batch_frames must be greater than zero")
        if preload_frame_groups <= 0:
            raise ValueError("preload_frame_groups must be greater than zero")

        self.group = group
        self.playback_rate_hz = playback_rate_hz
        self.batch_frames = batch_frames
        self.preload_frame_groups = preload_frame_groups
        self.max_frame_groups = max_frame_groups
        self._source = CsvPlaybackSource(group)
        self.streams = list(self._source.streams)
        self.writer = SessionBatchWriter(
            group=group,
            db_path=db_path,
            compress_raw_hist=compress_raw_hist,
            playback_rate_hz=playback_rate_hz,
            batch_frames=batch_frames,
            preload_frame_groups=preload_frame_groups,
        )
        self.preloader = FrameGroupPreloader(
            source=self._source,
            session_id_provider=lambda: self.session_id,
            max_queue_size=preload_frame_groups,
        )
        self.pending_rows: list[Dict[str, object]] = []
        self.pending_frame_groups = 0
        self.processed_frame_groups = 0
        self.last_timestamp_s = group.scan_datetime.timestamp()
        self.last_group: Optional[PlaybackFrameGroup] = None
        self.compress_raw_hist = compress_raw_hist
        self.finished = False

    @property
    def session_id(self) -> int:
        return self.writer.session_id

    def start(self) -> None:
        self.writer.start()
        self.preloader.start()

    def step(self) -> Optional[PlaybackFrameGroup]:
        if self.finished:
            return None

        if self.max_frame_groups is not None and self.processed_frame_groups >= self.max_frame_groups:
            self.complete()
            return None

        prepared_group = self.preloader.get_next()
        if prepared_group is None:
            self.complete()
            return None

        group = prepared_group.playback_group
        self.pending_rows.extend(prepared_group.db_rows)

        self.pending_frame_groups += 1
        self.processed_frame_groups += 1
        self.last_timestamp_s = group.timestamp_s
        self.last_group = group

        if self.pending_frame_groups >= self.batch_frames:
            self.flush_pending_rows()

        return group

    def flush_pending_rows(self) -> None:
        if not self.pending_rows:
            return
        rows_to_flush = self.pending_rows
        self.pending_rows = []
        self.pending_frame_groups = 0
        # Hand off the completed 15-second buffer to the writer thread so
        # playback can immediately continue filling the next buffer at 40 Hz.
        self.writer.submit_rows(rows_to_flush)

    def complete(self) -> None:
        if self.finished:
            return
        self.flush_pending_rows()
        self.writer.finish(self.last_timestamp_s)
        self.preloader.stop()
        self._source.close()
        self.finished = True


class PlaybackRunner:
    def __init__(
        self,
        *,
        pipeline: PlaybackPipeline,
        ui_queue_frame_groups: int,
    ) -> None:
        if ui_queue_frame_groups <= 0:
            raise ValueError("ui_queue_frame_groups must be greater than zero")

        self.pipeline = pipeline
        self.ui_queue: Queue[Optional[PlaybackFrameGroup]] = Queue(
            maxsize=ui_queue_frame_groups
        )
        self._thread = threading.Thread(
            target=self._run,
            name="playback-runner",
            daemon=True,
        )
        self._stop_requested = threading.Event()
        self._started = threading.Event()
        self._finished = threading.Event()
        self.error: Optional[BaseException] = None
        self.visual_frames_dropped = 0
        self.frames_published = 0

    def start(self) -> None:
        self._thread.start()
        self._started.wait()
        self.raise_if_error()

    def stop(self) -> None:
        self._stop_requested.set()
        self._thread.join(timeout=5.0)

    def wait(self, timeout: Optional[float] = None) -> bool:
        return self._finished.wait(timeout)

    @property
    def finished(self) -> bool:
        return self._finished.is_set()

    @property
    def queued_visual_frames(self) -> int:
        return self.ui_queue.qsize()

    def raise_if_error(self) -> None:
        if self.error is not None:
            raise RuntimeError(str(self.error)) from self.error

    def _publish_group(self, group: PlaybackFrameGroup) -> None:
        while not self._stop_requested.is_set():
            try:
                self.ui_queue.put_nowait(group)
                self.frames_published += 1
                return
            except Full:
                try:
                    dropped = self.ui_queue.get_nowait()
                except Empty:
                    continue
                if dropped is not None:
                    self.visual_frames_dropped += 1

    def _finish_queue(self) -> None:
        while True:
            try:
                self.ui_queue.put_nowait(None)
                return
            except Full:
                try:
                    dropped = self.ui_queue.get_nowait()
                except Empty:
                    continue
                if dropped is not None:
                    self.visual_frames_dropped += 1

    def _run(self) -> None:
        next_deadline = time.perf_counter()
        try:
            self.pipeline.start()
            self._started.set()
            frame_period_s = 1.0 / self.pipeline.playback_rate_hz

            while not self._stop_requested.is_set():
                group = self.pipeline.step()
                if group is None:
                    break

                self._publish_group(group)
                next_deadline += frame_period_s
                sleep_s = next_deadline - time.perf_counter()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_deadline = time.perf_counter()

            if not self.pipeline.finished:
                self.pipeline.complete()
        except BaseException as exc:
            self.error = exc
        finally:
            self._started.set()
            self._finish_queue()
            self._finished.set()


if QtWidgets is not None and QtCore is not None and QtGui is not None and pg is not None:
    class SimulatorWindow(QtWidgets.QMainWindow):
        def __init__(
            self,
            *,
            pipeline: PlaybackPipeline,
            runner: PlaybackRunner,
            ui_start_buffer_frames: int,
        ) -> None:
            super().__init__()
            self.pipeline = pipeline
            self.runner = runner
            self.ui_start_buffer_frames = max(1, ui_start_buffer_frames)
            self.plot_items: dict[CameraStream, pg.PlotDataItem] = {}
            self.stats_labels: dict[CameraStream, QtWidgets.QLabel] = {}
            self._x_axis = list(range(len(HISTOGRAM_COLUMNS)))
            self._pending_visual_frames: deque[PlaybackFrameGroup] = deque()
            self._render_frames: deque[PlaybackFrameGroup] = deque()
            self._render_started = False
            self._playback_finished = False
            self._render_interval_s = 1.0 / self.pipeline.playback_rate_hz
            self._next_render_at = time.perf_counter()

            self.setWindowTitle("Sensor Module Simulator")
            self.resize(1840, 980)
            self.setMinimumSize(1480, 840)
            self._configure_theme()
            self._build_ui()

            self.render_timer = QtCore.QTimer(self)
            self.render_timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
            self.render_timer.setInterval(max(1, round(self._render_interval_s * 1000)))
            self.render_timer.timeout.connect(self.on_render_tick)
            self.render_timer.start()
            self._update_status("Playback started.")

        def _configure_theme(self) -> None:
            app = QtWidgets.QApplication.instance()
            if app is None:
                return
            app.setStyle("Fusion")
            palette = QtGui.QPalette()
            palette.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor("#f6f1ea"))
            palette.setColor(QtGui.QPalette.ColorRole.WindowText, QtGui.QColor("#2b2118"))
            palette.setColor(QtGui.QPalette.ColorRole.Base, QtGui.QColor("#fffdfa"))
            palette.setColor(QtGui.QPalette.ColorRole.AlternateBase, QtGui.QColor("#efe4d6"))
            palette.setColor(QtGui.QPalette.ColorRole.Text, QtGui.QColor("#2b2118"))
            palette.setColor(QtGui.QPalette.ColorRole.Button, QtGui.QColor("#dfd1c0"))
            palette.setColor(QtGui.QPalette.ColorRole.ButtonText, QtGui.QColor("#2b2118"))
            palette.setColor(QtGui.QPalette.ColorRole.Highlight, QtGui.QColor("#b8d4c4"))
            palette.setColor(QtGui.QPalette.ColorRole.HighlightedText, QtGui.QColor("#18231d"))
            app.setPalette(palette)
            app.setStyleSheet(
                """
                QWidget {
                    font-family: "Segoe UI";
                    font-size: 10pt;
                }
                QMainWindow, QWidget#central {
                    background: #f6f1ea;
                    color: #2b2118;
                }
                QFrame[panel="true"] {
                    background: #fffaf4;
                    border: 1px solid #d7c8b7;
                    border-radius: 14px;
                }
                QLabel#title {
                    font-size: 21pt;
                    font-weight: 700;
                    color: #20170f;
                }
                QLabel#subtitle, QLabel#status {
                    color: #645648;
                }
                QLabel[plotstat="true"] {
                    color: #5d5042;
                }
                """
            )

        def _build_ui(self) -> None:
            central = QtWidgets.QWidget()
            central.setObjectName("central")
            self.setCentralWidget(central)

            root = QtWidgets.QVBoxLayout(central)
            root.setContentsMargins(16, 14, 16, 14)
            root.setSpacing(10)

            title = QtWidgets.QLabel("Sensor Module CSV Simulator")
            title.setObjectName("title")
            subtitle = QtWidgets.QLabel(
                f"Streaming {self.pipeline.group.label} / {self.pipeline.group.timestamp_key} "
                f"at {self.pipeline.playback_rate_hz:.1f} Hz with DB flushes every "
                f"{self.pipeline.batch_frames} frame groups and compression "
                f"{'on' if self.pipeline.compress_raw_hist else 'off'}."
            )
            subtitle.setObjectName("subtitle")
            root.addWidget(title)
            root.addWidget(subtitle)

            header = QtWidgets.QFrame()
            header.setProperty("panel", True)
            header_layout = QtWidgets.QGridLayout(header)
            header_layout.setContentsMargins(14, 12, 14, 12)
            header_layout.setHorizontalSpacing(18)
            header_layout.setVerticalSpacing(6)

            self.session_label = QtWidgets.QLabel(f"Session ID: {self.pipeline.session_id}")
            self.frame_label = QtWidgets.QLabel("Frame: waiting")
            self.db_label = QtWidgets.QLabel("Rows written: 0")
            self.batch_label = QtWidgets.QLabel("Batches written: 0")
            self.queue_label = QtWidgets.QLabel("UI queue: 0")
            header_layout.addWidget(self.session_label, 0, 0)
            header_layout.addWidget(self.frame_label, 0, 1)
            header_layout.addWidget(self.db_label, 1, 0)
            header_layout.addWidget(self.batch_label, 1, 1)
            header_layout.addWidget(self.queue_label, 2, 0, 1, 2)
            root.addWidget(header)

            grid = QtWidgets.QGridLayout()
            grid.setHorizontalSpacing(10)
            grid.setVerticalSpacing(10)
            root.addLayout(grid, 1)

            for index in range(MAX_VISIBLE_HISTOGRAMS):
                panel = QtWidgets.QFrame()
                panel.setProperty("panel", True)
                panel_layout = QtWidgets.QVBoxLayout(panel)
                panel_layout.setContentsMargins(10, 10, 10, 10)
                panel_layout.setSpacing(6)

                if index < len(self.pipeline.streams):
                    stream = self.pipeline.streams[index]
                    title_label = QtWidgets.QLabel(stream.label.upper())
                    plot = pg.PlotWidget()
                    plot.setBackground("#fffdfa")
                    plot.showGrid(x=True, y=True, alpha=0.15)
                    plot.setLabel("bottom", "Bin")
                    plot.setLabel("left", "Count")
                    plot.getAxis("bottom").setTextPen(pg.mkColor("#574b40"))
                    plot.getAxis("left").setTextPen(pg.mkColor("#574b40"))
                    plot.getAxis("bottom").setPen(pg.mkPen("#7c6b5c", width=1))
                    plot.getAxis("left").setPen(pg.mkPen("#7c6b5c", width=1))
                    plot.getViewBox().setMouseEnabled(x=False, y=False)
                    plot.getViewBox().setMenuEnabled(False)
                    curve = plot.plot(pen=pg.mkPen(PLOT_PENS[index], width=2))
                    stats = QtWidgets.QLabel("Waiting for data")
                    stats.setProperty("plotstat", True)
                    stats.setWordWrap(True)
                    panel_layout.addWidget(title_label)
                    panel_layout.addWidget(plot, 1)
                    panel_layout.addWidget(stats)
                    self.plot_items[stream] = curve
                    self.stats_labels[stream] = stats
                else:
                    empty_label = QtWidgets.QLabel("Unused slot")
                    empty_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                    empty_label.setMinimumHeight(280)
                    panel_layout.addWidget(empty_label, 1)

                row = index // 4
                col = index % 4
                grid.addWidget(panel, row, col)

            self.status_label = QtWidgets.QLabel("Running")
            self.status_label.setObjectName("status")
            root.addWidget(self.status_label)

        def on_render_tick(self) -> None:
            try:
                self._drain_visual_queue()
                self._maybe_start_rendering()
                frames_rendered = self._render_due_frames()

                self._refresh_writer_stats()
                if self._playback_finished and not self._render_frames and not self._pending_visual_frames:
                    self.render_timer.stop()
                    self.runner.raise_if_error()
                    self._update_status(
                        f"Playback complete. Frame groups: {self.pipeline.processed_frame_groups} | "
                        f"Rows written: {self.pipeline.writer.rows_written}"
                    )
                    return

                startup_state = (
                    "buffering"
                    if not self._render_started
                    else "rendering"
                )
                self._update_status(
                    f"{startup_state} | "
                    f"UI rendered this tick: {frames_rendered} | "
                    f"Write buffer: {self.pipeline.pending_frame_groups} | "
                    f"Read-ahead: {self.pipeline.preloader.buffered_frame_groups} | "
                    f"UI queue: {self.runner.queued_visual_frames}"
                )
            except Exception as exc:
                self.render_timer.stop()
                self._update_status(f"Playback failed: {exc}")
                QtWidgets.QMessageBox.critical(
                    self,
                    "Sensor Module Simulator",
                    str(exc),
                )

        def _drain_visual_queue(self) -> None:
            while True:
                try:
                    item = self.runner.ui_queue.get_nowait()
                except Empty:
                    return

                if item is None:
                    self._playback_finished = True
                    return

                self._pending_visual_frames.append(item)

        def _maybe_start_rendering(self) -> None:
            if self._render_started:
                return
            if len(self._pending_visual_frames) >= self.ui_start_buffer_frames:
                self._render_started = True
                self._swap_render_buffers()
                return
            if self._playback_finished and self._pending_visual_frames:
                self._render_started = True
                self._swap_render_buffers()

        def _swap_render_buffers(self) -> None:
            if not self._pending_visual_frames:
                return
            self._render_frames, self._pending_visual_frames = (
                self._pending_visual_frames,
                self._render_frames,
            )

        def _render_due_frames(self) -> int:
            if not self._render_started:
                return 0

            now = time.perf_counter()
            if now + 0.001 < self._next_render_at and self._render_frames:
                return 0

            if not self._render_frames and self._pending_visual_frames:
                self._swap_render_buffers()
            if not self._render_frames:
                self._next_render_at = now + self._render_interval_s
                return 0

            overdue_frames = 1
            if now > self._next_render_at:
                overdue_frames += int((now - self._next_render_at) / self._render_interval_s)

            max_burst = max(1, round(self.pipeline.playback_rate_hz))
            due_frames = min(max_burst, overdue_frames)
            frames_rendered = 0

            while frames_rendered < due_frames:
                if not self._render_frames:
                    if self._pending_visual_frames:
                        self._swap_render_buffers()
                    else:
                        break
                if not self._render_frames:
                    break
                self._render_group(self._render_frames.popleft())
                frames_rendered += 1

            if frames_rendered > 0:
                self._next_render_at += frames_rendered * self._render_interval_s
                if now - self._next_render_at > self._render_interval_s:
                    self._next_render_at = now + self._render_interval_s

            return frames_rendered

        def _render_group(self, group: PlaybackFrameGroup) -> None:
            self.frame_label.setText(
                f"Frame: {group.frame_id} | timestamp_s: {group.timestamp_s:.6f}"
            )
            for stream, curve in self.plot_items.items():
                frame = group.frames.get(stream)
                if frame is None:
                    curve.setData([], [])
                    self.stats_labels[stream].setText("No data in this frame group")
                    continue
                curve.setData(self._x_axis, frame.histogram)
                self.stats_labels[stream].setText(_format_histogram_stats(frame.histogram))

        def _refresh_writer_stats(self) -> None:
            self.db_label.setText(f"Rows written: {self.pipeline.writer.rows_written}")
            self.batch_label.setText(f"Batches written: {self.pipeline.writer.batches_written}")
            self.queue_label.setText(
                f"UI queue: {self.runner.queued_visual_frames} | "
                f"Visual drops: {self.runner.visual_frames_dropped}"
            )

        def _update_status(self, message: str) -> None:
            self.status_label.setText(message)

        def closeEvent(self, event: QtGui.QCloseEvent) -> None:
            try:
                self.render_timer.stop()
                self.runner.stop()
            except Exception:
                pass
            super().closeEvent(event)
else:
    SimulatorWindow = None


def select_group(
    *,
    session_label: str,
    scan_dir: Path,
    timestamp_key: Optional[str],
) -> ScanGroup:
    groups = discover_scan_groups(session_label, scan_dir)
    if not groups:
        raise FileNotFoundError(
            f"No scan files found for label '{session_label}' in {scan_dir}"
        )
    if timestamp_key is None:
        return groups[-1]

    for group in groups:
        if group.timestamp_key == timestamp_key:
            return group

    available = ", ".join(group.timestamp_key for group in groups)
    raise FileNotFoundError(
        f"No scan group matched timestamp key '{timestamp_key}'. Available values: {available}"
    )


def run_headless(pipeline: PlaybackPipeline) -> None:
    while True:
        group = pipeline.step()
        if group is None:
            break
    print(
        f"Completed headless playback for session {pipeline.session_id}: "
        f"{pipeline.processed_frame_groups} frame groups, "
        f"{pipeline.writer.rows_written} rows, "
        f"{pipeline.writer.batches_written} batches."
    )


def main() -> None:
    args = parse_args()
    scan_dir = Path(args.scan_dir)
    if not scan_dir.exists():
        raise FileNotFoundError(f"Scan directory does not exist: {scan_dir}")

    group = select_group(
        session_label=args.session_label,
        scan_dir=scan_dir,
        timestamp_key=args.timestamp_key,
    )
    pipeline = PlaybackPipeline(
        group=group,
        db_path=args.db_path,
        playback_rate_hz=args.playback_rate_hz,
        batch_frames=args.batch_frames,
        preload_frame_groups=args.preload_frame_groups,
        compress_raw_hist=args.compress_raw_hist,
        max_frame_groups=args.max_frame_groups,
    )

    if args.headless:
        pipeline.start()
        run_headless(pipeline)
        return

    if SimulatorWindow is None or QtWidgets is None or pg is None:
        raise ModuleNotFoundError(
            "GUI mode requires PyQt5 and pyqtgraph to be installed in the active interpreter."
        )

    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    runner = PlaybackRunner(
        pipeline=pipeline,
        ui_queue_frame_groups=args.ui_queue_frame_groups,
    )
    runner.start()
    window = SimulatorWindow(
        pipeline=pipeline,
        runner=runner,
        ui_start_buffer_frames=args.ui_start_buffer_frames,
    )
    window.show()
    sys.exit(app.exec())


def _optional_float(value: Optional[str]) -> Optional[float]:
    if value in (None, ""):
        return None
    return float(value)


def _optional_int(value: Optional[str]) -> Optional[int]:
    if value in (None, ""):
        return None
    return int(float(value))


def _format_histogram_stats(histogram: Sequence[int]) -> str:
    total = sum(histogram)
    maximum = max(histogram) if histogram else 0
    non_zero = sum(1 for value in histogram if value)
    return f"Total {total:,} | Max {maximum:,} | Non-zero bins {non_zero}"


if __name__ == "__main__":
    main()
