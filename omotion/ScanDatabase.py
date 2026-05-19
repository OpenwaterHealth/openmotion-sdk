"""
scan_db.py - SQLite database manager for openwater scan sessions.

The database stores session metadata, raw per-frame acquisition rows, and
derived metrics computed from pipeline windows.
"""

from __future__ import annotations

import json
import sqlite3
import zlib
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence


_DEFAULT_DB_NAME = "sqlite.db"
_DATA_DIR = Path(__file__).parent / "data"


class ScanDatabase:
    """
    Create or open a SQLite database for scan session storage.

    Parameters
    ----------
    db_path : str or Path, optional
        Path to the database file.
        - Absolute path  -> used as-is.
        - Bare filename  -> resolved inside the default ``data/`` folder.
        - None (default) -> ``data/sqlite.db``.
    force_create : bool, default False
        When True, raise ``FileExistsError`` if the target file already exists.
    """

    def __init__(
        self,
        db_path: Optional[str | Path] = None,
        force_create: bool = False,
        compress_raw_hist: Optional[bool] = None,
    ) -> None:
        resolved = self._resolve_path(db_path)
        db_already_exists = resolved.exists()

        if force_create and db_already_exists:
            raise FileExistsError(
                f"Database already exists at '{resolved}'. "
                "Remove or rename the file, or choose a different name."
            )

        resolved.parent.mkdir(parents=True, exist_ok=True)

        self._db_path: Path = resolved
        self._conn: Optional[sqlite3.Connection] = self._open_connection()
        self._init_schema()
        self._compress_raw_hist = self._init_compression_setting(
            db_already_exists=db_already_exists,
            compress_raw_hist=compress_raw_hist,
        )

    @staticmethod
    def _resolve_path(db_path: Optional[str | Path]) -> Path:
        if db_path is None:
            return _DATA_DIR / _DEFAULT_DB_NAME

        path = Path(db_path)
        if not path.is_absolute() and path.parent == Path("."):
            return _DATA_DIR / path
        return path

    def _open_connection(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self._db_path, check_same_thread=False)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        conn.execute("PRAGMA foreign_keys=ON")
        conn.execute("PRAGMA temp_store=MEMORY")
        conn.execute("PRAGMA busy_timeout=5000")
        return conn

    def _init_schema(self) -> None:
        self._connection().executescript(
            """
            CREATE TABLE IF NOT EXISTS sessions (
                id             INTEGER PRIMARY KEY,
                session_label  TEXT    NOT NULL,
                session_start  REAL    NOT NULL,
                session_end    REAL,
                session_notes  TEXT,
                session_meta   TEXT
            );

            CREATE TABLE IF NOT EXISTS session_raw (
                id           INTEGER PRIMARY KEY,
                session_id   INTEGER NOT NULL REFERENCES sessions(id)
                                       ON DELETE CASCADE,
                side         TEXT    NOT NULL CHECK(side IN ('left', 'right')),
                cam_id       INTEGER NOT NULL,
                frame_id     INTEGER NOT NULL,
                timestamp_s  REAL    NOT NULL,
                hist         BLOB    NOT NULL,
                temp         REAL,
                sum          INTEGER,
                tcm          REAL    NOT NULL DEFAULT 0,
                tcl          REAL    NOT NULL DEFAULT 0,
                pdc          REAL    NOT NULL DEFAULT 0
            );

            CREATE INDEX IF NOT EXISTS idx_session_raw_session_time
                ON session_raw(session_id, timestamp_s);

            CREATE INDEX IF NOT EXISTS idx_session_raw_session_cam_time
                ON session_raw(session_id, side, cam_id, timestamp_s);

            CREATE TABLE IF NOT EXISTS session_data (
                id               INTEGER PRIMARY KEY,
                session_id       INTEGER NOT NULL REFERENCES sessions(id)
                                          ON DELETE CASCADE,
                session_raw_id   INTEGER REFERENCES session_raw(id)
                                          ON DELETE SET NULL,
                cam_id           INTEGER NOT NULL,
                side             INTEGER NOT NULL CHECK(side IN (0, 1)),
                timestamp_s      REAL    NOT NULL,
                bfi              REAL,
                bvi              REAL,
                contrast         REAL,
                mean             REAL
            );

            CREATE INDEX IF NOT EXISTS idx_session_data_session_time
                ON session_data(session_id, timestamp_s);

            CREATE INDEX IF NOT EXISTS idx_session_data_session_cam
                ON session_data(session_id, side, cam_id, timestamp_s);

            CREATE TABLE IF NOT EXISTS database_settings (
                key            TEXT PRIMARY KEY,
                value          TEXT NOT NULL
            );
            """
        )
        self._connection().commit()

    def _init_compression_setting(
        self,
        *,
        db_already_exists: bool,
        compress_raw_hist: Optional[bool],
    ) -> bool:
        configured = self._get_setting("compress_raw_hist")
        if configured is None:
            enabled = bool(compress_raw_hist) if compress_raw_hist is not None else False
            self._set_setting("compress_raw_hist", "1" if enabled else "0")
            return enabled

        enabled = configured == "1"
        if compress_raw_hist is not None and enabled != bool(compress_raw_hist):
            raise ValueError(
                "Database compression setting does not match requested "
                f"compress_raw_hist={compress_raw_hist!r}"
            )
        return enabled

    def _get_setting(self, key: str) -> Optional[str]:
        row = self._connection().execute(
            "SELECT value FROM database_settings WHERE key = ?",
            (key,),
        ).fetchone()
        if row is None:
            return None
        return str(row["value"])

    def _set_setting(self, key: str, value: str) -> None:
        self._connection().execute(
            """
            INSERT INTO database_settings (key, value)
            VALUES (?, ?)
            ON CONFLICT(key) DO UPDATE SET value = excluded.value
            """,
            (key, value),
        )
        self._connection().commit()

    def _connection(self) -> sqlite3.Connection:
        if self._conn is None:
            raise RuntimeError("Database connection is closed")
        return self._conn

    # ------------------------------------------------------------------
    # Sessions
    # ------------------------------------------------------------------

    def create_session(
        self,
        session_label: str,
        session_start: float,
        session_end: Optional[float] = None,
        session_notes: Optional[str] = None,
        session_meta: Optional[Dict[str, Any] | str] = None,
    ) -> int:
        meta_json = _to_json_text(session_meta)
        cursor = self._connection().execute(
            """
            INSERT INTO sessions (
                session_label, session_start, session_end, session_notes, session_meta
            )
            VALUES (?, ?, ?, ?, ?)
            """,
            (session_label, session_start, session_end, session_notes, meta_json),
        )
        self._connection().commit()
        return int(cursor.lastrowid)

    def update_session(
        self,
        session_id: int,
        *,
        session_label: Optional[str] = None,
        session_start: Optional[float] = None,
        session_end: Optional[float] = None,
        session_notes: Optional[str] = None,
        session_meta: Optional[Dict[str, Any] | str] = None,
    ) -> bool:
        updates: list[str] = []
        bindings: list[Any] = []

        if session_label is not None:
            updates.append("session_label = ?")
            bindings.append(session_label)
        if session_start is not None:
            updates.append("session_start = ?")
            bindings.append(session_start)
        if session_end is not None:
            updates.append("session_end = ?")
            bindings.append(session_end)
        if session_notes is not None:
            updates.append("session_notes = ?")
            bindings.append(session_notes)
        if session_meta is not None:
            updates.append("session_meta = ?")
            bindings.append(_to_json_text(session_meta))

        if not updates:
            return False

        bindings.append(session_id)
        cursor = self._connection().execute(
            f"UPDATE sessions SET {', '.join(updates)} WHERE id = ?",
            bindings,
        )
        self._connection().commit()
        return cursor.rowcount > 0

    def close_session(self, session_id: int, session_end: float) -> bool:
        cursor = self._connection().execute(
            "UPDATE sessions SET session_end = ? WHERE id = ?",
            (session_end, session_id),
        )
        self._connection().commit()
        return cursor.rowcount > 0

    def get_session(self, session_id: int) -> Optional[Dict[str, Any]]:
        row = self._connection().execute(
            "SELECT * FROM sessions WHERE id = ?",
            (session_id,),
        ).fetchone()
        return _session_row_to_dict(row) if row else None

    def get_session_by_label(self, session_label: str) -> Optional[Dict[str, Any]]:
        row = self._connection().execute(
            """
            SELECT * FROM sessions
            WHERE session_label = ?
            ORDER BY session_start DESC, id DESC
            LIMIT 1
            """,
            (session_label,),
        ).fetchone()
        return _session_row_to_dict(row) if row else None

    def delete_session(self, session_id: int) -> bool:
        cursor = self._connection().execute(
            "DELETE FROM sessions WHERE id = ?",
            (session_id,),
        )
        self._connection().commit()
        return cursor.rowcount > 0

    def stream_sessions(
        self,
        batch_size: int = 100,
    ) -> Iterator[List[Dict[str, Any]]]:
        cursor = self._connection().execute(
            "SELECT * FROM sessions ORDER BY session_start, id"
        )
        while True:
            rows = cursor.fetchmany(batch_size)
            if not rows:
                break
            yield [_session_row_to_dict(row) for row in rows]

    def iter_sessions(self) -> Iterator[Dict[str, Any]]:
        cursor = self._connection().execute(
            "SELECT * FROM sessions ORDER BY session_start, id"
        )
        for row in cursor:
            yield _session_row_to_dict(row)

    # ------------------------------------------------------------------
    # Raw frames
    # ------------------------------------------------------------------

    def insert_raw_frame(
        self,
        session_id: int,
        side: str,
        cam_id: int,
        frame_id: int,
        timestamp_s: float,
        hist: bytes | bytearray | memoryview,
        *,
        temp: Optional[float] = None,
        sum_counts: Optional[int] = None,
        tcm: float = 0,
        tcl: float = 0,
        pdc: float = 0,
    ) -> int:
        _validate_side_text(side)
        encoded_hist = self._encode_hist_blob(hist)
        cursor = self._connection().execute(
            """
            INSERT INTO session_raw (
                session_id, side, cam_id, frame_id, timestamp_s,
                hist, temp, sum, tcm, tcl, pdc
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                session_id,
                side,
                cam_id,
                frame_id,
                timestamp_s,
                sqlite3.Binary(encoded_hist),
                temp,
                sum_counts,
                tcm,
                tcl,
                pdc,
            ),
        )
        self._connection().commit()
        return int(cursor.lastrowid)

    def insert_raw_frames(self, rows: Sequence[Dict[str, Any]]) -> int:
        params = []
        for row in rows:
            side = row["side"]
            _validate_side_text(side)
            encoded_hist = self._encode_hist_blob(row["hist"])
            params.append(
                (
                    row["session_id"],
                    side,
                    row["cam_id"],
                    row["frame_id"],
                    row["timestamp_s"],
                    sqlite3.Binary(encoded_hist),
                    row.get("temp"),
                    row.get("sum_counts"),
                    row.get("tcm", 0),
                    row.get("tcl", 0),
                    row.get("pdc", 0),
                )
            )

        self._connection().executemany(
            """
            INSERT INTO session_raw (
                session_id, side, cam_id, frame_id, timestamp_s,
                hist, temp, sum, tcm, tcl, pdc
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            params,
        )
        self._connection().commit()
        return len(params)

    def get_raw_frame(self, raw_frame_id: int) -> Optional[Dict[str, Any]]:
        row = self._connection().execute(
            "SELECT * FROM session_raw WHERE id = ?",
            (raw_frame_id,),
        ).fetchone()
        return _raw_frame_row_to_dict(row, self._compress_raw_hist) if row else None

    def stream_raw_frames(
        self,
        session_id: int,
        side: Optional[str] = None,
        cam_id: Optional[int] = None,
        batch_size: int = 256,
    ) -> Iterator[List[Dict[str, Any]]]:
        sql, bindings = _build_raw_query(session_id, side, cam_id)
        cursor = self._connection().execute(sql, bindings)
        while True:
            rows = cursor.fetchmany(batch_size)
            if not rows:
                break
            yield [_raw_frame_row_to_dict(row, self._compress_raw_hist) for row in rows]

    def iter_raw_frames(
        self,
        session_id: int,
        side: Optional[str] = None,
        cam_id: Optional[int] = None,
    ) -> Iterator[Dict[str, Any]]:
        sql, bindings = _build_raw_query(session_id, side, cam_id)
        cursor = self._connection().execute(sql, bindings)
        for row in cursor:
            yield _raw_frame_row_to_dict(row, self._compress_raw_hist)

    def delete_raw_frames(self, session_id: int) -> int:
        cursor = self._connection().execute(
            "DELETE FROM session_raw WHERE session_id = ?",
            (session_id,),
        )
        self._connection().commit()
        return cursor.rowcount

    # ------------------------------------------------------------------
    # Session data
    # ------------------------------------------------------------------

    def insert_session_data(
        self,
        session_id: int,
        cam_id: int,
        side: int,
        timestamp_s: float,
        *,
        session_raw_id: Optional[int] = None,
        bfi: Optional[float] = None,
        bvi: Optional[float] = None,
        contrast: Optional[float] = None,
        mean: Optional[float] = None,
    ) -> int:
        _validate_side(side)
        cursor = self._connection().execute(
            """
            INSERT INTO session_data (
                session_id, session_raw_id, cam_id, side,
                timestamp_s, bfi, bvi, contrast, mean
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                session_id,
                session_raw_id,
                cam_id,
                side,
                timestamp_s,
                bfi,
                bvi,
                contrast,
                mean,
            ),
        )
        self._connection().commit()
        return int(cursor.lastrowid)

    def insert_session_data_rows(self, rows: Sequence[Dict[str, Any]]) -> int:
        params = []
        for row in rows:
            _validate_side(row["side"])
            params.append(
                (
                    row["session_id"],
                    row.get("session_raw_id"),
                    row["cam_id"],
                    row["side"],
                    row["timestamp_s"],
                    row.get("bfi"),
                    row.get("bvi"),
                    row.get("contrast"),
                    row.get("mean"),
                )
            )

        self._connection().executemany(
            """
            INSERT INTO session_data (
                session_id, session_raw_id, cam_id, side,
                timestamp_s, bfi, bvi, contrast, mean
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            params,
        )
        self._connection().commit()
        return len(params)

    def stream_session_data(
        self,
        session_id: int,
        side: Optional[int] = None,
        cam_id: Optional[int] = None,
        batch_size: int = 256,
    ) -> Iterator[List[Dict[str, Any]]]:
        sql, bindings = _build_session_data_query(session_id, side, cam_id)
        cursor = self._connection().execute(sql, bindings)
        while True:
            rows = cursor.fetchmany(batch_size)
            if not rows:
                break
            yield [dict(row) for row in rows]

    def iter_session_data(
        self,
        session_id: int,
        side: Optional[int] = None,
        cam_id: Optional[int] = None,
    ) -> Iterator[Dict[str, Any]]:
        sql, bindings = _build_session_data_query(session_id, side, cam_id)
        cursor = self._connection().execute(sql, bindings)
        for row in cursor:
            yield dict(row)

    def delete_session_data(self, session_id: int) -> int:
        cursor = self._connection().execute(
            "DELETE FROM session_data WHERE session_id = ?",
            (session_id,),
        )
        self._connection().commit()
        return cursor.rowcount

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    @property
    def db_path(self) -> Path:
        return self._db_path

    @property
    def compress_raw_hist(self) -> bool:
        return self._compress_raw_hist

    def _encode_hist_blob(self, hist: bytes | bytearray | memoryview) -> bytes:
        raw = bytes(hist)
        if self._compress_raw_hist:
            return zlib.compress(raw)
        return raw

    def close(self) -> None:
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    def __enter__(self) -> "ScanDatabase":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"ScanDatabase(db_path='{self._db_path}')"


def _to_json_text(value: Optional[Dict[str, Any] | str]) -> Optional[str]:
    if isinstance(value, dict):
        return json.dumps(value)
    return value


def _session_row_to_dict(row: sqlite3.Row) -> Dict[str, Any]:
    data = dict(row)
    meta = data.get("session_meta")
    if isinstance(meta, str):
        try:
            data["session_meta"] = json.loads(meta)
        except json.JSONDecodeError:
            pass
    return data


def _raw_frame_row_to_dict(
    row: sqlite3.Row,
    compress_raw_hist: bool,
) -> Dict[str, Any]:
    data = dict(row)
    hist = data.get("hist")
    if isinstance(hist, (bytes, bytearray)):
        data["hist"] = _decode_hist_blob(bytes(hist), compress_raw_hist)
    return data


def _validate_side(side: int) -> None:
    if side not in (0, 1):
        raise ValueError(f"side must be 0 or 1, got {side!r}")


def _validate_side_text(side: str) -> None:
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def _build_raw_query(
    session_id: int,
    side: Optional[str],
    cam_id: Optional[int],
) -> tuple[str, list[Any]]:
    clauses = ["session_id = ?"]
    bindings: list[Any] = [session_id]

    if side is not None:
        _validate_side_text(side)
        clauses.append("side = ?")
        bindings.append(side)

    if cam_id is not None:
        clauses.append("cam_id = ?")
        bindings.append(cam_id)

    where = " AND ".join(clauses)
    sql = (
        f"SELECT * FROM session_raw WHERE {where} "
        "ORDER BY timestamp_s, side, cam_id, frame_id, id"
    )
    return sql, bindings


def _build_session_data_query(
    session_id: int,
    side: Optional[int],
    cam_id: Optional[int],
) -> tuple[str, list[Any]]:
    clauses = ["session_id = ?"]
    bindings: list[Any] = [session_id]

    if side is not None:
        _validate_side(side)
        clauses.append("side = ?")
        bindings.append(side)

    if cam_id is not None:
        clauses.append("cam_id = ?")
        bindings.append(cam_id)

    where = " AND ".join(clauses)
    sql = (
        f"SELECT * FROM session_data WHERE {where} "
        "ORDER BY timestamp_s, side, cam_id, id"
    )
    return sql, bindings


def _decode_hist_blob(blob: bytes, compress_raw_hist: bool) -> bytes:
    if compress_raw_hist:
        return zlib.decompress(blob)
    return blob
