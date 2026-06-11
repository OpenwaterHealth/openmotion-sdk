"""
ScanDatabase - SQLite database manager for openwater scan sessions.

The database stores session metadata (``sessions``) and the corrected
science record (``session_data`` — final-branch BFI/BVI/mean/contrast,
one row per camera per frame, with cam_id=-1 rows holding reduced-mode
side averages).

Raw histograms are NOT stored here: the raw CSVs written by the
pipeline's Tee("raw") → CsvSink are the only raw record. Databases
created by older SDKs may contain a ``session_raw`` table; this module
neither reads nor writes it, but leaves existing data untouched.
"""

from __future__ import annotations

import json
import sqlite3
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

            CREATE TABLE IF NOT EXISTS session_data (
                id               INTEGER PRIMARY KEY,
                session_id       INTEGER NOT NULL REFERENCES sessions(id)
                                          ON DELETE CASCADE,
                cam_id           INTEGER NOT NULL,
                side             INTEGER NOT NULL CHECK(side IN (0, 1)),
                frame_id         INTEGER NOT NULL DEFAULT -1,
                timestamp_s      REAL    NOT NULL,
                bfi              REAL,
                bvi              REAL,
                contrast         REAL,
                mean             REAL,
                quality          TEXT DEFAULT 'ok'
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
        # Issue #92 Step F: add session_data.frame_id to DBs created
        # before the column existed. Rows from those older sessions get
        # the sentinel value -1 ("frame_id unknown"); SessionPlayback
        # treats that as "this session can't be played back from the DB
        # — fall back to the corresponding _corrected.csv if present".
        # The index creation is outside the if-block because fresh DBs
        # (created with the new CREATE TABLE) also need it.
        cols = {
            r[1] for r in self._connection().execute("PRAGMA table_info('session_data')")
        }
        if "frame_id" not in cols:
            self._connection().execute(
                "ALTER TABLE session_data ADD COLUMN frame_id INTEGER NOT NULL DEFAULT -1"
            )
        if "quality" not in cols:
            try:
                self._connection().execute(
                    "ALTER TABLE session_data ADD COLUMN quality TEXT DEFAULT 'ok'"
                )
            except sqlite3.OperationalError:
                pass  # column already exists
        self._connection().execute(
            "CREATE INDEX IF NOT EXISTS idx_session_data_session_frame "
            "ON session_data(session_id, frame_id)"
        )
        self._connection().commit()

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
    # Session data
    # ------------------------------------------------------------------

    def insert_session_data(
        self,
        session_id: int,
        cam_id: int,
        side: int,
        timestamp_s: float,
        *,
        frame_id: int = -1,
        bfi: Optional[float] = None,
        bvi: Optional[float] = None,
        contrast: Optional[float] = None,
        mean: Optional[float] = None,
        quality: str = "ok",
    ) -> int:
        # frame_id defaults to the "unknown" sentinel (-1) so callers from
        # before #92 Step F still work; new callers (ScanDBSink) pass the
        # real absolute_frame_id so corrected-CSV playback can merge
        # per-side samples exactly the way the CSV writer did.
        _validate_side(side)
        cursor = self._connection().execute(
            """
            INSERT INTO session_data (
                session_id, cam_id, side,
                frame_id, timestamp_s, bfi, bvi, contrast, mean, quality
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                session_id,
                cam_id,
                side,
                int(frame_id),
                timestamp_s,
                bfi,
                bvi,
                contrast,
                mean,
                quality,
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
                    row["cam_id"],
                    row["side"],
                    int(row.get("frame_id", -1)),
                    row["timestamp_s"],
                    row.get("bfi"),
                    row.get("bvi"),
                    row.get("contrast"),
                    row.get("mean"),
                    row.get("quality", "ok"),
                )
            )

        self._connection().executemany(
            """
            INSERT INTO session_data (
                session_id, cam_id, side,
                frame_id, timestamp_s, bfi, bvi, contrast, mean, quality
            )
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
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
        t_lo: Optional[float] = None,
        t_hi: Optional[float] = None,
    ) -> Iterator[Dict[str, Any]]:
        """Iterate session_data rows for one session, optionally filtered
        by side, cam_id, and/or timestamp range [t_lo, t_hi] (inclusive).
        Time range is the lazy-load entry point — viewer queries a
        narrow window on pan-into-past instead of paging the full scan.
        Both indexes (session_id, timestamp_s) and (session_id, side,
        cam_id, timestamp_s) cover the access patterns we care about."""
        sql, bindings = _build_session_data_query(session_id, side, cam_id, t_lo, t_hi)
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


def _validate_side(side: int) -> None:
    if side not in (0, 1):
        raise ValueError(f"side must be 0 or 1, got {side!r}")


def _build_session_data_query(
    session_id: int,
    side: Optional[int],
    cam_id: Optional[int],
    t_lo: Optional[float] = None,
    t_hi: Optional[float] = None,
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

    if t_lo is not None:
        clauses.append("timestamp_s >= ?")
        bindings.append(float(t_lo))

    if t_hi is not None:
        clauses.append("timestamp_s <= ?")
        bindings.append(float(t_hi))

    where = " AND ".join(clauses)
    sql = (
        f"SELECT * FROM session_data WHERE {where} "
        "ORDER BY timestamp_s, side, cam_id, id"
    )
    return sql, bindings
