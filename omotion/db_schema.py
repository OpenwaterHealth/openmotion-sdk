"""Versioned schema migrations for the scan database.

The schema version lives in ``PRAGMA user_version`` (an integer in the SQLite
header, transactional, and readable before any table exists). ``upgrade()`` runs
every migration whose version is greater than the DB's current version, in
order, each in its own transaction — so an interrupted update leaves the DB at
the last fully-applied version rather than half-migrated.

**Migrations run automatically when a newer SDK opens an older database** —
``ScanDatabase._init_schema`` calls ``upgrade()`` on every open. A database
already at ``SCHEMA_VERSION`` costs one ``PRAGMA`` read and writes nothing, so
opening a read-only or up-to-date DB is unaffected.

Adding a migration
------------------
1. Write ``_migration_00N_<name>(conn)`` using individual ``conn.execute``
   calls. **Never use ``executescript`` inside a migration** — it issues an
   implicit COMMIT, which would break the surrounding transaction and defeat
   the all-or-nothing guarantee.
2. Append ``(N, "short description", _migration_00N_<name>)`` to ``MIGRATIONS``.
3. Bump ``SCHEMA_VERSION`` to N.
4. Write migrations defensively (``IF NOT EXISTS``, ``_add_column_if_missing``).
   Databases in the field predate versioning and report version 0 while already
   carrying the v1 schema, so migration 1 must be a no-op on them.

Backward compatibility: migration 1 is the historical schema exactly as
``_init_schema`` built it, and is fully idempotent, so an existing field
database baselines to v1 without touching a row.
"""
from __future__ import annotations

import logging

logger = logging.getLogger("omotion.db_schema")

# Bump this whenever a migration is appended to MIGRATIONS.
SCHEMA_VERSION = 2


class SchemaTooNewError(RuntimeError):
    """The database was written by a newer SDK than this one understands.

    Raised instead of opening, because an older SDK writing to a newer schema
    can silently violate constraints it does not know about.
    """


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _columns(conn, table: str) -> set[str]:
    return {row[1] for row in conn.execute(f"PRAGMA table_info('{table}')")}


def _add_column_if_missing(conn, table: str, column: str, decl: str) -> bool:
    """Idempotent ``ALTER TABLE ... ADD COLUMN``. Returns True if it was added.

    SQLite cannot add a column with a non-constant default or a UNIQUE/PRIMARY
    KEY constraint; keep ``decl`` to a type plus an optional constant DEFAULT.
    """
    if column in _columns(conn, table):
        return False
    conn.execute(f"ALTER TABLE {table} ADD COLUMN {column} {decl}")
    return True


# ---------------------------------------------------------------------------
# migrations
# ---------------------------------------------------------------------------

def _migration_001_baseline(conn) -> None:
    """The historical schema, exactly as _init_schema built it before
    versioning existed. Idempotent so field databases (version 0, schema
    already present) baseline to v1 without changing anything."""
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS sessions (
            id             INTEGER PRIMARY KEY,
            session_label  TEXT    NOT NULL,
            session_start  REAL    NOT NULL,
            session_end    REAL,
            session_notes  TEXT,
            session_meta   TEXT
        )
        """
    )
    conn.execute(
        """
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
        )
        """
    )
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS database_settings (
            key            TEXT PRIMARY KEY,
            value          TEXT NOT NULL
        )
        """
    )
    # Issue #92 Step F: databases created before these columns existed get them
    # here. Rows from those older sessions carry frame_id = -1 ("unknown");
    # SessionPlayback treats that as "not playable from the DB — fall back to
    # the corresponding _corrected.csv if present".
    _add_column_if_missing(conn, "session_data", "frame_id",
                           "INTEGER NOT NULL DEFAULT -1")
    _add_column_if_missing(conn, "session_data", "quality", "TEXT DEFAULT 'ok'")

    for name, cols in (
        ("idx_session_data_session_time", "session_id, timestamp_s"),
        ("idx_session_data_session_cam", "session_id, side, cam_id, timestamp_s"),
        ("idx_session_data_session_frame", "session_id, frame_id"),
    ):
        conn.execute(f"CREATE INDEX IF NOT EXISTS {name} ON session_data({cols})")


def _migration_002_session_data_temp(conn) -> None:
    """Per-frame camera temperature (°C) in the corrected record (issue
    #221). Light rows carry the firmware's cached stamp; the stencilled
    dark row's temp is interpolated from its neighbours like the rest of
    that fabricated row. NULL on reduced-mode side averages and all
    pre-migration rows."""
    _add_column_if_missing(conn, "session_data", "temp", "REAL")


# (version, description, function) — ordered, append-only.
#
# Only real, used schema belongs here: every entry lands in every database in
# the field permanently, so an unused table or column is debt that can never be
# cleanly removed. The runner itself is exercised end-to-end against a synthetic
# migration and a checked-in legacy database in tests/test_db_schema.py.
MIGRATIONS: list[tuple[int, str, object]] = [
    (1, "baseline schema (sessions, session_data, database_settings)",
     _migration_001_baseline),
    (2, "session_data.temp — per-frame camera temperature",
     _migration_002_session_data_temp),
]


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------

def current_version(conn) -> int:
    return int(conn.execute("PRAGMA user_version").fetchone()[0])


def upgrade(conn) -> int:
    """Apply pending migrations. Returns the resulting schema version.

    Each migration runs in its own transaction together with its version bump,
    so a failure rolls that migration back entirely and leaves the version at
    the last good value. Raises ``SchemaTooNewError`` if the database is newer
    than this SDK understands.
    """
    version = current_version(conn)

    if version > SCHEMA_VERSION:
        raise SchemaTooNewError(
            f"database schema version {version} is newer than this SDK supports "
            f"({SCHEMA_VERSION}); upgrade the SDK to open it"
        )
    if version == SCHEMA_VERSION:
        return version  # up to date: no write, so read-only DBs still open

    for target, description, migrate in MIGRATIONS:
        if target <= version:
            continue
        logger.info("scan DB: applying schema migration %d (%s)", target, description)
        # Explicit transaction: Python's sqlite3 does not open one for DDL, so
        # without this a failure would leave a half-applied migration.
        conn.execute("BEGIN")
        try:
            migrate(conn)
            # PRAGMA cannot be parameterized; target comes from MIGRATIONS, and
            # int() keeps it non-injectable regardless.
            conn.execute(f"PRAGMA user_version = {int(target)}")
            conn.commit()
        except Exception:
            conn.rollback()
            logger.exception("scan DB: migration %d (%s) failed and was rolled back",
                             target, description)
            raise
        version = target

    return version
