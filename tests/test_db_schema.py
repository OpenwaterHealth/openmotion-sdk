"""Tests for omotion.db_schema — versioned scan-database migrations.

The production registry deliberately contains only the baseline migration, so
the runner itself is verified two ways:

1. against a **checked-in legacy database** (``fixtures/legacy_scans_v0.db``) —
   a real pre-versioning, pre-``frame_id`` file, which is the upgrade an app
   update actually performs in the field;
2. against a **synthetic test migration** registered via monkeypatch, which
   exercises adding a table and altering an existing one without putting unused
   schema into every production database.
"""
import shutil
import sqlite3
from pathlib import Path

import pytest

from omotion import db_key, db_schema
from omotion.ScanDatabase import ScanDatabase

FIXTURE = Path(__file__).parent / "fixtures" / "legacy_scans_v0.db"
FIXED_KEY = "a" * 64


@pytest.fixture
def legacy_db(tmp_path):
    """A writable copy of the checked-in legacy database. Always a copy — the
    fixture is frozen and must never be migrated in place."""
    dest = tmp_path / "scans.db"
    shutil.copy2(FIXTURE, dest)
    return dest


@pytest.fixture
def clinical(monkeypatch):
    pytest.importorskip("sqlcipher3")
    monkeypatch.setattr(db_key, "require_encryption", lambda: True)
    monkeypatch.setattr(db_key, "get_key", lambda *, create=False: FIXED_KEY)


# ---------------------------------------------------------------------------
# The checked-in legacy database — the real update path
# ---------------------------------------------------------------------------

def test_fixture_is_the_expected_legacy_shape():
    """Guards the fixture itself: if this fails, the fixture was regenerated
    against the current schema and no longer represents what is on disk in the
    field, which would silently weaken every test below."""
    con = sqlite3.connect(FIXTURE)
    try:
        assert con.execute("PRAGMA user_version").fetchone()[0] == 0
        cols = {r[1] for r in con.execute("PRAGMA table_info('session_data')")}
        assert "frame_id" not in cols
        assert "quality" not in cols
        assert "temp" not in cols
        assert con.execute("SELECT count(*) FROM session_data").fetchone()[0] == 200
    finally:
        con.close()


def test_legacy_fixture_upgrades_and_preserves_data(legacy_db):
    before = sqlite3.connect(legacy_db)
    rows_before = before.execute("SELECT count(*) FROM session_data").fetchone()[0]
    label_before = before.execute("SELECT session_label FROM sessions").fetchone()[0]
    before.close()

    db = ScanDatabase(db_path=str(legacy_db))     # opening runs the migration
    try:
        conn = db._connection()
        assert db_schema.current_version(conn) == db_schema.SCHEMA_VERSION

        # the ADD COLUMN branch of migration 1 ran against a real legacy
        # file, and migration 2 added the temp column on top
        cols = {r[1] for r in conn.execute("PRAGMA table_info('session_data')")}
        assert "frame_id" in cols
        assert "quality" in cols
        assert "temp" in cols

        # the index that did not exist in the legacy file was created
        idx = {r[0] for r in conn.execute(
            "SELECT name FROM sqlite_master WHERE type='index' "
            "AND name NOT LIKE 'sqlite_%'")}
        assert "idx_session_data_session_frame" in idx

        # nothing was lost, and pre-existing rows got the documented sentinels
        assert conn.execute("SELECT count(*) FROM session_data").fetchone()[0] == rows_before
        assert db.get_session(1)["session_label"] == label_before
        row = next(iter(db.iter_session_data(1)))
        assert row["frame_id"] == -1          # "unknown" sentinel
        assert row["quality"] == "ok"
        assert row["temp"] is None            # pre-migration rows: no reading
        assert row["bfi"] is not None
    finally:
        db.close()


def test_legacy_fixture_upgrade_is_idempotent(legacy_db):
    ScanDatabase(db_path=str(legacy_db)).close()
    db = ScanDatabase(db_path=str(legacy_db))     # second open: nothing to do
    try:
        assert db_schema.current_version(db._connection()) == db_schema.SCHEMA_VERSION
        assert db._connection().execute(
            "SELECT count(*) FROM session_data").fetchone()[0] == 200
    finally:
        db.close()


def test_legacy_fixture_encrypted_then_upgraded(clinical, legacy_db):
    """The clinical upgrade path end to end: a legacy plaintext database is
    encrypted, then its schema is migrated on the next open."""
    from omotion import db_migrate

    assert db_migrate.migrate_plaintext_to_encrypted(legacy_db) is True
    with open(legacy_db, "rb") as fh:
        assert fh.read(16) != b"SQLite format 3\x00"

    db = ScanDatabase(db_path=str(legacy_db))
    try:
        conn = db._connection()
        assert db_schema.current_version(conn) == db_schema.SCHEMA_VERSION
        assert "frame_id" in {r[1] for r in conn.execute("PRAGMA table_info('session_data')")}
        assert conn.execute("SELECT count(*) FROM session_data").fetchone()[0] == 200
        assert db.get_session(1)["session_label"] == "20251217_160949_owTEST01"
    finally:
        db.close()


# ---------------------------------------------------------------------------
# Fresh databases
# ---------------------------------------------------------------------------

def test_fresh_db_is_created_at_current_version(tmp_path):
    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        conn = db._connection()
        assert db_schema.current_version(conn) == db_schema.SCHEMA_VERSION
        tables = {r[0] for r in conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table'")}
        assert {"sessions", "session_data", "database_settings"} <= tables
    finally:
        db.close()


def test_production_registry_holds_no_unused_schema():
    """Every migration lands in every field database permanently, so the
    registry must contain only real, used schema. The runner is proven by the
    synthetic migration below, not by shipping demo tables."""
    assert [v for v, _, _ in db_schema.MIGRATIONS] == [1, 2]
    assert db_schema.SCHEMA_VERSION == 2


# ---------------------------------------------------------------------------
# A synthetic migration — proves the runner handles a real schema change
# ---------------------------------------------------------------------------

def _test_migration_003(conn) -> None:
    """Stand-in for a future migration: adds a table AND alters an existing one."""
    conn.execute(
        "CREATE TABLE IF NOT EXISTS session_annotations ("
        " id INTEGER PRIMARY KEY,"
        " session_id INTEGER NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,"
        " timestamp_s REAL NOT NULL, label TEXT NOT NULL, note TEXT)"
    )
    conn.execute(
        "CREATE INDEX IF NOT EXISTS idx_session_annotations_session "
        "ON session_annotations(session_id, timestamp_s)"
    )
    db_schema._add_column_if_missing(conn, "sessions", "operator_id", "TEXT")


@pytest.fixture
def with_test_migration(monkeypatch):
    """Register a v3 migration for the duration of one test."""
    monkeypatch.setattr(
        db_schema, "MIGRATIONS",
        list(db_schema.MIGRATIONS) + [(3, "test: annotations", _test_migration_003)],
    )
    monkeypatch.setattr(db_schema, "SCHEMA_VERSION", 3)


def test_new_migration_applies_to_a_legacy_db(with_test_migration, legacy_db):
    """A legacy database jumps straight from version 0 to the newest version,
    running every intervening migration in order."""
    db = ScanDatabase(db_path=str(legacy_db))
    try:
        conn = db._connection()
        assert db_schema.current_version(conn) == 3

        # migration 1 ran (ADD COLUMN on the legacy table)
        assert "frame_id" in {r[1] for r in conn.execute("PRAGMA table_info('session_data')")}
        # migration 2 ran (temp column)
        assert "temp" in {r[1] for r in conn.execute("PRAGMA table_info('session_data')")}
        # migration 3 ran (new table + altered table)
        assert "operator_id" in {r[1] for r in conn.execute("PRAGMA table_info('sessions')")}
        conn.execute(
            "INSERT INTO session_annotations(session_id, timestamp_s, label)"
            " VALUES(1, 4.2, 'cuff inflated')")
        conn.commit()
        assert conn.execute(
            "SELECT label FROM session_annotations").fetchone()[0] == "cuff inflated"
        # and the original 200 rows are untouched
        assert conn.execute("SELECT count(*) FROM session_data").fetchone()[0] == 200
    finally:
        db.close()


def test_new_migration_applies_to_an_up_to_date_db(with_test_migration, tmp_path):
    """The common update case: a database already current gets only the new
    migration."""
    path = str(tmp_path / "scans.db")
    with_v1 = ScanDatabase(db_path=path)
    sid = with_v1.create_session("S", 1.0)
    with_v1.close()

    db = ScanDatabase(db_path=path)
    try:
        conn = db._connection()
        assert db_schema.current_version(conn) == 3
        assert "operator_id" in {r[1] for r in conn.execute("PRAGMA table_info('sessions')")}
        assert db.get_session(sid)["session_label"] == "S"
    finally:
        db.close()


def test_new_migration_applies_to_an_encrypted_db(clinical, with_test_migration, tmp_path):
    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        conn = db._connection()
        assert db_schema.current_version(conn) == 3
        conn.execute(
            "INSERT INTO sessions(session_label, session_start, operator_id)"
            " VALUES('S', 1.0, 'ethan')")
        conn.commit()
        assert conn.execute("SELECT operator_id FROM sessions").fetchone()[0] == "ethan"
    finally:
        db.close()
    with open(tmp_path / "scans.db", "rb") as fh:
        assert fh.read(16) != b"SQLite format 3\x00"


# ---------------------------------------------------------------------------
# Safety: atomicity, downgrade guard, no-write when current
# ---------------------------------------------------------------------------

def test_failed_migration_rolls_back_and_keeps_version(tmp_path, monkeypatch):
    def _boom(conn):
        conn.execute("CREATE TABLE half_applied(x)")
        raise RuntimeError("simulated migration failure")

    monkeypatch.setattr(
        db_schema, "MIGRATIONS",
        [(1, "baseline", db_schema._migration_001_baseline), (2, "boom", _boom)],
    )
    monkeypatch.setattr(db_schema, "SCHEMA_VERSION", 2)
    con = sqlite3.connect(tmp_path / "scans.db")
    try:
        with pytest.raises(RuntimeError, match="simulated migration failure"):
            db_schema.upgrade(con)
        assert db_schema.current_version(con) == 1        # stopped at last good
        tables = {r[0] for r in con.execute(
            "SELECT name FROM sqlite_master WHERE type='table'")}
        assert "half_applied" not in tables               # rolled back
        assert "sessions" in tables                       # migration 1 kept
    finally:
        con.close()


def test_db_from_a_newer_sdk_is_refused(tmp_path):
    con = sqlite3.connect(tmp_path / "future.db")
    try:
        con.execute(f"PRAGMA user_version = {db_schema.SCHEMA_VERSION + 5}")
        with pytest.raises(db_schema.SchemaTooNewError):
            db_schema.upgrade(con)
    finally:
        con.close()


def test_up_to_date_db_performs_no_write(tmp_path, monkeypatch):
    path = tmp_path / "scans.db"
    ScanDatabase(db_path=str(path)).close()

    con = sqlite3.connect(path)
    try:
        def _fail(*_a, **_k):
            raise AssertionError("upgrade() must not run a migration when current")

        monkeypatch.setattr(db_schema, "MIGRATIONS", [(1, "x", _fail)])
        assert db_schema.upgrade(con) == db_schema.SCHEMA_VERSION
    finally:
        con.close()
