"""Tests for omotion.db_migrate — plaintext -> encrypted migration."""
import sqlite3
import sys

import pytest

from omotion import db_key, db_migrate, db_open

pytest.importorskip("sqlcipher3")

FIXED_KEY = "a" * 64
MAGIC = b"SQLite format 3\x00"


@pytest.fixture
def clinical(monkeypatch):
    monkeypatch.setattr(db_key, "require_encryption", lambda: True)
    monkeypatch.setattr(db_key, "get_key", lambda *, create=False: FIXED_KEY)


def _make_plaintext(path):
    con = sqlite3.connect(path)
    con.executescript(
        "CREATE TABLE sessions(id INTEGER PRIMARY KEY, session_label TEXT);"
        "CREATE INDEX ix ON sessions(session_label);"
        "CREATE TABLE logs(id INTEGER PRIMARY KEY, ev TEXT);"
    )
    con.execute("INSERT INTO sessions(session_label) VALUES('OLD-PHI')")
    con.execute("INSERT INTO logs(ev) VALUES('e')")
    con.commit()
    con.close()


def test_migration_encrypts_and_preserves(clinical, tmp_path):
    p = tmp_path / "scans.db"
    _make_plaintext(p)
    assert db_migrate.migrate_plaintext_to_encrypted(p) is True

    with open(p, "rb") as fh:                    # opaque now
        assert fh.read(16) != MAGIC

    con = db_open.connect(p)                       # readable via keyed helper
    assert con.execute("SELECT session_label FROM sessions").fetchone()[0] == "OLD-PHI"
    assert con.execute("SELECT count(*) FROM logs").fetchone()[0] == 1
    idx = con.execute(
        "SELECT count(*) FROM sqlite_master "
        "WHERE type='index' AND name NOT LIKE 'sqlite_%'"
    ).fetchone()[0]
    assert idx == 1
    con.close()

    assert (tmp_path / "scans.db.pre-encryption.bak").exists()   # backup kept
    with open(tmp_path / "scans.db.pre-encryption.bak", "rb") as fh:
        assert fh.read(16) == MAGIC                # backup is the plaintext original


def test_migration_noop_on_already_encrypted(clinical, tmp_path):
    p = tmp_path / "scans.db"
    con = db_open.connect(p)
    con.execute("CREATE TABLE t(x)")
    con.commit()
    con.close()
    assert db_migrate.migrate_plaintext_to_encrypted(p) is False
    con = db_open.connect(p)
    assert con.execute("SELECT count(*) FROM t").fetchone()[0] == 0
    con.close()


def test_migration_noop_on_missing(clinical, tmp_path):
    assert db_migrate.migrate_plaintext_to_encrypted(tmp_path / "nope.db") is False


@pytest.mark.skipif(not sys.platform.startswith("win"),
                    reason="POSIX allows replacing an open file")
def test_migration_with_an_open_handle_fails_actionably(clinical, tmp_path):
    """Found by a real end-to-end run: a caller holding the DB open turns the
    atomic replace into a bare 'WinError 5: Access is denied'. It must instead
    say what to do, and must leave the original database intact."""
    p = tmp_path / "scans.db"
    _make_plaintext(p)
    holder = sqlite3.connect(p)          # deliberately left open
    try:
        with pytest.raises(PermissionError, match="still open"):
            db_migrate.migrate_plaintext_to_encrypted(p)
        # the original must survive untouched — still plaintext, still readable
        assert holder.execute(
            "SELECT session_label FROM sessions").fetchone()[0] == "OLD-PHI"
        with open(p, "rb") as fh:
            assert fh.read(16) == MAGIC
    finally:
        holder.close()


def test_migration_preserves_uncheckpointed_wal(clinical, tmp_path):
    """A real scans.db is WAL-mode. A crash can leave committed rows only in an
    uncheckpointed -wal. Migration must fold them in (checkpoint before export),
    not lose them when the stale sidecars are removed. We create a genuine
    uncheckpointed -wal via a child that hard-exits without closing."""
    import subprocess
    import sys
    import textwrap

    p = tmp_path / "scans.db"
    child = textwrap.dedent(
        """
        import os, sqlite3, sys
        con = sqlite3.connect(sys.argv[1])
        con.execute("PRAGMA journal_mode=WAL")
        con.execute("CREATE TABLE sessions(id INTEGER PRIMARY KEY, label TEXT)")
        con.execute("INSERT INTO sessions(label) VALUES('WAL-ROW')")
        con.commit()
        os._exit(0)  # hard exit: no checkpoint, no clean close -> -wal persists
        """
    )
    subprocess.run([sys.executable, "-c", child, str(p)], check=True)
    assert (tmp_path / "scans.db-wal").exists()          # precondition: real -wal

    assert db_migrate.migrate_plaintext_to_encrypted(p) is True
    con = db_open.connect(p)
    assert con.execute("SELECT label FROM sessions").fetchone()[0] == "WAL-ROW"
    con.close()
    assert not (tmp_path / "scans.db-wal").exists()      # stale sidecar removed


def test_migration_recovers_from_stale_enc_tmp(clinical, tmp_path):
    """A leftover .enc.tmp from a previously killed run must be cleared, not
    block a fresh migration."""
    p = tmp_path / "scans.db"
    _make_plaintext(p)
    (tmp_path / "scans.db.enc.tmp").write_bytes(b"leftover from a killed run")
    assert db_migrate.migrate_plaintext_to_encrypted(p) is True
    con = db_open.connect(p)
    assert con.execute("SELECT session_label FROM sessions").fetchone()[0] == "OLD-PHI"
    con.close()
    assert not (tmp_path / "scans.db.enc.tmp").exists()  # consumed by os.replace


def test_schema_upgrade_adds_columns_on_encrypted_db(clinical, tmp_path):
    """An app update over an OLD encrypted DB (predating frame_id/quality) must
    run _init_schema's ALTER ADD COLUMN branch on the sqlcipher3 driver and
    preserve pre-existing rows with the sentinel defaults. This exercises the
    ALTER path that the fresh-DB schema test cannot."""
    from sqlcipher3 import dbapi2 as sqlcipher

    from omotion import ScanDatabase

    path = str(tmp_path / "scans.db")
    # Build an OLD-schema encrypted DB directly: session_data WITHOUT frame_id
    # or quality columns.
    con = sqlcipher.connect(path)
    con.execute(f"PRAGMA key = \"x'{FIXED_KEY}'\"")
    con.executescript(
        "CREATE TABLE sessions(id INTEGER PRIMARY KEY, session_label TEXT NOT NULL,"
        " session_start REAL NOT NULL, session_end REAL, session_notes TEXT,"
        " session_meta TEXT);"
        "CREATE TABLE session_data(id INTEGER PRIMARY KEY, session_id INTEGER NOT NULL,"
        " cam_id INTEGER NOT NULL, side INTEGER NOT NULL, timestamp_s REAL NOT NULL,"
        " bfi REAL, bvi REAL, contrast REAL, mean REAL);"
        "CREATE TABLE database_settings(key TEXT PRIMARY KEY, value TEXT NOT NULL);"
    )
    con.execute("INSERT INTO sessions(session_label, session_start) VALUES('OLD', 1.0)")
    con.execute(
        "INSERT INTO session_data(session_id, cam_id, side, timestamp_s, bfi)"
        " VALUES(1, 0, 0, 1.0, 1.5)"
    )
    con.commit()
    con.close()

    # Open via ScanDatabase -> _init_schema runs ALTER ADD COLUMN on the
    # encrypted driver, adding frame_id (-1) and quality ('ok').
    db = ScanDatabase(db_path=path)
    try:
        cols = {r[1] for r in db._connection().execute("PRAGMA table_info('session_data')")}
        assert "frame_id" in cols and "correction_status" in cols
        row = next(iter(db.iter_session_data(1)))
        assert row["bfi"] == 1.5
        assert row["frame_id"] == -1
        assert row["correction_status"] == "ok"
    finally:
        db.close()


def test_schema_migration_runs_on_encrypted_db(clinical, tmp_path):
    """An app-update schema bump (_init_schema ALTER/CREATE INDEX on every open)
    must run cleanly on an encrypted DB and preserve data. See §7.1."""
    from omotion import ScanDatabase

    path = str(tmp_path / "scans.db")
    db = ScanDatabase(db_path=path)               # creates encrypted + schema
    sid = db.create_session("S", 1.0)
    db.insert_session_data(sid, cam_id=0, side=0, timestamp_s=1.0, bfi=1.5)
    db.close()
    # reopen — _init_schema re-runs ADD COLUMN / CREATE INDEX IF NOT EXISTS:
    db2 = ScanDatabase(db_path=path)
    rows = list(db2.iter_session_data(sid))
    db2.close()
    assert rows[0]["bfi"] == 1.5
