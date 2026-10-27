"""Encryption key provisioning + the process-level encryption policy.

This is the ONLY module that touches the OS keystore. ``ScanDatabase`` (via
``db_open``) self-fetches the key here; no other class ever holds it.

The policy defaults to plaintext so every existing caller keeps working with no
change. A clinical build opts in exactly once by calling ``set_policy`` from its
signed build config; see docs/superpowers/specs/2026-07-23-sqlite-encryption-design.md
(§5.2).
"""
from __future__ import annotations

import logging
import secrets
import sys
from pathlib import Path

logger = logging.getLogger("omotion.db_key")

_SERVICE = "openmotion"
_ENTRY = "scan-db"

# None until set; require_encryption() collapses it to a bool (default False).
_require_encryption: bool | None = None


class EncryptionUnavailable(RuntimeError):
    """The encryption backend or dependency is missing where it is required,
    or a database's encryption state does not match the active policy."""


class EncryptionKeyMissing(RuntimeError):
    """An encrypted DB was opened but no usable key is available."""


def set_policy(*, require_encryption: bool) -> None:
    """Set the process-wide encryption policy. Called once by the app at startup.

    When ``require_encryption`` is True the keyring backend is verified now, so a
    misconfigured clinical machine fails at startup rather than at scan time.
    """
    global _require_encryption
    _require_encryption = bool(require_encryption)
    if _require_encryption:
        _assert_backend()


def require_encryption() -> bool:
    """True iff new/existing scan DBs must be encrypted. Default False (unset)."""
    return bool(_require_encryption)


def _keyring():
    """Import ``keyring``, converting a missing dependency into the actionable
    ``EncryptionUnavailable`` rather than a bare ``ModuleNotFoundError``.

    ``keyring`` and ``sqlcipher3`` ship in the ``[encryption]`` extra, so a build
    that forgets it fails at the first keystore touch. Mirrors how ``db_open``
    handles the ``sqlcipher3`` import — the message has to name the fix, because
    the place this surfaces is a packaged clinical app on a bench.

    Also the single chokepoint where macOS is refused: every keystore touch in
    this module (``_assert_backend``, ``get_key``, ``import_key``, and
    ``export_key`` via ``get_key``) routes through here.
    """
    _assert_keystore_platform()
    try:
        import keyring
    except ImportError as exc:  # pragma: no cover - exercised via monkeypatch
        raise EncryptionUnavailable(
            "the scan-database encryption policy requires the 'keyring' package, "
            "which is not installed. Install the SDK with its encryption extra: "
            "pip install 'openmotion-sdk[encryption]' (a packaged app must also "
            "list keyring and sqlcipher3 among its build requirements, or "
            "PyInstaller cannot bundle them)."
        ) from exc
    return keyring


def _assert_keystore_platform() -> None:
    """Refuse every keystore access on macOS.

    macOS is a research-only platform for this product — it is never shipped in
    clinical mode, so the encrypted scan DB (and therefore the keystore) has no
    role there. The macOS Keychain would be a technically adequate backend, but
    accepting it would mean the encryption path silently exists on a platform
    that is not validated for clinical use. Reaching the keystore on darwin is
    therefore a build/configuration error, and a loud one is better than a
    working-but-unvalidated encryption path.

    A macOS *research* build never gets here: ``require_encryption()`` is False,
    so ``db_open`` takes the plaintext branch without asking for a key.
    """
    if sys.platform == "darwin":
        raise EncryptionUnavailable(
            "the scan-database keystore is not available on macOS. macOS builds "
            "are research-only and are never validated for clinical mode, so the "
            "encryption policy must stay off (require_encryption=False) there. "
            "Reaching the keystore on macOS means something enabled the clinical "
            "encryption path on an unsupported platform — fix the build config "
            "rather than the keystore."
        )


# The OS-owned keystores approved to hold the scan-db key, by backend module.
# Windows Credential Manager is the only one: it is hardware/OS-protected and
# per-user, and Windows is the only platform shipped in clinical mode. The
# macOS Keychain is deliberately absent — see _assert_keystore_platform, which
# refuses darwin outright and fires before this check is ever reached.
#
# Everything else is rejected — most importantly ``keyrings.alt`` (plaintext or
# obfuscated files), ``keyring.backends.fail`` (no keystore at all) and
# ``keyring.backends.chainer`` (defers to whatever it discovered, so it cannot
# be verified up front). Matching on the module rather than the class name
# matters: a plaintext backend is free to call its class ``WinVaultKeyring``,
# and a name-only check would wave it through.
_SUPPORTED_BACKENDS = {
    "keyring.backends.Windows": "Windows Credential Manager",
}


def _assert_backend() -> None:
    keyring = _keyring()

    kr = keyring.get_keyring()
    if type(kr).__module__ not in _SUPPORTED_BACKENDS:
        supported = ", ".join(sorted(_SUPPORTED_BACKENDS.values()))
        raise EncryptionUnavailable(
            f"encryption policy requires an OS keystore ({supported}), got "
            f"{type(kr).__module__}.{type(kr).__name__}"
        )


def get_key(*, create: bool = False) -> str:
    """Return the 64-hex-char (256-bit) DB key from the keystore.

    With ``create=True`` a fresh random key is generated and stored if none
    exists (used when creating a new encrypted DB). With ``create=False`` a
    missing key raises ``EncryptionKeyMissing`` (used when opening an existing
    encrypted DB). Never logs or echoes the key.
    """
    keyring = _keyring()

    value = keyring.get_password(_SERVICE, _ENTRY)
    if value is None:
        if not create:
            raise EncryptionKeyMissing(
                "no scan-db encryption key in the keystore for this user/machine"
            )
        value = secrets.token_bytes(32).hex()
        keyring.set_password(_SERVICE, _ENTRY, value)
        # The key is deliberately NEVER written to the disk it protects — a
        # recovery file sitting beside the encrypted DB defeats the stolen-disk
        # threat model. So provisioning is the one moment where forgetting to
        # escrow becomes unrecoverable; make it loud rather than silent.
        logger.warning(
            "scan-db encryption key provisioned for this Windows user on this "
            "machine. It is NOT backed up anywhere. If this profile or machine "
            "is rebuilt, every scan in the encrypted database becomes "
            "permanently unreadable. Escrow it now to secure storage OFF this "
            "machine:  python -m omotion.db_key export <path>"
        )
    return value


def export_key(path: str | Path) -> None:
    """Write the current key to ``path`` for planned recovery/migration.

    Raises ``EncryptionKeyMissing`` if there is no key to export. The caller is
    responsible for moving the file to secure storage (see design doc §6.1).
    """
    key = get_key(create=False)
    Path(path).write_text(key, encoding="ascii")


def import_key(path: str | Path) -> None:
    """Load a key exported by ``export_key`` back into the keystore."""
    keyring = _keyring()

    key = Path(path).read_text(encoding="ascii").strip()
    if len(key) != 64 or any(c not in "0123456789abcdef" for c in key.lower()):
        raise ValueError("recovery key must be 64 hex characters")
    keyring.set_password(_SERVICE, _ENTRY, key)


def _main(argv: list[str] | None = None) -> int:
    """``python -m omotion.db_key export|import <path>`` — the escrow CLI the
    provisioning warning points at. Kept deliberately small: recovery has to
    work from a bare console on a machine being rebuilt."""
    import argparse

    parser = argparse.ArgumentParser(
        prog="python -m omotion.db_key",
        description="Export or restore the scan-database encryption key. "
                    "Store exports in secure storage OFF this machine — a key "
                    "beside the database it protects provides no protection.",
    )
    sub = parser.add_subparsers(dest="command", required=True)
    p_exp = sub.add_parser("export", help="write this machine's key to a file")
    p_exp.add_argument("path")
    p_imp = sub.add_parser("import", help="restore a key from a file")
    p_imp.add_argument("path")
    args = parser.parse_args(argv)

    try:
        if args.command == "export":
            export_key(args.path)
            print(f"key exported to {args.path}\n"
                  f"Move it to secure storage off this machine and delete the local copy.")
        else:
            import_key(args.path)
            print("key restored to the Windows Credential Manager for this user.")
    except (EncryptionKeyMissing, ValueError, OSError) as exc:
        print(f"error: {exc}")
        return 1
    return 0


if __name__ == "__main__":  # pragma: no cover - exercised via subprocess in tests
    raise SystemExit(_main())
