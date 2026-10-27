"""dfu-util is gone: firmware update is pure Python (omotion.dfu).

The wheel used to carry dfu-util executables for four platforms and shell out
to them. Nothing in the package may bring a bundled flashing tool back, or
start a process to flash. Reads the files in this checkout; needs no hardware.
"""

from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "omotion"


def test_no_dfu_util_directory_or_binary_ships():
    stale = sorted(
        p.relative_to(PKG).as_posix()
        for p in PKG.rglob("*")
        if "dfu-util" in p.as_posix().lower() or p.suffix.lower() == ".exe"
    )
    assert stale == []


def test_package_data_does_not_list_dfu_util():
    assert "dfu-util" not in (ROOT / "pyproject.toml").read_text(encoding="utf-8")


def test_package_source_does_not_mention_dfu_util_except_as_history():
    """The only references left explain what a function replaced."""
    allowed = {"dfu/migration.py", "dfu/stm32dfu.py", "dfu/PROVENANCE.txt"}
    hits = {
        p.relative_to(PKG).as_posix()
        for p in PKG.rglob("*")
        if p.is_file() and p.suffix in {".py", ".txt"}
        and "dfu-util" in p.read_text(encoding="utf-8", errors="ignore")
    }
    assert hits <= allowed


def test_removed_modules_are_gone():
    import importlib

    for name in ("omotion.DFUProgrammer", "omotion.bootloader_install"):
        with pytest.raises(ModuleNotFoundError):
            importlib.import_module(name)


def test_firmware_update_does_not_spawn_processes():
    for rel in ("firmware_update.py", "dfu/stm32dfu.py", "dfu/migration.py"):
        assert "subprocess" not in (PKG / rel).read_text(encoding="utf-8")
