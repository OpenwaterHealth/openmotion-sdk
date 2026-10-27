"""Unit tests for scripts/update_firmware.py (no hardware).

The script picks the images for a device out of a release folder by name and
hands them to FirmwareUpdater. A wrong pick is not recoverable once a
bootloader is involved, so the picking is pinned here, along with the rule
that nothing irreversible is asked for without --acknowledge-irreversible.
"""

import importlib.util
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "update_firmware.py"

# The names of the 2026-10-12 release folder.
RELEASE_FILES = (
    "open-motion-console-bl-updater.bin",
    "open-motion-sensor-bl-updater.bin",
    "openmotion-console-fw-production.bin",
    "openmotion-console-fw-signed.bin",
    "openmotion-sensor-fw-production.bin",
    "openmotion-sensor-fw-signed.bin",
)


def _load_script():
    spec = importlib.util.spec_from_file_location("update_firmware_script", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def release_dir(tmp_path):
    for name in RELEASE_FILES:
        (tmp_path / name).write_bytes(b"\x00" * 16)
    return tmp_path


def test_script_source_is_ascii_only():
    # cp1252 consoles crash on emoji and U+2011 hyphens; keep the script pure ASCII.
    SCRIPT_PATH.read_bytes().decode("ascii")


@pytest.mark.parametrize("device, product", [
    ("console", "console"), ("left", "sensor"), ("right", "sensor"),
])
def test_device_maps_to_its_product(device, product):
    assert _load_script().product_of(device) == product


@pytest.mark.parametrize("product", ["console", "sensor"])
def test_release_folder_images_are_picked_by_product_and_role(release_dir, product):
    images = _load_script().find_release_images(release_dir, product)

    assert images == {
        "app": release_dir / f"openmotion-{product}-fw-signed.bin",
        "updater": release_dir / f"open-motion-{product}-bl-updater.bin",
        "production": release_dir / f"openmotion-{product}-fw-production.bin",
    }


def test_sdk_release_asset_names_are_recognised_too(tmp_path):
    for name in ("motion-sensor-fw-signed.bin", "motion-sensor-production.bin",
                 "motion-sensor-fw-baremetal-fpga.bin"):
        (tmp_path / name).write_bytes(b"\x00")

    images = _load_script().find_release_images(tmp_path, "sensor")

    assert images == {"app": tmp_path / "motion-sensor-fw-signed.bin",
                      "production": tmp_path / "motion-sensor-production.bin"}


def test_updater_with_signed_in_its_name_is_not_taken_for_the_application(tmp_path):
    updater = tmp_path / "open-motion-console-bl-updater-1.8.97-signed.bin"
    app = tmp_path / "openmotion-console-fw-signed.bin"
    for p in (updater, app):
        p.write_bytes(b"\x00")

    images = _load_script().find_release_images(tmp_path, "console")

    assert images == {"app": app, "updater": updater}


def test_missing_roles_are_simply_absent(tmp_path):
    (tmp_path / "openmotion-console-fw-signed.bin").write_bytes(b"\x00")
    assert set(_load_script().find_release_images(tmp_path, "console")) == {"app"}


def test_two_candidates_for_one_role_are_refused(release_dir):
    """Two updaters in the folder: do not guess which one rewrites the bootloader."""
    (release_dir / "open-motion-console-bl-updater-1.8.99.bin").write_bytes(b"\x00")
    with pytest.raises(ValueError, match="2 console updater images"):
        _load_script().find_release_images(release_dir, "console")


# ---------------------------------------------------------------------------
# main(): what reaches FirmwareUpdater.update()
# ---------------------------------------------------------------------------

class _Handle:
    def is_connected(self):
        return True

    def get_version(self):
        return "1.8.1"

    def get_boot_mode(self):
        from omotion.boot_mode import BootMode
        return BootMode.BOOTLOADER


class _Interface:
    def __init__(self):
        self.console = self.left = self.right = _Handle()

    def start(self):
        pass

    def stop(self):
        pass


@pytest.fixture
def run(monkeypatch, release_dir):
    """Run main() with fakes; returns (exit_code, update() calls)."""
    mod = _load_script()
    calls = []

    class Updater:
        last_boot_mode = None
        last_bootloader_version = None

        def __init__(self, **kwargs):
            pass

        def update(self, handle, bin_path, progress_cb=None, **kwargs):
            calls.append((bin_path, kwargs))
            from omotion.boot_mode import BootMode
            from omotion.firmware_update import FirmwareUpdateResult, UpdateAction
            return FirmwareUpdateResult(
                success=True, action=UpdateAction.APP_ONLY, boot_mode=BootMode.BOOTLOADER,
                bootloader_version_before="1.2.0", bootloader_version_after="1.2.0",
                app_reenumerated=True, needs_power_cycle=False,
                message="application installed")

    monkeypatch.setattr(mod, "MotionInterface", _Interface)
    monkeypatch.setattr(mod, "FirmwareUpdater", Updater)
    monkeypatch.setattr(mod.STM32DFU, "list_devices", classmethod(lambda cls: []))
    monkeypatch.setattr(mod, "stlink_probes", lambda: 0)
    monkeypatch.setattr(mod.time, "sleep", lambda s: None)
    # main() lowers the root log level for the run; not in the test process.
    monkeypatch.setattr(mod, "_quiet_connection_warnings", lambda: None)

    def _run(*argv, probes=0, dfu_devices=()):
        monkeypatch.setattr(mod, "stlink_probes", lambda: probes)
        monkeypatch.setattr(mod.STM32DFU, "list_devices",
                            classmethod(lambda cls: list(dfu_devices)))
        monkeypatch.setattr(sys, "argv", ["update_firmware.py", *argv, "--no-confirm"])
        return mod.main(), calls

    _run.release_dir = release_dir
    return _run


def test_release_folder_run_passes_all_three_images(run):
    code, calls = run("right", "--release-dir", str(run.release_dir),
                      "--acknowledge-irreversible")

    assert code == 0
    (bin_path, kwargs), = calls
    assert bin_path == run.release_dir / "openmotion-sensor-fw-signed.bin"
    assert kwargs["updater_image"] == run.release_dir / "open-motion-sensor-bl-updater.bin"
    assert kwargs["production_image"] == run.release_dir / "openmotion-sensor-fw-production.bin"
    assert kwargs["acknowledge_irreversible"] is True
    assert kwargs["kind"].value == "sensor"


def test_without_the_flag_the_acknowledgement_is_false(run):
    """The images may be at hand; the permission is not given by default."""
    _, calls = run("console", "--release-dir", str(run.release_dir))
    assert calls[0][1]["acknowledge_irreversible"] is False
    assert calls[0][1]["kind"].value == "console"


def test_explicit_image_overrides_the_release_folder(run, tmp_path):
    other = tmp_path / "another-signed.bin"
    other.write_bytes(b"\x00")
    _, calls = run("console", "--release-dir", str(run.release_dir), "--app", str(other))
    assert calls[0][0] == other


def test_no_application_image_is_an_error(run, tmp_path, capsys):
    empty = tmp_path / "empty"
    empty.mkdir()
    code, calls = run("console", "--release-dir", str(empty))
    assert code == 1 and calls == []
    assert "[FAIL]" in capsys.readouterr().out


def test_stlink_on_usb_blocks_an_acknowledged_run(run, capsys):
    code, calls = run("console", "--release-dir", str(run.release_dir),
                      "--acknowledge-irreversible", probes=1)
    assert code == 1 and calls == []
    assert "ST-LINK" in capsys.readouterr().out


def test_stlink_on_usb_does_not_block_an_ordinary_update(run):
    code, calls = run("console", "--release-dir", str(run.release_dir), probes=1)
    assert code == 0 and len(calls) == 1


def test_stray_dfu_device_blocks_the_run(run, capsys):
    code, calls = run("console", "--release-dir", str(run.release_dir),
                      dfu_devices=[{"serial": "X"}])
    assert code == 1 and calls == []
    assert "DFU device is already on USB" in capsys.readouterr().out
