"""What FirmwareUpdater.update() does in each state a unit can be found in.

    bootloader 1.2.0 or newer   signed application only
    older bootloader            legacy-key application; a current application
                                needs the updater and the acknowledgement
    no bootloader (ROM loader)  bare-metal application; the production image
                                needs the acknowledgement

Installing or replacing a bootloader is irreversible over USB. The bloodflow
app must not be able to do it by accident, and neither must an ordinary update:
without ``acknowledge_irreversible=True`` nothing below the application slot is
ever written on a bootloader unit, and no updater or production image is sent.

Needs no hardware.
"""

import pytest

from _dfu_fakes import FakeDfuUnit, FakeHandle, TestKeys, make_production
from omotion import firmware_update
from omotion.boot_mode import BootMode
from omotion.dfu.progress import DFUProgress
from omotion.firmware_update import (
    BootloaderChangeRequired,
    FirmwareKind,
    FirmwareUpdateError,
    FirmwareUpdater,
    ImageRejectedError,
    UpdateAction,
    UpdateInterruptedError,
    UpdaterRefusedError,
)

pytestmark = pytest.mark.unit

BASE = 0x08000000
SLOT = 0x08020000
BARE_METAL_IMAGE = b"\x00\x00\x08\x24\x01\x04\x00\x08" + b"\xa5" * 56
SENSOR = FirmwareKind.SENSOR


@pytest.fixture(scope="module")
def keys():
    return TestKeys()


@pytest.fixture(autouse=True)
def _test_keys(monkeypatch, keys):
    monkeypatch.setattr(firmware_update, "public_key_pem", keys.pem)


@pytest.fixture
def images(tmp_path, keys):
    """The files of a release, plus the updater: name -> path."""
    signed = keys.signed("sensor", fw_version=11300)
    production = make_production(signed)
    bootloader = production[:0x20000].rstrip(b"\xff")
    contents = {
        "signed": signed,
        "legacy_signed": keys.signed("sensor", legacy=True, fw_version=10801),
        "console_signed": keys.signed("console", fw_version=11300),
        # The updater embeds the new bootloader and is signed with the OLD key.
        "updater": keys.signed("sensor", legacy=True, fw_version=10898,
                               body=b"\x00" * 64 + bootloader + b"\x00" * 64),
        "production": production,
        "baremetal": BARE_METAL_IMAGE,
    }
    paths = {}
    for name, data in contents.items():
        paths[name] = tmp_path / f"{name}.bin"
        paths[name].write_bytes(data)
    return paths


def _updater(unit):
    updater = FirmwareUpdater(dfu=unit, app_present=unit.app_present)
    updater.app_wait_timeout_s = 0
    updater.power_cycle_timeout_s = 0
    return updater


def _update(unit, path, **kwargs):
    kwargs.setdefault("kind", SENSOR)
    return _updater(unit).update(FakeHandle(unit), path, **kwargs)


def _everything(images):
    """All the irreversible-path arguments, acknowledged."""
    return dict(updater_image=images["updater"], production_image=images["production"],
                acknowledge_irreversible=True)


# ---------------------------------------------------------------------------
# Bootloader 1.2.0 or newer
# ---------------------------------------------------------------------------

def test_current_bootloader_gets_the_application_only(images):
    unit = FakeDfuUnit(version="1.2.0")

    result = _update(unit, images["signed"])

    assert unit.calls == [("download", SLOT, images["signed"].read_bytes(), False)]
    assert result.success and result.action is UpdateAction.APP_ONLY
    assert result.boot_mode is BootMode.BOOTLOADER
    assert (result.bootloader_version_before, result.bootloader_version_after) == ("1.2.0", "1.2.0")
    assert result.app_reenumerated and not result.needs_power_cycle


def test_release_candidate_bootloader_counts_as_current(images):
    unit = FakeDfuUnit(version="1.2.0-rc.1")
    assert _update(unit, images["signed"]).action is UpdateAction.APP_ONLY


def test_acknowledgement_is_a_permission_not_a_command(images):
    """A unit already on 1.2.0 is not touched below the slot, whatever was
    passed along."""
    unit = FakeDfuUnit(version="1.2.0")

    result = _update(unit, images["signed"], **_everything(images))

    assert result.action is UpdateAction.APP_ONLY
    assert unit.calls == [("download", SLOT, images["signed"].read_bytes(), False)]


def test_downgrade_is_refused_before_the_erase(images):
    unit = FakeDfuUnit(version="1.2.0", installed_fw_version=11400)

    with pytest.raises(ImageRejectedError, match="downgrade"):
        _update(unit, images["signed"])
    assert unit.downloads == []


def test_refusal_before_any_write_restarts_the_installed_application(images):
    unit = FakeDfuUnit(version="1.2.0", installed_fw_version=11400)

    with pytest.raises(ImageRejectedError):
        _update(unit, images["signed"])
    assert unit.calls == [("leave",)]
    assert unit.app_running


def test_image_for_the_other_product_is_refused_and_named(images):
    unit = FakeDfuUnit(version="1.2.0")

    with pytest.raises(ImageRejectedError, match="console bootloader 1.2.0 or newer"):
        _update(unit, images["console_signed"])
    assert unit.downloads == []


def test_legacy_key_image_is_refused_by_a_current_bootloader(images):
    unit = FakeDfuUnit(version="1.2.0")

    with pytest.raises(ImageRejectedError, match="older than 1.2.0"):
        _update(unit, images["legacy_signed"])
    assert unit.downloads == []


def test_unsigned_image_is_refused_by_a_bootloader_unit(tmp_path):
    """Whatever the file is called, a slot image has to carry a signed header."""
    unsigned = tmp_path / "custom-build.bin"
    unsigned.write_bytes(BARE_METAL_IMAGE * 32)
    unit = FakeDfuUnit(version="1.2.0")

    with pytest.raises(ImageRejectedError, match="SFU1"):
        _update(unit, unsigned)
    assert unit.downloads == []


def test_download_refused_by_the_bootloader_is_an_image_rejection(images):
    unit = FakeDfuUnit(version="1.2.0", refuse_download=True)

    with pytest.raises(ImageRejectedError, match="errVENDOR"):
        _update(unit, images["signed"])


def test_application_that_never_starts_is_a_failure(images):
    unit = FakeDfuUnit(version="1.2.0", app_boots=False)

    with pytest.raises(FirmwareUpdateError, match="still in DFU"):
        _update(unit, images["signed"])


def test_unparseable_bootloader_version_refuses_to_guess(images):
    unit = FakeDfuUnit(version="fd1546a-dirty")

    with pytest.raises(FirmwareUpdateError, match="fd1546a-dirty"):
        _update(unit, images["signed"], **_everything(images))
    assert unit.calls == []


# ---------------------------------------------------------------------------
# Bootloader older than 1.2.0
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("version", ["1.0.0", "1.1.0"])
def test_old_bootloader_still_takes_a_legacy_key_application(images, version):
    unit = FakeDfuUnit(version=version)

    result = _update(unit, images["legacy_signed"])

    assert unit.calls == [("download", SLOT, images["legacy_signed"].read_bytes(), False)]
    assert result.action is UpdateAction.APP_ONLY


@pytest.mark.parametrize("extra, needs", [
    ({}, ("updater_image", "acknowledge_irreversible")),
    ({"acknowledge_irreversible": True}, ("updater_image",)),
    ({"updater_image": "updater"}, ("acknowledge_irreversible",)),
])
def test_current_application_on_an_old_bootloader_says_what_is_needed(images, extra, needs):
    unit = FakeDfuUnit(version="1.0.0")
    if "updater_image" in extra:
        extra = {"updater_image": images["updater"]}

    with pytest.raises(BootloaderChangeRequired) as exc:
        _update(unit, images["signed"], **extra)

    assert exc.value.needs == needs
    assert exc.value.boot_mode is BootMode.BOOTLOADER
    assert exc.value.bootloader_version == "1.0.0"
    assert unit.calls == []                     # nothing erased, nothing written


def test_acknowledged_upgrade_runs_the_updater_then_installs_the_application(images):
    updater_bytes = images["updater"].read_bytes()
    unit = FakeDfuUnit(version="1.0.0", updater=updater_bytes)

    result = _update(unit, images["signed"], updater_image=images["updater"],
                     acknowledge_irreversible=True)

    assert unit.calls == [
        ("erase_all",),
        ("download", SLOT, updater_bytes, True),
        ("download", SLOT, images["signed"].read_bytes(), False),
    ]
    assert result.action is UpdateAction.BOOTLOADER_UPGRADED
    assert (result.bootloader_version_before, result.bootloader_version_after) == ("1.0.0", "1.2.0")


def test_updater_refused_by_the_old_bootloader_is_reported(images):
    unit = FakeDfuUnit(version="1.0.0", updater=images["updater"].read_bytes(),
                       updater_outcome="refused")

    with pytest.raises(UpdaterRefusedError, match="came back as an application"):
        _update(unit, images["signed"], updater_image=images["updater"],
                acknowledge_irreversible=True)
    assert len(unit.downloads) == 1             # the application was not sent


def test_updater_that_leaves_the_old_bootloader_in_place_is_reported(images):
    unit = FakeDfuUnit(version="1.0.0", updater=images["updater"].read_bytes(),
                       updater_outcome="unchanged")

    with pytest.raises(UpdaterRefusedError, match="1.0.0"):
        _update(unit, images["signed"], updater_image=images["updater"],
                acknowledge_irreversible=True)
    assert len(unit.downloads) == 1


def test_unit_that_never_returns_after_the_updater_needs_the_bench(images):
    unit = FakeDfuUnit(version="1.0.0", updater=images["updater"].read_bytes(),
                       updater_outcome="dead")

    with pytest.raises(FirmwareUpdateError, match="SWD"):
        _update(unit, images["signed"], updater_image=images["updater"],
                acknowledge_irreversible=True)


def test_updater_not_signed_for_the_old_bootloader_is_refused_before_dfu(images):
    """The old bootloader would refuse it, after the slot had been erased."""
    unit = FakeDfuUnit(version="1.0.0")
    handle = FakeHandle(unit)

    with pytest.raises(ImageRejectedError, match="updater image"):
        _updater(unit).update(handle, images["signed"], kind=SENSOR,
                              updater_image=images["signed"],
                              acknowledge_irreversible=True)
    assert handle.enter_dfu_calls == 0
    assert unit.calls == []


# ---------------------------------------------------------------------------
# No bootloader (STM32 ROM loader)
# ---------------------------------------------------------------------------

def test_bare_metal_unit_gets_the_bare_metal_application(images):
    unit = FakeDfuUnit(mode="rom")

    result = _update(unit, images["baremetal"])

    assert unit.calls == [("set_rom_mode",), ("download", BASE, BARE_METAL_IMAGE, False),
                          ("verify", BASE, BARE_METAL_IMAGE), ("leave",)]
    assert result.action is UpdateAction.BARE_METAL_APP
    assert result.bootloader_version_before is None


def test_dfu_file_suffix_is_not_written_to_flash(tmp_path):
    suffixed = tmp_path / "fw.bin"
    suffixed.write_bytes(BARE_METAL_IMAGE + b"\xff" * 8 + b"UFD" + b"\x10" + b"\x00" * 4)
    unit = FakeDfuUnit(mode="rom")

    _update(unit, suffixed)

    assert unit.downloads == [(BASE, BARE_METAL_IMAGE)]


@pytest.mark.parametrize("extra, needs", [
    ({}, ("production_image", "acknowledge_irreversible")),
    ({"acknowledge_irreversible": True}, ("production_image",)),
    ({"production_image": "production"}, ("acknowledge_irreversible",)),
])
def test_signed_application_on_a_bare_metal_unit_says_what_is_needed(images, extra, needs):
    unit = FakeDfuUnit(mode="rom")
    if "production_image" in extra:
        extra = {"production_image": images["production"]}

    with pytest.raises(BootloaderChangeRequired) as exc:
        _update(unit, images["signed"], **extra)

    assert exc.value.needs == needs
    assert exc.value.boot_mode is BootMode.BARE_METAL
    assert unit.downloads == []


def test_acknowledged_install_writes_the_production_image(images):
    unit = FakeDfuUnit(mode="rom")

    result = _update(unit, images["signed"], production_image=images["production"],
                     acknowledge_irreversible=True)

    assert unit.calls == [
        ("set_rom_mode",),
        ("erase_range", BASE, 0x080C0000),
        ("download", BASE, images["production"].read_bytes(), True),
        ("leave",),
    ]
    assert result.action is UpdateAction.BOOTLOADER_INSTALLED


def test_production_image_is_not_written_without_the_acknowledgement(images):
    """An ordinary bare-metal update with a production image lying around."""
    unit = FakeDfuUnit(mode="rom")

    result = _update(unit, images["baremetal"], production_image=images["production"])

    assert result.action is UpdateAction.BARE_METAL_APP
    assert unit.downloads == [(BASE, BARE_METAL_IMAGE)]


def test_production_image_for_another_application_is_refused_before_dfu(images, tmp_path, keys):
    other = tmp_path / "other-production.bin"
    other.write_bytes(make_production(keys.signed("sensor", body=b"\x77" * 512)))
    unit = FakeDfuUnit(mode="rom")
    handle = FakeHandle(unit)

    with pytest.raises(FirmwareUpdateError, match="signed application"):
        _updater(unit).update(handle, images["signed"], kind=SENSOR,
                              production_image=other, acknowledge_irreversible=True)
    assert handle.enter_dfu_calls == 0


def test_unit_that_leaves_usb_after_a_rom_flash_needs_a_power_cycle(images):
    """The STM32 ROM loader's jump into a new image hangs on sensor modules."""
    unit = FakeDfuUnit(mode="rom", app_boots=False)
    seen = []

    result = _updater(unit).update(FakeHandle(unit), images["baremetal"], seen.append,
                                   kind=SENSOR)

    assert result.success and result.needs_power_cycle and not result.app_reenumerated
    assert "power_cycle" in [p.phase for p in seen]


def test_unit_power_cycled_in_time_is_a_normal_success(images):
    unit = FakeDfuUnit(mode="rom", app_boots=False)
    answers = iter([False, True])               # absent, then back after the power cycle
    updater = FirmwareUpdater(dfu=unit, app_present=lambda kind, handle: next(answers))
    updater.app_wait_timeout_s = updater.power_cycle_timeout_s = 0

    result = updater.update(FakeHandle(unit), images["baremetal"], kind=SENSOR)

    assert result.app_reenumerated and not result.needs_power_cycle


# ---------------------------------------------------------------------------
# A bare-metal write that fails (sdk#333)
#
# The first erase destroys the image in sector 0, and a bare-metal unit has no
# other way back into DFU. Starting the image before it is known to be whole,
# or giving up in a way that invites a power cycle, bricks the unit.
# ---------------------------------------------------------------------------

def _rom_unit(**kwargs):
    kwargs.setdefault("serial", "ROM-UID")
    return FakeDfuUnit(mode="rom", **kwargs)


def _steps(unit):
    return [c[0] for c in unit.calls if c[0] != "set_rom_mode"]


def _quick(unit):
    updater = _updater(unit)
    updater.rom_reattach_timeout_s = 0
    return updater


def test_the_image_is_read_back_before_the_unit_leaves_dfu(images):
    unit = _rom_unit()

    _update(unit, images["baremetal"])

    assert _steps(unit) == ["download", "verify", "leave"]


def test_a_failed_write_is_redone_while_the_unit_is_in_the_rom_loader(images):
    unit = _rom_unit(fail_downloads=1)
    seen = []

    result = _quick(unit).update(FakeHandle(unit), images["baremetal"], seen.append,
                                 kind=SENSOR)

    assert result.success and result.action is UpdateAction.BARE_METAL_APP
    assert _steps(unit) == ["download", "download", "verify", "leave"]
    assert unit.downloads == [(BASE, BARE_METAL_IMAGE)] * 2
    assert unit.connects == 2, "the retry reopens the ROM loader"
    assert any(p.phase == "retry" and "Retrying (2/3)" in p.message for p in seen)


def test_a_corrupt_read_back_is_redone_and_never_started(images):
    unit = _rom_unit(corrupt_reads=1)

    result = _update(unit, images["baremetal"])

    assert result.success
    assert _steps(unit) == ["download", "verify", "download", "verify", "leave"]


@pytest.mark.parametrize("failure", [{"fail_downloads": 99}, {"corrupt_reads": 99}])
def test_a_write_that_keeps_failing_is_reported_as_interrupted(images, failure):
    unit = _rom_unit(**failure)

    with pytest.raises(UpdateInterruptedError) as exc:
        _quick(unit).update(FakeHandle(unit), images["baremetal"], kind=SENSOR)

    e = exc.value
    assert isinstance(e, FirmwareUpdateError)
    assert "leave" not in _steps(unit), "a half-written image must never be started"
    assert unit.in_dfu, "the unit is left in the ROM loader"
    assert _steps(unit).count("download") == 3
    assert e.in_dfu and e.attempts == 3
    assert e.image_path == images["baremetal"] and e.kind is SENSOR
    assert e.dfu_serial == "ROM-UID"
    assert "do NOT power it off" in str(e)


def test_a_unit_that_drops_off_usb_is_not_retried_blind(images):
    unit = _rom_unit(fail_downloads=1, drops_off_usb=True)

    with pytest.raises(UpdateInterruptedError) as exc:
        _quick(unit).update(FakeHandle(unit), images["baremetal"], kind=SENSOR)

    assert not exc.value.in_dfu and exc.value.attempts == 1
    assert _steps(unit) == ["download"]
    assert "reconnect its USB cable" in str(exc.value)


def test_the_retry_count_is_the_updaters_to_set(images):
    unit = _rom_unit(fail_downloads=99)
    updater = _quick(unit)
    updater.rom_write_attempts = 1

    with pytest.raises(UpdateInterruptedError) as exc:
        updater.update(FakeHandle(unit), images["baremetal"], kind=SENSOR)

    assert exc.value.attempts == 1 and _steps(unit) == ["download"]


def test_an_interrupted_update_resumes_from_dfu(images):
    """The documented retry: update(None, ...) on the unit still in the ROM loader."""
    unit = _rom_unit(fail_downloads=3)
    with pytest.raises(UpdateInterruptedError) as exc:
        _quick(unit).update(FakeHandle(unit), images["baremetal"], kind=SENSOR)
    e = exc.value
    unit.calls.clear()

    result = _quick(unit).update(None, e.image_path, kind=e.kind, dfu_serial=e.dfu_serial)

    assert result.success and result.action is UpdateAction.BARE_METAL_APP
    assert _steps(unit) == ["download", "verify", "leave"]
    assert unit.downloads == [(BASE, BARE_METAL_IMAGE)]


def test_the_production_image_path_is_unchanged(images):
    """Its step order is the bench-tested one: no read-back, no retry."""
    unit = _rom_unit(fail_downloads=1)

    with pytest.raises(FirmwareUpdateError) as exc:
        _quick(unit).update(FakeHandle(unit), images["signed"], kind=SENSOR,
                            production_image=images["production"],
                            acknowledge_irreversible=True)

    assert not isinstance(exc.value, UpdateInterruptedError)
    assert _steps(unit) == ["erase_range", "download"]


# ---------------------------------------------------------------------------
# The acknowledgement
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("mode, version", [("bootloader", "1.0.0"), ("bootloader", "1.1.0"),
                                           ("bootloader", "1.2.0")])
@pytest.mark.parametrize("image", ["signed", "legacy_signed", "baremetal", "production"])
def test_without_acknowledgement_a_bootloader_unit_is_never_written_below_its_slot(
        images, mode, version, image):
    unit = FakeDfuUnit(mode=mode, version=version, updater=images["updater"].read_bytes())
    try:
        _update(unit, images[image], updater_image=images["updater"],
                production_image=images["production"])
    except FirmwareUpdateError:
        pass

    assert all(address >= SLOT for address, _ in unit.downloads)
    assert images["updater"].read_bytes() not in [data for _, data in unit.downloads]
    assert ("erase_all",) not in unit.calls


@pytest.mark.parametrize("value", [1, "yes", "True", None])
def test_acknowledgement_must_be_exactly_true(images, value):
    unit = FakeDfuUnit(version="1.0.0")
    handle = FakeHandle(unit)

    with pytest.raises(FirmwareUpdateError, match="acknowledge_irreversible"):
        _updater(unit).update(handle, images["signed"], kind=SENSOR,
                              updater_image=images["updater"],
                              acknowledge_irreversible=value)
    assert handle.enter_dfu_calls == 0


def test_no_separate_bootloader_installer_on_the_package_surface():
    """There is one way in, and it asks for the acknowledgement."""
    import omotion

    assert not hasattr(omotion, "install_bootloader")
    assert "install_bootloader" not in omotion.__all__


# ---------------------------------------------------------------------------
# Entering DFU, progress
# ---------------------------------------------------------------------------

def test_unit_already_in_dfu_is_resumed_without_a_handle(images):
    """After an interrupted run there is no application to ask."""
    unit = FakeDfuUnit(version="1.2.0", in_dfu=True)

    result = _updater(unit).update(None, images["signed"], kind=SENSOR)

    assert result.action is UpdateAction.APP_ONLY


def test_stray_dfu_device_makes_the_target_ambiguous(images):
    unit = FakeDfuUnit(version="1.2.0", in_dfu=True)
    handle = FakeHandle(unit)

    with pytest.raises(FirmwareUpdateError, match="already attached"):
        _updater(unit).update(handle, images["signed"], kind=SENSOR)
    assert handle.enter_dfu_calls == 0


def test_no_handle_and_no_dfu_device_is_an_error(images):
    unit = FakeDfuUnit(version="1.2.0")
    with pytest.raises(FirmwareUpdateError):
        _updater(unit).update(None, images["signed"], kind=SENSOR)


def test_progress_reports_erase_download_and_wait(images):
    unit = FakeDfuUnit(version="1.0.0", updater=images["updater"].read_bytes())
    seen = []

    _updater(unit).update(FakeHandle(unit), images["signed"], seen.append, kind=SENSOR,
                          updater_image=images["updater"], acknowledge_irreversible=True)

    assert all(isinstance(p, DFUProgress) for p in seen)
    assert {"erase", "download", "wait"} <= {p.phase for p in seen}
    assert any(p.phase == "download" and p.percent == 100 for p in seen)
    assert any("DO NOT POWER OFF" in p.message for p in seen)
    # One clock for the whole update, not one per flash step.
    elapsed = [p.elapsed_s for p in seen]
    assert elapsed == sorted(elapsed)


# ---------------------------------------------------------------------------
# "Install the bootloader" with nothing but the production image
# ---------------------------------------------------------------------------

def test_production_image_alone_installs_on_a_bare_metal_unit(images, tmp_path):
    production = tmp_path / "motion-sensor-production.bin"
    production.write_bytes(images["production"].read_bytes())
    unit = FakeDfuUnit(mode="rom")

    result = _update(unit, production, production_image=production,
                     acknowledge_irreversible=True)

    assert result.action is UpdateAction.BOOTLOADER_INSTALLED
    assert unit.downloads == [(BASE, production.read_bytes())]


@pytest.mark.parametrize("version", ["1.0.0", "1.2.0"])
def test_production_image_alone_writes_nothing_if_a_bootloader_is_present(
        images, tmp_path, version):
    """The guarantee the test app's Install Bootloader button rests on: the
    boot mode is only certain once the unit is in DFU, so the refusal is here."""
    production = tmp_path / "motion-sensor-production.bin"
    production.write_bytes(images["production"].read_bytes())
    unit = FakeDfuUnit(version=version)

    with pytest.raises(FirmwareUpdateError, match="already has a bootloader"):
        _update(unit, production, production_image=production,
                acknowledge_irreversible=True)
    assert unit.downloads == []
