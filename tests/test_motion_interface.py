"""Facade-level tests for MotionInterface (the one SDK front door).

All run in demo mode — no hardware. Covers: output-config constructor args,
trigger-config resolution, lazy workflow wiring, state queries, and the
start/stop lifecycle.
"""

import pytest

from omotion.MotionInterface import MotionInterface


# ---------------------------------------------------------------------------
# Construction — SDK-level output config args
# ---------------------------------------------------------------------------

@pytest.mark.parametrize(
    "kwarg, value, attr",
    [
        ("data_dir", "C:/tmp/scans", "data_dir"),
        ("scan_db_path", "C:/tmp/scans/scans.db", "scan_db_path"),
        ("operator_id", "bloodflow-app", "operator_id"),
    ],
)
def test_constructor_stores_output_config(kwarg, value, attr):
    motion = MotionInterface(demo_mode=True, **{kwarg: value})
    assert getattr(motion, attr) == value


def test_output_config_defaults_to_none_when_omitted():
    motion = MotionInterface(demo_mode=True)
    assert motion.data_dir is None
    assert motion.scan_db_path is None
    assert motion.operator_id is None


# ---------------------------------------------------------------------------
# Trigger-config resolution
# ---------------------------------------------------------------------------

def test_resolve_trigger_config_returns_default_when_no_override():
    motion = MotionInterface(demo_mode=True)
    assert motion.resolve_trigger_config() == motion.default_trigger_config


def test_resolve_trigger_config_shallow_merges_override():
    """An override replaces only the keys it names; the rest fall through to
    the resolved default."""
    motion = MotionInterface(demo_mode=True)
    resolved = motion.resolve_trigger_config({"TriggerFrequencyHz": 20})
    assert resolved["TriggerFrequencyHz"] == 20
    # An untouched key still equals the default.
    assert (
        resolved["LaserPulseSkipInterval"]
        == motion.default_trigger_config["LaserPulseSkipInterval"]
    )


def test_default_trigger_config_is_a_defensive_copy():
    """Mutating the returned dict must not corrupt the interface's default."""
    motion = MotionInterface(demo_mode=True)
    snapshot = motion.default_trigger_config
    snapshot["TriggerFrequencyHz"] = -999
    assert motion.default_trigger_config["TriggerFrequencyHz"] != -999


# ---------------------------------------------------------------------------
# Lazy workflow wiring
# ---------------------------------------------------------------------------

def test_lazy_loads_contact_quality_workflow():
    """contact_quality_workflow is a ContactQualityWorkflow and is cached."""
    from omotion.ContactQualityWorkflow import ContactQualityWorkflow

    motion = MotionInterface(demo_mode=True)
    cq = motion.contact_quality_workflow
    assert isinstance(cq, ContactQualityWorkflow)
    assert motion.contact_quality_workflow is cq  # cached


@pytest.mark.parametrize("prop", ["scan_workflow", "calibration_workflow"])
def test_workflow_properties_are_cached(prop):
    """Repeated access to a workflow property returns the same instance."""
    motion = MotionInterface(demo_mode=True)
    first = getattr(motion, prop)
    assert first is not None
    assert getattr(motion, prop) is first


def test_cq_workflow_shares_scan_workflow():
    """contact_quality_workflow must be wired to the same scan_workflow so the
    scan-running lock is shared."""
    motion = MotionInterface(demo_mode=True)
    assert motion.contact_quality_workflow._scan_workflow is motion.scan_workflow


# ---------------------------------------------------------------------------
# State queries + lifecycle
# ---------------------------------------------------------------------------

def test_is_device_connected_returns_three_bools():
    motion = MotionInterface(demo_mode=True)
    state = motion.is_device_connected()
    assert isinstance(state, tuple) and len(state) == 3
    assert all(isinstance(b, bool) for b in state)


def test_get_sdk_version_returns_nonempty_string():
    version = MotionInterface.get_sdk_version()
    assert isinstance(version, str) and version


def test_start_stop_lifecycle_is_clean_in_demo_mode():
    """start() spins up the connection monitor and stop() tears it down with
    no exception; both are idempotent."""
    motion = MotionInterface(demo_mode=True)
    motion.start(wait=False)
    # is_device_connected must be queryable while running.
    assert len(motion.is_device_connected()) == 3
    motion.start(wait=False)   # idempotent — second start is a no-op
    motion.stop()
    motion.stop()              # idempotent — second stop is a no-op
