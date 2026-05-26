"""Tests for MotionInterface's new SDK-level output config args."""

import pytest
from omotion.MotionInterface import MotionInterface


def test_motion_interface_accepts_data_dir():
    motion = MotionInterface(demo_mode=True, data_dir="C:/tmp/scans")
    assert motion.data_dir == "C:/tmp/scans"


def test_motion_interface_accepts_scan_db_path():
    motion = MotionInterface(demo_mode=True, scan_db_path="C:/tmp/scans/scans.db")
    assert motion.scan_db_path == "C:/tmp/scans/scans.db"


def test_motion_interface_accepts_operator_id():
    motion = MotionInterface(demo_mode=True, operator_id="bloodflow-app")
    assert motion.operator_id == "bloodflow-app"


def test_motion_interface_defaults_when_args_omitted():
    motion = MotionInterface(demo_mode=True)
    assert motion.data_dir is None
    assert motion.scan_db_path is None
    assert motion.operator_id is None


# ---------------------------------------------------------------------------
# Task 15: contact_quality_workflow lazy property
# ---------------------------------------------------------------------------

def test_motion_interface_lazy_loads_contact_quality_workflow():
    """contact_quality_workflow should be a ContactQualityWorkflow instance
    and should be cached (same object on repeated access)."""
    from omotion.ContactQualityWorkflow import ContactQualityWorkflow

    motion = MotionInterface(demo_mode=True)
    cq = motion.contact_quality_workflow
    assert isinstance(cq, ContactQualityWorkflow)
    # Second access returns the same cached instance.
    assert motion.contact_quality_workflow is cq


def test_motion_interface_cq_workflow_shares_scan_workflow():
    """The contact_quality_workflow must be wired to the same scan_workflow
    instance so the scan-running lock is shared."""
    motion = MotionInterface(demo_mode=True)
    cq = motion.contact_quality_workflow
    sw = motion.scan_workflow
    assert cq._scan_workflow is sw
