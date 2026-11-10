"""ContactQualityMonitor's verdict history and edge queue (bloodflow-app#589).

ContactQualityStage stamps each corrected frame with what the verdict *was*
at that frame (``verdict_at``), and publishes the edges it queued
(``take_transitions``).
"""

import numpy as np
import pytest

from omotion.contact_quality import (
    REASON_AMBIENT_LIGHT, REASON_OK, REASON_POOR_CONTACT,
    CQThresholds, ContactQualityMonitor,
)
from omotion.pipeline import quality_flags
from omotion.pipeline.batch import FrameBatch

THRESHOLDS = CQThresholds.from_sequences(dark=[10.0] * 8, light=[20.0] * 8)


class _FakeMeta:
    def __init__(self, left_camera_mask=0x01, right_camera_mask=0x01):
        self.left_camera_mask = left_camera_mask
        self.right_camera_mask = right_camera_mask


def _monitor(meta=None):
    mon = ContactQualityMonitor(
        thresholds=THRESHOLDS, on_transition=lambda *a: None,
        rolling_window=1, light_activate_debounce=1, light_clear_debounce=1,
    )
    mon.on_scan_start(meta or _FakeMeta())
    return mon


def _batch(side_idx, fids, values, *, cam_id=0, frame_types=None):
    """One camera on one side: a row per frame id, ``values`` in DN."""
    n = len(fids)
    arr = np.full((n, 2, 8), np.nan, dtype=np.float32)
    arr[:, side_idx, cam_id] = values
    return FrameBatch(
        cam_ids=np.full(n, cam_id, dtype=np.int8),
        frame_ids=np.array(fids, dtype=np.int64) & 0xFF,
        abs_frame_ids=np.array(fids, dtype=np.int64),
        side_ids=np.full(n, side_idx, dtype=np.int8),
        raw_histograms=None, temperature_c=None,
        timestamp_s=np.zeros(n), pdc=None, tcm=None, tcl=None,
        frame_type=np.array(frame_types or ["light"] * n, dtype="<U8"),
        subtracted_mean=arr, mean_dc_rt=arr.copy(),
    )


def test_reasons_are_quality_flags():
    """The verdict lands in the quality list under the monitor's own names."""
    assert REASON_POOR_CONTACT == quality_flags.POOR_CONTACT
    assert REASON_AMBIENT_LIGHT == quality_flags.AMBIENT_LIGHT
    assert REASON_OK == quality_flags.OK


def test_verdict_at_answers_as_of_each_frame():
    mon = _monitor()
    mon.consume("live", _batch(0, [10, 11, 12, 13, 14], [60, 60, 5, 5, 60]))
    assert [mon.verdict_at("left", 0, f) for f in range(9, 16)] == [
        "ok", "ok", "ok", "poor_contact", "poor_contact", "ok", "ok",
    ]


def test_verdict_before_any_edge_is_ok():
    mon = _monitor()
    assert mon.verdict_at("left", 0, 5) == "ok"
    mon.consume("live", _batch(0, [5], [60]))
    assert mon.verdict_at("left", 0, 5) == "ok"


def test_both_verdicts_at_once_are_one_flag_list():
    mon = _monitor()
    mon.consume("live", _batch(0, [20], [50], frame_types=["dark"]))   # ambient
    mon.consume("live", _batch(0, [21], [5]))                         # poor
    assert mon.verdict_at("left", 0, 20) == "ambient_light"
    assert mon.verdict_at("left", 0, 21) == "poor_contact;ambient_light"


def test_verdict_is_none_outside_the_scan_mask():
    mon = _monitor(_FakeMeta(left_camera_mask=0x01, right_camera_mask=0x00))
    assert mon.verdict_at("left", 0, 1) == "ok"
    assert mon.verdict_at("left", 1, 1) is None
    assert mon.verdict_at("right", 0, 1) is None


def test_on_scan_start_resets_the_history():
    mon = _monitor()
    mon.consume("live", _batch(0, [10], [5]))
    assert mon.verdict_at("left", 0, 10) == "poor_contact"
    mon.on_scan_start(_FakeMeta())
    assert mon.verdict_at("left", 0, 10) == "ok"
    assert mon.take_transitions() == []


@pytest.mark.parametrize("fid", [12, 13])
def test_edges_on_one_frame_keep_the_later_state(fid):
    """Raise and clear on the same frame id: the frame reads the final state."""
    mon = _monitor()
    mon.consume("live", _batch(0, [12], [5]))
    mon.consume("live", _batch(0, [12], [60]))   # re-delivered, recovered
    assert mon.verdict_at("left", 0, fid) == "ok"


def test_take_transitions_hands_out_each_edge_once():
    mon = _monitor()
    mon.consume("live", _batch(0, [10, 11, 12], [60, 5, 60]))
    edges = mon.take_transitions()
    assert [(t.reason, t.active, t.abs_frame_id) for t in edges] == [
        ("poor_contact", True, 11), ("poor_contact", False, 12),
    ]
    assert mon.take_transitions() == []
