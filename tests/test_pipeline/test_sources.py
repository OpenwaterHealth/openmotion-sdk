"""Source protocol — defines the FrameBatch iterator contract."""

import pytest
from omotion.pipeline.sources import Source, _BaseSource
from omotion.pipeline.sinks import ScanMetadata


def _meta():
    return ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        write_raw_csv=False, raw_csv_duration_sec=None,
    )


def test_source_protocol_is_runtime_checkable():
    class _Mock:
        metadata = _meta()
        def __iter__(self): yield from []
        def close(self): pass
    assert isinstance(_Mock(), Source)


def test_base_source_close_is_noop_by_default():
    class _Sub(_BaseSource):
        def __iter__(self): yield from []

    src = _Sub(metadata=_meta())
    src.close()
