"""Tests asserting that MotionProcessing.py is the post-Phase-F shim:
parsing helpers + dataclasses, no SciencePipeline class."""


def test_motion_processing_retains_parsing_helpers():
    from omotion.MotionProcessing import (
        parse_histogram_stream,
        parse_histogram_packet_structured,
        _rle_decompress,
        _util_crc16,
        EXPECTED_HISTOGRAM_SUM,
        HISTOGRAM_BYTES,
        Sample,
        CorrectedBatch,
        PEDESTAL_HEIGHT,
    )
    assert callable(parse_histogram_stream)
    assert callable(parse_histogram_packet_structured)
    assert EXPECTED_HISTOGRAM_SUM == 2_457_606


def test_motion_processing_removes_science_pipeline_class():
    import omotion.MotionProcessing as mp
    assert not hasattr(mp, "SciencePipeline")
    assert not hasattr(mp, "create_science_pipeline")
    assert not hasattr(mp, "FrameIdUnwrapper")


def test_motion_processing_removes_legacy_emit_functions():
    import omotion.MotionProcessing as mp
    assert not hasattr(mp, "compute_realtime_metrics")
    assert not hasattr(mp, "_emit_realtime_corrected")
    assert not hasattr(mp, "_emit_corrected_for_camera")
    assert not hasattr(mp, "_calibrate_bfi_bvi")
    assert not hasattr(mp, "_flush_terminal_dark")
    assert not hasattr(mp, "_check_dark_integrity")
