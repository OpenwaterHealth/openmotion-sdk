"""Tests for the Pipeline class — the stage-list driver."""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.pipeline import Pipeline, Stage


class _TagStage:
    name = "tag"
    def __init__(self, tag):
        self.tag = tag
        self.process_count = 0
        self.reset_count = 0
    def process(self, batch):
        batch.events.append(self.tag)
        self.process_count += 1
        return batch
    def reset(self):
        self.reset_count += 1


def _empty_batch():
    return FrameBatch(
        cam_ids=np.zeros(1, dtype=np.int8),
        frame_ids=np.zeros(1, dtype=np.uint8),
        raw_histograms=np.zeros((1, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((1, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(1, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def test_pipeline_runs_stages_in_order():
    stages = [_TagStage("a"), _TagStage("b"), _TagStage("c")]
    pipeline = Pipeline(stages)
    batch = pipeline.process(_empty_batch())
    assert batch.events == ["a", "b", "c"]


def test_pipeline_reset_calls_reset_on_every_stage():
    stages = [_TagStage("a"), _TagStage("b")]
    pipeline = Pipeline(stages)
    pipeline.reset()
    assert stages[0].reset_count == 1
    assert stages[1].reset_count == 1


def test_pipeline_process_returns_the_batch():
    pipeline = Pipeline([_TagStage("a")])
    batch = _empty_batch()
    result = pipeline.process(batch)
    assert result is batch
