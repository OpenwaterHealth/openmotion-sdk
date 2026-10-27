"""Etch-a-sketch regression suite (openmotion-sdk#220 / sensor-fw#123).

The 2026-08-06 field event corrupted one camera's frame_id byte in
flight (top two bits cleared, raw 0xC5 -> 0x05) while the packet
timestamps stayed truthful. The pipeline used to treat the frame_id as
ground truth and amplify the one-byte lie: 64 real frames discarded,
~60 rows fabricated, good timestamps rewritten ~1.6 s into the future,
and the app's live trace drawing zigzags.

This suite asserts the repaired contract, aligned with the sensor-fw
HIL fault-injection design (sensor-fw docs/superpowers/specs/
2026-08-11-histogram-fault-injection-design.md). One scenario per
fault mode, each pushed through the real classify -> repair ->
side-average chain:

  fid_single       -> FrameIdConsensusCorrection: packet-mate consensus
                      repairs the corrupted id BEFORE unwrapping; the
                      frame's histogram data is preserved (zero loss).
  fid_multi        -> FrameIdPacketAnomaly: a tie cannot be adjudicated;
                      the per-camera counter-vs-clock check quarantines
                      exactly the inconsistent frames.
  timestamp_freeze -> TimestampRepairInputAnomaly + re-timestamping.
  packet_drop      -> FrameGapFillAnomaly + honest nan_filled
                      placeholders when the next frame arrives.

plus a sustained-corruption scenario (independent per-camera hits all
scan long) proving the historic warning-flood/data-shredding
presentation is gone: consensus repairs the single-camera hits,
ambiguous packets degrade to bounded per-frame loss, and no
misalignment windows open at all.
"""

import random

import numpy as np
import pytest

from omotion.pipeline.batch import (
    FrameBatch, FrameGapFillAnomaly, FrameIdConsensusCorrection,
    FrameIdPacketAnomaly, LiveEmit, TimestampMisalignmentWindow,
    TimestampRepairInputAnomaly,
)
from omotion.pipeline.stages.classify import FrameClassificationStage
from omotion.pipeline.stages.side_avg import SideAverageStage
from omotion.pipeline.stages.timestamp_repair import TimestampRepairStage

SIDE = 1                      # right, as in the incident
CAMS = (0, 1, 2, 3)
VICTIM = 3                    # last in packet order, like the field event
PERIOD_S = 0.025              # 40 Hz
N_CAPTURES = 320              # 8 s scan
BURST_CAPTURES = (198, 199, 200)   # raw 0xC6..0xC8 -> corrupted 0x06..0x08
CAPTURES_PER_BATCH = 10
WARMUP = 9                    # classification discard_count

# Distinct per-camera BFI levels so a partial average is numerically
# distinguishable from the true 4-camera average.
CAM_BFI = {0: 1.0, 1: 2.0, 2: 3.0, 3: 6.0}
FULL_AVG = sum(CAM_BFI.values()) / len(CAM_BFI)          # 3.0


class _StubBfiStage:
    """Stand-in for the moments->dark->shot-noise->BFI chain.

    Real light/dark rows get a finite per-camera BFI; stale rows and
    nan_filled rows get NaN (in the real pipeline stale rows are skipped
    by DarkCorrectionStage and nan_filled rows carry zero histograms).
    """

    name = "stub_bfi"

    def process(self, batch: FrameBatch) -> FrameBatch:
        n = len(batch.cam_ids)
        bfi = np.full((n, 2, 8), np.nan, dtype=np.float32)
        for i in range(n):
            ftype = str(batch.frame_type[i])
            quality = (str(batch.quality[i])
                       if batch.quality is not None else "ok")
            if ftype in ("light", "dark") and quality != "nan_filled":
                s = int(batch.side_ids[i])
                c = int(batch.cam_ids[i])
                bfi[i, s, c] = CAM_BFI[c]
        batch.bfi_live = bfi
        batch.bvi_live = bfi.copy()
        return batch


def _clean_rows():
    """The uncorrupted wire stream: 4 cameras per capture sharing one
    packet timestamp, raw ids starting at 1."""
    return [(cam, capture & 0xFF, capture * PERIOD_S)
            for capture in range(1, N_CAPTURES + 1) for cam in CAMS]


def _fid_single_rows():
    """fid_single: the victim's frame_id byte loses its top two bits for
    3 consecutive captures; timestamps stay truthful."""
    rows = []
    for capture in range(1, N_CAPTURES + 1):
        t = capture * PERIOD_S
        for cam in CAMS:
            raw = capture & 0xFF
            if cam == VICTIM and capture in BURST_CAPTURES:
                raw &= 0x3F
            rows.append((cam, raw, t))
    return rows


def _fid_multi_rows():
    """fid_multi: TWO cameras take the same mutation in the same packets
    — a 2-2 tie no consensus can adjudicate."""
    rows = []
    for capture in range(1, N_CAPTURES + 1):
        t = capture * PERIOD_S
        for cam in CAMS:
            raw = capture & 0xFF
            if cam in (2, 3) and capture in BURST_CAPTURES:
                raw &= 0x3F
            rows.append((cam, raw, t))
    return rows


FREEZE_BASE = 141                    # honest capture whose ts gets reused
FREEZE_CAPTURES = (142, 143, 144)    # carry FREEZE_BASE's timestamp


def _timestamp_freeze_rows():
    """timestamp_freeze: three packets reuse the preceding packet's
    timestamp while the frame counter keeps advancing."""
    rows = []
    for capture in range(1, N_CAPTURES + 1):
        t = (FREEZE_BASE if capture in FREEZE_CAPTURES else capture) * PERIOD_S
        rows.extend((cam, capture & 0xFF, t) for cam in CAMS)
    return rows


DROPPED_CAPTURE = 200


def _packet_drop_rows():
    """packet_drop: one complete packet (all cameras) never arrives."""
    return [(cam, capture & 0xFF, capture * PERIOD_S)
            for capture in range(1, N_CAPTURES + 1)
            if capture != DROPPED_CAPTURE
            for cam in CAMS]


def _sustained_rows(p_corrupt: float = 0.10, seed: int = 56):
    """Sustained in-flight corruption: every camera has an independent
    per-frame chance of the top-two-bits mutation, all scan long.

    Also computes the expected outcome per the consensus contract, from
    the number of cameras k whose byte actually changed in each packet:
      k == 1 -> consensus repairs it        (+1 correction)
      k == 2 -> 2-2 tie                     (+1 anomaly, +2 quarantined)
      k == 3 -> the corrupt value IS the majority; consensus points the
                honest camera at it, the counter-vs-clock check rejects
                everything                  (+4 quarantined, no anomaly)
      k == 4 -> all four agree on the corrupt value — no disagreement to
                see; per-camera checks reject them (+4 quarantined)
    """
    rng = random.Random(seed)
    rows = []
    exp = {"corrections": 0, "anomalies": 0, "stale": 0}
    for capture in range(1, N_CAPTURES + 1):
        t = capture * PERIOD_S
        changed = 0
        for cam in CAMS:
            raw = capture & 0xFF
            if rng.random() < p_corrupt:
                corrupted = raw & 0x3F
                if corrupted != raw:
                    changed += 1
                raw = corrupted
            rows.append((cam, raw, t))
        if changed == 1:
            exp["corrections"] += 1
        elif changed == 2:
            exp["anomalies"] += 1
            exp["stale"] += 2
        elif changed >= 3:
            exp["stale"] += 4
    return rows, exp


def _run_chain(rows):
    """Push wire rows through the real classify -> repair -> side-average
    chain in LiveUsbSource-sized batches; collect every output row, the
    live_side samples, and the diagnostics events by type."""
    classify = FrameClassificationStage(discard_count=WARMUP, dark_interval=600)
    repair = TimestampRepairStage()
    stub = _StubBfiStage()
    side_avg = SideAverageStage(
        enabled=True, left_camera_mask=0x00, right_camera_mask=0x0F)

    rows_per_batch = CAPTURES_PER_BATCH * len(CAMS)
    out_rows = []       # (cam, abs_fid, ts, frame_type, quality)
    samples = []        # SideAverageSample in emission order
    events = {"windows": [], "corrections": [], "anomalies": [],
              "frozen": [], "gap_fills": []}

    def collect(batch_events):
        for ev in batch_events:
            if isinstance(ev, LiveEmit) and ev.channel == "live_side":
                samples.append(ev.payload)
            elif isinstance(ev, TimestampMisalignmentWindow):
                events["windows"].append(ev)
            elif isinstance(ev, FrameIdConsensusCorrection):
                events["corrections"].append(ev)
            elif isinstance(ev, FrameIdPacketAnomaly):
                events["anomalies"].append(ev)
            elif isinstance(ev, TimestampRepairInputAnomaly):
                events["frozen"].append(ev)
            elif isinstance(ev, FrameGapFillAnomaly):
                events["gap_fills"].append(ev)

    for start in range(0, len(rows), rows_per_batch):
        chunk = rows[start:start + rows_per_batch]
        n = len(chunk)
        batch = FrameBatch(
            cam_ids=np.array([r[0] for r in chunk], dtype=np.int8),
            frame_ids=np.array([r[1] for r in chunk], dtype=np.uint8),
            packet_ids=np.array(
                [(start + i) // len(CAMS) for i in range(n)],
                dtype=np.int64,
            ),
            side_ids=np.full(n, SIDE, dtype=np.int8),
            raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
            temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
            timestamp_s=np.array([r[2] for r in chunk], dtype=np.float64),
            pdc=None, tcm=None, tcl=None,
        )
        batch = classify.process(batch)
        batch = repair.process(batch)
        batch = stub.process(batch)
        batch = side_avg.process(batch)

        for i in range(len(batch.cam_ids)):
            out_rows.append((
                int(batch.cam_ids[i]),
                int(batch.abs_frame_ids[i]),
                float(batch.timestamp_s[i]),
                str(batch.frame_type[i]),
                str(batch.quality[i]),
            ))
        collect(batch.events)

    # Frozen-clock evidence is deliberately coalesced across the entire scan.
    before_stop = len(batch.events)
    repair.on_scan_stop(batch)
    collect(batch.events[before_stop:])

    return out_rows, samples, events


def _stale(out_rows):
    return [r for r in out_rows if r[3] == "stale"]


def _quality(out_rows, q):
    return [r for r in out_rows if r[4] == q]


# ── Baseline ─────────────────────────────────────────────────────────────


def test_clean_stream_emits_no_evidence():
    """A clean scan produces no fault evidence of any kind."""
    out_rows, samples, events = _run_chain(_clean_rows())
    assert all(not v for v in events.values()), events
    assert not _stale(out_rows)
    assert not _quality(out_rows, "ts_corrected")
    assert not _quality(out_rows, "nan_filled")
    assert len(samples) == N_CAPTURES - WARMUP - 1


# ── fid_single -> FrameIdConsensusCorrection ─────────────────────────────


class TestFidSingle:
    @pytest.fixture(autouse=True, scope="class")
    def run(self, request):
        request.cls.out_rows, request.cls.samples, request.cls.events = (
            _run_chain(_fid_single_rows()))

    def test_consensus_repairs_each_corrupted_frame(self):
        cs = self.events["corrections"]
        assert len(cs) == len(BURST_CAPTURES)
        for c in cs:
            assert c.side == SIDE and c.cam_id == VICTIM
            assert c.corrected_frame_id == c.abs_frame_id & 0xFF
            assert c.wire_frame_id == c.corrected_frame_id & 0x3F
        assert {c.abs_frame_id for c in cs} == set(BURST_CAPTURES)

    def test_zero_data_loss(self):
        """The historic outcome was 64 real frames lost + 60 fabricated;
        the quarantine-era outcome was 3 lost; consensus loses none."""
        assert not _stale(self.out_rows)
        assert not _quality(self.out_rows, "nan_filled")
        assert not _quality(self.out_rows, "ts_corrected")
        assert self.events["windows"] == []
        victim_abs = {r[1] for r in self.out_rows
                      if r[0] == VICTIM and r[3] in ("light", "dark")}
        assert victim_abs == set(range(WARMUP + 1, N_CAPTURES + 1))

    def test_live_side_trace_full_rate_and_monotonic(self):
        assert len(self.samples) == N_CAPTURES - WARMUP - 1
        ts = [s.t for s in self.samples]
        assert all(b >= a for a, b in zip(ts, ts[1:]))
        assert all(np.isfinite(s.bfi) and abs(s.bfi - FULL_AVG) < 1e-6
                   for s in self.samples)


# ── fid_multi -> FrameIdPacketAnomaly ────────────────────────────────────


class TestFidMulti:
    @pytest.fixture(autouse=True, scope="class")
    def run(self, request):
        request.cls.out_rows, request.cls.samples, request.cls.events = (
            _run_chain(_fid_multi_rows()))

    def test_tie_reported_as_packet_anomaly(self):
        anomalies = self.events["anomalies"]
        assert len(anomalies) == len(BURST_CAPTURES)
        for a in anomalies:
            assert a.side == SIDE
            assert sorted(a.cam_ids) == sorted(CAMS)
            assert len(set(a.frame_ids)) == 2      # the 2-2 tie
            assert a.reason == "no_single_outlier_consensus"
        assert self.events["corrections"] == []

    def test_loss_bounded_to_the_inconsistent_frames(self):
        stale = _stale(self.out_rows)
        assert len(stale) == 2 * len(BURST_CAPTURES)
        assert {r[0] for r in stale} == {2, 3}
        # Honest cameras' frames all survive untouched.
        for cam in (0, 1):
            cam_abs = {r[1] for r in self.out_rows
                       if r[0] == cam and r[3] in ("light", "dark")}
            assert cam_abs == set(range(WARMUP + 1, N_CAPTURES + 1))
        # The gaps are honestly placeholdered and reported.
        fills = _quality(self.out_rows, "nan_filled")
        assert len(fills) == 2 * len(BURST_CAPTURES)
        gap_events = self.events["gap_fills"]
        assert {(g.side, g.cam_id) for g in gap_events} == {(SIDE, 2), (SIDE, 3)}
        assert all(g.gap_start_fid == BURST_CAPTURES[0]
                   and g.gap_end_fid == BURST_CAPTURES[-1]
                   and g.n_filled == len(BURST_CAPTURES)
                   for g in gap_events)
        assert self.events["windows"] == []


# ── timestamp_freeze -> TimestampRepairInputAnomaly ──────────────────────


class TestTimestampFreeze:
    @pytest.fixture(autouse=True, scope="class")
    def run(self, request):
        request.cls.out_rows, request.cls.samples, request.cls.events = (
            _run_chain(_timestamp_freeze_rows()))

    def test_frozen_value_reported_once(self):
        frozen = self.events["frozen"]
        assert len(frozen) == 1
        assert frozen[0].side == SIDE
        assert abs(frozen[0].timestamp_s - FREEZE_BASE * PERIOD_S) < 1e-9
        assert frozen[0].n_frames == len(FREEZE_CAPTURES) * len(CAMS)

    def test_frozen_frames_re_timestamped_not_discarded(self):
        assert not _stale(self.out_rows)
        assert not _quality(self.out_rows, "nan_filled")
        corrected = _quality(self.out_rows, "ts_corrected")
        assert corrected, "frozen frames must be re-timestamped"
        # Repaired timestamps land back near the true capture grid.
        for cam, abs_fid, ts, _, _ in corrected:
            assert abs(ts - abs_fid * PERIOD_S) < 3 * PERIOD_S
        ts = [s.t for s in self.samples]
        assert all(b >= a for a, b in zip(ts, ts[1:]))


# ── packet_drop -> FrameGapFillAnomaly ───────────────────────────────────


class TestPacketDrop:
    @pytest.fixture(autouse=True, scope="class")
    def run(self, request):
        request.cls.out_rows, request.cls.samples, request.cls.events = (
            _run_chain(_packet_drop_rows()))

    def test_gap_reported_per_camera_when_next_packet_arrives(self):
        gap_events = self.events["gap_fills"]
        assert {(g.side, g.cam_id) for g in gap_events} == {
            (SIDE, cam) for cam in CAMS}
        assert all(g.gap_start_fid == DROPPED_CAPTURE
                   and g.gap_end_fid == DROPPED_CAPTURE
                   and g.n_filled == 1 for g in gap_events)
        assert abs(gap_events[0].timestamp_s
                   - (DROPPED_CAPTURE + 1) * PERIOD_S) < 1e-9

    def test_gap_backfilled_with_honest_placeholders(self):
        fills = _quality(self.out_rows, "nan_filled")
        assert len(fills) == len(CAMS)
        assert all(r[1] == DROPPED_CAPTURE for r in fills)
        for _, abs_fid, ts, _, _ in fills:
            assert abs(ts - abs_fid * PERIOD_S) < PERIOD_S
        assert not _stale(self.out_rows)
        assert not _quality(self.out_rows, "ts_corrected")
        assert self.events["windows"] == []


# ── Sustained corruption: the flood presentation is gone ─────────────────


def test_sustained_corruption_no_flood_no_shredding():
    """Independent 10%/cam/frame hits, whole scan — the historic
    'kill the session' presentation (157 windows, ~2.6 WARNINGs/s, half
    the stream fabricated or discarded). Now: single-camera hits are
    consensus-repaired with zero loss, ambiguous packets degrade to
    bounded per-frame quarantine, and no misalignment windows open."""
    rows, exp = _sustained_rows()
    out_rows, samples, events = _run_chain(rows)

    assert exp["corrections"] + exp["stale"] > 40    # the scenario has teeth
    assert len(events["corrections"]) == exp["corrections"]
    assert len(events["anomalies"]) == exp["anomalies"]
    assert len(_stale(out_rows)) == exp["stale"]

    assert events["windows"] == []
    assert not _quality(out_rows, "ts_corrected")

    ts = [s.t for s in samples]
    assert all(b >= a for a, b in zip(ts, ts[1:]))


# ── Adversarial witness combinations found during PR review ─────────────


def _replace_capture(rows, capture, *, frame_delta=0, timestamp_delta_s=0.0):
    """Mutate one complete packet while retaining its packet boundaries."""
    target_t = capture * PERIOD_S
    return [
        (
            cam,
            ((raw + frame_delta) & 0xFF) if abs(ts - target_t) < 1e-12 else raw,
            ts + timestamp_delta_s if abs(ts - target_t) < 1e-12 else ts,
        )
        for cam, raw, ts in rows
    ]


@pytest.mark.parametrize("timestamp_delta_s", [10.0, -1.0])
def test_timestamp_outlier_does_not_poison_frame_state(timestamp_delta_s):
    """One bad packet timestamp must cost no later, honest captures."""
    rows = _replace_capture(
        _clean_rows(), 50, timestamp_delta_s=timestamp_delta_s,
    )
    out_rows, samples, events = _run_chain(rows)

    assert not _stale(out_rows)
    assert not _quality(out_rows, "nan_filled")
    corrected = _quality(out_rows, "ts_corrected")
    assert len(corrected) == len(CAMS)
    assert {r[1] for r in corrected} == {50}
    assert not events["anomalies"]
    assert all(b.t >= a.t for a, b in zip(samples, samples[1:]))


def test_shared_one_count_frame_id_error_is_not_admitted():
    """A packet-wide +1 counter error is small but still the wrong capture."""
    rows = _replace_capture(_clean_rows(), 198, frame_delta=1)
    out_rows, _, events = _run_chain(rows)

    stale = _stale(out_rows)
    assert len(stale) == len(CAMS)
    assert all(abs(r[2] - 198 * PERIOD_S) < 1e-12 for r in stale)
    fills = _quality(out_rows, "nan_filled")
    assert len(fills) == len(CAMS)
    assert {r[1] for r in fills} == {198}
    assert not _quality(out_rows, "ts_corrected")
    assert not events["windows"]


def test_matched_large_counter_and_timestamp_jump_fails_closed():
    """Two mutually consistent corrupt witnesses must not fabricate a gap."""
    rows = _replace_capture(
        _clean_rows(), 198,
        frame_delta=64,
        timestamp_delta_s=64 * PERIOD_S,
    )
    out_rows, _, events = _run_chain(rows)

    assert len(_stale(out_rows)) == len(CAMS)
    assert len(_quality(out_rows, "nan_filled")) == len(CAMS)
    assert sum(e.n_filled for e in events["gap_fills"]) == len(CAMS)
    assert not _quality(out_rows, "ts_corrected")


def test_timestamp_freeze_is_not_misreported_as_frame_id_packet_anomaly():
    """Repeated timestamps identify a fault, not a 16-camera packet."""
    out_rows, _, events = _run_chain(_timestamp_freeze_rows())

    assert not events["anomalies"]
    assert len(events["frozen"]) == 1
    assert events["frozen"][0].n_frames == len(FREEZE_CAPTURES) * len(CAMS)
    corrected_ids = {r[1] for r in _quality(out_rows, "ts_corrected")}
    assert corrected_ids == set(FREEZE_CAPTURES)
