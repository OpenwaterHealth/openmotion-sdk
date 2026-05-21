import csv
import io

from omotion.ConsoleTelemetry import ConsoleTelemetry, PdcSample
from omotion.ScanWorkflow import _TELEMETRY_HEADERS, _pdc_row


def test_headers_include_new_columns_in_appended_order():
    assert _TELEMETRY_HEADERS[-5:] == [
        "frame_idx", "dark_slot", "pdc_flags", "pdc_dropped_delta", "slow_age_ms"
    ]
    # Existing columns preserved (subset spot-check).
    assert _TELEMETRY_HEADERS[0] == "timestamp"
    assert _TELEMETRY_HEADERS[1:4] == ["tcm", "tcl", "pdc"]


def test_pdc_row_with_fresh_snapshot():
    snap = ConsoleTelemetry()
    snap.timestamp = 1000.0
    snap.tcm = 555
    snap.tcl = 444
    snap.pdc = 12.34
    snap.tec_v_raw = 1.1
    snap.tec_set_raw = 1.2
    snap.tec_curr_raw = 1.3
    snap.tec_volt_raw = 1.4
    snap.tec_good = True
    snap.pdu_raws = list(range(16))
    snap.pdu_volts = [float(i) * 0.1 for i in range(16)]
    snap.safety_se = 0x00
    snap.safety_so = 0x00
    snap.safety_ok = True
    snap.read_ok = True
    snap.error = None

    sample = PdcSample(
        frame_idx=601,
        pdc_mA=15.0,
        dark_slot=True,
        host_recv_timestamp=1001.025,
        dropped_delta=0,
    )

    row = _pdc_row(sample, snap)
    assert len(row) == len(_TELEMETRY_HEADERS)
    d = dict(zip(_TELEMETRY_HEADERS, row))
    assert d["timestamp"] == 1001.025
    assert d["tcm"] == 601           # tcm == frame_idx for PDC-driven rows
    assert d["tcl"] == 444           # carry-forward from snapshot
    assert d["pdc"] == 15.0          # this row's per-frame value
    assert d["dark_slot"] == 1
    assert d["pdc_flags"] == 1       # bit 0 set
    assert d["pdc_dropped_delta"] == 0
    assert d["slow_age_ms"] == 1025  # 1001.025 - 1000.0 = 1.025 s
    assert d["frame_idx"] == 601


def test_pdc_row_before_first_slow_snapshot_leaves_slow_cols_blank():
    sample = PdcSample(frame_idx=1, pdc_mA=20.0, dark_slot=False,
                       host_recv_timestamp=10.0, dropped_delta=0)
    row = _pdc_row(sample, snap=None)
    d = dict(zip(_TELEMETRY_HEADERS, row))
    assert d["timestamp"] == 10.0
    assert d["tcm"] == 1
    assert d["tcl"] == ""
    assert d["pdc"] == 20.0
    assert d["tec_v_raw"] == ""
    assert d["pdu_raw_0"] == ""
    assert d["safety_se"] == ""
    assert d["dark_slot"] == 0
    assert d["slow_age_ms"] == ""


def test_pdc_row_round_trips_through_csv_writer():
    snap = ConsoleTelemetry()
    snap.timestamp = 100.0
    sample = PdcSample(frame_idx=42, pdc_mA=1.9, dark_slot=False,
                       host_recv_timestamp=100.05, dropped_delta=3)
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(_TELEMETRY_HEADERS)
    w.writerow(_pdc_row(sample, snap))

    buf.seek(0)
    rdr = csv.DictReader(buf)
    rows = list(rdr)
    assert len(rows) == 1
    assert rows[0]["frame_idx"] == "42"
    assert rows[0]["pdc"] == "1.9"
    assert rows[0]["pdc_dropped_delta"] == "3"
