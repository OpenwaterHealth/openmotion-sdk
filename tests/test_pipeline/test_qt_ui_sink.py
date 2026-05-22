"""QtUiSink stub — records live-channel payloads for testability."""


def test_qt_ui_sink_records_live_batches_only():
    from omotion.pipeline.sinks import QtUiSink
    sink = QtUiSink()
    sink.on_scan_start(None)
    sink.consume("live", "payload_1")
    sink.consume("rolling", "payload_2")  # not subscribed; would not arrive in practice
    sink.consume("live", "payload_3")
    sink.on_complete()
    assert sink.live_batches == ["payload_1", "payload_3"]


def test_qt_ui_sink_channels_attribute():
    from omotion.pipeline.sinks import QtUiSink
    sink = QtUiSink()
    assert "live" in sink.channels
