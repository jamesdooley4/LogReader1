"""Tests for log data processing utilities."""

from logreader.models import (
    LogData,
    LogMetadata,
    SignalData,
    SignalInfo,
    SignalType,
    TimestampedValue,
)
from logreader.processor import (
    compute_numeric_stats,
    filter_signals_by_prefix,
    filter_signals_by_type,
    get_time_range,
    slice_by_time,
)


def _make_log() -> LogData:
    meta = LogMetadata(file_path="test.wpilog", is_valid=True, signal_count=3)
    return LogData(
        metadata=meta,
        signals={
            "/SmartDashboard/Speed": SignalData(
                info=SignalInfo(
                    entry_id=1, name="/SmartDashboard/Speed", type=SignalType.DOUBLE
                ),
                values=[
                    TimestampedValue(1_000_000, 0.5),
                    TimestampedValue(2_000_000, 1.0),
                    TimestampedValue(3_000_000, 1.5),
                ],
            ),
            "/SmartDashboard/Name": SignalData(
                info=SignalInfo(
                    entry_id=2, name="/SmartDashboard/Name", type=SignalType.STRING
                ),
                values=[TimestampedValue(1_000_000, "auto")],
            ),
            "/Other/Flag": SignalData(
                info=SignalInfo(
                    entry_id=3, name="/Other/Flag", type=SignalType.BOOLEAN
                ),
                values=[
                    TimestampedValue(500_000, True),
                    TimestampedValue(4_000_000, False),
                ],
            ),
        },
    )


def test_filter_signals_by_type() -> None:
    ld = _make_log()
    doubles = filter_signals_by_type(ld, SignalType.DOUBLE)
    assert list(doubles.keys()) == ["/SmartDashboard/Speed"]


def test_filter_signals_by_prefix() -> None:
    ld = _make_log()
    sd = filter_signals_by_prefix(ld, "/SmartDashboard/")
    assert sorted(sd.keys()) == ["/SmartDashboard/Name", "/SmartDashboard/Speed"]


def test_get_time_range() -> None:
    ld = _make_log()
    tr = get_time_range(ld)
    assert tr is not None
    assert tr == (0.5, 4.0)


def test_get_time_range_empty() -> None:
    ld = LogData(metadata=LogMetadata(file_path="x"))
    assert get_time_range(ld) is None


def test_slice_by_time() -> None:
    ld = _make_log()
    sig = ld.signals["/SmartDashboard/Speed"]
    sliced = slice_by_time(sig, 1.5, 2.5)
    assert len(sliced.values) == 1
    assert sliced.values[0].value == 1.0


def test_compute_numeric_stats() -> None:
    ld = _make_log()
    stats = compute_numeric_stats(ld.signals["/SmartDashboard/Speed"])
    assert stats is not None
    assert stats["count"] == 3
    assert stats["min"] == 0.5
    assert stats["max"] == 1.5
    assert abs(stats["mean"] - 1.0) < 1e-9


def test_compute_numeric_stats_non_numeric() -> None:
    ld = _make_log()
    assert compute_numeric_stats(ld.signals["/SmartDashboard/Name"]) is None
