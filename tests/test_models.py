"""Tests for the models module."""

from logreader.models import (
    LogData,
    LogMetadata,
    SignalData,
    SignalInfo,
    SignalType,
    TimestampedValue,
)


def test_signal_type_values() -> None:
    assert SignalType.BOOLEAN.value == "boolean"
    assert SignalType.DOUBLE.value == "double"
    assert SignalType.INTEGER.value == "int64"
    assert SignalType.STRING_ARRAY.value == "string[]"


def test_timestamped_value() -> None:
    tv = TimestampedValue(timestamp_us=1_500_000, value=3.14)
    assert tv.timestamp_us == 1_500_000
    assert tv.value == 3.14


def test_signal_data_timestamps_seconds() -> None:
    info = SignalInfo(entry_id=1, name="/test", type=SignalType.DOUBLE)
    sd = SignalData(
        info=info,
        values=[
            TimestampedValue(timestamp_us=0, value=0.0),
            TimestampedValue(timestamp_us=1_000_000, value=1.0),
            TimestampedValue(timestamp_us=2_500_000, value=2.5),
        ],
    )
    assert sd.timestamps_seconds == [0.0, 1.0, 2.5]


def test_signal_data_raw_values() -> None:
    info = SignalInfo(entry_id=2, name="/v", type=SignalType.INTEGER)
    sd = SignalData(
        info=info,
        values=[
            TimestampedValue(timestamp_us=0, value=10),
            TimestampedValue(timestamp_us=100, value=20),
        ],
    )
    assert sd.raw_values == [10, 20]


def test_log_data_signal_names() -> None:
    meta = LogMetadata(file_path="test.wpilog", is_valid=True)
    ld = LogData(
        metadata=meta,
        signals={
            "/B": SignalData(
                info=SignalInfo(entry_id=2, name="/B", type=SignalType.BOOLEAN)
            ),
            "/A": SignalData(
                info=SignalInfo(entry_id=1, name="/A", type=SignalType.DOUBLE)
            ),
        },
    )
    assert ld.signal_names() == ["/A", "/B"]


def test_log_data_get_signal() -> None:
    meta = LogMetadata(file_path="test.wpilog", is_valid=True)
    sig = SignalData(info=SignalInfo(entry_id=1, name="/X", type=SignalType.STRING))
    ld = LogData(metadata=meta, signals={"/X": sig})
    assert ld.get_signal("/X") is sig
    assert ld.get_signal("/missing") is None
