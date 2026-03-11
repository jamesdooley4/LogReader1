"""Tests for utility helpers."""

from logreader.utils import (
    file_extension,
    format_duration,
    timestamp_us_to_seconds,
)


def test_timestamp_us_to_seconds() -> None:
    assert timestamp_us_to_seconds(0) == 0.0
    assert timestamp_us_to_seconds(1_000_000) == 1.0
    assert timestamp_us_to_seconds(1_500_000) == 1.5


def test_format_duration() -> None:
    assert format_duration(0.0) == "00:00:00.000"
    assert format_duration(61.5) == "00:01:01.500"
    assert format_duration(3661.123) == "01:01:01.123"


def test_file_extension() -> None:
    assert file_extension("robot.wpilog") == ".wpilog"
    assert file_extension("data.HOOT") == ".hoot"
    assert file_extension("/path/to/file.csv") == ".csv"
    assert file_extension("noext") == ""
