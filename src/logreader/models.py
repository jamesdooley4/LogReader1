"""Data models for log entries, signals, and metadata."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class SignalType(Enum):
    """The data type of a logged signal."""

    BOOLEAN = "boolean"
    DOUBLE = "double"
    FLOAT = "float"
    INTEGER = "int64"
    STRING = "string"
    BOOLEAN_ARRAY = "boolean[]"
    DOUBLE_ARRAY = "double[]"
    FLOAT_ARRAY = "float[]"
    INTEGER_ARRAY = "int64[]"
    STRING_ARRAY = "string[]"
    RAW = "raw"
    STRUCT = "struct"
    UNKNOWN = "unknown"


@dataclass
class SignalInfo:
    """Metadata about a single logged signal (from a start control record).

    Attributes:
        entry_id: The integer entry ID assigned by the DataLog.
        name: The full hierarchical name, e.g. ``/SmartDashboard/Speed``.
        type: The signal data type as a ``SignalType`` enum member.
        metadata: Arbitrary metadata string attached to the signal.
    """

    entry_id: int
    name: str
    type: SignalType
    metadata: str = ""


@dataclass
class TimestampedValue:
    """A single timestamped data point.

    Attributes:
        timestamp_us: Timestamp in integer microseconds (as returned by
            ``DataLogRecord.getTimestamp()``).
        value: The decoded value; its Python type depends on ``SignalType``.
    """

    timestamp_us: int
    value: Any


@dataclass
class SignalData:
    """A complete signal: metadata plus all recorded values.

    Attributes:
        info: Signal metadata (name, type, etc.).
        values: Time-ordered list of ``TimestampedValue`` instances.
    """

    info: SignalInfo
    values: list[TimestampedValue] = field(default_factory=list)

    @property
    def timestamps_seconds(self) -> list[float]:
        """Return timestamps converted to seconds."""
        return [v.timestamp_us / 1_000_000.0 for v in self.values]

    @property
    def raw_values(self) -> list[Any]:
        """Return just the values without timestamps."""
        return [v.value for v in self.values]


@dataclass
class LogMetadata:
    """Top-level metadata about a log file.

    Attributes:
        file_path: Original path of the log file on disk.
        version: DataLog format version number.
        extra_header: Extra header data from the DataLog.
        is_valid: Whether the log file passed validity checks.
        signal_count: Number of distinct signals found in the log.
        record_count: Total number of data records processed.
    """

    file_path: str
    version: int = 0
    extra_header: str = ""
    is_valid: bool = False
    signal_count: int = 0
    record_count: int = 0
    warnings: list[str] = field(default_factory=list)


@dataclass
class LogData:
    """Fully parsed log file contents.

    Attributes:
        metadata: File-level metadata.
        signals: Mapping of signal name → ``SignalData``.
    """

    metadata: LogMetadata
    signals: dict[str, SignalData] = field(default_factory=dict)

    def signal_names(self) -> list[str]:
        """Return a sorted list of all signal names."""
        return sorted(self.signals.keys())

    def get_signal(self, name: str) -> SignalData | None:
        """Look up a signal by name; returns ``None`` if not found."""
        return self.signals.get(name)
