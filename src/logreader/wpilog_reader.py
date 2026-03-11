"""WPILib .wpilog file reader using ``wpiutil.log.DataLogReader``."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from wpiutil.log import DataLogReader, DataLogRecord

from logreader.models import (
    LogData,
    LogMetadata,
    SignalData,
    SignalInfo,
    SignalType,
    TimestampedValue,
)
from logreader.utils import resolve_path

# Mapping from WPILib type strings to our SignalType enum
_TYPE_MAP: dict[str, SignalType] = {
    "boolean": SignalType.BOOLEAN,
    "double": SignalType.DOUBLE,
    "float": SignalType.FLOAT,
    "int64": SignalType.INTEGER,
    "string": SignalType.STRING,
    "boolean[]": SignalType.BOOLEAN_ARRAY,
    "double[]": SignalType.DOUBLE_ARRAY,
    "float[]": SignalType.FLOAT_ARRAY,
    "int64[]": SignalType.INTEGER_ARRAY,
    "string[]": SignalType.STRING_ARRAY,
    "raw": SignalType.RAW,
    "rawBytes": SignalType.RAW,
}

# Mapping from SignalType to the DataLogRecord getter method name
_GETTER_MAP: dict[SignalType, str] = {
    SignalType.BOOLEAN: "getBoolean",
    SignalType.DOUBLE: "getDouble",
    SignalType.FLOAT: "getFloat",
    SignalType.INTEGER: "getInteger",
    SignalType.STRING: "getString",
    SignalType.BOOLEAN_ARRAY: "getBooleanArray",
    SignalType.DOUBLE_ARRAY: "getDoubleArray",
    SignalType.FLOAT_ARRAY: "getFloatArray",
    SignalType.INTEGER_ARRAY: "getIntegerArray",
    SignalType.STRING_ARRAY: "getStringArray",
    SignalType.RAW: "getRaw",
}


def _decode_record_value(record: DataLogRecord, signal_type: SignalType) -> Any:
    """Decode a data record's value based on its signal type.

    Falls back to ``getRaw()`` for struct / unknown types.
    """
    getter_name = _GETTER_MAP.get(signal_type, "getRaw")
    getter = getattr(record, getter_name, None)
    if getter is None:
        return record.getRaw()
    return getter()


def _classify_type(type_str: str) -> SignalType:
    """Convert a WPILib type string to a ``SignalType``.

    Anything beginning with ``struct:`` is classified as STRUCT.
    """
    if type_str.startswith("struct:") or type_str.startswith("structschema:"):
        return SignalType.STRUCT
    return _TYPE_MAP.get(type_str, SignalType.UNKNOWN)


def read_wpilog(path: str | Path) -> LogData:
    """Read a ``.wpilog`` file and return fully parsed ``LogData``.

    Parameters:
        path: Path to the ``.wpilog`` file.

    Returns:
        A ``LogData`` instance containing all signals and metadata.

    Raises:
        FileNotFoundError: If *path* does not exist.
        ValueError: If the log file is invalid.
    """
    resolved = resolve_path(path)
    if not resolved.is_file():
        raise FileNotFoundError(f"Log file not found: {resolved}")

    reader = DataLogReader(str(resolved))

    if not reader.isValid():
        raise ValueError(f"Invalid WPILib log file: {resolved}")

    metadata = LogMetadata(
        file_path=str(resolved),
        version=reader.getVersion(),
        extra_header=reader.getExtraHeader(),
        is_valid=True,
    )

    # entry_id → SignalInfo  (populated from start control records)
    entry_map: dict[int, SignalInfo] = {}
    signals: dict[str, SignalData] = {}
    record_count = 0

    for record in reader:
        # --- Control records ---
        if record.isControl():
            if record.isStart():
                start = record.getStartData()
                sig_type = _classify_type(start.type)
                info = SignalInfo(
                    entry_id=start.entry,
                    name=start.name,
                    type=sig_type,
                    metadata=start.metadata,
                )
                entry_map[start.entry] = info
                if start.name not in signals:
                    signals[start.name] = SignalData(info=info)
            # finish and setMetadata control records are noted but not
            # required for basic data extraction.
            continue

        # --- Data records ---
        entry_id = record.getEntry()
        info = entry_map.get(entry_id)
        if info is None:
            # Data for an entry we haven't seen a start record for — skip.
            continue

        record_count += 1
        try:
            value = _decode_record_value(record, info.type)
        except (TypeError, RuntimeError):
            # If decoding fails, store the raw bytes.
            value = record.getRaw()

        signals[info.name].values.append(
            TimestampedValue(timestamp_us=record.getTimestamp(), value=value)
        )

    metadata.signal_count = len(signals)
    metadata.record_count = record_count
    return LogData(metadata=metadata, signals=signals)
