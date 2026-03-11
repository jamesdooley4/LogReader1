"""Log data processing and analysis utilities."""

from __future__ import annotations

from typing import Any

from logreader.models import LogData, SignalData, SignalType


def filter_signals_by_type(
    log_data: LogData, signal_type: SignalType
) -> dict[str, SignalData]:
    """Return only signals matching the given ``SignalType``.

    Parameters:
        log_data: The parsed log data.
        signal_type: The type to filter by.

    Returns:
        A dict of signal name → ``SignalData`` for matching signals.
    """
    return {
        name: sig
        for name, sig in log_data.signals.items()
        if sig.info.type == signal_type
    }


def filter_signals_by_prefix(log_data: LogData, prefix: str) -> dict[str, SignalData]:
    """Return only signals whose name starts with *prefix*.

    Parameters:
        log_data: The parsed log data.
        prefix: Signal name prefix to match (e.g. ``/SmartDashboard/``).

    Returns:
        A dict of matching signal name → ``SignalData``.
    """
    return {
        name: sig for name, sig in log_data.signals.items() if name.startswith(prefix)
    }


def get_time_range(log_data: LogData) -> tuple[float, float] | None:
    """Return the overall (min, max) time range in seconds across all signals.

    Returns:
        A ``(start_seconds, end_seconds)`` tuple, or ``None`` if there is no
        data.
    """
    min_ts: int | None = None
    max_ts: int | None = None

    for sig in log_data.signals.values():
        if not sig.values:
            continue
        first = sig.values[0].timestamp_us
        last = sig.values[-1].timestamp_us
        if min_ts is None or first < min_ts:
            min_ts = first
        if max_ts is None or last > max_ts:
            max_ts = last

    if min_ts is None or max_ts is None:
        return None

    return min_ts / 1_000_000.0, max_ts / 1_000_000.0


def slice_by_time(
    signal: SignalData,
    start_seconds: float,
    end_seconds: float,
) -> SignalData:
    """Return a new ``SignalData`` containing only values within a time window.

    Parameters:
        signal: The source signal.
        start_seconds: Start of the window (inclusive) in seconds.
        end_seconds: End of the window (inclusive) in seconds.

    Returns:
        A new ``SignalData`` with the same info but filtered values.
    """
    start_us = int(start_seconds * 1_000_000)
    end_us = int(end_seconds * 1_000_000)
    filtered = [v for v in signal.values if start_us <= v.timestamp_us <= end_us]
    return SignalData(info=signal.info, values=filtered)


def compute_numeric_stats(signal: SignalData) -> dict[str, Any] | None:
    """Compute basic statistics for a numeric signal.

    Only works for DOUBLE, FLOAT, and INTEGER signal types.

    Returns:
        A dict with keys ``min``, ``max``, ``mean``, ``count``,
        ``first_timestamp_s``, ``last_timestamp_s``, or ``None`` if the
        signal is not numeric or has no values.
    """
    if signal.info.type not in (
        SignalType.DOUBLE,
        SignalType.FLOAT,
        SignalType.INTEGER,
    ):
        return None
    if not signal.values:
        return None

    nums = [float(v.value) for v in signal.values]
    return {
        "min": min(nums),
        "max": max(nums),
        "mean": sum(nums) / len(nums),
        "count": len(nums),
        "first_timestamp_s": signal.values[0].timestamp_us / 1_000_000.0,
        "last_timestamp_s": signal.values[-1].timestamp_us / 1_000_000.0,
    }


def summarise(log_data: LogData) -> str:
    """Return a human-readable summary of the log file.

    Parameters:
        log_data: Parsed log data.

    Returns:
        A multi-line summary string.
    """
    lines: list[str] = []
    m = log_data.metadata
    lines.append(f"File        : {m.file_path}")
    lines.append(f"Valid       : {m.is_valid}")
    lines.append(f"Version     : {m.version:#06x}")
    lines.append(f"Signals     : {m.signal_count}")
    lines.append(f"Data records: {m.record_count}")

    time_range = get_time_range(log_data)
    if time_range:
        duration = time_range[1] - time_range[0]
        lines.append(
            f"Time range  : {time_range[0]:.3f} s – {time_range[1]:.3f} s  ({duration:.3f} s)"
        )
    else:
        lines.append("Time range  : (no data)")

    lines.append("")
    lines.append("Signals:")
    for name in log_data.signal_names():
        sig = log_data.signals[name]
        lines.append(f"  {name}  [{sig.info.type.value}]  ({len(sig.values)} pts)")

    return "\n".join(lines)
