"""FRC Driver Station .dslog and .dsevents file reader.

``.dslog`` files are binary (version 4), 50 Hz fixed-period records containing
battery voltage, trip time, packet loss, CPU/CAN utilization, mode flags, and
per-channel PDP/PDH currents.

``.dsevents`` files contain LabView-timestamped UTF-8 text messages (mode
changes, errors, warnings).

Both formats use LabView timestamps (seconds since 1904-01-01). This module
converts them to integer microseconds for consistency with the rest of
LogReader.

Reference implementation:
  https://github.com/Mechanical-Advantage/AdvantageScope/tree/main/src/hub/dataSources/dslog
"""

from __future__ import annotations

import re
import struct
from enum import Enum
from pathlib import Path

from logreader.models import (
    LogData,
    LogMetadata,
    SignalData,
    SignalInfo,
    SignalType,
    TimestampedValue,
)
from logreader.utils import resolve_path

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_LABVIEW_EPOCH_OFFSET = 2_082_844_800
"""Seconds between 1904-01-01 (LabView epoch) and 1970-01-01 (UNIX epoch).

Note: AdvantageScope uses 2_082_826_800 which is exactly 18 000 s (5 hours)
too small — likely because the offset was computed on a machine in the
US Eastern time zone (UTC-5) and the conversion API interpreted the
1904-01-01 date as local time rather than UTC.  This is harmless in
AdvantageScope because it only uses relative (0-based) timestamps for
records, so the constant cancels out.  We use absolute UNIX timestamps,
so the correct value matters.
"""

_HEADER_SIZE = 20  # 4 (version) + 8 (seconds) + 8 (fractional)
_RECORD_PERIOD_US = 20_000  # 20 ms = 50 Hz
_SUPPORTED_VERSION = 4

# XML-style tags stripped from dsevents text
_STRIP_TAGS = re.compile(
    r"<(?:TagVersion|time|count|flags|Code|location|stack)>[^<]*"
)


class _PDType(Enum):
    NONE = 0
    CTRE = 25
    REV = 33


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _convert_lv_time(seconds_be: bytes, fractional_be: bytes) -> float:
    """Convert a LabView timestamp to UNIX seconds (float)."""
    lv_seconds = struct.unpack(">q", seconds_be)[0]
    lv_frac = struct.unpack(">Q", fractional_be)[0]
    return (lv_seconds - _LABVIEW_EPOCH_OFFSET) + lv_frac / (2**64)


def _convert_lv_time_us(seconds_be: bytes, fractional_be: bytes) -> int:
    """Convert a LabView timestamp to UNIX microseconds (int)."""
    return int(_convert_lv_time(seconds_be, fractional_be) * 1_000_000)


def _get_pd_type(type_byte: int) -> _PDType:
    """Determine the power distribution type from the identifier byte."""
    if type_byte == _PDType.REV.value:
        return _PDType.REV
    if type_byte == _PDType.CTRE.value:
        return _PDType.CTRE
    return _PDType.NONE


def _decode_rev_currents(data: bytes, offset: int) -> tuple[list[float], int]:
    """Decode REV PDH per-channel currents.

    Returns (currents_list, bytes_consumed_past_offset).
    """
    # Skip CAN ID byte
    pos = offset + 1

    # 27 bytes → 216 bits, extract 20 × 10-bit values (LSB first)
    raw_bits: list[bool] = []
    for b in data[pos : pos + 27]:
        for i in range(8):
            raw_bits.append(bool(b & (1 << i)))
    pos += 27

    currents: list[float] = []
    for ch in range(20):
        read_pos = (ch // 3) * 32 + (ch % 3) * 10
        value = 0
        for i in range(10):
            if read_pos + i < len(raw_bits) and raw_bits[read_pos + i]:
                value += 1 << i
        currents.append(value / 8.0)

    # 4 switchable channels
    for i in range(4):
        if pos + i < len(data):
            currents.append(data[pos + i] / 16.0)
    pos += 4

    # Skip 1 temperature byte
    pos += 1

    return currents, pos - offset


def _decode_ctre_currents(data: bytes, offset: int) -> tuple[list[float], int]:
    """Decode CTRE PDP per-channel currents.

    Returns (currents_list, bytes_consumed_past_offset).
    """
    # Skip CAN ID byte
    pos = offset + 1

    # 21 bytes → 168 bits
    raw_bits: list[bool] = []
    for b in data[pos : pos + 21]:
        for i in range(8):
            raw_bits.append(bool(b & (1 << i)))

    currents: list[float] = []
    for ch in range(16):
        read_pos = (ch // 6) * 64 + (ch % 6) * 10
        value = 0
        for i in range(8):
            if read_pos + i < len(raw_bits) and raw_bits[read_pos + i]:
                value += 1 << i
        currents.append(value / 8.0)

    pos += 21
    # Skip 3 extra metadata bytes
    pos += 3

    return currents, pos - offset


def _clean_event_text(text: str) -> str:
    """Strip XML wrapper tags and prefixes from a dsevents message."""
    text = _STRIP_TAGS.sub("", text)
    text = text.replace("<message> ", "").replace("<details> ", "")
    return text.strip()


# ---------------------------------------------------------------------------
# Signal creation helpers
# ---------------------------------------------------------------------------

_NEXT_ENTRY_ID = 0


def _reset_entry_ids() -> None:
    """Reset the entry ID counter (called at the start of each read)."""
    global _NEXT_ENTRY_ID
    _NEXT_ENTRY_ID = 0


def _make_signal(name: str, sig_type: SignalType) -> SignalData:
    """Create an empty SignalData with a unique entry ID."""
    global _NEXT_ENTRY_ID
    _NEXT_ENTRY_ID += 1
    return SignalData(
        info=SignalInfo(
            entry_id=_NEXT_ENTRY_ID,
            name=name,
            type=sig_type,
        )
    )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def read_dslog(path: str | Path) -> LogData:
    """Read a ``.dslog`` file and return ``LogData`` with ``/DSLog/*`` signals.

    Parameters:
        path: Path to the ``.dslog`` file.

    Returns:
        A ``LogData`` containing telemetry signals at 50 Hz.

    Raises:
        FileNotFoundError: If *path* does not exist.
        ValueError: If the file version is not supported.
    """
    resolved = resolve_path(path)
    if not resolved.is_file():
        raise FileNotFoundError(f"DSLog file not found: {resolved}")

    data = resolved.read_bytes()
    if len(data) < _HEADER_SIZE:
        raise ValueError(f"DSLog file too small ({len(data)} bytes): {resolved}")

    version = struct.unpack(">i", data[0:4])[0]
    if version != _SUPPORTED_VERSION:
        raise ValueError(
            f"Unsupported DSLog version {version} (expected {_SUPPORTED_VERSION})"
        )

    start_time_us = _convert_lv_time_us(data[4:12], data[12:20])

    # Prepare signals
    _reset_entry_ids()

    sig_trip = _make_signal("/DSLog/TripTimeMS", SignalType.DOUBLE)
    sig_loss = _make_signal("/DSLog/PacketLoss", SignalType.DOUBLE)
    sig_batt = _make_signal("/DSLog/BatteryVoltage", SignalType.DOUBLE)
    sig_cpu = _make_signal("/DSLog/CPUUtilization", SignalType.DOUBLE)
    sig_can = _make_signal("/DSLog/CANUtilization", SignalType.DOUBLE)
    sig_brownout = _make_signal("/DSLog/Status/Brownout", SignalType.BOOLEAN)
    sig_watchdog = _make_signal("/DSLog/Status/Watchdog", SignalType.BOOLEAN)
    sig_ds_teleop = _make_signal("/DSLog/Status/DSTeleop", SignalType.BOOLEAN)
    sig_ds_disabled = _make_signal("/DSLog/Status/DSDisabled", SignalType.BOOLEAN)
    sig_robot_teleop = _make_signal("/DSLog/Status/RobotTeleop", SignalType.BOOLEAN)
    sig_robot_auto = _make_signal("/DSLog/Status/RobotAuto", SignalType.BOOLEAN)
    sig_robot_disabled = _make_signal("/DSLog/Status/RobotDisabled", SignalType.BOOLEAN)
    sig_pd_currents = _make_signal(
        "/DSLog/PowerDistributionCurrents", SignalType.DOUBLE_ARRAY
    )
    sig_wifi_db = _make_signal("/DSLog/WifiDb", SignalType.DOUBLE)
    sig_wifi_mb = _make_signal("/DSLog/WifiMb", SignalType.DOUBLE)

    all_signals: dict[str, SignalData] = {}
    for sig in [
        sig_trip, sig_loss, sig_batt, sig_cpu, sig_can,
        sig_brownout, sig_watchdog, sig_ds_teleop, sig_ds_disabled,
        sig_robot_teleop, sig_robot_auto, sig_robot_disabled,
        sig_pd_currents, sig_wifi_db, sig_wifi_mb,
    ]:
        all_signals[sig.info.name] = sig

    pos = _HEADER_SIZE
    record_count = 0
    last_batt_volts = 0.0
    warnings: list[str] = []

    try:
        while pos + 10 <= len(data):
            ts = start_time_us + record_count * _RECORD_PERIOD_US

            # Core 10 bytes
            trip_time_ms = data[pos] * 0.5
            packet_loss = max(0.0, min(1.0, struct.unpack(">b", data[pos + 1 : pos + 2])[0] * 4 * 0.01))
            batt_volts = struct.unpack(">H", data[pos + 2 : pos + 4])[0] / 256.0
            cpu_util = data[pos + 4] * 0.5 * 0.01
            mask = data[pos + 5]
            can_util = data[pos + 6] * 0.5 * 0.01
            wifi_db = data[pos + 7] * 0.5
            wifi_mb = struct.unpack(">H", data[pos + 8 : pos + 10])[0] / 256.0

            # Battery voltage spike guard
            if batt_volts > 20.0:
                batt_volts = last_batt_volts
            else:
                last_batt_volts = batt_volts

            # Status mask — bits are active-low
            brownout = (mask & 0x80) == 0
            watchdog = (mask & 0x40) == 0
            ds_teleop = (mask & 0x20) == 0
            ds_disabled = (mask & 0x08) == 0
            robot_teleop = (mask & 0x04) == 0
            robot_auto = (mask & 0x02) == 0
            robot_disabled = (mask & 0x01) == 0

            pos += 10

            # Power distribution block (4-byte header)
            currents: list[float] = []
            if pos + 4 <= len(data):
                pd_type = _get_pd_type(data[pos + 3])
                pos += 4

                if pd_type == _PDType.REV:
                    if pos + 33 <= len(data):
                        currents, consumed = _decode_rev_currents(data, pos)
                        pos += consumed
                    else:
                        # Truncated REV block — skip to best guess
                        remaining = len(data) - pos
                        pos += remaining

                elif pd_type == _PDType.CTRE:
                    if pos + 25 <= len(data):
                        currents, consumed = _decode_ctre_currents(data, pos)
                        pos += consumed
                    else:
                        remaining = len(data) - pos
                        pos += remaining

                # _PDType.NONE → no extra PD bytes
            else:
                # Not enough data for PD header — we're at the end
                break

            # Append values
            _tv = TimestampedValue
            sig_trip.values.append(_tv(ts, trip_time_ms))
            sig_loss.values.append(_tv(ts, packet_loss))
            sig_batt.values.append(_tv(ts, batt_volts))
            sig_cpu.values.append(_tv(ts, cpu_util))
            sig_can.values.append(_tv(ts, can_util))
            sig_brownout.values.append(_tv(ts, brownout))
            sig_watchdog.values.append(_tv(ts, watchdog))
            sig_ds_teleop.values.append(_tv(ts, ds_teleop))
            sig_ds_disabled.values.append(_tv(ts, ds_disabled))
            sig_robot_teleop.values.append(_tv(ts, robot_teleop))
            sig_robot_auto.values.append(_tv(ts, robot_auto))
            sig_robot_disabled.values.append(_tv(ts, robot_disabled))
            sig_pd_currents.values.append(_tv(ts, currents))
            sig_wifi_db.values.append(_tv(ts, wifi_db))
            sig_wifi_mb.values.append(_tv(ts, wifi_mb))

            record_count += 1

    except Exception as exc:
        warnings.append(f"Error reading DSLog at offset {pos}: {exc}")

    metadata = LogMetadata(
        file_path=str(resolved),
        version=version,
        is_valid=True,
        signal_count=len(all_signals),
        record_count=record_count,
        warnings=warnings,
    )

    return LogData(metadata=metadata, signals=all_signals)


def read_dsevents(path: str | Path) -> LogData:
    """Read a ``.dsevents`` file and return ``LogData`` with ``/DSEvents``.

    Parameters:
        path: Path to the ``.dsevents`` file.

    Returns:
        A ``LogData`` containing a single ``/DSEvents`` string signal.

    Raises:
        FileNotFoundError: If *path* does not exist.
        ValueError: If the file version is not supported.
    """
    resolved = resolve_path(path)
    if not resolved.is_file():
        raise FileNotFoundError(f"DSEvents file not found: {resolved}")

    data = resolved.read_bytes()
    if len(data) < _HEADER_SIZE:
        raise ValueError(
            f"DSEvents file too small ({len(data)} bytes): {resolved}"
        )

    version = struct.unpack(">i", data[0:4])[0]
    if version != _SUPPORTED_VERSION:
        raise ValueError(
            f"Unsupported DSEvents version {version} "
            f"(expected {_SUPPORTED_VERSION})"
        )

    start_time_us = _convert_lv_time_us(data[4:12], data[12:20])

    _reset_entry_ids()
    sig_events = _make_signal("/DSEvents", SignalType.STRING)

    pos = _HEADER_SIZE
    record_count = 0
    warnings: list[str] = []

    try:
        while pos + 20 <= len(data):
            # Absolute LabView timestamp for this event
            event_time_us = _convert_lv_time_us(
                data[pos : pos + 8], data[pos + 8 : pos + 16]
            )
            pos += 16

            # Text length
            if pos + 4 > len(data):
                break
            text_len = struct.unpack(">i", data[pos : pos + 4])[0]
            pos += 4

            if text_len < 0 or pos + text_len > len(data):
                warnings.append(
                    f"DSEvents: invalid text length {text_len} at offset {pos - 4}"
                )
                break

            text = data[pos : pos + text_len].decode("utf-8", errors="replace")
            pos += text_len

            text = _clean_event_text(text)

            # Compute relative timestamp from log start
            relative_us = event_time_us - start_time_us

            sig_events.values.append(
                TimestampedValue(relative_us, text)
            )
            record_count += 1

    except Exception as exc:
        warnings.append(f"Error reading DSEvents at offset {pos}: {exc}")

    metadata = LogMetadata(
        file_path=str(resolved),
        version=version,
        is_valid=True,
        signal_count=1,
        record_count=record_count,
        warnings=warnings,
    )

    return LogData(metadata=metadata, signals={"/DSEvents": sig_events})


def read_ds_logs(
    dslog_path: str | Path,
    dsevents_path: str | Path | None = None,
) -> LogData:
    """Read a ``.dslog`` and optionally a ``.dsevents``, merging into one ``LogData``.

    If only *dslog_path* is given, attempt to find a matching ``.dsevents``
    file alongside it (same stem).

    Parameters:
        dslog_path: Path to the ``.dslog`` file.
        dsevents_path: Path to the ``.dsevents`` file, or ``None`` to
            auto-discover.

    Returns:
        A merged ``LogData`` with ``/DSLog/*`` and optionally ``/DSEvents``.
    """
    log_data = read_dslog(dslog_path)

    # Auto-discover dsevents if not explicitly given
    if dsevents_path is None:
        candidate = Path(dslog_path).with_suffix(".dsevents")
        if candidate.is_file():
            dsevents_path = candidate

    if dsevents_path is not None:
        events_data = read_dsevents(dsevents_path)
        # Merge the events signal into the dslog LogData
        for name, sig in events_data.signals.items():
            log_data.signals[name] = sig
        log_data.metadata.signal_count = len(log_data.signals)
        log_data.metadata.record_count += events_data.metadata.record_count
        log_data.metadata.warnings.extend(events_data.metadata.warnings)

    return log_data
