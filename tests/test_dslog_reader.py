"""Tests for the dslog_reader module."""

from __future__ import annotations

import struct
import tempfile
from pathlib import Path

import pytest

from logreader.dslog_reader import (
    _HEADER_SIZE,
    _LABVIEW_EPOCH_OFFSET,
    _PDType,
    _clean_event_text,
    _convert_lv_time,
    _convert_lv_time_us,
    read_ds_logs,
    read_dsevents,
    read_dslog,
)
from logreader.models import SignalType


# ---------------------------------------------------------------------------
# Helper: build synthetic binary blobs
# ---------------------------------------------------------------------------

def _make_lv_timestamp(unix_seconds: float) -> bytes:
    """Build a 16-byte LabView timestamp from a UNIX time in seconds."""
    lv_seconds = int(unix_seconds) + _LABVIEW_EPOCH_OFFSET
    frac = int((unix_seconds - int(unix_seconds)) * (2**64))
    return struct.pack(">q", lv_seconds) + struct.pack(">Q", frac)


def _make_dslog_header(unix_start: float, version: int = 4) -> bytes:
    """Build a 20-byte .dslog header."""
    return struct.pack(">i", version) + _make_lv_timestamp(unix_start)


def _make_dslog_record(
    *,
    trip_time_raw: int = 10,      # × 0.5 → 5.0 ms
    packet_loss_raw: int = 0,     # × 4 × 0.01 → 0.0
    batt_voltage_raw: int = 3200, # ÷ 256 → 12.5 V
    cpu_raw: int = 100,           # × 0.5 × 0.01 → 0.50
    mask: int = 0xFF,             # all bits set → all flags False (active-low)
    can_raw: int = 50,            # × 0.5 × 0.01 → 0.25
    wifi_db_raw: int = 80,        # × 0.5 → 40.0 dB
    wifi_mb_raw: int = 512,       # ÷ 256 → 2.0 Mbps
    pd_type: _PDType = _PDType.NONE,
) -> bytes:
    """Build one dslog record (10 core bytes + 4-byte PD header, no PD data)."""
    core = struct.pack(
        ">BbHBBBBH",
        trip_time_raw,
        packet_loss_raw,
        batt_voltage_raw,
        cpu_raw,
        mask,
        can_raw,
        wifi_db_raw,
        wifi_mb_raw,
    )
    # PD header: 3 zero bytes + type byte
    pd_header = struct.pack(">BBBB", 0, 0, 0, pd_type.value)
    return core + pd_header


def _make_dsevents_record(
    unix_timestamp: float,
    text: str,
) -> bytes:
    """Build one dsevents record."""
    ts_bytes = _make_lv_timestamp(unix_timestamp)
    encoded = text.encode("utf-8")
    return ts_bytes + struct.pack(">i", len(encoded)) + encoded


# ---------------------------------------------------------------------------
# Unit tests: helpers
# ---------------------------------------------------------------------------

class TestConvertLVTime:
    def test_basic(self) -> None:
        unix_time = 1_700_000_000.0  # some recent UNIX time
        lv_seconds = int(unix_time) + _LABVIEW_EPOCH_OFFSET
        ts_bytes = struct.pack(">q", lv_seconds) + struct.pack(">Q", 0)
        result = _convert_lv_time(ts_bytes[:8], ts_bytes[8:])
        assert abs(result - unix_time) < 0.001

    def test_with_fractional(self) -> None:
        unix_time = 1_700_000_000.5
        lv_seconds = int(unix_time) + _LABVIEW_EPOCH_OFFSET
        frac = int(0.5 * (2**64))
        ts_bytes = struct.pack(">q", lv_seconds) + struct.pack(">Q", frac)
        result = _convert_lv_time(ts_bytes[:8], ts_bytes[8:])
        assert abs(result - unix_time) < 0.001

    def test_microseconds(self) -> None:
        unix_time = 1_700_000_000.0
        lv_seconds = int(unix_time) + _LABVIEW_EPOCH_OFFSET
        ts_bytes = struct.pack(">q", lv_seconds) + struct.pack(">Q", 0)
        result = _convert_lv_time_us(ts_bytes[:8], ts_bytes[8:])
        assert result == 1_700_000_000_000_000


class TestCleanEventText:
    def test_strips_xml_tags(self) -> None:
        text = "<TagVersion>1<time>12345<message> Hello world"
        assert _clean_event_text(text) == "Hello world"

    def test_strips_multiple_tags(self) -> None:
        text = "<Code>123<flags>0x0<message> Error occurred <details> some detail"
        result = _clean_event_text(text)
        assert "Error occurred" in result
        assert "<Code>" not in result

    def test_plain_text_unchanged(self) -> None:
        assert _clean_event_text("  Simple text  ") == "Simple text"


# ---------------------------------------------------------------------------
# Integration tests: read_dslog
# ---------------------------------------------------------------------------

class TestReadDslog:
    def _write_dslog(self, tmp_dir: Path, records: list[bytes], unix_start: float = 1_700_000_000.0) -> Path:
        """Write a synthetic .dslog to a temp file."""
        header = _make_dslog_header(unix_start)
        path = tmp_dir / "test.dslog"
        path.write_bytes(header + b"".join(records))
        return path

    def test_empty_log(self) -> None:
        """A dslog with only a header produces signals with zero values."""
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dslog(Path(td), [])
            log_data = read_dslog(path)

            assert log_data.metadata.is_valid
            assert log_data.metadata.version == 4
            assert log_data.metadata.record_count == 0
            assert "/DSLog/BatteryVoltage" in log_data.signals

    def test_single_record_values(self) -> None:
        """Parse a single record and verify decoded values."""
        rec = _make_dslog_record(
            trip_time_raw=10,       # → 5.0
            packet_loss_raw=0,      # → 0.0
            batt_voltage_raw=3200,  # → 12.5
            cpu_raw=100,            # → 0.50
            mask=0xFF,              # all flags False
            can_raw=50,             # → 0.25
            wifi_db_raw=80,         # → 40.0
            wifi_mb_raw=512,        # → 2.0
        )

        with tempfile.TemporaryDirectory() as td:
            path = self._write_dslog(Path(td), [rec])
            log_data = read_dslog(path)

            assert log_data.metadata.record_count == 1

            batt = log_data.signals["/DSLog/BatteryVoltage"]
            assert len(batt.values) == 1
            assert abs(batt.values[0].value - 12.5) < 0.01

            trip = log_data.signals["/DSLog/TripTimeMS"]
            assert abs(trip.values[0].value - 5.0) < 0.01

            cpu = log_data.signals["/DSLog/CPUUtilization"]
            assert abs(cpu.values[0].value - 0.50) < 0.01

            can = log_data.signals["/DSLog/CANUtilization"]
            assert abs(can.values[0].value - 0.25) < 0.01

    def test_status_mask_all_false(self) -> None:
        """mask = 0xFF → all bits set → all active-low conditions are False."""
        rec = _make_dslog_record(mask=0xFF)
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dslog(Path(td), [rec])
            log_data = read_dslog(path)

            assert log_data.signals["/DSLog/Status/Brownout"].values[0].value is False
            assert log_data.signals["/DSLog/Status/Watchdog"].values[0].value is False
            assert log_data.signals["/DSLog/Status/DSTeleop"].values[0].value is False
            assert log_data.signals["/DSLog/Status/DSDisabled"].values[0].value is False
            assert log_data.signals["/DSLog/Status/RobotTeleop"].values[0].value is False
            assert log_data.signals["/DSLog/Status/RobotAuto"].values[0].value is False
            assert log_data.signals["/DSLog/Status/RobotDisabled"].values[0].value is False

    def test_status_mask_all_true(self) -> None:
        """mask = 0x00 → all bits clear → all active-low conditions are True."""
        rec = _make_dslog_record(mask=0x00)
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dslog(Path(td), [rec])
            log_data = read_dslog(path)

            assert log_data.signals["/DSLog/Status/Brownout"].values[0].value is True
            assert log_data.signals["/DSLog/Status/Watchdog"].values[0].value is True
            assert log_data.signals["/DSLog/Status/DSTeleop"].values[0].value is True
            assert log_data.signals["/DSLog/Status/DSDisabled"].values[0].value is True
            assert log_data.signals["/DSLog/Status/RobotTeleop"].values[0].value is True
            assert log_data.signals["/DSLog/Status/RobotAuto"].values[0].value is True
            assert log_data.signals["/DSLog/Status/RobotDisabled"].values[0].value is True

    def test_status_mask_brownout_only(self) -> None:
        """mask = 0x7F → only bit 7 cleared → brownout True, rest False."""
        rec = _make_dslog_record(mask=0x7F)
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dslog(Path(td), [rec])
            log_data = read_dslog(path)

            assert log_data.signals["/DSLog/Status/Brownout"].values[0].value is True
            assert log_data.signals["/DSLog/Status/Watchdog"].values[0].value is False

    def test_multiple_records_timestamps(self) -> None:
        """Multiple records get correct 20 ms period timestamps."""
        recs = [_make_dslog_record() for _ in range(5)]
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dslog(Path(td), recs, unix_start=1_700_000_000.0)
            log_data = read_dslog(path)

            assert log_data.metadata.record_count == 5
            batt = log_data.signals["/DSLog/BatteryVoltage"]
            ts = [v.timestamp_us for v in batt.values]
            # Verify 20 ms spacing
            for i in range(1, len(ts)):
                assert ts[i] - ts[i - 1] == 20_000

    def test_battery_voltage_spike_guard(self) -> None:
        """Battery voltage > 20 V is replaced with previous valid reading."""
        recs = [
            _make_dslog_record(batt_voltage_raw=3200),   # 12.5 V
            _make_dslog_record(batt_voltage_raw=6000),   # 23.4 V → should use 12.5
        ]
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dslog(Path(td), recs)
            log_data = read_dslog(path)

            batt = log_data.signals["/DSLog/BatteryVoltage"]
            assert abs(batt.values[0].value - 12.5) < 0.01
            assert abs(batt.values[1].value - 12.5) < 0.01  # guarded

    def test_packet_loss_clamped(self) -> None:
        """Packet loss is clamped to [0, 1]."""
        recs = [
            _make_dslog_record(packet_loss_raw=127),  # 127 × 4 × 0.01 = 5.08 → clamped to 1.0
            _make_dslog_record(packet_loss_raw=-128),  # -128 × 4 × 0.01 = -5.12 → clamped to 0.0
        ]
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dslog(Path(td), recs)
            log_data = read_dslog(path)

            loss = log_data.signals["/DSLog/PacketLoss"]
            assert loss.values[0].value == 1.0
            assert loss.values[1].value == 0.0

    def test_unsupported_version(self) -> None:
        """Non-version-4 files raise ValueError."""
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "bad.dslog"
            path.write_bytes(struct.pack(">i", 3) + b"\x00" * 16)
            with pytest.raises(ValueError, match="Unsupported DSLog version"):
                read_dslog(path)

    def test_file_not_found(self) -> None:
        with pytest.raises(FileNotFoundError):
            read_dslog("/nonexistent/test.dslog")

    def test_signal_types(self) -> None:
        """Verify signal types are correct."""
        rec = _make_dslog_record()
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dslog(Path(td), [rec])
            log_data = read_dslog(path)

            assert log_data.signals["/DSLog/BatteryVoltage"].info.type == SignalType.DOUBLE
            assert log_data.signals["/DSLog/Status/Brownout"].info.type == SignalType.BOOLEAN
            assert log_data.signals["/DSLog/PowerDistributionCurrents"].info.type == SignalType.DOUBLE_ARRAY

    def test_no_pd_currents_when_none(self) -> None:
        """PD type NONE produces empty current arrays."""
        rec = _make_dslog_record(pd_type=_PDType.NONE)
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dslog(Path(td), [rec])
            log_data = read_dslog(path)

            pd = log_data.signals["/DSLog/PowerDistributionCurrents"]
            assert pd.values[0].value == []


# ---------------------------------------------------------------------------
# Integration tests: read_dsevents
# ---------------------------------------------------------------------------

class TestReadDsevents:
    def _write_dsevents(
        self, tmp_dir: Path, events: list[tuple[float, str]], unix_start: float = 1_700_000_000.0
    ) -> Path:
        """Write a synthetic .dsevents to a temp file."""
        header = _make_dslog_header(unix_start)  # same header format
        records = b"".join(_make_dsevents_record(ts, text) for ts, text in events)
        path = tmp_dir / "test.dsevents"
        path.write_bytes(header + records)
        return path

    def test_empty_events(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dsevents(Path(td), [])
            log_data = read_dsevents(path)

            assert log_data.metadata.is_valid
            assert log_data.metadata.record_count == 0
            assert "/DSEvents" in log_data.signals
            assert len(log_data.signals["/DSEvents"].values) == 0

    def test_single_event(self) -> None:
        start = 1_700_000_000.0
        event_time = start + 5.0
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dsevents(Path(td), [(event_time, "Teleop started")], start)
            log_data = read_dsevents(path)

            assert log_data.metadata.record_count == 1
            events = log_data.signals["/DSEvents"]
            assert len(events.values) == 1
            assert events.values[0].value == "Teleop started"
            # Relative timestamp should be ~5 seconds = 5_000_000 µs
            assert abs(events.values[0].timestamp_us - 5_000_000) < 10_000

    def test_xml_tag_stripping(self) -> None:
        start = 1_700_000_000.0
        text = "<TagVersion>1<time>12345<message> Robot enabled"
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dsevents(Path(td), [(start + 1.0, text)], start)
            log_data = read_dsevents(path)

            events = log_data.signals["/DSEvents"]
            assert events.values[0].value == "Robot enabled"

    def test_multiple_events_ordering(self) -> None:
        start = 1_700_000_000.0
        events_data = [
            (start + 1.0, "Event A"),
            (start + 2.0, "Event B"),
            (start + 3.0, "Event C"),
        ]
        with tempfile.TemporaryDirectory() as td:
            path = self._write_dsevents(Path(td), events_data, start)
            log_data = read_dsevents(path)

            events = log_data.signals["/DSEvents"]
            assert len(events.values) == 3
            assert events.values[0].value == "Event A"
            assert events.values[2].value == "Event C"
            # Verify timestamps are increasing
            for i in range(1, 3):
                assert events.values[i].timestamp_us > events.values[i - 1].timestamp_us

    def test_unsupported_version(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "bad.dsevents"
            path.write_bytes(struct.pack(">i", 5) + b"\x00" * 16)
            with pytest.raises(ValueError, match="Unsupported DSEvents version"):
                read_dsevents(path)


# ---------------------------------------------------------------------------
# Integration tests: read_ds_logs
# ---------------------------------------------------------------------------

class TestReadDsLogs:
    def test_dslog_with_auto_discover_dsevents(self) -> None:
        """read_ds_logs auto-discovers matching .dsevents file."""
        start = 1_700_000_000.0
        with tempfile.TemporaryDirectory() as td:
            td_path = Path(td)

            # Write .dslog
            header = _make_dslog_header(start)
            rec = _make_dslog_record()
            (td_path / "match.dslog").write_bytes(header + rec)

            # Write matching .dsevents
            event_rec = _make_dsevents_record(start + 1.0, "Enabled")
            (td_path / "match.dsevents").write_bytes(header + event_rec)

            log_data = read_ds_logs(td_path / "match.dslog")

            # Should have both dslog signals and dsevents
            assert "/DSLog/BatteryVoltage" in log_data.signals
            assert "/DSEvents" in log_data.signals
            assert log_data.signals["/DSEvents"].values[0].value == "Enabled"

    def test_dslog_without_dsevents(self) -> None:
        """read_ds_logs works fine with only a .dslog."""
        start = 1_700_000_000.0
        with tempfile.TemporaryDirectory() as td:
            td_path = Path(td)
            header = _make_dslog_header(start)
            rec = _make_dslog_record()
            (td_path / "match.dslog").write_bytes(header + rec)

            log_data = read_ds_logs(td_path / "match.dslog")

            assert "/DSLog/BatteryVoltage" in log_data.signals
            assert "/DSEvents" not in log_data.signals

    def test_dslog_with_explicit_dsevents(self) -> None:
        """read_ds_logs with explicit dsevents path."""
        start = 1_700_000_000.0
        with tempfile.TemporaryDirectory() as td:
            td_path = Path(td)

            header = _make_dslog_header(start)
            rec = _make_dslog_record()
            (td_path / "match.dslog").write_bytes(header + rec)

            event_rec = _make_dsevents_record(start + 2.0, "Auto started")
            (td_path / "events_other.dsevents").write_bytes(header + event_rec)

            log_data = read_ds_logs(
                td_path / "match.dslog",
                dsevents_path=td_path / "events_other.dsevents",
            )

            assert "/DSEvents" in log_data.signals
            assert log_data.signals["/DSEvents"].values[0].value == "Auto started"
