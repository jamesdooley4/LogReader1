"""Tests for the log_correlator module."""

from __future__ import annotations

import struct
import tempfile
from pathlib import Path

import pytest

from logreader.dslog_reader import _LABVIEW_EPOCH_OFFSET, _HEADER_SIZE
from logreader.log_correlator import (
    HootFile,
    MatchLogSet,
    TimeAlignment,
    _align_mode_edges,
    _classify_hoot,
    _dslog_header_unix_s,
    _extract_mode_edges,
    _extract_wpilog_time_offset,
    _hoot_filename_ts,
    _prefix_signals,
    _wpilog_filename_ts,
    compute_alignments,
    find_matching_logs,
    merge_logs,
    scan_log_files,
)
from logreader.models import (
    LogData,
    LogMetadata,
    SignalData,
    SignalInfo,
    SignalType,
    TimestampedValue,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_dslog_header(unix_seconds: float) -> bytes:
    """Build a 20-byte dslog header for a given UNIX time."""
    lv_secs = int(unix_seconds) + _LABVIEW_EPOCH_OFFSET
    return struct.pack(">i", 4) + struct.pack(">q", lv_secs) + struct.pack(">Q", 0)


def _make_signal(name: str, sig_type: SignalType, values: list) -> SignalData:
    """Create a SignalData with timestamped values."""
    info = SignalInfo(entry_id=0, name=name, type=sig_type)
    tvs = [TimestampedValue(ts, val) for ts, val in values]
    return SignalData(info=info, values=tvs)


def _make_log_data(signals: dict[str, SignalData], file_path: str = "test") -> LogData:
    return LogData(
        metadata=LogMetadata(file_path=file_path, is_valid=True),
        signals=signals,
    )


# ---------------------------------------------------------------------------
# Tests: filename parsing
# ---------------------------------------------------------------------------


class TestFilenameParsing:
    def test_wpilog_filename_ts(self) -> None:
        p = Path("FRC_20260404_193538.wpilog")
        ts = _wpilog_filename_ts(p)
        assert ts is not None
        # 2026-04-04 19:35:38 UTC
        assert abs(ts - 1775331338.0) < 2

    def test_wpilog_no_match(self) -> None:
        assert _wpilog_filename_ts(Path("random.wpilog")) is None

    def test_hoot_rio_ts(self) -> None:
        p = Path("rio_2026-04-05_00-28-37.hoot")
        ts = _hoot_filename_ts(p)
        assert ts is not None
        # 2026-04-05 00:28:37 UTC — just verify it's in the right ballpark
        assert abs(ts - 1775348917.0) < 2

    def test_hoot_canivore_ts(self) -> None:
        p = Path("ECA0983C3353385320202034170A03FF_2026-04-05_00-28-37.hoot")
        ts = _hoot_filename_ts(p)
        assert ts is not None

    def test_hoot_unknown(self) -> None:
        assert _hoot_filename_ts(Path("weird.hoot")) is None


class TestClassifyHoot:
    def test_rio(self) -> None:
        hf = _classify_hoot(Path("rio_2026-04-05_00-28-37.hoot"))
        assert hf.bus_name == "rio"

    def test_canivore(self) -> None:
        hf = _classify_hoot(Path("ECA0983C3353385320202034170A03FF_2026-04-05_00-28-37.hoot"))
        assert hf.bus_name == "ECA0983C3353385320202034170A03FF"

    def test_unknown(self) -> None:
        hf = _classify_hoot(Path("weird_2026-04-05.hoot"))
        assert hf.bus_name == "weird"


class TestDslogHeaderUnix:
    def test_reads_header(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "test.dslog"
            path.write_bytes(_make_dslog_header(1775331324.0) + b"\x00" * 100)
            ts = _dslog_header_unix_s(path)
            assert ts is not None
            assert abs(ts - 1775331324.0) < 1

    def test_wrong_version(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "test.dslog"
            path.write_bytes(struct.pack(">i", 3) + b"\x00" * 16)
            assert _dslog_header_unix_s(path) is None

    def test_too_small(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "test.dslog"
            path.write_bytes(b"\x00" * 10)
            assert _dslog_header_unix_s(path) is None


# ---------------------------------------------------------------------------
# Tests: mode-edge extraction
# ---------------------------------------------------------------------------


class TestExtractModeEdges:
    def test_wpilog_ds_enabled(self) -> None:
        sig = _make_signal("DS:enabled", SignalType.BOOLEAN, [
            (100_000, False),
            (200_000, True),
            (500_000, False),
            (600_000, True),
        ])
        log_data = _make_log_data({"DS:enabled": sig})
        edges = _extract_mode_edges(log_data)
        assert len(edges) == 4
        assert edges[0] == (100_000, False)
        assert edges[1] == (200_000, True)

    def test_dslog_disabled_inverted(self) -> None:
        sig = _make_signal("/DSLog/Status/DSDisabled", SignalType.BOOLEAN, [
            (100_000, True),   # disabled → enabled=False
            (200_000, False),  # not disabled → enabled=True
            (500_000, True),
        ])
        log_data = _make_log_data({"/DSLog/Status/DSDisabled": sig})
        edges = _extract_mode_edges(log_data, dslog_mode=True)
        assert len(edges) == 3
        assert edges[0] == (100_000, False)  # inverted
        assert edges[1] == (200_000, True)   # inverted

    def test_no_signal(self) -> None:
        log_data = _make_log_data({})
        edges = _extract_mode_edges(log_data)
        assert edges == []


# ---------------------------------------------------------------------------
# Tests: mode-edge alignment
# ---------------------------------------------------------------------------


class TestAlignModeEdges:
    def test_perfect_alignment(self) -> None:
        edges_a = [(100, True), (500, False), (1000, True)]
        edges_b = [(100, True), (500, False), (1000, True)]
        result = _align_mode_edges(edges_a, edges_b)
        assert result is not None
        offset, confidence, residual = result
        assert offset == 0
        assert confidence >= 0.5

    def test_offset_detection(self) -> None:
        # edges_b are 1000 µs behind edges_a
        edges_a = [(1100, True), (1500, False), (2000, True)]
        edges_b = [(100, True), (500, False), (1000, True)]
        result = _align_mode_edges(edges_a, edges_b)
        assert result is not None
        offset, confidence, residual = result
        assert offset == 1000  # need to add 1000 to b to match a

    def test_no_enable_edges(self) -> None:
        edges_a = [(100, False)]
        edges_b = [(200, False)]
        assert _align_mode_edges(edges_a, edges_b) is None

    def test_empty(self) -> None:
        assert _align_mode_edges([], []) is None


# ---------------------------------------------------------------------------
# Tests: systemTime extraction
# ---------------------------------------------------------------------------


class TestExtractWpilogTimeOffset:
    def test_with_system_time(self) -> None:
        sig = _make_signal("systemTime", SignalType.INTEGER, [
            (10_000, 1_000_010_000),
            (20_000, 1_000_020_000),
            (30_000, 1_000_030_000),
        ])
        log_data = _make_log_data({"systemTime": sig})
        offset = _extract_wpilog_time_offset(log_data)
        assert offset is not None
        assert offset == 1_000_000_000  # 1_000_010_000 - 10_000 = 1_000_000_000

    def test_without_system_time(self) -> None:
        log_data = _make_log_data({})
        assert _extract_wpilog_time_offset(log_data) is None


# ---------------------------------------------------------------------------
# Tests: compute_alignments
# ---------------------------------------------------------------------------


class TestComputeAlignments:
    def test_system_time_alignment(self) -> None:
        """When systemTime exists, alignment uses it."""
        wpilog = _make_log_data({
            "systemTime": _make_signal("systemTime", SignalType.INTEGER, [
                (10_000_000, 1_775_000_010_000_000),
            ]),
            "DS:enabled": _make_signal("DS:enabled", SignalType.BOOLEAN, [
                (5_000_000, False),
                (20_000_000, True),
            ]),
        })
        dslog = _make_log_data({
            "/DSLog/Status/DSDisabled": _make_signal(
                "/DSLog/Status/DSDisabled", SignalType.BOOLEAN, [
                    (1_775_000_005_000_000, True),
                    (1_775_000_020_000_000, False),
                ]
            ),
        })

        alignments = compute_alignments(wpilog, dslog)
        assert len(alignments) == 1
        a = alignments[0]
        assert a.method == "system_time"
        assert a.source_a == "wpilog"
        assert a.source_b == "dslog"
        # offset should be ~1_775_000_000_000_000
        assert abs(a.offset_us - 1_775_000_000_000_000) < 1_000_000

    def test_no_wpilog(self) -> None:
        dslog = _make_log_data({})
        alignments = compute_alignments(None, dslog)
        assert alignments == []


# ---------------------------------------------------------------------------
# Tests: signal prefixing
# ---------------------------------------------------------------------------


class TestPrefixSignals:
    def test_basic_prefix(self) -> None:
        sig = _make_signal("/Voltage", SignalType.DOUBLE, [(100, 12.5)])
        log_data = _make_log_data({"/Voltage": sig})
        result = _prefix_signals(log_data, "/Hoot/Rio")
        assert "/Hoot/Rio/Voltage" in result
        assert result["/Hoot/Rio/Voltage"].values[0].value == 12.5

    def test_offset_applied(self) -> None:
        sig = _make_signal("/Speed", SignalType.DOUBLE, [(1000, 5.0)])
        log_data = _make_log_data({"/Speed": sig})
        result = _prefix_signals(log_data, "/Hoot/Rio", offset_us=500)
        assert result["/Hoot/Rio/Speed"].values[0].timestamp_us == 1500

    def test_zero_offset_no_copy(self) -> None:
        sig = _make_signal("/X", SignalType.DOUBLE, [(1, 2.0)])
        log_data = _make_log_data({"/X": sig})
        result = _prefix_signals(log_data, "/P", offset_us=0)
        # With zero offset, values list is shared (no copy)
        assert result["/P/X"].values is sig.values

    def test_already_prefixed(self) -> None:
        """DSLog signals already have their prefix and should keep it."""
        sig = _make_signal("/DSLog/BatteryVoltage", SignalType.DOUBLE, [(1, 12.0)])
        log_data = _make_log_data({"/DSLog/BatteryVoltage": sig})
        result = _prefix_signals(log_data, "/DSLog")
        assert "/DSLog/BatteryVoltage" in result


# ---------------------------------------------------------------------------
# Tests: file discovery
# ---------------------------------------------------------------------------


class TestScanLogFiles:
    def test_finds_files_by_extension(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / "test.wpilog").touch()
            (root / "test.dslog").touch()
            (root / "test.dsevents").touch()
            subdir = root / "hoots"
            subdir.mkdir()
            (subdir / "rio_2026-01-01_00-00-00.hoot").touch()

            result = scan_log_files(root)
            assert len(result["wpilog"]) == 1
            assert len(result["dslog"]) == 1
            assert len(result["dsevents"]) == 1
            assert len(result["hoot"]) == 1

    def test_ignores_other_files(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            (Path(td) / "readme.txt").touch()
            (Path(td) / "data.csv").touch()
            result = scan_log_files(td)
            assert all(len(v) == 0 for v in result.values())


class TestFindMatchingLogs:
    def test_groups_by_proximity(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)

            # Create a dslog (with header for absolute time) and wpilog
            # that should be in the same group (~19:35 UTC)
            dslog_path = root / "2026_04_04 12_35_24 Sat.dslog"
            dslog_path.write_bytes(
                _make_dslog_header(1775331324.0)  # 2026-04-04 19:35:24 UTC
                + b"\x00" * 100
            )

            wpilog_path = root / "FRC_20260404_193538.wpilog"
            wpilog_path.write_bytes(b"\x00" * 100)  # not a valid wpilog, but fine for discovery

            result = find_matching_logs(root, match_window_s=120)
            assert len(result) >= 1
            ls = result[0]
            assert ls.dslog_path is not None
            assert ls.wpilog_path is not None

    def test_auto_pairs_dsevents(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)

            dslog_path = root / "2026_04_04 12_35_24 Sat.dslog"
            dslog_path.write_bytes(_make_dslog_header(1775331324.0) + b"\x00" * 100)

            dsevents_path = root / "2026_04_04 12_35_24 Sat.dsevents"
            dsevents_path.write_bytes(b"\x00" * 100)

            result = find_matching_logs(root)
            assert len(result) >= 1
            assert result[0].dsevents_path is not None

    def test_hoot_files_in_subdirectory(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)

            # Create hoot subdir with rio and canivore hoots
            subdir = root / "2026-04-04_19-35-30"
            subdir.mkdir()
            (subdir / "rio_2026-04-04_19-35-32.hoot").touch()
            (subdir / "ECA0983C3353385320202034170A03FF_2026-04-04_19-35-32.hoot").touch()

            result = find_matching_logs(root)
            assert len(result) >= 1
            ls = result[0]
            assert len(ls.hoot_files) == 2
            bus_names = {h.bus_name for h in ls.hoot_files}
            assert "rio" in bus_names
            assert "ECA0983C3353385320202034170A03FF" in bus_names

    def test_separate_groups(self) -> None:
        """Files >2 min apart should be in separate groups."""
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)

            dslog1 = root / "2026_04_04 12_00_00 Sat.dslog"
            dslog1.write_bytes(_make_dslog_header(1775328000.0) + b"\x00" * 100)

            dslog2 = root / "2026_04_04 12_30_00 Sat.dslog"
            dslog2.write_bytes(_make_dslog_header(1775329800.0) + b"\x00" * 100)

            result = find_matching_logs(root, match_window_s=120)
            assert len(result) == 2


# ---------------------------------------------------------------------------
# Tests: merge_logs
# ---------------------------------------------------------------------------


class TestMergeLogs:
    def test_wpilog_only(self) -> None:
        wpilog = _make_log_data({
            "/Speed": _make_signal("/Speed", SignalType.DOUBLE, [(100, 5.0)]),
        }, file_path="test.wpilog")

        ls = MatchLogSet(wpilog_path=Path("test.wpilog"), match_label="test")
        result = merge_logs(ls, wpilog_data=wpilog)
        assert result.merged is not None
        assert "/Speed" in result.merged.signals

    def test_dslog_merged_with_offset(self) -> None:
        """DSLog signals are shifted by the alignment offset."""
        wpilog = _make_log_data({
            "systemTime": _make_signal("systemTime", SignalType.INTEGER, [
                (10_000, 1_000_010_000),
            ]),
        })
        dslog = _make_log_data({
            "/DSLog/BatteryVoltage": _make_signal(
                "/DSLog/BatteryVoltage", SignalType.DOUBLE,
                [(1_000_500_000, 12.5)],
            ),
        })

        ls = MatchLogSet(
            wpilog_path=Path("test.wpilog"),
            dslog_path=Path("test.dslog"),
            match_label="test",
        )
        result = merge_logs(ls, wpilog_data=wpilog, dslog_data=dslog)
        assert result.merged is not None
        assert "/DSLog/BatteryVoltage" in result.merged.signals

        # The dslog voltage should be time-shifted
        batt = result.merged.signals["/DSLog/BatteryVoltage"]
        assert len(batt.values) == 1
        # Original: 1_000_500_000, offset=1_000_000_000
        # Shifted: 1_000_500_000 - 1_000_000_000 = 500_000
        assert batt.values[0].timestamp_us == 500_000
        assert batt.values[0].value == 12.5

    def test_empty_log_set(self) -> None:
        ls = MatchLogSet(match_label="empty")
        result = merge_logs(ls)
        assert result.merged is not None
        assert len(result.merged.signals) == 0
