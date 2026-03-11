"""Tests for the analyzer framework and built-in analyzers."""

from __future__ import annotations

from logreader.analyzers import list_analyzers, get_analyzer
from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer
from logreader.analyzers.pdh_power import PdhPowerAnalyzer
from logreader.models import (
    LogData,
    LogMetadata,
    SignalData,
    SignalInfo,
    SignalType,
    TimestampedValue,
)


# ── Registry tests ──────────────────────────────────────────────────────


def test_pdh_power_is_registered() -> None:
    names = list_analyzers()
    assert "pdh-power" in names


def test_get_analyzer_returns_class() -> None:
    cls = get_analyzer("pdh-power")
    assert cls is PdhPowerAnalyzer


def test_get_analyzer_unknown_raises() -> None:
    try:
        get_analyzer("no-such-analyzer")
        assert False, "Expected KeyError"
    except KeyError as exc:
        assert "no-such-analyzer" in str(exc)


def test_register_analyzer_decorator() -> None:
    @register_analyzer
    class _DummyAnalyzer(BaseAnalyzer):
        name = "_test-dummy"
        description = "test only"

        def run(self, log_data, **options):
            return AnalysisResult(analyzer_name=self.name, title="Dummy")

    assert "_test-dummy" in list_analyzers()
    assert get_analyzer("_test-dummy") is _DummyAnalyzer


# ── AnalysisResult formatting tests ─────────────────────────────────────


def test_analysis_result_format_table_empty() -> None:
    r = AnalysisResult(analyzer_name="x", title="X")
    assert r.format_table() == "(no data)"


def test_analysis_result_format_table() -> None:
    r = AnalysisResult(
        analyzer_name="x",
        title="X",
        columns=["Name", "Value"],
        rows=[
            {"Name": "A", "Value": 10},
            {"Name": "B", "Value": 200},
        ],
    )
    table = r.format_table()
    lines = table.strip().splitlines()
    assert len(lines) == 4  # header + separator + 2 rows
    assert "Name" in lines[0]
    assert "Value" in lines[0]


def test_analysis_result_format_table_max_rows() -> None:
    r = AnalysisResult(
        analyzer_name="x",
        title="X",
        columns=["V"],
        rows=[{"V": i} for i in range(10)],
    )
    table = r.format_table(max_rows=3)
    assert "7 more rows" in table


def test_analysis_result_format_report() -> None:
    r = AnalysisResult(
        analyzer_name="x",
        title="My Title",
        summary="Some summary",
        columns=["A"],
        rows=[{"A": 1}],
    )
    report = r.format_report()
    assert "=== My Title ===" in report
    assert "Some summary" in report


# ── PDH Power Analyzer tests ────────────────────────────────────────────

_PD_PREFIX = "NT:/LiveWindow/Ungrouped/PowerDistribution[0]/"


def _make_pd_log() -> LogData:
    """Build a minimal LogData with 3 PDH channels and voltage."""
    meta = LogMetadata(file_path="test.wpilog", is_valid=True)

    # Steady 12.0 V bus
    voltage = SignalData(
        info=SignalInfo(
            entry_id=100, name=_PD_PREFIX + "Voltage", type=SignalType.DOUBLE
        ),
        values=[
            TimestampedValue(1_000_000, 12.0),
            TimestampedValue(2_000_000, 12.0),
            TimestampedValue(3_000_000, 12.0),
        ],
    )

    # Ch0: high draw — 10A each sample → 120 W
    ch0 = SignalData(
        info=SignalInfo(entry_id=1, name=_PD_PREFIX + "Chan0", type=SignalType.DOUBLE),
        values=[
            TimestampedValue(1_000_000, 10.0),
            TimestampedValue(2_000_000, 10.0),
            TimestampedValue(3_000_000, 10.0),
        ],
    )

    # Ch1: low draw — 0.5A each sample → 6 W
    ch1 = SignalData(
        info=SignalInfo(entry_id=2, name=_PD_PREFIX + "Chan1", type=SignalType.DOUBLE),
        values=[
            TimestampedValue(1_000_000, 0.5),
            TimestampedValue(2_000_000, 0.5),
            TimestampedValue(3_000_000, 0.5),
        ],
    )

    # Ch2: zero draw
    ch2 = SignalData(
        info=SignalInfo(entry_id=3, name=_PD_PREFIX + "Chan2", type=SignalType.DOUBLE),
        values=[
            TimestampedValue(1_000_000, 0.0),
            TimestampedValue(2_000_000, 0.0),
            TimestampedValue(3_000_000, 0.0),
        ],
    )

    # TotalCurrent
    total = SignalData(
        info=SignalInfo(
            entry_id=50, name=_PD_PREFIX + "TotalCurrent", type=SignalType.DOUBLE
        ),
        values=[
            TimestampedValue(1_000_000, 10.5),
            TimestampedValue(2_000_000, 10.5),
            TimestampedValue(3_000_000, 10.5),
        ],
    )

    signals = {s.info.name: s for s in [voltage, ch0, ch1, ch2, total]}
    return LogData(metadata=meta, signals=signals)


def test_pdh_power_analyzer_basic() -> None:
    ld = _make_pd_log()
    analyzer = PdhPowerAnalyzer()
    result = analyzer.run(ld)

    assert result.analyzer_name == "pdh-power"
    assert len(result.rows) == 3  # 3 channels

    # Rows are sorted descending by avg watts
    assert result.rows[0]["Channel"] == "Ch0"
    assert result.rows[0]["Avg Watts"] == 120.0
    assert result.rows[0]["Peak Watts"] == 120.0
    assert result.rows[0]["Avg Amps"] == 10.0

    assert result.rows[1]["Channel"] == "Ch1"
    assert result.rows[1]["Avg Watts"] == 6.0

    assert result.rows[2]["Channel"] == "Ch2"
    assert result.rows[2]["Avg Watts"] == 0.0


def test_pdh_power_analyzer_no_pd_signals() -> None:
    ld = LogData(
        metadata=LogMetadata(file_path="empty.wpilog", is_valid=True),
        signals={
            "/Other/Stuff": SignalData(
                info=SignalInfo(
                    entry_id=1, name="/Other/Stuff", type=SignalType.STRING
                ),
                values=[TimestampedValue(1_000_000, "hello")],
            )
        },
    )
    analyzer = PdhPowerAnalyzer()
    result = analyzer.run(ld)
    assert "No PowerDistribution signals" in result.summary
    assert len(result.rows) == 0


def test_pdh_power_analyzer_voltage_stats_in_extra() -> None:
    ld = _make_pd_log()
    result = PdhPowerAnalyzer().run(ld)
    assert result.extra["avg_voltage"] == 12.0
    assert result.extra["min_voltage"] == 12.0
    assert result.extra["total_avg_watts"] == 126.0


def test_pdh_power_analyzer_report_format() -> None:
    ld = _make_pd_log()
    result = PdhPowerAnalyzer().run(ld)
    report = result.format_report()
    assert "PDH / PDP Power Analysis" in report
    assert "Ch0" in report
    assert "Ch1" in report
    assert "Ch2" in report
