"""Tests for the analyzer framework and built-in analyzers."""

from __future__ import annotations

from logreader.analyzers import list_analyzers, get_analyzer
from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer
from logreader.analyzers.pdh_power import PdhPowerAnalyzer
from logreader.analyzers.launch_counter import (
    LaunchCounterAnalyzer,
    LaunchEvent,
    detect_launches,
)
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


# ── Launch Counter Analyzer tests ────────────────────────────────────────


def _make_velocity_signal(
    name: str, time_value_pairs: list[tuple[float, float]]
) -> SignalData:
    """Helper to build a velocity SignalData from (time_s, velocity) pairs."""
    return SignalData(
        info=SignalInfo(entry_id=99, name=name, type=SignalType.DOUBLE),
        values=[TimestampedValue(int(t * 1_000_000), v) for t, v in time_value_pairs],
    )


def _cruise_then_dip(
    t_start: float, cruise: float, dip: float, dt: float = 0.02
) -> list[tuple[float, float]]:
    """Generate cruise → dip → recovery pattern starting at *t_start*.

    Returns ~10 samples: 4 at cruise, 1 dip, 5 recovery back to cruise.
    """
    t = t_start
    pts: list[tuple[float, float]] = []
    # Cruise
    for _ in range(4):
        pts.append((t, cruise))
        t += dt
    # Dip
    pts.append((t, dip))
    t += dt
    # Recovery
    for i in range(5):
        recovery_val = dip + (cruise - dip) * (i + 1) / 5
        pts.append((t, recovery_val))
        t += dt
    return pts


def test_launch_counter_is_registered() -> None:
    assert "launch-counter" in list_analyzers()
    assert get_analyzer("launch-counter") is LaunchCounterAnalyzer


def test_detect_launches_single_dip() -> None:
    """A single clean dip in a cruise should produce exactly 1 launch."""
    pts = _cruise_then_dip(1.0, cruise=75.0, dip=60.0)
    sig = _make_velocity_signal("NT:/launcher/velocity", pts)
    launches = detect_launches(sig)
    assert len(launches) == 1
    assert launches[0].dip_velocity == 60.0
    assert launches[0].drop_magnitude >= 10.0


def test_detect_launches_multiple_dips() -> None:
    """Three separated dips should produce 3 launches."""
    pts: list[tuple[float, float]] = []
    pts.extend(_cruise_then_dip(1.0, cruise=75.0, dip=62.0))
    pts.extend(_cruise_then_dip(2.0, cruise=75.0, dip=58.0))
    pts.extend(_cruise_then_dip(3.0, cruise=75.0, dip=65.0))
    sig = _make_velocity_signal("NT:/launcher/velocity", pts)
    launches = detect_launches(sig)
    assert len(launches) == 3


def test_detect_launches_no_dip() -> None:
    """Steady cruise with no dips → 0 launches."""
    pts = [(t * 0.02, 75.0) for t in range(50)]
    sig = _make_velocity_signal("NT:/launcher/velocity", pts)
    launches = detect_launches(sig)
    assert len(launches) == 0


def test_detect_launches_below_at_speed() -> None:
    """Dips while below at_speed threshold should not count."""
    pts = [(t * 0.02, 30.0 if t % 5 != 0 else 20.0) for t in range(50)]
    sig = _make_velocity_signal("NT:/launcher/velocity", pts)
    launches = detect_launches(sig)
    assert len(launches) == 0


def test_detect_launches_shutdown_not_counted() -> None:
    """A smooth coast-down to zero should produce 0 launches."""
    # Cruise at 75, then linear decline to 0 over 2 seconds
    pts: list[tuple[float, float]] = []
    for i in range(50):
        pts.append((i * 0.02, 75.0))
    for i in range(100):
        pts.append((1.0 + i * 0.02, 75.0 - i * 0.75))
    sig = _make_velocity_signal("NT:/launcher/velocity", pts)
    launches = detect_launches(sig)
    # A smooth decline has no sharp dips, so should produce 0
    assert len(launches) == 0


def test_detect_launches_rapid_burst() -> None:
    """Rapid-fire burst: dips with declining baseline should still count."""
    pts: list[tuple[float, float]] = []
    t = 0.0
    # Spin up
    for i in range(20):
        pts.append((t, 75.0))
        t += 0.02

    # Rapid burst: 4 dips, each from a lower baseline
    for shot, base in enumerate([75.0, 68.0, 60.0, 52.0]):
        dip = base - 12.0  # 12-unit drop
        # 2 samples at base, 1 at dip, 2 recovery
        pts.append((t, base))
        t += 0.02
        pts.append((t, base - 2))
        t += 0.02
        pts.append((t, dip))
        t += 0.02
        pts.append((t, dip + 6))  # partial recovery
        t += 0.02
        pts.append((t, dip + 10))  # more recovery
        t += 0.02

    sig = _make_velocity_signal("NT:/launcher/velocity", pts)
    launches = detect_launches(sig)
    # Should detect at least 3 of the 4 (last one may be at floor)
    assert len(launches) >= 3


def test_detect_launches_debounce() -> None:
    """Two dips closer than debounce_ms should only count as one."""
    pts: list[tuple[float, float]] = []
    t = 0.0
    for i in range(10):
        pts.append((t, 75.0))
        t += 0.02
    # First dip
    pts.append((t, 60.0))
    t += 0.02
    pts.append((t, 70.0))
    t += 0.02
    # Second dip only 40ms later (below 80ms debounce)
    pts.append((t, 58.0))
    t += 0.02
    pts.append((t, 70.0))
    t += 0.02
    for i in range(5):
        pts.append((t, 75.0))
        t += 0.02

    sig = _make_velocity_signal("NT:/launcher/velocity", pts)
    launches = detect_launches(sig, debounce_ms=80)
    assert len(launches) == 1


def test_launch_counter_analyzer_no_signal() -> None:
    """Analyzer should report gracefully when no velocity signal is found."""
    ld = LogData(
        metadata=LogMetadata(file_path="test.wpilog", is_valid=True),
        signals={
            "/Other/Stuff": SignalData(
                info=SignalInfo(
                    entry_id=1, name="/Other/Stuff", type=SignalType.STRING
                ),
                values=[TimestampedValue(1_000_000, "hello")],
            )
        },
    )
    result = LaunchCounterAnalyzer().run(ld)
    assert "No flywheel velocity signal" in result.summary


def test_launch_counter_analyzer_full_run() -> None:
    """Analyzer end-to-end with auto-detected signal."""
    pts: list[tuple[float, float]] = []
    pts.extend(_cruise_then_dip(1.0, cruise=80.0, dip=65.0))
    pts.extend(_cruise_then_dip(2.0, cruise=80.0, dip=60.0))

    sig = _make_velocity_signal("NT:/launcher/velocity", pts)
    ld = LogData(
        metadata=LogMetadata(file_path="test.wpilog", is_valid=True),
        signals={sig.info.name: sig},
    )

    result = LaunchCounterAnalyzer().run(ld)
    assert result.extra["total_launches"] == 2
    assert result.extra["velocity_signal"] == "NT:/launcher/velocity"
    assert "Total launches:  2" in result.summary


def test_launch_counter_detail_mode() -> None:
    """With detail=True the table should show individual launches."""
    pts = _cruise_then_dip(1.0, cruise=80.0, dip=65.0)
    sig = _make_velocity_signal("NT:/launcher/velocity", pts)
    ld = LogData(
        metadata=LogMetadata(file_path="test.wpilog", is_valid=True),
        signals={sig.info.name: sig},
    )

    result = LaunchCounterAnalyzer().run(ld, detail=True)
    assert "#" in result.columns
    assert "DipVel" in result.columns
    assert len(result.rows) == 1
