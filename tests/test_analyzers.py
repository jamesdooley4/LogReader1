"""Tests for the analyzer framework and built-in analyzers."""

from __future__ import annotations

from logreader.analyzers import list_analyzers, get_analyzer
from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer, _make_json_safe
from logreader.analyzers.pdh_power import PdhPowerAnalyzer
from logreader.analyzers.launch_counter import (
    LaunchCounterAnalyzer,
    LaunchEvent,
    detect_launches,
)
from logreader.analyzers.match_phases import (
    MatchPhase,
    MatchPhasesAnalyzer,
    MatchPhaseTimeline,
    PhaseInterval,
    classify_events_by_phase,
    detect_match_phases,
    phase_durations,
    slice_signal_by_phase,
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
    assert result.rows[0]["Min Watts"] == 120.0
    assert result.rows[0]["Avg Amps"] == 10.0
    assert result.rows[0]["Min Amps"] == 10.0

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


def test_pdh_power_negative_current() -> None:
    """Negative current (regenerative braking) should produce negative watts."""
    meta = LogMetadata(file_path="test.wpilog", is_valid=True)
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
    # Ch0: mix of positive and negative current (motor + regen)
    ch0 = SignalData(
        info=SignalInfo(entry_id=1, name=_PD_PREFIX + "Chan0", type=SignalType.DOUBLE),
        values=[
            TimestampedValue(1_000_000, 10.0),  # 120 W
            TimestampedValue(2_000_000, -3.0),  # -36 W (regenerative)
            TimestampedValue(3_000_000, 5.0),  # 60 W
        ],
    )
    signals = {s.info.name: s for s in [voltage, ch0]}
    ld = LogData(metadata=meta, signals=signals)

    result = PdhPowerAnalyzer().run(ld)
    row = result.rows[0]
    assert row["Channel"] == "Ch0"
    assert row["Peak Watts"] == 120.0
    assert row["Min Watts"] == -36.0
    assert row["Peak Amps"] == 10.0
    assert row["Min Amps"] == -3.0
    # avg = (120 + -36 + 60) / 3 = 48.0
    assert row["Avg Watts"] == 48.0


def _make_pd_log_with_phases() -> LogData:
    """Build a LogData with PDH channels and DS mode signals for phases."""
    meta = LogMetadata(file_path="test.wpilog", is_valid=True)

    # Timeline:  0-2s disabled, 2-4s auto, 4-8s teleop, 8-10s disabled
    # Voltage: 12.0 V throughout
    voltage = SignalData(
        info=SignalInfo(
            entry_id=100, name=_PD_PREFIX + "Voltage", type=SignalType.DOUBLE
        ),
        values=[TimestampedValue(i * 1_000_000, 12.0) for i in range(11)],
    )

    # Ch0: 5A during auto (2-4s), 10A during teleop (4-8s), -2A during
    # disabled (0-2s, 8-10s) to simulate regenerative braking.
    ch0_values = []
    for i in range(11):
        t_us = i * 1_000_000
        if i < 2 or i >= 8:
            ch0_values.append(TimestampedValue(t_us, -2.0))
        elif i < 4:
            ch0_values.append(TimestampedValue(t_us, 5.0))
        else:
            ch0_values.append(TimestampedValue(t_us, 10.0))
    ch0 = SignalData(
        info=SignalInfo(entry_id=1, name=_PD_PREFIX + "Chan0", type=SignalType.DOUBLE),
        values=ch0_values,
    )

    # DS mode signals (boolean)
    ds_auto = SignalData(
        info=SignalInfo(entry_id=200, name="DS:autonomous", type=SignalType.BOOLEAN),
        values=[
            TimestampedValue(0, False),
            TimestampedValue(2_000_000, True),
            TimestampedValue(4_000_000, False),
        ],
    )
    ds_teleop = SignalData(
        info=SignalInfo(entry_id=201, name="DS:teleop", type=SignalType.BOOLEAN),
        values=[
            TimestampedValue(0, False),
            TimestampedValue(4_000_000, True),
            TimestampedValue(8_000_000, False),
        ],
    )

    signals = {s.info.name: s for s in [voltage, ch0, ds_auto, ds_teleop]}
    return LogData(metadata=meta, signals=signals)


def test_pdh_power_per_phase_breakdown() -> None:
    """PDH analyzer should include per-phase stats when mode signals exist."""
    ld = _make_pd_log_with_phases()
    result = PdhPowerAnalyzer().run(ld)

    assert result.extra["phase_stats"] is not None
    phase_stats = result.extra["phase_stats"]

    # Auto phase: samples at t=2,3 → 5A × 12V = 60W each
    assert phase_stats["auto"]["avg"] == 60.0
    assert phase_stats["auto"]["peak"] == 60.0
    assert phase_stats["auto"]["min"] == 60.0

    # Teleop phase: samples at t=4,5,6,7 → 10A × 12V = 120W each
    assert phase_stats["teleop"]["avg"] == 120.0
    assert phase_stats["teleop"]["peak"] == 120.0

    # Disabled phase: samples at t=0,1,8,9,10 → -2A × 12V = -24W each
    assert phase_stats["disabled"]["avg"] == -24.0
    assert phase_stats["disabled"]["min"] == -24.0

    # Summary should mention per-phase
    assert "Per-phase power" in result.summary


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


# ── Match Phases Analyzer tests ──────────────────────────────────────────


def _make_bool_signal(name: str, transitions: list[tuple[int, bool]]) -> SignalData:
    """Build a boolean SignalData from (timestamp_us, value) pairs."""
    return SignalData(
        info=SignalInfo(entry_id=0, name=name, type=SignalType.BOOLEAN),
        values=[TimestampedValue(t, v) for t, v in transitions],
    )


def _make_match_log(
    *,
    auto_range: tuple[int, int] | None = (2_000_000, 17_000_000),
    teleop_range: tuple[int, int] | None = (17_500_000, 152_500_000),
    log_end: int = 155_000_000,
    extra_signals: dict[str, SignalData] | None = None,
) -> LogData:
    """Build a synthetic LogData with DS mode signals for a standard match.

    Parameters:
        auto_range: (start_us, end_us) for autonomous, or None.
        teleop_range: (start_us, end_us) for teleop, or None.
        log_end: Last timestamp in the log.
        extra_signals: Additional signals to include.
    """
    meta = LogMetadata(file_path="test.wpilog", is_valid=True)
    signals: dict[str, SignalData] = {}

    if auto_range is not None:
        signals["DS:autonomous"] = _make_bool_signal(
            "DS:autonomous",
            [(0, False), (auto_range[0], True), (auto_range[1], False)],
        )

    if teleop_range is not None:
        signals["DS:teleop"] = _make_bool_signal(
            "DS:teleop",
            [(0, False), (teleop_range[0], True), (teleop_range[1], False)],
        )

    # Need at least one signal that spans the full log time range so
    # _get_time_range works.
    signals["_marker"] = SignalData(
        info=SignalInfo(entry_id=999, name="_marker", type=SignalType.DOUBLE),
        values=[TimestampedValue(0, 0.0), TimestampedValue(log_end, 0.0)],
    )

    if extra_signals:
        signals.update(extra_signals)

    return LogData(metadata=meta, signals=signals)


def test_match_phases_is_registered() -> None:
    assert "match-phases" in list_analyzers()
    assert get_analyzer("match-phases") is MatchPhasesAnalyzer


def test_detect_match_phases_standard_match() -> None:
    """Standard match: disabled → auto → disabled → teleop → disabled."""
    ld = _make_match_log()
    timeline = detect_match_phases(ld)
    assert timeline is not None
    assert timeline.has_match is True
    assert timeline.appears_truncated is False
    assert timeline.appears_post_reboot is False

    # Should have: disabled, auto, disabled, teleop, disabled
    phases = [iv.phase for iv in timeline.intervals]
    assert MatchPhase.AUTONOMOUS in phases
    assert MatchPhase.TELEOP in phases

    auto = timeline.auto_interval()
    assert auto is not None
    assert auto.start_us == 2_000_000
    assert auto.end_us == 17_000_000

    teleop = timeline.teleop_interval()
    assert teleop is not None
    assert teleop.start_us == 17_500_000
    assert teleop.end_us == 152_500_000


def test_detect_match_phases_no_mode_signals() -> None:
    """No DS signals → returns None."""
    ld = LogData(
        metadata=LogMetadata(file_path="test.wpilog", is_valid=True),
        signals={
            "some/signal": SignalData(
                info=SignalInfo(entry_id=1, name="some/signal", type=SignalType.DOUBLE),
                values=[TimestampedValue(0, 1.0), TimestampedValue(1_000_000, 2.0)],
            )
        },
    )
    assert detect_match_phases(ld) is None


def test_detect_match_phases_partial_signals() -> None:
    """Only DSAuto and DSTeleop present (no DSDisabled) → infer disabled."""
    ld = _make_match_log()
    timeline = detect_match_phases(ld)
    assert timeline is not None
    # Should have disabled intervals inferred between auto and teleop
    disabled_intervals = timeline.intervals_for(MatchPhase.DISABLED)
    assert len(disabled_intervals) >= 1


def test_detect_match_phases_practice_session() -> None:
    """Practice: multiple enable/disable, no FMS → no crash, has_match False."""
    meta = LogMetadata(file_path="test.wpilog", is_valid=True)
    # Just teleop toggles, no auto at all
    signals: dict[str, SignalData] = {
        "DS:teleop": _make_bool_signal(
            "DS:teleop",
            [
                (0, False),
                (1_000_000, True),
                (5_000_000, False),
                (7_000_000, True),
                (10_000_000, False),
            ],
        ),
        "_marker": SignalData(
            info=SignalInfo(entry_id=999, name="_marker", type=SignalType.DOUBLE),
            values=[TimestampedValue(0, 0.0), TimestampedValue(12_000_000, 0.0)],
        ),
    }
    ld = LogData(metadata=meta, signals=signals)
    timeline = detect_match_phases(ld)
    assert timeline is not None
    assert timeline.has_match is False
    assert timeline.appears_truncated is False


def test_detect_match_phases_truncated() -> None:
    """Truncated match: auto + short teleop → appears_truncated True."""
    ld = _make_match_log(
        auto_range=(2_000_000, 17_000_000),
        teleop_range=(17_500_000, 47_000_000),  # only ~30 s of teleop
        log_end=48_000_000,
    )
    timeline = detect_match_phases(ld)
    assert timeline is not None
    assert timeline.has_match is True
    assert timeline.appears_truncated is True


def test_detect_match_phases_post_reboot() -> None:
    """Post-reboot file: disabled → teleop only → appears_post_reboot True."""
    ld = _make_match_log(
        auto_range=None,
        teleop_range=(2_000_000, 90_000_000),
        log_end=92_000_000,
    )
    timeline = detect_match_phases(ld)
    assert timeline is not None
    assert timeline.has_match is False  # no auto
    assert timeline.appears_post_reboot is True


def test_detect_match_phases_full_match_not_truncated() -> None:
    """Full-length match → not truncated and not post-reboot."""
    ld = _make_match_log()
    timeline = detect_match_phases(ld)
    assert timeline is not None
    assert timeline.appears_truncated is False
    assert timeline.appears_post_reboot is False


def test_phase_at() -> None:
    """phase_at should return the correct phase for timestamps."""
    ld = _make_match_log()
    timeline = detect_match_phases(ld)
    assert timeline is not None

    assert timeline.phase_at(1_000_000) == MatchPhase.DISABLED
    assert timeline.phase_at(10_000_000) == MatchPhase.AUTONOMOUS
    assert timeline.phase_at(100_000_000) == MatchPhase.TELEOP
    assert timeline.phase_at(154_000_000) == MatchPhase.DISABLED


def test_phase_durations() -> None:
    """phase_durations should sum correctly."""
    ld = _make_match_log()
    timeline = detect_match_phases(ld)
    assert timeline is not None
    durations = phase_durations(timeline)
    assert MatchPhase.AUTONOMOUS in durations
    assert durations[MatchPhase.AUTONOMOUS] == 15.0  # 17 - 2 s
    assert MatchPhase.TELEOP in durations
    assert durations[MatchPhase.TELEOP] == 135.0  # 152.5 - 17.5 s


def test_classify_events_by_phase() -> None:
    """Events should be classified into the correct phases."""
    timeline = MatchPhaseTimeline(
        intervals=[
            PhaseInterval(MatchPhase.DISABLED, 0, 2_000_000),
            PhaseInterval(MatchPhase.AUTONOMOUS, 2_000_000, 17_000_000),
            PhaseInterval(MatchPhase.DISABLED, 17_000_000, 17_500_000),
            PhaseInterval(MatchPhase.TELEOP, 17_500_000, 152_500_000),
            PhaseInterval(MatchPhase.DISABLED, 152_500_000, 155_000_000),
        ]
    )
    events = [
        (1_000_000, "pre-match"),
        (10_000_000, "auto-shot"),
        (50_000_000, "teleop-shot"),
        (153_000_000, "post-match"),
    ]
    by_phase = classify_events_by_phase(timeline, events)
    assert by_phase[MatchPhase.DISABLED] == ["pre-match", "post-match"]
    assert by_phase[MatchPhase.AUTONOMOUS] == ["auto-shot"]
    assert by_phase[MatchPhase.TELEOP] == ["teleop-shot"]


def test_classify_events_by_phase_with_grace() -> None:
    """Grace period should attribute post-phase events to the phase."""
    timeline = MatchPhaseTimeline(
        intervals=[
            PhaseInterval(MatchPhase.TELEOP, 0, 10_000_000),
            PhaseInterval(MatchPhase.DISABLED, 10_000_000, 15_000_000),
        ]
    )
    # Event at 10.5s — 0.5s after teleop ends.
    events = [(10_500_000, "spindown-launch")]

    # Without grace: should be DISABLED.
    by_phase_strict = classify_events_by_phase(timeline, events, grace_period_s=0.0)
    assert by_phase_strict.get(MatchPhase.DISABLED) == ["spindown-launch"]

    # With 1s grace: should be TELEOP.
    by_phase_grace = classify_events_by_phase(timeline, events, grace_period_s=1.0)
    assert by_phase_grace.get(MatchPhase.TELEOP) == ["spindown-launch"]


def test_classify_events_seconds() -> None:
    """classify_events_by_phase should accept timestamps in seconds."""
    timeline = MatchPhaseTimeline(
        intervals=[
            PhaseInterval(MatchPhase.AUTONOMOUS, 0, 15_000_000),
            PhaseInterval(MatchPhase.TELEOP, 15_000_000, 150_000_000),
        ]
    )
    events = [(5.0, "auto"), (100.0, "teleop")]
    by_phase = classify_events_by_phase(timeline, events, timestamps_are_seconds=True)
    assert by_phase[MatchPhase.AUTONOMOUS] == ["auto"]
    assert by_phase[MatchPhase.TELEOP] == ["teleop"]


def test_slice_signal_by_phase() -> None:
    """slice_signal_by_phase should return only in-phase samples."""
    timeline = MatchPhaseTimeline(
        intervals=[
            PhaseInterval(MatchPhase.DISABLED, 0, 2_000_000),
            PhaseInterval(MatchPhase.AUTONOMOUS, 2_000_000, 5_000_000),
            PhaseInterval(MatchPhase.TELEOP, 5_000_000, 10_000_000),
        ]
    )
    sig = SignalData(
        info=SignalInfo(entry_id=1, name="test", type=SignalType.DOUBLE),
        values=[TimestampedValue(i * 1_000_000, float(i)) for i in range(11)],
    )

    auto_sig = slice_signal_by_phase(timeline, sig, MatchPhase.AUTONOMOUS)
    auto_ts = [v.timestamp_us for v in auto_sig.values]
    assert auto_ts == [2_000_000, 3_000_000, 4_000_000]

    teleop_sig = slice_signal_by_phase(timeline, sig, MatchPhase.TELEOP)
    teleop_ts = [v.timestamp_us for v in teleop_sig.values]
    assert teleop_ts == [5_000_000, 6_000_000, 7_000_000, 8_000_000, 9_000_000]


def test_match_phases_analyzer_no_signals() -> None:
    """Analyzer reports gracefully with no mode signals."""
    ld = LogData(
        metadata=LogMetadata(file_path="test.wpilog", is_valid=True),
        signals={
            "x": SignalData(
                info=SignalInfo(entry_id=1, name="x", type=SignalType.DOUBLE),
                values=[TimestampedValue(0, 1.0)],
            )
        },
    )
    result = MatchPhasesAnalyzer().run(ld)
    assert "No Driver Station mode signals" in result.summary


def test_match_phases_analyzer_full_run() -> None:
    """Analyzer end-to-end with a standard match."""
    ld = _make_match_log()
    result = MatchPhasesAnalyzer().run(ld)
    assert result.extra["has_match"] is True
    assert "Match detected: Yes" in result.summary
    assert len(result.rows) >= 3  # at least disabled, auto, teleop
    assert result.extra["timeline"] is not None


# ── FMSControlData fallback tests ────────────────────────────────────────


def _make_fms_control_data_log(
    control_words: list[tuple[int, int]],
    log_end: int | None = None,
) -> LogData:
    """Build a LogData with only an FMSControlData integer signal.

    Parameters:
        control_words: List of (timestamp_us, control_word_int) pairs.
        log_end: Override for the last timestamp in the log.
    """
    meta = LogMetadata(file_path="test.wpilog", is_valid=True)
    fms_sig = SignalData(
        info=SignalInfo(
            entry_id=10,
            name="NT:/FMSInfo/FMSControlData",
            type=SignalType.INTEGER,
        ),
        values=[TimestampedValue(t, v) for t, v in control_words],
    )
    end = log_end or (control_words[-1][0] + 1_000_000 if control_words else 0)
    marker = SignalData(
        info=SignalInfo(entry_id=999, name="_marker", type=SignalType.DOUBLE),
        values=[TimestampedValue(0, 0.0), TimestampedValue(end, 0.0)],
    )
    return LogData(
        metadata=meta,
        signals={fms_sig.info.name: fms_sig, marker.info.name: marker},
    )


def test_fms_control_data_standard_match() -> None:
    """FMSControlData bitflags should produce a correct match timeline.

    Bit layout (from HAL_ControlWord):
      0x01=enabled, 0x02=auto, 0x04=test, 0x08=estop,
      0x10=fms_attached, 0x20=ds_attached

    Typical match sequence:
      0  → disabled (pre-match)
      51 → 0b110011 = enabled + auto + FMS + DS
      50 → 0b110010 = disabled + auto (auto→teleop gap)
      49 → 0b110001 = enabled + FMS + DS (teleop)
      48 → 0b110000 = disabled + FMS + DS (post-match)
    """
    ld = _make_fms_control_data_log(
        [
            (1_000_000, 0),  # disabled pre-match
            (2_000_000, 51),  # auto enabled
            (17_000_000, 50),  # auto disabled (gap)
            (17_500_000, 49),  # teleop enabled
            (152_500_000, 48),  # disabled post-match
        ],
        log_end=155_000_000,
    )

    timeline = detect_match_phases(ld)
    assert timeline is not None
    assert timeline.has_match is True

    auto = timeline.auto_interval()
    assert auto is not None
    assert auto.start_us == 2_000_000
    assert auto.end_us == 17_000_000

    teleop = timeline.teleop_interval()
    assert teleop is not None
    assert teleop.start_us == 17_500_000
    assert teleop.end_us == 152_500_000


def test_fms_control_data_no_boolean_signals_needed() -> None:
    """FMSControlData fallback works when no boolean DS signals exist."""
    ld = _make_fms_control_data_log(
        [
            (0, 0),
            (5_000_000, 51),  # auto
            (20_000_000, 49),  # teleop
            (100_000_000, 48),  # disabled
        ]
    )
    timeline = detect_match_phases(ld)
    assert timeline is not None
    assert timeline.has_match is True


def test_fms_control_data_disabled_only() -> None:
    """All-zero control words → single DISABLED interval."""
    ld = _make_fms_control_data_log(
        [
            (0, 0),
            (10_000_000, 0),
        ]
    )
    timeline = detect_match_phases(ld)
    assert timeline is not None
    phases = {iv.phase for iv in timeline.intervals}
    assert phases == {MatchPhase.DISABLED}


def test_fms_control_data_estop() -> None:
    """E-stop bit should produce DISABLED even if enabled bit is set."""
    # 0x09 = estop + enabled → should be DISABLED
    ld = _make_fms_control_data_log(
        [
            (0, 0),
            (5_000_000, 49),  # teleop
            (10_000_000, 9),  # estop + enabled
            (15_000_000, 48),  # disabled
        ]
    )
    timeline = detect_match_phases(ld)
    assert timeline is not None
    assert timeline.phase_at(12_000_000) == MatchPhase.DISABLED


def test_fms_control_data_test_mode() -> None:
    """Test mode bit should produce TEST phase."""
    # 0x05 = test + enabled
    ld = _make_fms_control_data_log(
        [
            (0, 0),
            (5_000_000, 5),  # test mode enabled
            (10_000_000, 0),  # disabled
        ]
    )
    timeline = detect_match_phases(ld)
    assert timeline is not None
    assert timeline.phase_at(7_000_000) == MatchPhase.TEST


def test_boolean_signals_preferred_over_fms_control_data() -> None:
    """When boolean DS signals exist, FMSControlData is ignored."""
    meta = LogMetadata(file_path="test.wpilog", is_valid=True)
    # Boolean DS signal says auto at t=5
    ds_auto = _make_bool_signal(
        "DS:autonomous",
        [
            (0, False),
            (5_000_000, True),
            (20_000_000, False),
        ],
    )
    # FMSControlData says teleop at t=5 (conflicting)
    fms_sig = SignalData(
        info=SignalInfo(
            entry_id=10,
            name="NT:/FMSInfo/FMSControlData",
            type=SignalType.INTEGER,
        ),
        values=[TimestampedValue(0, 0), TimestampedValue(5_000_000, 49)],
    )
    marker = SignalData(
        info=SignalInfo(entry_id=999, name="_marker", type=SignalType.DOUBLE),
        values=[TimestampedValue(0, 0.0), TimestampedValue(25_000_000, 0.0)],
    )
    ld = LogData(
        metadata=meta,
        signals={
            ds_auto.info.name: ds_auto,
            fms_sig.info.name: fms_sig,
            marker.info.name: marker,
        },
    )
    timeline = detect_match_phases(ld)
    assert timeline is not None
    # Boolean signal wins — phase at t=10 should be AUTO, not TELEOP.
    assert timeline.phase_at(10_000_000) == MatchPhase.AUTONOMOUS


# ── AnalysisResult.to_dict / JSON export ────────────────────────────────


def test_to_dict_basic_fields() -> None:
    result = AnalysisResult(
        analyzer_name="test",
        title="Test Result",
        summary="A summary",
        columns=["A", "B"],
        rows=[{"A": 1, "B": "x"}, {"A": 2, "B": "y"}],
    )
    d = result.to_dict()
    assert d["analyzer_name"] == "test"
    assert d["title"] == "Test Result"
    assert d["summary"] == "A summary"
    assert d["columns"] == ["A", "B"]
    assert len(d["rows"]) == 2
    assert d["rows"][0] == {"A": 1, "B": "x"}


def test_to_dict_extra_scalars() -> None:
    result = AnalysisResult(
        analyzer_name="test",
        title="T",
        extra={"count": 42, "rate": 3.14, "name": "hello", "flag": True},
    )
    d = result.to_dict()
    assert d["extra"]["count"] == 42
    assert d["extra"]["rate"] == 3.14
    assert d["extra"]["name"] == "hello"
    assert d["extra"]["flag"] is True


def test_to_dict_extra_dataclass() -> None:
    from dataclasses import dataclass

    @dataclass
    class SomeMetric:
        value: float
        label: str

    result = AnalysisResult(
        analyzer_name="test",
        title="T",
        extra={"metric": SomeMetric(value=1.5, label="speed")},
    )
    d = result.to_dict()
    assert d["extra"]["metric"] == {"value": 1.5, "label": "speed"}


def test_to_dict_skips_large_lists() -> None:
    result = AnalysisResult(
        analyzer_name="test",
        title="T",
        extra={"big": list(range(200)), "small": [1, 2, 3]},
    )
    d = result.to_dict(max_list_items=100)
    assert d["extra"]["big"]["__skipped__"] == "list too large"
    assert d["extra"]["big"]["length"] == 200
    assert d["extra"]["small"] == [1, 2, 3]


def test_to_dict_nested_dict() -> None:
    result = AnalysisResult(
        analyzer_name="test",
        title="T",
        extra={"nested": {"a": 1, "b": {"c": 2}}},
    )
    d = result.to_dict()
    assert d["extra"]["nested"]["b"]["c"] == 2


def test_to_dict_is_json_serializable() -> None:
    import json

    result = AnalysisResult(
        analyzer_name="test",
        title="T",
        summary="S",
        columns=["X"],
        rows=[{"X": 1}],
        extra={"val": 3.14, "items": [{"a": 1}, {"a": 2}]},
    )
    d = result.to_dict()
    json_str = json.dumps(d)
    parsed = json.loads(json_str)
    assert parsed["analyzer_name"] == "test"
    assert parsed["extra"]["val"] == 3.14


def test_make_json_safe_enum() -> None:
    from enum import Enum

    class Color(Enum):
        RED = "red"

    assert _make_json_safe(Color.RED, 100) == "Color.RED"


def test_make_json_safe_none() -> None:
    assert _make_json_safe(None, 100) is None
