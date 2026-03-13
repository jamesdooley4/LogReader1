"""Tests for the loop-overrun analyzer.

Covers the test strategy outlined in docs/design-loop-overruns.md:

1.  No console signal — reports "unavailable" gracefully.
2.  Console with no overruns — reports zero overruns and estimated loop count.
3.  Single overrun with Tracer breakdown — parses component timings.
4.  Multi-message Tracer dump — groups split console messages into one event.
5.  Multiple overruns across phases — per-phase breakdown is correct.
6.  Phase transition clustering — detects overrun bursts near init methods.
7.  Custom loop period — parses "Loop time of 0.01s" as 10 ms.
8.  Component categorisation — correctly classifies all categories.
9.  Contribution percentage — sums to ~100% using top-level epochs only.
10. Correlation with nearby events — finds nearby log messages.
11. robotPeriodic() containment — does not double-count in contribution %.
"""

from __future__ import annotations

import pytest

from logreader.analyzers.loop_overruns import (
    ComponentCategory,
    ComponentStats,
    LoopOverrunAnalyzer,
    NearbyEvent,
    OverrunEvent,
    OverrunSummary,
    UNNAMED_COMMAND_NAMES,
    analyse_loop_overruns,
    categorise_component,
    _compute_component_stats,
    _detect_bursts,
    _detect_period,
    _group_overrun_events,
    _is_top_level_epoch,
    _parse_console_signal,
    _parse_messages_signal,
    _percentile,
    _RawConsoleMsg,
)
from logreader.models import (
    LogData,
    LogMetadata,
    SignalData,
    SignalInfo,
    SignalType,
    TimestampedValue,
)


# ── Helpers ─────────────────────────────────────────────────────────────


def _make_log(
    signals: dict[str, SignalData] | None = None,
) -> LogData:
    """Build a minimal LogData."""
    meta = LogMetadata(file_path="test.wpilog", is_valid=True)
    return LogData(metadata=meta, signals=signals or {})


def _make_console_signal(
    messages: list[tuple[int, str]],
) -> SignalData:
    """Build a console signal from ``(timestamp_us, text)`` pairs."""
    return SignalData(
        info=SignalInfo(entry_id=1, name="console", type=SignalType.STRING),
        values=[TimestampedValue(timestamp_us=ts, value=text) for ts, text in messages],
    )


def _make_messages_signal(
    messages: list[tuple[int, str]],
) -> SignalData:
    """Build a messages signal from ``(timestamp_us, text)`` pairs."""
    return SignalData(
        info=SignalInfo(entry_id=2, name="messages", type=SignalType.STRING),
        values=[TimestampedValue(timestamp_us=ts, value=text) for ts, text in messages],
    )


def _make_fms_signal(
    values: list[tuple[int, int]],
) -> SignalData:
    """Build an FMSControlData signal from ``(timestamp_us, control_word)``."""
    return SignalData(
        info=SignalInfo(
            entry_id=3,
            name="NT:/FMSInfo/FMSControlData",
            type=SignalType.INTEGER,
        ),
        values=[TimestampedValue(timestamp_us=ts, value=cw) for ts, cw in values],
    )


def _make_scheduler_names_signal(
    values: list[tuple[int, list[str]]],
) -> SignalData:
    """Build a Scheduler/Names signal from ``(timestamp_us, names_list)``."""
    return SignalData(
        info=SignalInfo(
            entry_id=4,
            name="NT:/SmartDashboard/Scheduler/Names",
            type=SignalType.STRING_ARRAY,
        ),
        values=[TimestampedValue(timestamp_us=ts, value=names) for ts, names in values],
    )


# ── 1. No console signal ───────────────────────────────────────────────


class TestNoConsoleSignal:
    def test_no_signals_at_all(self) -> None:
        ld = _make_log()
        result = LoopOverrunAnalyzer().run(ld)
        assert (
            "unavailable" in result.summary.lower()
            or "no console" in result.summary.lower()
        )

    def test_has_other_signals_but_no_console(self) -> None:
        ld = _make_log(
            signals={
                "some/signal": SignalData(
                    info=SignalInfo(
                        entry_id=10, name="some/signal", type=SignalType.DOUBLE
                    ),
                    values=[
                        TimestampedValue(1_000_000, 1.0),
                        TimestampedValue(10_000_000, 2.0),
                    ],
                )
            }
        )
        result = LoopOverrunAnalyzer().run(ld)
        assert "no console" in result.summary.lower()


# ── 2. Console with no overruns ────────────────────────────────────────


class TestNoOverruns:
    def test_console_present_but_no_overrun_messages(self) -> None:
        console = _make_console_signal(
            [
                (1_000_000, "Some informational message"),
                (5_000_000, "Another message with no timing data"),
                (10_000_000, "Final message"),
            ]
        )
        ld = _make_log(signals={"console": console})
        result = LoopOverrunAnalyzer().run(ld)

        assert "no loop overruns" in result.summary.lower()
        assert result.extra["summary"].overrun_count == 0
        assert result.extra["summary"].estimated_total_loops > 0


# ── 3. Single overrun with Tracer breakdown ────────────────────────────


class TestSingleOverrun:
    def _make_single_overrun_log(self) -> LogData:
        """A log with one overrun and its Tracer dump in a single message."""
        console = _make_console_signal(
            [
                (
                    1_000_000,
                    "Warning at edu.wpi.first.wpilibj.IterativeRobotBase"
                    ".printLoopOverrunMessage(IterativeRobotBase.java:436): "
                    "Loop time of 0.02s overrun",
                ),
                (
                    1_001_000,
                    "Warning at edu.wpi.first.wpilibj.Tracer"
                    ".lambda$printEpochs$0(Tracer.java:62):\n"
                    "  teleopPeriodic(): 0.001000s\n"
                    "  SmartDashboard.updateValues(): 0.005000s\n"
                    "  robotPeriodic(): 0.030000s\n"
                    "  LiveWindow.updateValues(): 0.002000s\n"
                    "  Shuffleboard.update(): 0.000100s",
                ),
            ]
        )
        return _make_log(signals={"console": console})

    def test_parses_one_event(self) -> None:
        ld = self._make_single_overrun_log()
        summary = analyse_loop_overruns(ld, phases=False)

        assert summary.overrun_count == 1
        assert summary.configured_period_ms == 20.0

    def test_epoch_timings_extracted(self) -> None:
        ld = self._make_single_overrun_log()
        summary = analyse_loop_overruns(ld, phases=False)
        ev = summary.overrun_events[0]

        assert "teleopPeriodic()" in ev.epoch_timings
        assert "robotPeriodic()" in ev.epoch_timings
        assert "SmartDashboard.updateValues()" in ev.epoch_timings
        assert "LiveWindow.updateValues()" in ev.epoch_timings
        assert "Shuffleboard.update()" in ev.epoch_timings

        assert ev.epoch_timings["robotPeriodic()"] == pytest.approx(0.030, abs=1e-6)

    def test_total_duration(self) -> None:
        ld = self._make_single_overrun_log()
        summary = analyse_loop_overruns(ld, phases=False)
        ev = summary.overrun_events[0]

        # 1.0 + 5.0 + 30.0 + 2.0 + 0.1 = 38.1 ms
        assert ev.total_duration_ms == pytest.approx(38.1, abs=0.1)


# ── 4. Multi-message Tracer dump ──────────────────────────────────────


class TestMultiMessageGrouping:
    def test_split_tracer_groups_into_one_event(self) -> None:
        """Tracer output split across 3 messages should form 1 event."""
        console = _make_console_signal(
            [
                (
                    1_000_000,
                    "Warning at IterativeRobotBase: Loop time of 0.02s overrun",
                ),
                (
                    1_010_000,  # 10 µs later
                    "  teleopPeriodic(): 0.001000s\n"
                    "  SmartDashboard.updateValues(): 0.005000s",
                ),
                (
                    1_020_000,  # 20 µs later
                    "  robotPeriodic(): 0.025000s\n"
                    "  LiveWindow.updateValues(): 0.002000s\n"
                    "  Shuffleboard.update(): 0.000050s",
                ),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)

        assert summary.overrun_count == 1
        ev = summary.overrun_events[0]
        assert len(ev.epoch_timings) == 5

    def test_two_overruns_not_merged(self) -> None:
        """Two separate IterativeRobotBase warnings → two events."""
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (1_010_000, "  robotPeriodic(): 0.025000s"),
                # Second overrun 2 seconds later
                (3_000_000, "Warning: Loop time of 0.02s overrun"),
                (3_010_000, "  robotPeriodic(): 0.030000s"),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)

        assert summary.overrun_count == 2
        assert summary.overrun_events[0].epoch_timings[
            "robotPeriodic()"
        ] == pytest.approx(0.025)
        assert summary.overrun_events[1].epoch_timings[
            "robotPeriodic()"
        ] == pytest.approx(0.030)

    def test_scheduler_overrun_attached(self) -> None:
        """CommandScheduler overrun marker within window attaches to event."""
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (1_010_000, "  robotPeriodic(): 0.025000s"),
                (1_015_000, "CommandScheduler loop overrun"),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)

        assert summary.overrun_count == 1
        ev = summary.overrun_events[0]
        assert ev.scheduler_overrun_markers == 1
        assert summary.scheduler_overrun_count == 1


# ── 5. Multiple overruns across phases ─────────────────────────────────


class TestMultipleOverrunsAcrossPhases:
    def _make_phased_log(self) -> LogData:
        """Log with FMS data + overruns in auto and teleop."""
        # FMS: disabled 0–10s, auto 10–25s, disabled 25–30s, teleop 30–130s
        fms = _make_fms_signal(
            [
                (0, 0x20),  # disabled, DS attached
                (10_000_000, 0x23),  # enabled + auto + DS
                (25_000_000, 0x20),  # disabled
                (30_000_000, 0x21),  # enabled (teleop)
                (130_000_000, 0x20),  # disabled
            ]
        )
        console = _make_console_signal(
            [
                # Overrun during auto at 12s
                (12_000_000, "Warning: Loop time of 0.02s overrun"),
                (
                    12_001_000,
                    "  autonomousPeriodic(): 0.005000s\n  robotPeriodic(): 0.020000s",
                ),
                # Overrun during teleop at 50s
                (50_000_000, "Warning: Loop time of 0.02s overrun"),
                (
                    50_001_000,
                    "  teleopPeriodic(): 0.003000s\n  robotPeriodic(): 0.040000s",
                ),
                # Overrun during teleop at 80s
                (80_000_000, "Warning: Loop time of 0.02s overrun"),
                (
                    80_001_000,
                    "  teleopPeriodic(): 0.002000s\n  robotPeriodic(): 0.050000s",
                ),
            ]
        )
        return _make_log(
            signals={
                "console": console,
                "NT:/FMSInfo/FMSControlData": fms,
            }
        )

    def test_phase_annotation(self) -> None:
        ld = self._make_phased_log()
        summary = analyse_loop_overruns(ld, phases=True)

        assert summary.overrun_count == 3
        phases = [e.phase for e in summary.overrun_events]
        assert phases[0] == "auto"
        assert phases[1] == "teleop"
        assert phases[2] == "teleop"

    def test_phase_stats_computed(self) -> None:
        ld = self._make_phased_log()
        summary = analyse_loop_overruns(ld, phases=True)

        assert summary.phase_stats is not None
        phase_map = {ps.phase: ps for ps in summary.phase_stats}
        assert "auto" in phase_map
        assert "teleop" in phase_map
        assert phase_map["auto"].overrun_count == 1
        assert phase_map["teleop"].overrun_count == 2


# ── 6. Phase transition clustering ────────────────────────────────────


class TestPhaseTransitionClustering:
    def test_detects_overruns_near_transition(self) -> None:
        fms = _make_fms_signal(
            [
                (0, 0x20),  # disabled
                (10_000_000, 0x23),  # auto start
                (25_000_000, 0x20),  # disabled
                (30_000_000, 0x21),  # teleop start
                (130_000_000, 0x20),  # disabled
            ]
        )
        console = _make_console_signal(
            [
                # Overrun right at auto start
                (10_100_000, "Warning: Loop time of 0.02s overrun"),
                (
                    10_101_000,
                    "  autonomousInit(): 0.120000s\n  robotPeriodic(): 0.010000s",
                ),
                # Overrun 2s after auto start
                (12_000_000, "Warning: Loop time of 0.02s overrun"),
                (12_001_000, "  robotPeriodic(): 0.025000s"),
            ]
        )
        messages = _make_messages_signal(
            [
                (10_050_000, "Command initialized: AutoShoot"),
                (10_060_000, "Command initialized: DriveForward"),
            ]
        )
        ld = _make_log(
            signals={
                "console": console,
                "messages": messages,
                "NT:/FMSInfo/FMSControlData": fms,
            }
        )
        summary = analyse_loop_overruns(ld, phases=True)

        assert summary.phase_transitions is not None
        auto_trans = [t for t in summary.phase_transitions if t.to_phase == "auto"]
        assert len(auto_trans) == 1
        assert auto_trans[0].overruns_nearby >= 2
        assert auto_trans[0].init_method_ms is not None
        assert auto_trans[0].init_method_ms == pytest.approx(120.0, abs=0.1)
        assert "AutoShoot" in auto_trans[0].commands_initialized
        assert "DriveForward" in auto_trans[0].commands_initialized


# ── 7. Custom loop period ──────────────────────────────────────────────


class TestCustomLoopPeriod:
    def test_parses_10ms_period(self) -> None:
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.01s overrun"),
                (1_001_000, "  robotPeriodic(): 0.015000s"),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)

        assert summary.configured_period_ms == pytest.approx(10.0)

    def test_parses_5ms_period(self) -> None:
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.005s overrun"),
                (1_001_000, "  robotPeriodic(): 0.008000s"),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)

        assert summary.configured_period_ms == pytest.approx(5.0)


# ── 8. Component categorisation ───────────────────────────────────────


class TestComponentCategorisation:
    @pytest.mark.parametrize(
        "name, expected",
        [
            ("robotPeriodic()", ComponentCategory.FRAMEWORK),
            ("LiveWindow.updateValues()", ComponentCategory.FRAMEWORK),
            ("SmartDashboard.updateValues()", ComponentCategory.FRAMEWORK),
            ("Shuffleboard.update()", ComponentCategory.FRAMEWORK),
            ("autonomousPeriodic()", ComponentCategory.FRAMEWORK),
            ("teleopPeriodic()", ComponentCategory.FRAMEWORK),
            ("disabledPeriodic()", ComponentCategory.FRAMEWORK),
            ("testPeriodic()", ComponentCategory.FRAMEWORK),
            ("autonomousInit()", ComponentCategory.PHASE_INIT),
            ("teleopInit()", ComponentCategory.PHASE_INIT),
            ("disabledInit()", ComponentCategory.PHASE_INIT),
            ("testInit()", ComponentCategory.PHASE_INIT),
            ("TurretSubsystem.periodic()", ComponentCategory.SUBSYSTEM),
            ("IntakeRollers.periodic()", ComponentCategory.SUBSYSTEM),
            ("Hood.periodic()", ComponentCategory.SUBSYSTEM),
            ("C-Outpost-Depot.execute()", ComponentCategory.COMMAND),
            ("Drive.execute()", ComponentCategory.COMMAND),
            ("ParallelCommandGroup.execute()", ComponentCategory.COMMAND),
            ("SomeCommand.initialize()", ComponentCategory.COMMAND),
            ("unknownThing", ComponentCategory.OTHER),
        ],
    )
    def test_categorise(self, name: str, expected: ComponentCategory) -> None:
        assert categorise_component(name) == expected


# ── 9. Contribution percentage ─────────────────────────────────────────


class TestContributionPercentage:
    def test_top_level_sums_near_100(self) -> None:
        """Top-level epoch contribution % should sum to ~100%."""
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (
                    1_001_000,
                    "  teleopPeriodic(): 0.002000s\n"
                    "  SmartDashboard.updateValues(): 0.003000s\n"
                    "  robotPeriodic(): 0.020000s\n"
                    "  LiveWindow.updateValues(): 0.004000s\n"
                    "  Shuffleboard.update(): 0.001000s",
                ),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)

        total_pct = sum(c.mean_contribution_pct for c in summary.components)
        assert total_pct == pytest.approx(100.0, abs=1.0)


# ── 10. Correlation with nearby events ────────────────────────────────


class TestNearbyEventCorrelation:
    def test_finds_datalog_event(self) -> None:
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (1_001_000, "  robotPeriodic(): 0.025000s"),
                (1_500_000, "DataLog: Renamed log file from 'TBD' to 'final.wpilog'"),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, detail=True, phases=False)

        ev = summary.overrun_events[0]
        assert len(ev.nearby_events) >= 1
        cats = [ne.category for ne in ev.nearby_events]
        assert "datalog" in cats

    def test_finds_command_events(self) -> None:
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (1_001_000, "  robotPeriodic(): 0.025000s"),
            ]
        )
        messages = _make_messages_signal(
            [
                (1_200_000, "Command initialized: AutoShoot"),
                (1_300_000, "Command interrupted: Drive; Cause: <none>"),
            ]
        )
        ld = _make_log(signals={"console": console, "messages": messages})
        summary = analyse_loop_overruns(ld, detail=True, phases=False)

        ev = summary.overrun_events[0]
        cmd_events = [ne for ne in ev.nearby_events if ne.category == "command"]
        assert len(cmd_events) == 2
        texts = [ne.text for ne in cmd_events]
        assert any("AutoShoot" in t for t in texts)
        assert any("Drive" in t for t in texts)

    def test_dynamic_window_expands(self) -> None:
        """A large overrun should expand the correlation window."""
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (1_001_000, "  robotPeriodic(): 0.600000s"),  # 600ms overrun
                # This event is 1.5s away — should be included with expanded window
                # window = 1.0 + 0.6 = 1.6s
                (2_500_000, "DataLog: Something happened"),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, detail=True, phases=False)

        ev = summary.overrun_events[0]
        datalog_events = [ne for ne in ev.nearby_events if ne.category == "datalog"]
        assert len(datalog_events) == 1

    def test_dynamic_window_excludes_distant(self) -> None:
        """Events outside the dynamic window should be excluded."""
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (1_001_000, "  robotPeriodic(): 0.025000s"),  # 25ms overrun
                # window = 1.0 + 0.025 = 1.025s → this event at 2.1s away is outside
                (3_100_000, "DataLog: Too far away"),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, detail=True, phases=False)

        ev = summary.overrun_events[0]
        assert len(ev.nearby_events) == 0


# ── 11. robotPeriodic() containment — no double-counting ─────────────


class TestRobotPeriodicContainment:
    def test_top_level_excludes_sub_components(self) -> None:
        """Top-level stats should not include sub-epoch components."""
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (
                    1_001_000,
                    "  teleopPeriodic(): 0.002000s\n"
                    "  SmartDashboard.updateValues(): 0.003000s\n"
                    "  robotPeriodic(): 0.030000s\n"
                    "  LiveWindow.updateValues(): 0.004000s\n"
                    "  Shuffleboard.update(): 0.001000s\n"
                    "  TurretSubsystem.periodic(): 0.010000s\n"
                    "  Drive.execute(): 0.015000s",
                ),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)

        # Global components should be top-level only
        comp_names = [c.name for c in summary.components]
        assert "robotPeriodic()" in comp_names
        assert "TurretSubsystem.periodic()" not in comp_names
        assert "Drive.execute()" not in comp_names

    def test_all_components_available_in_events(self) -> None:
        """Individual OverrunEvent.epoch_timings should have all components."""
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (
                    1_001_000,
                    "  robotPeriodic(): 0.030000s\n"
                    "  TurretSubsystem.periodic(): 0.010000s\n"
                    "  Drive.execute(): 0.015000s",
                ),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)

        ev = summary.overrun_events[0]
        assert "TurretSubsystem.periodic()" in ev.epoch_timings
        assert "Drive.execute()" in ev.epoch_timings


# ── Burst detection ────────────────────────────────────────────────────


class TestBurstDetection:
    def test_detects_burst(self) -> None:
        """Overruns within 2s of each other form a burst."""
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (1_001_000, "  robotPeriodic(): 0.025000s"),
                (2_000_000, "Warning: Loop time of 0.02s overrun"),
                (2_001_000, "  robotPeriodic(): 0.030000s"),
                (2_500_000, "Warning: Loop time of 0.02s overrun"),
                (2_501_000, "  robotPeriodic(): 0.028000s"),
                # 10 second gap
                (12_500_000, "Warning: Loop time of 0.02s overrun"),
                (12_501_000, "  robotPeriodic(): 0.022000s"),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)

        assert len(summary.burst_periods) == 1
        start, end, count = summary.burst_periods[0]
        assert count == 3
        assert start == pytest.approx(1.0, abs=0.01)
        assert end == pytest.approx(2.5, abs=0.01)

    def test_no_burst_with_single_overrun(self) -> None:
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (1_001_000, "  robotPeriodic(): 0.025000s"),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)
        assert len(summary.burst_periods) == 0


# ── Percentile helper ─────────────────────────────────────────────────


class TestPercentile:
    def test_single_value(self) -> None:
        assert _percentile([42.0], 50) == 42.0

    def test_two_values_median(self) -> None:
        assert _percentile([10.0, 20.0], 50) == pytest.approx(15.0)

    def test_p95_large_list(self) -> None:
        data = list(range(1, 101))  # 1..100
        result = _percentile([float(x) for x in data], 95)
        assert result == pytest.approx(95.05, abs=0.1)


# ── Period detection ──────────────────────────────────────────────────


class TestPeriodDetection:
    def test_extracts_20ms(self) -> None:
        msgs = [
            _RawConsoleMsg(1_000_000, "Warning: Loop time of 0.02s overrun"),
        ]
        assert _detect_period(msgs) == pytest.approx(0.02)

    def test_extracts_10ms(self) -> None:
        msgs = [
            _RawConsoleMsg(1_000_000, "Warning: Loop time of 0.01s overrun"),
        ]
        assert _detect_period(msgs) == pytest.approx(0.01)

    def test_default_when_no_overruns(self) -> None:
        msgs = [
            _RawConsoleMsg(1_000_000, "Some other message"),
        ]
        assert _detect_period(msgs) == pytest.approx(0.02)


# ── Analyzer registration ────────────────────────────────────────────


class TestRegistration:
    def test_registered(self) -> None:
        from logreader.analyzers import list_analyzers

        assert "loop-overruns" in list_analyzers()

    def test_get_analyzer(self) -> None:
        from logreader.analyzers import get_analyzer

        assert get_analyzer("loop-overruns") is LoopOverrunAnalyzer


# ── CLI arguments ─────────────────────────────────────────────────────


class TestCLIArguments:
    def test_add_arguments(self) -> None:
        import argparse

        parser = argparse.ArgumentParser()
        LoopOverrunAnalyzer.add_arguments(parser)
        args = parser.parse_args(
            ["--detail", "--phases", "--worst", "10", "--min-time", "5"]
        )
        assert args.detail is True
        assert args.phases is True
        assert args.worst == 10
        assert args.min_time == 5.0


# ── Messages signal parsing ──────────────────────────────────────────


class TestMessagesSignalParsing:
    def test_parses_command_events(self) -> None:
        messages = _make_messages_signal(
            [
                (1_000_000, "Command initialized: AutoShoot"),
                (2_000_000, "Command interrupted: Drive; Cause: <none>"),
                (3_000_000, "Command finished: IntakeCommand"),
            ]
        )
        ld = _make_log(signals={"messages": messages})
        events = _parse_messages_signal(ld)

        assert len(events) == 3
        assert events[0].kind == "initialized"
        assert events[0].command_name == "AutoShoot"
        assert events[1].kind == "interrupted"
        assert events[1].command_name == "Drive"
        assert events[2].kind == "finished"
        assert events[2].command_name == "IntakeCommand"

    def test_empty_messages(self) -> None:
        ld = _make_log()
        events = _parse_messages_signal(ld)
        assert events == []


# ── Duration distribution ────────────────────────────────────────────


class TestDurationDistribution:
    def test_distribution_with_multiple_events(self) -> None:
        console = _make_console_signal(
            [
                (1_000_000, "Warning: Loop time of 0.02s overrun"),
                (1_001_000, "  robotPeriodic(): 0.025000s"),
                (2_000_000, "Warning: Loop time of 0.02s overrun"),
                (2_001_000, "  robotPeriodic(): 0.050000s"),
                (3_000_000, "Warning: Loop time of 0.02s overrun"),
                (3_001_000, "  robotPeriodic(): 0.100000s"),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)

        dist = summary.total_duration_distribution
        assert dist["max"] == pytest.approx(100.0, abs=0.1)
        assert dist["p50"] >= 25.0  # at least the smallest


# ── Top-level epoch classification ───────────────────────────────────


class TestTopLevelEpochClassification:
    def test_framework_components_are_top_level(self) -> None:
        assert _is_top_level_epoch("robotPeriodic()")
        assert _is_top_level_epoch("LiveWindow.updateValues()")
        assert _is_top_level_epoch("SmartDashboard.updateValues()")
        assert _is_top_level_epoch("Shuffleboard.update()")
        assert _is_top_level_epoch("teleopPeriodic()")
        assert _is_top_level_epoch("autonomousPeriodic()")

    def test_init_methods_are_top_level(self) -> None:
        assert _is_top_level_epoch("autonomousInit()")
        assert _is_top_level_epoch("teleopInit()")
        assert _is_top_level_epoch("disabledInit()")

    def test_sub_components_are_not_top_level(self) -> None:
        assert not _is_top_level_epoch("TurretSubsystem.periodic()")
        assert not _is_top_level_epoch("Drive.execute()")
        assert not _is_top_level_epoch("SomeCommand.initialize()")


# ── Running commands from Scheduler/Names ────────────────────────────


class TestRunningCommands:
    def test_populates_running_commands(self) -> None:
        """Overrun events get running_commands from Scheduler/Names."""
        console = _make_console_signal(
            [
                (10_000_000, "Warning: Loop time of 0.02s overrun"),
                (10_001_000, "  robotPeriodic(): 0.025000s"),
            ]
        )
        sched = _make_scheduler_names_signal(
            [
                (5_000_000, ["Drive", "IntakeCommand"]),
                (15_000_000, []),  # after the overrun
            ]
        )
        ld = _make_log(
            signals={
                "console": console,
                "NT:/SmartDashboard/Scheduler/Names": sched,
            }
        )
        summary = analyse_loop_overruns(ld, phases=False)

        ev = summary.overrun_events[0]
        assert ev.running_commands == ["Drive", "IntakeCommand"]

    def test_no_scheduler_signal_leaves_none(self) -> None:
        """Without Scheduler/Names, running_commands stays None."""
        console = _make_console_signal(
            [
                (10_000_000, "Warning: Loop time of 0.02s overrun"),
                (10_001_000, "  robotPeriodic(): 0.025000s"),
            ]
        )
        ld = _make_log(signals={"console": console})
        summary = analyse_loop_overruns(ld, phases=False)

        ev = summary.overrun_events[0]
        assert ev.running_commands is None

    def test_empty_running_list(self) -> None:
        """When no commands are running, returns empty list."""
        console = _make_console_signal(
            [
                (10_000_000, "Warning: Loop time of 0.02s overrun"),
                (10_001_000, "  robotPeriodic(): 0.025000s"),
            ]
        )
        sched = _make_scheduler_names_signal(
            [
                (5_000_000, []),
            ]
        )
        ld = _make_log(
            signals={
                "console": console,
                "NT:/SmartDashboard/Scheduler/Names": sched,
            }
        )
        summary = analyse_loop_overruns(ld, phases=False)

        ev = summary.overrun_events[0]
        assert ev.running_commands == []

    def test_nearest_earlier_lookup(self) -> None:
        """Uses the most recent Scheduler/Names before the overrun."""
        console = _make_console_signal(
            [
                (20_000_000, "Warning: Loop time of 0.02s overrun"),
                (20_001_000, "  robotPeriodic(): 0.025000s"),
            ]
        )
        sched = _make_scheduler_names_signal(
            [
                (5_000_000, ["OldCommand"]),
                (15_000_000, ["Drive", "FunctionalCommand"]),
                (25_000_000, ["NewCommand"]),
            ]
        )
        ld = _make_log(
            signals={
                "console": console,
                "NT:/SmartDashboard/Scheduler/Names": sched,
            }
        )
        summary = analyse_loop_overruns(ld, phases=False)

        ev = summary.overrun_events[0]
        assert ev.running_commands == ["Drive", "FunctionalCommand"]

    def test_unnamed_command_names_set(self) -> None:
        """The UNNAMED_COMMAND_NAMES set contains expected entries."""
        assert "InstantCommand" in UNNAMED_COMMAND_NAMES
        assert "FunctionalCommand" in UNNAMED_COMMAND_NAMES
        assert "ParallelCommandGroup" in UNNAMED_COMMAND_NAMES
        assert "SequentialCommandGroup" in UNNAMED_COMMAND_NAMES
        assert "WaitCommand" in UNNAMED_COMMAND_NAMES
        # Custom names should not be in the set
        assert "Drive" not in UNNAMED_COMMAND_NAMES
        assert "AutoShoot" not in UNNAMED_COMMAND_NAMES
