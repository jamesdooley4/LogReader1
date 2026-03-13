"""Tests for the unnamed-commands analyzer."""

from __future__ import annotations

import pytest

from logreader.analyzers.unnamed_commands import (
    UnnamedCommandInfo,
    UnnamedCommandsAnalyzer,
    UnnamedCommandsSummary,
    analyse_unnamed_commands,
)
from logreader.analyzers.loop_overruns import UNNAMED_COMMAND_NAMES
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
    meta = LogMetadata(file_path="test.wpilog", is_valid=True)
    return LogData(metadata=meta, signals=signals or {})


def _make_messages_signal(
    messages: list[tuple[int, str]],
) -> SignalData:
    return SignalData(
        info=SignalInfo(entry_id=1, name="messages", type=SignalType.STRING),
        values=[TimestampedValue(timestamp_us=ts, value=text) for ts, text in messages],
    )


def _make_scheduler_names_signal(
    values: list[tuple[int, list[str]]],
) -> SignalData:
    return SignalData(
        info=SignalInfo(
            entry_id=4,
            name="NT:/SmartDashboard/Scheduler/Names",
            type=SignalType.STRING_ARRAY,
        ),
        values=[TimestampedValue(timestamp_us=ts, value=names) for ts, names in values],
    )


def _make_fms_signal(
    values: list[tuple[int, int]],
) -> SignalData:
    return SignalData(
        info=SignalInfo(
            entry_id=3,
            name="NT:/FMSInfo/FMSControlData",
            type=SignalType.INTEGER,
        ),
        values=[TimestampedValue(timestamp_us=ts, value=cw) for ts, cw in values],
    )


# ── Registration ──────────────────────────────────────────────────────


class TestRegistration:
    def test_registered(self) -> None:
        from logreader.analyzers import list_analyzers

        assert "unnamed-commands" in list_analyzers()

    def test_get_analyzer(self) -> None:
        from logreader.analyzers import get_analyzer

        assert get_analyzer("unnamed-commands") is UnnamedCommandsAnalyzer


# ── No data ───────────────────────────────────────────────────────────


class TestNoData:
    def test_no_signals_at_all(self) -> None:
        ld = _make_log()
        result = UnnamedCommandsAnalyzer().run(ld)
        assert "no command data" in result.summary.lower()

    def test_empty_messages_signal(self) -> None:
        messages = _make_messages_signal([])
        ld = _make_log(signals={"messages": messages})
        result = UnnamedCommandsAnalyzer().run(ld)
        assert "no command data" in result.summary.lower()


# ── All commands are named ────────────────────────────────────────────


class TestAllNamed:
    def test_no_unnamed_commands(self) -> None:
        messages = _make_messages_signal(
            [
                (1_000_000, "Command initialized: AutoShoot"),
                (2_000_000, "Command initialized: Drive"),
                (3_000_000, "Command interrupted: AutoShoot; Cause: <none>"),
                (4_000_000, "Command finished: Drive"),
            ]
        )
        ld = _make_log(signals={"messages": messages})
        summary = analyse_unnamed_commands(ld, phases=False)

        assert summary.unnamed_count == 0
        assert summary.named_count == 2
        assert summary.total_commands_seen == 2

    def test_report_shows_all_clear(self) -> None:
        messages = _make_messages_signal(
            [
                (1_000_000, "Command initialized: AutoShoot"),
            ]
        )
        ld = _make_log(signals={"messages": messages})
        result = UnnamedCommandsAnalyzer().run(ld)
        assert "no action needed" in result.summary.lower()
        assert len(result.rows) == 0


# ── Detects unnamed commands from messages ────────────────────────────


class TestUnnamedFromMessages:
    def test_detects_instant_command(self) -> None:
        messages = _make_messages_signal(
            [
                (1_000_000, "Command initialized: InstantCommand"),
                (2_000_000, "Command finished: InstantCommand"),
                (3_000_000, "Command initialized: InstantCommand"),
                (4_000_000, "Command finished: InstantCommand"),
            ]
        )
        ld = _make_log(signals={"messages": messages})
        summary = analyse_unnamed_commands(ld, phases=False)

        assert summary.unnamed_count == 1
        cmd = summary.unnamed_commands[0]
        assert cmd.name == "InstantCommand"
        assert cmd.initialized == 2
        assert cmd.finished == 2
        assert cmd.interrupted == 0

    def test_detects_multiple_unnamed(self) -> None:
        messages = _make_messages_signal(
            [
                (1_000_000, "Command initialized: FunctionalCommand"),
                (2_000_000, "Command initialized: ParallelCommandGroup"),
                (3_000_000, "Command initialized: Drive"),  # named — skip
                (4_000_000, "Command interrupted: FunctionalCommand; Cause: <none>"),
                (5_000_000, "Command finished: ParallelCommandGroup"),
            ]
        )
        ld = _make_log(signals={"messages": messages})
        summary = analyse_unnamed_commands(ld, phases=False)

        assert summary.unnamed_count == 2
        assert summary.named_count == 1
        names = {c.name for c in summary.unnamed_commands}
        assert names == {"FunctionalCommand", "ParallelCommandGroup"}

    def test_report_shows_warning(self) -> None:
        messages = _make_messages_signal(
            [
                (1_000_000, "Command initialized: InstantCommand"),
            ]
        )
        ld = _make_log(signals={"messages": messages})
        result = UnnamedCommandsAnalyzer().run(ld)
        assert "default class names" in result.summary.lower()
        assert len(result.rows) == 1
        assert result.rows[0]["Command"] == "InstantCommand"


# ── Detects unnamed commands from Scheduler/Names ─────────────────────


class TestUnnamedFromScheduler:
    def test_detects_from_scheduler_only(self) -> None:
        """Commands only visible in Scheduler/Names (no messages events)."""
        sched = _make_scheduler_names_signal(
            [
                (1_000_000, ["Drive", "FunctionalCommand"]),
                (5_000_000, ["Drive"]),
                (10_000_000, ["Drive", "FunctionalCommand"]),
            ]
        )
        ld = _make_log(signals={"NT:/SmartDashboard/Scheduler/Names": sched})
        summary = analyse_unnamed_commands(ld, phases=False)

        assert summary.unnamed_count == 1
        cmd = summary.unnamed_commands[0]
        assert cmd.name == "FunctionalCommand"
        assert cmd.scheduler_appearances == 2
        assert cmd.initialized == 0  # no messages signal

    def test_combines_messages_and_scheduler(self) -> None:
        """Both sources contribute to the same unnamed command."""
        messages = _make_messages_signal(
            [
                (1_000_000, "Command initialized: FunctionalCommand"),
                (5_000_000, "Command finished: FunctionalCommand"),
            ]
        )
        sched = _make_scheduler_names_signal(
            [
                (2_000_000, ["FunctionalCommand", "Drive"]),
                (3_000_000, ["FunctionalCommand", "Drive"]),
                (4_000_000, ["FunctionalCommand", "Drive"]),
            ]
        )
        ld = _make_log(
            signals={
                "messages": messages,
                "NT:/SmartDashboard/Scheduler/Names": sched,
            }
        )
        summary = analyse_unnamed_commands(ld, phases=False)

        assert summary.unnamed_count == 1
        cmd = summary.unnamed_commands[0]
        assert cmd.name == "FunctionalCommand"
        assert cmd.initialized == 1
        assert cmd.finished == 1
        assert cmd.scheduler_appearances == 3


# ── Phase annotation ─────────────────────────────────────────────────


class TestPhaseAnnotation:
    def test_phase_tracking(self) -> None:
        fms = _make_fms_signal(
            [
                (0, 0x20),  # disabled
                (10_000_000, 0x23),  # auto
                (25_000_000, 0x20),  # disabled
                (30_000_000, 0x21),  # teleop
                (130_000_000, 0x20),  # disabled
            ]
        )
        messages = _make_messages_signal(
            [
                (12_000_000, "Command initialized: InstantCommand"),  # auto
                (15_000_000, "Command finished: InstantCommand"),  # auto
                (50_000_000, "Command initialized: InstantCommand"),  # teleop
                (60_000_000, "Command finished: InstantCommand"),  # teleop
            ]
        )
        ld = _make_log(
            signals={
                "messages": messages,
                "NT:/FMSInfo/FMSControlData": fms,
            }
        )
        summary = analyse_unnamed_commands(ld, phases=True)

        cmd = summary.unnamed_commands[0]
        assert "auto" in cmd.phases
        assert "teleop" in cmd.phases

    def test_phases_column_in_report(self) -> None:
        fms = _make_fms_signal(
            [
                (0, 0x20),
                (10_000_000, 0x21),  # teleop
                (130_000_000, 0x20),
            ]
        )
        messages = _make_messages_signal(
            [
                (50_000_000, "Command initialized: InstantCommand"),
            ]
        )
        ld = _make_log(
            signals={
                "messages": messages,
                "NT:/FMSInfo/FMSControlData": fms,
            }
        )
        result = UnnamedCommandsAnalyzer().run(ld, phases=True)

        assert "Phases" in result.columns
        assert result.rows[0]["Phases"] == "teleop"


# ── Sorting ──────────────────────────────────────────────────────────


class TestSorting:
    def test_sorted_by_activity(self) -> None:
        """Commands with more activity appear first."""
        messages = _make_messages_signal(
            [
                (1_000_000, "Command initialized: WaitCommand"),
                (2_000_000, "Command initialized: InstantCommand"),
                (3_000_000, "Command finished: InstantCommand"),
                (4_000_000, "Command initialized: InstantCommand"),
                (5_000_000, "Command finished: InstantCommand"),
                (6_000_000, "Command finished: WaitCommand"),
            ]
        )
        ld = _make_log(signals={"messages": messages})
        summary = analyse_unnamed_commands(ld, phases=False)

        assert summary.unnamed_commands[0].name == "InstantCommand"
        assert summary.unnamed_commands[1].name == "WaitCommand"


# ── CLI arguments ────────────────────────────────────────────────────


class TestCLIArguments:
    def test_add_arguments(self) -> None:
        import argparse

        parser = argparse.ArgumentParser()
        UnnamedCommandsAnalyzer.add_arguments(parser)
        args = parser.parse_args(["--phases"])
        assert args.phases is True


# ── Full set of default names ────────────────────────────────────────


class TestDefaultNamesCoverage:
    @pytest.mark.parametrize(
        "name",
        [
            "InstantCommand",
            "RunCommand",
            "StartEndCommand",
            "FunctionalCommand",
            "PrintCommand",
            "SequentialCommandGroup",
            "ParallelCommandGroup",
            "ParallelDeadlineGroup",
            "ParallelRaceGroup",
            "ConditionalCommand",
            "SelectCommand",
            "ProxyCommand",
            "RepeatCommand",
            "DeferredCommand",
            "ScheduleCommand",
            "WrapperCommand",
            "WaitCommand",
            "WaitUntilCommand",
            "PIDCommand",
            "ProfiledPIDCommand",
            "TrapezoidProfileCommand",
            "MecanumControllerCommand",
            "RamseteCommand",
            "SwerveControllerCommand",
            "NotifierCommand",
        ],
    )
    def test_name_is_in_unnamed_set(self, name: str) -> None:
        assert name in UNNAMED_COMMAND_NAMES

    def test_custom_names_not_in_set(self) -> None:
        for name in [
            "Drive",
            "AutoShoot",
            "IntakeCommand",
            "LED Default Command",
            "C-Outpost-Depot",
        ]:
            assert name not in UNNAMED_COMMAND_NAMES
