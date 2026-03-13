"""Unnamed-commands analyzer — find commands using WPILib default class names.

When teams use inline or anonymous commands (e.g. ``new InstantCommand(...)``
without subclassing or calling ``.withName()``), the scheduler logs them with
their generic class name, making debugging and match review much harder.

This analyzer scans two data sources:

1. The ``messages`` signal — command lifecycle events (initialized,
   interrupted, finished).
2. The ``Scheduler/Names`` string-array signal — the live list of
   currently running commands published each time the set changes.

It flags any commands whose names match a known set of WPILib built-in
command class names, and reports how many times each appeared.
"""

from __future__ import annotations

import re
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Any

from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer
from logreader.analyzers.loop_overruns import (
    UNNAMED_COMMAND_NAMES,
    _find_scheduler_names_signal,
    _parse_messages_signal,
)
from logreader.models import LogData

# ---------------------------------------------------------------------------
# Data contracts
# ---------------------------------------------------------------------------


@dataclass
class UnnamedCommandInfo:
    """Aggregated info about a single unnamed command seen in the log."""

    name: str
    initialized: int = 0
    interrupted: int = 0
    finished: int = 0
    scheduler_appearances: int = 0  # distinct Scheduler/Names snapshots
    phases: set[str] = field(default_factory=set)

    @property
    def total_lifecycle_events(self) -> int:
        return self.initialized + self.interrupted + self.finished


@dataclass
class UnnamedCommandsSummary:
    """Top-level analysis result."""

    total_commands_seen: int  # all unique command names (named + unnamed)
    unnamed_count: int
    named_count: int
    unnamed_commands: list[UnnamedCommandInfo]
    all_command_names: list[str]


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------


def analyse_unnamed_commands(
    log_data: LogData,
    *,
    phases: bool = True,
) -> UnnamedCommandsSummary:
    """Run the unnamed-commands analysis.

    Parameters:
        log_data: Parsed log data.
        phases: If ``True``, annotate each unnamed command with the match
            phases where it appeared.

    Returns:
        A fully populated ``UnnamedCommandsSummary``.
    """
    # Collect all unique command names and unnamed-command stats from messages
    all_names: set[str] = set()
    unnamed: dict[str, UnnamedCommandInfo] = {}

    # ── Messages signal ────────────────────────────────────────────────
    cmd_events = _parse_messages_signal(log_data)

    # Phase annotation (optional)
    timeline = None
    if phases:
        try:
            from logreader.analyzers.match_phases import detect_match_phases

            timeline = detect_match_phases(log_data)
        except ImportError:
            pass

    for ev in cmd_events:
        all_names.add(ev.command_name)

        if ev.command_name not in UNNAMED_COMMAND_NAMES:
            continue

        info = unnamed.setdefault(
            ev.command_name, UnnamedCommandInfo(name=ev.command_name)
        )
        if ev.kind == "initialized":
            info.initialized += 1
        elif ev.kind == "interrupted":
            info.interrupted += 1
        elif ev.kind == "finished":
            info.finished += 1

        # Phase annotation
        if timeline is not None:
            phase = timeline.phase_at(ev.timestamp_us)
            info.phases.add(phase.value)

    # ── Scheduler/Names signal ─────────────────────────────────────────
    parsed = _find_scheduler_names_signal(log_data)
    if parsed is not None:
        sched_ts, sched_names = parsed
        for i, names in enumerate(sched_names):
            for name in names:
                all_names.add(name)
                if name in UNNAMED_COMMAND_NAMES:
                    info = unnamed.setdefault(name, UnnamedCommandInfo(name=name))
                    info.scheduler_appearances += 1
                    # Phase annotation from scheduler timestamp
                    if timeline is not None:
                        phase = timeline.phase_at(sched_ts[i])
                        info.phases.add(phase.value)

    # Sort unnamed by total activity descending
    unnamed_list = sorted(
        unnamed.values(),
        key=lambda c: (c.total_lifecycle_events + c.scheduler_appearances),
        reverse=True,
    )
    named_names = sorted(all_names - UNNAMED_COMMAND_NAMES)

    return UnnamedCommandsSummary(
        total_commands_seen=len(all_names),
        unnamed_count=len(unnamed_list),
        named_count=len(named_names),
        unnamed_commands=unnamed_list,
        all_command_names=sorted(all_names),
    )


# ---------------------------------------------------------------------------
# Output formatting
# ---------------------------------------------------------------------------


def _format_summary(summary: UnnamedCommandsSummary) -> str:
    """Render the summary text."""
    lines = [
        f"  Total unique commands: {summary.total_commands_seen}",
        f"  Named (custom):       {summary.named_count}",
        f"  Unnamed (default):    {summary.unnamed_count}",
    ]

    if summary.unnamed_count == 0:
        lines.append("")
        lines.append("  ✅ All commands have custom names — no action needed.")
    else:
        lines.append("")
        lines.append("  ⚠ The following commands use WPILib default class names.")
        lines.append("    Consider naming them with .withName() or subclassing.")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Analyzer class
# ---------------------------------------------------------------------------


@register_analyzer
class UnnamedCommandsAnalyzer(BaseAnalyzer):
    """Find commands using WPILib default class names.

    Scans command lifecycle events and the Scheduler/Names signal to
    identify commands that haven't been given custom names.
    """

    name = "unnamed-commands"
    description = "Find commands using WPILib default class names"

    @classmethod
    def add_arguments(cls, parser: "argparse.ArgumentParser") -> None:
        import argparse  # noqa: F811

        parser.add_argument(
            "--phases",
            action="store_true",
            default=False,
            help="Show which match phases each unnamed command appeared in",
        )

    def run(self, log_data: LogData, **options: Any) -> AnalysisResult:
        show_phases = bool(options.get("phases", False))

        summary = analyse_unnamed_commands(log_data, phases=show_phases)

        # No command data at all
        if summary.total_commands_seen == 0:
            return AnalysisResult(
                analyzer_name=self.name,
                title="Unnamed Commands",
                summary="No command data found — "
                "neither messages nor Scheduler/Names signals are present.",
                extra={"summary": summary},
            )

        summary_text = _format_summary(summary)

        # Build table
        columns = [
            "Command",
            "Initialized",
            "Interrupted",
            "Finished",
            "Sched Snapshots",
        ]
        if show_phases:
            columns.append("Phases")

        rows: list[dict[str, Any]] = []
        for cmd in summary.unnamed_commands:
            row: dict[str, Any] = {
                "Command": cmd.name,
                "Initialized": cmd.initialized,
                "Interrupted": cmd.interrupted,
                "Finished": cmd.finished,
                "Sched Snapshots": cmd.scheduler_appearances,
            }
            if show_phases:
                row["Phases"] = ", ".join(sorted(cmd.phases)) if cmd.phases else ""
            rows.append(row)

        return AnalysisResult(
            analyzer_name=self.name,
            title="Unnamed Commands",
            summary=summary_text,
            columns=columns,
            rows=rows,
            extra={
                "summary": summary,
                "unnamed_commands": summary.unnamed_commands,
                "all_command_names": summary.all_command_names,
            },
        )
