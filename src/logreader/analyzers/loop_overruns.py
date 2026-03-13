"""Loop overrun analyzer — detect and report robot main-loop overruns.

Parses WPILib ``Tracer`` timing breakdowns from console log messages to
identify which subsystems and commands consume the most time, correlates
overruns with match phases, and flags the worst offenders.

See ``docs/design-loop-overruns.md`` for the full design rationale.
"""

from __future__ import annotations

import argparse
import re
import statistics
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer
from logreader.models import LogData

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_LOOP_PERIOD_S = 0.02  # 20 ms / 50 Hz
GROUP_WINDOW_S = 0.5  # max gap to group split Tracer output
PHASE_TRANSITION_WINDOW_S = 5.0  # ±5 s around phase transitions
BURST_GAP_S = 2.0  # max gap between overruns in a burst

# ---------------------------------------------------------------------------
# Regex patterns
# ---------------------------------------------------------------------------

# "Loop time of 0.02s overrun"
_OVERRUN_RE = re.compile(r"Loop time of ([\d.]+)s overrun")

# Tracer epoch line: "  teleopPeriodic(): 0.001317s"
_EPOCH_RE = re.compile(
    r"^\s*(\S+(?:\.\w+)*(?:\(\))?)\s*:\s*([\d.]+)s\s*$", re.MULTILINE
)

# "CommandScheduler loop overrun"
_SCHEDULER_OVERRUN_RE = re.compile(r"CommandScheduler loop overrun")

# Command lifecycle from the "messages" signal
_COMMAND_INIT_RE = re.compile(r"Command initialized:\s*(.+)")
_COMMAND_INTERRUPT_RE = re.compile(r"Command interrupted:\s*(.+?)(?:;\s*Cause:.*)?$")
_COMMAND_FINISH_RE = re.compile(r"Command finished:\s*(.+)")

# Interesting nearby-event patterns (for correlation)
_NEARBY_PATTERNS: list[tuple[str, re.Pattern[str]]] = [
    ("datalog", re.compile(r"DataLog:", re.IGNORECASE)),
    ("can", re.compile(r"CAN\b|CANbus|timeout|timed out", re.IGNORECASE)),
    ("nt", re.compile(r"NT:|NetworkTables|ntcore", re.IGNORECASE)),
    ("brownout", re.compile(r"brownout|brown-out|voltage", re.IGNORECASE)),
    ("warning", re.compile(r"Warning|Error", re.IGNORECASE)),
]

# ---------------------------------------------------------------------------
# Component categorisation
# ---------------------------------------------------------------------------


class ComponentCategory(Enum):
    """Classification for Tracer epoch components."""

    FRAMEWORK = "framework"
    PHASE_INIT = "phase_init"
    SUBSYSTEM = "subsystem"
    COMMAND = "command"
    OTHER = "other"


# Phase-specific periodics that are framework-level
_PHASE_PERIODIC_NAMES = {
    "autonomousPeriodic()",
    "teleopPeriodic()",
    "disabledPeriodic()",
    "testPeriodic()",
}

_FRAMEWORK_PREFIXES = (
    "robotPeriodic",
    "LiveWindow.",
    "SmartDashboard.",
    "Shuffleboard.",
)


def categorise_component(name: str) -> ComponentCategory:
    """Classify a Tracer epoch component by its name.

    Detection rules (from design doc):
    - Names ending in ``Init()`` → PHASE_INIT
    - ``robotPeriodic()``, ``LiveWindow.*``, ``SmartDashboard.*``,
      ``Shuffleboard.*``, phase periodics → FRAMEWORK
    - Names ending in ``.periodic()`` (excluding above) → SUBSYSTEM
    - Names ending in ``.execute()`` or ``.initialize()`` → COMMAND
    - Everything else → OTHER
    """
    # Phase init methods
    if name.endswith("Init()") or name.endswith("Init"):
        return ComponentCategory.PHASE_INIT

    # Phase periodics
    if name in _PHASE_PERIODIC_NAMES:
        return ComponentCategory.FRAMEWORK

    # Framework infrastructure
    for prefix in _FRAMEWORK_PREFIXES:
        if name.startswith(prefix):
            return ComponentCategory.FRAMEWORK

    # Subsystem periodics
    if name.endswith(".periodic()"):
        return ComponentCategory.SUBSYSTEM

    # Command execution / initialisation
    if name.endswith(".execute()") or name.endswith(".initialize()"):
        return ComponentCategory.COMMAND

    return ComponentCategory.OTHER


# ---------------------------------------------------------------------------
# Data contracts
# ---------------------------------------------------------------------------


@dataclass
class NearbyEvent:
    """An interesting event correlated with an overrun."""

    timestamp_us: int
    category: str  # datalog, can, command, nt, warning, other
    relative_offset_ms: float  # event time − overrun time
    text: str


@dataclass
class OverrunEvent:
    """A single loop iteration that exceeded the configured period."""

    timestamp_us: int
    total_duration_ms: float
    configured_period_ms: float
    epoch_timings: dict[str, float]  # component → duration in seconds
    phase: str | None
    scheduler_overrun_markers: int
    nearby_events: list[NearbyEvent] = field(default_factory=list)
    running_commands: list[str] | None = None  # from Scheduler/Names


@dataclass
class ComponentStats:
    """Aggregated timing statistics for a single Tracer component."""

    name: str
    category: ComponentCategory
    sample_count: int
    median_ms: float
    mean_ms: float
    p95_ms: float
    max_ms: float
    mean_contribution_pct: float  # mean % of loop time when this component runs


@dataclass
class PhaseOverrunStats:
    """Overrun statistics for a single match phase."""

    phase: str
    duration_s: float
    overrun_count: int
    overrun_rate_hz: float
    worst_total_ms: float
    top_components: list[ComponentStats]


@dataclass
class PhaseTransitionDetail:
    """Overrun burst and command activity around a phase transition."""

    timestamp_us: int
    from_phase: str | None
    to_phase: str
    overruns_nearby: int  # within ±5 s
    init_method_ms: float | None  # e.g. autonomousInit() timing if present
    commands_initialized: list[str]
    commands_interrupted: list[str]
    commands_finished: list[str]


@dataclass
class OverrunSummary:
    """Top-level analysis result."""

    configured_period_ms: float
    analysis_duration_s: float
    loop_estimation_basis: str  # "full_log_range" | "console_range"
    estimated_total_loops: int
    overrun_count: int
    scheduler_overrun_count: int
    overrun_pct: float
    total_duration_distribution: dict[str, float]  # p50, p90, p95, p99, max
    components: list[ComponentStats]
    phase_stats: list[PhaseOverrunStats] | None
    phase_transitions: list[PhaseTransitionDetail] | None
    overrun_events: list[OverrunEvent]
    burst_periods: list[tuple[float, float, int]]  # (start_s, end_s, count)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _classify_nearby(text: str) -> str:
    """Return the category for a nearby-event text string."""
    for cat, pattern in _NEARBY_PATTERNS:
        if pattern.search(text):
            return cat
    return "other"


def _is_tracer_line(text: str) -> bool:
    """True if *text* looks like a Tracer epoch dump fragment."""
    return bool(_EPOCH_RE.search(text))


def _is_overrun_warning(text: str) -> bool:
    """True if *text* is an IterativeRobotBase overrun message."""
    return bool(_OVERRUN_RE.search(text))


def _is_scheduler_overrun(text: str) -> bool:
    """True if *text* is a CommandScheduler overrun marker."""
    return bool(_SCHEDULER_OVERRUN_RE.search(text))


@dataclass
class _RawConsoleMsg:
    """A single parsed console message."""

    timestamp_us: int
    text: str


def _parse_console_signal(log_data: LogData) -> list[_RawConsoleMsg]:
    """Extract all console messages in timestamp order."""
    sig = log_data.get_signal("console")
    if sig is None or not sig.values:
        return []
    return [
        _RawConsoleMsg(timestamp_us=v.timestamp_us, text=str(v.value))
        for v in sig.values
    ]


@dataclass
class _RawCommandEvent:
    """A command lifecycle event from the messages signal."""

    timestamp_us: int
    kind: str  # "initialized", "interrupted", "finished"
    command_name: str


def _parse_messages_signal(log_data: LogData) -> list[_RawCommandEvent]:
    """Extract command lifecycle events from the ``messages`` signal."""
    sig = log_data.get_signal("messages")
    if sig is None or not sig.values:
        return []

    events: list[_RawCommandEvent] = []
    for v in sig.values:
        text = str(v.value)
        m = _COMMAND_INIT_RE.search(text)
        if m:
            events.append(
                _RawCommandEvent(v.timestamp_us, "initialized", m.group(1).strip())
            )
            continue
        m = _COMMAND_INTERRUPT_RE.search(text)
        if m:
            events.append(
                _RawCommandEvent(v.timestamp_us, "interrupted", m.group(1).strip())
            )
            continue
        m = _COMMAND_FINISH_RE.search(text)
        if m:
            events.append(
                _RawCommandEvent(v.timestamp_us, "finished", m.group(1).strip())
            )
    return events


# ---------------------------------------------------------------------------
# Scheduler/Names — running command context
# ---------------------------------------------------------------------------

# Signal name patterns for the CommandScheduler running-command list.
_SCHEDULER_NAMES_PATTERNS = [
    "NT:/SmartDashboard/Scheduler/Names",
    "NT:/LiveWindow/Ungrouped/Scheduler/Names",
]

# WPILib built-in command class names.  Commands using these names have not
# been given a custom name via `.withName()` or subclassing.
UNNAMED_COMMAND_NAMES: frozenset[str] = frozenset(
    {
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
    }
)


def _find_scheduler_names_signal(
    log_data: LogData,
) -> tuple[list[int], list[list[str]]] | None:
    """Find and parse the Scheduler/Names string-array signal.

    Returns ``(timestamps_us, names_lists)`` sorted by time, or ``None``
    if the signal is not present or empty.
    """
    for pattern in _SCHEDULER_NAMES_PATTERNS:
        sig = log_data.get_signal(pattern)
        if sig is not None and sig.values:
            ts = [v.timestamp_us for v in sig.values]
            names = [list(v.value) if v.value else [] for v in sig.values]
            return ts, names
    return None


def _running_commands_at(
    timestamp_us: int,
    sched_ts: list[int],
    sched_names: list[list[str]],
) -> list[str]:
    """Return the commands running at *timestamp_us* (nearest-earlier lookup)."""
    import bisect

    idx = bisect.bisect_right(sched_ts, timestamp_us) - 1
    if idx < 0:
        return []
    return sched_names[idx]


def _attach_running_commands(
    events: list[OverrunEvent],
    log_data: LogData,
) -> None:
    """Populate each event's ``running_commands`` from Scheduler/Names."""
    parsed = _find_scheduler_names_signal(log_data)
    if parsed is None:
        return
    sched_ts, sched_names = parsed

    for ev in events:
        ev.running_commands = _running_commands_at(
            ev.timestamp_us, sched_ts, sched_names
        )


# ---------------------------------------------------------------------------
# Grouping: raw console messages → logical OverrunEvents
# ---------------------------------------------------------------------------


def _group_overrun_events(
    msgs: list[_RawConsoleMsg],
    configured_period_s: float,
) -> list[OverrunEvent]:
    """Group raw console messages into logical ``OverrunEvent`` objects.

    Algorithm:
    1. Anchor each event on an ``IterativeRobotBase`` overrun warning.
    2. Attach following console messages that look like Tracer epoch lines
       within the grouping window.
    3. Attach any CommandScheduler overrun markers within the window.
    """
    events: list[OverrunEvent] = []
    used: set[int] = set()  # indices of consumed messages

    for i, msg in enumerate(msgs):
        if i in used:
            continue
        if not _is_overrun_warning(msg.text):
            continue

        # This is an anchor
        used.add(i)
        anchor_ts = msg.timestamp_us
        window_us = int(GROUP_WINDOW_S * 1_000_000)

        # Collect epoch timings from nearby Tracer dump fragments
        epoch_timings: dict[str, float] = {}
        sched_markers = 0

        # Also parse any epoch lines within the anchor message itself
        for em in _EPOCH_RE.finditer(msg.text):
            epoch_timings[em.group(1)] = float(em.group(2))

        # Look at subsequent messages within the window
        for j in range(i + 1, len(msgs)):
            if msgs[j].timestamp_us - anchor_ts > window_us:
                break
            if j in used:
                continue

            jtext = msgs[j].text

            # Another IterativeRobotBase overrun → belongs to a different event
            if _is_overrun_warning(jtext):
                break

            if _is_tracer_line(jtext):
                used.add(j)
                for em in _EPOCH_RE.finditer(jtext):
                    name = em.group(1)
                    val = float(em.group(2))
                    # If same component appears twice, take the larger
                    epoch_timings[name] = max(epoch_timings.get(name, 0.0), val)
            elif _is_scheduler_overrun(jtext):
                used.add(j)
                sched_markers += 1

        # Compute total duration from epoch timings
        total_s = sum(epoch_timings.values())
        total_ms = total_s * 1000.0

        events.append(
            OverrunEvent(
                timestamp_us=anchor_ts,
                total_duration_ms=total_ms,
                configured_period_ms=configured_period_s * 1000.0,
                epoch_timings=epoch_timings,
                phase=None,  # filled in later
                scheduler_overrun_markers=sched_markers,
            )
        )

    return events


def _count_orphan_scheduler_overruns(
    msgs: list[_RawConsoleMsg],
    used_by_events: set[int] | None = None,
) -> int:
    """Count CommandScheduler overrun markers not attached to an event.

    We need a second pass since ``_group_overrun_events`` marks consumed
    indices.  For simplicity we re-scan and count all scheduler overruns,
    then subtract those attached to events.
    """
    total = sum(1 for m in msgs if _is_scheduler_overrun(m.text))
    attached = 0
    if used_by_events is not None:
        attached = sum(
            1
            for idx in used_by_events
            if idx < len(msgs) and _is_scheduler_overrun(msgs[idx].text)
        )
    return total - attached


# ---------------------------------------------------------------------------
# Component statistics
# ---------------------------------------------------------------------------

_TOP_LEVEL_EPOCHS = {
    "robotPeriodic()",
    "LiveWindow.updateValues()",
    "SmartDashboard.updateValues()",
    "Shuffleboard.update()",
    "autonomousPeriodic()",
    "teleopPeriodic()",
    "disabledPeriodic()",
    "testPeriodic()",
}


def _is_top_level_epoch(name: str) -> bool:
    """True if *name* is one of the main pipeline stages."""
    if name in _TOP_LEVEL_EPOCHS:
        return True
    # Phase init methods are also top-level
    if categorise_component(name) == ComponentCategory.PHASE_INIT:
        return True
    return False


def _compute_component_stats(
    events: list[OverrunEvent],
    *,
    top_level_only: bool = False,
) -> list[ComponentStats]:
    """Aggregate per-component timing across all overrun events.

    Parameters:
        events: The overrun events to aggregate.
        top_level_only: If ``True``, only include top-level pipeline
            epochs (not sub-components inside ``robotPeriodic``).
    """
    # Collect per-component timing samples (in seconds)
    comp_samples: dict[str, list[float]] = {}
    # Per-component contribution ratios (component_time / event_total)
    comp_contrib_ratios: dict[str, list[float]] = {}

    for ev in events:
        if top_level_only:
            timings = {
                k: v for k, v in ev.epoch_timings.items() if _is_top_level_epoch(k)
            }
        else:
            timings = ev.epoch_timings

        ev_total = sum(timings.values())

        for name, dur_s in timings.items():
            comp_samples.setdefault(name, []).append(dur_s)
            ratio = (dur_s / ev_total) if ev_total > 0 else 0.0
            comp_contrib_ratios.setdefault(name, []).append(ratio)

    if not events:
        return []

    result: list[ComponentStats] = []
    for name, samples_s in comp_samples.items():
        samples_ms = [s * 1000.0 for s in samples_s]
        ratios = comp_contrib_ratios[name]
        contrib_pct = statistics.mean(ratios) * 100.0

        result.append(
            ComponentStats(
                name=name,
                category=categorise_component(name),
                sample_count=len(samples_ms),
                median_ms=statistics.median(samples_ms),
                mean_ms=statistics.mean(samples_ms),
                p95_ms=_percentile(samples_ms, 95),
                max_ms=max(samples_ms),
                mean_contribution_pct=round(contrib_pct, 1),
            )
        )

    # Sort by mean_ms descending
    result.sort(key=lambda c: c.mean_ms, reverse=True)
    return result


def _percentile(data: list[float], pct: int) -> float:
    """Compute a percentile. *data* must be non-empty."""
    if len(data) == 1:
        return data[0]
    sorted_data = sorted(data)
    k = (pct / 100.0) * (len(sorted_data) - 1)
    f = int(k)
    c = f + 1
    if c >= len(sorted_data):
        return sorted_data[-1]
    return sorted_data[f] + (k - f) * (sorted_data[c] - sorted_data[f])


# ---------------------------------------------------------------------------
# Duration distribution
# ---------------------------------------------------------------------------


def _duration_distribution(events: list[OverrunEvent]) -> dict[str, float]:
    """Compute p50, p90, p95, p99, max of total overrun durations."""
    if not events:
        return {"p50": 0.0, "p90": 0.0, "p95": 0.0, "p99": 0.0, "max": 0.0}

    durations = [ev.total_duration_ms for ev in events]
    return {
        "p50": round(_percentile(durations, 50), 1),
        "p90": round(_percentile(durations, 90), 1),
        "p95": round(_percentile(durations, 95), 1),
        "p99": round(_percentile(durations, 99), 1),
        "max": round(max(durations), 1),
    }


# ---------------------------------------------------------------------------
# Burst detection
# ---------------------------------------------------------------------------


def _detect_bursts(
    events: list[OverrunEvent],
    gap_s: float = BURST_GAP_S,
) -> list[tuple[float, float, int]]:
    """Detect clusters of overruns in rapid succession.

    Returns a list of ``(start_s, end_s, count)`` tuples for each burst
    of ≥2 overruns separated by at most *gap_s*.
    """
    if len(events) < 2:
        return []

    bursts: list[tuple[float, float, int]] = []
    burst_start_s = events[0].timestamp_us / 1_000_000.0
    burst_end_s = burst_start_s
    count = 1

    for ev in events[1:]:
        t_s = ev.timestamp_us / 1_000_000.0
        if t_s - burst_end_s <= gap_s:
            burst_end_s = t_s
            count += 1
        else:
            if count >= 2:
                bursts.append((round(burst_start_s, 3), round(burst_end_s, 3), count))
            burst_start_s = t_s
            burst_end_s = t_s
            count = 1

    if count >= 2:
        bursts.append((round(burst_start_s, 3), round(burst_end_s, 3), count))

    return bursts


# ---------------------------------------------------------------------------
# Nearby-event correlation
# ---------------------------------------------------------------------------


def _attach_nearby_events(
    events: list[OverrunEvent],
    console_msgs: list[_RawConsoleMsg],
    command_events: list[_RawCommandEvent],
) -> None:
    """Populate each event's ``nearby_events`` list.

    Uses a dynamic window: ``±(1 s + overrun_duration)``.  Filters out
    the Tracer output itself to avoid circular references.
    """
    for ev in events:
        overrun_dur_s = ev.total_duration_ms / 1000.0
        window_us = int((1.0 + overrun_dur_s) * 1_000_000)
        lo = ev.timestamp_us - window_us
        hi = ev.timestamp_us + window_us

        nearby: list[NearbyEvent] = []

        # Console messages (skip Tracer / overrun output)
        for msg in console_msgs:
            if msg.timestamp_us < lo:
                continue
            if msg.timestamp_us > hi:
                break  # messages are sorted
            # Skip the overrun's own Tracer output
            if _is_overrun_warning(msg.text) or _is_tracer_line(msg.text):
                continue
            if _is_scheduler_overrun(msg.text):
                continue

            cat = _classify_nearby(msg.text)
            nearby.append(
                NearbyEvent(
                    timestamp_us=msg.timestamp_us,
                    category=cat,
                    relative_offset_ms=(msg.timestamp_us - ev.timestamp_us) / 1000.0,
                    text=msg.text,
                )
            )

        # Command lifecycle events
        for ce in command_events:
            if ce.timestamp_us < lo:
                continue
            if ce.timestamp_us > hi:
                break
            nearby.append(
                NearbyEvent(
                    timestamp_us=ce.timestamp_us,
                    category="command",
                    relative_offset_ms=(ce.timestamp_us - ev.timestamp_us) / 1000.0,
                    text=f"Command {ce.kind}: {ce.command_name}",
                )
            )

        # Sort by absolute offset, then apply priority ordering for ties
        _PRIORITY = {
            "datalog": 0,
            "can": 1,
            "brownout": 1,
            "command": 2,
            "nt": 3,
            "warning": 4,
            "other": 5,
        }
        nearby.sort(
            key=lambda e: (abs(e.relative_offset_ms), _PRIORITY.get(e.category, 9))
        )
        ev.nearby_events = nearby


# ---------------------------------------------------------------------------
# Phase integration
# ---------------------------------------------------------------------------


def _annotate_phases(
    events: list[OverrunEvent],
    log_data: LogData,
) -> Any:
    """Annotate overrun events with phase labels.

    Returns the ``MatchPhaseTimeline`` if available, else ``None``.
    """
    try:
        from logreader.analyzers.match_phases import detect_match_phases
    except ImportError:
        return None

    timeline = detect_match_phases(log_data)
    if timeline is None:
        return None

    for ev in events:
        phase = timeline.phase_at(ev.timestamp_us)
        ev.phase = phase.value

    return timeline


def _compute_phase_stats(
    events: list[OverrunEvent],
    timeline: Any,
) -> list[PhaseOverrunStats] | None:
    """Compute per-phase overrun statistics."""
    if timeline is None:
        return None

    from logreader.analyzers.match_phases import MatchPhase, phase_durations

    durations = phase_durations(timeline)

    results: list[PhaseOverrunStats] = []
    for phase in [MatchPhase.AUTONOMOUS, MatchPhase.TELEOP, MatchPhase.DISABLED]:
        phase_events = [e for e in events if e.phase == phase.value]
        dur = durations.get(phase, 0.0)
        rate = len(phase_events) / dur if dur > 0 else 0.0
        worst = max((e.total_duration_ms for e in phase_events), default=0.0)
        comps = _compute_component_stats(phase_events, top_level_only=True)

        results.append(
            PhaseOverrunStats(
                phase=phase.value,
                duration_s=round(dur, 1),
                overrun_count=len(phase_events),
                overrun_rate_hz=round(rate, 2),
                worst_total_ms=round(worst, 1),
                top_components=comps[:5],
            )
        )

    return results


def _compute_phase_transitions(
    events: list[OverrunEvent],
    timeline: Any,
    command_events: list[_RawCommandEvent],
) -> list[PhaseTransitionDetail] | None:
    """Detect overrun clustering around phase transitions."""
    if timeline is None:
        return None

    from logreader.analyzers.match_phases import MatchPhase

    transitions: list[PhaseTransitionDetail] = []
    intervals = timeline.intervals

    for idx, iv in enumerate(intervals):
        if iv.phase == MatchPhase.DISABLED:
            continue

        ts_us = iv.start_us
        window_us = int(PHASE_TRANSITION_WINDOW_S * 1_000_000)

        # Count overruns within ±window
        nearby_count = sum(
            1 for e in events if abs(e.timestamp_us - ts_us) <= window_us
        )

        # Find init method timing from nearby overruns
        init_name = f"{iv.phase.value}Init()"
        # Also try capitalised version: autonomousInit, teleopInit
        init_ms: float | None = None
        for e in events:
            if abs(e.timestamp_us - ts_us) <= window_us:
                for name, dur in e.epoch_timings.items():
                    if name.lower().replace("ous", "ous") == init_name.lower() or (
                        iv.phase.value.lower() in name.lower() and "Init" in name
                    ):
                        init_ms = dur * 1000.0
                        break

        # Collect command events near this transition
        cmds_init: list[str] = []
        cmds_interrupt: list[str] = []
        cmds_finish: list[str] = []
        for ce in command_events:
            if abs(ce.timestamp_us - ts_us) <= window_us:
                if ce.kind == "initialized":
                    cmds_init.append(ce.command_name)
                elif ce.kind == "interrupted":
                    cmds_interrupt.append(ce.command_name)
                elif ce.kind == "finished":
                    cmds_finish.append(ce.command_name)

        # Determine from_phase
        from_phase = intervals[idx - 1].phase.value if idx > 0 else None

        transitions.append(
            PhaseTransitionDetail(
                timestamp_us=ts_us,
                from_phase=from_phase,
                to_phase=iv.phase.value,
                overruns_nearby=nearby_count,
                init_method_ms=round(init_ms, 1) if init_ms is not None else None,
                commands_initialized=cmds_init,
                commands_interrupted=cmds_interrupt,
                commands_finished=cmds_finish,
            )
        )

    return transitions


# ---------------------------------------------------------------------------
# Analysis duration & loop estimation
# ---------------------------------------------------------------------------


def _estimate_loops(
    console_msgs: list[_RawConsoleMsg],
    log_data: LogData,
    configured_period_s: float,
) -> tuple[float, str, int]:
    """Determine analysis duration and estimated total loops.

    Returns:
        ``(duration_s, basis, estimated_loops)``
    """
    from logreader.processor import get_time_range

    time_range = get_time_range(log_data)
    if time_range is not None:
        duration_s = time_range[1] - time_range[0]
        if duration_s > 0:
            loops = int(duration_s / configured_period_s)
            return duration_s, "full_log_range", loops

    # Fallback: console signal range
    if console_msgs:
        first_s = console_msgs[0].timestamp_us / 1_000_000.0
        last_s = console_msgs[-1].timestamp_us / 1_000_000.0
        duration_s = last_s - first_s
        if duration_s > 0:
            loops = int(duration_s / configured_period_s)
            return duration_s, "console_range", loops

    return 0.0, "full_log_range", 0


# ---------------------------------------------------------------------------
# Detect configured period
# ---------------------------------------------------------------------------


def _detect_period(msgs: list[_RawConsoleMsg]) -> float:
    """Extract the configured loop period from the first overrun message.

    Parses ``"Loop time of 0.02s overrun"`` and returns the period in
    seconds.  Falls back to ``DEFAULT_LOOP_PERIOD_S`` if none found.
    """
    for msg in msgs:
        m = _OVERRUN_RE.search(msg.text)
        if m:
            return float(m.group(1))
    return DEFAULT_LOOP_PERIOD_S


# ---------------------------------------------------------------------------
# Total scheduler-overrun count
# ---------------------------------------------------------------------------


def _count_all_scheduler_overruns(msgs: list[_RawConsoleMsg]) -> int:
    """Count every ``CommandScheduler loop overrun`` message."""
    return sum(1 for m in msgs if _is_scheduler_overrun(m.text))


# ---------------------------------------------------------------------------
# Output formatting helpers
# ---------------------------------------------------------------------------


def _format_summary(summary: OverrunSummary) -> str:
    """Render the summary section of the report."""
    period_ms = summary.configured_period_ms
    period_hz = 1000.0 / period_ms if period_ms > 0 else 0

    lines = [
        f"  Configured period:   {period_ms:.1f} ms ({period_hz:.0f} Hz)",
        f"  Analysis duration:   {summary.analysis_duration_s:.1f} s  "
        f"(basis: {summary.loop_estimation_basis})",
        f"  Estimated loops:     ~{summary.estimated_total_loops:,}",
        f"  Main-loop overruns:  {summary.overrun_count} "
        f"({summary.overrun_pct:.1f}%)",
        f"  Scheduler markers:   {summary.scheduler_overrun_count}  "
        f"(diagnostic; may co-occur with main-loop overruns)",
    ]

    dist = summary.total_duration_distribution
    if dist and dist.get("max", 0) > 0:
        lines.append("")
        lines.append("  Overrun severity:")
        for key in ["p50", "p90", "p95", "p99", "max"]:
            val = dist.get(key, 0.0)
            multiple = val / period_ms if period_ms > 0 else 0
            lines.append(
                f"    {key} total time:    {val:.1f} ms  ({multiple:.1f}× budget)"
            )

    return "\n".join(lines)


def _format_phase_stats(phase_stats: list[PhaseOverrunStats]) -> str:
    """Render the per-phase section."""
    lines = ["", "  Per-phase:"]
    for ps in phase_stats:
        label = ps.phase.capitalize()
        lines.append(
            f"    {label:12s} ({ps.duration_s:.0f} s):   "
            f"{ps.overrun_count} overruns ({ps.overrun_rate_hz:.2f}/s)"
        )
    return "\n".join(lines)


def _format_component_table(
    components: list[ComponentStats],
    *,
    title: str = "Top-level loop epochs",
    max_rows: int | None = None,
) -> tuple[list[str], list[dict[str, Any]]]:
    """Build columns and rows for the component table."""
    columns = [
        "Component",
        "Category",
        "Samples",
        "Median ms",
        "Mean ms",
        "p95 ms",
        "Max ms",
        "Contrib%",
    ]

    display = components[:max_rows] if max_rows else components
    rows: list[dict[str, Any]] = []
    for c in display:
        rows.append(
            {
                "Component": c.name,
                "Category": c.category.value,
                "Samples": c.sample_count,
                "Median ms": round(c.median_ms, 2),
                "Mean ms": round(c.mean_ms, 2),
                "p95 ms": round(c.p95_ms, 2),
                "Max ms": round(c.max_ms, 2),
                "Contrib%": f"{c.mean_contribution_pct:.1f}%",
            }
        )
    return columns, rows


def _format_worst_overruns(
    events: list[OverrunEvent],
    count: int = 5,
) -> str:
    """Render the worst overruns detail section."""
    if not events:
        return ""

    sorted_events = sorted(events, key=lambda e: e.total_duration_ms, reverse=True)
    top = sorted_events[:count]

    lines = [f"\n  Worst {count} overrun loops:"]
    for i, ev in enumerate(top, 1):
        t_s = ev.timestamp_us / 1_000_000.0
        phase_str = f" ({ev.phase})" if ev.phase else ""
        lines.append(
            f"    #{i}  {t_s:.1f}s{phase_str}  {ev.total_duration_ms:.1f}ms total"
        )
        # Top 3 epoch contributors
        sorted_epochs = sorted(
            ev.epoch_timings.items(), key=lambda x: x[1], reverse=True
        )
        top_epochs = sorted_epochs[:3]
        epoch_strs = [f"{n}: {v*1000:.1f}ms" for n, v in top_epochs]
        lines.append(f"      {', '.join(epoch_strs)}")
        # Running commands (with unnamed flagged)
        if ev.running_commands is not None and ev.running_commands:
            named = []
            for cmd in ev.running_commands:
                if cmd in UNNAMED_COMMAND_NAMES:
                    named.append(f"{cmd} ⚠")
                else:
                    named.append(cmd)
            lines.append(f"      Running: {', '.join(named)}")
        # Nearby events (max 3)
        for ne in ev.nearby_events[:3]:
            lines.append(f'      Nearby: "{ne.text}"')

    return "\n".join(lines)


def _format_phase_transitions(transitions: list[PhaseTransitionDetail]) -> str:
    """Render phase transition details."""
    if not transitions:
        return ""

    lines = ["\n  Phase transitions:"]
    for pt in transitions:
        t_s = pt.timestamp_us / 1_000_000.0
        from_str = pt.from_phase or "?"
        init_str = f"  {from_str}Init → " if pt.init_method_ms else ""
        init_time = f" ({pt.init_method_ms:.0f}ms)" if pt.init_method_ms else ""
        lines.append(
            f"    → {pt.to_phase.capitalize():12s} at {t_s:.1f}s  "
            f"{pt.overruns_nearby} overruns ±{PHASE_TRANSITION_WINDOW_S:.0f}s"
            f"{init_time}"
        )
        if pt.commands_initialized:
            lines.append(f"      initialized: {', '.join(pt.commands_initialized)}")
        if pt.commands_interrupted:
            lines.append(f"      interrupted: {', '.join(pt.commands_interrupted)}")
        if pt.commands_finished:
            lines.append(f"      finished: {', '.join(pt.commands_finished)}")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Public analysis function
# ---------------------------------------------------------------------------


def analyse_loop_overruns(
    log_data: LogData,
    *,
    detail: bool = False,
    phases: bool = True,
    worst: int = 5,
    min_time: float | None = None,
) -> OverrunSummary:
    """Run the full loop-overrun analysis.

    Parameters:
        log_data: Parsed log data.
        detail: If ``True``, attach nearby-event context.
        phases: If ``True``, compute per-phase breakdown (requires
            match_phases data).
        worst: Number of worst overruns to report.
        min_time: Minimum mean ms to include in component table.

    Returns:
        A fully populated ``OverrunSummary``.
    """
    console_msgs = _parse_console_signal(log_data)
    command_events = _parse_messages_signal(log_data)

    if not console_msgs:
        # No console signal
        duration_s, basis, est_loops = _estimate_loops(
            [], log_data, DEFAULT_LOOP_PERIOD_S
        )
        return OverrunSummary(
            configured_period_ms=DEFAULT_LOOP_PERIOD_S * 1000.0,
            analysis_duration_s=duration_s,
            loop_estimation_basis=basis,
            estimated_total_loops=est_loops,
            overrun_count=0,
            scheduler_overrun_count=0,
            overrun_pct=0.0,
            total_duration_distribution=_duration_distribution([]),
            components=[],
            phase_stats=None,
            phase_transitions=None,
            overrun_events=[],
            burst_periods=[],
        )

    configured_period_s = _detect_period(console_msgs)
    events = _group_overrun_events(console_msgs, configured_period_s)
    sched_total = _count_all_scheduler_overruns(console_msgs)

    duration_s, basis, est_loops = _estimate_loops(
        console_msgs, log_data, configured_period_s
    )
    overrun_pct = (len(events) / est_loops * 100.0) if est_loops > 0 else 0.0

    # Phase annotation
    timeline = None
    phase_stats: list[PhaseOverrunStats] | None = None
    phase_transitions: list[PhaseTransitionDetail] | None = None

    if phases:
        timeline = _annotate_phases(events, log_data)
        if timeline is not None:
            phase_stats = _compute_phase_stats(events, timeline)
            phase_transitions = _compute_phase_transitions(
                events, timeline, command_events
            )

    # Nearby event correlation
    if detail:
        _attach_nearby_events(events, console_msgs, command_events)

    # Running command context (always — it's cheap)
    _attach_running_commands(events, log_data)

    # Component stats (top-level only for global table)
    components = _compute_component_stats(events, top_level_only=True)
    if min_time is not None:
        components = [c for c in components if c.mean_ms >= min_time]

    return OverrunSummary(
        configured_period_ms=configured_period_s * 1000.0,
        analysis_duration_s=round(duration_s, 1),
        loop_estimation_basis=basis,
        estimated_total_loops=est_loops,
        overrun_count=len(events),
        scheduler_overrun_count=sched_total,
        overrun_pct=round(overrun_pct, 1),
        total_duration_distribution=_duration_distribution(events),
        components=components,
        phase_stats=phase_stats,
        phase_transitions=phase_transitions,
        overrun_events=events,
        burst_periods=_detect_bursts(events),
    )


# ---------------------------------------------------------------------------
# Analyzer class
# ---------------------------------------------------------------------------


@register_analyzer
class LoopOverrunAnalyzer(BaseAnalyzer):
    """Detect and report robot main-loop overruns.

    Parses WPILib Tracer timing breakdowns from console log messages to
    identify which subsystems and commands consume the most time.
    """

    name = "loop-overruns"
    description = "Detect and report robot main-loop timing overruns"

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--detail",
            action="store_true",
            default=False,
            help="Show worst overruns and nearby events",
        )
        parser.add_argument(
            "--phases",
            action="store_true",
            default=False,
            help="Show per-phase breakdown",
        )
        parser.add_argument(
            "--worst",
            type=int,
            default=5,
            help="Number of worst overruns to show (default: 5)",
        )
        parser.add_argument(
            "--min-time",
            type=float,
            default=None,
            help="Only show components with mean ≥ this many ms",
        )

    def run(self, log_data: LogData, **options: Any) -> AnalysisResult:
        detail = bool(options.get("detail", False))
        show_phases = bool(options.get("phases", False))
        worst = int(options.get("worst", 5))
        min_time = options.get("min_time")

        summary = analyse_loop_overruns(
            log_data,
            detail=detail,
            phases=show_phases,
            worst=worst,
            min_time=float(min_time) if min_time is not None else None,
        )

        # No console signal at all
        if log_data.get_signal("console") is None:
            return AnalysisResult(
                analyzer_name=self.name,
                title="Loop Overrun Analysis",
                summary="No console signal found — loop timing data unavailable.",
                extra={"summary": summary},
            )

        # Zero overruns
        if summary.overrun_count == 0:
            return AnalysisResult(
                analyzer_name=self.name,
                title="Loop Overrun Analysis",
                summary=(
                    "No loop overruns detected — all loops completed within the "
                    f"configured {summary.configured_period_ms:.1f} ms period.\n"
                    f"  Estimated loops: ~{summary.estimated_total_loops:,}"
                ),
                extra={"summary": summary},
            )

        # Build report text
        summary_text = _format_summary(summary)

        if show_phases and summary.phase_stats:
            summary_text += _format_phase_stats(summary.phase_stats)

        if detail:
            summary_text += _format_worst_overruns(summary.overrun_events, worst)

        if show_phases and summary.phase_transitions:
            summary_text += _format_phase_transitions(summary.phase_transitions)

        # Component table
        columns, rows = _format_component_table(summary.components, max_rows=20)

        return AnalysisResult(
            analyzer_name=self.name,
            title="Loop Overrun Analysis",
            summary=summary_text,
            columns=columns,
            rows=rows,
            extra={
                "summary": summary,
                "components": summary.components,
                "phase_stats": summary.phase_stats,
                "phase_transitions": summary.phase_transitions,
                "overrun_events": summary.overrun_events,
            },
        )
