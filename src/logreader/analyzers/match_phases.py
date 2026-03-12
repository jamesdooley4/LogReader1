"""Match-phase detection — identify auto / teleop / disabled boundaries.

This module is both a **standalone analyzer** (``match-phases``) and a
**shared service** that other analyzers import to partition results by
game phase.

Public API for other analyzers::

    from logreader.analyzers.match_phases import (
        MatchPhase,
        MatchPhaseTimeline,
        PhaseInterval,
        detect_match_phases,
        classify_events_by_phase,
        slice_signal_by_phase,
        phase_durations,
    )
"""

from __future__ import annotations

import re
from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Sequence, TypeVar

from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer
from logreader.models import LogData, SignalData, SignalType

T = TypeVar("T")

# ---------------------------------------------------------------------------
# Phase model
# ---------------------------------------------------------------------------


class MatchPhase(Enum):
    """The mode the robot is operating in."""

    DISABLED = "disabled"
    AUTONOMOUS = "auto"
    TELEOP = "teleop"
    TEST = "test"


@dataclass
class PhaseInterval:
    """A single contiguous interval where the robot is in one phase.

    Attributes:
        phase: Which mode the robot is in.
        start_us: Start timestamp in microseconds (inclusive).
        end_us: End timestamp in microseconds (inclusive).
    """

    phase: MatchPhase
    start_us: int
    end_us: int

    @property
    def start_s(self) -> float:
        return self.start_us / 1_000_000.0

    @property
    def end_s(self) -> float:
        return self.end_us / 1_000_000.0

    @property
    def duration_s(self) -> float:
        return (self.end_us - self.start_us) / 1_000_000.0

    def contains_us(self, timestamp_us: int) -> bool:
        return self.start_us <= timestamp_us < self.end_us

    def overlaps(self, start_us: int, end_us: int) -> bool:
        return self.start_us <= end_us and self.end_us >= start_us


@dataclass
class MatchPhaseTimeline:
    """Complete phase timeline for a log file.

    The intervals list is sorted by start time and covers the full log
    duration with no gaps (disabled fills the space between active phases).
    """

    intervals: list[PhaseInterval] = field(default_factory=list)

    @property
    def has_match(self) -> bool:
        """True if this looks like a real match (has both auto and teleop)."""
        phases = {iv.phase for iv in self.intervals}
        return MatchPhase.AUTONOMOUS in phases and MatchPhase.TELEOP in phases

    @property
    def appears_truncated(self) -> bool:
        """True if this looks like a partial match (possible crash/reboot)."""
        total_enabled = sum(
            iv.duration_s for iv in self.intervals if iv.phase != MatchPhase.DISABLED
        )
        return self.has_match and total_enabled < 120.0

    @property
    def appears_post_reboot(self) -> bool:
        """True if this looks like the second half of a rebooted match."""
        if not self.intervals:
            return False
        first_enabled = next(
            (iv for iv in self.intervals if iv.phase != MatchPhase.DISABLED),
            None,
        )
        return (
            first_enabled is not None
            and first_enabled.phase == MatchPhase.TELEOP
            and not self.has_match
        )

    def phase_at(self, timestamp_us: int) -> MatchPhase:
        """Return the phase active at a given timestamp.

        Intervals use half-open ranges ``[start, end)``.
        """
        for iv in self.intervals:
            if iv.contains_us(timestamp_us):
                return iv.phase
        return MatchPhase.DISABLED

    def intervals_for(self, phase: MatchPhase) -> list[PhaseInterval]:
        """Return all intervals matching the given phase."""
        return [iv for iv in self.intervals if iv.phase == phase]

    def auto_interval(self) -> PhaseInterval | None:
        """The first (usually only) autonomous interval, or ``None``."""
        autos = self.intervals_for(MatchPhase.AUTONOMOUS)
        return autos[0] if autos else None

    def teleop_interval(self) -> PhaseInterval | None:
        """The first (usually only) teleop interval, or ``None``."""
        teleops = self.intervals_for(MatchPhase.TELEOP)
        return teleops[0] if teleops else None


# ---------------------------------------------------------------------------
# Signal auto-detection
# ---------------------------------------------------------------------------

_MODE_PATTERNS: dict[str, list[re.Pattern[str]]] = {
    "autonomous": [
        re.compile(r".*(DS[:/]?auto|DSAutonomous|FMSInfo.*Auto).*", re.I),
    ],
    "teleop": [
        re.compile(r".*(DS[:/]?teleop|FMSInfo.*Teleop).*", re.I),
    ],
    "test": [
        re.compile(r".*(DS[:/]?test|FMSInfo.*Test).*", re.I),
    ],
    "estop": [
        re.compile(r".*(DS[:/]?e-?stop|DSEStop).*", re.I),
    ],
}

# Maps a mode key to the MatchPhase it activates when True.
_MODE_TO_PHASE: dict[str, MatchPhase] = {
    "autonomous": MatchPhase.AUTONOMOUS,
    "teleop": MatchPhase.TELEOP,
    "test": MatchPhase.TEST,
}


def _find_mode_signals(
    log_data: LogData,
) -> dict[str, str]:
    """Auto-detect boolean mode signals.

    Returns:
        A dict mapping mode key (``"autonomous"``, ``"teleop"``, etc.)
        to the signal name found in *log_data*.  Only keys with a
        detected signal are included.
    """
    found: dict[str, str] = {}
    boolean_names = [
        name
        for name, sig in log_data.signals.items()
        if sig.info.type == SignalType.BOOLEAN
    ]
    for mode_key, patterns in _MODE_PATTERNS.items():
        for pattern in patterns:
            for name in boolean_names:
                if pattern.match(name):
                    found[mode_key] = name
                    break
            if mode_key in found:
                break
    return found


# ---------------------------------------------------------------------------
# Detection algorithm
# ---------------------------------------------------------------------------

# Minimum interval duration to keep (100 ms) — filters signal jitter.
_MIN_INTERVAL_US = 100_000


def _get_time_range(log_data: LogData) -> tuple[int, int] | None:
    """Return (min_us, max_us) across all signals, or ``None``."""
    min_ts: int | None = None
    max_ts: int | None = None
    for sig in log_data.signals.values():
        if not sig.values:
            continue
        first = sig.values[0].timestamp_us
        last = sig.values[-1].timestamp_us
        if min_ts is None or first < min_ts:
            min_ts = first
        if max_ts is None or last > max_ts:
            max_ts = last
    if min_ts is None or max_ts is None:
        return None
    return min_ts, max_ts


def detect_match_phases(log_data: LogData) -> MatchPhaseTimeline | None:
    """Detect match phases from mode signals in the log.

    Returns ``None`` if no mode signals are found (the log has no phase
    data).  Returns a :class:`MatchPhaseTimeline` with at least one
    interval otherwise.
    """
    mode_signals = _find_mode_signals(log_data)
    if not mode_signals:
        return None

    time_range = _get_time_range(log_data)
    if time_range is None:
        return None

    log_start_us, log_end_us = time_range

    # Collect all (timestamp_us, mode_key, bool_value) transitions,
    # sorted by timestamp.
    transitions: list[tuple[int, str, bool]] = []
    for mode_key, sig_name in mode_signals.items():
        sig = log_data.get_signal(sig_name)
        if sig is None:
            continue
        for tv in sig.values:
            transitions.append((tv.timestamp_us, mode_key, bool(tv.value)))

    transitions.sort(key=lambda x: x[0])

    if not transitions:
        return MatchPhaseTimeline(
            intervals=[
                PhaseInterval(
                    phase=MatchPhase.DISABLED,
                    start_us=log_start_us,
                    end_us=log_end_us,
                )
            ]
        )

    # Track the current state of each mode signal.
    state: dict[str, bool] = {key: False for key in mode_signals}

    def _resolve_phase() -> MatchPhase:
        """Determine the active phase from current signal states."""
        if state.get("estop", False):
            return MatchPhase.DISABLED
        if state.get("autonomous", False):
            return MatchPhase.AUTONOMOUS
        if state.get("teleop", False):
            return MatchPhase.TELEOP
        if state.get("test", False):
            return MatchPhase.TEST
        return MatchPhase.DISABLED

    # Build intervals by walking transitions.
    intervals: list[PhaseInterval] = []
    current_phase = MatchPhase.DISABLED
    current_start = log_start_us

    for ts, mode_key, value in transitions:
        state[mode_key] = value
        new_phase = _resolve_phase()
        if new_phase != current_phase:
            if ts > current_start:
                intervals.append(
                    PhaseInterval(
                        phase=current_phase,
                        start_us=current_start,
                        end_us=ts,
                    )
                )
            current_phase = new_phase
            current_start = ts

    # Close the final interval.
    if log_end_us > current_start:
        intervals.append(
            PhaseInterval(
                phase=current_phase,
                start_us=current_start,
                end_us=log_end_us,
            )
        )

    # Merge adjacent DISABLED intervals.
    merged: list[PhaseInterval] = []
    for iv in intervals:
        if (
            merged
            and merged[-1].phase == MatchPhase.DISABLED
            and iv.phase == MatchPhase.DISABLED
        ):
            merged[-1] = PhaseInterval(
                phase=MatchPhase.DISABLED,
                start_us=merged[-1].start_us,
                end_us=iv.end_us,
            )
        else:
            merged.append(iv)

    # Drop very short intervals (< 100 ms) to filter jitter,
    # but keep any disabled gap between auto and teleop.
    filtered: list[PhaseInterval] = []
    for i, iv in enumerate(merged):
        if iv.duration_s * 1_000_000 < _MIN_INTERVAL_US:
            # Keep the auto→teleop disabled gap.
            is_auto_teleop_gap = (
                iv.phase == MatchPhase.DISABLED
                and i > 0
                and i < len(merged) - 1
                and merged[i - 1].phase == MatchPhase.AUTONOMOUS
                and merged[i + 1].phase == MatchPhase.TELEOP
            )
            if not is_auto_teleop_gap:
                # Extend the previous interval to cover this gap.
                if filtered:
                    filtered[-1] = PhaseInterval(
                        phase=filtered[-1].phase,
                        start_us=filtered[-1].start_us,
                        end_us=iv.end_us,
                    )
                continue
        filtered.append(iv)

    if not filtered:
        filtered = [
            PhaseInterval(
                phase=MatchPhase.DISABLED,
                start_us=log_start_us,
                end_us=log_end_us,
            )
        ]

    return MatchPhaseTimeline(intervals=filtered)


# ---------------------------------------------------------------------------
# Public utilities for other analyzers
# ---------------------------------------------------------------------------


def classify_events_by_phase(
    timeline: MatchPhaseTimeline,
    events: Sequence[tuple[int | float, T]],
    *,
    timestamps_are_seconds: bool = False,
    grace_period_s: float = 0.0,
) -> dict[MatchPhase, list[T]]:
    """Partition a list of (timestamp, event) pairs by match phase.

    Parameters:
        timeline: The phase timeline.
        events: Sequence of ``(timestamp, event_object)`` pairs.
        timestamps_are_seconds: If ``True``, timestamps are float seconds;
            if ``False`` (default), int microseconds.
        grace_period_s: Extend each phase boundary by this many seconds
            when classifying.  Events within *grace_period_s* after a
            phase ends are still attributed to that phase.
    """
    grace_us = int(grace_period_s * 1_000_000)
    result: dict[MatchPhase, list[T]] = defaultdict(list)

    for ts_raw, event in events:
        if timestamps_are_seconds:
            ts_us = int(float(ts_raw) * 1_000_000)
        else:
            ts_us = int(ts_raw)

        # Try exact match first.
        phase = timeline.phase_at(ts_us)
        if phase != MatchPhase.DISABLED or grace_us == 0:
            result[phase].append(event)
            continue

        # If we landed in DISABLED, check grace period of preceding
        # non-disabled intervals.
        assigned = False
        for iv in timeline.intervals:
            if iv.phase == MatchPhase.DISABLED:
                continue
            if iv.end_us <= ts_us <= iv.end_us + grace_us:
                result[iv.phase].append(event)
                assigned = True
                break
        if not assigned:
            result[MatchPhase.DISABLED].append(event)

    return dict(result)


def slice_signal_by_phase(
    timeline: MatchPhaseTimeline,
    signal: SignalData,
    phase: MatchPhase,
    *,
    grace_period_s: float = 0.0,
) -> SignalData:
    """Extract the portion of a signal that falls within a given phase.

    Like ``processor.slice_by_time`` but driven by match phases.
    """
    grace_us = int(grace_period_s * 1_000_000)
    matching_intervals = timeline.intervals_for(phase)

    filtered = []
    for tv in signal.values:
        ts = tv.timestamp_us
        for iv in matching_intervals:
            if iv.start_us <= ts < iv.end_us + grace_us:
                filtered.append(tv)
                break

    return SignalData(info=signal.info, values=filtered)


def phase_durations(timeline: MatchPhaseTimeline) -> dict[MatchPhase, float]:
    """Return total duration in seconds for each phase."""
    durations: dict[MatchPhase, float] = {}
    for iv in timeline.intervals:
        durations[iv.phase] = durations.get(iv.phase, 0.0) + iv.duration_s
    return durations


# ---------------------------------------------------------------------------
# Standalone analyzer
# ---------------------------------------------------------------------------


@register_analyzer
class MatchPhasesAnalyzer(BaseAnalyzer):
    """Detect match phase boundaries (auto / teleop / disabled).

    Identifies autonomous, teleoperated, test, and disabled intervals
    from Driver Station mode signals.  Provides a timeline that other
    analyzers can use for per-phase breakdowns.
    """

    name = "match-phases"
    description = "Detect match phase boundaries (auto / teleop / disabled)"

    def run(self, log_data: LogData, **options: Any) -> AnalysisResult:
        timeline = detect_match_phases(log_data)

        if timeline is None:
            return AnalysisResult(
                analyzer_name=self.name,
                title="Match Phase Timeline",
                summary="No Driver Station mode signals found in this log.",
            )

        durations = phase_durations(timeline)

        # Compute log duration.
        if timeline.intervals:
            log_start = timeline.intervals[0].start_s
            log_end = timeline.intervals[-1].end_s
            log_duration = log_end - log_start
        else:
            log_duration = 0.0

        # Build summary.
        summary_lines = [
            f"  Log duration:   {log_duration:.1f} s",
            f"  Match detected: {'Yes' if timeline.has_match else 'No'}",
        ]

        if timeline.appears_truncated:
            total_enabled = sum(
                iv.duration_s
                for iv in timeline.intervals
                if iv.phase != MatchPhase.DISABLED
            )
            summary_lines.insert(
                0,
                f"  \u26a0 This log appears to be a truncated match "
                f"({total_enabled:.1f} s enabled, expected ~150 s).\n"
                f"    Possible mid-match crash or reboot. Look for a second "
                f"log file from the same match.",
            )

        if timeline.appears_post_reboot:
            summary_lines.insert(
                0,
                "  \u26a0 This log appears to be the second half of a "
                "rebooted match (no autonomous phase).",
            )

        # Phase duration summary.
        summary_lines.append("")
        for phase in [
            MatchPhase.AUTONOMOUS,
            MatchPhase.TELEOP,
            MatchPhase.TEST,
            MatchPhase.DISABLED,
        ]:
            dur = durations.get(phase, 0.0)
            if dur > 0 or phase in (MatchPhase.AUTONOMOUS, MatchPhase.TELEOP):
                count = len(timeline.intervals_for(phase))
                extra = f" ({count} interval{'s' if count != 1 else ''})"
                summary_lines.append(
                    f"    {phase.value.capitalize():12s} {dur:6.1f} s{extra}"
                )

        # Interval table.
        rows: list[dict[str, Any]] = []
        for iv in timeline.intervals:
            rows.append(
                {
                    "Phase": iv.phase.value,
                    "Start(s)": round(iv.start_s, 1),
                    "End(s)": round(iv.end_s, 1),
                    "Duration(s)": round(iv.duration_s, 1),
                }
            )

        return AnalysisResult(
            analyzer_name=self.name,
            title="Match Phase Timeline",
            summary="\n".join(summary_lines),
            columns=["Phase", "Start(s)", "End(s)", "Duration(s)"],
            rows=rows,
            extra={
                "timeline": timeline,
                "has_match": timeline.has_match,
                "durations": durations,
            },
        )
