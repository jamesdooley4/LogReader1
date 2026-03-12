"""Launch counter analyzer — count game-element launches from flywheel velocity.

Detects velocity dips caused by game elements passing through a flywheel.
Each launch extracts energy, causing a brief velocity drop followed by
motor recovery.  Works during sustained rapid-fire bursts where the
baseline velocity declines between shots.

See PLAN.md § "Launch Counter Design" for the full analysis behind this
algorithm.
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from typing import Any

from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer
from logreader.models import LogData, SignalData

# ---------------------------------------------------------------------------
# Signal auto-detection
# ---------------------------------------------------------------------------
_VELOCITY_PATTERNS = [
    re.compile(r".*launcher/velocity$", re.IGNORECASE),
    re.compile(r".*shooter/velocity$", re.IGNORECASE),
    re.compile(r".*flywheel/velocity$", re.IGNORECASE),
    re.compile(r".*flywheel.*velocity.*", re.IGNORECASE),
]


def _find_velocity_signal(log_data: LogData) -> str | None:
    """Auto-detect the flywheel velocity signal name."""
    for pattern in _VELOCITY_PATTERNS:
        for name in log_data.signal_names():
            if pattern.match(name):
                return name
    return None


# ---------------------------------------------------------------------------
# Detection result
# ---------------------------------------------------------------------------
@dataclass
class LaunchEvent:
    """A single detected launch."""

    time_s: float
    dip_velocity: float
    drop_magnitude: float
    baseline: float


# ---------------------------------------------------------------------------
# Core detection algorithm
# ---------------------------------------------------------------------------

# Default parameters — tuned against real FRC match data
DEFAULT_AT_SPEED = 55.0
DEFAULT_DROP_MIN = 8.0
DEFAULT_FLOOR = 30.0
DEFAULT_RECOVERY = 5.0
DEFAULT_DEBOUNCE_MS = 80
DEFAULT_WINDOW = 5


def _rolling_max(values: list[float], window: int) -> list[float]:
    """Compute a rolling maximum over *window* samples."""
    result: list[float] = []
    for i in range(len(values)):
        start = max(0, i - window)
        result.append(max(values[start : i + 1]))
    return result


def detect_launches(
    signal: SignalData,
    *,
    at_speed: float = DEFAULT_AT_SPEED,
    drop_min: float = DEFAULT_DROP_MIN,
    floor: float = DEFAULT_FLOOR,
    recovery_min: float = DEFAULT_RECOVERY,
    debounce_ms: float = DEFAULT_DEBOUNCE_MS,
    window: int = DEFAULT_WINDOW,
) -> list[LaunchEvent]:
    """Run launch detection on a velocity signal.

    Parameters:
        signal: The flywheel velocity ``SignalData``.
        at_speed: Velocity threshold to begin detection.
        drop_min: Minimum velocity drop from baseline to enter a dip.
        floor: Hard floor — stop detecting below this velocity.
        recovery_min: Minimum recovery from dip minimum to confirm a launch.
        debounce_ms: Minimum gap in ms between consecutive launches.
        window: Rolling-max window size for baseline tracking.

    Returns:
        List of ``LaunchEvent`` instances in chronological order.
    """
    if len(signal.values) < 3:
        return []

    velocities = [float(v.value) for v in signal.values]
    timestamps = [v.timestamp_us for v in signal.values]
    baselines = _rolling_max(velocities, window)

    launches: list[LaunchEvent] = []
    last_launch_us = 0
    detecting = False
    in_dip = False
    dip_min_val = 0.0
    dip_min_time = 0
    dip_baseline = 0.0

    for i in range(1, len(velocities)):
        val = velocities[i]
        t_us = timestamps[i]
        # Use baseline from slightly before current position to avoid
        # the dip itself pulling down the baseline
        baseline = baselines[max(0, i - 2)]

        # Enter detection mode when at speed
        if not detecting:
            if val >= at_speed:
                detecting = True
            continue

        # Exit detection mode if below floor
        if val < floor and not in_dip:
            detecting = False
            continue

        if not in_dip:
            if baseline >= at_speed and (baseline - val) >= drop_min and val >= floor:
                in_dip = True
                dip_min_val = val
                dip_min_time = t_us
                dip_baseline = baseline
        else:
            # Track the dip minimum (but only if still above floor)
            if val < dip_min_val and val >= floor:
                dip_min_val = val
                dip_min_time = t_us

            # Check for recovery
            recovery = val - dip_min_val
            if recovery >= recovery_min:
                gap_ms = (dip_min_time - last_launch_us) / 1000.0
                if gap_ms >= debounce_ms:
                    launches.append(
                        LaunchEvent(
                            time_s=dip_min_time / 1_000_000.0,
                            dip_velocity=dip_min_val,
                            drop_magnitude=dip_baseline - dip_min_val,
                            baseline=dip_baseline,
                        )
                    )
                    last_launch_us = dip_min_time
                in_dip = False
            elif val < floor:
                # Dropped below floor — stop detection
                in_dip = False
                detecting = False

    return launches


# ---------------------------------------------------------------------------
# Spinning period detection
# ---------------------------------------------------------------------------
@dataclass
class SpinningPeriod:
    """A continuous period where the flywheel is spinning."""

    start_s: float
    end_s: float
    launches: int = 0
    rate: float = 0.0

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s


def _find_spinning_periods(
    signal: SignalData,
    at_speed: float,
    merge_gap: float = 2.0,
) -> list[SpinningPeriod]:
    """Identify periods where the flywheel is spinning above *at_speed*.

    Adjacent periods separated by less than *merge_gap* seconds are merged.
    """
    periods: list[SpinningPeriod] = []
    in_spin = False
    spin_start = 0.0

    for pt in signal.values:
        t_s = pt.timestamp_us / 1_000_000.0
        if pt.value > at_speed and not in_spin:
            in_spin = True
            spin_start = t_s
        elif pt.value <= at_speed * 0.4 and in_spin:
            in_spin = False
            periods.append(SpinningPeriod(start_s=spin_start, end_s=t_s))

    if in_spin:
        periods.append(
            SpinningPeriod(
                start_s=spin_start,
                end_s=signal.values[-1].timestamp_us / 1_000_000.0,
            )
        )

    if not periods:
        return []

    # Merge close periods
    merged = [periods[0]]
    for p in periods[1:]:
        if p.start_s - merged[-1].end_s < merge_gap:
            merged[-1] = SpinningPeriod(start_s=merged[-1].start_s, end_s=p.end_s)
        else:
            merged.append(p)

    return merged


# ---------------------------------------------------------------------------
# Analyzer
# ---------------------------------------------------------------------------
@register_analyzer
class LaunchCounterAnalyzer(BaseAnalyzer):
    """Count game-element launches from flywheel velocity dips.

    Detects sharp velocity drops caused by game elements extracting
    energy from the flywheel, including during rapid-fire bursts where
    the baseline velocity declines between shots.
    """

    name = "launch-counter"
    description = "Count game-element launches from flywheel velocity dips"

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--velocity-signal",
            help="Explicit velocity signal name (default: auto-detect)",
        )
        parser.add_argument(
            "--at-speed",
            type=float,
            default=DEFAULT_AT_SPEED,
            help=f"Velocity threshold to begin detection (default: {DEFAULT_AT_SPEED})",
        )
        parser.add_argument(
            "--drop-min",
            type=float,
            default=DEFAULT_DROP_MIN,
            help=f"Minimum velocity drop to count as a dip (default: {DEFAULT_DROP_MIN})",
        )
        parser.add_argument(
            "--floor",
            type=float,
            default=DEFAULT_FLOOR,
            help=f"Stop detecting below this velocity (default: {DEFAULT_FLOOR})",
        )
        parser.add_argument(
            "--detail",
            action="store_true",
            default=False,
            help="Show every individual launch event",
        )

    def run(self, log_data: LogData, **options: Any) -> AnalysisResult:
        # Resolve velocity signal
        vel_name = options.get("velocity_signal") or _find_velocity_signal(log_data)
        if vel_name is None:
            return AnalysisResult(
                analyzer_name=self.name,
                title="Launch Counter",
                summary="No flywheel velocity signal found. "
                "Use --velocity-signal to specify one.",
            )

        vel_sig = log_data.get_signal(vel_name)
        if vel_sig is None or not vel_sig.values:
            return AnalysisResult(
                analyzer_name=self.name,
                title="Launch Counter",
                summary=f"Velocity signal '{vel_name}' not found or empty.",
            )

        # Parameters
        at_speed = float(options.get("at_speed", DEFAULT_AT_SPEED))
        drop_min = float(options.get("drop_min", DEFAULT_DROP_MIN))
        floor_val = float(options.get("floor", DEFAULT_FLOOR))
        show_detail = bool(options.get("detail", False))

        # Detect launches
        launches = detect_launches(
            vel_sig,
            at_speed=at_speed,
            drop_min=drop_min,
            floor=floor_val,
        )

        # Find spinning periods and assign launch counts
        periods = _find_spinning_periods(vel_sig, at_speed)
        for period in periods:
            period_launches = [
                l
                for l in launches
                if period.start_s - 1 <= l.time_s <= period.end_s + 1
            ]
            period.launches = len(period_launches)
            if period.duration_s > 0:
                period.rate = period.launches / period.duration_s

        # Build summary statistics
        total = len(launches)
        drops = [l.drop_magnitude for l in launches] if launches else []
        burst_gaps: list[float] = []
        if len(launches) > 1:
            all_gaps = [
                launches[i + 1].time_s - launches[i].time_s
                for i in range(len(launches) - 1)
            ]
            burst_gaps = [g for g in all_gaps if g < 2.0]

        summary_lines = [
            f"  Signal:          {vel_name}",
            f"  Total launches:  {total}",
        ]
        if drops:
            sorted_drops = sorted(drops)
            summary_lines.append(
                f"  Dip magnitude:   min={min(drops):.1f}  "
                f"median={sorted_drops[len(sorted_drops)//2]:.1f}  "
                f"max={max(drops):.1f}"
            )
        if burst_gaps:
            sorted_gaps = sorted(burst_gaps)
            median_gap_ms = sorted_gaps[len(sorted_gaps) // 2] * 1000
            summary_lines.append(
                f"  Burst gaps:      min={min(burst_gaps)*1000:.0f} ms  "
                f"median={median_gap_ms:.0f} ms  "
                f"max={max(burst_gaps)*1000:.0f} ms"
            )
            if min(burst_gaps) > 0:
                summary_lines.append(f"  Peak burst rate: {1/min(burst_gaps):.1f} /s")

        # Period breakdown table
        period_rows: list[dict[str, Any]] = []
        for p in periods:
            if p.launches > 0 or p.duration_s > 1.0:
                period_rows.append(
                    {
                        "Start(s)": round(p.start_s, 1),
                        "End(s)": round(p.end_s, 1),
                        "Duration": round(p.duration_s, 1),
                        "Launches": p.launches,
                        "Rate(/s)": round(p.rate, 1),
                    }
                )

        columns = ["Start(s)", "End(s)", "Duration", "Launches", "Rate(/s)"]
        rows = period_rows

        # If --detail, replace table with per-launch events
        if show_detail and launches:
            columns = ["#", "Time(s)", "DipVel", "Drop", "Baseline"]
            rows = []
            for idx, l in enumerate(launches, 1):
                rows.append(
                    {
                        "#": idx,
                        "Time(s)": round(l.time_s, 3),
                        "DipVel": round(l.dip_velocity, 2),
                        "Drop": round(l.drop_magnitude, 2),
                        "Baseline": round(l.baseline, 2),
                    }
                )

        return AnalysisResult(
            analyzer_name=self.name,
            title="Launch Counter",
            summary="\n".join(summary_lines),
            columns=columns,
            rows=rows,
            extra={
                "velocity_signal": vel_name,
                "total_launches": total,
                "launches": launches,
                "periods": periods,
                "at_speed": at_speed,
                "drop_min": drop_min,
                "floor": floor_val,
            },
        )
