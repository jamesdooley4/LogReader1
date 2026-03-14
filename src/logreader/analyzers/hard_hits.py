"""Hard-hit / impact analyzer — detect collisions and landings from IMU data.

Analyses Pigeon 2 IMU signals from ``.hoot`` files (converted to ``.wpilog``
via ``owlet``) to detect impact events.  Supports two modes:

- **Licensed** — full 3-axis accelerometer + angular velocity data available.
  Computes acceleration magnitude, detects impacts above a configurable
  threshold, and classifies events as *landing* (ramp/airborne) or
  *collision* (robot/wall).
- **Unlicensed** — only ``AngularVelocityZWorld`` is available.  Detects
  rotational impacts via angular-acceleration spikes.  Cannot distinguish
  landing from collision.

See ``docs/design-hard-hits.md`` for the full design rationale.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any

from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer
from logreader.models import LogData, SignalData


# ---------------------------------------------------------------------------
# Constants / defaults
# ---------------------------------------------------------------------------

# Pigeon 2 signal name patterns (after owlet wpilog conversion)
_ACCEL_X_SUFFIX = "AccelerationX"
_ACCEL_Y_SUFFIX = "AccelerationY"
_ACCEL_Z_SUFFIX = "AccelerationZ"
_ANGVEL_X_SUFFIX = "AngularVelocityXWorld"
_ANGVEL_Y_SUFFIX = "AngularVelocityYWorld"
_ANGVEL_Z_SUFFIX = "AngularVelocityZWorld"
_PITCH_SUFFIX = "Pitch"

# Owlet signal IDs for Pigeon2-10 — used for targeted .hoot extraction.
# These are the hex IDs from ``owlet --scan`` for the specific device.
# Licensed signals (acceleration, all angular velocities, pitch):
_PIGEON2_LICENSED_SIGNAL_IDS = [
    "3ca0a02",  # AccelerationX
    "3cb0a02",  # AccelerationY
    "3cc0a02",  # AccelerationZ
    "3c70a02",  # AngularVelocityXWorld
    "3c80a02",  # AngularVelocityYWorld
    "3c90a02",  # AngularVelocityZWorld
    "3b50a02",  # Yaw
    "3b60a02",  # Pitch
    "3b70a02",  # Roll
]
# Unlicensed-safe subset (these export even with --unlicensed):
_PIGEON2_UNLICENSED_SIGNAL_IDS = [
    "3c90a02",  # AngularVelocityZWorld
    "3b50a02",  # Yaw
]
# Robot state signal for match phase detection (works in both modes):
_ROBOT_STATE_SIGNAL_IDS = [
    "1ff00",  # RobotEnable
    "4ff00",  # RobotMode
]

# Default thresholds for licensed acceleration-based detection
DEFAULT_IMPACT_HARD_G = 1.5
DEFAULT_IMPACT_MODERATE_G = 1.0
DEFAULT_IMPACT_LIGHT_G = 0.5

# Default thresholds for unlicensed angular-velocity detection
DEFAULT_ANG_ACCEL_HARD = 4000.0  # deg/s²
DEFAULT_ANG_ACCEL_MODERATE = 2500.0
DEFAULT_ANG_ACCEL_LIGHT = 1500.0
DEFAULT_OMEGA_STEP_HARD = 150.0  # deg/s over 50ms window
DEFAULT_OMEGA_STEP_MODERATE = 100.0
DEFAULT_OMEGA_STEP_LIGHT = 50.0

# Detection parameters
EVENT_GROUP_WINDOW_US = 300_000  # 300ms — merge nearby triggers
ANG_ACCEL_WINDOW_US = 20_000  # 20ms — dω/dt computation window
OMEGA_STEP_WINDOW_US = 50_000  # 50ms — Δω computation window

# Landing classification parameters
FREEFALL_LOOKBACK_US = 200_000  # look back 200ms for freefall precursor
FREEFALL_THRESHOLD_G = 0.3  # |a| below this → robot is airborne
LATERAL_FRACTION_THRESHOLD = 0.5  # lateral/total; below = landing
RAMP_PITCH_THRESHOLD_DEG = 10.0  # pitch above this → robot was on ramp
RAMP_PITCH_LOOKBACK_US = 2_000_000  # 2s before event for pitch check


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------


class Severity(Enum):
    """Impact severity level."""

    HARD = "HARD"
    MODERATE = "MODERATE"
    LIGHT = "LIGHT"


class Classification(Enum):
    """Impact type classification."""

    LANDING = "landing"
    COLLISION_SPINNING = "collision-spinning"
    COLLISION_HEAD_ON = "collision-head-on"
    COLLISION_DEFLECTION = "collision-deflection"
    ROTATIONAL_ONLY = "rotational-only"


class DetectionMode(Enum):
    """Which detection path was used."""

    LICENSED_ACCEL = "licensed-accel"
    UNLICENSED_ROTATIONAL = "unlicensed-rotational"


# ---------------------------------------------------------------------------
# Data contracts
# ---------------------------------------------------------------------------


@dataclass
class HardHitEvent:
    """A single detected impact event."""

    timestamp_us: int
    severity: Severity
    score: float  # impact_g for licensed, |α| for unlicensed
    mode: DetectionMode
    classification: Classification

    # Licensed fields (None when unlicensed)
    ax_g: float | None = None
    ay_g: float | None = None
    az_g: float | None = None
    impact_g: float | None = None
    lateral_fraction: float | None = None
    freefall_detected: bool | None = None
    pitch_before_deg: float | None = None

    # Angular velocity (always available if present)
    omega_x_dps: float | None = None
    omega_y_dps: float | None = None
    omega_z_dps: float | None = None

    # Unlicensed fields
    angular_accel_dps2: float | None = None

    # Match phase (from RobotMode or DS signals, if available)
    phase: str | None = None

    # Relative timestamps (set by _annotate_phases)
    match_time_s: float | None = None  # seconds since auto start (or first enable)
    phase_time_s: float | None = None  # seconds since current phase started

    @property
    def timestamp_s(self) -> float:
        """Timestamp in seconds."""
        return self.timestamp_us / 1_000_000.0


# ---------------------------------------------------------------------------
# Signal helpers
# ---------------------------------------------------------------------------


def _find_pigeon_signal(log_data: LogData, suffix: str) -> SignalData | None:
    """Find a Pigeon 2 signal by suffix, handling the Phoenix6 prefix."""
    for name, sig in log_data.signals.items():
        if name.endswith(suffix) and ("Pigeon" in name or "pigeon" in name):
            return sig
    return None


def _build_lookup(signal: SignalData) -> dict[int, float]:
    """Build a timestamp → value lookup from a signal."""
    return {v.timestamp_us: v.value for v in signal.values}


def _severity_from_impact(
    impact_g: float,
    hard: float = DEFAULT_IMPACT_HARD_G,
    moderate: float = DEFAULT_IMPACT_MODERATE_G,
    light: float = DEFAULT_IMPACT_LIGHT_G,
) -> Severity | None:
    """Classify severity from impact magnitude (g). Returns None if below light."""
    if impact_g >= hard:
        return Severity.HARD
    if impact_g >= moderate:
        return Severity.MODERATE
    if impact_g >= light:
        return Severity.LIGHT
    return None


def _severity_from_angular(
    abs_alpha: float,
    d_omega: float,
    alpha_hard: float = DEFAULT_ANG_ACCEL_HARD,
    alpha_mod: float = DEFAULT_ANG_ACCEL_MODERATE,
    alpha_light: float = DEFAULT_ANG_ACCEL_LIGHT,
    omega_hard: float = DEFAULT_OMEGA_STEP_HARD,
    omega_mod: float = DEFAULT_OMEGA_STEP_MODERATE,
    omega_light: float = DEFAULT_OMEGA_STEP_LIGHT,
) -> Severity | None:
    """Classify severity from angular metrics. Returns None if below light."""
    # Take the higher severity from either method
    sev_alpha: Severity | None = None
    if abs_alpha >= alpha_hard:
        sev_alpha = Severity.HARD
    elif abs_alpha >= alpha_mod:
        sev_alpha = Severity.MODERATE
    elif abs_alpha >= alpha_light:
        sev_alpha = Severity.LIGHT

    sev_omega: Severity | None = None
    if d_omega >= omega_hard:
        sev_omega = Severity.HARD
    elif d_omega >= omega_mod:
        sev_omega = Severity.MODERATE
    elif d_omega >= omega_light:
        sev_omega = Severity.LIGHT

    # Return the more severe of the two, or whichever is non-None
    order = [Severity.HARD, Severity.MODERATE, Severity.LIGHT]
    for s in order:
        if sev_alpha == s or sev_omega == s:
            return s
    return None


def _group_events(events: list[HardHitEvent], window_us: int) -> list[HardHitEvent]:
    """Group nearby events, keeping the one with the highest score."""
    if not events:
        return []
    events = sorted(events, key=lambda e: e.timestamp_us)
    grouped: list[HardHitEvent] = [events[0]]
    for ev in events[1:]:
        if (ev.timestamp_us - grouped[-1].timestamp_us) <= window_us:
            if ev.score > grouped[-1].score:
                grouped[-1] = ev
        else:
            grouped.append(ev)
    return grouped


def _annotate_phases(events: list[HardHitEvent], log_data: LogData) -> None:
    """Add match-phase labels and relative timestamps to events.

    Modifies events in place, setting:
    - ``phase``: ``"auto"``, ``"teleop"``, ``"disabled"``, ``"test"``
    - ``match_time_s``: seconds since auto start (for matches) or first
      enable (for practice/test logs)
    - ``phase_time_s``: seconds since the start of the current phase
    """
    from logreader.analyzers.match_phases import (
        MatchPhase,
        detect_match_phases,
    )

    timeline = detect_match_phases(log_data)
    if timeline is None:
        return

    # Determine t=0 reference point:
    #   - If the log has an auto period, use auto start (matches the match clock)
    #   - Otherwise use the first enabled interval (practice / test sessions)
    reference_us: int | None = None
    auto = timeline.auto_interval()
    if auto is not None:
        reference_us = auto.start_us
    else:
        first_enabled = next(
            (iv for iv in timeline.intervals if iv.phase != MatchPhase.DISABLED),
            None,
        )
        if first_enabled is not None:
            reference_us = first_enabled.start_us

    for ev in events:
        for interval in timeline.intervals:
            if interval.start_us <= ev.timestamp_us <= interval.end_us:
                ev.phase = interval.phase.value
                ev.phase_time_s = (ev.timestamp_us - interval.start_us) / 1_000_000.0
                break

        if reference_us is not None:
            ev.match_time_s = (ev.timestamp_us - reference_us) / 1_000_000.0


# ---------------------------------------------------------------------------
# Licensed detection
# ---------------------------------------------------------------------------


def detect_licensed(
    accel_x: SignalData,
    accel_y: SignalData,
    accel_z: SignalData,
    angvel_x: SignalData | None,
    angvel_y: SignalData | None,
    angvel_z: SignalData | None,
    pitch_sig: SignalData | None,
    *,
    hard_g: float = DEFAULT_IMPACT_HARD_G,
    moderate_g: float = DEFAULT_IMPACT_MODERATE_G,
    light_g: float = DEFAULT_IMPACT_LIGHT_G,
) -> list[HardHitEvent]:
    """Detect impacts using 3-axis acceleration data (licensed mode).

    Returns a grouped list of ``HardHitEvent`` sorted by time.
    """
    ay_lookup = _build_lookup(accel_y)
    az_lookup = _build_lookup(accel_z)
    ox_lookup = _build_lookup(angvel_x) if angvel_x else {}
    oy_lookup = _build_lookup(angvel_y) if angvel_y else {}
    oz_lookup = _build_lookup(angvel_z) if angvel_z else {}

    # Pre-compute all magnitude samples for freefall lookback
    mag_samples: list[tuple[int, float, float, float, float]] = []
    for v in accel_x.values:
        ts = v.timestamp_us
        ay = ay_lookup.get(ts)
        az = az_lookup.get(ts)
        if ay is None or az is None:
            continue
        ax_val = v.value
        mag = math.sqrt(ax_val * ax_val + ay * ay + az * az)
        mag_samples.append((ts, ax_val, ay, az, mag))

    if not mag_samples:
        return []

    # Build a ts → index map for freefall lookback
    ts_to_idx = {s[0]: i for i, s in enumerate(mag_samples)}

    # Build pitch lookup for ramp detection
    pitch_lookup = _build_lookup(pitch_sig) if pitch_sig else {}

    raw_events: list[HardHitEvent] = []

    for idx, (ts, ax, ay, az, mag) in enumerate(mag_samples):
        impact = abs(mag - 1.0)
        severity = _severity_from_impact(impact, hard_g, moderate_g, light_g)
        if severity is None:
            continue

        # --- Classify: landing vs collision ---
        freefall = _check_freefall(mag_samples, idx, FREEFALL_LOOKBACK_US)
        lateral = math.sqrt(ax * ax + ay * ay)
        lat_frac = lateral / mag if mag > 0 else 0.0
        pitch_before = _check_pitch_before(pitch_sig, ts, RAMP_PITCH_LOOKBACK_US)

        if freefall and lat_frac < LATERAL_FRACTION_THRESHOLD:
            classification = Classification.LANDING
        else:
            # Sub-classify collision using angular velocity
            omega_z = oz_lookup.get(ts, 0.0)
            omega_mag = math.sqrt(
                ox_lookup.get(ts, 0.0) ** 2 + oy_lookup.get(ts, 0.0) ** 2 + omega_z**2
            )
            if impact >= light_g and omega_mag > 50:
                classification = Classification.COLLISION_SPINNING
            elif impact >= light_g and omega_mag <= 50:
                classification = Classification.COLLISION_HEAD_ON
            else:
                classification = Classification.COLLISION_DEFLECTION

        ev = HardHitEvent(
            timestamp_us=ts,
            severity=severity,
            score=impact,
            mode=DetectionMode.LICENSED_ACCEL,
            classification=classification,
            ax_g=ax,
            ay_g=ay,
            az_g=az,
            impact_g=impact,
            lateral_fraction=lat_frac,
            freefall_detected=freefall,
            pitch_before_deg=pitch_before,
            omega_x_dps=ox_lookup.get(ts),
            omega_y_dps=oy_lookup.get(ts),
            omega_z_dps=oz_lookup.get(ts),
        )
        raw_events.append(ev)

    return _group_events(raw_events, EVENT_GROUP_WINDOW_US)


def _check_freefall(
    mag_samples: list[tuple[int, float, float, float, float]],
    current_idx: int,
    lookback_us: int,
) -> bool:
    """Check whether |a| dropped below the freefall threshold before this sample."""
    current_ts = mag_samples[current_idx][0]
    window_start = current_ts - lookback_us
    # Walk backwards from current_idx
    i = current_idx - 1
    while i >= 0 and mag_samples[i][0] >= window_start:
        if mag_samples[i][4] < FREEFALL_THRESHOLD_G:
            return True
        i -= 1
    return False


def _check_pitch_before(
    pitch_sig: SignalData | None,
    event_ts: int,
    lookback_us: int,
) -> float | None:
    """Return the peak |pitch| in the window before the event, or None."""
    if pitch_sig is None or not pitch_sig.values:
        return None
    window_start = event_ts - lookback_us
    max_pitch = 0.0
    found = False
    for v in pitch_sig.values:
        if v.timestamp_us < window_start:
            continue
        if v.timestamp_us >= event_ts:
            break
        found = True
        max_pitch = max(max_pitch, abs(v.value))
    return max_pitch if found else None


# ---------------------------------------------------------------------------
# Unlicensed detection
# ---------------------------------------------------------------------------


def detect_unlicensed(
    angvel_z: SignalData,
    *,
    alpha_hard: float = DEFAULT_ANG_ACCEL_HARD,
    alpha_moderate: float = DEFAULT_ANG_ACCEL_MODERATE,
    alpha_light: float = DEFAULT_ANG_ACCEL_LIGHT,
    omega_hard: float = DEFAULT_OMEGA_STEP_HARD,
    omega_moderate: float = DEFAULT_OMEGA_STEP_MODERATE,
    omega_light: float = DEFAULT_OMEGA_STEP_LIGHT,
) -> list[HardHitEvent]:
    """Detect rotational impacts using AngularVelocityZWorld only.

    Uses two complementary methods:
    - Angular acceleration (dω/dt) over a 20ms window
    - Angular velocity step (|Δω|) over a 50ms window

    Returns a grouped list of ``HardHitEvent`` sorted by time.
    """
    vals = [(v.timestamp_us, v.value) for v in angvel_z.values]
    if len(vals) < 2:
        return []

    raw_events: list[HardHitEvent] = []

    for i in range(1, len(vals)):
        ts_i, omega_i = vals[i]

        # --- Method 1: angular acceleration over 20ms ---
        abs_alpha = 0.0
        j_accel = i - 1
        while j_accel > 0 and (ts_i - vals[j_accel][0]) < ANG_ACCEL_WINDOW_US:
            j_accel -= 1
        dt_accel = (ts_i - vals[j_accel][0]) / 1_000_000.0
        if dt_accel > 0.005:  # need at least ~5ms for meaningful derivative
            d_omega_accel = omega_i - vals[j_accel][1]
            abs_alpha = abs(d_omega_accel / dt_accel)

        # --- Method 2: omega step over 50ms ---
        abs_d_omega = 0.0
        j_step = i - 1
        while j_step > 0 and (ts_i - vals[j_step][0]) < OMEGA_STEP_WINDOW_US:
            j_step -= 1
        dt_step = (ts_i - vals[j_step][0]) / 1_000_000.0
        if dt_step > 0.02:  # need at least ~20ms
            abs_d_omega = abs(omega_i - vals[j_step][1])

        severity = _severity_from_angular(
            abs_alpha,
            abs_d_omega,
            alpha_hard,
            alpha_moderate,
            alpha_light,
            omega_hard,
            omega_moderate,
            omega_light,
        )
        if severity is None:
            continue

        score = max(abs_alpha, abs_d_omega)  # use the larger metric as score

        ev = HardHitEvent(
            timestamp_us=ts_i,
            severity=severity,
            score=score,
            mode=DetectionMode.UNLICENSED_ROTATIONAL,
            classification=Classification.ROTATIONAL_ONLY,
            omega_z_dps=omega_i,
            angular_accel_dps2=abs_alpha,
        )
        raw_events.append(ev)

    return _group_events(raw_events, EVENT_GROUP_WINDOW_US)


# ---------------------------------------------------------------------------
# Hoot file helper — targeted extraction for speed
# ---------------------------------------------------------------------------


def read_hoot_for_hard_hits(hoot_path: str | Path) -> LogData:
    """Convert a ``.hoot`` file extracting only the Pigeon 2 IMU signals.

    This is much faster than a full hoot conversion because only ~9 signals
    are extracted instead of hundreds.  Tries licensed signal IDs first;
    if no acceleration data comes through, falls back to the unlicensed
    subset (which omits Pro-only signals).

    Parameters:
        hoot_path: Path to the ``.hoot`` file.

    Returns:
        A ``LogData`` containing the Pigeon 2 signals needed for hard-hit
        analysis, plus ``RobotMode`` for match-phase annotation.
    """
    from logreader.hoot_reader import read_hoot

    # Try licensed extraction first (includes acceleration + robot state)
    all_licensed = _PIGEON2_LICENSED_SIGNAL_IDS + _ROBOT_STATE_SIGNAL_IDS
    log_data = read_hoot(hoot_path, signal_ids=all_licensed)

    # Check if acceleration signals came through
    accel_x = _find_pigeon_signal(log_data, _ACCEL_X_SUFFIX)
    if accel_x is not None and accel_x.values:
        return log_data

    # Fall back to unlicensed subset + robot state
    all_unlicensed = _PIGEON2_UNLICENSED_SIGNAL_IDS + _ROBOT_STATE_SIGNAL_IDS
    log_data = read_hoot(hoot_path, signal_ids=all_unlicensed)
    return log_data


# ---------------------------------------------------------------------------
# Analyzer
# ---------------------------------------------------------------------------


@register_analyzer
class HardHitAnalyzer(BaseAnalyzer):
    """Detect hard hits (collisions and landings) from Pigeon 2 IMU data.

    Requires a ``.hoot``-sourced wpilog with Pigeon 2 signals.

    In **licensed** mode (``AccelerationX/Y/Z`` present): detects impacts
    from acceleration magnitude, classifies as landing or collision, and
    reports severity.

    In **unlicensed** mode (only ``AngularVelocityZWorld``): detects
    rotational impacts from angular-acceleration spikes.
    """

    name = "hard-hits"
    description = "Detect collisions and landings from Pigeon 2 IMU data"

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--impact-threshold",
            type=float,
            default=DEFAULT_IMPACT_LIGHT_G,
            help=(
                f"Minimum impact magnitude (g) to report "
                f"(default: {DEFAULT_IMPACT_LIGHT_G})"
            ),
        )

    def run(self, log_data: LogData, **options: Any) -> AnalysisResult:
        min_threshold = options.get("impact_threshold", DEFAULT_IMPACT_LIGHT_G)

        # Try licensed mode first
        accel_x = _find_pigeon_signal(log_data, _ACCEL_X_SUFFIX)
        accel_y = _find_pigeon_signal(log_data, _ACCEL_Y_SUFFIX)
        accel_z = _find_pigeon_signal(log_data, _ACCEL_Z_SUFFIX)

        if (
            accel_x is not None
            and accel_y is not None
            and accel_z is not None
            and accel_x.values
            and accel_y.values
            and accel_z.values
        ):
            return self._run_licensed(
                log_data, accel_x, accel_y, accel_z, min_threshold
            )

        # Fall back to unlicensed mode
        angvel_z = _find_pigeon_signal(log_data, _ANGVEL_Z_SUFFIX)
        if angvel_z is not None and angvel_z.values:
            return self._run_unlicensed(angvel_z, log_data)

        return AnalysisResult(
            analyzer_name=self.name,
            title="Hard Hit Detection",
            summary=(
                "No Pigeon 2 IMU signals found. "
                "This analyzer requires .hoot data converted via owlet."
            ),
        )

    def _run_licensed(
        self,
        log_data: LogData,
        accel_x: SignalData,
        accel_y: SignalData,
        accel_z: SignalData,
        min_threshold: float,
    ) -> AnalysisResult:
        angvel_x = _find_pigeon_signal(log_data, _ANGVEL_X_SUFFIX)
        angvel_y = _find_pigeon_signal(log_data, _ANGVEL_Y_SUFFIX)
        angvel_z = _find_pigeon_signal(log_data, _ANGVEL_Z_SUFFIX)
        pitch_sig = _find_pigeon_signal(log_data, _PITCH_SUFFIX)

        events = detect_licensed(
            accel_x,
            accel_y,
            accel_z,
            angvel_x,
            angvel_y,
            angvel_z,
            pitch_sig,
            light_g=min_threshold,
        )

        # Annotate events with match phase (auto/teleop/disabled)
        _annotate_phases(events, log_data)

        if not events:
            return AnalysisResult(
                analyzer_name=self.name,
                title="Hard Hit Detection (licensed)",
                summary="No impact events detected above threshold.",
            )

        landings = [e for e in events if e.classification == Classification.LANDING]
        collisions = [e for e in events if e.classification != Classification.LANDING]
        hard_count = sum(1 for e in events if e.severity == Severity.HARD)

        summary_parts = [
            f"Detected {len(events)} impact events "
            f"({hard_count} HARD, "
            f"{sum(1 for e in events if e.severity == Severity.MODERATE)} MODERATE, "
            f"{sum(1 for e in events if e.severity == Severity.LIGHT)} LIGHT).",
            f"  Landings: {len(landings)}, Collisions: {len(collisions)}.",
            f"  Mode: licensed (full acceleration data).",
        ]

        columns = [
            "Match t",
            "Phase",
            "Phase t",
            "Severity",
            "Impact (g)",
            "Type",
            "ax",
            "ay",
            "az",
            "wz (d/s)",
            "Lat%",
            "Freefall",
        ]

        rows = []
        for ev in events:
            rows.append(
                {
                    "Match t": (
                        f"{ev.match_time_s:.1f}"
                        if ev.match_time_s is not None
                        else f"{ev.timestamp_s:.1f}"
                    ),
                    "Phase": ev.phase or "",
                    "Phase t": (
                        f"{ev.phase_time_s:.1f}" if ev.phase_time_s is not None else ""
                    ),
                    "Severity": ev.severity.value,
                    "Impact (g)": f"{ev.impact_g:.2f}" if ev.impact_g else "",
                    "Type": ev.classification.value,
                    "ax": f"{ev.ax_g:+.2f}" if ev.ax_g is not None else "",
                    "ay": f"{ev.ay_g:+.2f}" if ev.ay_g is not None else "",
                    "az": f"{ev.az_g:+.2f}" if ev.az_g is not None else "",
                    "wz (d/s)": (
                        f"{ev.omega_z_dps:+.1f}" if ev.omega_z_dps is not None else ""
                    ),
                    "Lat%": (
                        f"{ev.lateral_fraction:.0%}"
                        if ev.lateral_fraction is not None
                        else ""
                    ),
                    "Freefall": (
                        "yes"
                        if ev.freefall_detected
                        else "no" if ev.freefall_detected is not None else ""
                    ),
                }
            )

        return AnalysisResult(
            analyzer_name=self.name,
            title="Hard Hit Detection (licensed)",
            summary="\n".join(summary_parts),
            columns=columns,
            rows=rows,
            extra={"events": events},
        )

    def _run_unlicensed(
        self, angvel_z: SignalData, log_data: LogData
    ) -> AnalysisResult:
        events = detect_unlicensed(angvel_z)

        # Annotate events with match phase (auto/teleop/disabled)
        _annotate_phases(events, log_data)

        if not events:
            return AnalysisResult(
                analyzer_name=self.name,
                title="Hard Hit Detection (unlicensed — rotational only)",
                summary="No rotational impact events detected.",
            )

        hard_count = sum(1 for e in events if e.severity == Severity.HARD)
        summary_parts = [
            f"Detected {len(events)} rotational impact events "
            f"({hard_count} HARD, "
            f"{sum(1 for e in events if e.severity == Severity.MODERATE)} MODERATE, "
            f"{sum(1 for e in events if e.severity == Severity.LIGHT)} LIGHT).",
            "  Mode: unlicensed (angular velocity Z only).",
            "  Note: cannot distinguish landings from collisions.",
        ]

        columns = [
            "Match t",
            "Phase",
            "Phase t",
            "Severity",
            "wz (d/s)",
            "|a| (d/s2)",
            "Score",
        ]

        rows = []
        for ev in events:
            rows.append(
                {
                    "Match t": (
                        f"{ev.match_time_s:.1f}"
                        if ev.match_time_s is not None
                        else f"{ev.timestamp_s:.1f}"
                    ),
                    "Phase": ev.phase or "",
                    "Phase t": (
                        f"{ev.phase_time_s:.1f}" if ev.phase_time_s is not None else ""
                    ),
                    "Severity": ev.severity.value,
                    "wz (d/s)": (
                        f"{ev.omega_z_dps:+.1f}" if ev.omega_z_dps is not None else ""
                    ),
                    "|a| (d/s2)": (
                        f"{ev.angular_accel_dps2:.0f}"
                        if ev.angular_accel_dps2 is not None
                        else ""
                    ),
                    "Score": f"{ev.score:.0f}",
                }
            )

        return AnalysisResult(
            analyzer_name=self.name,
            title="Hard Hit Detection (unlicensed — rotational only)",
            summary="\n".join(summary_parts),
            columns=columns,
            rows=rows,
            extra={"events": events},
        )
