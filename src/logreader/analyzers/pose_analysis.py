"""Pose analysis — discover, fuse, and compare localization sources.

This module is both a **standalone analyzer** (``pose-analysis``) and a
**shared service** that other analyzers import to obtain a best-estimate
reference path for the robot.

Public API for other analyzers::

    from logreader.analyzers.pose_analysis import (
        PoseSample,
        PoseSource,
        SourceMetrics,
        DivergenceEvent,
        discover_pose_sources,
        build_reference_path,
        interpolate_pose_at,
        compute_velocity_at,
        compute_divergence_metrics,
    )
"""

from __future__ import annotations

import argparse
import math
import re
import statistics
from dataclasses import dataclass, field
from typing import Any, Sequence

from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer
from logreader.models import LogData, SignalData, SignalType

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_DEG_TO_RAD = math.pi / 180.0
_RAD_TO_DEG = 180.0 / math.pi

# Physical plausibility — max robot speed ~5 m/s, reject vision jumps beyond this
_MAX_PLAUSIBLE_SPEED_MPS = 6.0

# Vision quality gating defaults
_MIN_TAG_AREA = 0.001  # ta < this → down-weight heavily
_MAX_AMBIGUITY = 0.2

# Gravity constant
_GRAVITY_MPS2 = 9.81

# Accel consistency thresholds
_ACCEL_IMPACT_THRESHOLD_MPS2 = 15.0  # residual accel spike → hard impact
_ACCEL_AIRBORNE_THRESHOLD_G = 0.3  # gravity magnitude < this → airborne
_ACCEL_SKID_THRESHOLD_MPS2 = 4.0  # sustained residual → traction loss
_ACCEL_SKID_MIN_DURATION_US = 100_000  # 100 ms minimum for skid event

# ---------------------------------------------------------------------------
# Data models
# ---------------------------------------------------------------------------


@dataclass
class PoseSample:
    """A single pose observation from any source."""

    timestamp_us: int
    x: float  # meters, field-relative
    y: float  # meters, field-relative
    theta: float  # radians, (-π, π]
    source_name: str  # signal name or camera name
    source_class: str  # "odometry" | "vision" | "imu"
    confidence: float = 1.0  # 0.0–1.0
    latency_us: int = 0  # estimated latency (0 if unknown)


@dataclass
class PoseSource:
    """A discovered and parsed source of pose information."""

    name: str
    source_class: str  # "odometry" | "vision" | "gyro" | "imu"
    signal_names: list[str] = field(default_factory=list)
    sample_count: int = 0
    start_us: int = 0
    end_us: int = 0
    median_rate_hz: float | None = None
    quality_signals: list[str] = field(default_factory=list)
    samples: list[PoseSample] = field(default_factory=list, repr=False)
    notes: list[str] = field(default_factory=list)


@dataclass
class DivergenceEvent:
    """A time window where one source diverged from the reference."""

    source_name: str
    start_us: int
    end_us: int
    peak_translation_error_m: float
    peak_heading_error_rad: float
    phase: str | None = None
    likely_cause: str = "unknown"


@dataclass
class SourceMetrics:
    """Per-source divergence statistics against the reference path."""

    source_name: str
    source_class: str
    translation_rms_m: float
    translation_max_m: float
    heading_rms_rad: float
    heading_max_rad: float
    sample_count: int
    coverage_fraction: float
    confidence: str  # "high" | "medium" | "low"


@dataclass
class AccelSample:
    """A single accelerometer reading (robot-relative, NOT gravity-compensated)."""

    timestamp_us: int
    ax: float  # m/s², robot-relative X (forward)
    ay: float  # m/s², robot-relative Y (left)
    az: float  # m/s², robot-relative Z (up, includes gravity ~-9.8)
    source_name: str
    gravity_x: float = 0.0  # gravity vector component, if available
    gravity_y: float = 0.0
    gravity_z: float = -_GRAVITY_MPS2  # default: gravity is straight down


@dataclass
class AccelSource:
    """A discovered accelerometer/IMU source."""

    name: str
    signal_names: list[str] = field(default_factory=list)
    sample_count: int = 0
    start_us: int = 0
    end_us: int = 0
    median_rate_hz: float | None = None
    has_gravity_vector: bool = False
    samples: list[AccelSample] = field(default_factory=list, repr=False)
    notes: list[str] = field(default_factory=list)


@dataclass
class AccelConsistencyEvent:
    """A detected acceleration anomaly (impact, airborne, skid/traction loss)."""

    start_us: int
    end_us: int
    event_type: str  # "impact" | "airborne" | "skid" | "unknown"
    peak_residual_mps2: float  # max accel discrepancy
    peak_gravity_fraction: float  # min gravity magnitude / 9.81 (< 1 = airborne)
    description: str


# ---------------------------------------------------------------------------
# Heading utilities
# ---------------------------------------------------------------------------


def _normalize_angle(rad: float) -> float:
    """Normalize angle to (-π, π]."""
    rad = rad % (2 * math.pi)
    if rad > math.pi:
        rad -= 2 * math.pi
    return rad


def _angle_diff(a: float, b: float) -> float:
    """Signed shortest angular difference a - b, result in (-π, π]."""
    d = a - b
    d = (d + math.pi) % (2 * math.pi) - math.pi
    return d


# ---------------------------------------------------------------------------
# Signal discovery patterns
# ---------------------------------------------------------------------------

# Odometry / fused pose — double[] with [x, y, theta_degrees]
_ODOM_POSE_PATTERNS: list[re.Pattern[str]] = [
    re.compile(r"NT:/Pose/robotPose$", re.I),
    re.compile(r"NT:/SmartDashboard/Field/Robot$", re.I),
    re.compile(r".*/EstimatedPose$", re.I),
    re.compile(r".*/OdometryPose$", re.I),
    re.compile(r".*/Pose/.*Pose$", re.I),
]

# Vision — Limelight botpose arrays
_VISION_BOTPOSE_PATTERN = re.compile(
    r"NT:/limelight-([a-z0-9_-]+)/botpose_wpiblue$", re.I
)
_VISION_BOTPOSE_FALLBACK = re.compile(r"NT:/limelight-([a-z0-9_-]+)/botpose$", re.I)

# Vision — team-published raw vision pose
_VISION_RAW_PATTERNS: list[re.Pattern[str]] = [
    re.compile(r"NT:/SmartDashboard/Field/RawVision$", re.I),
    re.compile(r".*/VisionPose$", re.I),
]

# Limelight quality / latency signals
_LL_QUALITY_PATTERNS = {
    "tv": re.compile(r"NT:/limelight-([a-z0-9_-]+)/tv$", re.I),
    "tl": re.compile(r"NT:/limelight-([a-z0-9_-]+)/tl$", re.I),
    "cl": re.compile(r"NT:/limelight-([a-z0-9_-]+)/cl$", re.I),
    "ta": re.compile(r"NT:/limelight-([a-z0-9_-]+)/ta$", re.I),
    "tid": re.compile(r"NT:/limelight-([a-z0-9_-]+)/tid$", re.I),
    "stddevs": re.compile(r"NT:/limelight-([a-z0-9_-]+)/stddevs$", re.I),
}

# Limelight IMU
_LL_IMU_PATTERN = re.compile(r"NT:/limelight-([a-z0-9_-]+)/imu$", re.I)

# Struct poses (detected but not parsed in v1)
_STRUCT_POSE_PATTERNS: list[re.Pattern[str]] = [
    re.compile(r"NT:/DriveState/Pose$", re.I),
    re.compile(r"NT:vision/fieldPose3d$", re.I),
    re.compile(r"NT:vision/rawFieldPose3d[A-Z]$", re.I),
    re.compile(r"NT:/PathPlanner/currentPose$", re.I),
]

# Pigeon2 accelerometer (from hoot-converted wpilog)
_PIGEON2_ACCEL_PATTERN = re.compile(r"Phoenix6/Pigeon2-(\d+)/Acceleration([XYZ])$")
_PIGEON2_GRAVITY_PATTERN = re.compile(r"Phoenix6/Pigeon2-(\d+)/GravityVector([XYZ])$")


# ---------------------------------------------------------------------------
# Signal discovery
# ---------------------------------------------------------------------------


def _median_gap_hz(samples: Sequence[PoseSample]) -> float | None:
    """Compute median sample rate from a sorted sample list."""
    if len(samples) < 2:
        return None
    gaps = [
        samples[i + 1].timestamp_us - samples[i].timestamp_us
        for i in range(len(samples) - 1)
    ]
    med = statistics.median(gaps)
    if med <= 0:
        return None
    return 1_000_000.0 / med


def _parse_double_array_pose(
    sig: SignalData,
    source_name: str,
    source_class: str,
    *,
    theta_is_degrees: bool = True,
    min_array_len: int = 3,
) -> list[PoseSample]:
    """Parse a double[] signal containing [x, y, theta, ...] into PoseSamples."""
    samples: list[PoseSample] = []
    for tv in sig.values:
        arr = tv.value
        if not isinstance(arr, (list, tuple)) or len(arr) < min_array_len:
            continue
        x, y = float(arr[0]), float(arr[1])
        theta_raw = float(arr[2])
        theta = _normalize_angle(
            theta_raw * _DEG_TO_RAD if theta_is_degrees else theta_raw
        )
        samples.append(
            PoseSample(
                timestamp_us=tv.timestamp_us,
                x=x,
                y=y,
                theta=theta,
                source_name=source_name,
                source_class=source_class,
            )
        )
    return samples


def _parse_limelight_botpose(
    sig: SignalData,
    camera_name: str,
    quality_sigs: dict[str, SignalData],
) -> list[PoseSample]:
    """Parse a Limelight botpose_wpiblue signal into PoseSamples.

    The Limelight botpose_wpiblue array layout (when targets are visible):
        [0] x  (meters, field-relative)
        [1] y  (meters, field-relative)
        [2] z
        [3] roll
        [4] pitch
        [5] yaw (degrees)
        [6] total_latency_ms (pipeline + capture)
        [7] tag_count
        [8] tag_span
        [9] avg_tag_dist
        [10] avg_tag_area
        ...per-tag data follows in groups of 7 when len > 11

    When no target: array may be all zeros (len 11) or very small.
    """
    # Build a fast lookup for tv (target valid) transitions
    tv_sig = quality_sigs.get("tv")
    tv_state: dict[int, float] = {}
    if tv_sig:
        for v in tv_sig.values:
            tv_state[v.timestamp_us] = float(v.value)

    # Build ta lookup for area-based quality
    ta_sig = quality_sigs.get("ta")
    ta_lookup: dict[int, float] = {}
    if ta_sig:
        for v in ta_sig.values:
            ta_lookup[v.timestamp_us] = float(v.value)

    samples: list[PoseSample] = []
    for tv in sig.values:
        arr = tv.value
        if not isinstance(arr, (list, tuple)) or len(arr) < 7:
            continue

        x, y = float(arr[0]), float(arr[1])
        yaw_deg = float(arr[5])
        total_latency_ms = float(arr[6])
        tag_count = float(arr[7]) if len(arr) > 7 else 0.0

        # Skip zero-pose samples (no target)
        if abs(x) < 0.01 and abs(y) < 0.01 and abs(yaw_deg) < 0.01:
            continue

        # Skip when tag_count is 0
        if tag_count < 0.5:
            continue

        theta = _normalize_angle(yaw_deg * _DEG_TO_RAD)
        latency_us = int(total_latency_ms * 1000)

        # Confidence based on tag count and area
        confidence = 1.0
        if tag_count < 1.5:  # single tag
            confidence = 0.6
        if tag_count >= 2.5:
            confidence = 1.0

        # Area-based down-weighting
        area = ta_lookup.get(tv.timestamp_us)
        if area is not None and area < _MIN_TAG_AREA:
            confidence *= 0.3

        # Avg tag distance (index 9 if available)
        if len(arr) > 9:
            avg_dist = float(arr[9])
            if avg_dist > 5.0:
                confidence *= 0.5
            elif avg_dist > 3.0:
                confidence *= 0.8

        samples.append(
            PoseSample(
                timestamp_us=tv.timestamp_us,
                x=x,
                y=y,
                theta=theta,
                source_name=camera_name,
                source_class="vision",
                confidence=confidence,
                latency_us=latency_us,
            )
        )

    return samples


def _median_gap_hz_accel(samples: Sequence[AccelSample]) -> float | None:
    """Compute median sample rate from a sorted AccelSample list."""
    if len(samples) < 2:
        return None
    gaps = [
        samples[i + 1].timestamp_us - samples[i].timestamp_us
        for i in range(len(samples) - 1)
    ]
    med = statistics.median(gaps)
    if med <= 0:
        return None
    return 1_000_000.0 / med


def _parse_limelight_imu(sig: SignalData, camera_name: str) -> list[AccelSample]:
    """Parse a Limelight imu signal (double[10]) into AccelSamples.

    Limelight IMU array layout::
        [0] accel_x  (m/s², robot-relative, includes gravity)
        [1] accel_y
        [2] accel_z
        [3] accel_x  (duplicate)
        [4] gyro_x   (deg/s)
        [5] gyro_y   (deg/s)
        [6] gyro_z   (deg/s)
        [7] gravity_x (approximate gravity vector component)
        [8] gravity_y
        [9] gravity_z
    """
    samples: list[AccelSample] = []
    for tv in sig.values:
        arr = tv.value
        if not isinstance(arr, (list, tuple)) or len(arr) < 10:
            continue
        samples.append(
            AccelSample(
                timestamp_us=tv.timestamp_us,
                ax=float(arr[0]),
                ay=float(arr[1]),
                az=float(arr[2]),
                source_name=camera_name,
                gravity_x=float(arr[7]),
                gravity_y=float(arr[8]),
                gravity_z=float(arr[9]),
            )
        )
    return samples


def _parse_pigeon2_accel(log_data: LogData, device_id: str) -> list[AccelSample]:
    """Parse Pigeon2 AccelerationX/Y/Z signals into AccelSamples.

    Pigeon2 accel values are in G (not m/s²).  We convert to m/s².
    GravityVectorX/Y/Z are also in G if available.
    """
    prefix = f"Phoenix6/Pigeon2-{device_id}"
    ax_sig = log_data.get_signal(f"{prefix}/AccelerationX")
    ay_sig = log_data.get_signal(f"{prefix}/AccelerationY")
    az_sig = log_data.get_signal(f"{prefix}/AccelerationZ")

    if not ax_sig or not ay_sig or not az_sig:
        return []
    if len(ax_sig.values) < 10:
        return []

    gx_sig = log_data.get_signal(f"{prefix}/GravityVectorX")
    gy_sig = log_data.get_signal(f"{prefix}/GravityVectorY")
    gz_sig = log_data.get_signal(f"{prefix}/GravityVectorZ")
    has_gravity = bool(gx_sig and gy_sig and gz_sig and len(gx_sig.values) >= 10)

    # Build timestamp-indexed lookups for Y/Z (X is the backbone)
    ay_map: dict[int, float] = {v.timestamp_us: float(v.value) for v in ay_sig.values}
    az_map: dict[int, float] = {v.timestamp_us: float(v.value) for v in az_sig.values}

    gx_map: dict[int, float] = {}
    gy_map: dict[int, float] = {}
    gz_map: dict[int, float] = {}
    if has_gravity:
        gx_map = {v.timestamp_us: float(v.value) for v in gx_sig.values}
        gy_map = {v.timestamp_us: float(v.value) for v in gy_sig.values}
        gz_map = {v.timestamp_us: float(v.value) for v in gz_sig.values}

    samples: list[AccelSample] = []
    name = f"Pigeon2-{device_id}"
    for v in ax_sig.values:
        ts = v.timestamp_us
        ay_val = ay_map.get(ts)
        az_val = az_map.get(ts)
        if ay_val is None or az_val is None:
            continue

        # Pigeon2 acceleration values are in G — convert to m/s²
        ax_mps2 = float(v.value) * _GRAVITY_MPS2
        ay_mps2 = ay_val * _GRAVITY_MPS2
        az_mps2 = az_val * _GRAVITY_MPS2

        gx = gx_map.get(ts, 0.0) * _GRAVITY_MPS2 if has_gravity else 0.0
        gy = gy_map.get(ts, 0.0) * _GRAVITY_MPS2 if has_gravity else 0.0
        gz = gz_map.get(ts, 0.0) * _GRAVITY_MPS2 if has_gravity else -_GRAVITY_MPS2

        samples.append(
            AccelSample(
                timestamp_us=ts,
                ax=ax_mps2,
                ay=ay_mps2,
                az=az_mps2,
                source_name=name,
                gravity_x=gx,
                gravity_y=gy,
                gravity_z=gz,
            )
        )
    return samples


def discover_accel_sources(log_data: LogData) -> list[AccelSource]:
    """Auto-detect accelerometer/IMU sources in the log.

    Discovers:
    - Limelight IMU (``limelight-*/imu`` double[10] signals)
    - Pigeon2 (``Phoenix6/Pigeon2-*/AccelerationX/Y/Z``)
    """
    sources: list[AccelSource] = []
    all_names = list(log_data.signals.keys())

    # --- Limelight IMU ---
    for name in all_names:
        m = _LL_IMU_PATTERN.match(name)
        if not m:
            continue
        cam = m.group(1)
        sig = log_data.signals[name]
        if sig.info.type != SignalType.DOUBLE_ARRAY:
            continue
        if len(sig.values) < 10:
            continue

        samples = _parse_limelight_imu(sig, f"limelight-{cam}")
        if not samples:
            continue

        rate = _median_gap_hz_accel(samples)
        sources.append(
            AccelSource(
                name=f"limelight-{cam}-imu",
                signal_names=[name],
                sample_count=len(samples),
                start_us=samples[0].timestamp_us,
                end_us=samples[-1].timestamp_us,
                median_rate_hz=rate,
                has_gravity_vector=True,
                samples=samples,
                notes=["Limelight built-in IMU; accel includes gravity"],
            )
        )

    # --- Pigeon2 from hoot-converted data ---
    pigeon_ids: set[str] = set()
    for name in all_names:
        m = _PIGEON2_ACCEL_PATTERN.match(name)
        if m:
            pigeon_ids.add(m.group(1))

    for pid in sorted(pigeon_ids):
        samples = _parse_pigeon2_accel(log_data, pid)
        if not samples:
            continue

        prefix = f"Phoenix6/Pigeon2-{pid}"
        has_gravity = bool(log_data.get_signal(f"{prefix}/GravityVectorX"))
        rate = _median_gap_hz_accel(samples)

        sig_names = [
            f"{prefix}/AccelerationX",
            f"{prefix}/AccelerationY",
            f"{prefix}/AccelerationZ",
        ]
        if has_gravity:
            sig_names += [
                f"{prefix}/GravityVectorX",
                f"{prefix}/GravityVectorY",
                f"{prefix}/GravityVectorZ",
            ]

        sources.append(
            AccelSource(
                name=f"Pigeon2-{pid}",
                signal_names=sig_names,
                sample_count=len(samples),
                start_us=samples[0].timestamp_us,
                end_us=samples[-1].timestamp_us,
                median_rate_hz=rate,
                has_gravity_vector=has_gravity,
                samples=samples,
                notes=[
                    f"CTRE Pigeon2 (CAN ID {pid}); "
                    f"{'has gravity vector' if has_gravity else 'no gravity vector'}"
                ],
            )
        )

    return sources


# ---------------------------------------------------------------------------
# Accelerometer consistency analysis
# ---------------------------------------------------------------------------


def analyze_accel_consistency(
    accel_source: AccelSource,
    reference_path: list[PoseSample],
    *,
    impact_threshold_mps2: float = _ACCEL_IMPACT_THRESHOLD_MPS2,
    airborne_threshold_g: float = _ACCEL_AIRBORNE_THRESHOLD_G,
    skid_threshold_mps2: float = _ACCEL_SKID_THRESHOLD_MPS2,
    skid_min_duration_us: int = _ACCEL_SKID_MIN_DURATION_US,
) -> list[AccelConsistencyEvent]:
    """Compare IMU acceleration against path-derived acceleration.

    Detects three categories of events:

    - **impact**: sudden, brief acceleration spike (collision, bump)
    - **airborne**: gravity magnitude drops well below 1g (robot in the air)
    - **skid / traction loss**: sustained disagreement between IMU-measured
      planar accel and path-derived accel (wheels spinning but robot not
      accelerating as expected, or robot pushed sideways)

    The comparison works in the robot-relative horizontal plane:
      1. Compute path-derived accel from reference path finite differences.
      2. Remove gravity from the IMU reading using the gravity vector.
      3. Compute the residual = IMU_planar - path_planar.
      4. Large residuals indicate the wheels and chassis disagree.
    """
    events: list[AccelConsistencyEvent] = []
    if not accel_source.samples or len(reference_path) < 3:
        return events

    # Determine the valid time range (only analyze where the reference path
    # has data and the robot is actually doing something).
    ref_start = reference_path[0].timestamp_us
    ref_end = reference_path[-1].timestamp_us

    # --- Filter out uninitialized IMU samples ---
    # Limelight IMU reports all-zero gravity vector before it connects.
    # Skip any sample where:
    #  - it's outside the reference path time range, or
    #  - raw accel magnitude is near zero (IMU not reporting), or
    #  - gravity vector magnitude is far from 9.81 m/s² (not yet calibrated
    #    or sensor not providing real data).  Even a severely tilted robot
    #    still sees full gravity; only free-fall should drop below ~5 m/s².
    _MIN_GRAVITY_INIT = 5.0  # m/s²; below this, the IMU is probably not ready
    _MAX_PLAUSIBLE_ACCEL = 80.0  # m/s²; ~8g, beyond this is sensor saturation
    valid_samples = [
        s
        for s in accel_source.samples
        if (
            ref_start <= s.timestamp_us <= ref_end
            and math.sqrt(s.ax**2 + s.ay**2 + s.az**2) > 1.0
            and math.sqrt(s.gravity_x**2 + s.gravity_y**2 + s.gravity_z**2)
            > _MIN_GRAVITY_INIT
            and math.sqrt(s.ax**2 + s.ay**2 + s.az**2) < _MAX_PLAUSIBLE_ACCEL
        )
    ]
    if not valid_samples:
        return events

    # --- Detect airborne events from gravity magnitude ---
    in_airborne = False
    ab_start = 0
    ab_peak = 1.0

    for s in valid_samples:
        grav_mag = math.sqrt(s.gravity_x**2 + s.gravity_y**2 + s.gravity_z**2)
        grav_fraction = grav_mag / _GRAVITY_MPS2

        if grav_fraction < airborne_threshold_g:
            if not in_airborne:
                in_airborne = True
                ab_start = s.timestamp_us
                ab_peak = grav_fraction
            else:
                ab_peak = min(ab_peak, grav_fraction)
        else:
            if in_airborne:
                duration = s.timestamp_us - ab_start
                if duration >= 20_000:  # at least 20 ms
                    events.append(
                        AccelConsistencyEvent(
                            start_us=ab_start,
                            end_us=s.timestamp_us,
                            event_type="airborne",
                            peak_residual_mps2=0.0,
                            peak_gravity_fraction=ab_peak,
                            description=(
                                f"Robot appears airborne: gravity magnitude "
                                f"dropped to {ab_peak:.2f}g for "
                                f"{(s.timestamp_us - ab_start) / 1000:.0f} ms"
                            ),
                        )
                    )
                in_airborne = False

    # --- Compute path-derived acceleration for residual analysis ---
    # Use a wider finite-difference window to avoid amplifying small
    # step-wise pose quantization.  We compute velocity from samples
    # ~20 ms apart, then acceleration from velocity pairs ~20 ms apart.
    ACCEL_DIFF_STEP = max(1, 5)  # use samples 5 apart (~20 ms at 250 Hz)
    path_accel: list[tuple[int, float, float]] = []  # (ts, ax, ay) field-relative
    step = ACCEL_DIFF_STEP
    for i in range(step, len(reference_path) - step):
        p0 = reference_path[i - step]
        p1 = reference_path[i]
        p2 = reference_path[i + step]

        dt01 = (p1.timestamp_us - p0.timestamp_us) / 1_000_000.0
        dt12 = (p2.timestamp_us - p1.timestamp_us) / 1_000_000.0
        if dt01 <= 0.001 or dt12 <= 0.001:
            continue

        vx1 = (p1.x - p0.x) / dt01
        vy1 = (p1.y - p0.y) / dt01
        vx2 = (p2.x - p1.x) / dt12
        vy2 = (p2.y - p1.y) / dt12

        dt_mid = (dt01 + dt12) / 2
        ax = (vx2 - vx1) / dt_mid
        ay = (vy2 - vy1) / dt_mid

        # Clamp to physically plausible range — an FRC robot maxes out
        # around 10-15 m/s².  Values beyond 30 m/s² are pose-jump artifacts.
        MAX_PATH_ACCEL = 30.0
        ax = max(-MAX_PATH_ACCEL, min(MAX_PATH_ACCEL, ax))
        ay = max(-MAX_PATH_ACCEL, min(MAX_PATH_ACCEL, ay))

        path_accel.append((p1.timestamp_us, ax, ay))

    if not path_accel:
        return events

    # --- Compare IMU planar accel vs path accel ---
    # Remove gravity from IMU to get motion-only acceleration
    pa_idx = 0
    in_skid = False
    skid_start = 0
    skid_peak = 0.0
    residuals: list[tuple[int, float]] = []

    for s in valid_samples:
        # Advance path accel index
        while (
            pa_idx < len(path_accel) - 1 and path_accel[pa_idx + 1][0] <= s.timestamp_us
        ):
            pa_idx += 1

        if pa_idx >= len(path_accel):
            break

        pa_ts, pa_ax, pa_ay = path_accel[pa_idx]

        # Skip if timestamps are too far apart
        if abs(s.timestamp_us - pa_ts) > 50_000:  # 50 ms tolerance
            continue

        # Remove gravity from IMU reading to get motion-only acceleration
        imu_motion_ax = s.ax - s.gravity_x
        imu_motion_ay = s.ay - s.gravity_y

        # Both are now in their respective frames.  For a first approximation,
        # compare magnitudes (avoids needing exact frame rotation).
        imu_planar_mag = math.sqrt(imu_motion_ax**2 + imu_motion_ay**2)
        path_planar_mag = math.sqrt(pa_ax**2 + pa_ay**2)

        residual = abs(imu_planar_mag - path_planar_mag)
        residuals.append((s.timestamp_us, residual))

        # Detect impact: sudden spike
        if residual > impact_threshold_mps2:
            # Check it's brief (not sustained)
            events.append(
                AccelConsistencyEvent(
                    start_us=s.timestamp_us,
                    end_us=s.timestamp_us,
                    event_type="impact",
                    peak_residual_mps2=residual,
                    peak_gravity_fraction=1.0,
                    description=(
                        f"Acceleration spike: {residual:.1f} m/s² residual "
                        f"(IMU={imu_planar_mag:.1f}, path={path_planar_mag:.1f})"
                    ),
                )
            )

        # Detect skid: sustained moderate residual
        if residual > skid_threshold_mps2:
            if not in_skid:
                in_skid = True
                skid_start = s.timestamp_us
                skid_peak = residual
            else:
                skid_peak = max(skid_peak, residual)
        else:
            if in_skid:
                duration = s.timestamp_us - skid_start
                if duration >= skid_min_duration_us:
                    events.append(
                        AccelConsistencyEvent(
                            start_us=skid_start,
                            end_us=s.timestamp_us,
                            event_type="skid",
                            peak_residual_mps2=skid_peak,
                            peak_gravity_fraction=1.0,
                            description=(
                                f"Traction loss / skid: {skid_peak:.1f} m/s² "
                                f"peak residual for "
                                f"{duration / 1000:.0f} ms"
                            ),
                        )
                    )
                in_skid = False

    # Close any open skid event
    if in_skid and valid_samples:
        duration = valid_samples[-1].timestamp_us - skid_start
        if duration >= skid_min_duration_us:
            events.append(
                AccelConsistencyEvent(
                    start_us=skid_start,
                    end_us=valid_samples[-1].timestamp_us,
                    event_type="skid",
                    peak_residual_mps2=skid_peak,
                    peak_gravity_fraction=1.0,
                    description=(
                        f"Traction loss / skid: {skid_peak:.1f} m/s² "
                        f"peak residual for {duration / 1000:.0f} ms"
                    ),
                )
            )

    # Sort by time, deduplicate impacts that overlap with skids
    events.sort(key=lambda e: e.start_us)
    return events


def discover_pose_sources(log_data: LogData) -> list[PoseSource]:
    """Auto-detect and parse all pose-related signals in the log.

    Returns a list of ``PoseSource`` objects, each containing parsed
    ``PoseSample`` lists ready for fusion or comparison.
    """
    sources: list[PoseSource] = []
    all_names = list(log_data.signals.keys())

    # --- 1. Odometry / fused pose (double[] signals) ---
    seen_odom_names: set[str] = set()
    for pattern in _ODOM_POSE_PATTERNS:
        for name in all_names:
            if name in seen_odom_names:
                continue
            if pattern.match(name):
                sig = log_data.signals[name]
                if sig.info.type != SignalType.DOUBLE_ARRAY:
                    continue
                if len(sig.values) < 10:
                    continue
                samples = _parse_double_array_pose(
                    sig, name, "odometry", theta_is_degrees=True
                )
                if not samples:
                    continue
                seen_odom_names.add(name)
                rate = _median_gap_hz(samples)
                sources.append(
                    PoseSource(
                        name=name,
                        source_class="odometry",
                        signal_names=[name],
                        sample_count=len(samples),
                        start_us=samples[0].timestamp_us,
                        end_us=samples[-1].timestamp_us,
                        median_rate_hz=rate,
                        samples=samples,
                    )
                )

    # --- 2. Limelight vision cameras ---
    # Find all limelight camera names
    ll_cameras: dict[str, dict[str, SignalData]] = {}
    for name in all_names:
        for key, pat in _LL_QUALITY_PATTERNS.items():
            m = pat.match(name)
            if m:
                cam = m.group(1)
                ll_cameras.setdefault(cam, {})[key] = log_data.signals[name]

    # Parse botpose_wpiblue for each camera, falling back to botpose
    seen_ll_cameras: set[str] = set()  # track which cameras we've already parsed
    # First pass: prefer botpose_wpiblue
    for name in all_names:
        m = _VISION_BOTPOSE_PATTERN.match(name)
        if not m:
            continue

        cam = m.group(1)
        if cam in seen_ll_cameras:
            continue

        sig = log_data.signals[name]
        if sig.info.type != SignalType.DOUBLE_ARRAY:
            continue
        if len(sig.values) < 5:
            continue

        quality_sigs = ll_cameras.get(cam, {})
        samples = _parse_limelight_botpose(sig, f"limelight-{cam}", quality_sigs)
        if not samples:
            continue

        seen_ll_cameras.add(cam)
        rate = _median_gap_hz(samples)
        quality_names = [f"NT:/limelight-{cam}/{k}" for k in quality_sigs.keys()]

        sources.append(
            PoseSource(
                name=f"limelight-{cam}",
                source_class="vision",
                signal_names=[name],
                sample_count=len(samples),
                start_us=samples[0].timestamp_us,
                end_us=samples[-1].timestamp_us,
                median_rate_hz=rate,
                quality_signals=quality_names,
                samples=samples,
                notes=["Using WPILib blue-alliance origin"],
            )
        )

    # Second pass: fallback to botpose for cameras not already discovered
    for name in all_names:
        m = _VISION_BOTPOSE_FALLBACK.match(name)
        if not m:
            continue
        # Skip botpose_wpiblue/wpired variants (already handled above)
        if "wpiblue" in name.lower() or "wpired" in name.lower():
            continue

        cam = m.group(1)
        if cam in seen_ll_cameras:
            continue

        sig = log_data.signals[name]
        if sig.info.type != SignalType.DOUBLE_ARRAY:
            continue
        if len(sig.values) < 5:
            continue

        quality_sigs = ll_cameras.get(cam, {})
        samples = _parse_limelight_botpose(sig, f"limelight-{cam}", quality_sigs)
        if not samples:
            continue

        seen_ll_cameras.add(cam)
        rate = _median_gap_hz(samples)
        quality_names = [f"NT:/limelight-{cam}/{k}" for k in quality_sigs.keys()]

        sources.append(
            PoseSource(
                name=f"limelight-{cam}",
                source_class="vision",
                signal_names=[name],
                sample_count=len(samples),
                start_us=samples[0].timestamp_us,
                end_us=samples[-1].timestamp_us,
                median_rate_hz=rate,
                quality_signals=quality_names,
                samples=samples,
                notes=[
                    "Using raw botpose (not alliance-normalized); "
                    "results may need origin correction"
                ],
            )
        )

    # --- 3. Team-published raw vision pose (double[]) ---
    for pattern in _VISION_RAW_PATTERNS:
        for name in all_names:
            if pattern.match(name):
                sig = log_data.signals[name]
                if sig.info.type != SignalType.DOUBLE_ARRAY:
                    continue
                if len(sig.values) < 5:
                    continue
                samples = _parse_double_array_pose(
                    sig, name, "vision", theta_is_degrees=True
                )
                if not samples:
                    continue
                rate = _median_gap_hz(samples)
                sources.append(
                    PoseSource(
                        name=name,
                        source_class="vision",
                        signal_names=[name],
                        sample_count=len(samples),
                        start_us=samples[0].timestamp_us,
                        end_us=samples[-1].timestamp_us,
                        median_rate_hz=rate,
                        samples=samples,
                    )
                )

    # --- 4. Detect struct poses (not parseable yet, report only) ---
    for pattern in _STRUCT_POSE_PATTERNS:
        for name in all_names:
            if pattern.match(name):
                sig = log_data.signals[name]
                if sig.info.type != SignalType.STRUCT:
                    continue
                sources.append(
                    PoseSource(
                        name=name,
                        source_class="odometry" if "Drive" in name else "vision",
                        signal_names=[name],
                        sample_count=len(sig.values),
                        start_us=(sig.values[0].timestamp_us if sig.values else 0),
                        end_us=(sig.values[-1].timestamp_us if sig.values else 0),
                        notes=[
                            "Struct-typed signal detected but not parseable "
                            "without struct decoding support"
                        ],
                    )
                )

    return sources


# ---------------------------------------------------------------------------
# Reference path construction (v1 — weighted blend + smoothing)
# ---------------------------------------------------------------------------


def _lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation."""
    return a + (b - a) * t


def _lerp_angle(a: float, b: float, t: float) -> float:
    """Interpolate between two angles on the shortest arc."""
    diff = _angle_diff(b, a)
    return _normalize_angle(a + diff * t)


def interpolate_pose_at(path: list[PoseSample], timestamp_us: int) -> PoseSample | None:
    """Interpolate the reference path at a given timestamp.

    Uses linear interpolation for position and shortest-arc lerp for
    heading.  Returns ``None`` if timestamp is outside the path range.
    """
    if not path:
        return None
    if timestamp_us <= path[0].timestamp_us:
        return path[0]
    if timestamp_us >= path[-1].timestamp_us:
        return path[-1]

    # Binary search for the bracketing interval
    lo, hi = 0, len(path) - 1
    while lo < hi - 1:
        mid = (lo + hi) // 2
        if path[mid].timestamp_us <= timestamp_us:
            lo = mid
        else:
            hi = mid

    a, b = path[lo], path[hi]
    dt = b.timestamp_us - a.timestamp_us
    if dt <= 0:
        return a
    t = (timestamp_us - a.timestamp_us) / dt

    return PoseSample(
        timestamp_us=timestamp_us,
        x=_lerp(a.x, b.x, t),
        y=_lerp(a.y, b.y, t),
        theta=_lerp_angle(a.theta, b.theta, t),
        source_name="reference",
        source_class="fused",
    )


def compute_velocity_at(
    path: list[PoseSample], timestamp_us: int
) -> tuple[float, float, float]:
    """Compute (vx, vy, omega) at a timestamp via finite differences.

    Returns (0, 0, 0) if the path is too short or the timestamp is out
    of range.
    """
    if len(path) < 2:
        return (0.0, 0.0, 0.0)

    # Find nearest index
    idx = _bisect_path(path, timestamp_us)

    # Use a centered difference where possible
    if idx <= 0:
        a, b = path[0], path[1]
    elif idx >= len(path) - 1:
        a, b = path[-2], path[-1]
    else:
        a, b = path[idx - 1], path[idx + 1]

    dt = (b.timestamp_us - a.timestamp_us) / 1_000_000.0
    if dt <= 0:
        return (0.0, 0.0, 0.0)

    vx = (b.x - a.x) / dt
    vy = (b.y - a.y) / dt
    omega = _angle_diff(b.theta, a.theta) / dt
    return (vx, vy, omega)


def _bisect_path(path: list[PoseSample], timestamp_us: int) -> int:
    """Return the index of the sample closest to *timestamp_us*."""
    lo, hi = 0, len(path) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if path[mid].timestamp_us < timestamp_us:
            lo = mid + 1
        else:
            hi = mid
    return lo


def build_reference_path(
    log_data: LogData,
    sources: list[PoseSource] | None = None,
    *,
    resample_interval_us: int | None = None,
    smoothing_window_us: int = 80_000,  # 80 ms centered moving average
) -> list[PoseSample]:
    """Build the best-estimate reference path from available sources.

    **v1 algorithm:** weighted average of odometry (backbone) and
    latency-corrected vision, followed by centered moving-average
    smoothing.

    Parameters:
        log_data: Parsed log data (used for signal access if sources
            is None).
        sources: Pre-discovered sources (from ``discover_pose_sources``).
            If None, discovery runs automatically.
        resample_interval_us: Resample interval in microseconds.  If
            None, uses the odometry sample rate.
        smoothing_window_us: Half-width of the centered moving-average
            smoother in microseconds.

    Returns:
        A list of ``PoseSample`` objects on a uniform timeline,
        representing the best-estimate path.  Empty if no usable
        sources.
    """
    if sources is None:
        sources = discover_pose_sources(log_data)

    # Separate odometry and vision sources (with actual samples)
    odom_sources = [s for s in sources if s.source_class == "odometry" and s.samples]
    vision_sources = [s for s in sources if s.source_class == "vision" and s.samples]

    if not odom_sources:
        # Without odometry we can't build a reliable backbone
        # If there's only vision, return the highest-rate vision source
        if vision_sources:
            best = max(vision_sources, key=lambda s: s.sample_count)
            return best.samples[:]
        return []

    # Pick the highest-rate odometry source as the backbone
    primary_odom = max(odom_sources, key=lambda s: s.sample_count)
    backbone = primary_odom.samples

    if not vision_sources:
        # Odometry-only: apply smoothing and return
        return _smooth_path(backbone, smoothing_window_us)

    # --- Latency-correct vision samples ---
    corrected_vision: list[PoseSample] = []
    for vs in vision_sources:
        for s in vs.samples:
            # Shift timestamp backward by reported latency
            corrected_ts = s.timestamp_us - s.latency_us
            corrected_vision.append(
                PoseSample(
                    timestamp_us=corrected_ts,
                    x=s.x,
                    y=s.y,
                    theta=s.theta,
                    source_name=s.source_name,
                    source_class="vision",
                    confidence=s.confidence,
                    latency_us=s.latency_us,
                )
            )
    corrected_vision.sort(key=lambda s: s.timestamp_us)

    # --- Build fused path ---
    # Walk through the odometry backbone.  At each odometry sample, check
    # for nearby (within ±20 ms) vision updates.  If present, blend.
    VISION_WINDOW_US = 20_000  # ±20 ms window to associate vision updates
    ODOM_WEIGHT = 0.85
    # Vision weight is (1 - ODOM_WEIGHT) * confidence

    fused: list[PoseSample] = []
    vi = 0  # vision index

    for odom in backbone:
        ts = odom.timestamp_us

        # Collect vision samples within the window
        nearby_vision: list[PoseSample] = []
        while (
            vi < len(corrected_vision)
            and corrected_vision[vi].timestamp_us < ts - VISION_WINDOW_US
        ):
            vi += 1
        j = vi
        while (
            j < len(corrected_vision)
            and corrected_vision[j].timestamp_us <= ts + VISION_WINDOW_US
        ):
            vs = corrected_vision[j]
            # Reject physically implausible jumps
            dist = math.hypot(vs.x - odom.x, vs.y - odom.y)
            if dist < 2.0:  # within 2 m of odometry — plausible
                nearby_vision.append(vs)
            j += 1

        if not nearby_vision:
            fused.append(odom)
            continue

        # Weighted blend
        total_vision_conf = sum(v.confidence for v in nearby_vision)
        vision_weight = (1.0 - ODOM_WEIGHT) * min(total_vision_conf, 1.0)
        odom_w = 1.0 - vision_weight

        # Average the vision samples (weighted by confidence)
        vx_sum = sum(v.x * v.confidence for v in nearby_vision)
        vy_sum = sum(v.y * v.confidence for v in nearby_vision)
        # For heading, use circular mean via sin/cos
        vsin = sum(math.sin(v.theta) * v.confidence for v in nearby_vision)
        vcos = sum(math.cos(v.theta) * v.confidence for v in nearby_vision)

        if total_vision_conf > 0:
            vx_avg = vx_sum / total_vision_conf
            vy_avg = vy_sum / total_vision_conf
            vtheta_avg = math.atan2(vsin / total_vision_conf, vcos / total_vision_conf)
        else:
            vx_avg, vy_avg, vtheta_avg = odom.x, odom.y, odom.theta

        blend_x = odom_w * odom.x + vision_weight * vx_avg
        blend_y = odom_w * odom.y + vision_weight * vy_avg
        blend_theta = _lerp_angle(odom.theta, vtheta_avg, vision_weight)

        fused.append(
            PoseSample(
                timestamp_us=ts,
                x=blend_x,
                y=blend_y,
                theta=blend_theta,
                source_name="fused",
                source_class="fused",
                confidence=min(odom_w + vision_weight, 1.0),
            )
        )

    return _smooth_path(fused, smoothing_window_us)


def _smooth_path(path: list[PoseSample], half_window_us: int) -> list[PoseSample]:
    """Apply a centered moving-average smooth to a path."""
    if len(path) < 3 or half_window_us <= 0:
        return path[:]

    smoothed: list[PoseSample] = []
    n = len(path)

    for i in range(n):
        ts = path[i].timestamp_us
        lo = i
        hi = i

        # Expand window backward
        while lo > 0 and (ts - path[lo - 1].timestamp_us) <= half_window_us:
            lo -= 1
        # Expand window forward
        while hi < n - 1 and (path[hi + 1].timestamp_us - ts) <= half_window_us:
            hi += 1

        count = hi - lo + 1
        avg_x = sum(path[j].x for j in range(lo, hi + 1)) / count
        avg_y = sum(path[j].y for j in range(lo, hi + 1)) / count

        # Circular mean for heading
        sin_sum = sum(math.sin(path[j].theta) for j in range(lo, hi + 1))
        cos_sum = sum(math.cos(path[j].theta) for j in range(lo, hi + 1))
        avg_theta = math.atan2(sin_sum / count, cos_sum / count)

        smoothed.append(
            PoseSample(
                timestamp_us=ts,
                x=avg_x,
                y=avg_y,
                theta=avg_theta,
                source_name=path[i].source_name,
                source_class=path[i].source_class,
                confidence=path[i].confidence,
            )
        )

    return smoothed


# ---------------------------------------------------------------------------
# Divergence measurement
# ---------------------------------------------------------------------------


def compute_divergence_metrics(
    source_samples: list[PoseSample],
    reference_path: list[PoseSample],
) -> SourceMetrics:
    """Measure how a source compares to the reference path.

    For each source sample, interpolate the reference at the same
    timestamp and compute translation / heading error.
    """
    if not source_samples or not reference_path:
        return SourceMetrics(
            source_name=source_samples[0].source_name if source_samples else "",
            source_class=source_samples[0].source_class if source_samples else "",
            translation_rms_m=0.0,
            translation_max_m=0.0,
            heading_rms_rad=0.0,
            heading_max_rad=0.0,
            sample_count=0,
            coverage_fraction=0.0,
            confidence="low",
        )

    ref_start = reference_path[0].timestamp_us
    ref_end = reference_path[-1].timestamp_us
    ref_duration = ref_end - ref_start

    trans_errors: list[float] = []
    heading_errors: list[float] = []

    for s in source_samples:
        ref = interpolate_pose_at(reference_path, s.timestamp_us)
        if ref is None:
            continue

        te = math.hypot(s.x - ref.x, s.y - ref.y)
        he = abs(_angle_diff(s.theta, ref.theta))
        trans_errors.append(te)
        heading_errors.append(he)

    if not trans_errors:
        return SourceMetrics(
            source_name=source_samples[0].source_name,
            source_class=source_samples[0].source_class,
            translation_rms_m=0.0,
            translation_max_m=0.0,
            heading_rms_rad=0.0,
            heading_max_rad=0.0,
            sample_count=0,
            coverage_fraction=0.0,
            confidence="low",
        )

    n = len(trans_errors)
    trans_rms = math.sqrt(sum(e * e for e in trans_errors) / n)
    heading_rms = math.sqrt(sum(e * e for e in heading_errors) / n)

    # Coverage: fraction of reference timeline spanned by this source
    src_start = source_samples[0].timestamp_us
    src_end = source_samples[-1].timestamp_us
    overlap_start = max(src_start, ref_start)
    overlap_end = min(src_end, ref_end)
    coverage = max(0.0, (overlap_end - overlap_start)) / max(ref_duration, 1)

    # Confidence based on sample count and coverage
    if n >= 100 and coverage > 0.5:
        conf = "high"
    elif n >= 20 and coverage > 0.2:
        conf = "medium"
    else:
        conf = "low"

    return SourceMetrics(
        source_name=source_samples[0].source_name,
        source_class=source_samples[0].source_class,
        translation_rms_m=trans_rms,
        translation_max_m=max(trans_errors),
        heading_rms_rad=heading_rms,
        heading_max_rad=max(heading_errors),
        sample_count=n,
        coverage_fraction=coverage,
        confidence=conf,
    )


def find_divergence_events(
    source_samples: list[PoseSample],
    reference_path: list[PoseSample],
    *,
    translation_threshold_m: float = 0.5,
    min_duration_us: int = 100_000,  # 100 ms
) -> list[DivergenceEvent]:
    """Find time windows where a source diverges significantly."""
    events: list[DivergenceEvent] = []
    if not source_samples or not reference_path:
        return events

    in_event = False
    event_start = 0
    peak_trans = 0.0
    peak_heading = 0.0

    for s in source_samples:
        ref = interpolate_pose_at(reference_path, s.timestamp_us)
        if ref is None:
            continue

        te = math.hypot(s.x - ref.x, s.y - ref.y)
        he = abs(_angle_diff(s.theta, ref.theta))

        if te > translation_threshold_m:
            if not in_event:
                in_event = True
                event_start = s.timestamp_us
                peak_trans = te
                peak_heading = he
            else:
                peak_trans = max(peak_trans, te)
                peak_heading = max(peak_heading, he)
        else:
            if in_event:
                duration = s.timestamp_us - event_start
                if duration >= min_duration_us:
                    events.append(
                        DivergenceEvent(
                            source_name=s.source_name,
                            start_us=event_start,
                            end_us=s.timestamp_us,
                            peak_translation_error_m=peak_trans,
                            peak_heading_error_rad=peak_heading,
                        )
                    )
                in_event = False

    # Close any open event
    if in_event and source_samples:
        duration = source_samples[-1].timestamp_us - event_start
        if duration >= min_duration_us:
            events.append(
                DivergenceEvent(
                    source_name=source_samples[0].source_name,
                    start_us=event_start,
                    end_us=source_samples[-1].timestamp_us,
                    peak_translation_error_m=peak_trans,
                    peak_heading_error_rad=peak_heading,
                )
            )

    return events


# ---------------------------------------------------------------------------
# Analyzer
# ---------------------------------------------------------------------------


@register_analyzer
class PoseAnalysisAnalyzer(BaseAnalyzer):
    """Analyze robot pose sources and measure divergence."""

    name = "pose-analysis"
    description = (
        "Discover pose sources, build a best-estimate reference path, "
        "and measure how each source diverges. "
        "Use --accel-file to add a companion hoot/wpilog with IMU data."
    )

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--accel-file",
            help=(
                "Path to a companion .hoot or .wpilog file containing "
                "accelerometer/IMU data (e.g. a CANivore hoot file). "
                "Signals from this file are merged for accel consistency "
                "analysis."
            ),
        )

    def run(self, log_data: LogData, **options: Any) -> AnalysisResult:
        sources = discover_pose_sources(log_data)

        # --- Load companion accel file if provided ---
        accel_file = options.get("accel_file")
        accel_log_data: LogData | None = None
        if accel_file:
            from logreader.utils import file_extension

            ext = file_extension(accel_file)
            if ext == ".hoot":
                from logreader.hoot_reader import read_hoot

                accel_log_data = read_hoot(accel_file)
            elif ext == ".wpilog":
                from logreader.wpilog_reader import read_wpilog

                accel_log_data = read_wpilog(accel_file)

        # --- Discovery report ---
        parseable = [s for s in sources if s.samples]
        struct_only = [s for s in sources if not s.samples and s.notes]
        all_metrics: list[SourceMetrics] = []
        all_events: list[DivergenceEvent] = []

        # Build reference path
        reference_path = build_reference_path(log_data, sources)

        # Compute per-source metrics
        for src in parseable:
            if src.source_class == "odometry" and reference_path:
                # Compare raw odometry against the fused reference
                m = compute_divergence_metrics(src.samples, reference_path)
                all_metrics.append(m)
                evts = find_divergence_events(src.samples, reference_path)
                all_events.extend(evts)
            elif src.source_class == "vision" and reference_path:
                m = compute_divergence_metrics(src.samples, reference_path)
                all_metrics.append(m)
                evts = find_divergence_events(src.samples, reference_path)
                all_events.extend(evts)

        # --- Accel consistency analysis ---
        # Discover IMU sources from primary log AND companion file
        accel_sources: list[AccelSource] = discover_accel_sources(log_data)
        if accel_log_data:
            accel_sources.extend(discover_accel_sources(accel_log_data))

        accel_events: list[AccelConsistencyEvent] = []
        if reference_path:
            for asrc in accel_sources:
                if asrc.samples:
                    evts = analyze_accel_consistency(asrc, reference_path)
                    accel_events.extend(evts)

        # --- Build report ---
        summary_parts: list[str] = []
        summary_parts.append(
            f"Discovered {len(parseable)} parseable source(s), "
            f"{len(struct_only)} struct-only (not yet parseable)."
        )
        if accel_sources:
            summary_parts.append(f"Found {len(accel_sources)} accelerometer source(s).")

        if reference_path:
            dur_s = (
                reference_path[-1].timestamp_us - reference_path[0].timestamp_us
            ) / 1_000_000.0
            summary_parts.append(
                f"Reference path: {len(reference_path)} samples, "
                f"{dur_s:.1f} s duration."
            )

        # Table: discovered sources
        disc_columns = [
            "Source",
            "Class",
            "Samples",
            "Rate (Hz)",
            "Notes",
        ]
        disc_rows: list[dict[str, Any]] = []
        for src in sources:
            disc_rows.append(
                {
                    "Source": src.name,
                    "Class": src.source_class,
                    "Samples": src.sample_count,
                    "Rate (Hz)": (
                        f"{src.median_rate_hz:.1f}" if src.median_rate_hz else "-"
                    ),
                    "Notes": "; ".join(src.notes) if src.notes else "",
                }
            )

        # Table: divergence metrics
        div_columns = [
            "Source",
            "Class",
            "Samples",
            "Trans RMS (m)",
            "Trans Max (m)",
            "Heading RMS (°)",
            "Heading Max (°)",
            "Coverage",
            "Confidence",
        ]
        div_rows: list[dict[str, Any]] = []
        for m in all_metrics:
            div_rows.append(
                {
                    "Source": m.source_name,
                    "Class": m.source_class,
                    "Samples": m.sample_count,
                    "Trans RMS (m)": f"{m.translation_rms_m:.4f}",
                    "Trans Max (m)": f"{m.translation_max_m:.4f}",
                    "Heading RMS (°)": f"{m.heading_rms_rad * _RAD_TO_DEG:.2f}",
                    "Heading Max (°)": f"{m.heading_max_rad * _RAD_TO_DEG:.2f}",
                    "Coverage": f"{m.coverage_fraction:.0%}",
                    "Confidence": m.confidence,
                }
            )

        # Event summary
        if all_events:
            summary_parts.append(f"{len(all_events)} divergence event(s) detected.")

        # Build multi-section report
        report_lines: list[str] = []
        report_lines.append("=== Pose Analysis ===")
        report_lines.append("\n".join(summary_parts))
        report_lines.append("")

        # Discovery table
        report_lines.append("--- Discovered Sources ---")
        disc_result = AnalysisResult(
            analyzer_name=self.name,
            title="Discovered Sources",
            columns=disc_columns,
            rows=disc_rows,
        )
        report_lines.append(disc_result.format_table())
        report_lines.append("")

        # Divergence table
        if div_rows:
            report_lines.append("--- Source Divergence vs Reference Path ---")
            div_result = AnalysisResult(
                analyzer_name=self.name,
                title="Divergence",
                columns=div_columns,
                rows=div_rows,
            )
            report_lines.append(div_result.format_table())
            report_lines.append("")

        # Divergence events
        if all_events:
            report_lines.append("--- Divergence Events ---")
            for evt in all_events[:20]:
                start_s = evt.start_us / 1_000_000.0
                end_s = evt.end_us / 1_000_000.0
                dur_s = end_s - start_s
                report_lines.append(
                    f"  {evt.source_name}: "
                    f"{start_s:.1f}–{end_s:.1f} s ({dur_s:.1f} s), "
                    f"peak trans={evt.peak_translation_error_m:.3f} m, "
                    f"peak heading={evt.peak_heading_error_rad * _RAD_TO_DEG:.1f}°"
                )
            if len(all_events) > 20:
                report_lines.append(f"  ... and {len(all_events) - 20} more events")

        # Accel sources
        if accel_sources:
            report_lines.append("")
            report_lines.append("--- Accelerometer Sources ---")
            for asrc in accel_sources:
                rate_str = (
                    f"{asrc.median_rate_hz:.0f} Hz" if asrc.median_rate_hz else "-"
                )
                report_lines.append(
                    f"  {asrc.name}: {asrc.sample_count} samples, "
                    f"{rate_str}, "
                    f"gravity={'yes' if asrc.has_gravity_vector else 'no'}"
                )
                for n in asrc.notes:
                    report_lines.append(f"    {n}")

        # Accel consistency events
        if accel_events:
            report_lines.append("")
            impacts = [e for e in accel_events if e.event_type == "impact"]
            skids = [e for e in accel_events if e.event_type == "skid"]
            airborne = [e for e in accel_events if e.event_type == "airborne"]
            summary_parts.append(
                f"Accel consistency: {len(impacts)} impact(s), "
                f"{len(skids)} skid/traction-loss event(s), "
                f"{len(airborne)} airborne event(s)."
            )
            report_lines.append("--- Accel Consistency Events ---")
            for evt in accel_events[:30]:
                start_s = evt.start_us / 1_000_000.0
                end_s = evt.end_us / 1_000_000.0
                dur_ms = (evt.end_us - evt.start_us) / 1000.0
                report_lines.append(
                    f"  [{evt.event_type:>8s}] "
                    f"{start_s:.2f}–{end_s:.2f} s "
                    f"({dur_ms:.0f} ms): "
                    f"{evt.description}"
                )
            if len(accel_events) > 30:
                report_lines.append(f"  ... and {len(accel_events) - 30} more events")

        # Determine overall confidence
        source_classes = {s.source_class for s in parseable}
        if len(source_classes) >= 2:
            overall_conf = "high"
        elif len(source_classes) == 1:
            overall_conf = "medium"
        else:
            overall_conf = "low"

        return AnalysisResult(
            analyzer_name=self.name,
            title="Pose Analysis",
            summary="\n".join(report_lines),
            columns=div_columns if div_rows else disc_columns,
            rows=div_rows if div_rows else disc_rows,
            extra={
                "confidence": overall_conf,
                "discovered_sources": [
                    {
                        "name": s.name,
                        "class": s.source_class,
                        "samples": s.sample_count,
                        "rate_hz": s.median_rate_hz,
                        "notes": s.notes,
                    }
                    for s in sources
                ],
                "source_metrics": [
                    {
                        "source_name": m.source_name,
                        "source_class": m.source_class,
                        "translation_rms_m": m.translation_rms_m,
                        "translation_max_m": m.translation_max_m,
                        "heading_rms_deg": m.heading_rms_rad * _RAD_TO_DEG,
                        "heading_max_deg": m.heading_max_rad * _RAD_TO_DEG,
                        "sample_count": m.sample_count,
                        "coverage_fraction": m.coverage_fraction,
                        "confidence": m.confidence,
                    }
                    for m in all_metrics
                ],
                "divergence_events": [
                    {
                        "source": e.source_name,
                        "start_s": e.start_us / 1_000_000.0,
                        "end_s": e.end_us / 1_000_000.0,
                        "peak_trans_m": e.peak_translation_error_m,
                        "peak_heading_deg": e.peak_heading_error_rad * _RAD_TO_DEG,
                    }
                    for e in all_events
                ],
                "reference_path_samples": len(reference_path),
                "accel_sources": [
                    {
                        "name": a.name,
                        "samples": a.sample_count,
                        "rate_hz": a.median_rate_hz,
                        "has_gravity": a.has_gravity_vector,
                        "notes": a.notes,
                    }
                    for a in accel_sources
                ],
                "accel_events": [
                    {
                        "type": e.event_type,
                        "start_s": e.start_us / 1_000_000.0,
                        "end_s": e.end_us / 1_000_000.0,
                        "peak_residual_mps2": e.peak_residual_mps2,
                        "peak_gravity_fraction": e.peak_gravity_fraction,
                        "description": e.description,
                    }
                    for e in accel_events
                ],
            },
        )
