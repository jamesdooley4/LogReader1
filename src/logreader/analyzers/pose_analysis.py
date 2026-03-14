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
        "and measure how each source diverges"
    )

    def run(self, log_data: LogData, **options: Any) -> AnalysisResult:
        sources = discover_pose_sources(log_data)

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

        # --- Build report ---
        summary_parts: list[str] = []
        summary_parts.append(
            f"Discovered {len(parseable)} parseable source(s), "
            f"{len(struct_only)} struct-only (not yet parseable)."
        )

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
            },
        )
