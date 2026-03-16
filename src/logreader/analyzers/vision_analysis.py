"""Vision analysis — Limelight performance metrics across cameras.

Analyses Limelight vision data to produce per-frame quality metrics,
per-camera summaries, per-tag-count and per-distance-band tables,
detection gap analysis, and FPS/thermal monitoring.

Tier 1 metrics implemented:
- Detection rate (valid_frames / total_frames)
- Tag count distribution (0, 1, 2, 3, 4+)
- Per-frame latency (tl, cl, total latency)
- Pose residual vs reference pose
- Per-tag ambiguity and distance
- Detection gaps and reacquisition time
- FPS and thermal trends

See ``docs/design-vision-analysis.md`` for the full design rationale.
"""

from __future__ import annotations

import argparse
import math
import re
import statistics
from dataclasses import dataclass, field
from typing import Any

from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer
from logreader.models import LogData, SignalData, SignalType

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Signal detection patterns
_LL_BOTPOSE_PATTERN = re.compile(
    r"NT:/limelight-([a-z0-9_-]+)/botpose_wpiblue$", re.I
)
_LL_BOTPOSE_ORB_PATTERN = re.compile(
    r"NT:/limelight-([a-z0-9_-]+)/botpose_orb_wpiblue$", re.I
)

# Companion signal suffixes (keyed by short name)
_LL_SIGNAL_SUFFIXES = {
    "hb": "hb",
    "rawfiducials": "rawfiducials",
    "tl": "tl",
    "cl": "cl",
    "stddevs": "stddevs",
    "hw": "hw",
    "tv": "tv",
    "tid": "tid",
}

# Default odometry signal patterns (same as pose_analysis)
_ODOM_POSE_PATTERNS: list[re.Pattern[str]] = [
    re.compile(r"NT:/Pose/robotPose$", re.I),
    re.compile(r"NT:/SmartDashboard/Field/Robot$", re.I),
    re.compile(r".*/EstimatedPose$", re.I),
    re.compile(r".*/OdometryPose$", re.I),
]

# Thresholds
DEFAULT_OUTLIER_M = 1.0  # residual above this → outlier
DEFAULT_GAP_THRESHOLD_MS = 500.0  # vision gap if no valid frame for this long
DEFAULT_DROPPED_FRAME_THRESHOLD_MS = 50.0  # heartbeat gap → dropped frame
HIGH_AMBIGUITY_THRESHOLD = 0.5

# Distance band boundaries (meters)
_DISTANCE_BANDS = [
    ("0–1m", 0.0, 1.0),
    ("1–2m", 1.0, 2.0),
    ("2–3m", 2.0, 3.0),
    ("3–4m", 3.0, 4.0),
    ("4–5m", 4.0, 5.0),
    ("5m+", 5.0, float("inf")),
]


# ---------------------------------------------------------------------------
# Data models
# ---------------------------------------------------------------------------


@dataclass
class TagDetection:
    """A single AprilTag detection within one frame."""

    tag_id: int
    txnc: float  # X offset from crosshair (degrees)
    tync: float  # Y offset from crosshair (degrees)
    area: float  # fraction of image (0–1)
    dist_to_camera: float  # meters
    dist_to_robot: float  # meters
    ambiguity: float  # 0–1


@dataclass
class VisionFrame:
    """Per-frame metrics for one camera."""

    timestamp_us: int
    camera: str

    # Pose (only valid when tag_count > 0)
    x: float
    y: float
    yaw_deg: float
    tag_count: int
    tag_span: float  # meters
    avg_tag_dist: float  # meters
    avg_tag_area: float  # fraction

    # Latency
    total_latency_ms: float
    pipeline_latency_ms: float
    capture_latency_ms: float

    # Quality
    stddev_mt1_x: float = 0.0
    stddev_mt1_y: float = 0.0
    stddev_mt1_yaw: float = 0.0
    stddev_mt2_x: float = 0.0
    stddev_mt2_y: float = 0.0

    # Per-tag details (from rawfiducials)
    tags: list[TagDetection] = field(default_factory=list)

    # Computed
    pose_residual_m: float | None = None  # vs odometry
    mt1_mt2_divergence_m: float | None = None
    max_ambiguity: float = 0.0


@dataclass
class CameraSummary:
    """Aggregate metrics for one camera in one match."""

    camera: str
    total_frames: int
    valid_frames: int
    valid_pct: float
    mean_latency_ms: float
    p95_latency_ms: float
    mean_fps: float
    mean_cpu_temp: float
    peak_cpu_temp: float
    dropped_frames: int
    mean_residual_m: float
    median_residual_m: float
    outlier_frames: int  # residual > threshold
    outlier_pct: float
    tag_count_distribution: dict[str, int] = field(default_factory=dict)


@dataclass
class TagSummary:
    """Aggregate metrics for one AprilTag ID as seen by one camera."""

    tag_id: int
    camera: str
    detections: int
    pct_of_valid: float
    mean_ambiguity: float
    mean_distance: float
    mean_area: float
    mean_residual_when_primary: float
    high_ambiguity_pct: float  # fraction with ambiguity > 0.5


@dataclass
class TagCountBand:
    """Aggregate metrics for frames at a given tag count."""

    tag_count: int
    frames: int
    mean_residual_m: float
    median_residual_m: float
    mean_ambiguity: float
    mean_stddev_x: float
    mean_stddev_y: float


@dataclass
class DistanceBand:
    """Aggregate metrics for detections in a distance range."""

    band_label: str  # e.g. "2–3m"
    band_min: float
    band_max: float
    camera: str
    detections: int
    mean_ambiguity: float
    high_ambiguity_pct: float
    mean_residual_m: float


@dataclass
class DetectionGap:
    """A period with no valid frames from a camera."""

    camera: str
    start_us: int
    end_us: int
    duration_ms: float
    reacquisition_delay_ms: float  # time until next valid frame


@dataclass
class HardwareStats:
    """Aggregated hardware statistics for one camera."""

    camera: str
    mean_fps: float
    min_fps: float
    max_fps: float
    mean_cpu_temp: float
    peak_cpu_temp: float
    mean_board_temp: float
    peak_board_temp: float
    mean_ram_mb: float


# ---------------------------------------------------------------------------
# Signal discovery
# ---------------------------------------------------------------------------


def _discover_cameras(log_data: LogData) -> dict[str, dict[str, SignalData | None]]:
    """Auto-detect Limelight cameras and their companion signals.

    Returns a dict mapping camera name → dict of signal short names →
    SignalData (or None if that signal is not present).
    """
    cameras: dict[str, dict[str, SignalData | None]] = {}
    all_names = list(log_data.signals.keys())

    # Find all cameras via botpose_wpiblue
    for name in all_names:
        m = _LL_BOTPOSE_PATTERN.match(name)
        if not m:
            continue
        cam = m.group(1)
        sig = log_data.signals[name]
        if sig.info.type != SignalType.DOUBLE_ARRAY or len(sig.values) < 10:
            continue

        signals: dict[str, SignalData | None] = {"botpose": sig}

        # Find companion signals
        prefix = f"NT:/limelight-{cam}/"
        for short_name, suffix in _LL_SIGNAL_SUFFIXES.items():
            full_name = prefix + suffix
            companion = log_data.get_signal(full_name)
            signals[short_name] = companion

        # Find MegaTag2/ORB botpose
        orb_name = f"NT:/limelight-{cam}/botpose_orb_wpiblue"
        signals["botpose_orb"] = log_data.get_signal(orb_name)

        cameras[f"limelight-{cam}"] = signals

    return cameras


def _find_odometry_signal(log_data: LogData) -> SignalData | None:
    """Find the best odometry / fused robot pose signal."""
    for pattern in _ODOM_POSE_PATTERNS:
        for name, sig in log_data.signals.items():
            if pattern.match(name) and sig.info.type == SignalType.DOUBLE_ARRAY:
                if len(sig.values) >= 10:
                    return sig
    return None


# ---------------------------------------------------------------------------
# Nearest-timestamp join helper
# ---------------------------------------------------------------------------


def _find_nearest(
    values: list[Any], target_us: int, *, max_delta_us: int = 100_000
) -> Any | None:
    """Find the value in a sorted TimestampedValue list nearest to target_us.

    Returns None if the nearest sample is further than max_delta_us.
    Uses binary search for efficiency.
    """
    if not values:
        return None

    lo, hi = 0, len(values) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if values[mid].timestamp_us < target_us:
            lo = mid + 1
        else:
            hi = mid

    # Check both lo and lo-1 for closest
    best_idx = lo
    if lo > 0:
        d_lo = abs(values[lo].timestamp_us - target_us)
        d_prev = abs(values[lo - 1].timestamp_us - target_us)
        if d_prev < d_lo:
            best_idx = lo - 1

    delta = abs(values[best_idx].timestamp_us - target_us)
    if delta > max_delta_us:
        return None
    return values[best_idx]


# ---------------------------------------------------------------------------
# Frame parsing
# ---------------------------------------------------------------------------


def _parse_rawfiducials(arr: list | tuple) -> list[TagDetection]:
    """Parse a rawfiducials double array into TagDetection objects.

    Each tag is 7 consecutive values:
    [tag_id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
    """
    tags: list[TagDetection] = []
    n = len(arr)
    i = 0
    while i + 6 < n:
        tag_id = int(arr[i])
        # Skip invalid tag IDs
        if tag_id < 0 or tag_id > 100:
            i += 7
            continue
        tags.append(
            TagDetection(
                tag_id=tag_id,
                txnc=float(arr[i + 1]),
                tync=float(arr[i + 2]),
                area=float(arr[i + 3]),
                dist_to_camera=float(arr[i + 4]),
                dist_to_robot=float(arr[i + 5]),
                ambiguity=float(arr[i + 6]),
            )
        )
        i += 7
    return tags


def _parse_frames(
    camera_name: str,
    signals: dict[str, SignalData | None],
) -> tuple[list[VisionFrame], int]:
    """Parse all botpose frames for a camera into VisionFrame objects.

    Returns (list of valid VisionFrames, total frame count from heartbeat).
    """
    botpose_sig = signals["botpose"]
    if botpose_sig is None:
        return [], 0

    # Build lookup indices for companion signals
    rawfid_sig = signals.get("rawfiducials")
    tl_sig = signals.get("tl")
    cl_sig = signals.get("cl")
    stddevs_sig = signals.get("stddevs")

    # Count total frames from heartbeat signal
    hb_sig = signals.get("hb")
    total_frames = len(hb_sig.values) if hb_sig and hb_sig.values else len(botpose_sig.values)

    frames: list[VisionFrame] = []

    for tv in botpose_sig.values:
        arr = tv.value
        if not isinstance(arr, (list, tuple)) or len(arr) < 11:
            continue

        x = float(arr[0])
        y = float(arr[1])
        yaw_deg = float(arr[5])
        total_latency_ms = float(arr[6])
        tag_count = int(float(arr[7]))
        tag_span = float(arr[8])
        avg_tag_dist = float(arr[9])
        avg_tag_area = float(arr[10])

        # Skip invalid frames (tag_count == 0 → pose is stale/zero)
        if tag_count < 1:
            continue

        # Skip zero-pose (clearly invalid)
        if abs(x) < 0.001 and abs(y) < 0.001:
            continue

        # Find nearest pipeline and capture latency
        pipeline_latency = 0.0
        capture_latency = 0.0
        if tl_sig and tl_sig.values:
            nearest_tl = _find_nearest(tl_sig.values, tv.timestamp_us)
            if nearest_tl is not None:
                pipeline_latency = float(nearest_tl.value)
        if cl_sig and cl_sig.values:
            nearest_cl = _find_nearest(cl_sig.values, tv.timestamp_us)
            if nearest_cl is not None:
                capture_latency = float(nearest_cl.value)

        # Find nearest stddevs
        stddev_mt1_x = 0.0
        stddev_mt1_y = 0.0
        stddev_mt1_yaw = 0.0
        stddev_mt2_x = 0.0
        stddev_mt2_y = 0.0
        if stddevs_sig and stddevs_sig.values:
            nearest_sd = _find_nearest(stddevs_sig.values, tv.timestamp_us)
            if nearest_sd is not None:
                sd_arr = nearest_sd.value
                if isinstance(sd_arr, (list, tuple)) and len(sd_arr) >= 12:
                    stddev_mt1_x = float(sd_arr[0])
                    stddev_mt1_y = float(sd_arr[1])
                    stddev_mt1_yaw = float(sd_arr[5])
                    stddev_mt2_x = float(sd_arr[6])
                    stddev_mt2_y = float(sd_arr[7])

        # Find nearest rawfiducials
        tags: list[TagDetection] = []
        if rawfid_sig and rawfid_sig.values:
            nearest_rf = _find_nearest(rawfid_sig.values, tv.timestamp_us)
            if nearest_rf is not None:
                rf_arr = nearest_rf.value
                if isinstance(rf_arr, (list, tuple)):
                    tags = _parse_rawfiducials(rf_arr)

        max_ambiguity = max((t.ambiguity for t in tags), default=0.0)

        frame = VisionFrame(
            timestamp_us=tv.timestamp_us,
            camera=camera_name,
            x=x,
            y=y,
            yaw_deg=yaw_deg,
            tag_count=tag_count,
            tag_span=tag_span,
            avg_tag_dist=avg_tag_dist,
            avg_tag_area=avg_tag_area,
            total_latency_ms=total_latency_ms,
            pipeline_latency_ms=pipeline_latency,
            capture_latency_ms=capture_latency,
            stddev_mt1_x=stddev_mt1_x,
            stddev_mt1_y=stddev_mt1_y,
            stddev_mt1_yaw=stddev_mt1_yaw,
            stddev_mt2_x=stddev_mt2_x,
            stddev_mt2_y=stddev_mt2_y,
            tags=tags,
            max_ambiguity=max_ambiguity,
        )
        frames.append(frame)

    return frames, total_frames


# ---------------------------------------------------------------------------
# Pose residual computation
# ---------------------------------------------------------------------------


def _compute_residuals(
    frames: list[VisionFrame],
    log_data: LogData,
) -> None:
    """Compute pose residual for each frame against the reference path.

    Mutates frames in-place, setting pose_residual_m.
    Uses pose_analysis's build_reference_path and interpolate_pose_at.
    """
    from logreader.analyzers.pose_analysis import (
        build_reference_path,
        interpolate_pose_at,
    )

    ref_path = build_reference_path(log_data)
    if not ref_path:
        return

    for frame in frames:
        # Latency-compensate: the frame's measurement corresponds to an
        # earlier time by total_latency_ms
        latency_us = int(frame.total_latency_ms * 1000)
        query_ts = frame.timestamp_us - latency_us

        ref = interpolate_pose_at(ref_path, query_ts)
        if ref is None:
            continue

        dx = frame.x - ref.x
        dy = frame.y - ref.y
        frame.pose_residual_m = math.sqrt(dx * dx + dy * dy)


# ---------------------------------------------------------------------------
# Dropped-frame detection
# ---------------------------------------------------------------------------


def _count_dropped_frames(
    signals: dict[str, SignalData | None],
    threshold_ms: float = DEFAULT_DROPPED_FRAME_THRESHOLD_MS,
) -> int:
    """Count heartbeat gaps exceeding threshold (dropped frames)."""
    hb_sig = signals.get("hb")
    if not hb_sig or len(hb_sig.values) < 2:
        return 0

    threshold_us = int(threshold_ms * 1000)
    dropped = 0
    prev_ts = hb_sig.values[0].timestamp_us
    for v in hb_sig.values[1:]:
        if v.timestamp_us - prev_ts > threshold_us:
            dropped += 1
        prev_ts = v.timestamp_us
    return dropped


# ---------------------------------------------------------------------------
# Detection gap analysis
# ---------------------------------------------------------------------------


def _find_detection_gaps(
    frames: list[VisionFrame],
    camera: str,
    gap_threshold_ms: float = DEFAULT_GAP_THRESHOLD_MS,
) -> list[DetectionGap]:
    """Find periods where a camera had no valid frames."""
    cam_frames = [f for f in frames if f.camera == camera]
    if len(cam_frames) < 2:
        return []

    cam_frames.sort(key=lambda f: f.timestamp_us)
    threshold_us = int(gap_threshold_ms * 1000)

    gaps: list[DetectionGap] = []
    for i in range(1, len(cam_frames)):
        gap_us = cam_frames[i].timestamp_us - cam_frames[i - 1].timestamp_us
        if gap_us > threshold_us:
            duration_ms = gap_us / 1000.0
            # Reacquisition delay is the gap itself (time from last
            # valid frame to next valid frame)
            gaps.append(
                DetectionGap(
                    camera=camera,
                    start_us=cam_frames[i - 1].timestamp_us,
                    end_us=cam_frames[i].timestamp_us,
                    duration_ms=duration_ms,
                    reacquisition_delay_ms=duration_ms,
                )
            )

    return gaps


# ---------------------------------------------------------------------------
# Hardware stats
# ---------------------------------------------------------------------------


def _compute_hw_stats(
    camera: str,
    signals: dict[str, SignalData | None],
) -> HardwareStats:
    """Extract FPS and thermal stats from the hw signal.

    hw array: [fps, cpu_temp, ram_mb, board_temp]
    """
    hw_sig = signals.get("hw")
    default = HardwareStats(
        camera=camera,
        mean_fps=0.0,
        min_fps=0.0,
        max_fps=0.0,
        mean_cpu_temp=0.0,
        peak_cpu_temp=0.0,
        mean_board_temp=0.0,
        peak_board_temp=0.0,
        mean_ram_mb=0.0,
    )

    if not hw_sig or not hw_sig.values:
        return default

    fps_vals: list[float] = []
    cpu_temps: list[float] = []
    board_temps: list[float] = []
    ram_vals: list[float] = []

    for v in hw_sig.values:
        arr = v.value
        if not isinstance(arr, (list, tuple)) or len(arr) < 4:
            continue
        fps_vals.append(float(arr[0]))
        cpu_temps.append(float(arr[1]))
        ram_vals.append(float(arr[2]))
        board_temps.append(float(arr[3]))

    if not fps_vals:
        return default

    return HardwareStats(
        camera=camera,
        mean_fps=statistics.mean(fps_vals),
        min_fps=min(fps_vals),
        max_fps=max(fps_vals),
        mean_cpu_temp=statistics.mean(cpu_temps),
        peak_cpu_temp=max(cpu_temps),
        mean_board_temp=statistics.mean(board_temps),
        peak_board_temp=max(board_temps),
        mean_ram_mb=statistics.mean(ram_vals),
    )


# ---------------------------------------------------------------------------
# Aggregation: per-camera summary
# ---------------------------------------------------------------------------


def _camera_summary(
    camera: str,
    frames: list[VisionFrame],
    total_frames: int,
    signals: dict[str, SignalData | None],
    outlier_threshold: float,
) -> CameraSummary:
    """Compute the per-camera summary table row."""
    cam_frames = [f for f in frames if f.camera == camera]
    valid = len(cam_frames)
    valid_pct = (valid / total_frames * 100.0) if total_frames > 0 else 0.0

    # Latency stats
    latencies = [f.total_latency_ms for f in cam_frames]
    mean_lat = statistics.mean(latencies) if latencies else 0.0
    p95_lat = _percentile(latencies, 0.95) if latencies else 0.0

    # Residual stats
    residuals = [f.pose_residual_m for f in cam_frames if f.pose_residual_m is not None]
    mean_res = statistics.mean(residuals) if residuals else 0.0
    median_res = statistics.median(residuals) if residuals else 0.0
    outliers = sum(1 for r in residuals if r > outlier_threshold)
    outlier_pct = (outliers / valid * 100.0) if valid > 0 else 0.0

    # Dropped frames
    dropped = _count_dropped_frames(signals)

    # Hardware stats
    hw = _compute_hw_stats(camera, signals)

    # Tag count distribution
    tag_dist: dict[str, int] = {"0": total_frames - valid}
    for f in cam_frames:
        key = str(f.tag_count) if f.tag_count <= 3 else "4+"
        tag_dist[key] = tag_dist.get(key, 0) + 1

    return CameraSummary(
        camera=camera,
        total_frames=total_frames,
        valid_frames=valid,
        valid_pct=valid_pct,
        mean_latency_ms=mean_lat,
        p95_latency_ms=p95_lat,
        mean_fps=hw.mean_fps,
        mean_cpu_temp=hw.mean_cpu_temp,
        peak_cpu_temp=hw.peak_cpu_temp,
        dropped_frames=dropped,
        mean_residual_m=mean_res,
        median_residual_m=median_res,
        outlier_frames=outliers,
        outlier_pct=outlier_pct,
        tag_count_distribution=tag_dist,
    )


# ---------------------------------------------------------------------------
# Aggregation: per-tag-count table
# ---------------------------------------------------------------------------


def _tag_count_table(frames: list[VisionFrame]) -> list[TagCountBand]:
    """Group frames by tag count and compute aggregate stats."""
    buckets: dict[int, list[VisionFrame]] = {}
    for f in frames:
        tc = f.tag_count if f.tag_count <= 4 else 4
        buckets.setdefault(tc, []).append(f)

    bands: list[TagCountBand] = []
    for tc in sorted(buckets):
        bucket = buckets[tc]
        residuals = [
            f.pose_residual_m for f in bucket if f.pose_residual_m is not None
        ]
        ambiguities = [f.max_ambiguity for f in bucket if f.tags]
        stddevs_x = [f.stddev_mt1_x for f in bucket if f.stddev_mt1_x > 0]
        stddevs_y = [f.stddev_mt1_y for f in bucket if f.stddev_mt1_y > 0]

        bands.append(
            TagCountBand(
                tag_count=tc,
                frames=len(bucket),
                mean_residual_m=statistics.mean(residuals) if residuals else 0.0,
                median_residual_m=statistics.median(residuals) if residuals else 0.0,
                mean_ambiguity=statistics.mean(ambiguities) if ambiguities else 0.0,
                mean_stddev_x=statistics.mean(stddevs_x) if stddevs_x else 0.0,
                mean_stddev_y=statistics.mean(stddevs_y) if stddevs_y else 0.0,
            )
        )
    return bands


# ---------------------------------------------------------------------------
# Aggregation: per-tag-ID summary
# ---------------------------------------------------------------------------


def _tag_id_table(
    frames: list[VisionFrame],
    camera: str,
) -> list[TagSummary]:
    """Build per-tag-ID summary for one camera."""
    cam_frames = [f for f in frames if f.camera == camera]
    valid_count = len(cam_frames)

    # Gather all tag detections with frame context
    tag_data: dict[int, list[tuple[TagDetection, VisionFrame]]] = {}
    for f in cam_frames:
        for tag in f.tags:
            tag_data.setdefault(tag.tag_id, []).append((tag, f))

    summaries: list[TagSummary] = []
    for tag_id in sorted(tag_data):
        detections = tag_data[tag_id]
        n = len(detections)
        pct = (n / valid_count * 100.0) if valid_count > 0 else 0.0

        ambiguities = [t.ambiguity for t, _ in detections]
        distances = [t.dist_to_camera for t, _ in detections]
        areas = [t.area for t, _ in detections]

        # Mean residual when this tag was the "primary" (closest) tag
        primary_residuals: list[float] = []
        for tag, frame in detections:
            if frame.pose_residual_m is None:
                continue
            # Tag is primary if it's the closest to the camera
            if frame.tags:
                closest = min(frame.tags, key=lambda t: t.dist_to_camera)
                if closest.tag_id == tag_id:
                    primary_residuals.append(frame.pose_residual_m)

        high_amb = sum(1 for a in ambiguities if a > HIGH_AMBIGUITY_THRESHOLD)
        high_amb_pct = (high_amb / n * 100.0) if n > 0 else 0.0

        summaries.append(
            TagSummary(
                tag_id=tag_id,
                camera=camera,
                detections=n,
                pct_of_valid=pct,
                mean_ambiguity=statistics.mean(ambiguities),
                mean_distance=statistics.mean(distances),
                mean_area=statistics.mean(areas),
                mean_residual_when_primary=(
                    statistics.mean(primary_residuals) if primary_residuals else 0.0
                ),
                high_ambiguity_pct=high_amb_pct,
            )
        )

    # Sort by detection count descending
    summaries.sort(key=lambda s: s.detections, reverse=True)
    return summaries


# ---------------------------------------------------------------------------
# Aggregation: per-distance-band table
# ---------------------------------------------------------------------------


def _distance_band_table(
    frames: list[VisionFrame],
    camera: str,
) -> list[DistanceBand]:
    """Build ambiguity-distance band table for one camera."""
    cam_frames = [f for f in frames if f.camera == camera]

    # Collect all individual tag detections with their frame's residual
    band_data: dict[str, list[tuple[TagDetection, float | None]]] = {
        label: [] for label, _, _ in _DISTANCE_BANDS
    }

    for f in cam_frames:
        for tag in f.tags:
            for label, lo, hi in _DISTANCE_BANDS:
                if lo <= tag.dist_to_camera < hi:
                    band_data[label].append((tag, f.pose_residual_m))
                    break

    bands: list[DistanceBand] = []
    for label, lo, hi in _DISTANCE_BANDS:
        entries = band_data[label]
        if not entries:
            continue

        ambiguities = [t.ambiguity for t, _ in entries]
        high_amb = sum(1 for a in ambiguities if a > HIGH_AMBIGUITY_THRESHOLD)
        residuals = [r for _, r in entries if r is not None]

        bands.append(
            DistanceBand(
                band_label=label,
                band_min=lo,
                band_max=hi,
                camera=camera,
                detections=len(entries),
                mean_ambiguity=statistics.mean(ambiguities),
                high_ambiguity_pct=(high_amb / len(entries) * 100.0),
                mean_residual_m=statistics.mean(residuals) if residuals else 0.0,
            )
        )

    return bands


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------


def _percentile(data: list[float], p: float) -> float:
    """Compute a percentile (0–1) from a list of floats."""
    if not data:
        return 0.0
    s = sorted(data)
    k = (len(s) - 1) * p
    f = int(k)
    c = f + 1
    if c >= len(s):
        return s[-1]
    return s[f] + (s[c] - s[f]) * (k - f)


# ---------------------------------------------------------------------------
# Report formatting
# ---------------------------------------------------------------------------


def _format_summary_report(
    camera_summaries: list[CameraSummary],
    tag_count_bands: list[TagCountBand],
    detection_gaps: list[DetectionGap],
    tag_summaries: list[TagSummary],
    distance_bands: list[DistanceBand],
    *,
    detail: bool = False,
) -> str:
    """Format the analysis results as a human-readable report."""
    lines: list[str] = []

    # Per-camera summaries
    for cs in camera_summaries:
        lines.append(f"Camera: {cs.camera}")
        lines.append(
            f"  Frames: {cs.total_frames} total, "
            f"{cs.valid_frames} valid ({cs.valid_pct:.1f}%)"
        )
        lines.append(
            f"  Latency: mean {cs.mean_latency_ms:.1f}ms, "
            f"P95 {cs.p95_latency_ms:.1f}ms"
        )
        if cs.mean_fps > 0:
            lines.append(
                f"  FPS: {cs.mean_fps:.1f} avg, "
                f"CPU: {cs.mean_cpu_temp:.1f}\u00b0C avg / "
                f"{cs.peak_cpu_temp:.1f}\u00b0C peak"
            )
        lines.append(f"  Dropped frames: {cs.dropped_frames} ({cs.dropped_frames / max(cs.total_frames, 1) * 100:.1f}%)")
        if cs.mean_residual_m > 0 or cs.median_residual_m > 0:
            lines.append(
                f"  Pose residual: mean {cs.mean_residual_m:.2f}m, "
                f"median {cs.median_residual_m:.2f}m"
            )
            lines.append(
                f"  Outliers (>1m): {cs.outlier_frames} ({cs.outlier_pct:.1f}%)"
            )
        # Tag count distribution
        dist_parts = []
        for key in ["0", "1", "2", "3", "4+"]:
            count = cs.tag_count_distribution.get(key, 0)
            if count > 0:
                dist_parts.append(f"{key} tags: {count}")
        if dist_parts:
            lines.append(f"  Tag distribution: {', '.join(dist_parts)}")
        lines.append("")

    # Tag count breakdown (all cameras combined)
    if tag_count_bands:
        lines.append("Tag count breakdown (all cameras):")
        for band in tag_count_bands:
            tc_label = f"{band.tag_count}+" if band.tag_count >= 4 else str(band.tag_count)
            resid_str = f"  mean residual {band.mean_residual_m:.2f}m" if band.mean_residual_m > 0 else ""
            lines.append(
                f"  {tc_label} tag{'s' if band.tag_count != 1 else ''}: "
                f"{band.frames:5d} frames{resid_str}"
            )
        lines.append("")

    # Detection gaps summary
    if detection_gaps:
        lines.append("Detection gaps (>500ms):")
        for camera in sorted(set(g.camera for g in detection_gaps)):
            cam_gaps = [g for g in detection_gaps if g.camera == camera]
            lines.append(
                f"  {camera}: {len(cam_gaps)} gaps, "
                f"longest {max(g.duration_ms for g in cam_gaps):.0f}ms, "
                f"mean {statistics.mean(g.duration_ms for g in cam_gaps):.0f}ms"
            )
        lines.append("")

    # Detail tables
    if detail and tag_summaries:
        for camera in sorted(set(ts.camera for ts in tag_summaries)):
            cam_tags = [ts for ts in tag_summaries if ts.camera == camera]
            if not cam_tags:
                continue
            lines.append(f"Per-tag detail: {camera}")
            lines.append(
                f"{'Tag':>4s}  {'Detect':>6s}  {'Valid%':>6s}  "
                f"{'MeanAmb':>7s}  {'MeanDist':>8s}  {'MeanArea':>8s}  "
                f"{'MeanRes':>7s}  {'HighAmb%':>8s}"
            )
            lines.append(
                f"{'----':>4s}  {'------':>6s}  {'------':>6s}  "
                f"{'-------':>7s}  {'--------':>8s}  {'--------':>8s}  "
                f"{'-------':>7s}  {'--------':>8s}"
            )
            for ts in cam_tags:
                lines.append(
                    f"{ts.tag_id:4d}  {ts.detections:6d}  "
                    f"{ts.pct_of_valid:5.1f}%  "
                    f"{ts.mean_ambiguity:7.3f}  "
                    f"{ts.mean_distance:7.2f}m  "
                    f"{ts.mean_area:8.4f}  "
                    f"{ts.mean_residual_when_primary:6.2f}m  "
                    f"{ts.high_ambiguity_pct:7.1f}%"
                )
            lines.append("")

    if detail and distance_bands:
        for camera in sorted(set(db.camera for db in distance_bands)):
            cam_bands = [db for db in distance_bands if db.camera == camera]
            if not cam_bands:
                continue
            lines.append(f"Ambiguity-distance: {camera}")
            lines.append(
                f"{'Distance':>10s}  {'Detect':>6s}  {'MeanAmb':>7s}  "
                f"{'HighAmb%':>8s}  {'MeanRes':>7s}"
            )
            lines.append(
                f"{'--------':>10s}  {'------':>6s}  {'-------':>7s}  "
                f"{'--------':>8s}  {'-------':>7s}"
            )
            for db in cam_bands:
                lines.append(
                    f"{db.band_label:>10s}  {db.detections:6d}  "
                    f"{db.mean_ambiguity:7.3f}  "
                    f"{db.high_ambiguity_pct:7.1f}%  "
                    f"{db.mean_residual_m:6.2f}m"
                )
            lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Analyzer
# ---------------------------------------------------------------------------


@register_analyzer
class VisionAnalyzer(BaseAnalyzer):
    """Limelight vision performance analyzer."""

    name = "vision-analysis"
    description = "Analyse Limelight vision quality, coverage, and reliability"

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--detail",
            action="store_true",
            default=False,
            help="Show per-tag and per-distance-band detail tables",
        )
        parser.add_argument(
            "--cameras",
            type=str,
            default=None,
            help="Comma-separated camera names (e.g. limelight-a,limelight-b)",
        )
        parser.add_argument(
            "--outlier-threshold",
            type=float,
            default=DEFAULT_OUTLIER_M,
            help=f"Residual threshold for outlier flagging (default: {DEFAULT_OUTLIER_M}m)",
        )

    def run(self, log_data: LogData, **options: Any) -> AnalysisResult:
        detail: bool = options.get("detail", False)
        camera_filter: str | None = options.get("cameras")
        outlier_threshold: float = options.get(
            "outlier_threshold", DEFAULT_OUTLIER_M
        )

        # --- Discover cameras ---
        all_cameras = _discover_cameras(log_data)

        if not all_cameras:
            return AnalysisResult(
                analyzer_name=self.name,
                title="Vision Analysis",
                summary="No Limelight cameras detected. Check signal names.",
            )

        # Apply camera filter
        if camera_filter:
            requested = {c.strip() for c in camera_filter.split(",")}
            all_cameras = {
                k: v for k, v in all_cameras.items() if k in requested
            }
            if not all_cameras:
                return AnalysisResult(
                    analyzer_name=self.name,
                    title="Vision Analysis",
                    summary=f"No matching cameras found for: {camera_filter}",
                )

        # --- Parse frames per camera ---
        all_frames: list[VisionFrame] = []
        camera_frame_counts: dict[str, int] = {}

        for cam_name, signals in all_cameras.items():
            frames, total = _parse_frames(cam_name, signals)
            all_frames.extend(frames)
            camera_frame_counts[cam_name] = total

        # --- Compute residuals ---
        _compute_residuals(all_frames, log_data)

        # --- Aggregate per camera ---
        camera_summaries: list[CameraSummary] = []
        for cam_name, signals in all_cameras.items():
            summary = _camera_summary(
                cam_name,
                all_frames,
                camera_frame_counts[cam_name],
                signals,
                outlier_threshold,
            )
            camera_summaries.append(summary)

        # --- Tag count breakdown ---
        tag_count_bands = _tag_count_table(all_frames)

        # --- Per-tag and per-distance-band tables ---
        tag_summaries: list[TagSummary] = []
        distance_bands: list[DistanceBand] = []
        for cam_name in all_cameras:
            tag_summaries.extend(_tag_id_table(all_frames, cam_name))
            distance_bands.extend(_distance_band_table(all_frames, cam_name))

        # --- Detection gaps ---
        detection_gaps: list[DetectionGap] = []
        for cam_name in all_cameras:
            detection_gaps.extend(_find_detection_gaps(all_frames, cam_name))

        # --- Format report ---
        report = _format_summary_report(
            camera_summaries,
            tag_count_bands,
            detection_gaps,
            tag_summaries,
            distance_bands,
            detail=detail,
        )

        # --- Build result ---
        # Summary line
        total_valid = sum(cs.valid_frames for cs in camera_summaries)
        total_all = sum(cs.total_frames for cs in camera_summaries)
        summary_line = (
            f"{len(camera_summaries)} camera(s), "
            f"{total_valid}/{total_all} valid frames "
            f"({total_valid / max(total_all, 1) * 100:.1f}%)"
        )

        # Per-camera summary table for AnalysisResult
        columns = [
            "Camera",
            "Total",
            "Valid",
            "Valid%",
            "MeanLat",
            "MeanRes",
            "MedRes",
            "Outliers",
        ]
        rows: list[dict[str, Any]] = []
        for cs in camera_summaries:
            rows.append(
                {
                    "Camera": cs.camera,
                    "Total": cs.total_frames,
                    "Valid": cs.valid_frames,
                    "Valid%": f"{cs.valid_pct:.1f}%",
                    "MeanLat": f"{cs.mean_latency_ms:.1f}ms",
                    "MeanRes": f"{cs.mean_residual_m:.2f}m",
                    "MedRes": f"{cs.median_residual_m:.2f}m",
                    "Outliers": f"{cs.outlier_frames} ({cs.outlier_pct:.1f}%)",
                }
            )

        return AnalysisResult(
            analyzer_name=self.name,
            title="Vision Analysis",
            summary=summary_line + "\n\n" + report,
            columns=columns,
            rows=rows,
            extra={
                "cameras": camera_summaries,
                "per_tag": tag_summaries,
                "per_tag_count": tag_count_bands,
                "per_distance_band": distance_bands,
                "frames": all_frames,
                "detection_gaps": [
                    (g.camera, g.start_us / 1e6, g.end_us / 1e6)
                    for g in detection_gaps
                ],
                "detection_gap_details": detection_gaps,
                "hardware_stats": {
                    cam: _compute_hw_stats(cam, sigs)
                    for cam, sigs in all_cameras.items()
                },
            },
        )
