#!/usr/bin/env python3
"""Camera pose calibration analysis.

Estimates whether the configured ``camerapose_robotspace`` for each Limelight
is correct by analyzing systematic pose residual patterns across multiple log
files.

Algorithm overview
------------------
1. For each camera, collect frames where tags are visible and odometry is
   available.
2. Compute the *signed* residual vector (vision_pose - odometry_pose) for each
   valid frame.  A consistent directional bias indicates a camera pose error.
3. Group residuals by tag ID, tag count, and robot heading to separate
   geometry-dependent biases from random noise.
4. Use the ``targetpose_cameraspace`` signal (which depends only on camera
   intrinsics, not robot config) combined with known AprilTag field positions
   to independently reconstruct what the robot pose *should* be for a given
   ``camerapose_robotspace``.  Then optimize the 6-DOF camera pose to minimize
   residuals against odometry.

Usage
-----
    python scripts/camera_pose_calibration.py D:\\Temp\\2026-04-6_Practice_Test

Processes all .wpilog files in the given directory (or a single file).
"""

from __future__ import annotations

import math
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

# ---------------------------------------------------------------------------
# AprilTag 2026 field layout (inches → meters)
# CSV columns: ID, X, Y, Z, Z-Rotation (yaw deg), X-Rotation (pitch deg)
# ---------------------------------------------------------------------------
INCHES_TO_METERS = 0.0254

_RAW_TAGS = [
    (1, 467.637, 292.314, 35, 180, 0),
    (2, 469.111, 182.6, 44.25, 90, 0),
    (3, 445.349, 172.844, 44.25, 180, 0),
    (4, 445.349, 158.844, 44.25, 180, 0),
    (5, 469.111, 135.088, 44.25, 270, 0),
    (6, 467.637, 25.374, 35, 180, 0),
    (7, 470.586, 25.374, 35, 0, 0),
    (8, 483.111, 135.088, 44.25, 270, 0),
    (9, 492.881, 144.844, 44.25, 0, 0),
    (10, 492.881, 158.844, 44.25, 0, 0),
    (11, 483.111, 182.6, 44.25, 90, 0),
    (12, 470.586, 292.314, 35, 0, 0),
    (13, 650.918, 291.469, 21.75, 180, 0),
    (14, 650.918, 274.469, 21.75, 180, 0),
    (15, 650.904, 170.219, 21.75, 180, 0),
    (16, 650.904, 153.219, 21.75, 180, 0),
    (17, 183.586, 25.374, 35, 0, 0),
    (18, 182.111, 135.088, 44.25, 270, 0),
    (19, 205.873, 144.844, 44.25, 0, 0),
    (20, 205.873, 158.844, 44.25, 0, 0),
    (21, 182.111, 182.6, 44.25, 90, 0),
    (22, 183.586, 292.314, 35, 0, 0),
    (23, 180.637, 292.314, 35, 180, 0),
    (24, 168.111, 182.6, 44.25, 90, 0),
    (25, 158.341, 172.844, 44.25, 180, 0),
    (26, 158.341, 158.844, 44.25, 180, 0),
    (27, 168.111, 135.088, 44.25, 270, 0),
    (28, 180.637, 25.374, 35, 180, 0),
    (29, 0.305, 26.219, 21.75, 0, 0),
    (30, 0.305, 43.219, 21.75, 0, 0),
    (31, 0.318, 147.469, 21.75, 0, 0),
    (32, 0.318, 164.469, 21.75, 0, 0),
]


@dataclass
class TagPose:
    """Known field position of an AprilTag."""
    tag_id: int
    x: float  # meters, WPILib Blue origin
    y: float
    z: float
    yaw_deg: float
    pitch_deg: float


TAG_FIELD_POSES: dict[int, TagPose] = {}
for _id, _x, _y, _z, _yaw, _pitch in _RAW_TAGS:
    TAG_FIELD_POSES[_id] = TagPose(
        tag_id=_id,
        x=_x * INCHES_TO_METERS,
        y=_y * INCHES_TO_METERS,
        z=_z * INCHES_TO_METERS,
        yaw_deg=_yaw,
        pitch_deg=_pitch,
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def _rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


def _wrap_deg(d: float) -> float:
    """Wrap angle to [-180, 180]."""
    return (d + 180) % 360 - 180


def _pose6_to_matrix(tx: float, ty: float, tz: float,
                      roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Build a 4×4 homogeneous transform from [tx, ty, tz, roll, pitch, yaw].

    Limelight convention: the 6-element arrays are
    [tx, ty, tz, roll, pitch, yaw] in meters and degrees.
    WPILib rotation order: Z (yaw) → Y (pitch) → X (roll), i.e. intrinsic ZYX.
    """
    r = Rotation.from_euler("ZYX", [yaw_deg, pitch_deg, roll_deg], degrees=True)
    T = np.eye(4)
    T[:3, :3] = r.as_matrix()
    T[:3, 3] = [tx, ty, tz]
    return T


def _matrix_to_pose6(T: np.ndarray) -> tuple[float, float, float, float, float, float]:
    """Extract [tx, ty, tz, roll, pitch, yaw] from a 4×4 transform."""
    tx, ty, tz = T[:3, 3]
    r = Rotation.from_matrix(T[:3, :3])
    yaw, pitch, roll = r.as_euler("ZYX", degrees=True)
    return tx, ty, tz, roll, pitch, yaw


def _tag_field_transform(tag: TagPose) -> np.ndarray:
    """4×4 transform: field → tag (tag pose in field coordinates)."""
    return _pose6_to_matrix(tag.x, tag.y, tag.z, 0.0, tag.pitch_deg, tag.yaw_deg)


@dataclass
class CameraFrame:
    """One camera observation for calibration."""
    timestamp_us: int
    camera: str
    # botpose from Limelight (MegaTag1, WPILib blue)
    vision_x: float
    vision_y: float
    vision_yaw_deg: float
    tag_count: int
    avg_tag_dist: float
    total_latency_ms: float
    # Odometry reference (interpolated)
    odom_x: float
    odom_y: float
    odom_yaw_deg: float
    # Per-tag data from rawfiducials
    tag_ids: list[int] = field(default_factory=list)
    tag_ambiguities: list[float] = field(default_factory=list)
    tag_distances: list[float] = field(default_factory=list)
    # targetpose_cameraspace for primary tag
    tgt_cam_pose: list[float] | None = None
    primary_tag_id: int | None = None


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def _decode_pose2d(raw: bytes) -> tuple[float, float, float]:
    """Unpack a WPILib Pose2d struct (3 doubles: x, y, theta_rad)."""
    if len(raw) != 24:
        raise ValueError(f"Expected 24 bytes for Pose2d, got {len(raw)}")
    x, y, theta = struct.unpack("<ddd", raw)
    return x, y, _rad2deg(theta)


def load_log(path: Path) -> dict[str, Any]:
    """Load a .wpilog file and return signal data."""
    from logreader.wpilog_reader import read_wpilog
    return read_wpilog(path)


def _find_cameras(log) -> list[str]:
    cameras = set()
    for name in log.signals:
        if "/botpose_wpiblue" in name and "limelight" in name:
            # Extract camera name from NT:/limelight-X/botpose_wpiblue
            parts = name.split("/")
            for p in parts:
                if p.startswith("limelight"):
                    cameras.add(p)
    return sorted(cameras)


def _build_odom_arrays(log) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
    """Build odometry arrays from DriveState/Pose or Field/Robot.

    Returns (timestamps_us, x, y, yaw_deg) numpy arrays, or None.
    """
    # Try DriveState/Pose (struct Pose2d)
    key = "NT:/DriveState/Pose"
    if key in log.signals:
        sig = log.signals[key]
        ts = []
        xs = []
        ys = []
        yaws = []
        for v in sig.values:
            try:
                x, y, yaw = _decode_pose2d(v.value)
                ts.append(v.timestamp_us)
                xs.append(x)
                ys.append(y)
                yaws.append(yaw)
            except (ValueError, struct.error):
                continue
        if ts:
            return np.array(ts), np.array(xs), np.array(ys), np.array(yaws)

    # Try Field/Robot (double[3]: x, y, yaw_deg)
    key = "NT:/SmartDashboard/Field/Robot"
    if key in log.signals:
        sig = log.signals[key]
        ts = []
        xs = []
        ys = []
        yaws = []
        for v in sig.values:
            arr = v.value
            if len(arr) >= 3:
                ts.append(v.timestamp_us)
                xs.append(arr[0])
                ys.append(arr[1])
                yaws.append(arr[2])
        if ts:
            return np.array(ts), np.array(xs), np.array(ys), np.array(yaws)

    return None


def _interp_odom(odom_ts: np.ndarray, odom_x: np.ndarray, odom_y: np.ndarray,
                 odom_yaw: np.ndarray, query_ts: int) -> tuple[float, float, float] | None:
    """Interpolate odometry at a given timestamp."""
    if query_ts < odom_ts[0] or query_ts > odom_ts[-1]:
        return None
    idx = np.searchsorted(odom_ts, query_ts)
    if idx == 0:
        return float(odom_x[0]), float(odom_y[0]), float(odom_yaw[0])
    if idx >= len(odom_ts):
        return float(odom_x[-1]), float(odom_y[-1]), float(odom_yaw[-1])
    # Linear interpolation
    t0, t1 = odom_ts[idx - 1], odom_ts[idx]
    if t1 == t0:
        return float(odom_x[idx]), float(odom_y[idx]), float(odom_yaw[idx])
    frac = (query_ts - t0) / (t1 - t0)
    x = odom_x[idx - 1] + frac * (odom_x[idx] - odom_x[idx - 1])
    y = odom_y[idx - 1] + frac * (odom_y[idx] - odom_y[idx - 1])
    # Yaw needs wrapping-aware interpolation
    y0 = odom_yaw[idx - 1]
    y1 = odom_yaw[idx]
    diff = _wrap_deg(y1 - y0)
    yaw = y0 + frac * diff
    return float(x), float(y), float(_wrap_deg(yaw))


def _build_ts_index(sig_values) -> tuple[np.ndarray, list]:
    """Pre-build a sorted timestamp array for fast nearest-neighbor lookup."""
    if not sig_values:
        return np.array([], dtype=np.int64), []
    ts = np.array([v.timestamp_us for v in sig_values], dtype=np.int64)
    return ts, sig_values


def _find_nearest(ts_arr: np.ndarray, values: list, query_ts: int,
                  max_dt_us: int = 50_000):
    """Find the value nearest to query_ts within max_dt_us using pre-built index."""
    if len(ts_arr) == 0:
        return None
    idx = np.searchsorted(ts_arr, query_ts)
    best = None
    best_dt = max_dt_us + 1
    for candidate in [idx - 1, idx]:
        if 0 <= candidate < len(ts_arr):
            dt = abs(int(ts_arr[candidate]) - query_ts)
            if dt < best_dt:
                best_dt = dt
                best = values[candidate]
    return best if best_dt <= max_dt_us else None


def collect_frames(log, camera: str, odom_data) -> list[CameraFrame]:
    """Collect calibration frames for one camera."""
    odom_ts, odom_x, odom_y, odom_yaw = odom_data

    bp_key = f"NT:/{camera}/botpose_wpiblue"
    rf_key = f"NT:/{camera}/rawfiducials"
    tpc_key = f"NT:/{camera}/targetpose_cameraspace"
    tid_key = f"NT:/{camera}/tid"

    if bp_key not in log.signals:
        return []

    bp_sig = log.signals[bp_key]
    rf_sig = log.signals.get(rf_key)
    tpc_sig = log.signals.get(tpc_key)
    tid_sig = log.signals.get(tid_key)

    rf_values = rf_sig.values if rf_sig else []
    tpc_values = tpc_sig.values if tpc_sig else []
    tid_values = tid_sig.values if tid_sig else []

    # Pre-build timestamp indices for fast lookup
    rf_ts, rf_vals = _build_ts_index(rf_values)
    tpc_ts, tpc_vals = _build_ts_index(tpc_values)
    tid_ts, tid_vals = _build_ts_index(tid_values)

    frames = []
    for v in bp_sig.values:
        arr = v.value
        if len(arr) < 11:
            continue
        tag_count = int(arr[7])
        if tag_count < 1:
            continue

        ts = v.timestamp_us
        total_latency_ms = arr[6]

        # Latency-compensate: shift query back by total latency
        capture_ts = ts - int(total_latency_ms * 1000)
        odom = _interp_odom(odom_ts, odom_x, odom_y, odom_yaw, capture_ts)
        if odom is None:
            continue

        frame = CameraFrame(
            timestamp_us=ts,
            camera=camera,
            vision_x=arr[0],
            vision_y=arr[1],
            vision_yaw_deg=arr[5],
            tag_count=tag_count,
            avg_tag_dist=arr[9],
            total_latency_ms=total_latency_ms,
            odom_x=odom[0],
            odom_y=odom[1],
            odom_yaw_deg=odom[2],
        )

        # Match rawfiducials
        rf_match = _find_nearest(rf_ts, rf_vals, ts, max_dt_us=30_000)
        if rf_match is not None:
            rf_arr = rf_match.value
            n_tags = len(rf_arr) // 7
            for t in range(n_tags):
                frame.tag_ids.append(int(rf_arr[t * 7]))
                frame.tag_ambiguities.append(rf_arr[t * 7 + 6])
                frame.tag_distances.append(rf_arr[t * 7 + 4])

        # Match targetpose_cameraspace
        tpc_match = _find_nearest(tpc_ts, tpc_vals, ts, max_dt_us=30_000)
        if tpc_match is not None and any(x != 0 for x in tpc_match.value):
            frame.tgt_cam_pose = list(tpc_match.value)

        # Match tid for primary tag ID
        tid_match = _find_nearest(tid_ts, tid_vals, ts, max_dt_us=100_000)
        if tid_match is not None:
            frame.primary_tag_id = int(tid_match.value)

        frames.append(frame)

    return frames


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

@dataclass
class ResidualStats:
    """Signed residual statistics."""
    n: int
    mean_dx: float
    mean_dy: float
    mean_dyaw: float
    std_dx: float
    std_dy: float
    std_dyaw: float
    median_dist: float


def compute_residual_stats(frames: list[CameraFrame]) -> ResidualStats | None:
    if not frames:
        return None
    dx = [f.vision_x - f.odom_x for f in frames]
    dy = [f.vision_y - f.odom_y for f in frames]
    dyaw = [_wrap_deg(f.vision_yaw_deg - f.odom_yaw_deg) for f in frames]
    dist = [math.sqrt(dx[i]**2 + dy[i]**2) for i in range(len(dx))]
    return ResidualStats(
        n=len(frames),
        mean_dx=float(np.mean(dx)),
        mean_dy=float(np.mean(dy)),
        mean_dyaw=float(np.mean(dyaw)),
        std_dx=float(np.std(dx)),
        std_dy=float(np.std(dy)),
        std_dyaw=float(np.std(dyaw)),
        median_dist=float(np.median(dist)),
    )


def fit_camera_pose_correction(
    frames: list[CameraFrame],
    current_cam_pose: list[float],
) -> tuple[list[float], dict]:
    """Estimate camera pose corrections using analytical linear least squares.

    Models how each camera pose parameter affects botpose_wpiblue residuals:

    For a camera at yaw_cam in robot frame, with robot at heading θ_r, seeing
    tags at distance d:

        residual_x ≈  δfwd·cos(θ_r) - δleft·sin(θ_r)
                     - d·δyaw·sin(θ_r + yaw_cam)
                     + d·δpitch·cos(θ_r + yaw_cam)

        residual_y ≈  δfwd·sin(θ_r) + δleft·cos(θ_r)
                     + d·δyaw·cos(θ_r + yaw_cam)
                     + d·δpitch·sin(θ_r + yaw_cam)

    This is linear in [δfwd, δleft, δyaw_rad, δpitch_rad] and solved
    via ordinary least squares.

    Parameters
    ----------
    frames : calibration frames with odometry
    current_cam_pose : [forward, left, up, roll, pitch, yaw] in m and degrees

    Returns
    -------
    corrected_pose : [forward, left, up, roll, pitch, yaw]
    info : dict with fit details
    """
    yaw_cam_rad = _deg2rad(current_cam_pose[5])

    # Use high-quality frames: multi-tag OR (close + low ambiguity)
    cal_frames = [
        f for f in frames
        if (f.tag_count >= 2)
        or (f.avg_tag_dist < 3.5
            and len(f.tag_ambiguities) > 0
            and max(f.tag_ambiguities) < 0.35)
    ]

    if len(cal_frames) < 30:
        return current_cam_pose, {"status": "insufficient_data", "n_frames": len(cal_frames)}

    # Build linear system: A @ [δfwd, δleft, δyaw_rad, δpitch_rad] = b
    n = len(cal_frames)
    A = np.zeros((2 * n, 4))
    b = np.zeros(2 * n)

    for i, f in enumerate(cal_frames):
        θ = _deg2rad(f.odom_yaw_deg)
        d = f.avg_tag_dist
        cam_field_angle = θ + yaw_cam_rad

        # Row for residual_x
        A[2*i, 0] = math.cos(θ)                    # δfwd
        A[2*i, 1] = -math.sin(θ)                   # δleft
        A[2*i, 2] = -d * math.sin(cam_field_angle)  # δyaw_rad
        A[2*i, 3] = d * math.cos(cam_field_angle)   # δpitch_rad
        b[2*i] = f.vision_x - f.odom_x

        # Row for residual_y
        A[2*i+1, 0] = math.sin(θ)                   # δfwd
        A[2*i+1, 1] = math.cos(θ)                   # δleft
        A[2*i+1, 2] = d * math.cos(cam_field_angle)  # δyaw_rad
        A[2*i+1, 3] = d * math.sin(cam_field_angle)  # δpitch_rad
        b[2*i+1] = f.vision_y - f.odom_y

    # Solve via least squares
    result, residuals_sum, rank, sv = np.linalg.lstsq(A, b, rcond=None)
    δfwd, δleft, δyaw_rad, δpitch_rad = result

    # Compute RMS of original vs corrected residuals
    original_rms = float(np.sqrt(np.mean(b**2)))
    corrected_residuals = b - A @ result
    corrected_rms = float(np.sqrt(np.mean(corrected_residuals**2)))

    # Convert to degrees
    δyaw_deg = _rad2deg(δyaw_rad)
    δpitch_deg = _rad2deg(δpitch_rad)

    # Build corrected pose
    corrected = list(current_cam_pose)
    corrected[0] += δfwd    # forward
    corrected[1] += δleft   # left
    # up (idx 2) and roll (idx 3) not fitted — leave unchanged
    corrected[4] += δpitch_deg  # pitch
    corrected[5] += δyaw_deg    # yaw

    info = {
        "status": "success",
        "n_frames": len(cal_frames),
        "rank": int(rank),
        "original_rms": original_rms,
        "corrected_rms": corrected_rms,
        "improvement_pct": (1 - corrected_rms / original_rms) * 100 if original_rms > 0 else 0,
        "delta_fwd": δfwd,
        "delta_left": δleft,
        "delta_yaw_deg": δyaw_deg,
        "delta_pitch_deg": δpitch_deg,
        "singular_values": sv.tolist(),
    }

    return corrected, info


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def analyze_camera(frames: list[CameraFrame], camera: str,
                   current_cam_pose: list[float]) -> None:
    """Run full calibration analysis for one camera."""
    print(f"\n{'=' * 70}")
    print(f"Camera: {camera}")
    print(f"  Current camerapose_robotspace: {current_cam_pose}")
    print(f"  (forward={current_cam_pose[0]:.3f}m, left={current_cam_pose[1]:.3f}m, "
          f"up={current_cam_pose[2]:.3f}m)")
    print(f"  (roll={current_cam_pose[3]:.1f}°, pitch={current_cam_pose[4]:.1f}°, "
          f"yaw={current_cam_pose[5]:.1f}°)")
    print(f"  Total valid frames: {len(frames)}")
    print(f"{'=' * 70}")

    if len(frames) < 10:
        print("  Insufficient data for analysis.")
        return

    # --- Overall residual stats ---
    stats = compute_residual_stats(frames)
    print(f"\n  Overall residual (vision - odometry):")
    print(f"    Mean ΔX: {stats.mean_dx:+.4f}m  (std: {stats.std_dx:.4f}m)")
    print(f"    Mean ΔY: {stats.mean_dy:+.4f}m  (std: {stats.std_dy:.4f}m)")
    print(f"    Mean ΔYaw: {stats.mean_dyaw:+.2f}°  (std: {stats.std_dyaw:.2f}°)")
    print(f"    Median 2D distance: {stats.median_dist:.4f}m")

    # --- By tag count ---
    print(f"\n  By tag count:")
    for tc in sorted(set(f.tag_count for f in frames)):
        tc_frames = [f for f in frames if f.tag_count == tc]
        tc_stats = compute_residual_stats(tc_frames)
        if tc_stats:
            print(f"    {tc} tag{'s' if tc > 1 else ' '}: n={tc_stats.n:5d}  "
                  f"ΔX={tc_stats.mean_dx:+.4f}  ΔY={tc_stats.mean_dy:+.4f}  "
                  f"ΔYaw={tc_stats.mean_dyaw:+.2f}°  "
                  f"median_dist={tc_stats.median_dist:.3f}m")

    # --- By tag ID (for single-tag frames) ---
    single_frames = [f for f in frames if f.tag_count == 1 and len(f.tag_ids) == 1]
    if single_frames:
        print(f"\n  By tag ID (single-tag frames only, n={len(single_frames)}):")
        tag_groups: dict[int, list[CameraFrame]] = {}
        for f in single_frames:
            tag_groups.setdefault(f.tag_ids[0], []).append(f)
        for tid in sorted(tag_groups.keys()):
            grp = tag_groups[tid]
            s = compute_residual_stats(grp)
            if s and s.n >= 3:
                mean_amb = np.mean([f.tag_ambiguities[0] for f in grp])
                mean_dist = np.mean([f.tag_distances[0] for f in grp])
                print(f"    Tag {tid:2d}: n={s.n:4d}  "
                      f"ΔX={s.mean_dx:+.4f}  ΔY={s.mean_dy:+.4f}  "
                      f"ΔYaw={s.mean_dyaw:+.2f}°  "
                      f"dist={mean_dist:.2f}m  amb={mean_amb:.3f}")

    # --- Multi-tag only stats (higher confidence) ---
    multi_frames = [f for f in frames if f.tag_count >= 2]
    if multi_frames:
        mt_stats = compute_residual_stats(multi_frames)
        print(f"\n  Multi-tag frames only (n={mt_stats.n}):")
        print(f"    Mean ΔX: {mt_stats.mean_dx:+.4f}m  (std: {mt_stats.std_dx:.4f}m)")
        print(f"    Mean ΔY: {mt_stats.mean_dy:+.4f}m  (std: {mt_stats.std_dy:.4f}m)")
        print(f"    Mean ΔYaw: {mt_stats.mean_dyaw:+.2f}°  (std: {mt_stats.std_dyaw:.2f}°)")
        print(f"    Median 2D distance: {mt_stats.median_dist:.4f}m")

    # --- Low-ambiguity only stats (highest confidence) ---
    low_amb = [f for f in frames
               if len(f.tag_ambiguities) > 0
               and max(f.tag_ambiguities) < 0.3
               and f.avg_tag_dist < 3.0]
    if low_amb:
        la_stats = compute_residual_stats(low_amb)
        print(f"\n  Low-ambiguity + close frames (amb<0.3, dist<3m, n={la_stats.n}):")
        print(f"    Mean ΔX: {la_stats.mean_dx:+.4f}m  (std: {la_stats.std_dx:.4f}m)")
        print(f"    Mean ΔY: {la_stats.mean_dy:+.4f}m  (std: {la_stats.std_dy:.4f}m)")
        print(f"    Mean ΔYaw: {la_stats.mean_dyaw:+.2f}°  (std: {la_stats.std_dyaw:.2f}°)")
        print(f"    Median 2D distance: {la_stats.median_dist:.4f}m")

    # --- Residual by robot heading quadrant ---
    print(f"\n  By robot heading quadrant:")
    quadrants = {
        "0°–90° (facing +X/+Y)": (0, 90),
        "90°–180° (facing -X/+Y)": (90, 180),
        "(-180)°–(-90)° (facing -X/-Y)": (-180, -90),
        "(-90)°–0° (facing +X/-Y)": (-90, 0),
    }
    for label, (lo, hi) in quadrants.items():
        q_frames = [f for f in frames if lo <= f.odom_yaw_deg < hi]
        if q_frames:
            q_stats = compute_residual_stats(q_frames)
            print(f"    {label}: n={q_stats.n:5d}  "
                  f"ΔX={q_stats.mean_dx:+.4f}  ΔY={q_stats.mean_dy:+.4f}  "
                  f"ΔYaw={q_stats.mean_dyaw:+.2f}°")

    # --- Analytical Camera Pose Fit ---
    print(f"\n  --- Camera Pose Correction (Linear Least Squares) ---")
    corrected, info = fit_camera_pose_correction(frames, current_cam_pose)

    if info["status"] == "insufficient_data":
        print(f"    Insufficient calibration data ({info['n_frames']} high-quality frames)")
        return

    print(f"    Calibration frames used: {info['n_frames']}")
    print(f"    Matrix rank: {info['rank']} (of 4)")
    print(f"    Original RMS residual: {info['original_rms']:.4f}m")
    print(f"    After correction RMS:  {info['corrected_rms']:.4f}m")
    print(f"    Improvement: {info['improvement_pct']:.1f}%")
    print()

    print(f"    Estimated corrections (config error → how to fix):")
    corrections = [
        ("forward", info["delta_fwd"], "m"),
        ("left", info["delta_left"], "m"),
        ("pitch", info["delta_pitch_deg"], "°"),
        ("yaw", info["delta_yaw_deg"], "°"),
    ]
    significant = False
    for label, delta, unit in corrections:
        threshold = 0.01 if unit == "m" else 0.5
        if abs(delta) > threshold:
            significant = True
            print(f"      {label:>8s}: {delta:+.4f}{unit}  "
                  f"(config is {'too low' if delta > 0 else 'too high'} "
                  f"by {abs(delta):.4f}{unit})")
        else:
            print(f"      {label:>8s}: {delta:+.4f}{unit}  (negligible)")

    print()
    print(f"    Current config:   [{', '.join(f'{current_cam_pose[i]:.6f}' for i in range(6))}]")
    print(f"    Suggested config: [{', '.join(f'{corrected[i]:.6f}' for i in range(6))}]")

    if not significant:
        print(f"\n    ✓ Camera pose appears well-calibrated (all corrections < threshold)")
    else:
        print(f"\n    ⚠ Corrections above suggest the camera pose may need adjustment.")
        print(f"      Note: corrections < 3cm / < 2° are within odometry noise floor.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: python camera_pose_calibration.py <log_dir_or_file> [max_files]")
        sys.exit(1)

    path = Path(sys.argv[1])
    max_files = int(sys.argv[2]) if len(sys.argv) > 2 else 10

    if path.is_file():
        log_files = [path]
    elif path.is_dir():
        log_files = sorted(path.glob("*.wpilog"))
        # Skip tiny files
        log_files = [f for f in log_files if f.stat().st_size > 1_000_000]
        if len(log_files) > max_files:
            # Pick the largest files (more data = better calibration)
            log_files.sort(key=lambda f: f.stat().st_size, reverse=True)
            log_files = log_files[:max_files]
            log_files.sort()  # Re-sort by name/time
    else:
        print(f"Path not found: {path}")
        sys.exit(1)

    print(f"Processing {len(log_files)} log file(s)...")
    print()

    # Collect all frames across all logs, per camera
    all_frames: dict[str, list[CameraFrame]] = {}
    camera_poses: dict[str, list[float]] = {}

    for log_path in log_files:
        print(f"Loading {log_path.name}...")
        try:
            log = load_log(log_path)
        except Exception as e:
            print(f"  Error loading: {e}")
            continue

        odom_data = _build_odom_arrays(log)
        if odom_data is None:
            print("  No odometry data found, skipping.")
            continue

        odom_ts, odom_x, odom_y, odom_yaw = odom_data
        odom_duration = (odom_ts[-1] - odom_ts[0]) / 1e6
        print(f"  Odometry: {len(odom_ts)} samples, {odom_duration:.1f}s")

        cameras = _find_cameras(log)
        print(f"  Cameras: {cameras}")

        for cam in cameras:
            # Get configured pose
            cp_key = f"NT:/{cam}/camerapose_robotspace"
            if cp_key in log.signals and log.signals[cp_key].values:
                cp = list(log.signals[cp_key].values[0].value)
                camera_poses[cam] = cp

            frames = collect_frames(log, cam, odom_data)
            print(f"  {cam}: {len(frames)} valid frames")
            all_frames.setdefault(cam, []).extend(frames)

    # --- Run analysis per camera ---
    for cam in sorted(all_frames.keys()):
        frames = all_frames[cam]
        cam_pose = camera_poses.get(cam, [0, 0, 0, 0, 0, 0])
        analyze_camera(frames, cam, cam_pose)

    # --- Cross-camera comparison ---
    print(f"\n{'=' * 70}")
    print("Cross-Camera Comparison")
    print(f"{'=' * 70}")
    for cam in sorted(all_frames.keys()):
        frames = all_frames[cam]
        mt = [f for f in frames if f.tag_count >= 2]
        stats = compute_residual_stats(mt) if mt else compute_residual_stats(frames)
        if stats:
            print(f"  {cam}: n={stats.n:5d}  "
                  f"bias: ΔX={stats.mean_dx:+.4f}m  ΔY={stats.mean_dy:+.4f}m  "
                  f"ΔYaw={stats.mean_dyaw:+.2f}°  "
                  f"median_err={stats.median_dist:.3f}m")

    # --- Summary of recommended changes ---
    print(f"\n{'=' * 70}")
    print("Summary of Camera Pose Configurations")
    print(f"{'=' * 70}")
    for cam in sorted(camera_poses.keys()):
        cp = camera_poses[cam]
        print(f"\n  {cam}:")
        print(f"    Current:  forward={cp[0]:.4f}m  left={cp[1]:.4f}m  up={cp[2]:.4f}m  "
              f"roll={cp[3]:.1f}°  pitch={cp[4]:.1f}°  yaw={cp[5]:.1f}°")


if __name__ == "__main__":
    main()
