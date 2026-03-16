"""Vision analysis plots — matplotlib-based visual diagnostics.

Generates PNG plots for vision analysis results. Requires matplotlib
(optional dependency: ``pip install logreader[plots]``).

All plots are 1920×1080 or smaller, saved as lossless PNGs.
"""

from __future__ import annotations

import math
import os
import statistics
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from logreader.analyzers.vision_analysis import (
        CameraAgreement,
        CameraSummary,
        DetectionGap,
        FieldCell,
        HardwareStats,
        PhaseBreakdown,
        VisionFrame,
    )
    from logreader.models import LogData, SignalData

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_FIG_W, _FIG_H = 19.2, 10.8  # 1920x1080 at 100 dpi
_DPI = 100

# FRC field dimensions (meters)
_FIELD_X_MIN, _FIELD_X_MAX = 0.0, 16.54
_FIELD_Y_MIN, _FIELD_Y_MAX = 0.0, 8.21
_FIELD_CELL_SIZE_M = 0.5

# Camera colors (consistent across plots)
_CAM_COLORS = {
    "limelight-a": "#2196F3",  # blue
    "limelight-b": "#FF9800",  # orange
    "limelight-c": "#4CAF50",  # green
    "limelight-d": "#E91E63",  # pink
}


def _cam_color(name: str) -> str:
    return _CAM_COLORS.get(name, "#9E9E9E")


def _ensure_dir(output_dir: str) -> Path:
    p = Path(output_dir)
    p.mkdir(parents=True, exist_ok=True)
    return p


def _savefig(fig: object, path: Path) -> None:
    """Save figure and close it."""
    fig.savefig(str(path), dpi=_DPI, bbox_inches="tight", pad_inches=0.3)  # type: ignore[attr-defined]
    import matplotlib.pyplot as plt

    plt.close(fig)  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# 1. Vision Availability Heatmap
# ---------------------------------------------------------------------------


def plot_availability_heatmap(
    heatmap_cells: list[FieldCell],
    output_dir: str,
    camera: str | None = None,
    title_suffix: str = "",
) -> str | None:
    """Field heatmap colored by vision detection rate (availability)."""
    import matplotlib.pyplot as plt
    import numpy as np

    if not heatmap_cells:
        return None

    n_cols = int((_FIELD_X_MAX - _FIELD_X_MIN) / _FIELD_CELL_SIZE_M) + 1
    n_rows = int((_FIELD_Y_MAX - _FIELD_Y_MIN) / _FIELD_CELL_SIZE_M) + 1

    grid = np.full((n_rows, n_cols), float("nan"))
    for cell in heatmap_cells:
        if cell.total_samples > 0:
            grid[cell.row, cell.col] = cell.availability * 100.0

    fig, ax = plt.subplots(figsize=(_FIG_W, _FIG_H))
    im = ax.imshow(
        grid,
        origin="lower",
        extent=[_FIELD_X_MIN, _FIELD_X_MAX, _FIELD_Y_MIN, _FIELD_Y_MAX],
        aspect="equal",
        cmap="RdYlGn",
        vmin=0,
        vmax=100,
        interpolation="nearest",
    )
    cbar = fig.colorbar(im, ax=ax, shrink=0.7, label="Valid frame rate (%)")
    cam_label = camera or "all cameras"
    ax.set_title(f"Vision Availability — {cam_label}{title_suffix}", fontsize=14)
    ax.set_xlabel("Field X (m)")
    ax.set_ylabel("Field Y (m)")
    ax.set_xlim(_FIELD_X_MIN, _FIELD_X_MAX)
    ax.set_ylim(_FIELD_Y_MIN, _FIELD_Y_MAX)

    fname = f"availability_{'all' if camera is None else camera.replace('-', '_')}.png"
    out = _ensure_dir(output_dir) / fname
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 2. Pose Accuracy / Residual Heatmap
# ---------------------------------------------------------------------------


def plot_residual_heatmap(
    heatmap_cells: list[FieldCell],
    output_dir: str,
    camera: str | None = None,
    title_suffix: str = "",
) -> str | None:
    """Field heatmap colored by median pose residual."""
    import matplotlib.pyplot as plt
    import numpy as np

    if not heatmap_cells:
        return None

    n_cols = int((_FIELD_X_MAX - _FIELD_X_MIN) / _FIELD_CELL_SIZE_M) + 1
    n_rows = int((_FIELD_Y_MAX - _FIELD_Y_MIN) / _FIELD_CELL_SIZE_M) + 1

    grid = np.full((n_rows, n_cols), float("nan"))
    for cell in heatmap_cells:
        if cell.valid_samples > 0:
            grid[cell.row, cell.col] = cell.median_residual_m

    fig, ax = plt.subplots(figsize=(_FIG_W, _FIG_H))
    im = ax.imshow(
        grid,
        origin="lower",
        extent=[_FIELD_X_MIN, _FIELD_X_MAX, _FIELD_Y_MIN, _FIELD_Y_MAX],
        aspect="equal",
        cmap="RdYlGn_r",
        vmin=0,
        vmax=1.0,
        interpolation="nearest",
    )
    fig.colorbar(im, ax=ax, shrink=0.7, label="Median residual (m)")
    cam_label = camera or "all cameras"
    ax.set_title(f"Pose Residual — {cam_label}{title_suffix}", fontsize=14)
    ax.set_xlabel("Field X (m)")
    ax.set_ylabel("Field Y (m)")
    ax.set_xlim(_FIELD_X_MIN, _FIELD_X_MAX)
    ax.set_ylim(_FIELD_Y_MIN, _FIELD_Y_MAX)

    fname = f"residual_heatmap_{'all' if camera is None else camera.replace('-', '_')}.png"
    out = _ensure_dir(output_dir) / fname
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 3. Ambiguity vs Distance Scatter
# ---------------------------------------------------------------------------


def plot_ambiguity_vs_distance(
    frames: list[VisionFrame],
    output_dir: str,
    camera: str | None = None,
    title_suffix: str = "",
) -> str | None:
    """Scatter plot: per-tag ambiguity (Y) vs distance (X), colored by tag ID."""
    import matplotlib.pyplot as plt
    import numpy as np

    cam_frames = frames if camera is None else [
        f for f in frames if f.camera == camera
    ]

    distances: list[float] = []
    ambiguities: list[float] = []
    tag_ids: list[int] = []
    for f in cam_frames:
        for tag in f.tags:
            distances.append(tag.dist_to_camera)
            ambiguities.append(tag.ambiguity)
            tag_ids.append(tag.tag_id)

    if not distances:
        return None

    fig, ax = plt.subplots(figsize=(_FIG_W, _FIG_H))
    scatter = ax.scatter(
        distances,
        ambiguities,
        c=tag_ids,
        cmap="tab20",
        alpha=0.4,
        s=8,
        edgecolors="none",
    )
    fig.colorbar(scatter, ax=ax, shrink=0.7, label="Tag ID")

    # Trust cutoff line
    ax.axhline(y=0.5, color="red", linestyle="--", linewidth=1.5, alpha=0.7, label="Ambiguity = 0.5")
    ax.legend(fontsize=10)

    cam_label = camera or "all cameras"
    ax.set_title(f"Ambiguity vs Distance — {cam_label}{title_suffix}", fontsize=14)
    ax.set_xlabel("Camera-to-tag distance (m)")
    ax.set_ylabel("Ambiguity")
    ax.set_xlim(0, 8)
    ax.set_ylim(-0.02, 1.02)

    fname = f"ambiguity_vs_distance_{'all' if camera is None else camera.replace('-', '_')}.png"
    out = _ensure_dir(output_dir) / fname
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 4. Tag Count Over Time
# ---------------------------------------------------------------------------


def plot_tag_count_over_time(
    frames: list[VisionFrame],
    camera_names: list[str],
    output_dir: str,
    log_data: LogData | None = None,
    title_suffix: str = "",
) -> str | None:
    """Time-series of tag count per camera with phase boundaries."""
    import matplotlib.pyplot as plt

    if not frames:
        return None

    # Determine time origin
    min_ts = min(f.timestamp_us for f in frames)

    fig, ax = plt.subplots(figsize=(_FIG_W, _FIG_H))

    for cam in sorted(camera_names):
        cam_frames = sorted(
            [f for f in frames if f.camera == cam],
            key=lambda f: f.timestamp_us,
        )
        if not cam_frames:
            continue
        times_s = [(f.timestamp_us - min_ts) / 1e6 for f in cam_frames]
        counts = [f.tag_count for f in cam_frames]
        ax.scatter(times_s, counts, s=3, alpha=0.4, label=cam, color=_cam_color(cam))

    # Phase boundaries
    _add_phase_lines(ax, log_data, min_ts)

    ax.set_title(f"Tag Count Over Time{title_suffix}", fontsize=14)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Tag count")
    ax.set_ylim(-0.5, max(f.tag_count for f in frames) + 1)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    out = _ensure_dir(output_dir) / "tag_count_over_time.png"
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 5. Pose Residual Over Time
# ---------------------------------------------------------------------------


def plot_residual_over_time(
    frames: list[VisionFrame],
    camera_names: list[str],
    output_dir: str,
    log_data: LogData | None = None,
    title_suffix: str = "",
) -> str | None:
    """Scatter of pose residual per frame with threshold lines."""
    import matplotlib.pyplot as plt

    valid = [f for f in frames if f.pose_residual_m is not None]
    if not valid:
        return None

    min_ts = min(f.timestamp_us for f in frames)

    fig, ax = plt.subplots(figsize=(_FIG_W, _FIG_H))

    for cam in sorted(camera_names):
        cam_frames = sorted(
            [f for f in valid if f.camera == cam],
            key=lambda f: f.timestamp_us,
        )
        if not cam_frames:
            continue
        times_s = [(f.timestamp_us - min_ts) / 1e6 for f in cam_frames]
        residuals = [f.pose_residual_m for f in cam_frames]
        ax.scatter(times_s, residuals, s=4, alpha=0.4, label=cam, color=_cam_color(cam))

    ax.axhline(y=0.5, color="orange", linestyle="--", linewidth=1, alpha=0.7, label="0.5m threshold")
    ax.axhline(y=1.0, color="red", linestyle="--", linewidth=1, alpha=0.7, label="1.0m threshold")

    _add_phase_lines(ax, log_data, min_ts)

    ax.set_title(f"Pose Residual Over Time{title_suffix}", fontsize=14)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Residual (m)")
    ax.set_ylim(-0.05, min(max(f.pose_residual_m for f in valid) * 1.1, 10.0))
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    out = _ensure_dir(output_dir) / "residual_over_time.png"
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 6. Latency Over Time
# ---------------------------------------------------------------------------


def plot_latency_over_time(
    frames: list[VisionFrame],
    camera_names: list[str],
    output_dir: str,
    log_data: LogData | None = None,
    title_suffix: str = "",
) -> str | None:
    """Time-series of total, pipeline, and capture latency."""
    import matplotlib.pyplot as plt

    if not frames:
        return None

    min_ts = min(f.timestamp_us for f in frames)

    fig, ax = plt.subplots(figsize=(_FIG_W, _FIG_H))

    for cam in sorted(camera_names):
        cam_frames = sorted(
            [f for f in frames if f.camera == cam],
            key=lambda f: f.timestamp_us,
        )
        if not cam_frames:
            continue
        times_s = [(f.timestamp_us - min_ts) / 1e6 for f in cam_frames]
        total_lat = [f.total_latency_ms for f in cam_frames]
        ax.plot(times_s, total_lat, linewidth=0.5, alpha=0.6, label=f"{cam} total", color=_cam_color(cam))

    ax.axhline(y=50, color="red", linestyle="--", linewidth=1, alpha=0.5, label="50ms threshold")

    _add_phase_lines(ax, log_data, min_ts)

    ax.set_title(f"Latency Over Time{title_suffix}", fontsize=14)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Latency (ms)")
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    out = _ensure_dir(output_dir) / "latency_over_time.png"
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 7. FPS and Thermal Over Time
# ---------------------------------------------------------------------------


def plot_fps_thermal(
    camera_signals: dict[str, dict[str, SignalData | None]],
    output_dir: str,
    title_suffix: str = "",
) -> str | None:
    """Dual-axis: FPS (left) and temperature (right) over time."""
    import matplotlib.pyplot as plt

    has_data = False
    fig, ax1 = plt.subplots(figsize=(_FIG_W, _FIG_H))
    ax2 = ax1.twinx()

    min_ts: int | None = None

    for cam_name, signals in sorted(camera_signals.items()):
        hw_sig = signals.get("hw")
        if not hw_sig or not hw_sig.values:
            continue

        if min_ts is None:
            min_ts = hw_sig.values[0].timestamp_us
        else:
            min_ts = min(min_ts, hw_sig.values[0].timestamp_us)

    if min_ts is None:
        import matplotlib.pyplot as plt
        plt.close(fig)
        return None

    for cam_name, signals in sorted(camera_signals.items()):
        hw_sig = signals.get("hw")
        if not hw_sig or not hw_sig.values:
            continue

        times_s: list[float] = []
        fps_vals: list[float] = []
        cpu_temps: list[float] = []

        for v in hw_sig.values:
            arr = v.value
            if not isinstance(arr, (list, tuple)) or len(arr) < 4:
                continue
            times_s.append((v.timestamp_us - min_ts) / 1e6)
            fps_vals.append(float(arr[0]))
            cpu_temps.append(float(arr[1]))

        if not times_s:
            continue

        has_data = True
        color = _cam_color(cam_name)
        ax1.plot(times_s, fps_vals, linewidth=1, alpha=0.7, label=f"{cam_name} FPS", color=color)
        ax2.plot(times_s, cpu_temps, linewidth=1, alpha=0.5, linestyle="--", label=f"{cam_name} CPU °C", color=color)

    if not has_data:
        import matplotlib.pyplot as plt
        plt.close(fig)
        return None

    ax1.axhline(y=30, color="red", linestyle=":", linewidth=1, alpha=0.5, label="30 FPS min")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("FPS")
    ax2.set_ylabel("CPU Temperature (°C)")
    ax1.set_title(f"FPS & Thermal Over Time{title_suffix}", fontsize=14)

    # Combine legends
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, fontsize=9, loc="upper left")
    ax1.grid(True, alpha=0.3)

    out = _ensure_dir(output_dir) / "fps_thermal.png"
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 8. Detection Gaps Timeline
# ---------------------------------------------------------------------------


def plot_detection_gaps(
    gaps: list[DetectionGap],
    frames: list[VisionFrame],
    output_dir: str,
    log_data: LogData | None = None,
    title_suffix: str = "",
) -> str | None:
    """Timeline showing detection gaps per camera."""
    import matplotlib.pyplot as plt

    if not gaps:
        return None

    min_ts = min(f.timestamp_us for f in frames) if frames else 0

    cameras = sorted(set(g.camera for g in gaps))
    cam_y = {cam: i for i, cam in enumerate(cameras)}

    fig, ax = plt.subplots(figsize=(_FIG_W, _FIG_H))

    for gap in gaps:
        y = cam_y[gap.camera]
        start_s = (gap.start_us - min_ts) / 1e6
        dur_s = gap.duration_ms / 1000.0
        ax.barh(
            y,
            dur_s,
            left=start_s,
            height=0.6,
            color=_cam_color(gap.camera),
            alpha=0.6,
        )

    _add_phase_lines(ax, log_data, min_ts)

    ax.set_yticks(list(cam_y.values()))
    ax.set_yticklabels(list(cam_y.keys()))
    ax.set_xlabel("Time (s)")
    ax.set_title(f"Detection Gaps (>500ms){title_suffix}", fontsize=14)
    ax.grid(True, alpha=0.3, axis="x")

    out = _ensure_dir(output_dir) / "detection_gaps.png"
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 9. Camera Agreement Over Time
# ---------------------------------------------------------------------------


def plot_camera_agreement_over_time(
    frames: list[VisionFrame],
    camera_names: list[str],
    output_dir: str,
    log_data: LogData | None = None,
    title_suffix: str = "",
) -> str | None:
    """Time-series of inter-camera pose disagreement."""
    import matplotlib.pyplot as plt

    if len(camera_names) < 2:
        return None

    # Group frames by camera
    from logreader.analyzers.vision_analysis import _CAMERA_AGREE_WINDOW_US

    by_camera: dict[str, list[VisionFrame]] = {}
    for f in frames:
        by_camera.setdefault(f.camera, []).append(f)
    for cam in by_camera:
        by_camera[cam].sort(key=lambda f: f.timestamp_us)

    sorted_cams = sorted(camera_names)
    if len(sorted_cams) < 2:
        return None

    cam_a, cam_b = sorted_cams[0], sorted_cams[1]
    frames_a = by_camera.get(cam_a, [])
    frames_b = by_camera.get(cam_b, [])

    if not frames_a or not frames_b:
        return None

    min_ts = min(f.timestamp_us for f in frames)

    times_s: list[float] = []
    disagreements: list[float] = []
    yaw_diffs: list[float] = []

    b_idx = 0
    for fa in frames_a:
        while b_idx < len(frames_b) - 1 and frames_b[b_idx + 1].timestamp_us <= fa.timestamp_us:
            b_idx += 1

        best_fb = None
        best_delta = _CAMERA_AGREE_WINDOW_US + 1
        for ci in (b_idx, b_idx + 1):
            if 0 <= ci < len(frames_b):
                delta = abs(frames_b[ci].timestamp_us - fa.timestamp_us)
                if delta < best_delta:
                    best_delta = delta
                    best_fb = frames_b[ci]

        if best_fb is None or best_delta > _CAMERA_AGREE_WINDOW_US:
            continue

        dx = fa.x - best_fb.x
        dy = fa.y - best_fb.y
        dist = math.sqrt(dx * dx + dy * dy)
        times_s.append((fa.timestamp_us - min_ts) / 1e6)
        disagreements.append(dist)

        yd = (fa.yaw_deg - best_fb.yaw_deg + 180.0) % 360.0 - 180.0
        yaw_diffs.append(abs(yd))

    if not times_s:
        return None

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(_FIG_W, _FIG_H), sharex=True)

    ax1.scatter(times_s, disagreements, s=4, alpha=0.4, color="#2196F3")
    ax1.axhline(y=0.5, color="orange", linestyle="--", linewidth=1, alpha=0.6)
    ax1.set_ylabel("Translation disagreement (m)")
    ax1.set_title(f"Camera Agreement: {cam_a} vs {cam_b}{title_suffix}", fontsize=14)
    ax1.grid(True, alpha=0.3)

    ax2.scatter(times_s, yaw_diffs, s=4, alpha=0.4, color="#FF9800")
    ax2.set_ylabel("Yaw disagreement (°)")
    ax2.set_xlabel("Time (s)")
    ax2.grid(True, alpha=0.3)

    _add_phase_lines(ax1, log_data, min_ts)
    _add_phase_lines(ax2, log_data, min_ts)

    out = _ensure_dir(output_dir) / "camera_agreement_over_time.png"
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 10. Preferred Camera Spatial Map
# ---------------------------------------------------------------------------


def plot_preferred_camera_map(
    preferred_data: list[tuple[float, float, str, float]],
    camera_names: list[str],
    output_dir: str,
    title_suffix: str = "",
) -> str | None:
    """Field map colored by which camera has lower residual per cell."""
    import matplotlib.pyplot as plt
    import numpy as np

    if not preferred_data or len(camera_names) < 2:
        return None

    sorted_cams = sorted(camera_names)

    fig, ax = plt.subplots(figsize=(_FIG_W, _FIG_H))

    for x, y, cam, adv in preferred_data:
        color = _cam_color(cam)
        ax.scatter(x, y, s=60, c=color, alpha=min(0.3 + adv * 2, 0.9), marker="s", edgecolors="none")

    # Legend
    for cam in sorted_cams:
        ax.scatter([], [], c=_cam_color(cam), label=cam, s=60, marker="s")
    ax.legend(fontsize=11)

    ax.set_xlim(_FIELD_X_MIN, _FIELD_X_MAX)
    ax.set_ylim(_FIELD_Y_MIN, _FIELD_Y_MAX)
    ax.set_aspect("equal")
    ax.set_title(f"Preferred Camera by Field Position{title_suffix}", fontsize=14)
    ax.set_xlabel("Field X (m)")
    ax.set_ylabel("Field Y (m)")
    ax.grid(True, alpha=0.2)

    out = _ensure_dir(output_dir) / "preferred_camera_map.png"
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 11. Residual vs Robot Speed
# ---------------------------------------------------------------------------


def plot_residual_vs_speed(
    speed_data: list[tuple[float, float, str]],
    output_dir: str,
    title_suffix: str = "",
) -> str | None:
    """Scatter: pose residual (Y) vs robot speed (X), per camera."""
    import matplotlib.pyplot as plt

    if not speed_data:
        return None

    fig, ax = plt.subplots(figsize=(_FIG_W, _FIG_H))

    by_cam: dict[str, tuple[list[float], list[float]]] = {}
    for speed, resid, cam in speed_data:
        by_cam.setdefault(cam, ([], []))
        by_cam[cam][0].append(speed)
        by_cam[cam][1].append(resid)

    for cam in sorted(by_cam):
        speeds, residuals = by_cam[cam]
        ax.scatter(speeds, residuals, s=4, alpha=0.3, label=cam, color=_cam_color(cam))

    ax.set_title(f"Residual vs Robot Speed{title_suffix}", fontsize=14)
    ax.set_xlabel("Robot speed (m/s)")
    ax.set_ylabel("Pose residual (m)")
    ax.set_xlim(0, min(max(s for s, _, _ in speed_data) * 1.1, 8.5))
    ax.set_ylim(0, min(max(r for _, r, _ in speed_data) * 1.1, 5.0))
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    out = _ensure_dir(output_dir) / "residual_vs_speed.png"
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 12. MT1 vs MT2 Divergence Heatmap
# ---------------------------------------------------------------------------


def plot_mt1_mt2_heatmap(
    frames: list[VisionFrame],
    log_data: LogData,
    output_dir: str,
    title_suffix: str = "",
) -> str | None:
    """Field heatmap of MegaTag1 vs MegaTag2 pose divergence."""
    import matplotlib.pyplot as plt
    import numpy as np

    divg_frames = [f for f in frames if f.mt1_mt2_divergence_m is not None]
    if not divg_frames:
        return None

    n_cols = int((_FIELD_X_MAX - _FIELD_X_MIN) / _FIELD_CELL_SIZE_M) + 1
    n_rows = int((_FIELD_Y_MAX - _FIELD_Y_MIN) / _FIELD_CELL_SIZE_M) + 1

    grid_sum = np.zeros((n_rows, n_cols))
    grid_count = np.zeros((n_rows, n_cols))

    for f in divg_frames:
        col = int((f.x - _FIELD_X_MIN) / _FIELD_CELL_SIZE_M)
        row = int((f.y - _FIELD_Y_MIN) / _FIELD_CELL_SIZE_M)
        if 0 <= row < n_rows and 0 <= col < n_cols:
            grid_sum[row, col] += f.mt1_mt2_divergence_m
            grid_count[row, col] += 1

    grid = np.full((n_rows, n_cols), float("nan"))
    mask = grid_count > 0
    grid[mask] = grid_sum[mask] / grid_count[mask]

    fig, ax = plt.subplots(figsize=(_FIG_W, _FIG_H))
    im = ax.imshow(
        grid,
        origin="lower",
        extent=[_FIELD_X_MIN, _FIELD_X_MAX, _FIELD_Y_MIN, _FIELD_Y_MAX],
        aspect="equal",
        cmap="YlOrRd",
        vmin=0,
        vmax=2.0,
        interpolation="nearest",
    )
    fig.colorbar(im, ax=ax, shrink=0.7, label="Mean MT1-MT2 divergence (m)")
    ax.set_title(f"MegaTag1 vs MegaTag2 Divergence{title_suffix}", fontsize=14)
    ax.set_xlabel("Field X (m)")
    ax.set_ylabel("Field Y (m)")
    ax.set_xlim(_FIELD_X_MIN, _FIELD_X_MAX)
    ax.set_ylim(_FIELD_Y_MIN, _FIELD_Y_MAX)

    out = _ensure_dir(output_dir) / "mt1_mt2_divergence_heatmap.png"
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# 13. Heading-Conditioned Availability (Polar Plot)
# ---------------------------------------------------------------------------


def plot_heading_coverage(
    heading_data: dict[str, dict[str, int]],
    output_dir: str,
    title_suffix: str = "",
) -> str | None:
    """Polar bar chart of valid frames per heading octant, per camera."""
    import matplotlib.pyplot as plt
    import numpy as np

    if not heading_data:
        return None

    octant_labels = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    angles = np.linspace(0, 2 * np.pi, len(octant_labels), endpoint=False)
    width = 2 * np.pi / len(octant_labels) * 0.8

    fig, ax = plt.subplots(figsize=(_FIG_W, _FIG_H), subplot_kw={"projection": "polar"})

    n_cams = len(heading_data)
    bar_width = width / max(n_cams, 1)

    for idx, (cam, counts) in enumerate(sorted(heading_data.items())):
        values = [counts.get(label, 0) for label in octant_labels]
        offset = (idx - n_cams / 2 + 0.5) * bar_width
        ax.bar(
            angles + offset,
            values,
            width=bar_width,
            alpha=0.6,
            label=cam,
            color=_cam_color(cam),
        )

    ax.set_xticks(angles)
    ax.set_xticklabels(octant_labels)
    ax.set_title(f"Detection Count by Robot Heading{title_suffix}", fontsize=14, y=1.08)
    ax.legend(fontsize=10, loc="upper right", bbox_to_anchor=(1.3, 1.0))

    out = _ensure_dir(output_dir) / "heading_coverage.png"
    _savefig(fig, out)
    return str(out)


# ---------------------------------------------------------------------------
# Phase line helper
# ---------------------------------------------------------------------------


def _add_phase_lines(
    ax: object,
    log_data: LogData | None,
    min_ts_us: int,
) -> None:
    """Add vertical lines for match phase transitions."""
    if log_data is None:
        return

    try:
        from logreader.analyzers.match_phases import detect_match_phases
    except ImportError:
        return

    timeline = detect_match_phases(log_data)
    if timeline is None or not timeline.has_match:
        return

    colors = {"Auto": "#4CAF50", "Teleop": "#2196F3", "Disabled": "#9E9E9E"}
    phase_map = {"AUTONOMOUS": "Auto", "TELEOP": "Teleop", "DISABLED": "Disabled"}

    for iv in timeline.intervals:
        phase_name = phase_map.get(iv.phase.name, iv.phase.name)
        t_s = (iv.start_us - min_ts_us) / 1e6
        color = colors.get(phase_name, "#9E9E9E")
        ax.axvline(x=t_s, color=color, linestyle=":", linewidth=1, alpha=0.6)  # type: ignore[attr-defined]
        ax.text(  # type: ignore[attr-defined]
            t_s,
            ax.get_ylim()[1] * 0.95,  # type: ignore[attr-defined]
            f" {phase_name}",
            fontsize=8,
            color=color,
            alpha=0.8,
            verticalalignment="top",
        )


# ---------------------------------------------------------------------------
# Master generate function
# ---------------------------------------------------------------------------


def generate_all_plots(
    frames: list[VisionFrame],
    camera_names: list[str],
    camera_signals: dict[str, dict[str, SignalData | None]],
    heatmap_cells: list[FieldCell],
    per_camera_heatmaps: dict[str, list[FieldCell]],
    detection_gaps: list[DetectionGap],
    preferred_camera_data: list[tuple[float, float, str, float]],
    heading_data: dict[str, dict[str, int]],
    speed_data: list[tuple[float, float, str]],
    log_data: LogData,
    output_dir: str,
    title_suffix: str = "",
) -> list[str]:
    """Generate all vision analysis plots and return list of output paths."""
    generated: list[str] = []

    def _add(path: str | None) -> None:
        if path:
            generated.append(path)

    # Spatial heatmaps
    _add(plot_availability_heatmap(heatmap_cells, output_dir, title_suffix=title_suffix))
    _add(plot_residual_heatmap(heatmap_cells, output_dir, title_suffix=title_suffix))
    for cam in sorted(camera_names):
        cam_cells = per_camera_heatmaps.get(cam, [])
        _add(plot_availability_heatmap(cam_cells, output_dir, camera=cam, title_suffix=title_suffix))
        _add(plot_residual_heatmap(cam_cells, output_dir, camera=cam, title_suffix=title_suffix))

    # Per-camera and combined scatter plots
    _add(plot_ambiguity_vs_distance(frames, output_dir, title_suffix=title_suffix))
    for cam in sorted(camera_names):
        _add(plot_ambiguity_vs_distance(frames, output_dir, camera=cam, title_suffix=title_suffix))

    # Temporal plots
    _add(plot_tag_count_over_time(frames, camera_names, output_dir, log_data, title_suffix))
    _add(plot_residual_over_time(frames, camera_names, output_dir, log_data, title_suffix))
    _add(plot_latency_over_time(frames, camera_names, output_dir, log_data, title_suffix))
    _add(plot_fps_thermal(camera_signals, output_dir, title_suffix))
    _add(plot_detection_gaps(detection_gaps, frames, output_dir, log_data, title_suffix))
    _add(plot_camera_agreement_over_time(frames, camera_names, output_dir, log_data, title_suffix))

    # Tier 2/3 specialized plots
    _add(plot_mt1_mt2_heatmap(frames, log_data, output_dir, title_suffix))
    _add(plot_preferred_camera_map(preferred_camera_data, camera_names, output_dir, title_suffix))
    _add(plot_residual_vs_speed(speed_data, output_dir, title_suffix))
    _add(plot_heading_coverage(heading_data, output_dir, title_suffix))

    return generated
