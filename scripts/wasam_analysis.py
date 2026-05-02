"""WASAM 2026 event analysis — pose accuracy and vision investigation.

Processes all match logs from D:\\Temp\\2026_WASAM to investigate:
1. Overall pose accuracy per match and per phase (auto/teleop)
2. Limelight tracking quality (residuals, ambiguity, tag counts, gaps)
3. Cross-match trends to identify software/hardware changes
4. Instances of exceptionally good or poor performance
"""

from __future__ import annotations

import math
import os
import re
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

# Force UTF-8 output on Windows
if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')
if sys.stderr.encoding != 'utf-8':
    sys.stderr.reconfigure(encoding='utf-8')

# Add src to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from logreader.wpilog_reader import read_wpilog
from logreader.models import LogData, SignalType
from logreader.analyzers.pose_analysis import (
    PoseSample,
    PoseSource,
    SourceMetrics,
    discover_pose_sources,
    build_reference_path,
    interpolate_pose_at,
    compute_divergence_metrics,
    find_divergence_events,
)
from logreader.analyzers.match_phases import (
    MatchPhase,
    MatchPhaseTimeline,
    detect_match_phases,
)
from logreader.analyzers.vision_analysis import (
    VisionFrame,
    TagDetection,
    CameraSummary,
    DetectionGap,
    _discover_cameras,
    _parse_frames,
    _compute_residuals,
    _find_detection_gaps,
    _camera_summary,
    _tag_count_table,
    _tag_id_table,
    _distance_band_table,
    _compute_camera_agreement,
    _compute_mt1_mt2_divergence,
    _summarize_mt1_mt2,
    _compute_hw_stats,
    DEFAULT_OUTLIER_M,
)


LOG_DIR = Path(r"D:\Temp\2026_WASAM")


@dataclass
class MatchResult:
    """Collected analysis results for one match."""
    filename: str
    match_label: str
    timestamp: str  # from filename e.g. "174035"
    has_match_phases: bool
    auto_duration_s: float
    teleop_duration_s: float
    total_duration_s: float

    # Pose sources
    odom_sources: int
    vision_sources: int
    ref_path_samples: int

    # Per-camera vision metrics
    cameras: list[str] = field(default_factory=list)
    per_camera: dict = field(default_factory=dict)

    # Phase-segmented vision metrics
    auto_vision: dict = field(default_factory=dict)
    teleop_vision: dict = field(default_factory=dict)

    # Pose divergence metrics
    pose_metrics: list = field(default_factory=list)
    divergence_events: list = field(default_factory=list)

    # Detection gaps
    detection_gaps: list = field(default_factory=list)

    # Hardware stats
    hw_stats: dict = field(default_factory=dict)

    # MT1 vs MT2
    mt1_mt2: dict = field(default_factory=dict)

    # Camera agreement
    camera_agreement: list = field(default_factory=list)

    # Tag count distribution
    tag_count_bands: list = field(default_factory=list)

    # Tag-level summaries
    tag_summaries: list = field(default_factory=list)

    # Distance band summaries
    distance_bands: list = field(default_factory=list)

    # Raw frames for deeper analysis
    all_frames: list = field(default_factory=list, repr=False)
    ref_path: list = field(default_factory=list, repr=False)
    timeline: MatchPhaseTimeline | None = None


def extract_match_label(filename: str) -> str:
    """Extract match label like 'Q4' from filename."""
    m = re.search(r"WASAM_(Q\d+)", filename)
    if m:
        return m.group(1)
    # Non-match files (practice/test)
    m2 = re.search(r"FRC_\d{8}_(\d{6})", filename)
    if m2:
        return f"Practice_{m2.group(1)}"
    return filename


def extract_timestamp(filename: str) -> str:
    """Extract time portion from filename."""
    m = re.search(r"FRC_\d{8}_(\d{6})", filename)
    return m.group(1) if m else ""


def compute_phase_vision_stats(
    frames: list[VisionFrame],
    timeline: MatchPhaseTimeline,
    phase: MatchPhase,
    cameras: list[str],
) -> dict:
    """Compute vision stats for frames in a specific match phase."""
    intervals = timeline.intervals_for(phase)
    if not intervals:
        return {}

    phase_frames = []
    for f in frames:
        for iv in intervals:
            if iv.contains_us(f.timestamp_us):
                phase_frames.append(f)
                break

    if not phase_frames:
        return {"total_frames": 0}

    result = {"total_frames": len(phase_frames)}

    for cam in cameras:
        cam_frames = [f for f in phase_frames if f.camera == cam]
        if not cam_frames:
            continue

        residuals = [f.pose_residual_m for f in cam_frames if f.pose_residual_m is not None]
        tag_counts = [f.tag_count for f in cam_frames]
        ambiguities = [f.max_ambiguity for f in cam_frames if f.tags]
        latencies = [f.total_latency_ms for f in cam_frames]

        cam_stats = {
            "valid_frames": len(cam_frames),
            "mean_tag_count": statistics.mean(tag_counts) if tag_counts else 0,
            "mean_latency_ms": statistics.mean(latencies) if latencies else 0,
        }

        if residuals:
            cam_stats["mean_residual_m"] = statistics.mean(residuals)
            cam_stats["median_residual_m"] = statistics.median(residuals)
            cam_stats["p95_residual_m"] = sorted(residuals)[int(len(residuals) * 0.95)] if len(residuals) > 1 else residuals[0]
            cam_stats["max_residual_m"] = max(residuals)
            cam_stats["outlier_pct"] = sum(1 for r in residuals if r > DEFAULT_OUTLIER_M) / len(residuals) * 100
            cam_stats["residuals"] = residuals
        else:
            cam_stats["mean_residual_m"] = None
            cam_stats["median_residual_m"] = None
            cam_stats["outlier_pct"] = None

        if ambiguities:
            cam_stats["mean_ambiguity"] = statistics.mean(ambiguities)
            cam_stats["high_ambiguity_pct"] = sum(1 for a in ambiguities if a > 0.5) / len(ambiguities) * 100

        result[cam] = cam_stats

    return result


def analyze_pose_stability_windows(
    ref_path: list[PoseSample],
    vision_sources: list[PoseSource],
    window_s: float = 5.0,
) -> list[dict]:
    """Find windows where vision and odometry diverge significantly.

    Slides a window across the match and measures vision-vs-odom divergence.
    """
    if not ref_path or not vision_sources:
        return []

    windows = []
    window_us = int(window_s * 1_000_000)

    for vs in vision_sources:
        if not vs.samples:
            continue

        start_us = vs.samples[0].timestamp_us
        end_us = vs.samples[-1].timestamp_us

        t = start_us
        while t + window_us <= end_us:
            window_samples = [
                s for s in vs.samples
                if t <= s.timestamp_us < t + window_us
            ]

            if len(window_samples) >= 3:
                errors = []
                for s in window_samples:
                    ref = interpolate_pose_at(ref_path, s.timestamp_us)
                    if ref:
                        errors.append(math.hypot(s.x - ref.x, s.y - ref.y))

                if errors:
                    windows.append({
                        "source": vs.name,
                        "start_s": (t - ref_path[0].timestamp_us) / 1_000_000,
                        "end_s": (t + window_us - ref_path[0].timestamp_us) / 1_000_000,
                        "mean_error_m": statistics.mean(errors),
                        "max_error_m": max(errors),
                        "sample_count": len(window_samples),
                    })

            t += window_us // 2  # 50% overlap

    return windows


def analyze_single_match(filepath: Path) -> MatchResult | None:
    """Run full analysis on a single match log file."""
    filename = filepath.name
    match_label = extract_match_label(filename)
    timestamp = extract_timestamp(filename)

    print(f"\n{'='*70}")
    print(f"Processing: {filename} ({match_label})")
    print(f"{'='*70}")

    try:
        log_data = read_wpilog(str(filepath))
    except Exception as e:
        print(f"  ERROR reading file: {e}")
        return None

    print(f"  Signals: {len(log_data.signals)}, Records: {log_data.metadata.record_count}")

    # Detect match phases
    timeline = detect_match_phases(log_data)
    if timeline is None:
        timeline = MatchPhaseTimeline()
    has_match = timeline.has_match

    auto_dur = sum(iv.duration_s for iv in timeline.intervals_for(MatchPhase.AUTONOMOUS))
    teleop_dur = sum(iv.duration_s for iv in timeline.intervals_for(MatchPhase.TELEOP))
    total_dur = 0.0
    if timeline.intervals:
        total_dur = (timeline.intervals[-1].end_us - timeline.intervals[0].start_us) / 1e6

    print(f"  Match phases: {'YES' if has_match else 'NO'} "
          f"(auto={auto_dur:.1f}s, teleop={teleop_dur:.1f}s, total={total_dur:.1f}s)")

    # Discover pose sources
    sources = discover_pose_sources(log_data)
    odom_sources = [s for s in sources if s.source_class == "odometry" and s.samples]
    vision_sources_list = [s for s in sources if s.source_class == "vision" and s.samples]

    print(f"  Pose sources: {len(odom_sources)} odometry, {len(vision_sources_list)} vision")
    for s in sources:
        if s.samples:
            print(f"    {s.name}: {s.sample_count} samples, "
                  f"{s.median_rate_hz:.1f} Hz" if s.median_rate_hz else
                  f"    {s.name}: {s.sample_count} samples")

    # Build reference path
    ref_path = build_reference_path(log_data, sources)
    print(f"  Reference path: {len(ref_path)} samples")

    # Compute divergence metrics
    pose_metrics = []
    divergence_events = []
    for src in sources:
        if src.samples and ref_path:
            m = compute_divergence_metrics(src.samples, ref_path)
            pose_metrics.append(m)
            evts = find_divergence_events(src.samples, ref_path)
            divergence_events.extend(evts)

    # Discover cameras and parse vision frames
    all_cameras = _discover_cameras(log_data)
    camera_names = list(all_cameras.keys())

    all_frames: list[VisionFrame] = []
    camera_frame_counts: dict[str, int] = {}
    for cam_name, signals in all_cameras.items():
        frames, total = _parse_frames(cam_name, signals)
        all_frames.extend(frames)
        camera_frame_counts[cam_name] = total

    print(f"  Cameras: {camera_names}")
    print(f"  Vision frames: {len(all_frames)} valid")

    # Compute residuals
    _compute_residuals(all_frames, log_data)

    # MT1 vs MT2
    _compute_mt1_mt2_divergence(all_frames, all_cameras)
    mt1_mt2_summary = _summarize_mt1_mt2(all_frames, camera_names)

    # Camera summaries
    per_camera = {}
    for cam_name, signals in all_cameras.items():
        cs = _camera_summary(cam_name, all_frames, camera_frame_counts[cam_name],
                             signals, DEFAULT_OUTLIER_M)
        per_camera[cam_name] = cs

    # Tag count bands
    tag_count_bands = _tag_count_table(all_frames)

    # Per-tag summaries
    tag_summaries = []
    distance_bands = []
    for cam_name in all_cameras:
        tag_summaries.extend(_tag_id_table(all_frames, cam_name))
        distance_bands.extend(_distance_band_table(all_frames, cam_name))

    # Detection gaps
    detection_gaps = []
    for cam_name in all_cameras:
        detection_gaps.extend(_find_detection_gaps(all_frames, cam_name))

    # Camera agreement
    camera_agreement = _compute_camera_agreement(all_frames, camera_names, DEFAULT_OUTLIER_M)

    # Hardware stats
    hw_stats = {}
    for cam_name, signals in all_cameras.items():
        hw_stats[cam_name] = _compute_hw_stats(cam_name, signals)

    # Phase-segmented analysis
    auto_vision = {}
    teleop_vision = {}
    if has_match:
        auto_vision = compute_phase_vision_stats(all_frames, timeline, MatchPhase.AUTONOMOUS, camera_names)
        teleop_vision = compute_phase_vision_stats(all_frames, timeline, MatchPhase.TELEOP, camera_names)

    result = MatchResult(
        filename=filename,
        match_label=match_label,
        timestamp=timestamp,
        has_match_phases=has_match,
        auto_duration_s=auto_dur,
        teleop_duration_s=teleop_dur,
        total_duration_s=total_dur,
        odom_sources=len(odom_sources),
        vision_sources=len(vision_sources_list),
        ref_path_samples=len(ref_path),
        cameras=camera_names,
        per_camera=per_camera,
        auto_vision=auto_vision,
        teleop_vision=teleop_vision,
        pose_metrics=pose_metrics,
        divergence_events=divergence_events,
        detection_gaps=detection_gaps,
        hw_stats=hw_stats,
        mt1_mt2=mt1_mt2_summary,
        camera_agreement=camera_agreement,
        tag_count_bands=tag_count_bands,
        tag_summaries=tag_summaries,
        distance_bands=distance_bands,
        all_frames=all_frames,
        ref_path=ref_path,
        timeline=timeline,
    )

    return result


def print_match_summary(r: MatchResult) -> None:
    """Print per-match summary."""
    print(f"\n  --- Camera Summaries ---")
    for cam, cs in r.per_camera.items():
        print(f"  {cam}:")
        print(f"    Valid frames: {cs.valid_frames}/{cs.total_frames} ({cs.valid_pct:.1f}%)")
        print(f"    Mean residual: {cs.mean_residual_m:.3f} m, Median: {cs.median_residual_m:.3f} m")
        print(f"    Outliers (>{DEFAULT_OUTLIER_M}m): {cs.outlier_frames} ({cs.outlier_pct:.1f}%)")
        print(f"    Mean latency: {cs.mean_latency_ms:.1f} ms")
        print(f"    Tag count dist: {cs.tag_count_distribution}")

    # Pose metrics
    if r.pose_metrics:
        print(f"\n  --- Pose Divergence Metrics ---")
        for m in r.pose_metrics:
            print(f"  {m.source_name} ({m.source_class}):")
            print(f"    Trans RMS: {m.translation_rms_m:.4f} m, Max: {m.translation_max_m:.4f} m")
            print(f"    Heading RMS: {math.degrees(m.heading_rms_rad):.2f}°, "
                  f"Max: {math.degrees(m.heading_max_rad):.2f}°")
            print(f"    Coverage: {m.coverage_fraction:.1%}, Confidence: {m.confidence}")

    # Phase breakdown
    if r.has_match_phases:
        for phase_name, phase_data in [("AUTO", r.auto_vision), ("TELEOP", r.teleop_vision)]:
            print(f"\n  --- {phase_name} Phase ---")
            for cam in r.cameras:
                if cam in phase_data:
                    cs = phase_data[cam]
                    res = cs.get("mean_residual_m")
                    med = cs.get("median_residual_m")
                    outlier = cs.get("outlier_pct")
                    tc = cs.get("mean_tag_count", 0)
                    lat = cs.get("mean_latency_ms", 0)
                    print(f"  {cam} ({phase_name}):")
                    print(f"    Frames: {cs.get('valid_frames', 0)}, "
                          f"Mean tag count: {tc:.1f}, Mean latency: {lat:.1f} ms")
                    if res is not None:
                        print(f"    Mean residual: {res:.3f} m, Median: {med:.3f} m, "
                              f"Outliers: {outlier:.1f}%")

    # Detection gaps
    big_gaps = [g for g in r.detection_gaps if g.duration_ms > 1000]
    if big_gaps:
        print(f"\n  --- Significant Detection Gaps (>1s) ---")
        for g in sorted(big_gaps, key=lambda x: x.duration_ms, reverse=True)[:10]:
            # Compute relative time
            t_start_s = g.start_us / 1e6
            print(f"  {g.camera}: {g.duration_ms:.0f} ms gap at t={t_start_s:.1f}s")

    # Hardware
    print(f"\n  --- Hardware Stats ---")
    for cam, hw in r.hw_stats.items():
        print(f"  {cam}: FPS={hw.mean_fps:.1f} (min={hw.min_fps:.0f}), "
              f"CPU={hw.mean_cpu_temp:.1f}C (peak={hw.peak_cpu_temp:.1f}C), "
              f"CPUUsage={hw.mean_cpu_usage_pct:.1f}% (peak={hw.peak_cpu_usage_pct:.1f}%)")

    # MT1 vs MT2
    if r.mt1_mt2:
        print(f"\n  --- MegaTag1 vs MegaTag2 ---")
        for cam, data in r.mt1_mt2.items():
            print(f"  {cam}: mean={data['mean']:.3f} m, "
                  f"median={data['median']:.3f} m, p95={data['p95']:.3f} m "
                  f"({int(data['count'])} frames)")

    # Camera agreement
    if r.camera_agreement:
        print(f"\n  --- Camera Agreement ---")
        for ca in r.camera_agreement:
            print(f"  {ca.camera_a} vs {ca.camera_b}: "
                  f"mean={ca.mean_disagreement_m:.3f} m, "
                  f"p95={ca.p95_disagreement_m:.3f} m, "
                  f"yaw={ca.mean_yaw_disagreement_deg:.1f}°, "
                  f"overlap={ca.overlap_frames} frames")


def print_cross_match_report(results: list[MatchResult]) -> None:
    """Print cross-match comparative analysis."""
    # Include actual match logs (labeled Q*) regardless of phase detection
    match_results = [r for r in results if r.match_label.startswith("Q")]

    print("\n" + "=" * 90)
    print("CROSS-MATCH ANALYSIS")
    print("=" * 90)

    # 1. Overall pose accuracy trend
    print("\n" + "-" * 90)
    print("1. POSE ACCURACY BY MATCH (sorted chronologically)")
    print("-" * 90)
    print(f"{'Match':<12} {'Time':<8} {'Auto(s)':<8} {'Tele(s)':<8} "
          f"{'Cameras':<12} {'Frames':<8} {'MeanRes':<8} {'MedRes':<8} "
          f"{'P95Res':<8} {'Outlier%':<9} {'MaxRes':<8}")

    for r in match_results:
        total_frames = sum(cs.valid_frames for cs in r.per_camera.values())
        all_residuals = [f.pose_residual_m for f in r.all_frames if f.pose_residual_m is not None]

        mean_res = statistics.mean(all_residuals) if all_residuals else 0
        med_res = statistics.median(all_residuals) if all_residuals else 0
        p95_res = sorted(all_residuals)[int(len(all_residuals) * 0.95)] if len(all_residuals) > 1 else 0
        max_res = max(all_residuals) if all_residuals else 0
        outlier_pct = sum(1 for r2 in all_residuals if r2 > DEFAULT_OUTLIER_M) / max(len(all_residuals), 1) * 100

        cams = ",".join(c.replace("limelight-", "ll-") for c in r.cameras)
        print(f"{r.match_label:<12} {r.timestamp:<8} {r.auto_duration_s:<8.1f} {r.teleop_duration_s:<8.1f} "
              f"{cams:<12} {total_frames:<8} {mean_res:<8.3f} {med_res:<8.3f} "
              f"{p95_res:<8.3f} {outlier_pct:<9.1f} {max_res:<8.2f}")

    # 2. Auto vs Teleop comparison (if any match has phases)
    has_any_phases = any(r.has_match_phases for r in match_results)
    if has_any_phases:
        print("\n" + "-" * 90)
        print("2. AUTO vs TELEOP ACCURACY COMPARISON")
        print("-" * 90)
        print(f"{'Match':<12} {'Camera':<14} {'AutoMedRes':<11} {'AutoOut%':<9} "
              f"{'TeleMedRes':<11} {'TeleOut%':<9} {'AutoTC':<7} {'TeleTC':<7}")

        for r in match_results:
            for cam in r.cameras:
                auto_d = r.auto_vision.get(cam, {})
                tele_d = r.teleop_vision.get(cam, {})

                auto_med = auto_d.get("median_residual_m")
                auto_out = auto_d.get("outlier_pct")
                tele_med = tele_d.get("median_residual_m")
                tele_out = tele_d.get("outlier_pct")
                auto_tc = auto_d.get("mean_tag_count", 0)
                tele_tc = tele_d.get("mean_tag_count", 0)

                cam_short = cam.replace("limelight-", "ll-")
                print(f"{r.match_label:<12} {cam_short:<14} "
                      f"{auto_med if auto_med is not None else '-':>10} "
                      f"{auto_out if auto_out is not None else '-':>8} "
                      f"{tele_med if tele_med is not None else '-':>10} "
                      f"{tele_out if tele_out is not None else '-':>8} "
                      f"{auto_tc:>6.1f} {tele_tc:>6.1f}")

    # 3. Per-camera cross-match trend
    print("\n" + "-" * 90)
    print("3. PER-CAMERA RESIDUAL TREND ACROSS MATCHES")
    print("-" * 90)

    # Gather all camera names
    all_cams = set()
    for r in match_results:
        all_cams.update(r.cameras)

    for cam in sorted(all_cams):
        print(f"\n  Camera: {cam}")
        print(f"  {'Match':<12} {'Valid':<7} {'MeanRes':<9} {'MedRes':<9} "
              f"{'P95Res':<9} {'Out%':<7} {'MeanTC':<8} {'MeanLat':<9} "
              f"{'FPS':<6} {'CPUTemp':<8} {'Gaps>1s':<7}")

        for r in match_results:
            if cam not in r.per_camera:
                continue
            cs = r.per_camera[cam]
            hw = r.hw_stats.get(cam)
            big_gaps = sum(1 for g in r.detection_gaps if g.camera == cam and g.duration_ms > 1000)

            # Get mean tag count
            cam_frames = [f for f in r.all_frames if f.camera == cam]
            mean_tc = statistics.mean([f.tag_count for f in cam_frames]) if cam_frames else 0

            residuals = [f.pose_residual_m for f in cam_frames if f.pose_residual_m is not None]
            p95 = sorted(residuals)[int(len(residuals) * 0.95)] if len(residuals) > 1 else 0

            fps = hw.mean_fps if hw else 0
            cpu_temp = hw.peak_cpu_temp if hw else 0

            print(f"  {r.match_label:<12} {cs.valid_frames:<7} "
                  f"{cs.mean_residual_m:<9.3f} {cs.median_residual_m:<9.3f} "
                  f"{p95:<9.3f} {cs.outlier_pct:<7.1f} {mean_tc:<8.1f} "
                  f"{cs.mean_latency_ms:<9.1f} {fps:<6.0f} {cpu_temp:<8.1f} {big_gaps:<7}")

    # 4. Tag count impact analysis
    print("\n" + "-" * 90)
    print("4. TAG COUNT IMPACT ON ACCURACY (all matches combined)")
    print("-" * 90)

    combined_by_tc: dict[int, list[float]] = {}
    for r in match_results:
        for f in r.all_frames:
            if f.pose_residual_m is not None:
                tc = min(f.tag_count, 4)
                combined_by_tc.setdefault(tc, []).append(f.pose_residual_m)

    print(f"  {'TagCount':<10} {'Frames':<10} {'MeanRes':<10} {'MedRes':<10} "
          f"{'P95Res':<10} {'Out%':<8}")
    for tc in sorted(combined_by_tc):
        residuals = combined_by_tc[tc]
        n = len(residuals)
        mean_r = statistics.mean(residuals)
        med_r = statistics.median(residuals)
        p95_r = sorted(residuals)[int(n * 0.95)] if n > 1 else residuals[0]
        out_pct = sum(1 for r2 in residuals if r2 > DEFAULT_OUTLIER_M) / n * 100
        print(f"  {tc:<10} {n:<10} {mean_r:<10.3f} {med_r:<10.3f} {p95_r:<10.3f} {out_pct:<8.1f}")

    # 5. Divergence events
    print("\n" + "-" * 90)
    print("5. SIGNIFICANT DIVERGENCE EVENTS")
    print("-" * 90)

    for r in match_results:
        significant = [e for e in r.divergence_events if e.peak_translation_error_m > 0.5]
        if significant:
            print(f"\n  {r.match_label}:")
            for e in sorted(significant, key=lambda x: x.peak_translation_error_m, reverse=True)[:5]:
                dur_s = (e.end_us - e.start_us) / 1e6
                print(f"    {e.source_name}: peak={e.peak_translation_error_m:.3f} m, "
                      f"heading={math.degrees(e.peak_heading_error_rad):.1f}°, "
                      f"duration={dur_s:.1f}s, cause={e.likely_cause}")

    # 6. Detection gap analysis
    print("\n" + "-" * 90)
    print("6. DETECTION GAPS (>1 second) ACROSS MATCHES")
    print("-" * 90)

    for r in match_results:
        big_gaps = [g for g in r.detection_gaps if g.duration_ms > 1000]
        if big_gaps:
            print(f"\n  {r.match_label}: {len(big_gaps)} gaps >1s")
            for g in sorted(big_gaps, key=lambda x: x.duration_ms, reverse=True)[:5]:
                print(f"    {g.camera}: {g.duration_ms:.0f} ms")

    # 7. Hardware health trends
    print("\n" + "-" * 90)
    print("7. HARDWARE HEALTH TRENDS (Limelight)")
    print("-" * 90)

    for cam in sorted(all_cams):
        print(f"\n  Camera: {cam}")
        print(f"  {'Match':<12} {'MeanFPS':<9} {'MinFPS':<8} {'MeanCPU':<9} "
              f"{'PeakCPU':<9} {'MeanUsg%':<10} {'PeakUsg%':<10}")
        for r in match_results:
            hw = r.hw_stats.get(cam)
            if hw:
                print(f"  {r.match_label:<12} {hw.mean_fps:<9.1f} {hw.min_fps:<8.0f} "
                      f"{hw.mean_cpu_temp:<9.1f} {hw.peak_cpu_temp:<9.1f} "
                      f"{hw.mean_cpu_usage_pct:<10.1f} {hw.peak_cpu_usage_pct:<10.1f}")

    # 8. MT1 vs MT2 trends
    print("\n" + "-" * 90)
    print("8. MEGATAG1 vs MEGATAG2 DIVERGENCE TREND")
    print("-" * 90)

    for cam in sorted(all_cams):
        print(f"\n  Camera: {cam}")
        print(f"  {'Match':<12} {'Frames':<8} {'MeanDiv':<9} {'MedDiv':<9} {'P95Div':<9}")
        for r in match_results:
            mt_data = r.mt1_mt2.get(cam)
            if mt_data:
                print(f"  {r.match_label:<12} {int(mt_data['count']):<8} "
                      f"{mt_data['mean']:<9.3f} {mt_data['median']:<9.3f} "
                      f"{mt_data['p95']:<9.3f}")

    # 9. Best and worst windows
    print("\n" + "-" * 90)
    print("9. BEST AND WORST 5-SECOND WINDOWS (per match)")
    print("-" * 90)

    for r in match_results:
        sources_for_windows = [s for s in discover_pose_sources(LogData(metadata=r.ref_path[0].source_name if r.ref_path else "", signals={}))
                               ] if False else []

        # Use frame-level data instead for windowed analysis
        if not r.all_frames or not r.ref_path:
            continue

        # Group frames into 5-second windows
        if not r.all_frames:
            continue

        frames_with_res = [f for f in r.all_frames if f.pose_residual_m is not None]
        if not frames_with_res:
            continue

        frames_with_res.sort(key=lambda f: f.timestamp_us)
        base_ts = frames_with_res[0].timestamp_us
        window_us = 5_000_000

        windows = []
        i = 0
        while i < len(frames_with_res):
            t_start = frames_with_res[i].timestamp_us
            win_frames = []
            j = i
            while j < len(frames_with_res) and frames_with_res[j].timestamp_us < t_start + window_us:
                win_frames.append(frames_with_res[j])
                j += 1

            if len(win_frames) >= 5:
                res = [f.pose_residual_m for f in win_frames]
                t_rel = (t_start - base_ts) / 1e6
                phase = r.timeline.phase_at(t_start) if r.timeline else MatchPhase.DISABLED
                windows.append({
                    "t_start_s": t_rel,
                    "phase": phase.value,
                    "n_frames": len(win_frames),
                    "mean_res": statistics.mean(res),
                    "median_res": statistics.median(res),
                    "max_res": max(res),
                    "mean_tc": statistics.mean([f.tag_count for f in win_frames]),
                })

            # Advance by 2.5s (50% overlap)
            next_t = t_start + window_us // 2
            while i < len(frames_with_res) and frames_with_res[i].timestamp_us < next_t:
                i += 1

        if windows:
            windows.sort(key=lambda w: w["mean_res"])
            print(f"\n  {r.match_label}:")
            print(f"    BEST 3 windows (lowest mean residual):")
            for w in windows[:3]:
                print(f"      t={w['t_start_s']:.1f}s ({w['phase']}): "
                      f"mean={w['mean_res']:.3f}m, med={w['median_res']:.3f}m, "
                      f"max={w['max_res']:.3f}m, TC={w['mean_tc']:.1f}, "
                      f"n={w['n_frames']}")
            print(f"    WORST 3 windows (highest mean residual):")
            for w in windows[-3:]:
                print(f"      t={w['t_start_s']:.1f}s ({w['phase']}): "
                      f"mean={w['mean_res']:.3f}m, med={w['median_res']:.3f}m, "
                      f"max={w['max_res']:.3f}m, TC={w['mean_tc']:.1f}, "
                      f"n={w['n_frames']}")

    # 10. Cross-match residual distribution
    print("\n" + "-" * 90)
    print("10. CROSS-MATCH RESIDUAL DISTRIBUTION SUMMARY")
    print("-" * 90)

    all_match_medians = []
    for r in match_results:
        all_res = [f.pose_residual_m for f in r.all_frames if f.pose_residual_m is not None]
        if all_res:
            all_match_medians.append((r.match_label, statistics.median(all_res)))

    if all_match_medians:
        all_match_medians.sort(key=lambda x: x[1])
        print(f"\n  Matches ranked by median residual (best -> worst):")
        for label, med in all_match_medians:
            marker = " [EXCELLENT]" if med < 0.2 else " [POOR]" if med > 0.5 else ""
            print(f"    {label}: {med:.3f} m{marker}")

    # 11. Vision-vs-odometry agreement per match
    print("\n" + "-" * 90)
    print("11. VISION-vs-ODOMETRY DIVERGENCE PER MATCH")
    print("-" * 90)

    for r in match_results:
        vis_metrics = [m for m in r.pose_metrics if m.source_class == "vision"]
        odom_metrics = [m for m in r.pose_metrics if m.source_class == "odometry"]

        print(f"\n  {r.match_label}:")
        for m in odom_metrics:
            print(f"    Odometry ({m.source_name}): RMS={m.translation_rms_m:.4f}m, "
                  f"Max={m.translation_max_m:.3f}m, "
                  f"HeadRMS={math.degrees(m.heading_rms_rad):.2f}°")
        for m in vis_metrics:
            print(f"    Vision ({m.source_name}): RMS={m.translation_rms_m:.4f}m, "
                  f"Max={m.translation_max_m:.3f}m, "
                  f"HeadRMS={math.degrees(m.heading_rms_rad):.2f}°")

    # 12. Tag-level analysis (which tags are most/least reliable)
    print("\n" + "-" * 90)
    print("12. TAG RELIABILITY ACROSS ALL MATCHES")
    print("-" * 90)

    combined_tag_data: dict[int, dict] = {}
    for r in match_results:
        for ts in r.tag_summaries:
            if ts.tag_id not in combined_tag_data:
                combined_tag_data[ts.tag_id] = {
                    "detections": 0,
                    "ambiguity_sum": 0.0,
                    "distance_sum": 0.0,
                    "residual_sum": 0.0,
                    "residual_count": 0,
                    "high_amb_count": 0,
                }
            d = combined_tag_data[ts.tag_id]
            d["detections"] += ts.detections
            d["ambiguity_sum"] += ts.mean_ambiguity * ts.detections
            d["distance_sum"] += ts.mean_distance * ts.detections
            if ts.mean_residual_when_primary > 0:
                d["residual_sum"] += ts.mean_residual_when_primary * ts.detections
                d["residual_count"] += ts.detections
            d["high_amb_count"] += int(ts.high_ambiguity_pct / 100 * ts.detections)

    print(f"  {'TagID':<8} {'Detections':<12} {'MeanAmb':<10} {'MeanDist':<10} "
          f"{'MeanRes':<10} {'HighAmb%':<10}")
    for tid in sorted(combined_tag_data):
        d = combined_tag_data[tid]
        n = d["detections"]
        mean_amb = d["ambiguity_sum"] / n if n > 0 else 0
        mean_dist = d["distance_sum"] / n if n > 0 else 0
        mean_res = d["residual_sum"] / d["residual_count"] if d["residual_count"] > 0 else 0
        high_amb_pct = d["high_amb_count"] / n * 100 if n > 0 else 0
        print(f"  {tid:<8} {n:<12} {mean_amb:<10.3f} {mean_dist:<10.1f} "
              f"{mean_res:<10.3f} {high_amb_pct:<10.1f}")

    # 13. Distance band analysis across all matches
    print("\n" + "-" * 90)
    print("13. ACCURACY BY DISTANCE BAND (all matches combined)")
    print("-" * 90)

    combined_dist: dict[str, dict] = {}
    for r in match_results:
        for db in r.distance_bands:
            key = db.band_label
            if key not in combined_dist:
                combined_dist[key] = {
                    "detections": 0,
                    "amb_sum": 0.0,
                    "res_sum": 0.0,
                    "res_count": 0,
                    "high_amb": 0,
                }
            d = combined_dist[key]
            d["detections"] += db.detections
            d["amb_sum"] += db.mean_ambiguity * db.detections
            if db.mean_residual_m > 0:
                d["res_sum"] += db.mean_residual_m * db.detections
                d["res_count"] += db.detections
            d["high_amb"] += int(db.high_ambiguity_pct / 100 * db.detections)

    print(f"  {'Band':<10} {'Detections':<12} {'MeanAmb':<10} {'MeanRes':<10} {'HighAmb%':<10}")
    for band in ["0–1m", "1–2m", "2–3m", "3–4m", "4–5m", "5m+"]:
        if band in combined_dist:
            d = combined_dist[band]
            n = d["detections"]
            mean_amb = d["amb_sum"] / n if n > 0 else 0
            mean_res = d["res_sum"] / d["res_count"] if d["res_count"] > 0 else 0
            high_amb = d["high_amb"] / n * 100 if n > 0 else 0
            print(f"  {band:<10} {n:<12} {mean_amb:<10.3f} {mean_res:<10.3f} {high_amb:<10.1f}")

    # 14. Camera agreement trends
    if any(r.camera_agreement for r in match_results):
        print("\n" + "-" * 90)
        print("14. CAMERA AGREEMENT ACROSS MATCHES")
        print("-" * 90)
        print(f"  {'Match':<12} {'CamA':<14} {'CamB':<14} {'MeanDisagree':<14} "
              f"{'P95Disagree':<14} {'YawDiff':<10} {'Overlap':<8}")
        for r in match_results:
            for ca in r.camera_agreement:
                print(f"  {r.match_label:<12} "
                      f"{ca.camera_a.replace('limelight-','ll-'):<14} "
                      f"{ca.camera_b.replace('limelight-','ll-'):<14} "
                      f"{ca.mean_disagreement_m:<14.3f} "
                      f"{ca.p95_disagreement_m:<14.3f} "
                      f"{ca.mean_yaw_disagreement_deg:<10.1f} "
                      f"{ca.overlap_frames:<8}")

    # 15. Identify exceptional matches
    print("\n" + "-" * 90)
    print("15. EXCEPTIONAL PERFORMANCE HIGHLIGHTS")
    print("-" * 90)

    if all_match_medians:
        best_match = min(match_results, key=lambda r: statistics.median(
            [f.pose_residual_m for f in r.all_frames if f.pose_residual_m is not None] or [999]))
        worst_match = max(match_results, key=lambda r: statistics.median(
            [f.pose_residual_m for f in r.all_frames if f.pose_residual_m is not None] or [0]))

        best_res = [f.pose_residual_m for f in best_match.all_frames if f.pose_residual_m is not None]
        worst_res = [f.pose_residual_m for f in worst_match.all_frames if f.pose_residual_m is not None]

        print(f"\n  BEST MATCH: {best_match.match_label}")
        if best_res:
            print(f"    Median residual: {statistics.median(best_res):.3f} m")
            print(f"    Mean residual: {statistics.mean(best_res):.3f} m")
            print(f"    Outlier rate: {sum(1 for r2 in best_res if r2 > 1.0) / len(best_res) * 100:.1f}%")

        print(f"\n  WORST MATCH: {worst_match.match_label}")
        if worst_res:
            print(f"    Median residual: {statistics.median(worst_res):.3f} m")
            print(f"    Mean residual: {statistics.mean(worst_res):.3f} m")
            print(f"    Outlier rate: {sum(1 for r2 in worst_res if r2 > 1.0) / len(worst_res) * 100:.1f}%")

    # Identify auto-specific performance
    if has_any_phases:
        print("\n  AUTO PHASE - Best and Worst matches:")
        auto_performance = []
        for r in match_results:
            if r.auto_vision:
                auto_res = []
                for cam in r.cameras:
                    if cam in r.auto_vision and "residuals" in r.auto_vision[cam]:
                        auto_res.extend(r.auto_vision[cam]["residuals"])
                if auto_res:
                    auto_performance.append((r.match_label, statistics.median(auto_res), auto_res))

        if auto_performance:
            auto_performance.sort(key=lambda x: x[1])
            print(f"    Best auto: {auto_performance[0][0]} (median={auto_performance[0][1]:.3f}m)")
            print(f"    Worst auto: {auto_performance[-1][0]} (median={auto_performance[-1][1]:.3f}m)")


    # 16. Look for pattern changes (software/hardware)
    print("\n" + "-" * 90)
    print("16. SOFTWARE/HARDWARE CHANGE DETECTION")
    print("-" * 90)
    print("  Looking for jumps in key metrics between consecutive matches...\n")

    prev_r = None
    for r in match_results:
        if prev_r is None:
            prev_r = r
            continue

        curr_res = [f.pose_residual_m for f in r.all_frames if f.pose_residual_m is not None]
        prev_res = [f.pose_residual_m for f in prev_r.all_frames if f.pose_residual_m is not None]

        if curr_res and prev_res:
            curr_med = statistics.median(curr_res)
            prev_med = statistics.median(prev_res)
            delta = curr_med - prev_med
            pct_change = delta / max(prev_med, 0.001) * 100

            if abs(pct_change) > 30:
                direction = "IMPROVED" if delta < 0 else "DEGRADED"
                print(f"  {prev_r.match_label} → {r.match_label}: {direction} by {abs(pct_change):.0f}%")
                print(f"    Median residual: {prev_med:.3f}m → {curr_med:.3f}m")

                # Check what changed
                changes = []
                for cam in r.cameras:
                    if cam in r.hw_stats and cam in prev_r.hw_stats:
                        curr_hw = r.hw_stats[cam]
                        prev_hw = prev_r.hw_stats[cam]
                        if abs(curr_hw.mean_fps - prev_hw.mean_fps) > 5:
                            changes.append(f"  FPS: {prev_hw.mean_fps:.0f} → {curr_hw.mean_fps:.0f}")
                        if abs(curr_hw.peak_cpu_temp - prev_hw.peak_cpu_temp) > 5:
                            changes.append(f"  CPU temp: {prev_hw.peak_cpu_temp:.1f} → {curr_hw.peak_cpu_temp:.1f}°C")

                    curr_cam_frames = [f for f in r.all_frames if f.camera == cam]
                    prev_cam_frames = [f for f in prev_r.all_frames if f.camera == cam]
                    if curr_cam_frames and prev_cam_frames:
                        curr_tc = statistics.mean([f.tag_count for f in curr_cam_frames])
                        prev_tc = statistics.mean([f.tag_count for f in prev_cam_frames])
                        if abs(curr_tc - prev_tc) > 0.5:
                            changes.append(f"  Mean tag count: {prev_tc:.1f} → {curr_tc:.1f}")

                        curr_lat = statistics.mean([f.total_latency_ms for f in curr_cam_frames])
                        prev_lat = statistics.mean([f.total_latency_ms for f in prev_cam_frames])
                        if abs(curr_lat - prev_lat) > 5:
                            changes.append(f"  Latency: {prev_lat:.1f} → {curr_lat:.1f} ms")

                        curr_ambs = [f.max_ambiguity for f in curr_cam_frames if f.tags]
                        prev_ambs = [f.max_ambiguity for f in prev_cam_frames if f.tags]
                        if curr_ambs and prev_ambs:
                            curr_amb = statistics.mean(curr_ambs)
                            prev_amb = statistics.mean(prev_ambs)
                            if abs(curr_amb - prev_amb) > 0.05:
                                changes.append(f"  Mean ambiguity: {prev_amb:.3f} → {curr_amb:.3f}")

                if changes:
                    print(f"    Concurrent metric changes ({cam}):")
                    for c in changes:
                        print(f"      {c}")

        prev_r = r

    # 17. Limelight exposure/focus investigation
    print("\n" + "-" * 90)
    print("17. LIMELIGHT TUNING INVESTIGATION (exposure/focus/brightness effects)")
    print("-" * 90)
    print("  Proxies for tuning changes: latency shifts, ambiguity shifts, detection rate changes\n")

    for cam in sorted(all_cams):
        print(f"  Camera: {cam}")
        print(f"  {'Match':<12} {'DetRate%':<10} {'MeanAmb':<10} {'MedAmb':<10} "
              f"{'MeanLat':<10} {'P95Lat':<10} {'AvgDist':<10} {'AvgArea':<10}")
        for r in match_results:
            if cam not in r.per_camera:
                continue
            cs = r.per_camera[cam]
            cam_frames = [f for f in r.all_frames if f.camera == cam]

            ambs = [f.max_ambiguity for f in cam_frames if f.tags]
            lats = [f.total_latency_ms for f in cam_frames]
            dists = [f.avg_tag_dist for f in cam_frames]
            areas = [f.avg_tag_area for f in cam_frames]

            det_rate = cs.valid_pct
            mean_amb = statistics.mean(ambs) if ambs else 0
            med_amb = statistics.median(ambs) if ambs else 0
            mean_lat = statistics.mean(lats) if lats else 0
            p95_lat = sorted(lats)[int(len(lats) * 0.95)] if len(lats) > 1 else 0
            mean_dist = statistics.mean(dists) if dists else 0
            mean_area = statistics.mean(areas) if areas else 0

            print(f"  {r.match_label:<12} {det_rate:<10.1f} {mean_amb:<10.3f} "
                  f"{med_amb:<10.3f} {mean_lat:<10.1f} {p95_lat:<10.1f} "
                  f"{mean_dist:<10.2f} {mean_area:<10.4f}")


def main():
    # Collect all match wpilog files
    wpilog_files = sorted(LOG_DIR.glob("FRC_*.wpilog"))
    print(f"Found {len(wpilog_files)} .wpilog files in {LOG_DIR}")
    for f in wpilog_files:
        print(f"  {f.name}")

    # Process each file
    results: list[MatchResult] = []
    for filepath in wpilog_files:
        result = analyze_single_match(filepath)
        if result:
            print_match_summary(result)
            results.append(result)

    if not results:
        print("No valid match results to analyze.")
        return

    # Cross-match report
    print_cross_match_report(results)

    print("\n" + "=" * 90)
    print("ANALYSIS COMPLETE")
    print(f"Processed {len(results)} log files, "
          f"{sum(1 for r in results if r.has_match_phases)} with match phases")
    print("=" * 90)


if __name__ == "__main__":
    main()
