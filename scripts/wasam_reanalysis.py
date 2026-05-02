"""WASAM comprehensive re-analysis with confound controls.

For each match, computes vision accuracy broken down by:
- Robot speed
- Tag distance
- Tag count
- Ambiguity
- Camera (ll-a vs ll-b)
- Match phase (auto vs teleop)
- Hub vs DS
Then identifies what actually explains the per-match variance.
"""

from __future__ import annotations

import math
import statistics
import sys
import re
from pathlib import Path
from dataclasses import dataclass, field

if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from logreader.wpilog_reader import read_wpilog
from logreader.models import LogData
from logreader.analyzers.vision_analysis import (
    VisionFrame, _discover_cameras, _parse_frames, _compute_residuals,
    _compute_hw_stats, DEFAULT_OUTLIER_M,
)
from logreader.analyzers.pose_analysis import (
    discover_pose_sources, build_reference_path, compute_velocity_at,
    compute_divergence_metrics, PoseSource,
)
from logreader.analyzers.match_phases import (
    detect_match_phases, MatchPhase, MatchPhaseTimeline,
)

ALL_HUB = {2,3,4,5,8,9,10,11, 18,19,20,21,24,25,26,27}
ALL_DS = {13,14,15,16, 29,30,31,32}

LOG_DIR = Path(r"D:\Temp\2026_WASAM")


def pct(n: int, d: int) -> float:
    return n / d * 100 if d > 0 else 0.0


def stats_bucket(vals: list[float]) -> dict:
    if not vals:
        return {"n": 0, "mean": 0, "med": 0, "p95": 0, "max": 0, "out_pct": 0}
    s = sorted(vals)
    return {
        "n": len(s),
        "mean": statistics.mean(s),
        "med": statistics.median(s),
        "p95": s[int(len(s) * 0.95)] if len(s) > 1 else s[0],
        "max": s[-1],
        "out_pct": sum(1 for r in s if r > 1.0) / len(s) * 100,
    }


def fmt(d: dict) -> str:
    if d["n"] == 0:
        return "no data"
    return (f"n={d['n']}, med={d['med']:.3f}m, mean={d['mean']:.3f}m, "
            f"p95={d['p95']:.3f}m, out%={d['out_pct']:.1f}%")


def classify_target(frame: VisionFrame) -> str:
    if not frame.tags:
        return "unknown"
    ids = {t.tag_id for t in frame.tags}
    hub = ids & ALL_HUB
    ds = ids & ALL_DS
    if hub and not ds:
        return "hub"
    elif ds and not hub:
        return "ds"
    elif hub and ds:
        return "mixed"
    return "other"


def main():
    wpilog_files = sorted(LOG_DIR.glob("FRC_*_WASAM_Q*.wpilog"))
    print(f"Found {len(wpilog_files)} match files\n")

    # Collect enriched per-frame records across all matches
    all_records = []

    for filepath in wpilog_files:
        m = re.search(r"WASAM_(Q\d+)", filepath.name)
        if not m:
            continue
        match_label = m.group(1)
        print(f"Loading {match_label}...")

        log_data = read_wpilog(str(filepath))

        # Phases
        timeline = detect_match_phases(log_data)
        if timeline is None:
            timeline = MatchPhaseTimeline()

        # Pose sources
        sources = discover_pose_sources(log_data)
        ref_path = build_reference_path(log_data, sources)
        odom_sources = [s for s in sources if s.source_class == "odometry" and s.samples]
        vision_sources = [s for s in sources if s.source_class == "vision" and s.samples]

        # Vision frames
        all_cameras = _discover_cameras(log_data)
        all_frames: list[VisionFrame] = []
        camera_frame_counts = {}
        for cam_name, signals in all_cameras.items():
            frames, total = _parse_frames(cam_name, signals)
            all_frames.extend(frames)
            camera_frame_counts[cam_name] = total
        _compute_residuals(all_frames, log_data)

        # Hardware
        hw_stats = {}
        for cam_name, signals in all_cameras.items():
            hw_stats[cam_name] = _compute_hw_stats(cam_name, signals)

        # Divergence
        odom_metrics = []
        vis_metrics = []
        for src in sources:
            if src.samples and ref_path:
                m2 = compute_divergence_metrics(src.samples, ref_path)
                if src.source_class == "odometry":
                    odom_metrics.append(m2)
                elif src.source_class == "vision":
                    vis_metrics.append(m2)

        # Enrich each frame with speed and phase
        for f in all_frames:
            if f.pose_residual_m is None:
                continue

            speed = 0.0
            if ref_path:
                vx, vy, _ = compute_velocity_at(ref_path, f.timestamp_us)
                speed = math.hypot(vx, vy)
                if speed > 8.0:
                    speed = 0.0

            phase = timeline.phase_at(f.timestamp_us).value

            # Tag stats
            dists = [t.dist_to_camera for t in f.tags] if f.tags else [0]
            ambs = [t.ambiguity for t in f.tags] if f.tags else [0]

            all_records.append({
                "match": match_label,
                "camera": f.camera,
                "residual": f.pose_residual_m,
                "tag_count": f.tag_count,
                "mean_dist": statistics.mean(dists),
                "max_amb": max(ambs),
                "mean_amb": statistics.mean(ambs),
                "avg_tag_dist": f.avg_tag_dist,
                "avg_tag_area": f.avg_tag_area,
                "aspect_ratio": f.tag_aspect_ratio,
                "latency_ms": f.total_latency_ms,
                "speed_mps": speed,
                "phase": phase,
                "target": classify_target(f),
                "robot_x": f.x,
                "robot_y": f.y,
            })

        # Print per-match summary
        valid_frames = [f for f in all_frames if f.pose_residual_m is not None]
        residuals = [f.pose_residual_m for f in valid_frames]
        rs = stats_bucket(residuals)
        cam_names = sorted(all_cameras.keys())

        print(f"\n{'='*90}")
        print(f"  {match_label}: {len(valid_frames)} valid frames, {fmt(rs)}")

        # Phases
        auto_dur = sum(iv.duration_s for iv in timeline.intervals_for(MatchPhase.AUTONOMOUS))
        teleop_dur = sum(iv.duration_s for iv in timeline.intervals_for(MatchPhase.TELEOP))
        print(f"  Phases: auto={auto_dur:.1f}s, teleop={teleop_dur:.1f}s")

        # Hardware (corrected)
        for cam_name in cam_names:
            hw = hw_stats.get(cam_name)
            if hw:
                print(f"  {cam_name}: FPS={hw.mean_fps:.0f}, "
                      f"CPU={hw.mean_cpu_temp:.1f}C (peak={hw.peak_cpu_temp:.1f}C), "
                      f"CPUUsage={hw.mean_cpu_usage_pct:.0f}% (peak={hw.peak_cpu_usage_pct:.0f}%)")

        # Per-camera breakdown
        for cam in cam_names:
            cam_frames = [f for f in valid_frames if f.camera == cam]
            cam_res = [f.pose_residual_m for f in cam_frames]
            cs = stats_bucket(cam_res)

            # Confound distributions for this camera in this match
            speeds = [r["speed_mps"] for r in all_records if r["match"] == match_label and r["camera"] == cam]
            dists = [r["avg_tag_dist"] for r in all_records if r["match"] == match_label and r["camera"] == cam]
            tcs = [r["tag_count"] for r in all_records if r["match"] == match_label and r["camera"] == cam]
            ambs = [r["max_amb"] for r in all_records if r["match"] == match_label and r["camera"] == cam]
            lats = [r["latency_ms"] for r in all_records if r["match"] == match_label and r["camera"] == cam]

            total = camera_frame_counts.get(cam, 0)
            det_rate = pct(len(cam_frames), total)

            print(f"  {cam}: {fmt(cs)}")
            print(f"    DetRate={det_rate:.1f}%, "
                  f"meanTC={statistics.mean(tcs):.1f}, "
                  f"meanDist={statistics.mean(dists):.1f}m, "
                  f"meanAmb={statistics.mean(ambs):.3f}, "
                  f"meanSpeed={statistics.mean(speeds):.1f}m/s, "
                  f"meanLat={statistics.mean(lats):.1f}ms")

        # Vision vs odometry divergence
        for m2 in odom_metrics:
            print(f"  Odom({m2.source_name}): RMS={m2.translation_rms_m:.3f}m, Max={m2.translation_max_m:.2f}m")
        for m2 in vis_metrics:
            print(f"  Vision({m2.source_name}): RMS={m2.translation_rms_m:.3f}m, Max={m2.translation_max_m:.2f}m")

        # Auto vs Teleop
        for ph in ["auto", "teleop"]:
            ph_recs = [r for r in all_records if r["match"] == match_label and r["phase"] == ph]
            ph_res = [r["residual"] for r in ph_recs]
            if ph_res:
                ph_s = stats_bucket(ph_res)
                ph_speed = statistics.mean([r["speed_mps"] for r in ph_recs])
                ph_dist = statistics.mean([r["avg_tag_dist"] for r in ph_recs])
                ph_tc = statistics.mean([r["tag_count"] for r in ph_recs])
                ph_amb = statistics.mean([r["max_amb"] for r in ph_recs])
                print(f"  {ph.upper()}: {fmt(ph_s)}  "
                      f"speed={ph_speed:.1f}m/s, dist={ph_dist:.1f}m, TC={ph_tc:.1f}, amb={ph_amb:.3f}")

        # Hub vs DS
        for target in ["hub", "ds"]:
            t_recs = [r for r in all_records if r["match"] == match_label and r["target"] == target]
            t_res = [r["residual"] for r in t_recs]
            if t_res:
                t_s = stats_bucket(t_res)
                t_dist = statistics.mean([r["avg_tag_dist"] for r in t_recs])
                t_tc = statistics.mean([r["tag_count"] for r in t_recs])
                t_amb = statistics.mean([r["max_amb"] for r in t_recs])
                print(f"  {target.upper()}: {fmt(t_s)}  "
                      f"dist={t_dist:.1f}m, TC={t_tc:.1f}, amb={t_amb:.3f}")

    # =========================================================================
    # CROSS-MATCH ANALYSIS
    # =========================================================================
    match_labels = sorted(set(r["match"] for r in all_records), key=lambda x: int(x[1:]))

    print(f"\n\n{'='*100}")
    print("CROSS-MATCH ANALYSIS")
    print(f"{'='*100}")

    # 1. Summary table
    print(f"\n{'-'*100}")
    print("1. MATCH SUMMARY TABLE")
    print(f"{'-'*100}")
    print(f"{'Match':<8} {'Frames':<8} {'MedRes':<9} {'MeanRes':<9} {'Out%':<7} "
          f"{'MedDist':<9} {'MedTC':<7} {'MedAmb':<8} {'MedSpd':<8} {'MedLat':<8}")

    for ml in match_labels:
        recs = [r for r in all_records if r["match"] == ml]
        res = [r["residual"] for r in recs]
        rs = stats_bucket(res)
        med_dist = statistics.median([r["avg_tag_dist"] for r in recs])
        med_tc = statistics.median([r["tag_count"] for r in recs])
        med_amb = statistics.median([r["max_amb"] for r in recs])
        med_spd = statistics.median([r["speed_mps"] for r in recs])
        med_lat = statistics.median([r["latency_ms"] for r in recs])
        print(f"{ml:<8} {rs['n']:<8} {rs['med']:<9.3f} {rs['mean']:<9.3f} {rs['out_pct']:<7.1f} "
              f"{med_dist:<9.2f} {med_tc:<7.0f} {med_amb:<8.3f} {med_spd:<8.2f} {med_lat:<8.1f}")

    # 2. Per-camera summary table
    print(f"\n{'-'*100}")
    print("2. PER-CAMERA SUMMARY")
    print(f"{'-'*100}")

    cameras = sorted(set(r["camera"] for r in all_records))
    for cam in cameras:
        print(f"\n  {cam}:")
        print(f"  {'Match':<8} {'N':<7} {'MedRes':<9} {'Out%':<7} "
              f"{'MedDist':<9} {'MedTC':<7} {'MedAmb':<8} {'MedSpd':<8} {'MedLat':<8}")
        for ml in match_labels:
            recs = [r for r in all_records if r["match"] == ml and r["camera"] == cam]
            if not recs:
                continue
            res = [r["residual"] for r in recs]
            rs = stats_bucket(res)
            print(f"  {ml:<8} {rs['n']:<7} {rs['med']:<9.3f} {rs['out_pct']:<7.1f} "
                  f"{statistics.median([r['avg_tag_dist'] for r in recs]):<9.2f} "
                  f"{statistics.median([r['tag_count'] for r in recs]):<7.0f} "
                  f"{statistics.median([r['max_amb'] for r in recs]):<8.3f} "
                  f"{statistics.median([r['speed_mps'] for r in recs]):<8.2f} "
                  f"{statistics.median([r['latency_ms'] for r in recs]):<8.1f}")

    # 3. Corrected hardware table
    print(f"\n{'-'*100}")
    print("3. CORRECTED HARDWARE STATS (per docs: [cpu_temp, cpu_usage%, ram%, fps])")
    print(f"{'-'*100}")
    # Re-load hw stats with corrected parsing
    for cam in cameras:
        print(f"\n  {cam}:")
        print(f"  {'Match':<8} {'FPS':<6} {'CPUTemp':<9} {'PeakCPU':<9} "
              f"{'CPUUsg%':<9} {'PeakUsg%':<10}")
        for filepath in sorted(LOG_DIR.glob("FRC_*_WASAM_Q*.wpilog")):
            m2 = re.search(r"WASAM_(Q\d+)", filepath.name)
            if not m2:
                continue
            ml = m2.group(1)
            ld = read_wpilog(str(filepath))
            cams = _discover_cameras(ld)
            if cam in cams:
                hw = _compute_hw_stats(cam, cams[cam])
                print(f"  {ml:<8} {hw.mean_fps:<6.0f} {hw.mean_cpu_temp:<9.1f} "
                      f"{hw.peak_cpu_temp:<9.1f} {hw.mean_cpu_usage_pct:<9.1f} "
                      f"{hw.peak_cpu_usage_pct:<10.1f}")

    # 4. Speed-controlled comparison across matches
    print(f"\n{'-'*100}")
    print("4. ACCURACY BY ROBOT SPEED (all matches, per-match breakdown)")
    print(f"{'-'*100}")

    speed_bands = [
        ("Stopped(<0.3)", 0.0, 0.3),
        ("Slow(0.3-1)", 0.3, 1.0),
        ("Medium(1-2)", 1.0, 2.0),
        ("Fast(2-4)", 2.0, 4.0),
        ("Sprint(4+)", 4.0, 999),
    ]

    # Overall
    print(f"\n  Overall (all matches combined):")
    print(f"  {'Speed':<15} {'N':<8} {'MedRes':<9} {'Out%':<7} {'MedDist':<9} {'MedTC':<7} {'MedAmb':<8}")
    for label, lo, hi in speed_bands:
        recs = [r for r in all_records if lo <= r["speed_mps"] < hi]
        res = [r["residual"] for r in recs]
        rs = stats_bucket(res)
        if rs["n"] > 0:
            print(f"  {label:<15} {rs['n']:<8} {rs['med']:<9.3f} {rs['out_pct']:<7.1f} "
                  f"{statistics.median([r['avg_tag_dist'] for r in recs]):<9.2f} "
                  f"{statistics.median([r['tag_count'] for r in recs]):<7.0f} "
                  f"{statistics.median([r['max_amb'] for r in recs]):<8.3f}")

    # Per-match at each speed
    print(f"\n  Per-match median residual by speed:")
    print(f"  {'Match':<8}", end="")
    for label, _, _ in speed_bands:
        print(f" {label:<15}", end="")
    print()
    for ml in match_labels:
        print(f"  {ml:<8}", end="")
        for _, lo, hi in speed_bands:
            recs = [r for r in all_records if r["match"] == ml and lo <= r["speed_mps"] < hi]
            if recs:
                med = statistics.median([r["residual"] for r in recs])
                print(f" {med:<15.3f}", end="")
            else:
                print(f" {'-':<15}", end="")
        print()

    # 5. Distance-controlled comparison
    print(f"\n{'-'*100}")
    print("5. ACCURACY BY TAG DISTANCE (per-match)")
    print(f"{'-'*100}")

    dist_bands = [
        ("0-2m", 0.0, 2.0), ("2-3m", 2.0, 3.0),
        ("3-4m", 3.0, 4.0), ("4m+", 4.0, 999),
    ]

    print(f"  {'Match':<8}", end="")
    for label, _, _ in dist_bands:
        print(f" {label:<15}", end="")
    print()
    for ml in match_labels:
        print(f"  {ml:<8}", end="")
        for _, lo, hi in dist_bands:
            recs = [r for r in all_records if r["match"] == ml and lo <= r["avg_tag_dist"] < hi]
            if recs:
                med = statistics.median([r["residual"] for r in recs])
                n = len(recs)
                print(f" {med:.3f}({n:<5})", end="")
            else:
                print(f" {'-':<15}", end="")
        print()

    # 6. Tag count controlled
    print(f"\n{'-'*100}")
    print("6. ACCURACY BY TAG COUNT (per-match)")
    print(f"{'-'*100}")

    print(f"  {'Match':<8} {'TC=1 med':<12} {'TC=1 n':<8} {'TC=2 med':<12} {'TC=2 n':<8} "
          f"{'TC=3+ med':<12} {'TC=3+ n':<8}")
    for ml in match_labels:
        parts = []
        for tc_lo, tc_hi in [(1, 2), (2, 3), (3, 99)]:
            recs = [r for r in all_records if r["match"] == ml and tc_lo <= r["tag_count"] < tc_hi]
            if recs:
                med = statistics.median([r["residual"] for r in recs])
                parts.append(f"{med:<12.3f}{len(recs):<8}")
            else:
                parts.append(f"{'-':<12}{0:<8}")
        print(f"  {ml:<8} {''.join(parts)}")

    # 7. LL-a vs LL-b comparison controlling for confounds
    print(f"\n{'-'*100}")
    print("7. LIMELIGHT-A vs LIMELIGHT-B (controlling for distance and tag count)")
    print(f"{'-'*100}")

    for tc in [1, 2]:
        print(f"\n  Tag count = {tc}:")
        print(f"  {'Dist':<8} {'LL-A med':<10} {'LL-A n':<8} {'LL-A amb':<10} "
              f"{'LL-B med':<10} {'LL-B n':<8} {'LL-B amb':<10} {'Better'}")
        for dlabel, dlo, dhi in dist_bands:
            a_recs = [r for r in all_records if r["camera"] == "limelight-a"
                      and r["tag_count"] == tc and dlo <= r["avg_tag_dist"] < dhi]
            b_recs = [r for r in all_records if r["camera"] == "limelight-b"
                      and r["tag_count"] == tc and dlo <= r["avg_tag_dist"] < dhi]
            a_res = [r["residual"] for r in a_recs]
            b_res = [r["residual"] for r in b_recs]
            a_s = stats_bucket(a_res)
            b_s = stats_bucket(b_res)
            a_amb = statistics.mean([r["max_amb"] for r in a_recs]) if a_recs else 0
            b_amb = statistics.mean([r["max_amb"] for r in b_recs]) if b_recs else 0
            a_m = a_s.get("med")
            b_m = b_s.get("med")
            better = "A" if a_m and b_m and a_m < b_m else "B" if a_m and b_m else "-"
            print(f"  {dlabel:<8} {a_m if a_m else '-':>9} {a_s['n']:<8} {a_amb:>9.3f} "
                  f"{b_m if b_m else '-':>9} {b_s['n']:<8} {b_amb:>9.3f} {better:>7}")

    # 8. Latency comparison between cameras
    print(f"\n{'-'*100}")
    print("8. LATENCY COMPARISON (ll-a vs ll-b per match)")
    print(f"{'-'*100}")
    print(f"  {'Match':<8} {'LL-A medLat':<12} {'LL-A p95Lat':<12} "
          f"{'LL-B medLat':<12} {'LL-B p95Lat':<12}")
    for ml in match_labels:
        a_lat = [r["latency_ms"] for r in all_records if r["match"] == ml and r["camera"] == "limelight-a"]
        b_lat = [r["latency_ms"] for r in all_records if r["match"] == ml and r["camera"] == "limelight-b"]
        a_med = statistics.median(a_lat) if a_lat else 0
        a_p95 = sorted(a_lat)[int(len(a_lat)*0.95)] if len(a_lat) > 1 else 0
        b_med = statistics.median(b_lat) if b_lat else 0
        b_p95 = sorted(b_lat)[int(len(b_lat)*0.95)] if len(b_lat) > 1 else 0
        print(f"  {ml:<8} {a_med:<12.1f} {a_p95:<12.1f} {b_med:<12.1f} {b_p95:<12.1f}")

    # 9. What explains Q7 and Q25?
    print(f"\n{'-'*100}")
    print("9. DEEP DIVE: Q7 AND Q25 vs GOOD MATCHES (Q4, Q40)")
    print(f"{'-'*100}")

    for compare in [("Q7", "Q4"), ("Q25", "Q40")]:
        bad, good = compare
        print(f"\n  --- {bad} (bad) vs {good} (good) ---")

        for ml in [bad, good]:
            recs = [r for r in all_records if r["match"] == ml]
            if not recs:
                continue
            res = [r["residual"] for r in recs]
            rs = stats_bucket(res)
            print(f"\n  {ml}: {fmt(rs)}")
            print(f"    Speed: mean={statistics.mean([r['speed_mps'] for r in recs]):.2f}, "
                  f"med={statistics.median([r['speed_mps'] for r in recs]):.2f}")
            print(f"    Dist:  mean={statistics.mean([r['avg_tag_dist'] for r in recs]):.2f}, "
                  f"med={statistics.median([r['avg_tag_dist'] for r in recs]):.2f}")
            print(f"    TC:    mean={statistics.mean([r['tag_count'] for r in recs]):.2f}, "
                  f"med={statistics.median([r['tag_count'] for r in recs]):.0f}")
            print(f"    Amb:   mean={statistics.mean([r['max_amb'] for r in recs]):.3f}, "
                  f"med={statistics.median([r['max_amb'] for r in recs]):.3f}")
            print(f"    Lat:   mean={statistics.mean([r['latency_ms'] for r in recs]):.1f}, "
                  f"med={statistics.median([r['latency_ms'] for r in recs]):.1f}")

            # Breakdown by camera
            for cam in cameras:
                cr = [r for r in recs if r["camera"] == cam]
                if not cr:
                    continue
                crs = stats_bucket([r["residual"] for r in cr])
                print(f"    {cam}: {fmt(crs)}")
                print(f"      dist={statistics.mean([r['avg_tag_dist'] for r in cr]):.2f}m, "
                      f"TC={statistics.mean([r['tag_count'] for r in cr]):.1f}, "
                      f"amb={statistics.mean([r['max_amb'] for r in cr]):.3f}, "
                      f"spd={statistics.mean([r['speed_mps'] for r in cr]):.1f}m/s, "
                      f"lat={statistics.mean([r['latency_ms'] for r in cr]):.1f}ms")

            # At matched distance (2-3m), how do they compare?
            for dlabel, dlo, dhi in [("0-2m", 0, 2), ("2-3m", 2, 3), ("3-4m", 3, 4)]:
                dr = [r for r in recs if dlo <= r["avg_tag_dist"] < dhi]
                if dr:
                    drs = stats_bucket([r["residual"] for r in dr])
                    print(f"    At {dlabel}: {fmt(drs)}, TC={statistics.mean([r['tag_count'] for r in dr]):.1f}, "
                          f"amb={statistics.mean([r['max_amb'] for r in dr]):.3f}")

    # 10. Does the RawVision (fusion output) agree with raw LL data?
    print(f"\n{'-'*100}")
    print("10. RAW LL POSES vs FUSED POSE (RawVision) vs ODOMETRY")
    print(f"{'-'*100}")

    for filepath in sorted(LOG_DIR.glob("FRC_*_WASAM_Q*.wpilog")):
        m2 = re.search(r"WASAM_(Q\d+)", filepath.name)
        if not m2:
            continue
        ml = m2.group(1)
        ld = read_wpilog(str(filepath))
        sources = discover_pose_sources(ld)
        ref = build_reference_path(ld, sources)
        if not ref:
            continue

        print(f"\n  {ml}:")
        for src in sources:
            if src.samples:
                m3 = compute_divergence_metrics(src.samples, ref)
                name_short = src.name.replace("NT:/SmartDashboard/Field/", "").replace("NT:/", "")
                print(f"    {name_short:<25s} ({src.source_class:<9s}): "
                      f"RMS={m3.translation_rms_m:.3f}m, Max={m3.translation_max_m:.2f}m, "
                      f"n={m3.sample_count}")

    # 11. Match-to-match change detection with confound awareness
    print(f"\n{'-'*100}")
    print("11. MATCH-TO-MATCH CHANGES WITH CONFOUND CONTEXT")
    print(f"{'-'*100}")

    prev_ml = None
    for ml in match_labels:
        if prev_ml is None:
            prev_ml = ml
            continue

        curr = [r for r in all_records if r["match"] == ml]
        prev = [r for r in all_records if r["match"] == prev_ml]

        if not curr or not prev:
            prev_ml = ml
            continue

        curr_med = statistics.median([r["residual"] for r in curr])
        prev_med = statistics.median([r["residual"] for r in prev])
        delta_pct = (curr_med - prev_med) / max(prev_med, 0.001) * 100

        if abs(delta_pct) > 30:
            direction = "IMPROVED" if delta_pct < 0 else "DEGRADED"
            print(f"\n  {prev_ml} -> {ml}: {direction} by {abs(delta_pct):.0f}%")
            print(f"    Median residual: {prev_med:.3f}m -> {curr_med:.3f}m")

            # Check confound shifts
            for var, label in [("speed_mps", "Speed"), ("avg_tag_dist", "Distance"),
                               ("tag_count", "Tag count"), ("max_amb", "Ambiguity"),
                               ("latency_ms", "Latency")]:
                p = statistics.median([r[var] for r in prev])
                c = statistics.median([r[var] for r in curr])
                if var in ("speed_mps", "avg_tag_dist", "latency_ms"):
                    if abs(c - p) > 0.3:
                        print(f"    {label}: {p:.2f} -> {c:.2f}")
                elif var == "tag_count":
                    if abs(c - p) > 0.3:
                        print(f"    {label}: {p:.1f} -> {c:.1f}")
                elif var == "max_amb":
                    if abs(c - p) > 0.05:
                        print(f"    {label}: {p:.3f} -> {c:.3f}")

            # Per-camera detail
            for cam in cameras:
                pc = [r for r in prev if r["camera"] == cam]
                cc = [r for r in curr if r["camera"] == cam]
                if pc and cc:
                    pm = statistics.median([r["residual"] for r in pc])
                    cm = statistics.median([r["residual"] for r in cc])
                    pl = statistics.median([r["latency_ms"] for r in pc])
                    cl = statistics.median([r["latency_ms"] for r in cc])
                    print(f"    {cam.replace('limelight-','ll-')}: "
                          f"med {pm:.3f} -> {cm:.3f}m, lat {pl:.1f} -> {cl:.1f}ms")

        prev_ml = ml

    # 12. Best and worst windows with confound data
    print(f"\n{'-'*100}")
    print("12. BEST AND WORST 5s WINDOWS WITH CONFOUND CONTEXT")
    print(f"{'-'*100}")

    for ml in match_labels:
        recs = sorted([r for r in all_records if r["match"] == ml],
                      key=lambda r: r.get("_ts", 0))  # already ordered by append order

        if len(recs) < 20:
            continue

        # Assign approximate ordering index
        for i, r in enumerate(recs):
            r["_idx"] = i

        # Window of ~50 consecutive frames
        windows = []
        step = max(1, len(recs) // 50)
        for start in range(0, len(recs) - 20, step):
            win = recs[start:start + 50]
            if len(win) < 20:
                continue
            res = [r["residual"] for r in win]
            windows.append({
                "start_idx": start,
                "n": len(win),
                "med_res": statistics.median(res),
                "mean_res": statistics.mean(res),
                "out_pct": sum(1 for r in res if r > 1.0) / len(res) * 100,
                "med_speed": statistics.median([r["speed_mps"] for r in win]),
                "med_dist": statistics.median([r["avg_tag_dist"] for r in win]),
                "med_tc": statistics.median([r["tag_count"] for r in win]),
                "med_amb": statistics.median([r["max_amb"] for r in win]),
                "phase": win[len(win)//2]["phase"],
            })

        if not windows:
            continue

        windows.sort(key=lambda w: w["med_res"])
        print(f"\n  {ml}:")
        print(f"    BEST:")
        for w in windows[:2]:
            print(f"      med={w['med_res']:.3f}m, out%={w['out_pct']:.0f}%, "
                  f"spd={w['med_speed']:.1f}, dist={w['med_dist']:.1f}m, "
                  f"TC={w['med_tc']:.0f}, amb={w['med_amb']:.3f}, phase={w['phase']}")
        print(f"    WORST:")
        for w in windows[-2:]:
            print(f"      med={w['med_res']:.3f}m, out%={w['out_pct']:.0f}%, "
                  f"spd={w['med_speed']:.1f}, dist={w['med_dist']:.1f}m, "
                  f"TC={w['med_tc']:.0f}, amb={w['med_amb']:.3f}, phase={w['phase']}")

    print(f"\n{'='*100}")
    print("ANALYSIS COMPLETE")
    print(f"{'='*100}")


if __name__ == "__main__":
    main()
