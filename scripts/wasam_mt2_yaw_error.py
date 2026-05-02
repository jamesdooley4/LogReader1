"""Analyze MT2 IMU heading error across all WASAM matches.

For each match, during enabled phases only, compares MT1 yaw vs MT2 yaw
to identify when and how badly the LL IMU heading was wrong.
"""

import sys
import math
import statistics
import re
from pathlib import Path

if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from logreader.wpilog_reader import read_wpilog
from logreader.models import LogData
from logreader.analyzers.vision_analysis import (
    VisionFrame, _discover_cameras, _parse_frames,
    _compute_residuals, _compute_mt1_mt2_divergence,
)
from logreader.analyzers.match_phases import (
    detect_match_phases, MatchPhase, MatchPhaseTimeline,
)

LOG_DIR = Path(r"D:\Temp\2026_WASAM")


def main():
    wpilog_files = sorted(LOG_DIR.glob("FRC_*_WASAM_Q*.wpilog"))
    print(f"Found {len(wpilog_files)} match files\n")

    all_match_data = []

    for filepath in wpilog_files:
        m = re.search(r"WASAM_(Q\d+)", filepath.name)
        if not m:
            continue
        match_label = m.group(1)
        print(f"Loading {match_label}...")

        log_data = read_wpilog(str(filepath))
        timeline = detect_match_phases(log_data)
        if timeline is None:
            timeline = MatchPhaseTimeline()

        all_cameras = _discover_cameras(log_data)
        all_frames: list[VisionFrame] = []
        for cam_name, signals in all_cameras.items():
            frames, _ = _parse_frames(cam_name, signals)
            all_frames.extend(frames)

        _compute_mt1_mt2_divergence(all_frames, all_cameras)
        _compute_residuals(all_frames, log_data)

        # Filter to enabled phases only
        enabled_phases = {MatchPhase.AUTONOMOUS, MatchPhase.TELEOP, MatchPhase.TEST}
        enabled_frames = [
            f for f in all_frames
            if timeline.phase_at(f.timestamp_us) in enabled_phases
        ]

        # Get base timestamp for relative times
        base_us = timeline.intervals[0].start_us if timeline.intervals else 0

        # Collect per-camera yaw divergence data
        camera_data = {}
        for cam in sorted(all_cameras.keys()):
            cam_frames = [
                f for f in enabled_frames
                if f.camera == cam and f.mt1_mt2_yaw_diff_deg is not None
            ]
            if not cam_frames:
                continue

            yaw_diffs = [abs(f.mt1_mt2_yaw_diff_deg) for f in cam_frames]
            mt1_res = [f.pose_residual_m for f in cam_frames if f.pose_residual_m is not None]
            mt2_res = [f.mt2_residual_m for f in cam_frames if f.mt2_residual_m is not None]

            # Find worst windows (sliding 2-second windows)
            cam_frames_sorted = sorted(cam_frames, key=lambda f: f.timestamp_us)
            worst_windows = []
            window_us = 2_000_000
            i = 0
            while i < len(cam_frames_sorted):
                t_start = cam_frames_sorted[i].timestamp_us
                win = []
                j = i
                while j < len(cam_frames_sorted) and cam_frames_sorted[j].timestamp_us < t_start + window_us:
                    win.append(cam_frames_sorted[j])
                    j += 1
                if len(win) >= 3:
                    yd = [abs(f.mt1_mt2_yaw_diff_deg) for f in win]
                    med_yd = statistics.median(yd)
                    phase = timeline.phase_at(win[len(win)//2].timestamp_us).value
                    t_rel = (t_start - base_us) / 1e6
                    worst_windows.append({
                        "t_start_s": t_rel,
                        "phase": phase,
                        "n": len(win),
                        "med_yaw_diff": med_yd,
                        "mean_yaw_diff": statistics.mean(yd),
                        "max_yaw_diff": max(yd),
                    })
                # Advance by 1 second
                next_t = t_start + 1_000_000
                while i < len(cam_frames_sorted) and cam_frames_sorted[i].timestamp_us < next_t:
                    i += 1

            camera_data[cam] = {
                "n": len(cam_frames),
                "yaw_diffs": yaw_diffs,
                "med_yaw": statistics.median(yaw_diffs),
                "mean_yaw": statistics.mean(yaw_diffs),
                "p95_yaw": sorted(yaw_diffs)[int(len(yaw_diffs) * 0.95)] if len(yaw_diffs) > 1 else yaw_diffs[0],
                "max_yaw": max(yaw_diffs),
                "pct_over_15": sum(1 for d in yaw_diffs if d > 15) / len(yaw_diffs) * 100,
                "pct_over_30": sum(1 for d in yaw_diffs if d > 30) / len(yaw_diffs) * 100,
                "pct_over_45": sum(1 for d in yaw_diffs if d > 45) / len(yaw_diffs) * 100,
                "mt1_med_res": statistics.median(mt1_res) if mt1_res else None,
                "mt2_med_res": statistics.median(mt2_res) if mt2_res else None,
                "worst_windows": sorted(worst_windows, key=lambda w: w["med_yaw_diff"], reverse=True),
            }

        all_match_data.append({
            "match": match_label,
            "timeline": timeline,
            "camera_data": camera_data,
            "enabled_frames": enabled_frames,
            "base_us": base_us,
        })

    # =========================================================================
    # Summary table
    # =========================================================================
    print(f"\n{'='*100}")
    print("MT2 YAW ERROR SUMMARY (enabled phases only)")
    print(f"{'='*100}")

    print(f"\n{'Match':<8} {'Camera':<14} {'N':<7} {'MedYawErr':<11} {'MeanYawErr':<11} "
          f"{'P95YawErr':<11} {'MaxYawErr':<11} {'>15deg%':<9} {'>30deg%':<9} {'>45deg%':<9} "
          f"{'MT1medRes':<10} {'MT2medRes':<10}")

    for md in all_match_data:
        for cam, cd in sorted(md["camera_data"].items()):
            flag = " !!!" if cd["med_yaw"] > 15 else " !" if cd["med_yaw"] > 5 else ""
            mt1r = f"{cd['mt1_med_res']:.3f}" if cd['mt1_med_res'] is not None else "-"
            mt2r = f"{cd['mt2_med_res']:.3f}" if cd['mt2_med_res'] is not None else "-"
            print(f"{md['match']:<8} {cam.replace('limelight-','ll-'):<14} "
                  f"{cd['n']:<7} {cd['med_yaw']:<11.1f} {cd['mean_yaw']:<11.1f} "
                  f"{cd['p95_yaw']:<11.1f} {cd['max_yaw']:<11.1f} "
                  f"{cd['pct_over_15']:<9.1f} {cd['pct_over_30']:<9.1f} {cd['pct_over_45']:<9.1f} "
                  f"{mt1r:<10} {mt2r:<10}{flag}")

    # =========================================================================
    # Worst segments
    # =========================================================================
    print(f"\n{'='*100}")
    print("WORST SEGMENTS BY MATCH (2-second windows, enabled only)")
    print(f"{'='*100}")

    for md in all_match_data:
        has_bad = False
        for cam, cd in sorted(md["camera_data"].items()):
            worst = [w for w in cd["worst_windows"] if w["med_yaw_diff"] > 10]
            if worst:
                if not has_bad:
                    print(f"\n  {md['match']}:")
                    has_bad = True
                print(f"    {cam.replace('limelight-','ll-')}:")
                for w in worst[:5]:
                    print(f"      t={w['t_start_s']:.1f}s ({w['phase']}): "
                          f"medYawErr={w['med_yaw_diff']:.1f}deg, "
                          f"maxYawErr={w['max_yaw_diff']:.1f}deg, "
                          f"n={w['n']}")

    # =========================================================================
    # Per-phase breakdown
    # =========================================================================
    print(f"\n{'='*100}")
    print("AUTO vs TELEOP YAW ERROR COMPARISON")
    print(f"{'='*100}")

    print(f"\n{'Match':<8} {'Camera':<14} "
          f"{'AutoN':<7} {'AutoMedYaw':<11} {'Auto>15%':<9} "
          f"{'TeleN':<7} {'TeleMedYaw':<11} {'Tele>15%':<9}")

    for md in all_match_data:
        tl = md["timeline"]
        for cam, cd in sorted(md["camera_data"].items()):
            cam_enabled = [
                f for f in md["enabled_frames"]
                if f.camera == cam.replace("ll-", "limelight-") and f.mt1_mt2_yaw_diff_deg is not None
            ]

            auto_yd = [abs(f.mt1_mt2_yaw_diff_deg) for f in cam_enabled
                       if tl.phase_at(f.timestamp_us) == MatchPhase.AUTONOMOUS]
            tele_yd = [abs(f.mt1_mt2_yaw_diff_deg) for f in cam_enabled
                       if tl.phase_at(f.timestamp_us) == MatchPhase.TELEOP]

            auto_med = statistics.median(auto_yd) if auto_yd else 0
            auto_pct = sum(1 for d in auto_yd if d > 15) / max(len(auto_yd), 1) * 100
            tele_med = statistics.median(tele_yd) if tele_yd else 0
            tele_pct = sum(1 for d in tele_yd if d > 15) / max(len(tele_yd), 1) * 100

            print(f"{md['match']:<8} {cam.replace('limelight-','ll-'):<14} "
                  f"{len(auto_yd):<7} {auto_med:<11.1f} {auto_pct:<9.1f} "
                  f"{len(tele_yd):<7} {tele_med:<11.1f} {tele_pct:<9.1f}")

    # =========================================================================
    # Timeline of yaw error evolution per match
    # =========================================================================
    print(f"\n{'='*100}")
    print("YAW ERROR EVOLUTION OVER TIME (5-second bins, enabled only)")
    print(f"{'='*100}")

    for md in all_match_data:
        tl = md["timeline"]
        base_us = md["base_us"]

        for cam, cd in sorted(md["camera_data"].items()):
            if cd["med_yaw"] < 3:
                continue  # Skip cameras with consistently low error

            cam_frames = sorted(
                [f for f in md["enabled_frames"]
                 if f.camera == cam and f.mt1_mt2_yaw_diff_deg is not None],
                key=lambda f: f.timestamp_us
            )
            if not cam_frames:
                continue

            print(f"\n  {md['match']} {cam.replace('limelight-','ll-')} (med={cd['med_yaw']:.1f}deg):")
            print(f"  {'Time':>8} {'Phase':>8} {'N':>5} {'MedYawErr':>10} {'MeanYawErr':>11} "
                  f"{'MaxYawErr':>10} {'>15deg%':>8}")

            # Bin into 5-second windows
            enabled_start = min(f.timestamp_us for f in cam_frames)
            enabled_end = max(f.timestamp_us for f in cam_frames)
            bin_us = 5_000_000

            t = enabled_start
            while t < enabled_end:
                bin_frames = [f for f in cam_frames if t <= f.timestamp_us < t + bin_us]
                if len(bin_frames) >= 3:
                    yd = [abs(f.mt1_mt2_yaw_diff_deg) for f in bin_frames]
                    t_rel = (t - base_us) / 1e6
                    phase = tl.phase_at(t + bin_us // 2).value
                    med = statistics.median(yd)
                    mean = statistics.mean(yd)
                    mx = max(yd)
                    pct15 = sum(1 for d in yd if d > 15) / len(yd) * 100
                    flag = " <-- BAD" if med > 20 else ""
                    print(f"  {t_rel:8.1f} {phase:>8} {len(bin_frames):5d} "
                          f"{med:10.1f} {mean:11.1f} {mx:10.1f} {pct15:8.1f}{flag}")
                t += bin_us

    # =========================================================================
    # Ranking
    # =========================================================================
    print(f"\n{'='*100}")
    print("MATCHES RANKED BY SEVERITY OF MT2 YAW ERROR")
    print(f"{'='*100}")

    rankings = []
    for md in all_match_data:
        for cam, cd in md["camera_data"].items():
            rankings.append({
                "match": md["match"],
                "camera": cam.replace("limelight-", "ll-"),
                "med_yaw": cd["med_yaw"],
                "pct_over_15": cd["pct_over_15"],
                "n": cd["n"],
            })

    rankings.sort(key=lambda r: r["med_yaw"], reverse=True)
    print(f"\n{'Rank':<6} {'Match':<8} {'Camera':<14} {'MedYawErr':<11} {'>15deg%':<9} {'Frames':<8}")
    for i, r in enumerate(rankings, 1):
        print(f"{i:<6} {r['match']:<8} {r['camera']:<14} {r['med_yaw']:<11.1f} "
              f"{r['pct_over_15']:<9.1f} {r['n']:<8}")

    print("\nDONE")


if __name__ == "__main__":
    main()
