"""Compare MT2 yaw performance before/after LL IMU rate and vision fix changes.

Groups:
  A: Before both changes (Q4-Q44, day 1)
  B: After IMU rate change, before vision fix (Q49, Q56, E4)
  C: After both changes (E8-E16)

Analyzes: pre-auto (30s before), auto, and teleop separately.
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

# Group assignment based on timestamps
# Change 1 (IMU rate): 2026-03-22 09:58:28 -0700 = 16:58 UTC = between Q49(1627) and Q56(1734)
# Change 2 (vision fix): 2026-03-22 13:22:49 -0700 = 20:22 UTC = between E4(2015) and E8(2058)
GROUP_A_MATCHES = {"Q4", "Q7", "Q14", "Q17", "Q21", "Q25", "Q32", "Q37", "Q40", "Q44", "Q49"}
GROUP_B_MATCHES = {"Q56", "E4"}
GROUP_C_MATCHES = {"E8", "E10", "E12", "E13", "E14", "E15", "E16"}


def extract_match_label(filename: str) -> str | None:
    m = re.search(r"WASAM_((?:Q|E)\d+)", filename)
    return m.group(1) if m else None


def stats_for(vals: list[float]) -> dict:
    if not vals:
        return {"n": 0, "med": 0, "mean": 0, "p95": 0, "max": 0}
    s = sorted(vals)
    return {
        "n": len(s),
        "med": statistics.median(s),
        "mean": statistics.mean(s),
        "p95": s[int(len(s) * 0.95)] if len(s) > 1 else s[0],
        "max": s[-1],
        "pct_over_5": sum(1 for v in s if v > 5) / len(s) * 100,
        "pct_over_15": sum(1 for v in s if v > 15) / len(s) * 100,
        "pct_over_30": sum(1 for v in s if v > 30) / len(s) * 100,
    }


def fmt(s: dict) -> str:
    if s["n"] == 0:
        return "no data"
    return (f"n={s['n']}, med={s['med']:.1f}deg, mean={s['mean']:.1f}deg, "
            f">15deg={s.get('pct_over_15',0):.1f}%, >30deg={s.get('pct_over_30',0):.1f}%")


def get_group(match_label: str) -> str:
    if match_label in GROUP_A_MATCHES:
        return "A (before both)"
    elif match_label in GROUP_B_MATCHES:
        return "B (after IMU rate)"
    elif match_label in GROUP_C_MATCHES:
        return "C (after vision fix)"
    return "unknown"


def main():
    wpilog_files = sorted(LOG_DIR.glob("FRC_*_WASAM_*.wpilog"))
    print(f"Found {len(wpilog_files)} match files\n")

    all_match_data = []

    for filepath in wpilog_files:
        match_label = extract_match_label(filepath.name)
        if not match_label:
            continue

        group = get_group(match_label)
        print(f"Loading {match_label} (group {group})...")

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

        # Find auto start for pre-auto window
        auto_intervals = timeline.intervals_for(MatchPhase.AUTONOMOUS)
        auto_start_us = auto_intervals[0].start_us if auto_intervals else None
        auto_end_us = auto_intervals[0].end_us if auto_intervals else None

        teleop_intervals = timeline.intervals_for(MatchPhase.TELEOP)
        teleop_start_us = teleop_intervals[0].start_us if teleop_intervals else None
        teleop_end_us = teleop_intervals[0].end_us if teleop_intervals else None

        base_us = timeline.intervals[0].start_us if timeline.intervals else 0

        # Categorize frames by phase
        pre_auto_frames = []  # 30s before auto start
        auto_frames = []
        teleop_frames = []

        for f in all_frames:
            if f.mt1_mt2_yaw_diff_deg is None:
                continue

            if auto_start_us and auto_start_us - 30_000_000 <= f.timestamp_us < auto_start_us:
                pre_auto_frames.append(f)
            elif auto_start_us and auto_end_us and auto_start_us <= f.timestamp_us < auto_end_us:
                auto_frames.append(f)
            elif teleop_start_us and teleop_end_us and teleop_start_us <= f.timestamp_us < teleop_end_us:
                teleop_frames.append(f)

        # Per-camera stats for each phase
        cameras = sorted(all_cameras.keys())
        camera_phases = {}
        for cam in cameras:
            camera_phases[cam] = {}
            for phase_name, phase_frames in [
                ("pre_auto", pre_auto_frames),
                ("auto", auto_frames),
                ("teleop", teleop_frames),
            ]:
                cf = [f for f in phase_frames if f.camera == cam]
                yaw_diffs = [abs(f.mt1_mt2_yaw_diff_deg) for f in cf]
                mt1_res = [f.pose_residual_m for f in cf if f.pose_residual_m is not None]
                mt2_res = [f.mt2_residual_m for f in cf if f.mt2_residual_m is not None]

                camera_phases[cam][phase_name] = {
                    "yaw": stats_for(yaw_diffs),
                    "mt1_res": stats_for(mt1_res),
                    "mt2_res": stats_for(mt2_res),
                }

        auto_dur = (auto_end_us - auto_start_us) / 1e6 if auto_start_us and auto_end_us else 0
        teleop_dur = (teleop_end_us - teleop_start_us) / 1e6 if teleop_start_us and teleop_end_us else 0

        all_match_data.append({
            "match": match_label,
            "group": group,
            "cameras": cameras,
            "camera_phases": camera_phases,
            "auto_dur": auto_dur,
            "teleop_dur": teleop_dur,
        })

    # =========================================================================
    # Per-match summary table
    # =========================================================================
    print(f"\n{'='*120}")
    print("PER-MATCH MT2 YAW ERROR BY PHASE")
    print(f"{'='*120}")

    print(f"\n{'Match':<8} {'Group':<22} {'Camera':<10} {'AutoDur':<8} {'TeleDur':<8} "
          f"{'PreAuto':<12} {'PreAuto>15':<11} "
          f"{'AutoMedYaw':<11} {'Auto>15%':<9} "
          f"{'TeleMedYaw':<11} {'Tele>15%':<9} "
          f"{'AutoMT2res':<11} {'TeleMT2res':<11}")

    for md in all_match_data:
        for cam in md["cameras"]:
            cp = md["camera_phases"][cam]
            pa = cp["pre_auto"]["yaw"]
            au = cp["auto"]["yaw"]
            te = cp["teleop"]["yaw"]
            au_mt2 = cp["auto"]["mt2_res"]
            te_mt2 = cp["teleop"]["mt2_res"]

            cam_short = cam.replace("limelight-", "ll-")
            pa_med = f"{pa['med']:.1f}" if pa["n"] > 0 else "-"
            pa_pct = f"{pa.get('pct_over_15',0):.0f}%" if pa["n"] > 0 else "-"
            au_med = f"{au['med']:.1f}" if au["n"] > 0 else "-"
            au_pct = f"{au.get('pct_over_15',0):.0f}%" if au["n"] > 0 else "-"
            te_med = f"{te['med']:.1f}" if te["n"] > 0 else "-"
            te_pct = f"{te.get('pct_over_15',0):.0f}%" if te["n"] > 0 else "-"
            au_mt2_med = f"{au_mt2['med']:.3f}" if au_mt2["n"] > 0 else "-"
            te_mt2_med = f"{te_mt2['med']:.3f}" if te_mt2["n"] > 0 else "-"

            print(f"{md['match']:<8} {md['group']:<22} {cam_short:<10} "
                  f"{md['auto_dur']:<8.1f} {md['teleop_dur']:<8.1f} "
                  f"{pa_med:<12} {pa_pct:<11} "
                  f"{au_med:<11} {au_pct:<9} "
                  f"{te_med:<11} {te_pct:<9} "
                  f"{au_mt2_med:<11} {te_mt2_med:<11}")

    # =========================================================================
    # Group comparison
    # =========================================================================
    print(f"\n{'='*120}")
    print("GROUP COMPARISON: AGGREGATE MT2 YAW ERROR BY PHASE")
    print(f"{'='*120}")

    groups = ["A (before both)", "B (after IMU rate)", "C (after vision fix)"]
    cameras_all = sorted(set(cam for md in all_match_data for cam in md["cameras"]))

    for cam in cameras_all:
        cam_short = cam.replace("limelight-", "ll-")
        print(f"\n  {cam_short}:")
        print(f"  {'Group':<25} {'Phase':<10} {'N':<7} {'MedYawErr':<11} {'MeanYawErr':<11} "
              f"{'P95YawErr':<11} {'>15deg%':<9} {'>30deg%':<9}")

        for group in groups:
            group_matches = [md for md in all_match_data if md["group"] == group]
            for phase_name in ["pre_auto", "auto", "teleop"]:
                all_yaw = []
                for md in group_matches:
                    if cam in md["camera_phases"]:
                        cp = md["camera_phases"][cam][phase_name]
                        # Reconstruct individual values from the matches
                        # We need to re-collect from raw data
                        pass

                # Simpler: just collect from match stats
                n_total = sum(md["camera_phases"].get(cam, {}).get(phase_name, {}).get("yaw", {}).get("n", 0)
                              for md in group_matches)
                if n_total == 0:
                    continue

                # Weighted stats from per-match medians
                meds = []
                means = []
                p95s = []
                pct15s = []
                pct30s = []
                ns = []
                for md in group_matches:
                    if cam not in md["camera_phases"]:
                        continue
                    ys = md["camera_phases"][cam][phase_name]["yaw"]
                    if ys["n"] > 0:
                        meds.append(ys["med"])
                        means.append(ys["mean"])
                        p95s.append(ys["p95"])
                        pct15s.append(ys.get("pct_over_15", 0))
                        pct30s.append(ys.get("pct_over_30", 0))
                        ns.append(ys["n"])

                if not meds:
                    continue

                # Weighted averages
                total_n = sum(ns)
                w_med = sum(m * n for m, n in zip(meds, ns)) / total_n
                w_mean = sum(m * n for m, n in zip(means, ns)) / total_n
                w_p95 = sum(m * n for m, n in zip(p95s, ns)) / total_n
                w_15 = sum(m * n for m, n in zip(pct15s, ns)) / total_n
                w_30 = sum(m * n for m, n in zip(pct30s, ns)) / total_n

                print(f"  {group:<25} {phase_name:<10} {total_n:<7} "
                      f"{w_med:<11.1f} {w_mean:<11.1f} {w_p95:<11.1f} "
                      f"{w_15:<9.1f} {w_30:<9.1f}")

    # =========================================================================
    # MT2 residual group comparison
    # =========================================================================
    print(f"\n{'='*120}")
    print("GROUP COMPARISON: MT2 RESIDUAL (position accuracy) BY PHASE")
    print(f"{'='*120}")

    for cam in cameras_all:
        cam_short = cam.replace("limelight-", "ll-")
        print(f"\n  {cam_short}:")
        print(f"  {'Group':<25} {'Phase':<10} {'N':<7} {'MT1medRes':<11} {'MT2medRes':<11} "
              f"{'MT2/MT1':<9}")

        for group in groups:
            group_matches = [md for md in all_match_data if md["group"] == group]
            for phase_name in ["pre_auto", "auto", "teleop"]:
                mt1_meds = []
                mt2_meds = []
                ns = []
                for md in group_matches:
                    if cam not in md["camera_phases"]:
                        continue
                    mt1s = md["camera_phases"][cam][phase_name]["mt1_res"]
                    mt2s = md["camera_phases"][cam][phase_name]["mt2_res"]
                    if mt1s["n"] > 0 and mt2s["n"] > 0:
                        mt1_meds.append(mt1s["med"])
                        mt2_meds.append(mt2s["med"])
                        ns.append(mt1s["n"])

                if not mt1_meds:
                    continue

                total_n = sum(ns)
                w_mt1 = sum(m * n for m, n in zip(mt1_meds, ns)) / total_n
                w_mt2 = sum(m * n for m, n in zip(mt2_meds, ns)) / total_n
                ratio = w_mt2 / w_mt1 if w_mt1 > 0.001 else 0

                print(f"  {group:<25} {phase_name:<10} {total_n:<7} "
                      f"{w_mt1:<11.3f} {w_mt2:<11.3f} {ratio:<9.2f}")

    # =========================================================================
    # Detailed per-match pre-auto analysis
    # =========================================================================
    print(f"\n{'='*120}")
    print("PRE-AUTO 30s DETAIL (what was the LL IMU doing before auto?)")
    print(f"{'='*120}")

    for md in all_match_data:
        has_pre = any(
            md["camera_phases"].get(cam, {}).get("pre_auto", {}).get("yaw", {}).get("n", 0) > 0
            for cam in md["cameras"]
        )
        if not has_pre:
            continue

        print(f"\n  {md['match']} ({md['group']}):")
        for cam in md["cameras"]:
            cp = md["camera_phases"].get(cam, {}).get("pre_auto", {})
            ys = cp.get("yaw", {"n": 0})
            if ys["n"] == 0:
                continue
            cam_short = cam.replace("limelight-", "ll-")
            print(f"    {cam_short}: {fmt(ys)}")

    # =========================================================================
    # Change impact summary
    # =========================================================================
    print(f"\n{'='*120}")
    print("CHANGE IMPACT SUMMARY")
    print(f"{'='*120}")

    for cam in cameras_all:
        cam_short = cam.replace("limelight-", "ll-")
        print(f"\n  {cam_short}:")

        for phase_name in ["auto", "teleop"]:
            for ga, gb, change_name in [
                ("A (before both)", "B (after IMU rate)", "IMU rate change"),
                ("B (after IMU rate)", "C (after vision fix)", "Vision fix"),
                ("A (before both)", "C (after vision fix)", "Both changes combined"),
            ]:
                a_matches = [md for md in all_match_data if md["group"] == ga]
                b_matches = [md for md in all_match_data if md["group"] == gb]

                def weighted_median_yaw(matches):
                    meds = []
                    ns = []
                    for md in matches:
                        if cam in md["camera_phases"]:
                            ys = md["camera_phases"][cam][phase_name]["yaw"]
                            if ys["n"] > 0:
                                meds.append(ys["med"])
                                ns.append(ys["n"])
                    if not meds:
                        return None, 0
                    total = sum(ns)
                    return sum(m * n for m, n in zip(meds, ns)) / total, total

                a_med, a_n = weighted_median_yaw(a_matches)
                b_med, b_n = weighted_median_yaw(b_matches)

                if a_med is not None and b_med is not None and a_n > 0 and b_n > 0:
                    delta = b_med - a_med
                    direction = "IMPROVED" if delta < 0 else "DEGRADED" if delta > 0 else "SAME"
                    pct = abs(delta) / max(a_med, 0.1) * 100
                    print(f"    {phase_name} {change_name}: "
                          f"{a_med:.1f}deg -> {b_med:.1f}deg ({direction} by {pct:.0f}%)")

    print(f"\n{'='*120}")
    print("DONE")
    print(f"{'='*120}")


if __name__ == "__main__":
    main()
