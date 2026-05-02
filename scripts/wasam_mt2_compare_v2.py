"""Compare MT2 performance before/after IMU rate and vision fix changes.

Uses parallel processing for data loading. Analyzes:
- Pre-auto (5s before auto enable)
- Auto phase
- Teleop phase

Groups:
  A: Before both changes (Q4-Q49)
  B: After IMU rate change, before vision fix (Q56, E4)
  C: After both changes (E8-E16)
"""

import sys
import math
import statistics
import re
import time
import os
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed

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

GROUP_A = {"Q4", "Q7", "Q14", "Q17", "Q21", "Q25", "Q32", "Q37", "Q40", "Q44", "Q49"}
GROUP_B = {"Q56", "E4"}
GROUP_C = {"E8", "E10", "E12", "E13", "E14", "E15", "E16"}


def get_group(ml: str) -> str:
    if ml in GROUP_A: return "A (before both)"
    if ml in GROUP_B: return "B (after IMU rate)"
    if ml in GROUP_C: return "C (after vision fix)"
    return "?"


def extract_label(fn: str) -> str | None:
    m = re.search(r"WASAM_((?:Q|E)\d+)", fn)
    return m.group(1) if m else None


def stats_for(vals: list[float]) -> dict:
    if not vals:
        return {"n": 0}
    s = sorted(vals)
    return {
        "n": len(s), "med": statistics.median(s), "mean": statistics.mean(s),
        "p95": s[int(len(s) * 0.95)] if len(s) > 1 else s[0],
        "pct_gt15": sum(1 for v in s if v > 15) / len(s) * 100,
        "pct_gt30": sum(1 for v in s if v > 30) / len(s) * 100,
    }


def process_match(filepath: str) -> dict | None:
    """Process one match file. Returns structured per-phase MT2 data."""
    label = extract_label(Path(filepath).name)
    if not label:
        return None

    ld = read_wpilog(filepath)
    tl = detect_match_phases(ld)
    if tl is None:
        tl = MatchPhaseTimeline()

    cams = _discover_cameras(ld)
    frames: list[VisionFrame] = []
    for cn, sigs in cams.items():
        f, _ = _parse_frames(cn, sigs)
        frames.extend(f)
    _compute_mt1_mt2_divergence(frames, cams)
    _compute_residuals(frames, ld)

    auto_ivs = tl.intervals_for(MatchPhase.AUTONOMOUS)
    teleop_ivs = tl.intervals_for(MatchPhase.TELEOP)
    auto_start = auto_ivs[0].start_us if auto_ivs else None
    auto_end = auto_ivs[0].end_us if auto_ivs else None
    teleop_start = teleop_ivs[0].start_us if teleop_ivs else None
    teleop_end = teleop_ivs[0].end_us if teleop_ivs else None

    # Also get robot_orientation_set and imumode_set at auto start
    orient_at_auto = None
    imumode_at_auto = None
    init_heading = None

    if auto_start:
        robot_pose = ld.get_signal("NT:/Pose/robotPose")
        if robot_pose:
            for tv in robot_pose.values:
                if auto_start <= tv.timestamp_us <= auto_start + 500_000:
                    arr = tv.value
                    if isinstance(arr, (list, tuple)) and len(arr) >= 3:
                        init_heading = float(arr[2])
                    break

        for cam_name in cams:
            orient_sig = ld.get_signal(f"NT:/{cam_name}/robot_orientation_set")
            if orient_sig:
                for tv in orient_sig.values:
                    if auto_start - 100_000 <= tv.timestamp_us <= auto_start + 500_000:
                        arr = tv.value
                        if isinstance(arr, (list, tuple)):
                            orient_at_auto = float(arr[0])
                        break
            imu_sig = ld.get_signal(f"NT:/{cam_name}/imumode_set")
            if imu_sig:
                for tv in imu_sig.values:
                    if auto_start - 500_000 <= tv.timestamp_us <= auto_start + 500_000:
                        imumode_at_auto = float(tv.value)
                        break
            if orient_at_auto is not None:
                break

    # Classify frames by phase
    result = {
        "match": label,
        "group": get_group(label),
        "init_heading": init_heading,
        "orient_at_auto": orient_at_auto,
        "imumode_at_auto": imumode_at_auto,
        "auto_dur": (auto_end - auto_start) / 1e6 if auto_start and auto_end else 0,
        "teleop_dur": (teleop_end - teleop_start) / 1e6 if teleop_start and teleop_end else 0,
        "cameras": {},
    }

    for cam in sorted(cams.keys()):
        cam_data = {}
        for phase_name, start, end in [
            ("pre_auto", (auto_start - 5_000_000) if auto_start else None, auto_start),
            ("auto", auto_start, auto_end),
            ("teleop", teleop_start, teleop_end),
        ]:
            if start is None or end is None:
                cam_data[phase_name] = {
                    "yaw": {"n": 0},
                    "mt1_res": {"n": 0},
                    "mt2_res": {"n": 0},
                    "yaw_values": [],
                    "mt1_res_values": [],
                    "mt2_res_values": [],
                }
                continue

            phase_frames = [
                f for f in frames
                if f.camera == cam and start <= f.timestamp_us < end
                and f.mt1_mt2_yaw_diff_deg is not None
            ]

            yaw_diffs = [abs(f.mt1_mt2_yaw_diff_deg) for f in phase_frames]
            mt1_res = [f.pose_residual_m for f in phase_frames if f.pose_residual_m is not None]
            mt2_res = [f.mt2_residual_m for f in phase_frames if f.mt2_residual_m is not None]

            cam_data[phase_name] = {
                "yaw": stats_for(yaw_diffs),
                "mt1_res": stats_for(mt1_res),
                "mt2_res": stats_for(mt2_res),
                "yaw_values": yaw_diffs,
                "mt1_res_values": mt1_res,
                "mt2_res_values": mt2_res,
            }

        result["cameras"][cam] = cam_data

    return result


def main():
    files = sorted(LOG_DIR.glob("FRC_*_WASAM_*.wpilog"))
    print(f"Found {len(files)} match files")

    # Process in parallel
    workers = min(os.cpu_count() or 1, len(files), 8)
    print(f"Processing with {workers} workers...")
    t0 = time.monotonic()

    all_data = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(process_match, str(fp)): fp for fp in files}
        done = 0
        for future in as_completed(futures):
            done += 1
            fp = futures[future]
            try:
                result = future.result()
                if result:
                    all_data.append(result)
                    print(f"  [{done}/{len(files)}] {result['match']} ({result['group']}) -- done",
                          file=sys.stderr, flush=True)
            except Exception as e:
                print(f"  [{done}/{len(files)}] {fp.name} -- ERROR: {e}", file=sys.stderr)

    elapsed = time.monotonic() - t0
    print(f"Loaded {len(all_data)} matches in {elapsed:.1f}s\n")

    all_data.sort(key=lambda d: d["match"])

    # =========================================================================
    print(f"{'='*130}")
    print("PER-MATCH MT2 YAW ERROR BY PHASE (tag_count >= 1 only)")
    print(f"{'='*130}")

    print(f"\n{'Match':<8} {'Group':<22} {'Cam':<8} {'InitHd':<8} {'Orient':<8} {'IMUmode':<8} "
          f"{'PreAutoMedY':<12} {'AutoMedYaw':<11} {'Auto>15%':<9} "
          f"{'TeleMedYaw':<11} {'Tele>15%':<9} "
          f"{'AutoMT2res':<11} {'TeleMT2res':<11}")

    for d in all_data:
        for cam, cp in sorted(d["cameras"].items()):
            cam_short = cam.replace("limelight-", "ll-")
            pa = cp["pre_auto"]["yaw"]
            au = cp["auto"]["yaw"]
            te = cp["teleop"]["yaw"]
            au_r = cp["auto"]["mt2_res"]
            te_r = cp["teleop"]["mt2_res"]

            h = f"{d['init_heading']:.0f}" if d["init_heading"] is not None else "-"
            o = f"{d['orient_at_auto']:.0f}" if d["orient_at_auto"] is not None else "-"
            im = f"{d['imumode_at_auto']:.0f}" if d["imumode_at_auto"] is not None else "-"
            pa_m = f"{pa['med']:.1f}" if pa["n"] > 0 else "-"
            au_m = f"{au['med']:.1f}" if au["n"] > 0 else "-"
            au_p = f"{au.get('pct_gt15',0):.0f}%" if au["n"] > 0 else "-"
            te_m = f"{te['med']:.1f}" if te["n"] > 0 else "-"
            te_p = f"{te.get('pct_gt15',0):.0f}%" if te["n"] > 0 else "-"
            ar = f"{au_r['med']:.3f}" if au_r["n"] > 0 else "-"
            tr = f"{te_r['med']:.3f}" if te_r["n"] > 0 else "-"

            print(f"{d['match']:<8} {d['group']:<22} {cam_short:<8} {h:<8} {o:<8} {im:<8} "
                  f"{pa_m:<12} {au_m:<11} {au_p:<9} "
                  f"{te_m:<11} {te_p:<9} "
                  f"{ar:<11} {tr:<11}")

    # =========================================================================
    print(f"\n{'='*130}")
    print("GROUP COMPARISON: AGGREGATE BY PHASE")
    print(f"{'='*130}")

    groups = ["A (before both)", "B (after IMU rate)", "C (after vision fix)"]
    cameras_all = sorted(set(cam for d in all_data for cam in d["cameras"]))

    for cam in cameras_all:
        cam_short = cam.replace("limelight-", "ll-")
        print(f"\n  {cam_short}:")
        print(f"  {'Group':<25} {'Phase':<12} {'N':<7} {'MedYaw':<9} {'MeanYaw':<9} "
              f"{'P95Yaw':<9} {'>15%':<7} {'>30%':<7} "
              f"{'MT1res':<9} {'MT2res':<9} {'MT2/MT1':<8}")

        for group in groups:
            gm = [d for d in all_data if d["group"] == group]
            for phase in ["pre_auto", "auto", "teleop"]:
                all_yaw = []
                all_mt1 = []
                all_mt2 = []
                for d in gm:
                    if cam not in d["cameras"]:
                        continue
                    pdata = d["cameras"][cam][phase]
                    all_yaw.extend(pdata["yaw_values"])
                    all_mt1.extend(pdata["mt1_res_values"])
                    all_mt2.extend(pdata["mt2_res_values"])

                if not all_yaw:
                    continue

                yaw_stats = stats_for(all_yaw)
                total_n = yaw_stats["n"]

                mt1_med = "-"
                mt2_med = "-"
                ratio = "-"
                if all_mt1:
                    m1v = statistics.median(all_mt1)
                    mt1_med = f"{m1v:.3f}"
                if all_mt2:
                    m2v = statistics.median(all_mt2)
                    mt2_med = f"{m2v:.3f}"
                if all_mt1 and all_mt2:
                    ratio = f"{m2v / max(m1v, 0.001):.2f}"

                print(f"  {group:<25} {phase:<12} {total_n:<7} "
                      f"{yaw_stats['med']:<9.1f} {yaw_stats['mean']:<9.1f} {yaw_stats['p95']:<9.1f} "
                      f"{yaw_stats.get('pct_gt15', 0):<7.1f} {yaw_stats.get('pct_gt30', 0):<7.1f} "
                      f"{mt1_med:<9} {mt2_med:<9} {ratio:<8}")

    # =========================================================================
    print(f"\n{'='*130}")
    print("CHANGE IMPACT SUMMARY")
    print(f"{'='*130}")

    for cam in cameras_all:
        cam_short = cam.replace("limelight-", "ll-")
        print(f"\n  {cam_short}:")

        for phase in ["pre_auto", "auto", "teleop"]:
            for ga, gb, change_name in [
                ("A (before both)", "B (after IMU rate)", "IMU rate change"),
                ("B (after IMU rate)", "C (after vision fix)", "Vision fix"),
                ("A (before both)", "C (after vision fix)", "Both combined"),
            ]:
                def weighted_med(group_name):
                    gm = [d for d in all_data if d["group"] == group_name]
                    vals = []
                    for d in gm:
                        if cam in d["cameras"]:
                            vals.extend(d["cameras"][cam][phase]["yaw_values"])
                    if not vals:
                        return None, 0
                    return statistics.median(vals), len(vals)

                a_m, a_n = weighted_med(ga)
                b_m, b_n = weighted_med(gb)
                if a_m is not None and b_m is not None and a_n > 0 and b_n > 0:
                    delta = b_m - a_m
                    direction = "IMPROVED" if delta < -1 else "DEGRADED" if delta > 1 else "~same"
                    pct = abs(delta) / max(a_m, 0.1) * 100
                    print(f"    {phase:<12} {change_name:<20}: "
                          f"{a_m:.1f} -> {b_m:.1f} deg ({direction}, {pct:.0f}%)")

    print(f"\n{'='*130}")
    print("DONE")
    print(f"{'='*130}")


if __name__ == "__main__":
    main()
