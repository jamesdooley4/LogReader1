"""Check initial robot heading at auto start vs MT2 yaw error.

For each match, reads DriveState/Pose or Pose/robotPose heading right
after auto starts, and correlates with the MT2 yaw error magnitude.
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
from logreader.models import LogData, SignalType
from logreader.analyzers.vision_analysis import (
    VisionFrame, _discover_cameras, _parse_frames,
    _compute_mt1_mt2_divergence,
)
from logreader.analyzers.match_phases import (
    detect_match_phases, MatchPhase, MatchPhaseTimeline,
)

LOG_DIR = Path(r"D:\Temp\2026_WASAM")


def extract_match_label(filename: str) -> str | None:
    m = re.search(r"WASAM_((?:Q|E)\d+)", filename)
    return m.group(1) if m else None


def get_group(match_label: str) -> str:
    GROUP_A = {"Q4", "Q7", "Q14", "Q17", "Q21", "Q25", "Q32", "Q37", "Q40", "Q44", "Q49"}
    GROUP_B = {"Q56", "E4"}
    GROUP_C = {"E8", "E10", "E12", "E13", "E14", "E15", "E16"}
    if match_label in GROUP_A:
        return "A"
    elif match_label in GROUP_B:
        return "B"
    elif match_label in GROUP_C:
        return "C"
    return "?"


def main():
    wpilog_files = sorted(LOG_DIR.glob("FRC_*_WASAM_*.wpilog"))
    print(f"{'Match':<8} {'Group':<6} {'InitHead':<10} {'IMU rYaw':<10} {'IMU iYaw':<10} "
          f"{'OrientSet':<10} {'LL-A autoYawErr':<16} {'LL-B autoYawErr':<16} "
          f"{'LL-A MT1yaw':<12} {'LL-A MT2yaw':<12}")

    results = []

    for filepath in wpilog_files:
        match_label = extract_match_label(filepath.name)
        if not match_label:
            continue

        group = get_group(match_label)
        ld = read_wpilog(str(filepath))
        tl = detect_match_phases(ld)
        if tl is None:
            tl = MatchPhaseTimeline()

        auto_ivs = tl.intervals_for(MatchPhase.AUTONOMOUS)
        if not auto_ivs:
            continue
        auto_start = auto_ivs[0].start_us
        auto_end = auto_ivs[0].end_us

        # Get robot heading right after auto starts (first 0.5s)
        init_heading = None
        robot_pose_sig = ld.get_signal("NT:/Pose/robotPose")
        if robot_pose_sig:
            for tv in robot_pose_sig.values:
                if auto_start <= tv.timestamp_us <= auto_start + 500_000:
                    arr = tv.value
                    if isinstance(arr, (list, tuple)) and len(arr) >= 3:
                        init_heading = float(arr[2])
                        break

        # Get LL-A IMU robot_yaw at auto start
        imu_robot_yaw = None
        imu_int_yaw = None
        imu_sig = ld.get_signal("NT:/limelight-a/imu")
        if imu_sig:
            for tv in imu_sig.values:
                if auto_start <= tv.timestamp_us <= auto_start + 500_000:
                    arr = tv.value
                    if isinstance(arr, (list, tuple)) and len(arr) >= 4:
                        imu_robot_yaw = float(arr[0])
                        imu_int_yaw = float(arr[3])
                        break

        # Get robot_orientation_set value at auto start
        orient_yaw = None
        orient_sig = ld.get_signal("NT:/limelight-a/robot_orientation_set")
        if orient_sig:
            for tv in orient_sig.values:
                if auto_start - 100_000 <= tv.timestamp_us <= auto_start + 500_000:
                    arr = tv.value
                    if isinstance(arr, (list, tuple)) and len(arr) >= 1:
                        orient_yaw = float(arr[0])
                        break

        # Get MT2 yaw error during auto (first 5s)
        all_cameras = _discover_cameras(ld)
        all_frames: list[VisionFrame] = []
        for cam_name, signals in all_cameras.items():
            frames, _ = _parse_frames(cam_name, signals)
            all_frames.extend(frames)
        _compute_mt1_mt2_divergence(all_frames, all_cameras)

        # Auto frames in first 5 seconds
        auto_early = [f for f in all_frames
                      if auto_start <= f.timestamp_us <= auto_start + 5_000_000
                      and f.mt1_mt2_yaw_diff_deg is not None]

        lla_yaw_errs = [abs(f.mt1_mt2_yaw_diff_deg) for f in auto_early if f.camera == "limelight-a"]
        llb_yaw_errs = [abs(f.mt1_mt2_yaw_diff_deg) for f in auto_early if f.camera == "limelight-b"]

        lla_med = statistics.median(lla_yaw_errs) if lla_yaw_errs else None
        llb_med = statistics.median(llb_yaw_errs) if llb_yaw_errs else None

        # Get first MT1 and MT2 yaw values in auto
        lla_mt1_yaw = None
        lla_mt2_yaw = None
        for f in sorted(auto_early, key=lambda f: f.timestamp_us):
            if f.camera == "limelight-a":
                lla_mt1_yaw = f.yaw_deg
                if f.mt2_yaw_deg is not None:
                    lla_mt2_yaw = f.mt2_yaw_deg
                break

        h = f"{init_heading:.1f}" if init_heading is not None else "-"
        ir = f"{imu_robot_yaw:.1f}" if imu_robot_yaw is not None else "-"
        ii = f"{imu_int_yaw:.1f}" if imu_int_yaw is not None else "-"
        oy = f"{orient_yaw:.1f}" if orient_yaw is not None else "-"
        ae = f"{lla_med:.1f}" if lla_med is not None else "-"
        be = f"{llb_med:.1f}" if llb_med is not None else "-"
        mt1 = f"{lla_mt1_yaw:.1f}" if lla_mt1_yaw is not None else "-"
        mt2 = f"{lla_mt2_yaw:.1f}" if lla_mt2_yaw is not None else "-"

        print(f"{match_label:<8} {group:<6} {h:<10} {ir:<10} {ii:<10} "
              f"{oy:<10} {ae:<16} {be:<16} {mt1:<12} {mt2:<12}")

        results.append({
            "match": match_label,
            "group": group,
            "init_heading": init_heading,
            "imu_robot_yaw": imu_robot_yaw,
            "imu_int_yaw": imu_int_yaw,
            "orient_yaw": orient_yaw,
            "lla_auto_yaw_err": lla_med,
            "llb_auto_yaw_err": llb_med,
            "lla_mt1_yaw": lla_mt1_yaw,
            "lla_mt2_yaw": lla_mt2_yaw,
        })

    # Correlation analysis
    print(f"\n{'='*100}")
    print("CORRELATION: Initial Heading vs MT2 Auto Yaw Error")
    print(f"{'='*100}")

    print(f"\n{'Match':<8} {'Group':<6} {'InitHead':<10} {'|Head|':<8} "
          f"{'orient_set':<12} {'|Head-Orient|':<14} "
          f"{'IMU rYaw':<10} {'|Head-IMU|':<12} "
          f"{'AutoYawErr':<12}")

    for r in results:
        h = r["init_heading"]
        oy = r["orient_yaw"]
        ir = r["imu_robot_yaw"]
        err = r["lla_auto_yaw_err"]

        abs_h = abs(h) if h is not None else None
        h_orient_diff = abs(h - oy) if h is not None and oy is not None else None
        h_imu_diff = abs(h - ir) if h is not None and ir is not None else None

        ah = f"{abs_h:.1f}" if abs_h is not None else "-"
        hod = f"{h_orient_diff:.1f}" if h_orient_diff is not None else "-"
        hid = f"{h_imu_diff:.1f}" if h_imu_diff is not None else "-"
        e = f"{err:.1f}" if err is not None else "-"
        h_str = f"{h:.1f}" if h is not None else "-"
        oy_str = f"{oy:.1f}" if oy is not None else "-"
        ir_str = f"{ir:.1f}" if ir is not None else "-"

        print(f"{r['match']:<8} {r['group']:<6} {h_str:<10} {ah:<8} "
              f"{oy_str:<12} {hod:<14} "
              f"{ir_str:<10} {hid:<12} "
              f"{e:<12}")

    # Group A only: does heading magnitude predict error?
    print(f"\n{'='*100}")
    print("GROUP A ONLY: Does |heading| predict auto yaw error?")
    print(f"{'='*100}")

    ga = [r for r in results if r["group"] == "A"
          and r["init_heading"] is not None and r["lla_auto_yaw_err"] is not None]
    ga.sort(key=lambda r: abs(r["init_heading"]))

    print(f"\n{'Match':<8} {'InitHead':<10} {'|Head|':<8} {'AutoYawErr':<12} {'Correlation?'}")
    for r in ga:
        h = r["init_heading"]
        err = r["lla_auto_yaw_err"]
        # The error should roughly be |heading - orient_set| if orient_set is near 0
        expected_err = abs(h - (r["orient_yaw"] or 0))
        match_ok = "~match" if abs(err - expected_err) < 15 else "mismatch"
        print(f"{r['match']:<8} {h:<10.1f} {abs(h):<8.1f} {err:<12.1f} {match_ok}")


if __name__ == "__main__":
    main()
