"""E12 deep-dive: MT1 vs MT2 with strict tag_count > 0 filtering.

Focuses on the auto period and the 30s before, looking at only frames
where tags are actually visible (tag_count >= 1 in the respective signal).
"""

import sys
import math
import statistics
from pathlib import Path

if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from logreader.wpilog_reader import read_wpilog
from logreader.models import LogData, SignalType
from logreader.analyzers.match_phases import detect_match_phases, MatchPhase, MatchPhaseTimeline
from logreader.analyzers.vision_analysis import _find_nearest

ld = read_wpilog(r"D:\Temp\2026_WASAM\FRC_20260322_214928_WASAM_E12.wpilog")
tl = detect_match_phases(ld)
if tl is None:
    tl = MatchPhaseTimeline()

print("Match phases:")
for iv in tl.intervals:
    print(f"  {iv.phase.value:12s} {iv.start_s:10.2f}s - {iv.end_s:10.2f}s ({iv.duration_s:.1f}s)")

auto_ivs = tl.intervals_for(MatchPhase.AUTONOMOUS)
auto_start = auto_ivs[0].start_us if auto_ivs else None
auto_end = auto_ivs[0].end_us if auto_ivs else None

teleop_ivs = tl.intervals_for(MatchPhase.TELEOP)
teleop_start = teleop_ivs[0].start_us if teleop_ivs else None
teleop_end = teleop_ivs[0].end_us if teleop_ivs else None

base_us = tl.intervals[0].start_us if tl.intervals else 0

# Define time windows
windows = []
if auto_start:
    windows.append(("pre_auto_30s", auto_start - 30_000_000, auto_start))
    windows.append(("auto", auto_start, auto_end))
if teleop_start:
    windows.append(("teleop", teleop_start, teleop_end))


def parse_botpose_samples(sig_name: str, start_us: int, end_us: int):
    """Get all botpose samples with tag_count >= 1 in time range."""
    sig = ld.get_signal(sig_name)
    if sig is None:
        return []
    results = []
    for tv in sig.values:
        if start_us <= tv.timestamp_us <= end_us:
            arr = tv.value
            if not isinstance(arr, (list, tuple)) or len(arr) < 8:
                continue
            tag_count = int(float(arr[7]))
            if tag_count < 1:
                continue
            x, y = float(arr[0]), float(arr[1])
            # Skip zero-pose
            if abs(x) < 0.001 and abs(y) < 0.001:
                continue
            yaw = float(arr[5])
            latency = float(arr[6])
            results.append({
                "ts": tv.timestamp_us,
                "t_rel": (tv.timestamp_us - base_us) / 1e6,
                "x": x,
                "y": y,
                "yaw": yaw,
                "tag_count": tag_count,
                "latency": latency,
            })
    return results


# Print robot pose at auto start
robot_pose = ld.get_signal("NT:/Pose/robotPose")
if robot_pose and auto_start:
    for tv in robot_pose.values:
        if auto_start <= tv.timestamp_us <= auto_start + 200_000:
            arr = tv.value
            if isinstance(arr, (list, tuple)) and len(arr) >= 3:
                print(f"\nRobot pose at auto start: x={float(arr[0]):.3f}, y={float(arr[1]):.3f}, heading={float(arr[2]):.1f}")
                break

# Print robot_orientation_set around auto
orient_sig = ld.get_signal("NT:/limelight-a/robot_orientation_set")
if orient_sig and auto_start:
    print("\nrobot_orientation_set around auto start:")
    for tv in orient_sig.values:
        if auto_start - 2_000_000 <= tv.timestamp_us <= auto_start + 2_000_000:
            arr = tv.value
            t = (tv.timestamp_us - base_us) / 1e6
            if isinstance(arr, (list, tuple)):
                print(f"  t={t:.2f}: yaw={float(arr[0]):.2f}, yawRate={float(arr[1]):.2f}")

# Print IMU around auto
imu_sig = ld.get_signal("NT:/limelight-a/imu")
if imu_sig and auto_start:
    print("\nLL-A IMU around auto start (first 5 samples in auto):")
    count = 0
    for tv in imu_sig.values:
        if auto_start <= tv.timestamp_us <= auto_start + 2_000_000:
            arr = tv.value
            if isinstance(arr, (list, tuple)) and len(arr) >= 4:
                t = (tv.timestamp_us - base_us) / 1e6
                print(f"  t={t:.2f}: robotYaw={float(arr[0]):.2f}, intYaw={float(arr[3]):.2f}")
                count += 1
                if count >= 5:
                    break

# Print imumode_set
imumode = ld.get_signal("NT:/limelight-a/imumode_set")
if imumode:
    print("\nimumode_set values:")
    for tv in imumode.values:
        t = (tv.timestamp_us - base_us) / 1e6
        print(f"  t={t:.2f}: {tv.value}")

print(f"\n{'='*100}")
print("MT1 vs MT2 COMPARISON (tag_count >= 1 ONLY)")
print(f"{'='*100}")

for cam_suffix in ["a", "b"]:
    cam = f"limelight-{cam_suffix}"
    mt1_sig_name = f"NT:/{cam}/botpose_wpiblue"
    mt2_sig_name = f"NT:/{cam}/botpose_orb_wpiblue"

    print(f"\n{'='*80}")
    print(f"  Camera: {cam}")
    print(f"{'='*80}")

    for win_name, win_start, win_end in windows:
        mt1_samples = parse_botpose_samples(mt1_sig_name, win_start, win_end)
        mt2_samples = parse_botpose_samples(mt2_sig_name, win_start, win_end)

        print(f"\n  --- {win_name} ---")
        print(f"  MT1 valid frames (tag_count>=1): {len(mt1_samples)}")
        print(f"  MT2 valid frames (tag_count>=1): {len(mt2_samples)}")

        if not mt1_samples and not mt2_samples:
            print(f"  (no data)")
            continue

        # Print MT1 timeline
        if mt1_samples:
            mt1_yaws = [s["yaw"] for s in mt1_samples]
            print(f"  MT1 yaw: med={statistics.median(mt1_yaws):.1f}, "
                  f"mean={statistics.mean(mt1_yaws):.1f}, "
                  f"range=[{min(mt1_yaws):.1f}, {max(mt1_yaws):.1f}]")

        if mt2_samples:
            mt2_yaws = [s["yaw"] for s in mt2_samples]
            print(f"  MT2 yaw: med={statistics.median(mt2_yaws):.1f}, "
                  f"mean={statistics.mean(mt2_yaws):.1f}, "
                  f"range=[{min(mt2_yaws):.1f}, {max(mt2_yaws):.1f}]")

        # Paired comparison: for each MT1 frame, find nearest MT2 frame
        if mt1_samples and mt2_samples:
            yaw_diffs = []
            pos_diffs = []
            for mt1 in mt1_samples:
                # Find nearest MT2 by timestamp
                best_mt2 = None
                best_dt = float("inf")
                for mt2 in mt2_samples:
                    dt = abs(mt2["ts"] - mt1["ts"])
                    if dt < best_dt:
                        best_dt = dt
                        best_mt2 = mt2
                if best_mt2 and best_dt < 100_000:  # within 100ms
                    yd = mt1["yaw"] - best_mt2["yaw"]
                    yd = (yd + 180) % 360 - 180
                    yaw_diffs.append(yd)
                    pd = math.hypot(mt1["x"] - best_mt2["x"], mt1["y"] - best_mt2["y"])
                    pos_diffs.append(pd)

            if yaw_diffs:
                abs_yd = [abs(d) for d in yaw_diffs]
                print(f"  MT1-MT2 yaw diff (paired): "
                      f"med={statistics.median(abs_yd):.1f}deg, "
                      f"mean={statistics.mean(abs_yd):.1f}deg, "
                      f"range=[{min(abs_yd):.1f}, {max(abs_yd):.1f}]")
                print(f"  MT1-MT2 pos diff (paired): "
                      f"med={statistics.median(pos_diffs):.3f}m, "
                      f"mean={statistics.mean(pos_diffs):.3f}m")

        # Detailed frame-by-frame for short windows
        if len(mt1_samples) <= 60 or win_name == "pre_auto_30s":
            print(f"\n  Frame-by-frame ({win_name}):")
            print(f"  {'Time':>8} {'MT1x':>7} {'MT1y':>7} {'MT1yaw':>8} {'MT1tc':>5} "
                  f"{'MT2x':>7} {'MT2y':>7} {'MT2yaw':>8} {'MT2tc':>5} {'YawDiff':>8} {'PosDiff':>8}")

            # Merge MT1 and MT2 by timestamp
            all_ts = sorted(set(s["ts"] for s in mt1_samples + mt2_samples))
            mt1_by_ts = {s["ts"]: s for s in mt1_samples}
            mt2_by_ts = {s["ts"]: s for s in mt2_samples}

            for ts in all_ts:
                t = (ts - base_us) / 1e6
                m1 = mt1_by_ts.get(ts)
                # Find nearest MT2
                m2 = mt2_by_ts.get(ts)
                if m2 is None:
                    best = None
                    best_dt = float("inf")
                    for mt2 in mt2_samples:
                        dt = abs(mt2["ts"] - ts)
                        if dt < best_dt:
                            best_dt = dt
                            best = mt2
                    if best and best_dt < 50_000:
                        m2 = best

                if m1 is None:
                    # MT2-only frame
                    if m2:
                        print(f"  {t:8.2f} {'-':>7} {'-':>7} {'-':>8} {'-':>5} "
                              f"{m2['x']:7.2f} {m2['y']:7.2f} {m2['yaw']:8.1f} {m2['tag_count']:5d}")
                    continue

                if m2:
                    yd = m1["yaw"] - m2["yaw"]
                    yd = (yd + 180) % 360 - 180
                    pd = math.hypot(m1["x"] - m2["x"], m1["y"] - m2["y"])
                    print(f"  {t:8.2f} {m1['x']:7.2f} {m1['y']:7.2f} {m1['yaw']:8.1f} {m1['tag_count']:5d} "
                          f"{m2['x']:7.2f} {m2['y']:7.2f} {m2['yaw']:8.1f} {m2['tag_count']:5d} "
                          f"{abs(yd):8.1f} {pd:8.3f}")
                else:
                    print(f"  {t:8.2f} {m1['x']:7.2f} {m1['y']:7.2f} {m1['yaw']:8.1f} {m1['tag_count']:5d} "
                          f"{'-':>7} {'-':>7} {'-':>8} {'-':>5}")

# Also check: robot pose during auto for reference
print(f"\n{'='*100}")
print("ROBOT POSE (Pose/robotPose) DURING AUTO")
print(f"{'='*100}")

if robot_pose and auto_start and auto_end:
    print(f"  {'Time':>8} {'X':>8} {'Y':>8} {'Heading':>8}")
    count = 0
    for tv in robot_pose.values:
        if auto_start <= tv.timestamp_us <= auto_end:
            arr = tv.value
            if isinstance(arr, (list, tuple)) and len(arr) >= 3:
                t = (tv.timestamp_us - base_us) / 1e6
                count += 1
                if count <= 10 or count % 50 == 0:
                    print(f"  {t:8.2f} {float(arr[0]):8.3f} {float(arr[1]):8.3f} {float(arr[2]):8.1f}")
    print(f"  ... ({count} total samples)")

print("\nDONE")
