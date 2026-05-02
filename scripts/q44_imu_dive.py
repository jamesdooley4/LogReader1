"""Q44 Auto deep-dive: botpose_wpiblue vs botpose_orb_wpiblue vs IMU vs DriveState.

Compares the heading (yaw) from all pose sources during Q44 autonomous to
understand why MegaTag2 (orb) was wrong (~67 deg) vs correct (~90 deg).
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

ld = read_wpilog(r"D:\Temp\2026_WASAM\FRC_20260322_020642_WASAM_Q44.wpilog")
tl = detect_match_phases(ld)
if tl is None:
    tl = MatchPhaseTimeline()

print("Match phases:")
for iv in tl.intervals:
    print(f"  {iv.phase.value:12s} {iv.start_s:10.2f}s - {iv.end_s:10.2f}s ({iv.duration_s:.1f}s)")

auto_ivs = tl.intervals_for(MatchPhase.AUTONOMOUS)
if not auto_ivs:
    print("No autonomous phase found!")
    sys.exit(1)

auto_start = auto_ivs[0].start_us
auto_end = auto_ivs[0].end_us
base_us = tl.intervals[0].start_us if tl.intervals else 0

print(f"\nAuto phase: {auto_ivs[0].start_s:.2f}s - {auto_ivs[0].end_s:.2f}s ({auto_ivs[0].duration_s:.1f}s)")

# Find all relevant signals
print("\nAll limelight-a signals:")
for name in sorted(ld.signals.keys()):
    if "limelight-a" in name:
        sig = ld.signals[name]
        print(f"  {name}: {sig.info.type.name}, {len(sig.values)} samples")

print("\nDriveState signals:")
for name in sorted(ld.signals.keys()):
    if "DriveState" in name or "Pose/robot" in name:
        sig = ld.signals[name]
        print(f"  {name}: {sig.info.type.name}, {len(sig.values)} samples")

# =========================================================================
# Extract data during auto
# =========================================================================

def get_auto_samples(sig_name):
    """Get all (timestamp_us, value) during auto."""
    sig = ld.get_signal(sig_name)
    if sig is None:
        return []
    return [(tv.timestamp_us, tv.value) for tv in sig.values
            if auto_start <= tv.timestamp_us <= auto_end]


def print_pose_timeline(sig_name, label, yaw_index=5, has_latency=True):
    """Print x, y, yaw timeline for a botpose-type signal."""
    samples = get_auto_samples(sig_name)
    if not samples:
        print(f"\n{label}: NO DATA during auto")
        return

    print(f"\n{label}: {len(samples)} samples during auto")
    print(f"  {'Time(s)':>8} {'X':>8} {'Y':>8} {'Yaw':>8} {'TagCnt':>7} {'Latency':>8}")

    yaws = []
    for ts, arr in samples:
        if not isinstance(arr, (list, tuple)) or len(arr) < yaw_index + 1:
            continue
        t = (ts - base_us) / 1e6
        x = float(arr[0])
        y = float(arr[1])
        yaw = float(arr[yaw_index])
        tag_count = int(float(arr[7])) if len(arr) > 7 else 0
        latency = float(arr[6]) if len(arr) > 6 and has_latency else 0

        # Skip zero-pose (no tags)
        if tag_count < 1 and has_latency:
            continue

        yaws.append(yaw)
        print(f"  {t:8.2f} {x:8.3f} {y:8.3f} {yaw:8.1f} {tag_count:7d} {latency:8.1f}")

    if yaws:
        print(f"  Yaw stats: mean={statistics.mean(yaws):.1f}, "
              f"med={statistics.median(yaws):.1f}, "
              f"min={min(yaws):.1f}, max={max(yaws):.1f}, "
              f"stdev={statistics.stdev(yaws):.2f}" if len(yaws) > 1 else
              f"  Yaw: {yaws[0]:.1f}")


# botpose_wpiblue (MegaTag1, vision-only)
print_pose_timeline("NT:/limelight-a/botpose_wpiblue", "LL-A botpose_wpiblue (MT1)")

# botpose_orb_wpiblue (MegaTag2, IMU+vision fused)
print_pose_timeline("NT:/limelight-a/botpose_orb_wpiblue", "LL-A botpose_orb_wpiblue (MT2/ORB)")

# Same for LL-B if available
print_pose_timeline("NT:/limelight-b/botpose_wpiblue", "LL-B botpose_wpiblue (MT1)")
print_pose_timeline("NT:/limelight-b/botpose_orb_wpiblue", "LL-B botpose_orb_wpiblue (MT2/ORB)")

# =========================================================================
# IMU data
# =========================================================================
print("\n" + "="*80)
print("LIMELIGHT IMU DATA DURING AUTO")
print("="*80)

imu_samples = get_auto_samples("NT:/limelight-a/imu")
if imu_samples:
    print(f"\nLL-A IMU: {len(imu_samples)} samples during auto")
    # imu: [robot_yaw, roll, pitch, internal_yaw, roll_rate, pitch_rate, yaw_rate, accel_x, accel_y, accel_z]
    print(f"  {'Time(s)':>8} {'RobotYaw':>10} {'Roll':>8} {'Pitch':>8} "
          f"{'IntYaw':>10} {'YawRate':>9} {'AccX':>8} {'AccY':>8} {'AccZ':>8}")

    robot_yaws = []
    int_yaws = []
    for ts, arr in imu_samples:
        if not isinstance(arr, (list, tuple)) or len(arr) < 10:
            continue
        t = (ts - base_us) / 1e6
        robot_yaw = float(arr[0])
        roll = float(arr[1])
        pitch = float(arr[2])
        int_yaw = float(arr[3])
        roll_rate = float(arr[4])
        pitch_rate = float(arr[5])
        yaw_rate = float(arr[6])
        accel_x = float(arr[7])
        accel_y = float(arr[8])
        accel_z = float(arr[9])

        robot_yaws.append(robot_yaw)
        int_yaws.append(int_yaw)

        # Print every ~1 second (subsample)
        idx = len(robot_yaws)
        if idx <= 5 or idx % 50 == 0 or idx == len(imu_samples):
            print(f"  {t:8.2f} {robot_yaw:10.2f} {roll:8.2f} {pitch:8.2f} "
                  f"{int_yaw:10.2f} {yaw_rate:9.2f} {accel_x:8.2f} {accel_y:8.2f} {accel_z:8.2f}")

    if robot_yaws:
        print(f"\n  Robot Yaw stats: mean={statistics.mean(robot_yaws):.1f}, "
              f"range=[{min(robot_yaws):.1f}, {max(robot_yaws):.1f}]")
    if int_yaws:
        print(f"  Internal Yaw stats: mean={statistics.mean(int_yaws):.1f}, "
              f"range=[{min(int_yaws):.1f}, {max(int_yaws):.1f}]")
    if robot_yaws and int_yaws:
        diffs = [r - i for r, i in zip(robot_yaws, int_yaws)]
        print(f"  Robot - Internal Yaw diff: mean={statistics.mean(diffs):.1f}, "
              f"range=[{min(diffs):.1f}, {max(diffs):.1f}]")
else:
    print("\nLL-A IMU: NO DATA during auto")

# LL-B IMU
imu_b_samples = get_auto_samples("NT:/limelight-b/imu")
if imu_b_samples:
    print(f"\nLL-B IMU: {len(imu_b_samples)} samples during auto")
    for ts, arr in imu_b_samples[:3]:
        if isinstance(arr, (list, tuple)) and len(arr) >= 4:
            t = (ts - base_us) / 1e6
            print(f"  t={t:.2f}: robotYaw={float(arr[0]):.2f}, intYaw={float(arr[3]):.2f}")

# =========================================================================
# DriveState/Pose (robot's own estimate)
# =========================================================================
print("\n" + "="*80)
print("ROBOT DRIVESTATE/POSE DURING AUTO")
print("="*80)

# Try struct pose
for sig_name in ["NT:/DriveState/Pose", "NT:/Pose/robotPose", "NT:/SmartDashboard/Field/Robot"]:
    samples = get_auto_samples(sig_name)
    if not samples:
        continue

    sig = ld.get_signal(sig_name)
    print(f"\n{sig_name}: {len(samples)} samples during auto (type={sig.info.type.name})")

    if sig.info.type == SignalType.DOUBLE_ARRAY:
        print(f"  {'Time(s)':>8} {'X':>8} {'Y':>8} {'Heading':>8}")
        headings = []
        for ts, arr in samples:
            if not isinstance(arr, (list, tuple)) or len(arr) < 3:
                continue
            t = (ts - base_us) / 1e6
            x, y, heading = float(arr[0]), float(arr[1]), float(arr[2])
            headings.append(heading)
            idx = len(headings)
            if idx <= 5 or idx % 50 == 0 or ts >= auto_end - 100000:
                print(f"  {t:8.2f} {x:8.3f} {y:8.3f} {heading:8.1f}")
        if headings:
            print(f"  Heading stats: mean={statistics.mean(headings):.1f}, "
                  f"range=[{min(headings):.1f}, {max(headings):.1f}]")
    elif sig.info.type == SignalType.STRUCT:
        print(f"  (struct type, {len(samples)} samples - raw bytes, not parsed)")

# =========================================================================
# Compare yaw values at matched timestamps
# =========================================================================
print("\n" + "="*80)
print("YAW COMPARISON AT KEY MOMENTS")
print("="*80)

# Get all data as lookup dicts
def sig_to_dict(sig_name, value_extractor):
    """Build {timestamp_us: extracted_value} dict."""
    sig = ld.get_signal(sig_name)
    if sig is None:
        return {}
    result = {}
    for tv in sig.values:
        if auto_start <= tv.timestamp_us <= auto_end:
            val = value_extractor(tv.value)
            if val is not None:
                result[tv.timestamp_us] = val
    return result

def extract_botpose_yaw(arr):
    if isinstance(arr, (list, tuple)) and len(arr) > 7 and int(float(arr[7])) >= 1:
        return float(arr[5])
    return None

def extract_botpose_yaw_any(arr):
    """Extract yaw even with 0 tags (for orb which may still have IMU heading)."""
    if isinstance(arr, (list, tuple)) and len(arr) > 5:
        yaw = float(arr[5])
        # Skip zero/uninitialized
        if abs(float(arr[0])) > 0.001 or abs(float(arr[1])) > 0.001:
            return yaw
    return None

def extract_imu_yaw(arr):
    if isinstance(arr, (list, tuple)) and len(arr) >= 4:
        return float(arr[0])  # robot_yaw
    return None

def extract_imu_int_yaw(arr):
    if isinstance(arr, (list, tuple)) and len(arr) >= 4:
        return float(arr[3])  # internal_yaw
    return None

def extract_pose_heading(arr):
    if isinstance(arr, (list, tuple)) and len(arr) >= 3:
        return float(arr[2])
    return None

mt1_yaws = sig_to_dict("NT:/limelight-a/botpose_wpiblue", extract_botpose_yaw)
mt2_yaws = sig_to_dict("NT:/limelight-a/botpose_orb_wpiblue", extract_botpose_yaw_any)
imu_robot_yaws = sig_to_dict("NT:/limelight-a/imu", extract_imu_yaw)
imu_int_yaws = sig_to_dict("NT:/limelight-a/imu", extract_imu_int_yaw)
robot_pose_yaws = sig_to_dict("NT:/Pose/robotPose", extract_pose_heading)

# Find nearest timestamp match
def nearest_val(d, target_ts, max_delta_us=100_000):
    if not d:
        return None
    best_ts = min(d.keys(), key=lambda ts: abs(ts - target_ts))
    if abs(best_ts - target_ts) > max_delta_us:
        return None
    return d[best_ts]

# Print comparison for MT1 timestamps
if mt1_yaws:
    print(f"\n{'Time(s)':>8} {'MT1 yaw':>9} {'MT2 yaw':>9} {'IMU rYaw':>9} "
          f"{'IMU iYaw':>9} {'RobPose':>9} {'MT1-MT2':>9} {'MT1-IMU':>9}")

    for ts in sorted(mt1_yaws.keys()):
        t = (ts - base_us) / 1e6
        mt1 = mt1_yaws[ts]
        mt2 = nearest_val(mt2_yaws, ts)
        imu_r = nearest_val(imu_robot_yaws, ts)
        imu_i = nearest_val(imu_int_yaws, ts)
        rp = nearest_val(robot_pose_yaws, ts)

        mt1_mt2_diff = mt1 - mt2 if mt2 is not None else None
        mt1_imu_diff = mt1 - imu_r if imu_r is not None else None

        print(f"  {t:8.2f} {mt1:9.1f} "
              f"{mt2 if mt2 is not None else '-':>9} "
              f"{imu_r if imu_r is not None else '-':>9} "
              f"{imu_i if imu_i is not None else '-':>9} "
              f"{rp if rp is not None else '-':>9} "
              f"{mt1_mt2_diff if mt1_mt2_diff is not None else '-':>9} "
              f"{mt1_imu_diff if mt1_imu_diff is not None else '-':>9}")

# =========================================================================
# Also check: what was the robot_orientation_set signal?
# =========================================================================
print("\n" + "="*80)
print("ROBOT ORIENTATION SET (if present)")
print("="*80)

for name in sorted(ld.signals.keys()):
    if "orientation" in name.lower() or "imumode" in name.lower():
        sig = ld.signals[name]
        samples = [(tv.timestamp_us, tv.value) for tv in sig.values
                   if auto_start - 5_000_000 <= tv.timestamp_us <= auto_end]
        print(f"\n{name}: {sig.info.type.name}, {len(samples)} samples near auto")
        for ts, val in samples[:20]:
            t = (ts - base_us) / 1e6
            print(f"  t={t:.2f}: {val}")

# Check stddevs comparison
print("\n" + "="*80)
print("STDDEVS: MT1 vs MT2 DURING AUTO")
print("="*80)

stddev_samples = get_auto_samples("NT:/limelight-a/stddevs")
if stddev_samples:
    print(f"\nLL-A stddevs: {len(stddev_samples)} samples")
    # [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
    print(f"  {'Time':>6} {'MT1x':>8} {'MT1y':>8} {'MT1yaw':>8} {'MT2x':>8} {'MT2y':>8} {'MT2yaw':>8}")
    for ts, arr in stddev_samples:
        if isinstance(arr, (list, tuple)) and len(arr) >= 12:
            t = (ts - base_us) / 1e6
            print(f"  {t:6.2f} {float(arr[0]):8.4f} {float(arr[1]):8.4f} {float(arr[5]):8.4f} "
                  f"{float(arr[6]):8.4f} {float(arr[7]):8.4f} {float(arr[11]):8.4f}")

print("\nDONE")
