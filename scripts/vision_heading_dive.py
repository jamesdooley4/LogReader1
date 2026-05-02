"""Investigate robot diving toward red side during auto at timestamps 381-385 and 530-535.

Extracts:
- DriveState pose (fused / estimated pose)
- ModuleStates (wheel speeds/angles)
- Limelight botpose (MT1 and MT2) from all cameras
- robot_orientation_set (heading supplied to LLs)
- Vision reject reasons
- Pigeon2 yaw (raw gyro)
"""

import sys
import math
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from logreader.wpilog_reader import read_wpilog
from logreader.models import LogData, SignalType

LOG_PATH = r"D:\Temp\2026-04-07_programming\FRC_20260407_231323.wpilog"

# Time windows of interest (seconds from log start)
WINDOWS = [
    ("Incident 1", 379.0, 387.0),
    ("Incident 2", 528.0, 537.0),
]

# Signals of interest (regex-ish patterns for manual matching)
INTEREST_PATTERNS = [
    "DriveState",
    "ModuleStates",
    "botpose_wpiblue",
    "botpose_orb_wpiblue",
    "robot_orientation_set",
    "imumode_set",
    "imu",
    "Pigeon",
    "yaw",
    "Heading",
    "rejectReason",
    "visionError",
    "tagSpread",
    "STD",
    "underDefense",
    "Pose",
    "robotPose",
    "EstimatedPose",
    "rawfiducials",
    "tv",
    "tid",
]


def main():
    print(f"Reading {LOG_PATH}...")
    log_data = read_wpilog(LOG_PATH)
    print(f"Loaded {len(log_data.signals)} signals")

    # First, list all signal names so we know what we're working with
    print("\n=== ALL SIGNAL NAMES ===")
    for name in sorted(log_data.signals.keys()):
        sig = log_data.signals[name]
        n = len(sig.values)
        print(f"  {name:80s}  type={sig.info.type.name:15s}  samples={n}")

    # Find time base
    all_timestamps = []
    for sig in log_data.signals.values():
        if sig.values:
            all_timestamps.append(sig.values[0].timestamp_us)
    if not all_timestamps:
        print("No data!")
        return
    t0_us = min(all_timestamps)
    print(f"\nLog start (t0): {t0_us} us")

    # Filter signals of interest
    interesting = {}
    for name, sig in log_data.signals.items():
        for pat in INTEREST_PATTERNS:
            if pat.lower() in name.lower():
                interesting[name] = sig
                break

    print(f"\n=== INTERESTING SIGNALS ({len(interesting)}) ===")
    for name in sorted(interesting.keys()):
        sig = interesting[name]
        print(f"  {name:80s}  type={sig.info.type.name:15s}  samples={len(sig.values)}")

    # Extract data in each window
    for label, t_start, t_end in WINDOWS:
        start_us = t0_us + int(t_start * 1e6)
        end_us = t0_us + int(t_end * 1e6)
        print(f"\n{'='*100}")
        print(f"=== {label}: {t_start:.1f}s - {t_end:.1f}s ===")
        print(f"{'='*100}")

        for name in sorted(interesting.keys()):
            sig = interesting[name]
            window_vals = [
                v for v in sig.values
                if start_us <= v.timestamp_us <= end_us
            ]
            if not window_vals:
                continue

            print(f"\n--- {name} ({len(window_vals)} samples in window) ---")
            
            # For arrays, show structured
            if sig.info.type == SignalType.DOUBLE_ARRAY:
                for v in window_vals:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    arr = v.value
                    if isinstance(arr, (list, tuple)):
                        # For botpose: x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgDist, avgArea
                        if "botpose" in name.lower() and len(arr) >= 11:
                            x, y, z = arr[0], arr[1], arr[2]
                            yaw = arr[5]
                            latency = arr[6]
                            tag_count = int(arr[7])
                            avg_dist = arr[9] if len(arr) > 9 else 0
                            print(f"  t={t_rel:8.3f}s  x={x:7.3f} y={y:7.3f} yaw={yaw:7.1f}° tags={tag_count} avgDist={avg_dist:.2f}m lat={latency:.1f}ms")
                        elif "modulestates" in name.lower():
                            # Module states: typically [speed0, angle0, speed1, angle1, ...]
                            short = [f"{v:.2f}" for v in arr[:16]]
                            print(f"  t={t_rel:8.3f}s  {short}")
                        elif "robot_orientation" in name.lower():
                            # Heading sent to LL: [yaw, yawRate, pitch, pitchRate, roll, rollRate]
                            if len(arr) >= 1:
                                print(f"  t={t_rel:8.3f}s  heading={arr[0]:7.1f}°  full={[f'{v:.2f}' for v in arr[:6]]}")
                        elif "rawfiducials" in name.lower():
                            # Each tag: [id, txnc, tync, area, distCam, distRobot, ambiguity]
                            i = 0
                            tags = []
                            while i + 6 < len(arr):
                                tag_id = int(arr[i])
                                ambig = arr[i+6]
                                dist = arr[i+4]
                                tags.append(f"id={tag_id} d={dist:.2f}m a={ambig:.3f}")
                                i += 7
                            print(f"  t={t_rel:8.3f}s  tags=[{', '.join(tags)}]")
                        elif "imu" in name.lower():
                            # LL IMU data
                            short = [f"{v:.2f}" for v in arr[:6]]
                            print(f"  t={t_rel:8.3f}s  {short}")
                        elif "stddevs" in name.lower():
                            short = [f"{v:.4f}" for v in arr[:12]]
                            print(f"  t={t_rel:8.3f}s  {short}")
                        elif "pose" in name.lower() or "Pose" in name:
                            if len(arr) >= 3:
                                print(f"  t={t_rel:8.3f}s  x={arr[0]:7.3f} y={arr[1]:7.3f} yaw={arr[2] if len(arr)>2 else '?'}°")
                            else:
                                print(f"  t={t_rel:8.3f}s  {arr}")
                        else:
                            short = [f"{v:.3f}" for v in arr[:8]]
                            print(f"  t={t_rel:8.3f}s  {short}")
                    else:
                        print(f"  t={t_rel:8.3f}s  {v.value}")
            elif sig.info.type == SignalType.STRING:
                for v in window_vals:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    print(f"  t={t_rel:8.3f}s  {v.value}")
            elif sig.info.type in (SignalType.DOUBLE, SignalType.FLOAT):
                for v in window_vals:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    print(f"  t={t_rel:8.3f}s  {v.value:.4f}")
            elif sig.info.type == SignalType.INTEGER:
                for v in window_vals:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    print(f"  t={t_rel:8.3f}s  {v.value}")
            elif sig.info.type == SignalType.BOOLEAN:
                for v in window_vals:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    print(f"  t={t_rel:8.3f}s  {v.value}")
            else:
                # Struct or other
                for v in window_vals[:5]:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    print(f"  t={t_rel:8.3f}s  {type(v.value).__name__}: {v.value}")

    # Also show a pre-incident baseline
    print(f"\n{'='*100}")
    print(f"=== BASELINE: 370-379s (before incident 1) ===")
    print(f"{'='*100}")
    start_us = t0_us + int(370 * 1e6)
    end_us = t0_us + int(379 * 1e6)
    
    # Just show pose-related signals in baseline
    for name in sorted(interesting.keys()):
        if not any(p in name.lower() for p in ["drivestate", "botpose_wpiblue", "robot_orientation", "reject", "pose"]):
            continue
        sig = interesting[name]
        window_vals = [v for v in sig.values if start_us <= v.timestamp_us <= end_us]
        if not window_vals:
            continue
        # Show only first and last few
        show = window_vals[:3] + window_vals[-3:] if len(window_vals) > 6 else window_vals
        print(f"\n--- {name} ({len(window_vals)} in window, showing {len(show)}) ---")
        for v in show:
            t_rel = (v.timestamp_us - t0_us) / 1e6
            if sig.info.type == SignalType.DOUBLE_ARRAY:
                arr = v.value
                if isinstance(arr, (list, tuple)):
                    if "botpose" in name.lower() and len(arr) >= 11:
                        x, y, yaw = arr[0], arr[1], arr[5]
                        tag_count = int(arr[7])
                        print(f"  t={t_rel:8.3f}s  x={x:7.3f} y={y:7.3f} yaw={yaw:7.1f}° tags={tag_count}")
                    elif "robot_orientation" in name.lower() and len(arr) >= 1:
                        print(f"  t={t_rel:8.3f}s  heading={arr[0]:7.1f}°")
                    elif "pose" in name.lower():
                        if len(arr) >= 3:
                            print(f"  t={t_rel:8.3f}s  x={arr[0]:7.3f} y={arr[1]:7.3f} yaw={arr[2] if len(arr)>2 else '?'}")
                        else:
                            print(f"  t={t_rel:8.3f}s  {arr}")
                    else:
                        print(f"  t={t_rel:8.3f}s  {arr[:6]}")
            elif sig.info.type == SignalType.STRING:
                print(f"  t={t_rel:8.3f}s  {v.value}")
            else:
                print(f"  t={t_rel:8.3f}s  {v.value}")


if __name__ == "__main__":
    main()
