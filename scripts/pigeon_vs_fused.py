"""Extract Pigeon2 raw yaw from hoot wpilog and compare to fused heading.

Streams through the large hoot wpilog files without loading everything.
"""
from wpiutil.log import DataLogReader
import struct
import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

HOOT_FILES = [
    r"D:\Temp\2026-04-07_programming\hoot_235323.wpilog",
    r"D:\Temp\2026-04-07_programming\hoot_235323.2.wpilog",
    r"D:\Temp\2026-04-07_programming\hoot_235323.3.wpilog",
]

WPILOG_PATH = r"D:\Temp\2026-04-07_programming\FRC_20260407_235325.wpilog"

# Time window (AdvantageScope 0-based seconds -> microseconds)
T_START_US = int(225e6)
T_END_US = int(250e6)


def extract_pigeon_yaw():
    """Extract Pigeon2 yaw from hoot wpilog files."""
    yaw_samples = []  # (timestamp_us, yaw_deg)

    for path in HOOT_FILES:
        if not os.path.exists(path):
            continue
        print(f"Scanning {os.path.basename(path)}...")
        reader = DataLogReader(path)

        # First pass: find the entry ID for Pigeon2/Yaw
        entry_map = {}
        yaw_entry_id = None
        timestamp_entry_id = None

        for record in reader:
            if record.isStart():
                data = record.getStartData()
                entry_map[data.entry] = data.name
                if data.name == "Phoenix6/Pigeon2-10/Yaw":
                    yaw_entry_id = data.entry
                if data.name == "Phoenix6/Pigeon2-10/AngularVelocityZWorld":
                    timestamp_entry_id = data.entry
            elif not record.isControl():
                # Once we've seen all start records, check if we have our target
                if yaw_entry_id is not None:
                    break

        if yaw_entry_id is None:
            print(f"  No Pigeon2/Yaw in {path}")
            continue

        print(f"  Found Pigeon2/Yaw at entry {yaw_entry_id}")

        # Second pass: extract yaw values in our time window
        reader2 = DataLogReader(path)
        count = 0
        for record in reader2:
            if record.isControl() or record.isStart():
                continue

            ts = record.getTimestamp()

            # Skip records outside our window (with margin)
            if ts < T_START_US - 5_000_000:
                continue
            if ts > T_END_US + 5_000_000:
                break

            if record.getEntry() == yaw_entry_id:
                try:
                    yaw = record.getDouble()
                    yaw_samples.append((ts, yaw))
                    count += 1
                except:
                    pass

        print(f"  Extracted {count} yaw samples in window")

    return yaw_samples


def load_fused_heading():
    """Load fused heading from DriveState/Pose in the main wpilog."""
    from logreader.wpilog_reader import read_wpilog
    log_data = read_wpilog(WPILOG_PATH)
    pose_sig = log_data.signals.get("NT:/DriveState/Pose")

    heading_samples = []
    if pose_sig:
        for v in pose_sig.values:
            if T_START_US <= v.timestamp_us <= T_END_US:
                if len(v.value) >= 24:
                    x, y, angle_rad = struct.unpack("<3d", v.value[:24])
                    heading_samples.append((v.timestamp_us, math.degrees(angle_rad), x, y))

    return heading_samples


def main():
    print("=== Extracting Pigeon2 raw yaw ===")
    pigeon_yaw = extract_pigeon_yaw()

    if not pigeon_yaw:
        print("No Pigeon2 yaw data found in window!")
        return

    print(f"\nTotal Pigeon2 yaw samples: {len(pigeon_yaw)}")
    print(f"Time range: {pigeon_yaw[0][0]/1e6:.3f}s - {pigeon_yaw[-1][0]/1e6:.3f}s")

    # Show Pigeon2 yaw subsampled
    step = max(1, len(pigeon_yaw) // 60)
    print(f"\n=== PIGEON2 RAW YAW (every {step}th) ===")
    for ts, yaw in pigeon_yaw[::step]:
        print(f"  t={ts/1e6:.4f}s  pigeon_yaw={yaw:.2f} deg")

    print("\n=== Loading fused heading ===")
    fused = load_fused_heading()
    print(f"Fused heading samples: {len(fused)}")

    # Compare: for each fused heading sample, find nearest Pigeon yaw
    print(f"\n=== COMPARISON: Fused heading vs Pigeon2 raw yaw ===")
    print(f"{'time':>12s}  {'fused_hdg':>10s}  {'pigeon_yaw':>11s}  {'delta':>8s}  {'fused_x':>8s}  {'fused_y':>8s}")

    step = max(1, len(fused) // 80)
    pigeon_idx = 0

    for ts, hdg, x, y in fused[::step]:
        # Find nearest pigeon sample
        while pigeon_idx < len(pigeon_yaw) - 1 and pigeon_yaw[pigeon_idx + 1][0] <= ts:
            pigeon_idx += 1

        # Check both sides
        best_p = None
        best_dt = float("inf")
        for idx in (pigeon_idx, pigeon_idx + 1):
            if 0 <= idx < len(pigeon_yaw):
                dt = abs(pigeon_yaw[idx][0] - ts)
                if dt < best_dt:
                    best_dt = dt
                    best_p = pigeon_yaw[idx]

        if best_p and best_dt < 100000:  # within 100ms
            p_yaw = best_p[1]
            # Delta (wrapped)
            delta = hdg - p_yaw
            delta = (delta + 180) % 360 - 180
            flag = "  *** DRIFT" if abs(delta) > 5 else ""
            print(f"  {ts/1e6:10.4f}s  {hdg:9.2f} deg  {p_yaw:10.2f} deg  {delta:+7.2f} deg  {x:7.3f}  {y:7.3f}{flag}")
        else:
            print(f"  {ts/1e6:10.4f}s  {hdg:9.2f} deg  {'?':>10s}  {'?':>7s}  {x:7.3f}  {y:7.3f}")

    # Find largest divergence
    print(f"\n=== LARGEST DIVERGENCES ===")
    divergences = []
    pigeon_idx = 0
    for ts, hdg, x, y in fused:
        while pigeon_idx < len(pigeon_yaw) - 1 and pigeon_yaw[pigeon_idx + 1][0] <= ts:
            pigeon_idx += 1
        if 0 <= pigeon_idx < len(pigeon_yaw):
            p_yaw = pigeon_yaw[pigeon_idx][1]
            dt = abs(pigeon_yaw[pigeon_idx][0] - ts)
            if dt < 100000:
                delta = hdg - p_yaw
                delta = (delta + 180) % 360 - 180
                divergences.append((ts, hdg, p_yaw, delta, x, y))

    divergences.sort(key=lambda d: abs(d[3]), reverse=True)
    for ts, hdg, p_yaw, delta, x, y in divergences[:20]:
        print(f"  t={ts/1e6:.4f}s  fused={hdg:.2f}  pigeon={p_yaw:.2f}  delta={delta:+.2f}  x={x:.3f} y={y:.3f}")


if __name__ == "__main__":
    main()
