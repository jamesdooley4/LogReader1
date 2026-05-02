"""Investigate heading drift during auto in FRC_20260407_235325.wpilog.

Tracks: DriveState/Pose heading, accepted vision measurements, MT1 theta
contributions, and stale data patterns from 230s to 248s.
"""
import struct
import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from logreader.wpilog_reader import read_wpilog

LOG_PATH = r"D:\Temp\2026-04-07_programming\FRC_20260407_235325.wpilog"

def decode_pose2d(raw_bytes):
    if len(raw_bytes) < 24:
        return None
    x, y, angle_rad = struct.unpack("<3d", raw_bytes[:24])
    return (x, y, math.degrees(angle_rad))

def decode_chassis_speeds(raw_bytes):
    if len(raw_bytes) < 24:
        return None
    vx, vy, omega = struct.unpack("<3d", raw_bytes[:24])
    return (vx, vy, omega)

def get_window(sig, t_start, t_end):
    start_us = int(t_start * 1e6)
    end_us = int(t_end * 1e6)
    return [v for v in sig.values if start_us <= v.timestamp_us <= end_us]

def find_nearest(vals, target_us, max_delta=100000):
    if not vals:
        return None
    lo, hi = 0, len(vals) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if vals[mid].timestamp_us < target_us:
            lo = mid + 1
        else:
            hi = mid
    best = lo
    if lo > 0 and abs(vals[lo-1].timestamp_us - target_us) < abs(vals[lo].timestamp_us - target_us):
        best = lo - 1
    if abs(vals[best].timestamp_us - target_us) > max_delta:
        return None
    return vals[best]

def t_s(us):
    return f"{us/1e6:.4f}"

def main():
    print(f"Reading {LOG_PATH}...")
    log_data = read_wpilog(LOG_PATH)
    sigs = log_data.signals
    print(f"Loaded {len(sigs)} signals")

    # Key signals
    pose_sig = sigs.get("NT:/DriveState/Pose")
    speeds_sig = sigs.get("NT:/DriveState/Speeds")
    cameras = ["a", "b", "c"]
    bp = {c: sigs.get(f"NT:/limelight-{c}/botpose_wpiblue") for c in cameras}
    bp_orb = {c: sigs.get(f"NT:/limelight-{c}/botpose_orb_wpiblue") for c in cameras}
    ro = {c: sigs.get(f"NT:/limelight-{c}/robot_orientation_set") for c in cameras}
    imu = {c: sigs.get(f"NT:/limelight-{c}/imu") for c in cameras}
    rf = {c: sigs.get(f"NT:/limelight-{c}/rawfiducials") for c in cameras}
    rr = {c: sigs.get(f"NT:/SmartDashboard//vision/limelight-{c}_rejectReason") for c in cameras}
    ve = {c: sigs.get(f"NT:/SmartDashboard//vision/limelight-{c}_visionError") for c in cameras}
    mt1_std_xy = {c: sigs.get(f"NT:/SmartDashboard//vision/limelight-{c} Mt1 STD xy") for c in cameras}
    mt1_std_th = {c: sigs.get(f"NT:/SmartDashboard//vision/limelight-{c} Mt1 STD theta") for c in cameras}
    mt2_std_xy = {c: sigs.get(f"NT:/SmartDashboard//vision/limelight-{c} Mt2 STD xy") for c in cameras}
    ud = sigs.get("NT:/SmartDashboard//vision/underDefense")

    T_START = 228.0
    T_END = 250.0

    # === DS Mode transitions ===
    print("\n=== DS MODE ===")
    for name in ["DS:autonomous", "DS:enabled"]:
        sig = sigs.get(name)
        if sig:
            for v in sig.values:
                t = v.timestamp_us / 1e6
                if 220 < t < 260:
                    print(f"  t={t:.3f}s  {name.split(':')[1]}={v.value}")

    # === Fused pose heading over time ===
    print(f"\n=== DRIVESTATE POSE (heading focus) {T_START}-{T_END}s ===")
    if pose_sig:
        vals = get_window(pose_sig, T_START, T_END)
        # Show every ~0.25s
        step = max(1, len(vals) // 80)
        prev_hdg = None
        for v in vals[::step]:
            p = decode_pose2d(v.value)
            if p:
                hdg_change = ""
                if prev_hdg is not None:
                    d = p[2] - prev_hdg
                    d = (d + 180) % 360 - 180
                    if abs(d) > 0.5:
                        hdg_change = f"  dHdg={d:+.1f}"
                print(f"  t={t_s(v.timestamp_us)}s  x={p[0]:7.3f} y={p[1]:7.3f} hdg={p[2]:8.2f} deg{hdg_change}")
                prev_hdg = p[2]

    # === Find ALL heading jumps > 1 degree ===
    print(f"\n=== HEADING JUMPS > 1 deg ===")
    if pose_sig:
        vals = get_window(pose_sig, T_START, T_END)
        prev_hdg = None
        prev_us = None
        for v in vals:
            p = decode_pose2d(v.value)
            if not p:
                continue
            if prev_hdg is not None:
                d = p[2] - prev_hdg
                d = (d + 180) % 360 - 180
                if abs(d) > 1.0:
                    dt_ms = (v.timestamp_us - prev_us) / 1000.0
                    print(f"  t={t_s(v.timestamp_us)}s  hdg={p[2]:8.2f}  jumped {d:+.2f} deg in {dt_ms:.1f}ms  x={p[0]:.3f} y={p[1]:.3f}")
            prev_hdg = p[2]
            prev_us = v.timestamp_us

    # === Chassis speeds (omega) ===
    print(f"\n=== CHASSIS SPEEDS (omega focus) ===")
    if speeds_sig:
        vals = get_window(speeds_sig, T_START, T_END)
        step = max(1, len(vals) // 40)
        for v in vals[::step]:
            s = decode_chassis_speeds(v.value)
            if s:
                print(f"  t={t_s(v.timestamp_us)}s  vx={s[0]:6.2f} vy={s[1]:6.2f} omega={s[2]:7.3f} rad/s ({math.degrees(s[2]):6.1f} deg/s)")

    # === Accepted vision measurements (reject=none) ===
    print(f"\n=== ACCEPTED VISION MEASUREMENTS ===")
    for c in cameras:
        if not rr[c]:
            continue
        vals = get_window(rr[c], T_START, T_END)
        accepted = [v for v in vals if v.value == "none"]
        if not accepted:
            reasons = {}
            for v in vals:
                reasons[v.value] = reasons.get(v.value, 0) + 1
            print(f"  LL-{c}: 0 accepted in window. Reasons: {reasons}")
            continue

        print(f"\n  LL-{c}: {len(accepted)} accepted measurements")
        for v in accepted:
            t_us = v.timestamp_us

            # Find nearest fused pose
            fused = ""
            if pose_sig:
                pv = find_nearest(pose_sig.values, t_us, 50000)
                if pv:
                    p = decode_pose2d(pv.value)
                    if p:
                        fused = f"fusedHdg={p[2]:.1f}"

            # MT1 botpose
            mt1_str = ""
            bp_v = find_nearest(bp[c].values, t_us, 50000) if bp[c] else None
            if bp_v and isinstance(bp_v.value, (list, tuple)) and len(bp_v.value) >= 11:
                arr = bp_v.value
                tc = int(arr[7])
                if tc > 0:
                    mt1_str = f"MT1: x={arr[0]:.3f} y={arr[1]:.3f} yaw={arr[5]:.1f} tags={tc} avgD={arr[9]:.2f}m"

            # MT2 botpose
            mt2_str = ""
            orb_v = find_nearest(bp_orb[c].values, t_us, 50000) if bp_orb[c] else None
            if orb_v and isinstance(orb_v.value, (list, tuple)) and len(orb_v.value) >= 11:
                arr = orb_v.value
                tc = int(arr[7])
                if tc > 0:
                    mt2_str = f"MT2: x={arr[0]:.3f} y={arr[1]:.3f} yaw={arr[5]:.1f} tags={tc}"

            # Heading sent to LL
            sent_str = ""
            ro_v = find_nearest(ro[c].values, t_us, 50000) if ro[c] else None
            if ro_v and isinstance(ro_v.value, (list, tuple)):
                sent_str = f"sentHdg={ro_v.value[0]:.1f}"

            # STD devs
            std_str = ""
            if mt1_std_th[c]:
                th_v = find_nearest(mt1_std_th[c].values, t_us, 100000)
                if th_v:
                    std_str = f"mt1StdTheta={th_v.value:.4f}"
            if mt1_std_xy[c]:
                xy_v = find_nearest(mt1_std_xy[c].values, t_us, 100000)
                if xy_v:
                    std_str += f" mt1StdXY={xy_v.value:.4f}"

            # Vision error
            err_str = ""
            if ve[c]:
                ev = find_nearest(ve[c].values, t_us, 50000)
                if ev:
                    err_str = f"err={ev.value:.3f}m"

            print(f"    t={t_s(t_us)}s  {fused}  {sent_str}  {err_str}  {mt1_str}  {mt2_str}  {std_str}")

    # === Stale data analysis per camera ===
    print(f"\n=== STALE DATA ANALYSIS ===")
    for c in cameras:
        if not bp[c]:
            continue
        vals = get_window(bp[c], T_START, T_END)
        prev = None
        stale = 0
        fresh = 0
        fresh_frames = []
        for v in vals:
            arr = v.value
            if not isinstance(arr, (list, tuple)) or len(arr) < 11:
                continue
            key = tuple(round(x, 6) for x in arr[:6])
            tc = int(arr[7])
            if key == prev:
                stale += 1
            else:
                fresh += 1
                fresh_frames.append((v.timestamp_us, arr[0], arr[1], arr[5], tc, arr[9] if len(arr) > 9 else 0))
                prev = key

        print(f"\n  LL-{c}: {fresh} new frames, {stale} stale repeats ({stale/max(fresh,1):.0f}:1 ratio)")
        if fresh_frames:
            print(f"  Fresh frames:")
            for ts, x, y, yaw, tc, avgd in fresh_frames:
                marker = ""
                if tc > 0 and (abs(x) > 0.001 or abs(y) > 0.001):
                    marker = " <-- VALID"
                elif tc == 0:
                    marker = " (no tags)"
                print(f"    t={t_s(ts)}s  x={x:.3f} y={y:.3f} yaw={yaw:.1f} tags={tc} avgD={avgd:.2f}m{marker}")

    # === Rawfiducials during the window ===
    print(f"\n=== RAWFIDUCIALS ===")
    for c in cameras:
        if not rf[c]:
            continue
        vals = get_window(rf[c], T_START, T_END)
        if vals:
            print(f"\n  LL-{c}: {len(vals)} rawfiducial samples")
            for v in vals:
                arr = v.value
                if not isinstance(arr, (list, tuple)):
                    continue
                i = 0
                tags = []
                while i + 6 < len(arr):
                    tag_id = int(arr[i])
                    if 0 <= tag_id <= 100:
                        tags.append(f"id={tag_id} d={arr[i+4]:.2f}m amb={arr[i+6]:.3f}")
                    i += 7
                if tags:
                    print(f"    t={t_s(v.timestamp_us)}s  [{', '.join(tags)}]")

    # === Heading sent vs fused pose heading ===
    print(f"\n=== HEADING SENT vs FUSED (per sample) ===")
    if pose_sig and ro["c"]:
        vals = get_window(pose_sig, 230.0, 248.0)
        step = max(1, len(vals) // 50)
        for v in vals[::step]:
            p = decode_pose2d(v.value)
            if not p:
                continue
            ro_v = find_nearest(ro["c"].values, v.timestamp_us, 50000)
            sent = ro_v.value[0] if ro_v and isinstance(ro_v.value, (list, tuple)) else None
            delta = ""
            if sent is not None:
                d = p[2] - sent
                d = (d + 180) % 360 - 180
                delta = f"  delta={d:+.1f}"
            sent_str = f"sent={sent:.1f}" if sent is not None else "sent=?"
            print(f"  t={t_s(v.timestamp_us)}s  fused={p[2]:8.2f}  {sent_str}{delta}")

    # === LL IMU internal yaw ===
    print(f"\n=== LL IMU INTERNAL YAW ===")
    for c in cameras:
        if not imu[c]:
            continue
        vals = get_window(imu[c], 230.0, 248.0)
        if not vals:
            continue
        step = max(1, len(vals) // 20)
        print(f"\n  LL-{c} IMU:")
        for v in vals[::step]:
            arr = v.value
            if isinstance(arr, (list, tuple)) and len(arr) >= 2:
                print(f"    t={t_s(v.timestamp_us)}s  imu_yaw={arr[0]:.1f} yawRate={arr[1]:.2f}")

    # === Under defense ===
    if ud:
        vals = get_window(ud, T_START, T_END)
        if vals:
            print(f"\n=== UNDER DEFENSE ===")
            prev = None
            for v in vals:
                if v.value != prev:
                    print(f"  t={t_s(v.timestamp_us)}s  {v.value}")
                    prev = v.value

    # === Detailed heading timeline with ALL vision data interleaved ===
    print(f"\n{'='*130}")
    print("=== DETAILED HEADING TIMELINE: What moved the heading? ===")
    print(f"{'='*130}")

    # Build a unified timeline of events
    events = []

    # Pose heading samples (subsample to ~50Hz)
    if pose_sig:
        prev_hdg = None
        for v in get_window(pose_sig, 229.0, 249.0):
            p = decode_pose2d(v.value)
            if not p:
                continue
            dh = 0
            if prev_hdg is not None:
                dh = p[2] - prev_hdg
                dh = (dh + 180) % 360 - 180
            events.append((v.timestamp_us, "POSE", f"hdg={p[2]:8.2f} x={p[0]:.3f} y={p[1]:.3f} dHdg={dh:+.2f}"))
            prev_hdg = p[2]

    # Vision accept/reject
    for c in cameras:
        if not rr[c]:
            continue
        for v in get_window(rr[c], 229.0, 249.0):
            events.append((v.timestamp_us, f"RJ-{c}", v.value))

    # Sort and filter to show heading changes and vision events
    events.sort(key=lambda e: e[0])

    # Only show: heading jumps > 0.3 deg, and vision accepts
    for ts, kind, data in events:
        if kind == "POSE":
            # Extract dHdg
            try:
                dh_str = data.split("dHdg=")[1]
                dh = float(dh_str)
                if abs(dh) > 0.3:
                    print(f"  t={t_s(ts)}s  [{kind}] {data}")
            except (IndexError, ValueError):
                pass
        elif "none" in str(data):
            print(f"  t={t_s(ts)}s  [{kind}] ACCEPTED")


if __name__ == "__main__":
    main()
