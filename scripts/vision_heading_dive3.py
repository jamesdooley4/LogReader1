"""Deep-dive: vision heading feedback during auto 'dive' incidents.

Uses log_us/1e6 as time reference (matching AdvantageScope display).
Incident 1: ~381-385s, Incident 2: ~530-535s
"""
import struct
import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from logreader.wpilog_reader import read_wpilog
from logreader.models import SignalType

LOG_PATH = r"D:\Temp\2026-04-07_programming\FRC_20260407_231323.wpilog"

# Time windows — log_us/1e6 seconds
WINDOWS = [
    ("Pre-Auto2", 368.0, 372.0),
    ("Auto2-start", 372.0, 377.0),
    ("INCIDENT-1", 377.0, 386.0),
    ("Pre-Auto3", 513.0, 517.0),
    ("Auto3-start", 517.0, 525.0),
    ("INCIDENT-2", 525.0, 536.0),
]


def decode_pose2d(raw_bytes):
    # Pose2d = Translation2d(x,y) + Rotation2d(value) = 3 doubles = 24 bytes
    if len(raw_bytes) < 24:
        return None
    x, y, angle_rad = struct.unpack("<3d", raw_bytes[:24])
    heading_deg = math.degrees(angle_rad)
    return (x, y, heading_deg)


def decode_chassis_speeds(raw_bytes):
    if len(raw_bytes) < 24:
        return None
    vx, vy, omega = struct.unpack("<3d", raw_bytes[:24])
    return (vx, vy, omega)


def decode_swerve_module_states(raw_bytes):
    # SwerveModuleState = speed(double) + Rotation2d(value: double) = 2 doubles = 16 bytes each
    element_size = 16
    count = len(raw_bytes) // element_size
    states = []
    for i in range(count):
        offset = i * element_size
        speed, angle_rad = struct.unpack("<2d", raw_bytes[offset:offset+16])
        # Normalize angle to [-180, 180]
        angle_deg = math.degrees(angle_rad) % 360
        if angle_deg > 180:
            angle_deg -= 360
        states.append((speed, angle_deg))
    return states


def get_window(sig, t_start_s, t_end_s):
    """Get values where log_us/1e6 is in [t_start_s, t_end_s]."""
    start_us = int(t_start_s * 1e6)
    end_us = int(t_end_s * 1e6)
    return [v for v in sig.values if start_us <= v.timestamp_us <= end_us]


def find_nearest_val(sig, target_us, max_delta_us=100000):
    """Binary search for nearest value."""
    if not sig or not sig.values:
        return None
    vals = sig.values
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
    if abs(vals[best].timestamp_us - target_us) > max_delta_us:
        return None
    return vals[best]


def t_str(timestamp_us):
    return f"{timestamp_us / 1e6:.3f}"


def main():
    print(f"Reading {LOG_PATH}...")
    log_data = read_wpilog(LOG_PATH)
    sigs = log_data.signals
    
    # Key signals
    pose_sig = sigs.get("NT:/DriveState/Pose")
    speeds_sig = sigs.get("NT:/DriveState/Speeds")
    mod_states_sig = sigs.get("NT:/DriveState/ModuleStates")
    mod_targets_sig = sigs.get("NT:/DriveState/ModuleTargets")
    
    pp_current = sigs.get("NT:/PathPlanner/currentPose")
    pp_target = sigs.get("NT:/PathPlanner/targetPose")
    
    cameras = ["a", "b", "c"]
    bp = {c: sigs.get(f"NT:/limelight-{c}/botpose_wpiblue") for c in cameras}
    bp_orb = {c: sigs.get(f"NT:/limelight-{c}/botpose_orb_wpiblue") for c in cameras}
    ro = {c: sigs.get(f"NT:/limelight-{c}/robot_orientation_set") for c in cameras}
    imu = {c: sigs.get(f"NT:/limelight-{c}/imu") for c in cameras}
    rf = {c: sigs.get(f"NT:/limelight-{c}/rawfiducials") for c in cameras}
    sd = {c: sigs.get(f"NT:/limelight-{c}/stddevs") for c in cameras}
    rr = {c: sigs.get(f"NT:/SmartDashboard//vision/limelight-{c}_rejectReason") for c in cameras}
    ve = {c: sigs.get(f"NT:/SmartDashboard//vision/limelight-{c}_visionError") for c in cameras}
    ud = sigs.get("NT:/SmartDashboard//vision/underDefense")
    
    # DS mode
    print("\n=== DS MODE TRANSITIONS (log_us/1e6) ===")
    for name in ["DS:autonomous", "DS:enabled"]:
        sig = sigs.get(name)
        if sig:
            for v in sig.values:
                print(f"  t={t_str(v.timestamp_us):>10s}s  {name.split(':')[1]}={v.value}")

    for label, t_start, t_end in WINDOWS:
        print(f"\n{'='*130}")
        print(f"=== {label}: {t_start:.1f}s - {t_end:.1f}s ===")
        print(f"{'='*130}")

        # --- Fused Pose ---
        if pose_sig:
            vals = get_window(pose_sig, t_start, t_end)
            step = max(1, len(vals) // 40)
            print(f"\n--- DriveState/Pose ({len(vals)} samples, every {step}th) ---")
            for v in vals[::step]:
                p = decode_pose2d(v.value)
                if p:
                    print(f"  t={t_str(v.timestamp_us)}s  x={p[0]:7.3f}  y={p[1]:7.3f}  hdg={p[2]:7.1f}°")

        # --- Chassis Speeds ---
        if speeds_sig:
            vals = get_window(speeds_sig, t_start, t_end)
            step = max(1, len(vals) // 25)
            if vals:
                print(f"\n--- DriveState/Speeds ({len(vals)} samples, every {step}th) ---")
                for v in vals[::step]:
                    s = decode_chassis_speeds(v.value)
                    if s:
                        speed = math.hypot(s[0], s[1])
                        print(f"  t={t_str(v.timestamp_us)}s  vx={s[0]:6.3f}  vy={s[1]:6.3f}  omega={s[2]:6.3f}rad/s  |v|={speed:.3f}m/s")

        # --- Module States (wheels) ---
        if mod_states_sig:
            vals = get_window(mod_states_sig, t_start, t_end)
            step = max(1, len(vals) // 25)
            if vals:
                print(f"\n--- ModuleStates (actual wheels, {len(vals)} samples, every {step}th) ---")
                for v in vals[::step]:
                    states = decode_swerve_module_states(v.value)
                    if states:
                        parts = [f"({s:5.2f}m/s,{a:6.0f}°)" for s, a in states]
                        print(f"  t={t_str(v.timestamp_us)}s  {' '.join(parts)}")

        # --- Module Targets (commanded) ---
        if mod_targets_sig:
            vals = get_window(mod_targets_sig, t_start, t_end)
            step = max(1, len(vals) // 15)
            if vals:
                print(f"\n--- ModuleTargets (commanded, {len(vals)} samples, every {step}th) ---")
                for v in vals[::step]:
                    states = decode_swerve_module_states(v.value)
                    if states:
                        parts = [f"({s:5.2f}m/s,{a:6.0f}°)" for s, a in states]
                        print(f"  t={t_str(v.timestamp_us)}s  {' '.join(parts)}")

        # --- PathPlanner ---
        for pp_name, pp_sig in [("PP/currentPose", pp_current), ("PP/targetPose", pp_target)]:
            if not pp_sig:
                continue
            vals = get_window(pp_sig, t_start, t_end)
            step = max(1, len(vals) // 15)
            if vals:
                print(f"\n--- {pp_name} ({len(vals)} samples, every {step}th) ---")
                for v in vals[::step]:
                    p = decode_pose2d(v.value)
                    if p:
                        print(f"  t={t_str(v.timestamp_us)}s  x={p[0]:7.3f}  y={p[1]:7.3f}  hdg={p[2]:7.1f}°")

        # --- Heading sent to LLs ---
        for c in cameras:
            if not ro[c]:
                continue
            vals = get_window(ro[c], t_start, t_end)
            step = max(1, len(vals) // 15)
            if vals:
                print(f"\n--- robot_orientation_set LL-{c} ({len(vals)} samples, every {step}th) ---")
                for v in vals[::step]:
                    arr = v.value
                    if isinstance(arr, (list, tuple)) and len(arr) >= 1:
                        print(f"  t={t_str(v.timestamp_us)}s  heading_sent={arr[0]:7.1f}°")

        # --- Botpose MT1 & MT2 (valid frames only) ---
        for c in cameras:
            for tag, bp_sig in [("MT1", bp[c]), ("MT2", bp_orb[c])]:
                if not bp_sig:
                    continue
                vals = get_window(bp_sig, t_start, t_end)
                valid = []
                for v in vals:
                    arr = v.value
                    if isinstance(arr, (list, tuple)) and len(arr) >= 11 and int(arr[7]) > 0:
                        if abs(arr[0]) > 0.001 or abs(arr[1]) > 0.001:
                            valid.append(v)
                if valid:
                    print(f"\n--- botpose LL-{c} {tag} ({len(valid)} valid of {len(vals)}) ---")
                    for v in valid:
                        arr = v.value
                        print(f"  t={t_str(v.timestamp_us)}s  x={arr[0]:7.3f} y={arr[1]:7.3f} yaw={arr[5]:7.1f}° tags={int(arr[7])} avgD={arr[9]:.2f}m lat={arr[6]:.1f}ms")

        # --- Rawfiducials ---
        for c in cameras:
            if not rf[c]:
                continue
            vals = get_window(rf[c], t_start, t_end)
            if vals:
                print(f"\n--- rawfiducials LL-{c} ({len(vals)} samples) ---")
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
                        print(f"  t={t_str(v.timestamp_us)}s  [{', '.join(tags)}]")

        # --- LL IMU (internal yaw) ---
        for c in cameras:
            if not imu[c]:
                continue
            vals = get_window(imu[c], t_start, t_end)
            nonzero = [v for v in vals if isinstance(v.value, (list, tuple)) and len(v.value) >= 6]
            if nonzero:
                step = max(1, len(nonzero) // 15)
                print(f"\n--- LL-{c} IMU ({len(nonzero)} samples, every {step}th) ---")
                for v in nonzero[::step]:
                    arr = v.value
                    # imu: [yaw, yawRate, pitch, pitchRate, roll, rollRate, ...]
                    print(f"  t={t_str(v.timestamp_us)}s  imu_yaw={arr[0]:7.1f}° yawRate={arr[1]:7.2f}°/s")

        # --- Reject reasons (transitions only) ---
        for c in cameras:
            if not rr[c]:
                continue
            vals = get_window(rr[c], t_start, t_end)
            if vals:
                prev = None
                changes = []
                for v in vals:
                    if v.value != prev:
                        changes.append(v)
                        prev = v.value
                print(f"\n--- rejectReason LL-{c} ({len(changes)} transitions of {len(vals)}) ---")
                for v in changes:
                    print(f"  t={t_str(v.timestamp_us)}s  {v.value}")

        # --- Vision error > 0.5m ---
        for c in cameras:
            if not ve[c]:
                continue
            vals = get_window(ve[c], t_start, t_end)
            big = [v for v in vals if abs(v.value) > 0.5]
            if big:
                step = max(1, len(big) // 10)
                print(f"\n--- visionError LL-{c} > 0.5m ({len(big)} of {len(vals)}) ---")
                for v in big[::step]:
                    print(f"  t={t_str(v.timestamp_us)}s  err={v.value:.3f}m")

        # --- Under defense ---
        if ud:
            vals = get_window(ud, t_start, t_end)
            if vals:
                prev = None
                print(f"\n--- underDefense ---")
                for v in vals:
                    if v.value != prev:
                        print(f"  t={t_str(v.timestamp_us)}s  {v.value}")
                        prev = v.value

    # =====================================================================
    # CRITICAL ANALYSIS: Heading comparison table at incident timestamps
    # =====================================================================
    print(f"\n{'='*130}")
    print("=== HEADING COMPARISON TABLE ===")
    print("Comparing: fused_pose_heading vs heading_sent_to_LL vs LL_MT1_yaw vs LL_MT2_yaw vs LL_internal_IMU_yaw")
    print(f"{'='*130}")

    for label, t_start, t_end in [("INCIDENT-1", 377.0, 386.0), ("INCIDENT-2", 525.0, 536.0)]:
        print(f"\n--- {label} ---")
        header = (f"{'t(s)':>10s}  {'fused_x':>8s} {'fused_y':>8s} {'fused_hdg':>10s}  "
                  f"{'sent_hdg':>10s} {'d(f-s)':>7s}  "
                  f"{'C_mt1_x':>8s} {'C_mt1_y':>8s} {'C_mt1_yaw':>10s}  "
                  f"{'C_mt2_x':>8s} {'C_mt2_y':>8s} {'C_mt2_yaw':>10s}  "
                  f"{'C_imu_yaw':>10s}  {'reject_C':>25s}")
        print(header)
        print("-" * len(header))

        # Use pose timestamps as reference, subsample
        if not pose_sig:
            continue
        pose_vals = get_window(pose_sig, t_start, t_end)
        step = max(1, len(pose_vals) // 60)
        
        for pv in pose_vals[::step]:
            t_us = pv.timestamp_us
            ts = t_str(t_us)
            
            p = decode_pose2d(pv.value)
            if not p:
                continue
            fx, fy, fhdg = p
            
            # Nearest heading sent
            sent_hdg = "-"
            delta_str = "-"
            ro_v = find_nearest_val(ro["c"], t_us, 50000)
            if ro_v and isinstance(ro_v.value, (list, tuple)):
                s = ro_v.value[0]
                sent_hdg = f"{s:7.1f}°"
                d = fhdg - s
                d = (d + 180) % 360 - 180
                delta_str = f"{d:6.1f}°"
            
            # Camera C MT1
            c_mt1_x, c_mt1_y, c_mt1_yaw = "-", "-", "-"
            bp_v = find_nearest_val(bp["c"], t_us, 50000)
            if bp_v and isinstance(bp_v.value, (list, tuple)) and len(bp_v.value) >= 11:
                arr = bp_v.value
                if int(arr[7]) > 0 and (abs(arr[0]) > 0.001 or abs(arr[1]) > 0.001):
                    c_mt1_x = f"{arr[0]:7.3f}"
                    c_mt1_y = f"{arr[1]:7.3f}"
                    c_mt1_yaw = f"{arr[5]:7.1f}°"
            
            # Camera C MT2
            c_mt2_x, c_mt2_y, c_mt2_yaw = "-", "-", "-"
            orb_v = find_nearest_val(bp_orb["c"], t_us, 50000)
            if orb_v and isinstance(orb_v.value, (list, tuple)) and len(orb_v.value) >= 11:
                arr = orb_v.value
                if int(arr[7]) > 0 and (abs(arr[0]) > 0.001 or abs(arr[1]) > 0.001):
                    c_mt2_x = f"{arr[0]:7.3f}"
                    c_mt2_y = f"{arr[1]:7.3f}"
                    c_mt2_yaw = f"{arr[5]:7.1f}°"
            
            # Camera C LL IMU yaw
            c_imu_yaw = "-"
            imu_v = find_nearest_val(imu["c"], t_us, 50000)
            if imu_v and isinstance(imu_v.value, (list, tuple)) and len(imu_v.value) >= 1:
                c_imu_yaw = f"{imu_v.value[0]:7.1f}°"
            
            # Reject reason
            rej = "-"
            rr_v = find_nearest_val(rr["c"], t_us, 100000)
            if rr_v:
                rej = str(rr_v.value)[:25]
            
            print(f"  {ts:>8s}  {fx:8.3f} {fy:8.3f} {fhdg:9.1f}°  "
                  f"{sent_hdg:>10s} {delta_str:>7s}  "
                  f"{c_mt1_x:>8s} {c_mt1_y:>8s} {c_mt1_yaw:>10s}  "
                  f"{c_mt2_x:>8s} {c_mt2_y:>8s} {c_mt2_yaw:>10s}  "
                  f"{c_imu_yaw:>10s}  {rej:>25s}")

    # Also check: which cameras had accepted vision measurements during incidents
    print(f"\n{'='*130}")
    print("=== ACCEPTED VISION MEASUREMENTS (reject=none) DURING INCIDENTS ===")
    print(f"{'='*130}")
    
    for label, t_start, t_end in [("INCIDENT-1", 377.0, 386.0), ("INCIDENT-2", 525.0, 536.0)]:
        print(f"\n--- {label} ---")
        for c in cameras:
            if not rr[c]:
                continue
            vals = get_window(rr[c], t_start, t_end)
            accepted = [v for v in vals if v.value == "none"]
            if accepted:
                print(f"  LL-{c}: {len(accepted)} accepted measurements")
                for v in accepted:
                    ts = t_str(v.timestamp_us)
                    # Find corresponding vision error
                    ve_v = find_nearest_val(ve[c], v.timestamp_us, 50000)
                    err_str = f"err={ve_v.value:.3f}m" if ve_v else "err=?"
                    
                    # Find corresponding botpose
                    bp_v = find_nearest_val(bp[c], v.timestamp_us, 50000)
                    bp_str = ""
                    if bp_v and isinstance(bp_v.value, (list, tuple)) and len(bp_v.value) >= 11:
                        arr = bp_v.value
                        bp_str = f"MT1: x={arr[0]:.3f} y={arr[1]:.3f} yaw={arr[5]:.1f}° tags={int(arr[7])}"
                    
                    orb_v = find_nearest_val(bp_orb[c], v.timestamp_us, 50000)
                    orb_str = ""
                    if orb_v and isinstance(orb_v.value, (list, tuple)) and len(orb_v.value) >= 11:
                        arr = orb_v.value
                        if int(arr[7]) > 0:
                            orb_str = f"MT2: x={arr[0]:.3f} y={arr[1]:.3f} yaw={arr[5]:.1f}° tags={int(arr[7])}"
                    
                    print(f"    t={ts}s  {err_str}  {bp_str}  {orb_str}")
            else:
                rejs = get_window(rr[c], t_start, t_end)
                if rejs:
                    reasons = set(v.value for v in rejs)
                    print(f"  LL-{c}: 0 accepted, reasons: {reasons}")


if __name__ == "__main__":
    main()
