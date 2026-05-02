"""Deep-dive into vision heading feedback during auto 'dive' incidents.

Decodes struct signals (Pose2d, SwerveModuleState, ChassisSpeeds) and
cross-correlates with Limelight botpose, robot_orientation_set, and
diagnostics to identify what caused the robot to lurch toward red side.
"""

import struct
import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from logreader.wpilog_reader import read_wpilog
from logreader.models import SignalType

LOG_PATH = r"D:\Temp\2026-04-07_programming\FRC_20260407_231323.wpilog"

WINDOWS = [
    ("Pre-Incident-1", 375.0, 380.0),
    ("Incident 1", 380.0, 387.0),
    ("Between", 520.0, 529.0),
    ("Incident 2", 529.0, 537.0),
]


def decode_pose2d(raw_bytes):
    """Decode struct:Pose2d → (x, y, heading_rad).
    WPILib Pose2d struct: Translation2d(x,y) + Rotation2d(cos,sin) = 4 doubles.
    """
    if len(raw_bytes) < 32:
        return None
    x, y, cos_h, sin_h = struct.unpack("<4d", raw_bytes[:32])
    heading_deg = math.degrees(math.atan2(sin_h, cos_h))
    return (x, y, heading_deg)


def decode_chassis_speeds(raw_bytes):
    """Decode struct:ChassisSpeeds → (vx, vy, omega)."""
    if len(raw_bytes) < 24:
        return None
    vx, vy, omega = struct.unpack("<3d", raw_bytes[:24])
    return (vx, vy, omega)


def decode_swerve_module_states(raw_bytes):
    """Decode struct:SwerveModuleState[] → list of (speed_mps, angle_rad).
    Each SwerveModuleState is 3 doubles: speed + Rotation2d(cos,sin) = 24 bytes.
    """
    # Array prefix: 4 bytes length
    if len(raw_bytes) < 4:
        return []
    count = struct.unpack("<I", raw_bytes[:4])[0]
    states = []
    offset = 4
    for _ in range(count):
        if offset + 24 > len(raw_bytes):
            break
        speed, cos_a, sin_a = struct.unpack("<3d", raw_bytes[offset:offset+24])
        angle_deg = math.degrees(math.atan2(sin_a, cos_a))
        states.append((speed, angle_deg))
        offset += 24
    return states


def get_window(sig, t0_us, t_start, t_end):
    """Get values in a time window."""
    start_us = t0_us + int(t_start * 1e6)
    end_us = t0_us + int(t_end * 1e6)
    return [v for v in sig.values if start_us <= v.timestamp_us <= end_us]


def main():
    print(f"Reading {LOG_PATH}...")
    log_data = read_wpilog(LOG_PATH)
    
    # Find t0
    all_ts = [sig.values[0].timestamp_us for sig in log_data.signals.values() if sig.values]
    t0_us = min(all_ts)
    print(f"Log start: {t0_us} us")

    # Get key signals
    sigs = log_data.signals
    
    # Struct signals
    pose_sig = sigs.get("NT:/DriveState/Pose")
    speeds_sig = sigs.get("NT:/DriveState/Speeds")
    mod_states_sig = sigs.get("NT:/DriveState/ModuleStates")
    mod_targets_sig = sigs.get("NT:/DriveState/ModuleTargets")
    
    # PathPlanner
    pp_current = sigs.get("NT:/PathPlanner/currentPose")
    pp_target = sigs.get("NT:/PathPlanner/targetPose")
    
    # Limelight botpose (DOUBLE_ARRAY)
    bp_a = sigs.get("NT:/limelight-a/botpose_wpiblue")
    bp_b = sigs.get("NT:/limelight-b/botpose_wpiblue")
    bp_c = sigs.get("NT:/limelight-c/botpose_wpiblue")
    
    bp_orb_a = sigs.get("NT:/limelight-a/botpose_orb_wpiblue")
    bp_orb_b = sigs.get("NT:/limelight-b/botpose_orb_wpiblue")
    bp_orb_c = sigs.get("NT:/limelight-c/botpose_orb_wpiblue")
    
    # Robot orientation sent to LLs
    ro_a = sigs.get("NT:/limelight-a/robot_orientation_set")
    ro_b = sigs.get("NT:/limelight-b/robot_orientation_set")  
    ro_c = sigs.get("NT:/limelight-c/robot_orientation_set")
    
    # IMU data from LLs
    imu_a = sigs.get("NT:/limelight-a/imu")
    imu_b = sigs.get("NT:/limelight-b/imu")
    imu_c = sigs.get("NT:/limelight-c/imu")
    
    # Rawfiducials
    rf_a = sigs.get("NT:/limelight-a/rawfiducials")
    rf_b = sigs.get("NT:/limelight-b/rawfiducials")
    rf_c = sigs.get("NT:/limelight-c/rawfiducials")
    
    # Stddevs
    sd_a = sigs.get("NT:/limelight-a/stddevs")
    sd_b = sigs.get("NT:/limelight-b/stddevs")
    sd_c = sigs.get("NT:/limelight-c/stddevs")

    # Reject reasons
    rr_a = sigs.get("NT:/SmartDashboard//vision/limelight-a_rejectReason")
    rr_b = sigs.get("NT:/SmartDashboard//vision/limelight-b_rejectReason")
    rr_c = sigs.get("NT:/SmartDashboard//vision/limelight-c_rejectReason")
    
    # Vision error
    ve_a = sigs.get("NT:/SmartDashboard//vision/limelight-a_visionError")
    ve_b = sigs.get("NT:/SmartDashboard//vision/limelight-b_visionError")
    ve_c = sigs.get("NT:/SmartDashboard//vision/limelight-c_visionError")
    
    # Under defense
    ud = sigs.get("NT:/SmartDashboard//vision/underDefense")
    
    # DS mode signals
    ds_auto = sigs.get("DS:autonomous")
    ds_enabled = sigs.get("DS:enabled")

    # Print DS mode transitions
    print("\n=== DS MODE TRANSITIONS ===")
    if ds_auto:
        for v in ds_auto.values:
            t_rel = (v.timestamp_us - t0_us) / 1e6
            print(f"  t={t_rel:8.3f}s  autonomous={v.value}")
    if ds_enabled:
        for v in ds_enabled.values:
            t_rel = (v.timestamp_us - t0_us) / 1e6
            print(f"  t={t_rel:8.3f}s  enabled={v.value}")

    for label, t_start, t_end in WINDOWS:
        print(f"\n{'='*120}")
        print(f"=== {label}: {t_start:.1f}s - {t_end:.1f}s ===")
        print(f"{'='*120}")

        # --- DriveState/Pose (fused pose) ---
        if pose_sig:
            vals = get_window(pose_sig, t0_us, t_start, t_end)
            # Subsample to ~10Hz for readability (every 5th sample at 50Hz)
            step = max(1, len(vals) // 50)
            show = vals[::step]
            print(f"\n--- DriveState/Pose ({len(vals)} samples, showing every {step}th) ---")
            for v in show:
                t_rel = (v.timestamp_us - t0_us) / 1e6
                p = decode_pose2d(v.value)
                if p:
                    print(f"  t={t_rel:8.3f}s  x={p[0]:7.3f}  y={p[1]:7.3f}  heading={p[2]:7.1f}°")

        # --- DriveState/Speeds (chassis speeds) ---
        if speeds_sig:
            vals = get_window(speeds_sig, t0_us, t_start, t_end)
            step = max(1, len(vals) // 30)
            show = vals[::step]
            print(f"\n--- DriveState/Speeds ({len(vals)} samples, showing every {step}th) ---")
            for v in show:
                t_rel = (v.timestamp_us - t0_us) / 1e6
                s = decode_chassis_speeds(v.value)
                if s:
                    speed = math.hypot(s[0], s[1])
                    print(f"  t={t_rel:8.3f}s  vx={s[0]:6.3f}  vy={s[1]:6.3f}  omega={s[2]:6.3f} rad/s  |v|={speed:.3f} m/s")
        
        # --- DriveState/ModuleStates (actual wheel states) ---
        if mod_states_sig:
            vals = get_window(mod_states_sig, t0_us, t_start, t_end)
            step = max(1, len(vals) // 30)
            show = vals[::step]
            print(f"\n--- DriveState/ModuleStates ({len(vals)} samples, showing every {step}th) ---")
            for v in show:
                t_rel = (v.timestamp_us - t0_us) / 1e6
                states = decode_swerve_module_states(v.value)
                if states:
                    parts = [f"({s:.2f}m/s, {a:.0f}°)" for s, a in states]
                    print(f"  t={t_rel:8.3f}s  {' '.join(parts)}")

        # --- DriveState/ModuleTargets (commanded wheel states) ---
        if mod_targets_sig:
            vals = get_window(mod_targets_sig, t0_us, t_start, t_end)
            step = max(1, len(vals) // 20)
            show = vals[::step]
            print(f"\n--- DriveState/ModuleTargets ({len(vals)} samples, showing every {step}th) ---")
            for v in show:
                t_rel = (v.timestamp_us - t0_us) / 1e6
                states = decode_swerve_module_states(v.value)
                if states:
                    parts = [f"({s:.2f}m/s, {a:.0f}°)" for s, a in states]
                    print(f"  t={t_rel:8.3f}s  {' '.join(parts)}")
        
        # --- PathPlanner poses ---
        if pp_current:
            vals = get_window(pp_current, t0_us, t_start, t_end)
            step = max(1, len(vals) // 20)
            show = vals[::step]
            if show:
                print(f"\n--- PathPlanner/currentPose ({len(vals)} samples) ---")
                for v in show:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    p = decode_pose2d(v.value)
                    if p:
                        print(f"  t={t_rel:8.3f}s  x={p[0]:7.3f}  y={p[1]:7.3f}  heading={p[2]:7.1f}°")
        
        if pp_target:
            vals = get_window(pp_target, t0_us, t_start, t_end)
            step = max(1, len(vals) // 20)
            show = vals[::step]
            if show:
                print(f"\n--- PathPlanner/targetPose ({len(vals)} samples) ---")
                for v in show:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    p = decode_pose2d(v.value)
                    if p:
                        print(f"  t={t_rel:8.3f}s  x={p[0]:7.3f}  y={p[1]:7.3f}  heading={p[2]:7.1f}°")

        # --- Robot orientation sent to all LLs ---
        for cam_name, ro_sig in [("LL-A", ro_a), ("LL-B", ro_b), ("LL-C", ro_c)]:
            if not ro_sig:
                continue
            vals = get_window(ro_sig, t0_us, t_start, t_end)
            step = max(1, len(vals) // 20)
            show = vals[::step]
            if show:
                print(f"\n--- robot_orientation_set {cam_name} ({len(vals)} samples) ---")
                for v in show:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    arr = v.value
                    if isinstance(arr, (list, tuple)) and len(arr) >= 1:
                        print(f"  t={t_rel:8.3f}s  heading_sent={arr[0]:7.1f}°")

        # --- Botpose from each camera (MT1 and MT2) ---
        for cam_name, bp_mt1, bp_mt2 in [
            ("LL-A", bp_a, bp_orb_a),
            ("LL-B", bp_b, bp_orb_b),
            ("LL-C", bp_c, bp_orb_c),
        ]:
            for tag, bp_sig in [("MT1", bp_mt1), ("MT2", bp_mt2)]:
                if not bp_sig:
                    continue
                vals = get_window(bp_sig, t0_us, t_start, t_end)
                # Only show frames with tags > 0
                valid = []
                for v in vals:
                    arr = v.value
                    if isinstance(arr, (list, tuple)) and len(arr) >= 11:
                        tc = int(arr[7])
                        if tc > 0:
                            valid.append(v)
                if valid:
                    print(f"\n--- botpose {cam_name} {tag} ({len(valid)} valid of {len(vals)} total) ---")
                    for v in valid:
                        t_rel = (v.timestamp_us - t0_us) / 1e6
                        arr = v.value
                        x, y = arr[0], arr[1]
                        yaw = arr[5]
                        tc = int(arr[7])
                        avg_dist = arr[9]
                        lat = arr[6]
                        print(f"  t={t_rel:8.3f}s  x={x:7.3f} y={y:7.3f} yaw={yaw:7.1f}° tags={tc} avgDist={avg_dist:.2f}m lat={lat:.1f}ms")

        # --- Rawfiducials (tag details) ---
        for cam_name, rf_sig in [("LL-A", rf_a), ("LL-B", rf_b), ("LL-C", rf_c)]:
            if not rf_sig:
                continue
            vals = get_window(rf_sig, t0_us, t_start, t_end)
            if vals:
                print(f"\n--- rawfiducials {cam_name} ({len(vals)} samples) ---")
                for v in vals:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    arr = v.value
                    if not isinstance(arr, (list, tuple)):
                        continue
                    i = 0
                    tags = []
                    while i + 6 < len(arr):
                        tag_id = int(arr[i])
                        if 0 <= tag_id <= 100:
                            ambig = arr[i+6]
                            dist_cam = arr[i+4]
                            dist_robot = arr[i+5]
                            tags.append(f"id={tag_id} dCam={dist_cam:.2f}m dRob={dist_robot:.2f}m amb={ambig:.3f}")
                        i += 7
                    if tags:
                        print(f"  t={t_rel:8.3f}s  [{', '.join(tags)}]")

        # --- LL IMU data ---
        for cam_name, imu_sig in [("LL-A", imu_a), ("LL-B", imu_b), ("LL-C", imu_c)]:
            if not imu_sig:
                continue
            vals = get_window(imu_sig, t0_us, t_start, t_end)
            # Show LL internal yaw (arr[2]) - subsample
            valid = [v for v in vals if isinstance(v.value, (list, tuple)) and len(v.value) >= 6
                     and any(abs(x) > 0.01 for x in v.value[:6])]
            if valid:
                step = max(1, len(valid) // 20)
                show = valid[::step]
                print(f"\n--- LL IMU {cam_name} ({len(valid)} nonzero of {len(vals)}, showing every {step}th) ---")
                # imu array: [robotYaw, robotYawRate, robotPitch, robotPitchRate, robotRoll, robotRollRate, ...]
                for v in show:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    arr = v.value
                    print(f"  t={t_rel:8.3f}s  yaw={arr[0]:7.1f}° yawRate={arr[1]:7.2f} pitch={arr[2]:6.1f}° roll={arr[4]:6.1f}°")
        
        # --- Reject reasons ---
        for cam_name, rr_sig in [("LL-A", rr_a), ("LL-B", rr_b), ("LL-C", rr_c)]:
            if not rr_sig:
                continue
            vals = get_window(rr_sig, t0_us, t_start, t_end)
            if vals:
                # Show transitions only
                prev = None
                print(f"\n--- rejectReason {cam_name} ({len(vals)} samples) ---")
                for v in vals:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    if v.value != prev:
                        print(f"  t={t_rel:8.3f}s  {v.value}")
                        prev = v.value

        # --- Vision errors ---
        for cam_name, ve_sig in [("LL-A", ve_a), ("LL-B", ve_b), ("LL-C", ve_c)]:
            if not ve_sig:
                continue
            vals = get_window(ve_sig, t0_us, t_start, t_end)
            if vals:
                # Show samples with large errors (> 0.3m)
                big = [v for v in vals if abs(v.value) > 0.3]
                if big:
                    print(f"\n--- visionError {cam_name} > 0.3m ({len(big)} of {len(vals)}) ---")
                    step = max(1, len(big) // 15)
                    for v in big[::step]:
                        t_rel = (v.timestamp_us - t0_us) / 1e6
                        print(f"  t={t_rel:8.3f}s  error={v.value:.3f}m")
        
        # --- Under defense ---
        if ud:
            vals = get_window(ud, t0_us, t_start, t_end)
            if vals:
                print(f"\n--- underDefense ({len(vals)} samples) ---")
                prev = None
                for v in vals:
                    t_rel = (v.timestamp_us - t0_us) / 1e6
                    if v.value != prev:
                        print(f"  t={t_rel:8.3f}s  {v.value}")
                        prev = v.value

    # === KEY COMPARISON: heading sent vs fused pose heading vs botpose yaw ===
    print(f"\n{'='*120}")
    print("=== HEADING COMPARISON: fused pose heading vs heading sent to LLs vs LL botpose yaw ===")
    print(f"{'='*120}")
    
    for label, t_start, t_end in [("Incident 1", 380.0, 387.0), ("Incident 2", 529.0, 537.0)]:
        print(f"\n--- {label} ---")
        print(f"{'time':>10s}  {'fused_hdg':>10s}  {'sent_hdg':>10s}  {'delta':>7s}  {'MT1_C_yaw':>10s}  {'MT2_C_yaw':>10s}  {'MT1_C_x':>8s}  {'MT1_C_y':>8s}  {'MT2_C_x':>8s}  {'MT2_C_y':>8s}  {'fused_x':>8s}  {'fused_y':>8s}")
        
        start_us = t0_us + int(t_start * 1e6)
        end_us = t0_us + int(t_end * 1e6)
        
        # Use robot_orientation_set timestamps as reference
        if not ro_c:
            continue
        ro_vals = get_window(ro_c, t0_us, t_start, t_end)
        
        for v in ro_vals:
            t_us = v.timestamp_us
            t_rel = (t_us - t0_us) / 1e6
            sent_heading = v.value[0] if isinstance(v.value, (list, tuple)) else 0.0
            
            # Find nearest fused pose
            fused_x, fused_y, fused_hdg = 0, 0, 0
            if pose_sig:
                best = None
                best_dt = 1e9
                for p in pose_sig.values:
                    dt = abs(p.timestamp_us - t_us)
                    if dt < best_dt:
                        best_dt = dt
                        best = p
                    elif dt > best_dt + 100000:
                        break
                if best and best_dt < 50000:
                    decoded = decode_pose2d(best.value)
                    if decoded:
                        fused_x, fused_y, fused_hdg = decoded
            
            # Find nearest MT1 botpose (camera C - most active)
            mt1_x, mt1_y, mt1_yaw = "-", "-", "-"
            if bp_c:
                for bp in bp_c.values:
                    if abs(bp.timestamp_us - t_us) < 50000:
                        arr = bp.value
                        if isinstance(arr, (list, tuple)) and len(arr) >= 11 and int(arr[7]) > 0:
                            mt1_x = f"{arr[0]:.3f}"
                            mt1_y = f"{arr[1]:.3f}"
                            mt1_yaw = f"{arr[5]:.1f}"
                        break
            
            # Find nearest MT2 botpose (camera C)
            mt2_x, mt2_y, mt2_yaw = "-", "-", "-"
            if bp_orb_c:
                for bp in bp_orb_c.values:
                    if abs(bp.timestamp_us - t_us) < 50000:
                        arr = bp.value
                        if isinstance(arr, (list, tuple)) and len(arr) >= 11 and int(arr[7]) > 0:
                            mt2_x = f"{arr[0]:.3f}"
                            mt2_y = f"{arr[1]:.3f}"
                            mt2_yaw = f"{arr[5]:.1f}"
                        break
            
            delta = fused_hdg - sent_heading
            delta = (delta + 180) % 360 - 180
            
            print(f"  {t_rel:8.3f}s  {fused_hdg:9.1f}°  {sent_heading:9.1f}°  {delta:6.1f}°  {mt1_yaw:>10s}  {mt2_yaw:>10s}  {mt1_x:>8s}  {mt1_y:>8s}  {mt2_x:>8s}  {mt2_y:>8s}  {fused_x:7.3f}  {fused_y:7.3f}")


if __name__ == "__main__":
    main()
