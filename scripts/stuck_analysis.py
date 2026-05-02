"""Analyze wheel-slip-against-obstacle events in the log.

Look for frames where wheels say the robot is moving but pose says it isn't
(robot stuck against a field element). Compare old vs new algorithm response.
"""

from __future__ import annotations
import sys, math, struct as pystruct, bisect
from pathlib import Path
from collections import defaultdict

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from logreader.wpilog_reader import read_wpilog

LOG_PATH = r"C:\Users\jdool_46clpzz\Documents\FRC_Sessions\.staging\logs\FRC_20260425_202750.wpilog"
print(f"Loading {LOG_PATH} ...")
log = read_wpilog(LOG_PATH)

def decode_pose2d(raw):
    if len(raw) != 24:
        return (float("nan"), float("nan"), float("nan"))
    return pystruct.unpack("<ddd", raw)

def decode_chassis_speeds(raw):
    if len(raw) != 24:
        return (float("nan"), float("nan"), float("nan"))
    return pystruct.unpack("<ddd", raw)

def ts_val(sig):
    return [(v.timestamp_us / 1e6, v.value) for v in sig.values]

def interp_nearest(series, t, max_gap=0.1):
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    best = lo
    if best > 0 and abs(series[best - 1][0] - t) < abs(series[best][0] - t):
        best -= 1
    if abs(series[best][0] - t) > max_gap:
        return None
    return series[best][1]


# Load signals
pose_sig = log.signals["NT:/DriveState/Pose"]
ts_sig = log.signals["NT:/DriveState/Timestamp"]
speeds_sig = log.signals["NT:/DriveState/Speeds"]
dist_hub = ts_val(log.signals["NT:/SmartDashboard/LiveLauncherData/Turret to hub distance"])

poses = []
for v in pose_sig.values:
    x, y, th = decode_pose2d(v.value)
    poses.append((v.timestamp_us / 1e6, x, y, th))

ds_ts = [(v.timestamp_us / 1e6, v.value) for v in ts_sig.values]

ws_list = []
for v in speeds_sig.values:
    vx, vy, omega = decode_chassis_speeds(v.value)
    ws_list.append((v.timestamp_us / 1e6, vx, vy, omega))
ws_times = [w[0] for w in ws_list]

def get_wheel_speed(t):
    idx = bisect.bisect_right(ws_times, t) - 1
    if idx < 0: idx = 0
    if idx >= len(ws_list): idx = len(ws_list) - 1
    if abs(ws_list[idx][0] - t) > 0.1:
        return 0.0, 0.0, 0.0
    return ws_list[idx][1], ws_list[idx][2], ws_list[idx][3]


# Build indexed data
pose_with_ts = []
for i in range(len(poses)):
    if i < len(ds_ts):
        pose_with_ts.append((ds_ts[i][1], poses[i][1], poses[i][2], poses[i][3], poses[i][0]))
    else:
        pose_with_ts.append((poses[i][0], poses[i][1], poses[i][2], poses[i][3], poses[i][0]))


MIN_POSE_DT = 0.005
MAX_POSE_DT = 0.05

# ============================================================
# 1. Find "stuck" events: wheels > 0.3 but pose_speed < 0.1
# ============================================================
print("\n" + "=" * 70)
print("1. STUCK EVENTS: wheels > 0.3 m/s but pose_speed < 0.1 m/s")
print("=" * 70)

stuck_frames = []
for i in range(1, len(pose_with_ts)):
    t_prev, x_prev, y_prev, th_prev, _ = pose_with_ts[i - 1]
    t_curr, x_curr, y_curr, th_curr, log_t = pose_with_ts[i]
    dt = t_curr - t_prev
    if dt < MIN_POSE_DT or dt > MAX_POSE_DT:
        continue

    ws_vx, ws_vy, ws_omega = get_wheel_speed(log_t)
    wheel_mag = math.hypot(ws_vx, ws_vy)

    pose_vx = (x_curr - x_prev) / dt
    pose_vy = (y_curr - y_prev) / dt
    pose_speed = math.hypot(pose_vx, pose_vy)

    if wheel_mag > 0.3 and pose_speed < 0.1:
        d = interp_nearest(dist_hub, log_t)
        stuck_frames.append((log_t, wheel_mag, pose_speed, dt, d,
                             ws_vx, ws_vy, pose_vx, pose_vy))

print(f"Found {len(stuck_frames)} stuck frames")

# Also find: wheels > 0.2 and pose_speed < wheel_speed * 0.3 (significant mismatch)
mismatch_frames = []
for i in range(1, len(pose_with_ts)):
    t_prev, x_prev, y_prev, th_prev, _ = pose_with_ts[i - 1]
    t_curr, x_curr, y_curr, th_curr, log_t = pose_with_ts[i]
    dt = t_curr - t_prev
    if dt < MIN_POSE_DT or dt > MAX_POSE_DT:
        continue

    ws_vx, ws_vy, ws_omega = get_wheel_speed(log_t)
    wheel_mag = math.hypot(ws_vx, ws_vy)

    pose_vx = (x_curr - x_prev) / dt
    pose_vy = (y_curr - y_prev) / dt
    pose_speed = math.hypot(pose_vx, pose_vy)

    if wheel_mag > 0.2 and pose_speed < wheel_mag * 0.3:
        d = interp_nearest(dist_hub, log_t)
        mismatch_frames.append((log_t, wheel_mag, pose_speed, dt, d,
                                ws_vx, ws_vy, pose_vx, pose_vy))

print(f"Found {len(mismatch_frames)} mismatch frames (pose < 30% of wheels, wheels > 0.2)")

if stuck_frames:
    # Group into events (consecutive frames within 200ms)
    events = []
    current_event = [stuck_frames[0]]
    for f in stuck_frames[1:]:
        if f[0] - current_event[-1][0] < 0.2:
            current_event.append(f)
        else:
            events.append(current_event)
            current_event = [f]
    events.append(current_event)

    print(f"\nGrouped into {len(events)} distinct events:")
    for ev in events:
        t_start = ev[0][0]
        t_end = ev[-1][0]
        dur = t_end - t_start
        avg_wheel = sum(f[1] for f in ev) / len(ev)
        avg_pose = sum(f[2] for f in ev) / len(ev)
        print(f"  t={t_start:.3f}-{t_end:.3f} ({dur*1000:.0f}ms, {len(ev)} frames)  "
              f"avg_wheel={avg_wheel:.3f}  avg_pose={avg_pose:.3f}  "
              f"dist={ev[0][4]:.1f}m" if ev[0][4] else "")


# ============================================================
# 2. Simulate OLD, NEW (alpha=0.8), NEW (alpha=0.5) on stuck events
# ============================================================
print("\n" + "=" * 70)
print("2. ALGORITHM COMPARISON ON STUCK/MISMATCH EVENTS")
print("=" * 70)

# Find all mismatch event windows and show context around them
# Group mismatches into events
if mismatch_frames:
    m_events = []
    current_event = [mismatch_frames[0]]
    for f in mismatch_frames[1:]:
        if f[0] - current_event[-1][0] < 0.2:
            current_event.append(f)
        else:
            m_events.append(current_event)
            current_event = [f]
    m_events.append(current_event)
else:
    m_events = []

print(f"\nMismatch events (pose < 30% wheels): {len(m_events)}")

# For each event, simulate all 3 algorithms with context
# We need to run the full simulation to get filter state right
# Run full timeline simulation with all 3 variants

OLD_BLEND = 0.9
OLD_MIN_MOVING = 0.05
NEW_BLEND = 0.6
NEW_ENABLE = 0.10
NEW_DISABLE = 0.03

all_frames = []  # (log_t, wheel_mag, pose_speed, old_eff, new08_eff, new05_eff, dt, ws_vx, ws_vy, pose_vx, pose_vy)

# State for new (alpha=0.8)
slip08_x, slip08_y = 0.0, 0.0
gate08_active = False
# State for new (alpha=0.5)
slip05_x, slip05_y = 0.0, 0.0
gate05_active = False

for i in range(1, len(pose_with_ts)):
    t_prev, x_prev, y_prev, th_prev, _ = pose_with_ts[i - 1]
    t_curr, x_curr, y_curr, th_curr, log_t = pose_with_ts[i]
    dt = t_curr - t_prev

    ws_vx, ws_vy, ws_omega = get_wheel_speed(log_t)
    wheel_mag = math.hypot(ws_vx, ws_vy)

    if dt > 0.001:
        pose_vx = (x_curr - x_prev) / dt
        pose_vy = (y_curr - y_prev) / dt
    else:
        pose_vx, pose_vy = 0, 0
    pose_speed = math.hypot(pose_vx, pose_vy)
    poseDtValid = MIN_POSE_DT < dt < MAX_POSE_DT

    # OLD
    old_vx, old_vy = ws_vx, ws_vy
    if wheel_mag > OLD_MIN_MOVING and poseDtValid:
        blend = OLD_BLEND * (1.0 - (dt - MIN_POSE_DT) / (MAX_POSE_DT - MIN_POSE_DT))
        old_vx = blend * pose_vx + (1 - blend) * ws_vx
        old_vy = blend * pose_vy + (1 - blend) * ws_vy
    old_eff = math.hypot(old_vx, old_vy)

    # NEW alpha=0.8
    if gate08_active:
        gate08_active = wheel_mag > NEW_DISABLE
    else:
        gate08_active = wheel_mag > NEW_ENABLE
    new08_vx, new08_vy = ws_vx, ws_vy
    if gate08_active and poseDtValid:
        sr_x = pose_vx - ws_vx
        sr_y = pose_vy - ws_vy
        slip08_x = 0.8 * slip08_x + 0.2 * sr_x
        slip08_y = 0.8 * slip08_y + 0.2 * sr_y
        blend = NEW_BLEND * (1.0 - (dt - MIN_POSE_DT) / (MAX_POSE_DT - MIN_POSE_DT))
        new08_vx = ws_vx + blend * slip08_x
        new08_vy = ws_vy + blend * slip08_y
    elif not poseDtValid:
        slip08_x, slip08_y = 0.0, 0.0
    new08_eff = math.hypot(new08_vx, new08_vy)

    # NEW alpha=0.5
    if gate05_active:
        gate05_active = wheel_mag > NEW_DISABLE
    else:
        gate05_active = wheel_mag > NEW_ENABLE
    new05_vx, new05_vy = ws_vx, ws_vy
    if gate05_active and poseDtValid:
        sr_x = pose_vx - ws_vx
        sr_y = pose_vy - ws_vy
        slip05_x = 0.5 * slip05_x + 0.5 * sr_x
        slip05_y = 0.5 * slip05_y + 0.5 * sr_y
        blend = NEW_BLEND * (1.0 - (dt - MIN_POSE_DT) / (MAX_POSE_DT - MIN_POSE_DT))
        new05_vx = ws_vx + blend * slip05_x
        new05_vy = ws_vy + blend * slip05_y
    elif not poseDtValid:
        slip05_x, slip05_y = 0.0, 0.0
    new05_eff = math.hypot(new05_vx, new05_vy)

    all_frames.append((log_t, wheel_mag, pose_speed, old_eff, new08_eff, new05_eff,
                        dt, ws_vx, ws_vy, pose_vx, pose_vy))

# Build lookup by time
frame_by_time = {f[0]: f for f in all_frames}
frame_times = [f[0] for f in all_frames]

# Show context around each mismatch event
shown_count = 0
for ev in m_events:
    if len(ev) < 2:
        continue  # skip single-frame glitches
    shown_count += 1
    if shown_count > 10:
        break

    t_center = ev[len(ev) // 2][0]
    # Find this time in all_frames
    idx = bisect.bisect_left(frame_times, ev[0][0]) - 3
    idx = max(0, idx)
    end_idx = bisect.bisect_right(frame_times, ev[-1][0]) + 5
    end_idx = min(len(all_frames), end_idx)

    dur = ev[-1][0] - ev[0][0]
    d = ev[0][4]
    d_str = f"{d:.1f}m" if d else "?"
    print(f"\n--- Event at t={ev[0][0]:.3f} ({dur*1000:.0f}ms, {len(ev)} frames, dist={d_str}) ---")
    print(f"  Ground truth = pose (vision correct, wheels slipping)")
    print(f"  {'t':>8s}  {'Wheels':>7s}  {'Pose':>7s}  {'Old':>7s}  {'New.8':>7s}  {'New.5':>7s}  "
          f"{'OldErr':>7s}  {'N8Err':>7s}  {'N5Err':>7s}  {'Note':s}")
    for j in range(idx, end_idx):
        f = all_frames[j]
        log_t, wm, ps, old_e, n8_e, n5_e = f[0], f[1], f[2], f[3], f[4], f[5]
        # Error = distance from pose_speed (ground truth in this scenario)
        old_err = abs(old_e - ps)
        n8_err = abs(n8_e - ps)
        n5_err = abs(n5_e - ps)
        is_stuck = any(abs(sf[0] - log_t) < 0.001 for sf in ev)
        note = " <STUCK" if is_stuck else ""
        print(f"  {log_t:8.3f}  {wm:7.3f}  {ps:7.3f}  {old_e:7.3f}  {n8_e:7.3f}  {n5_e:7.3f}  "
              f"{old_err:7.3f}  {n8_err:7.3f}  {n5_err:7.3f}  {note}")


# ============================================================
# 3. Overall accuracy when wheels disagree with pose
# ============================================================
print("\n" + "=" * 70)
print("3. ERROR vs GROUND TRUTH (pose) DURING ALL MISMATCH FRAMES")
print("=" * 70)

# Collect errors for all mismatch frames
old_errs = []
n8_errs = []
n5_errs = []
wheel_errs = []

for mf in mismatch_frames:
    t = mf[0]
    pose_speed = mf[2]
    # Find in all_frames
    idx = bisect.bisect_left(frame_times, t)
    if idx >= len(all_frames):
        continue
    if abs(all_frames[idx][0] - t) > 0.01:
        continue
    f = all_frames[idx]
    old_errs.append(abs(f[3] - pose_speed))
    n8_errs.append(abs(f[4] - pose_speed))
    n5_errs.append(abs(f[5] - pose_speed))
    wheel_errs.append(abs(f[1] - pose_speed))

if old_errs:
    for label, errs in [("Wheel only", wheel_errs), ("Old blend", old_errs),
                         ("New a=0.8", n8_errs), ("New a=0.5", n5_errs)]:
        s = sorted(errs)
        n = len(s)
        print(f"  {label:12s}:  med={s[n//2]:.3f}  P90={s[int(n*0.9)]:.3f}  "
              f"P95={s[int(n*0.95)]:.3f}  max={s[-1]:.3f}  n={n}")
else:
    print("  No mismatch frames found")


# ============================================================
# 4. How fast does each algorithm converge when stuck?
# ============================================================
print("\n" + "=" * 70)
print("4. CONVERGENCE SPEED: frames to reach <50% error after stuck onset")
print("=" * 70)

for ev in m_events:
    if len(ev) < 3:
        continue
    t_start = ev[0][0]
    # Find frame index
    idx = bisect.bisect_left(frame_times, t_start)
    if idx >= len(all_frames) or abs(all_frames[idx][0] - t_start) > 0.01:
        continue

    # Check how many frames until error < 50% of initial for each algo
    initial_wheel = all_frames[idx][1]  # wheel speed at onset
    if initial_wheel < 0.2:
        continue

    old_conv, n8_conv, n5_conv = None, None, None
    for j in range(idx, min(idx + 30, len(all_frames))):
        f = all_frames[j]
        ps = f[2]  # pose = truth
        if old_conv is None and abs(f[3] - ps) < initial_wheel * 0.5:
            old_conv = j - idx
        if n8_conv is None and abs(f[4] - ps) < initial_wheel * 0.5:
            n8_conv = j - idx
        if n5_conv is None and abs(f[5] - ps) < initial_wheel * 0.5:
            n5_conv = j - idx

    d = ev[0][4]
    d_str = f"{d:.1f}" if d else "?"
    print(f"  t={t_start:.3f} (wheel={initial_wheel:.2f}, dist={d_str}m): "
          f"Old={old_conv if old_conv is not None else '>30'} frames  "
          f"New0.8={n8_conv if n8_conv is not None else '>30'} frames  "
          f"New0.5={n5_conv if n5_conv is not None else '>30'} frames")


print("\n=== ANALYSIS COMPLETE ===")
