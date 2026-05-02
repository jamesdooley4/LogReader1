"""Simulate the proposed LaunchCalculator changes against logged data.

Compares the OLD (direct blend) and NEW (complementary slip filter with hysteresis)
approaches using the same DriveState/Pose and DriveState/Speeds data from the log.
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

# Decode poses with DriveState/Timestamp
poses = []
for v in pose_sig.values:
    x, y, th = decode_pose2d(v.value)
    poses.append((v.timestamp_us / 1e6, x, y, th))

ds_ts = [(v.timestamp_us / 1e6, v.value) for v in ts_sig.values]

# Build indexed wheel speeds by log timestamp
ws_list = []
for v in speeds_sig.values:
    vx, vy, omega = decode_chassis_speeds(v.value)
    ws_list.append((v.timestamp_us / 1e6, vx, vy, omega))
ws_times = [w[0] for w in ws_list]

def get_wheel_speed(t):
    idx = bisect.bisect_right(ws_times, t) - 1
    if idx < 0:
        idx = 0
    if idx >= len(ws_list):
        idx = len(ws_list) - 1
    if abs(ws_list[idx][0] - t) > 0.1:
        return 0.0, 0.0, 0.0
    return ws_list[idx][1], ws_list[idx][2], ws_list[idx][3]


# ============================================================
# Simulate BOTH algorithms
# ============================================================

# OLD constants
OLD_BLEND = 0.9
OLD_MIN_MOVING = 0.05
MIN_POSE_DT = 0.005
MAX_POSE_DT = 0.05

# NEW constants
NEW_BLEND = 0.6
NEW_ENABLE = 0.10
NEW_DISABLE = 0.03
SLIP_FILTER_ALPHA = 0.8

# Use DriveState/Timestamp for dt
pose_with_ts = []
for i in range(len(poses)):
    if i < len(ds_ts):
        pose_with_ts.append((ds_ts[i][1], poses[i][1], poses[i][2], poses[i][3], poses[i][0]))
    else:
        pose_with_ts.append((poses[i][0], poses[i][1], poses[i][2], poses[i][3], poses[i][0]))

# Run simulation
old_results = []  # (log_t, eff_speed_old, wheel_speed, pose_speed, dist)
new_results = []  # (log_t, eff_speed_new, wheel_speed, pose_speed, dist)

# NEW state
filtered_slip_x = 0.0
filtered_slip_y = 0.0
filtered_slip_omega = 0.0
pose_diff_active = False

for i in range(1, len(pose_with_ts)):
    t_prev, x_prev, y_prev, th_prev, _ = pose_with_ts[i - 1]
    t_curr, x_curr, y_curr, th_curr, log_t = pose_with_ts[i]
    dt = t_curr - t_prev

    ws_vx, ws_vy, ws_omega = get_wheel_speed(log_t)
    wheel_mag = math.hypot(ws_vx, ws_vy)

    # Distance to hub
    d = interp_nearest(dist_hub, log_t)

    # Pose-derived velocity (simplified: dx/dt, not full Twist2d.log)
    if dt > 0.001:
        pose_vx = (x_curr - x_prev) / dt
        pose_vy = (y_curr - y_prev) / dt
        pose_omega = (th_curr - th_prev) / dt
    else:
        pose_vx, pose_vy, pose_omega = 0, 0, 0
    pose_speed = math.hypot(pose_vx, pose_vy)

    poseDtValid = MIN_POSE_DT < dt < MAX_POSE_DT

    # ---- OLD algorithm ----
    old_eff_vx, old_eff_vy = ws_vx, ws_vy
    robot_is_moving_old = wheel_mag > OLD_MIN_MOVING
    if robot_is_moving_old and poseDtValid:
        blend = OLD_BLEND * (1.0 - (dt - MIN_POSE_DT) / (MAX_POSE_DT - MIN_POSE_DT))
        old_eff_vx = blend * pose_vx + (1 - blend) * ws_vx
        old_eff_vy = blend * pose_vy + (1 - blend) * ws_vy
    old_eff_speed = math.hypot(old_eff_vx, old_eff_vy)

    # ---- NEW algorithm ----
    if pose_diff_active:
        pose_diff_active = wheel_mag > NEW_DISABLE
    else:
        pose_diff_active = wheel_mag > NEW_ENABLE

    new_eff_vx, new_eff_vy = ws_vx, ws_vy
    if pose_diff_active and poseDtValid:
        slip_raw_x = pose_vx - ws_vx
        slip_raw_y = pose_vy - ws_vy
        filtered_slip_x = SLIP_FILTER_ALPHA * filtered_slip_x + (1 - SLIP_FILTER_ALPHA) * slip_raw_x
        filtered_slip_y = SLIP_FILTER_ALPHA * filtered_slip_y + (1 - SLIP_FILTER_ALPHA) * slip_raw_y
        blend = NEW_BLEND * (1.0 - (dt - MIN_POSE_DT) / (MAX_POSE_DT - MIN_POSE_DT))
        new_eff_vx = ws_vx + blend * filtered_slip_x
        new_eff_vy = ws_vy + blend * filtered_slip_y
    elif not poseDtValid:
        filtered_slip_x = 0.0
        filtered_slip_y = 0.0
        filtered_slip_omega = 0.0
    new_eff_speed = math.hypot(new_eff_vx, new_eff_vy)

    old_results.append((log_t, old_eff_speed, wheel_mag, pose_speed, d, dt,
                         old_eff_vx, old_eff_vy))
    new_results.append((log_t, new_eff_speed, wheel_mag, pose_speed, d, dt,
                         new_eff_vx, new_eff_vy))


# ============================================================
# Analysis
# ============================================================

print("\n" + "=" * 70)
print("EFFECTIVE SPEED: OLD vs NEW — OVERALL")
print("=" * 70)

old_speeds = [r[1] for r in old_results if r[5] > MIN_POSE_DT and r[5] < MAX_POSE_DT]
new_speeds = [r[1] for r in new_results if r[5] > MIN_POSE_DT and r[5] < MAX_POSE_DT]
wheel_speeds_valid = [r[2] for r in old_results if r[5] > MIN_POSE_DT and r[5] < MAX_POSE_DT]

for label, vals in [("Old effective", old_speeds), ("New effective", new_speeds),
                    ("Wheel only", wheel_speeds_valid)]:
    s = sorted(vals)
    n = len(s)
    print(f"\n{label} ({n} samples):")
    for pct in [50, 90, 95, 99, 99.5, 100]:
        idx = min(int(n * pct / 100), n - 1)
        print(f"  P{pct}: {s[idx]:.4f} m/s")


# ============================================================
# Noise: |effective - wheel| when robot is nearly stationary (wheel < 0.15)
# ============================================================
print("\n" + "=" * 70)
print("NOISE INJECTION: |effective - wheel_speed| when wheels < 0.15 m/s")
print("=" * 70)

old_noise = []
new_noise = []
for i in range(len(old_results)):
    if old_results[i][2] < 0.15 and old_results[i][5] > MIN_POSE_DT and old_results[i][5] < MAX_POSE_DT:
        old_n = abs(old_results[i][1] - old_results[i][2])
        new_n = abs(new_results[i][1] - new_results[i][2])
        old_noise.append((old_results[i][0], old_n, old_results[i][2]))
        new_noise.append((new_results[i][0], new_n, new_results[i][2]))

old_noise_vals = sorted([n[1] for n in old_noise])
new_noise_vals = sorted([n[1] for n in new_noise])
n = len(old_noise_vals)
print(f"\nSamples where wheels < 0.15 m/s: {n}")
print(f"{'Metric':>12s}  {'Old':>10s}  {'New':>10s}  {'Reduction':>10s}")
for pct in [50, 90, 95, 99, 99.5, 100]:
    idx = min(int(n * pct / 100), n - 1)
    ov = old_noise_vals[idx]
    nv = new_noise_vals[idx]
    reduction = (1 - nv / ov) * 100 if ov > 0 else 0
    print(f"  P{pct:>5}  {ov:10.4f}  {nv:10.4f}  {reduction:9.1f}%")


# ============================================================
# Effective speed spikes: how many exceed threshold while wheels < 0.15
# ============================================================
print("\n" + "=" * 70)
print("SPIKE COUNTS: effective_speed > threshold while wheels < 0.15")
print("=" * 70)

for thresh in [0.3, 0.5, 1.0, 2.0]:
    old_cnt = sum(1 for r in old_results if r[2] < 0.15 and r[1] > thresh
                  and r[5] > MIN_POSE_DT and r[5] < MAX_POSE_DT)
    new_cnt = sum(1 for r in new_results if r[2] < 0.15 and r[1] > thresh
                  and r[5] > MIN_POSE_DT and r[5] < MAX_POSE_DT)
    print(f"  > {thresh:.1f} m/s:  OLD={old_cnt:5d}  NEW={new_cnt:5d}")


# ============================================================
# Hysteresis effect: compare which samples are active
# ============================================================
print("\n" + "=" * 70)
print("GATE ACTIVATION: OLD (simple threshold) vs NEW (hysteresis)")
print("=" * 70)

old_active = sum(1 for r in old_results if r[2] > OLD_MIN_MOVING)
# Replay hysteresis
hyst_active = False
new_active_cnt = 0
hyst_transitions = 0
old_transitions = 0
prev_old_active = False
for r in old_results:
    wm = r[2]
    cur_old = wm > OLD_MIN_MOVING
    if cur_old != prev_old_active:
        old_transitions += 1
    prev_old_active = cur_old

    prev_hyst = hyst_active
    if hyst_active:
        hyst_active = wm > NEW_DISABLE
    else:
        hyst_active = wm > NEW_ENABLE
    if hyst_active != prev_hyst:
        hyst_transitions += 1
    if hyst_active:
        new_active_cnt += 1

print(f"Old gate active: {old_active}/{len(old_results)} ({100*old_active/len(old_results):.1f}%)")
print(f"New gate active: {new_active_cnt}/{len(new_results)} ({100*new_active_cnt/len(new_results):.1f}%)")
print(f"Old gate transitions (on/off): {old_transitions}")
print(f"New gate transitions (on/off): {hyst_transitions}")


# ============================================================
# Slip filter response to a REAL push scenario
# ============================================================
print("\n" + "=" * 70)
print("SLIP FILTER: RESPONSE TO SUSTAINED PUSH EVENTS")
print("=" * 70)
# Find windows where pose_speed >> wheel_speed for multiple consecutive frames
# (sustained push, not just a vision correction)

# Look at filtered slip magnitude over time at those windows
sustained_push = []
window = 5  # consecutive frames
for i in range(window, len(old_results)):
    # Check if ALL of the last `window` frames had pose_speed > wheel_speed + 0.2
    all_pushed = all(
        old_results[j][3] > old_results[j][2] + 0.2
        and old_results[j][2] > 0.1  # robot is moving
        for j in range(i - window, i)
    )
    if all_pushed:
        sustained_push.append(i)

if sustained_push:
    # Show a few examples
    shown = set()
    examples = 0
    for idx in sustained_push:
        t = old_results[idx][0]
        t_bucket = round(t, 0)
        if t_bucket in shown:
            continue
        shown.add(t_bucket)
        examples += 1
        if examples > 5:
            break
        print(f"\n  Push event near t={t:.2f}s:")
        start = max(0, idx - 3)
        end = min(len(old_results), idx + 5)
        print(f"  {'t':>8s}  {'WhlSpd':>7s}  {'PoseSpd':>8s}  {'OldEff':>7s}  {'NewEff':>7s}  "
              f"{'OldErr':>7s}  {'NewErr':>7s}  {'Dist':>5s}")
        for j in range(start, end):
            r_old = old_results[j]
            r_new = new_results[j]
            # "Error" = how far effective speed is from pose-derived (ground truth during push)
            old_err = abs(r_old[1] - r_old[3])
            new_err = abs(r_new[1] - r_new[3])
            d_str = f"{r_old[4]:.1f}" if r_old[4] else "?"
            marker = " <--" if j == idx else ""
            print(f"  {r_old[0]:8.3f}  {r_old[2]:7.3f}  {r_old[3]:8.3f}  {r_old[1]:7.3f}  "
                  f"{r_new[1]:7.3f}  {old_err:7.3f}  {new_err:7.3f}  {d_str:>5s}{marker}")
else:
    print("  No sustained push events found (>5 consecutive frames with pose >> wheels)")


# ============================================================
# Frame-to-frame jitter in effective speed (variability measure)
# ============================================================
print("\n" + "=" * 70)
print("EFFECTIVE SPEED FRAME-TO-FRAME JITTER (|eff[i] - eff[i-1]|)")
print("=" * 70)

old_ff_jitter = []
new_ff_jitter = []
for i in range(1, len(old_results)):
    if (old_results[i][5] > MIN_POSE_DT and old_results[i][5] < MAX_POSE_DT
        and old_results[i-1][5] > MIN_POSE_DT and old_results[i-1][5] < MAX_POSE_DT):
        old_ff_jitter.append(abs(old_results[i][1] - old_results[i-1][1]))
        new_ff_jitter.append(abs(new_results[i][1] - new_results[i-1][1]))

old_ff = sorted(old_ff_jitter)
new_ff = sorted(new_ff_jitter)
n = len(old_ff)
print(f"\nSamples: {n}")
print(f"{'Metric':>12s}  {'Old':>10s}  {'New':>10s}  {'Reduction':>10s}")
for pct in [50, 90, 95, 99, 99.5, 100]:
    idx = min(int(n * pct / 100), n - 1)
    ov = old_ff[idx]
    nv = new_ff[idx]
    reduction = (1 - nv / ov) * 100 if ov > 0 else 0
    print(f"  P{pct:>5}  {ov:10.4f}  {nv:10.4f}  {reduction:9.1f}%")


# ============================================================
# Worst-case: biggest single-frame effective speed under NEW that
# exceeds wheel speed while robot is slow
# ============================================================
print("\n" + "=" * 70)
print("TOP 15 REMAINING NOISE SPIKES UNDER NEW ALGORITHM (wheels < 0.15)")
print("=" * 70)

new_spikes = [(new_results[i][0], new_results[i][1], new_results[i][2],
               old_results[i][1], new_results[i][4])
              for i in range(len(new_results))
              if new_results[i][2] < 0.15 and new_results[i][1] > 0.2
              and new_results[i][5] > MIN_POSE_DT and new_results[i][5] < MAX_POSE_DT]
new_spikes.sort(key=lambda x: x[1], reverse=True)
print(f"{'Time':>10s}  {'NewEff':>8s}  {'OldEff':>8s}  {'Wheels':>8s}  {'Dist':>6s}")
for s in new_spikes[:15]:
    d_str = f"{s[4]:.1f}" if s[4] else "?"
    print(f"  {s[0]:8.3f}  {s[1]:8.3f}  {s[3]:8.3f}  {s[2]:8.3f}  {d_str:>6s}")


print("\n=== SIMULATION COMPLETE ===")
