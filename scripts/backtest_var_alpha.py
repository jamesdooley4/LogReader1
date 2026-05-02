"""Back-test the variable-alpha slip filter against logged data.

Compares four algorithms:
  OLD  — direct blend (POSE_VELOCITY_BLEND=0.9, single threshold)
  V1   — fixed-alpha slip filter (alpha=0.8, from first proposal)
  V2   — variable-alpha slip filter (current proposal in working tree)
  WHEEL — pure wheel speeds (baseline)

Evaluates noise rejection, stuck/obstacle response, stale-slip, and convergence.
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
    if len(raw) != 24: return (float("nan"), float("nan"), float("nan"))
    return pystruct.unpack("<ddd", raw)

def decode_chassis_speeds(raw):
    if len(raw) != 24: return (float("nan"), float("nan"), float("nan"))
    return pystruct.unpack("<ddd", raw)

def ts_val(sig):
    return [(v.timestamp_us / 1e6, v.value) for v in sig.values]

def interp_nearest(series, t, max_gap=0.1):
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t: lo = mid + 1
        else: hi = mid
    best = lo
    if best > 0 and abs(series[best-1][0] - t) < abs(series[best][0] - t): best -= 1
    if abs(series[best][0] - t) > max_gap: return None
    return series[best][1]


# ── Load signals ──
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
    if abs(ws_list[idx][0] - t) > 0.1: return 0.0, 0.0, 0.0
    return ws_list[idx][1], ws_list[idx][2], ws_list[idx][3]

pose_with_ts = []
for i in range(len(poses)):
    if i < len(ds_ts):
        pose_with_ts.append((ds_ts[i][1], poses[i][1], poses[i][2], poses[i][3], poses[i][0]))
    else:
        pose_with_ts.append((poses[i][0], poses[i][1], poses[i][2], poses[i][3], poses[i][0]))


# ── Constants ──
MIN_POSE_DT = 0.005
MAX_POSE_DT = 0.05

# OLD
OLD_BLEND = 0.9
OLD_MIN_MOVING = 0.05

# V1 (fixed alpha=0.8)
V1_BLEND = 0.6
V1_ENABLE = 0.10
V1_DISABLE = 0.03
V1_ALPHA = 0.8

# V2 (variable alpha — current proposal)
V2_BLEND = 0.9
V2_ENABLE = 0.10
V2_DISABLE = 0.03
V2_ALPHA_SLOW = 0.85
V2_ALPHA_FAST = 0.20
V2_FAST_ENTER = 1.0
V2_FAST_EXIT = 0.5
V2_MAX_SLIP = 5.0


# ── Simulate all algorithms ──

# State: V1
v1_slip_x, v1_slip_y = 0.0, 0.0
v1_gate = False

# State: V2
v2_slip_x, v2_slip_y = 0.0, 0.0
v2_gate = False
v2_fast = False

results = []  # each: dict with keys for each algo

for i in range(1, len(pose_with_ts)):
    t_prev, x_prev, y_prev, th_prev, _ = pose_with_ts[i-1]
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

    d = interp_nearest(dist_hub, log_t)

    # ─── OLD ───
    old_vx, old_vy = ws_vx, ws_vy
    if wheel_mag > OLD_MIN_MOVING and poseDtValid:
        blend = OLD_BLEND * (1.0 - (dt - MIN_POSE_DT) / (MAX_POSE_DT - MIN_POSE_DT))
        old_vx = blend * pose_vx + (1 - blend) * ws_vx
        old_vy = blend * pose_vy + (1 - blend) * ws_vy
    old_eff = math.hypot(old_vx, old_vy)

    # ─── V1 (fixed alpha=0.8) ───
    if v1_gate: v1_gate = wheel_mag > V1_DISABLE
    else: v1_gate = wheel_mag > V1_ENABLE
    v1_vx, v1_vy = ws_vx, ws_vy
    if v1_gate and poseDtValid:
        sr_x = pose_vx - ws_vx
        sr_y = pose_vy - ws_vy
        v1_slip_x = V1_ALPHA * v1_slip_x + (1 - V1_ALPHA) * sr_x
        v1_slip_y = V1_ALPHA * v1_slip_y + (1 - V1_ALPHA) * sr_y
        blend = V1_BLEND * (1.0 - (dt - MIN_POSE_DT) / (MAX_POSE_DT - MIN_POSE_DT))
        v1_vx = ws_vx + blend * v1_slip_x
        v1_vy = ws_vy + blend * v1_slip_y
    elif not poseDtValid:
        v1_slip_x, v1_slip_y = 0.0, 0.0
    v1_eff = math.hypot(v1_vx, v1_vy)

    # ─── V2 (variable alpha) ───
    if v2_gate: v2_gate = wheel_mag > V2_DISABLE
    else: v2_gate = wheel_mag > V2_ENABLE
    v2_vx, v2_vy = ws_vx, ws_vy
    if v2_gate and poseDtValid:
        sr_x = pose_vx - ws_vx
        sr_y = pose_vy - ws_vy
        slip_raw_mag = math.hypot(sr_x, sr_y)

        # Variable alpha with hysteresis
        if v2_fast: v2_fast = slip_raw_mag > V2_FAST_EXIT
        else: v2_fast = slip_raw_mag > V2_FAST_ENTER
        alpha = V2_ALPHA_FAST if v2_fast else V2_ALPHA_SLOW

        new_sx = alpha * v2_slip_x + (1 - alpha) * sr_x
        new_sy = alpha * v2_slip_y + (1 - alpha) * sr_y

        # Safety cap
        new_smag = math.hypot(new_sx, new_sy)
        if new_smag > V2_MAX_SLIP:
            scale = V2_MAX_SLIP / new_smag
            new_sx *= scale
            new_sy *= scale

        v2_slip_x, v2_slip_y = new_sx, new_sy
        blend = V2_BLEND * (1.0 - (dt - MIN_POSE_DT) / (MAX_POSE_DT - MIN_POSE_DT))
        v2_vx = ws_vx + blend * v2_slip_x
        v2_vy = ws_vy + blend * v2_slip_y
    elif not poseDtValid:
        v2_slip_x, v2_slip_y = 0.0, 0.0
        v2_fast = False
    v2_eff = math.hypot(v2_vx, v2_vy)

    results.append({
        't': log_t, 'dt': dt, 'wheel_mag': wheel_mag, 'pose_speed': pose_speed,
        'old_eff': old_eff, 'v1_eff': v1_eff, 'v2_eff': v2_eff, 'dist': d,
        'v2_fast': v2_fast, 'v2_slip_mag': math.hypot(v2_slip_x, v2_slip_y),
        'valid': poseDtValid,
    })


valid = [r for r in results if r['valid']]
print(f"\nTotal frames: {len(results)}, valid poseDt: {len(valid)}")


# ══════════════════════════════════════════════════════════════
# 1. OVERALL EFFECTIVE SPEED DISTRIBUTION
# ══════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("1. EFFECTIVE SPEED DISTRIBUTION (valid poseDt only)")
print("=" * 70)

for label, key in [("Wheel only", "wheel_mag"), ("Old blend", "old_eff"),
                   ("V1 fixed-a", "v1_eff"), ("V2 var-a", "v2_eff")]:
    vals = sorted(r[key] for r in valid)
    n = len(vals)
    print(f"\n  {label}:")
    for pct in [50, 90, 95, 99, 99.5, 100]:
        idx = min(int(n * pct / 100), n - 1)
        print(f"    P{pct}: {vals[idx]:.4f} m/s")


# ══════════════════════════════════════════════════════════════
# 2. NOISE: |eff - wheel| when wheels < 0.15 m/s
# ══════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("2. NOISE INJECTION: |effective - wheel| when wheels < 0.15 m/s")
print("=" * 70)

slow = [r for r in valid if r['wheel_mag'] < 0.15]
n = len(slow)
print(f"  Samples: {n}")
print(f"  {'Pct':>6}  {'Old':>10}  {'V1(0.8)':>10}  {'V2(var)':>10}")
for pct in [50, 90, 95, 99, 99.5, 100]:
    idx = min(int(n * pct / 100), n - 1)
    old_v = sorted(abs(r['old_eff'] - r['wheel_mag']) for r in slow)[idx]
    v1_v = sorted(abs(r['v1_eff'] - r['wheel_mag']) for r in slow)[idx]
    v2_v = sorted(abs(r['v2_eff'] - r['wheel_mag']) for r in slow)[idx]
    print(f"  P{pct:>5}  {old_v:10.4f}  {v1_v:10.4f}  {v2_v:10.4f}")


# ══════════════════════════════════════════════════════════════
# 3. SPIKE COUNTS
# ══════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("3. SPIKES: effective > threshold while wheels < 0.15")
print("=" * 70)

for thresh in [0.3, 0.5, 1.0, 2.0]:
    old_c = sum(1 for r in slow if r['old_eff'] > thresh)
    v1_c = sum(1 for r in slow if r['v1_eff'] > thresh)
    v2_c = sum(1 for r in slow if r['v2_eff'] > thresh)
    print(f"  > {thresh:.1f}:  Old={old_c:5d}  V1={v1_c:5d}  V2={v2_c:5d}")


# ══════════════════════════════════════════════════════════════
# 4. FRAME-TO-FRAME JITTER IN EFFECTIVE SPEED
# ══════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("4. FRAME-TO-FRAME JITTER |eff[i] - eff[i-1]|")
print("=" * 70)

old_jit, v1_jit, v2_jit = [], [], []
for i in range(1, len(valid)):
    old_jit.append(abs(valid[i]['old_eff'] - valid[i-1]['old_eff']))
    v1_jit.append(abs(valid[i]['v1_eff'] - valid[i-1]['v1_eff']))
    v2_jit.append(abs(valid[i]['v2_eff'] - valid[i-1]['v2_eff']))

old_jit.sort(); v1_jit.sort(); v2_jit.sort()
n = len(old_jit)
print(f"  Samples: {n}")
print(f"  {'Pct':>6}  {'Old':>10}  {'V1(0.8)':>10}  {'V2(var)':>10}")
for pct in [50, 90, 95, 99, 99.5, 100]:
    idx = min(int(n * pct / 100), n - 1)
    print(f"  P{pct:>5}  {old_jit[idx]:10.4f}  {v1_jit[idx]:10.4f}  {v2_jit[idx]:10.4f}")


# ══════════════════════════════════════════════════════════════
# 5. STUCK/OBSTACLE EVENTS (wheels > 0.3, pose < 0.1)
# ══════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("5. STUCK EVENTS: ERROR vs POSE (ground truth)")
print("=" * 70)

mismatch = [r for r in valid if r['wheel_mag'] > 0.2 and r['pose_speed'] < r['wheel_mag'] * 0.3]
n = len(mismatch)
if n > 0:
    print(f"  Mismatch frames (pose < 30% wheels, wheels > 0.2): {n}")
    for label, key in [("Wheel", "wheel_mag"), ("Old", "old_eff"),
                       ("V1(0.8)", "v1_eff"), ("V2(var)", "v2_eff")]:
        errs = sorted(abs(r[key] - r['pose_speed']) for r in mismatch)
        print(f"  {label:12s}:  med={errs[n//2]:.3f}  P90={errs[int(n*0.9)]:.3f}  "
              f"P95={errs[int(n*0.95)]:.3f}  max={errs[-1]:.3f}")
else:
    print("  No mismatch frames found")


# ══════════════════════════════════════════════════════════════
# 6. CONVERGENCE ON STUCK EVENTS
# ══════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("6. CONVERGENCE: frames to <50% error after stuck onset")
print("=" * 70)

# Group mismatch into events
mismatch_times = [r['t'] for r in mismatch]
m_events = []
if mismatch:
    cur_ev = [mismatch[0]]
    for r in mismatch[1:]:
        if r['t'] - cur_ev[-1]['t'] < 0.2:
            cur_ev.append(r)
        else:
            if len(cur_ev) >= 2:
                m_events.append(cur_ev)
            cur_ev = [r]
    if len(cur_ev) >= 2:
        m_events.append(cur_ev)

valid_times = [r['t'] for r in valid]

for ev in m_events[:10]:
    t_start = ev[0]['t']
    initial_wheel = ev[0]['wheel_mag']
    if initial_wheel < 0.2: continue

    idx = bisect.bisect_left(valid_times, t_start)
    if idx >= len(valid) or abs(valid[idx]['t'] - t_start) > 0.05: continue

    old_conv, v1_conv, v2_conv = None, None, None
    for j in range(idx, min(idx + 30, len(valid))):
        ps = valid[j]['pose_speed']
        if old_conv is None and abs(valid[j]['old_eff'] - ps) < initial_wheel * 0.5:
            old_conv = j - idx
        if v1_conv is None and abs(valid[j]['v1_eff'] - ps) < initial_wheel * 0.5:
            v1_conv = j - idx
        if v2_conv is None and abs(valid[j]['v2_eff'] - ps) < initial_wheel * 0.5:
            v2_conv = j - idx

    d = ev[0].get('dist')
    d_str = f"{d:.1f}" if d else "?"
    print(f"  t={t_start:.3f} (wheel={initial_wheel:.2f}, dist={d_str}m): "
          f"Old={old_conv if old_conv is not None else '>30'}  "
          f"V1={v1_conv if v1_conv is not None else '>30'}  "
          f"V2={v2_conv if v2_conv is not None else '>30'} frames")


# ══════════════════════════════════════════════════════════════
# 7. DETAILED STUCK EVENT EXAMPLES
# ══════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("7. DETAILED STUCK EVENTS (largest events)")
print("=" * 70)

for ev in sorted(m_events, key=lambda e: len(e), reverse=True)[:3]:
    t_start = ev[0]['t']
    t_end = ev[-1]['t']
    d = ev[0].get('dist')
    d_str = f"{d:.1f}" if d else "?"
    print(f"\n--- Event t={t_start:.3f}-{t_end:.3f} ({(t_end-t_start)*1000:.0f}ms, "
          f"{len(ev)} frames, dist={d_str}m) ---")

    idx = bisect.bisect_left(valid_times, t_start) - 3
    idx = max(0, idx)
    end_idx = bisect.bisect_right(valid_times, t_end) + 5
    end_idx = min(len(valid), end_idx)

    print(f"  {'t':>8}  {'Wheel':>6}  {'Pose':>6}  {'Old':>6}  {'V1':>6}  {'V2':>6}  "
          f"{'v2Fast':>6}  {'v2Slip':>6}  {'OldE':>5}  {'V1E':>5}  {'V2E':>5}")
    stuck_times = {r['t'] for r in ev}
    for j in range(idx, end_idx):
        r = valid[j]
        is_stuck = r['t'] in stuck_times
        oe = abs(r['old_eff'] - r['pose_speed'])
        v1e = abs(r['v1_eff'] - r['pose_speed'])
        v2e = abs(r['v2_eff'] - r['pose_speed'])
        fast = "FAST" if r['v2_fast'] else "slow"
        mark = " <" if is_stuck else ""
        print(f"  {r['t']:8.3f}  {r['wheel_mag']:6.3f}  {r['pose_speed']:6.3f}  "
              f"{r['old_eff']:6.3f}  {r['v1_eff']:6.3f}  {r['v2_eff']:6.3f}  "
              f"{fast:>6}  {r['v2_slip_mag']:6.3f}  {oe:5.2f}  {v1e:5.2f}  {v2e:5.2f}{mark}")


# ══════════════════════════════════════════════════════════════
# 8. STALE SLIP: V2 slip magnitude when robot is stationary
# ══════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("8. STALE SLIP: V2 filtered slip when wheels < 0.05 (should be ~0)")
print("=" * 70)

stale = [r for r in valid if r['wheel_mag'] < 0.05 and r['v2_slip_mag'] > 0.01]
print(f"  Frames with wheels<0.05 and slip>0.01: {len(stale)}")
if stale:
    slip_vals = sorted(r['v2_slip_mag'] for r in stale)
    n = len(slip_vals)
    for pct in [50, 90, 95, 100]:
        idx = min(int(n * pct / 100), n - 1)
        print(f"    P{pct}: {slip_vals[idx]:.4f} m/s")

    # Worst cases
    worst = sorted(stale, key=lambda r: r['v2_slip_mag'], reverse=True)[:10]
    print(f"\n  Top 10 stale slip frames:")
    print(f"  {'t':>8}  {'Wheel':>6}  {'v2Slip':>7}  {'v2Eff':>6}  {'v2Fast':>6}")
    for r in worst:
        fast = "FAST" if r['v2_fast'] else "slow"
        print(f"  {r['t']:8.3f}  {r['wheel_mag']:6.3f}  {r['v2_slip_mag']:7.4f}  "
              f"{r['v2_eff']:6.3f}  {fast:>6}")


# ══════════════════════════════════════════════════════════════
# 9. V2 FAST MODE ACTIVATION PROFILE
# ══════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("9. V2 FAST MODE ACTIVATION")
print("=" * 70)

fast_frames = [r for r in valid if r['v2_fast']]
print(f"  Fast mode active: {len(fast_frames)}/{len(valid)} frames "
      f"({100*len(fast_frames)/len(valid):.1f}%)")

# Group fast mode into episodes
if fast_frames:
    episodes = []
    cur = [fast_frames[0]]
    for r in fast_frames[1:]:
        if r['t'] - cur[-1]['t'] < 0.1:
            cur.append(r)
        else:
            episodes.append(cur)
            cur = [r]
    episodes.append(cur)
    print(f"  Fast mode episodes: {len(episodes)}")
    print(f"  Episode durations (ms): ", end="")
    durs = [(ep[-1]['t'] - ep[0]['t'])*1000 for ep in episodes]
    durs.sort(reverse=True)
    print(", ".join(f"{d:.0f}" for d in durs[:20]))

    # How many fast episodes are real stuck vs vision noise?
    real_stuck = 0
    vision_noise = 0
    for ep in episodes:
        avg_pose = sum(r['pose_speed'] for r in ep) / len(ep)
        avg_wheel = sum(r['wheel_mag'] for r in ep) / len(ep)
        if avg_wheel > 0.3 and avg_pose < avg_wheel * 0.5:
            real_stuck += 1
        else:
            vision_noise += 1
    print(f"  Real stuck events (wheel>>pose): {real_stuck}")
    print(f"  Vision noise triggers: {vision_noise}")


# ══════════════════════════════════════════════════════════════
# 10. REMAINING V2 SPIKES (new artifacts)
# ══════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("10. TOP 15 V2 NOISE SPIKES (wheels < 0.15, v2_eff > 0.2)")
print("=" * 70)

v2_spikes = [(r['t'], r['v2_eff'], r['old_eff'], r['wheel_mag'], r['v2_slip_mag'],
              r['v2_fast'], r['dist'])
             for r in valid if r['wheel_mag'] < 0.15 and r['v2_eff'] > 0.2]
v2_spikes.sort(key=lambda x: x[1], reverse=True)
print(f"  {'t':>8}  {'V2eff':>7}  {'OldEff':>7}  {'Wheel':>6}  {'Slip':>6}  {'Fast':>5}  {'Dist':>5}")
for s in v2_spikes[:15]:
    fast = "FAST" if s[5] else "slow"
    d_str = f"{s[6]:.1f}" if s[6] else "?"
    print(f"  {s[0]:8.3f}  {s[1]:7.3f}  {s[2]:7.3f}  {s[3]:6.3f}  {s[4]:6.3f}  {fast:>5}  {d_str:>5}")


print("\n=== ANALYSIS COMPLETE ===")
