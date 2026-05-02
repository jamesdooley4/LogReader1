"""Analyze whether close-range pose estimation uncertainty drives turret jitter.

Theory: each vision estimator correction shifts the bearing to target, and at
close range the angular sensitivity (d_bearing/d_pose) is much larger, so the
turret target oscillates and the motor draws abrupt current spikes to track it.

Signals used:
  NT:/SmartDashboard//Turret/Target   – turret target angle (degrees)
  NT:/SmartDashboard//Turret/Position – actual turret position (degrees)
  NT:/SmartDashboard//Turret/Current  – motor stator current (amps)
  NT:/SmartDashboard/LiveLauncherData/Turret to hub distance – range (m)
  NT:/DriveState/Pose                 – estimator pose (struct:Pose2d)
  NT:/SmartDashboard//vision/*        – vision accept/reject + error
"""

from __future__ import annotations
import sys, math, struct as pystruct, statistics
from pathlib import Path
from collections import defaultdict

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from logreader.wpilog_reader import read_wpilog

LOG_PATH = r"C:\Users\jdool_46clpzz\Documents\FRC_Sessions\.staging\logs\FRC_20260425_202750.wpilog"
print(f"Loading {LOG_PATH} ...")
log = read_wpilog(LOG_PATH)
print(f"Loaded {len(log.signals)} signals\n")


# ── helpers ──
def ts_val(sig):
    """Return list of (time_s, value)."""
    return [(v.timestamp_us / 1e6, v.value) for v in sig.values]

def decode_pose2d(raw):
    if len(raw) != 24:
        return (float("nan"), float("nan"), float("nan"))
    return pystruct.unpack("<ddd", raw)

def interp_nearest(series, t, max_gap=0.1):
    """Nearest-neighbor lookup in a sorted (t, val) list."""
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    best = lo
    if best > 0 and abs(series[best - 1][0] - t) < abs(series[best][0] - t):
        best = best - 1
    if abs(series[best][0] - t) > max_gap:
        return None
    return series[best][1]


# ── load signals ──
turret_target = ts_val(log.signals["NT:/SmartDashboard//Turret/Target"])
turret_pos    = ts_val(log.signals["NT:/SmartDashboard//Turret/Position"])
turret_cur    = ts_val(log.signals["NT:/SmartDashboard//Turret/Current"])
dist_hub      = ts_val(log.signals["NT:/SmartDashboard/LiveLauncherData/Turret to hub distance"])

print(f"Turret target : {len(turret_target)} pts")
print(f"Turret position: {len(turret_pos)} pts")
print(f"Turret current : {len(turret_cur)} pts")
print(f"Distance to hub: {len(dist_hub)} pts")

# Pose data (for detecting vision corrections)
pose_sig = log.signals["NT:/DriveState/Pose"]
poses = []
for v in pose_sig.values:
    x, y, th = decode_pose2d(v.value)
    poses.append((v.timestamp_us / 1e6, x, y, th))

# Vision accept timestamps (rejectReason == "none")
vision_accept_times = []
for cam in ["limelight-b", "limelight-c"]:
    sig = log.signals.get(f"NT:/SmartDashboard//vision/{cam}_rejectReason")
    if sig:
        for v in sig.values:
            if v.value == "none":
                vision_accept_times.append(v.timestamp_us / 1e6)
vision_accept_times.sort()
print(f"Vision accepted measurements: {len(vision_accept_times)}")


# ═══════════════════════════════════════════════════════════════════
# 1. TURRET CURRENT vs DISTANCE — the core test
# ═══════════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("1. TURRET CURRENT (|amps|) vs DISTANCE TO HUB")
print("=" * 70)

# For each current sample, find nearest distance
cur_dist_pairs = []
for t, cur in turret_cur:
    d = interp_nearest(dist_hub, t)
    if d is not None and d > 0.5:
        cur_dist_pairs.append((d, abs(cur)))

# Bin by 1-meter distance brackets
bins = defaultdict(list)
for d, c in cur_dist_pairs:
    bins[round(d)].append(c)

print(f"{'Dist(m)':>8s}  {'N':>6s}  {'MedCur':>8s}  {'P90Cur':>8s}  {'P95Cur':>8s}  {'MaxCur':>8s}  {'StdCur':>8s}")
for b in sorted(bins.keys()):
    vals = sorted(bins[b])
    n = len(vals)
    if n < 10:
        continue
    med = vals[n // 2]
    p90 = vals[int(n * 0.90)]
    p95 = vals[int(n * 0.95)]
    mx = vals[-1]
    std = statistics.stdev(vals) if n > 1 else 0
    print(f"  {b:6.0f}  {n:6d}  {med:8.2f}  {p90:8.2f}  {p95:8.2f}  {mx:8.2f}  {std:8.3f}")


# ═══════════════════════════════════════════════════════════════════
# 2. TURRET TARGET JITTER (frame-to-frame delta) vs DISTANCE
# ═══════════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("2. TURRET TARGET JITTER (deg/frame) vs DISTANCE")
print("=" * 70)

target_jitter = []
for i in range(1, len(turret_target)):
    dt = turret_target[i][0] - turret_target[i - 1][0]
    if 0.001 < dt < 0.1:
        delta = abs(turret_target[i][1] - turret_target[i - 1][1])
        d = interp_nearest(dist_hub, turret_target[i][0])
        if d is not None and d > 0.5:
            target_jitter.append((d, delta, turret_target[i][0]))

bins_tj = defaultdict(list)
for d, delta, _ in target_jitter:
    bins_tj[round(d)].append(delta)

print(f"{'Dist(m)':>8s}  {'N':>6s}  {'MedD':>8s}  {'P90D':>8s}  {'P95D':>8s}  {'P99D':>8s}  {'MaxD':>8s}")
for b in sorted(bins_tj.keys()):
    vals = sorted(bins_tj[b])
    n = len(vals)
    if n < 10:
        continue
    print(f"  {b:6.0f}  {n:6d}  {vals[n//2]:8.4f}  {vals[int(n*0.90)]:8.4f}  "
          f"{vals[int(n*0.95)]:8.4f}  {vals[int(n*0.99)]:8.4f}  {vals[-1]:8.4f}")


# ═══════════════════════════════════════════════════════════════════
# 3. TURRET TRACKING ERROR vs DISTANCE
# ═══════════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("3. TURRET TRACKING ERROR (|target - position|) vs DISTANCE")
print("=" * 70)

tracking_err = []
for t_t, tgt in turret_target:
    pos = interp_nearest(turret_pos, t_t, max_gap=0.05)
    d = interp_nearest(dist_hub, t_t)
    if pos is not None and d is not None and d > 0.5:
        tracking_err.append((d, abs(tgt - pos), t_t))

bins_te = defaultdict(list)
for d, err, _ in tracking_err:
    bins_te[round(d)].append(err)

print(f"{'Dist(m)':>8s}  {'N':>6s}  {'MedErr':>8s}  {'P90Err':>8s}  {'P95Err':>8s}  {'MaxErr':>8s}")
for b in sorted(bins_te.keys()):
    vals = sorted(bins_te[b])
    n = len(vals)
    if n < 10:
        continue
    print(f"  {b:6.0f}  {n:6d}  {vals[n//2]:8.3f}  {vals[int(n*0.90)]:8.3f}  "
          f"{vals[int(n*0.95)]:8.3f}  {vals[-1]:8.3f}")


# ═══════════════════════════════════════════════════════════════════
# 4. CURRENT SPIKES COINCIDING WITH VISION ACCEPTS
# ═══════════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("4. TURRET CURRENT AROUND VISION ACCEPTANCE EVENTS")
print("=" * 70)
# For each vision accept, look at turret current in a ±100ms window
# Compare to baseline current (random times)

WINDOW = 0.100  # seconds
cur_near_vision = []
cur_baseline = []

# Build fast current lookup: sorted list
turret_cur_sorted = turret_cur  # already time-sorted

import bisect
cur_times = [c[0] for c in turret_cur]
cur_vals  = [abs(c[1]) for c in turret_cur]

for vt in vision_accept_times:
    lo = bisect.bisect_left(cur_times, vt - WINDOW)
    hi = bisect.bisect_right(cur_times, vt + WINDOW)
    for j in range(lo, hi):
        cur_near_vision.append(cur_vals[j])

# Baseline: samples NOT within ±200ms of any vision accept
vision_set_times = set()
for vt in vision_accept_times:
    lo = bisect.bisect_left(cur_times, vt - 0.200)
    hi = bisect.bisect_right(cur_times, vt + 0.200)
    for j in range(lo, hi):
        vision_set_times.add(j)

for j in range(len(cur_vals)):
    if j not in vision_set_times:
        cur_baseline.append(cur_vals[j])

if cur_near_vision and cur_baseline:
    nv = sorted(cur_near_vision)
    bl = sorted(cur_baseline)
    print(f"Current within ±{WINDOW*1000:.0f}ms of vision accept ({len(nv)} samples):")
    print(f"  Median: {nv[len(nv)//2]:.2f}A   P90: {nv[int(len(nv)*0.90)]:.2f}A   "
          f"P95: {nv[int(len(nv)*0.95)]:.2f}A   Max: {nv[-1]:.2f}A")
    print(f"Baseline current (no vision within ±200ms, {len(bl)} samples):")
    print(f"  Median: {bl[len(bl)//2]:.2f}A   P90: {bl[int(len(bl)*0.90)]:.2f}A   "
          f"P95: {bl[int(len(bl)*0.95)]:.2f}A   Max: {bl[-1]:.2f}A")


# ═══════════════════════════════════════════════════════════════════
# 5. CURRENT SPIKES NEAR VISION ACCEPTS — BINNED BY DISTANCE
# ═══════════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("5. CURRENT NEAR VISION ACCEPTS — BY DISTANCE BRACKET")
print("=" * 70)

# For each vision accept, get distance + peak current in ±100ms window
vision_cur_dist = []
for vt in vision_accept_times:
    d = interp_nearest(dist_hub, vt)
    if d is None or d < 0.5:
        continue
    lo = bisect.bisect_left(cur_times, vt - WINDOW)
    hi = bisect.bisect_right(cur_times, vt + WINDOW)
    if lo < hi:
        peak = max(cur_vals[lo:hi])
        vision_cur_dist.append((round(d), peak))

bins_vcd = defaultdict(list)
for d, pk in vision_cur_dist:
    bins_vcd[d].append(pk)

print(f"{'Dist(m)':>8s}  {'Events':>7s}  {'MedPeak':>8s}  {'P90Peak':>9s}  {'P95Peak':>9s}  {'MaxPeak':>9s}")
for b in sorted(bins_vcd.keys()):
    vals = sorted(bins_vcd[b])
    n = len(vals)
    if n < 5:
        continue
    print(f"  {b:6.0f}  {n:7d}  {vals[n//2]:8.2f}  {vals[int(n*0.90)]:9.2f}  "
          f"{vals[int(n*0.95)]:9.2f}  {vals[-1]:9.2f}")


# ═══════════════════════════════════════════════════════════════════
# 6. POSE JUMP SIZE AT VISION ACCEPTS — BY DISTANCE
# ═══════════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("6. POSE JUMP AT VISION ACCEPT vs DISTANCE")
print("=" * 70)
# For each vision accept, measure how much the pose moved vs expected (wheel speed * dt)
# in the enclosing pose frame pair

pose_times = [p[0] for p in poses]

vision_pose_jump = []
for vt in vision_accept_times:
    # Find pose frame just after vision accept
    idx = bisect.bisect_right(pose_times, vt)
    if idx < 1 or idx >= len(poses):
        continue
    p0 = poses[idx - 1]
    p1 = poses[idx]
    dt = p1[0] - p0[0]
    if dt < 0.001 or dt > 0.1:
        continue
    disp = math.hypot(p1[1] - p0[1], p1[2] - p0[2])
    d = interp_nearest(dist_hub, vt)
    if d is None or d < 0.5:
        continue
    # Angular shift caused by this displacement at this distance
    angular_shift_deg = math.degrees(math.atan2(disp, d)) if d > 0 else 0
    vision_pose_jump.append((round(d), disp * 100, angular_shift_deg))  # cm, deg

bins_vpj = defaultdict(list)
bins_vpj_ang = defaultdict(list)
for d, cm, ang in vision_pose_jump:
    bins_vpj[d].append(cm)
    bins_vpj_ang[d].append(ang)

print(f"{'Dist(m)':>8s}  {'Events':>7s}  {'MedJump':>9s}  {'P95Jump':>9s}  {'MaxJump':>9s}  "
      f"{'MedAng':>8s}  {'P95Ang':>8s}  {'MaxAng':>8s}")
for b in sorted(bins_vpj.keys()):
    cm_vals = sorted(bins_vpj[b])
    ang_vals = sorted(bins_vpj_ang[b])
    n = len(cm_vals)
    if n < 5:
        continue
    print(f"  {b:6.0f}  {n:7d}  {cm_vals[n//2]:8.2f}cm  {cm_vals[int(n*0.95)]:8.2f}cm  "
          f"{cm_vals[-1]:8.2f}cm  {ang_vals[n//2]:7.3f}dg  {ang_vals[int(n*0.95)]:7.3f}dg  "
          f"{ang_vals[-1]:7.3f}dg")


# ═══════════════════════════════════════════════════════════════════
# 7. TIME-DOMAIN: show a few windows where current spikes at close range
# ═══════════════════════════════════════════════════════════════════
print("\n" + "=" * 70)
print("7. EXAMPLE WINDOWS: HIGH CURRENT AT CLOSE RANGE (<4m)")
print("=" * 70)

# Find top-20 current peaks at dist < 4m
close_cur_peaks = []
for t, cur in turret_cur:
    d = interp_nearest(dist_hub, t)
    if d is not None and d < 4.0:
        close_cur_peaks.append((t, abs(cur), d))

close_cur_peaks.sort(key=lambda x: x[1], reverse=True)
print(f"Top 20 current peaks at < 4m:")
print(f"  {'Time':>10s}  {'|Cur|(A)':>8s}  {'Dist(m)':>8s}  {'TgtDlt':>8s}  {'TrkErr':>8s}  {'NearVision':>10s}")
for t, cur, d in close_cur_peaks[:20]:
    # Target delta around this time
    tgt_delta = "?"
    for i in range(1, len(turret_target)):
        if abs(turret_target[i][0] - t) < 0.025:
            tgt_delta = f"{abs(turret_target[i][1] - turret_target[i-1][1]):.3f}"
            break
    # Tracking error
    pos = interp_nearest(turret_pos, t, max_gap=0.05)
    tgt = interp_nearest([(tt[0], tt[1]) for tt in turret_target], t, max_gap=0.05)
    trk = f"{abs(tgt - pos):.3f}" if pos is not None and tgt is not None else "?"
    # Nearest vision accept
    nearest_va = min(vision_accept_times, key=lambda vt: abs(vt - t))
    va_offset = nearest_va - t
    va_str = f"{va_offset*1000:+.0f}ms"
    print(f"  {t:10.3f}  {cur:8.2f}  {d:8.2f}  {tgt_delta:>8s}  {trk:>8s}  {va_str:>10s}")


print("\n=== ANALYSIS COMPLETE ===")
