"""Compare turret-to-hub distance distributions during shooting intervals.

A shooting interval is defined as a continuous span where
NT:/launcher/velocity >= 50 RPS (entered when >= 50, exited when < 30).

We sample NT:/SmartDashboard/LiveLauncherData/Turret to hub distance
ONLY during those intervals.

Caveat (per user): when the robot is in the neutral zone or opposing
alliance, the active aim target is NOT the hub, so this signal does not
represent the actual launch distance. We approximate "near our alliance"
by also gating on robot pose if available -- but as a simpler proxy we
also report the distribution restricted to distances <= 6.0 m which is a
plausible launch-from-our-side range.
"""

from __future__ import annotations

import statistics
import sys
from pathlib import Path

from logreader.wpilog_reader import read_wpilog


DIST = "NT:/SmartDashboard/LiveLauncherData/Turret to hub distance"
FLY = "NT:/launcher/velocity"
FEEDER_VEL = "NT:/SmartDashboard/FeederSubsystem/VelocityRPS"


def pct(vs, qs=(0.05, 0.10, 0.25, 0.50, 0.75, 0.90, 0.95)):
    if not vs:
        return ""
    s = sorted(vs)
    out = []
    for q in qs:
        if len(s) == 1:
            out.append(s[0]); continue
        pos = q * (len(s) - 1)
        lo = int(pos); hi = min(lo + 1, len(s) - 1)
        out.append(s[lo] * (1 - (pos - lo)) + s[hi] * (pos - lo))
    return "  ".join(f"p{int(q*100)}={v:.2f}" for q, v in zip(qs, out))


def shooting_intervals(fly_pairs, enter=50.0, exit_below=30.0):
    intervals = []
    in_int = False; start = 0
    for t, v in fly_pairs:
        if v >= enter and not in_int:
            start = t; in_int = True
        elif v < exit_below and in_int:
            intervals.append((start, t)); in_int = False
    if in_int and fly_pairs:
        intervals.append((start, fly_pairs[-1][0]))
    return intervals


def value_in_intervals(sig, intervals):
    if sig is None:
        return []
    out = []
    j = 0
    intervals = sorted(intervals)
    for tv in sig.values:
        if not isinstance(tv.value, (int, float)):
            continue
        t = tv.timestamp_us
        # advance j
        while j < len(intervals) and intervals[j][1] < t:
            j += 1
        if j >= len(intervals):
            break
        a, b = intervals[j]
        if a <= t <= b:
            out.append(float(tv.value))
    return out


def main(paths):
    for p in paths:
        data = read_wpilog(p)
        fly = data.signals.get(FLY)
        if fly is None:
            print(f"\n=== {Path(p).name}: no flywheel signal"); continue
        fly_pairs = [(tv.timestamp_us, tv.value) for tv in fly.values
                     if isinstance(tv.value, (int, float))]
        intervals = shooting_intervals(fly_pairs)
        total_s = sum((b - a) for a, b in intervals) / 1e6

        dists = value_in_intervals(data.signals.get(DIST), intervals)
        feeders = value_in_intervals(data.signals.get(FEEDER_VEL), intervals)

        print(f"\n=== {Path(p).name} ===")
        print(f"  shooting intervals: n={len(intervals)} total={total_s:.1f}s")
        if dists:
            print(f"  distance ALL    (n={len(dists)}): {pct(dists)}")
            print(f"    mean={statistics.mean(dists):.2f}  min={min(dists):.2f}  max={max(dists):.2f}")
            close = [d for d in dists if 1.0 <= d <= 6.0]
            print(f"  distance 1-6m   (n={len(close)}): {pct(close)}")
            if close:
                print(f"    mean={statistics.mean(close):.2f}")
            far = [d for d in dists if d > 6.0]
            print(f"  distance >6m    (n={len(far)})  share={len(far)/len(dists)*100:.1f}%")
        if feeders:
            print(f"  feeder vel ALL  (n={len(feeders)}): {pct(feeders)}")
            print(f"    mean={statistics.mean(feeders):.2f}")


if __name__ == "__main__":
    main(sys.argv[1:])
