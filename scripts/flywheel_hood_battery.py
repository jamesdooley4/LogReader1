"""Compare hood goal/position and battery voltage between an early and late log."""
from __future__ import annotations
import sys
import statistics
from pathlib import Path
from logreader.wpilog_reader import read_wpilog


def pct(vs, qs=(0.05, 0.25, 0.50, 0.75, 0.95)):
    if not vs: return ""
    s = sorted(vs)
    out = []
    for q in qs:
        if len(s) == 1: out.append(s[0]); continue
        pos = q * (len(s) - 1); lo = int(pos); hi = min(lo+1, len(s)-1)
        out.append(s[lo]*(1-(pos-lo)) + s[hi]*(pos-lo))
    return "  ".join(f"p{int(q*100)}={v:.3f}" for q, v in zip(qs, out))


def main(path: str):
    data = read_wpilog(path)
    print(f"\n=== {Path(path).name} ===")
    for name in ("NT:/hood/goal", "NT:/hood/position",
                 "NT:/LiveWindow/Ungrouped/PowerDistribution[0]/Voltage",
                 "NT:/launcher/velocity"):
        sig = data.signals.get(name)
        if sig is None: continue
        vs = [tv.value for tv in sig.values if isinstance(tv.value, (int, float))]
        if not vs: continue
        print(f"  {name} (n={len(vs)})")
        print(f"    {pct(vs)}")
        # while flywheel >= 60 (i.e., during shooting attempts) for hood/voltage
    # Now: filter hood/voltage to moments when flywheel velocity >= 60
    fly = data.signals.get("NT:/launcher/velocity")
    if fly is None: return
    fly_pairs = [(tv.timestamp_us, tv.value) for tv in fly.values if isinstance(tv.value, (int, float))]
    if not fly_pairs: return
    # Step through fly samples; collect time intervals where vel >= 50
    intervals: list[tuple[int, int]] = []
    in_int = False
    start = 0
    for t, v in fly_pairs:
        if v >= 50 and not in_int:
            start = t; in_int = True
        elif v < 30 and in_int:
            intervals.append((start, t)); in_int = False
    if in_int:
        intervals.append((start, fly_pairs[-1][0]))

    if not intervals:
        print("  (no shooting intervals)")
        return
    total_s = sum((b - a) for a, b in intervals) / 1e6
    print(f"  Shooting intervals: n={len(intervals)} total={total_s:.1f}s")

    for sname in ("NT:/hood/position", "NT:/hood/goal",
                  "NT:/LiveWindow/Ungrouped/PowerDistribution[0]/Voltage"):
        sig = data.signals.get(sname)
        if sig is None: continue
        vals = []
        for tv in sig.values:
            if not isinstance(tv.value, (int, float)): continue
            for a, b in intervals:
                if a <= tv.timestamp_us <= b:
                    vals.append(float(tv.value)); break
        if vals:
            print(f"  during shooting: {sname} (n={len(vals)})")
            print(f"    {pct(vals)}")
            print(f"    mean={statistics.mean(vals):.3f}  min={min(vals):.3f}  max={max(vals):.3f}")


if __name__ == "__main__":
    for p in sys.argv[1:]: main(p)
