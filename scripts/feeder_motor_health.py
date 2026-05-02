"""Compare feeder TalonFX motor health for the post-roller-replacement match.

Defines "feeder commanded high" intervals as PIDVelocity_Reference >= 80 RPS
(which is when the feeder is being told to spin up to feed a ball through).
For each such interval, computes:
- velocity tracking error (Reference - Velocity), median + p90 magnitude
- stator current peak + median
- supply voltage during command
- motor voltage applied
- closed-loop error percentile

Plus overall maximum velocity achieved while commanded high.
"""

from __future__ import annotations

import statistics
import sys
from bisect import bisect_left, bisect_right
from pathlib import Path

from logreader.wpilog_reader import read_wpilog


def _series(sig):
    if sig is None: return [], []
    ts, vs = [], []
    for tv in sig.values:
        if isinstance(tv.value, (int, float)):
            ts.append(tv.timestamp_us); vs.append(float(tv.value))
    return ts, vs


def _slice(ts, vs, t0, t1):
    return vs[bisect_left(ts, t0):bisect_right(ts, t1)]


def _at(ts, vs, t):
    if not ts: return None
    i = bisect_right(ts, t) - 1
    return vs[i] if i >= 0 else None


def pct(vs, qs=(0.05, 0.10, 0.50, 0.90, 0.95, 1.0)):
    if not vs: return ""
    s = sorted(vs)
    out = []
    for q in qs:
        if len(s) == 1: out.append(s[0]); continue
        pos = q * (len(s) - 1); lo = int(pos); hi = min(lo+1, len(s)-1)
        out.append(s[lo]*(1-(pos-lo)) + s[hi]*(pos-lo))
    return "  ".join(f"p{int(q*100)}={v:.2f}" for q, v in zip(qs, out))


def main(path: str) -> None:
    data = read_wpilog(path)
    base = "Phoenix6/TalonFX-30"
    ref_ts, ref_vs = _series(data.signals.get(f"{base}/PIDVelocity_Reference"))
    vel_ts, vel_vs = _series(data.signals.get(f"{base}/Velocity"))
    err_ts, err_vs = _series(data.signals.get(f"{base}/PIDVelocity_ClosedLoopError"))
    stator_ts, stator_vs = _series(data.signals.get(f"{base}/StatorCurrent"))
    supply_ts, supply_vs = _series(data.signals.get(f"{base}/SupplyCurrent"))
    mvolt_ts, mvolt_vs = _series(data.signals.get(f"{base}/MotorVoltage"))
    svolt_ts, svolt_vs = _series(data.signals.get(f"{base}/SupplyVoltage"))
    temp_ts, temp_vs = _series(data.signals.get(f"{base}/DeviceTemp"))

    # Find intervals where reference >= 80 (feeding mode)
    intervals = []
    in_int = False; start = 0
    for t, v in zip(ref_ts, ref_vs):
        if v >= 80 and not in_int:
            start = t; in_int = True
        elif v < 30 and in_int:
            intervals.append((start, t)); in_int = False
    if in_int and ref_ts:
        intervals.append((start, ref_ts[-1]))

    total_s = sum((b-a) for a,b in intervals) / 1e6
    print(f"\n=== {Path(path).name} | TalonFX-30 (feeder) ===")
    print(f"  feeding intervals (ref>=80): n={len(intervals)} total={total_s:.1f}s")

    # Aggregate velocity / error / stator / supply during these intervals
    vels, refs, errs, stators, supplies, mvolts, svolts = [], [], [], [], [], [], []
    for a, b in intervals:
        vels.extend(_slice(vel_ts, vel_vs, a, b))
        refs.extend(_slice(ref_ts, ref_vs, a, b))
        errs.extend(_slice(err_ts, err_vs, a, b))
        stators.extend(_slice(stator_ts, stator_vs, a, b))
        supplies.extend(_slice(supply_ts, supply_vs, a, b))
        mvolts.extend(_slice(mvolt_ts, mvolt_vs, a, b))
        svolts.extend(_slice(svolt_ts, svolt_vs, a, b))

    if vels:
        print(f"  Velocity during ref>=80 (n={len(vels)}): {pct(vels)}")
        print(f"    mean={statistics.mean(vels):.2f}  max={max(vels):.2f}")
    if refs:
        print(f"  Reference during ref>=80 (n={len(refs)}): {pct(refs)}")
    if errs:
        # error sign convention: Reference - Velocity (positive = under target)
        print(f"  ClosedLoopError (n={len(errs)}): {pct(errs)}")
        # absolute magnitude
        abs_errs = [abs(e) for e in errs]
        print(f"  |error| (n={len(abs_errs)}): {pct(abs_errs)}")
        print(f"    mean={statistics.mean(abs_errs):.2f}")
    if stators:
        print(f"  StatorCurrent (n={len(stators)}): {pct(stators)}")
        print(f"    mean={statistics.mean(stators):.2f}")
    if supplies:
        print(f"  SupplyCurrent (n={len(supplies)}): {pct(supplies)}")
    if mvolts:
        print(f"  MotorVoltage (n={len(mvolts)}): {pct(mvolts)}")
    if svolts:
        print(f"  SupplyVoltage (n={len(svolts)}): {pct(svolts)}")
    if temp_vs:
        print(f"  DeviceTemp overall: min={min(temp_vs):.1f} median={statistics.median(temp_vs):.1f} max={max(temp_vs):.1f}")

    # Per-interval peak velocity achieved (best response per "shot")
    print("\n  Per-interval peak velocity achieved:")
    print(f"  {'idx':>3} {'dur_s':>6} {'ref_max':>7} {'vel_max':>7} {'err_min':>7} "
          f"{'stator_pk':>9} {'volt_pk':>7} {'svolt_min':>9}")
    peak_vels = []
    for i, (a, b) in enumerate(intervals):
        ref_max = max(_slice(ref_ts, ref_vs, a, b) or [0])
        vel_seg = _slice(vel_ts, vel_vs, a, b)
        vel_max = max(vel_seg) if vel_seg else None
        if vel_max is not None:
            peak_vels.append(vel_max)
        err_seg = _slice(err_ts, err_vs, a, b)
        err_min = min(err_seg) if err_seg else None
        stator_seg = _slice(stator_ts, stator_vs, a, b)
        stator_pk = max(stator_seg) if stator_seg else None
        mvolt_seg = _slice(mvolt_ts, mvolt_vs, a, b)
        volt_pk = max((abs(v) for v in mvolt_seg), default=None)
        svolt_seg = _slice(svolt_ts, svolt_vs, a, b)
        svolt_min = min(svolt_seg) if svolt_seg else None
        def f(x, p=2):
            return f"{x:.{p}f}" if x is not None else "-"
        print(f"  {i:>3} {(b-a)/1e6:>6.2f} {f(ref_max)} {f(vel_max)} {f(err_min)} "
              f"{f(stator_pk)} {f(volt_pk)} {f(svolt_min)}")

    if peak_vels:
        print(f"\n  Peak velocity per interval: median={statistics.median(peak_vels):.2f}  "
              f"min={min(peak_vels):.2f}  max={max(peak_vels):.2f}")


if __name__ == "__main__":
    main(sys.argv[1])
