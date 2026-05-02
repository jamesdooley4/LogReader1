"""Detailed velocity-distribution probe for a single log."""

from __future__ import annotations

import sys
from pathlib import Path
import statistics

from logreader.wpilog_reader import read_wpilog


SIGS = [
    "NT:/launcher/velocity",
    "NT:/launcher/current",
    "NT:/launcher/flywheelTuner",
    "NT:/SmartDashboard/FeederSubsystem/VelocityRPS",
    "NT:SmartDashboard/FeederSubsystem/TargetVelocityRPS",
    "/FeederLogs/statorCurrent",
    "/FeederLogs/supplyCurrent",
]


def percentiles(vs: list[float], qs=(0.0, 0.10, 0.25, 0.50, 0.75, 0.90, 0.99, 1.0)):
    if not vs:
        return ""
    s = sorted(vs)
    out = []
    for q in qs:
        if len(s) == 1:
            out.append(s[0])
            continue
        pos = q * (len(s) - 1)
        lo = int(pos); hi = min(lo + 1, len(s) - 1)
        out.append(s[lo] * (1 - (pos - lo)) + s[hi] * (pos - lo))
    return "  ".join(f"{q*100:>4.0f}%={v:.2f}" for q, v in zip(qs, out))


def main(path: str) -> None:
    data = read_wpilog(path)
    print(f"File: {path}")
    print(f"Duration: ", end="")
    if data.signals:
        all_t = []
        for s in data.signals.values():
            if s.values:
                all_t.append(s.values[0].timestamp_us)
                all_t.append(s.values[-1].timestamp_us)
        if all_t:
            print(f"{(max(all_t) - min(all_t)) / 1e6:.1f} s")
    for name in SIGS:
        sig = data.signals.get(name)
        if sig is None:
            print(f"  {name}: NOT PRESENT")
            continue
        vs = [tv.value for tv in sig.values if isinstance(tv.value, (int, float))]
        if not vs:
            print(f"  {name}: 0 records")
            continue
        print(f"  {name} (n={len(vs)}):")
        print(f"    {percentiles([float(v) for v in vs])}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        main(p)
        print()
