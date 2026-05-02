"""Probe for battery / voltage / hood / setpoint signals."""

from __future__ import annotations

import sys

from logreader.wpilog_reader import read_wpilog


def main(path: str) -> None:
    data = read_wpilog(path)
    print(f"\n=== {path} ===")
    needles = ("battery", "Battery", "voltage", "Voltage", "BusV", "BusVoltage",
               "hood", "Hood", "TargetVelocity", "setpoint", "Setpoint",
               "PDH", "PDP", "ResultantBattery", "InputVoltage")
    seen = set()
    for name in sorted(data.signals.keys()):
        if any(n in name for n in needles):
            sig = data.signals[name]
            if len(sig.values) == 0:
                continue
            key = name
            if key in seen: continue
            seen.add(key)
            sample_values = [tv.value for tv in sig.values[:3] if isinstance(tv.value, (int, float, str, bool))]
            print(f"  {name}  type={sig.info.type.value}  records={len(sig.values)}  sample={sample_values}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        main(p)
