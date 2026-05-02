"""Probe a single log file for the launcher/feeder signal names."""

from __future__ import annotations

import sys
from pathlib import Path

from logreader.wpilog_reader import read_wpilog


def main(path: str) -> None:
    data = read_wpilog(path)
    print(f"File: {path}")
    print(f"Signals: {len(data.signals)}")
    needles = (
        "launcher",
        "Launcher",
        "FeederSubsystem",
        "FeederLogs",
        "Feeder",
        "feeder",
    )
    for name in sorted(data.signals.keys()):
        if any(n in name for n in needles):
            sig = data.signals[name]
            print(f"  {name!r}  type={sig.info.type.value}  records={len(sig.values)}")


if __name__ == "__main__":
    main(sys.argv[1])
