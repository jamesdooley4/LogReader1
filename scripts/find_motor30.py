"""Find motor-30 (feeder) signals in a Phoenix6/Canivore hoot-converted wpilog."""

from __future__ import annotations

import sys
from logreader.wpilog_reader import read_wpilog


def main(path: str) -> None:
    data = read_wpilog(path)
    print(f"File: {path}  signals={len(data.signals)}")
    needles = ("Talon FX 30", "TalonFX 30", "TalonFX-30", "Talon FX-30",
               "/30/", "_30_", "TalonFX30", " 30 ", "id30")
    for name in sorted(data.signals.keys()):
        if any(n in name for n in needles):
            sig = data.signals[name]
            if len(sig.values) > 0:
                print(f"  {name}  type={sig.info.type.value}  records={len(sig.values)}")


if __name__ == "__main__":
    main(sys.argv[1])
