"""Run v2-style per-shot analysis on a single log."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from flywheel_decline_v2 import detect_and_window, per_log_stats  # type: ignore


def _fmt(v, prec=2):
    if v is None: return "-"
    if isinstance(v, int): return str(v)
    return f"{v:.{prec}f}"


def main(paths: list[str]) -> None:
    for p in paths:
        path = Path(p)
        shots = detect_and_window(path)
        s = per_log_stats(shots)
        print(f"\n=== {path.name} ===  shots={s.get('n', 0)}")
        for k in ("fly_pre_med", "fly_pre_p90", "fly_pre_max",
                  "fly_dip_med", "fly_recover_med",
                  "feeder_pre_med", "feeder_pre_p90", "feeder_pre_max",
                  "feeder_drop_med",
                  "launcher_cur_peak_med",
                  "feeder_stator_peak_med", "feeder_supply_peak_med"):
            v = s.get(k)
            print(f"  {k:<25} {_fmt(v, 3 if 'recover' in k else 2)}")


if __name__ == "__main__":
    main(sys.argv[1:])
