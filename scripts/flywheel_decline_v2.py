"""Sharper comparison of flywheel + feeder behavior across the session.

For each shot detected (rapid drop in NT:/launcher/velocity above 60 RPS)
we capture a window from -300 ms to +300 ms around the dip and record:
  * fly_pre   - median launcher velocity over [-200, -20] ms
  * fly_min   - minimum launcher velocity over [0, +300] ms
  * fly_dip   - fly_pre - fly_min          (energy transferred proxy)
  * fly_recover_s - time after fly_min until velocity returns within 3 RPS
  * feeder_pre - median feeder velocity over [-300, -50] ms (what the ball
                  was being fed at)
  * feeder_min - minimum feeder velocity over [-50, +200] ms (dip when ball
                  contacted flywheel)
  * feeder_drop = feeder_pre - feeder_min
  * launcher_cur_peak - peak supply current of launcher in the window
  * feeder_stator_peak / feeder_supply_peak

Then per log we compute the medians + p10/p90 of every metric, separate the
logs into early (HH < 12) and late (HH >= 12) groups, and finally print the
group medians of the per-log medians (so each log is weighted equally).

Outputs a per-log CSV at out/flywheel_decline_v2.csv plus a console table.
"""

from __future__ import annotations

import csv
import statistics
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, asdict
from pathlib import Path

from logreader.models import LogData, SignalData
from logreader.wpilog_reader import read_wpilog

LOG_DIR = Path(r"C:\Users\jdool_46clpzz\Documents\FRC_Sessions\.staging\logs")
GLOB = "FRC_20260425_*.wpilog"
OUT_CSV = Path(__file__).resolve().parent.parent / "out" / "flywheel_decline_v2.csv"

SHOT_MIN_RPS = 60.0
DIP_THRESHOLD_RPS = 8.0
DIP_WINDOW_S = 0.30
EVENT_GAP_S = 0.5

LAUNCHER_VELOCITY = "NT:/launcher/velocity"
LAUNCHER_CURRENT  = "NT:/launcher/current"
FEEDER_VELOCITY   = "NT:/SmartDashboard/FeederSubsystem/VelocityRPS"
FEEDER_STATOR     = "/FeederLogs/statorCurrent"
FEEDER_SUPPLY     = "/FeederLogs/supplyCurrent"


def _series(sig: SignalData | None) -> tuple[list[int], list[float]]:
    if sig is None:
        return [], []
    ts: list[int] = []
    vs: list[float] = []
    for tv in sig.values:
        v = tv.value
        if isinstance(v, (int, float)):
            ts.append(tv.timestamp_us)
            vs.append(float(v))
    return ts, vs


def _slice(ts: list[int], vs: list[float], t0: int, t1: int) -> list[float]:
    lo = bisect_left(ts, t0)
    hi = bisect_right(ts, t1)
    return vs[lo:hi]


def _value_at(ts: list[int], vs: list[float], t_us: int) -> float | None:
    if not ts:
        return None
    i = bisect_right(ts, t_us) - 1
    return vs[i] if i >= 0 else None


def _median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def _quantile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    pos = q * (len(s) - 1)
    lo = int(pos); hi = min(lo + 1, len(s) - 1)
    return s[lo] * (1 - (pos - lo)) + s[hi] * (pos - lo)


@dataclass
class Shot:
    log: str
    t_us: int
    fly_pre: float
    fly_min: float
    fly_dip: float
    fly_recover_s: float | None
    feeder_pre: float | None
    feeder_min: float | None
    feeder_drop: float | None
    launcher_cur_peak: float | None
    feeder_stator_peak: float | None
    feeder_supply_peak: float | None


def detect_and_window(path: Path) -> list[Shot]:
    data = read_wpilog(path)
    fly_ts, fly_vs = _series(data.signals.get(LAUNCHER_VELOCITY))
    cur_ts, cur_vs = _series(data.signals.get(LAUNCHER_CURRENT))
    fv_ts, fv_vs = _series(data.signals.get(FEEDER_VELOCITY))
    fs_ts, fs_vs = _series(data.signals.get(FEEDER_STATOR))
    fp_ts, fp_vs = _series(data.signals.get(FEEDER_SUPPLY))
    if len(fly_ts) < 5:
        return []

    shots: list[Shot] = []
    last_event_t = -10**18
    n = len(fly_ts)
    i = 0
    while i < n:
        if fly_vs[i] < SHOT_MIN_RPS or fly_ts[i] - last_event_t < EVENT_GAP_S * 1e6:
            i += 1; continue

        # pre-window median (-200, -20 ms)
        pre_lo = fly_ts[i] - 200_000
        pre_hi = fly_ts[i] - 20_000
        pre_vals = _slice(fly_ts, fly_vs, pre_lo, pre_hi)
        pre_vel = statistics.median(pre_vals) if pre_vals else fly_vs[i]
        if pre_vel < SHOT_MIN_RPS:
            i += 1; continue

        # post-window min (0, +DIP_WINDOW_S)
        post_lo = fly_ts[i]
        post_hi = fly_ts[i] + int(DIP_WINDOW_S * 1e6)
        hi_idx = bisect_right(fly_ts, post_hi)
        if hi_idx <= i + 1:
            i += 1; continue
        seg = fly_vs[i:hi_idx]
        min_off = min(range(len(seg)), key=lambda k: seg[k])
        min_idx = i + min_off
        min_vel = seg[min_off]
        if pre_vel - min_vel < DIP_THRESHOLD_RPS:
            i += 1; continue

        # recovery
        target = pre_vel - 3.0
        rec_s: float | None = None
        for k in range(min_idx + 1, n):
            if fly_ts[k] > fly_ts[min_idx] + 1_500_000:
                break
            if fly_vs[k] >= target:
                rec_s = (fly_ts[k] - fly_ts[min_idx]) / 1e6
                break

        # feeder pre-window: median over (-300, -50) ms
        f_pre_vals = _slice(fv_ts, fv_vs, fly_ts[i] - 300_000, fly_ts[i] - 50_000)
        feeder_pre = statistics.median(f_pre_vals) if f_pre_vals else _value_at(fv_ts, fv_vs, fly_ts[i])

        # feeder during shot: min over (-50, +200) ms
        f_dip_vals = _slice(fv_ts, fv_vs, fly_ts[i] - 50_000, fly_ts[min_idx] + 200_000)
        feeder_min = min(f_dip_vals) if f_dip_vals else None

        feeder_drop: float | None = None
        if feeder_pre is not None and feeder_min is not None:
            feeder_drop = feeder_pre - feeder_min

        cur_window = _slice(cur_ts, cur_vs, fly_ts[i] - 50_000, fly_ts[min_idx] + 250_000)
        cur_peak = max(cur_window) if cur_window else None
        fst_window = _slice(fs_ts, fs_vs, fly_ts[i] - 50_000, fly_ts[min_idx] + 250_000)
        fst_peak = max(fst_window) if fst_window else None
        fsp_window = _slice(fp_ts, fp_vs, fly_ts[i] - 50_000, fly_ts[min_idx] + 250_000)
        fsp_peak = max(fsp_window) if fsp_window else None

        shots.append(Shot(
            log=path.name,
            t_us=fly_ts[i],
            fly_pre=pre_vel,
            fly_min=min_vel,
            fly_dip=pre_vel - min_vel,
            fly_recover_s=rec_s,
            feeder_pre=feeder_pre,
            feeder_min=feeder_min,
            feeder_drop=feeder_drop,
            launcher_cur_peak=cur_peak,
            feeder_stator_peak=fst_peak,
            feeder_supply_peak=fsp_peak,
        ))
        last_event_t = fly_ts[i]
        # advance past dip + gap
        skip_t = fly_ts[min_idx] + int(EVENT_GAP_S * 1e6)
        i = max(min_idx + 1, bisect_right(fly_ts, skip_t))
    return shots


def per_log_stats(shots: list[Shot]) -> dict[str, float | None | int]:
    if not shots:
        return {"n": 0}

    def stat(key: str, q=None):
        vals = [getattr(s, key) for s in shots if getattr(s, key) is not None]
        if not vals:
            return None
        if q is None:
            return _median(vals)
        return _quantile(vals, q)

    return {
        "n": len(shots),
        "fly_pre_med": stat("fly_pre"),
        "fly_pre_p90": stat("fly_pre", 0.90),
        "fly_pre_max": stat("fly_pre", 1.0),
        "fly_dip_med": stat("fly_dip"),
        "fly_recover_med": stat("fly_recover_s"),
        "feeder_pre_med": stat("feeder_pre"),
        "feeder_pre_p90": stat("feeder_pre", 0.90),
        "feeder_pre_max": stat("feeder_pre", 1.0),
        "feeder_drop_med": stat("feeder_drop"),
        "launcher_cur_peak_med": stat("launcher_cur_peak"),
        "feeder_stator_peak_med": stat("feeder_stator_peak"),
        "feeder_supply_peak_med": stat("feeder_supply_peak"),
    }


def _fmt(v, prec=2):
    if v is None: return "-"
    if isinstance(v, int): return str(v)
    return f"{v:.{prec}f}"


def main() -> None:
    files = sorted(LOG_DIR.glob(GLOB))
    print(f"Analyzing {len(files)} logs (sharper window)...")

    rows: list[dict] = []
    for f in files:
        try:
            shots = detect_and_window(f)
        except Exception as exc:  # noqa: BLE001
            print(f"  {f.name}: ERROR {exc}")
            continue
        s = per_log_stats(shots)
        s["log"] = f.name
        s["hour"] = int(f.name.split("_")[2][:2])
        rows.append(s)

    # Filter to logs that actually shot
    shooting = [r for r in rows if r.get("n", 0) >= 5]
    print(f"\n{len(shooting)} logs had >=5 shots")

    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    cols = ["log", "hour", "n",
            "fly_pre_med", "fly_pre_p90", "fly_pre_max",
            "fly_dip_med", "fly_recover_med",
            "feeder_pre_med", "feeder_pre_p90", "feeder_pre_max",
            "feeder_drop_med",
            "launcher_cur_peak_med",
            "feeder_stator_peak_med", "feeder_supply_peak_med"]
    with OUT_CSV.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(cols)
        for r in rows:
            w.writerow([_fmt(r.get(c), 3) if isinstance(r.get(c), float) else r.get(c, "")
                        for c in cols])
    print(f"Per-log v2 -> {OUT_CSV}")

    # ---- Print per-log table for shooting logs ----
    print("\n=== Per-log medians (shooting logs only) ===")
    print(f"{'log':<30} {'n':>3}  "
          f"{'fly_pre_med':>11} {'fly_pre_p90':>11} {'fly_pre_max':>11}  "
          f"{'fly_dip':>7} {'fly_rec':>7}  "
          f"{'fdrPre_med':>10} {'fdrPre_p90':>10} {'fdrPre_max':>10}  "
          f"{'fdrDrop':>7}  {'curPk':>6}  {'fstPk':>6}  {'fspPk':>6}")
    for r in shooting:
        print(f"{r['log']:<30} {r['n']:>3}  "
              f"{_fmt(r['fly_pre_med']):>11} {_fmt(r['fly_pre_p90']):>11} {_fmt(r['fly_pre_max']):>11}  "
              f"{_fmt(r['fly_dip_med']):>7} {_fmt(r['fly_recover_med'], 3):>7}  "
              f"{_fmt(r['feeder_pre_med']):>10} {_fmt(r['feeder_pre_p90']):>10} {_fmt(r['feeder_pre_max']):>10}  "
              f"{_fmt(r['feeder_drop_med']):>7}  "
              f"{_fmt(r['launcher_cur_peak_med']):>6}  "
              f"{_fmt(r['feeder_stator_peak_med']):>6}  "
              f"{_fmt(r['feeder_supply_peak_med']):>6}")

    # ---- Group comparison: practice (HH < 12) vs match (HH >= 12) ----
    early = [r for r in shooting if r["hour"] < 12]
    late  = [r for r in shooting if r["hour"] >= 12]

    def group_med(group: list[dict], key: str) -> float | None:
        vals = [r[key] for r in group if r.get(key) is not None]
        return _median(vals) if vals else None

    print("\n=== Group medians (per-log medians, equal weight) ===")
    print(f"{'metric':<25} {'EARLY (HH<12)':>15} {'LATE (HH>=12)':>15}  delta")
    metrics = [
        "fly_pre_med", "fly_pre_p90", "fly_pre_max",
        "fly_dip_med", "fly_recover_med",
        "feeder_pre_med", "feeder_pre_p90", "feeder_pre_max",
        "feeder_drop_med",
        "launcher_cur_peak_med",
        "feeder_stator_peak_med", "feeder_supply_peak_med",
    ]
    for m in metrics:
        e = group_med(early, m)
        l = group_med(late, m)
        d = (l - e) if (e is not None and l is not None) else None
        d_str = f"{d:+.3f}" if d is not None else "-"
        print(f"{m:<25} {_fmt(e):>15} {_fmt(l):>15}  {d_str:>8}")

    # ---- Match-only late (HH >= 19) since 16-18 was probably qualification practice ----
    late_match = [r for r in shooting if r["hour"] >= 19]
    print(f"\n=== HH>=19 (probably eliminations / late matches) ===")
    print(f"{'metric':<25} {'LATE_MATCH(HH>=19)':>20}")
    for m in metrics:
        l = group_med(late_match, m)
        print(f"{m:<25} {_fmt(l):>20}")

    # Specific log comparisons: pick clear matched pairs
    pairs = [
        ("FRC_20260425_042056.wpilog", "FRC_20260425_202750.wpilog"),
        ("FRC_20260425_034807.wpilog", "FRC_20260425_201136.wpilog"),
        ("FRC_20260425_040029.wpilog", "FRC_20260425_201848.wpilog"),
    ]
    print("\n=== Direct early-vs-late log pair comparisons ===")
    by_log = {r["log"]: r for r in rows}
    for early_log, late_log in pairs:
        e = by_log.get(early_log)
        l = by_log.get(late_log)
        if not e or not l:
            continue
        print(f"\n  {early_log}  vs  {late_log}")
        for m in metrics:
            ev, lv = e.get(m), l.get(m)
            d = (lv - ev) if (isinstance(ev, (int, float)) and isinstance(lv, (int, float))) else None
            d_str = f"{d:+.3f}" if d is not None else "-"
            print(f"    {m:<25} {_fmt(ev):>10} -> {_fmt(lv):>10}  {d_str:>8}")


if __name__ == "__main__":
    main()
