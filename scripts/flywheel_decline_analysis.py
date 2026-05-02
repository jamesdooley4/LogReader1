"""Investigate flywheel performance decline across April 25 2026 session.

Strategy:
- For each .wpilog, detect "ball-shot" events as rapid drops in
  ``NT:/launcher/velocity`` (the flywheel motor RPS).  A drop > DIP_THRESHOLD
  RPS within DIP_WINDOW_S, while the pre-drop velocity is above SHOT_MIN_RPS,
  is treated as one shot event.
- For each event capture:
    * pre-shot velocity  (median of last ~100 ms before drop)
    * minimum velocity   (lowest value during DIP_WINDOW_S)
    * recovery time      (time to return within RECOVER_BAND_RPS of pre-shot)
    * supply current peak during the dip
    * feeder velocity at start of dip
    * feeder stator/supply current peak during dip window
- Aggregate per-log medians/p10/p90 and dump a CSV plus a console summary
  comparing the early logs to the late logs.
"""

from __future__ import annotations

import csv
import statistics
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
from pathlib import Path

from logreader.models import LogData, SignalData
from logreader.wpilog_reader import read_wpilog

LOG_DIR = Path(r"C:\Users\jdool_46clpzz\Documents\FRC_Sessions\.staging\logs")
GLOB = "FRC_20260425_*.wpilog"
OUT_CSV = Path(__file__).resolve().parent.parent / "out" / "flywheel_decline_summary.csv"
OUT_EVENTS_CSV = OUT_CSV.with_name("flywheel_decline_events.csv")

# Shot detection tuning
SHOT_MIN_RPS = 60.0          # only consider drops while flywheel was spinning fast
DIP_THRESHOLD_RPS = 8.0      # min drop in velocity to count as a shot
DIP_WINDOW_S = 0.30          # look this far ahead for the min after a candidate
PRE_WINDOW_S = 0.20          # time window prior used to estimate pre-shot velocity
RECOVER_BAND_RPS = 3.0       # how close to pre-shot to consider recovered
RECOVER_TIMEOUT_S = 1.5
EVENT_GAP_S = 0.5            # minimum spacing between successive shot events

LAUNCHER_VELOCITY = "NT:/launcher/velocity"
LAUNCHER_CURRENT  = "NT:/launcher/current"
FEEDER_VELOCITY   = "NT:/SmartDashboard/FeederSubsystem/VelocityRPS"
FEEDER_STATOR     = "/FeederLogs/statorCurrent"
FEEDER_SUPPLY     = "/FeederLogs/supplyCurrent"


@dataclass
class ShotEvent:
    log: str
    t_us: int
    pre_vel: float
    min_vel: float
    dip_rps: float
    recover_s: float | None
    launcher_current_peak: float | None
    feeder_vel: float | None
    feeder_stator_peak: float | None
    feeder_supply_peak: float | None


@dataclass
class LogSummary:
    log: str
    duration_s: float
    n_shots: int
    pre_vel_median: float | None
    pre_vel_p10: float | None
    pre_vel_p90: float | None
    dip_median: float | None
    recover_median: float | None
    launcher_current_median: float | None
    launcher_current_p90: float | None
    feeder_vel_median: float | None
    feeder_stator_median: float | None
    feeder_stator_p90: float | None
    feeder_supply_median: float | None


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


def _value_at(ts: list[int], vs: list[float], t_us: int) -> float | None:
    """Last value at-or-before t_us (zero-order hold)."""
    if not ts:
        return None
    i = bisect_right(ts, t_us) - 1
    if i < 0:
        return None
    return vs[i]


def _peak_in_window(ts: list[int], vs: list[float], t0: int, t1: int) -> float | None:
    if not ts:
        return None
    lo = bisect_left(ts, t0)
    hi = bisect_right(ts, t1)
    if lo >= hi:
        v = _value_at(ts, vs, t1)
        return v
    return max(vs[lo:hi])


def _median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def _quantile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    pos = q * (len(s) - 1)
    lo = int(pos)
    hi = min(lo + 1, len(s) - 1)
    frac = pos - lo
    return s[lo] * (1 - frac) + s[hi] * frac


def _detect_shots(ts: list[int], vs: list[float]) -> list[tuple[int, int, float, float]]:
    """Return list of (start_idx, min_idx, pre_vel, min_vel) for shot events."""
    events: list[tuple[int, int, float, float]] = []
    if len(ts) < 5:
        return events

    last_event_t = -10**18
    n = len(ts)
    i = 0
    while i < n:
        if vs[i] < SHOT_MIN_RPS:
            i += 1
            continue
        if ts[i] - last_event_t < EVENT_GAP_S * 1e6:
            i += 1
            continue
        # estimate pre-shot velocity = median over PRE_WINDOW_S preceding ts[i]
        t_pre_lo = ts[i] - int(PRE_WINDOW_S * 1e6)
        lo = bisect_left(ts, t_pre_lo)
        if lo >= i:
            pre_vel = vs[i]
        else:
            pre_vel = statistics.median(vs[lo:i + 1])

        if pre_vel < SHOT_MIN_RPS:
            i += 1
            continue

        # find minimum within DIP_WINDOW_S
        t_end = ts[i] + int(DIP_WINDOW_S * 1e6)
        hi = bisect_right(ts, t_end)
        if hi <= i + 1:
            i += 1
            continue
        seg = vs[i:hi]
        min_idx_offset = min(range(len(seg)), key=lambda k: seg[k])
        min_idx = i + min_idx_offset
        min_vel = seg[min_idx_offset]
        if pre_vel - min_vel >= DIP_THRESHOLD_RPS:
            events.append((i, min_idx, pre_vel, min_vel))
            last_event_t = ts[i]
            # advance past the dip
            i = max(min_idx + 1, i + 1)
            # also skip forward past the recovery window so we don't double-count
            t_skip = ts[min_idx] + int(EVENT_GAP_S * 1e6)
            i = max(i, bisect_right(ts, t_skip))
            continue
        i += 1
    return events


def _recovery_seconds(ts: list[int], vs: list[float], min_idx: int, pre_vel: float) -> float | None:
    target = pre_vel - RECOVER_BAND_RPS
    t0 = ts[min_idx]
    t_limit = t0 + int(RECOVER_TIMEOUT_S * 1e6)
    n = len(ts)
    for k in range(min_idx + 1, n):
        if ts[k] > t_limit:
            return None
        if vs[k] >= target:
            return (ts[k] - t0) / 1e6
    return None


def analyze_log(path: Path) -> tuple[LogSummary, list[ShotEvent]]:
    data: LogData = read_wpilog(path)
    fly_ts, fly_vs = _series(data.signals.get(LAUNCHER_VELOCITY))
    cur_ts, cur_vs = _series(data.signals.get(LAUNCHER_CURRENT))
    fv_ts, fv_vs = _series(data.signals.get(FEEDER_VELOCITY))
    fs_ts, fs_vs = _series(data.signals.get(FEEDER_STATOR))
    fp_ts, fp_vs = _series(data.signals.get(FEEDER_SUPPLY))

    duration_s = 0.0
    if fly_ts:
        duration_s = (fly_ts[-1] - fly_ts[0]) / 1e6
    elif fs_ts:
        duration_s = (fs_ts[-1] - fs_ts[0]) / 1e6

    events: list[ShotEvent] = []
    if fly_ts:
        for i, min_idx, pre_vel, min_vel in _detect_shots(fly_ts, fly_vs):
            t_event = fly_ts[i]
            t_end = fly_ts[min_idx] + int(DIP_WINDOW_S * 1e6)
            recover = _recovery_seconds(fly_ts, fly_vs, min_idx, pre_vel)
            cur_peak = _peak_in_window(cur_ts, cur_vs, t_event, t_end)
            feeder_v = _value_at(fv_ts, fv_vs, t_event)
            feeder_stator = _peak_in_window(fs_ts, fs_vs, t_event, t_end)
            feeder_supply = _peak_in_window(fp_ts, fp_vs, t_event, t_end)
            events.append(ShotEvent(
                log=path.name,
                t_us=t_event,
                pre_vel=pre_vel,
                min_vel=min_vel,
                dip_rps=pre_vel - min_vel,
                recover_s=recover,
                launcher_current_peak=cur_peak,
                feeder_vel=feeder_v,
                feeder_stator_peak=feeder_stator,
                feeder_supply_peak=feeder_supply,
            ))

    pre_vels = [e.pre_vel for e in events]
    dips = [e.dip_rps for e in events]
    recovers = [e.recover_s for e in events if e.recover_s is not None]
    cur_peaks = [e.launcher_current_peak for e in events if e.launcher_current_peak is not None]
    feeder_vels = [e.feeder_vel for e in events if e.feeder_vel is not None]
    feeder_stator = [e.feeder_stator_peak for e in events if e.feeder_stator_peak is not None]
    feeder_supply = [e.feeder_supply_peak for e in events if e.feeder_supply_peak is not None]

    summary = LogSummary(
        log=path.name,
        duration_s=duration_s,
        n_shots=len(events),
        pre_vel_median=_median(pre_vels),
        pre_vel_p10=_quantile(pre_vels, 0.10),
        pre_vel_p90=_quantile(pre_vels, 0.90),
        dip_median=_median(dips),
        recover_median=_median(recovers),
        launcher_current_median=_median(cur_peaks),
        launcher_current_p90=_quantile(cur_peaks, 0.90),
        feeder_vel_median=_median(feeder_vels),
        feeder_stator_median=_median(feeder_stator),
        feeder_stator_p90=_quantile(feeder_stator, 0.90),
        feeder_supply_median=_median(feeder_supply),
    )
    return summary, events


def _fmt(v: float | None, prec: int = 2) -> str:
    return f"{v:.{prec}f}" if v is not None else "-"


def main() -> None:
    files = sorted(LOG_DIR.glob(GLOB))
    if not files:
        raise SystemExit(f"No logs found at {LOG_DIR / GLOB}")

    summaries: list[LogSummary] = []
    all_events: list[ShotEvent] = []

    print(f"Analyzing {len(files)} logs...")
    for f in files:
        try:
            s, evs = analyze_log(f)
        except Exception as exc:  # noqa: BLE001
            print(f"  {f.name}: ERROR {exc}")
            continue
        summaries.append(s)
        all_events.extend(evs)
        print(f"  {f.name}: shots={s.n_shots:3d}  "
              f"pre_med={_fmt(s.pre_vel_median)}  "
              f"dip_med={_fmt(s.dip_median)}  "
              f"recover_med={_fmt(s.recover_median, 3)} s  "
              f"launcher_cur_med={_fmt(s.launcher_current_median)} A  "
              f"feeder_vel={_fmt(s.feeder_vel_median)}  "
              f"feeder_stator_p90={_fmt(s.feeder_stator_p90)} A")

    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with OUT_CSV.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow([
            "log", "duration_s", "n_shots",
            "pre_vel_median", "pre_vel_p10", "pre_vel_p90",
            "dip_median", "recover_median",
            "launcher_current_median", "launcher_current_p90",
            "feeder_vel_median", "feeder_stator_median",
            "feeder_stator_p90", "feeder_supply_median",
        ])
        for s in summaries:
            w.writerow([
                s.log, f"{s.duration_s:.1f}", s.n_shots,
                _fmt(s.pre_vel_median), _fmt(s.pre_vel_p10), _fmt(s.pre_vel_p90),
                _fmt(s.dip_median), _fmt(s.recover_median, 3),
                _fmt(s.launcher_current_median), _fmt(s.launcher_current_p90),
                _fmt(s.feeder_vel_median), _fmt(s.feeder_stator_median),
                _fmt(s.feeder_stator_p90), _fmt(s.feeder_supply_median),
            ])
    print(f"\nPer-log summary -> {OUT_CSV}")

    with OUT_EVENTS_CSV.open("w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow([
            "log", "t_us", "pre_vel", "min_vel", "dip_rps", "recover_s",
            "launcher_current_peak", "feeder_vel",
            "feeder_stator_peak", "feeder_supply_peak",
        ])
        for e in all_events:
            w.writerow([
                e.log, e.t_us,
                f"{e.pre_vel:.3f}", f"{e.min_vel:.3f}", f"{e.dip_rps:.3f}",
                _fmt(e.recover_s, 3),
                _fmt(e.launcher_current_peak),
                _fmt(e.feeder_vel),
                _fmt(e.feeder_stator_peak),
                _fmt(e.feeder_supply_peak),
            ])
    print(f"All shot events -> {OUT_EVENTS_CSV}")

    # Bucket summary: by hour bucket of file name (HH portion)
    def bucket(name: str) -> str:
        # FRC_20260425_HHMMSS.wpilog
        try:
            hh = name.split("_")[2][:2]
            return f"{hh}xx"
        except Exception:  # noqa: BLE001
            return "??"

    buckets: dict[str, list[ShotEvent]] = {}
    for e in all_events:
        buckets.setdefault(bucket(e.log), []).append(e)

    print("\n=== Hour-bucket aggregate (across all shots in that hour) ===")
    print(f"{'hour':>5} {'n':>5} {'pre_med':>8} {'pre_p10':>8} {'dip_med':>8} "
          f"{'rec_med':>8} {'cur_med':>8} {'cur_p90':>8} {'fdrV_med':>9} "
          f"{'fdrSt_p90':>10}")
    for h in sorted(buckets.keys()):
        evs = buckets[h]
        pres = [e.pre_vel for e in evs]
        dips = [e.dip_rps for e in evs]
        recs = [e.recover_s for e in evs if e.recover_s is not None]
        curs = [e.launcher_current_peak for e in evs if e.launcher_current_peak is not None]
        fvs  = [e.feeder_vel for e in evs if e.feeder_vel is not None]
        fst  = [e.feeder_stator_peak for e in evs if e.feeder_stator_peak is not None]
        print(f"{h:>5} {len(evs):>5} "
              f"{_fmt(_median(pres)):>8} "
              f"{_fmt(_quantile(pres, 0.10)):>8} "
              f"{_fmt(_median(dips)):>8} "
              f"{_fmt(_median(recs), 3):>8} "
              f"{_fmt(_median(curs)):>8} "
              f"{_fmt(_quantile(curs, 0.90)):>8} "
              f"{_fmt(_median(fvs)):>9} "
              f"{_fmt(_quantile(fst, 0.90)):>10}")


if __name__ == "__main__":
    main()
