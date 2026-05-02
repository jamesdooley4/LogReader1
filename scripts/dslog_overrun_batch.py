"""Batch analysis of dslog/dsevents files for loop overrun patterns.

Reads all dslog sessions from a directory. For each session:
- Extracts roboRIO memory-free timeline from dsevents
- Extracts trip time and CPU utilization from dslog
- Correlates memory pressure with loop overruns
- Detects GC pauses (memory jumps) and their timing
- Identifies behavioral changes between sessions (possible code deploys)
"""

from __future__ import annotations

import re
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

from logreader.dslog_reader import read_ds_logs

# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------

_MEMORY_RE = re.compile(
    r"roboRIO:\s+Disk Free\s+(\d+)\s+MB,\s+Memory Free\s+(\d+)\s+MB"
)

OVERRUN_THRESHOLD_MS = 20.0  # nominal 50 Hz loop period


@dataclass
class MemorySample:
    timestamp_s: float
    disk_free_mb: int
    mem_free_mb: int


@dataclass
class GCEvent:
    """A detected GC pause — memory jumps up between consecutive samples."""
    timestamp_s: float
    mem_before_mb: int
    mem_after_mb: int
    reclaimed_mb: int


@dataclass
class SessionSummary:
    filename: str
    duration_s: float
    # Trip time
    total_samples: int
    overrun_count: int
    overrun_pct: float
    trip_p50_ms: float
    trip_p90_ms: float
    trip_p95_ms: float
    trip_p99_ms: float
    trip_max_ms: float
    # CPU
    cpu_mean_pct: float
    cpu_p95_pct: float
    cpu_max_pct: float
    # Memory
    mem_start_mb: int | None
    mem_min_mb: int | None
    mem_end_mb: int | None
    mem_samples: list[MemorySample] = field(default_factory=list)
    gc_events: list[GCEvent] = field(default_factory=list)
    # Overrun-vs-memory correlation
    overruns_when_low_mem: int = 0  # overruns when mem < 20 MB
    overruns_when_high_mem: int = 0  # overruns when mem >= 20 MB
    samples_low_mem: int = 0
    samples_high_mem: int = 0
    # Mode info
    has_teleop: bool = False
    has_auto: bool = False
    teleop_overrun_count: int = 0
    disabled_overrun_count: int = 0


def _percentile(data: list[float], pct: int) -> float:
    if not data:
        return 0.0
    if len(data) == 1:
        return data[0]
    s = sorted(data)
    k = (pct / 100.0) * (len(s) - 1)
    f = int(k)
    c = min(f + 1, len(s) - 1)
    return s[f] + (k - f) * (s[c] - s[f])


def _parse_memory_events(log_data) -> list[MemorySample]:
    sig = log_data.get_signal("/DSEvents")
    if sig is None:
        return []
    samples = []
    for v in sig.values:
        m = _MEMORY_RE.search(str(v.value))
        if m:
            samples.append(MemorySample(
                timestamp_s=v.timestamp_us / 1e6,
                disk_free_mb=int(m.group(1)),
                mem_free_mb=int(m.group(2)),
            ))
    return samples


def _detect_gc_events(mem_samples: list[MemorySample], min_reclaim_mb: int = 10) -> list[GCEvent]:
    events = []
    for i in range(1, len(mem_samples)):
        prev = mem_samples[i - 1]
        curr = mem_samples[i]
        reclaimed = curr.mem_free_mb - prev.mem_free_mb
        if reclaimed >= min_reclaim_mb:
            events.append(GCEvent(
                timestamp_s=curr.timestamp_s,
                mem_before_mb=prev.mem_free_mb,
                mem_after_mb=curr.mem_free_mb,
                reclaimed_mb=reclaimed,
            ))
    return events


def _mem_at_time(mem_samples: list[MemorySample], t_s: float) -> int | None:
    """Get the closest memory reading at or before time t_s."""
    if not mem_samples:
        return None
    best = None
    for s in mem_samples:
        if s.timestamp_s <= t_s + 5.0:  # allow 5s slack
            best = s.mem_free_mb
        else:
            break
    return best


def analyse_session(dslog_path: Path) -> SessionSummary | None:
    """Analyse a single dslog+dsevents session."""
    try:
        log_data = read_ds_logs(str(dslog_path))
    except Exception as e:
        print(f"  SKIP {dslog_path.name}: {e}", file=sys.stderr)
        return None

    trip_sig = log_data.get_signal("/DSLog/TripTimeMS")
    cpu_sig = log_data.get_signal("/DSLog/CPUUtilization")
    teleop_sig = log_data.get_signal("/DSLog/Status/RobotTeleop")
    auto_sig = log_data.get_signal("/DSLog/Status/RobotAuto")
    disabled_sig = log_data.get_signal("/DSLog/Status/RobotDisabled")

    if trip_sig is None or not trip_sig.values:
        return None

    # Basic trip time stats
    trips = [v.value for v in trip_sig.values if isinstance(v.value, (int, float))]
    if not trips:
        return None

    duration_s = (trip_sig.values[-1].timestamp_us - trip_sig.values[0].timestamp_us) / 1e6
    if duration_s < 5.0:
        return None

    overruns = [t for t in trips if t > OVERRUN_THRESHOLD_MS]

    # CPU stats
    cpus = [v.value * 100 for v in cpu_sig.values if isinstance(v.value, (int, float))] if cpu_sig else []

    # Memory
    mem_samples = _parse_memory_events(log_data)
    gc_events = _detect_gc_events(mem_samples)

    # Mode detection
    has_teleop = teleop_sig is not None and any(v.value for v in teleop_sig.values)
    has_auto = auto_sig is not None and any(v.value for v in auto_sig.values)

    # Correlate overruns with memory level
    low_mem_threshold = 20  # MB
    overruns_low = 0
    overruns_high = 0
    samples_low = 0
    samples_high = 0

    if mem_samples:
        for v in trip_sig.values:
            t_s = v.timestamp_us / 1e6
            mem = _mem_at_time(mem_samples, t_s)
            if mem is None:
                continue
            is_overrun = isinstance(v.value, (int, float)) and v.value > OVERRUN_THRESHOLD_MS
            if mem < low_mem_threshold:
                samples_low += 1
                if is_overrun:
                    overruns_low += 1
            else:
                samples_high += 1
                if is_overrun:
                    overruns_high += 1

    # Phase-correlated overrun counts
    teleop_overruns = 0
    disabled_overruns = 0
    if teleop_sig and disabled_sig:
        # Build a quick phase lookup from boolean signals
        teleop_times = {v.timestamp_us: bool(v.value) for v in teleop_sig.values}
        disabled_times = {v.timestamp_us: bool(v.value) for v in disabled_sig.values}

        # For each trip sample, find nearest phase
        import bisect
        teleop_ts = sorted(teleop_times.keys())
        disabled_ts = sorted(disabled_times.keys())
        teleop_vals = [teleop_times[t] for t in teleop_ts]
        disabled_vals = [disabled_times[t] for t in disabled_ts]

        for v in trip_sig.values:
            if not (isinstance(v.value, (int, float)) and v.value > OVERRUN_THRESHOLD_MS):
                continue
            t = v.timestamp_us
            # Check teleop
            idx = bisect.bisect_right(teleop_ts, t) - 1
            is_teleop = idx >= 0 and teleop_vals[idx]
            idx = bisect.bisect_right(disabled_ts, t) - 1
            is_disabled = idx >= 0 and disabled_vals[idx]
            if is_teleop:
                teleop_overruns += 1
            elif is_disabled:
                disabled_overruns += 1

    return SessionSummary(
        filename=dslog_path.stem,
        duration_s=round(duration_s, 1),
        total_samples=len(trips),
        overrun_count=len(overruns),
        overrun_pct=round(len(overruns) / len(trips) * 100, 2) if trips else 0,
        trip_p50_ms=round(_percentile(trips, 50), 2),
        trip_p90_ms=round(_percentile(trips, 90), 2),
        trip_p95_ms=round(_percentile(trips, 95), 2),
        trip_p99_ms=round(_percentile(trips, 99), 2),
        trip_max_ms=round(max(trips), 2),
        cpu_mean_pct=round(statistics.mean(cpus), 1) if cpus else 0,
        cpu_p95_pct=round(_percentile(cpus, 95), 1) if cpus else 0,
        cpu_max_pct=round(max(cpus), 1) if cpus else 0,
        mem_start_mb=mem_samples[0].mem_free_mb if mem_samples else None,
        mem_min_mb=min(s.mem_free_mb for s in mem_samples) if mem_samples else None,
        mem_end_mb=mem_samples[-1].mem_free_mb if mem_samples else None,
        mem_samples=mem_samples,
        gc_events=gc_events,
        overruns_when_low_mem=overruns_low,
        overruns_when_high_mem=overruns_high,
        samples_low_mem=samples_low,
        samples_high_mem=samples_high,
        has_teleop=has_teleop,
        has_auto=has_auto,
        teleop_overrun_count=teleop_overruns,
        disabled_overrun_count=disabled_overruns,
    )


def main() -> None:
    dslog_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(r"D:\Temp\2026-04-4_Practice\dslogs")

    dslog_files = sorted(dslog_dir.glob("*.dslog"))
    print(f"Found {len(dslog_files)} dslog files in {dslog_dir}\n")

    summaries: list[SessionSummary] = []
    for f in dslog_files:
        s = analyse_session(f)
        if s is not None:
            summaries.append(s)

    if not summaries:
        print("No valid sessions found.")
        return

    # -----------------------------------------------------------------------
    # Session overview table
    # -----------------------------------------------------------------------
    print("=" * 140)
    print("SESSION OVERVIEW")
    print("=" * 140)
    print(
        f"{'#':>3s}  {'Time':>8s}  {'Dur':>5s}  {'Mode':>6s}  "
        f"{'Overruns':>8s}  {'%':>6s}  "
        f"{'p50':>6s} {'p95':>6s} {'p99':>6s} {'Max':>7s}  "
        f"{'CPU%':>5s} {'C95%':>5s}  "
        f"{'MemStart':>8s} {'MemMin':>6s} {'MemEnd':>6s}  "
        f"{'GCs':>3s}  {'OvLow':>5s} {'OvHi':>5s}"
    )
    print("-" * 140)

    for i, s in enumerate(summaries):
        time_str = s.filename.split(" ")[1].replace("_", ":")
        mode = ""
        if s.has_auto:
            mode += "A"
        if s.has_teleop:
            mode += "T"
        if not mode:
            mode = "D"

        mem_start = f"{s.mem_start_mb}MB" if s.mem_start_mb is not None else "-"
        mem_min = f"{s.mem_min_mb}MB" if s.mem_min_mb is not None else "-"
        mem_end = f"{s.mem_end_mb}MB" if s.mem_end_mb is not None else "-"
        gc_count = len(s.gc_events)

        # Overrun rates by memory zone
        ov_low_str = f"{s.overruns_when_low_mem}" if s.samples_low_mem > 0 else "-"
        ov_hi_str = f"{s.overruns_when_high_mem}" if s.samples_high_mem > 0 else "-"

        print(
            f"{i+1:>3d}  {time_str:>8s}  {s.duration_s:>5.0f}  {mode:>6s}  "
            f"{s.overrun_count:>8d}  {s.overrun_pct:>5.1f}%  "
            f"{s.trip_p50_ms:>6.1f} {s.trip_p95_ms:>6.1f} {s.trip_p99_ms:>6.1f} {s.trip_max_ms:>7.1f}  "
            f"{s.cpu_mean_pct:>5.1f} {s.cpu_p95_pct:>5.1f}  "
            f"{mem_start:>8s} {mem_min:>6s} {mem_end:>6s}  "
            f"{gc_count:>3d}  {ov_low_str:>5s} {ov_hi_str:>5s}"
        )

    # -----------------------------------------------------------------------
    # Memory-vs-overrun correlation
    # -----------------------------------------------------------------------
    print("\n" + "=" * 80)
    print("MEMORY vs OVERRUN CORRELATION")
    print("=" * 80)

    total_low_samples = sum(s.samples_low_mem for s in summaries)
    total_high_samples = sum(s.samples_high_mem for s in summaries)
    total_low_overruns = sum(s.overruns_when_low_mem for s in summaries)
    total_high_overruns = sum(s.overruns_when_high_mem for s in summaries)

    low_rate = total_low_overruns / total_low_samples * 100 if total_low_samples > 0 else 0
    high_rate = total_high_overruns / total_high_samples * 100 if total_high_samples > 0 else 0

    print(f"  Memory >= 20 MB:  {total_high_overruns:>6d} overruns in {total_high_samples:>8d} samples ({high_rate:.2f}%)")
    print(f"  Memory <  20 MB:  {total_low_overruns:>6d} overruns in {total_low_samples:>8d} samples ({low_rate:.2f}%)")
    if high_rate > 0:
        print(f"  Relative risk:    {low_rate/high_rate:.1f}× more overruns when memory is low" if high_rate > 0 and low_rate > 0 else "")

    # -----------------------------------------------------------------------
    # GC event details
    # -----------------------------------------------------------------------
    print("\n" + "=" * 80)
    print("GC EVENTS (memory jumps >= 10 MB)")
    print("=" * 80)

    all_gc = [(s.filename, gc) for s in summaries for gc in s.gc_events]
    if all_gc:
        for fname, gc in all_gc:
            time_str = fname.split(" ")[1].replace("_", ":")
            print(f"  {time_str} +{gc.timestamp_s:>6.0f}s: {gc.mem_before_mb:>3d} MB → {gc.mem_after_mb:>3d} MB (+{gc.reclaimed_mb} MB reclaimed)")
    else:
        print("  No significant GC events detected.")

    # -----------------------------------------------------------------------
    # Memory drain rate analysis
    # -----------------------------------------------------------------------
    print("\n" + "=" * 80)
    print("MEMORY DRAIN RATE (MB/min during initial decline)")
    print("=" * 80)

    for s in summaries:
        if len(s.mem_samples) < 3:
            continue
        # Find the initial decline phase (before first GC or mem < 10 MB)
        decline_samples = []
        for i, ms in enumerate(s.mem_samples):
            if i > 0 and ms.mem_free_mb > s.mem_samples[i-1].mem_free_mb + 5:
                break  # GC happened
            decline_samples.append(ms)
            if ms.mem_free_mb < 10:
                break

        if len(decline_samples) >= 2:
            dt_min = (decline_samples[-1].timestamp_s - decline_samples[0].timestamp_s) / 60
            dmem = decline_samples[0].mem_free_mb - decline_samples[-1].mem_free_mb
            rate = dmem / dt_min if dt_min > 0 else 0
            time_str = s.filename.split(" ")[1].replace("_", ":")
            print(
                f"  {time_str}: {decline_samples[0].mem_free_mb}→{decline_samples[-1].mem_free_mb} MB "
                f"over {dt_min:.1f} min = {rate:.1f} MB/min"
            )

    # -----------------------------------------------------------------------
    # Detect behavioral changes between sessions
    # -----------------------------------------------------------------------
    print("\n" + "=" * 80)
    print("BEHAVIORAL CHANGES BETWEEN SESSIONS")
    print("=" * 80)

    prev = None
    for i, s in enumerate(summaries):
        if prev is None:
            prev = s
            continue

        changes = []

        # Check for significant overrun rate change
        if prev.overrun_pct > 0 and s.overrun_pct > 0:
            ratio = s.overrun_pct / prev.overrun_pct
            if ratio > 2.0:
                changes.append(f"overrun rate {ratio:.1f}× INCREASE ({prev.overrun_pct:.1f}% → {s.overrun_pct:.1f}%)")
            elif ratio < 0.5:
                changes.append(f"overrun rate {1/ratio:.1f}× DECREASE ({prev.overrun_pct:.1f}% → {s.overrun_pct:.1f}%)")
        elif prev.overrun_pct == 0 and s.overrun_pct > 0.5:
            changes.append(f"overruns appeared: 0% → {s.overrun_pct:.1f}%")
        elif prev.overrun_pct > 0.5 and s.overrun_pct == 0:
            changes.append(f"overruns GONE: {prev.overrun_pct:.1f}% → 0%")

        # Check trip time p95 change
        if prev.trip_p95_ms > 0:
            delta = s.trip_p95_ms - prev.trip_p95_ms
            if abs(delta) > 2.0:
                direction = "UP" if delta > 0 else "DOWN"
                changes.append(f"p95 trip time {direction}: {prev.trip_p95_ms:.1f} → {s.trip_p95_ms:.1f} ms")

        # Check memory drain rate change
        if (prev.mem_start_mb and s.mem_start_mb and
            prev.mem_start_mb > 50 and s.mem_start_mb > 50):
            # Compare initial memory — different start point suggests reboot/redeploy
            delta = abs(s.mem_start_mb - prev.mem_start_mb)
            if delta > 30:
                changes.append(f"mem start changed: {prev.mem_start_mb}→{s.mem_start_mb} MB (possible redeploy)")

        # Check CPU utilization change
        delta_cpu = s.cpu_mean_pct - prev.cpu_mean_pct
        if abs(delta_cpu) > 5:
            direction = "UP" if delta_cpu > 0 else "DOWN"
            changes.append(f"mean CPU {direction}: {prev.cpu_mean_pct:.0f}% → {s.cpu_mean_pct:.0f}%")

        if changes:
            t1 = prev.filename.split(" ")[1].replace("_", ":")
            t2 = s.filename.split(" ")[1].replace("_", ":")
            print(f"\n  {t1} → {t2}:")
            for c in changes:
                print(f"    ▸ {c}")

        prev = s

    # -----------------------------------------------------------------------
    # Per-session detailed memory + overrun timeline (long sessions only)
    # -----------------------------------------------------------------------
    print("\n" + "=" * 80)
    print("DETAILED MEMORY TIMELINE (sessions > 60s with memory data)")
    print("=" * 80)

    for s in summaries:
        if s.duration_s < 60 or not s.mem_samples:
            continue

        time_str = s.filename.split(" ")[1].replace("_", ":")
        mode = "AT" if s.has_auto and s.has_teleop else ("T" if s.has_teleop else ("A" if s.has_auto else "D"))
        print(f"\n  Session {time_str} ({s.duration_s:.0f}s, {mode})  "
              f"Overruns: {s.overrun_count} ({s.overrun_pct:.1f}%)")

        # Show memory at key intervals
        for ms in s.mem_samples[::3]:  # every 30s
            elapsed = ms.timestamp_s - s.mem_samples[0].timestamp_s
            bar_len = max(0, ms.mem_free_mb // 4)
            bar = "█" * bar_len
            print(f"    {elapsed:>6.0f}s  {ms.mem_free_mb:>4d} MB  {bar}")

    # -----------------------------------------------------------------------
    # Summary findings
    # -----------------------------------------------------------------------
    print("\n" + "=" * 80)
    print("AGGREGATE FINDINGS")
    print("=" * 80)

    total_overruns = sum(s.overrun_count for s in summaries)
    total_samples = sum(s.total_samples for s in summaries)
    sessions_with_teleop = [s for s in summaries if s.has_teleop]
    sessions_disabled_only = [s for s in summaries if not s.has_teleop and not s.has_auto]

    print(f"  Total sessions analysed:  {len(summaries)}")
    print(f"  Total samples:            {total_samples:,}")
    print(f"  Total overruns:           {total_overruns} ({total_overruns/total_samples*100:.2f}%)")
    print(f"  Sessions with teleop:     {len(sessions_with_teleop)}")
    print(f"  Sessions disabled-only:   {len(sessions_disabled_only)}")

    if sessions_with_teleop:
        teleop_ovr = sum(s.teleop_overrun_count for s in sessions_with_teleop)
        disabled_ovr = sum(s.disabled_overrun_count for s in sessions_with_teleop)
        print(f"  Teleop overruns:          {teleop_ovr}")
        print(f"  Disabled overruns:        {disabled_ovr}")

    # Worst sessions
    by_rate = sorted(summaries, key=lambda s: s.overrun_pct, reverse=True)
    print(f"\n  Worst overrun sessions:")
    for s in by_rate[:5]:
        time_str = s.filename.split(" ")[1].replace("_", ":")
        print(f"    {time_str}: {s.overrun_pct:.1f}% ({s.overrun_count} overruns, dur={s.duration_s:.0f}s)")


if __name__ == "__main__":
    main()
