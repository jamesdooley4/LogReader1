"""Analyze DS logs from practice session for loop overrun patterns.

Reads .dslog + .dsevents files from 6:00 PM (18:xx) onward,
extracts loop overrun messages, CPU/CAN utilization spikes,
brownouts, watchdog events, and mode transitions to identify
patterns that might explain recurring overruns.
"""

from __future__ import annotations

import re
import statistics
import sys
from datetime import datetime, timezone
from pathlib import Path

# Ensure logreader is importable
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from logreader.dslog_reader import read_ds_logs

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

LOG_DIR = Path(r"D:\Temp\2026-04-6_Practice_Test\dslogs")
MIN_HOUR = 18  # 6:00 PM

# Thresholds
CPU_HIGH_THRESHOLD = 0.80   # 80%
CAN_HIGH_THRESHOLD = 0.70   # 70%
TRIP_HIGH_MS = 20.0          # >20 ms round-trip is noteworthy

# Overrun detection in dsevents
OVERRUN_RE = re.compile(r"Loop time of ([\d.]+)s overrun", re.IGNORECASE)
SCHEDULER_OVERRUN_RE = re.compile(r"CommandScheduler loop overrun", re.IGNORECASE)
TRACER_EPOCH_RE = re.compile(
    r"^\s*(\S+(?:\.\w+)*(?:\(\))?)\s*:\s*([\d.]+)s\s*$", re.MULTILINE
)
WATCHDOG_EXPIRED_RE = re.compile(r"Watchdog.*not fed|Watchdog.*expired", re.IGNORECASE)
CAN_ERROR_RE = re.compile(r"CAN\b.*error|CAN\b.*timeout|HAL.*CAN", re.IGNORECASE)
GC_RE = re.compile(r"GC|garbage collect", re.IGNORECASE)


def get_eligible_files() -> list[Path]:
    """Return .dslog files with timestamp >= MIN_HOUR."""
    files = []
    for f in sorted(LOG_DIR.glob("*.dslog")):
        # Filename format: "2026_04_06 HH_MM_SS Mon.dslog"
        parts = f.stem.split()
        if len(parts) >= 2:
            time_part = parts[1]  # "HH_MM_SS"
            hour = int(time_part.split("_")[0])
            if hour >= MIN_HOUR:
                files.append(f)
    return files


def analyze_single_log(dslog_path: Path) -> dict:
    """Analyze one dslog+dsevents pair and return a summary dict."""
    log_data = read_ds_logs(str(dslog_path))

    result = {
        "file": dslog_path.stem,
        "records": 0,
        "duration_s": 0.0,
        "overrun_events": [],
        "scheduler_overruns": 0,
        "tracer_breakdowns": [],
        "watchdog_events": [],
        "can_errors": [],
        "gc_events": [],
        "other_warnings": [],
        "cpu_stats": {},
        "can_stats": {},
        "trip_stats": {},
        "battery_stats": {},
        "modes_seen": set(),
        "high_cpu_samples": 0,
        "high_can_samples": 0,
        "high_trip_samples": 0,
        "brownout_samples": 0,
        "watchdog_flag_samples": 0,
        "total_samples": 0,
    }

    # ---------- Parse DSLog telemetry ----------
    cpu_sig = log_data.get_signal("/DSLog/CPUUtilization")
    can_sig = log_data.get_signal("/DSLog/CANUtilization")
    trip_sig = log_data.get_signal("/DSLog/TripTimeMS")
    batt_sig = log_data.get_signal("/DSLog/BatteryVoltage")
    brownout_sig = log_data.get_signal("/DSLog/Status/Brownout")
    watchdog_sig = log_data.get_signal("/DSLog/Status/Watchdog")
    teleop_sig = log_data.get_signal("/DSLog/Status/RobotTeleop")
    auto_sig = log_data.get_signal("/DSLog/Status/RobotAuto")
    disabled_sig = log_data.get_signal("/DSLog/Status/RobotDisabled")

    if cpu_sig and cpu_sig.values:
        cpu_vals = [v.value for v in cpu_sig.values]
        result["total_samples"] = len(cpu_vals)
        result["records"] = len(cpu_vals)
        duration_us = cpu_sig.values[-1].timestamp_us - cpu_sig.values[0].timestamp_us
        result["duration_s"] = duration_us / 1e6

        result["cpu_stats"] = {
            "min": min(cpu_vals),
            "max": max(cpu_vals),
            "mean": statistics.mean(cpu_vals),
            "median": statistics.median(cpu_vals),
            "p95": sorted(cpu_vals)[int(len(cpu_vals) * 0.95)] if len(cpu_vals) > 20 else max(cpu_vals),
        }
        result["high_cpu_samples"] = sum(1 for v in cpu_vals if v >= CPU_HIGH_THRESHOLD)

    if can_sig and can_sig.values:
        can_vals = [v.value for v in can_sig.values]
        result["can_stats"] = {
            "min": min(can_vals),
            "max": max(can_vals),
            "mean": statistics.mean(can_vals),
            "median": statistics.median(can_vals),
            "p95": sorted(can_vals)[int(len(can_vals) * 0.95)] if len(can_vals) > 20 else max(can_vals),
        }
        result["high_can_samples"] = sum(1 for v in can_vals if v >= CAN_HIGH_THRESHOLD)

    if trip_sig and trip_sig.values:
        trip_vals = [v.value for v in trip_sig.values]
        result["trip_stats"] = {
            "min": min(trip_vals),
            "max": max(trip_vals),
            "mean": statistics.mean(trip_vals),
            "median": statistics.median(trip_vals),
        }
        result["high_trip_samples"] = sum(1 for v in trip_vals if v > TRIP_HIGH_MS)

    if batt_sig and batt_sig.values:
        batt_vals = [v.value for v in batt_sig.values]
        result["battery_stats"] = {
            "min": min(batt_vals),
            "max": max(batt_vals),
            "mean": statistics.mean(batt_vals),
        }

    if brownout_sig and brownout_sig.values:
        result["brownout_samples"] = sum(1 for v in brownout_sig.values if v.value)

    if watchdog_sig and watchdog_sig.values:
        result["watchdog_flag_samples"] = sum(1 for v in watchdog_sig.values if v.value)

    # Track modes
    if teleop_sig and teleop_sig.values:
        if any(v.value for v in teleop_sig.values):
            result["modes_seen"].add("teleop")
    if auto_sig and auto_sig.values:
        if any(v.value for v in auto_sig.values):
            result["modes_seen"].add("auto")
    if disabled_sig and disabled_sig.values:
        if any(v.value for v in disabled_sig.values):
            result["modes_seen"].add("disabled")

    # ---------- Mode transition detection ----------
    # Find timestamps where mode changes, correlate with CPU spikes
    if cpu_sig and teleop_sig and auto_sig and disabled_sig:
        mode_transitions = []
        prev_mode = None
        for i in range(len(cpu_sig.values)):
            if i >= len(teleop_sig.values):
                break
            cur_mode = "disabled"
            if i < len(teleop_sig.values) and teleop_sig.values[i].value:
                cur_mode = "teleop"
            elif i < len(auto_sig.values) and auto_sig.values[i].value:
                cur_mode = "auto"
            if cur_mode != prev_mode and prev_mode is not None:
                ts_s = (cpu_sig.values[i].timestamp_us - cpu_sig.values[0].timestamp_us) / 1e6
                # Look at CPU in a window around the transition
                start_idx = max(0, i - 25)  # 0.5s before
                end_idx = min(len(cpu_sig.values), i + 50)  # 1s after
                window_cpu = [cpu_sig.values[j].value for j in range(start_idx, end_idx)]
                max_cpu_at_transition = max(window_cpu) if window_cpu else 0
                mode_transitions.append({
                    "time_s": round(ts_s, 2),
                    "from": prev_mode,
                    "to": cur_mode,
                    "peak_cpu_around": round(max_cpu_at_transition, 3),
                })
            prev_mode = cur_mode
        result["mode_transitions"] = mode_transitions

    # ---------- Parse DSEvents messages ----------
    events_sig = log_data.get_signal("/DSEvents")
    if events_sig and events_sig.values:
        for v in events_sig.values:
            text = str(v.value)
            ts_s = v.timestamp_us / 1e6

            # Overrun messages
            m = OVERRUN_RE.search(text)
            if m:
                overrun_s = float(m.group(1))
                # Check for Tracer epoch breakdowns in the same message
                epochs = dict(TRACER_EPOCH_RE.findall(text))
                result["overrun_events"].append({
                    "time_s": round(ts_s, 3),
                    "rel_time_s": round((v.timestamp_us - (events_sig.values[0].timestamp_us if events_sig.values else 0)) / 1e6, 2),
                    "loop_time_s": overrun_s,
                    "epochs": epochs,
                })
                continue

            # Tracer lines (not attached to overrun)
            if TRACER_EPOCH_RE.search(text) and not OVERRUN_RE.search(text):
                epochs = dict(TRACER_EPOCH_RE.findall(text))
                result["tracer_breakdowns"].append({
                    "time_s": round(ts_s, 3),
                    "epochs": epochs,
                })
                continue

            # CommandScheduler overrun
            if SCHEDULER_OVERRUN_RE.search(text):
                result["scheduler_overruns"] += 1
                continue

            # Watchdog
            if WATCHDOG_EXPIRED_RE.search(text):
                result["watchdog_events"].append(text[:120])
                continue

            # CAN errors
            if CAN_ERROR_RE.search(text):
                result["can_errors"].append(text[:120])
                continue

            # GC
            if GC_RE.search(text):
                result["gc_events"].append(text[:120])
                continue

    return result


def main() -> None:
    files = get_eligible_files()
    print(f"Found {len(files)} .dslog files from {MIN_HOUR}:00 onward\n")

    all_results = []
    for f in files:
        try:
            r = analyze_single_log(f)
            all_results.append(r)
        except Exception as e:
            print(f"  ERROR reading {f.stem}: {e}")

    # ========================== SUMMARY REPORT ==========================
    print("=" * 80)
    print("LOOP OVERRUN ANALYSIS — Practice Session 6:00 PM+")
    print("=" * 80)

    total_overruns = sum(len(r["overrun_events"]) for r in all_results)
    total_scheduler = sum(r["scheduler_overruns"] for r in all_results)
    total_brownouts = sum(r["brownout_samples"] for r in all_results)
    total_watchdog = sum(r["watchdog_flag_samples"] for r in all_results)

    print(f"\nFiles analyzed:        {len(all_results)}")
    print(f"Total overrun events:  {total_overruns}")
    print(f"Scheduler overruns:    {total_scheduler}")
    print(f"Brownout samples:      {total_brownouts}")
    print(f"Watchdog flag samples: {total_watchdog}")

    # ---------- Per-file summary ----------
    print("\n" + "-" * 80)
    print("PER-FILE SUMMARY")
    print("-" * 80)
    print(f"{'File':<32} {'Dur(s)':>7} {'Ovr':>4} {'SchOvr':>6} {'CPU_p95':>8} {'CAN_p95':>8} {'CPU_max':>8} {'Batt_min':>9} {'Modes':<15}")
    print("-" * 120)

    for r in all_results:
        cpu_p95 = f"{r['cpu_stats'].get('p95', 0)*100:.1f}%" if r['cpu_stats'] else "N/A"
        cpu_max = f"{r['cpu_stats'].get('max', 0)*100:.1f}%" if r['cpu_stats'] else "N/A"
        can_p95 = f"{r['can_stats'].get('p95', 0)*100:.1f}%" if r['can_stats'] else "N/A"
        batt_min = f"{r['battery_stats'].get('min', 0):.2f}V" if r['battery_stats'] else "N/A"
        modes = ",".join(sorted(r["modes_seen"]))
        n_ovr = len(r["overrun_events"])
        print(f"{r['file']:<32} {r['duration_s']:>7.1f} {n_ovr:>4} {r['scheduler_overruns']:>6} {cpu_p95:>8} {can_p95:>8} {cpu_max:>8} {batt_min:>9} {modes:<15}")

    # ---------- Overrun detail ----------
    if total_overruns > 0:
        print("\n" + "-" * 80)
        print("OVERRUN EVENTS DETAIL")
        print("-" * 80)

        for r in all_results:
            if not r["overrun_events"]:
                continue
            print(f"\n  File: {r['file']}")
            print(f"  CPU mean={r['cpu_stats'].get('mean', 0)*100:.1f}%, "
                  f"p95={r['cpu_stats'].get('p95', 0)*100:.1f}%, "
                  f"max={r['cpu_stats'].get('max', 0)*100:.1f}%")
            print(f"  CAN mean={r['can_stats'].get('mean', 0)*100:.1f}%, "
                  f"p95={r['can_stats'].get('p95', 0)*100:.1f}%")

            for ov in r["overrun_events"]:
                print(f"    @{ov['rel_time_s']:>8.2f}s  loop={ov['loop_time_s']:.4f}s", end="")
                if ov["epochs"]:
                    # Sort by duration descending
                    sorted_epochs = sorted(ov["epochs"].items(), key=lambda x: float(x[1]), reverse=True)
                    top = sorted_epochs[:5]
                    breakdown = ", ".join(f"{n}={float(t)*1000:.1f}ms" for n, t in top)
                    print(f"  [{breakdown}]", end="")
                print()

    # ---------- Tracer component aggregate ----------
    all_epochs: dict[str, list[float]] = {}
    for r in all_results:
        for ov in r["overrun_events"]:
            for comp, dur_str in ov["epochs"].items():
                dur = float(dur_str)
                all_epochs.setdefault(comp, []).append(dur)
        for tb in r["tracer_breakdowns"]:
            for comp, dur_str in tb["epochs"].items():
                dur = float(dur_str)
                all_epochs.setdefault(comp, []).append(dur)

    if all_epochs:
        print("\n" + "-" * 80)
        print("TRACER COMPONENT AGGREGATE (from overrun + tracer messages)")
        print("-" * 80)
        print(f"{'Component':<40} {'Count':>6} {'Mean(ms)':>9} {'P95(ms)':>9} {'Max(ms)':>9}")
        print("-" * 80)

        sorted_comps = sorted(all_epochs.items(), key=lambda x: max(x[1]), reverse=True)
        for comp, durations in sorted_comps[:30]:
            ms = [d * 1000 for d in durations]
            mean_ms = statistics.mean(ms)
            p95_ms = sorted(ms)[int(len(ms) * 0.95)] if len(ms) > 5 else max(ms)
            max_ms = max(ms)
            print(f"  {comp:<38} {len(ms):>6} {mean_ms:>9.2f} {p95_ms:>9.2f} {max_ms:>9.2f}")

    # ---------- Mode transition + CPU correlation ----------
    print("\n" + "-" * 80)
    print("MODE TRANSITIONS WITH CPU SPIKES (peak CPU in ±0.5s window)")
    print("-" * 80)

    for r in all_results:
        transitions = r.get("mode_transitions", [])
        hot_transitions = [t for t in transitions if t["peak_cpu_around"] >= 0.5]
        if hot_transitions:
            print(f"\n  File: {r['file']}")
            for t in hot_transitions:
                print(f"    @{t['time_s']:>7.2f}s  {t['from']:>8} -> {t['to']:<8}  peak CPU: {t['peak_cpu_around']*100:.1f}%")

    # ---------- CAN errors ----------
    all_can_errors = []
    for r in all_results:
        for err in r["can_errors"]:
            all_can_errors.append((r["file"], err))

    if all_can_errors:
        print("\n" + "-" * 80)
        print(f"CAN ERRORS ({len(all_can_errors)} total)")
        print("-" * 80)
        for f, err in all_can_errors[:30]:
            print(f"  [{f}] {err}")

    # ---------- Watchdog events ----------
    all_wd = []
    for r in all_results:
        for w in r["watchdog_events"]:
            all_wd.append((r["file"], w))

    if all_wd:
        print("\n" + "-" * 80)
        print(f"WATCHDOG EVENTS ({len(all_wd)} total)")
        print("-" * 80)
        for f, w in all_wd[:30]:
            print(f"  [{f}] {w}")

    # ---------- High CPU files ----------
    print("\n" + "-" * 80)
    print("FILES WITH SUSTAINED HIGH CPU (>80% for >2% of samples)")
    print("-" * 80)

    for r in all_results:
        if r["total_samples"] > 0:
            pct = r["high_cpu_samples"] / r["total_samples"]
            if pct > 0.02:
                print(f"  {r['file']}: {r['high_cpu_samples']}/{r['total_samples']} samples ({pct*100:.1f}%) above {CPU_HIGH_THRESHOLD*100:.0f}%")

    # ---------- High CAN files ----------
    print("\n" + "-" * 80)
    print("FILES WITH HIGH CAN UTILIZATION (>70% for >2% of samples)")
    print("-" * 80)

    for r in all_results:
        if r["total_samples"] > 0:
            pct = r["high_can_samples"] / r["total_samples"]
            if pct > 0.02:
                print(f"  {r['file']}: {r['high_can_samples']}/{r['total_samples']} samples ({pct*100:.1f}%) above {CAN_HIGH_THRESHOLD*100:.0f}%")

    # ---------- Overall pattern summary ----------
    print("\n" + "=" * 80)
    print("PATTERN ANALYSIS")
    print("=" * 80)

    # Which modes had overruns?
    overrun_in_mode = {"teleop": 0, "auto": 0, "disabled": 0, "mixed": 0}
    for r in all_results:
        if r["overrun_events"]:
            modes = r["modes_seen"]
            if "teleop" in modes and "auto" in modes:
                overrun_in_mode["mixed"] += len(r["overrun_events"])
            elif "teleop" in modes:
                overrun_in_mode["teleop"] += len(r["overrun_events"])
            elif "auto" in modes:
                overrun_in_mode["auto"] += len(r["overrun_events"])
            else:
                overrun_in_mode["disabled"] += len(r["overrun_events"])

    print(f"\nOverruns by mode context:")
    for mode, count in overrun_in_mode.items():
        if count > 0:
            print(f"  {mode}: {count}")

    # CPU correlation
    overrun_files_cpu = [r["cpu_stats"].get("mean", 0) for r in all_results if r["overrun_events"] and r["cpu_stats"]]
    no_overrun_files_cpu = [r["cpu_stats"].get("mean", 0) for r in all_results if not r["overrun_events"] and r["cpu_stats"]]

    if overrun_files_cpu:
        print(f"\nMean CPU in files WITH overruns: {statistics.mean(overrun_files_cpu)*100:.1f}%")
    if no_overrun_files_cpu:
        print(f"Mean CPU in files WITHOUT overruns: {statistics.mean(no_overrun_files_cpu)*100:.1f}%")

    # Overrun timing within sessions
    print("\n--- Overrun timing within each session ---")
    for r in all_results:
        if len(r["overrun_events"]) >= 2:
            times = [ov["rel_time_s"] for ov in r["overrun_events"]]
            print(f"  {r['file']}: {len(times)} overruns at t={times}")
            if len(times) >= 2:
                gaps = [times[i+1] - times[i] for i in range(len(times)-1)]
                print(f"    Inter-overrun gaps: {[round(g, 2) for g in gaps]}")

    # ---------- DSEvents messages (all unique) ----------
    print("\n" + "-" * 80)
    print("ALL UNIQUE DSEVENTS MESSAGES (deduped)")
    print("-" * 80)

    unique_msgs: dict[str, int] = {}
    for r in all_results:
        events_sig = None
        # Re-read events to get all messages
        dsevents_path = Path(LOG_DIR) / (r["file"] + ".dsevents")
        if dsevents_path.is_file():
            from logreader.dslog_reader import read_dsevents
            ev_data = read_dsevents(str(dsevents_path))
            ev_sig = ev_data.get_signal("/DSEvents")
            if ev_sig:
                for v in ev_sig.values:
                    text = str(v.value).strip()
                    if text:
                        # Normalize timestamps and numbers for dedup
                        key = text[:200]
                        unique_msgs[key] = unique_msgs.get(key, 0) + 1

    for msg, count in sorted(unique_msgs.items(), key=lambda x: -x[1])[:50]:
        print(f"  [{count:>3}x] {msg}")

    print("\n" + "=" * 80)
    print("ANALYSIS COMPLETE")
    print("=" * 80)


if __name__ == "__main__":
    main()
