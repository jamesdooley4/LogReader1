"""Compare signals using median inter-sample period to be robust to DataLog gaps.

The fix sessions have periodic pauses where the DataLog writer stalls for ~1s.
Using median period filters out those gaps and shows the true logging rate.
Also excludes limelight signals that depend on AprilTag visibility.
"""

import os
import subprocess
import tempfile
import statistics
from pathlib import Path
from collections import defaultdict

from wpiutil.log import DataLogReader

BASE_DIR = Path(r"D:\Temp\2026-04-4_Practice")
OWLET = r"c:\Tools\owlet-26.1.0-windowsx86-64.exe"


def analyze_wpilog_signals(path: str) -> dict:
    """Analyze a wpilog file. Compute median-based frequency for each signal."""
    reader = DataLogReader(path)

    entries = {}
    signal_samples = defaultdict(list)

    for record in reader:
        if record.isStart():
            start = record.getStartData()
            entries[start.entry] = {"name": start.name, "type": start.type}
        elif not record.isControl():
            entry_id = record.getEntry()
            ts = record.getTimestamp()
            if entry_id in entries:
                signal_samples[entries[entry_id]["name"]].append(ts)

    signals = {}
    for name, timestamps in sorted(signal_samples.items()):
        entry_info = next((v for v in entries.values() if v["name"] == name), None)
        timestamps.sort()
        count = len(timestamps)
        first_ts = timestamps[0]
        last_ts = timestamps[-1]

        # Compute inter-sample periods
        periods_ms = [(timestamps[i+1] - timestamps[i]) / 1000.0
                      for i in range(len(timestamps) - 1)]

        median_period_ms = None
        median_freq_hz = None
        p10_period_ms = None
        p90_period_ms = None
        if periods_ms:
            median_period_ms = statistics.median(periods_ms)
            if median_period_ms > 0:
                median_freq_hz = 1000.0 / median_period_ms
            sorted_periods = sorted(periods_ms)
            n = len(sorted_periods)
            p10_period_ms = sorted_periods[int(n * 0.1)]
            p90_period_ms = sorted_periods[int(n * 0.9)]

        # Also compute "active frequency": exclude gaps > 2 seconds
        active_periods = [p for p in periods_ms if p < 2000]
        active_median_freq = None
        if active_periods:
            med = statistics.median(active_periods)
            if med > 0:
                active_median_freq = 1000.0 / med

        # Count gaps > 2 seconds
        gap_count = sum(1 for p in periods_ms if p >= 2000)

        signals[name] = {
            "type": entry_info["type"] if entry_info else "unknown",
            "count": count,
            "duration_s": (last_ts - first_ts) / 1_000_000,
            "median_period_ms": median_period_ms,
            "median_freq_hz": median_freq_hz,
            "active_median_freq_hz": active_median_freq,
            "p10_period_ms": p10_period_ms,
            "p90_period_ms": p90_period_ms,
            "gap_count": gap_count,
        }

    all_ts = [ts for tss in signal_samples.values() for ts in tss]
    total_duration = (max(all_ts) - min(all_ts)) / 1_000_000 if all_ts else 0
    return {"signals": signals, "duration_s": total_duration}


def is_apriltag_dependent(name: str) -> bool:
    """Check if a signal depends on AprilTag visibility."""
    # These signals only publish when a camera sees tags
    patterns = [
        "limelight-a/botpose", "limelight-b/botpose",
        "limelight-a/targetpose", "limelight-b/targetpose",
        "limelight-a/camerapose", "limelight-b/camerapose",
        "limelight-a/rawfiducials", "limelight-b/rawfiducials",
        "limelight-a/stddevs", "limelight-b/stddevs",
        "limelight-a/ta", "limelight-b/ta",
        "limelight-a/tv", "limelight-b/tv",
        "limelight-a/tx", "limelight-b/tx",
        "limelight-a/ty", "limelight-b/ty",
        "limelight-a/txnc", "limelight-b/txnc",
        "limelight-a/tync", "limelight-b/tync",
        "limelight-a/tid", "limelight-b/tid",
        "limelight-a/t2d", "limelight-b/t2d",
        "limelight-a/tc", "limelight-b/tc",
        "limelight-a/tl", "limelight-b/tl",
        "limelight-a/cl", "limelight-b/cl",
        "limelight-a/hb", "limelight-b/hb",
        "limelight-a/imu", "limelight-b/imu",
        "limelight-a/hw", "limelight-b/hw",
        "limelight-a/botpose_orb", "limelight-b/botpose_orb",
        "vision/rawFieldPose3d", "vision/fieldPose3d",
        "vision/compBot", "Field/RawVision",
        "_rejectReason", "_visionError", "_tagSpread",
        "Mt1 STD", "Mt2 STD", "_Num targets",
        "_Last timestamp", "_time since last",
    ]
    return any(p in name for p in patterns)


def fmt_freq(f):
    if f is None:
        return "N/A"
    if f >= 100:
        return f"{f:.0f}"
    if f >= 10:
        return f"{f:.1f}"
    return f"{f:.2f}"


def print_comparison(label_a, info_a, label_b, info_b):
    sigs_a = set(info_a["signals"].keys())
    sigs_b = set(info_b["signals"].keys())
    only_a = sorted(sigs_a - sigs_b)
    only_b = sorted(sigs_b - sigs_a)
    common = sorted(sigs_a & sigs_b)

    print(f"\n  Duration: {label_a}={info_a['duration_s']:.1f}s, {label_b}={info_b['duration_s']:.1f}s")
    print(f"  Signal count: {label_a}={len(sigs_a)}, {label_b}={len(sigs_b)}")

    if only_a:
        print(f"\n  Signals ONLY in {label_a} ({len(only_a)}):")
        for name in only_a:
            s = info_a["signals"][name]
            tag = " [AprilTag-dep]" if is_apriltag_dependent(name) else ""
            freq = fmt_freq(s["active_median_freq_hz"])
            print(f"    {name:<70s}  type={s['type']:<14s}  n={s['count']:>6d}  activeFreq={freq:>8s} Hz{tag}")

    if only_b:
        print(f"\n  Signals ONLY in {label_b} ({len(only_b)}):")
        for name in only_b:
            s = info_b["signals"][name]
            tag = " [AprilTag-dep]" if is_apriltag_dependent(name) else ""
            freq = fmt_freq(s["active_median_freq_hz"])
            print(f"    {name:<70s}  type={s['type']:<14s}  n={s['count']:>6d}  activeFreq={freq:>8s} Hz{tag}")

    # Common signals - compare active median frequency
    changed = []
    unchanged = []
    apriltag_dep = []

    for name in common:
        sa = info_a["signals"][name]
        sb = info_b["signals"][name]
        fa = sa["active_median_freq_hz"]
        fb = sb["active_median_freq_hz"]

        if is_apriltag_dependent(name):
            apriltag_dep.append((name, sa, sb))
            continue

        if fa is None or fb is None:
            continue

        ratio = fb / fa if fa > 0 else float('inf')
        if abs(ratio - 1.0) > 0.15:
            changed.append((name, sa, sb, ratio))
        else:
            unchanged.append((name, sa, sb, ratio))

    print(f"\n  Excluding {len(apriltag_dep)} AprilTag-dependent signals from comparison")

    if changed:
        print(f"\n  === CHANGED active frequencies ({len(changed)}) ===")
        print(f"    {'Signal':<70s}  {'Type':<14s}  {label_a+' Hz':>10s}  {label_b+' Hz':>10s}  {'Change':>8s}  {label_b+' gaps':>8s}")
        print(f"    {'-'*70}  {'-'*14}  {'-'*10}  {'-'*10}  {'-'*8}  {'-'*8}")
        for name, sa, sb, ratio in sorted(changed, key=lambda x: x[3]):
            fa = sa["active_median_freq_hz"]
            fb = sb["active_median_freq_hz"]
            pct = (ratio - 1) * 100
            gaps = sb["gap_count"]
            print(f"    {name:<70s}  {sa['type']:<14s}  {fmt_freq(fa):>10s}  {fmt_freq(fb):>10s}  {pct:>+7.0f}%  {gaps:>8d}")

    n_unch = len(unchanged)
    print(f"\n  === UNCHANGED active frequencies ({n_unch}) ===")
    if n_unch <= 50:
        for name, sa, sb, ratio in sorted(unchanged, key=lambda x: x[0]):
            fa = sa["active_median_freq_hz"]
            fb = sb["active_median_freq_hz"]
            gaps = sb["gap_count"]
            print(f"    {name:<70s}  {sa['type']:<14s}  {fmt_freq(fa):>10s}  {fmt_freq(fb):>10s}  gaps={gaps}")
    else:
        # Group by frequency band
        bands = defaultdict(list)
        for name, sa, sb, ratio in unchanged:
            fa = sa["active_median_freq_hz"]
            if fa >= 200:
                bands["200+ Hz"].append(name)
            elif fa >= 50:
                bands["50-200 Hz"].append(name)
            elif fa >= 10:
                bands["10-50 Hz"].append(name)
            elif fa >= 1:
                bands["1-10 Hz"].append(name)
            else:
                bands["<1 Hz"].append(name)
        for band in ["200+ Hz", "50-200 Hz", "10-50 Hz", "1-10 Hz", "<1 Hz"]:
            if band in bands:
                names = bands[band]
                print(f"    {band}: {len(names)} signals (e.g., {names[0]}, ...)" if len(names) > 3
                      else f"    {band}: {', '.join(names)}")


def convert_hoot(hoot_path, output_dir):
    out_path = output_dir / (hoot_path.stem + ".wpilog")
    try:
        result = subprocess.run(
            [OWLET, str(hoot_path), str(out_path), "-f", "wpilog"],
            capture_output=True, text=True, timeout=120
        )
        if result.returncode != 0:
            print(f"  owlet error for {hoot_path.name}: {result.stderr[:200]}")
            return None
        return out_path
    except Exception as e:
        print(f"  owlet exception: {e}")
        return None


def main():
    print("=" * 120)
    print("Signal Comparison (Median Frequency, Gap-Robust)")
    print("=" * 120)

    # Before-fix wpilog (no DataLog error)
    before_wpilog = BASE_DIR / "FRC_20260404_193538.wpilog"   # DS 12:35, long clean session
    # Fix sessions
    fix_wpilog_1 = BASE_DIR / "FRC_20260404_222104.wpilog"    # DS 15:20, 86 errors
    fix_wpilog_2 = BASE_DIR / "FRC_20260404_222638.wpilog"    # DS 15:26, 207 errors

    print("\n--- WPILOG ---")

    print(f"\nAnalyzing before-fix: {before_wpilog.name} ...")
    before_info = analyze_wpilog_signals(str(before_wpilog))
    print(f"  Duration: {before_info['duration_s']:.1f}s, Signals: {len(before_info['signals'])}")

    print(f"\nAnalyzing fix-1: {fix_wpilog_1.name} ...")
    fix1_info = analyze_wpilog_signals(str(fix_wpilog_1))
    print(f"  Duration: {fix1_info['duration_s']:.1f}s, Signals: {len(fix1_info['signals'])}")

    print(f"\nAnalyzing fix-2: {fix_wpilog_2.name} ...")
    fix2_info = analyze_wpilog_signals(str(fix_wpilog_2))
    print(f"  Duration: {fix2_info['duration_s']:.1f}s, Signals: {len(fix2_info['signals'])}")

    # Show gap stats for fix sessions
    for label, info in [("Fix-1", fix1_info), ("Fix-2", fix2_info)]:
        # Find a high-rate signal to characterize gaps
        for probe_name in ["NT:/DriveState/Pose", "NT:/DriveState/Timestamp"]:
            if probe_name in info["signals"]:
                s = info["signals"][probe_name]
                print(f"\n  {label} gap stats ({probe_name}): "
                      f"gaps(>2s)={s['gap_count']}, "
                      f"median_period={s['median_period_ms']:.1f}ms, "
                      f"p10={s['p10_period_ms']:.1f}ms, p90={s['p90_period_ms']:.1f}ms, "
                      f"active_freq={fmt_freq(s['active_median_freq_hz'])} Hz")
                break

    print(f"\n{'=' * 120}")
    print(f"WPILOG: {before_wpilog.name} (before) vs {fix_wpilog_1.name} (fix-1)")
    print(f"{'=' * 120}")
    print_comparison("Before", before_info, "Fix-1", fix1_info)

    print(f"\n{'=' * 120}")
    print(f"WPILOG: {before_wpilog.name} (before) vs {fix_wpilog_2.name} (fix-2)")
    print(f"{'=' * 120}")
    print_comparison("Before", before_info, "Fix-2", fix2_info)

    # --- HOOT Rio ---
    print(f"\n\n{'=' * 120}")
    print("HOOT (Rio) COMPARISON")
    print(f"{'=' * 120}")

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)

        before_hoot = BASE_DIR / "2026-04-04_19-35-30" / "rio_2026-04-04_19-35-32.hoot"
        fix_hoot = BASE_DIR / "2026-04-04_22-20-57" / "rio_2026-04-04_22-21-01.hoot"

        print(f"\nConverting {before_hoot.parent.name}/{before_hoot.name} ...")
        bw = convert_hoot(before_hoot, tmpdir)
        print(f"Converting {fix_hoot.parent.name}/{fix_hoot.name} ...")
        fw = convert_hoot(fix_hoot, tmpdir)

        if bw and fw:
            bi = analyze_wpilog_signals(str(bw))
            fi = analyze_wpilog_signals(str(fw))
            print(f"  Before: {bi['duration_s']:.1f}s, {len(bi['signals'])} signals")
            print(f"  Fix:    {fi['duration_s']:.1f}s, {len(fi['signals'])} signals")
            print(f"\n{'=' * 120}")
            print(f"HOOT Rio: before vs fix")
            print(f"{'=' * 120}")
            print_comparison("Before", bi, "Fix", fi)


if __name__ == "__main__":
    main()
