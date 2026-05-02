"""Compare signals and logging frequencies between before-fix and fix wpilog/hoot files.

Identifies differences in:
- Which signals are present
- Sampling frequency of each signal
"""

import os
import subprocess
import tempfile
from pathlib import Path
from collections import defaultdict

from wpiutil.log import DataLogReader, DataLogRecord

OWLET = r"c:\Tools\owlet-26.1.0-windowsx86-64.exe"
BASE_DIR = Path(r"D:\Temp\2026-04-4_Practice")


def analyze_wpilog_signals(path: str) -> dict:
    """Analyze a wpilog file and return signal info.
    
    Returns dict with:
      - signals: {name: {type, count, first_ts, last_ts, avg_period_ms}}
      - duration_s: total duration in seconds
    """
    reader = DataLogReader(path)
    
    # Build entry ID -> name/type map from start records
    entries = {}  # id -> {name, type, metadata}
    signal_samples = defaultdict(list)  # name -> [timestamps_us]
    
    for record in reader:
        if record.isStart():
            start = record.getStartData()
            entries[start.entry] = {
                "name": start.name,
                "type": start.type,
                "metadata": start.metadata,
            }
        elif record.isFinish():
            pass
        elif record.isSetMetadata():
            pass
        elif record.isControl():
            pass
        else:
            # Data record
            entry_id = record.getEntry()
            ts = record.getTimestamp()
            if entry_id in entries:
                name = entries[entry_id]["name"]
                signal_samples[name].append(ts)
    
    # Compute statistics
    signals = {}
    for name, timestamps in sorted(signal_samples.items()):
        entry_info = None
        for eid, info in entries.items():
            if info["name"] == name:
                entry_info = info
                break
        
        count = len(timestamps)
        first_ts = min(timestamps)
        last_ts = max(timestamps)
        duration_ms = (last_ts - first_ts) / 1000.0
        
        avg_period_ms = None
        freq_hz = None
        if count > 1 and duration_ms > 0:
            avg_period_ms = duration_ms / (count - 1)
            freq_hz = 1000.0 / avg_period_ms if avg_period_ms > 0 else None
        
        signals[name] = {
            "type": entry_info["type"] if entry_info else "unknown",
            "count": count,
            "first_ts_us": first_ts,
            "last_ts_us": last_ts,
            "duration_s": (last_ts - first_ts) / 1_000_000,
            "avg_period_ms": avg_period_ms,
            "freq_hz": freq_hz,
        }
    
    all_ts = [ts for tss in signal_samples.values() for ts in tss]
    total_duration = (max(all_ts) - min(all_ts)) / 1_000_000 if all_ts else 0
    
    return {"signals": signals, "duration_s": total_duration}


def convert_hoot(hoot_path: Path, output_dir: Path) -> Path | None:
    """Convert a .hoot file to .wpilog using owlet."""
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
        print(f"  owlet exception for {hoot_path.name}: {e}")
        return None


def print_comparison(label_a: str, info_a: dict, label_b: str, info_b: dict):
    """Print a comparison of two signal analyses."""
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
            freq = f"{s['freq_hz']:.1f} Hz" if s['freq_hz'] else "N/A"
            print(f"    {name:<70s}  type={s['type']:<12s}  n={s['count']:>6d}  freq={freq}")
    
    if only_b:
        print(f"\n  Signals ONLY in {label_b} ({len(only_b)}):")
        for name in only_b:
            s = info_b["signals"][name]
            freq = f"{s['freq_hz']:.1f} Hz" if s['freq_hz'] else "N/A"
            print(f"    {name:<70s}  type={s['type']:<12s}  n={s['count']:>6d}  freq={freq}")
    
    # Common signals with frequency differences
    print(f"\n  Common signals with frequency changes ({len(common)} total common):")
    print(f"    {'Signal':<70s}  {'Type':<12s}  {label_a+' Hz':>12s}  {label_b+' Hz':>12s}  {'Change':>10s}")
    print(f"    {'─'*70}  {'─'*12}  {'─'*12}  {'─'*12}  {'─'*10}")
    
    changed = []
    unchanged = []
    for name in common:
        sa = info_a["signals"][name]
        sb = info_b["signals"][name]
        fa = sa["freq_hz"]
        fb = sb["freq_hz"]
        
        if fa is None or fb is None:
            continue
        
        ratio = fb / fa if fa > 0 else float('inf')
        if abs(ratio - 1.0) > 0.15:  # >15% change
            changed.append((name, sa, sb, ratio))
        else:
            unchanged.append((name, sa, sb, ratio))
    
    if changed:
        print(f"\n    === CHANGED frequencies ({len(changed)}) ===")
        for name, sa, sb, ratio in sorted(changed, key=lambda x: x[3]):
            fa = sa["freq_hz"]
            fb = sb["freq_hz"]
            pct = (ratio - 1) * 100
            print(f"    {name:<70s}  {sa['type']:<12s}  {fa:>10.1f}  {fb:>10.1f}  {pct:>+8.0f}%")
    
    if unchanged:
        print(f"\n    === UNCHANGED frequencies ({len(unchanged)}) ===")
        for name, sa, sb, ratio in sorted(unchanged, key=lambda x: x[0]):
            fa = sa["freq_hz"]
            fb = sb["freq_hz"]
            print(f"    {name:<70s}  {sa['type']:<12s}  {fa:>10.1f}  {fb:>10.1f}")


def main():
    print("=" * 120)
    print("Signal Comparison: Before-Fix vs Fix Sessions")
    print("=" * 120)
    
    # ── WPILOG comparison ─────────────────────────────────────────────────
    # Before-fix: first two sessions with no DataLog error
    before_wpilogs = [
        BASE_DIR / "FRC_20260404_192241.wpilog",   # DS 12:22:40 - no error
        BASE_DIR / "FRC_20260404_193538.wpilog",    # DS 12:35:24 - no error
    ]
    # Fix sessions
    fix_wpilogs = [
        BASE_DIR / "FRC_20260404_222104.wpilog",    # DS 15:20:52 - 86 errors (fix)
        BASE_DIR / "FRC_20260404_222638.wpilog",    # DS 15:26:36 - 207 errors (fix)
    ]
    
    print("\n" + "─" * 120)
    print("WPILOG COMPARISON")
    print("─" * 120)
    
    before_infos = []
    fix_infos = []
    
    for path in before_wpilogs:
        print(f"\nAnalyzing before-fix: {path.name} ...")
        info = analyze_wpilog_signals(str(path))
        before_infos.append((path.name, info))
        print(f"  Duration: {info['duration_s']:.1f}s, Signals: {len(info['signals'])}")
    
    for path in fix_wpilogs:
        print(f"\nAnalyzing fix:        {path.name} ...")
        info = analyze_wpilog_signals(str(path))
        fix_infos.append((path.name, info))
        print(f"  Duration: {info['duration_s']:.1f}s, Signals: {len(info['signals'])}")
    
    # Compare before[0] vs fix[0] and before[1] vs fix[1]
    for i in range(2):
        bn, bi = before_infos[i]
        fn, fi = fix_infos[i]
        print(f"\n{'=' * 120}")
        print(f"WPILOG: {bn} (before) vs {fn} (fix)")
        print(f"{'=' * 120}")
        print_comparison("Before", bi, "Fix", fi)
    
    # ── HOOT comparison ──────────────────────────────────────────────────
    # Before-fix hoot dirs
    before_hoot_dirs = [
        BASE_DIR / "2026-04-04_19-35-30",   # DS 12:35:24
        BASE_DIR / "2026-04-04_21-40-39",   # DS 14:40:34 (before fix)
    ]
    # Fix hoot dir
    fix_hoot_dirs = [
        BASE_DIR / "2026-04-04_22-20-57",   # DS 15:20:52 (fix session 1)
        BASE_DIR / "2026-04-04_22-42-40",   # DS 15:42:35 (after fix, for comparison)
    ]
    
    print(f"\n\n{'=' * 120}")
    print("HOOT (Rio) COMPARISON")
    print("=" * 120)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)
        
        # Find and convert rio hoot files
        before_hoot_infos = []
        fix_hoot_infos = []
        
        for hdir in before_hoot_dirs:
            rio_hoots = list(hdir.glob("rio_*.hoot"))
            if rio_hoots:
                hoot = rio_hoots[0]
                print(f"\nConverting before-fix hoot: {hdir.name}/{hoot.name} ...")
                wpilog = convert_hoot(hoot, tmpdir)
                if wpilog:
                    info = analyze_wpilog_signals(str(wpilog))
                    before_hoot_infos.append((f"{hdir.name}/{hoot.name}", info))
                    print(f"  Duration: {info['duration_s']:.1f}s, Signals: {len(info['signals'])}")
        
        for hdir in fix_hoot_dirs:
            rio_hoots = list(hdir.glob("rio_*.hoot"))
            if rio_hoots:
                hoot = rio_hoots[0]
                print(f"\nConverting fix hoot:        {hdir.name}/{hoot.name} ...")
                wpilog = convert_hoot(hoot, tmpdir)
                if wpilog:
                    info = analyze_wpilog_signals(str(wpilog))
                    fix_hoot_infos.append((f"{hdir.name}/{hoot.name}", info))
                    print(f"  Duration: {info['duration_s']:.1f}s, Signals: {len(info['signals'])}")
        
        # Compare
        if before_hoot_infos and fix_hoot_infos:
            bn, bi = before_hoot_infos[0]
            fn, fi = fix_hoot_infos[0]
            print(f"\n{'=' * 120}")
            print(f"HOOT Rio: {bn} (before) vs {fn} (fix)")
            print(f"{'=' * 120}")
            print_comparison("Before", bi, "Fix", fi)
            
            if len(before_hoot_infos) > 1 and len(fix_hoot_infos) > 1:
                bn, bi = before_hoot_infos[1]
                fn, fi = fix_hoot_infos[1]
                print(f"\n{'=' * 120}")
                print(f"HOOT Rio: {bn} (before) vs {fn} (after-fix)")
                print(f"{'=' * 120}")
                print_comparison("Before", bi, "After", fi)
    
    # ── HOOT CANivore comparison ──────────────────────────────────────────
    print(f"\n\n{'=' * 120}")
    print("HOOT (CANivore) COMPARISON")
    print("=" * 120)
    
    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)
        
        before_can_infos = []
        fix_can_infos = []
        
        for hdir in before_hoot_dirs:
            can_hoots = list(hdir.glob("ECA*.hoot"))
            if can_hoots:
                hoot = can_hoots[0]
                print(f"\nConverting before-fix CANivore: {hdir.name}/{hoot.name} ...")
                wpilog = convert_hoot(hoot, tmpdir)
                if wpilog:
                    info = analyze_wpilog_signals(str(wpilog))
                    before_can_infos.append((f"{hdir.name}/{hoot.name}", info))
                    print(f"  Duration: {info['duration_s']:.1f}s, Signals: {len(info['signals'])}")
        
        for hdir in fix_hoot_dirs:
            can_hoots = list(hdir.glob("ECA*.hoot"))
            if can_hoots:
                hoot = can_hoots[0]
                print(f"\nConverting fix CANivore:        {hdir.name}/{hoot.name} ...")
                wpilog = convert_hoot(hoot, tmpdir)
                if wpilog:
                    info = analyze_wpilog_signals(str(wpilog))
                    fix_can_infos.append((f"{hdir.name}/{hoot.name}", info))
                    print(f"  Duration: {info['duration_s']:.1f}s, Signals: {len(info['signals'])}")
        
        if before_can_infos and fix_can_infos:
            bn, bi = before_can_infos[0]
            fn, fi = fix_can_infos[0]
            print(f"\n{'=' * 120}")
            print(f"HOOT CANivore: {bn} (before) vs {fn} (fix)")
            print(f"{'=' * 120}")
            print_comparison("Before", bi, "Fix", fi)


if __name__ == "__main__":
    main()
