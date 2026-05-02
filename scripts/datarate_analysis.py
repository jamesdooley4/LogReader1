"""Estimate wpilog data rate reduction from the fix.

Computes bytes/second for:
1. Before-fix sessions (no pauses)
2. Fix sessions - full file, excluding pause time
3. Fix sessions - only the initial segment before the first pause
"""

import os
from pathlib import Path
from collections import defaultdict
from wpiutil.log import DataLogReader

BASE_DIR = Path(r"D:\Temp\2026-04-4_Practice")

BEFORE_FILES = [
    BASE_DIR / "FRC_20260404_192241.wpilog",
    BASE_DIR / "FRC_20260404_193538.wpilog",
]

FIX_FILES = [
    BASE_DIR / "FRC_20260404_222104.wpilog",  # 86 errors, 3:15 duration
    BASE_DIR / "FRC_20260404_222638.wpilog",  # 207 errors, 13:50 duration
]

# From the datalog_pause_analysis, first error timestamps (seconds into session):
# FRC_222104: first error at 0:24.69
# FRC_222638: first error at 0:18.42

GAP_THRESHOLD_US = 2_000_000  # 2 seconds in microseconds


def analyze_datarate(path: Path):
    """Analyze data rate of a wpilog file.

    Computes rates by finding contiguous active segments (no gap > 2s)
    and measuring data density in each. Also identifies the first
    "real" active segment (>5s long) and the first DataLog-pause gap
    after that segment ends.
    """
    file_size = path.stat().st_size
    reader = DataLogReader(str(path))

    all_timestamps = []
    record_count = 0
    entries = {}

    for record in reader:
        if record.isStart():
            start = record.getStartData()
            entries[start.entry] = {"name": start.name, "type": start.type}
        elif not record.isControl():
            ts = record.getTimestamp()
            all_timestamps.append(ts)
            record_count += 1

    if not all_timestamps:
        return None

    all_timestamps.sort()
    first_ts = all_timestamps[0]
    last_ts = all_timestamps[-1]
    total_duration_s = (last_ts - first_ts) / 1_000_000

    # Find gaps > threshold
    gaps = []
    for i in range(len(all_timestamps) - 1):
        dt = all_timestamps[i + 1] - all_timestamps[i]
        if dt >= GAP_THRESHOLD_US:
            gaps.append((all_timestamps[i], all_timestamps[i + 1], dt))

    total_gap_us = sum(g[2] for g in gaps)
    total_gap_s = total_gap_us / 1_000_000
    active_duration_s = total_duration_s - total_gap_s

    # Build active segments: intervals between gaps
    segments = []  # (start_ts, end_ts, record_count)
    seg_start_idx = 0
    gap_idx = 0
    gap_starts = {g[0] for g in gaps}

    for i in range(len(all_timestamps) - 1):
        if all_timestamps[i] in gap_starts:
            # End current segment
            seg_records = i - seg_start_idx + 1
            seg_start = all_timestamps[seg_start_idx]
            seg_end = all_timestamps[i]
            segments.append((seg_start, seg_end, seg_records))
            # Next segment starts after gap
            seg_start_idx = i + 1
    # Final segment
    if seg_start_idx < len(all_timestamps):
        seg_records = len(all_timestamps) - seg_start_idx
        segments.append((all_timestamps[seg_start_idx], all_timestamps[-1], seg_records))

    # Find the first "real" active segment (> 5 seconds)
    first_real_seg = None
    first_real_seg_idx = None
    for si, (s, e, n) in enumerate(segments):
        dur = (e - s) / 1_000_000
        if dur >= 5.0:
            first_real_seg = (s, e, n)
            first_real_seg_idx = si
            break

    # Find the first DataLog-pause gap that occurs after the first real segment
    first_pause_gap = None
    first_pause_gap_ts_s = None  # relative to file start
    if first_real_seg is not None:
        for g in gaps:
            if g[0] >= first_real_seg[0]:
                first_pause_gap = g
                first_pause_gap_ts_s = (g[0] - first_ts) / 1_000_000
                break

    # Records and duration up to the first pause gap (from first_real_seg start)
    pre_pause_records = None
    pre_pause_duration_s = None
    if first_pause_gap is not None and first_real_seg is not None:
        pre_pause_records = sum(
            1 for ts in all_timestamps
            if first_real_seg[0] <= ts <= first_pause_gap[0]
        )
        pre_pause_duration_s = (first_pause_gap[0] - first_real_seg[0]) / 1_000_000
    elif first_real_seg is not None:
        # No pause gap after first real segment
        pre_pause_records = first_real_seg[2]
        pre_pause_duration_s = (first_real_seg[1] - first_real_seg[0]) / 1_000_000

    bytes_per_record = file_size / record_count if record_count > 0 else 0

    pre_pause_bytes_est = None
    if pre_pause_records is not None:
        pre_pause_bytes_est = pre_pause_records * bytes_per_record

    # Segment summary
    seg_summary = []
    for s, e, n in segments:
        dur = (e - s) / 1_000_000
        if dur > 0:
            rate = (n * bytes_per_record) / dur
        else:
            rate = 0
        seg_summary.append({"start_s": (s - first_ts) / 1e6, "dur_s": dur,
                            "records": n, "rate_bps": rate})

    return {
        "file_size": file_size,
        "record_count": record_count,
        "total_duration_s": total_duration_s,
        "active_duration_s": active_duration_s,
        "gap_count": len(gaps),
        "total_gap_s": total_gap_s,
        "bytes_per_record": bytes_per_record,
        "segments": seg_summary,
        "first_pause_gap_ts_s": first_pause_gap_ts_s,
        "pre_pause_records": pre_pause_records,
        "pre_pause_duration_s": pre_pause_duration_s,
        "pre_pause_bytes_est": pre_pause_bytes_est,
    }


def fmt_size(b):
    if b >= 1_000_000:
        return f"{b/1_000_000:.1f} MB"
    if b >= 1_000:
        return f"{b/1_000:.1f} KB"
    return f"{b} B"


def fmt_rate(bps):
    if bps >= 1_000_000:
        return f"{bps/1_000_000:.2f} MB/s"
    if bps >= 1_000:
        return f"{bps/1_000:.1f} KB/s"
    return f"{bps:.0f} B/s"


def main():
    print("=" * 100)
    print("WPILOG Data Rate Analysis")
    print("=" * 100)

    all_results = []

    for label, files in [("BEFORE-FIX", BEFORE_FILES), ("FIX", FIX_FILES)]:
        print(f"\n--- {label} ---")
        for f in files:
            print(f"\nAnalyzing {f.name}...")
            info = analyze_datarate(f)
            if not info:
                print("  No data")
                continue

            all_results.append((label, f.name, info))

            rate_active = info["file_size"] / info["active_duration_s"] if info["active_duration_s"] > 0 else 0

            print(f"  File size:         {fmt_size(info['file_size'])}")
            print(f"  Records:           {info['record_count']:,}")
            print(f"  Bytes/record:      {info['bytes_per_record']:.1f}")
            print(f"  Total duration:    {info['total_duration_s']:.1f}s")
            print(f"  Active duration:   {info['active_duration_s']:.1f}s")
            print(f"  Gaps (>2s):        {info['gap_count']}  ({info['total_gap_s']:.1f}s total)")
            print(f"  Rate (active):     {fmt_rate(rate_active)}")

            # Show segments
            print(f"  Segments:")
            for i, seg in enumerate(info["segments"]):
                if seg["dur_s"] >= 1.0:
                    print(f"    [{i}] {seg['start_s']:.1f}s - {seg['start_s']+seg['dur_s']:.1f}s"
                          f"  dur={seg['dur_s']:.1f}s  records={seg['records']:,}"
                          f"  rate={fmt_rate(seg['rate_bps'])}")

            if info["pre_pause_duration_s"] is not None and info["pre_pause_duration_s"] > 1:
                pre_rate = info["pre_pause_bytes_est"] / info["pre_pause_duration_s"]
                print(f"  --- Pre-first-pause segment ---")
                if info["first_pause_gap_ts_s"] is not None:
                    print(f"  First pause at:    {info['first_pause_gap_ts_s']:.1f}s into file")
                print(f"  Pre-pause duration:{info['pre_pause_duration_s']:.1f}s")
                print(f"  Pre-pause records: {info['pre_pause_records']:,}")
                print(f"  Pre-pause bytes:   ~{fmt_size(info['pre_pause_bytes_est'])}")
                print(f"  Pre-pause rate:    ~{fmt_rate(pre_rate)}")

    # Summary comparison
    print(f"\n\n{'=' * 100}")
    print("COMPARISON SUMMARY")
    print(f"{'=' * 100}")

    before_results = [(n, i) for (l, n, i) in all_results if l == "BEFORE-FIX"]
    fix_results = [(n, i) for (l, n, i) in all_results if l == "FIX"]

    # Compute rates from the longest real segment in each file
    print(f"\n  Per-file active data rate:")
    for label_files, results in [("Before-fix", before_results), ("Fix", fix_results)]:
        print(f"  {label_files}:")
        for name, info in results:
            rate = info["file_size"] / info["active_duration_s"]
            # Find largest segment rate
            biggest_seg = max(info["segments"], key=lambda s: s["dur_s"])
            print(f"    {name}:")
            print(f"      Full active:     {fmt_rate(rate)}  ({fmt_size(info['file_size'])} / {info['active_duration_s']:.0f}s active)")
            print(f"      Longest segment: {fmt_rate(biggest_seg['rate_bps'])}  ({biggest_seg['dur_s']:.0f}s)")
            if info["pre_pause_duration_s"] and info["pre_pause_duration_s"] > 5:
                pre_rate = info["pre_pause_bytes_est"] / info["pre_pause_duration_s"]
                print(f"      Pre-first-pause: {fmt_rate(pre_rate)}  ({info['pre_pause_duration_s']:.0f}s)")

    # Averages
    def avg_rate(results):
        rates = [i["file_size"] / i["active_duration_s"] for _, i in results]
        return sum(rates) / len(rates) if rates else 0

    def avg_biggest_seg_rate(results):
        rates = []
        for _, i in results:
            biggest = max(i["segments"], key=lambda s: s["dur_s"])
            rates.append(biggest["rate_bps"])
        return sum(rates) / len(rates) if rates else 0

    def avg_pre_pause_rate(results):
        rates = []
        for _, i in results:
            if i["pre_pause_duration_s"] and i["pre_pause_duration_s"] > 5:
                rates.append(i["pre_pause_bytes_est"] / i["pre_pause_duration_s"])
        return sum(rates) / len(rates) if rates else 0

    ab = avg_rate(before_results)
    af = avg_rate(fix_results)
    ab_seg = avg_biggest_seg_rate(before_results)
    af_seg = avg_biggest_seg_rate(fix_results)
    ab_pre = avg_pre_pause_rate(before_results)
    af_pre = avg_pre_pause_rate(fix_results)

    print(f"\n  --- Reduction estimates ---")
    if ab > 0 and af > 0:
        print(f"  Full active rate:   {fmt_rate(ab)} -> {fmt_rate(af)}  = {(1-af/ab)*100:.1f}% reduction")
    if ab_seg > 0 and af_seg > 0:
        print(f"  Longest segment:    {fmt_rate(ab_seg)} -> {fmt_rate(af_seg)}  = {(1-af_seg/ab_seg)*100:.1f}% reduction")
    if ab_pre > 0 and af_pre > 0:
        print(f"  Pre-first-pause:    {fmt_rate(ab_pre)} -> {fmt_rate(af_pre)}  = {(1-af_pre/ab_pre)*100:.1f}% reduction")


if __name__ == "__main__":
    main()
