"""Cross-match validation: is the PnP degeneracy zone (AR 1.05-1.10) consistent?

Runs the aspect-ratio-banded analysis across ALL Bonney Lake match logs
to check that the E14 finding isn't an outlier.
"""
import sys
import os
import statistics
import glob

sys.path.insert(0, "src")
from logreader.wpilog_reader import read_wpilog
from logreader.analyzers.vision_analysis import (
    _discover_cameras,
    _parse_frames,
    _compute_residuals,
)

LOG_DIR = r"D:\Temp\2026_BonneyLake\2412_matchesonly"
AR_BANDS = [
    ("1.00-1.05", 1.00, 1.05),
    ("1.05-1.10", 1.05, 1.10),
    ("1.10-1.20", 1.10, 1.20),
    ("1.20-1.50", 1.20, 1.50),
    ("1.50-2.00", 1.50, 2.00),
    ("2.00+",     2.00, 100.0),
]

log_files = sorted(glob.glob(os.path.join(LOG_DIR, "*.wpilog")))
print(f"Found {len(log_files)} match logs\n")

# Accumulate per-band stats across all matches
all_match_results = []

for log_path in log_files:
    match_name = os.path.basename(log_path).replace("FRC_", "").replace(".wpilog", "")
    # Extract short name (e.g. "Q15", "E14")
    parts = match_name.split("_")
    short = parts[-1] if len(parts) >= 2 else match_name

    print(f"Processing {short}...", end=" ", flush=True)
    try:
        log = read_wpilog(log_path)
    except Exception as e:
        print(f"SKIP ({e})")
        continue

    cameras = _discover_cameras(log)
    if not cameras:
        print("no cameras")
        continue

    all_frames = []
    for cam, sigs in cameras.items():
        frames, _ = _parse_frames(cam, sigs)
        all_frames.extend(frames)
    _compute_residuals(all_frames, log)

    # Filter to single-tag frames with t2d data and residual
    single = [
        f for f in all_frames
        if f.tag_count == 1
        and f.tag_aspect_ratio > 0
        and f.pose_residual_m is not None
    ]

    match_data = {"name": short, "total_single": len(single), "bands": {}}

    for label, lo, hi in AR_BANDS:
        band = [f for f in single if lo <= f.tag_aspect_ratio < hi]
        if not band:
            match_data["bands"][label] = None
            continue

        residuals = [f.pose_residual_m for f in band]
        ambiguities = [f.max_ambiguity for f in band if f.max_ambiguity > 0]
        outliers = sum(1 for r in residuals if r > 1.0)

        match_data["bands"][label] = {
            "n": len(band),
            "mean_res": statistics.mean(residuals),
            "med_res": statistics.median(residuals),
            "mean_amb": statistics.mean(ambiguities) if ambiguities else 0,
            "outlier_pct": outliers / len(band) * 100,
        }

    all_match_results.append(match_data)
    print(f"{len(single)} single-tag frames")

# === Summary table ===
print("\n" + "=" * 100)
print("SINGLE-TAG FRAMES: OUTLIER RATE (>1m) BY ASPECT RATIO BAND")
print("=" * 100)

header = f"{'Match':>8s}  {'Total':>5s}"
for label, _, _ in AR_BANDS:
    header += f"  {label:>10s}"
print(header)
print("-" * len(header))

for mr in all_match_results:
    row = f"{mr['name']:>8s}  {mr['total_single']:5d}"
    for label, _, _ in AR_BANDS:
        bd = mr["bands"].get(label)
        if bd is None:
            row += f"  {'---':>10s}"
        else:
            row += f"  {bd['outlier_pct']:5.1f}%({bd['n']:>3d})"
    print(row)

# === Aggregate across all matches ===
print("\n" + "=" * 100)
print("AGGREGATE ACROSS ALL MATCHES")
print("=" * 100)

header2 = f"{'Band':>10s}  {'Frames':>6s}  {'MnRes':>7s}  {'MdRes':>7s}  {'MnAmb':>7s}  {'Out%':>6s}"
print(header2)
print("-" * len(header2))

for label, lo, hi in AR_BANDS:
    agg_res = []
    agg_amb = []
    agg_n = 0
    agg_out = 0
    for mr in all_match_results:
        bd = mr["bands"].get(label)
        if bd is None:
            continue
        agg_n += bd["n"]
        # Reconstruct from individual matches isn't perfect for mean,
        # but outlier_pct * n gives outlier count
        agg_out += int(bd["outlier_pct"] * bd["n"] / 100)
        agg_res.append((bd["mean_res"], bd["n"]))
        agg_amb.append((bd["mean_amb"], bd["n"]))

    if agg_n == 0:
        continue

    # Weighted mean
    wmean_res = sum(r * n for r, n in agg_res) / agg_n
    wmean_amb = sum(a * n for a, n in agg_amb) / agg_n
    out_pct = agg_out / agg_n * 100

    print(f"{label:>10s}  {agg_n:6d}  {wmean_res:7.3f}  {'':>7s}  {wmean_amb:7.3f}  {out_pct:5.1f}%")

# === Distance-controlled check (2-3m range) ===
print("\n" + "=" * 100)
print("DISTANCE-CONTROLLED (2-3m): SINGLE-TAG OUTLIER RATE BY AR BAND, PER MATCH")
print("=" * 100)

header3 = f"{'Match':>8s}  {'Total':>5s}"
for label, _, _ in AR_BANDS[:4]:  # Only show the interesting bands
    header3 += f"  {label:>10s}"
print(header3)
print("-" * len(header3))

for log_path in log_files:
    match_name = os.path.basename(log_path).replace("FRC_", "").replace(".wpilog", "")
    parts = match_name.split("_")
    short = parts[-1] if len(parts) >= 2 else match_name

    try:
        log = read_wpilog(log_path)
    except Exception:
        continue

    cameras = _discover_cameras(log)
    if not cameras:
        continue

    all_frames = []
    for cam, sigs in cameras.items():
        frames, _ = _parse_frames(cam, sigs)
        all_frames.extend(frames)
    _compute_residuals(all_frames, log)

    # Single-tag, 2-3m distance
    controlled = [
        f for f in all_frames
        if f.tag_count == 1
        and f.tag_aspect_ratio > 0
        and f.pose_residual_m is not None
        and 2.0 <= f.avg_tag_dist < 3.0
    ]

    row = f"{short:>8s}  {len(controlled):5d}"
    for label, lo, hi in AR_BANDS[:4]:
        band = [f for f in controlled if lo <= f.tag_aspect_ratio < hi]
        if len(band) < 3:
            row += f"  {'---':>10s}"
        else:
            outliers = sum(1 for f in band if f.pose_residual_m > 1.0)
            row += f"  {outliers/len(band)*100:5.1f}%({len(band):>3d})"
    print(row)
