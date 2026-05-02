"""WASAM hub vs DS deep-dive: controlling for confounding variables.

Investigates whether the hub vs DS accuracy difference is explained by
tag position itself, or by confounding factors like distance, aspect
ratio, ambiguity, tag count, robot speed, etc.
"""

from __future__ import annotations

import math
import statistics
import sys
import re
from pathlib import Path
from collections import defaultdict

if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from logreader.wpilog_reader import read_wpilog
from logreader.models import LogData
from logreader.analyzers.vision_analysis import (
    VisionFrame,
    TagDetection,
    _discover_cameras,
    _parse_frames,
    _compute_residuals,
    DEFAULT_OUTLIER_M,
)
from logreader.analyzers.pose_analysis import (
    build_reference_path,
    compute_velocity_at,
)

# Tag groups
RED_HUB = {2, 3, 4, 5, 8, 9, 10, 11}
BLUE_HUB = {18, 19, 20, 21, 24, 25, 26, 27}
RED_DS = {13, 14, 15, 16}
BLUE_DS = {29, 30, 31, 32}
ALL_HUB = RED_HUB | BLUE_HUB
ALL_DS = RED_DS | BLUE_DS
ALL_OTHER = {1, 6, 7, 12, 17, 22, 23, 28}

LOG_DIR = Path(r"D:\Temp\2026_WASAM")


def tag_group(tag_id: int) -> str:
    if tag_id in ALL_HUB:
        return "hub"
    elif tag_id in ALL_DS:
        return "ds"
    elif tag_id in ALL_OTHER:
        return "other"
    return "unknown"


def stats_bucket(vals: list[float]) -> dict:
    if not vals:
        return {"n": 0}
    s = sorted(vals)
    return {
        "n": len(s),
        "mean": statistics.mean(s),
        "med": statistics.median(s),
        "p95": s[int(len(s) * 0.95)] if len(s) > 1 else s[0],
        "out_pct": sum(1 for r in s if r > 1.0) / len(s) * 100,
    }


def fmt(d: dict, short: bool = False) -> str:
    if d["n"] == 0:
        return "no data"
    if short:
        return f"n={d['n']:<6} med={d['med']:.3f}m  out%={d['out_pct']:.1f}%"
    return (f"n={d['n']}, med={d['med']:.3f}m, mean={d['mean']:.3f}m, "
            f"p95={d['p95']:.3f}m, out%={d['out_pct']:.1f}%")


def main():
    wpilog_files = sorted(LOG_DIR.glob("FRC_*_WASAM_Q*.wpilog"))
    print(f"Found {len(wpilog_files)} match files\n")

    # Collect per-frame data with all confounding variables
    all_records = []  # list of dicts per valid vision frame

    for filepath in wpilog_files:
        m = re.search(r"WASAM_(Q\d+)", filepath.name)
        if not m:
            continue
        match_label = m.group(1)
        print(f"  Loading {match_label}...")

        log_data = read_wpilog(str(filepath))
        all_cameras = _discover_cameras(log_data)

        all_frames: list[VisionFrame] = []
        for cam_name, signals in all_cameras.items():
            frames, _ = _parse_frames(cam_name, signals)
            all_frames.extend(frames)
        _compute_residuals(all_frames, log_data)

        # Build reference path for velocity computation
        ref_path = build_reference_path(log_data)

        for f in all_frames:
            if f.pose_residual_m is None or not f.tags:
                continue

            # Classify frame by which group it's primarily seeing
            hub_tags = [t for t in f.tags if t.tag_id in ALL_HUB]
            ds_tags = [t for t in f.tags if t.tag_id in ALL_DS]
            other_tags = [t for t in f.tags if t.tag_id in ALL_OTHER]

            if hub_tags and not ds_tags:
                group = "hub"
                primary_tags = hub_tags
            elif ds_tags and not hub_tags:
                group = "ds"
                primary_tags = ds_tags
            elif hub_tags and ds_tags:
                group = "mixed"
                primary_tags = hub_tags + ds_tags
            else:
                group = "other"
                primary_tags = other_tags

            if group in ("mixed", "other"):
                continue  # Focus on pure hub vs pure DS

            # Per-tag stats for this frame
            dists = [t.dist_to_camera for t in primary_tags]
            ambs = [t.ambiguity for t in primary_tags]
            areas = [t.area for t in primary_tags]

            mean_dist = statistics.mean(dists)
            min_dist = min(dists)
            max_amb = max(ambs)
            mean_amb = statistics.mean(ambs)
            mean_area = statistics.mean(areas)

            # Robot speed
            speed = 0.0
            if ref_path:
                vx, vy, _ = compute_velocity_at(ref_path, f.timestamp_us)
                speed = math.hypot(vx, vy)
                # Clamp implausible speeds
                if speed > 8.0:
                    speed = 0.0

            all_records.append({
                "match": match_label,
                "camera": f.camera,
                "group": group,
                "residual": f.pose_residual_m,
                "tag_count": f.tag_count,
                "mean_dist": mean_dist,
                "min_dist": min_dist,
                "max_amb": max_amb,
                "mean_amb": mean_amb,
                "mean_area": mean_area,
                "aspect_ratio": f.tag_aspect_ratio,
                "long_side_px": f.tag_long_side_px,
                "short_side_px": f.tag_short_side_px,
                "skew_deg": f.tag_skew_deg,
                "h_extent_px": f.tag_h_extent_px,
                "v_extent_px": f.tag_v_extent_px,
                "latency_ms": f.total_latency_ms,
                "stddev_mt1_x": f.stddev_mt1_x,
                "stddev_mt1_y": f.stddev_mt1_y,
                "speed_mps": speed,
                "robot_x": f.x,
                "robot_y": f.y,
            })

    hub = [r for r in all_records if r["group"] == "hub"]
    ds = [r for r in all_records if r["group"] == "ds"]

    print(f"\nTotal records: {len(all_records)} (hub={len(hub)}, ds={len(ds)})")

    # =========================================================================
    print(f"\n{'='*90}")
    print("1. RAW COMPARISON (uncontrolled)")
    print(f"{'='*90}")
    print(f"  Hub: {fmt(stats_bucket([r['residual'] for r in hub]))}")
    print(f"  DS:  {fmt(stats_bucket([r['residual'] for r in ds]))}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("2. CONFOUNDING VARIABLE DISTRIBUTIONS")
    print("   Are hub and DS frames inherently different?")
    print(f"{'='*90}")

    for var, label in [
        ("mean_dist", "Mean tag distance (m)"),
        ("max_amb", "Max ambiguity"),
        ("mean_area", "Mean tag area"),
        ("tag_count", "Tag count"),
        ("aspect_ratio", "Aspect ratio (t2d)"),
        ("long_side_px", "Tag long side (px)"),
        ("speed_mps", "Robot speed (m/s)"),
        ("latency_ms", "Total latency (ms)"),
        ("stddev_mt1_x", "StdDev MT1 X"),
    ]:
        hub_vals = [r[var] for r in hub if r[var] > 0]
        ds_vals = [r[var] for r in ds if r[var] > 0]
        if hub_vals and ds_vals:
            print(f"\n  {label}:")
            print(f"    Hub: mean={statistics.mean(hub_vals):.3f}, "
                  f"med={statistics.median(hub_vals):.3f}, "
                  f"p10={sorted(hub_vals)[len(hub_vals)//10]:.3f}, "
                  f"p90={sorted(hub_vals)[int(len(hub_vals)*0.9)]:.3f}")
            print(f"    DS:  mean={statistics.mean(ds_vals):.3f}, "
                  f"med={statistics.median(ds_vals):.3f}, "
                  f"p10={sorted(ds_vals)[len(ds_vals)//10]:.3f}, "
                  f"p90={sorted(ds_vals)[int(len(ds_vals)*0.9)]:.3f}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("3. CONTROLLING FOR DISTANCE")
    print(f"{'='*90}")

    dist_bands = [
        ("0-1.5m", 0.0, 1.5), ("1.5-2m", 1.5, 2.0), ("2-2.5m", 2.0, 2.5),
        ("2.5-3m", 2.5, 3.0), ("3-3.5m", 3.0, 3.5), ("3.5-4m", 3.5, 4.0),
        ("4-5m", 4.0, 5.0), ("5m+", 5.0, float("inf")),
    ]

    print(f"{'Band':<10} {'HubN':<7} {'HubMed':<9} {'HubOut%':<9} {'HubAmb':<9} "
          f"{'DSN':<7} {'DSMed':<9} {'DSOut%':<9} {'DSAmb':<9} {'Better'}")

    for label, lo, hi in dist_bands:
        h = [r["residual"] for r in hub if lo <= r["mean_dist"] < hi]
        d = [r["residual"] for r in ds if lo <= r["mean_dist"] < hi]
        h_amb = [r["mean_amb"] for r in hub if lo <= r["mean_dist"] < hi]
        d_amb = [r["mean_amb"] for r in ds if lo <= r["mean_dist"] < hi]
        hs = stats_bucket(h)
        ds_s = stats_bucket(d)
        h_a = statistics.mean(h_amb) if h_amb else 0
        d_a = statistics.mean(d_amb) if d_amb else 0
        h_m = hs.get("med")
        d_m = ds_s.get("med")
        better = "HUB" if h_m and d_m and h_m < d_m else "DS" if h_m and d_m and d_m < h_m else "-"
        print(f"{label:<10} {hs.get('n',0):<7} "
              f"{h_m if h_m else '-':>8} {hs.get('out_pct',0):>8.1f}  {h_a:>8.3f}  "
              f"{ds_s.get('n',0):<7} "
              f"{d_m if d_m else '-':>8} {ds_s.get('out_pct',0):>8.1f}  {d_a:>8.3f}  "
              f"{better}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("4. CONTROLLING FOR TAG COUNT")
    print(f"{'='*90}")

    print(f"{'TC':<5} {'HubN':<7} {'HubMed':<9} {'HubOut%':<9} {'HubDist':<9} "
          f"{'DSN':<7} {'DSMed':<9} {'DSOut%':<9} {'DSDist':<9} {'Better'}")

    for tc in [1, 2, 3, 4]:
        h = [r for r in hub if r["tag_count"] == tc]
        d = [r for r in ds if r["tag_count"] == tc]
        hr = [r["residual"] for r in h]
        dr = [r["residual"] for r in d]
        hd = statistics.mean([r["mean_dist"] for r in h]) if h else 0
        dd = statistics.mean([r["mean_dist"] for r in d]) if d else 0
        hs = stats_bucket(hr)
        ds_s = stats_bucket(dr)
        h_m = hs.get("med")
        d_m = ds_s.get("med")
        better = "HUB" if h_m and d_m and h_m < d_m else "DS" if h_m and d_m else "-"
        print(f"{tc:<5} {hs.get('n',0):<7} "
              f"{h_m if h_m else '-':>8} {hs.get('out_pct',0):>8.1f}  {hd:>8.1f}  "
              f"{ds_s.get('n',0):<7} "
              f"{d_m if d_m else '-':>8} {ds_s.get('out_pct',0):>8.1f}  {dd:>8.1f}  "
              f"{better}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("5. CONTROLLING FOR AMBIGUITY")
    print(f"{'='*90}")

    amb_bands = [
        ("0-0.1", 0.0, 0.1), ("0.1-0.2", 0.1, 0.2), ("0.2-0.3", 0.2, 0.3),
        ("0.3-0.4", 0.3, 0.4), ("0.4-0.5", 0.4, 0.5), ("0.5-0.6", 0.5, 0.6),
        ("0.6-0.7", 0.6, 0.7), ("0.7-0.8", 0.7, 0.8), ("0.8+", 0.8, 1.0),
    ]

    print(f"{'Amb':<10} {'HubN':<7} {'HubMed':<9} {'HubOut%':<9} {'HubDist':<9} "
          f"{'DSN':<7} {'DSMed':<9} {'DSOut%':<9} {'DSDist':<9} {'Better'}")

    for label, lo, hi in amb_bands:
        h = [r for r in hub if lo <= r["max_amb"] < hi]
        d = [r for r in ds if lo <= r["max_amb"] < hi]
        hr = [r["residual"] for r in h]
        dr = [r["residual"] for r in d]
        hd = statistics.mean([r["mean_dist"] for r in h]) if h else 0
        dd = statistics.mean([r["mean_dist"] for r in d]) if d else 0
        hs = stats_bucket(hr)
        ds_s = stats_bucket(dr)
        h_m = hs.get("med")
        d_m = ds_s.get("med")
        better = "HUB" if h_m and d_m and h_m < d_m else "DS" if h_m and d_m else "-"
        print(f"{label:<10} {hs.get('n',0):<7} "
              f"{h_m if h_m else '-':>8} {hs.get('out_pct',0):>8.1f}  {hd:>8.1f}  "
              f"{ds_s.get('n',0):<7} "
              f"{d_m if d_m else '-':>8} {ds_s.get('out_pct',0):>8.1f}  {dd:>8.1f}  "
              f"{better}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("6. CONTROLLING FOR ASPECT RATIO")
    print(f"{'='*90}")

    ar_bands = [
        ("1.0-1.1", 1.0, 1.1), ("1.1-1.2", 1.1, 1.2), ("1.2-1.5", 1.2, 1.5),
        ("1.5-2.0", 1.5, 2.0), ("2.0-3.0", 2.0, 3.0), ("3.0+", 3.0, float("inf")),
    ]

    print(f"{'AR':<10} {'HubN':<7} {'HubMed':<9} {'HubOut%':<9} {'HubDist':<9} "
          f"{'DSN':<7} {'DSMed':<9} {'DSOut%':<9} {'DSDist':<9} {'Better'}")

    for label, lo, hi in ar_bands:
        h = [r for r in hub if r["aspect_ratio"] > 0 and lo <= r["aspect_ratio"] < hi]
        d = [r for r in ds if r["aspect_ratio"] > 0 and lo <= r["aspect_ratio"] < hi]
        hr = [r["residual"] for r in h]
        dr = [r["residual"] for r in d]
        hd = statistics.mean([r["mean_dist"] for r in h]) if h else 0
        dd = statistics.mean([r["mean_dist"] for r in d]) if d else 0
        hs = stats_bucket(hr)
        ds_s = stats_bucket(dr)
        h_m = hs.get("med")
        d_m = ds_s.get("med")
        better = "HUB" if h_m and d_m and h_m < d_m else "DS" if h_m and d_m else "-"
        print(f"{label:<10} {hs.get('n',0):<7} "
              f"{h_m if h_m else '-':>8} {hs.get('out_pct',0):>8.1f}  {hd:>8.1f}  "
              f"{ds_s.get('n',0):<7} "
              f"{d_m if d_m else '-':>8} {ds_s.get('out_pct',0):>8.1f}  {dd:>8.1f}  "
              f"{better}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("7. CONTROLLING FOR TAG SIZE (long side pixels)")
    print(f"{'='*90}")

    px_bands = [
        ("0-20px", 0, 20), ("20-40px", 20, 40), ("40-60px", 40, 60),
        ("60-100px", 60, 100), ("100-150px", 100, 150), ("150px+", 150, 10000),
    ]

    print(f"{'Size':<10} {'HubN':<7} {'HubMed':<9} {'HubOut%':<9} {'HubDist':<9} "
          f"{'DSN':<7} {'DSMed':<9} {'DSOut%':<9} {'DSDist':<9} {'Better'}")

    for label, lo, hi in px_bands:
        h = [r for r in hub if r["long_side_px"] > 0 and lo <= r["long_side_px"] < hi]
        d = [r for r in ds if r["long_side_px"] > 0 and lo <= r["long_side_px"] < hi]
        hr = [r["residual"] for r in h]
        dr = [r["residual"] for r in d]
        hd = statistics.mean([r["mean_dist"] for r in h]) if h else 0
        dd = statistics.mean([r["mean_dist"] for r in d]) if d else 0
        hs = stats_bucket(hr)
        ds_s = stats_bucket(dr)
        h_m = hs.get("med")
        d_m = ds_s.get("med")
        better = "HUB" if h_m and d_m and h_m < d_m else "DS" if h_m and d_m else "-"
        print(f"{label:<10} {hs.get('n',0):<7} "
              f"{h_m if h_m else '-':>8} {hs.get('out_pct',0):>8.1f}  {hd:>8.1f}  "
              f"{ds_s.get('n',0):<7} "
              f"{d_m if d_m else '-':>8} {ds_s.get('out_pct',0):>8.1f}  {dd:>8.1f}  "
              f"{better}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("8. CONTROLLING FOR ROBOT SPEED")
    print(f"{'='*90}")

    speed_bands = [
        ("Stopped", 0.0, 0.3), ("Slow", 0.3, 1.0), ("Medium", 1.0, 2.0),
        ("Fast", 2.0, 4.0), ("Sprint", 4.0, float("inf")),
    ]

    print(f"{'Speed':<10} {'HubN':<7} {'HubMed':<9} {'HubOut%':<9} "
          f"{'DSN':<7} {'DSMed':<9} {'DSOut%':<9} {'Better'}")

    for label, lo, hi in speed_bands:
        h = [r["residual"] for r in hub if lo <= r["speed_mps"] < hi]
        d = [r["residual"] for r in ds if lo <= r["speed_mps"] < hi]
        hs = stats_bucket(h)
        ds_s = stats_bucket(d)
        h_m = hs.get("med")
        d_m = ds_s.get("med")
        better = "HUB" if h_m and d_m and h_m < d_m else "DS" if h_m and d_m else "-"
        print(f"{label:<10} {hs.get('n',0):<7} "
              f"{h_m if h_m else '-':>8} {hs.get('out_pct',0):>8.1f}  "
              f"{ds_s.get('n',0):<7} "
              f"{d_m if d_m else '-':>8} {ds_s.get('out_pct',0):>8.1f}  "
              f"{better}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("9. DOUBLE-CONTROLLED: DISTANCE + TAG COUNT")
    print("   (Isolates the hub vs DS effect at matched distance & tag count)")
    print(f"{'='*90}")

    for tc in [1, 2]:
        print(f"\n  Tag count = {tc}:")
        print(f"  {'Band':<10} {'HubN':<7} {'HubMed':<9} {'HubOut%':<9} {'HubAmb':<9} "
              f"{'DSN':<7} {'DSMed':<9} {'DSOut%':<9} {'DSAmb':<9} {'Better'}")
        for label, lo, hi in dist_bands:
            h = [r for r in hub if r["tag_count"] == tc and lo <= r["mean_dist"] < hi]
            d = [r for r in ds if r["tag_count"] == tc and lo <= r["mean_dist"] < hi]
            hr = [r["residual"] for r in h]
            dr = [r["residual"] for r in d]
            h_amb = statistics.mean([r["mean_amb"] for r in h]) if h else 0
            d_amb = statistics.mean([r["mean_amb"] for r in d]) if d else 0
            hs = stats_bucket(hr)
            ds_s = stats_bucket(dr)
            h_m = hs.get("med")
            d_m = ds_s.get("med")
            better = "HUB" if h_m and d_m and h_m < d_m else "DS" if h_m and d_m else "-"
            if hs.get("n", 0) > 0 or ds_s.get("n", 0) > 0:
                print(f"  {label:<10} {hs.get('n',0):<7} "
                      f"{h_m if h_m else '-':>8} {hs.get('out_pct',0):>8.1f}  {h_amb:>8.3f}  "
                      f"{ds_s.get('n',0):<7} "
                      f"{d_m if d_m else '-':>8} {ds_s.get('out_pct',0):>8.1f}  {d_amb:>8.3f}  "
                      f"{better}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("10. DOUBLE-CONTROLLED: DISTANCE + AMBIGUITY")
    print("    (Same distance AND same ambiguity range)")
    print(f"{'='*90}")

    dist_coarse = [("0-2m", 0.0, 2.0), ("2-3m", 2.0, 3.0), ("3-4m", 3.0, 4.0), ("4m+", 4.0, float("inf"))]
    amb_coarse = [("lowAmb(<0.3)", 0.0, 0.3), ("midAmb(0.3-0.5)", 0.3, 0.5), ("hiAmb(>0.5)", 0.5, 1.0)]

    print(f"{'Dist':<8} {'Amb':<16} {'HubN':<7} {'HubMed':<9} {'HubOut%':<9} "
          f"{'DSN':<7} {'DSMed':<9} {'DSOut%':<9} {'Better'}")

    for dlabel, dlo, dhi in dist_coarse:
        for alabel, alo, ahi in amb_coarse:
            h = [r["residual"] for r in hub
                 if dlo <= r["mean_dist"] < dhi and alo <= r["max_amb"] < ahi]
            d = [r["residual"] for r in ds
                 if dlo <= r["mean_dist"] < dhi and alo <= r["max_amb"] < ahi]
            hs = stats_bucket(h)
            ds_s = stats_bucket(d)
            h_m = hs.get("med")
            d_m = ds_s.get("med")
            better = "HUB" if h_m and d_m and h_m < d_m else "DS" if h_m and d_m else "-"
            if hs.get("n", 0) > 10 or ds_s.get("n", 0) > 10:
                print(f"{dlabel:<8} {alabel:<16} {hs.get('n',0):<7} "
                      f"{h_m if h_m else '-':>8} {hs.get('out_pct',0):>8.1f}  "
                      f"{ds_s.get('n',0):<7} "
                      f"{d_m if d_m else '-':>8} {ds_s.get('out_pct',0):>8.1f}  "
                      f"{better}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("11. TRIPLE-CONTROLLED: DISTANCE + TAG COUNT + AMBIGUITY")
    print("    (Maximally controlled comparison)")
    print(f"{'='*90}")

    for tc in [1, 2]:
        print(f"\n  Tag count = {tc}:")
        print(f"  {'Dist':<8} {'Amb':<16} {'HubN':<7} {'HubMed':<9} "
              f"{'DSN':<7} {'DSMed':<9} {'Better'}")
        for dlabel, dlo, dhi in dist_coarse:
            for alabel, alo, ahi in amb_coarse:
                h = [r["residual"] for r in hub
                     if r["tag_count"] == tc and dlo <= r["mean_dist"] < dhi and alo <= r["max_amb"] < ahi]
                d = [r["residual"] for r in ds
                     if r["tag_count"] == tc and dlo <= r["mean_dist"] < dhi and alo <= r["max_amb"] < ahi]
                hs = stats_bucket(h)
                ds_s = stats_bucket(d)
                h_m = hs.get("med")
                d_m = ds_s.get("med")
                better = "HUB" if h_m and d_m and h_m < d_m else "DS" if h_m and d_m else "-"
                if hs.get("n", 0) >= 20 or ds_s.get("n", 0) >= 20:
                    print(f"  {dlabel:<8} {alabel:<16} {hs.get('n',0):<7} "
                          f"{h_m if h_m else '-':>8}  "
                          f"{ds_s.get('n',0):<7} "
                          f"{d_m if d_m else '-':>8}  "
                          f"{better}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("12. PER-CAMERA: HUB vs DS (are cameras pointed differently?)")
    print(f"{'='*90}")

    for cam in sorted({r["camera"] for r in all_records}):
        cam_hub = [r for r in hub if r["camera"] == cam]
        cam_ds = [r for r in ds if r["camera"] == cam]
        print(f"\n  {cam}:")
        print(f"    Hub: {fmt(stats_bucket([r['residual'] for r in cam_hub]))}")
        if cam_hub:
            print(f"         meanDist={statistics.mean([r['mean_dist'] for r in cam_hub]):.2f}m, "
                  f"meanAmb={statistics.mean([r['max_amb'] for r in cam_hub]):.3f}, "
                  f"meanAR={statistics.mean([r['aspect_ratio'] for r in cam_hub if r['aspect_ratio'] > 0]):.2f}, "
                  f"meanTC={statistics.mean([r['tag_count'] for r in cam_hub]):.1f}")
        print(f"    DS:  {fmt(stats_bucket([r['residual'] for r in cam_ds]))}")
        if cam_ds:
            print(f"         meanDist={statistics.mean([r['mean_dist'] for r in cam_ds]):.2f}m, "
                  f"meanAmb={statistics.mean([r['max_amb'] for r in cam_ds]):.3f}, "
                  f"meanAR={statistics.mean([r['aspect_ratio'] for r in cam_ds if r['aspect_ratio'] > 0]):.2f}, "
                  f"meanTC={statistics.mean([r['tag_count'] for r in cam_ds]):.1f}")

        # Distance-controlled per-camera
        print(f"    Distance-controlled:")
        print(f"    {'Band':<10} {'HubMed':<9} {'HubN':<7} {'DSMed':<9} {'DSN':<7} {'Better'}")
        for label, lo, hi in [("0-2m", 0, 2), ("2-3m", 2, 3), ("3-4m", 3, 4), ("4m+", 4, 999)]:
            ch = [r["residual"] for r in cam_hub if lo <= r["mean_dist"] < hi]
            cd = [r["residual"] for r in cam_ds if lo <= r["mean_dist"] < hi]
            chs = stats_bucket(ch)
            cds = stats_bucket(cd)
            h_m = chs.get("med")
            d_m = cds.get("med")
            better = "HUB" if h_m and d_m and h_m < d_m else "DS" if h_m and d_m else "-"
            if chs.get("n", 0) > 0 or cds.get("n", 0) > 0:
                print(f"    {label:<10} {h_m if h_m else '-':>8} {chs.get('n',0):<7} "
                      f"{d_m if d_m else '-':>8} {cds.get('n',0):<7} {better}")

    # =========================================================================
    print(f"\n{'='*90}")
    print("13. STDDEV COMPARISON (Limelight's own confidence)")
    print("    MT1 stddev_x and stddev_y by group")
    print(f"{'='*90}")

    hub_sdx = [r["stddev_mt1_x"] for r in hub if r["stddev_mt1_x"] > 0]
    ds_sdx = [r["stddev_mt1_x"] for r in ds if r["stddev_mt1_x"] > 0]
    hub_sdy = [r["stddev_mt1_y"] for r in hub if r["stddev_mt1_y"] > 0]
    ds_sdy = [r["stddev_mt1_y"] for r in ds if r["stddev_mt1_y"] > 0]

    if hub_sdx and ds_sdx:
        print(f"  StdDev X:")
        print(f"    Hub: mean={statistics.mean(hub_sdx):.4f}, med={statistics.median(hub_sdx):.4f}")
        print(f"    DS:  mean={statistics.mean(ds_sdx):.4f}, med={statistics.median(ds_sdx):.4f}")
    if hub_sdy and ds_sdy:
        print(f"  StdDev Y:")
        print(f"    Hub: mean={statistics.mean(hub_sdy):.4f}, med={statistics.median(hub_sdy):.4f}")
        print(f"    DS:  mean={statistics.mean(ds_sdy):.4f}, med={statistics.median(ds_sdy):.4f}")

    print(f"\n{'='*90}")
    print("ANALYSIS COMPLETE")
    print(f"{'='*90}")


if __name__ == "__main__":
    main()
