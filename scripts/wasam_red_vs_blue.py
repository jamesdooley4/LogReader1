"""WASAM red-vs-blue side analysis.

Investigates whether field-side (red vs blue) affects vision accuracy,
controlling for distance, to detect potential lighting or environmental
differences.
"""

from __future__ import annotations

import math
import statistics
import sys
import re
from pathlib import Path

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
from logreader.analyzers.match_phases import detect_match_phases, MatchPhase, MatchPhaseTimeline

# ---------------------------------------------------------------------------
# Tag classification by field side
# ---------------------------------------------------------------------------

# Red side tags (x > 8.27, the red half of the field)
RED_SIDE_TAGS = set(range(1, 17))   # Tags 1-16
# Blue side tags (x < 8.27, the blue half)
BLUE_SIDE_TAGS = set(range(17, 33))  # Tags 17-32

# Hub tags
RED_HUB = {2, 3, 4, 5, 8, 9, 10, 11}
BLUE_HUB = {18, 19, 20, 21, 24, 25, 26, 27}
RED_DS = {13, 14, 15, 16}
BLUE_DS = {29, 30, 31, 32}
RED_OTHER = {1, 6, 7, 12}
BLUE_OTHER = {17, 22, 23, 28}

# Tag positions
TAG_POS = {
    1: (11.864, 7.411), 2: (11.901, 4.625), 3: (11.298, 4.377),
    4: (11.298, 4.021), 5: (11.901, 3.418), 6: (11.864, 0.631),
    7: (11.939, 0.631), 8: (12.257, 3.418), 9: (12.505, 3.666),
    10: (12.505, 4.021), 11: (12.257, 4.625), 12: (11.939, 7.411),
    13: (16.499, 7.392), 14: (16.499, 6.960), 15: (16.499, 4.312),
    16: (16.499, 3.881), 17: (4.649, 0.631), 18: (4.612, 3.418),
    19: (5.215, 3.666), 20: (5.215, 4.021), 21: (4.612, 4.625),
    22: (4.649, 7.411), 23: (4.574, 7.411), 24: (4.256, 4.625),
    25: (4.008, 4.377), 26: (4.008, 4.021), 27: (4.256, 3.418),
    28: (4.574, 0.631), 29: (0.014, 0.651), 30: (0.014, 1.083),
    31: (0.014, 3.730), 32: (0.014, 4.162),
}

LOG_DIR = Path(r"D:\Temp\2026_WASAM")

DIST_BANDS = [
    ("0-2m", 0.0, 2.0),
    ("2-3m", 2.0, 3.0),
    ("3-4m", 3.0, 4.0),
    ("4-5m", 4.0, 5.0),
    ("5m+",  5.0, float("inf")),
]


def tag_side(tag_id: int) -> str:
    if tag_id in RED_SIDE_TAGS:
        return "red"
    elif tag_id in BLUE_SIDE_TAGS:
        return "blue"
    return "unknown"


def tag_group(tag_id: int) -> str:
    if tag_id in RED_HUB or tag_id in BLUE_HUB:
        return "hub"
    elif tag_id in RED_DS or tag_id in BLUE_DS:
        return "ds"
    else:
        return "other"


def frame_side(frame: VisionFrame) -> str:
    """Which side's tags is this frame primarily seeing?"""
    if not frame.tags:
        return "unknown"
    red = sum(1 for t in frame.tags if t.tag_id in RED_SIDE_TAGS)
    blue = sum(1 for t in frame.tags if t.tag_id in BLUE_SIDE_TAGS)
    if red > 0 and blue == 0:
        return "red"
    elif blue > 0 and red == 0:
        return "blue"
    elif red > 0 and blue > 0:
        return "both"
    return "unknown"


def robot_alliance(frames: list[VisionFrame]) -> str:
    """Determine robot's alliance from which tags it sees most."""
    red = sum(1 for f in frames for t in f.tags if t.tag_id in RED_SIDE_TAGS)
    blue = sum(1 for f in frames for t in f.tags if t.tag_id in BLUE_SIDE_TAGS)
    if red > blue * 1.5:
        return "red"
    elif blue > red * 1.5:
        return "blue"
    return "mixed"


def stats_bucket(residuals: list[float]) -> dict:
    if not residuals:
        return {"n": 0}
    s = sorted(residuals)
    return {
        "n": len(s),
        "mean": statistics.mean(s),
        "med": statistics.median(s),
        "p95": s[int(len(s) * 0.95)] if len(s) > 1 else s[0],
        "out_pct": sum(1 for r in s if r > 1.0) / len(s) * 100,
    }


def fmt_stats(d: dict) -> str:
    if d["n"] == 0:
        return "no data"
    return (f"n={d['n']}, med={d['med']:.3f}m, mean={d['mean']:.3f}m, "
            f"p95={d['p95']:.3f}m, out%={d['out_pct']:.1f}%")


def main():
    wpilog_files = sorted(LOG_DIR.glob("FRC_*_WASAM_Q*.wpilog"))
    print(f"Found {len(wpilog_files)} match files\n")

    # Collect per-tag-detection data across all matches
    # Each entry: (match, camera, tag_id, side, group, dist, frame_residual, ambiguity, robot_x, robot_y)
    all_detections = []
    per_match_data = []

    for filepath in wpilog_files:
        m = re.search(r"WASAM_(Q\d+)", filepath.name)
        if not m:
            continue
        match_label = m.group(1)

        log_data = read_wpilog(str(filepath))
        all_cameras = _discover_cameras(log_data)
        all_frames: list[VisionFrame] = []
        for cam_name, signals in all_cameras.items():
            frames, _ = _parse_frames(cam_name, signals)
            all_frames.extend(frames)
        _compute_residuals(all_frames, log_data)

        alliance = robot_alliance(all_frames)

        # Classify frames by side
        red_frames = [f for f in all_frames if frame_side(f) == "red"]
        blue_frames = [f for f in all_frames if frame_side(f) == "blue"]

        red_res = [f.pose_residual_m for f in red_frames if f.pose_residual_m is not None]
        blue_res = [f.pose_residual_m for f in blue_frames if f.pose_residual_m is not None]

        per_match_data.append({
            "match": match_label,
            "alliance": alliance,
            "red_frames": red_frames,
            "blue_frames": blue_frames,
            "all_frames": all_frames,
            "red_stats": stats_bucket(red_res),
            "blue_stats": stats_bucket(blue_res),
        })

        # Collect individual tag detections with distance
        for f in all_frames:
            if f.pose_residual_m is None:
                continue
            for t in f.tags:
                all_detections.append({
                    "match": match_label,
                    "alliance": alliance,
                    "camera": f.camera,
                    "tag_id": t.tag_id,
                    "side": tag_side(t.tag_id),
                    "group": tag_group(t.tag_id),
                    "dist": t.dist_to_camera,
                    "ambiguity": t.ambiguity,
                    "residual": f.pose_residual_m,
                    "robot_x": f.x,
                    "robot_y": f.y,
                    "tag_count": f.tag_count,
                })

        print(f"{match_label}: alliance={alliance}, "
              f"red_tags={len(red_frames)}, blue_tags={len(blue_frames)}, "
              f"red_med={stats_bucket(red_res).get('med', '-')}, "
              f"blue_med={stats_bucket(blue_res).get('med', '-')}")

    # =========================================================================
    # 1. Per-match red vs blue comparison
    # =========================================================================
    print(f"\n{'='*90}")
    print("1. PER-MATCH: RED SIDE vs BLUE SIDE ACCURACY")
    print(f"{'='*90}")
    print(f"{'Match':<8} {'Alliance':<10} {'RedN':<7} {'RedMed':<9} {'RedOut%':<9} "
          f"{'BlueN':<7} {'BlueMed':<9} {'BlueOut%':<9} {'Better'}")

    for d in per_match_data:
        rs = d["red_stats"]
        bs = d["blue_stats"]
        r_med = rs.get("med")
        b_med = bs.get("med")
        r_out = rs.get("out_pct", 0)
        b_out = bs.get("out_pct", 0)

        if r_med is not None and b_med is not None:
            better = "RED" if r_med < b_med else "BLUE"
        else:
            better = "-"

        print(f"{d['match']:<8} {d['alliance']:<10} "
              f"{rs.get('n', 0):<7} {r_med if r_med else '-':>8} {r_out:>8.1f}  "
              f"{bs.get('n', 0):<7} {b_med if b_med else '-':>8} {b_out:>8.1f}  "
              f"{better}")

    # =========================================================================
    # 2. Same-group comparison controlling for tag type
    # =========================================================================
    print(f"\n{'='*90}")
    print("2. RED vs BLUE BY TAG GROUP (hub, ds, other) - all matches")
    print(f"{'='*90}")

    for grp in ["hub", "ds", "other"]:
        red_dets = [d for d in all_detections if d["side"] == "red" and d["group"] == grp]
        blue_dets = [d for d in all_detections if d["side"] == "blue" and d["group"] == grp]

        red_r = [d["residual"] for d in red_dets]
        blue_r = [d["residual"] for d in blue_dets]

        red_amb = [d["ambiguity"] for d in red_dets]
        blue_amb = [d["ambiguity"] for d in blue_dets]

        red_dist = [d["dist"] for d in red_dets]
        blue_dist = [d["dist"] for d in blue_dets]

        print(f"\n  {grp.upper()} tags:")
        print(f"    Red:  {fmt_stats(stats_bucket(red_r))}")
        print(f"          meanAmb={statistics.mean(red_amb):.3f}, meanDist={statistics.mean(red_dist):.1f}m" if red_amb else "")
        print(f"    Blue: {fmt_stats(stats_bucket(blue_r))}")
        print(f"          meanAmb={statistics.mean(blue_amb):.3f}, meanDist={statistics.mean(blue_dist):.1f}m" if blue_amb else "")

    # =========================================================================
    # 3. Distance-controlled comparison
    # =========================================================================
    print(f"\n{'='*90}")
    print("3. RED vs BLUE CONTROLLING FOR DISTANCE (all tag types)")
    print(f"{'='*90}")
    print(f"{'Band':<8} {'RedN':<8} {'RedMed':<9} {'RedOut%':<9} {'RedAmb':<9} "
          f"{'BlueN':<8} {'BlueMed':<9} {'BlueOut%':<9} {'BlueAmb':<9} {'Better'}")

    for label, lo, hi in DIST_BANDS:
        red_in_band = [d["residual"] for d in all_detections
                       if d["side"] == "red" and lo <= d["dist"] < hi]
        blue_in_band = [d["residual"] for d in all_detections
                        if d["side"] == "blue" and lo <= d["dist"] < hi]
        red_amb_band = [d["ambiguity"] for d in all_detections
                        if d["side"] == "red" and lo <= d["dist"] < hi]
        blue_amb_band = [d["ambiguity"] for d in all_detections
                         if d["side"] == "blue" and lo <= d["dist"] < hi]

        rs = stats_bucket(red_in_band)
        bs = stats_bucket(blue_in_band)
        r_amb = statistics.mean(red_amb_band) if red_amb_band else 0
        b_amb = statistics.mean(blue_amb_band) if blue_amb_band else 0

        r_med = rs.get("med")
        b_med = bs.get("med")
        better = "RED" if r_med and b_med and r_med < b_med else "BLUE" if r_med and b_med else "-"

        print(f"{label:<8} {rs.get('n',0):<8} "
              f"{r_med if r_med else '-':>8} {rs.get('out_pct',0):>8.1f}  {r_amb:>8.3f}  "
              f"{bs.get('n',0):<8} "
              f"{b_med if b_med else '-':>8} {bs.get('out_pct',0):>8.1f}  {b_amb:>8.3f}  "
              f"{better}")

    # =========================================================================
    # 4. Distance-controlled hub-only comparison (removes DS geometry advantage)
    # =========================================================================
    print(f"\n{'='*90}")
    print("4. RED HUB vs BLUE HUB CONTROLLING FOR DISTANCE")
    print(f"{'='*90}")
    print(f"{'Band':<8} {'RedN':<8} {'RedMed':<9} {'RedOut%':<9} {'RedAmb':<9} "
          f"{'BlueN':<8} {'BlueMed':<9} {'BlueOut%':<9} {'BlueAmb':<9} {'Better'}")

    for label, lo, hi in DIST_BANDS:
        red_in_band = [d["residual"] for d in all_detections
                       if d["side"] == "red" and d["group"] == "hub" and lo <= d["dist"] < hi]
        blue_in_band = [d["residual"] for d in all_detections
                        if d["side"] == "blue" and d["group"] == "hub" and lo <= d["dist"] < hi]
        red_amb_band = [d["ambiguity"] for d in all_detections
                        if d["side"] == "red" and d["group"] == "hub" and lo <= d["dist"] < hi]
        blue_amb_band = [d["ambiguity"] for d in all_detections
                         if d["side"] == "blue" and d["group"] == "hub" and lo <= d["dist"] < hi]

        rs = stats_bucket(red_in_band)
        bs = stats_bucket(blue_in_band)
        r_amb = statistics.mean(red_amb_band) if red_amb_band else 0
        b_amb = statistics.mean(blue_amb_band) if blue_amb_band else 0

        r_med = rs.get("med")
        b_med = bs.get("med")
        better = "RED" if r_med and b_med and r_med < b_med else "BLUE" if r_med and b_med else "-"

        print(f"{label:<8} {rs.get('n',0):<8} "
              f"{r_med if r_med else '-':>8} {rs.get('out_pct',0):>8.1f}  {r_amb:>8.3f}  "
              f"{bs.get('n',0):<8} "
              f"{b_med if b_med else '-':>8} {bs.get('out_pct',0):>8.1f}  {b_amb:>8.3f}  "
              f"{better}")

    # =========================================================================
    # 5. Per-match, same-side comparison (when robot is on red vs blue)
    # =========================================================================
    print(f"\n{'='*90}")
    print("5. SAME-SIDE vs CROSS-FIELD ACCURACY")
    print("   (Does the robot do better looking at tags on its own side?)")
    print(f"{'='*90}")
    print(f"{'Match':<8} {'Alliance':<10} {'OwnN':<7} {'OwnMed':<9} {'OwnOut%':<9} "
          f"{'OppN':<7} {'OppMed':<9} {'OppOut%':<9}")

    for d in per_match_data:
        alliance = d["alliance"]
        if alliance == "mixed":
            continue

        own_side = alliance  # "red" or "blue"
        opp_side = "blue" if own_side == "red" else "red"

        own_frames = d["red_frames"] if own_side == "red" else d["blue_frames"]
        opp_frames = d["blue_frames"] if own_side == "red" else d["red_frames"]

        own_res = [f.pose_residual_m for f in own_frames if f.pose_residual_m is not None]
        opp_res = [f.pose_residual_m for f in opp_frames if f.pose_residual_m is not None]

        own_s = stats_bucket(own_res)
        opp_s = stats_bucket(opp_res)

        print(f"{d['match']:<8} {alliance:<10} "
              f"{own_s.get('n',0):<7} {own_s.get('med','-'):>8} {own_s.get('out_pct',0):>8.1f}  "
              f"{opp_s.get('n',0):<7} {opp_s.get('med','-'):>8} {opp_s.get('out_pct',0):>8.1f}")

    # =========================================================================
    # 6. Per-camera red vs blue
    # =========================================================================
    print(f"\n{'='*90}")
    print("6. PER-CAMERA RED vs BLUE (all matches)")
    print(f"{'='*90}")

    cameras = sorted({d["camera"] for d in all_detections})
    for cam in cameras:
        red_r = [d["residual"] for d in all_detections if d["camera"] == cam and d["side"] == "red"]
        blue_r = [d["residual"] for d in all_detections if d["camera"] == cam and d["side"] == "blue"]
        red_amb = [d["ambiguity"] for d in all_detections if d["camera"] == cam and d["side"] == "red"]
        blue_amb = [d["ambiguity"] for d in all_detections if d["camera"] == cam and d["side"] == "blue"]

        print(f"\n  {cam}:")
        print(f"    Red:  {fmt_stats(stats_bucket(red_r))}")
        if red_amb:
            print(f"          meanAmb={statistics.mean(red_amb):.3f}")
        print(f"    Blue: {fmt_stats(stats_bucket(blue_r))}")
        if blue_amb:
            print(f"          meanAmb={statistics.mean(blue_amb):.3f}")

    # =========================================================================
    # 7. Symmetric tag pair comparison
    # =========================================================================
    print(f"\n{'='*90}")
    print("7. SYMMETRIC TAG PAIR COMPARISON")
    print("   Red tag vs its blue mirror, controlling for structure")
    print(f"{'='*90}")

    # Symmetric pairs: (red_id, blue_id) with same structural position
    symmetric_pairs = [
        # Hub face tags (same relative position on each hub)
        (2, 21), (3, 25), (4, 26), (5, 18),
        (8, 27), (9, 19), (10, 20), (11, 24),
        # DS wall
        (13, 29), (14, 30), (15, 31), (16, 32),
        # Field boundary
        (1, 23), (6, 28), (7, 17), (12, 22),
    ]

    print(f"{'RedID':<7} {'BlueID':<8} {'Group':<7} "
          f"{'RedN':<7} {'RedMed':<9} {'RedAmb':<9} {'RedDist':<9} "
          f"{'BlueN':<7} {'BlueMed':<9} {'BlueAmb':<9} {'BlueDist':<9} {'Better'}")

    for red_id, blue_id in symmetric_pairs:
        red_dets = [d for d in all_detections if d["tag_id"] == red_id]
        blue_dets = [d for d in all_detections if d["tag_id"] == blue_id]

        red_r = [d["residual"] for d in red_dets]
        blue_r = [d["residual"] for d in blue_dets]

        rs = stats_bucket(red_r)
        bs = stats_bucket(blue_r)

        r_amb = statistics.mean([d["ambiguity"] for d in red_dets]) if red_dets else 0
        b_amb = statistics.mean([d["ambiguity"] for d in blue_dets]) if blue_dets else 0
        r_dist = statistics.mean([d["dist"] for d in red_dets]) if red_dets else 0
        b_dist = statistics.mean([d["dist"] for d in blue_dets]) if blue_dets else 0

        grp = tag_group(red_id)

        r_med = rs.get("med")
        b_med = bs.get("med")
        if r_med and b_med:
            better = "RED" if r_med < b_med else "BLUE" if b_med < r_med else "TIED"
        else:
            better = "-"

        print(f"{red_id:<7} {blue_id:<8} {grp:<7} "
              f"{rs.get('n',0):<7} {r_med if r_med else '-':>8} {r_amb:>8.3f} {r_dist:>8.1f}  "
              f"{bs.get('n',0):<7} {b_med if b_med else '-':>8} {b_amb:>8.3f} {b_dist:>8.1f}  "
              f"{better}")

    # =========================================================================
    # 8. Red-alliance matches vs blue-alliance matches
    # =========================================================================
    print(f"\n{'='*90}")
    print("8. ALL RED-ALLIANCE MATCHES vs ALL BLUE-ALLIANCE MATCHES")
    print(f"{'='*90}")

    red_alliance_matches = [d for d in per_match_data if d["alliance"] == "red"]
    blue_alliance_matches = [d for d in per_match_data if d["alliance"] == "blue"]

    ra_all_res = []
    for d in red_alliance_matches:
        ra_all_res.extend(f.pose_residual_m for f in d["all_frames"] if f.pose_residual_m is not None)
    ba_all_res = []
    for d in blue_alliance_matches:
        ba_all_res.extend(f.pose_residual_m for f in d["all_frames"] if f.pose_residual_m is not None)

    print(f"\n  Red alliance matches ({len(red_alliance_matches)}): "
          f"{', '.join(d['match'] for d in red_alliance_matches)}")
    print(f"    {fmt_stats(stats_bucket(ra_all_res))}")

    print(f"  Blue alliance matches ({len(blue_alliance_matches)}): "
          f"{', '.join(d['match'] for d in blue_alliance_matches)}")
    print(f"    {fmt_stats(stats_bucket(ba_all_res))}")

    # =========================================================================
    # 9. Robot position analysis - where was the robot when accuracy was good/bad?
    # =========================================================================
    print(f"\n{'='*90}")
    print("9. ACCURACY BY ROBOT POSITION ON FIELD (x-axis bins)")
    print("   x=0 is blue DS wall, x=16.5 is red DS wall")
    print(f"{'='*90}")

    x_bins = [
        ("0-3m (blue DS)",   0,  3),
        ("3-6m (blue hub)",  3,  6),
        ("6-8m (mid-blue)",  6,  8),
        ("8-11m (mid-red)",  8, 11),
        ("11-14m (red hub)", 11, 14),
        ("14-17m (red DS)",  14, 17),
    ]

    all_frames_all = []
    for d in per_match_data:
        all_frames_all.extend(d["all_frames"])

    print(f"{'X-Bin':<22} {'N':<8} {'MedRes':<9} {'MeanRes':<9} {'Out%':<8} {'MeanTC':<8}")
    for label, xlo, xhi in x_bins:
        bin_res = [f.pose_residual_m for f in all_frames_all
                   if f.pose_residual_m is not None and xlo <= f.x < xhi]
        bin_tc = [f.tag_count for f in all_frames_all
                  if f.pose_residual_m is not None and xlo <= f.x < xhi]
        s = stats_bucket(bin_res)
        mean_tc = statistics.mean(bin_tc) if bin_tc else 0
        if s["n"] > 0:
            print(f"{label:<22} {s['n']:<8} {s['med']:<9.3f} {s['mean']:<9.3f} "
                  f"{s['out_pct']:<8.1f} {mean_tc:<8.1f}")
        else:
            print(f"{label:<22} 0")


if __name__ == "__main__":
    main()
