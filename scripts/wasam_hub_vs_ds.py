"""WASAM hub vs driver-station wall accuracy comparison.

Compares vision accuracy when the Limelight is seeing hub tags vs
driver station wall tags, per match and overall.
"""

from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from logreader.wpilog_reader import read_wpilog
from logreader.models import LogData
from logreader.analyzers.vision_analysis import (
    VisionFrame,
    _discover_cameras,
    _parse_frames,
    _compute_residuals,
    DEFAULT_OUTLIER_M,
)
from logreader.analyzers.match_phases import detect_match_phases, MatchPhase, MatchPhaseTimeline

# ---------------------------------------------------------------------------
# Tag group definitions
# ---------------------------------------------------------------------------

RED_HUB_TAGS = {2, 3, 4, 5, 8, 9, 10, 11}
RED_DS_TAGS = {13, 14, 15, 16}
BLUE_HUB_TAGS = {18, 19, 20, 21, 24, 25, 26, 27}
BLUE_DS_TAGS = {29, 30, 31, 32}

# Combine both alliances
ALL_HUB_TAGS = RED_HUB_TAGS | BLUE_HUB_TAGS
ALL_DS_TAGS = RED_DS_TAGS | BLUE_DS_TAGS

# Tag positions (x, y) from the field layout JSON
TAG_POSITIONS = {
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

# Hub centers (approximate geometric center of hub tags)
RED_HUB_CENTER = (
    statistics.mean(TAG_POSITIONS[t][0] for t in RED_HUB_TAGS),
    statistics.mean(TAG_POSITIONS[t][1] for t in RED_HUB_TAGS),
)
BLUE_HUB_CENTER = (
    statistics.mean(TAG_POSITIONS[t][0] for t in BLUE_HUB_TAGS),
    statistics.mean(TAG_POSITIONS[t][1] for t in BLUE_HUB_TAGS),
)
RED_DS_CENTER = (
    statistics.mean(TAG_POSITIONS[t][0] for t in RED_DS_TAGS),
    statistics.mean(TAG_POSITIONS[t][1] for t in RED_DS_TAGS),
)
BLUE_DS_CENTER = (
    statistics.mean(TAG_POSITIONS[t][0] for t in BLUE_DS_TAGS),
    statistics.mean(TAG_POSITIONS[t][1] for t in BLUE_DS_TAGS),
)


def classify_frame(frame: VisionFrame) -> str:
    """Classify a frame as 'hub', 'ds', 'mixed', or 'other' based on visible tags."""
    if not frame.tags:
        return "unknown"

    tag_ids = {t.tag_id for t in frame.tags}

    hub_tags = tag_ids & ALL_HUB_TAGS
    ds_tags = tag_ids & ALL_DS_TAGS
    other_tags = tag_ids - ALL_HUB_TAGS - ALL_DS_TAGS

    if hub_tags and not ds_tags and not other_tags:
        return "hub"
    elif ds_tags and not hub_tags and not other_tags:
        return "ds"
    elif hub_tags and ds_tags:
        return "mixed"
    elif other_tags and not hub_tags and not ds_tags:
        return "other"
    else:
        return "mixed"


def classify_frame_alliance(frame: VisionFrame) -> str:
    """Classify which alliance's tags the frame is seeing."""
    if not frame.tags:
        return "unknown"
    tag_ids = {t.tag_id for t in frame.tags}
    red = tag_ids & (RED_HUB_TAGS | RED_DS_TAGS | {1, 6, 7, 12})
    blue = tag_ids & (BLUE_HUB_TAGS | BLUE_DS_TAGS | {17, 22, 23, 28})
    if red and not blue:
        return "red"
    elif blue and not red:
        return "blue"
    elif red and blue:
        return "both"
    return "unknown"


LOG_DIR = Path(r"D:\Temp\2026_WASAM")

import re

def extract_match_label(filename: str) -> str:
    m = re.search(r"WASAM_(Q\d+)", filename)
    return m.group(1) if m else None


def analyze_match(filepath: Path):
    """Analyze one match, returning hub vs DS stats."""
    filename = filepath.name
    match_label = extract_match_label(filename)
    if not match_label:
        return None

    log_data = read_wpilog(str(filepath))

    # Phase detection
    timeline = detect_match_phases(log_data)
    if timeline is None:
        timeline = MatchPhaseTimeline()

    # Discover and parse vision frames
    all_cameras = _discover_cameras(log_data)
    camera_names = list(all_cameras.keys())

    all_frames: list[VisionFrame] = []
    for cam_name, signals in all_cameras.items():
        frames, _ = _parse_frames(cam_name, signals)
        all_frames.extend(frames)

    _compute_residuals(all_frames, log_data)

    # Classify each frame
    results = {
        "match": match_label,
        "cameras": camera_names,
        "timeline": timeline,
        "hub_frames": [],
        "ds_frames": [],
        "other_frames": [],
        "mixed_frames": [],
        "all_frames": all_frames,
    }

    for f in all_frames:
        cat = classify_frame(f)
        if cat == "hub":
            results["hub_frames"].append(f)
        elif cat == "ds":
            results["ds_frames"].append(f)
        elif cat == "other":
            results["other_frames"].append(f)
        elif cat == "mixed":
            results["mixed_frames"].append(f)

    return results


def stats_for_frames(frames: list[VisionFrame], label: str = "") -> dict:
    """Compute summary stats for a list of frames."""
    if not frames:
        return {"label": label, "count": 0}

    residuals = [f.pose_residual_m for f in frames if f.pose_residual_m is not None]
    ambiguities = [f.max_ambiguity for f in frames if f.tags]
    tag_counts = [f.tag_count for f in frames]
    distances = [f.avg_tag_dist for f in frames]
    latencies = [f.total_latency_ms for f in frames]

    result = {
        "label": label,
        "count": len(frames),
        "with_residual": len(residuals),
    }

    if residuals:
        result["mean_res"] = statistics.mean(residuals)
        result["med_res"] = statistics.median(residuals)
        sorted_r = sorted(residuals)
        result["p95_res"] = sorted_r[int(len(sorted_r) * 0.95)] if len(sorted_r) > 1 else sorted_r[0]
        result["max_res"] = max(residuals)
        result["outlier_pct"] = sum(1 for r in residuals if r > DEFAULT_OUTLIER_M) / len(residuals) * 100
    else:
        result["mean_res"] = None
        result["med_res"] = None
        result["outlier_pct"] = None

    if ambiguities:
        result["mean_amb"] = statistics.mean(ambiguities)
        result["med_amb"] = statistics.median(ambiguities)
        result["high_amb_pct"] = sum(1 for a in ambiguities if a > 0.5) / len(ambiguities) * 100

    if tag_counts:
        result["mean_tc"] = statistics.mean(tag_counts)

    if distances:
        result["mean_dist"] = statistics.mean(distances)

    if latencies:
        result["mean_lat"] = statistics.mean(latencies)

    return result


def print_stats_line(s: dict, indent: str = "    "):
    if s["count"] == 0:
        print(f"{indent}{s['label']}: no frames")
        return
    if s.get("med_res") is None:
        print(f"{indent}{s['label']}: {s['count']} frames (no residual data)")
        return
    print(f"{indent}{s['label']}: {s['count']} frames, "
          f"medRes={s['med_res']:.3f}m, meanRes={s['mean_res']:.3f}m, "
          f"p95={s.get('p95_res', 0):.3f}m, "
          f"out%={s.get('outlier_pct', 0):.1f}%, "
          f"meanTC={s.get('mean_tc', 0):.1f}, "
          f"meanAmb={s.get('mean_amb', 0):.3f}, "
          f"meanDist={s.get('mean_dist', 0):.1f}m")


def main():
    wpilog_files = sorted(LOG_DIR.glob("FRC_*_WASAM_Q*.wpilog"))
    print(f"Found {len(wpilog_files)} match files\n")

    all_hub = []
    all_ds = []
    all_other = []
    all_mixed = []

    per_match_results = []

    for filepath in wpilog_files:
        result = analyze_match(filepath)
        if result is None:
            continue

        per_match_results.append(result)
        all_hub.extend(result["hub_frames"])
        all_ds.extend(result["ds_frames"])
        all_other.extend(result["other_frames"])
        all_mixed.extend(result["mixed_frames"])

        match = result["match"]
        timeline = result["timeline"]

        print(f"{'='*80}")
        print(f"Match: {match}")
        print(f"{'='*80}")

        # Determine alliance from tag distribution
        alliance_counts = {"red": 0, "blue": 0}
        for f in result["all_frames"]:
            a = classify_frame_alliance(f)
            if a in alliance_counts:
                alliance_counts[a] += 1
        primary_alliance = max(alliance_counts, key=alliance_counts.get) if any(alliance_counts.values()) else "unknown"
        print(f"  Primary alliance tags seen: {primary_alliance} "
              f"(red={alliance_counts['red']}, blue={alliance_counts['blue']})")

        # Overall breakdown
        n_hub = len(result["hub_frames"])
        n_ds = len(result["ds_frames"])
        n_other = len(result["other_frames"])
        n_mixed = len(result["mixed_frames"])
        n_total = n_hub + n_ds + n_other + n_mixed
        print(f"  Frame breakdown: hub={n_hub}, ds={n_ds}, other={n_other}, mixed={n_mixed} (total={n_total})")

        # Overall hub vs DS stats
        hub_stats = stats_for_frames(result["hub_frames"], "Hub tags")
        ds_stats = stats_for_frames(result["ds_frames"], "DS wall tags")
        other_stats = stats_for_frames(result["other_frames"], "Other tags")

        print(f"\n  Overall:")
        print_stats_line(hub_stats)
        print_stats_line(ds_stats)
        print_stats_line(other_stats)

        # Per-camera breakdown
        for cam in result["cameras"]:
            print(f"\n  {cam}:")
            cam_hub = [f for f in result["hub_frames"] if f.camera == cam]
            cam_ds = [f for f in result["ds_frames"] if f.camera == cam]
            cam_other = [f for f in result["other_frames"] if f.camera == cam]
            print_stats_line(stats_for_frames(cam_hub, "Hub"), "      ")
            print_stats_line(stats_for_frames(cam_ds, "DS"), "      ")
            print_stats_line(stats_for_frames(cam_other, "Other"), "      ")

        # Per-phase breakdown (hub vs DS in auto vs teleop)
        if timeline.has_match:
            for phase, phase_name in [(MatchPhase.AUTONOMOUS, "AUTO"), (MatchPhase.TELEOP, "TELEOP")]:
                intervals = timeline.intervals_for(phase)
                if not intervals:
                    continue
                phase_hub = [f for f in result["hub_frames"] if any(iv.contains_us(f.timestamp_us) for iv in intervals)]
                phase_ds = [f for f in result["ds_frames"] if any(iv.contains_us(f.timestamp_us) for iv in intervals)]
                print(f"\n  {phase_name}:")
                print_stats_line(stats_for_frames(phase_hub, f"Hub ({phase_name})"), "      ")
                print_stats_line(stats_for_frames(phase_ds, f"DS ({phase_name})"), "      ")

        # Per-tag-ID residual detail for this match
        tag_data: dict[int, list[float]] = {}
        for f in result["all_frames"]:
            if f.pose_residual_m is None or not f.tags:
                continue
            for t in f.tags:
                tag_data.setdefault(t.tag_id, []).append(f.pose_residual_m)

        if tag_data:
            print(f"\n  Per-tag residuals:")
            print(f"    {'ID':<5} {'Group':<8} {'N':<8} {'MedRes':<10} {'MeanRes':<10} {'Out%':<8} {'MeanDist'}")
            for tid in sorted(tag_data):
                residuals = tag_data[tid]
                n = len(residuals)
                med = statistics.median(residuals)
                mean = statistics.mean(residuals)
                out_pct = sum(1 for r in residuals if r > 1.0) / n * 100

                # Determine group
                if tid in ALL_HUB_TAGS:
                    group = "hub"
                elif tid in ALL_DS_TAGS:
                    group = "ds"
                else:
                    group = "other"

                # Get mean distance for this tag
                tag_dists = []
                for f in result["all_frames"]:
                    for t in f.tags:
                        if t.tag_id == tid:
                            tag_dists.append(t.dist_to_camera)
                mean_dist = statistics.mean(tag_dists) if tag_dists else 0

                print(f"    {tid:<5} {group:<8} {n:<8} {med:<10.3f} {mean:<10.3f} {out_pct:<8.1f} {mean_dist:.1f}m")

    # =========================================================================
    # Cross-match summary
    # =========================================================================
    print(f"\n{'='*80}")
    print(f"CROSS-MATCH SUMMARY: HUB vs DRIVER STATION WALL")
    print(f"{'='*80}")

    print(f"\n--- All matches combined ---")
    print_stats_line(stats_for_frames(all_hub, "Hub tags (all matches)"))
    print_stats_line(stats_for_frames(all_ds, "DS wall tags (all matches)"))
    print_stats_line(stats_for_frames(all_other, "Other tags (all matches)"))
    print_stats_line(stats_for_frames(all_mixed, "Mixed (hub+DS together)"))

    # Per-camera combined
    all_frames_combined = all_hub + all_ds + all_other + all_mixed
    camera_names = sorted({f.camera for f in all_frames_combined})
    for cam in camera_names:
        print(f"\n--- {cam} ---")
        cam_hub = [f for f in all_hub if f.camera == cam]
        cam_ds = [f for f in all_ds if f.camera == cam]
        cam_other = [f for f in all_other if f.camera == cam]
        print_stats_line(stats_for_frames(cam_hub, "Hub"))
        print_stats_line(stats_for_frames(cam_ds, "DS"))
        print_stats_line(stats_for_frames(cam_other, "Other"))

    # Per-match comparison table
    print(f"\n--- Per-match hub vs DS comparison ---")
    print(f"{'Match':<8} {'HubN':<7} {'HubMedRes':<11} {'HubOut%':<9} "
          f"{'DSN':<7} {'DSMedRes':<11} {'DSOut%':<9} "
          f"{'Diff':<10} {'Better'}")
    for r in per_match_results:
        hs = stats_for_frames(r["hub_frames"])
        ds = stats_for_frames(r["ds_frames"])
        h_med = hs.get("med_res")
        d_med = ds.get("med_res")
        h_out = hs.get("outlier_pct", 0)
        d_out = ds.get("outlier_pct", 0)

        if h_med is not None and d_med is not None:
            diff = h_med - d_med
            better = "HUB" if diff < 0 else "DS" if diff > 0 else "TIED"
        else:
            diff = None
            better = "-"

        h_str = f"{h_med:.3f}" if h_med is not None else "-"
        d_str = f"{d_med:.3f}" if d_med is not None else "-"
        diff_str = f"{diff:+.3f}" if diff is not None else "-"

        print(f"{r['match']:<8} {hs['count']:<7} {h_str:<11} {h_out if h_out else '-':>7}  "
              f"{ds['count']:<7} {d_str:<11} {d_out if d_out else '-':>7}  "
              f"{diff_str:<10} {better}")

    # Alliance-specific hub breakdown
    print(f"\n--- Red hub vs Blue hub (all matches) ---")
    red_hub_frames = [f for f in all_hub if all(t.tag_id in RED_HUB_TAGS for t in f.tags)]
    blue_hub_frames = [f for f in all_hub if all(t.tag_id in BLUE_HUB_TAGS for t in f.tags)]
    print_stats_line(stats_for_frames(red_hub_frames, "Red hub"))
    print_stats_line(stats_for_frames(blue_hub_frames, "Blue hub"))

    print(f"\n--- Red DS vs Blue DS (all matches) ---")
    red_ds_frames = [f for f in all_ds if all(t.tag_id in RED_DS_TAGS for t in f.tags)]
    blue_ds_frames = [f for f in all_ds if all(t.tag_id in BLUE_DS_TAGS for t in f.tags)]
    print_stats_line(stats_for_frames(red_ds_frames, "Red DS wall"))
    print_stats_line(stats_for_frames(blue_ds_frames, "Blue DS wall"))

    # Distance analysis: how far was the robot from hub vs DS when looking at each?
    print(f"\n--- Average distance to tags by group ---")
    hub_dists = []
    ds_dists = []
    for f in all_hub:
        for t in f.tags:
            if t.tag_id in ALL_HUB_TAGS:
                hub_dists.append(t.dist_to_camera)
    for f in all_ds:
        for t in f.tags:
            if t.tag_id in ALL_DS_TAGS:
                ds_dists.append(t.dist_to_camera)

    if hub_dists:
        print(f"    Hub tags: mean={statistics.mean(hub_dists):.2f}m, "
              f"median={statistics.median(hub_dists):.2f}m, "
              f"range=[{min(hub_dists):.1f}, {max(hub_dists):.1f}]m, "
              f"n={len(hub_dists)}")
    if ds_dists:
        print(f"    DS tags:  mean={statistics.mean(ds_dists):.2f}m, "
              f"median={statistics.median(ds_dists):.2f}m, "
              f"range=[{min(ds_dists):.1f}, {max(ds_dists):.1f}]m, "
              f"n={len(ds_dists)}")

    # Ambiguity by group
    print(f"\n--- Ambiguity comparison ---")
    hub_ambs = [f.max_ambiguity for f in all_hub if f.tags]
    ds_ambs = [f.max_ambiguity for f in all_ds if f.tags]
    if hub_ambs:
        print(f"    Hub: mean={statistics.mean(hub_ambs):.3f}, "
              f"median={statistics.median(hub_ambs):.3f}, "
              f"high(>0.5)={sum(1 for a in hub_ambs if a > 0.5)/len(hub_ambs)*100:.1f}%")
    if ds_ambs:
        print(f"    DS:  mean={statistics.mean(ds_ambs):.3f}, "
              f"median={statistics.median(ds_ambs):.3f}, "
              f"high(>0.5)={sum(1 for a in ds_ambs if a > 0.5)/len(ds_ambs)*100:.1f}%")

    # Tag count when looking at hub vs DS
    print(f"\n--- Tag count when looking at each group ---")
    hub_tc = [f.tag_count for f in all_hub]
    ds_tc = [f.tag_count for f in all_ds]
    if hub_tc:
        tc_dist_hub = {}
        for tc in hub_tc:
            tc_dist_hub[tc] = tc_dist_hub.get(tc, 0) + 1
        print(f"    Hub: mean={statistics.mean(hub_tc):.1f}, distribution={dict(sorted(tc_dist_hub.items()))}")
    if ds_tc:
        tc_dist_ds = {}
        for tc in ds_tc:
            tc_dist_ds[tc] = tc_dist_ds.get(tc, 0) + 1
        print(f"    DS:  mean={statistics.mean(ds_tc):.1f}, distribution={dict(sorted(tc_dist_ds.items()))}")


if __name__ == "__main__":
    main()
