"""Compare per-camera vision accuracy across the 2026 CMP matches.

Reads the JSON exported by:
    logreader export-results vision-analysis <files...> -o results.json
"""
from __future__ import annotations

import json
import statistics as st
import sys
from pathlib import Path
from collections import defaultdict

JSON_PATH = Path(r"D:\Temp\2026_cmps\vision_results.json")


def fmt(v, n=2):
    if v is None:
        return "  -  "
    if isinstance(v, float):
        return f"{v:.{n}f}"
    return str(v)


def match_label(source_file: str) -> str:
    stem = Path(source_file).stem
    for tok in stem.split("_"):
        if tok and tok[0] in "QP" and tok[1:].isdigit():
            return tok
    return stem[-6:]


def main() -> int:
    data = json.loads(JSON_PATH.read_text())

    print("=" * 116)
    print("PER-MATCH PER-CAMERA SUMMARY")
    print("=" * 116)
    print(f"{'Match':<6} {'Camera':<13} {'Total':>6} {'Valid':>6} {'Val%':>6} "
          f"{'MeanRes':>8} {'MedRes':>7} {'Outl%':>6} {'P95Lat':>7} "
          f"{'PeakT':>6} {'Aspect':>7} {'Obliq%':>7}")
    print("-" * 116)

    cams_agg: dict[str, list[dict]] = defaultdict(list)
    mt_agg: dict[str, list[dict]] = defaultdict(list)
    geom_agg: dict[str, list[dict]] = defaultdict(list)
    pair_agg: dict[tuple[str, str], list[dict]] = defaultdict(list)

    # sort by source_file
    data = sorted(data, key=lambda e: e["source_file"])

    for entry in data:
        match = match_label(entry["source_file"])
        cams = sorted(entry["extra"]["cameras"], key=lambda c: c["camera"])
        geom_by_cam = entry["extra"].get("tag_geometry", {})
        for c in cams:
            cam = c["camera"]
            g = geom_by_cam.get(cam, {})
            print(
                f"{match:<6} {cam:<13} "
                f"{c['total_frames']:>6} {c['valid_frames']:>6} "
                f"{c['valid_pct']:>5.1f}% "
                f"{c['mean_residual_m']:>7.2f}m "
                f"{c['median_residual_m']:>6.2f}m "
                f"{c['outlier_pct']:>5.1f}% "
                f"{c['p95_latency_ms']:>6.1f}ms "
                f"{c['peak_cpu_temp']:>5.1f}C "
                f"{fmt(g.get('median_aspect_ratio')):>7} "
                f"{fmt(g.get('pct_oblique'), 1):>6}%"
            )
            cams_agg[cam].append({"match": match, **c})
            if g:
                geom_agg[cam].append({"match": match, **g})

        for cam, m in entry["extra"].get("mt1_mt2_summary", {}).items():
            mt_agg[cam].append({"match": match, **m})

        for a in entry["extra"].get("camera_agreements", []):
            ca, cb = a["camera_a"], a["camera_b"]
            key = tuple(sorted([ca, cb]))
            pair_agg[key].append(a)
        print()

    print("=" * 116)
    print("AGGREGATE PER CAMERA (across all matches)")
    print("=" * 116)
    print(f"{'Camera':<14} {'Mch':>3} {'TotalFr':>8} {'ValidFr':>8} {'Val%':>6} "
          f"{'MeanRes':>8} {'MedRes':>7} {'Outl%':>6} {'P95Lat':>7} {'PeakT':>6} "
          f"{'Aspect':>7} {'Obliq%':>7}")
    print("-" * 116)
    for cam in sorted(cams_agg):
        rows = cams_agg[cam]
        tot = sum(r["total_frames"] for r in rows)
        val = sum(r["valid_frames"] for r in rows)
        outliers = sum(r["outlier_frames"] for r in rows)
        mean_res = (
            sum(r["mean_residual_m"] * r["valid_frames"] for r in rows)
            / max(val, 1)
        )
        med_res = st.median([r["median_residual_m"] for r in rows])
        p95_lat = st.median([r["p95_latency_ms"] for r in rows])
        peak_t = max(r["peak_cpu_temp"] for r in rows)
        grows = geom_agg.get(cam, [])
        aspect = st.median([g["median_aspect_ratio"] for g in grows]) if grows else None
        oblique = st.median([g["pct_oblique"] for g in grows]) if grows else None
        print(
            f"{cam:<14} {len(rows):>3} {tot:>8} {val:>8} "
            f"{100*val/max(tot,1):>5.1f}% "
            f"{mean_res:>7.2f}m "
            f"{med_res:>6.2f}m "
            f"{100*outliers/max(val,1):>5.1f}% "
            f"{p95_lat:>6.1f}ms "
            f"{peak_t:>5.1f}C "
            f"{fmt(aspect):>7} "
            f"{fmt(oblique, 1):>6}%"
        )

    print()
    print("=" * 116)
    print("AGGREGATE MT1 vs MT2 RESIDUAL VS REFERENCE PATH (per camera)")
    print("=" * 116)
    print(f"{'Camera':<14} {'Mch':>3} {'Frames':>7} "
          f"{'MT1med':>7} {'MT1mean':>8} {'MT2med':>7} {'MT2mean':>8} "
          f"{'YawMed':>7} {'YawP95':>7}")
    print("-" * 116)
    for cam in sorted(mt_agg):
        rows = mt_agg[cam]
        nfr = int(sum(r.get("count", 0) for r in rows))
        mt1med = st.median([r["mt1_res_median"] for r in rows])
        mt1mean = st.median([r["mt1_res_mean"] for r in rows])
        mt2med = st.median([r["mt2_res_median"] for r in rows])
        mt2mean = st.median([r["mt2_res_mean"] for r in rows])
        yawmed = st.median([r["yaw_median"] for r in rows])
        yawp95 = st.median([r["yaw_p95"] for r in rows])
        print(
            f"{cam:<14} {len(rows):>3} {nfr:>7} "
            f"{mt1med:>6.2f}m {mt1mean:>7.2f}m "
            f"{mt2med:>6.2f}m {mt2mean:>7.2f}m "
            f"{yawmed:>6.1f}d {yawp95:>6.1f}d"
        )

    print()
    print("=" * 116)
    print("CAMERA-vs-CAMERA AGREEMENT (aggregate)")
    print("=" * 116)
    for (a, b), rows in sorted(pair_agg.items()):
        n = sum(r["overlap_frames"] for r in rows)
        if n == 0:
            continue
        a_better = b_better = tie = 0
        for r in rows:
            ra = r["camera_a"]; rb = r["camera_b"]
            ab = r["a_better_count"]; bb = r["b_better_count"]; tt = r["tie_count"]
            if ra == a and rb == b:
                a_better += ab; b_better += bb; tie += tt
            else:
                a_better += bb; b_better += ab; tie += tt
        total = a_better + b_better + tie
        pose_dis = sum(r["mean_disagreement_m"] * r["overlap_frames"] for r in rows) / n
        yaw_dis = sum(r["mean_yaw_disagreement_deg"] * r["overlap_frames"] for r in rows) / n
        print(f"{a} vs {b}:  overlap={n:5d}  pose_disagree={pose_dis:.2f}m  yaw_disagree={yaw_dis:.1f}deg")
        if total:
            print(f"     {a} better: {100*a_better/total:5.1f}%   "
                  f"{b} better: {100*b_better/total:5.1f}%   "
                  f"tie: {100*tie/total:.1f}%")

    return 0


if __name__ == "__main__":
    sys.exit(main())
