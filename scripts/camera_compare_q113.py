"""Compare Q113 (post-recalibration) vs the prior 9-match baseline."""
from __future__ import annotations
import json
import statistics as st
from pathlib import Path
from collections import defaultdict

BASELINE = Path(r"D:\Temp\2026_cmps\vision_results.json")
NEW = Path(r"D:\Temp\2026_cmps\vision_results_Q113.json")


def fmt(v, n=2):
    if v is None:
        return "  -  "
    if isinstance(v, float):
        return f"{v:.{n}f}"
    return str(v)


def cam_aggregates(data: list[dict]) -> dict[str, dict]:
    """Compute per-camera aggregates over a list of analyzer results."""
    by_cam: dict[str, list[dict]] = defaultdict(list)
    geom: dict[str, list[dict]] = defaultdict(list)
    mt: dict[str, list[dict]] = defaultdict(list)
    bands: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for entry in data:
        for c in entry["extra"]["cameras"]:
            by_cam[c["camera"]].append(c)
        for cam, g in entry["extra"].get("tag_geometry", {}).items():
            geom[cam].append(g)
        for cam, m in entry["extra"].get("mt1_mt2_summary", {}).items():
            mt[cam].append(m)
        for r in entry["extra"].get("per_distance_band", []):
            bands[(r["camera"], r["band_label"])].append(r)

    out: dict[str, dict] = {}
    for cam, rows in by_cam.items():
        tot = sum(r["total_frames"] for r in rows)
        val = sum(r["valid_frames"] for r in rows)
        outl = sum(r["outlier_frames"] for r in rows)
        mean_res = sum(r["mean_residual_m"] * r["valid_frames"] for r in rows) / max(val, 1)
        med_res = st.median([r["median_residual_m"] for r in rows])
        p95_lat = st.median([r["p95_latency_ms"] for r in rows])
        peak_t = max(r["peak_cpu_temp"] for r in rows)
        grows = geom.get(cam, [])
        aspect = st.median([g["median_aspect_ratio"] for g in grows]) if grows else None
        oblique = st.median([g["pct_oblique"] for g in grows]) if grows else None
        mt_rows = mt.get(cam, [])
        mt1_med = st.median([m["mt1_res_median"] for m in mt_rows]) if mt_rows else None
        mt2_med = st.median([m["mt2_res_median"] for m in mt_rows]) if mt_rows else None
        yaw_p95 = st.median([m["yaw_p95"] for m in mt_rows]) if mt_rows else None
        out[cam] = dict(
            matches=len(rows),
            total=tot, valid=val, valid_pct=100*val/max(tot,1),
            mean_res=mean_res, med_res=med_res,
            outlier_pct=100*outl/max(val,1),
            p95_lat=p95_lat, peak_t=peak_t,
            aspect=aspect, oblique=oblique,
            mt1_med=mt1_med, mt2_med=mt2_med, yaw_p95=yaw_p95,
        )
    out["_bands"] = bands  # type: ignore
    return out


baseline_data = json.loads(BASELINE.read_text())
new_data = json.loads(NEW.read_text())

base = cam_aggregates(baseline_data)
new = cam_aggregates(new_data)

base_bands = base.pop("_bands")
new_bands = new.pop("_bands")

print("=" * 110)
print("PER-CAMERA SUMMARY: PRIOR 9 MATCHES vs Q113 (post-recalibration)")
print("=" * 110)
print(f"{'Camera':<14} {'Set':<8} {'Total':>7} {'Valid':>7} {'Val%':>6} "
      f"{'MeanRes':>8} {'MedRes':>7} {'Outl%':>6} {'YawP95':>7} "
      f"{'MT1med':>7} {'MT2med':>7} {'Aspect':>7} {'Obliq%':>7}")
print("-" * 110)
for cam in sorted(set(list(base) + list(new))):
    for label, src in (("prior", base), ("Q113", new)):
        s = src.get(cam)
        if not s:
            print(f"{cam:<14} {label:<8} {'(no data)':>7}")
            continue
        print(f"{cam:<14} {label:<8} "
              f"{s['total']:>7} {s['valid']:>7} {s['valid_pct']:>5.1f}% "
              f"{s['mean_res']:>7.2f}m {s['med_res']:>6.2f}m "
              f"{s['outlier_pct']:>5.1f}% "
              f"{fmt(s['yaw_p95'],1):>6}d "
              f"{fmt(s['mt1_med']):>7} {fmt(s['mt2_med']):>7} "
              f"{fmt(s['aspect']):>7} {fmt(s['oblique'],1):>6}%")
    print()

# Per-distance-band comparison
print("=" * 110)
print("PER-DISTANCE-BAND RESIDUAL: prior(P) vs Q113(N)")
print("=" * 110)
bands_order = ["0\u20131m", "1\u20132m", "2\u20133m", "3\u20134m", "4\u20135m", "5m+"]
cams = sorted({k[0] for k in list(base_bands) + list(new_bands)})
print(f"{'Band':<6} ", end="")
for c in cams:
    print(f"| {c:^28}", end="")
print()
print(f"{'':<6} ", end="")
for _ in cams:
    print(f"| {'P det':>5} {'P res':>6} {'N det':>5} {'N res':>6} ", end="")
print()
print("-" * (7 + len(cams) * 30))

for band in bands_order:
    print(f"{band:<6} ", end="")
    for c in cams:
        for src in (base_bands, new_bands):
            rows = src.get((c, band), [])
            det = sum(r["detections"] for r in rows)
            if det:
                res = sum(r["mean_residual_m"] * r["detections"] for r in rows) / det
                print(f"| {det:>5} {res:>5.2f}m", end="") if src is base_bands else print(f" {det:>5} {res:>5.2f}m ", end="")
            else:
                print(f"| {'-':>5} {'-':>6}", end="") if src is base_bands else print(f" {'-':>5} {'-':>6} ", end="")
    print()

# Q113 camera agreement
print()
print("=" * 110)
print("Q113 camera-to-camera agreement")
print("=" * 110)
for a in new_data[0]["extra"].get("camera_agreements", []):
    print(f"{a['camera_a']} vs {a['camera_b']}: overlap={a['overlap_frames']}  "
          f"pose_disagree={a['mean_disagreement_m']:.2f}m  yaw={a['mean_yaw_disagreement_deg']:.1f}d  "
          f"a_better={a['a_better_pct']:.1f}%  b_better={a['b_better_pct']:.1f}%")
