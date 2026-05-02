"""Drill into per-distance-band accuracy for each camera (aggregated)."""
from __future__ import annotations
import json
from pathlib import Path
from collections import defaultdict

JSON_PATH = Path(r"D:\Temp\2026_cmps\vision_results.json")
data = json.loads(JSON_PATH.read_text())

# (cam, band) -> [(detections, mean_residual, mean_ambig, high_ambig_pct), ...]
agg: dict[tuple[str, str], list[dict]] = defaultdict(list)
band_order: list[str] = []
for entry in data:
    for r in entry["extra"].get("per_distance_band", []):
        key = (r["camera"], r["band_label"])
        agg[key].append(r)
        if r["band_label"] not in band_order:
            band_order.append(r["band_label"])

bands = ["0\u20131m", "1\u20132m", "2\u20133m", "3\u20134m", "4\u20135m", "5m+"]
cams = sorted({k[0] for k in agg})

print("Per-distance-band aggregate (det = detections summed, res = det-weighted mean residual):")
print()
print(f"{'Band':<7} ", end="")
for c in cams:
    print(f"| {c:^28}", end="")
print()
print(f"{'':<7} ", end="")
for _ in cams:
    print(f"| {'det':>6} {'res(m)':>7} {'ambig':>5} {'hi%':>5}  ", end="")
print()
print("-" * (8 + len(cams) * 30))

for band in bands:
    print(f"{band:<7} ", end="")
    for c in cams:
        rows = agg.get((c, band), [])
        det = sum(r["detections"] for r in rows)
        if det:
            res = sum(r["mean_residual_m"] * r["detections"] for r in rows) / det
            amb = sum(r["mean_ambiguity"] * r["detections"] for r in rows) / det
            hi = sum(r["high_ambiguity_pct"] * r["detections"] for r in rows) / det
            print(f"| {det:>6} {res:>7.2f} {amb:>5.2f} {hi:>4.0f}%  ", end="")
        else:
            print(f"| {'-':>6} {'-':>7} {'-':>5} {'-':>5}  ", end="")
    print()

# Per-tag table
print()
print("Per-tag aggregate (det-weighted mean residual):")
agg_tag: dict[tuple[str, int], list[dict]] = defaultdict(list)
for entry in data:
    for r in entry["extra"].get("per_tag", []):
        agg_tag[(r["camera"], r["tag_id"])].append(r)

tag_ids = sorted({k[1] for k in agg_tag})
print(f"{'Tag':<5} ", end="")
for c in cams:
    print(f"| {c:^22}", end="")
print()
print(f"{'':<5} ", end="")
for _ in cams:
    print(f"| {'det':>6} {'res(m)':>7} {'ambig':>5}  ", end="")
print()
print("-" * (6 + len(cams) * 24))
for t in tag_ids:
    print(f"{t:<5} ", end="")
    for c in cams:
        rows = agg_tag.get((c, t), [])
        det = sum(r["detections"] for r in rows)
        if det:
            res = sum(r["mean_residual_when_primary"] * r["detections"] for r in rows) / det
            amb = sum(r["mean_ambiguity"] * r["detections"] for r in rows) / det
            print(f"| {det:>6} {res:>7.2f} {amb:>5.2f}  ", end="")
        else:
            print(f"| {'-':>6} {'-':>7} {'-':>5}  ", end="")
    print()
