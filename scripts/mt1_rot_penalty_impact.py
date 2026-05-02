"""Estimate the impact of the proposed VisionSubsystem changes (commit 8bcbe1b) on
calculated robot pose for CURIE matches Q6, Q20, Q40, Q50.

Key changes evaluated
---------------------
1) MT1 std-dev formula change:
   old: xy = A_XY_MT1 * dist^P_XY / sqrt(harmonicSum) * ambigInflation
   new: xy = A_XY_MT1 * (dist / angularBaseline) * ambigInflation * MT1_ROTATION_PENALTY

2) Defense reset (hard-snap odometry to vision while underDefense, when
   high-confidence vision disagrees with odometry by > 0.4 m).

3) Reset cooldown (0.5 s) shared between off-field and defense paths.

The analysis re-derives, per MT1 vision frame, both the OLD and NEW xy std-devs,
runs a simple Kalman-style alpha-blend filter on the resulting pose updates, and
counts defense-reset events that would have fired under the new code.

Pose stability proxy: integrated frame-to-frame change in fused pose due to
vision (smaller weight => smaller jump per frame).
Pose correctness proxy: total drift between blended pose and DriveState/Pose
(production result, which used the OLD code).
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path

from logreader.wpilog_reader import read_wpilog
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

# --- constants from VisionSubsystem ----------------------------------------
A_XY_MT1 = 0.09
P_XY = 1.4
MT1_ROTATION_PENALTY = 1.3
MIN_ANGULAR_BASELINE = 0.05
MAX_AMBIGUITY = 0.4
MAX_DISTANCE_MT1 = 2.0
SPREAD_REJECT = 0.50
SPREAD_INFLATE_START = 0.10
DEFENSE_RESET_DISAGREEMENT = 0.4
DEFENSE_RESET_MAX_AMBIGUITY = 0.15
DEFENSE_RESET_MIN_TAGS = 2
DEFENSE_RESET_MAX_SPREAD = 0.10
RESET_COOLDOWN = 0.5
RESET_MAX_AMBIGUITY = 0.15
RESET_MIN_TAGS = 2

# Camera transforms in robot frame (translation only — heading rotation not used
# for angular baseline calculation since both tag and camera are in field XY).
# A=LEFT, B=FRONT, C=RIGHT
CAM_TRANSFORMS = {
    "limelight-a": (-0.076, 0.311),
    "limelight-b": (0.295, -0.288),
    "limelight-c": (0.212, -0.371),
}

LAYOUT = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)
FIELD_X_MAX = LAYOUT.getFieldLength() + 0.30  # FIELD_MARGIN
FIELD_Y_MAX = LAYOUT.getFieldWidth() + 0.30

TAG_POSES: dict[int, tuple[float, float, float]] = {}
for tid in range(1, 23):
    p = LAYOUT.getTagPose(tid)
    if p is not None:
        TAG_POSES[tid] = (p.X(), p.Y(), p.Z())


# --- helpers ---------------------------------------------------------------
def is_on_field(x: float, y: float) -> bool:
    return -0.30 <= x <= FIELD_X_MAX and -0.30 <= y <= FIELD_Y_MAX


def harmonic_sum(fids: list[tuple]) -> float:
    s = 0.0
    for _id, _tx, _ty, _ta, dcam, drob, _amb in fids:
        d = drob if drob > 0.01 else dcam
        if d > 0.01:
            s += 1.0 / (d * d)
    return s


def angular_baseline(
    fids: list[tuple], robot_pose: tuple[float, float, float], cam_dx_dy: tuple[float, float]
) -> float:
    """Compute max pairwise angular separation in radians from camera POV."""
    rx, ry, rth = robot_pose
    cdx, cdy = cam_dx_dy
    cos_t = math.cos(rth)
    sin_t = math.sin(rth)
    cam_x = rx + cos_t * cdx - sin_t * cdy
    cam_y = ry + sin_t * cdx + cos_t * cdy

    valid = []
    for fid in fids:
        tid = int(fid[0])
        if tid in TAG_POSES:
            tx, ty, _ = TAG_POSES[tid]
            valid.append((tx - cam_x, ty - cam_y))
    if len(valid) < 2:
        return MIN_ANGULAR_BASELINE
    max_ang = 0.0
    for i in range(len(valid)):
        ix, iy = valid[i]
        mi = math.hypot(ix, iy)
        if mi < 0.01:
            continue
        for j in range(i + 1, len(valid)):
            jx, jy = valid[j]
            mj = math.hypot(jx, jy)
            if mj < 0.01:
                continue
            cos_a = max(-1.0, min(1.0, (ix * jx + iy * jy) / (mi * mj)))
            ang = math.acos(cos_a)
            if ang > max_ang:
                max_ang = ang
    return max(max_ang, MIN_ANGULAR_BASELINE)


def multi_tag_spread(
    fids: list[tuple], reported_pose: tuple[float, float, float, float]
) -> float:
    """RMS deviation of per-tag distance vs distance implied by reported pose.
    reported_pose: (x, y, z, _yaw)
    """
    if len(fids) < 2:
        return 0.0
    rx, ry, rz, _ = reported_pose
    sumsq = 0.0
    n = 0
    for fid in fids:
        tid = int(fid[0])
        if tid not in TAG_POSES:
            return SPREAD_REJECT + 1.0
        tx, ty, tz = TAG_POSES[tid]
        d_pose = math.sqrt((tx - rx) ** 2 + (ty - ry) ** 2 + (tz - rz) ** 2)
        d_meas = fid[4]  # distToCamera
        sumsq += (d_pose - d_meas) ** 2
        n += 1
    return math.sqrt(sumsq / n) if n else 0.0


def std_dev_old(avg_dist: float, n_tags: int, max_amb: float, fids: list[tuple]) -> float:
    if max_amb >= MAX_AMBIGUITY:
        return float("inf")
    if n_tags == 1 and avg_dist > MAX_DISTANCE_MT1:
        return float("inf")
    amb_inf = 1.0 / (1.0 - max_amb) ** 2
    hs = harmonic_sum(fids)
    if hs <= 0:
        hs = 1.0 / (avg_dist * avg_dist + 1e-6)
    return A_XY_MT1 * (avg_dist**P_XY) / math.sqrt(hs) * amb_inf


def std_dev_new(
    avg_dist: float,
    n_tags: int,
    max_amb: float,
    fids: list[tuple],
    robot_pose: tuple[float, float, float],
    cam_dx_dy: tuple[float, float],
) -> float:
    if max_amb >= MAX_AMBIGUITY:
        return float("inf")
    if n_tags == 1 and avg_dist > MAX_DISTANCE_MT1:
        return float("inf")
    amb_inf = 1.0 / (1.0 - max_amb) ** 2
    ang = angular_baseline(fids, robot_pose, cam_dx_dy)
    return A_XY_MT1 * (avg_dist / ang) * amb_inf * MT1_ROTATION_PENALTY


# --- main analysis ---------------------------------------------------------
@dataclass
class FrameRow:
    t: float  # FPGA seconds
    cam: str
    n_tags: int
    avg_dist: float
    max_amb: float
    spread: float
    odom_x: float
    odom_y: float
    odom_th: float
    vis_x: float
    vis_y: float
    sigma_old: float
    sigma_new: float
    on_field_odom: bool
    on_field_vis: bool


def _interp_pose_at(pose_signal, t_us_query: int) -> tuple[float, float, float] | None:
    """Find Pose2d at given timestamp using nearest-prior sample."""
    vals = pose_signal.values
    if not vals:
        return None
    # binary search
    lo, hi = 0, len(vals) - 1
    if t_us_query < vals[0].timestamp_us:
        return None
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if vals[mid].timestamp_us <= t_us_query:
            lo = mid
        else:
            hi = mid - 1
    raw = vals[lo].value
    if not isinstance(raw, (bytes, bytearray)) or len(raw) < 24:
        return None
    return struct.unpack("<3d", bytes(raw[:24]))


def _interp_bool_at(sig, t_us: int, default=False) -> bool:
    if sig is None:
        return default
    vals = sig.values
    if not vals or t_us < vals[0].timestamp_us:
        return default
    lo, hi = 0, len(vals) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if vals[mid].timestamp_us <= t_us:
            lo = mid
        else:
            hi = mid - 1
    return bool(vals[lo].value)


def analyse(path: str) -> dict:
    print(f"\n=== {Path(path).name} ===")
    ld = read_wpilog(path)
    s = ld.signals
    pose_sig = s.get("NT:/DriveState/Pose")
    underdef_sig = s.get("NT:/SmartDashboard//vision/underDefense")
    if pose_sig is None:
        print("  no DriveState/Pose")
        return {}

    rows: list[FrameRow] = []
    defense_reset_events: list[dict] = []
    offfield_reset_events: list[dict] = []

    last_reset_t_mt1 = -10.0  # seconds
    # Process per camera so the cooldown matches the firmware (single MT1 path
    # is what changes; we'll only simulate MT1 resets here).

    # collect & merge MT1 frames sorted by timestamp across cameras
    all_frames: list[tuple[int, str]] = []
    for cam in CAM_TRANSFORMS:
        bp = s.get(f"NT:/{cam}/botpose_wpiblue")
        if bp is None:
            continue
        for v in bp.values:
            arr = v.value
            if not arr or len(arr) < 11:
                continue
            if arr[7] <= 0:  # tagCount
                continue
            all_frames.append((v.timestamp_us, cam))
    all_frames.sort()

    # Pre-index rawfiducials by camera
    rf_by_cam = {
        cam: s.get(f"NT:/{cam}/rawfiducials") for cam in CAM_TRANSFORMS
    }
    bp_by_cam = {
        cam: s.get(f"NT:/{cam}/botpose_wpiblue") for cam in CAM_TRANSFORMS
    }

    # For each cam keep an iterator pointer
    cam_idx = {cam: 0 for cam in CAM_TRANSFORMS}

    for t_us, cam in all_frames:
        bp_sig = bp_by_cam[cam]
        # advance pointer to matching timestamp
        idx = cam_idx[cam]
        while idx < len(bp_sig.values) and bp_sig.values[idx].timestamp_us < t_us:
            idx += 1
        if idx >= len(bp_sig.values) or bp_sig.values[idx].timestamp_us != t_us:
            continue
        cam_idx[cam] = idx + 1
        bp = bp_sig.values[idx].value
        vis_x, vis_y, vis_z = bp[0], bp[1], bp[2]
        yaw = math.radians(bp[5])
        latency_ms = bp[6]
        n_tags = int(bp[7])
        avg_dist = bp[9]
        if n_tags <= 0 or avg_dist <= 0:
            continue
        if not is_on_field(vis_x, vis_y):
            continue

        # rawfiducials at same timestamp
        rf_sig = rf_by_cam[cam]
        if rf_sig is None:
            continue
        # find rf at same t_us (they're published together)
        # binary search
        rfvals = rf_sig.values
        lo, hi = 0, len(rfvals) - 1
        if t_us < rfvals[0].timestamp_us:
            continue
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if rfvals[mid].timestamp_us <= t_us:
                lo = mid
            else:
                hi = mid - 1
        rf_arr = rfvals[lo].value
        if not rf_arr or len(rf_arr) % 7 != 0 or len(rf_arr) // 7 != n_tags:
            # use whatever rawfiducials are present
            pass
        fids = []
        for k in range(len(rf_arr) // 7):
            base = k * 7
            fids.append(tuple(rf_arr[base : base + 7]))
        if not fids:
            continue
        max_amb = max(f[6] for f in fids)
        if max_amb >= MAX_AMBIGUITY:
            continue

        # odom pose at vision capture time (latency-compensated)
        capture_t_us = t_us - int(latency_ms * 1000)
        op = _interp_pose_at(pose_sig, capture_t_us)
        if op is None:
            continue
        odom_x, odom_y, odom_th = op

        # spread
        spread = multi_tag_spread(fids, (vis_x, vis_y, vis_z, yaw))
        if spread > SPREAD_REJECT:
            continue

        sigma_old = std_dev_old(avg_dist, n_tags, max_amb, fids)
        sigma_new = std_dev_new(
            avg_dist, n_tags, max_amb, fids, (odom_x, odom_y, odom_th), CAM_TRANSFORMS[cam]
        )
        # spread inflation (applied to both)
        spread_infl = (
            (spread / SPREAD_INFLATE_START) ** 2 if spread > SPREAD_INFLATE_START else 1.0
        )
        sigma_old *= spread_infl
        sigma_new *= spread_infl

        on_field_odom = is_on_field(odom_x, odom_y)
        rows.append(
            FrameRow(
                t=t_us / 1e6,
                cam=cam,
                n_tags=n_tags,
                avg_dist=avg_dist,
                max_amb=max_amb,
                spread=spread,
                odom_x=odom_x,
                odom_y=odom_y,
                odom_th=odom_th,
                vis_x=vis_x,
                vis_y=vis_y,
                sigma_old=sigma_old,
                sigma_new=sigma_new,
                on_field_odom=on_field_odom,
                on_field_vis=True,
            )
        )

        # defense reset eligibility
        underdef = _interp_bool_at(underdef_sig, t_us, False)
        disagreement = math.hypot(vis_x - odom_x, vis_y - odom_y)
        t_s = t_us / 1e6
        cool_ok = (t_s - last_reset_t_mt1) >= RESET_COOLDOWN
        if cool_ok:
            # off-field reset path (existed before; still triggers under new code)
            if (
                not on_field_odom
                and max_amb < RESET_MAX_AMBIGUITY
                and n_tags >= RESET_MIN_TAGS
            ):
                offfield_reset_events.append(
                    dict(t=t_s, cam=cam, dis=disagreement, n=n_tags, amb=max_amb)
                )
                last_reset_t_mt1 = t_s
            # NEW defense reset path
            elif (
                underdef
                and max_amb < DEFENSE_RESET_MAX_AMBIGUITY
                and n_tags >= DEFENSE_RESET_MIN_TAGS
                and spread < DEFENSE_RESET_MAX_SPREAD
                and disagreement > DEFENSE_RESET_DISAGREEMENT
            ):
                defense_reset_events.append(
                    dict(
                        t=t_s,
                        cam=cam,
                        dis=disagreement,
                        n=n_tags,
                        amb=max_amb,
                        spread=spread,
                    )
                )
                last_reset_t_mt1 = t_s

    # ---- summarise ----
    if not rows:
        print("  no MT1 frames qualified")
        return {}
    n = len(rows)
    finite_old = [r.sigma_old for r in rows if math.isfinite(r.sigma_old)]
    finite_new = [r.sigma_new for r in rows if math.isfinite(r.sigma_new)]
    ratios = [
        r.sigma_new / r.sigma_old
        for r in rows
        if math.isfinite(r.sigma_old) and math.isfinite(r.sigma_new) and r.sigma_old > 0
    ]
    ratios.sort()

    def pct(arr, p):
        if not arr:
            return float("nan")
        k = int(round((len(arr) - 1) * p / 100))
        return arr[k]

    finite_old.sort()
    finite_new.sort()

    print(f"  MT1 frames analyzed: {n}")
    print(
        f"  sigma_old (m):  median {pct(finite_old,50):.3f}  P10 {pct(finite_old,10):.3f}  P90 {pct(finite_old,90):.3f}"
    )
    print(
        f"  sigma_new (m):  median {pct(finite_new,50):.3f}  P10 {pct(finite_new,10):.3f}  P90 {pct(finite_new,90):.3f}"
    )
    print(
        f"  ratio new/old:  median {pct(ratios,50):.2f}  P10 {pct(ratios,10):.2f}  P90 {pct(ratios,90):.2f}"
    )

    # Kalman gain proxy with sigma_odom = 0.05 m (per-frame swerve odom noise)
    SIGMA_ODOM = 0.05

    def gain(sigma_v):
        if not math.isfinite(sigma_v) or sigma_v <= 0:
            return 0.0
        return SIGMA_ODOM**2 / (SIGMA_ODOM**2 + sigma_v**2)

    # integrated absolute correction magnitude — proxy for jitter contribution
    jitter_old = sum(
        gain(r.sigma_old) * math.hypot(r.vis_x - r.odom_x, r.vis_y - r.odom_y) for r in rows
    )
    jitter_new = sum(
        gain(r.sigma_new) * math.hypot(r.vis_x - r.odom_x, r.vis_y - r.odom_y) for r in rows
    )
    print(
        f"  integrated |K * (vis-odom)| (m):  old={jitter_old:.2f}   new={jitter_new:.2f}  "
        f"ratio={jitter_new/jitter_old:.2f}"
    )

    # Per-camera breakdown
    by_cam: dict[str, list] = {}
    for r in rows:
        by_cam.setdefault(r.cam, []).append(r)
    for cam, lst in by_cam.items():
        olds = sorted(x.sigma_old for x in lst if math.isfinite(x.sigma_old))
        news = sorted(x.sigma_new for x in lst if math.isfinite(x.sigma_new))
        rs = sorted(
            x.sigma_new / x.sigma_old
            for x in lst
            if math.isfinite(x.sigma_old)
            and math.isfinite(x.sigma_new)
            and x.sigma_old > 0
        )
        if not olds:
            continue
        # tag count distribution
        tagdist: dict[int, int] = {}
        for x in lst:
            tagdist[x.n_tags] = tagdist.get(x.n_tags, 0) + 1
        print(
            f"    {cam}: n={len(lst)} tags={tagdist}  "
            f"sigma_old med={pct(olds,50):.3f}  sigma_new med={pct(news,50):.3f}  "
            f"ratio med={pct(rs,50):.2f}"
        )

    # singletag
    single = [r for r in rows if r.n_tags == 1]
    multi = [r for r in rows if r.n_tags >= 2]
    if single:
        os_ = sorted(r.sigma_old for r in single if math.isfinite(r.sigma_old))
        ns_ = sorted(r.sigma_new for r in single if math.isfinite(r.sigma_new))
        rs_ = sorted(
            r.sigma_new / r.sigma_old
            for r in single
            if math.isfinite(r.sigma_old) and math.isfinite(r.sigma_new) and r.sigma_old > 0
        )
        print(
            f"  single-tag MT1: n={len(single)} sigma_old med={pct(os_,50):.3f} "
            f"sigma_new med={pct(ns_,50):.3f} ratio med={pct(rs_,50):.2f}"
        )
    if multi:
        os_ = sorted(r.sigma_old for r in multi if math.isfinite(r.sigma_old))
        ns_ = sorted(r.sigma_new for r in multi if math.isfinite(r.sigma_new))
        rs_ = sorted(
            r.sigma_new / r.sigma_old
            for r in multi
            if math.isfinite(r.sigma_old) and math.isfinite(r.sigma_new) and r.sigma_old > 0
        )
        print(
            f"  multi-tag MT1: n={len(multi)} sigma_old med={pct(os_,50):.3f} "
            f"sigma_new med={pct(ns_,50):.3f} ratio med={pct(rs_,50):.2f}"
        )

    # Reset events
    print(f"  off-field resets (would fire): {len(offfield_reset_events)}")
    for e in offfield_reset_events[:5]:
        print(f"    t={e['t']:.2f}s cam={e['cam']} dis={e['dis']:.2f}m amb={e['amb']:.3f}")
    print(f"  NEW defense resets (would fire): {len(defense_reset_events)}")
    for e in defense_reset_events[:10]:
        print(
            f"    t={e['t']:.2f}s cam={e['cam']} dis={e['dis']:.2f}m amb={e['amb']:.3f} "
            f"spread={e['spread']:.3f}"
        )

    return {
        "n_frames": n,
        "ratio_median": pct(ratios, 50),
        "jitter_old": jitter_old,
        "jitter_new": jitter_new,
        "defense_resets": len(defense_reset_events),
        "offfield_resets": len(offfield_reset_events),
    }


if __name__ == "__main__":
    files = [
        r"D:\Temp\2026_cmps\FRC_20260430_134357_CURIE_Q6.wpilog",
        r"D:\Temp\2026_cmps\FRC_20260430_155123_CURIE_Q20.wpilog",
        r"D:\Temp\2026_cmps\FRC_20260430_195909_CURIE_Q40.wpilog",
        r"D:\Temp\2026_cmps\FRC_20260430_212244_CURIE_Q50.wpilog",
    ]
    summary = []
    for f in files:
        r = analyse(f)
        if r:
            summary.append((Path(f).name, r))
    print("\n=== summary ===")
    for name, r in summary:
        print(
            f"{name}: n={r['n_frames']} ratio_med={r['ratio_median']:.2f} "
            f"jit_old={r['jitter_old']:.1f} jit_new={r['jitter_new']:.1f} "
            f"def_resets={r['defense_resets']} off_resets={r['offfield_resets']}"
        )
