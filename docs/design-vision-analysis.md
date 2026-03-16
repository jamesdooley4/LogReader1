# Vision Analysis — Design Document

> Analyzer: `vision-analysis` · Module: `src/logreader/analyzers/vision_analysis.py`

Analyse Limelight vision performance across one or more cameras. Produce per-frame quality metrics, per-match summary tables, cross-match trend comparisons, field-position heatmaps, and temporal diagnostics. The goal is actionable feedback for tuning vision-based pose estimation — where on the field is vision trustworthy, which tags are problematic, and when does processing degrade.

**Status:** Design complete
**Depends on:** `match-phases` (optional, for per-phase breakdown), `pose-analysis` (optional, for higher-quality pose reference)
**Validated against:** `FRC_20260307_225328_WABON_Q15.wpilog`, `FRC_20260308_004101_WABON_Q28.wpilog`, `FRC_20260308_225734_WABON_E14.wpilog`

---

## Overview

### Goals

1. **Per-frame quality metrics** — For every camera frame, compute tag count, ambiguity, distance, latency, pose standard deviations, and pose-vs-odometry residual. These are the atomic building blocks for all other analyses.
2. **Tabular summaries** — Aggregate per-frame metrics into per-camera, per-tag-ID, per-tag-count, per-distance-band, and per-phase tables for a single match. Support cross-match comparison tables for multiple log files.
3. **Visual diagnostics** — Field-position heatmaps showing vision availability, pose accuracy, tag coverage, and camera-to-camera agreement. Ambiguity-vs-distance scatter plots. Stddev confidence maps.
4. **Temporal diagnostics** — Time-series of tag count, pose residual, latency, FPS, CPU temperature, and camera agreement. Detect dropped frames, detection gaps, reacquisition delay, and thermal throttling.
5. **Hardware health** — Monitor Limelight FPS, CPU temperature, and RAM usage to detect thermal throttling and processing bottlenecks.
6. **Robot-activity correlation** — Correlate vision detection success, accuracy, and quality degradation with concurrent robot activities. This includes physical occlusion (e.g. intake position blocking a camera's FOV), vibration and camera shake (e.g. hard hits jarring the chassis, fuel launches shaking the turret and nearby cameras), and mechanism motion (e.g. elevator or arm movement). Quantify the impact per activity (e.g. "LL-a valid rate drops from 22% to 3% when intake is INTERMEDIATE", "mean residual increases 2× within 200ms of a hard hit", "LL-b ambiguity spikes during launch bursts"). Cross-references `intake-analysis`, `hard-hits`, and `launch-counter` results when available.
7. **Actionable trust guidance** — Recommend simple, defensible thresholds the robot code can use later (for example: ignore single-tag detections beyond 4m, down-weight frames with ambiguity > 0.55, prefer camera B in specific field regions).

### Non-Goals

- **Implementing a pose estimator** — That belongs in `pose-analysis`. This analyzer evaluates the *raw camera outputs*, not the fused robot pose.
- **Limelight network debugging** — Detecting NT connection drops, IP assignment issues, or bandwidth problems is outside scope (though dropped frame detection may surface some of these symptoms).
- **Tag map validation** — We assume the field AprilTag layout is correct. Detecting misplaced field tags is an interesting but separate problem.
- **Real-time tuning** — This is offline post-match analysis. Live dashboards are a long-term goal for the project, not this analyzer.
- **Absolute ground truth** — Odometry or fused robot pose is only a reference signal. This analyzer measures consistency and practical trustworthiness, not laboratory-grade accuracy.
- **Pixel-level reprojection error** — Computing true reprojection error per tag requires detected corner pixel coordinates and camera intrinsics (focal length, principal point, distortion coefficients). The Limelight computes reprojection internally (that is how the `ambiguity` field is derived), but the raw corner locations and per-tag reprojection residual are never published to NetworkTables or logged in `.wpilog` files. The `tcornxy` signal (from `getCornerCoordinates()`) requires "send contours" to be enabled in the Limelight Output tab and is not present in our logs.
- **Image-level blur / sharpness score** — No raw pixel data, Laplacian-variance metric, or any image-quality signal is exposed through NetworkTables. Motion blur and focus issues cannot be measured directly from the logged data.

---

## Review of the Current Plan

The existing draft already captures the important raw Limelight signals and several strong output ideas. The main strengths are:

1. It uses the right atomic units: per-frame metrics, per-tag unpacking, and latency-aware pose comparison.
2. It already separates summary tables, spatial plots, and time-series diagnostics.
3. It supports more than one Limelight and acknowledges cross-match comparison.

The main gaps to fix before implementation are:

1. The plan needs a clearer execution order so implementation can proceed in testable phases.
2. The metric set should distinguish must-have trust metrics from optional exploratory plots.
3. Multi-camera analysis needs to be more explicit: camera agreement, camera handoff regions, and per-camera strengths are central when a robot has more than one Limelight.
4. The outputs should answer concrete tuning questions, not just enumerate data.

This revision addresses those gaps by defining a step-by-step workflow, a prioritized metric taxonomy, and explicit multi-camera comparison outputs.

---

## Key Questions This Analyzer Should Answer

1. Which camera is trustworthy in which parts of the field?
2. How much does vision quality improve when two or more tags are visible?
3. At what distance or ambiguity level do single-tag estimates stop being useful?
4. Are bad vision estimates caused mainly by geometry, latency, heat, robot activity, or field position?
5. When two Limelights disagree, which one is usually closer to the robot reference pose?
6. Does vision performance degrade over a match or across a full event day?

---

## Step-by-Step Analysis Plan

1. **Discover cameras and reference signals** — Auto-detect every Limelight stream, its companion quality signals, and the best available robot reference pose.
2. **Normalize all camera frames** — Parse `botpose`, `rawfiducials`, `stddevs`, and hardware data into a canonical per-frame structure. Shift timestamps backward by reported latency where possible.
3. **Compute per-frame trust metrics** — For each frame, compute detection validity, ambiguity, tag geometry, latency, residual, and stale-data indicators.
4. **Aggregate per-camera and per-tag tables** — Produce concise tables that identify which cameras, tags, and geometry conditions are reliable or problematic.
5. **Build spatial diagnostics** — Map availability, accuracy, and confidence across field position so the team can see camera coverage and dead zones.
6. **Build temporal diagnostics** — Show how quality changes during the match and around robot activities such as impacts, launches, and intake movement.
7. **Compare cameras and matches** — For multi-camera robots or multiple logs, quantify relative strengths, agreement, and drift over time.
8. **Emit tuning guidance** — Convert analysis into simple recommended gates, thresholds, and follow-up checks for robot code and camera mounting.

---

## Recommended Metric Priority

These priorities keep the implementation focused on the metrics most likely to change robot behavior.

### Tier 1: Must-Have Metrics

- Detection rate (`valid_frames / total_frames`)
- Tag count distribution (0, 1, 2, 3, 4+)
- Per-frame latency (`tl`, `cl`, total latency)
- Pose residual vs reference pose
- Per-tag ambiguity and distance
- Detection gaps and reacquisition time
- FPS and thermal trends

### Tier 2: Strongly Recommended Metrics

- Per-tag-ID trust table
- Distance-band and tag-count-band residual tables
- Field heatmaps for availability and residual
- MegaTag1 vs MegaTag2 divergence
- Camera-to-camera agreement when two cameras overlap in time
- Per-phase breakdown (auto / teleop / disabled)

### Tier 3: Nice-to-Have Metrics

- Heading-conditioned coverage maps
- Spatial maps of preferred camera by field zone
- Residual vs robot speed or angular velocity
- Correlation with intake, launch, or impact events
- Cross-event trend comparison over many matches

### Tier 4: `t2d`-Derived Tag Geometry Quality Proxies

The `t2d` signal provides per-frame primary-target metadata that serves as indirect quality indicators when raw pixel corners and camera intrinsics are unavailable (see Non-Goals). These are the best available proxies for detection quality:

- **Tag pixel size** (`longSidePx`, `shortSidePx`) — how many pixels the tag occupies. Very small tags (< 20px) are more susceptible to pixel noise, quantisation error, and ambiguity spikes. Strongly correlated with distance.
- **Tag aspect ratio** (`longSidePx / shortSidePx`) — proxies for viewing angle. A ratio near 1.0 means the tag is seen nearly head-on; a high ratio (> 2) means oblique viewing angle. **Critically, the relationship with pose quality is non-monotonic** — see the PnP degeneracy analysis below.
- **Tag skew** (`skewDeg`) — rotation of the tag bounding box in the image. High skew combined with high aspect ratio indicates a tag seen at a steep angle.
- **Tag pixel extent** (`hExtentPx`, `vExtentPx`) — bounding box width and height in pixels. Provides resolution-at-range information.

#### PnP Head-On Degeneracy — Single-Match vs Cross-Match Findings

Initial analysis of match E14 alone (5869 frames) found a dramatic PnP solver degeneracy at aspect ratio 1.05–1.10: 66% outlier rate at 2–2.5m distance, with median residual >1m. This appeared to be the well-known head-on ambiguity problem where the PnP solver cannot distinguish two candidate poses.

**Cross-match validation across 18 Bonney Lake matches (85,000+ single-tag frames) showed this narrow-band finding was E14-specific:**

| AR Band | Total Frames (all matches) | Mean Residual | Aggregate Outlier% |
|---|---|---|---|
| 1.00–1.05 (head-on) | 11,340 | 1.077m | 25.1% |
| 1.05–1.10 | 10,149 | 0.795m | 24.3% |
| 1.10–1.20 | 21,615 | 0.571m | 13.3% |
| 1.20–1.50 | 20,646 | 2.361m | 35.8% |
| 1.50–2.00 | 7,312 | 0.630m | 14.8% |
| 2.00+ | 5,869 | 0.358m | 3.1% |

**Corrected findings:**
1. The 1.05–1.10 band is not uniquely bad across matches. Its aggregate outlier rate (24.3%) is nearly identical to 1.00–1.05 (25.1%). E14's 66% was an extreme outlier driven by match-specific conditions.
2. Both near-head-on bands (AR < 1.10) are problematic — about 25% outlier rate for single-tag detections — but this is not narrowly concentrated; the PnP ambiguity issue affects the entire near-head-on range.
3. The 1.20–1.50 band has the highest aggregate outlier rate (35.8%), driven by specific matches (e.g. Q42 at 78%). This was invisible in E14-only analysis.
4. High aspect ratio (2.00+) is consistently the safest band — only 3.1% outliers across all matches. Oblique views provide strong geometric constraints.
5. Outlier rates vary dramatically match-to-match within each AR band, suggesting the dominant quality factors are tag-specific geometry, field position, and lighting conditions, not aspect ratio alone.

**Revised guidance for pose fusion code:**
- **Tag count** remains the strongest quality predictor. Multi-tag detections reduce residuals by 3–4× vs single-tag and are immune to head-on degeneracy.
- **Ambiguity** (from `rawfiducials`) is the best single-value quality gate for single-tag detections. Reject frames with ambiguity > 0.5 at distance > 3m.
- **Aspect ratio** is a useful diagnostic tool for understanding *why* a detection failed (head-on geometry, oblique view, etc.) but is **not reliable as a standalone filter** — its effect varies too much match-to-match.
- The aspect-ratio plots (AR vs residual, AR vs ambiguity) remain valuable for post-match analysis and camera mounting decisions.

#### Recommended Visualization

Two plots surface the aspect ratio–quality relationship in reports:

1. **Aspect ratio vs residual scatter** — X: aspect ratio (log scale), Y: residual, colored by tag count (single vs multi). Shows whether head-on degeneracy was significant in a given match.
2. **Aspect ratio vs ambiguity scatter** — X: aspect ratio, Y: ambiguity, colored by residual. Reveals the relationship between the Limelight's ambiguity estimate and actual pose quality across viewing angles.

---

## Data Observations

Analysed across three matches from the Bonney Lake 2026 event. The robot has two Limelights: `limelight-a` (front-facing) and `limelight-b` (left-facing).

### Signal Inventory

Each Limelight publishes the following signals to NetworkTables (and thus to the `.wpilog`):

| Signal | Type | Rate | Description |
|--------|------|------|-------------|
| `botpose_wpiblue` | double[11] | ~46–49 Hz | Robot pose estimate in WPILib Blue alliance coords. Published every frame. |
| `botpose_orb_wpiblue` | double[11] | ~46–49 Hz | MegaTag2 (ORB-SLAM) pose estimate. Published every frame. |
| `rawfiducials` | double[7×N] | ~3–9 Hz | Per-tag detection data. Only published when tags visible. |
| `stddevs` | double[12] | ~3–9 Hz | Pose standard deviations (6 for MegaTag1 + 6 for MegaTag2). |
| `tl` | double | ~46–49 Hz | Pipeline processing latency (ms). |
| `cl` | double | ~46–49 Hz | Image capture latency (ms). |
| `hb` | double | ~46–49 Hz | Frame heartbeat counter. |
| `tv` | double | ~3–9 Hz | Target valid flag (1.0 = tags visible). Change-driven. |
| `tid` | double | ~1–2 Hz | Primary target fiducial ID. Change-driven. |
| `tx`, `ty` | double | ~3–9 Hz | Primary target X/Y offset from crosshair (degrees). |
| `txnc`, `tync` | double | ~3–9 Hz | Primary target X/Y offset, no-crosshair (degrees). |
| `ta` | double | ~3–9 Hz | Primary target area (% of image). |
| `hw` | double[4] | ~1.4 Hz | Hardware stats: [fps, cpu_temp, ram_mb, temp]. |
| `imu` | double[10] | ~46–49 Hz | Onboard IMU data (if applicable). |
| `t2d` | double[17] | ~46–49 Hz | Primary target 2D metadata (validity, counts, latency, pixel extents, skew). See layout below. |
| `tc` | double[3] | ~15–43 Hz | Average BGR color under crosshair region. Not useful for quality analysis. |
| `botpose_targetspace` | double[] | ~3–9 Hz | Robot pose in target reference frame. |
| `camerapose_targetspace` | double[] | ~3–9 Hz | Camera pose in target reference frame. |
| `targetpose_cameraspace` | double[] | ~3–9 Hz | Target pose in camera reference frame. |
| `targetpose_robotspace` | double[] | ~3–9 Hz | Target pose in robot reference frame. |
| `robot_orientation_set` | double[] | variable | Robot orientation hint sent to Limelight. |

Robot-side vision signals (processed/fused by robot code):

| Signal | Type | Rate | Description |
|--------|------|------|-------------|
| `NT:vision/fieldPose3d` | struct (Pose3d) | ~23 Hz | Fused field pose from all cameras. |
| `NT:vision/rawFieldPose3dA` | struct (Pose3d) | ~11 Hz | Raw pose from camera A. |
| `NT:vision/rawFieldPose3dB` | struct (Pose3d) | ~19 Hz | Raw pose from camera B. |
| `NT:/Pose/robotPose` | double[] | ~50 Hz | Final robot pose (odometry + vision fused). |
| `NT:/SmartDashboard/Field/RawVision` | double[] | ~23 Hz | Vision poses for dashboard display. |

### `botpose_wpiblue` Array Layout (11 elements)

| Index | Field | Units | Notes |
|-------|-------|-------|-------|
| 0 | X | meters | Blue alliance origin |
| 1 | Y | meters | Blue alliance origin |
| 2 | Z | meters | Typically 0.0 (2D mode) |
| 3 | Roll | degrees | Typically 0.0 |
| 4 | Pitch | degrees | Typically 0.0 |
| 5 | Yaw | degrees | -180 to +180 |
| 6 | Total latency | ms | Pipeline + capture + NT transport |
| 7 | Tag count | count | 0 = no tags → pose data is invalid/stale |
| 8 | Tag span | meters | Distance between furthest tags in view |
| 9 | Avg tag distance | meters | Average distance from camera to visible tags |
| 10 | Avg tag area | % of image | Average area of visible tags in image |

> **Gotcha:** When `tag_count` (index 7) is 0, the pose fields (0–5) are **stale/zero** — they hold the last valid pose or all zeros. Always filter by `tag_count > 0`.

### `rawfiducials` Per-Tag Packing (7 values per tag)

| Offset | Field | Description |
|--------|-------|-------------|
| 0 | Tag ID | AprilTag fiducial ID |
| 1 | txnc | Tag center X offset from crosshair (degrees) |
| 2 | tync | Tag center Y offset from crosshair (degrees) |
| 3 | ta | Tag area (fraction of image, 0–1 scale) |
| 4 | distToCamera | Distance from camera lens to tag (meters) |
| 5 | distToRobot | Distance from robot center to tag (meters) |
| 6 | ambiguity | Pose ambiguity (0 = unambiguous, 1 = highly ambiguous) |

Multiple tags are concatenated: a frame seeing 3 tags has a 21-element array.

### `stddevs` Array Layout (12 elements)

Two sets of 6 standard deviations: MegaTag1 (indices 0–5) and MegaTag2/ORB (indices 6–11).

| Index | Field | Typical range (observed) |
|-------|-------|--------------------------|
| 0 | MegaTag1 X stddev | 0.01 – 5.8 m |
| 1 | MegaTag1 Y stddev | 0.00 – 2.2 m |
| 2 | MegaTag1 Z stddev | ~0 (2D mode) |
| 3 | MegaTag1 Roll stddev | ~0 |
| 4 | MegaTag1 Pitch stddev | ~0 |
| 5 | MegaTag1 Yaw stddev | 0.4 – 167 deg |
| 6 | MegaTag2 X stddev | 0.00 – 5.8 m |
| 7 | MegaTag2 Y stddev | 0.01 – 2.4 m |
| 8–11 | MegaTag2 Z/Roll/Pitch/Yaw | ~0 (2D mode) |

### `t2d` Array Layout (17 elements)

Per-frame primary-target 2D metadata. Published every frame at pipeline rate.

| Index | Field | Units | Notes |
|-------|-------|-------|-------|
| 0 | targetValid | 0/1 | 1.0 when a target is detected |
| 1 | targetCount | count | Number of targets in view |
| 2 | targetLatency | ms | Pipeline processing latency (same as `tl`) |
| 3 | captureLatency | ms | Capture latency (same as `cl`) |
| 4 | tx | degrees | Horizontal offset from crosshair |
| 5 | ty | degrees | Vertical offset from crosshair |
| 6 | txnc | degrees | Horizontal offset from principal point |
| 7 | tync | degrees | Vertical offset from principal point |
| 8 | ta | fraction | Target area as fraction of image |
| 9 | tid | ID | Primary target fiducial ID (-1 when none) |
| 10 | classIdx_detector | index | Neural detector class index (-1 for AprilTag pipeline) |
| 11 | classIdx_classifier | index | Neural classifier class index (-1 for AprilTag pipeline) |
| 12 | longSidePx | pixels | Tag bounding-box long side length |
| 13 | shortSidePx | pixels | Tag bounding-box short side length |
| 14 | hExtentPx | pixels | Horizontal pixel extent of bounding box |
| 15 | vExtentPx | pixels | Vertical pixel extent of bounding box |
| 16 | skewDeg | degrees | Tag skew / rotation in image |

> When `targetValid` (index 0) is 0, indices 4–16 are stale or zero.

> **Quality proxies:** See Tier 4 metrics. `longSidePx / shortSidePx` approximates obliqueness, `shortSidePx` approximates effective resolution at range, and `skewDeg` indicates image-plane rotation.

### `tc` Array Layout (3 elements)

Average BGR color under the crosshair region: `[Blue, Green, Red]`. Values are 0.0–1.0 normalised channel intensities. This signal is not useful for vision quality analysis.

### `hw` Array Layout (4 elements)

| Index | Field | Units | Observed range |
|-------|-------|-------|----------------|
| 0 | FPS | frames/s | 26–51 |
| 1 | CPU temp | °C | 5–75 |
| 2 | RAM usage | MB | ~62 (stable) |
| 3 | Board temp | °C | 5–63 |

### Cross-Match Signal Characteristics

| Metric | Q15 | Q28 | E14 |
|--------|-----|-----|-----|
| LL-a total frames | 19525 | 19997 | 22127 |
| LL-a valid (tags seen) | 4270 (21.9%) | 2638 (13.2%) | 1453 (6.6%) |
| LL-a mean latency | 21.8 ms | 21.7 ms | 23.5 ms |
| LL-b total frames | 19512 | 19562 | 24014 |
| LL-b valid (tags seen) | 3592 (18.4%) | 3489 (17.8%) | 4416 (18.4%) |
| LL-b mean latency | 30.9 ms | 31.4 ms | 31.2 ms |
| LL-a HB median interval | 16.7 ms | — | 16.7 ms |
| LL-b HB median interval | 16.8 ms | — | 16.8 ms |
| LL-a dropped frames (>50ms) | — | — | 2 (0.0%) |
| LL-b dropped frames (>50ms) | — | — | 2 (0.0%) |

> **Key observation:** LL-a valid-frame rate varies dramatically across matches (6.6% – 21.9%), while LL-b is stable (~18%). This is driven by differences in how much time the robot spends facing tags that LL-a can see. Field-relative heading analysis would explain this.

### Pose Quality: Residual vs Odometry (E14)

| Metric | LL-a | LL-b |
|--------|------|------|
| Frames matched | 1453 | 4416 |
| Mean residual | 0.43 m | 0.55 m |
| Median residual | 0.17 m | 0.21 m |
| Max residual | 6.09 m | 7.54 m |
| 1-tag mean residual | 0.54 m | 0.80 m |
| 2-tag mean residual | 0.15 m | 0.19 m |
| 3-tag mean residual | 0.27 m | 0.13 m |
| Outliers (>1m) | 119 (8.2%) | 1075 (24.3%) |

> **Key insight:** Multi-tag detections dramatically reduce pose error — 2-tag residuals are 3–4× smaller than 1-tag. Single-tag frames with high ambiguity are the primary source of large outliers.

### Ambiguity vs Distance (E14)

| Distance band | LL-a mean ambiguity | LL-b mean ambiguity |
|---------------|---------------------|---------------------|
| Close (< 2m) | 0.175 | 0.088 |
| Mid (2–4m) | 0.543 | 0.507 |
| Far (> 4m) | 0.637 | 0.679 |

High ambiguity (> 0.5): LL-a 44.6%, LL-b 49.9% of all detections. Ambiguity increases sharply with distance — single-tag detections beyond 3m should be treated with low confidence.

### Tag Geometry Quality Proxies from `t2d` (E14)

The `t2d` signal provides per-frame primary-target pixel-level metadata that serves as an indirect quality proxy when raw corner coordinates and camera intrinsics are unavailable (corner data requires "send contours" to be enabled in the Limelight Output tab; camera intrinsics are never logged).

| Metric | LL-a | LL-b |
|--------|------|------|
| Valid frames | 1453 | 4416 |
| longSidePx range | 26–1080 | 20–1283 |
| longSidePx median | 70 | 67 |
| shortSidePx range | 23–218 | 17–329 |
| shortSidePx median | 46 | 47 |
| Aspect ratio (long/short) median | 1.23 | 1.32 |
| Aspect ratio > 2 (steep oblique) | 431 (29.7%) | 1492 (33.8%) |
| shortSidePx < 20px (tiny tag) | 0 (0.0%) | 5 (0.1%) |
| \|skew\| median | 6.5° | 6.4° |
| \|skew\| max | 90° | 90° |

> **Key observations:**
> - About 30–34% of detections have aspect ratio > 2, meaning the tag is seen at a steep oblique angle. Cross-match validation shows these are actually the *safest* detections (3.1% outlier rate across 18 matches).
> - Median `shortSidePx` of ~46 pixels provides reasonable detection quality. Tags below 20px are rare but may appear at long range.
> - Skew is bimodal: most frames have low skew (< 10°), but extreme skew (up to 90°) occurs, likely at edges of the camera FOV or during rapid rotation.
> - The `longSidePx` can be very large (>1000px) when a tag is close and viewed at an oblique angle, inflating the aspect ratio while `shortSidePx` remains reasonable.
> - Near-head-on detections (AR < 1.10) have ~25% outlier rates across all matches, but the severity varies dramatically match-to-match (4% to 73%), so aspect ratio alone is not a reliable filter.

### Investigated but Unavailable Metrics

Two promising quality metrics were investigated and found to be **not available** from the logged Limelight data:

1. **Pixel reprojection error per tag** — Would require detected corner pixel coordinates (not logged for AprilTag pipeline — `tcornxy` / `getCornerCoordinates()` needs "send contours" enabled in Limelight Output tab, which is not the default) and camera intrinsic matrix (not logged). The Limelight computes reprojection internally to derive the `ambiguity` field, but the raw per-corner pixel residuals are not exposed.

2. **Corner sharpness / blur score** — A Laplacian-variance metric per frame would quantify motion blur or focus issues and should correlate with missed detections. However, no raw pixel data, image-quality metrics, or sharpness signals are published to NetworkTables.

The `t2d`-derived tag pixel size, aspect ratio, and skew (documented above) are the best available indirect quality proxies.

---

## Analysis Categories

### 1. Per-Frame Numeric Metrics

These are the atomic computations performed for every camera frame with a valid tag detection. They feed into all aggregate and visual outputs.

| Metric | Source | Computation |
|--------|--------|-------------|
| **Tag count** | `botpose_wpiblue[7]` | Direct read |
| **Tag IDs** | `rawfiducials[0, 7, 14, ...]` | Extract from packed array |
| **Per-tag ambiguity** | `rawfiducials[6, 13, 20, ...]` | Extract from packed array |
| **Per-tag distance** | `rawfiducials[4, 11, 18, ...]` | Camera-to-tag distance (m) |
| **Per-tag area** | `rawfiducials[3, 10, 17, ...]` | Tag area in image (fraction) |
| **Total latency** | `botpose_wpiblue[6]` | Direct read (ms) |
| **Pipeline latency** | `tl` | Direct read (ms) |
| **Capture latency** | `cl` | Direct read (ms) |
| **MegaTag1 stddev (x,y,yaw)** | `stddevs[0,1,5]` | Direct read |
| **MegaTag2 stddev (x,y)** | `stddevs[6,7]` | Direct read |
| **Pose (x, y, yaw)** | `botpose_wpiblue[0,1,5]` | Direct read (only when tag_count > 0) |
| **Pose residual** | botpose vs odometry | Euclidean distance between latency-compensated botpose and nearest odometry sample |
| **MT1 vs MT2 divergence** | `botpose_wpiblue` vs `botpose_orb_wpiblue` | Euclidean distance when both have tags |
| **Max ambiguity** | per-tag ambiguities | Worst individual tag ambiguity in frame |
| **Tag span** | `botpose_wpiblue[8]` | Distance between furthest visible tags (m) |
| **Frame interval** | `hb` timestamps | Time since previous frame from same camera |
| **Dropped-frame flag** | frame interval | `True` when interval exceeds threshold (default 50ms) |
| **Detection-gap flag** | valid-frame history | `True` when valid detections resume after a gap > threshold |
| **Reacquisition delay** | valid-frame history | Time from start of gap to next valid frame |
| **Stale-pose flag** | `tag_count`, pose deltas | `True` when pose is repeated or zero while no tags are visible |
| **Publish-to-capture age** | `tl`, `cl` | Estimated age of the measurement at publish time |
| **Reference pose speed** | odometry | Linear speed near the frame timestamp |
| **Reference yaw rate** | odometry / gyro | Angular speed near the frame timestamp |
| **Tag pixel size (long)** | `t2d[12]` | Primary target bounding-box long side (pixels) |
| **Tag pixel size (short)** | `t2d[13]` | Primary target bounding-box short side (pixels) |
| **Tag aspect ratio** | `t2d[12] / t2d[13]` | Obliqueness proxy; 1.0 = head-on, >2 = steep angle |
| **Tag skew** | `t2d[16]` | Primary target rotation in image (degrees) |
| **Tag pixel extent (h, v)** | `t2d[14], t2d[15]` | Bounding box width and height in pixels |

#### Multi-Camera Per-Frame Metrics

When two or more cameras produce frames close together in time, compute additional overlap metrics.

| Metric | Source | Computation |
|--------|--------|-------------|
| **Camera agreement residual** | camera pose vs camera pose | Euclidean distance between latency-aligned camera poses within a small time window |
| **Camera heading agreement** | camera yaw vs camera yaw | Smallest wrapped yaw difference |
| **Best-camera flag** | per-camera residuals | Camera with lower residual to reference at overlapping timestamps |
| **Overlap coverage** | frame timestamps | Fraction of valid frames that have another camera valid nearby |

### 2. Tabular Metrics (Per-Match and Cross-Match)

#### a) Per-Camera Summary Table

One row per camera per match.

| Column | Description |
|--------|-------------|
| Camera | Camera name (e.g. `limelight-a`) |
| Total frames | Heartbeat count |
| Valid frames | Frames with tag_count > 0 |
| Valid % | Detection rate |
| Mean latency (ms) | Average total latency for valid frames |
| P95 latency (ms) | 95th percentile latency |
| Mean FPS | From hw signal |
| Mean CPU temp (°C) | From hw signal |
| Peak CPU temp (°C) | Thermal throttling indicator |
| Dropped frames | Frame gaps > 50ms |
| Mean pose residual (m) | vs odometry, valid frames only |
| Median pose residual (m) | vs odometry |
| Outlier frames (>1m) | Count and percentage |

#### b) Per-Tag-ID Table

One row per tag ID per camera, sorted by detection count.

| Column | Description |
|--------|-------------|
| Tag ID | AprilTag fiducial number |
| Detections | Number of frames where this tag was seen |
| % of valid frames | How often this tag contributed |
| Mean ambiguity | Average pose ambiguity for this tag |
| Mean distance (m) | Average camera-to-tag distance |
| Mean area (%) | Average tag area in image |
| Mean residual when primary (m) | Pose residual when this was the closest/primary tag |
| High-ambiguity rate (%) | Fraction of detections with ambiguity > 0.5 |

This table directly answers "which tags should I trust?" and "which tags am I relying on most?"

#### c) Per-Tag-Count Table

One row per tag count level (1, 2, 3, 4+).

| Column | Description |
|--------|-------------|
| Tag count | Number of simultaneous tags |
| Frames | Number of frames at this tag count |
| Mean residual (m) | Pose accuracy at this tag count |
| Median residual (m) | More robust accuracy measure |
| Mean ambiguity | Average per-tag ambiguity |
| Mean stddev X (m) | Limelight's own confidence |
| Mean stddev Y (m) | Limelight's own confidence |

#### d) Ambiguity-Distance Band Table

Rows: distance bands (0–1m, 1–2m, 2–3m, 3–4m, 4–5m, 5m+). Columns per camera.

| Column | Description |
|--------|-------------|
| Distance band | Range in meters |
| Detections | Tag detections in this band |
| Mean ambiguity | Average ambiguity |
| High-ambiguity % | Fraction > 0.5 |
| Mean residual (m) | Pose accuracy in this band |

Directly informs the team's "max trust distance" for single-tag detections.

#### e) Per-Phase Summary (if match-phases available)

Break down valid frame rate and mean residual by match phase (Auto / Teleop / Disabled). Vision availability often differs dramatically between auto (robot at known positions, specific tags in view) and teleop (robot moving freely).

#### f) Cross-Match Comparison Table

When multiple log files are provided, present the per-camera summary table with one column-group per match. Highlights trends: is valid % declining match-over-match (thermal? lens fogging?), is latency increasing (CPU load growth)?

#### g) Multi-Camera Comparison Table

One row per camera pair per match.

| Column | Description |
|--------|-------------|
| Camera pair | e.g. `limelight-a` vs `limelight-b` |
| Overlap frames | Frames where both cameras have valid nearby timestamps |
| Mean pose disagreement (m) | Mean translation difference between cameras |
| P95 disagreement (m) | High-end disagreement |
| Mean yaw disagreement (deg) | Wrapped heading difference |
| A better than B | Count and % of overlap frames where A has lower residual |
| B better than A | Count and % of overlap frames where B has lower residual |
| Tie / both poor | Both similar or both above residual threshold |

This is the main table for deciding whether one Limelight is consistently better than another or whether each has different coverage strengths.

### 3. Visual Diagnostics

These require a plotting library (matplotlib or plotly, gated behind an optional dependency).

#### a) Vision Availability Heatmap

- **What:** 2D field heatmap (top-down, field dimensions ~16.5m × 8.2m) colored by detection rate.
- **How:** Bin the field into a grid (e.g. 0.5m cells). For each valid frame, place the robot's odometry position into a cell. Count valid vs total frames that fall in each cell. Color by valid/total ratio.
- **Value:** Shows *where on the field* vision works well vs poorly. Dead zones may indicate obstructed sightlines, far-away tags, or robot heading issues.
- **Per-camera variant:** Separate heatmaps per camera reveal each camera's coverage zone.

#### b) Pose Accuracy Heatmap

- **What:** Same field grid, colored by median pose residual (botpose vs odometry).
- **How:** For each cell, collect all valid-frame residuals when the robot was in that cell. Color by median residual (green < 0.15m, yellow 0.15–0.5m, red > 0.5m).
- **Value:** Shows *where on the field* vision is most/least accurate. Combined with the availability heatmap: a cell can be "available but inaccurate" (high valid % but high residual) or "unavailable" (no valid frames).

#### c) Tag ID Scatter or Polar Plot

- **What:** For each tag ID, plot the robot positions (or headings) where that tag was detected.
- **How:** Plot robot (x,y) colored by tag ID for all valid frames. Or: for each tag, show a polar histogram of robot heading at time of detection.
- **Value:** Reveals which tags cover which field regions and heading arcs. Helps teams understand if certain tags are never seen due to camera mounting angles.

#### d) Ambiguity vs Distance Scatter

- **What:** Scatter plot of per-tag ambiguity (Y) vs camera-to-tag distance (X), colored by tag ID.
- **How:** One point per tag detection from `rawfiducials`.
- **Value:** Shows the distance threshold beyond which ambiguity becomes unacceptable. Different tags may behave differently (damaged, partially obscured, or at awkward angles). An overlay line at ambiguity = 0.5 highlights the "trust cutoff."

#### e) Pose Stddev Confidence Map

- **What:** Field heatmap colored by mean Limelight-reported stddev (x and y combined).
- **How:** For each field cell, average the stddev values from frames where the robot was in that cell.
- **Value:** Shows Limelight's own confidence across the field. Compare to the residual heatmap — if stddevs are low but residuals are high, the Limelight is overconfident (miscalibration or tag map error).

#### f) MegaTag1 vs MegaTag2 Divergence Map

- **What:** Field heatmap of mean MegaTag1-vs-MegaTag2 pose divergence.
- **Value:** Regions where the two algorithms disagree strongly suggest one is struggling (e.g., MegaTag1 with high ambiguity vs MegaTag2 with incorrect ORB features). Helps teams decide which algorithm to trust in each field zone.

#### g) Preferred-Camera Spatial Map

- **What:** Field grid colored by which camera has the lower median residual in that cell.
- **How:** In cells where both cameras have enough valid samples, compare per-camera median residual and color by the winner; use neutral color when sample count is too low.
- **Value:** Directly answers camera placement questions such as "front camera is better near source side, side camera is better while crossing midfield."

#### h) Heading-Conditioned Availability Map

- **What:** Field heatmap or polar plot that combines robot position with heading buckets.
- **How:** Bin detections by field cell and heading quadrant or octant.
- **Value:** Distinguishes "camera cannot see tags here" from "camera can see tags here only when the robot faces a certain way." This matters for driver behavior and autonomous path design.

#### i) Aspect Ratio vs Residual Scatter (PnP Degeneracy Diagnostic)

- **What:** Scatter plot with aspect ratio (X, log scale) vs pose residual (Y), colored by tag count (single vs multi-tag).
- **How:** One point per valid frame. Use log scale on X to spread the critical 1.0–1.2 range. Overlay a shaded region for the 1.05–1.10 degeneracy zone. Separate colors for single-tag (where degeneracy strikes) vs multi-tag (immune).
- **Value:** Directly visualises the PnP head-on degeneracy. Teams can see the spike in residuals at aspect ratio 1.05–1.10 and understand why their pose estimator occasionally produces wild outliers. The single-tag vs multi-tag coloring shows that multi-tag detections are immune. Provides a visual justification for the recommended aspect-ratio gating rule.

#### j) Aspect Ratio vs Ambiguity Scatter

- **What:** Scatter plot with aspect ratio (X) vs ambiguity (Y), colored by pose residual (green → red).
- **How:** One point per valid frame with rawfiducials data. Color by residual using a diverging colormap (green = low residual, red = high).
- **Value:** Reveals the gap between what ambiguity promises and what actually happens. In the 1.05–1.10 zone, ambiguity is moderate (~0.5–0.6) but residuals are catastrophic (>1m). This plot demonstrates that ambiguity alone is insufficient for quality gating and that aspect ratio provides complementary information in the near-head-on regime.

### 4. Temporal Diagnostics (Time-Series)

#### a) Tag Count Over Time

- **What:** Line chart of tag count per frame over the match duration. One line per camera, optionally stacked.
- **Annotations:** Match phase boundaries (auto → teleop → endgame).
- **Value:** Instantly shows when vision is blind (tag count = 0), degraded (1 tag), or strong (2+ tags). Phase correlation reveals if auto routines keep good tag visibility but teleop driving does not.

#### b) Pose Residual Over Time

- **What:** Scatter or line chart of pose-vs-odometry residual for each valid frame.
- **Annotations:** Threshold lines at 0.5m and 1.0m. Outlier points highlighted.
- **Value:** Spikes reveal specific moments of bad pose estimates — often a single-tag high-ambiguity detection. Teams can investigate what the robot was doing at that moment (heading, speed, field position).

#### c) Latency Over Time

- **What:** Line chart of total latency (`botpose[6]`) per frame. Separate lines for pipeline (`tl`) and capture (`cl`) latency.
- **Annotations:** Latency threshold (e.g. 50ms).
- **Value:** Latency spikes indicate processing overload. Sustained high latency degrades pose estimation because the latency-compensated timestamp becomes less reliable.

#### d) FPS and Thermal Over Time

- **What:** Dual-axis chart: FPS on left axis, CPU temp and board temp on right axis.
- **Annotations:** FPS drop below nominal (e.g. < 30), temp thresholds (thermal throttle typically ~85°C, observed up to 75°C).
- **Value:** Correlates FPS drops with temperature rises. If temp rises over a competition day, early matches may have good performance but later matches degrade.

#### e) Detection Gaps

- **What:** Timeline showing periods with no valid frames (per camera).
- **How:** Identify periods where no `tag_count > 0` frame exists for > 500ms. Plot as colored bars on a timeline.
- **Annotations:** Match phase, robot speed (are gaps during fast driving? turns?), robot heading (facing away from tags?).
- **Value:** Long detection gaps during critical moments (auto scoring, teleop aiming) are directly actionable — the team may need to adjust camera placement or driving strategy.

#### f) Camera Agreement Over Time

- **What:** Time-series of inter-camera disagreement when both cameras are valid.
- **How:** Plot translation and heading disagreement for overlapping frames.
- **Value:** Shows whether disagreements are isolated outliers or long windows where one camera drifts systematically.

#### g) Reacquisition Delay Timeline

- **What:** Event series marking each transition from blind to valid vision, with the gap duration attached.
- **How:** Detect valid-frame gaps and annotate the next valid frame with its gap duration.
- **Value:** Useful for measuring how quickly each camera recovers after fast turns, occlusions, or mechanism movement.

---

## Output Priorities

To keep the first implementation practical, outputs should be built in this order:

1. **Core tables** — Per-camera summary, per-tag-count, per-distance-band, per-tag-ID.
2. **Core time-series** — Tag count, residual, latency, FPS/temperature, detection gaps.
3. **Core field maps** — Availability heatmap and residual heatmap.
4. **Multi-camera diagnostics** — Agreement table, preferred-camera map, camera-agreement time-series.
5. **Activity correlations** — Intake / launch / hard-hit overlays once the base analyzer is stable.

---

## Data Models

```python
from dataclasses import dataclass, field
from enum import Enum


@dataclass
class TagDetection:
    """A single AprilTag detection within one frame."""
    tag_id: int
    txnc: float           # X offset from crosshair (degrees)
    tync: float           # Y offset from crosshair (degrees)
    area: float           # fraction of image (0–1)
    dist_to_camera: float # meters
    dist_to_robot: float  # meters
    ambiguity: float      # 0–1


@dataclass
class VisionFrame:
    """Per-frame metrics for one camera."""
    timestamp_us: int
    camera: str

    # Pose (only valid when tag_count > 0)
    x: float
    y: float
    yaw_deg: float
    tag_count: int
    tag_span: float       # meters
    avg_tag_dist: float   # meters
    avg_tag_area: float   # fraction

    # Latency
    total_latency_ms: float
    pipeline_latency_ms: float
    capture_latency_ms: float

    # Quality
    stddev_mt1_x: float
    stddev_mt1_y: float
    stddev_mt1_yaw: float
    stddev_mt2_x: float
    stddev_mt2_y: float

    # Tag geometry quality proxies (from t2d, primary target only)
    tag_long_side_px: float = 0.0    # bounding-box long side (pixels)
    tag_short_side_px: float = 0.0   # bounding-box short side (pixels)
    tag_aspect_ratio: float = 0.0    # longSide / shortSide; 1.0 = head-on
    tag_skew_deg: float = 0.0        # image-plane rotation (degrees)
    tag_h_extent_px: float = 0.0     # horizontal pixel extent
    tag_v_extent_px: float = 0.0     # vertical pixel extent

    # Per-tag details (from rawfiducials)
    tags: list[TagDetection] = field(default_factory=list)

    # Computed
    pose_residual_m: float | None = None  # vs odometry
    mt1_mt2_divergence_m: float | None = None
    max_ambiguity: float = 0.0


@dataclass
class CameraSummary:
    """Aggregate metrics for one camera in one match."""
    camera: str
    total_frames: int
    valid_frames: int
    valid_pct: float
    mean_latency_ms: float
    p95_latency_ms: float
    mean_fps: float
    mean_cpu_temp: float
    peak_cpu_temp: float
    dropped_frames: int
    mean_residual_m: float
    median_residual_m: float
    outlier_frames: int       # residual > 1m
    outlier_pct: float


@dataclass
class TagSummary:
    """Aggregate metrics for one AprilTag ID as seen by one camera."""
    tag_id: int
    camera: str
    detections: int
    pct_of_valid: float
    mean_ambiguity: float
    mean_distance: float
    mean_area: float
    mean_residual_when_primary: float
    high_ambiguity_pct: float  # fraction with ambiguity > 0.5


@dataclass
class TagCountBand:
    """Aggregate metrics for frames at a given tag count."""
    tag_count: int
    frames: int
    mean_residual_m: float
    median_residual_m: float
    mean_ambiguity: float
    mean_stddev_x: float
    mean_stddev_y: float


@dataclass
class DistanceBand:
    """Aggregate metrics for detections in a distance range."""
    band_label: str        # e.g. "2–3m"
    band_min: float
    band_max: float
    detections: int
    mean_ambiguity: float
    high_ambiguity_pct: float
    mean_residual_m: float


@dataclass
class VisionAnalysisResult:
    """Complete vision analysis output."""
    cameras: list[CameraSummary]
    per_tag: list[TagSummary]
    per_tag_count: list[TagCountBand]
    per_distance_band: list[DistanceBand]
    frames: list[VisionFrame]        # all valid frames (for plots)
    detection_gaps: list[tuple[str, float, float]]  # (camera, start_s, end_s)
```

---

## Output Design

### Summary (CLI default)

```
Vision Analysis
===============

Camera: limelight-a
  Frames: 22127 total, 1453 valid (6.6%)
  Latency: mean 23.5ms, P95 32.1ms
  FPS: 38.1 avg, CPU: 45.5°C avg / 68.3°C peak
  Dropped frames: 2 (0.0%)
  Pose residual: mean 0.43m, median 0.17m
  Outliers (>1m): 119 (8.2%)

Camera: limelight-b
  Frames: 24014 total, 4416 valid (18.4%)
  Latency: mean 31.2ms, P95 41.7ms
  FPS: 43.0 avg, CPU: 64.9°C avg / 74.8°C peak
  Dropped frames: 2 (0.0%)
  Pose residual: mean 0.55m, median 0.21m
  Outliers (>1m): 1075 (24.3%)

Tag count breakdown (both cameras combined):
  1 tag:  3643 frames  mean residual 0.69m
  2 tags: 2003 frames  mean residual 0.17m
  3 tags:  200 frames  mean residual 0.14m
  4 tags:   23 frames  mean residual 0.20m
```

### Tag Detail Table (`--detail`)

```
Camera: limelight-a
Tag  Detections  Valid%  MeanAmb  MeanDist  MeanArea  MeanResid  HighAmb%
  7       21      1.4%   0.421     2.89m    0.0011     0.32m     42.9%
  8       14      1.0%   0.503     3.21m    0.0008     0.48m     57.1%
  9       71      4.9%   0.312     1.94m    0.0032     0.21m     28.2%
 10       70      4.8%   0.345     2.12m    0.0028     0.24m     31.4%
 ...
```

### Ambiguity-Distance Table (`--detail`)

```
Camera: limelight-b
Distance   Detections  MeanAmb  HighAmb%  MeanResid
  0–1m           82     0.032     0.0%      0.05m
  1–2m          998     0.108     2.1%      0.12m
  2–3m         2814     0.478    43.2%      0.31m
  3–4m         1581     0.558    56.8%      0.62m
  4–5m          718     0.644    71.2%      0.89m
  5m+           252     0.721    82.4%      1.34m
```

---

## Signal Auto-Detection

Detect cameras by scanning for signal name patterns:

| Pattern | Matches |
|---------|---------|
| `NT:/limelight-*/botpose_wpiblue` | Primary camera identifier and pose |
| `NT:/limelight-*/hb` | Heartbeat (frame count) |
| `NT:/limelight-*/rawfiducials` | Per-tag detection data |
| `NT:/Pose/robotPose` or `NT:/DriveState/Pose` | Odometry reference |

Camera names are extracted from the signal path (e.g. `limelight-a` from `NT:/limelight-a/botpose_wpiblue`).

Allow CLI overrides: `--cameras limelight-a,limelight-b`, `--odometry-signal "NT:/Pose/robotPose"`.

---

## CLI Usage

```bash
# Basic — auto-detect cameras, single match
logreader vision-analysis path/to/log.wpilog

# Detailed per-tag and distance-band tables
logreader vision-analysis path/to/log.wpilog --detail

# Generate visual diagnostics (requires matplotlib)
logreader vision-analysis path/to/log.wpilog --plots --output-dir ./vision_plots/

# Specific cameras only
logreader vision-analysis path/to/log.wpilog --cameras limelight-b

# Cross-match comparison
logreader vision-analysis match1.wpilog match2.wpilog match3.wpilog --compare

# Custom residual threshold for outlier flagging
logreader vision-analysis path/to/log.wpilog --outlier-threshold 0.5
```

---

## `extra` Dict Schema

```python
result.extra = {
    "cameras": list[CameraSummary],
    "per_tag": list[TagSummary],
    "per_tag_count": list[TagCountBand],
    "per_distance_band": list[DistanceBand],
    "frames": list[VisionFrame],           # all valid frames
    "detection_gaps": list[tuple],          # (camera, start_s, end_s)
    "vision_result": VisionAnalysisResult,  # full structured result
}
```

---

## Edge Cases & Fallbacks

- **No Limelight signals:** Report "No Limelight cameras detected" and exit cleanly. Suggest checking signal names.
- **Single camera:** Works fine — just produce one column in comparison tables.
- **No rawfiducials:** Skip per-tag analysis. Per-frame still works using botpose tag_count and latency.
- **No odometry signal:** Skip pose residual computation. All other metrics still computed. Warn the user.
- **All-zero botpose:** When tag_count is 0, pose is invalid. Filter these frames from all quality computations.
- **Camera disconnect / reboot:** Detect via heartbeat gaps > 1s. Report as a connection event, restart frame counting.
- **MegaTag2 not enabled:** If `botpose_orb_wpiblue` data is all-zero even when tags are visible, skip MT1-vs-MT2 divergence.
- **Multiple matches (--compare):** Align on match phases where possible. If match durations differ, compare by phase (auto, teleop) rather than absolute time.
- **No plotting library:** If `--plots` is requested but matplotlib is not installed, print a clear error message with install instructions rather than crashing.
- **No `t2d` signal:** Skip tag geometry quality proxies (pixel size, aspect ratio, skew). All other metrics still computed. Log a note that `t2d` was not found.
- **`t2d` all-zero when no target:** When `t2d[0]` (targetValid) is 0, indices 12–16 are stale or zero. Only populate tag geometry fields for frames where `t2d[0] == 1`.

---

## Testing Strategy

1. **Synthetic data — per-camera summary:** Create a `LogData` with botpose frames at known positions (some valid, some zero). Verify valid %, latency stats, and detection counting.
2. **Synthetic data — rawfiducials parsing:** Generate rawfiducials arrays with known tag IDs, ambiguities, and distances. Verify correct unpacking and per-tag aggregation.
3. **Synthetic data — pose residual:** Create botpose frames and odometry frames at known positions. Verify residual computation including latency compensation.
4. **Synthetic data — detection gaps:** Create a sequence with a 2-second gap in valid frames. Verify gap detection and reporting.
5. **Synthetic data — multi-camera overlap:** Create two cameras with overlapping valid frames and known disagreement. Verify camera-agreement metrics and preferred-camera selection.
6. **Synthetic data — heading-conditioned coverage:** Build detections that only appear for specific headings in the same field cell. Verify the heading-bucket aggregation.
7. **Real data regression:** Run against E14 and verify tag counts, valid percentages, latency stats, and major residual trends match the manually observed values documented above.
8. **Synthetic data — t2d quality proxies:** Create `t2d` frames with known pixel sizes, aspect ratios, and skew values. Verify correct extraction and that invalid `t2d` frames (targetValid=0) produce zero values.

---

## Implementation Sequence

1. **Parser layer** — Discover Limelight signals, unpack `botpose`, `rawfiducials`, `stddevs`, and hardware stats into `VisionFrame` objects.
2. **Reference alignment** — Find odometry or fused robot pose, apply latency compensation, and compute per-frame residuals.
3. **Summary tables** — Implement per-camera, per-tag-count, per-distance-band, and per-tag-ID aggregation.
4. **Gap and overlap metrics** — Add dropped-frame detection, detection gaps, reacquisition delay, and camera-overlap comparison.
5. **Plotting hooks** — Add optional plot generation for heatmaps and time-series.
6. **Cross-analyzer overlays** — Add optional intake / launch / impact annotations once the core analyzer is validated.
