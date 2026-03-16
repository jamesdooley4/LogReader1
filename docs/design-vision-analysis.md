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
2. **Tabular summaries** — Aggregate per-frame metrics into per-camera, per-tag-ID, per-tag-count, and per-distance-band tables for a single match. Support cross-match comparison tables for multiple log files.
3. **Visual diagnostics** — Field-position heatmaps showing vision availability, pose accuracy, and tag coverage. Ambiguity-vs-distance scatter plots. Stddev confidence maps.
4. **Temporal diagnostics** — Time-series of tag count, pose residual, latency, FPS, and CPU temperature. Detect dropped frames, detection gaps, and thermal throttling.
5. **Hardware health** — Monitor Limelight FPS, CPU temperature, and RAM usage to detect thermal throttling and processing bottlenecks.
6. **Robot-activity correlation** — Correlate vision detection success, accuracy, and quality degradation with concurrent robot activities. This includes physical occlusion (e.g. intake position blocking a camera's FOV), vibration and camera shake (e.g. hard hits jarring the chassis, fuel launches shaking the turret and nearby cameras), and mechanism motion (e.g. elevator or arm movement). Quantify the impact per activity (e.g. "LL-a valid rate drops from 22% to 3% when intake is INTERMEDIATE", "mean residual increases 2× within 200ms of a hard hit", "LL-b ambiguity spikes during launch bursts"). Cross-references `intake-analysis`, `hard-hits`, and `launch-counter` results when available.

### Non-Goals

- **Implementing a pose estimator** — That belongs in `pose-analysis`. This analyzer evaluates the *raw camera outputs*, not the fused robot pose.
- **Limelight network debugging** — Detecting NT connection drops, IP assignment issues, or bandwidth problems is outside scope (though dropped frame detection may surface some of these symptoms).
- **Tag map validation** — We assume the field AprilTag layout is correct. Detecting misplaced field tags is an interesting but separate problem.
- **Real-time tuning** — This is offline post-match analysis. Live dashboards are a long-term goal for the project, not this analyzer.

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
| `t2d` | double[17] | ~46–49 Hz | 2D target corner data with metadata. |
| `tc` | double[N] | ~15–43 Hz | Target corner pixel locations. |
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

---

## Testing Strategy

1. **Synthetic data — per-camera summary:** Create a `LogData` with botpose frames at known positions (some valid, some zero). Verify valid %, latency stats, and detection counting.
2. **Synthetic data — rawfiducials parsing:** Generate rawfiducials arrays with known tag IDs, ambiguities, and distances. Verify correct unpacking and per-tag aggregation.
3. **Synthetic data — pose residual:** Create botpose frames and odometry frames at known positions. Verify residual computation including latency compensation.
4. **Synthetic data — detection gaps:** Create a sequence with a 2-second gap in valid frames. Verify gap detection and reporting.
5. **Real data regression:** Run against E14 and verify tag counts, valid percentages, and latency stats match the manually observed values documented above.
