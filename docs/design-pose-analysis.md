# Pose Correlation and Divergence Analysis — Design Document

> Analyzer: `pose-analysis` · Module: `src/logreader/analyzers/pose_analysis.py`

**Status:** Design complete, not yet implemented
**Role:** Standalone analyzer + shared pose-reference service for other analyzers (especially `drive-analysis`)
**Depends on:** Optional integration with `match-phases` for phase-aware divergence reporting
**Validated against:** *(needs validation against real logs — see [Open Questions](#open-questions))*

Build the best offline estimate of robot pose available from the log, then measure how each individual sensor modality (wheel odometry, vision, accelerometer) correlates or diverges from that estimate.

---

## Overview

This feature is both a **standalone analyzer** (`pose-analysis`) and a **shared service** that other analyzers can import to obtain a single notion of actual robot motion. The design has two layers:

1. **`pose_analysis.py`** — a module in `src/logreader/analyzers/` that exports reusable pose-fusion functions, data models, and signal-discovery helpers.
2. **`PoseAnalysisAnalyzer`** — the `@register_analyzer` CLI analyzer that uses layer 1 and presents a human-readable divergence report.

Other analyzers (e.g. `drive-analysis`) import the service functions directly — they never need to instantiate `PoseAnalysisAnalyzer`.

---

## Goals

- Build the best offline estimate of robot pose available from the log.
- Measure drift, outliers, and disagreement for each source individually.
- Produce actionable tuning guidance for real-time estimators such as `SwerveDrivePoseEstimator`.
- Highlight time windows where one sensor was likely misleading while others remained consistent.

## Non-goals

- Claim true physical ground truth without an external reference system (this is a **best-estimate reference path**, not ground truth).
- Reconstruct a perfect pose when all sources are poor or missing.
- Replace WPILib estimators in robot code; this is an offline diagnostic and tuning tool.

---

## Data Observations

### Expected signal names and patterns

FRC robot logs contain pose-related signals under several common naming conventions. The analyzer must auto-detect from multiple patterns.

**Wheel odometry / fused robot pose:**

| Signal pattern | Type | Source | Notes |
|----------------|------|--------|-------|
| `NT:/SmartDashboard/Field/Robot` | `double[]` | Shuffleboard / AdvantageScope | `[x, y, theta]` or `[x, y, theta, ...]` |
| `NT:/SmartDashboard/*/Pose` | `struct:Pose2d` or `double[]` | Custom NT publisher | Common team convention |
| `DriveState/Pose` | `struct:Pose2d` or `double[]` | Subsystem periodic | High-rate (~239 Hz in observed logs) |
| `*/EstimatedPose` | `struct:Pose2d` | `SwerveDrivePoseEstimator` output | Already-fused pose from robot code |
| `*/OdometryPose` | `struct:Pose2d` | Raw wheel-only odometry | No vision corrections |

**Vision camera pose estimates:**

| Signal pattern | Type | Source | Notes |
|----------------|------|--------|-------|
| `limelight-*/botpose` | `double[]` | Limelight | `[x, y, z, roll, pitch, yaw, ...]` field-relative |
| `limelight-*/botpose_wpiblue` | `double[]` | Limelight | Blue-alliance-origin field-relative |
| `photonvision/*/result` | `struct` or `raw` | PhotonVision | Contains pose estimate + metadata |
| `*/VisionPose` | `struct:Pose2d` or `double[]` | Custom publisher | Team-specific |

**Vision quality metadata:**

| Signal pattern | Type | Meaning |
|----------------|------|---------|
| `limelight-*/tv` | `double` | Target valid (0 or 1) |
| `limelight-*/ta` | `double` | Target area (proxy for distance) |
| `limelight-*/tl` | `double` | Pipeline latency (ms) |
| `limelight-*/cl` | `double` | Capture latency (ms) |
| `limelight-*/tid` | `double` | Primary AprilTag ID |
| `photonvision/*/hasTarget` | `boolean` | Whether any target was seen |
| `*/ambiguity` | `double` | Pose ambiguity ratio |

**IMU / accelerometer:**

| Signal pattern | Type | Source | Notes |
|----------------|------|--------|-------|
| `DS:accX`, `DS:accY`, `DS:accZ` | `double` | Driver Station | Robot-relative, gravity-compensation varies |
| `*/accel_x`, `*/accel_y` | `double` | CTRE Pigeon2, NavX | May be robot-relative or gravity-compensated |
| `*/RawGyro_*` | `double` | Pigeon2 | Raw gyro rates |

**Gyro / heading:**

| Signal pattern | Type | Source | Notes |
|----------------|------|--------|-------|
| `*/Yaw` or `*/yaw` | `double` | NavX, Pigeon2 | Degrees, +/-180 or continuous |
| `*/angle` | `double` | Generic gyro | May be continuous (not wrapped) |
| `DriveState/GyroAngle` | `double` | Subsystem periodic | Degrees |

**Chassis speeds (optional but useful):**

| Signal pattern | Type | Source |
|----------------|------|--------|
| `DriveState/MeasuredSpeeds` | `struct:ChassisSpeeds` or `double[]` | Swerve kinematics output |
| `*/vx`, `*/vy`, `*/omega` | `double` | Individual components |

### Source rate characteristics

| Source class | Typical rate | Notes |
|---|---|---|
| Wheel odometry / fused pose | 50-250 Hz | `DriveState/Pose` observed at ~239 Hz |
| Vision (Limelight) | 10-30 Hz | Bursty -- depends on tag visibility |
| Vision (PhotonVision) | 15-90 Hz | Depends on resolution and pipeline |
| Accelerometer | 50-200 Hz | Depends on device and log rate |
| Gyro heading | 50-250 Hz | Often same rate as odometry |

### Key timing considerations

- Limelight `botpose` is published at receive time, but actual capture was `tl + cl` ms earlier. The analyzer must shift vision timestamps backward by this latency when those signals are available.
- PhotonVision results may include a `timestampSeconds` field in the struct payload.
- Odometry and gyro are typically synchronous (same main-loop timestamp).

---

## Input Contract

The analyzer should explicitly separate **required**, **optional**, and **derived** inputs.

### Required for useful analysis

At least two of the following source classes must be present:

1. **Wheel odometry / fused robot pose** -- directly logged pose samples or chassis speeds + gyro heading.
2. **Vision camera pose estimates** -- one or more camera-provided robot poses.
3. **IMU / accelerometer data** -- at minimum, planar acceleration (`ax`, `ay`).

If only one source class is present, the analyzer should still report signal discovery results and basic self-consistency checks, but should mark fusion/divergence results as `low` confidence or unavailable.

### Strongly preferred optional inputs

- Vision latency per update or timestamp of image capture (`tl`, `cl` signals)
- Vision quality signals (ambiguity, tag count, target area, target valid)
- Gyro yaw / yaw rate
- Chassis speeds (`vx`, `vy`, `omega`)
- Match phases from `match-phases`
- Robot code's own estimator output (useful for comparison, not as authoritative truth)

---

## Source Characteristics

1. **Wheel odometry + gyro**
   - High frequency (50-250 Hz), locally precise.
   - Prone to long-term drift and sudden divergence during wheel slip, impacts, or poor characterization.
2. **Vision camera(s)**
   - Lower frequency (10-30 Hz) and often bursty -- depends on tag visibility.
   - Globally informative but vulnerable to noise, ambiguity, latency, and occasional gross outliers.
   - Limelight pipeline latency is typically 11-33 ms; capture latency adds another 5-15 ms.
3. **Accelerometer (IMU)**
   - Very high frequency (50-200 Hz depending on device and log rate).
   - Useful for short-term dynamics, impacts, and detecting motion changes.
   - Not reliable as a standalone long-term position source due to integration drift.

---

## Coordinate Frames and Normalization

Before comparing sources, the analyzer must normalize them into a common representation.

### Canonical reference frame

- Position: field-relative `x`, `y` in **meters**
- Heading: field-relative `theta` in **radians**, normalized to `(-pi, pi]`
- Velocity: field-relative `vx`, `vy` in **m/s**, `omega` in **rad/s**
- Acceleration: field-relative after rotation from IMU frame; raw IMU-frame values preserved separately for noise analysis

### Required normalization steps

- Normalize heading wraparound to `(-pi, pi]` consistently
- Detect whether accelerometer values are robot-relative (rotate by heading) or already field-relative
- Track whether accelerometer is gravity-compensated (flag as assumption if unknown)
- Align units across pose sources (some use degrees, some radians; some use inches, most use meters)
- If multiple cameras exist, preserve both per-camera streams and an optional combined vision stream
- Handle timestamp alignment: Limelight `botpose` uses publish time but actual capture is `tl + cl` ms earlier; shift vision timestamps backward by this latency when those signals are available
- Detect WPILib blue-alliance vs red-alliance origin conventions

If frame assumptions cannot be validated from the signals alone, the analyzer should record them in its output as explicit `assumptions`.

---

## Data Models

```python
@dataclass
class PoseSample:
    """A single pose observation from any source."""
    timestamp_us: int
    x: float          # meters, field-relative
    y: float          # meters, field-relative
    theta: float      # radians, (-pi, pi]
    source_name: str   # signal name that produced this sample
    source_class: str  # "odometry" | "vision" | "accelerometer"
    confidence: float  # 0.0-1.0, based on available quality metadata
    latency_us: int    # estimated latency (0 if unknown)


@dataclass
class PoseSource:
    """A discovered and parsed source of pose information."""
    name: str
    source_class: str        # "odometry" | "vision" | "accelerometer" | "gyro"
    signal_names: list[str]  # raw signal names used
    sample_count: int
    start_us: int
    end_us: int
    median_rate_hz: float | None
    quality_signals: list[str]  # associated quality/latency signals found


@dataclass
class DivergenceEvent:
    """A time window where one source diverged significantly from the reference."""
    source_name: str
    start_us: int
    end_us: int
    peak_translation_error_m: float
    peak_heading_error_rad: float
    phase: str | None          # from match-phases, if available
    likely_cause: str          # "wheel_slip" | "vision_outlier" | "impact" | "drift" | "unknown"


@dataclass
class SourceMetrics:
    """Per-source divergence statistics against the reference path."""
    source_name: str
    source_class: str          # "odometry" | "vision" | "accelerometer"
    translation_rms_m: float
    translation_max_m: float
    heading_rms_rad: float
    heading_max_rad: float
    sample_count: int
    coverage_fraction: float   # fraction of reference timeline covered
    confidence: str            # "high" | "medium" | "low"
```

---

## Public API for Other Analyzers

Other analyzers (especially `drive-analysis`) need a simple way to obtain chassis motion without re-implementing pose fusion. The module exports:

```python
from logreader.analyzers.pose_analysis import (
    PoseSample,
    PoseSource,
    DivergenceEvent,
    SourceMetrics,
    discover_pose_sources,
    build_reference_path,
    interpolate_pose_at,
    compute_velocity_at,
    compute_divergence_metrics,
)

# Discover what pose/vision/IMU signals exist in the log
sources: list[PoseSource] = discover_pose_sources(log_data)

# Build the best-estimate reference path (list of PoseSamples on a uniform timeline)
reference_path: list[PoseSample] = build_reference_path(log_data, sources)

# Query the reference path at a specific timestamp (interpolated)
pose: PoseSample | None = interpolate_pose_at(reference_path, timestamp_us)

# Get velocity at a timestamp (derived from reference path via finite differences)
vx, vy, omega = compute_velocity_at(reference_path, timestamp_us)

# Measure how a specific source compares to the reference
metrics: SourceMetrics = compute_divergence_metrics(
    source_samples, reference_path
)
```

---

## Implementation Phases

### Phase 1: Signal discovery and normalization

- Auto-detect candidate pose, gyro, vision, and acceleration signals using the patterns in [Data Observations](#data-observations).
- Parse them into normalized `PoseSample` lists.
- Report discovery results: source names, sample counts, timestamp ranges, rates, and any quality metadata found.
- This phase is useful on its own -- "what pose data exists in this log?" is a common question.

### Phase 2: Build best-estimate reference path

Because this is offline analysis, we are not restricted to causal filters.

#### v1 scope (initial implementation)

Weighted-average fusion with moving-average smoothing:

1. Resample all sources onto a common timeline at the highest available odometry rate.
2. At each timestep, compute a weighted average of available source poses:
   - Odometry weight: high (it's the backbone), but reduced during detected slip windows.
   - Vision weight: inversely proportional to latency-adjusted residual from the current odometry estimate. Zero if `tv == 0` or ambiguity is high.
   - Accelerometer: not used for position directly in v1 -- used only for acceleration consistency checks.
3. Apply a centered moving-average smooth (window ~50-100 ms) to remove high-frequency noise from the fused path.
4. Record the method, weights, and assumptions in the output.

This is deliberately simple. It requires no external dependencies beyond NumPy (already effectively required by the project).

#### v2 scope

- Replace the moving-average with a Rauch-Tung-Striebel (RTS) forward-backward Kalman smoother.
- Model odometry as the process model and vision as measurement updates.
- Improve per-camera trust weighting using tag count, distance, and ambiguity.

#### v3 scope

- Factor-graph-style offline optimization if v2 proves insufficient for complex multi-camera setups.

### Phase 3: Divergence measurement and reporting

With the reference path established, evaluate each source independently and produce the divergence report.

---

## Divergence Measurement and Analysis

With the best-estimate reference path established, evaluate each source independently.

### 1. Odometry divergence

- **Measurement:** Compare raw odometry-derived path against the reference path.
- **Useful metrics:** RMS translation error, RMS heading error, peak error, drift rate during vision-poor windows.
- **Likely insights:**
  - Gradual divergence indicates wheel diameter / track width / kinematics characterization issues.
  - Sudden divergence indicates wheel slip, collisions, or external disturbance.

### 2. Vision correlation, precision, and outliers

- **Measurement:** Compare each camera stream (and any combined vision stream) against the reference path with latency accounted for.
- **Useful metrics:** RMS pose error per camera, outlier count and duration, error vs tag distance / tag count / ambiguity.
- **Likely insights:**
  - Identify "vision jumps" that a real-time estimator should reject or down-weight.
  - Detect cameras that are consistently biased or noisy relative to others.
  - Produce concrete tuning guidance for vision standard deviations in `SwerveDrivePoseEstimator`.

### 3. Accelerometer consistency

- **Measurement:** Compare IMU acceleration against acceleration implied by the reference path (second derivative of position), after frame normalization.
- **Useful metrics:** Correlation coefficient during motion windows, noise floor during disabled/stationary windows, impact event detection.
- **Likely insights:**
  - Identify hard impacts or pushing not represented in the smoothed path.
  - Estimate process noise parameters for real-time filters.

---

## Vision Quality Gating

Vision should not be treated as uniformly trustworthy. The following gating rules apply when the needed metadata exists:

- **No target:** `tv == 0` -- weight = 0 (skip entirely)
- **High ambiguity:** ambiguity > 0.2 -- strongly down-weight
- **Impossible jump:** pose change exceeds physically plausible velocity (> ~5 m/s for FRC) -- reject
- **Single-tag far distance:** `ta < 0.5%` or estimated distance > 5 m with single tag -- down-weight
- **Multiple cameras:** score cameras separately before combining

If the log lacks vision quality metadata, the analyzer should note that only geometric residual checks (impossible-jump rejection) were available.

---

## Fallback Modes

| Available sources | Reference path quality | Notes |
|---|---|---|
| Odometry + vision + IMU | Best -- full fusion | All divergence metrics available |
| Odometry + vision | Good -- primary use case | Skip IMU consistency |
| Odometry + IMU | Lower confidence | Emphasize impact detection and drift rather than absolute correction |
| Vision + IMU | Limited | Report camera consistency and latency sensitivity |
| Single source only | No fused path | Report only discovery, self-consistency, and noise metrics |

---

## Confidence Scoring

Every major result carries a confidence level:

| Level | Criteria |
|-------|----------|
| `high` | >= 2 trustworthy source classes with strong temporal overlap; vision quality metadata available; sources agree during apparently normal driving |
| `medium` | 2 sources but with notable gaps, missing quality metadata, or moderate disagreement |
| `low` | Largely single-source, sparse overlap, or high unexplained disagreement |

---

## Deliverables

- **Discovery summary:** which pose/vision/IMU signals were found and used
- **Reference path summary:** time span, source weights, smoothing mode, confidence
- **Per-source divergence metrics:** RMS / max translation and heading error
- **Vision quality summary:** per-camera outlier counts, latency assumptions, confidence
- **Anomaly highlighting:** timestamps where traction was lost, vision failed, or impacts occurred
- **Tuning recommendations:** suggested standard-deviation values for `SwerveDrivePoseEstimator`

---

## `AnalysisResult.extra` Schema

The user-facing table can stay compact, but the full structured output should use proper dataclasses (matching the project convention):

```python
@dataclass
class PoseAnalysisExtra:
    confidence: str                      # "high" | "medium" | "low"
    assumptions: list[str]               # e.g. ["accel assumed gravity-compensated", ...]
    discovered_sources: list[PoseSource]
    reference_path_summary: dict         # method, weights, start_s, end_s
    source_metrics: list[SourceMetrics]
    divergence_events: list[DivergenceEvent]
    vision_camera_metrics: list[dict]    # per-camera: outlier_count, rms, latency_ms
    time_alignment_notes: list[str]      # any caveats about timestamp handling
```

---

## Relationship to Other Analyzers

- **`drive-analysis`** should consume this feature's `build_reference_path()` and `interpolate_pose_at()` when available, falling back to raw odometry otherwise.
- **`match-phases`** should be used to partition error metrics by disabled / auto / teleop where useful.
- **`hard-hits`** could cross-reference the accelerometer consistency data to corroborate impact detection.
- Future report-generation tools can render this as charts and phase-aware summaries.

---

## Edge Cases

- **No pose signals at all:** Report discovery failure and exit gracefully with a clear message.
- **Struct-typed poses:** Currently unsupported by the log reader (`SignalType.STRUCT`). Phase 1 should detect and report these signals even if it can't parse them yet, flagging them as "available but requires struct decoding."
- **Alliance-relative origins:** Limelight `botpose_wpiblue` vs `botpose_wpired` -- detect which is present and normalize.
- **Mid-match reboot:** Odometry resets to origin after reboot. Detect via `match-phases` `appears_post_reboot` and handle the origin discontinuity.
- **Very short logs / pit testing:** May only have one source or very sparse data -- fall back gracefully to single-source mode.

---

## Open Questions

- What real-log signal names appear for PhotonVision pose estimates? (Need a log from a PhotonVision team to validate patterns.)
- Are there common conventions for logging raw wheel module states (per-module velocity + angle) that could serve as a higher-fidelity odometry source than the fused pose?
- Should the v1 fusion algorithm use NumPy directly, or is there a lightweight Python Kalman filter library worth depending on?
