# Pose Correlation and Divergence Analysis Plan

This document outlines the strategy for analyzing robot poses by evaluating multiple odometry and localization sources. The primary goal is to retrospectively fuse these sources into a **best-estimate reference path** and then measure how each individual sensor modality (wheel odometry, vision, accelerometer) correlates or diverges from that path.

This should likely be implemented as both:

- a standalone `pose-analysis` analyzer for user-facing reports, and
- a shared helper/service layer used by other analyzers (especially `drive-analysis`) that need a single notion of actual robot motion.

## Goals

- Build the best offline estimate of robot pose available from the log.
- Measure drift, outliers, and disagreement for each source individually.
- Produce actionable tuning guidance for real-time estimators such as `SwerveDrivePoseEstimator`.
- Highlight time windows where one sensor was likely misleading while others remained consistent.

## Non-goals

- Claim true physical ground truth without an external reference system.
- Reconstruct a perfect pose when all sources are poor or missing.
- Replace WPILib estimators in robot code; this is an offline diagnostic and tuning tool.

## Input contract

The analyzer should explicitly separate **required**, **optional**, and **derived** inputs.

### Required for useful analysis

At least two of the following source classes must be present:

1. **Wheel odometry / fused robot pose**
   - Preferred: directly logged pose samples (`Pose2d`, x/y/theta triples, field pose arrays, or equivalent struct/raw shapes).
   - Acceptable fallback: chassis speeds + gyro heading if a pose must be reconstructed.
2. **Vision camera pose estimates**
   - One or more camera-provided robot poses, ideally field-relative AprilTag estimates.
3. **IMU / accelerometer data**
   - At minimum, planar acceleration (`ax`, `ay`) or equivalent.

If only one source class is present, the analyzer should still report signal discovery results and basic self-consistency checks, but should mark fusion/divergence results as low-confidence or unavailable.

### Strongly preferred optional inputs

- Vision latency per update or timestamp of image capture
- Vision quality signals (ambiguity, tag count, reprojection error, target distance)
- Gyro yaw / yaw rate
- Chassis speeds (`vx`, `vy`, `omega`)
- Match phases from `match-phases`
- Controller-estimator outputs already published by the robot code (useful for comparison, not as authoritative truth)

## Source characteristics

1. **Wheel odometry + gyro**
   - High frequency, locally precise.
   - Prone to long-term drift and sudden divergence during wheel slip, impacts, or poor characterization.
2. **Vision camera(s)**
   - Lower frequency and often bursty.
   - Globally informative but vulnerable to noise, ambiguity, latency, and occasional gross outliers.
3. **Accelerometer (IMU)**
   - Very high frequency.
   - Useful for short-term dynamics, impacts, and detecting motion changes.
   - Not reliable as a standalone long-term position source due to integration drift.

## Coordinate frames and normalization

Before comparing sources, the analyzer must normalize them into a common representation.

### Canonical reference frame

- Position: field-relative `x`, `y` in meters
- Heading: field-relative `theta` in radians or degrees (pick one and document it consistently)
- Velocity: field-relative by default for cross-source comparison
- Acceleration: both raw IMU-frame and transformed field-frame values may be useful, but the comparison contract must be explicit

### Required normalization steps

- Normalize heading wraparound (`-π..π` or `0..2π`, consistently)
- Track whether accelerometer values are robot-relative, gravity-compensated, or vendor-filtered
- Align units across pose sources
- If multiple cameras exist, preserve both per-camera streams and an optional combined vision stream
- Handle timestamp alignment carefully when vision timestamps refer to capture time rather than publish time

If frame assumptions cannot be validated from the signals alone, the analyzer should record them in its output as explicit assumptions.

## Shared helper layer

Implementation will be much simpler if this feature first introduces reusable utilities for:

- pose signal discovery
- converting heterogeneous signals into a `PoseSample`-like internal representation
- interpolation/resampling onto a shared timeline
- latency correction and timestamp shifting
- comparing trajectories and reporting drift metrics

This matches the project pattern already used by `match-phases`, which acts as both an analyzer and a reusable service.

## Implementation phases

### Phase 1: Signal discovery and normalization

- Auto-detect candidate pose, gyro, vision, and acceleration signals.
- Parse them into normalized internal samples.
- Report discovery results, sample counts, timestamp ranges, and any frame/quality metadata found.

### Phase 2: Build best-estimate reference path

Because this is offline analysis, we are not restricted to causal filters like the standard `SwerveDrivePoseEstimator`. However, the first implementation should stay simple and robust.

#### v1 scope

- Resample sources onto a common timeline.
- Use weighted fusion and smoothing to create a **best-estimate reference path**.
- Prefer a practical weighted/blended approach over a heavy optimization stack.

#### v2 scope

- Add non-causal smoothing such as a Rauch-Tung-Striebel smoother or related backward pass.
- Improve latency handling and per-camera trust weighting.

#### v3 scope

- Consider more advanced offline optimization / factor-graph-style reconstruction if the simpler approach proves insufficient.

The key design point is that v1 should be implementable with the current project scope and common Python dependencies.

## Divergence measurement and analysis

With the best-estimate reference path established, evaluate each source independently.

### 1. Odometry divergence

- **Measurement:** Compare raw odometry-derived path against the reference path.
- **Useful metrics:**
  - RMS translation error
  - RMS heading error
  - peak error
  - drift rate during vision-poor windows
- **Likely insights:**
  - gradual divergence indicates wheel diameter / track width / kinematics characterization issues
  - sudden divergence indicates wheel slip, collisions, or external disturbance

### 2. Vision correlation, precision, and outliers

- **Measurement:** Compare each camera stream, and any combined vision stream, against the reference path with latency accounted for.
- **Useful metrics:**
  - RMS pose error per camera
  - outlier count and outlier duration
  - error vs tag distance / tag count / ambiguity when those signals exist
- **Likely insights:**
  - identify “vision jumps” that a real-time estimator should reject or down-weight
  - detect cameras that are consistently biased or noisy relative to others
  - produce concrete tuning guidance for vision standard deviations in the real-time estimator

### 3. Accelerometer consistency

- **Measurement:** Compare IMU acceleration against acceleration implied by the reference path, after frame normalization.
- **Useful metrics:**
  - correlation coefficient during motion windows
  - noise floor during disabled or stationary windows
  - impact event detection
- **Likely insights:**
  - identify hard impacts or pushing not well represented in the smoother path
  - estimate process noise parameters for real-time filters

## Vision quality gating

Vision should not be treated as uniformly trustworthy. The analyzer should include gating/down-weighting rules where the needed metadata exists.

Examples:

- reject or strongly down-weight samples with high ambiguity
- reject impossible jumps beyond configured velocity/acceleration thresholds
- down-weight single-tag far-distance solutions
- score cameras separately before combining them into a shared vision contribution

If the log lacks vision quality metadata, the analyzer should explicitly note that only geometric residual checks were available.

## Fallback modes

The analyzer should remain useful even when some inputs are missing.

### Odometry + vision, no accelerometer

- Build the reference path from odometry and vision only.
- Skip IMU consistency metrics.

### Odometry + accelerometer, no vision

- Build a short-horizon smoothed path with lower confidence.
- Emphasize impact detection, drift behavior, and self-consistency rather than absolute correction.

### Vision + accelerometer, no odometry

- Usually lower utility, but still report alignment, latency sensitivity, and camera consistency.

### Single-source only

- Do not claim a fused reference path.
- Report only self-consistency, noise, and discovery information.

## Confidence scoring

Every major result should carry a confidence level derived from:

- number of independent source classes available
- sample coverage overlap across time
- vision quality metadata availability
- timestamp alignment certainty
- agreement between sources during apparently normal-driving windows

Suggested buckets:

- `high`: strong overlap and at least two trustworthy source classes
- `medium`: usable but with notable gaps or assumptions
- `low`: sparse, weakly aligned, or largely single-source

## Deliverables

- **Discovery summary:** which pose/vision/IMU signals were found and used
- **Reference path summary:** time span, source weights, smoothing mode, confidence
- **Divergence metrics:** RMS / max translation and heading error for each source
- **Vision quality summary:** per-camera outlier counts, latency assumptions, confidence
- **Anomaly highlighting:** timestamps where traction was lost, vision failed, or impacts occurred
- **Tuning recommendations:** suggested weighting and standard-deviation guidance for real-time estimators

## Suggested `AnalysisResult.extra` schema

The user-facing table can stay compact, but the full structured output should expose enough detail for future analyzers.

```python
{
    "confidence": "high|medium|low",
    "assumptions": [...],
    "discovered_sources": {
        "odometry": [...],
        "vision": [...],
        "accelerometer": [...],
        "gyro": [...],
    },
    "reference_path_summary": {
        "start_s": ...,
        "end_s": ...,
        "method": "weighted_blend|smoothed_blend|rts_smoother|other",
        "weights": {...},
    },
    "source_metrics": [
        {
            "source_name": ...,
            "source_type": "odometry|vision|accelerometer",
            "translation_rms_m": ...,
            "translation_max_m": ...,
            "heading_rms_deg": ...,
            "heading_max_deg": ...,
            "confidence": "high|medium|low",
        }
    ],
    "vision_camera_metrics": [...],
    "divergence_events": [...],
    "time_alignment_notes": [...],
}
```

## Relationship to other analyzers

- `drive-analysis` should consume this feature’s best-estimate chassis motion when available.
- `match-phases` should be used to partition error metrics by disabled / auto / teleop where useful.
- Future report-generation tools can render this as charts and phase-aware summaries.
