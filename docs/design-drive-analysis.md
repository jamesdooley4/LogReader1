# Drive Motor Discovery and Traction Analysis — Design Document

> Analyzer: `drive-analysis` · Module: `src/logreader/analyzers/drive_analysis.py`

**Status:** Design complete, not yet implemented
**Depends on:** `pose-analysis` for best-estimate chassis motion; optional `match-phases` for phase-aware breakdown
**Validated against:** *(needs validation against real logs — see [Open Questions](#open-questions))*

Automatically discover swerve drive and steer motors, then perform traction and current-limit analysis by comparing wheel behavior against actual chassis motion.

---

## Overview

This analyzer builds a pipeline that:

1. Discovers candidate motor-controller signals.
2. Scores each motor as likely drive (propulsion) or steer (azimuth).
3. Optionally clusters motors into four swerve modules.
4. Compares wheel behavior against a chassis-motion reference to detect grip-limited, supply-current-limited, and stator-current-limited intervals.

The intended outcome is not merely "find the four highest-current motors," but to produce a scored, explained classification with confidence levels, and flag specific intervals of limiting behavior with supporting evidence.

---

## Goals

- Auto-detect likely propulsion and steer motors without requiring manual mapping.
- Use pose/IMU evidence to distinguish normal wheel behavior from traction loss.
- Detect likely current limiting behavior with confidence levels, not just hard-coded thresholds.
- Produce structured outputs that can be consumed by future multi-match or report-generation features.

## Non-goals

- Guarantee perfect motor identification in every robot codebase with zero ambiguity.
- Infer exact drivetrain geometry from logs alone in all cases.
- Replace hardware configuration knowledge when teams already know their CAN layout; explicit config can still be a useful future override.
- Support non-swerve drivetrains in v1 (tank and mecanum could be added later, but swerve is the primary target).

---

## Data Observations

### Motor controller signal naming conventions

FRC motor controllers publish telemetry under vendor-specific naming conventions. The analyzer must auto-detect from multiple patterns.

**CTRE Phoenix 6 (TalonFX, CANcoder, Pigeon2):**

| Signal pattern | Type | Meaning |
|----------------|------|---------|
| `Device/TalonFX */Velocity` | `double` | Motor velocity (RPS by default) |
| `Device/TalonFX */Position` | `double` | Motor position (rotations) |
| `Device/TalonFX */SupplyCurrent` | `double` | Supply current (amps) |
| `Device/TalonFX */StatorCurrent` | `double` | Stator current (amps) |
| `Device/TalonFX */DutyCycle` | `double` | Applied duty cycle (-1 to 1) |
| `Device/TalonFX */MotorVoltage` | `double` | Applied voltage |
| `Device/TalonFX */ClosedLoopOutput` | `double` | PID output |
| `Device/TalonFX */DeviceTemp` | `double` | Temperature (deg C) |

Device names typically include a CAN ID or user-assigned name, e.g. `Device/TalonFX 1/Velocity` or `Device/TalonFX DriveFL/Velocity`.

**REV SparkMax / SparkFlex:**

| Signal pattern | Type | Meaning |
|----------------|------|---------|
| `*/SparkMax[*]/Velocity` | `double` | Motor velocity (RPM) |
| `*/SparkMax[*]/Position` | `double` | Motor position (rotations) |
| `*/SparkMax[*]/Output Current` | `double` | Output current (amps) |
| `*/SparkMax[*]/Applied Output` | `double` | Applied duty cycle (-1 to 1) |
| `*/SparkMax[*]/Bus Voltage` | `double` | Bus voltage |

**Team-published NT signals (common conventions):**

| Signal pattern | Type | Meaning |
|----------------|------|---------|
| `*/Drive*/Velocity` or `*/drive*/speed` | `double` | Drive motor speed |
| `*/Steer*/Angle` or `*/steer*/position` | `double` | Steer motor angle |
| `*/Module*/driveVelocity` | `double` | Per-module drive speed |
| `*/Module*/steerAngle` | `double` | Per-module steer angle |
| `SwerveStates/*` or `SwerveModules/*` | various | AdvantageScope-style module states |

### Expected motor counts and roles

A swerve drivetrain has **8 motors**: 4 drive (propulsion) + 4 steer (azimuth). A typical log will show:

- **Drive motors:** sustained current during motion, continuous velocity correlated with robot translational speed, moderate-to-high duty cycle during enabled phases.
- **Steer motors:** brief current spikes during direction changes, velocity profile that is discontinuous (move to target angle, then hold), lower average current.
- **Non-drive motors:** intake, shooter, elevator, climber — these can sometimes draw high current but will NOT correlate with chassis translational speed.

### Differentiating drive from mechanism motors

The key discriminator is **correlation with chassis motion**, not just current magnitude. A shooter flywheel can draw 60+ A sustained, but its velocity will not correlate with the robot's translational speed. The scoring approach below captures this.

---

## Input Contract

### Required for useful auto-detection

- Candidate motor signals for multiple controllers, ideally including some combination of:
  - velocity
  - position
  - duty cycle / applied output / voltage command
  - supply current
  - stator current

### Required for traction analysis

- Some estimate of robot motion:
  - **Preferred:** best-estimate reference path from `pose-analysis` (`build_reference_path()`)
  - **Fallback:** odometry pose / chassis speeds / gyro
- Optionally accelerometer data for higher-confidence acceleration checks

### Strongly preferred optional inputs

- PDH total current / voltage
- Battery voltage / brownout-related signals
- Enabled/disabled state and match phases
- Controller-specific metadata that reveals configured current limits

If the log lacks enough controller telemetry to distinguish motor roles, the analyzer should still report candidate signals, weak rankings, and what evidence is missing.

---

## Dependencies and Shared Services

This analyzer depends on:

- **`pose-analysis`** — for the best-estimate chassis motion reference. If unavailable, falls back to raw odometry with reduced confidence.
- **`match-phases`** — for phase-aware analysis. Optional but strongly preferred.

The interface with `pose-analysis` is:

```python
from logreader.analyzers.pose_analysis import (
    discover_pose_sources,
    build_reference_path,
    interpolate_pose_at,
    compute_velocity_at,
)

# Get chassis motion reference
sources = discover_pose_sources(log_data)
reference_path = build_reference_path(log_data, sources)

# At any timestamp, get actual chassis velocity
vx, vy, omega = compute_velocity_at(reference_path, timestamp_us)
chassis_speed = sqrt(vx**2 + vy**2)
```

If `pose-analysis` is not available or produces a `low`-confidence path, the analyzer should:
1. Attempt to find a raw odometry pose signal directly.
2. Derive velocity from pose finite differences.
3. Mark traction analysis confidence as reduced.

---

## Implementation Pipeline

### Phase 1: Identify candidate motors

- Scan signals for likely motor-controller namespaces using the patterns in [Data Observations](#data-observations).
- Group per-controller signals together where naming patterns imply they belong to the same device (e.g. all `Device/TalonFX 1/*` signals form one candidate).
- Build a `MotorCandidate` record for each motor/controller with all available telemetry.

```python
@dataclass
class MotorCandidate:
    """A discovered motor controller with its available telemetry."""
    name: str                     # e.g. "TalonFX 1" or "SparkMax[3]"
    vendor: str                   # "ctre" | "rev" | "unknown"
    signals: dict[str, str]       # role -> signal name, e.g. {"velocity": "Device/TalonFX 1/Velocity"}
    has_velocity: bool
    has_position: bool
    has_supply_current: bool
    has_stator_current: bool
    has_duty_cycle: bool
    sample_count: int             # from the most-sampled signal
    active_fraction: float        # fraction of enabled time with non-trivial output
```

Each candidate should note what telemetry exists, because confidence depends heavily on whether velocity/current/output are all available.

### Phase 2: Score motors as drive vs steer

Compute a feature vector per candidate motor, then rank them.

**Feature definitions:**

| Feature | Computation | Drive tendency | Steer tendency |
|---------|-------------|----------------|----------------|
| `sustained_current` | P75 of supply or stator current during enabled | High | Low-moderate |
| `current_burstiness` | (P99 - P50) / P50 of current | Low-moderate | High |
| `velocity_continuity` | Fraction of enabled time with velocity > threshold and changing smoothly | High | Low |
| `translation_correlation` | Pearson r between abs(motor velocity) and chassis translational speed | High (> 0.7) | Low (< 0.3) |
| `rotation_correlation` | Pearson r between motor velocity changes and chassis angular velocity | Low-moderate | High |
| `output_saturation` | Fraction of time abs(duty_cycle) > 0.9 | Moderate | Low |
| `active_duty` | Fraction of enabled time the motor is meaningfully active | High | Moderate |

**Scoring algorithm (v1):**

```
drive_score = w1 * sustained_current_percentile
            + w2 * velocity_continuity
            + w3 * translation_correlation
            - w4 * current_burstiness
            - w5 * rotation_correlation

steer_score = w1 * rotation_correlation
            + w2 * current_burstiness
            - w3 * translation_correlation
            - w4 * velocity_continuity
```

Where `w1..w5` are tunable weights (initial values chosen empirically, documented as defaults).

**Classification rules:**

1. Rank all candidates by `drive_score`. The top 4 with `drive_score > threshold` are candidate drive motors.
2. Rank remaining candidates by `steer_score`. The top 4 with `steer_score > threshold` are candidate steer motors.
3. If `translation_correlation` is unavailable (no chassis motion reference), rely on current/velocity features only — mark confidence as `medium` at best.
4. If fewer than 4 strong drive candidates are found, report what was found and flag the gap.

The analyzer should produce ranked candidates with scores, not just a binary answer.

### Phase 3: Optional module clustering

If naming conventions and correlations permit, attempt to cluster motors into four swerve modules:

- **Naming heuristic:** `TalonFX 1` and `TalonFX 2` with IDs differing by 1 are likely a drive/steer pair. Similarly, `DriveFL`/`SteerFL` patterns.
- **Timing correlation:** a drive motor and its paired steer motor will have correlated activity windows (both active when that module is being used).
- **Spatial plausibility:** if module positions can be inferred (e.g. FL, FR, BL, BR from names), verify that all four modules show consistent translational contribution.

This should be optional and best-effort; classification should remain useful even if module pairing is inconclusive.

### Phase 4: Establish chassis-motion reference

To diagnose traction loss, compare wheel behavior against actual robot movement.

| Priority | Source | How to obtain |
|----------|--------|---------------|
| 1 | `pose-analysis` reference path | `build_reference_path()` |
| 2 | Raw odometry pose | Direct signal lookup |
| 3 | Odometry + gyro | Chassis speeds + heading integration |
| 4 | Gyro only | Heading-only reference (rotation analysis only) |

The analyzer should record which reference source it used.

---

## Speed Calibration Modes

Comparing wheel-derived speed to chassis speed requires handling unknown scaling.

### Calibrated mode

Use known or configured values:

- Wheel radius / diameter
- Drive gear ratio
- Motor velocity unit scaling (e.g. TalonFX default is rotations/sec, SparkMax is RPM)

This allows direct computation: `wheel_surface_speed = motor_velocity * gear_ratio * wheel_circumference`.

### Relative mode (default when geometry is unknown)

Infer a scale factor during apparently normal, non-slip windows:

1. Identify windows where all drive motors are running at similar scaled velocities AND the chassis is translating smoothly (low jerk, consistent heading rate).
2. Compute `scale_factor = median(chassis_speed / motor_velocity)` across these windows.
3. Apply this factor for subsequent slip detection.

The key insight is that **we don't need absolute speed to detect slip** — we only need to detect when the ratio of wheel speed to chassis speed changes significantly from its baseline.

---

## Traction and Current Limit Analysis

Once likely drive motors are identified, analyze their behavior in conjunction with chassis motion.

### 1. Grip-limited (traction loss)

#### Condition

One or more wheels appear to be spinning or producing torque inconsistent with the robot's actual motion.

#### Detection algorithm

```
For each drive motor at each timestep:
    wheel_speed = motor_velocity * scale_factor
    chassis_speed = sqrt(vx^2 + vy^2)  # from reference path
    slip_ratio = (wheel_speed - chassis_speed) / max(chassis_speed, epsilon)

    If slip_ratio > SLIP_THRESHOLD (e.g. 0.3 = 30% excess):
        Flag as potential traction loss
```

#### Sub-classification

- **Whole-robot slip:** Most or all drive motors show excess wheel speed relative to chassis motion (e.g. aggressive acceleration on a slippery surface).
- **Partial slip / wheel unload:** Only a subset of drive motors diverge strongly, often accompanied by unexpected yaw (e.g. one wheel lifted during a collision, carpet seam).
- **Impact / shove event:** Chassis motion diverges from wheel expectations, but accelerometer shows a large impulse — suggests external force rather than self-induced wheel spin.

#### Evidence weighting

| Evidence | Weight | Notes |
|----------|--------|-------|
| slip_ratio > 0.3 for drive motor | Primary | Direct wheel-vs-chassis comparison |
| High current + low chassis acceleration | Supporting | Motor is working hard but robot isn't moving |
| Accelerometer spike | Supporting | Distinguishes impact from self-induced slip |
| Odometry divergence from vision | Supporting | Cross-validates with `pose-analysis` |
| Subset of motors diverge | Differentiating | Partial slip vs whole-robot slip |

### 2. Supply-current-limited

#### Condition

The motor/controller appears constrained by supply current availability or a configured supply current limit.

#### Detection algorithm

```
For each drive motor:
    Find intervals where:
        supply_current is within 2A of a stable plateau for > 50 ms
        AND abs(duty_cycle) > 0.85 (motor is asking for more)
        AND motor velocity is NOT decreasing (rules out back-EMF limiting)

    Cluster plateau levels across all events for this motor.
    If a single plateau level appears repeatedly (e.g. 38-42A seen 10+ times):
        Likely configured supply current limit = ~40A
        Confidence: high

    If plateau levels vary but duty_cycle is consistently maxed:
        Probable supply limiting (battery sag or main breaker proximity)
        Confidence: medium
```

#### Important note

Do not rely only on common nominal thresholds such as 40 A or 60 A. Teams may configure different limits, and battery sag can create plateau-like behavior without a strict software clamp. The analyzer should **discover** the effective limit from the data rather than assuming it.

### 3. Stator-current-limited

#### Condition

The motor torque output appears constrained by a stator current limit, commonly during heavy load at low speed.

#### Detection algorithm

```
For each drive motor:
    Find intervals where:
        stator_current is within 3A of a stable plateau for > 50 ms
        AND abs(motor_velocity) < LOW_SPEED_THRESHOLD
        AND duty_cycle or voltage command remains high

    Cluster plateau levels.
    If a single plateau level appears repeatedly (e.g. 78-82A):
        Likely configured stator current limit = ~80A
        Confidence: high
```

This often occurs during:

- Initial acceleration from standstill
- Pushing matches / defense contact
- Obstruction or pinning

As with supply limits, report confidence rather than treating every high-current plateau as definitive proof.

---

## Data Models

```python
@dataclass
class MotorScore:
    """Scoring result for a single motor candidate."""
    motor: MotorCandidate
    drive_score: float
    steer_score: float
    features: dict[str, float]    # individual feature values
    likely_role: str              # "drive" | "steer" | "mechanism" | "unknown"
    confidence: str               # "high" | "medium" | "low"


@dataclass
class SwerveModule:
    """A paired drive + steer motor forming one swerve module."""
    name: str                     # e.g. "FL", "Module 0", etc.
    drive_motor: MotorScore
    steer_motor: MotorScore
    pairing_confidence: str       # "high" | "medium" | "low"
    pairing_evidence: str         # why these two were paired


@dataclass
class TractionEvent:
    """A detected interval of abnormal traction behavior."""
    start_us: int
    end_us: int
    event_type: str               # "whole_robot_slip" | "partial_slip" | "impact"
    affected_motors: list[str]
    peak_slip_ratio: float
    phase: str | None
    supporting_evidence: list[str]
    confidence: str


@dataclass
class CurrentLimitEvent:
    """A detected interval of current limiting."""
    start_us: int
    end_us: int
    motor_name: str
    limit_type: str               # "supply" | "stator"
    plateau_amps: float
    inferred_limit: float | None  # estimated configured limit, if determinable
    phase: str | None
    confidence: str               # "confirmed" | "probable"
```

---

## Confidence Model

Every major classification carries a confidence value.

| Classification | `high` criteria | `medium` criteria | `low` criteria |
|---|---|---|---|
| Motor role (drive/steer) | >= 5 telemetry channels, translation_correlation > 0.7, clear separation from other candidates | 3+ channels, moderate correlation, some separation | Few channels, no correlation data |
| Traction loss | slip_ratio > 0.5, accelerometer + odometry corroborate, pose-analysis reference is `high` confidence | slip_ratio > 0.3, some corroboration | slip_ratio marginal, no corroboration |
| Supply current limit | Repeated plateau at same level, duty cycle maxed | Plateau observed but not repeated or duty cycle ambiguous | Single event only |
| Stator current limit | Repeated plateau at same level, low speed confirmed | Plateau observed but speed data missing | Single event only |

---

## Output Model

### User-facing summary

- Likely drive motors (ranked with scores)
- Likely steer motors (ranked with scores)
- Confidence of classification
- Swerve module pairings (if determined)
- Number and duration of grip-limited, supply-limited, and stator-limited intervals
- Inferred current limits per motor (if determinable)
- Strongest evidence windows for each limiting category

### `AnalysisResult.extra` Schema

```python
@dataclass
class DriveAnalysisExtra:
    confidence: str                      # overall "high" | "medium" | "low"
    assumptions: list[str]               # e.g. ["scale factor inferred (relative mode)"]
    reference_motion_source: dict        # {"type": "pose-analysis|odometry|...", "confidence": "..."}
    scaling_mode: str                    # "calibrated" | "relative"
    candidate_motors: list[MotorScore]
    module_clusters: list[SwerveModule]  # empty if clustering was inconclusive
    traction_events: list[TractionEvent]
    supply_limit_events: list[CurrentLimitEvent]
    stator_limit_events: list[CurrentLimitEvent]
    inferred_limits: dict[str, dict]     # motor_name -> {"supply_a": ..., "stator_a": ...}
```

---

## Fallback Behavior

| Telemetry available | Pose reference | Result |
|---|---|---|
| Rich controller telemetry (velocity + current + output) | `high`/`medium` confidence reference path | Full classification and event detection |
| Rich controller telemetry | No pose reference or `low` confidence | Classify drive vs steer with reduced confidence (no translation_correlation); detect current-limited intervals; reduce confidence on grip-limited conclusions |
| Velocity + current only (no output signal) | Any | Classification works; current limit detection works but cannot confirm duty-cycle saturation |
| Sparse telemetry (e.g. current only) | Any | Report candidate rankings and missing evidence; do not overclaim |
| No motor signals found | Any | Report discovery failure and exit gracefully |

---

## Edge Cases

- **Non-swerve drivetrains:** The 4-drive + 4-steer assumption won't hold. The analyzer should detect if the candidate count doesn't match expectations and note it. Future versions could support tank (2 or 4 drive, 0 steer) and mecanum (4 drive, 0 steer).
- **Mechanism motors that look like drive motors:** A shooter flywheel draws high sustained current. The `translation_correlation` feature is the key discriminator — it will be low for mechanisms. Without a chassis motion reference, this disambiguation is weaker.
- **Brownout / reboot splits:** Motor signals may restart mid-log. Detect via timestamp gaps or `match-phases` `appears_post_reboot`.
- **Mixed vendors:** A robot may use TalonFX for drive and SparkMax for steer (or vice versa). The signal grouping logic must handle mixed naming conventions.
- **Custom signal names:** Teams that publish motor telemetry under non-standard NT paths. The analyzer should report unclassified signals so the user can see what was missed.
- **Disabled periods:** Motor current and velocity during disabled should be excluded from scoring (motors are not commanded).

---

## Relationship to Pose Analysis

This analyzer should not independently redefine "actual robot motion" when a stronger shared result already exists.

- `pose-analysis` provides the preferred best-estimate chassis path via `build_reference_path()` and `compute_velocity_at()`.
- `drive-analysis` consumes that path and focuses on motor-role discovery and traction/current interpretation.
- If `pose-analysis` is unavailable, `drive-analysis` falls back to raw odometry — but should clearly note the reduced confidence.

That separation avoids duplicated logic and keeps the motion-reference contract consistent across analyzers.

---

## Open Questions

- What are the exact signal naming patterns for REV SparkFlex? (Similar to SparkMax but may differ in details.)
- Should the scoring weights be user-configurable via CLI arguments, or are fixed defaults sufficient for v1?
- Is there a reliable way to detect configured current limits from logged controller status signals (e.g. CTRE `SupplyCurrentLimitEn`, `StatorCurrentLimit`)?
- How should the analyzer handle swerve module states logged as struct arrays (e.g. `SwerveModuleState[]`) — this would require struct decoding support first.
