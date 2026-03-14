# Drive Motor Discovery and Traction Analysis Plan

This document outlines the plan for a new analyzer (or set of analyzers) to automatically discover swerve drive motors and perform advanced traction and current limit analysis based on telemetry, poses, and accelerometer data.

The intended outcome is not merely “find the four highest-current motors,” but to build a robust pipeline that:

- identifies likely drive vs steer motors,
- explains the evidence behind that classification,
- measures how wheel behavior relates to actual chassis motion, and
- flags intervals where the robot appears grip-limited, supply-current-limited, or stator-current-limited.

## Goals

- Auto-detect likely propulsion and steer motors without requiring manual mapping.
- Use pose/IMU evidence to distinguish normal wheel behavior from traction loss.
- Detect likely current limiting behavior with confidence levels, not just hard-coded thresholds.
- Produce structured outputs that can be consumed by future multi-match or report-generation features.

## Non-goals

- Guarantee perfect motor identification in every robot codebase with zero ambiguity.
- Infer exact drivetrain geometry from logs alone in all cases.
- Replace hardware configuration knowledge when teams already know their CAN layout; explicit config can still be a useful future override.

## Dependencies and shared services

This analyzer should depend on shared pose utilities or the future `pose-analysis` service where possible.

In particular, it benefits from:

- best-estimate chassis motion from `pose-analysis`
- match-phase segmentation from `match-phases`
- future signal-parsing helpers for motor controller naming conventions

If pose-analysis output is unavailable, this analyzer should fall back to odometry-only or odometry-plus-IMU motion estimates with reduced confidence.

## Input contract

### Required for useful auto-detection

- candidate motor signals for multiple controllers, ideally including some combination of:
  - velocity
  - position
  - duty cycle / applied output / voltage command
  - supply current
  - stator current

### Required for traction analysis

- some estimate of robot motion:
  - preferred: best-estimate reference path from `pose-analysis`
  - fallback: odometry pose / chassis speeds / gyro
- optionally accelerometer data for higher-confidence acceleration checks

### Strongly preferred optional inputs

- PDH total current / voltage
- battery voltage / brownout-related signals
- enabled/disabled state and match phases
- controller-specific metadata that reveals configured current limits

If the log lacks enough controller telemetry to distinguish motor roles, the analyzer should still report candidate signals, weak rankings, and what evidence is missing.

## Implementation pipeline

The analyzer should be structured as a staged classification and diagnosis pipeline.

### Phase 1: Identify candidate motors

- Scan signals for likely motor-controller namespaces and telemetry fields.
- Group per-controller signals together where naming patterns imply they belong to the same device.
- Build a candidate record for each motor/controller with all available telemetry.

Each candidate should note what telemetry exists, because confidence depends heavily on whether velocity/current/output are all available.

### Phase 2: Score motors as drive vs steer

Rather than relying on a single heuristic, compute a feature vector per candidate motor.

Suggested features:

- **sustained current score**: percentile-based measure of prolonged current draw
- **current burstiness score**: brief spikes followed by idle/hold behavior suggest steer
- **velocity continuity score**: continuous velocity over long motion windows suggests drive
- **translation correlation score**: correlation between motor velocity and chassis translational speed
- **rotation correlation score**: correlation between motor position/velocity changes and chassis angular velocity
- **output saturation fraction**: fraction of time output is near commanded maximum
- **active-duty fraction**: fraction of enabled time the motor is meaningfully active

Expected tendencies:

- **Drive motors**: high sustained current, high translation correlation, more continuous velocity profiles
- **Steer motors**: lower sustained current, burstier motion, stronger association with sudden heading/path-curvature changes

The analyzer should produce ranked drive and steer candidates with confidence, not just a binary answer.

### Phase 3: Optional module clustering

If naming conventions and correlations permit, attempt to cluster motors into four swerve modules by associating:

- one likely drive motor and one likely steer motor per module
- similar activity timing
- spatially plausible rotational/translational contribution patterns

This should be optional and best-effort; classification should remain useful even if module pairing is inconclusive.

### Phase 4: Establish chassis-motion reference

To diagnose traction loss, compare wheel behavior against actual robot movement.

Preferred reference:

- offline best-estimate path from `pose-analysis`

Fallback references:

- odometry-only translated/rotated motion
- odometry + gyro
- short-horizon pose + IMU consistency checks

The analyzer should record which reference source it used.

## Speed calibration modes

Comparing wheel-derived speed to chassis speed requires handling unknown scaling carefully.

### Calibrated mode

Use known or configured values when available:

- wheel radius / diameter
- drive gear ratio
- velocity-unit scaling from controller telemetry

This mode allows more direct interpretation of theoretical wheel speed vs measured chassis speed.

### Relative mode

If wheel geometry/scaling is unknown, infer a mapping during apparently normal, non-slip windows.

Examples:

- fit a scale factor between candidate drive-motor velocity and observed chassis translational speed
- use the most self-consistent intervals to learn expected relationships

This mode is less absolute, but still highly useful for identifying divergence and anomalies.

## Traction and current limit analysis

Once likely drive motors are identified, analyze their behavior in conjunction with chassis motion.

### 1. Grip-limited (traction loss)

#### Condition

One or more wheels appear to be spinning or producing torque inconsistent with the robot’s actual motion.

#### Evidence to consider

- wheel-derived speed significantly exceeds chassis speed
- high drive output / current with weak corresponding acceleration
- disagreement between left/right or per-module behavior and resulting chassis twist
- sudden odometry divergence relative to the pose-analysis reference path
- accelerometer indicates less acceleration than wheel activity predicts

#### Classification guidance

- **whole-robot slip**: most or all drive motors show excess wheel speed relative to chassis motion
- **partial slip / wheel unload**: only a subset of drive motors diverge strongly, often accompanied by unexpected yaw
- **impact / shove event**: chassis motion diverges from wheel expectations, but signatures suggest external force rather than self-induced wheel spin

### 2. Supply-current-limited

#### Condition

The motor/controller appears constrained by supply current availability or a configured supply current limit.

#### Evidence to consider

- supply current plateaus while command/output demand remains high
- repeated plateau levels across multiple events suggest a configured limit
- PDH total current or system voltage behavior supports available-power constraints
- motor velocity or acceleration stops increasing despite continued demand

#### Important note

Do not rely only on common nominal thresholds such as 40 A or 60 A. Teams may configure different limits, and battery sag can create plateau-like behavior without a strict software clamp.

The analyzer should distinguish between:

- **confirmed configured-limit behavior**: controller config or unmistakable repeated plateau evidence exists
- **probable supply limiting**: strong telemetry evidence but no explicit config confirmation

### 3. Stator-current-limited

#### Condition

The motor torque output appears constrained by a stator current limit, commonly during heavy load at low speed.

#### Evidence to consider

- stator current plateaus at a repeated threshold
- low or moderate wheel speed but high torque demand
- pushing or acceleration-from-rest signatures
- output demand remains high while motor response flattens

This often occurs during:

- initial acceleration
- pushing matches
- obstruction / pinning / defense contact

As with supply limits, report confidence rather than treating every high-current plateau as definitive proof.

## Confidence model

Every major classification should carry a confidence value.

Suggested inputs to confidence:

- how many telemetry channels exist per candidate motor
- whether pose-analysis produced a medium/high-confidence chassis reference
- whether scaling is calibrated or only relative
- whether evidence repeats across multiple events
- how separable the drive-vs-steer score distributions are

Suggested buckets:

- `high`: rich telemetry, consistent evidence, repeatable patterns
- `medium`: workable but with missing channels or ambiguous grouping
- `low`: sparse signals or weakly separable behavior

## Output model

The analyzer should report both candidate classification and event-level diagnostics.

### User-facing summary

- likely drive motors
- likely steer motors
- confidence of classification
- number and duration of grip-limited, supply-limited, and stator-limited intervals
- strongest evidence windows for each category

### Suggested `AnalysisResult.extra` schema

```python
{
    "confidence": "high|medium|low",
    "assumptions": [...],
    "reference_motion_source": {
        "type": "pose-analysis|odometry|odometry+imu|other",
        "confidence": "high|medium|low",
    },
    "candidate_motors": [
        {
            "name": ...,
            "signals": {...},
            "role_scores": {
                "drive": ...,
                "steer": ...,
            },
            "confidence": "high|medium|low",
        }
    ],
    "module_clusters": [...],
    "traction_loss_events": [...],
    "supply_limit_events": [...],
    "stator_limit_events": [...],
    "scaling_mode": "calibrated|relative",
}
```

## Fallback behavior

### Rich controller telemetry + good pose reference

- full classification and event detection

### Good motor telemetry + weak pose reference

- classify drive vs steer with medium confidence
- detect likely current-limited intervals
- reduce confidence on grip-limited conclusions

### Sparse motor telemetry

- report candidate rankings and missing evidence
- do not overclaim module assignments or limit classifications

## Relationship to pose analysis

This analyzer should not independently redefine “actual robot motion” when a stronger shared result already exists.

- `pose-analysis` should provide the preferred best-estimate chassis path
- `drive-analysis` should consume that path and focus on motor-role discovery and traction/current interpretation

That separation avoids duplicated logic and keeps the motion-reference contract consistent across analyzers.
