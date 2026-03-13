# Hard Hit Detection — Investigation Findings

## Summary

**Goal**: Detect "hard hits" (collisions) during FRC matches by analyzing robot log data.

**Conclusion**: Hard hits **are detectable** using the **Pigeon 2 IMU data** from `.hoot` files converted via `owlet`. When Pro-licensed signals are exported, all three acceleration axes plus world-frame angular velocity are available at **~245 Hz** and provide a strong basis for detection. The `.wpilog` files alone do not contain raw IMU data and are not sufficient for reliable impact detection.

---

## Data Source Investigation

### .wpilog files (WPILib DataLog on roboRIO)

The .wpilog has **no raw gyro, accelerometer, or IMU signals**. The Pigeon 2 data is consumed internally by the swerve drive's odometry but is not published to NetworkTables/DataLog. Pose-derived velocity is unreliable for hit detection because field pose is fused from wheel odometry plus potentially noisy vision updates, which can create non-physical jumps.

### .hoot files (CTRE Phoenix 6 Signal Logger)

The `.hoot` file from the CANivore contains all CAN bus device data including the full Pigeon 2 IMU.

**⚠️ Critical: Do NOT use `--unlicensed` with owlet.** The acceleration and most Pigeon 2 signals are Phoenix Pro features. Using `--unlicensed` silently skips them and only exports Yaw, AngularVelocityZWorld, and SupplyVoltage. The robot has a valid Pro license, so omit the flag.

**Pigeon 2 (ID 10) signals available at ~245 Hz (confirmed from WABON Q15):**

| Signal | Rate | Usefulness for Hit Detection |
|--------|------|------------------------------|
| **`AccelerationX`** | 245 Hz | 🏆 Primary — direct linear acceleration (g) |
| **`AccelerationY`** | 245 Hz | 🏆 Primary — direct linear acceleration (g) |
| **`AccelerationZ`** | 245 Hz | 🏆 Primary — reads about -1.026g at rest (gravity + mounting/orientation bias) |
| **`AngularVelocityXWorld`** | 245 Hz | ✅ Detects roll impacts |
| **`AngularVelocityYWorld`** | 245 Hz | ✅ Detects pitch impacts |
| **`AngularVelocityZWorld`** | 245 Hz | ✅ Detects yaw impacts (spin) |
| `Pitch` | 245 Hz | ⚠️ Orientation context |
| `Roll` | 245 Hz | ⚠️ Orientation context |
| `Yaw` | 245 Hz | ⚠️ Integrated heading |

**owlet conversion command** (must omit `--unlicensed`):
```
owlet <hoot_file> <output.wpilog> -f wpilog -s 3ca0a02,3cb0a02,3cc0a02,3c70a02,3c80a02,3c90a02,3b50a02,3b60a02,3b70a02
```

Signal IDs for Pigeon2-10:

| ID | Signal |
|----|--------|
| `3ca0a02` | AccelerationX |
| `3cb0a02` | AccelerationY |
| `3cc0a02` | AccelerationZ |
| `3c70a02` | AngularVelocityXWorld |
| `3c80a02` | AngularVelocityYWorld |
| `3c90a02` | AngularVelocityZWorld |
| `3b50a02` | Yaw |
| `3b60a02` | Pitch |
| `3b70a02` | Roll |

---

## Measured Data (WABON Q15 — 2026 Bonney Lake)

### Acceleration Magnitude

The Pigeon 2 reports acceleration in g's. At rest, $|a| = \sqrt{a_x^2 + a_y^2 + a_z^2} \approx 1.0g$ (gravity only), though the baseline may drift slightly due to mounting angle, calibration error, or vibration.

A hard hit adds to this. A simple first-pass metric is **impact magnitude**: $||a| - 1g|$.

This is a practical heuristic, not a pure linear-acceleration estimate. It works because collisions produce large transients, but it also responds to tilt, wheel hop, and brief drivetrain shock. In a future refinement, gravity-vector compensation or a high-pass / jerk-based method would reduce false sensitivity to chassis attitude changes.

| Metric | Value |
|--------|-------|
| Total $|a|$ range | 0.05g – 2.99g |
| Mean $|a|$ | 1.06g |
| Peak impact ($|a| - 1g$) | **1.99g** |

### Impact Events by Severity

| Threshold | Events | Description |
|-----------|--------|-------------|
| > 0.3g | 145 | All noticeable bumps |
| > 0.5g | 111 | Moderate bumps and above |
| > 1.0g | 23 | Significant impacts |
| > 1.5g | **6** | Hard hits |
| > 2.0g | 0 | (None this match — no severe crashes) |

### Top 6 Hard Hits (> 1.5g impact)

| Time (s) | $a_x$ | $a_y$ | $a_z$ | $|a|$ | Impact | $\omega_z$ |
|----------|--------|--------|--------|-------|--------|-------------|
| 243.9 | -1.84 | +1.51 | -1.80 | 2.99g | 1.99g | -5.6 deg/s |
| 226.8 | -1.71 | +1.67 | -1.71 | 2.94g | 1.94g | **-184.0 deg/s** |
| 216.6 | -0.61 | -1.97 | +1.87 | 2.78g | 1.78g | +18.4 deg/s |
| 220.4 | -1.62 | +1.95 | -0.51 | 2.59g | 1.59g | +37.5 deg/s |
| 254.3 | -1.04 | -1.96 | -1.18 | 2.51g | 1.51g | -26.5 deg/s |
| 189.0 | +1.94 | +0.95 | -1.26 | 2.50g | 1.50g | **+110.4 deg/s** |

Note: The hit at t=226.8s shows both high acceleration (2.94g) and extreme angular velocity (-184 deg/s), indicating a hard spinning collision. The hit at t=243.9s was the highest acceleration but low angular velocity — likely a more head-on or mostly translational impact.

---

## Detection Strategy

### Licensed Logs (Full Acceleration Data)

When owlet is run **without `--unlicensed`** (i.e., the hoot file contains a Pro-licensed device), all Pigeon 2 signals are available.

**Primary signal**: acceleration magnitude.

Compute $|a| = \sqrt{a_x^2 + a_y^2 + a_z^2}$ at each sample. Use impact magnitude $||a| - 1g|$ as the first-pass score.

For implementation, the detector should not assume the baseline is exactly $1.000g$ at every robot orientation. A more robust version should estimate a rolling baseline from a quiet window or use the median of $|a|$ over the match and compare against that baseline instead of a fixed 1g constant.

| Severity | Threshold | Description |
|----------|-----------|-------------|
| **HARD** | > 1.5g impact | Significant collision |
| **MODERATE** | > 1.0g impact | Noticeable impact |
| **LIGHT** | > 0.5g impact | Minor bump |

These thresholds are empirical from one analyzed event set and should be treated as **starting values**, not universal constants. The final detector should keep them configurable.

**Secondary signal**: angular velocity for hit type classification and confidence boosting.

Cross-reference with $\omega = \sqrt{\omega_x^2 + \omega_y^2 + \omega_z^2}$ to classify:
- High accel + high angular velocity → **glancing/spinning hit**
- High accel + low angular velocity → **head-on hit**
- Low accel + high angular velocity → **scrape/deflection**

In practice, classification should be advisory only. A hit can excite different axes depending on robot orientation, bumper contact location, defense interaction, or whether the robot is already turning.

#### Landing vs Collision Classification (Licensed Only)

The 2025/2026 game fields include a ramp/bump structure that robots drive over. When the robot crests the ramp it can briefly go airborne and then impact the ground on landing. These **landing events** are physically distinct from **collisions** with other robots or field walls and should be classified separately.

The key difference is which acceleration axis dominates and what happens in the moments *before* the spike:

| Feature | Landing (ramp) | Collision (robot/wall) |
|---------|----------------|----------------------|
| Dominant axis | **Z** (vertical) | **X/Y** (lateral) |
| Pre-event signature | Freefall: $\|a\| \to 0g$ for 50–200ms | No freefall: $\|a\| \approx 1g$ right before spike |
| Pitch/Roll before event | Pitched (on ramp), then changing rapidly | Roughly flat ($\text{pitch} \approx 0$, $\text{roll} \approx 0$) |
| Angular velocity at impact | Primarily $\omega_y$ (pitch axis) | Primarily $\omega_z$ (yaw axis) |
| Post-event | Magnitude returns to ~1g quickly (robot lands flat) | May oscillate / bounce as robot is pushed |

**Detection algorithm for landing events:**

1. When an impact event is detected (impact > threshold), look at the **50–200ms window before** the peak sample.
2. Compute the **minimum $\|a\|$** in that window. If $\|a\|_{min} < 0.3g$, the robot was likely airborne → classify as **landing**.
3. As a secondary check, compute the **lateral fraction**: $f_{lateral} = \sqrt{a_x^2 + a_y^2} / \|a\|$. A landing has $f_{lateral} < 0.5$ (Z-dominant), while a collision has $f_{lateral} > 0.5$ (X/Y-dominant).
4. Optionally check **Pitch** signal: if $|\text{pitch}|$ exceeded ~10° in the 1–2 seconds before the event, the robot was likely on or transitioning off the ramp.

This classification works because:
- A swerve-drive robot on flat ground has gravity entirely in the Z axis ($a_z \approx -1g$, $a_x \approx a_y \approx 0$).
- Going airborne means $\|a\| \to 0$ (freefall), then the landing spike is dominated by $a_z$ (the ground pushes back up).
- A lateral collision has no freefall precursor — the robot goes from $\|a\| \approx 1g$ to a sudden lateral jolt.

**Note:** This classification requires licensed (Pro) data — it uses all three acceleration axes plus pitch. The unlicensed fallback cannot distinguish landing from collision since it only has $\omega_z$.

### Unlicensed Logs (Angular Velocity Only Fallback)

When the hoot file has no Pro license (or `--unlicensed` is used), only three Pigeon 2 signals are exported: **Yaw**, **AngularVelocityZWorld**, and **SupplyVoltage**. The accelerometer data is not available.

Hard hits can still be detected using **angular acceleration** — the rate of change of `AngularVelocityZWorld` ($\alpha = d\omega_z/dt$) — and **angular velocity step changes** ($\Delta\omega_z$ over a 50ms window).

This fallback is best understood as a **rotation-impact detector**, not a general collision detector.

#### Cross-Validation Results (WABON Q15)

Comparing angular-velocity-only detection against the ground-truth acceleration-based detection:

| Accel severity | Caught by ω | Total | Detection rate |
|----------------|-------------|-------|----------------|
| **HARD** (>1.5g) | 6 | 6 | **100%** |
| MODERATE (>1.0g) | 9 | 16 | 56% |
| LIGHT (>0.5g) | 48 | 89 | 54% |
| **False positives** | — | — | **0** |

**All hard hits are detected in this sample.** Moderate and light hits are partially missed because many involve primarily linear (non-rotational) impacts, which do not strongly excite Z-axis angular velocity. The zero false-positive count in this sample is encouraging, but it should not be treated as a guaranteed property across all matches.

#### Unlicensed Detection Thresholds

Two complementary methods, using only `AngularVelocityZWorld` at ~245 Hz:

**Method 1: Angular acceleration** — compute $\alpha = d\omega_z/dt$ over a 20ms sliding window.

| Severity | Threshold | Description |
|----------|-----------|-------------|
| **HARD** | $|\alpha| > 4000$ deg/s² | Severe rotational impact |
| **MODERATE** | $|\alpha| > 2500$ deg/s² | Significant rotational impact |
| **LIGHT** | $|\alpha| > 1500$ deg/s² | Minor rotational jolt |

**Method 2: Angular velocity step** — compute $|\Delta\omega_z|$ over a 50ms window.

| Severity | Threshold | Description |
|----------|-----------|-------------|
| **HARD** | $|\Delta\omega| > 150$ deg/s | Sudden large spin |
| **MODERATE** | $|\Delta\omega| > 100$ deg/s | Noticeable spin change |
| **LIGHT** | $|\Delta\omega| > 50$ deg/s | Small spin change |

An event is detected if **either** method triggers. Group triggers within 300ms into one event and retain the strongest peak.

#### Unlicensed Limitations

- **Only detects yaw-axis impacts** — head-on collisions with no spin component are invisible to $\omega_z$ alone. AngularVelocityX/YWorld would help but are Pro-only signals.
- **Cannot distinguish landing from collision** — without acceleration axes, the freefall-precursor and axis-dominance checks are impossible. All events are reported as `rotational-only`.
- **Commanded turns** (e.g., autonomous heading corrections, driver snap turns) can produce large $\omega_z$ changes, so the threshold must remain above normal driving dynamics.
- **Misses many translational impacts** — these are typically linear bumps without significant rotation.
- **Should be gated by robot-enabled time** where possible, to avoid startup, disable, or connection transients.

### Analyzer Implementation Plan

A `hard_hits.py` analyzer should:

1. **Read the .hoot file** via owlet conversion.
2. **Detect whether Pro signals are present** (for example by checking for `AccelerationX` in the converted wpilog).
3. **If licensed**:
	- extract `AccelerationX/Y/Z`, `AngularVelocityX/Y/ZWorld`, and `Pitch`
	- compute acceleration magnitude
	- compute a configurable baseline for $|a|$ (defaulting near 1g)
	- detect candidate events where impact exceeds threshold
	- **classify each event** as `landing` or `collision` by checking for a freefall precursor and Z-axis dominance (see "Landing vs Collision Classification")
	- sub-classify collisions using angular velocity (spinning/head-on/deflection)
	- optionally boost confidence when angular velocity is also elevated
4. **If unlicensed**:
	- extract `AngularVelocityZWorld`
	- detect events using angular acceleration and angular velocity step methods
	- label results clearly as **rotational-impact candidates**
5. **Gate to enabled robot time** if robot-enable state is available from companion logs or surrounding context.
6. **Group nearby events** within 300ms as one hit (keep the peak sample).
7. **Report** timestamp, severity, score, and available supporting data (acceleration components when licensed, angular velocity always).
8. **Expose thresholds as configuration**, not hard-coded constants.

### Recommended Output Shape

Each detected event should include at least:

- `timestamp_s`
- `severity` — `HARD`, `MODERATE`, or `LIGHT`
- `score` — impact magnitude (g) or angular acceleration (deg/s²)
- `mode` — `licensed-accel` or `unlicensed-rotational`
- `classification` — one of:
  - `landing` — ramp/airborne landing (Z-dominant spike preceded by freefall)
  - `collision-spinning` — lateral hit with significant rotation
  - `collision-head-on` — lateral hit with minimal rotation
  - `collision-deflection` — scrape/glancing contact
  - `rotational-only` — unlicensed fallback, cannot distinguish landing from collision
- `ax_g`, `ay_g`, `az_g` when available
- `omega_x_dps`, `omega_y_dps`, `omega_z_dps` when available
- `impact_g` when available
- `lateral_fraction` — $\sqrt{a_x^2 + a_y^2} / |a|$ when available (low = landing, high = collision)
- `freefall_detected` — whether $|a| < 0.3g$ was seen in the 200ms before the event
- `angular_accel_dps2` for unlicensed fallback
- `pitch_before_deg` when available (elevated = ramp traversal)

---

## Limitations

- **Requires .hoot file** — the .wpilog alone does not contain raw IMU data. Robot pose is based on wheel odometry fused with potentially noisy vision signals, so it is not reliable for impact detection.
- **Licensed vs unlicensed**: acceleration data requires omitting the `--unlicensed` flag. The angular-velocity fallback is useful, but it is less complete and should be presented as lower confidence for non-rotational hits.
- **Z-axis only (unlicensed)**: the fallback only has yaw angular velocity. Head-on impacts with no spin component can be missed entirely.
- **Thresholds are empirical** — the current values are grounded in a small sample and should be tuned across more matches before being treated as production defaults.
- **Sampling and export behavior matter** — owlet export behavior depends on license state and selected signals; detector code should fail clearly when expected channels are absent.
- **Pigeon 2 accelerometer range / units should be validated in implementation** — the observed values are consistent with g-units, but production code should document units explicitly and guard against saturation or clipped signals.
- **Robot tipping/tilting** changes the gravity vector orientation, which can look like lateral acceleration. Gravity compensation or a jerk/high-pass approach would make the licensed detector more robust.

## Review Verdict

The overall plan is **sound and implementable**, with one important framing change:

- the **licensed path** is a real collision detector,
- the **unlicensed path** is a reliable **rotational collision detector**.

That distinction should stay explicit in both implementation and user-facing output so the tool doesn't overclaim what it can infer from unlicensed exports.
