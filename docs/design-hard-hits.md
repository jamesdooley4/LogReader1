# Hard Hit Detection — Investigation Findings

## Summary

**Goal**: Detect "hard hits" (collisions) during FRC matches by analyzing robot log data.

**Conclusion**: Hard hits **are detectable** using the **Pigeon 2 accelerometer data** from `.hoot` files converted via `owlet`. All three acceleration axes plus full angular velocity are available at **~245 Hz**. The `.wpilog` files alone do not contain IMU data and are not sufficient for hit detection.

---

## Data Source Investigation

### .wpilog files (WPILib DataLog on roboRIO)

The .wpilog has **no raw gyro, accelerometer, or IMU signals**. The Pigeon 2 data is consumed internally by the swerve drive's odometry but is not published to NetworkTables/DataLog. Pose-derived velocity is unreliable for hit detection due to vision-correction jumps that produce false artifacts.

### .hoot files (CTRE Phoenix 6 Signal Logger)

The `.hoot` file from the CANivore contains all CAN bus device data including the full Pigeon 2 IMU.

**⚠️ Critical: Do NOT use `--unlicensed` with owlet.** The acceleration and most Pigeon 2 signals are Phoenix Pro features. Using `--unlicensed` silently skips them and only exports Yaw, AngularVelocityZWorld, and SupplyVoltage. The robot has a valid Pro license, so omit the flag.

**Pigeon 2 (ID 10) signals available at ~245 Hz (confirmed from WABON Q15):**

| Signal | Rate | Usefulness for Hit Detection |
|--------|------|------------------------------|
| **`AccelerationX`** | 245 Hz | 🏆 Primary — direct linear acceleration (g) |
| **`AccelerationY`** | 245 Hz | 🏆 Primary — direct linear acceleration (g) |
| **`AccelerationZ`** | 245 Hz | 🏆 Primary — reads -1.026g at rest (gravity) |
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

The Pigeon 2 reports acceleration in g's. At rest, $|a| = \sqrt{a_x^2 + a_y^2 + a_z^2} \approx 1.0g$ (gravity only).

A hard hit adds to this. The **impact magnitude** is $||a| - 1g|$.

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

Note: The hit at t=226.8s shows both high acceleration (2.94g) AND extreme angular velocity (-184 deg/s), indicating a hard spinning collision. The hit at t=243.9s was the highest acceleration but low angular velocity — likely a more head-on impact.

---

## Detection Strategy

### Licensed Logs (Full Acceleration Data)

When owlet is run **without `--unlicensed`** (i.e., the hoot file contains a Pro-licensed device), all Pigeon 2 signals are available.

**Primary signal**: Acceleration magnitude.

Compute $|a| = \sqrt{a_x^2 + a_y^2 + a_z^2}$ at each sample. Impact = $||a| - 1g|$.

| Severity | Threshold | Description |
|----------|-----------|-------------|
| **HARD** | > 1.5g impact | Significant collision |
| **MODERATE** | > 1.0g impact | Noticeable impact |
| **LIGHT** | > 0.5g impact | Minor bump |

**Secondary signal**: Angular velocity for hit type classification.

Cross-reference with $\omega = \sqrt{\omega_x^2 + \omega_y^2 + \omega_z^2}$ to classify:
- High accel + high angular velocity → **glancing/spinning hit**
- High accel + low angular velocity → **head-on hit**
- Low accel + high angular velocity → **scrape/deflection**

### Unlicensed Logs (Angular Velocity Only Fallback)

When the hoot file has no Pro license (or `--unlicensed` is used), only three Pigeon 2 signals are exported: **Yaw**, **AngularVelocityZWorld**, and **SupplyVoltage**. The accelerometer data is not available.

Hard hits can still be detected using **angular acceleration** — the rate of change of `AngularVelocityZWorld` ($\alpha = d\omega_z/dt$) — and **angular velocity step changes** ($\Delta\omega_z$ over a 50ms window).

#### Cross-Validation Results (WABON Q15)

Comparing angular-velocity-only detection against the ground-truth acceleration-based detection:

| Accel severity | Caught by ω | Total | Detection rate |
|----------------|-------------|-------|----------------|
| **HARD** (>1.5g) | 6 | 6 | **100%** |
| MODERATE (>1.0g) | 9 | 16 | 56% |
| LIGHT (>0.5g) | 48 | 89 | 54% |
| **False positives** | — | — | **0** |

**All hard hits are detected.** Moderate and light hits are partially missed because many involve primarily linear (non-rotational) impacts, which don't show up in Z-axis angular velocity. However, the zero false-positive rate means detections are trustworthy.

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

An event is detected if **either** method triggers. Group events within 300ms.

#### Unlicensed Limitations

- **Only detects yaw-axis impacts** — head-on collisions with no spin component are invisible to $\omega_z$ alone. AngularVelocityX/YWorld would help but are Pro-only signals.
- **Commanded turns** (e.g., PathPlanner rotation) can produce angular accelerations of 1000–1500 deg/s², so the threshold must remain above normal driving dynamics.
- **Misses ~44% of MODERATE impacts** — these are typically linear bumps without significant rotation.

### Analyzer Implementation Plan

A `hard_hits.py` analyzer should:

1. **Read the .hoot file** via owlet conversion
2. **Detect whether Pro signals are present** (check for `AccelerationX` in the converted wpilog)
3. **If licensed**: extract `AccelerationX/Y/Z` and `AngularVelocityX/Y/ZWorld`, compute acceleration magnitude, detect events where impact > threshold, classify using angular velocity
4. **If unlicensed**: extract `AngularVelocityZWorld` only, detect events using angular acceleration and angular velocity step methods
5. **Group nearby events** within 300ms as one hit (keep peak)
6. **Report** timestamp, severity, and available data (acceleration components when licensed, angular velocity always)

---

## Limitations

- **Requires .hoot file** — the .wpilog alone does not contain IMU data. Robot pose is based on wheel odometry fused with potentially noisy vision signals, so it is not reliable for impact detection.
- **Licensed vs unlicensed**: acceleration data requires omitting the `--unlicensed` flag. The angular-velocity fallback catches all hard hits but misses ~44% of moderate linear impacts.
- **Z-axis only (unlicensed)**: the fallback only has yaw angular velocity. Head-on impacts with no spin are invisible.
- **Pigeon 2 accelerometer range** is ±2g (16-bit). The max observed was 2.99g, suggesting some impacts may be clipped. Severe crashes could saturate the sensor (the Pigeon 2 has a `Fault_SaturatedAccelerometer` signal for this case).
- **Normal driving dynamics** produce accelerations up to ~0.3–0.5g (turning, braking), so the threshold must be set above this.
- **Robot tipping/tilting** changes the gravity vector orientation, which can look like lateral acceleration. Using the gravity vector signals could compensate for this.
