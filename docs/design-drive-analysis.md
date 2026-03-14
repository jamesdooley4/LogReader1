# Drive Motor Discovery and Traction Analysis Plan

This document outlines the plan for a new analyzer (or set of analyzers) to automatically discover swerve drive motors and perform advanced traction and current limit analysis based on telemetry, poses, and accelerometer data.

## Phase 1: Drive Motor Discovery

The goal is to automatically identify which motor controllers correspond to the four propulsion (drive) motors and the four steering (azimuth) motors, without requiring explicit configuration.

### 1. Identify Candidate Motors
- Scan all motor controller signals (e.g., device signals for TalonFX, SparkMax, etc.).
- Collect current (supply and stator), velocity, and duty cycle data for each motor.

### 2. Differentiate Propulsion vs. Steer
- **Current Draw:** Propulsion motors experience significantly higher sustained current draws (both supply and stator) compared to steer motors. Steer motors typically have sharp, brief current spikes during quick orientation changes.
- **Velocity Profiles:** Propulsion motor velocities will be continuous and proportional to the robot's translational speed. Steer motor movement will be discontinuous, moving to target angles and holding.
- **Correlation with Robot Movement (Poses):**
  - Read odometry, vision, or fused poses (e.g., `/SmartDashboard/Field/Robot`).
  - Calculate translational velocity and rotational velocity from the poses.
  - **Propulsion:** High correlation between motor velocity and the robot's translational velocity.
  - **Steer:** High correlation between motor position changes and the robot's rotational velocity/sudden path curvature changes.
- **Accelerometer Data:**
  - Read IMU/accelerometer data.
  - Correlate motor acceleration phases with actual robot acceleration.

### 3. Pose Divergence & Validation (Sub-analyzer)
- Compare wheel odometry rates with actual spatial translation (from vision-corrected poses or IMU double-integration, acknowledging drift).
- Establish a baseline mapping of wheel velocity to expected robot speed.

## Phase 2: Traction and Current Limit Analysis

Once the drive and steer motors are identified, analyze their behavior in conjunction with the robot's motion to index specific limiting states.

### 1. Grip Limited (Traction Loss)
- **Condition:** Wheels are spinning, but the robot is not moving at the expected corresponding velocity, or is accelerating significantly less than expected.
- **Detection:**
  - Compare calculated theoretical speed (from propulsion motor velocity and assumed wheel diameter/gear ratio) to actual speed (from pose differences or accelerometer integration).
  - A significant divergence (theoretical speed > actual speed) indicates wheel slip.
  - If a subset of motors diverges while others don't, it indicates partial grip loss (e.g., one wheel lifted or on a slippery surface), often causing unintended rotation.

### 2. Supply Current Limited
- **Condition:** The motor demands more power, but is restricted by the battery, main breaker, or software-imposed supply current limits.
- **Detection:**
  - Observe the supply current signal for each propulsion motor.
  - Identify periods where the supply current plateaus exactly at known common limits (e.g., 40A, 60A) or configured software limits, while the duty cycle/voltage demand remains pegged at maximum (e.g., 1.0 or 12V).
  - Can also correlate with overall PDH total current reaching the main breaker trip curve thresholds.

### 3. Stator Current Limited
- **Condition:** The motor torque output is constrained by a stator current limit, often due to high load at low speeds (e.g., pushing matches).
- **Detection:**
  - Observe the stator current signal.
  - Identify plateauing of stator current at a high threshold (e.g., 80A, 100A, 120A, depending on team configuration).
  - This typically occurs during acceleration from a standstill or when pushing against an immovable object (velocities are low, but commanded torque is maximum).
