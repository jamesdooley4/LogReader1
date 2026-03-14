# Pose Correlation and Divergence Analysis Plan

This document outlines the strategy for analyzing robot poses by evaluating multiple odometry and localization sources. The primary goal is to retrospectively fuse these sources into a "most accurate" ground-truth path, and then measure how each individual sensor modality (wheel odometry, vision, accelerometer) correlates or diverges from that path.

## Localisation Sources

1. **Wheel Odometry + Gyro:**
   - High frequency, locally very precise.
   - Prone to long-term drift and sudden divergence during wheel slip or collisions.
2. **Vision Camera(s):**
   - Typically discrete, lower frequency updates (e.g., AprilTag pose estimates).
   - Globally accurate but subject to high noise, latency, multi-tag ambiguity problems, and false positives.
3. **Accelerometer (IMU):**
   - Extremely high frequency.
   - Good for capturing high-frequency dynamics and sudden impacts.
   - Extremely prone to drift if double-integrated alone, but useful for short-term bridging.

## Phase 1: Post-Match Sensor Fusion (Ground Truth Generation)

Because we are processing log files after the fact, we are not restricted to real-time (causal) filters like the standard `SwerveDrivePoseEstimator`. We can look both backward and forward in time to generate a smoothed, highly accurate ground-truth pose.

1. **Non-Causal Filtering / Smoothing:**
   - Implement an offline smoothing algorithm (such as a Rauch-Tung-Striebel smoother or offline Factor Graph optimization).
   - Align the timelines of Odometry, Vision Updates (accounting for recorded latency), and IMU.
   - Generate the "Best Estimate Path" for the entire match.

## Phase 2: Divergence Measurement and Analysis

With the "Best Estimate Path" established, we can evaluate the individual inputs.

### 1. Odometry Divergence
- **Measurement:** Compare the raw wheel odometry path against the Best Estimate Path.
- **Insights:** 
  - Gradual divergence indicates poor wheel diameter characterization or track width tuning.
  - Sudden divergence indicates wheel slip (loss of traction) or physical collisions causing the robot to be moved against its driven intent.

### 2. Vision Correlation and Noise
- **Measurement:** Compare raw vision pose estimates (handling latency appropriately) against the Best Estimate Path.
- **Insights:**
  - Standard deviation and noise profiling of the camera system at different distances.
  - Detect "Vision Jumps" — moments where the camera reported a completely erroneous pose (e.g., due to field reflections or ambiguous tag combinations) that the real-time filter might have incorrectly trusted.
  - Provide concrete tuning recommendations for standard deviations for the team's real-time `SwerveDrivePoseEstimator`.

### 3. Accelerometer Consistency
- **Measurement:** Compare IMU accelerations with the derived acceleration from the Best Estimate Path.
- **Insights:**
  - Identify unmeasured physical disturbances (hard impacts with other robots or the field perimeter).
  - Noise profiling for the IMU to tune process noise parameters.

## Deliverables

- **Divergence Metrics:** Max and average error (in meters/degrees) for each individual source over the match.
- **Tuning Recommendations:** Ideal weights/standard deviations for the team's real-time pose estimator.
- **Anomaly Highlighting:** Timestamps where traction was completely lost or vision completely failed.
