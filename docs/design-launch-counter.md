# Launch Counter — Design Document

> Analyzer: `launch-counter` · Module: `src/logreader/analyzers/launch_counter.py`

Count game-element launches from flywheel velocity data. Optionally breaks down launches per match phase (auto / teleop / disabled), with a configurable grace period to capture spin-down launches that fire after a phase officially ends. See [match-phases design](design-match-phases.md) for phase integration.

---

## Data Observations

Analysed across two matches from the same event:
- `FRC_20260307_221838_WABON_Q11.wpilog` (qualification match)
- `FRC_20260308_232912_WABON_E15.wpilog` (elimination match)

**Available signals:**
- `NT:/launcher/velocity` — flywheel speed (units appear to be RPS or similar). ~1400–3300 samples per match, ~47 Hz median sample rate (21 ms median interval). Range: -1.9 to 100.6.
- `NT:/launcher/current` — motor current draw. Only ~100–265 samples per match (~4 Hz) — **too sparse for launch detection**. Best used as a secondary confirmation signal only.
- `NT:/AutoAim/flywheelGoal` — setpoint. Only 1 sample in both logs (not useful).

**Flywheel operating characteristics:**
- Cruising speed varies by shooting mode: ~65–70 (close range), ~80–90 (far range), up to ~100 (long range in E15)
- Idle / disabled: ~0
- Spin-up takes ~1–2 seconds from 0 to cruise
- Velocity distribution while spinning peaks strongly in the 70–80 range

**Launch signature in velocity data:**
- A launch causes a **sharp dip of 8–25 units** from the cruising velocity
- The dip lasts **~50–150 ms** before the motor recovers
- Recovery is **5+ units** back up from the dip minimum
- Median dip magnitude is very consistent across matches: **14.5–14.7 units**

**Observed launch rates:**
- Burst rates up to **~5 Hz** (fastest gap between shots: ~80 ms)
- Typical sustained rate: **2–4 launches/s** (median gap: ~177–190 ms)
- The theoretical max of 8/s is not seen in this data; real-world peak is ~5/s
- Not close to the ~47 Hz sample rate, so the velocity signal has sufficient resolution

**Cross-match consistency:**

| Metric                  | Q11 (Qual) | E15 (Elim) |
|-------------------------|------------|------------|
| Launches detected       | 84         | 56         |
| Spinning periods        | 10         | 5          |
| Peak burst rate         | 3.6/s      | 3.2/s      |
| Median burst gap        | 177 ms     | 190 ms     |
| Median dip magnitude    | 14.7       | 14.5       |
| Fastest single gap      | 80 ms      | 83 ms      |

---

## Rapid-Fire Burst Behaviour (Critical Finding)

During aggressive feeding, game elements pass through the flywheel faster than the motor can recover. This creates a **sustained velocity decline with sawtooth dip patterns superimposed** — each dip is a real launch, but the baseline keeps falling because the motor can't replenish energy between shots.

**Example — Q11 at 294.9s (5+ shots in ~0.7s):**
```
294.958   41.40  ◄ launch #1 (drop from ~64)
295.097   34.44  ◄ launch #2 (drop from ~46, baseline falling)
295.181   27.91  ◄ launch #3 (baseline still falling)
295.334   20.01  ◄ launch #4
295.672   15.13  ◄ launch #5
295.838   -1.04  ← finally shutdown (motor gives up)
```

**Example — E15 at 406–408s (7 shots from ~100 RPS):**
```
406.470   80.12  ◄ launch #1 (drop from ~94)
406.982   69.09  ◄ launch #2 (from ~95, big 25-unit dip)
407.090   68.42  ◄ launch #3 (partial recovery only)
407.228   60.52  ◄ launch #4 (baseline still falling)
407.505   56.56  ◄ launch #5
407.624   59.10  ◄ launch #6
407.866   67.71  ◄ launch #7 (finally starting to recover)
```

**Implications for the algorithm:**
1. A **fixed floor is too aggressive** — during rapid bursts, velocity can drop below 40 while still actively launching. The floor must be lower or relative to the recent trend.
2. A **"must recover to X% of baseline" post-check would be wrong** — it would discard legitimate rapid-fire shots. The sawtooth shape is the distinguishing feature vs. a smooth coast-down.
3. **Smooth coast-down vs. launch burst**: a shutdown declines at ~1–2 units per sample with no sharp dips. A burst has **sharp drops of 8+ units in a single sample** even as the overall velocity declines. This is the key distinguishing feature.
4. The algorithm should detect dips based on **sample-to-sample drop magnitude**, not just deviation from a high baseline.

---

## Proposed Algorithm

```
State machine with 3 states: IDLE → AT_SPEED → IN_DIP → (launch detected) → AT_SPEED

Parameters (should be configurable via CLI args):
  AT_SPEED_THRESHOLD = 55.0    # velocity must exceed this to START detection
  DROP_MIN           = 8.0     # minimum velocity drop to enter IN_DIP
  DIP_FLOOR          = 30.0    # hard floor — below this, stop detecting (was 40, lowered
                               # based on rapid-burst analysis showing real launches in the 30s)
  RECOVERY_MIN       = 5.0     # must recover this much from dip minimum
  DEBOUNCE_MS        = 80      # minimum gap between consecutive launches

Baseline tracking:
  - Use a rolling max over the last N samples (N=5, ~100ms window)
  - This provides the "recent cruising speed" reference
  - Once AT_SPEED is entered, detection stays active even as baseline declines
    (to handle rapid-burst scenarios) until velocity drops below DIP_FLOOR

Detection logic:
  1. If velocity > AT_SPEED_THRESHOLD → enter AT_SPEED, begin detection
  2. If (baseline - velocity) >= DROP_MIN and velocity >= DIP_FLOOR → enter IN_DIP
  3. While IN_DIP, track the minimum velocity
  4. If velocity < DIP_FLOOR → exit detection mode (shutdown or burst exhaustion)
  5. If (velocity - dip_minimum) >= RECOVERY_MIN → LAUNCH DETECTED
  6. Apply debounce: ignore if < DEBOUNCE_MS since last launch
  7. Continue detection (don't require re-entering AT_SPEED between shots
     in a burst — the baseline naturally tracks the declining envelope)

Coast-down discrimination (future refinement):
  - If velocity is declining steadily (no sharp dips > 5 units/sample)
    for more than ~10 consecutive samples, exit detection mode
  - This would filter the smooth tail-off at end of a burst
  - Not implemented in v1 — the FLOOR + DROP_MIN already handle most cases
```

---

## Output Design

The analyzer should report:
1. **Total launch count** for the match
2. **Per-spinning-period breakdown** (merge adjacent spinning periods < 2s apart)
   - Time range, duration, launch count, launches/second
3. **Summary statistics**
   - Average dip magnitude (how much speed is lost per launch)
   - Burst rate (max instantaneous launch rate)
   - Median gap between launches
4. **Tabular per-launch detail** (time, dip velocity, drop magnitude, baseline)
   - With a `--detail` flag since this can be very long

---

## Signal Auto-Detection

The analyzer should auto-detect the velocity signal by searching for signal names containing common patterns:
- `launcher/velocity`, `shooter/velocity`, `flywheel/velocity`
- `Launcher/Velocity`, `Shooter/Velocity`, `Flywheel/Velocity`
- Allow the user to override with `--velocity-signal <name>`
- Optionally use a current signal for confirmation if available

---

## CLI Usage

```bash
# Basic — auto-detect signals, default thresholds
logreader launch-counter path/to/log.wpilog

# Custom thresholds
logreader launch-counter path/to/log.wpilog --drop-min 10 --floor 30

# Explicit signal name
logreader launch-counter path/to/log.wpilog --velocity-signal "NT:/launcher/velocity"

# Show every individual launch
logreader launch-counter path/to/log.wpilog --detail
```
