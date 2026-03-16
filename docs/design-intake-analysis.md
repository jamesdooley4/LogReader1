# Intake Position Analysis — Design Document

> Analyzer: `intake-analysis` · Module: `src/logreader/analyzers/intake_analysis.py`

Analyse the intake pivot position to detect extension/retraction events, measure move timing, flag outliers, and correlate with roller speed dips and launch counter data to track fuel element flow through the robot. See [launch-counter design](design-launch-counter.md) for launch detection and [hard-hits design](design-hard-hits.md) for impact correlation.

**Status:** Design complete
**Depends on:** `match-phases` (optional, for per-phase breakdown), `launch-counter` (optional, for fuel-out correlation), `hard-hits` (optional, for fuel-loss correlation)
**Validated against:** `FRC_20260307_225328_WABON_Q15.wpilog`, `FRC_20260308_004101_WABON_Q28.wpilog`, `FRC_20260308_225734_WABON_E14.wpilog`

---

## Data Observations

Analysed across three matches from the Bonney Lake event (2026 week 1):

### Available Signals

| Signal | Type | Meaning | Typical sample count |
|--------|------|---------|---------------------|
| `NT:intake/pivotTargetPosition` | double | Commanded position setpoint | 8–15 pts/match (logged on change only) |
| `NT:intake/pivotCurrentPosition` | double | Actual encoder position | 1300–2100 pts/match (change-driven) |
| `NT:intake/leftRollerSpeed` | double | Left intake roller speed (RPS) | 2200–3600 pts/match |
| `NT:intake/rightRollerSpeed` | double | Right intake roller speed (RPS) | 1700–3800 pts/match |
| `NT:/Zero/intakePivotZero` | boolean | Whether the intake pivot has been zeroed | 2 pts (init + zeroed) |

### Pivot Target Positions (Discrete States)

The target position signal has exactly **three discrete values** across all analysed matches:

| Value | Meaning | Description |
|-------|---------|-------------|
| `0.0` | **IN** (retracted) | Intake stowed inside frame perimeter |
| `-0.39` | **OUT** (extended) | Intake fully deployed for ground pickup |
| `-0.21` | **INTERMEDIATE** | Partially retracted — hopper position for holding fuel elements |

The target only changes 7–14 times per match. A typical cycle is: IN → OUT (deploy to collect fuel) → INTERMEDIATE (partially retract with loaded hopper) → OUT (deploy again) → INTERMEDIATE → ... → OUT (remain out at end).

### Pivot Current Position Characteristics

The current position signal uses **change-driven logging** — it records rapidly during motion and rarely when stationary:

| Interval bucket | % of samples (E14) |
|-----------------|-------------------|
| < 50 ms | 80.9% |
| 50–200 ms | 14.9% |
| 200 ms – 1 s | 3.4% |
| 1–5 s | 0.6% |
| > 5 s | 0.3% |

**Position range:** `[-0.43, 0.0]` — the actual position slightly overshoots the `-0.39` target (reaches ~`-0.43`) before settling.

**Position zone distribution** (E14 match):

| Zone | Range | % of samples |
|------|-------|-------------|
| Near zero (IN) | > -0.05 | 0.6% |
| Transit (deploying) | -0.05 to -0.16 | 0.3% |
| Intermediate | -0.16 to -0.26 | 10.8% |
| Transit (deep) | -0.26 to -0.34 | 5.2% |
| Extended (OUT) | < -0.34 | 83.0% |

> **Gotcha:** Because of change-driven logging, sample counts are heavily biased toward motion periods. The intake spends much more *time* retracted/stationary than the sample distribution suggests, since few samples are recorded when the position isn't changing.

### Move Timing Observations

**Extension (IN/INTERMEDIATE → OUT, target = -0.39):**
- Very consistent: **0.32–0.62 s** across all matches
- Mean: ~0.43 s
- Purely motor-driven, no obstruction expected

**Retraction (OUT → INTERMEDIATE, target = -0.21):**
- Highly variable: **0.74–39.0 s**
- Varies because fuel elements in the hopper physically impede the pivot from reaching the intermediate position
- Frequently **never reached** — the target changes back to -0.39 (re-extend) before the intake arrives at -0.21
- In Q15, **6 out of 7 retract-to-intermediate commands** never reached their target

**Cross-match comparison:**

| Metric | Q15 (Qual) | Q28 (Qual) | E14 (Elim) |
|--------|-----------|-----------|-----------|
| Target changes | 15 | 12 | 8 |
| Extend moves (→ -0.39) | 8 | 6 | 4 |
| Retract moves (→ -0.21) | 7 | 5 | 3 |
| Extension time (mean) | 0.41 s | 0.44 s | 0.45 s |
| Retract time (mean, when reached) | 0.74 s | 3.56 s | 14.6 s |
| Retractions never reached | 6 / 7 | 1 / 5 | 1 / 3 |

> **Key insight:** The variability in retraction time and "never reached" rate is a strong signal for how full the hopper is — a full hopper blocks retraction. This is the single most valuable metric from this analyzer.

### Roller Speed Characteristics

| Metric | Left roller | Right roller |
|--------|-------------|--------------|
| Nominal speed (running) | ~56 RPS | ~57 RPS |
| Max observed | ~70 RPS | ~71 RPS |
| Negative (brief reversals) | down to -3.7 | down to -3.3 |
| Typical dip (fuel contact) | down to 20–40 RPS | similar |

**Roller speed dip patterns:**
- Fuel elements contacting the rollers cause transient speed drops of 15–35 RPS below nominal
- Dip durations range from **25ms** (brief contact/no pickup) to **3–5s** (sustained feeding/stalled)
- Long dips (>1s) correlate with intake pivot retraction commands — the sequence is: rollers grab fuel → speed dips → retract command issued → intake partially retracts with loaded hopper
- Short dips (<200ms) often indicate fuel passing through rapidly or a ball bounce

**Roller dip correlation with pivot retraction (E14):**

| Retract cmd time | Min roller speed near event | Timing |
|-----------------|---------------------------|--------|
| t=349.2s | 1.3 RPS | 3.0s before cmd |
| t=356.2s | 21.8 RPS | 3.2s after cmd |
| t=392.3s | 24.9 RPS | 3.5s before cmd |

The rollers slow before or during retraction, not after — confirming that fuel contact → roller load → retract sequence.

---

## Algorithm

The analyzer performs three main analyses:

### 1. Position State Machine

Track the intake through discrete states using the target position signal, with the current position providing actual motion timing.

```
States:
  IN          — target = 0.0, current near 0.0
  EXTENDING   — target = -0.39, current moving from near 0.0 toward -0.39
  OUT         — target = -0.39, current near -0.39 (within ARRIVE_THRESHOLD)
  RETRACTING  — target = -0.21, current moving from near -0.39 toward -0.21
  INTERMEDIATE — target = -0.21, current near -0.21 (within ARRIVE_THRESHOLD)

Parameters:
  ARRIVE_THRESHOLD = 0.02    # position units — "close enough" to target
  IN_THRESHOLD     = 0.05    # position units — "near zero" for IN state
  EXTENDED_ZONE    = -0.34   # below this = in extended zone
```

Each target change creates a **MoveEvent** capturing the commanded transition. The current position is monitored to determine when (or if) the move completes.

### 2. Move Timing

For each target change:
1. Record the **command timestamp** (`t_cmd`) from `pivotTargetPosition`
2. Scan `pivotCurrentPosition` samples starting at `t_cmd`
3. The move is **complete** when `|current - target| < ARRIVE_THRESHOLD`
4. **Move duration** = `t_arrived - t_cmd`
5. If the target changes again before arrival, mark the move as **interrupted** and record how close the position got

**Outlier detection:**
- Compute median and IQR for extension times and retraction times separately
- Flag any move > `median + 2 × IQR` as an **outlier** (or the simpler approach: flag retractions > 2× the median retraction time)
- For retractions, "never reached" is always an outlier worth reporting

### 3. Fuel Flow Correlation

#### a) Roller Speed Dips → Fuel Entry Events

A **fuel entry event** is detected when the average of left + right roller speed drops below a threshold while rollers are otherwise active:

```
Parameters:
  ROLLER_NOMINAL       = 55.0   # RPS — expected running speed
  ROLLER_DIP_THRESHOLD = 40.0   # RPS — below this = dip (fuel contact)
  ROLLER_ACTIVE_MIN    = 5.0    # RPS — below this = rollers off, not a dip
  MIN_DIP_DURATION_MS  = 20     # ignore sub-20ms noise spikes
```

For each dip period, record start time, end time, minimum speed, and duration. Classify dips:
- **Brief contact** (< 200 ms): likely a ball bounce or light touch
- **Normal intake** (200 ms – 2 s): fuel element entering normally
- **Sustained load** (> 2 s): hopper feeding or stall condition

#### b) Fuel Entry ↔ Launch Counter Correlation

If `launch-counter` results are available (via `result.extra`), compare fuel-in events to fuel-out (launch) events:
- Count fuel entry events (roller dips of "normal intake" or longer) = estimated **fuel IN**
- Count launches from launch-counter = **fuel OUT**
- **Net accumulation** = fuel IN - fuel OUT (should trend to 0 at end of match)
- Identify periods where fuel IN significantly exceeds fuel OUT (hopper building up) or vice versa

#### c) Fuel Loss ↔ Hard Hits Correlation

If `hard-hits` results are available, check for temporal correlation:
- For each hard hit event, check if the intake was in INTERMEDIATE or transitioning state
- Check if net fuel count decreased shortly after a hit (fuel spilled)
- Flag hits that may have caused fuel loss for investigation

---

## Data Models

```python
from dataclasses import dataclass
from enum import Enum


class IntakeState(Enum):
    """Discrete intake position states."""
    IN = "in"
    EXTENDING = "extending"
    OUT = "out"
    RETRACTING = "retracting"
    INTERMEDIATE = "intermediate"
    UNKNOWN = "unknown"


class MoveResult(Enum):
    """Outcome of a commanded move."""
    COMPLETED = "completed"
    INTERRUPTED = "interrupted"  # target changed before arrival
    NEVER_REACHED = "never_reached"  # match ended or data ended


@dataclass
class MoveEvent:
    """A single commanded intake move."""
    command_time_s: float           # when the target changed
    target_position: float          # new target value
    previous_position: float        # previous target value
    direction: str                  # "extend" or "retract" or "stow"
    result: MoveResult
    duration_s: float | None        # time to arrive (None if not reached)
    closest_position: float         # closest the intake got to the target
    is_outlier: bool                # move time is statistically unusual


@dataclass
class RollerDip:
    """A detected dip in intake roller speed (fuel contact event)."""
    start_time_s: float
    end_time_s: float
    duration_s: float
    min_speed: float                # lowest roller speed during dip
    mean_speed: float               # average roller speed during dip
    classification: str             # "brief_contact", "normal_intake", "sustained_load"


@dataclass
class IntakeAnalysisResult:
    """Complete intake analysis output."""
    # Move events
    moves: list[MoveEvent]
    extend_count: int
    retract_count: int
    extend_median_s: float
    retract_median_s: float | None  # None if no retractions completed
    interrupted_count: int

    # Roller analysis
    roller_dips: list[RollerDip]
    fuel_entry_count: int           # dips classified as normal_intake or sustained_load

    # Correlation (if launch-counter results available)
    fuel_out_count: int | None      # from launch-counter
    net_fuel: int | None            # fuel_in - fuel_out

    # Time in each state
    time_in_s: float
    time_out_s: float
    time_intermediate_s: float
    time_transit_s: float
```

---

## Output Design

### Summary

```
Intake Position Analysis
========================

Pivot moves:      12 total (7 extend, 5 retract)
  Extend:         median 0.43s  (range 0.32–0.62s)  [0 outliers]
  Retract:        median 2.37s  (range 0.74–6.98s)  [1 outlier]
  Interrupted:    3 (target changed before arrival)
  Never reached:  1

Time in state:
  IN (retracted):     12.4s  (8%)
  OUT (extended):    118.7s  (77%)
  INTERMEDIATE:       9.2s  (6%)
  Transit:           13.4s  (9%)

Roller analysis:
  Fuel entry events:  8 normal intake, 2 sustained load
  Brief contacts:     14
```

### Move Detail Table (with `--detail`)

```
#   Time      Direction   Target  Duration  Result       Notes
1   142.57s   extend      -0.39   0.45s     completed
2   145.31s   retract     -0.21   0.74s     completed
3   172.75s   extend      -0.39   —         interrupted  target changed after 1.85s
4   174.60s   retract     -0.21   0.43s     completed
5   182.65s   extend      -0.39   —         interrupted  target changed after 2.67s
6   185.32s   retract     -0.21   6.98s     completed    ⚠ OUTLIER (2.9× median)
...
```

### Correlation Summary (when launch-counter available)

```
Fuel flow:
  Estimated fuel IN:   10 (from roller dip events)
  Fuel OUT (launched): 8  (from launch-counter)
  Net fuel:            +2 (fuel remaining in robot)

⚠ Hard hit at t=312.1s while intake INTERMEDIATE — check for fuel loss
```

---

## `extra` Dict Schema

```python
result.extra = {
    "moves": list[MoveEvent],                # all move events
    "roller_dips": list[RollerDip],           # all detected roller dips
    "intake_result": IntakeAnalysisResult,    # full structured result
    "thresholds": {
        "arrive_threshold": float,
        "roller_dip_threshold": float,
        "roller_nominal": float,
    },
}
```

---

## Signal Auto-Detection

The analyzer should auto-detect intake signals by searching for common naming patterns:

| Pattern | Matches |
|---------|---------|
| `intake/pivotCurrentPosition`, `intake/position`, `intake/currentPosition` | Current position |
| `intake/pivotTargetPosition`, `intake/targetPosition`, `intake/goal` | Target/setpoint |
| `intake/leftRollerSpeed`, `intake/rightRollerSpeed`, `intake/rollerSpeed` | Roller speeds |

Allow CLI overrides: `--position-signal`, `--target-signal`, `--roller-signal`.

---

## CLI Usage

```bash
# Basic — auto-detect signals, default thresholds
logreader intake-analysis path/to/log.wpilog

# Show every individual move and roller dip
logreader intake-analysis path/to/log.wpilog --detail

# Custom arrival threshold
logreader intake-analysis path/to/log.wpilog --arrive-threshold 0.03

# Custom roller dip threshold
logreader intake-analysis path/to/log.wpilog --dip-threshold 35

# Explicit signal names
logreader intake-analysis path/to/log.wpilog --position-signal "NT:intake/pivotCurrentPosition"

# Correlate with launch counter (runs launch-counter first if not already run)
logreader intake-analysis path/to/log.wpilog --correlate
```

---

## Edge Cases & Fallbacks

- **No target signal:** If only `pivotCurrentPosition` is available (no target signal), infer transitions from position changes. Detect moves by watching for velocity exceeding a threshold (position delta / time delta). Less precise but still useful.
- **No roller signals:** Skip fuel entry analysis. Position-only analysis is still valuable.
- **Intake never deployed:** Report "intake was not used during this match" with time-in-state showing 100% IN.
- **Mid-match reboot:** If `pivotCurrentPosition` has a gap (no samples for > 5s during active match), note the gap and restart state tracking.
- **Multiple intake mechanisms:** Some robots have separate left/right intake pivots. The signal naming patterns should distinguish them; if multiple position signals match, report them separately.

---

## Testing Strategy

1. **Synthetic data — basic state tracking:** Create a `LogData` with target values [0.0, -0.39, -0.21, -0.39] and current position ramps. Verify states and move durations.
2. **Synthetic data — interrupted move:** Target changes from -0.39 to -0.21, then back to -0.39 before current reaches -0.21. Verify `MoveResult.INTERRUPTED`.
3. **Synthetic data — roller dips:** Generate roller speed with dips of varying duration. Verify classification (brief, normal, sustained).
4. **Synthetic data — outlier detection:** Include one move that takes 5× longer than the others. Verify it's flagged.
5. **Real data regression:** Run against the three validated log files and assert launch counts and move counts match manual observations.
