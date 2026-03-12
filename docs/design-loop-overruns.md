# Loop Overrun Analysis — Design Document

> Analyzer: `loop-overruns` · Module: `src/logreader/analyzers/loop_overruns.py`

**Status:** Design complete, not yet implemented
**Depends on:** Optional integration with `match_phases` for phase-aware breakdown
**Validated against:** `FRC_20260308_225734_WABON_E14.wpilog`, `FRC_20260307_221838_WABON_Q11.wpilog`, `FRC_20260308_232912_WABON_E15.wpilog`, `FRC_20260307_173313.wpilog` (practice)

Detect and report robot main-loop overruns caused by excessive CPU usage on the roboRIO. Parse WPILib `Tracer` timing breakdowns from console log messages to identify which subsystems and commands are consuming the most time, correlate overruns with match phases, and flag the worst offenders.

---

## Data Observations

### How WPILib reports loop timing

WPILib's `IterativeRobotBase` runs the robot main loop at a configurable period (default **20 ms / 50 Hz**). Each iteration runs a pipeline of "epochs" tracked by the `Tracer` class:

```
1. Phase-specific periodic (autonomousPeriodic, teleopPeriodic, etc.)
2. SmartDashboard.updateValues()
3. robotPeriodic() — runs the CommandScheduler and all subsystem periodics
4. LiveWindow.updateValues()
5. Shuffleboard.update()
```

Inside `robotPeriodic()`, the `CommandScheduler` runs:
- All registered subsystem `.periodic()` methods
- All scheduled command `.execute()` methods
- Button/trigger bindings via `buttons.run()`

When the total loop time exceeds the configured period, WPILib emits two kinds of console messages:

**1. `IterativeRobotBase` overrun warning** — always emitted on overrun:
```
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:436): Loop time of 0.02s overrun
```

**2. `Tracer` epoch breakdown** — emitted alongside the overrun, listing every tracked epoch and its execution time:
```
Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:62):
    teleopPeriodic(): 0.001317s
    SmartDashboard.updateValues(): 0.021871s
    robotPeriodic(): 0.106885s
    LiveWindow.updateValues(): 0.003506s
    Shuffleboard.update(): 0.000019s
```

**3. `CommandScheduler` overrun** — emitted by the scheduler itself:
```
CommandScheduler loop overrun
```

### Critical insight: only overrun loops are logged

WPILib's `Tracer` only prints epoch timings when an overrun occurs. Normal (good) loops produce **no timing output**. This means:

- We have **detailed timing breakdowns for every bad loop** — the exact data we need.
- We do **not** have timing data for normal loops — we can only infer them from the absence of overrun messages.
- The total number of loops can be estimated from the log duration and the configured loop period.

### What the data looks like across real logs

| Log | Type | Console msgs | Overruns | Sched overruns | Worst component | Worst time |
|-----|------|-------------|----------|----------------|-----------------|------------|
| E14 | Elim match | 387 | 102 | 106 | `robotPeriodic()` | 644.7 ms |
| Q11 | Qual match | 595 | 152 | 156 | `robotPeriodic()` | 1007.0 ms |
| E15 | Elim match | 274 | 76 | 74 | — | — |
| Practice | Practice | 393 | 16 | 2 | — | — |

### Overrun distribution by match phase (E14)

| Phase | Duration | Overruns | Rate |
|-------|----------|----------|------|
| Auto (283–298 s) | 15 s | 10 | 0.67/s |
| Teleop (308–410 s) | 102 s | 77 | 0.75/s |
| Disabled | ~393 s | 14 | 0.04/s |

**Key finding:** Overruns are **20× more frequent during enabled phases** than during disabled. This is expected — subsystem periodics and commands only run when the robot is enabled (or at least more of them run during enabled phases).

### Phase transition correlation (E14)

| Transition | Time | Overruns ±5 s |
|------------|------|---------------|
| Auto start (→ enabled+auto) | 283.2 s | 4 |
| Auto end (→ disabled) | 297.7 s | 3 |
| Teleop start (→ enabled) | 307.9 s | 6 |
| Teleop end (→ disabled) | 409.6 s | 2 |

Phase transitions correlate with overrun bursts, likely due to:
- `autonomousInit()` / `teleopInit()` / `disabledInit()` running (expensive one-shot setup)
- Command scheduling changes (old commands interrupted, new ones initialized)

The `messages` signal captures exactly which commands were involved. For example, around the auto→disabled→teleop transition in E14:

```
[297.8s] Command interrupted: C-Outpost-Depot; Cause: <none>
[297.8s] Command interrupted: Intake Default Command; Cause: <none>
[297.8s] Command interrupted: FunctionalCommand; Cause: <none>
[297.8s] Command interrupted: LED Default Command; Cause: <none>
[308.1s] Command initialized: Intake Default Command
[308.1s] Command initialized: FunctionalCommand
[308.1s] Command initialized: LED Default Command
[308.1s] Command initialized: Drive
```

Surfacing these command names alongside the overrun burst makes the report directly actionable — the team can see that the transition teardown and re-initialization of specific commands contributed to the overrun cluster.

### Component timing breakdown (E14)

Only overrun loops are logged, so these stats represent the **worst** loop iterations:

**Top offenders by max single-iteration time:**

| Component | Samples | Median | Mean | Max | Category |
|-----------|---------|--------|------|-----|----------|
| `robotPeriodic()` | 101 | 15.0 ms | 33.9 ms | 644.7 ms | Framework |
| `LiveWindow.updateValues()` | 101 | 5.4 ms | 12.6 ms | 204.9 ms | Framework |
| `autonomousInit()` | 1 | — | 122.0 ms | 122.0 ms | Init |
| `teleopInit()` | 1 | — | 100.4 ms | 100.4 ms | Init |
| `C-Outpost-Depot.execute()` | 7 | 5.0 ms | 19.2 ms | 88.5 ms | Command |
| `disabledInit()` | 2 | 65.6 ms | 65.6 ms | 67.3 ms | Init |
| `TurretSubsystem.periodic()` | 52 | 0.4 ms | 3.3 ms | 32.5 ms | Subsystem |
| `SmartDashboard.updateValues()` | 101 | 2.3 ms | 4.3 ms | 26.4 ms | Framework |
| `Drive.execute()` | 36 | 0.1 ms | 2.6 ms | 26.3 ms | Command |

**Categories of time consumers:**

1. **Framework overhead** — `robotPeriodic()`, `LiveWindow.updateValues()`, `SmartDashboard.updateValues()`, `Shuffleboard.update()`. These are WPILib infrastructure. `LiveWindow` and `SmartDashboard` can be surprisingly expensive.
2. **Phase init methods** — `autonomousInit()`, `teleopInit()`, `disabledInit()`. One-shot costs at phase transitions.
3. **Subsystem periodics** — `TurretSubsystem.periodic()`, `IntakeRollers.periodic()`, `Hood.periodic()`, etc. Run every loop when registered.
4. **Command execution** — `C-Outpost-Depot.execute()`, `Drive.execute()`, `ParallelCommandGroup.execute()`, etc. Run while scheduled.

### Reconstructed total loop time (from Tracer epochs)

Summing all Tracer-reported epochs for each overrun loop gives the total time spent in that iteration:

```
p50 =  39.6 ms   (2× the 20 ms budget)
p90 = 156.9 ms   (8× budget)
p95 = 190.9 ms   (10× budget)
max = 654.7 ms   (33× budget)
mean = 65.6 ms
```

The worst loop at 25.7 s coincided with a log file rename (`DataLog: Renamed log file`), suggesting filesystem I/O contributed.

### Overruns vs. nearby events

The worst overrun spike at 25.7 s coincided with:
```
DataLog: Renamed log file from 'FRC_TBD_...' to 'FRC_20260308_225729.wpilog'
```

This suggests that **filesystem operations** (log file rename, possibly GC) can cause multi-hundred-millisecond stalls. Other nearby overruns correlate with command scheduling bursts (multiple `Command initialized` / `Command interrupted` messages).

---

## Analysis Strategy

### Data extraction

1. **Find the `console` signal** — WPILib logs Tracer output to the `console` string signal.
2. **Parse overrun messages** — extract `IterativeRobotBase` loop overrun warnings and the configured loop period.
3. **Parse Tracer epoch breakdowns** — extract `(component_name, duration_s)` pairs from each Tracer dump.
4. **Parse CommandScheduler overruns** — count `CommandScheduler loop overrun` messages.
5. **Group by loop iteration** — Tracer dumps for a single overrun loop arrive as multiple console messages within a short time window (~0.5 s). Group them by proximity.
6. **Parse `messages` signal** — extract command lifecycle events (`Command initialized`, `Command interrupted`, `Command finished`) with their command names and timestamps. These are used for phase-transition correlation and nearby-event context.

### Metrics to compute

**Global statistics:**
- Total overrun count (IterativeRobotBase + CommandScheduler)
- Overrun rate (overruns per second, overall and per-phase)
- Estimated total loops (log duration ÷ configured period)
- Overrun percentage
- Reconstructed total loop time distribution (p50, p90, p95, p99, max)

**Per-component statistics:**
- Sample count (how many overrun loops included this component)
- Median, mean, p95, max execution time
- Fraction of total loop time consumed (mean contribution %)
- Category classification (framework / init / subsystem / command)

**Per-phase breakdown** (when `match_phases` is available):
- Overrun count and rate per phase
- Per-component stats filtered to each phase
- Phase-transition overrun clustering (±5 s around each transition)

**Temporal analysis:**
- Overrun density over time (e.g. 5-second windows)
- Burst detection — clusters of overruns in rapid succession
- Worst contiguous overrun period

**Correlation with other log events:**
- Scan `console` and `messages` signals for events near overruns:
  - CAN errors / timeouts
  - Command scheduling events (init/interrupt/finish)
  - DataLog file operations
  - NT connection events (device connect/disconnect)
  - Any other warnings or errors

### Component categorisation

```python
class ComponentCategory(Enum):
    FRAMEWORK = "framework"     # robotPeriodic, LiveWindow, SmartDashboard, Shuffleboard
    PHASE_INIT = "phase_init"   # autonomousInit, teleopInit, disabledInit, testInit
    SUBSYSTEM = "subsystem"     # *.periodic()
    COMMAND = "command"         # *.execute(), *.initialize()
    OTHER = "other"             # anything else
```

Detection rules:
- Names ending in `Init()` or containing `Init` at the end → `PHASE_INIT`
- Names matching `robotPeriodic()`, `LiveWindow.*`, `SmartDashboard.*`, `Shuffleboard.*` → `FRAMEWORK`
- Names ending in `.periodic()` (excluding `robotPeriodic`, phase periodics) → `SUBSYSTEM`
- Names ending in `.execute()` or `.initialize()` → `COMMAND`
- Phase-specific periodics like `teleopPeriodic()`, `autonomousPeriodic()` → `FRAMEWORK`

### Public data contract

```python
@dataclass
class OverrunEvent:
    """A single loop iteration that exceeded the configured period."""
    timestamp_us: int
    total_duration_ms: float
    configured_period_ms: float
    epoch_timings: dict[str, float]  # component → duration in seconds
    phase: str | None
    nearby_events: list[str]  # relevant console/messages entries within ±1s

@dataclass
class ComponentStats:
    """Aggregated timing statistics for a single Tracer component."""
    name: str
    category: ComponentCategory
    sample_count: int
    median_ms: float
    mean_ms: float
    p95_ms: float
    max_ms: float
    mean_contribution_pct: float  # % of total loop time this component consumes

@dataclass
class PhaseOverrunStats:
    """Overrun statistics for a single match phase."""
    phase: str
    duration_s: float
    overrun_count: int
    overrun_rate_hz: float
    worst_total_ms: float
    top_components: list[ComponentStats]

@dataclass
class PhaseTransitionDetail:
    """Overrun burst and command activity around a phase transition."""
    timestamp_us: int
    from_phase: str | None
    to_phase: str
    overruns_nearby: int          # within ±5 s
    init_method_ms: float | None  # e.g. autonomousInit() timing if present
    commands_initialized: list[str]
    commands_interrupted: list[str]
    commands_finished: list[str]

@dataclass
class OverrunSummary:
    """Top-level analysis result."""
    configured_period_ms: float
    estimated_total_loops: int
    overrun_count: int
    scheduler_overrun_count: int
    overrun_pct: float
    total_duration_distribution: dict[str, float]  # p50, p90, p95, p99, max
    components: list[ComponentStats]
    phase_stats: list[PhaseOverrunStats] | None
    phase_transitions: list[PhaseTransitionDetail] | None
    overrun_events: list[OverrunEvent]
    burst_periods: list[tuple[float, float, int]]  # (start_s, end_s, count)
```

---

## Output Design

### Summary

```
=== Loop Overrun Analysis ===
  Configured period:   20.0 ms (50 Hz)
  Estimated loops:     ~24,040
  Overruns:            102 (0.4%)
  Scheduler overruns:  106

  Overrun severity:
    p50 total time:    39.6 ms  (2.0× budget)
    p90 total time:    156.9 ms (7.8× budget)
    p95 total time:    190.9 ms (9.5× budget)
    max total time:    654.7 ms (32.7× budget)

  Per-phase:
    Auto     (15 s):   10 overruns (0.67/s)
    Teleop  (102 s):   77 overruns (0.75/s)
    Disabled (393 s):  14 overruns (0.04/s)
```

### Component table

```
Component                          Category    Samples  Median ms  Mean ms  p95 ms  Max ms  Contrib%
─────────────────────────────────  ──────────  ───────  ─────────  ───────  ──────  ──────  ────────
robotPeriodic()                    framework       101      15.03    33.94  142.61  644.69     51.7%
LiveWindow.updateValues()          framework       101       5.38    12.55   51.64  204.94     19.1%
SmartDashboard.updateValues()      framework       101       2.30     4.32   14.17   26.40      6.6%
teleopPeriodic()                   framework        76       2.13     3.05   11.33   17.92      4.6%
C-Outpost-Depot.execute()          command           7       4.95    19.22   88.50   88.50      2.1%
TurretSubsystem.periodic()         subsystem        52       0.40     3.25   12.69   32.50      2.6%
IntakeRollers.periodic()           subsystem        52       0.55     2.68   13.73   20.53      2.1%
autonomousInit()                   phase_init        1        —     122.02     —    122.02      —
```

### Phase transition detail

```
Phase Transition           Time     Overruns ±5s  Likely cause
────────────────────────  ───────  ─────────────  ──────────────────────
→ Auto (enabled+auto)     283.2s             4    autonomousInit() 122ms
→ Disabled (auto end)     297.7s             3    disabledInit() 64ms
→ Teleop (enabled)        307.9s             6    teleopInit() 100ms
→ Disabled (match end)    409.6s             2    disabledInit() 68ms

  Commands at auto start (283.2s):
    initialized: C-Outpost-Depot, Intake Default Command, FunctionalCommand, LED Default Command
  Commands at auto end (297.7s):
    interrupted: C-Outpost-Depot, Intake Default Command, FunctionalCommand, LED Default Command
  Commands at teleop start (307.9s):
    initialized: Intake Default Command, FunctionalCommand, LED Default Command, Drive
  Commands at match end (409.6s):
    interrupted: Intake Default Command, Drive
```

### Worst overruns detail (with `--detail` flag)

```
Worst 5 overrun loops:
  #1  25.7s (disabled)  654.7ms total
      robotPeriodic(): 644.69ms, SmartDashboard: 1.04ms, LiveWindow: 5.38ms
      Nearby: "DataLog: Renamed log file from 'FRC_TBD_...' to '...'"
  #2  23.5s (disabled)  530.5ms total
      robotPeriodic(): 186.80ms, LiveWindow: 204.94ms, ...
  #3  287.5s (auto)     242.5ms total
      robotPeriodic(): 181.07ms, LiveWindow: 35.95ms, ...
```

### `extra` dict

```python
result.extra = {
    "summary": OverrunSummary,
    "components": list[ComponentStats],
    "phase_stats": list[PhaseOverrunStats] | None,
    "overrun_events": list[OverrunEvent],
}
```

---

## Gotchas and Edge Cases

### Tracer output is split across multiple console messages

A single overrun loop's Tracer dump is often split across 3–5 consecutive console messages (WPILib's console logger has a max message length). The parser must:
- Group consecutive console messages within a ~0.5 s window as belonging to the same loop iteration.
- Concatenate the text before parsing epoch timings.
- Handle partial lines at message boundaries.

### `robotPeriodic()` is a container, not a leaf

`robotPeriodic()` includes the time for all subsystem periodics and command executions. The Tracer reports it as a single epoch, but then also reports sub-epochs (the subsystem and command timings). This means:

- **Do not double-count.** The component table should show `robotPeriodic()` as a top-level container and the subsystem/command entries as its children.
- Alternatively, present two views: "top-level epochs" (the 5 main pipeline stages) and "detailed breakdown" (all subsystems and commands).
- The contribution percentage should be calculated from the top-level epochs to avoid exceeding 100%.

### Loop period detection

The configured loop period is embedded in the overrun message: `"Loop time of 0.02s overrun"`. Parse it from the first overrun message. Default to 20 ms if no overrun messages exist.

> **Gotcha:** Some teams configure a different loop period (e.g., 10 ms for higher-rate control). The analyzer must not hard-code 20 ms.

### Logs with no overruns

If a log has zero overruns, the analyzer should still report useful information:
- "No loop overruns detected — all loops completed within the configured period."
- Estimated total loops
- Note that this doesn't mean the robot was fast — it just means no loop exceeded the budget.

### Console signal may not exist

Some log configurations may not include the `console` signal. If absent, the analyzer should report "No console signal found — loop timing data unavailable."

### Correlation window

When correlating overruns with nearby events, use a dynamic window sized to the overrun itself:

```
window = ±(1 s + overrun_duration)
```

For a typical 40 ms overrun this is ±1.04 s. For a 645 ms spike it expands to ±1.645 s, capturing the filesystem I/O or CAN timeout that may have triggered the stall. This avoids both clipping context on long overruns and pulling in too much noise on short ones.
- CAN errors, timeouts, disconnects
- Command lifecycle events (init/interrupt/finish)
- NT connection events
- DataLog file operations
- Brownout warnings

Ignore routine Tracer output (which is the overrun itself) to avoid circular references.

---

## CLI Usage

```bash
# Basic — loop overrun summary
logreader loop-overruns path/to/log.wpilog

# Detailed — show worst overruns and nearby events
logreader loop-overruns path/to/log.wpilog --detail

# Show per-phase breakdown
logreader loop-overruns path/to/log.wpilog --phases

# Show only components above a time threshold
logreader loop-overruns path/to/log.wpilog --min-time 5

# Top N worst overruns
logreader loop-overruns path/to/log.wpilog --worst 10
```

---

## Integration with match-phases

When match-phase data is available:
1. **Phase-annotate each overrun** — tag with auto/teleop/disabled.
2. **Per-phase statistics** — overrun count, rate, worst components per phase.
3. **Phase transition analysis** — detect overrun clustering around init methods.
4. **Init method attribution** — correlate `autonomousInit()` / `teleopInit()` timing with the corresponding phase transition.

When phase data is unavailable, report global statistics only.

---

## Testing Strategy

1. **No console signal** — analyzer reports "unavailable" gracefully.
2. **Console with no overruns** — analyzer reports zero overruns and estimated loop count.
3. **Single overrun with Tracer breakdown** — parses component timings correctly.
4. **Multi-message Tracer dump** — groups split console messages into one overrun event.
5. **Multiple overruns across phases** — per-phase breakdown is correct.
6. **Phase transition clustering** — detects overrun bursts near init methods.
7. **Custom loop period** — parses "Loop time of 0.01s overrun" as 10 ms, not 20 ms.
8. **Component categorisation** — correctly classifies framework/init/subsystem/command.
9. **Contribution percentage** — sums to ~100% using top-level epochs only.
10. **Correlation with nearby events** — finds CAN errors, command events near overruns.
11. **robotPeriodic() containment** — does not double-count subsystem times in contribution %.
