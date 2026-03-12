# LogReader1 — Development Plan

## Current Status

### ✅ Completed (v0.1.0)

- [x] Project scaffolding (`pyproject.toml`, editable install, CLI entry point)
- [x] Data models (`SignalType`, `SignalInfo`, `TimestampedValue`, `SignalData`, `LogData`, `LogMetadata`)
- [x] `.wpilog` reader via `DataLogReader` with full type dispatch
- [x] `.hoot` reader via `owlet` CLI conversion pipeline
- [x] Processing utilities (filter by type/prefix, time slicing, numeric stats, summarise)
- [x] CLI commands: `info`, `signals`, `stats`, `export`
- [x] Analyzer framework (`BaseAnalyzer`, `@register_analyzer`, `AnalysisResult`)
- [x] First analyzer: `pdh-power` (all-channel power draw in watts)
- [x] CLI integration: each analyzer auto-registers as a subcommand
- [x] Unit tests (28 passing) covering models, processor, utils, analyzer framework, PDH analyzer
- [x] Python 3.14 venv with `robotpy-wpiutil` 2026.2.1.1

---

## Short-Term — New Analyzers

These are the most common analysis needs for FRC teams. Each is a single file in `src/logreader/analyzers/`.

### 🔲 `battery-health`
Analyse battery voltage over time. Detect sag events (voltage drops below threshold during high-current draws), report min voltage, recovery time, and flag brownout risk.

### 🔲 `motor-performance`
For each motor controller signal (detected by naming conventions like `/SmartDashboard/Drive*` or CAN-based names), report duty cycle statistics, stall detection (high current + zero velocity), and thermal risk indicators.

### 🔲 `can-utilization`
Analyse CAN bus utilization signals if available. Flag periods of high utilization that could cause packet loss and control latency.

### 🔲 `match-phases`
Detect autonomous / teleop / disabled phase boundaries from mode signals (e.g., `DSDisabled`, `DSTeleop`, `DSAuto`). Provide per-phase breakdowns for other metrics. Other analyzers can optionally use phase boundaries.

See [Match-Phase Design](#match-phase-design) below for detailed architecture.

### 🔲 `signal-gaps`
Identify signals with missing data or irregular sample rates. Flag signals that dropped out mid-match (possible CAN disconnect, code crash, or sensor failure).

### 🔲 `mechanism-cycle`
For mechanism signals (intake, shooter, elevator), detect on/off cycles, compute cycle times, and report duty cycles. Useful for understanding mechanism utilization during a match.

### 🔲 `launch-counter`
Count game-element launches from flywheel velocity data. Optionally breaks down launches per match phase (auto / teleop / disabled), with a configurable grace period to capture spin-down launches that fire after a phase officially ends. See [Launch Counter Design](#launch-counter-design) for the detection algorithm and [Match-Phase Design](#match-phase-design) for phase integration.

---

## Medium-Term — Infrastructure

### 🔲 Multi-file / multi-match analysis
- Accept a directory of log files and aggregate results across matches
- Compare performance trends over a competition day
- Output summary tables and per-match breakdowns

### 🔲 Channel / device labelling
- Support a user-provided mapping file (JSON/YAML) that maps PDH channel numbers and signal names to human-readable device names (e.g., `Ch0 → "Front Left Drive"`)
- Apply labels in analyzer output automatically

### 🔲 Time-alignment for `.hoot` + `.wpilog` pairs
- When a team has both a `.wpilog` (roboRIO) and `.hoot` (CANivore) from the same match, align them on a common time base for combined analysis

### 🔲 Report generation
- HTML report output with tables and embedded charts (via matplotlib or plotly)
- Markdown report output for pasting into team documents
- PDF export for printing at competition

### 🔲 Configuration file
- `.logreaderrc` or `logreader.toml` for default settings: device labels, thresholds, preferred analyzers, output format

---

## Long-Term — Advanced Features

### 🔲 GUI
- Simple desktop GUI (tkinter or Dear PyGui) for non-CLI users
- Drag-and-drop log file loading
- Interactive signal browser and chart viewer

### 🔲 Live streaming
- Connect to NetworkTables (NT4) for live data viewing during practice
- Real-time analyzer dashboards

### 🔲 Struct decoding
- Decode WPILib struct-typed signals (e.g., `Pose2d`, `SwerveModuleState`) using schema definitions
- Display structured data in a readable format

### 🔲 Additional log formats
- Driver Station logs (`.dslog` / `.dsevents`)
- REV Robotics logs (`.revlog`)
- CSV import for generic data sources

### 🔲 Plugin system
- Allow teams to distribute custom analyzers as separate pip-installable packages
- Entry-point-based discovery (`logreader.analyzers` group in `pyproject.toml`)

---

## Analyzer Development Guide

To add a new analyzer:

1. Create a new file in `src/logreader/analyzers/`, e.g. `battery_health.py`
2. Define a class inheriting from `BaseAnalyzer`
3. Set `name` (kebab-case CLI name) and `description` (one-line help text)
4. Implement `run(self, log_data, **options) -> AnalysisResult`
5. Decorate with `@register_analyzer`
6. Import the module in `analyzers/__init__.py`
7. Add tests in `tests/test_analyzers.py` using synthetic `LogData`

The analyzer automatically gets a CLI subcommand. Override `add_arguments(cls, parser)` to add custom flags.

### Conventions

- Always show **all** relevant items (all channels, all motors, etc.) — not just top-N. Spotting unexpected values in "quiet" items is often the most valuable insight.
- Return structured `AnalysisResult` objects so results are consumable by code, not just humans.
- Include an `extra` dict in results for programmatic access to computed values (thresholds, totals, etc.).
- Use `_find_*` helpers to auto-detect signal prefixes so analyzers work across different robot codebases without configuration.

---

## Version Roadmap

| Version | Milestone |
|---------|-----------|
| 0.1.0 | ✅ Core reading, processing, CLI, analyzer framework, `pdh-power` |
| 0.2.0 | `battery-health`, `match-phases`, `signal-gaps` analyzers |
| 0.3.0 | Device labelling, multi-file analysis, `motor-performance` |
| 0.4.0 | Report generation (HTML/Markdown), configuration file |
| 0.5.0 | Struct decoding, additional log formats |
| 1.0.0 | Stable API, plugin system, GUI |

---

## Launch Counter Design

### Data Observations

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

### Rapid-Fire Burst Behaviour (Critical Finding)

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

### Proposed Algorithm

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

### Output Design

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

### Signal Auto-Detection

The analyzer should auto-detect the velocity signal by searching for signal names containing common patterns:
- `launcher/velocity`, `shooter/velocity`, `flywheel/velocity`
- `Launcher/Velocity`, `Shooter/Velocity`, `Flywheel/Velocity`
- Allow the user to override with `--velocity-signal <name>`
- Optionally use a current signal for confirmation if available

### CLI Usage (planned)

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

---

## Match-Phase Design

### Overview

Match-phase detection is both a **standalone analyzer** (`match-phases`) and a **shared service** that other analyzers can import to partition their results by game phase. The design has two layers:

1. **`match_phases.py`** — a module in `src/logreader/analyzers/` that exports reusable detection functions and data models.
2. **`MatchPhasesAnalyzer`** — the `@register_analyzer` CLI analyzer that uses layer 1 and presents a human-readable report.

Other analyzers (e.g. `launch-counter`, `pdh-power`, `battery-health`) import the detection functions directly — they never need to instantiate `MatchPhasesAnalyzer`.

### Data Observations

WPILib logs the Driver Station mode as boolean signals. The standard signal names are:

| Signal name | Meaning |
|-------------|---------|
| `DS:enabled` or `DSEnabled` | Robot is enabled (any mode) |
| `DS:autonomous` or `DSAuto` | Autonomous mode active |
| `DS:teleop` or `DSTeleop` | Teleoperated mode active |
| `DS:test` or `DSTest` | Test mode active |
| `DS:disabled` or `DSDisabled` | Robot disabled |
| `DS:estop` or `DSEStop` | Emergency-stopped |

Hoot-converted logs may use a different prefix (e.g. `NT:/FMSInfo/...` or CTRE-specific names). The detector must auto-detect from multiple naming conventions.

A typical FRC match has this phase sequence:

```
[Pre-match disabled] → Auto (15 s) → [Brief disabled gap] → Teleop (135 s) → [Post-match disabled]
```

But real logs may contain:
- Practice sessions with no FMS (arbitrary enable/disable sequences)
- Multiple enable cycles during pit testing
- Very short disabled gaps between auto and teleop (< 1 s)
- E-stop events cutting a phase short
- Logs that start mid-match (late recording start)
- Mid-match reboots (code crash or brownout power cycle) — see [Reboot Handling](#reboot-handling) below

### Phase Model

```python
class MatchPhase(Enum):
    """The mode the robot is operating in."""
    DISABLED = "disabled"
    AUTONOMOUS = "auto"
    TELEOP = "teleop"
    TEST = "test"

@dataclass
class PhaseInterval:
    """A single contiguous interval where the robot is in one phase.

    Attributes:
        phase: Which mode the robot is in.
        start_us: Start timestamp in microseconds (inclusive).
        end_us: End timestamp in microseconds (inclusive).
    """
    phase: MatchPhase
    start_us: int
    end_us: int

    @property
    def start_s(self) -> float:
        return self.start_us / 1_000_000.0

    @property
    def end_s(self) -> float:
        return self.end_us / 1_000_000.0

    @property
    def duration_s(self) -> float:
        return (self.end_us - self.start_us) / 1_000_000.0

    def contains_us(self, timestamp_us: int) -> bool:
        return self.start_us <= timestamp_us <= self.end_us

    def overlaps(self, start_us: int, end_us: int) -> bool:
        return self.start_us <= end_us and self.end_us >= start_us

@dataclass
class MatchPhaseTimeline:
    """Complete phase timeline for a log file.

    The intervals list is sorted by start time and covers the full log
    duration with no gaps (disabled fills the space between active phases).
    """
    intervals: list[PhaseInterval]

    @property
    def has_match(self) -> bool:
        """True if this looks like a real match (has both auto and teleop)."""
        phases = {i.phase for i in self.intervals}
        return MatchPhase.AUTONOMOUS in phases and MatchPhase.TELEOP in phases

    def phase_at(self, timestamp_us: int) -> MatchPhase:
        """Return the phase active at a given timestamp."""
        for iv in self.intervals:
            if iv.contains_us(timestamp_us):
                return iv.phase
        return MatchPhase.DISABLED

    def intervals_for(self, phase: MatchPhase) -> list[PhaseInterval]:
        """Return all intervals matching the given phase."""
        return [iv for iv in self.intervals if iv.phase == phase]

    def auto_interval(self) -> PhaseInterval | None:
        """The first (usually only) autonomous interval, or None."""
        autos = self.intervals_for(MatchPhase.AUTONOMOUS)
        return autos[0] if autos else None

    def teleop_interval(self) -> PhaseInterval | None:
        """The first (usually only) teleop interval, or None."""
        teleops = self.intervals_for(MatchPhase.TELEOP)
        return teleops[0] if teleops else None
```

### Signal Auto-Detection

```python
_MODE_PATTERNS: dict[str, list[re.Pattern]] = {
    "enabled":    [re.compile(r".*(DS[:/]?enabled|FMSInfo.*Enabled).*", re.I)],
    "autonomous": [re.compile(r".*(DS[:/]?auto|DSAutonomous|FMSInfo.*Auto).*", re.I)],
    "teleop":     [re.compile(r".*(DS[:/]?teleop|FMSInfo.*Teleop).*", re.I)],
    "test":       [re.compile(r".*(DS[:/]?test|FMSInfo.*Test).*", re.I)],
    "disabled":   [re.compile(r".*(DS[:/]?disabled).*", re.I)],
    "estop":      [re.compile(r".*(DS[:/]?e-?stop|DSEStop).*", re.I)],
}
```

Search all boolean signals for matches. The detector works with any subset — e.g. if only `DSAuto` and `DSTeleop` are present, it infers disabled as "neither auto nor teleop".

### Detection Algorithm

```
1. Auto-detect mode signals from signal names.
2. For each mode signal, extract (timestamp_us, bool) transitions.
3. Merge into a unified timeline:
   a. Walk through all mode transitions in timestamp order.
   b. At each timestamp, determine the active mode from signal priorities:
      - If estop is True → DISABLED (treat as disabled)
      - If autonomous is True → AUTONOMOUS
      - If teleop is True → TELEOP
      - If test is True → TEST
      - Otherwise → DISABLED
   c. On each mode change, close the current interval and start a new one.
4. Fill any leading/trailing time (from log start/end) with DISABLED.
5. Merge adjacent DISABLED intervals (e.g. from brief signal jitter).
6. Drop intervals shorter than a configurable minimum (default: 100 ms)
   to filter out transition noise, EXCEPT for the auto→teleop disabled gap
   which is real and meaningful.
7. Return a MatchPhaseTimeline.
```

### Reboot Handling

A mid-match reboot — caused by a code crash, a watchdog timeout, or a brownout power cycle — is important context for phase detection.

#### How reboots manifest in WPILib logs

WPILib's `DataLog` is instantiated by the robot process on startup. It creates a **new `.wpilog` file** (with a timestamp-based filename) each time the process starts. When the robot process exits — whether from a code crash, watchdog kill, or power loss — the old `DataLog` file is closed (or left truncated). When the roboRIO's systemd service restarts the process (or the roboRIO finishes rebooting after a brownout), a **new** `DataLog` is constructed, creating a **new file**.

This means a mid-match restart **always produces two separate `.wpilog` files**. However, the timestamp behaviour differs depending on the type of restart:

**Code crash (no power loss):**
The roboRIO's Linux kernel stays running. WPILib timestamps come from `CLOCK_MONOTONIC` (via `wpi::Now()`), which is a kernel-level clock that does **not** reset when the robot process restarts. So:
1. **Pre-crash file** — timestamps run from ~0 (system boot) to the crash point, e.g. 0–45 s.
2. **Post-restart file** — timestamps **continue from where the kernel clock is**, e.g. starting at ~47 s (after a ~2 s restart). They do **not** reset to zero.
3. The gap between files is typically **2–5 seconds** (just the time for systemd to restart the process and robot code to initialise).

**Full power cycle (brownout):**
The roboRIO reboots entirely, resetting the kernel's monotonic clock.
1. **Pre-crash file** — timestamps run from ~0 to the crash point. File may be truncated.
2. **Post-reboot file** — timestamps start from **~0 again** (fresh kernel boot).
3. The gap is typically **15–30 seconds** (full Linux boot + JVM/Python start + robot code init).

In both cases, the gap itself is not recorded in either file.

#### Implications for match-phase detection

Since restarts produce separate files, the **single-file** `detect_match_phases()` function does not need restart-aware logic. Each file is self-consistent with monotonic timestamps:

- **Pre-crash file**: The phase timeline ends abruptly (e.g. teleop phase has no proper end — it's inferred from the last timestamp in the file). The standalone report should note that the file appears truncated (match duration is much shorter than the expected ~155 s).
- **Post-restart file**: The phase timeline starts with a disabled interval (during robot code init), then typically enters teleop when the FMS re-enables. There's no auto phase because the FMS won't restart autonomous.

Stitching these two files into a single match timeline is the responsibility of the **multi-file analysis** feature (Medium-Term roadmap), not the single-file phase detector.

#### Identifying same-match file pairs

For code crashes (no power loss), the two files share the same `CLOCK_MONOTONIC` epoch — their timestamp ranges are **non-overlapping and contiguous**. This is a strong signal that the files belong to the same match and can be directly time-aligned without any estimation.

For brownout reboots, the clock resets, so timestamp alignment requires an external reference (FMS match timer or filename timestamps).

#### Multi-file stitching (future — multi-file analysis feature)

When the multi-file analysis feature is implemented, it should handle restarts:

```
1. Identify file pairs that belong to the same match:
   - Same event code and match number in the filename
   - Or overlapping FMS match timer values (NT:/FMSInfo/MatchTime)
   - Or user-specified grouping

2. Order the files chronologically:
   - Use FMS match timer if available (does NOT reset on reboot)
   - For code crashes: CLOCK_MONOTONIC is shared, so file timestamps
     are directly comparable (file 2's timestamps are later than file 1's)
   - For brownouts: use filename timestamps or FMS match timer

3. Stitch the phase timelines:
   - File 1's timeline covers match start to crash
   - Insert a DISABLED gap for the restart duration
   - File 2's timeline covers recovery to match end
   - Produce a combined MatchPhaseTimeline with a reboot annotation

4. Determine the restart gap:
   - Code crash (shared clock): gap = file2_first_timestamp - file1_last_timestamp
     (exact, no estimation needed)
   - Brownout with FMS timer: gap = file2_first_fms_time - file1_last_fms_time
   - Brownout without FMS timer: use filename timestamp difference,
     or a heuristic default (15 s)
```

#### Data model additions

The `MatchPhaseTimeline` should still support restart annotations for the multi-file case:

```python
@dataclass
class RestartEvent:
    """A detected mid-match restart (from multi-file stitching)."""
    pre_crash_last_us: int     # last timestamp before the crash (file 1)
    post_restart_first_us: int # first timestamp after restart (file 2)
    gap_s: float               # wall-clock duration of the gap
    restart_type: str           # "code-crash" | "power-cycle"
    gap_confidence: str         # "exact" (shared clock) | "fms-anchored" | "estimated"
    pre_file: str              # path to the pre-crash log file
    post_file: str             # path to the post-restart log file

@dataclass
class MatchPhaseTimeline:
    intervals: list[PhaseInterval]
    restarts: list[RestartEvent] = field(default_factory=list)

    @property
    def had_restart(self) -> bool:
        return len(self.restarts) > 0
```

#### Single-file truncation detection

Even though restarts don't appear within a single file, the phase detector should flag when a file appears to be a **truncated match fragment**:

- If the file contains auto but no teleop, and auto ends abruptly → likely crashed during or just after auto.
- If the file starts directly in teleop with no auto → likely post-reboot second half.
- If total enabled time is much less than 150 s and the file has FMS match info → likely a fragment.

This is exposed via the `MatchPhaseTimeline`:

```python
@property
def appears_truncated(self) -> bool:
    """True if this looks like a partial match (possible crash/reboot)."""
    total_enabled = sum(
        iv.duration_s for iv in self.intervals
        if iv.phase != MatchPhase.DISABLED
    )
    return self.has_match and total_enabled < 120.0  # expected ~150 s

@property
def appears_post_reboot(self) -> bool:
    """True if this looks like the second half of a rebooted match."""
    if not self.intervals:
        return False
    first_enabled = next(
        (iv for iv in self.intervals if iv.phase != MatchPhase.DISABLED),
        None,
    )
    return (
        first_enabled is not None
        and first_enabled.phase == MatchPhase.TELEOP  # no auto at all
        and self.has_match is False
    )
```

#### Impact on consumers

- **Single-file analyzers** — work normally on each file independently. The `appears_truncated` / `appears_post_reboot` flags let them add a note to their output (e.g. "⚠ This appears to be a partial match log").
- **Multi-file stitching** (future) — produces a combined `MatchPhaseTimeline` with `had_restart = True`. All the consumer APIs (`classify_events_by_phase`, `slice_signal_by_phase`, etc.) work on the stitched timeline unchanged.
- **The standalone `match-phases` report** should show a warning when truncation is detected:

```
=== Match Phase Timeline ===
  ⚠ This log appears to be a truncated match (45.2 s enabled, expected ~150 s).
    Possible mid-match crash or reboot. Look for a second log file from
    the same match.

  Phase       Start(s)   End(s)   Duration(s)
  ---------   --------   ------   -----------
  disabled         0.0     2.3           2.3
  auto             2.3    17.3          15.0
  disabled        17.3    17.8           0.5
  teleop          17.8    47.5          29.7
```

#### Edge cases

| Scenario | Behaviour |
|----------|-----------|
| Code crash during teleop | Pre-crash file has auto + partial teleop. Post-restart file has disabled + teleop (no auto). Timestamps share the same `CLOCK_MONOTONIC` epoch — gap is exact. |
| Brownout during auto | Pre-crash file has partial auto only. Post-reboot file starts in disabled, then teleop. Clock resets to ~0 — gap requires FMS timer or estimation. |
| Multiple crashes | Multiple files, each a fragment. Multi-file stitching chains them. Code crashes with shared clocks can be ordered exactly. |
| Clean match (no crash) | Single file, `appears_truncated` is `False`. No restart concerns. |
| Practice/pit (no FMS) | Short enable cycles are normal, not truncation. `has_match` is `False`, so `appears_truncated` is `False` (it only flags when `has_match` is `True`). |

#### Testing additions

Add to the match-phases test suite:

9. **Truncated match file** — synthetic data with auto (15 s) + teleop (30 s) only. Verify `appears_truncated` is `True`.
10. **Post-reboot file** — synthetic data starting with disabled + teleop, no auto. Verify `appears_post_reboot` is `True`.
11. **Full match** — verify `appears_truncated` is `False` and `appears_post_reboot` is `False`.
12. **Practice session** — short enables with no FMS. Verify `appears_truncated` is `False` (because `has_match` is `False`).

### Public API (for other analyzers)

The module exports these functions that any analyzer can import:

```python
def detect_match_phases(log_data: LogData) -> MatchPhaseTimeline | None:
    """Detect match phases from mode signals in the log.

    Returns None if no mode signals are found (the log has no phase data).
    Returns a MatchPhaseTimeline with at least one interval otherwise.
    """

def classify_events_by_phase(
    timeline: MatchPhaseTimeline,
    events: Sequence[tuple[int | float, T]],
    *,
    timestamps_are_seconds: bool = False,
    grace_period_s: float = 0.0,
) -> dict[MatchPhase, list[T]]:
    """Partition a list of (timestamp, event) pairs by match phase.

    Parameters:
        timeline: The phase timeline.
        events: Sequence of (timestamp, event_object) pairs.
        timestamps_are_seconds: If True, timestamps are float seconds;
            if False (default), int microseconds.
        grace_period_s: Extend each phase boundary by this many seconds
            when classifying. Events that occur within *grace_period_s*
            after a phase ends are still attributed to that phase.
            This handles hardware lag (e.g. flywheel spin-down launches
            that fire after teleop officially ends).

    Returns:
        Dict mapping each MatchPhase to a list of events that occurred
        during (or within the grace period of) that phase.
    """

def slice_signal_by_phase(
    timeline: MatchPhaseTimeline,
    signal: SignalData,
    phase: MatchPhase,
    *,
    grace_period_s: float = 0.0,
) -> SignalData:
    """Extract the portion of a signal that falls within a given phase.

    Like processor.slice_by_time but driven by match phases.
    """

def phase_durations(timeline: MatchPhaseTimeline) -> dict[MatchPhase, float]:
    """Return total duration in seconds for each phase."""
```

### Grace Period (Spin-Down / Lag Handling)

Many robot mechanisms don't stop instantly when a phase ends. The most common case is a flywheel that keeps spinning (and can still launch game elements) for 0.5–2 seconds after teleop ends, while the robot transitions to disabled.

The `grace_period_s` parameter on `classify_events_by_phase` and `slice_signal_by_phase` handles this:

- A launch at `t = teleop_end + 0.3s` with `grace_period_s=1.0` is attributed to TELEOP, not DISABLED.
- This is purely a **classification** feature — it doesn't move phase boundaries. The timeline itself remains accurate.
- Default is `0.0` (strict boundaries). Analyzers that know about hardware lag opt in to a grace period.

### Usage by Other Analyzers

#### Launch Counter — per-phase breakdown

```python
from logreader.analyzers.match_phases import (
    detect_match_phases, classify_events_by_phase, MatchPhase,
)

class LaunchCounterAnalyzer(BaseAnalyzer):
    def run(self, log_data, **options):
        # ... existing detection logic ...
        launches = detect_launches(vel_sig, ...)

        # Phase breakdown (if phase data available)
        timeline = detect_match_phases(log_data)
        if timeline is not None:
            events = [(l.time_s, l) for l in launches]
            by_phase = classify_events_by_phase(
                timeline,
                events,
                timestamps_are_seconds=True,
                grace_period_s=2.0,  # flywheel spin-down
            )
            # Add per-phase rows to the result:
            # "Auto: 12 launches, Teleop: 44, Disabled: 0"
            # Include launches-during-spindown note when
            # disabled count would have been nonzero without grace.
```

#### PDH Power — per-phase power totals

```python
timeline = detect_match_phases(log_data)
if timeline is not None:
    for phase in [MatchPhase.AUTONOMOUS, MatchPhase.TELEOP, MatchPhase.DISABLED]:
        phase_data = slice_signal_by_phase(timeline, current_sig, phase)
        # compute average/peak power during this phase
        # NOTE: power can go NEGATIVE during disabled phases —
        # motors coast and act as generators (back-EMF feeds
        # current back into the bus). Track min power as well
        # as max; negative values are electrically real and
        # indicate regenerative energy from spinning mechanisms.
```

#### Battery Health — per-phase voltage sag

```python
timeline = detect_match_phases(log_data)
if timeline is not None:
    auto_voltage = slice_signal_by_phase(timeline, volt_sig, MatchPhase.AUTONOMOUS)
    teleop_voltage = slice_signal_by_phase(timeline, volt_sig, MatchPhase.TELEOP)
    # Compare min voltage, sag events, etc. across phases
```

### Standalone Analyzer Output

When run as `logreader match-phases <file>`, the analyzer reports:

```
=== Match Phase Timeline ===
  Log duration:   155.2 s
  Match detected: Yes

  Phase       Start(s)   End(s)   Duration(s)
  ---------   --------   ------   -----------
  disabled         0.0     2.3           2.3
  auto             2.3    17.3          15.0
  disabled        17.3    17.8           0.5
  teleop          17.8   152.8         135.0
  disabled       152.8   155.2           2.4

  Summary:
    Autonomous: 15.0 s
    Teleop:     135.0 s
    Disabled:   5.2 s (3 intervals)
```

The `extra` dict in `AnalysisResult` contains the full `MatchPhaseTimeline` object for programmatic consumers:

```python
result.extra["timeline"]   # MatchPhaseTimeline
result.extra["has_match"]  # bool
result.extra["durations"]  # dict[MatchPhase, float]
```

### File Organisation

```
src/logreader/analyzers/
    match_phases.py          # MatchPhase, PhaseInterval, MatchPhaseTimeline,
                             # detect_match_phases(), classify_events_by_phase(),
                             # slice_signal_by_phase(), phase_durations(),
                             # MatchPhasesAnalyzer (the CLI analyzer)
```

Everything lives in one module. The data models and utility functions are importable by other analyzer modules directly:

```python
from logreader.analyzers.match_phases import (
    MatchPhase,
    MatchPhaseTimeline,
    PhaseInterval,
    detect_match_phases,
    classify_events_by_phase,
    slice_signal_by_phase,
    phase_durations,
)
```

### CLI Usage

```bash
# Show match phase timeline
logreader match-phases path/to/log.wpilog

# Launch counter with per-phase breakdown
logreader launch-counter path/to/log.wpilog --by-phase

# Launch counter with custom spin-down grace period
logreader launch-counter path/to/log.wpilog --by-phase --grace-period 2.0
```

### Testing Strategy

Tests use synthetic `LogData` with crafted boolean mode signals:

1. **Standard match** — auto (15 s) + disabled gap + teleop (135 s) → verify correct intervals.
2. **Practice session** — multiple enable/disable cycles, no FMS → verify no crash, reasonable intervals.
3. **No mode signals** — `detect_match_phases()` returns `None`.
4. **Partial signals** — only `DSAuto` and `DSTeleop` present (no `DSDisabled`) → infer disabled.
5. **E-stop** — phase cut short mid-teleop → verify early termination.
6. **Grace period** — events after phase end classified correctly with/without grace.
7. **`classify_events_by_phase`** — unit test with known events and known timeline.
8. **`slice_signal_by_phase`** — verify returned `SignalData` contains only in-phase samples.
