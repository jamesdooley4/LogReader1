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
- [x] Match-phase infrastructure and analyzer (`match-phases`) with DS boolean + `FMSControlData` fallback
- [x] Per-phase integration in analyzers where useful (currently `pdh-power`)
- [x] CLI integration: each analyzer auto-registers as a subcommand
- [x] Pytest coverage for models, processor, utils, analyzer framework, and implemented analyzers
- [x] Python 3.14 venv with `robotpy-wpiutil` 2026.2.1.1

---

## Short-Term — New Analyzers

These are the most common analysis needs for FRC teams. Each is a single file in `src/logreader/analyzers/`.

### `battery-health` *(planned)*
Analyse battery voltage over time. Detect sag events (voltage drops below threshold during high-current draws), report min voltage, recovery time, and flag brownout risk.

### `motor-performance` *(planned)*
For each motor controller signal (detected by naming conventions like `/SmartDashboard/Drive*` or CAN-based names), report duty cycle statistics, stall detection (high current + zero velocity), and thermal risk indicators.

### `can-utilization` *(planned)*
Analyse CAN bus utilization signals if available. Flag periods of high utilization that could cause packet loss and control latency.

### `match-phases` *(implemented)*
Detect autonomous / teleop / disabled phase boundaries from mode signals (e.g., `DSDisabled`, `DSTeleop`, `DSAuto`) or the `FMSControlData` bitflag. Provides reusable per-phase breakdowns for other analyzers.

**Design doc:** [docs/design-match-phases.md](docs/design-match-phases.md)

### `signal-gaps` *(design complete)*
Identify signals with missing data, irregular sample rates, or mid-match dropouts. Auto-classifies signals (continuous, active-only, heartbeat, event, config) and applies appropriate gap detection for each type. Detects device reboots via heartbeat counter resets. Phase-aware — distinguishes expected disabled-period silence from concerning mid-match dropouts.

**Design doc:** [docs/design-signal-gaps.md](docs/design-signal-gaps.md)

### `loop-overruns` *(implemented)*
Detect and report robot main-loop overruns caused by excessive CPU usage on the roboRIO. Parses WPILib `Tracer` timing breakdowns from the `console` log signal to identify which subsystems and commands consume the most time. Reports overrun statistics (count, rate, severity distribution), per-component timing breakdowns, phase-transition correlation, and correlation with nearby log events (CAN errors, command scheduling, filesystem I/O).

**Design doc:** [docs/design-loop-overruns.md](docs/design-loop-overruns.md)

### `unnamed-commands` *(implemented)*
Find and report commands using WPILib default class names instead of meaningful names. When teams use inline or anonymous commands (e.g. `new InstantCommand(...)` without subclassing or calling `.withName()`), the scheduler logs them with their generic class name, making debugging, overrun analysis, and match review much harder. This analyzer scans the `messages` signal for command lifecycle events and the `NT:/LiveWindow/Ungrouped/Scheduler/Names` string-array signal (which contains the list of currently running commands each loop) to flag any commands whose names match a known set of WPILib built-in command class names.

**Default command class names** (from [`wpilibNewCommands`](https://github.com/wpilibsuite/allwpilib/tree/7ca35e5678cf32caec6a1a866ca51d0136c4c398/wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command)):

- **Instant / simple:** `InstantCommand`, `RunCommand`, `StartEndCommand`, `FunctionalCommand`, `PrintCommand`
- **Composition:** `SequentialCommandGroup`, `ParallelCommandGroup`, `ParallelDeadlineGroup`, `ParallelRaceGroup`
- **Wrappers / control flow:** `ConditionalCommand`, `SelectCommand`, `ProxyCommand`, `RepeatCommand`, `DeferredCommand`, `ScheduleCommand`, `WrapperCommand`
- **Timing:** `WaitCommand`, `WaitUntilCommand`
- **PID / motion:** `PIDCommand`, `ProfiledPIDCommand`, `TrapezoidProfileCommand`, `MecanumControllerCommand`, `RamseteCommand`, `SwerveControllerCommand`
- **Notifications:** `NotifierCommand`

The report should list each unnamed command, how many times it was initialized / interrupted / finished, and the match phases where it appeared — giving the team a clear checklist of commands to name with `.withName()` or by subclassing.

### `mechanism-cycle` *(planned)*
For mechanism signals (intake, shooter, elevator), detect on/off cycles, compute cycle times, and report duty cycles. Useful for understanding mechanism utilization during a match.

### `drive-analysis` *(design complete)*
Automatically discover swerve drive and steer motors by scoring candidates on current draw, velocity continuity, and correlation with chassis translational/rotational speed. Detect grip-limited (traction loss), supply-current-limited, and stator-current-limited intervals by comparing wheel behavior against a chassis-motion reference from `pose-analysis`.

**Design doc:** [docs/design-drive-analysis.md](docs/design-drive-analysis.md)

### `pose-analysis` *(design complete)*
Build the best offline estimate of robot pose by fusing wheel odometry, vision camera poses, and accelerometer data. Measure how each sensor modality correlates or diverges from this best-estimate reference path. Detect wheel slip, vision outliers, and physical impacts. Produce concrete tuning recommendations for real-time pose estimators. Also serves as a shared service layer for other analyzers (especially `drive-analysis`) that need a single notion of actual robot motion.

**Design doc:** [docs/design-pose-analysis.md](docs/design-pose-analysis.md)

### `launch-counter` *(design complete / implementation in progress)*
Count game-element launches from flywheel velocity data. Optionally breaks down launches per match phase (auto / teleop / disabled), with a configurable grace period to capture spin-down launches that fire after a phase officially ends.

**Design doc:** [docs/design-launch-counter.md](docs/design-launch-counter.md)

### `intake-analysis` *(design complete)*
Analyse the intake pivot position to detect extension/retraction events, measure move timing, and flag outliers. Tracks three discrete states: IN (retracted), OUT (extended), and INTERMEDIATE (partially retracted with hopper full of fuel elements). Measures the delay between target position changes and the intake arriving, identifying slow retractions that indicate a full hopper or mechanical issues. Correlates intake roller speed dips with fuel entry events, and cross-references with `launch-counter` data (fuel leaving the robot) and `hard-hits` data (fuel loss after collisions or bump crossings) to build a complete picture of fuel flow through the robot.

**Design doc:** [docs/design-intake-analysis.md](docs/design-intake-analysis.md)

### `vision-analysis` *(implemented — Tiers 1–4)*
Analyse Limelight vision performance across one or more cameras. Per-frame quality metrics, per-match summary tables, field-position heatmaps, temporal diagnostics, multi-camera agreement, and optional PNG plot generation (`--plots`). Includes `t2d`-derived tag geometry quality proxies and cross-match validation of aspect-ratio-based quality patterns across 18 Bonney Lake matches.

**Design doc:** [docs/design-vision-analysis.md](docs/design-vision-analysis.md)

---

## Medium-Term — Infrastructure

### Multi-file / multi-match analysis
- [x] `export-results` command — run any analyzer on multiple log files and export summary-level results to a single JSON file for cross-match analysis. Large per-frame arrays are automatically skipped; summary tables, scalar metrics, and aggregate stats are preserved. (~50KB per match for vision-analysis.)
- [ ] Accept a directory of log files and aggregate results across matches
- [ ] Compare performance trends over a competition day
- [ ] Output summary tables and per-match breakdowns
- [ ] Stitch split log files from mid-match reboots (see [reboot handling](docs/design-match-phases.md#reboot-handling))

### Channel / device labelling
- Support a user-provided mapping file (JSON/YAML) that maps PDH channel numbers and signal names to human-readable device names (e.g., `Ch0 -> "Front Left Drive"`)
- Apply labels in analyzer output automatically

### Time-alignment for `.hoot` + `.wpilog` pairs
- When a team has both a `.wpilog` (roboRIO) and `.hoot` (CANivore) from the same match, align them on a common time base for combined analysis

### Report generation
- HTML report output with tables and embedded charts (via matplotlib or plotly)
- Markdown report output for pasting into team documents
- PDF export for printing at competition

### Configuration file
- `.logreaderrc` or `logreader.toml` for default settings: device labels, thresholds, preferred analyzers, output format

---

## Long-Term — Advanced Features

### GUI
- Simple desktop GUI (tkinter or Dear PyGui) for non-CLI users
- Drag-and-drop log file loading
- Interactive signal browser and chart viewer

### Live streaming
- Connect to NetworkTables (NT4) for live data viewing during practice
- Real-time analyzer dashboards

### Struct decoding
- Decode WPILib struct-typed signals (e.g., `Pose2d`, `SwerveModuleState`) using schema definitions
- Display structured data in a readable format

### Additional log formats
- Driver Station logs (`.dslog` / `.dsevents`)
- REV Robotics logs (`.revlog`)
- CSV import for generic data sources

### Plugin system
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
8. If the design is non-trivial, add a design doc in `docs/design-<name>.md`

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
| 0.1.0 | Core reading, processing, CLI, analyzer framework, `pdh-power`, `match-phases` |
| 0.2.0 | `battery-health`, `signal-gaps`, `launch-counter`, `loop-overruns`, `unnamed-commands` analyzers |
| 0.3.0 | Device labelling, multi-file analysis, `motor-performance` |
| 0.4.0 | Report generation (HTML/Markdown), configuration file |
| 0.5.0 | Struct decoding, additional log formats |
| 1.0.0 | Stable API, plugin system, GUI |

---

## Design Documents

Detailed design docs for fully fleshed-out features live in `docs/`:

| Document | Feature |
|----------|---------|
| [design-launch-counter.md](docs/design-launch-counter.md) | Launch counter algorithm, data observations, rapid-fire burst handling |
| [design-match-phases.md](docs/design-match-phases.md) | Match-phase detection, FMSControlData fallback, reboot handling, public API, grace periods |
| [design-signal-gaps.md](docs/design-signal-gaps.md) | Signal gap detection, auto-classification, heartbeat resets, phase-aware filtering |
| [design-loop-overruns.md](docs/design-loop-overruns.md) | Loop overrun detection, Tracer parsing, component breakdown, phase correlation |
| [design-pose-analysis.md](docs/design-pose-analysis.md) | Pose fusion, divergence metrics, vision quality gating, public API for chassis motion |
| [design-drive-analysis.md](docs/design-drive-analysis.md) | Drive motor discovery, motor scoring, traction loss, supply/stator current limiting |
| [design-intake-analysis.md](docs/design-intake-analysis.md) | Intake position state tracking, move timing outliers, roller dip fuel detection, cross-analyzer correlation |
| [design-vision-analysis.md](docs/design-vision-analysis.md) | Limelight per-frame metrics, per-tag-ID/distance-band tables, field heatmaps, pose residuals, temporal diagnostics |
