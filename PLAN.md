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

### `battery-health`
Analyse battery voltage over time. Detect sag events (voltage drops below threshold during high-current draws), report min voltage, recovery time, and flag brownout risk.

### `motor-performance`
For each motor controller signal (detected by naming conventions like `/SmartDashboard/Drive*` or CAN-based names), report duty cycle statistics, stall detection (high current + zero velocity), and thermal risk indicators.

### `can-utilization`
Analyse CAN bus utilization signals if available. Flag periods of high utilization that could cause packet loss and control latency.

### `match-phases`
Detect autonomous / teleop / disabled phase boundaries from mode signals (e.g., `DSDisabled`, `DSTeleop`, `DSAuto`) or the `FMSControlData` bitflag. Provide per-phase breakdowns for other metrics. Other analyzers can optionally use phase boundaries.

**Design doc:** [docs/design-match-phases.md](docs/design-match-phases.md)

### `signal-gaps`
Identify signals with missing data or irregular sample rates. Flag signals that dropped out mid-match (possible CAN disconnect, code crash, or sensor failure).

### `mechanism-cycle`
For mechanism signals (intake, shooter, elevator), detect on/off cycles, compute cycle times, and report duty cycles. Useful for understanding mechanism utilization during a match.

### `launch-counter`
Count game-element launches from flywheel velocity data. Optionally breaks down launches per match phase (auto / teleop / disabled), with a configurable grace period to capture spin-down launches that fire after a phase officially ends.

**Design doc:** [docs/design-launch-counter.md](docs/design-launch-counter.md)

---

## Medium-Term — Infrastructure

### Multi-file / multi-match analysis
- Accept a directory of log files and aggregate results across matches
- Compare performance trends over a competition day
- Output summary tables and per-match breakdowns
- Stitch split log files from mid-match reboots (see [reboot handling](docs/design-match-phases.md#reboot-handling))

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
| 0.1.0 | Core reading, processing, CLI, analyzer framework, `pdh-power` |
| 0.2.0 | `battery-health`, `match-phases`, `signal-gaps` analyzers |
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
