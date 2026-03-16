# LogReader1

FRC Robot Log Reader — read and process WPILib `.wpilog` and CTRE `.hoot` log files. Works on Windows, macOS, and Linux.

## Features

- **WPILib `.wpilog` reader** — Uses `robotpy-wpiutil` (`DataLogReader`) to parse all signal types: boolean, double, float, int64, string, arrays, raw, and structs.
- **CTRE `.hoot` reader** — Converts `.hoot` files to `.wpilog` via the `owlet` CLI (from AdvantageScope / Phoenix Tuner), then reads them with the standard reader.
- **Data processing** — Filter signals by type or name prefix, slice by time window, compute numeric statistics.
- **CSV export** — Export signal data for external analysis.
- **CLI interface** — Quick commands for inspecting, listing, analysing, and exporting log data.

## Requirements

- **Python 3.10 – 3.14** (3.14 recommended — matches the robot runtime)
- **Windows, macOS, or Linux**
- **owlet** CLI (optional, for `.hoot` support) — bundled with [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope)

## Installation

```bash
# Clone the repository
git clone <repo-url> LogReader1
cd LogReader1

# Create a virtual environment
python -m venv .venv
.venv\Scripts\activate

# Install in editable mode with dev dependencies
pip install -e ".[dev]"
```

## Usage

### CLI

```bash
# Print a log file summary
logreader info path/to/log.wpilog

# List all signals
logreader signals path/to/log.wpilog

# Show statistics for a specific signal
logreader stats path/to/log.wpilog "/SmartDashboard/Speed"

# Export all signals to CSV
logreader export path/to/log.wpilog

# Export a single signal
logreader export path/to/log.wpilog -s "/SmartDashboard/Speed" -o speed.csv
```

### Analyzers

```bash
# List available analyzers
logreader analyzers

# Run an analyzer
logreader vision-analysis path/to/log.wpilog
logreader pdh-power path/to/log.wpilog
logreader hard-hits path/to/log.hoot

# Vision analysis with plots
logreader vision-analysis path/to/log.wpilog --plots --output-dir ./plots/

# Export results from multiple matches to JSON for cross-match analysis
logreader export-results vision-analysis match1.wpilog match2.wpilog -o results.json
```

### Hoot files

Hoot files require the `owlet` CLI. Either:

1. Add `owlet` to your system `PATH`, or
2. Set the `OWLET_PATH` environment variable to the `owlet` executable.

Then use the CLI normally — format is auto-detected by extension:

```bash
logreader info path/to/log.hoot
```

### Python API

```python
from logreader.wpilog_reader import read_wpilog
from logreader.processor import summarise, compute_numeric_stats

log_data = read_wpilog("path/to/log.wpilog")
print(summarise(log_data))

speed = log_data.get_signal("/SmartDashboard/Speed")
if speed:
    stats = compute_numeric_stats(speed)
    print(stats)
```

## Development

```bash
# Run tests
pytest

# Run tests with coverage
pytest --cov=logreader

# Lint
ruff check src/
```

## Project Structure

```
pyproject.toml          # Project config, dependencies, CLI entry point
src/logreader/
    __init__.py         # Package init
    models.py           # Data models (SignalInfo, LogData, etc.)
    wpilog_reader.py    # WPILib .wpilog file reader
    hoot_reader.py      # CTRE .hoot file reader (owlet conversion)
    processor.py        # Data processing and analysis
    cli.py              # Command-line interface
    utils.py            # Shared utilities
    analyzers/
        base.py         # BaseAnalyzer, AnalysisResult, registry
        pdh_power.py    # Power draw analysis
        match_phases.py # Match phase detection (auto/teleop/disabled)
        loop_overruns.py# Robot loop overrun detection
        hard_hits.py    # Collision/impact detection from IMU
        pose_analysis.py# Pose fusion and divergence analysis
        vision_analysis.py # Limelight vision performance (Tiers 1-4)
        vision_plots.py # Optional matplotlib plots for vision analysis
        launch_counter.py  # Game element launch counting
        unnamed_commands.py # Find unnamed WPILib commands
tests/
    test_models.py      # Model tests
    test_processor.py   # Processor tests
    test_utils.py       # Utility tests
    test_analyzers.py   # Analyzer framework & built-in analyzer tests
    test_vision_analysis.py # Vision analysis tests
docs/
    design-*.md         # Design documents for each analyzer
```

## Key APIs Used

| API | Source |
|-----|--------|
| `wpiutil.log.DataLogReader` | Reads `.wpilog` files |
| `wpiutil.log.DataLogRecord` | Individual records with typed getters |
| `wpiutil.log.StartRecordData` | Signal metadata from start control records |
| `owlet` CLI | Converts CTRE `.hoot` → `.wpilog` |

## License

MIT
