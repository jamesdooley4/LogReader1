# LogReader1 — Specification

## Purpose

LogReader1 is a cross-platform Python tool for FRC (FIRST Robotics Competition) teams to read, process, and analyse robot log files after matches. It targets the two primary log formats in the FRC ecosystem:

1. **WPILib `.wpilog`** — the standard data-log format produced by WPILib's `DataLog` on the roboRIO.
2. **CTRE `.hoot`** — the proprietary format produced by CTRE's Phoenix 6 signal logger.

The tool provides both a CLI and a Python API so it can be used interactively at competition or scripted into automated post-match workflows.

---

## Target Users

- FRC drive-team members reviewing match data at competition
- FRC programmers debugging robot behaviour from logs
- FRC mentors performing post-event analysis across multiple matches

---

## Platform & Runtime

| Constraint | Value |
|------------|-------|
| OS | Windows, macOS, Linux |
| Python | 3.10 – 3.14 (3.14 preferred — matches roboRIO runtime) |
| Core dependency | `robotpy-wpiutil` (provides `DataLogReader`, `DataLogRecord`, etc.) |
| Hoot support | `owlet` CLI (bundled with AdvantageScope / Phoenix Tuner) |

---

## Functional Requirements

### FR-1: Log File Reading

- **FR-1.1** Read `.wpilog` files using `wpiutil.log.DataLogReader`.
- **FR-1.2** Decode all standard data types: boolean, double, float, int64, string, and their array variants, plus raw bytes.
- **FR-1.3** Parse control records (start, finish, set-metadata) to extract signal names, types, and metadata.
- **FR-1.4** Read `.hoot` files by converting them to `.wpilog` via the `owlet` CLI, then reading the result.
- **FR-1.5** Auto-detect file format from the file extension.

### FR-2: Data Model

- **FR-2.1** Represent each signal as a named, typed collection of timestamped values.
- **FR-2.2** Timestamps are integer microseconds (as provided by `DataLogRecord.getTimestamp()`).
- **FR-2.3** Provide convenience conversions to seconds.
- **FR-2.4** Store file-level metadata (path, version, validity, signal/record counts).

### FR-3: Processing

- **FR-3.1** Filter signals by type or name prefix.
- **FR-3.2** Slice signal data by time window.
- **FR-3.3** Compute basic numeric statistics (min, max, mean, count, time range).
- **FR-3.4** Generate a human-readable file summary.

### FR-4: Analysis Framework

- **FR-4.1** Provide a base `BaseAnalyzer` class that all analysis tools inherit from.
- **FR-4.2** Auto-register analyzers via a `@register_analyzer` decorator; no manual wiring.
- **FR-4.3** Each analyzer produces a structured `AnalysisResult` with title, summary, tabular data, and arbitrary extras.
- **FR-4.4** Each registered analyzer automatically becomes a CLI subcommand.
- **FR-4.5** Analyzers can define custom CLI arguments via `add_arguments()`.

### FR-5: CLI

- **FR-5.1** `logreader info <file>` — print a log file summary.
- **FR-5.2** `logreader signals <file>` — list all signals with types and sample counts.
- **FR-5.3** `logreader stats <file> <signal>` — show statistics for a single numeric signal.
- **FR-5.4** `logreader export <file>` — export signal data to CSV.
- **FR-5.5** `logreader analyzers` — list all registered analyzers.
- **FR-5.6** `logreader <analyzer-name> <file>` — run a specific analyzer.

### FR-6: Built-in Analyzers

- **FR-6.1 `pdh-power`** — Compute watts (current × voltage) for all PDH/PDP channels. Report average power, peak power, average current, peak current, and sample count for every channel (not just top-N). Include bus voltage stats and total power summary.

---

## Non-Functional Requirements

| ID | Requirement |
|----|-------------|
| NFR-1 | Type hints on all public APIs |
| NFR-2 | PEP 8 style, enforced by `ruff` |
| NFR-3 | Unit tests via `pytest`; all analysers must have tests with synthetic data |
| NFR-4 | `pyproject.toml` for all project configuration (no `setup.py` / `setup.cfg`) |
| NFR-5 | Editable-install workflow (`pip install -e ".[dev]"`) |
| NFR-6 | Structured result objects from analyzers (not just printed strings) so downstream code can consume results programmatically |

---

## Data Flow

```
.wpilog ──────────────────────┐
                              ▼
.hoot ──► owlet ──► .wpilog ──► DataLogReader ──► LogData ──► Processor / Analyzer
                                                                   │
                                                    ┌──────────────┼──────────────┐
                                                    ▼              ▼              ▼
                                              CLI output      CSV export    Python API
```

---

## Key APIs (from `robotpy-wpiutil`)

| Class / Method | Purpose |
|----------------|---------|
| `DataLogReader(filename)` | Open and iterate a `.wpilog` file |
| `DataLogReader.isValid()` | Check file validity |
| `DataLogReader.getVersion()` | Format version number |
| `DataLogRecord.isControl()` / `.isStart()` / `.isFinish()` | Identify control records |
| `DataLogRecord.getStartData()` | Decode start record → `StartRecordData` (name, type, metadata, entry ID) |
| `DataLogRecord.getTimestamp()` | Timestamp in integer microseconds |
| `DataLogRecord.getBoolean()` / `.getDouble()` / `.getFloat()` / `.getInteger()` / `.getString()` | Typed value decoders |
| `DataLogRecord.get*Array()` | Array-typed value decoders |
| `DataLogRecord.getRaw()` | Raw bytes fallback |

---

## Hoot File Pipeline

CTRE `.hoot` files are binary and proprietary. AdvantageScope converts them using the `owlet` CLI:

```
owlet <hoot_path> <output_path> -f wpilog
```

- `owlet` reads the hoot compliancy byte at file offset 70 and selects the matching decoder.
- Compliancy must be ≥ 6 (Phoenix 2024+).
- The output is a standard `.wpilog` file readable by `DataLogReader`.
- `owlet` is located via the `OWLET_PATH` environment variable or system `PATH`.
