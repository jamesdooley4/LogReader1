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

## Multi-File Processing Performance

When analysing a full competition day (10–20+ match files), sequential processing is a significant bottleneck. Each `.wpilog` file takes 5–30 seconds to load and analyse depending on file size and analyzer complexity. A 20-match event can take 5–10 minutes sequentially.

### Design Constraints (Why Parallelism Is Safe)

The current architecture is already well-suited for parallelism:

1. **File processing is independent.** `read_wpilog()` and `read_hoot()` are pure functions: filepath in, `LogData` out, no shared state.
2. **Analyzers are stateless.** Each `BaseAnalyzer.run(log_data)` call receives a self-contained `LogData` and returns an `AnalysisResult`. No analyzer modifies global state during execution.
3. **The analyzer registry is read-only at runtime.** `_REGISTRY` is populated at import time via `@register_analyzer` decorators and only read during processing.
4. **`LogData` objects are not shared.** Each file produces its own `LogData` that is consumed by exactly one analyzer invocation.

### Implementation Plan

#### Phase 1: `concurrent.futures` Process Pool (recommended)

Use `concurrent.futures.ProcessPoolExecutor` with `max_workers` defaulting to `min(cpu_count(), len(files), 8)`:

```python
from concurrent.futures import ProcessPoolExecutor, as_completed

def _process_one_file(file_path: str, analyzer_name: str) -> dict | None:
    """Process a single file — runs in a worker process."""
    log_data = _read_log(file_path)
    analyzer = get_analyzer(analyzer_name)()
    result = analyzer.run(log_data)
    entry = result.to_dict()
    entry["source_file"] = Path(file_path).name
    return entry

with ProcessPoolExecutor(max_workers=workers) as pool:
    futures = {
        pool.submit(_process_one_file, fp, analyzer_name): fp
        for fp in files
    }
    for future in as_completed(futures):
        result = future.result()  # raises if worker crashed
        if result:
            results.append(result)
```

- **Why processes, not threads?** Log parsing is CPU-bound (decoding binary records, computing statistics). Python's GIL prevents true parallelism with threads for CPU work. `ProcessPoolExecutor` sidesteps the GIL entirely.
- **Memory consideration:** Each worker loads a full `LogData` (50–200 MB for large logs). With 8 workers, peak memory could reach ~1.5 GB. The `max_workers` cap of 8 limits this.
- **Hoot conversion:** `read_hoot()` spawns an `owlet` subprocess. This is I/O-bound and parallelizes well — multiple `owlet` processes can run concurrently.

#### Phase 2: Progress Reporting

Print per-file status as each completes:

```
Processing 20 files with 8 workers...
  [1/20] FRC_20260321_184843_WASAM_Q4.wpilog — done (3.2s)
  [2/20] FRC_20260321_191931_WASAM_Q7.wpilog — done (2.8s)
  ...
  [20/20] FRC_20260322_233702_WASAM_E16.wpilog — done (4.1s)
Completed 20 files in 12.4s (vs ~58s sequential)
```

#### Phase 3: CLI Integration

Add `--workers` flag to `export-results` and any future multi-file commands:

```bash
# Auto-detect worker count
logreader export-results vision-analysis *.wpilog -o results.json

# Explicit worker count
logreader export-results vision-analysis *.wpilog -o results.json --workers 4

# Sequential (for debugging or low-memory systems)
logreader export-results vision-analysis *.wpilog -o results.json --workers 1
```

#### Fallback: Sequential When Needed

Maintain `--workers 1` as a fully supported mode for:
- Debugging analyzer issues (stack traces are clearer in-process)
- Low-memory systems
- Environments where `multiprocessing` is unavailable (some embedded Python distributions)

### Constraints and Edge Cases

- **Result ordering:** `as_completed` returns results in completion order, not input order. Sort results by filename before output to ensure deterministic CSV/JSON.
- **Error isolation:** A crash in one worker must not abort the entire batch. Catch exceptions per-future and report them as skipped files with error messages.
- **Hoot temp files:** `read_hoot()` creates temporary `.wpilog` files during conversion. Each worker must use its own temp directory to avoid collisions. The current implementation already uses `tempfile` for this.
- **Matplotlib in workers:** Plot generation (`--plots`) uses matplotlib, which may not be fork-safe. If `--plots` is requested with `--workers > 1`, either generate plots in the main process after collecting results, or use `spawn` start method instead of `fork`.
- **No new dependencies:** `concurrent.futures` is in the Python standard library.

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
