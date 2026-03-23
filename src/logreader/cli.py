"""Command-line interface for LogReader."""

from __future__ import annotations

import argparse
import csv
import io
import json
import os
import sys
import time
from pathlib import Path

from logreader.analyzers import get_analyzer, list_analyzers
from logreader.models import LogData
from logreader.processor import compute_numeric_stats, summarise
from logreader.utils import file_extension


def _read_log(path: str) -> LogData:
    """Read a log file, auto-detecting format by extension."""
    ext = file_extension(path)
    if ext == ".wpilog":
        from logreader.wpilog_reader import read_wpilog

        return read_wpilog(path)
    elif ext == ".hoot":
        from logreader.hoot_reader import read_hoot

        print(
            "Converting .hoot file via owlet (this may take 10-30s for "
            "large files)...",
            file=sys.stderr,
            flush=True,
        )
        return read_hoot(path)
    else:
        print(
            f"Error: unsupported file extension '{ext}'. Supported: .wpilog, .hoot",
            file=sys.stderr,
        )
        sys.exit(1)


def cmd_info(args: argparse.Namespace) -> None:
    """Print a summary of a log file."""
    log_data = _read_log(args.file)
    print(summarise(log_data))


def cmd_signals(args: argparse.Namespace) -> None:
    """List all signal names in a log file."""
    log_data = _read_log(args.file)
    for name in log_data.signal_names():
        sig = log_data.signals[name]
        print(f"{name}\t{sig.info.type.value}\t{len(sig.values)} pts")


def cmd_stats(args: argparse.Namespace) -> None:
    """Print statistics for a specific signal."""
    log_data = _read_log(args.file)
    sig = log_data.get_signal(args.signal)
    if sig is None:
        print(f"Error: signal '{args.signal}' not found.", file=sys.stderr)
        sys.exit(1)

    stats = compute_numeric_stats(sig)
    if stats is None:
        print(
            f"Signal '{args.signal}' is not numeric (type={sig.info.type.value}).",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Signal : {args.signal}")
    print(f"Type   : {sig.info.type.value}")
    print(f"Count  : {stats['count']}")
    print(f"Min    : {stats['min']:.6f}")
    print(f"Max    : {stats['max']:.6f}")
    print(f"Mean   : {stats['mean']:.6f}")
    print(f"Start  : {stats['first_timestamp_s']:.3f} s")
    print(f"End    : {stats['last_timestamp_s']:.3f} s")


def cmd_export(args: argparse.Namespace) -> None:
    """Export signal data to CSV."""
    log_data = _read_log(args.file)

    # Decide which signals to export
    if args.signal:
        names = [args.signal]
    else:
        names = log_data.signal_names()

    out = Path(args.output) if args.output else Path(args.file).with_suffix(".csv")

    with open(out, "w", newline="") as f:
        f.write("signal,timestamp_us,value\n")
        for name in names:
            sig = log_data.get_signal(name)
            if sig is None:
                continue
            for v in sig.values:
                f.write(f"{name},{v.timestamp_us},{v.value}\n")

    print(f"Exported {len(names)} signal(s) to {out}")


def cmd_analyze(args: argparse.Namespace) -> None:
    """Run a named analyzer against a log file."""
    ext = file_extension(args.file)

    # Special case: hard-hits analyzer with .hoot files uses targeted
    # extraction (only Pigeon 2 signals) for dramatically faster conversion.
    if args.analyzer == "hard-hits" and ext == ".hoot":
        from logreader.analyzers.hard_hits import read_hoot_for_hard_hits

        print(
            "Converting .hoot → wpilog (Pigeon 2 signals only)...",
            file=sys.stderr,
            flush=True,
        )
        log_data = read_hoot_for_hard_hits(args.file)
    elif ext == ".hoot" and args.analyzer not in ("hard-hits", "match-phases"):
        # Most analyzers need WPILib roboRIO signals (console, DS:enabled,
        # PowerDistribution, Scheduler/Names, systemTime, etc.) which use
        # different naming conventions and are only in roboRIO .wpilog files.
        # A full hoot conversion is also very slow (30+ seconds) and
        # produces millions of records, most irrelevant.
        #
        # hard-hits and match-phases both support hoot signals natively.
        print(
            f"Error: the '{args.analyzer}' analyzer requires WPILib "
            f"roboRIO signals which are not in .hoot files.\n"
            f"\n"
            f"  .hoot files contain CAN device data and robot state "
            f"(RobotEnable, RobotMode) but not the WPILib signals\n"
            f"  this analyzer needs (console, PowerDistribution, "
            f"Scheduler, etc.).\n"
            f"\n"
            f"  Use the .wpilog file from the same match for this "
            f"analyzer.\n"
            f"  Analyzers that work with .hoot files: "
            f"hard-hits, match-phases",
            file=sys.stderr,
        )
        sys.exit(1)
    else:
        log_data = _read_log(args.file)

    analyzer_cls = get_analyzer(args.analyzer)
    analyzer = analyzer_cls()

    # Collect any extra options the analyzer defined
    options: dict[str, object] = {}
    for key, val in vars(args).items():
        if key not in ("file", "analyzer", "func", "command"):
            options[key] = val

    result = analyzer.run(log_data, **options)
    print(result.format_report())


def cmd_analyze_list(args: argparse.Namespace) -> None:
    """List all available analyzers."""
    names = list_analyzers()
    if not names:
        print("No analyzers registered.")
        return
    print("Available analyzers:")
    for name in names:
        cls = get_analyzer(name)
        print(f"  {name:20s}  {cls.description}")


def _process_one_file(
    file_path: str, analyzer_name: str
) -> dict[str, object] | None:
    """Process a single file with an analyzer. Runs in a worker process."""
    t0 = time.monotonic()
    try:
        log_data = _read_log(file_path)
    except (FileNotFoundError, ValueError) as e:
        return {"__error__": str(e), "source_file": Path(file_path).name}

    analyzer_cls = get_analyzer(analyzer_name)
    analyzer = analyzer_cls()
    result = analyzer.run(log_data)
    entry = result.to_dict()
    entry["source_file"] = str(Path(file_path).name)
    entry["__elapsed__"] = time.monotonic() - t0
    return entry


def cmd_export_results(args: argparse.Namespace) -> None:
    """Run an analyzer on one or more log files and export results."""
    analyzer_name: str = args.analyzer
    files: list[str] = args.files
    output: str | None = args.output
    fmt: str = args.format
    workers: int = getattr(args, "workers", 0)

    # Determine worker count
    if workers <= 0:
        workers = min(os.cpu_count() or 1, len(files), 8)

    results: list[dict[str, object]] = []

    if workers == 1 or len(files) == 1:
        # Sequential mode
        for i, file_path in enumerate(files, 1):
            print(
                f"Processing {Path(file_path).name}...",
                file=sys.stderr,
                flush=True,
            )
            entry = _process_one_file(file_path, analyzer_name)
            if entry and "__error__" in entry:
                print(
                    f"  SKIP: {entry['__error__']}",
                    file=sys.stderr,
                )
            elif entry:
                elapsed_f = entry.pop("__elapsed__", 0)
                results.append(entry)
                if len(files) > 1:
                    print(
                        f"  [{i}/{len(files)}] done ({elapsed_f:.1f}s)",
                        file=sys.stderr,
                    )
    else:
        # Parallel mode
        from concurrent.futures import ProcessPoolExecutor, as_completed

        print(
            f"Processing {len(files)} files with {workers} workers...",
            file=sys.stderr,
            flush=True,
        )
        t_start = time.monotonic()
        completed = 0

        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = {
                pool.submit(_process_one_file, fp, analyzer_name): fp
                for fp in files
            }
            for future in as_completed(futures):
                fp = futures[future]
                completed += 1
                fname = Path(fp).name
                try:
                    entry = future.result()
                except Exception as e:
                    print(
                        f"  [{completed}/{len(files)}] {fname} "
                        f"-- ERROR: {e}",
                        file=sys.stderr,
                    )
                    continue

                if entry and "__error__" in entry:
                    print(
                        f"  [{completed}/{len(files)}] {fname} "
                        f"-- SKIP: {entry['__error__']}",
                        file=sys.stderr,
                    )
                elif entry:
                    elapsed_f = entry.pop("__elapsed__", 0)
                    results.append(entry)
                    print(
                        f"  [{completed}/{len(files)}] {fname} "
                        f"-- done ({elapsed_f:.1f}s)",
                        file=sys.stderr,
                    )

        elapsed = time.monotonic() - t_start
        print(
            f"Completed {len(results)}/{len(files)} files "
            f"in {elapsed:.1f}s",
            file=sys.stderr,
        )

    if not results:
        print("No results produced.", file=sys.stderr)
        sys.exit(1)

    # Sort by filename for deterministic output
    results.sort(key=lambda r: str(r.get("source_file", "")))

    if not results:
        print("No results produced.", file=sys.stderr)
        sys.exit(1)

    if fmt == "csv":
        out_str = _results_to_csv(results)
    else:
        out_str = json.dumps(results, indent=2, default=str)

    if output:
        out_path = Path(output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(out_str, encoding="utf-8")
        print(
            f"Exported {len(results)} result(s) to {out_path}",
            file=sys.stderr,
        )
    else:
        print(out_str)


def _flatten_extra(extra: dict[str, object]) -> dict[str, object]:
    """Extract scalar values from the extra dict for CSV columns.

    Walks one level into dicts whose values are all scalars (e.g.
    ``{"cam-a": {"count": 10, "pct": 5.2}}`` becomes
    ``{"cam-a.count": 10, "cam-a.pct": 5.2}``).
    """
    flat: dict[str, object] = {}
    for k, v in extra.items():
        if isinstance(v, (bool, int, float, str)):
            flat[k] = v
        elif v is None:
            flat[k] = ""
        elif isinstance(v, dict):
            # Check if it's a skipped placeholder
            if "__skipped__" in v:
                continue
            # If all values are scalars, flatten with dot notation
            if all(isinstance(sv, (bool, int, float, str, type(None))) for sv in v.values()):
                for sk, sv in v.items():
                    flat[f"{k}.{sk}"] = sv if sv is not None else ""
            # If values are dicts of scalars (e.g. per-camera stats),
            # flatten two levels deep
            elif all(isinstance(sv, dict) for sv in v.values()):
                for sub_key, sub_dict in v.items():
                    if not isinstance(sub_dict, dict):
                        continue
                    for sk, sv in sub_dict.items():
                        if isinstance(sv, (bool, int, float, str, type(None))):
                            flat[f"{k}.{sub_key}.{sk}"] = sv if sv is not None else ""
        # Skip lists, complex objects, etc.
    return flat


def _results_to_csv(results: list[dict[str, object]]) -> str:
    """Convert a list of result dicts to a CSV string.

    Each match becomes one row.  Columns are ``source_file``,
    ``analyzer_name``, ``title``, then all scalar extra fields
    discovered across all results.
    """
    # Build rows and discover all columns
    fixed_cols = ["source_file", "analyzer_name", "title"]
    rows: list[dict[str, object]] = []
    extra_keys: list[str] = []
    seen_keys: set[str] = set()

    for entry in results:
        flat = _flatten_extra(entry.get("extra", {}))
        row: dict[str, object] = {
            "source_file": entry.get("source_file", ""),
            "analyzer_name": entry.get("analyzer_name", ""),
            "title": entry.get("title", ""),
        }
        row.update(flat)
        rows.append(row)

        # Track column order (preserve insertion order)
        for k in flat:
            if k not in seen_keys:
                seen_keys.add(k)
                extra_keys.append(k)

    columns = fixed_cols + extra_keys

    buf = io.StringIO()
    writer = csv.DictWriter(
        buf, fieldnames=columns, extrasaction="ignore", lineterminator="\n"
    )
    writer.writeheader()
    for row in rows:
        writer.writerow(row)
    return buf.getvalue()


def build_parser() -> argparse.ArgumentParser:
    """Build the argument parser."""
    # Collect analyzer names for the epilog
    analyzer_names = list_analyzers()
    analyzer_lines: list[str] = []
    for name in analyzer_names:
        cls = get_analyzer(name)
        analyzer_lines.append(f"  {name:20s}  {cls.description}")

    epilog_parts = [
        "basic commands:",
        "  info                  Print log file summary",
        "  signals               List all signals in a log",
        "  stats                 Show stats for a specific signal",
        "  export                Export signal data to CSV",
        "",
        "analyzers (run as: logreader <analyzer> <file>):",
        *analyzer_lines,
        "",
        "other:",
        "  analyzers             List all available analyzers",
        "  export-results        Export analyzer results to JSON for cross-match analysis",
        "",
        "examples:",
        "  logreader info match.wpilog",
        "  logreader pdh-power match.wpilog",
        "  logreader loop-overruns match.wpilog --detail --phases",
        "  logreader launch-counter match.wpilog --detail --phases",
        "  logreader export-results vision-analysis *.wpilog -o results.json",
    ]

    parser = argparse.ArgumentParser(
        prog="logreader",
        usage="logreader <command> [options] <file>",
        description="FRC Robot Log Reader — read and analyse .wpilog and .hoot files",
        epilog="\n".join(epilog_parts),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(
        dest="command",
        required=True,
        metavar="<command>",
        help=argparse.SUPPRESS,
    )

    # info
    p_info = subparsers.add_parser(
        "info",
        help="Print log file summary",
        description="Print a summary of a log file.",
    )
    p_info.add_argument("file", help="Path to a .wpilog or .hoot file")
    p_info.set_defaults(func=cmd_info)

    # signals
    p_signals = subparsers.add_parser("signals", help="List all signals")
    p_signals.add_argument("file", help="Path to a .wpilog or .hoot file")
    p_signals.set_defaults(func=cmd_signals)

    # stats
    p_stats = subparsers.add_parser("stats", help="Show stats for a signal")
    p_stats.add_argument("file", help="Path to a .wpilog or .hoot file")
    p_stats.add_argument("signal", help="Signal name (e.g. /SmartDashboard/Speed)")
    p_stats.set_defaults(func=cmd_stats)

    # export
    p_export = subparsers.add_parser("export", help="Export signal data to CSV")
    p_export.add_argument("file", help="Path to a .wpilog or .hoot file")
    p_export.add_argument(
        "-s", "--signal", help="Export only this signal (default: all)"
    )
    p_export.add_argument("-o", "--output", help="Output CSV file path")
    p_export.set_defaults(func=cmd_export)

    # analyzers (list available)
    p_analyzers = subparsers.add_parser("analyzers", help="List available analyzers")
    p_analyzers.set_defaults(func=cmd_analyze_list)

    # export-results <analyzer> <file> [file ...] -o output.json
    p_export_results = subparsers.add_parser(
        "export-results",
        help="Run analyzer on files and export results to JSON or CSV",
        description=(
            "Run a named analyzer on one or more log files and write "
            "the summary-level results (tables and scalar metrics) "
            "to JSON or CSV for cross-match analysis."
        ),
    )
    p_export_results.add_argument(
        "analyzer",
        choices=list_analyzers(),
        help="Analyzer to run",
    )
    p_export_results.add_argument(
        "files",
        nargs="+",
        help="One or more .wpilog or .hoot files",
    )
    p_export_results.add_argument(
        "-o", "--output",
        help="Output file path (default: print to stdout)",
    )
    p_export_results.add_argument(
        "-f", "--format",
        choices=["json", "csv"],
        default="json",
        help="Output format (default: json)",
    )
    p_export_results.add_argument(
        "-w", "--workers",
        type=int,
        default=0,
        help=(
            "Number of parallel worker processes. "
            "0 = auto (min of CPU count, file count, 8). "
            "1 = sequential (default: 0)"
        ),
    )
    p_export_results.set_defaults(func=cmd_export_results)

    # analyze <name> <file> [analyzer-specific args]
    for analyzer_name in list_analyzers():
        analyzer_cls = get_analyzer(analyzer_name)
        p_az = subparsers.add_parser(
            analyzer_name,
            help=analyzer_cls.description,
        )
        p_az.add_argument("file", help="Path to a .wpilog or .hoot file")
        p_az.set_defaults(func=cmd_analyze, analyzer=analyzer_name)
        analyzer_cls.add_arguments(p_az)

    return parser


def main(argv: list[str] | None = None) -> None:
    """Entry point."""
    parser = build_parser()
    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
