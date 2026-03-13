"""Command-line interface for LogReader."""

from __future__ import annotations

import argparse
import sys
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

        log_data = read_hoot_for_hard_hits(args.file)
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
        "",
        "examples:",
        "  logreader info match.wpilog",
        "  logreader pdh-power match.wpilog",
        "  logreader loop-overruns match.wpilog --detail --phases",
        "  logreader launch-counter match.wpilog --detail --phases",
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
