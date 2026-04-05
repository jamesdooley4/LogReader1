"""Log-correlation analyzer — discover, align, and compare multi-source logs.

This analyzer takes a directory containing log files from one or more
matches and produces a report on:
  1. Discovered file groups
  2. Time-alignment offsets and confidence
  3. Mode-transition consistency across sources
  4. Overlapping-signal comparison (e.g. battery voltage)
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from logreader.analyzers.base import AnalysisResult, BaseAnalyzer, register_analyzer
from logreader.log_correlator import (
    CorrelationResult,
    MatchLogSet,
    TimeAlignment,
    compute_alignments,
    find_matching_logs,
    merge_logs,
    _extract_mode_edges,
)
from logreader.models import LogData


def _format_us(us: int) -> str:
    """Format microseconds as a human-readable string."""
    if abs(us) < 1_000:
        return f"{us} µs"
    if abs(us) < 1_000_000:
        return f"{us / 1_000:.2f} ms"
    return f"{us / 1_000_000:.3f} s"


def _compare_battery_voltage(
    wpilog_data: LogData, dslog_data: LogData, offset_us: int
) -> dict[str, Any] | None:
    """Compare battery voltage between wpilog and dslog.

    Returns summary stats or None if voltage signals aren't found.
    """
    # Find wpilog voltage signal
    wpi_voltage = None
    for name in sorted(wpilog_data.signals.keys()):
        if "voltage" in name.lower() and "power" in name.lower():
            sig = wpilog_data.signals[name]
            if sig.values and sig.info.type.value == "double":
                wpi_voltage = sig
                break

    ds_voltage = dslog_data.signals.get("/DSLog/BatteryVoltage")
    if wpi_voltage is None or ds_voltage is None:
        return None
    if not wpi_voltage.values or not ds_voltage.values:
        return None

    # Compare at dslog sample times (50 Hz, lower rate)
    # Shift dslog timestamps to wpilog FPGA time: fpga = unix - offset
    diffs: list[float] = []
    wpi_idx = 0
    for dv in ds_voltage.values:
        ds_fpga = dv.timestamp_us - offset_us
        # Find nearest wpilog sample
        while wpi_idx < len(wpi_voltage.values) - 1 and \
                wpi_voltage.values[wpi_idx + 1].timestamp_us <= ds_fpga:
            wpi_idx += 1
        if wpi_idx < len(wpi_voltage.values):
            diff = abs(dv.value - wpi_voltage.values[wpi_idx].value)
            diffs.append(diff)

    if not diffs:
        return None

    return {
        "mean_diff_v": sum(diffs) / len(diffs),
        "max_diff_v": max(diffs),
        "samples_compared": len(diffs),
        "wpilog_signal": wpi_voltage.info.name,
    }


def _compare_mode_transitions(
    wpilog_data: LogData, dslog_data: LogData, offset_us: int
) -> list[dict[str, Any]]:
    """Compare mode transitions between wpilog and dslog.

    Returns a list of dicts describing each transition pair.
    """
    wpi_edges = _extract_mode_edges(wpilog_data)
    ds_edges = _extract_mode_edges(dslog_data, dslog_mode=True)

    if not wpi_edges or not ds_edges:
        return []

    # Shift dslog edges to FPGA time
    ds_shifted = [(ts - offset_us, en) for ts, en in ds_edges]

    comparisons: list[dict[str, Any]] = []
    for ds_ts, ds_en in ds_shifted:
        # Find closest wpilog edge
        best_wpi_ts = None
        best_diff = float("inf")
        for wpi_ts, wpi_en in wpi_edges:
            if wpi_en == ds_en:
                diff = abs(wpi_ts - ds_ts)
                if diff < best_diff:
                    best_diff = diff
                    best_wpi_ts = wpi_ts

        if best_wpi_ts is not None and best_diff < 5_000_000:
            comparisons.append({
                "transition": "enable" if ds_en else "disable",
                "ds_fpga_us": ds_ts,
                "wpilog_fpga_us": best_wpi_ts,
                "diff_us": best_wpi_ts - ds_ts,
                "diff_ms": (best_wpi_ts - ds_ts) / 1000,
            })

    return comparisons


@register_analyzer
class LogCorrelationAnalyzer(BaseAnalyzer):
    """Discover and correlate multi-source log files from the same match."""

    name = "log-correlation"
    description = (
        "Discover, time-align, and compare .dslog + .wpilog + .hoot logs"
    )

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--match-window",
            type=float,
            default=120.0,
            help="Maximum time difference (seconds) for grouping files "
            "into the same match (default: 120)",
        )
        parser.add_argument(
            "--align",
            type=int,
            default=None,
            metavar="N",
            help="Compute time alignment for group N (1-based). "
            "Without this flag, only file discovery is performed (fast).",
        )
        parser.add_argument(
            "--align-all",
            action="store_true",
            help="Compute time alignment for ALL groups (slow for large "
            "directories with many wpilog files).",
        )

    def run(self, log_data: LogData, **options: Any) -> AnalysisResult:
        """Run the log-correlation analysis.

        This analyzer is special: it expects the ``file`` argument to be
        a directory containing multiple log files, or a single log file
        alongside which other files will be discovered.

        If run on a single file, it will attempt to discover matching
        files in the same directory.
        """
        file_path = Path(log_data.metadata.file_path.split(",")[0].strip())
        search_dir = file_path.parent if file_path.is_file() else file_path

        match_window = options.get("match_window", 120.0)
        align_group = options.get("align", None)
        align_all = options.get("align_all", False)

        # Discover file groups
        log_sets = find_matching_logs(search_dir, match_window_s=match_window)

        if not log_sets:
            return AnalysisResult(
                analyzer_name=self.name,
                title="Log Correlation",
                summary="No matching log file groups found.",
            )

        summary_lines: list[str] = []
        rows: list[dict[str, Any]] = []
        extra: dict[str, Any] = {"groups": []}

        summary_lines.append(
            f"Found {len(log_sets)} log group(s) in {search_dir}"
        )
        if not align_all and align_group is None:
            summary_lines.append(
                "Use --align N to compute alignment for group N, "
                "or --align-all for all groups."
            )

        for i, ls in enumerate(log_sets):
            group_info: dict[str, Any] = {
                "label": ls.match_label,
                "wpilog": ls.wpilog_path.name if ls.wpilog_path else None,
                "dslog": ls.dslog_path.name if ls.dslog_path else None,
                "dsevents": ls.dsevents_path.name
                if ls.dsevents_path
                else None,
                "hoot_files": [
                    {"bus": h.bus_name, "file": h.path.name}
                    for h in ls.hoot_files
                ],
            }

            source_count = sum([
                ls.wpilog_path is not None,
                ls.dslog_path is not None,
                len(ls.hoot_files) > 0,
            ])

            row: dict[str, Any] = {
                "#": i + 1,
                "Label": ls.match_label,
                "WPILog": ls.wpilog_path.name if ls.wpilog_path else "-",
                "DSLog": ls.dslog_path.name if ls.dslog_path else "-",
                "Hoots": len(ls.hoot_files),
                "Sources": source_count,
            }

            # Only compute alignment for requested groups
            should_align = (
                (align_all or align_group == i + 1)
                and ls.wpilog_path
                and ls.dslog_path
            )

            if should_align:
                from logreader.wpilog_reader import read_wpilog
                from logreader.dslog_reader import read_ds_logs

                try:
                    wpilog = read_wpilog(ls.wpilog_path)
                    dslog = read_ds_logs(ls.dslog_path, ls.dsevents_path)

                    alignments = compute_alignments(wpilog, dslog)
                    group_info["alignments"] = []

                    for a in alignments:
                        row["Offset"] = _format_us(a.offset_us)
                        row["Method"] = a.method
                        row["Confidence"] = f"{a.confidence:.0%}"
                        group_info["alignments"].append({
                            "source_a": a.source_a,
                            "source_b": a.source_b,
                            "offset_us": a.offset_us,
                            "method": a.method,
                            "confidence": a.confidence,
                            "residual_us": a.residual_us,
                        })

                        # Battery voltage comparison
                        batt_cmp = _compare_battery_voltage(
                            wpilog, dslog, a.offset_us
                        )
                        if batt_cmp:
                            row["Batt ΔV"] = f"{batt_cmp['mean_diff_v']:.3f}"
                            group_info["battery_comparison"] = batt_cmp

                        # Mode transition comparison
                        mode_cmp = _compare_mode_transitions(
                            wpilog, dslog, a.offset_us
                        )
                        if mode_cmp:
                            avg_diff = (
                                sum(abs(m["diff_us"]) for m in mode_cmp)
                                / len(mode_cmp)
                            )
                            row["Mode Δ"] = f"{avg_diff / 1000:.1f} ms"
                            group_info["mode_transitions"] = mode_cmp

                except Exception as exc:
                    row["Offset"] = f"error: {exc}"

            rows.append(row)
            extra["groups"].append(group_info)

        columns = ["#", "Label", "WPILog", "DSLog", "Hoots", "Sources"]
        if any("Offset" in r for r in rows):
            columns.extend(["Offset", "Method", "Confidence", "Batt ΔV", "Mode Δ"])

        return AnalysisResult(
            analyzer_name=self.name,
            title="Log Correlation",
            summary="\n".join(summary_lines),
            columns=columns,
            rows=rows,
            extra=extra,
        )
