"""Multi-source log correlation.

Discover, time-align, and merge ``.dslog``, ``.wpilog``, and ``.hoot`` log
files from the same match into a unified ``LogData``.

Design doc: ``docs/design-log-correlation.md``
"""

from __future__ import annotations

import re
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

from logreader.models import (
    LogData,
    LogMetadata,
    SignalData,
    SignalInfo,
    SignalType,
    TimestampedValue,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_LABVIEW_EPOCH_OFFSET = 2_082_844_800  # see dslog_reader.py for derivation
_DSLOG_HEADER_SIZE = 20

# Filename patterns
_WPILOG_RE = re.compile(
    r"FRC_(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})\.wpilog$"
)
_DSLOG_RE = re.compile(
    r"(\d{4})_(\d{2})_(\d{2})\s+(\d{2})_(\d{2})_(\d{2})\b.*\.dslog$"
)
_DSEVENTS_RE = re.compile(
    r"(\d{4})_(\d{2})_(\d{2})\s+(\d{2})_(\d{2})_(\d{2})\b.*\.dsevents$"
)
_HOOT_RIO_RE = re.compile(
    r"rio_(\d{4})-(\d{2})-(\d{2})_(\d{2})-(\d{2})-(\d{2})\.hoot$"
)
_HOOT_CANivore_RE = re.compile(
    r"([0-9A-Fa-f]{32})_(\d{4})-(\d{2})-(\d{2})_(\d{2})-(\d{2})-(\d{2})\.hoot$"
)

# Default matching window: files within this many seconds are candidates
_DEFAULT_MATCH_WINDOW_S = 120.0

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------


@dataclass
class HootFile:
    """A discovered ``.hoot`` file with its CAN bus identity."""

    path: Path
    bus_name: str  # "rio" or 32-char hex GUID
    friendly_name: str | None = None


@dataclass
class MatchLogSet:
    """A group of log files believed to be from the same match / session."""

    dslog_path: Path | None = None
    dsevents_path: Path | None = None
    wpilog_path: Path | None = None
    hoot_files: list[HootFile] = field(default_factory=list)
    match_label: str = ""


@dataclass
class TimeAlignment:
    """Computed time offset between two log sources."""

    source_a: str
    source_b: str
    offset_us: int  # add to source_b timestamps to align with source_a
    method: str  # "system_time", "mode_edge", "filename"
    confidence: float  # 0–1
    residual_us: int  # RMS residual of the alignment


@dataclass
class CorrelationResult:
    """Full result of correlating logs from a match."""

    log_set: MatchLogSet
    alignments: list[TimeAlignment] = field(default_factory=list)
    merged: LogData | None = None
    warnings: list[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _filename_timestamp_s(groups: tuple[str, ...]) -> float:
    """Convert regex match groups (Y, M, D, H, Min, S) to approx UNIX seconds.

    This is an *approximate* value used only for file-grouping, not for
    precise alignment.  It assumes the timestamp is UTC (good enough for
    grouping with a 2-minute window).
    """
    from datetime import datetime, timezone

    y, mo, d, h, mi, s = (int(g) for g in groups[:6])
    try:
        dt = datetime(y, mo, d, h, mi, s, tzinfo=timezone.utc)
        return dt.timestamp()
    except ValueError:
        return 0.0


def _dslog_header_unix_s(path: Path) -> float | None:
    """Read just the 20-byte dslog header and return absolute UNIX seconds."""
    try:
        data = path.read_bytes()
        if len(data) < _DSLOG_HEADER_SIZE:
            return None
        version = struct.unpack(">i", data[0:4])[0]
        if version != 4:
            return None
        lv_secs = struct.unpack(">q", data[4:12])[0]
        lv_frac = struct.unpack(">Q", data[12:20])[0]
        return (lv_secs - _LABVIEW_EPOCH_OFFSET) + lv_frac / (2**64)
    except Exception:
        return None


def _extract_wpilog_time_offset(log_data: LogData) -> int | None:
    """Extract the FPGA→UNIX offset from the ``systemTime`` signal.

    ``systemTime`` is a WPILib int64 signal whose value is the wall-clock
    time in microseconds (UNIX epoch).  Its FPGA timestamp gives the
    corresponding FPGA time.  The offset is ``value - fpga_timestamp``.

    Returns the median offset in microseconds, or ``None`` if the signal
    is missing or empty.
    """
    sig = log_data.signals.get("systemTime")
    if sig is None or not sig.values:
        return None
    offsets = [v.value - v.timestamp_us for v in sig.values]
    offsets.sort()
    return offsets[len(offsets) // 2]


def _extract_mode_edges(
    log_data: LogData,
    *,
    signal_name: str | None = None,
    dslog_mode: bool = False,
) -> list[tuple[int, bool]]:
    """Extract enable/disable transition edges from a log.

    Returns a list of ``(timestamp_us, enabled)`` tuples sorted by time.

    For wpilog: looks for ``DS:enabled`` boolean signal.
    For dslog: looks for ``/DSLog/Status/DSDisabled`` (inverted).
    """
    if dslog_mode:
        candidates = ["/DSLog/Status/DSDisabled"]
    else:
        candidates = [signal_name] if signal_name else [
            "DS:enabled",
        ]

    for name in candidates:
        sig = log_data.signals.get(name)
        if sig is None or not sig.values:
            continue
        edges: list[tuple[int, bool]] = []
        prev: bool | None = None
        for v in sig.values:
            val = v.value
            if dslog_mode or name.endswith("Disabled"):
                val = not val  # invert: disabled=True → enabled=False
            if val != prev:
                edges.append((v.timestamp_us, bool(val)))
                prev = val
        if edges:
            return edges

    return []


def _align_mode_edges(
    edges_a: list[tuple[int, bool]],
    edges_b: list[tuple[int, bool]],
) -> tuple[int, float, int] | None:
    """Compute the offset to align edges_b to edges_a using enable edges.

    Returns ``(offset_us, confidence, residual_us)`` or ``None``.
    The offset means: ``aligned_b_time = b_time + offset``.
    """
    # Extract only enable edges (False→True transitions)
    enable_a = [ts for ts, en in edges_a if en]
    enable_b = [ts for ts, en in edges_b if en]

    if not enable_a or not enable_b:
        return None

    # Try matching each enable_b[0] to each enable_a[i] and score
    best_offset = 0
    best_score = float("inf")
    best_matched = 0

    for anchor_a in enable_a:
        candidate_offset = anchor_a - enable_b[0]

        # Score: for each enable_b, find the closest enable_a
        total_err_sq = 0.0
        matched = 0
        for tb in enable_b:
            shifted = tb + candidate_offset
            # Find closest enable_a
            min_dist = min(abs(shifted - ta) for ta in enable_a)
            if min_dist < 5_000_000:  # within 5 seconds
                total_err_sq += min_dist**2
                matched += 1

        if matched > best_matched or (
            matched == best_matched and total_err_sq < best_score
        ):
            best_offset = candidate_offset
            best_score = total_err_sq
            best_matched = matched

    if best_matched == 0:
        return None

    residual = int((best_score / best_matched) ** 0.5)
    confidence = min(1.0, best_matched / max(len(enable_b), 1))
    return best_offset, confidence, residual


# ---------------------------------------------------------------------------
# Public API: File Discovery
# ---------------------------------------------------------------------------


def scan_log_files(directory: str | Path) -> dict[str, list[Path]]:
    """Recursively scan a directory for log files, grouped by extension.

    Returns a dict with keys ``"wpilog"``, ``"dslog"``, ``"dsevents"``,
    ``"hoot"`` — each mapping to a sorted list of file paths.
    """
    root = Path(directory)
    result: dict[str, list[Path]] = {
        "wpilog": [],
        "dslog": [],
        "dsevents": [],
        "hoot": [],
    }
    for p in sorted(root.rglob("*")):
        if not p.is_file():
            continue
        ext = p.suffix.lower()
        if ext == ".wpilog":
            result["wpilog"].append(p)
        elif ext == ".dslog":
            result["dslog"].append(p)
        elif ext == ".dsevents":
            result["dsevents"].append(p)
        elif ext == ".hoot":
            result["hoot"].append(p)
    return result


def _classify_hoot(path: Path) -> HootFile:
    """Classify a hoot file as rio or CANivore from its filename."""
    name = path.name
    m = _HOOT_CANivore_RE.match(name)
    if m:
        return HootFile(path=path, bus_name=m.group(1))
    m = _HOOT_RIO_RE.match(name)
    if m:
        return HootFile(path=path, bus_name="rio")
    # Unknown format — use stem as bus name
    return HootFile(path=path, bus_name=path.stem.split("_")[0])


def _hoot_filename_ts(path: Path) -> float | None:
    """Extract an approximate UNIX timestamp from a hoot filename."""
    name = path.name
    m = _HOOT_CANivore_RE.match(name)
    if m:
        return _filename_timestamp_s(m.groups()[1:])
    m = _HOOT_RIO_RE.match(name)
    if m:
        return _filename_timestamp_s(m.groups())
    return None


def _wpilog_filename_ts(path: Path) -> float | None:
    """Extract an approximate UNIX timestamp from a wpilog filename."""
    m = _WPILOG_RE.search(path.name)
    if m:
        return _filename_timestamp_s(m.groups())
    return None


def find_matching_logs(
    directory: str | Path,
    *,
    match_window_s: float = _DEFAULT_MATCH_WINDOW_S,
) -> list[MatchLogSet]:
    """Discover groups of log files from the same match in a directory.

    Scans recursively for ``.wpilog``, ``.dslog``, ``.dsevents``, and
    ``.hoot`` files.  Groups them by timestamp proximity.

    Parameters:
        directory: Root directory to scan.
        match_window_s: Maximum time difference (seconds) for two files
            to be considered part of the same session.

    Returns:
        A list of ``MatchLogSet`` instances, sorted chronologically.
    """
    files = scan_log_files(directory)

    # Build a list of (approx_unix_time, file_info) tuples
    TimedItem = tuple[float, str, Path | HootFile]
    timed: list[TimedItem] = []

    for p in files["wpilog"]:
        ts = _wpilog_filename_ts(p)
        if ts:
            timed.append((ts, "wpilog", p))

    for p in files["dslog"]:
        ts = _dslog_header_unix_s(p)
        if ts:
            timed.append((ts, "dslog", p))

    for p in files["dsevents"]:
        # Match dsevents to dslog by stem
        dslog_stem = _dsevents_stem(p)
        if dslog_stem:
            timed.append((0.0, "dsevents_pending", p))  # will pair later
        else:
            pass  # skip unparseable dsevents

    for p in files["hoot"]:
        ts = _hoot_filename_ts(p)
        if ts:
            hf = _classify_hoot(p)
            timed.append((ts, "hoot", hf))

    # Pair dsevents with dslog by stem
    dslog_by_stem: dict[str, Path] = {}
    for p in files["dslog"]:
        stem = _dslog_stem(p)
        if stem:
            dslog_by_stem[stem] = p

    dsevents_by_stem: dict[str, Path] = {}
    for p in files["dsevents"]:
        stem = _dsevents_stem(p)
        if stem:
            dsevents_by_stem[stem] = p

    # Remove pending dsevents from timed list
    timed = [(ts, kind, item) for ts, kind, item in timed if kind != "dsevents_pending"]

    # Sort by timestamp
    timed.sort(key=lambda x: x[0])

    # Group by proximity
    groups: list[list[TimedItem]] = []
    current_group: list[TimedItem] = []

    for item in timed:
        ts = item[0]
        if current_group and ts - current_group[0][0] > match_window_s:
            groups.append(current_group)
            current_group = [item]
        else:
            current_group.append(item)
    if current_group:
        groups.append(current_group)

    # Convert groups to MatchLogSet
    result: list[MatchLogSet] = []
    for group in groups:
        log_set = MatchLogSet()
        for ts, kind, item in group:
            if kind == "wpilog":
                assert isinstance(item, Path)
                if log_set.wpilog_path is None:
                    log_set.wpilog_path = item
            elif kind == "dslog":
                assert isinstance(item, Path)
                if log_set.dslog_path is None:
                    log_set.dslog_path = item
                    # Auto-pair dsevents
                    stem = _dslog_stem(item)
                    if stem and stem in dsevents_by_stem:
                        log_set.dsevents_path = dsevents_by_stem[stem]
            elif kind == "hoot":
                assert isinstance(item, HootFile)
                log_set.hoot_files.append(item)

        # Build a label from the earliest timestamp
        if group:
            from datetime import datetime, timezone

            dt = datetime.fromtimestamp(group[0][0], tz=timezone.utc)
            log_set.match_label = dt.strftime("%Y-%m-%d_%H-%M-%S")

        # Only include groups that have at least 2 sources
        source_count = sum([
            log_set.wpilog_path is not None,
            log_set.dslog_path is not None,
            len(log_set.hoot_files) > 0,
        ])
        if source_count >= 1:
            result.append(log_set)

    return result


def _dslog_stem(path: Path) -> str | None:
    """Extract the timestamp stem from a dslog filename for pairing."""
    m = _DSLOG_RE.match(path.name)
    if m:
        return f"{m.group(1)}_{m.group(2)}_{m.group(3)} {m.group(4)}_{m.group(5)}_{m.group(6)}"
    return None


def _dsevents_stem(path: Path) -> str | None:
    """Extract the timestamp stem from a dsevents filename for pairing."""
    m = _DSEVENTS_RE.match(path.name)
    if m:
        return f"{m.group(1)}_{m.group(2)}_{m.group(3)} {m.group(4)}_{m.group(5)}_{m.group(6)}"
    return None


# ---------------------------------------------------------------------------
# Public API: Time Alignment
# ---------------------------------------------------------------------------


def compute_alignments(
    wpilog_data: LogData | None = None,
    dslog_data: LogData | None = None,
    hoot_data: dict[str, LogData] | None = None,
) -> list[TimeAlignment]:
    """Compute time alignments between log sources.

    The wpilog is used as the reference clock.  Returns a list of
    ``TimeAlignment`` objects describing how to shift each other source
    to align with the wpilog.

    When a wpilog has a ``systemTime`` signal, it provides a direct
    FPGA→UNIX anchor (sub-ms accuracy).  The dslog already uses absolute
    UNIX timestamps, so the offset between dslog and wpilog is simply
    ``wpilog_fpga_offset - 0`` (since dslog timestamps are already UNIX).

    Parameters:
        wpilog_data: Parsed wpilog ``LogData`` (reference clock).
        dslog_data: Parsed dslog ``LogData`` (absolute UNIX timestamps).
        hoot_data: Dict of ``bus_name → LogData`` for each hoot file.
    """
    alignments: list[TimeAlignment] = []
    wpilog_offset: int | None = None

    if wpilog_data is not None:
        wpilog_offset = _extract_wpilog_time_offset(wpilog_data)

    # Align dslog → wpilog
    if wpilog_data is not None and dslog_data is not None:
        alignment = _align_dslog_to_wpilog(
            wpilog_data, dslog_data, wpilog_offset
        )
        if alignment:
            alignments.append(alignment)

    # Align hoot files → wpilog
    if wpilog_data is not None and hoot_data:
        for bus_name, hdata in hoot_data.items():
            alignment = _align_hoot_to_wpilog(
                wpilog_data, hdata, bus_name, wpilog_offset
            )
            if alignment:
                alignments.append(alignment)

    return alignments


def _align_dslog_to_wpilog(
    wpilog_data: LogData,
    dslog_data: LogData,
    wpilog_offset: int | None,
) -> TimeAlignment | None:
    """Align dslog (absolute UNIX µs) to wpilog (FPGA µs).

    The offset means: ``wpilog_fpga_us = dslog_unix_us - offset``
    or equivalently: ``dslog_unix_us = wpilog_fpga_us + offset``.

    So to shift dslog timestamps into wpilog's FPGA time:
    ``shifted = dslog_ts - offset``.
    """
    # Method 1: systemTime anchor
    if wpilog_offset is not None:
        # wpilog_offset = systemTime_value - fpga_us
        # dslog timestamps are already UNIX µs
        # To convert dslog → wpilog FPGA: fpga = dslog_ts - wpilog_offset
        # Validate via mode edges
        wpi_edges = _extract_mode_edges(wpilog_data)
        ds_edges = _extract_mode_edges(dslog_data, dslog_mode=True)

        residual = 0
        confidence = 0.9

        if wpi_edges and ds_edges:
            # Check: shift dslog edges to FPGA time and compare
            shifted_ds = [(ts - wpilog_offset, en) for ts, en in ds_edges]
            match_result = _align_mode_edges(wpi_edges, shifted_ds)
            if match_result:
                extra_offset, conf, res = match_result
                # extra_offset should be near zero if systemTime is correct
                residual = abs(extra_offset)
                if residual < 1_000_000:  # within 1 second
                    confidence = min(1.0, 0.9 + conf * 0.1)
                else:
                    confidence = 0.5  # systemTime disagrees with edges

        return TimeAlignment(
            source_a="wpilog",
            source_b="dslog",
            offset_us=wpilog_offset,
            method="system_time",
            confidence=confidence,
            residual_us=residual,
        )

    # Method 2: mode-edge matching only
    wpi_edges = _extract_mode_edges(wpilog_data)
    ds_edges = _extract_mode_edges(dslog_data, dslog_mode=True)

    if not wpi_edges or not ds_edges:
        return None

    result = _align_mode_edges(wpi_edges, ds_edges)
    if result is None:
        return None

    # This offset means: wpi_ts = ds_ts + offset → offset = wpi_ts - ds_ts
    # But we want: dslog_unix = fpga + offset → offset = dslog_unix - fpga
    # The mode_edge result gives: shifted_b = b + offset aligns with a
    # So wpi_ts ≈ ds_ts + offset → offset = wpi_ts - ds_ts
    # But since ds timestamps are UNIX and wpi are FPGA:
    # We want the offset such that: fpga = unix - offset
    # So offset = unix - fpga = -(offset_from_mode_edges)
    raw_offset, confidence, residual = result
    return TimeAlignment(
        source_a="wpilog",
        source_b="dslog",
        offset_us=-raw_offset,
        method="mode_edge",
        confidence=confidence,
        residual_us=residual,
    )


def _align_hoot_to_wpilog(
    wpilog_data: LogData,
    hoot_data: LogData,
    bus_name: str,
    wpilog_offset: int | None,
) -> TimeAlignment | None:
    """Align a hoot file (FPGA-like µs) to the wpilog (FPGA µs).

    After owlet conversion, hoot timestamps are FPGA-like microseconds
    that may have a small fixed offset relative to the wpilog's FPGA clock.
    """
    # Try mode-edge alignment if both have mode signals
    wpi_edges = _extract_mode_edges(wpilog_data)
    hoot_edges = _extract_mode_edges(hoot_data)

    if wpi_edges and hoot_edges:
        result = _align_mode_edges(wpi_edges, hoot_edges)
        if result:
            offset, confidence, residual = result
            return TimeAlignment(
                source_a="wpilog",
                source_b=f"hoot_{bus_name}",
                offset_us=offset,
                method="mode_edge",
                confidence=confidence,
                residual_us=residual,
            )

    # Fallback: assume zero offset (rio hoot usually shares FPGA clock)
    if bus_name == "rio":
        return TimeAlignment(
            source_a="wpilog",
            source_b=f"hoot_{bus_name}",
            offset_us=0,
            method="assumed_zero",
            confidence=0.5,
            residual_us=0,
        )

    return None


# ---------------------------------------------------------------------------
# Public API: Merging
# ---------------------------------------------------------------------------


def _hoot_signal_prefix(hoot_file: HootFile) -> str:
    """Determine the signal prefix for a hoot file."""
    if hoot_file.bus_name == "rio":
        return "/Hoot/Rio"
    if hoot_file.friendly_name:
        return f"/Hoot/{hoot_file.friendly_name}"
    # Use first 8 chars of GUID for readability
    short = hoot_file.bus_name[:8]
    return f"/Hoot/{short}"


def _prefix_signals(
    log_data: LogData,
    prefix: str,
    offset_us: int = 0,
) -> dict[str, SignalData]:
    """Copy signals with a name prefix and optional timestamp offset.

    Parameters:
        log_data: Source log data.
        prefix: String to prepend to signal names (e.g. ``"/DSLog"``).
        offset_us: Microseconds to add to each timestamp.

    Returns:
        A dict of prefixed ``SignalData`` objects.
    """
    result: dict[str, SignalData] = {}
    for name, sig in log_data.signals.items():
        if name.startswith(prefix):
            # Already has the prefix (e.g., dslog signals already under /DSLog)
            new_name = name
        else:
            new_name = f"{prefix}/{name}" if not name.startswith("/") else f"{prefix}{name}"

        new_info = SignalInfo(
            entry_id=sig.info.entry_id,
            name=new_name,
            type=sig.info.type,
            metadata=sig.info.metadata,
        )

        if offset_us == 0:
            new_values = sig.values
        else:
            new_values = [
                TimestampedValue(v.timestamp_us + offset_us, v.value)
                for v in sig.values
            ]

        result[new_name] = SignalData(info=new_info, values=new_values)

    return result


def merge_logs(
    log_set: MatchLogSet,
    *,
    wpilog_data: LogData | None = None,
    dslog_data: LogData | None = None,
    hoot_data: dict[str, LogData] | None = None,
    alignments: list[TimeAlignment] | None = None,
) -> CorrelationResult:
    """Read all available logs from a match and merge into a unified LogData.

    If pre-read ``LogData`` objects are provided, they are used directly.
    Otherwise, the files are read from disk.

    The wpilog's FPGA clock is used as the reference time base.  Other
    sources' timestamps are shifted via the computed alignments.

    Parameters:
        log_set: The matched file group.
        wpilog_data: Pre-read wpilog data, or ``None`` to read from disk.
        dslog_data: Pre-read dslog data, or ``None`` to read from disk.
        hoot_data: Pre-read hoot data keyed by bus name, or ``None``.
        alignments: Pre-computed alignments, or ``None`` to compute.

    Returns:
        A ``CorrelationResult`` with merged ``LogData`` and alignment info.
    """
    warnings: list[str] = []

    # Read files if not pre-loaded
    if log_set.wpilog_path and wpilog_data is None:
        from logreader.wpilog_reader import read_wpilog

        wpilog_data = read_wpilog(log_set.wpilog_path)

    if log_set.dslog_path and dslog_data is None:
        from logreader.dslog_reader import read_ds_logs

        dslog_data = read_ds_logs(
            log_set.dslog_path, log_set.dsevents_path
        )

    if log_set.hoot_files and hoot_data is None:
        hoot_data = {}
        for hf in log_set.hoot_files:
            try:
                from logreader.hoot_reader import read_hoot

                hoot_data[hf.bus_name] = read_hoot(hf.path)
            except Exception as exc:
                warnings.append(f"Failed to read hoot {hf.path.name}: {exc}")

    # Compute alignments if not provided
    if alignments is None:
        alignments = compute_alignments(wpilog_data, dslog_data, hoot_data)

    # Build alignment lookup: source_b → offset_us
    offset_map: dict[str, int] = {}
    for a in alignments:
        offset_map[a.source_b] = a.offset_us

    # Merge signals
    merged_signals: dict[str, SignalData] = {}

    # 1. WPILog signals (reference, no prefix, no offset)
    if wpilog_data is not None:
        merged_signals.update(wpilog_data.signals)

    # 2. DSLog signals (already prefixed as /DSLog/*, need timestamp shift)
    if dslog_data is not None:
        ds_offset = offset_map.get("dslog", 0)
        # dslog timestamps are absolute UNIX µs.
        # To convert to wpilog FPGA µs: fpga = unix - offset
        # So we subtract the offset from dslog timestamps.
        shift = -ds_offset
        ds_signals = _prefix_signals(dslog_data, "", shift)
        merged_signals.update(ds_signals)

    # 3. Hoot signals (prefixed by bus name, may need small offset)
    if hoot_data is not None:
        for hf in log_set.hoot_files:
            hdata = hoot_data.get(hf.bus_name)
            if hdata is None:
                continue
            prefix = _hoot_signal_prefix(hf)
            hoot_offset = offset_map.get(f"hoot_{hf.bus_name}", 0)
            hoot_signals = _prefix_signals(hdata, prefix, hoot_offset)
            merged_signals.update(hoot_signals)

    # Build merged metadata
    source_files = []
    if log_set.wpilog_path:
        source_files.append(log_set.wpilog_path.name)
    if log_set.dslog_path:
        source_files.append(log_set.dslog_path.name)
    for hf in log_set.hoot_files:
        source_files.append(hf.path.name)

    metadata = LogMetadata(
        file_path=", ".join(source_files),
        is_valid=True,
        signal_count=len(merged_signals),
        record_count=sum(len(s.values) for s in merged_signals.values()),
        warnings=warnings,
    )

    merged = LogData(metadata=metadata, signals=merged_signals)

    return CorrelationResult(
        log_set=log_set,
        alignments=alignments,
        merged=merged,
        warnings=warnings,
    )
