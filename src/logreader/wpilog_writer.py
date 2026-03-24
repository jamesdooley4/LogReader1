"""WPILib .wpilog file writer — copy and augment log files.

This module provides infrastructure for reading a ``.wpilog`` file
record-by-record, copying every record to a new file, and injecting
additional derived signals (augmentations) alongside the originals.

The primary use-case is adding ``struct:Pose3d`` entries from Limelight
``botpose_wpiblue`` / ``botpose_orb_wpiblue`` double-array signals so
that AdvantageScope can visualise them in its 2D/3D field views.

See ``docs/design-augment.md`` for full design rationale.
"""

from __future__ import annotations

import math
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from wpiutil._wpiutil import DataLogWriter
from wpiutil.log import DataLogReader

# DataLogWriter deadlocks when its internal buffer fills (~11K appends).
# Flushing periodically prevents the overflow.
_FLUSH_INTERVAL = 5000

# ---------------------------------------------------------------------------
# Botpose → Pose3d augmentation
# ---------------------------------------------------------------------------

# Suffixes that identify the botpose signals to augment.
_BOTPOSE_SUFFIXES = (
    "/botpose_wpiblue",
    "/botpose_orb_wpiblue",
)


def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


@dataclass
class AugmentResult:
    """Summary of an augmentation run."""

    input_path: str
    output_path: str
    input_size_bytes: int = 0
    output_size_bytes: int = 0
    total_records: int = 0
    total_entries: int = 0
    pose3d_written: int = 0
    pose3d_skipped: int = 0
    pose3d_signals: list[str] = field(default_factory=list)
    elapsed_s: float = 0.0


def _is_botpose_signal(name: str) -> bool:
    """Return True if *name* is a botpose signal we should augment."""
    return any(name.endswith(suffix) for suffix in _BOTPOSE_SUFFIXES)


def _pose3d_entry_name(original_name: str) -> str:
    """Derive the Pose3d signal name from the original botpose signal."""
    return original_name + "_pose3d"


def _botpose_array_to_packed_pose3d(raw: bytes) -> bytes | None:
    """Convert raw double-array bytes to packed ``Pose3d`` struct bytes.

    Returns *None* when the sample should be skipped (no tags, zero pose,
    or array too short).

    This function avoids constructing ``Pose3d`` / ``Rotation3d`` objects
    for speed — it packs the 56-byte struct directly using the known
    WPILib serialisation layout::

        Translation3d: [x: f64, y: f64, z: f64]           (24 bytes)
        Rotation3d → Quaternion: [w: f64, x: f64, y: f64, z: f64]  (32 bytes)

    Euler angles (roll, pitch, yaw in radians) are converted to a
    quaternion using the intrinsic ZYX convention which matches WPILib's
    ``Rotation3d(roll, pitch, yaw)`` constructor.
    """
    # Need at least 8 doubles (64 bytes) to have tag_count at index 7
    if len(raw) < 64:
        return None

    n_doubles = len(raw) // 8
    arr = struct.unpack(f"<{n_doubles}d", raw)

    # arr[7] = tag_count — skip if no tags visible
    if arr[7] < 1.0:
        return None

    x, y, z = arr[0], arr[1], arr[2]

    # Skip zero-pose (no valid detection)
    if abs(x) < 0.001 and abs(y) < 0.001:
        return None

    roll = _deg_to_rad(arr[3])
    pitch = _deg_to_rad(arr[4])
    yaw = _deg_to_rad(arr[5])

    # Euler ZYX → quaternion (intrinsic rotations: roll about X, pitch
    # about Y, yaw about Z, applied in Z-Y-X order).
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    # Pack as Translation3d (x, y, z) + Quaternion (w, x, y, z)
    return struct.pack("<7d", x, y, z, qw, qx, qy, qz)


def _register_pose3d_schema(writer: DataLogWriter, timestamp: int) -> None:
    """Register the ``Pose3d`` struct schema and its dependencies.

    Uses ``wpimath.geometry.Pose3d`` for schema registration so the schema
    bytes exactly match what AdvantageScope expects.
    """
    from wpimath.geometry import Pose3d

    writer.addStructSchema(Pose3d, timestamp)


def copy_and_augment(
    input_path: str | Path,
    output_path: str | Path,
    *,
    verbose: bool = True,
) -> AugmentResult:
    """Copy a ``.wpilog`` file and add ``struct:Pose3d`` botpose signals.

    Parameters
    ----------
    input_path:
        Path to the source ``.wpilog`` file.
    output_path:
        Path for the augmented output ``.wpilog`` file.  Any existing
        file at this path is replaced.
    verbose:
        If *True*, print progress to stderr.

    Returns
    -------
    AugmentResult
        Summary of what was written.
    """
    input_path = str(input_path)
    output_path = str(output_path)

    reader = DataLogReader(input_path)
    if not reader.isValid():
        raise ValueError(f"Invalid wpilog file: {input_path}")

    extra_header = reader.getExtraHeader()

    # --- Pass 1: read everything into plain Python data ---
    starts: list[tuple[str, str, str, int, int]] = []   # (name, type, meta, ts, in_eid)
    finishes: list[tuple[int, int]] = []                 # (in_eid, ts)
    set_metas: list[tuple[int, str, int]] = []           # (in_eid, meta, ts)
    data_recs: list[tuple[int, bytes, int]] = []         # (in_eid, raw, ts)
    target_in_eids: set[int] = set()

    for rec in reader:
        ts = rec.getTimestamp()
        if rec.isControl():
            if rec.isStart():
                s = rec.getStartData()
                starts.append((s.name, s.type, s.metadata, ts, s.entry))
                if _is_botpose_signal(s.name):
                    target_in_eids.add(s.entry)
            elif rec.isFinish():
                finishes.append((rec.getFinishEntry(), ts))
            elif rec.isSetMetadata():
                m = rec.getSetMetadataData()
                set_metas.append((m.entry, m.metadata, ts))
        else:
            data_recs.append((rec.getEntry(), bytes(rec.getRaw()), ts))

    del reader

    if verbose:
        print(
            f"Read {len(starts)} entries, {len(data_recs)} data records",
            file=sys.stderr,
            flush=True,
        )

    # --- Pass 2: write output ---
    out = Path(output_path)
    if out.exists():
        out.unlink()
    writer = DataLogWriter(output_path, extra_header)

    id_remap: dict[int, int] = {}
    target_entries: dict[int, int] = {}   # input_eid → output_eid of Pose3d
    entry_type: dict[int, str] = {}
    pose3d_names: list[str] = []
    seen_pose3d_names: set[str] = set()
    schema_registered = False

    t0 = time.perf_counter()

    for name, typ, meta, ts, in_eid in starts:
        out_eid = writer.start(name, typ, meta, ts)
        id_remap[in_eid] = out_eid
        entry_type[in_eid] = typ

        if in_eid in target_in_eids:
            if not schema_registered:
                _register_pose3d_schema(writer, ts)
                schema_registered = True
            p3d_name = _pose3d_entry_name(name)
            p3d_eid = writer.start(p3d_name, "struct:Pose3d", "", ts)
            target_entries[in_eid] = p3d_eid
            if p3d_name not in seen_pose3d_names:
                pose3d_names.append(p3d_name)
                seen_pose3d_names.add(p3d_name)

    for in_eid, metadata, ts in set_metas:
        writer.setMetadata(id_remap.get(in_eid, in_eid), metadata, ts)

    pose3d_count = 0
    skipped_count = 0
    writes_since_flush = 0

    for i, (in_eid, raw, ts) in enumerate(data_recs):
        out_eid = id_remap.get(in_eid)
        if out_eid is None:
            continue
        writer.appendRaw(out_eid, raw, ts)
        writes_since_flush += 1
        if writes_since_flush >= _FLUSH_INTERVAL:
            writer.flush()
            writes_since_flush = 0

        # Convert target botpose arrays to Pose3d
        if in_eid in target_entries:
            typ = entry_type.get(in_eid, "")
            if typ == "double[]":
                packed = _botpose_array_to_packed_pose3d(raw)
                if packed is not None:
                    writer.appendRaw(target_entries[in_eid], packed, ts)
                    writes_since_flush += 1
                    if writes_since_flush >= _FLUSH_INTERVAL:
                        writer.flush()
                        writes_since_flush = 0
                    pose3d_count += 1
                else:
                    skipped_count += 1
            else:
                skipped_count += 1

        if verbose and (i + 1) % 200000 == 0:
            print(
                f"  {i + 1}/{len(data_recs)} records...",
                file=sys.stderr,
                flush=True,
            )

    for in_eid, ts in finishes:
        writer.finish(id_remap.get(in_eid, in_eid), ts)

    writer.flush()
    writer.stop()

    elapsed = time.perf_counter() - t0
    input_size = Path(input_path).stat().st_size
    output_size = Path(output_path).stat().st_size

    return AugmentResult(
        input_path=input_path,
        output_path=output_path,
        input_size_bytes=input_size,
        output_size_bytes=output_size,
        total_records=len(data_recs),
        total_entries=len(starts),
        pose3d_written=pose3d_count,
        pose3d_skipped=skipped_count,
        pose3d_signals=pose3d_names,
        elapsed_s=elapsed,
    )
