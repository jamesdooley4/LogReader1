"""Augment a .wpilog file with Pose3d struct signals derived from Limelight botpose arrays.

For each ``botpose_wpiblue`` and ``botpose_orb_wpiblue`` double-array signal
found in the input log, this script creates a parallel ``struct:Pose3d`` signal
that AdvantageScope can visualize in its 2D/3D field view.

Usage:
    python scripts/augment_botpose.py <input.wpilog> [-o <output.wpilog>]

If no output path is given, the output file is written next to the input with
an ``_augmented`` suffix.

Botpose array layout (Limelight NetworkTables):
    [0] x (meters)      [1] y (meters)      [2] z (meters)
    [3] roll (deg)      [4] pitch (deg)      [5] yaw (deg)
    [6] latency (ms)    [7] tag_count        [8] tag_span
    [9] avg_dist        [10] avg_area        [11..] per-tag data
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from pathlib import Path

from wpiutil._wpiutil import DataLogWriter
from wpiutil.log import DataLogReader
from wpiutil import wpistruct
from wpimath.geometry import Pose3d, Rotation3d

# DataLogWriter deadlocks when its internal buffer fills up (pybind11/GIL
# issue).  Flushing every N appends prevents the overflow.
_FLUSH_INTERVAL = 5000

# Suffixes that identify the botpose signals we want to augment.
_TARGET_SUFFIXES = (
    "/botpose_wpiblue",
    "/botpose_orb_wpiblue",
)


def _deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def _botpose_to_pose3d(arr: list[float]) -> Pose3d | None:
    """Convert a Limelight botpose double array to a Pose3d.

    Returns None if the array is too short or has no tag detections.
    """
    if len(arr) < 8:
        return None
    tag_count = int(arr[7])
    if tag_count < 1:
        return None
    x, y, z = arr[0], arr[1], arr[2]
    # Skip zero-pose (no valid detection)
    if abs(x) < 0.001 and abs(y) < 0.001:
        return None
    roll_rad = _deg_to_rad(arr[3])
    pitch_rad = _deg_to_rad(arr[4])
    yaw_rad = _deg_to_rad(arr[5])
    return Pose3d(x, y, z, Rotation3d(roll_rad, pitch_rad, yaw_rad))


def _is_target_signal(name: str) -> bool:
    return any(name.endswith(suffix) for suffix in _TARGET_SUFFIXES)


def _pose3d_name(original_name: str) -> str:
    """Derive the Pose3d signal name from the original botpose signal name."""
    return original_name + "_pose3d"


def augment(input_path: str, output_path: str) -> None:
    reader = DataLogReader(input_path)
    if not reader.isValid():
        print(f"Error: invalid wpilog file: {input_path}", file=sys.stderr)
        sys.exit(1)

    extra_header = reader.getExtraHeader()

    # --- Pass 1: read everything into plain Python data ---
    # We must fully separate reading from writing because the C++ DataLog
    # mutex deadlocks if appendRaw is called while DataLogRecord buffers
    # from the reader are still live.
    starts: list[tuple[str, str, str, int, int]] = []  # (name, type, meta, ts, in_eid)
    finishes: list[tuple[int, int]] = []                # (in_eid, ts)
    set_metas: list[tuple[int, str, int]] = []          # (in_eid, meta, ts)
    data_recs: list[tuple[int, bytes, int]] = []        # (in_eid, raw, ts)
    target_in_eids: set[int] = set()                    # entry IDs for botpose signals

    for rec in reader:
        ts = rec.getTimestamp()
        if rec.isControl():
            if rec.isStart():
                s = rec.getStartData()
                starts.append((s.name, s.type, s.metadata, ts, s.entry))
                if _is_target_signal(s.name):
                    target_in_eids.add(s.entry)
            elif rec.isFinish():
                finishes.append((rec.getFinishEntry(), ts))
            elif rec.isSetMetadata():
                m = rec.getSetMetadataData()
                set_metas.append((m.entry, m.metadata, ts))
        else:
            in_eid = rec.getEntry()
            data_recs.append((in_eid, bytes(rec.getRaw()), ts))

    del reader  # release all C++ resources

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
    target_entries: dict[int, int] = {}
    pose3d_names: list[str] = []
    entry_type: dict[int, str] = {}
    schema_registered = False
    seen_pose3d_names: set[str] = set()

    t0 = time.perf_counter()

    for name, typ, meta, ts, in_eid in starts:
        out_eid = writer.start(name, typ, meta, ts)
        id_remap[in_eid] = out_eid
        entry_type[in_eid] = typ

        if in_eid in target_in_eids:
            if not schema_registered:
                writer.addStructSchema(Pose3d, ts)
                schema_registered = True
            p3d_name = _pose3d_name(name)
            # start() is ref-counted: duplicate name+type returns same ID
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
            if typ == "double[]" and len(raw) >= 64:  # at least 8 doubles
                n_doubles = len(raw) // 8
                arr = list(struct.unpack(f"<{n_doubles}d", raw))
                pose = _botpose_to_pose3d(arr)
                if pose is not None:
                    packed = wpistruct.pack(pose)
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

        if (i + 1) % 200000 == 0:
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
    input_size_mb = Path(input_path).stat().st_size / (1024 * 1024)
    output_size_mb = Path(output_path).stat().st_size / (1024 * 1024)

    print(f"Input:   {input_path} ({input_size_mb:.1f} MB)")
    print(f"Output:  {output_path} ({output_size_mb:.1f} MB)")
    print(f"Records: {len(data_recs)} data records copied")
    print(f"Pose3d:  {pose3d_count} samples written, {skipped_count} skipped (no tags / zero pose)")
    print(f"Signals: {', '.join(sorted(pose3d_names))}")
    print(f"Time:    {elapsed:.1f}s")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Augment a .wpilog with Pose3d struct signals from Limelight botpose arrays."
    )
    parser.add_argument("input", help="Path to input .wpilog file")
    parser.add_argument(
        "-o", "--output",
        help="Path to output .wpilog file (default: <input>_augmented.wpilog)",
    )
    args = parser.parse_args()

    input_path = args.input
    if not Path(input_path).is_file():
        print(f"Error: file not found: {input_path}", file=sys.stderr)
        sys.exit(1)

    if args.output:
        output_path = args.output
    else:
        p = Path(input_path)
        output_path = str(p.with_stem(p.stem + "_augmented"))

    augment(input_path, output_path)


if __name__ == "__main__":
    main()
