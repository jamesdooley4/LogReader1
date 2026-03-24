"""Tests for wpilog_writer — log augmentation with Pose3d signals."""

from __future__ import annotations

import math
import struct
import tempfile
from pathlib import Path

import pytest

from logreader.wpilog_writer import (
    AugmentResult,
    _botpose_array_to_packed_pose3d,
    _deg_to_rad,
    _is_botpose_signal,
    _pose3d_entry_name,
    copy_and_augment,
)


# ---------------------------------------------------------------------------
# Unit tests for helper functions
# ---------------------------------------------------------------------------


class TestDegToRad:
    def test_zero(self) -> None:
        assert _deg_to_rad(0.0) == 0.0

    def test_ninety(self) -> None:
        assert abs(_deg_to_rad(90.0) - math.pi / 2) < 1e-12

    def test_negative(self) -> None:
        assert abs(_deg_to_rad(-180.0) - (-math.pi)) < 1e-12


class TestIsBotposeSignal:
    def test_wpiblue(self) -> None:
        assert _is_botpose_signal("NT:/limelight-a/botpose_wpiblue")

    def test_orb_wpiblue(self) -> None:
        assert _is_botpose_signal("NT:/limelight-b/botpose_orb_wpiblue")

    def test_non_target(self) -> None:
        assert not _is_botpose_signal("NT:/limelight-a/botpose_wpired")
        assert not _is_botpose_signal("NT:/limelight-a/botpose")
        assert not _is_botpose_signal("NT:/limelight-a/botpose_orb")

    def test_targetspace(self) -> None:
        assert not _is_botpose_signal("NT:/limelight-a/botpose_targetspace")


class TestPose3dEntryName:
    def test_suffix(self) -> None:
        assert (
            _pose3d_entry_name("NT:/limelight-a/botpose_wpiblue")
            == "NT:/limelight-a/botpose_wpiblue_pose3d"
        )


# ---------------------------------------------------------------------------
# Tests for botpose array → packed Pose3d conversion
# ---------------------------------------------------------------------------


def _make_botpose_raw(
    x: float = 5.0,
    y: float = 3.0,
    z: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 45.0,
    latency: float = 15.0,
    tag_count: float = 1.0,
    tag_span: float = 0.0,
    avg_dist: float = 4.0,
    avg_area: float = 0.02,
) -> bytes:
    """Build raw bytes for a Limelight botpose double array."""
    return struct.pack(
        "<11d",
        x, y, z, roll, pitch, yaw,
        latency, tag_count, tag_span, avg_dist, avg_area,
    )


class TestBotposeArrayToPackedPose3d:
    def test_valid_sample(self) -> None:
        raw = _make_botpose_raw(x=5.0, y=3.0, z=0.1, yaw=90.0, tag_count=2.0)
        packed = _botpose_array_to_packed_pose3d(raw)
        assert packed is not None
        assert len(packed) == 56  # 7 doubles

        # Unpack and verify translation
        values = struct.unpack("<7d", packed)
        assert abs(values[0] - 5.0) < 1e-12  # x
        assert abs(values[1] - 3.0) < 1e-12  # y
        assert abs(values[2] - 0.1) < 1e-12  # z

        # Verify quaternion is unit quaternion
        qw, qx, qy, qz = values[3], values[4], values[5], values[6]
        norm = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
        assert abs(norm - 1.0) < 1e-10

    def test_yaw_only_90(self) -> None:
        """90° yaw → quaternion with qw≈0.707, qz≈0.707."""
        raw = _make_botpose_raw(x=1.0, y=1.0, yaw=90.0, tag_count=1.0)
        packed = _botpose_array_to_packed_pose3d(raw)
        assert packed is not None
        values = struct.unpack("<7d", packed)
        qw, qx, qy, qz = values[3], values[4], values[5], values[6]
        assert abs(qw - math.cos(math.pi / 4)) < 1e-10
        assert abs(qx) < 1e-10
        assert abs(qy) < 1e-10
        assert abs(qz - math.sin(math.pi / 4)) < 1e-10

    def test_no_tags_returns_none(self) -> None:
        raw = _make_botpose_raw(tag_count=0.0)
        assert _botpose_array_to_packed_pose3d(raw) is None

    def test_zero_pose_returns_none(self) -> None:
        raw = _make_botpose_raw(x=0.0, y=0.0, tag_count=1.0)
        assert _botpose_array_to_packed_pose3d(raw) is None

    def test_short_array_returns_none(self) -> None:
        # Only 6 doubles = 48 bytes, fewer than the 64 needed
        raw = struct.pack("<6d", 1.0, 2.0, 0.0, 0.0, 0.0, 45.0)
        assert _botpose_array_to_packed_pose3d(raw) is None

    def test_empty_returns_none(self) -> None:
        assert _botpose_array_to_packed_pose3d(b"") is None


# ---------------------------------------------------------------------------
# Integration test: copy_and_augment with a real DataLogWriter round-trip
# ---------------------------------------------------------------------------

class TestCopyAndAugment:
    """Integration tests that write a .wpilog, augment it, and verify."""

    def _create_test_wpilog(self, path: str) -> None:
        """Create a minimal .wpilog with a botpose signal."""
        from wpiutil._wpiutil import DataLogWriter

        writer = DataLogWriter(path)

        # Non-botpose signal
        eid_temp = writer.start("NT:/sensors/temp", "double", "", 1000)
        writer.appendRaw(eid_temp, struct.pack("<d", 42.5), 10000)
        writer.appendRaw(eid_temp, struct.pack("<d", 43.0), 20000)

        # Botpose signal with valid detections
        eid_bp = writer.start(
            "NT:/limelight-a/botpose_wpiblue", "double[]", "", 1000
        )
        # Sample with tags
        arr_valid = _make_botpose_raw(
            x=5.0, y=3.0, z=0.0, yaw=45.0, tag_count=2.0
        )
        writer.appendRaw(eid_bp, arr_valid, 15000)

        # Sample with no tags (should be skipped)
        arr_no_tags = _make_botpose_raw(x=5.0, y=3.0, tag_count=0.0)
        writer.appendRaw(eid_bp, arr_no_tags, 25000)

        # Another valid sample
        arr_valid2 = _make_botpose_raw(
            x=7.0, y=1.0, z=0.2, yaw=-90.0, tag_count=1.0
        )
        writer.appendRaw(eid_bp, arr_valid2, 35000)

        writer.flush()
        writer.stop()

    def test_basic_augmentation(self, tmp_path: Path) -> None:
        input_file = str(tmp_path / "input.wpilog")
        output_file = str(tmp_path / "output.wpilog")

        self._create_test_wpilog(input_file)
        result = copy_and_augment(input_file, output_file, verbose=False)

        assert isinstance(result, AugmentResult)
        assert result.pose3d_written == 2  # 2 valid, 1 skipped (no tags)
        assert result.pose3d_skipped == 1
        assert len(result.pose3d_signals) == 1
        assert result.pose3d_signals[0] == "NT:/limelight-a/botpose_wpiblue_pose3d"
        assert Path(output_file).exists()
        assert result.output_size_bytes > result.input_size_bytes

    def test_output_readable(self, tmp_path: Path) -> None:
        """Verify the output file is valid and contains expected entries."""
        from wpiutil.log import DataLogReader

        input_file = str(tmp_path / "input.wpilog")
        output_file = str(tmp_path / "output.wpilog")

        self._create_test_wpilog(input_file)
        copy_and_augment(input_file, output_file, verbose=False)

        reader = DataLogReader(output_file)
        assert reader.isValid()

        entries: dict[int, object] = {}
        data_counts: dict[str, int] = {}
        for rec in reader:
            if rec.isControl() and rec.isStart():
                s = rec.getStartData()
                entries[s.entry] = s
            elif not rec.isControl():
                eid = rec.getEntry()
                info = entries.get(eid)
                if info:
                    name = info.name
                    data_counts[name] = data_counts.get(name, 0) + 1

        # Original signals preserved
        assert "NT:/sensors/temp" in data_counts
        assert data_counts["NT:/sensors/temp"] == 2
        assert "NT:/limelight-a/botpose_wpiblue" in data_counts
        assert data_counts["NT:/limelight-a/botpose_wpiblue"] == 3  # all 3 samples

        # Pose3d signal added
        assert "NT:/limelight-a/botpose_wpiblue_pose3d" in data_counts
        assert data_counts["NT:/limelight-a/botpose_wpiblue_pose3d"] == 2  # only valid

        # Verify struct type
        pose3d_entry = None
        for eid, info in entries.items():
            if info.name == "NT:/limelight-a/botpose_wpiblue_pose3d":
                pose3d_entry = info
                break
        assert pose3d_entry is not None
        assert pose3d_entry.type == "struct:Pose3d"

    def test_pose3d_values_correct(self, tmp_path: Path) -> None:
        """Verify Pose3d values decode correctly from the output file."""
        from wpiutil.log import DataLogReader
        from wpimath.geometry import Pose3d
        from wpiutil import wpistruct

        input_file = str(tmp_path / "input.wpilog")
        output_file = str(tmp_path / "output.wpilog")

        self._create_test_wpilog(input_file)
        copy_and_augment(input_file, output_file, verbose=False)

        reader = DataLogReader(output_file)
        entries: dict[int, object] = {}
        poses: list[Pose3d] = []

        for rec in reader:
            if rec.isControl() and rec.isStart():
                s = rec.getStartData()
                entries[s.entry] = s
            elif not rec.isControl():
                eid = rec.getEntry()
                info = entries.get(eid)
                if info and info.type == "struct:Pose3d":
                    raw = rec.getRaw()
                    pose = wpistruct.unpack(Pose3d, raw)
                    poses.append(pose)

        assert len(poses) == 2

        # First pose: x=5, y=3, z=0, yaw=45°
        assert abs(poses[0].X() - 5.0) < 1e-6
        assert abs(poses[0].Y() - 3.0) < 1e-6
        assert abs(poses[0].Z() - 0.0) < 1e-6

        # Second pose: x=7, y=1, z=0.2, yaw=-90°
        assert abs(poses[1].X() - 7.0) < 1e-6
        assert abs(poses[1].Y() - 1.0) < 1e-6
        assert abs(poses[1].Z() - 0.2) < 1e-6

    def test_no_target_signals(self, tmp_path: Path) -> None:
        """Log with no botpose signals produces zero augmentations."""
        from wpiutil._wpiutil import DataLogWriter

        input_file = str(tmp_path / "input.wpilog")
        output_file = str(tmp_path / "output.wpilog")

        writer = DataLogWriter(input_file)
        eid = writer.start("NT:/sensors/temp", "double", "", 1000)
        writer.appendRaw(eid, struct.pack("<d", 42.5), 10000)
        writer.flush()
        writer.stop()

        result = copy_and_augment(input_file, output_file, verbose=False)
        assert result.pose3d_written == 0
        assert result.pose3d_skipped == 0
        assert result.pose3d_signals == []

    def test_overwrites_existing_output(self, tmp_path: Path) -> None:
        """Verify that an existing output file is replaced."""
        input_file = str(tmp_path / "input.wpilog")
        output_file = str(tmp_path / "output.wpilog")

        self._create_test_wpilog(input_file)

        # Create a dummy output file
        Path(output_file).write_text("dummy")

        result = copy_and_augment(input_file, output_file, verbose=False)
        assert result.pose3d_written == 2
        assert result.output_size_bytes > 5  # bigger than "dummy"

    def test_timestamps_preserved(self, tmp_path: Path) -> None:
        """Verify data record timestamps match the input."""
        from wpiutil.log import DataLogReader

        input_file = str(tmp_path / "input.wpilog")
        output_file = str(tmp_path / "output.wpilog")

        self._create_test_wpilog(input_file)
        copy_and_augment(input_file, output_file, verbose=False)

        # Read output timestamps
        reader = DataLogReader(output_file)
        timestamps = set()
        for rec in reader:
            if not rec.isControl():
                timestamps.add(rec.getTimestamp())

        # Original data timestamps: 10000, 15000, 20000, 25000, 35000
        # Pose3d timestamps mirror valid botpose: 15000, 35000
        assert 10000 in timestamps
        assert 15000 in timestamps
        assert 20000 in timestamps
        assert 25000 in timestamps
        assert 35000 in timestamps
