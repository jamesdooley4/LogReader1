"""Tests for the pose-analysis analyzer and its public API."""

from __future__ import annotations

import math

from logreader.analyzers.pose_analysis import (
    PoseSample,
    PoseSource,
    SourceMetrics,
    DivergenceEvent,
    discover_pose_sources,
    build_reference_path,
    interpolate_pose_at,
    compute_velocity_at,
    compute_divergence_metrics,
    find_divergence_events,
    PoseAnalysisAnalyzer,
    _normalize_angle,
    _angle_diff,
)
from logreader.models import (
    LogData,
    LogMetadata,
    SignalData,
    SignalInfo,
    SignalType,
    TimestampedValue,
)
from logreader.analyzers import list_analyzers, get_analyzer


# ── Helpers ─────────────────────────────────────────────────────────────


def _make_log(signals: dict[str, SignalData] | None = None) -> LogData:
    """Create a minimal LogData for testing."""
    return LogData(
        metadata=LogMetadata(file_path="test.wpilog", is_valid=True),
        signals=signals or {},
    )


def _make_double_array_signal(
    name: str, values: list[tuple[int, list[float]]]
) -> SignalData:
    """Create a double[] signal from (timestamp_us, [values]) pairs."""
    info = SignalInfo(entry_id=1, name=name, type=SignalType.DOUBLE_ARRAY)
    return SignalData(
        info=info,
        values=[TimestampedValue(timestamp_us=ts, value=vals) for ts, vals in values],
    )


def _make_double_signal(name: str, values: list[tuple[int, float]]) -> SignalData:
    """Create a double signal from (timestamp_us, value) pairs."""
    info = SignalInfo(entry_id=1, name=name, type=SignalType.DOUBLE)
    return SignalData(
        info=info,
        values=[TimestampedValue(timestamp_us=ts, value=val) for ts, val in values],
    )


def _make_straight_line_odom(
    n: int = 100,
    dt_us: int = 4000,
    speed_mps: float = 1.0,
    theta_deg: float = 0.0,
    start_us: int = 0,
) -> list[tuple[int, list[float]]]:
    """Generate odometry data for a straight-line drive.

    Returns (timestamp_us, [x, y, theta_degrees]) tuples.
    """
    dt_s = dt_us / 1_000_000.0
    result = []
    for i in range(n):
        ts = start_us + i * dt_us
        x = speed_mps * i * dt_s * math.cos(math.radians(theta_deg))
        y = speed_mps * i * dt_s * math.sin(math.radians(theta_deg))
        result.append((ts, [x, y, theta_deg]))
    return result


# ── Angle utilities ─────────────────────────────────────────────────────


class TestAngleUtils:
    def test_normalize_angle_positive(self) -> None:
        assert abs(_normalize_angle(math.pi) - math.pi) < 1e-10

    def test_normalize_angle_wrap(self) -> None:
        result = _normalize_angle(3 * math.pi)
        assert abs(result - math.pi) < 1e-10

    def test_normalize_angle_negative(self) -> None:
        result = _normalize_angle(-math.pi / 2)
        assert abs(result + math.pi / 2) < 1e-10

    def test_angle_diff_zero(self) -> None:
        assert abs(_angle_diff(1.0, 1.0)) < 1e-10

    def test_angle_diff_wraparound(self) -> None:
        # Going from near -π to near π should give a small negative diff
        d = _angle_diff(-math.pi + 0.1, math.pi - 0.1)
        assert abs(d) < 0.3  # should be ~-0.2


# ── Registration ────────────────────────────────────────────────────────


def test_pose_analysis_registered() -> None:
    assert "pose-analysis" in list_analyzers()


def test_get_analyzer_returns_class() -> None:
    cls = get_analyzer("pose-analysis")
    assert cls is PoseAnalysisAnalyzer


# ── Signal discovery ────────────────────────────────────────────────────


class TestDiscovery:
    def test_discovers_robotPose(self) -> None:
        odom_data = _make_straight_line_odom(50)
        sig = _make_double_array_signal("NT:/Pose/robotPose", odom_data)
        log = _make_log({"NT:/Pose/robotPose": sig})

        sources = discover_pose_sources(log)
        parseable = [s for s in sources if s.samples]
        assert len(parseable) == 1
        assert parseable[0].name == "NT:/Pose/robotPose"
        assert parseable[0].source_class == "odometry"
        assert parseable[0].sample_count == 50

    def test_discovers_field_robot(self) -> None:
        odom_data = _make_straight_line_odom(20)
        sig = _make_double_array_signal("NT:/SmartDashboard/Field/Robot", odom_data)
        log = _make_log({"NT:/SmartDashboard/Field/Robot": sig})

        sources = discover_pose_sources(log)
        parseable = [s for s in sources if s.samples]
        assert len(parseable) == 1
        assert parseable[0].source_class == "odometry"

    def test_discovers_limelight(self) -> None:
        # Create a botpose_wpiblue signal with valid targets
        bp_data = [
            (
                100_000 + i * 20_000,
                [8.0, 4.0, 0.0, 0.0, 0.0, 45.0, 25.0, 2.0, 0.0, 3.0, 0.5, 1.0],
            )
            for i in range(20)
        ]
        bp_sig = _make_double_array_signal("NT:/limelight-a/botpose_wpiblue", bp_data)
        tv_sig = _make_double_signal("NT:/limelight-a/tv", [])

        log = _make_log(
            {
                "NT:/limelight-a/botpose_wpiblue": bp_sig,
                "NT:/limelight-a/tv": tv_sig,
            }
        )

        sources = discover_pose_sources(log)
        vision = [s for s in sources if s.source_class == "vision" and s.samples]
        assert len(vision) == 1
        assert vision[0].name == "limelight-a"

    def test_skips_botpose_when_wpiblue_exists(self) -> None:
        """Should not create duplicate sources for the same camera."""
        bp_data = [
            (
                100_000 + i * 20_000,
                [8.0, 4.0, 0.0, 0.0, 0.0, 45.0, 25.0, 2.0, 0.0, 3.0, 0.5, 1.0],
            )
            for i in range(20)
        ]
        bp_wpiblue = _make_double_array_signal(
            "NT:/limelight-a/botpose_wpiblue", bp_data
        )
        bp_raw = _make_double_array_signal("NT:/limelight-a/botpose", bp_data)

        log = _make_log(
            {
                "NT:/limelight-a/botpose_wpiblue": bp_wpiblue,
                "NT:/limelight-a/botpose": bp_raw,
            }
        )

        sources = discover_pose_sources(log)
        vision = [s for s in sources if s.source_class == "vision" and s.samples]
        assert len(vision) == 1  # not 2

    def test_skips_short_signals(self) -> None:
        odom_data = _make_straight_line_odom(3)  # too few samples
        sig = _make_double_array_signal("NT:/Pose/robotPose", odom_data)
        log = _make_log({"NT:/Pose/robotPose": sig})

        sources = discover_pose_sources(log)
        parseable = [s for s in sources if s.samples]
        assert len(parseable) == 0

    def test_no_duplicate_odom_sources(self) -> None:
        """A signal matching multiple patterns should only appear once."""
        odom_data = _make_straight_line_odom(50)
        sig = _make_double_array_signal("NT:/Pose/robotPose", odom_data)
        log = _make_log({"NT:/Pose/robotPose": sig})

        sources = discover_pose_sources(log)
        parseable = [s for s in sources if s.samples]
        odom_names = [s.name for s in parseable if s.source_class == "odometry"]
        assert len(odom_names) == len(set(odom_names))

    def test_detects_struct_signals(self) -> None:
        struct_info = SignalInfo(
            entry_id=2, name="NT:/DriveState/Pose", type=SignalType.STRUCT
        )
        struct_sig = SignalData(
            info=struct_info,
            values=[TimestampedValue(timestamp_us=0, value=b"\x00" * 24)],
        )
        log = _make_log({"NT:/DriveState/Pose": struct_sig})

        sources = discover_pose_sources(log)
        struct_only = [s for s in sources if not s.samples and s.notes]
        assert len(struct_only) == 1
        assert "struct" in struct_only[0].notes[0].lower()


# ── Interpolation ───────────────────────────────────────────────────────


class TestInterpolation:
    def _make_path(self) -> list[PoseSample]:
        return [
            PoseSample(0, 0.0, 0.0, 0.0, "test", "fused"),
            PoseSample(100_000, 1.0, 0.0, 0.0, "test", "fused"),
            PoseSample(200_000, 2.0, 0.0, 0.0, "test", "fused"),
        ]

    def test_interpolate_at_boundary(self) -> None:
        path = self._make_path()
        p = interpolate_pose_at(path, 0)
        assert p is not None
        assert p.x == 0.0

    def test_interpolate_midpoint(self) -> None:
        path = self._make_path()
        p = interpolate_pose_at(path, 50_000)
        assert p is not None
        assert abs(p.x - 0.5) < 1e-6

    def test_interpolate_beyond_end(self) -> None:
        path = self._make_path()
        p = interpolate_pose_at(path, 300_000)
        assert p is not None
        assert p.x == 2.0

    def test_interpolate_empty_path(self) -> None:
        assert interpolate_pose_at([], 100) is None

    def test_interpolate_angle_wraparound(self) -> None:
        path = [
            PoseSample(0, 0.0, 0.0, math.pi - 0.1, "t", "f"),
            PoseSample(100_000, 0.0, 0.0, -math.pi + 0.1, "t", "f"),
        ]
        p = interpolate_pose_at(path, 50_000)
        assert p is not None
        # Should interpolate through ±π, not through 0
        assert abs(p.theta) > 2.5


# ── Velocity ────────────────────────────────────────────────────────────


class TestVelocity:
    def test_constant_velocity(self) -> None:
        # 1 m/s in x direction
        path = [PoseSample(i * 100_000, i * 0.1, 0.0, 0.0, "t", "f") for i in range(10)]
        vx, vy, omega = compute_velocity_at(path, 500_000)
        assert abs(vx - 1.0) < 0.01
        assert abs(vy) < 0.01
        assert abs(omega) < 0.01

    def test_stationary(self) -> None:
        path = [PoseSample(i * 100_000, 5.0, 3.0, 0.5, "t", "f") for i in range(10)]
        vx, vy, omega = compute_velocity_at(path, 500_000)
        assert abs(vx) < 0.01
        assert abs(vy) < 0.01
        assert abs(omega) < 0.01

    def test_empty_path(self) -> None:
        vx, vy, omega = compute_velocity_at([], 100)
        assert (vx, vy, omega) == (0.0, 0.0, 0.0)


# ── Reference path building ────────────────────────────────────────────


class TestBuildReferencePath:
    def test_odom_only(self) -> None:
        odom_data = _make_straight_line_odom(100, speed_mps=2.0)
        sig = _make_double_array_signal("NT:/Pose/robotPose", odom_data)
        log = _make_log({"NT:/Pose/robotPose": sig})

        path = build_reference_path(log)
        assert len(path) == 100
        # Should roughly follow the straight line (smoothed)
        assert path[-1].x > 0.5

    def test_odom_plus_vision(self) -> None:
        # Odometry going straight at 1 m/s in x
        odom_data = _make_straight_line_odom(100, dt_us=10_000, speed_mps=1.0)
        odom_sig = _make_double_array_signal("NT:/Pose/robotPose", odom_data)

        # Vision says the robot is slightly offset in y (0.1 m)
        vision_data = [
            (
                i * 50_000,
                [i * 0.05, 0.1, 0.0, 0.0, 0.0, 0.0, 20.0, 2.0, 0.0, 2.0, 0.5, 1.0],
            )
            for i in range(20)
        ]
        vision_sig = _make_double_array_signal(
            "NT:/limelight-a/botpose_wpiblue", vision_data
        )

        log = _make_log(
            {
                "NT:/Pose/robotPose": odom_sig,
                "NT:/limelight-a/botpose_wpiblue": vision_sig,
            }
        )

        path = build_reference_path(log)
        assert len(path) == 100
        # The fused path should be slightly pulled toward y=0.1
        mid = path[len(path) // 2]
        assert mid.y > 0.0  # should have been nudged up from 0

    def test_empty_log(self) -> None:
        log = _make_log({})
        path = build_reference_path(log)
        assert path == []


# ── Divergence metrics ──────────────────────────────────────────────────


class TestDivergenceMetrics:
    def test_identical_source(self) -> None:
        """A source identical to the reference should have near-zero error."""
        samples = [
            PoseSample(i * 10_000, i * 0.01, 0.0, 0.0, "test", "odometry")
            for i in range(100)
        ]
        reference = samples[:]
        m = compute_divergence_metrics(samples, reference)
        assert m.translation_rms_m < 0.001
        assert m.heading_rms_rad < 0.001
        assert m.confidence == "high"

    def test_offset_source(self) -> None:
        """A source with a constant 1 m offset."""
        reference = [
            PoseSample(i * 10_000, i * 0.01, 0.0, 0.0, "ref", "fused")
            for i in range(100)
        ]
        source = [
            PoseSample(i * 10_000, i * 0.01 + 1.0, 0.0, 0.0, "test", "vision")
            for i in range(100)
        ]
        m = compute_divergence_metrics(source, reference)
        assert abs(m.translation_rms_m - 1.0) < 0.01
        assert m.translation_max_m > 0.99

    def test_empty_source(self) -> None:
        m = compute_divergence_metrics([], [])
        assert m.sample_count == 0
        assert m.confidence == "low"


# ── Divergence events ──────────────────────────────────────────────────


class TestDivergenceEvents:
    def test_finds_divergence_window(self) -> None:
        reference = [
            PoseSample(i * 10_000, 0.0, 0.0, 0.0, "ref", "fused") for i in range(100)
        ]
        # Source matches for first 30 samples, then jumps 2 m for 30 samples
        source = []
        for i in range(100):
            x = 0.0 if i < 30 or i >= 60 else 2.0
            source.append(PoseSample(i * 10_000, x, 0.0, 0.0, "test", "vision"))

        events = find_divergence_events(source, reference)
        assert len(events) >= 1
        assert events[0].peak_translation_error_m > 1.5


# ── Analyzer run ────────────────────────────────────────────────────────


class TestAnalyzerRun:
    def test_run_with_synthetic_data(self) -> None:
        odom_data = _make_straight_line_odom(100, speed_mps=1.0)
        sig = _make_double_array_signal("NT:/Pose/robotPose", odom_data)
        log = _make_log({"NT:/Pose/robotPose": sig})

        analyzer = PoseAnalysisAnalyzer()
        result = analyzer.run(log)
        assert result.analyzer_name == "pose-analysis"
        assert "Discovered" in result.summary
        assert result.extra["reference_path_samples"] == 100

    def test_run_empty_log(self) -> None:
        log = _make_log({})
        analyzer = PoseAnalysisAnalyzer()
        result = analyzer.run(log)
        assert result.analyzer_name == "pose-analysis"
        assert result.extra["reference_path_samples"] == 0
