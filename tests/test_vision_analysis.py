"""Tests for the vision-analysis analyzer."""

from __future__ import annotations

import math

from logreader.analyzers.vision_analysis import (
    VisionAnalyzer,
    VisionFrame,
    TagDetection,
    CameraSummary,
    TagCountBand,
    TagSummary,
    DistanceBand,
    DetectionGap,
    HardwareStats,
    PhaseBreakdown,
    CameraAgreement,
    FieldCell,
    _discover_cameras,
    _parse_rawfiducials,
    _parse_frames,
    _find_detection_gaps,
    _compute_hw_stats,
    _camera_summary,
    _tag_count_table,
    _tag_id_table,
    _distance_band_table,
    _percentile,
    _find_nearest,
    _wrap_yaw_diff,
    _compute_camera_agreement,
    _compute_mt1_mt2_divergence,
    _summarize_mt1_mt2,
    _heading_to_octant,
    _compute_heading_coverage,
    _compute_preferred_camera_map,
    _compute_residual_vs_speed,
    _compute_tag_geometry_summary,
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


def _make_botpose(
    x: float,
    y: float,
    yaw: float,
    latency: float,
    tag_count: int,
    tag_span: float = 0.0,
    avg_dist: float = 2.0,
    avg_area: float = 0.01,
) -> list[float]:
    """Build an 11-element botpose_wpiblue array."""
    return [x, y, 0.0, 0.0, 0.0, yaw, latency, float(tag_count), tag_span, avg_dist, avg_area]


def _make_rawfiducials(*tags: tuple[int, float, float, float, float, float, float]) -> list[float]:
    """Build a rawfiducials array from (id, txnc, tync, area, dist_cam, dist_robot, ambiguity) tuples."""
    arr: list[float] = []
    for tag in tags:
        arr.extend(tag)
    return arr


def _make_t2d(
    valid: bool = True,
    count: int = 1,
    tl: float = 15.0,
    cl: float = 8.0,
    tx: float = 5.0,
    ty: float = 3.0,
    txnc: float = 4.5,
    tync: float = 2.5,
    ta: float = 0.01,
    tid: int = 7,
    long_side_px: float = 60.0,
    short_side_px: float = 45.0,
    h_extent_px: float = 50.0,
    v_extent_px: float = 55.0,
    skew_deg: float = -7.0,
) -> list[float]:
    """Build a 17-element t2d array."""
    return [
        1.0 if valid else 0.0, float(count), tl, cl,
        tx, ty, txnc, tync, ta, float(tid),
        -1.0, -1.0,  # classIdx_det, classIdx_cls
        long_side_px, short_side_px,
        h_extent_px, v_extent_px, skew_deg,
    ]


def _build_camera_signals(
    cam: str = "a",
    botpose_data: list[tuple[int, list[float]]] | None = None,
    hb_data: list[tuple[int, float]] | None = None,
    rawfid_data: list[tuple[int, list[float]]] | None = None,
    tl_data: list[tuple[int, float]] | None = None,
    cl_data: list[tuple[int, float]] | None = None,
    stddevs_data: list[tuple[int, list[float]]] | None = None,
    hw_data: list[tuple[int, list[float]]] | None = None,
    t2d_data: list[tuple[int, list[float]]] | None = None,
) -> dict[str, SignalData]:
    """Build a complete set of camera signals for use in a LogData."""
    prefix = f"NT:/limelight-{cam}/"
    signals: dict[str, SignalData] = {}

    if botpose_data:
        signals[prefix + "botpose_wpiblue"] = _make_double_array_signal(
            prefix + "botpose_wpiblue", botpose_data
        )
    if hb_data:
        signals[prefix + "hb"] = _make_double_signal(prefix + "hb", hb_data)
    if rawfid_data:
        signals[prefix + "rawfiducials"] = _make_double_array_signal(
            prefix + "rawfiducials", rawfid_data
        )
    if tl_data:
        signals[prefix + "tl"] = _make_double_signal(prefix + "tl", tl_data)
    if cl_data:
        signals[prefix + "cl"] = _make_double_signal(prefix + "cl", cl_data)
    if stddevs_data:
        signals[prefix + "stddevs"] = _make_double_array_signal(
            prefix + "stddevs", stddevs_data
        )
    if hw_data:
        signals[prefix + "hw"] = _make_double_array_signal(
            prefix + "hw", hw_data
        )
    if t2d_data:
        signals[prefix + "t2d"] = _make_double_array_signal(
            prefix + "t2d", t2d_data
        )
    return signals


# ── Registration ────────────────────────────────────────────────────────


class TestRegistration:
    def test_analyzer_registered(self) -> None:
        assert "vision-analysis" in list_analyzers()

    def test_get_analyzer(self) -> None:
        cls = get_analyzer("vision-analysis")
        assert cls is VisionAnalyzer


# ── Rawfiducials parsing ───────────────────────────────────────────────


class TestParseRawfiducials:
    def test_single_tag(self) -> None:
        arr = _make_rawfiducials((7, 1.5, -2.3, 0.005, 2.5, 3.0, 0.15))
        tags = _parse_rawfiducials(arr)
        assert len(tags) == 1
        assert tags[0].tag_id == 7
        assert tags[0].txnc == 1.5
        assert tags[0].tync == -2.3
        assert tags[0].area == 0.005
        assert tags[0].dist_to_camera == 2.5
        assert tags[0].dist_to_robot == 3.0
        assert tags[0].ambiguity == 0.15

    def test_multiple_tags(self) -> None:
        arr = _make_rawfiducials(
            (1, 0.0, 0.0, 0.01, 1.0, 1.5, 0.1),
            (2, 1.0, 1.0, 0.02, 2.0, 2.5, 0.3),
            (3, 2.0, 2.0, 0.03, 3.0, 3.5, 0.6),
        )
        tags = _parse_rawfiducials(arr)
        assert len(tags) == 3
        assert [t.tag_id for t in tags] == [1, 2, 3]

    def test_empty_array(self) -> None:
        assert _parse_rawfiducials([]) == []

    def test_partial_array(self) -> None:
        # Less than 7 elements → no tags
        assert _parse_rawfiducials([1, 2, 3]) == []


# ── Camera discovery ───────────────────────────────────────────────────


class TestDiscoverCameras:
    def test_single_camera(self) -> None:
        signals = _build_camera_signals(
            cam="a",
            botpose_data=[(i * 20000, _make_botpose(1.0, 2.0, 45.0, 20.0, 1)) for i in range(20)],
        )
        log_data = _make_log(signals)
        cameras = _discover_cameras(log_data)
        assert "limelight-a" in cameras
        assert cameras["limelight-a"]["botpose"] is not None

    def test_two_cameras(self) -> None:
        signals = {}
        for cam in ("a", "b"):
            signals.update(
                _build_camera_signals(
                    cam=cam,
                    botpose_data=[(i * 20000, _make_botpose(1.0, 2.0, 0.0, 20.0, 1)) for i in range(20)],
                )
            )
        log_data = _make_log(signals)
        cameras = _discover_cameras(log_data)
        assert "limelight-a" in cameras
        assert "limelight-b" in cameras

    def test_no_cameras(self) -> None:
        log_data = _make_log({})
        cameras = _discover_cameras(log_data)
        assert cameras == {}


# ── Frame parsing ──────────────────────────────────────────────────────


class TestParseFrames:
    def test_valid_frames_counted(self) -> None:
        botpose = [
            (i * 20000, _make_botpose(1.0 + i * 0.1, 2.0, 0.0, 20.0, 2))
            for i in range(10)
        ]
        # Add some zero-tag frames
        botpose.extend(
            [(200000 + i * 20000, _make_botpose(0.0, 0.0, 0.0, 20.0, 0)) for i in range(5)]
        )
        signals = _build_camera_signals(cam="a", botpose_data=botpose)
        camera_signals: dict[str, SignalData | None] = {
            "botpose": signals["NT:/limelight-a/botpose_wpiblue"],
            "rawfiducials": None,
            "tl": None,
            "cl": None,
            "stddevs": None,
            "hb": None,
        }
        frames, total = _parse_frames("limelight-a", camera_signals)
        assert len(frames) == 10  # Only tag_count > 0 frames
        assert total == 15  # All frames from botpose (no hb signal)

    def test_latency_joined(self) -> None:
        botpose = [(100000, _make_botpose(1.0, 2.0, 0.0, 25.0, 1))]
        tl = [(100000, 15.0)]
        cl = [(100000, 8.0)]
        signals = _build_camera_signals(cam="a", botpose_data=botpose, tl_data=tl, cl_data=cl)
        prefix = "NT:/limelight-a/"
        camera_signals: dict[str, SignalData | None] = {
            "botpose": signals[prefix + "botpose_wpiblue"],
            "rawfiducials": None,
            "tl": signals[prefix + "tl"],
            "cl": signals[prefix + "cl"],
            "stddevs": None,
            "hb": None,
        }
        frames, _ = _parse_frames("limelight-a", camera_signals)
        assert len(frames) == 1
        assert frames[0].pipeline_latency_ms == 15.0
        assert frames[0].capture_latency_ms == 8.0

    def test_rawfiducials_joined(self) -> None:
        botpose = [(100000, _make_botpose(1.0, 2.0, 0.0, 20.0, 2))]
        rawfid = [(100000, _make_rawfiducials(
            (7, 0.0, 0.0, 0.01, 2.0, 2.5, 0.15),
            (8, 1.0, 1.0, 0.02, 3.0, 3.5, 0.25),
        ))]
        signals = _build_camera_signals(cam="a", botpose_data=botpose, rawfid_data=rawfid)
        prefix = "NT:/limelight-a/"
        camera_signals: dict[str, SignalData | None] = {
            "botpose": signals[prefix + "botpose_wpiblue"],
            "rawfiducials": signals[prefix + "rawfiducials"],
            "tl": None,
            "cl": None,
            "stddevs": None,
            "hb": None,
        }
        frames, _ = _parse_frames("limelight-a", camera_signals)
        assert len(frames) == 1
        assert len(frames[0].tags) == 2
        assert frames[0].tags[0].tag_id == 7
        assert frames[0].tags[1].tag_id == 8
        assert frames[0].max_ambiguity == 0.25


# ── Detection gaps ─────────────────────────────────────────────────────


class TestDetectionGaps:
    def test_gap_detected(self) -> None:
        # Frames at 0, 100ms, 800ms — 700ms gap between frames 2 and 3
        frames = [
            VisionFrame(
                timestamp_us=0, camera="cam", x=1.0, y=1.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0, capture_latency_ms=5.0,
            ),
            VisionFrame(
                timestamp_us=100_000, camera="cam", x=1.0, y=1.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0, capture_latency_ms=5.0,
            ),
            VisionFrame(
                timestamp_us=800_000, camera="cam", x=1.0, y=1.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0, capture_latency_ms=5.0,
            ),
        ]
        gaps = _find_detection_gaps(frames, "cam")
        assert len(gaps) == 1
        assert gaps[0].duration_ms == 700.0

    def test_no_gap(self) -> None:
        # All frames within 100ms
        frames = [
            VisionFrame(
                timestamp_us=i * 50_000, camera="cam", x=1.0, y=1.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0, capture_latency_ms=5.0,
            )
            for i in range(10)
        ]
        gaps = _find_detection_gaps(frames, "cam")
        assert gaps == []

    def test_other_camera_ignored(self) -> None:
        frames = [
            VisionFrame(
                timestamp_us=0, camera="cam-a", x=1.0, y=1.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0, capture_latency_ms=5.0,
            ),
            VisionFrame(
                timestamp_us=1_000_000, camera="cam-b", x=1.0, y=1.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0, capture_latency_ms=5.0,
            ),
        ]
        # cam-a has only 1 frame → not enough for gap detection
        gaps = _find_detection_gaps(frames, "cam-a")
        assert gaps == []


# ── Hardware stats ─────────────────────────────────────────────────────


class TestHardwareStats:
    def test_hw_stats(self) -> None:
        hw_data = [
            (i * 700_000, [40.0, 55.0 + i, 62.0, 45.0 + i])
            for i in range(5)
        ]
        signals = _build_camera_signals(cam="a", hw_data=hw_data)
        camera_signals: dict[str, SignalData | None] = {
            "hw": signals["NT:/limelight-a/hw"],
        }
        stats = _compute_hw_stats("limelight-a", camera_signals)
        assert stats.mean_fps == 40.0
        assert stats.peak_cpu_temp == 59.0
        assert stats.mean_ram_mb == 62.0

    def test_no_hw_signal(self) -> None:
        stats = _compute_hw_stats("limelight-a", {"hw": None})
        assert stats.mean_fps == 0.0


# ── Tag count table ────────────────────────────────────────────────────


class TestTagCountTable:
    def test_grouping(self) -> None:
        frames = []
        for tc in [1, 1, 1, 2, 2, 3]:
            frames.append(
                VisionFrame(
                    timestamp_us=0, camera="cam", x=1.0, y=1.0, yaw_deg=0.0,
                    tag_count=tc, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                    total_latency_ms=20.0, pipeline_latency_ms=15.0, capture_latency_ms=5.0,
                    pose_residual_m=0.5 / tc,
                )
            )
        bands = _tag_count_table(frames)
        assert len(bands) == 3
        assert bands[0].tag_count == 1
        assert bands[0].frames == 3
        assert bands[1].tag_count == 2
        assert bands[1].frames == 2
        assert bands[2].tag_count == 3
        assert bands[2].frames == 1


# ── Per-tag table ──────────────────────────────────────────────────────


class TestTagIdTable:
    def test_detections_counted(self) -> None:
        tag1 = TagDetection(tag_id=7, txnc=0.0, tync=0.0, area=0.01,
                           dist_to_camera=2.0, dist_to_robot=2.5, ambiguity=0.15)
        tag2 = TagDetection(tag_id=8, txnc=0.0, tync=0.0, area=0.02,
                           dist_to_camera=3.0, dist_to_robot=3.5, ambiguity=0.55)
        frames = [
            VisionFrame(
                timestamp_us=0, camera="cam", x=1.0, y=1.0, yaw_deg=0.0,
                tag_count=2, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0, capture_latency_ms=5.0,
                tags=[tag1, tag2], pose_residual_m=0.2,
            ),
            VisionFrame(
                timestamp_us=50000, camera="cam", x=1.1, y=1.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0, capture_latency_ms=5.0,
                tags=[tag1], pose_residual_m=0.3,
            ),
        ]
        summaries = _tag_id_table(frames, "cam")
        assert len(summaries) == 2
        # Tag 7 seen twice, tag 8 once
        tag7 = next(s for s in summaries if s.tag_id == 7)
        tag8 = next(s for s in summaries if s.tag_id == 8)
        assert tag7.detections == 2
        assert tag8.detections == 1
        assert tag8.high_ambiguity_pct == 100.0  # ambiguity 0.55 > 0.5


# ── Distance band table ───────────────────────────────────────────────


class TestDistanceBandTable:
    def test_bands(self) -> None:
        tags_close = TagDetection(tag_id=1, txnc=0.0, tync=0.0, area=0.05,
                                  dist_to_camera=0.8, dist_to_robot=1.0, ambiguity=0.05)
        tags_mid = TagDetection(tag_id=2, txnc=0.0, tync=0.0, area=0.01,
                                dist_to_camera=2.5, dist_to_robot=3.0, ambiguity=0.45)
        tags_far = TagDetection(tag_id=3, txnc=0.0, tync=0.0, area=0.002,
                                dist_to_camera=5.5, dist_to_robot=6.0, ambiguity=0.75)
        frames = [
            VisionFrame(
                timestamp_us=0, camera="cam", x=1.0, y=1.0, yaw_deg=0.0,
                tag_count=3, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0, capture_latency_ms=5.0,
                tags=[tags_close, tags_mid, tags_far], pose_residual_m=0.15,
            ),
        ]
        bands = _distance_band_table(frames, "cam")
        assert len(bands) == 3
        labels = [b.band_label for b in bands]
        assert "0–1m" in labels
        assert "2–3m" in labels
        assert "5m+" in labels


# ── Percentile helper ──────────────────────────────────────────────────


class TestPercentile:
    def test_p50(self) -> None:
        assert _percentile([1.0, 2.0, 3.0, 4.0, 5.0], 0.5) == 3.0

    def test_p95(self) -> None:
        data = list(range(1, 101))
        p95 = _percentile([float(x) for x in data], 0.95)
        assert 95.0 <= p95 <= 96.0

    def test_empty(self) -> None:
        assert _percentile([], 0.5) == 0.0


# ── Nearest-timestamp search ──────────────────────────────────────────


class TestFindNearest:
    def test_exact_match(self) -> None:
        values = [TimestampedValue(timestamp_us=i * 1000, value=float(i)) for i in range(10)]
        result = _find_nearest(values, 5000)
        assert result is not None
        assert result.value == 5.0

    def test_nearest_before(self) -> None:
        values = [TimestampedValue(timestamp_us=i * 1000, value=float(i)) for i in range(10)]
        result = _find_nearest(values, 5400)
        assert result is not None
        assert result.value == 5.0

    def test_beyond_max_delta(self) -> None:
        values = [TimestampedValue(timestamp_us=0, value=0.0)]
        result = _find_nearest(values, 200_000, max_delta_us=100_000)
        assert result is None

    def test_empty(self) -> None:
        assert _find_nearest([], 1000) is None


# ── Full analyzer run (no cameras) ────────────────────────────────────


class TestFullRun:
    def test_no_cameras(self) -> None:
        log_data = _make_log({})
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data)
        assert "No Limelight cameras detected" in result.summary

    def test_single_camera_basic(self) -> None:
        # Build a camera with some valid and some invalid frames
        botpose_data = []
        hb_data = []
        for i in range(50):
            ts = i * 20_000
            hb_data.append((ts, float(i)))
            if i < 20:
                # Valid frames
                botpose_data.append(
                    (ts, _make_botpose(1.0 + i * 0.05, 2.0, 0.0, 22.0, 2))
                )
            else:
                # Invalid frames (no tags)
                botpose_data.append(
                    (ts, _make_botpose(0.0, 0.0, 0.0, 22.0, 0))
                )

        # Add odometry for residual computation
        odom_data = [
            (i * 20_000, [1.0 + i * 0.05, 2.0, 0.0])
            for i in range(50)
        ]

        signals = _build_camera_signals(
            cam="a",
            botpose_data=botpose_data,
            hb_data=hb_data,
        )
        signals["NT:/Pose/robotPose"] = _make_double_array_signal(
            "NT:/Pose/robotPose", odom_data
        )

        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data)

        assert "Vision Analysis" in result.title
        assert "1 camera(s)" in result.summary
        assert len(result.extra["cameras"]) == 1

        cam = result.extra["cameras"][0]
        assert cam.camera == "limelight-a"
        assert cam.valid_frames == 20
        assert cam.total_frames == 50
        assert cam.valid_pct == 40.0

    def test_camera_filter(self) -> None:
        signals = {}
        for cam in ("a", "b"):
            signals.update(
                _build_camera_signals(
                    cam=cam,
                    botpose_data=[
                        (i * 20000, _make_botpose(1.0, 2.0, 0.0, 20.0, 1))
                        for i in range(20)
                    ],
                )
            )

        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data, cameras="limelight-b")

        assert "1 camera(s)" in result.summary
        assert result.extra["cameras"][0].camera == "limelight-b"

    def test_detail_output(self) -> None:
        botpose = [(i * 20_000, _make_botpose(1.0, 2.0, 0.0, 20.0, 2)) for i in range(20)]
        rawfid = [(i * 20_000, _make_rawfiducials(
            (7, 0.0, 0.0, 0.01, 2.0, 2.5, 0.15),
            (8, 1.0, 1.0, 0.02, 3.0, 3.5, 0.25),
        )) for i in range(20)]

        signals = _build_camera_signals(
            cam="a", botpose_data=botpose, rawfid_data=rawfid,
        )
        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data, detail=True)

        # When detail=True, per-tag and distance tables are in the summary
        assert "Per-tag detail" in result.summary
        assert "Ambiguity-distance" in result.summary


# ── Tier 2: Yaw wrapping ──────────────────────────────────────────────


class TestWrapYawDiff:
    def test_small_diff(self) -> None:
        assert abs(_wrap_yaw_diff(10.0, 15.0) - 5.0) < 0.001

    def test_wrap_around(self) -> None:
        # 170 and -170 are 20 degrees apart, not 340
        assert abs(_wrap_yaw_diff(170.0, -170.0) - 20.0) < 0.001

    def test_same(self) -> None:
        assert _wrap_yaw_diff(45.0, 45.0) == 0.0


# ── Tier 2: MT1 vs MT2 divergence ─────────────────────────────────────


class TestMT1MT2Divergence:
    def test_divergence_computed(self) -> None:
        # Create camera with botpose and botpose_orb at slightly different poses
        botpose_data = [
            (i * 20_000, _make_botpose(1.0 + i * 0.1, 2.0, 0.0, 20.0, 2))
            for i in range(10)
        ]
        orb_data = [
            (i * 20_000, _make_botpose(1.05 + i * 0.1, 2.03, 0.0, 20.0, 2))
            for i in range(10)
        ]
        signals = _build_camera_signals(cam="a", botpose_data=botpose_data)
        signals["NT:/limelight-a/botpose_orb_wpiblue"] = _make_double_array_signal(
            "NT:/limelight-a/botpose_orb_wpiblue", orb_data
        )

        log_data = _make_log(signals)
        cameras = _discover_cameras(log_data)
        prefix = "NT:/limelight-a/"
        camera_signals: dict[str, "SignalData | None"] = {
            "botpose": signals[prefix + "botpose_wpiblue"],
            "rawfiducials": None,
            "tl": None,
            "cl": None,
            "stddevs": None,
            "hb": None,
        }
        frames, _ = _parse_frames("limelight-a", camera_signals)
        assert len(frames) == 10

        _compute_mt1_mt2_divergence(frames, cameras)

        # All frames should have divergence computed
        with_div = [f for f in frames if f.mt1_mt2_divergence_m is not None]
        assert len(with_div) == 10

        # Divergence should be sqrt(0.05^2 + 0.03^2) ≈ 0.058
        for f in with_div:
            assert 0.04 < f.mt1_mt2_divergence_m < 0.08

    def test_no_orb_data(self) -> None:
        botpose_data = [
            (i * 20_000, _make_botpose(1.0, 2.0, 0.0, 20.0, 1))
            for i in range(5)
        ]
        signals = _build_camera_signals(cam="a", botpose_data=botpose_data)
        log_data = _make_log(signals)
        cameras = _discover_cameras(log_data)
        camera_signals: dict[str, "SignalData | None"] = {
            "botpose": signals["NT:/limelight-a/botpose_wpiblue"],
            "rawfiducials": None, "tl": None, "cl": None,
            "stddevs": None, "hb": None,
        }
        frames, _ = _parse_frames("limelight-a", camera_signals)
        _compute_mt1_mt2_divergence(frames, cameras)
        # No ORB data → no divergence
        assert all(f.mt1_mt2_divergence_m is None for f in frames)


class TestSummarizeMT1MT2:
    def test_summary_stats(self) -> None:
        frames = [
            VisionFrame(
                timestamp_us=i * 20_000, camera="cam-a", x=1.0, y=2.0,
                yaw_deg=0.0, tag_count=1, tag_span=0.0, avg_tag_dist=2.0,
                avg_tag_area=0.01, total_latency_ms=20.0,
                pipeline_latency_ms=15.0, capture_latency_ms=5.0,
                mt1_mt2_divergence_m=0.1 * (i + 1),
            )
            for i in range(5)
        ]
        result = _summarize_mt1_mt2(frames, ["cam-a"])
        assert "cam-a" in result
        assert result["cam-a"]["count"] == 5.0
        assert abs(result["cam-a"]["mean"] - 0.3) < 0.001
        assert abs(result["cam-a"]["median"] - 0.3) < 0.001

    def test_no_data_returns_empty(self) -> None:
        frames = [
            VisionFrame(
                timestamp_us=0, camera="cam-a", x=1.0, y=2.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0,
                capture_latency_ms=5.0,
            )
        ]
        result = _summarize_mt1_mt2(frames, ["cam-a"])
        assert result == {}


# ── Tier 2: Camera-to-camera agreement ────────────────────────────────


class TestCameraAgreement:
    def test_two_cameras_overlap(self) -> None:
        frames = []
        # Camera A: frames at 0, 20, 40, 60, 80 ms
        for i in range(5):
            frames.append(VisionFrame(
                timestamp_us=i * 20_000, camera="cam-a",
                x=1.0 + i * 0.1, y=2.0, yaw_deg=10.0,
                tag_count=2, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0,
                capture_latency_ms=5.0, pose_residual_m=0.1,
            ))
        # Camera B: frames at 5, 25, 45, 65, 85 ms (offset by 5ms)
        for i in range(5):
            frames.append(VisionFrame(
                timestamp_us=i * 20_000 + 5_000, camera="cam-b",
                x=1.05 + i * 0.1, y=2.02, yaw_deg=12.0,
                tag_count=2, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0,
                capture_latency_ms=5.0, pose_residual_m=0.15,
            ))

        agreements = _compute_camera_agreement(frames, ["cam-a", "cam-b"], 1.0)
        assert len(agreements) == 1
        ca = agreements[0]
        assert ca.overlap_frames == 5  # all frames should pair
        assert ca.mean_disagreement_m > 0
        assert ca.mean_yaw_disagreement_deg > 0
        # A has lower residual (0.1 vs 0.15)
        assert ca.a_better_count == 5

    def test_single_camera_no_agreement(self) -> None:
        frames = [
            VisionFrame(
                timestamp_us=0, camera="cam-a", x=1.0, y=2.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0,
                capture_latency_ms=5.0,
            )
        ]
        assert _compute_camera_agreement(frames, ["cam-a"], 1.0) == []

    def test_no_overlap(self) -> None:
        # Cameras at very different times (>50ms apart)
        frames = [
            VisionFrame(
                timestamp_us=0, camera="cam-a", x=1.0, y=2.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0,
                capture_latency_ms=5.0,
            ),
            VisionFrame(
                timestamp_us=200_000, camera="cam-b", x=1.0, y=2.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0,
                capture_latency_ms=5.0,
            ),
        ]
        assert _compute_camera_agreement(frames, ["cam-a", "cam-b"], 1.0) == []


# ── Tier 2: Full run with Tier 2 outputs ──────────────────────────────


class TestFullRunTier2:
    def test_mt1_mt2_in_output(self) -> None:
        botpose = [
            (i * 20_000, _make_botpose(1.0 + i * 0.05, 2.0, 0.0, 20.0, 2))
            for i in range(20)
        ]
        orb = [
            (i * 20_000, _make_botpose(1.05 + i * 0.05, 2.03, 0.0, 20.0, 2))
            for i in range(20)
        ]
        signals = _build_camera_signals(cam="a", botpose_data=botpose)
        signals["NT:/limelight-a/botpose_orb_wpiblue"] = _make_double_array_signal(
            "NT:/limelight-a/botpose_orb_wpiblue", orb
        )
        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data)

        assert "MegaTag1 vs MegaTag2" in result.summary
        assert result.extra["mt1_mt2_summary"]
        assert "limelight-a" in result.extra["mt1_mt2_summary"]

    def test_camera_agreement_in_output(self) -> None:
        signals = {}
        for cam in ("a", "b"):
            offset = 0 if cam == "a" else 5000
            signals.update(
                _build_camera_signals(
                    cam=cam,
                    botpose_data=[
                        (i * 20000 + offset,
                         _make_botpose(1.0 + i * 0.05, 2.0, 0.0, 20.0, 2))
                        for i in range(20)
                    ],
                )
            )
        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data)

        assert "Camera-to-camera agreement" in result.summary
        assert result.extra["camera_agreements"]

    def test_heatmap_data_generated(self) -> None:
        botpose = [
            (i * 20_000, _make_botpose(3.0 + i * 0.1, 4.0, 0.0, 20.0, 2))
            for i in range(20)
        ]
        odom = [
            (i * 20_000, [3.0 + i * 0.1, 4.0, 0.0])
            for i in range(20)
        ]
        signals = _build_camera_signals(cam="a", botpose_data=botpose)
        signals["NT:/Pose/robotPose"] = _make_double_array_signal(
            "NT:/Pose/robotPose", odom
        )
        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data)

        assert "heatmap_cells" in result.extra
        cells = result.extra["heatmap_cells"]
        # Should have some cells with data
        assert len(cells) > 0
        # All cells should have reasonable coordinates
        for cell in cells:
            assert 0.0 <= cell.x_center <= 17.0
            assert 0.0 <= cell.y_center <= 9.0

    def test_phases_flag(self) -> None:
        # Without phases flag, no phase breakdowns
        botpose = [
            (i * 20_000, _make_botpose(1.0, 2.0, 0.0, 20.0, 1))
            for i in range(20)
        ]
        signals = _build_camera_signals(cam="a", botpose_data=botpose)
        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()

        result = analyzer.run(log_data, phases=False)
        assert result.extra["phase_breakdowns"] == []

        # With phases=True but no match phase signals, still works (empty)
        result = analyzer.run(log_data, phases=True)
        assert result.extra["phase_breakdowns"] == []


# ── Tier 3: Heading-conditioned coverage ───────────────────────────────


class TestHeadingCoverage:
    def test_octant_mapping(self) -> None:
        assert _heading_to_octant(0.0) == "N"
        assert _heading_to_octant(45.0) == "NE"
        assert _heading_to_octant(90.0) == "E"
        assert _heading_to_octant(135.0) == "SE"
        assert _heading_to_octant(180.0) == "S"
        assert _heading_to_octant(-90.0) == "W"  # -90 → 270
        assert _heading_to_octant(-45.0) == "NW"  # -45 → 315

    def test_coverage_counts(self) -> None:
        frames = [
            VisionFrame(
                timestamp_us=i * 20_000, camera="cam-a", x=1.0, y=2.0,
                yaw_deg=float(i * 45), tag_count=1, tag_span=0.0,
                avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0,
                capture_latency_ms=5.0,
            )
            for i in range(8)
        ]
        counts = _compute_heading_coverage(frames, "cam-a")
        # 8 frames, one in each octant (0, 45, 90, 135, 180, 225, 270, 315)
        assert counts["N"] == 1
        assert counts["NE"] == 1
        assert counts["E"] == 1
        assert counts["S"] == 1

    def test_filters_by_camera(self) -> None:
        frames = [
            VisionFrame(
                timestamp_us=0, camera="cam-a", x=1.0, y=2.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0,
                capture_latency_ms=5.0,
            ),
            VisionFrame(
                timestamp_us=0, camera="cam-b", x=1.0, y=2.0, yaw_deg=90.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0,
                capture_latency_ms=5.0,
            ),
        ]
        counts = _compute_heading_coverage(frames, "cam-a")
        assert counts["N"] == 1
        assert counts["E"] == 0  # That's cam-b


# ── Tier 3: Preferred camera map ───────────────────────────────────────


class TestPreferredCameraMap:
    def test_preferred_computed(self) -> None:
        # Create two cameras with different residuals in the same cell
        cells_a = [FieldCell(row=5, col=10, x_center=5.25, y_center=2.75,
                            total_samples=20, valid_samples=10,
                            availability=0.5, median_residual_m=0.1, mean_residual_m=0.12)]
        cells_b = [FieldCell(row=5, col=10, x_center=5.25, y_center=2.75,
                            total_samples=20, valid_samples=10,
                            availability=0.5, median_residual_m=0.3, mean_residual_m=0.35)]
        heatmaps = {"cam-a": cells_a, "cam-b": cells_b}
        result = _compute_preferred_camera_map(heatmaps)
        assert len(result) == 1
        x, y, cam, adv = result[0]
        assert cam == "cam-a"  # lower residual
        assert adv > 0  # advantage over cam-b

    def test_insufficient_samples(self) -> None:
        cells_a = [FieldCell(row=5, col=10, x_center=5.25, y_center=2.75,
                            total_samples=20, valid_samples=2,
                            availability=0.1, median_residual_m=0.1, mean_residual_m=0.12)]
        cells_b = [FieldCell(row=5, col=10, x_center=5.25, y_center=2.75,
                            total_samples=20, valid_samples=10,
                            availability=0.5, median_residual_m=0.3, mean_residual_m=0.35)]
        heatmaps = {"cam-a": cells_a, "cam-b": cells_b}
        result = _compute_preferred_camera_map(heatmaps)
        assert result == []  # cam-a has < 5 samples

    def test_single_camera(self) -> None:
        cells_a = [FieldCell(row=5, col=10, x_center=5.25, y_center=2.75,
                            total_samples=20, valid_samples=10,
                            availability=0.5, median_residual_m=0.1, mean_residual_m=0.12)]
        assert _compute_preferred_camera_map({"cam-a": cells_a}) == []


# ── Tier 3: Full run with Tier 3 outputs ──────────────────────────────


class TestFullRunTier3:
    def test_heading_coverage_in_output(self) -> None:
        botpose = [
            (i * 20_000, _make_botpose(1.0, 2.0, float(i * 45), 20.0, 1))
            for i in range(20)
        ]
        signals = _build_camera_signals(cam="a", botpose_data=botpose)
        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data)

        assert "heading_coverage" in result.extra
        hc = result.extra["heading_coverage"]
        assert "limelight-a" in hc
        assert sum(hc["limelight-a"].values()) == 20

    def test_preferred_camera_in_output(self) -> None:
        signals = {}
        for cam in ("a", "b"):
            signals.update(
                _build_camera_signals(
                    cam=cam,
                    botpose_data=[
                        (i * 20000, _make_botpose(3.0, 4.0, 0.0, 20.0, 2))
                        for i in range(20)
                    ],
                )
            )
        # Add odometry so heatmap data can be computed
        signals["NT:/Pose/robotPose"] = _make_double_array_signal(
            "NT:/Pose/robotPose",
            [(i * 20000, [3.0, 4.0, 0.0]) for i in range(50)],
        )
        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data)

        assert "preferred_camera" in result.extra

    def test_speed_data_in_output(self) -> None:
        botpose = [
            (i * 20_000, _make_botpose(1.0 + i * 0.05, 2.0, 0.0, 20.0, 2))
            for i in range(20)
        ]
        odom = [
            (i * 20_000, [1.0 + i * 0.05, 2.0, 0.0])
            for i in range(50)
        ]
        signals = _build_camera_signals(cam="a", botpose_data=botpose)
        signals["NT:/Pose/robotPose"] = _make_double_array_signal(
            "NT:/Pose/robotPose", odom
        )
        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data)

        assert "residual_vs_speed" in result.extra
        speed_data = result.extra["residual_vs_speed"]
        assert len(speed_data) > 0
        # Each entry is (speed, residual, camera)
        for speed, resid, cam in speed_data:
            assert speed >= 0
            assert resid >= 0
            assert cam == "limelight-a"

    def test_plot_paths_empty_without_flag(self) -> None:
        botpose = [
            (i * 20_000, _make_botpose(1.0, 2.0, 0.0, 20.0, 1))
            for i in range(20)
        ]
        signals = _build_camera_signals(cam="a", botpose_data=botpose)
        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data)
        assert result.extra["plot_paths"] == []


# ── Tier 4: t2d tag geometry parsing ───────────────────────────────────


class TestT2DParsing:
    def test_t2d_fields_populated(self) -> None:
        botpose = [(100_000, _make_botpose(1.0, 2.0, 0.0, 20.0, 2))]
        t2d = [(100_000, _make_t2d(
            valid=True, long_side_px=60.0, short_side_px=45.0,
            h_extent_px=50.0, v_extent_px=55.0, skew_deg=-7.0,
        ))]
        signals = _build_camera_signals(cam="a", botpose_data=botpose, t2d_data=t2d)
        prefix = "NT:/limelight-a/"
        camera_signals: dict[str, "SignalData | None"] = {
            "botpose": signals[prefix + "botpose_wpiblue"],
            "rawfiducials": None, "tl": None, "cl": None,
            "stddevs": None, "hb": None,
            "t2d": signals[prefix + "t2d"],
        }
        frames, _ = _parse_frames("limelight-a", camera_signals)
        assert len(frames) == 1
        f = frames[0]
        assert f.tag_long_side_px == 60.0
        assert f.tag_short_side_px == 45.0
        assert abs(f.tag_aspect_ratio - 60.0 / 45.0) < 0.01
        assert f.tag_skew_deg == -7.0
        assert f.tag_h_extent_px == 50.0
        assert f.tag_v_extent_px == 55.0

    def test_t2d_invalid_target_yields_zeros(self) -> None:
        botpose = [(100_000, _make_botpose(1.0, 2.0, 0.0, 20.0, 1))]
        t2d = [(100_000, _make_t2d(
            valid=False, long_side_px=99.0, short_side_px=88.0,
        ))]
        signals = _build_camera_signals(cam="a", botpose_data=botpose, t2d_data=t2d)
        prefix = "NT:/limelight-a/"
        camera_signals: dict[str, "SignalData | None"] = {
            "botpose": signals[prefix + "botpose_wpiblue"],
            "rawfiducials": None, "tl": None, "cl": None,
            "stddevs": None, "hb": None,
            "t2d": signals[prefix + "t2d"],
        }
        frames, _ = _parse_frames("limelight-a", camera_signals)
        assert len(frames) == 1
        # targetValid=0 → geometry fields should be zero
        assert frames[0].tag_long_side_px == 0.0
        assert frames[0].tag_short_side_px == 0.0
        assert frames[0].tag_aspect_ratio == 0.0

    def test_t2d_missing_signal_yields_zeros(self) -> None:
        botpose = [(100_000, _make_botpose(1.0, 2.0, 0.0, 20.0, 1))]
        signals = _build_camera_signals(cam="a", botpose_data=botpose)
        prefix = "NT:/limelight-a/"
        camera_signals: dict[str, "SignalData | None"] = {
            "botpose": signals[prefix + "botpose_wpiblue"],
            "rawfiducials": None, "tl": None, "cl": None,
            "stddevs": None, "hb": None,
            "t2d": None,
        }
        frames, _ = _parse_frames("limelight-a", camera_signals)
        assert len(frames) == 1
        assert frames[0].tag_long_side_px == 0.0
        assert frames[0].tag_aspect_ratio == 0.0


# ── Tier 4: Tag geometry summary ───────────────────────────────────────


class TestTagGeometrySummary:
    def test_summary_computed(self) -> None:
        frames = [
            VisionFrame(
                timestamp_us=i * 20_000, camera="cam-a", x=1.0, y=2.0,
                yaw_deg=0.0, tag_count=1, tag_span=0.0, avg_tag_dist=2.0,
                avg_tag_area=0.01, total_latency_ms=20.0,
                pipeline_latency_ms=15.0, capture_latency_ms=5.0,
                tag_long_side_px=60.0, tag_short_side_px=45.0,
                tag_aspect_ratio=60.0 / 45.0, tag_skew_deg=-7.0,
                tag_h_extent_px=50.0, tag_v_extent_px=55.0,
            )
            for i in range(10)
        ]
        result = _compute_tag_geometry_summary(frames, ["cam-a"])
        assert "cam-a" in result
        s = result["cam-a"]
        assert s["count"] == 10.0
        assert abs(s["median_short_side_px"] - 45.0) < 0.01
        assert abs(s["median_aspect_ratio"] - 60.0 / 45.0) < 0.01
        assert s["pct_oblique"] == 0.0  # aspect ratio < 2
        assert s["pct_tiny"] == 0.0  # short side > 20

    def test_oblique_and_tiny_detection(self) -> None:
        frames = [
            VisionFrame(
                timestamp_us=0, camera="cam-a", x=1.0, y=2.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=4.0, avg_tag_area=0.001,
                total_latency_ms=20.0, pipeline_latency_ms=15.0,
                capture_latency_ms=5.0,
                tag_long_side_px=50.0, tag_short_side_px=15.0,
                tag_aspect_ratio=50.0 / 15.0, tag_skew_deg=-30.0,
                tag_h_extent_px=20.0, tag_v_extent_px=52.0,
            ),
        ]
        result = _compute_tag_geometry_summary(frames, ["cam-a"])
        s = result["cam-a"]
        assert s["pct_oblique"] == 100.0  # 50/15 = 3.33 > 2
        assert s["pct_tiny"] == 100.0  # 15 < 20

    def test_no_t2d_data_returns_empty(self) -> None:
        frames = [
            VisionFrame(
                timestamp_us=0, camera="cam-a", x=1.0, y=2.0, yaw_deg=0.0,
                tag_count=1, tag_span=0.0, avg_tag_dist=2.0, avg_tag_area=0.01,
                total_latency_ms=20.0, pipeline_latency_ms=15.0,
                capture_latency_ms=5.0,
                # tag_short_side_px defaults to 0.0 → excluded
            ),
        ]
        result = _compute_tag_geometry_summary(frames, ["cam-a"])
        assert result == {}


# ── Tier 4: Full run with tag geometry ─────────────────────────────────


class TestFullRunTier4:
    def test_tag_geometry_in_output(self) -> None:
        botpose = [
            (i * 20_000, _make_botpose(1.0, 2.0, 0.0, 20.0, 2))
            for i in range(20)
        ]
        t2d = [
            (i * 20_000, _make_t2d(
                valid=True, long_side_px=60.0, short_side_px=45.0,
                skew_deg=-5.0,
            ))
            for i in range(20)
        ]
        signals = _build_camera_signals(cam="a", botpose_data=botpose, t2d_data=t2d)
        log_data = _make_log(signals)
        analyzer = VisionAnalyzer()
        result = analyzer.run(log_data)

        assert "tag_geometry" in result.extra
        tg = result.extra["tag_geometry"]
        assert "limelight-a" in tg
        assert tg["limelight-a"]["count"] == 20.0
        assert "Tag geometry quality" in result.summary
