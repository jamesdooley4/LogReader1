"""Tests for the hard-hits analyzer."""

from __future__ import annotations

import math

from logreader.analyzers.hard_hits import (
    Classification,
    DEFAULT_IMPACT_HARD_G,
    DEFAULT_IMPACT_LIGHT_G,
    DEFAULT_IMPACT_MODERATE_G,
    DetectionMode,
    HardHitEvent,
    HardHitAnalyzer,
    Severity,
    _group_events,
    _severity_from_angular,
    _severity_from_impact,
    detect_licensed,
    detect_unlicensed,
)
from logreader.models import (
    LogData,
    LogMetadata,
    SignalData,
    SignalInfo,
    SignalType,
    TimestampedValue,
)


# ---------------------------------------------------------------------------
# Helpers to build synthetic signals
# ---------------------------------------------------------------------------


def _make_signal(
    name: str,
    values: list[tuple[int, float]],
) -> SignalData:
    """Create a SignalData with the given (timestamp_us, value) pairs."""
    info = SignalInfo(entry_id=0, name=name, type=SignalType.DOUBLE)
    return SignalData(
        info=info,
        values=[TimestampedValue(timestamp_us=ts, value=val) for ts, val in values],
    )


def _make_constant_signal(
    name: str,
    value: float,
    start_us: int = 0,
    count: int = 100,
    period_us: int = 4000,
) -> SignalData:
    """Create a constant-value signal with uniform timestamps."""
    return _make_signal(
        name,
        [(start_us + i * period_us, value) for i in range(count)],
    )


def _make_log_data(signals: dict[str, SignalData]) -> LogData:
    """Wrap signals in a LogData."""
    return LogData(
        metadata=LogMetadata(file_path="test.wpilog", is_valid=True),
        signals=signals,
    )


def _pigeon_name(suffix: str) -> str:
    return f"Phoenix6/Pigeon2-10/{suffix}"


# ---------------------------------------------------------------------------
# Severity classification unit tests
# ---------------------------------------------------------------------------


class TestSeverityFromImpact:
    def test_hard(self) -> None:
        assert _severity_from_impact(2.0) == Severity.HARD

    def test_moderate(self) -> None:
        assert _severity_from_impact(1.2) == Severity.MODERATE

    def test_light(self) -> None:
        assert _severity_from_impact(0.6) == Severity.LIGHT

    def test_below_threshold(self) -> None:
        assert _severity_from_impact(0.3) is None

    def test_exact_boundary_hard(self) -> None:
        assert _severity_from_impact(DEFAULT_IMPACT_HARD_G) == Severity.HARD

    def test_exact_boundary_moderate(self) -> None:
        assert _severity_from_impact(DEFAULT_IMPACT_MODERATE_G) == Severity.MODERATE

    def test_exact_boundary_light(self) -> None:
        assert _severity_from_impact(DEFAULT_IMPACT_LIGHT_G) == Severity.LIGHT

    def test_custom_thresholds(self) -> None:
        assert (
            _severity_from_impact(0.4, hard=0.5, moderate=0.3, light=0.1)
            == Severity.MODERATE
        )


class TestSeverityFromAngular:
    def test_alpha_hard(self) -> None:
        assert _severity_from_angular(5000.0, 0.0) == Severity.HARD

    def test_omega_hard(self) -> None:
        assert _severity_from_angular(0.0, 200.0) == Severity.HARD

    def test_alpha_moderate_omega_light(self) -> None:
        # alpha is moderate, omega is light → should return MODERATE (higher)
        assert _severity_from_angular(3000.0, 60.0) == Severity.MODERATE

    def test_both_below(self) -> None:
        assert _severity_from_angular(500.0, 20.0) is None

    def test_either_triggers(self) -> None:
        # alpha below, omega moderate → MODERATE
        assert _severity_from_angular(100.0, 120.0) == Severity.MODERATE


# ---------------------------------------------------------------------------
# Event grouping tests
# ---------------------------------------------------------------------------


class TestGroupEvents:
    def _ev(self, ts_us: int, score: float) -> HardHitEvent:
        return HardHitEvent(
            timestamp_us=ts_us,
            severity=Severity.HARD,
            score=score,
            mode=DetectionMode.LICENSED_ACCEL,
            classification=Classification.COLLISION_HEAD_ON,
        )

    def test_empty(self) -> None:
        assert _group_events([], 300_000) == []

    def test_single(self) -> None:
        evts = [self._ev(1_000_000, 2.0)]
        result = _group_events(evts, 300_000)
        assert len(result) == 1

    def test_two_far_apart(self) -> None:
        evts = [self._ev(1_000_000, 1.0), self._ev(5_000_000, 2.0)]
        result = _group_events(evts, 300_000)
        assert len(result) == 2

    def test_two_close_keeps_higher(self) -> None:
        evts = [self._ev(1_000_000, 1.0), self._ev(1_200_000, 2.5)]
        result = _group_events(evts, 300_000)
        assert len(result) == 1
        assert result[0].score == 2.5

    def test_three_events_two_groups(self) -> None:
        evts = [
            self._ev(1_000_000, 1.0),
            self._ev(1_100_000, 3.0),
            self._ev(5_000_000, 0.5),
        ]
        result = _group_events(evts, 300_000)
        assert len(result) == 2
        assert result[0].score == 3.0
        assert result[1].score == 0.5


# ---------------------------------------------------------------------------
# Licensed detection tests
# ---------------------------------------------------------------------------


class TestDetectLicensed:
    def test_no_data(self) -> None:
        empty = _make_signal(_pigeon_name("AccelerationX"), [])
        result = detect_licensed(empty, empty, empty, None, None, None, None)
        assert result == []

    def test_quiet_robot_no_events(self) -> None:
        """At rest (az ≈ -1g, ax=ay=0), |a|=1g, impact=0 → no events."""
        n = 200
        period = 4000  # 4ms = 250Hz
        ax = _make_constant_signal(
            _pigeon_name("AccelerationX"), 0.0, count=n, period_us=period
        )
        ay = _make_constant_signal(
            _pigeon_name("AccelerationY"), 0.0, count=n, period_us=period
        )
        az = _make_constant_signal(
            _pigeon_name("AccelerationZ"), -1.0, count=n, period_us=period
        )
        result = detect_licensed(ax, ay, az, None, None, None, None)
        assert result == []

    def test_lateral_collision_detected(self) -> None:
        """Sudden lateral spike → collision (not landing)."""
        n = 200
        period = 4000
        # Normal at rest, then a spike at sample 100
        ax_vals = [(i * period, 0.0) for i in range(n)]
        ay_vals = [(i * period, 0.0) for i in range(n)]
        az_vals = [(i * period, -1.0) for i in range(n)]
        # Insert a lateral hit at sample 100: ax=2.0, ay=0, az=-1 → |a|=√5≈2.24, impact≈1.24
        spike_ts = 100 * period
        ax_vals[100] = (spike_ts, 2.0)

        ax = _make_signal(_pigeon_name("AccelerationX"), ax_vals)
        ay = _make_signal(_pigeon_name("AccelerationY"), ay_vals)
        az = _make_signal(_pigeon_name("AccelerationZ"), az_vals)

        events = detect_licensed(ax, ay, az, None, None, None, None)
        assert len(events) >= 1
        hit = events[0]
        assert hit.severity == Severity.MODERATE
        assert hit.classification in (
            Classification.COLLISION_HEAD_ON,
            Classification.COLLISION_SPINNING,
        )
        assert hit.freefall_detected is False
        assert hit.lateral_fraction is not None
        assert hit.lateral_fraction > 0.5  # lateral-dominant

    def test_landing_detected_with_freefall(self) -> None:
        """Freefall (|a|→0) followed by Z-dominant spike → landing."""
        n = 200
        period = 4000
        # Normal at rest
        ax_vals = [(i * period, 0.0) for i in range(n)]
        ay_vals = [(i * period, 0.0) for i in range(n)]
        az_vals = [(i * period, -1.0) for i in range(n)]

        # Freefall at samples 90-98 (|a| → 0)
        for j in range(90, 99):
            ax_vals[j] = (j * period, 0.0)
            ay_vals[j] = (j * period, 0.0)
            az_vals[j] = (j * period, 0.0)  # freefall: az goes to 0

        # Landing impact at sample 100: Z-dominant spike
        # ax=0.2, ay=0.1, az=-2.5 → |a|≈2.51, lat_frac=√(0.04+0.01)/2.51≈0.09
        ax_vals[100] = (100 * period, 0.2)
        ay_vals[100] = (100 * period, 0.1)
        az_vals[100] = (100 * period, -2.5)

        ax = _make_signal(_pigeon_name("AccelerationX"), ax_vals)
        ay = _make_signal(_pigeon_name("AccelerationY"), ay_vals)
        az = _make_signal(_pigeon_name("AccelerationZ"), az_vals)

        events = detect_licensed(ax, ay, az, None, None, None, None)
        assert len(events) >= 1
        landing = events[0]
        assert landing.classification == Classification.LANDING
        assert landing.freefall_detected is True
        assert landing.lateral_fraction is not None
        assert landing.lateral_fraction < 0.5

    def test_hard_severity_threshold(self) -> None:
        """A very large spike should be classified as HARD."""
        n = 100
        period = 4000
        ax_vals = [(i * period, 0.0) for i in range(n)]
        ay_vals = [(i * period, 0.0) for i in range(n)]
        az_vals = [(i * period, -1.0) for i in range(n)]
        # Massive lateral hit: ax=2.5, ay=1.5 → |a|=√(6.25+2.25+1)≈3.08, impact≈2.08
        ax_vals[50] = (50 * period, 2.5)
        ay_vals[50] = (50 * period, 1.5)

        ax = _make_signal(_pigeon_name("AccelerationX"), ax_vals)
        ay = _make_signal(_pigeon_name("AccelerationY"), ay_vals)
        az = _make_signal(_pigeon_name("AccelerationZ"), az_vals)

        events = detect_licensed(ax, ay, az, None, None, None, None)
        hard_events = [e for e in events if e.severity == Severity.HARD]
        assert len(hard_events) >= 1

    def test_spinning_collision_classified(self) -> None:
        """A lateral spike with large ωz → collision-spinning."""
        n = 100
        period = 4000
        ax_vals = [(i * period, 0.0) for i in range(n)]
        ay_vals = [(i * period, 0.0) for i in range(n)]
        az_vals = [(i * period, -1.0) for i in range(n)]
        oz_vals = [(i * period, 0.0) for i in range(n)]

        spike_ts = 50 * period
        ax_vals[50] = (spike_ts, 2.0)
        oz_vals[50] = (spike_ts, 150.0)  # large angular velocity

        ax = _make_signal(_pigeon_name("AccelerationX"), ax_vals)
        ay = _make_signal(_pigeon_name("AccelerationY"), ay_vals)
        az = _make_signal(_pigeon_name("AccelerationZ"), az_vals)
        oz = _make_signal(_pigeon_name("AngularVelocityZWorld"), oz_vals)

        events = detect_licensed(ax, ay, az, None, None, oz, None)
        assert len(events) >= 1
        assert events[0].classification == Classification.COLLISION_SPINNING


# ---------------------------------------------------------------------------
# Unlicensed detection tests
# ---------------------------------------------------------------------------


class TestDetectUnlicensed:
    def test_no_data(self) -> None:
        empty = _make_signal(_pigeon_name("AngularVelocityZWorld"), [])
        assert detect_unlicensed(empty) == []

    def test_quiet_no_events(self) -> None:
        """Constant angular velocity → no spikes."""
        sig = _make_constant_signal(
            _pigeon_name("AngularVelocityZWorld"), 5.0, count=500, period_us=4000
        )
        events = detect_unlicensed(sig)
        assert events == []

    def test_spike_detected(self) -> None:
        """A sudden angular velocity step should be detected."""
        n = 500
        period = 4000
        vals: list[tuple[int, float]] = []
        for i in range(n):
            ts = i * period
            if i < 200:
                vals.append((ts, 0.0))
            elif i == 200:
                vals.append((ts, 200.0))  # sudden jump
            else:
                vals.append((ts, 200.0))

        sig = _make_signal(_pigeon_name("AngularVelocityZWorld"), vals)
        events = detect_unlicensed(sig)
        assert len(events) >= 1
        assert events[0].severity == Severity.HARD
        assert events[0].classification == Classification.ROTATIONAL_ONLY
        assert events[0].mode == DetectionMode.UNLICENSED_ROTATIONAL

    def test_moderate_step(self) -> None:
        """A moderate step → MODERATE or LIGHT."""
        n = 500
        period = 4000
        vals: list[tuple[int, float]] = []
        for i in range(n):
            ts = i * period
            if i < 200:
                vals.append((ts, 0.0))
            elif i == 200:
                vals.append((ts, 60.0))  # moderate jump (60 deg/s in one sample)
            else:
                vals.append((ts, 60.0))

        sig = _make_signal(_pigeon_name("AngularVelocityZWorld"), vals)
        events = detect_unlicensed(sig)
        assert len(events) >= 1
        # Should be at least LIGHT (>50 deg/s step)
        assert events[0].severity in (Severity.LIGHT, Severity.MODERATE, Severity.HARD)


# ---------------------------------------------------------------------------
# Full analyzer integration tests
# ---------------------------------------------------------------------------


class TestHardHitAnalyzer:
    def test_registered(self) -> None:
        from logreader.analyzers import list_analyzers

        assert "hard-hits" in list_analyzers()

    def test_no_signals_returns_message(self) -> None:
        log_data = _make_log_data({})
        analyzer = HardHitAnalyzer()
        result = analyzer.run(log_data)
        assert "No Pigeon 2 IMU signals found" in result.summary

    def test_licensed_mode_produces_result(self) -> None:
        """Full licensed mode with a collision spike."""
        n = 200
        period = 4000
        ax_vals = [(i * period, 0.0) for i in range(n)]
        ay_vals = [(i * period, 0.0) for i in range(n)]
        az_vals = [(i * period, -1.0) for i in range(n)]
        ax_vals[100] = (100 * period, 2.0)

        signals = {
            _pigeon_name("AccelerationX"): _make_signal(
                _pigeon_name("AccelerationX"), ax_vals
            ),
            _pigeon_name("AccelerationY"): _make_signal(
                _pigeon_name("AccelerationY"), ay_vals
            ),
            _pigeon_name("AccelerationZ"): _make_signal(
                _pigeon_name("AccelerationZ"), az_vals
            ),
        }
        log_data = _make_log_data(signals)
        analyzer = HardHitAnalyzer()
        result = analyzer.run(log_data)

        assert "licensed" in result.title.lower()
        assert result.rows  # should have at least one row
        assert result.extra["events"]

    def test_unlicensed_fallback(self) -> None:
        """When only AngularVelocityZWorld is present, use unlicensed mode."""
        n = 500
        period = 4000
        vals = [(i * period, 0.0) for i in range(n)]
        vals[200] = (200 * period, 200.0)

        signals = {
            _pigeon_name("AngularVelocityZWorld"): _make_signal(
                _pigeon_name("AngularVelocityZWorld"), vals
            ),
        }
        log_data = _make_log_data(signals)
        analyzer = HardHitAnalyzer()
        result = analyzer.run(log_data)

        assert "unlicensed" in result.title.lower()
        assert "rotational" in result.title.lower()
        assert result.rows

    def test_event_output_shape(self) -> None:
        """Verify the extra['events'] list has the expected fields."""
        n = 200
        period = 4000
        ax_vals = [(i * period, 0.0) for i in range(n)]
        ay_vals = [(i * period, 0.0) for i in range(n)]
        az_vals = [(i * period, -1.0) for i in range(n)]
        ax_vals[100] = (100 * period, 2.0)

        signals = {
            _pigeon_name("AccelerationX"): _make_signal(
                _pigeon_name("AccelerationX"), ax_vals
            ),
            _pigeon_name("AccelerationY"): _make_signal(
                _pigeon_name("AccelerationY"), ay_vals
            ),
            _pigeon_name("AccelerationZ"): _make_signal(
                _pigeon_name("AccelerationZ"), az_vals
            ),
        }
        log_data = _make_log_data(signals)
        result = HardHitAnalyzer().run(log_data)
        ev = result.extra["events"][0]

        assert isinstance(ev, HardHitEvent)
        assert ev.timestamp_us > 0
        assert ev.severity in Severity
        assert ev.mode == DetectionMode.LICENSED_ACCEL
        assert ev.classification in Classification
        assert ev.impact_g is not None
        assert ev.ax_g is not None
        assert ev.lateral_fraction is not None
        assert ev.freefall_detected is not None


# ---------------------------------------------------------------------------
# Match-phases integration tests (hoot RobotMode signal)
# ---------------------------------------------------------------------------


def _make_string_signal(
    name: str,
    values: list[tuple[int, str]],
) -> SignalData:
    """Create a string SignalData."""
    info = SignalInfo(entry_id=0, name=name, type=SignalType.STRING)
    return SignalData(
        info=info,
        values=[TimestampedValue(timestamp_us=ts, value=val) for ts, val in values],
    )


class TestMatchPhaseHootSupport:
    """Test that match_phases detects phases from hoot RobotMode signal."""

    def test_robot_mode_detects_phases(self) -> None:
        from logreader.analyzers.match_phases import detect_match_phases, MatchPhase

        signals = {
            "RobotMode": _make_string_signal(
                "RobotMode",
                [
                    (0, "Disabled"),
                    (100_000_000, "Autonomous"),
                    (115_000_000, "Disabled"),
                    (118_000_000, "Teleop"),
                    (253_000_000, "Disabled"),
                ],
            ),
        }
        log_data = _make_log_data(signals)
        timeline = detect_match_phases(log_data)

        assert timeline is not None
        phases = [iv.phase for iv in timeline.intervals]
        assert MatchPhase.AUTONOMOUS in phases
        assert MatchPhase.TELEOP in phases
        assert MatchPhase.DISABLED in phases

    def test_robot_mode_no_signal_returns_none(self) -> None:
        from logreader.analyzers.match_phases import detect_match_phases

        log_data = _make_log_data({})
        assert detect_match_phases(log_data) is None


class TestHardHitPhaseAnnotation:
    """Test that hard-hit events get annotated with match phase."""

    def test_licensed_events_have_phase(self) -> None:
        n = 200
        period = 4000
        ax_vals = [(i * period, 0.0) for i in range(n)]
        ay_vals = [(i * period, 0.0) for i in range(n)]
        az_vals = [(i * period, -1.0) for i in range(n)]
        # Hit at sample 100 = 400ms
        ax_vals[100] = (100 * period, 2.0)

        signals = {
            _pigeon_name("AccelerationX"): _make_signal(
                _pigeon_name("AccelerationX"), ax_vals
            ),
            _pigeon_name("AccelerationY"): _make_signal(
                _pigeon_name("AccelerationY"), ay_vals
            ),
            _pigeon_name("AccelerationZ"): _make_signal(
                _pigeon_name("AccelerationZ"), az_vals
            ),
            # RobotMode: Teleop for the whole range
            "RobotMode": _make_string_signal(
                "RobotMode",
                [
                    (0, "Teleop"),
                ],
            ),
        }
        log_data = _make_log_data(signals)
        result = HardHitAnalyzer().run(log_data)
        ev = result.extra["events"][0]

        assert ev.phase == "teleop"

    def test_licensed_events_during_auto(self) -> None:
        n = 200
        period = 4000
        ax_vals = [(i * period, 0.0) for i in range(n)]
        ay_vals = [(i * period, 0.0) for i in range(n)]
        az_vals = [(i * period, -1.0) for i in range(n)]
        ax_vals[50] = (50 * period, 2.0)  # Hit at 200ms (during auto)

        signals = {
            _pigeon_name("AccelerationX"): _make_signal(
                _pigeon_name("AccelerationX"), ax_vals
            ),
            _pigeon_name("AccelerationY"): _make_signal(
                _pigeon_name("AccelerationY"), ay_vals
            ),
            _pigeon_name("AccelerationZ"): _make_signal(
                _pigeon_name("AccelerationZ"), az_vals
            ),
            "RobotMode": _make_string_signal(
                "RobotMode",
                [
                    (0, "Autonomous"),
                    (300_000, "Teleop"),  # Switch at 300ms
                ],
            ),
        }
        log_data = _make_log_data(signals)
        result = HardHitAnalyzer().run(log_data)
        ev = result.extra["events"][0]

        assert ev.phase == "auto"

    def test_no_phase_data_leaves_none(self) -> None:
        n = 200
        period = 4000
        ax_vals = [(i * period, 0.0) for i in range(n)]
        ay_vals = [(i * period, 0.0) for i in range(n)]
        az_vals = [(i * period, -1.0) for i in range(n)]
        ax_vals[100] = (100 * period, 2.0)

        signals = {
            _pigeon_name("AccelerationX"): _make_signal(
                _pigeon_name("AccelerationX"), ax_vals
            ),
            _pigeon_name("AccelerationY"): _make_signal(
                _pigeon_name("AccelerationY"), ay_vals
            ),
            _pigeon_name("AccelerationZ"): _make_signal(
                _pigeon_name("AccelerationZ"), az_vals
            ),
        }
        log_data = _make_log_data(signals)
        result = HardHitAnalyzer().run(log_data)
        ev = result.extra["events"][0]

        assert ev.phase is None

    def test_phase_column_in_output(self) -> None:
        n = 200
        period = 4000
        ax_vals = [(i * period, 0.0) for i in range(n)]
        ay_vals = [(i * period, 0.0) for i in range(n)]
        az_vals = [(i * period, -1.0) for i in range(n)]
        ax_vals[100] = (100 * period, 2.0)

        signals = {
            _pigeon_name("AccelerationX"): _make_signal(
                _pigeon_name("AccelerationX"), ax_vals
            ),
            _pigeon_name("AccelerationY"): _make_signal(
                _pigeon_name("AccelerationY"), ay_vals
            ),
            _pigeon_name("AccelerationZ"): _make_signal(
                _pigeon_name("AccelerationZ"), az_vals
            ),
            "RobotMode": _make_string_signal(
                "RobotMode",
                [(0, "Teleop")],
            ),
        }
        log_data = _make_log_data(signals)
        result = HardHitAnalyzer().run(log_data)

        assert "Phase" in result.columns
        assert result.rows[0]["Phase"] == "teleop"
