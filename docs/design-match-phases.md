# Match-Phase Detection — Design Document

> Analyzer: `match-phases` · Module: `src/logreader/analyzers/match_phases.py`

**Status:** Implemented  
**Role:** Standalone analyzer + shared phase-timeline service for other analyzers  
**Validated against:** Synthetic unit tests plus real logs where DS booleans may be absent and `FMSControlData` must be used as fallback

Detect autonomous / teleop / disabled phase boundaries from mode signals. Provide per-phase breakdowns for other metrics. Other analyzers can optionally use phase boundaries.

---

## Overview

Match-phase detection is both a **standalone analyzer** (`match-phases`) and a **shared service** that other analyzers can import to partition their results by game phase. The design has two layers:

1. **`match_phases.py`** — a module in `src/logreader/analyzers/` that exports reusable detection functions and data models.
2. **`MatchPhasesAnalyzer`** — the `@register_analyzer` CLI analyzer that uses layer 1 and presents a human-readable report.

Other analyzers (e.g. `launch-counter`, `pdh-power`, `battery-health`) import the detection functions directly — they never need to instantiate `MatchPhasesAnalyzer`.

---

## Data Observations

WPILib logs the Driver Station mode as boolean signals. The standard signal names are:

| Signal name | Meaning |
|-------------|---------|
| `DS:enabled` or `DSEnabled` | Robot is enabled (any mode) |
| `DS:autonomous` or `DSAuto` | Autonomous mode active |
| `DS:teleop` or `DSTeleop` | Teleoperated mode active |
| `DS:test` or `DSTest` | Test mode active |
| `DS:disabled` or `DSDisabled` | Robot disabled |
| `DS:estop` or `DSEStop` | Emergency-stopped |

However, some teams' robot code does **not** log these boolean DS signals. A more universally available fallback is `NT:/FMSInfo/FMSControlData`, an integer signal whose value is a bitflag encoding of `HAL_ControlWord`:

| Bit | Mask | Meaning |
|-----|------|---------|
| 0   | 0x01 | Enabled |
| 1   | 0x02 | Autonomous |
| 2   | 0x04 | Test |
| 3   | 0x08 | Emergency stop |
| 4   | 0x10 | FMS attached |
| 5   | 0x20 | DS attached |

Source: [WPILib NTFMS.cpp](https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/glass/src/libnt/native/cpp/NTFMS.cpp#L63)

**Common values observed in real match logs:**

| Value | Binary | Meaning |
|-------|--------|---------|
| 0     | `000000` | Disabled, no FMS/DS connection |
| 48    | `110000` | Disabled + FMS + DS (post-match) |
| 49    | `110001` | Enabled + FMS + DS (teleop) |
| 50    | `110010` | Disabled + auto bit + FMS + DS (auto→teleop gap) |
| 51    | `110011` | Enabled + auto + FMS + DS (autonomous) |

`FMSControlData` updates only on state changes (typically 4–6 samples per match), not every loop cycle, so it is very sparse but sufficient for phase boundary detection.

Hoot-converted logs may use a different prefix (e.g. CTRE-specific names). The detector must auto-detect from multiple naming conventions.

A typical FRC match has this phase sequence:

```
[Pre-match disabled] → Auto (15 s) → [Brief disabled gap] → Teleop (135 s) → [Post-match disabled]
```

But real logs may contain:
- Practice sessions with no FMS (arbitrary enable/disable sequences)
- Multiple enable cycles during pit testing
- Very short disabled gaps between auto and teleop (< 1 s)
- E-stop events cutting a phase short
- Logs that start mid-match (late recording start)
- Mid-match reboots (code crash or brownout power cycle) — see [Reboot Handling](#reboot-handling) below

---

## Phase Model

```python
class MatchPhase(Enum):
    """The mode the robot is operating in."""
    DISABLED = "disabled"
    AUTONOMOUS = "auto"
    TELEOP = "teleop"
    TEST = "test"

@dataclass
class PhaseInterval:
    """A single contiguous interval where the robot is in one phase.

    Attributes:
        phase: Which mode the robot is in.
        start_us: Start timestamp in microseconds (inclusive).
        end_us: End timestamp in microseconds (exclusive).
    """
    phase: MatchPhase
    start_us: int
    end_us: int

    @property
    def start_s(self) -> float:
        return self.start_us / 1_000_000.0

    @property
    def end_s(self) -> float:
        return self.end_us / 1_000_000.0

    @property
    def duration_s(self) -> float:
        return (self.end_us - self.start_us) / 1_000_000.0

    def contains_us(self, timestamp_us: int) -> bool:
        return self.start_us <= timestamp_us < self.end_us

    def overlaps(self, start_us: int, end_us: int) -> bool:
        return self.start_us <= end_us and self.end_us >= start_us

@dataclass
class MatchPhaseTimeline:
    """Complete phase timeline for a log file.

    The intervals list is sorted by start time and covers the full log
    duration with no gaps (disabled fills the space between active phases).
    """
    intervals: list[PhaseInterval]

    @property
    def has_match(self) -> bool:
        """True if this looks like a real match (has both auto and teleop)."""
        phases = {i.phase for i in self.intervals}
        return MatchPhase.AUTONOMOUS in phases and MatchPhase.TELEOP in phases

    @property
    def appears_truncated(self) -> bool:
        """True if enabled time is much shorter than a full match."""

    @property
    def appears_post_reboot(self) -> bool:
        """True if the file looks like a teleop-only post-restart segment."""

    def phase_at(self, timestamp_us: int) -> MatchPhase:
        """Return the phase active at a given timestamp."""
        for iv in self.intervals:
            if iv.contains_us(timestamp_us):
                return iv.phase
        return MatchPhase.DISABLED

    def intervals_for(self, phase: MatchPhase) -> list[PhaseInterval]:
        """Return all intervals matching the given phase."""
        return [iv for iv in self.intervals if iv.phase == phase]

    def auto_interval(self) -> PhaseInterval | None:
        """The first (usually only) autonomous interval, or None."""
        autos = self.intervals_for(MatchPhase.AUTONOMOUS)
        return autos[0] if autos else None

    def teleop_interval(self) -> PhaseInterval | None:
        """The first (usually only) teleop interval, or None."""
        teleops = self.intervals_for(MatchPhase.TELEOP)
        return teleops[0] if teleops else None
```

### Interval semantics

`PhaseInterval` uses **half-open** ranges: `[start_us, end_us)`.

That means:

- `start_us` is included in the interval
- `end_us` is the first timestamp *after* the interval
- adjacent intervals can share a boundary without overlap ambiguity

This is important for helpers like `phase_at()` and for other analyzers that slice signals exactly at phase transitions.

### Timeline helper properties

The implemented timeline model exposes a few lightweight interpretation helpers:

- `has_match` — both autonomous and teleop are present
- `appears_truncated` — the file looks like an incomplete match segment, often due to a crash or restart
- `appears_post_reboot` — the file starts disabled, then enters teleop without any autonomous phase, consistent with a post-restart continuation

These helpers are intentionally heuristic. They are meant to improve reporting and severity decisions in downstream analyzers, not to serve as strict proof of a reboot.

---

## Signal Auto-Detection

The detector uses a two-tier strategy:

**Tier 1 — Boolean DS signals** (preferred, if present):

```python
_MODE_PATTERNS: dict[str, list[re.Pattern]] = {
    "autonomous": [re.compile(r".*(DS[:/]?auto|DSAutonomous|FMSInfo.*Auto).*", re.I)],
    "teleop":     [re.compile(r".*(DS[:/]?teleop|FMSInfo.*Teleop).*", re.I)],
    "test":       [re.compile(r".*(DS[:/]?test|FMSInfo.*Test).*", re.I)],
    "estop":      [re.compile(r".*(DS[:/]?e-?stop|DSEStop).*", re.I)],
}
```

Search all boolean signals for matches. The detector works with any subset — e.g. if only `DSAuto` and `DSTeleop` are present, it infers disabled as "neither auto nor teleop".

**Tier 2 — `FMSControlData` bitflag fallback** (when no boolean DS signals exist):

If no boolean mode signals are found, the detector looks for `NT:/FMSInfo/FMSControlData` (an integer signal). Each sample is a `HAL_ControlWord` bitflag that encodes all mode information in a single value. The detector decomposes each sample into the same `(timestamp, mode_key, bool)` transition tuples that the boolean path produces, so the downstream timeline-building algorithm is unchanged.

```python
_FMS_CONTROL_DATA_PATTERNS = [re.compile(r".*FMSInfo/FMSControlData$", re.I)]
```

This fallback is critical because most team robot code publishes `FMSControlData` by default (via WPILib's `DriverStation` / `FMSInfo` NetworkTables publisher) even when teams don't explicitly log individual DS mode booleans.

---

## Detection Algorithm

```
1. Auto-detect mode signals from signal names (boolean DS signals).
2. If no boolean signals found, look for FMSControlData integer signal.
   Decompose each bitflag sample into (timestamp, mode_key, bool) tuples:
     enabled  = controlWord & 0x01
     auto     = enabled AND (controlWord & 0x02)
     test     = enabled AND (controlWord & 0x04)
     estop    = controlWord & 0x08
     teleop   = enabled AND NOT auto AND NOT test
   This produces the same transition format as the boolean path.
3. If neither source is available, return None (no phase data).
4. For each mode signal/transition, extract (timestamp_us, mode_key, bool_value).
5. Merge into a unified timeline:
   a. Walk through all mode transitions in timestamp order.
   b. At each timestamp, determine the active mode from signal priorities:
      - If estop is True → DISABLED (treat as disabled)
      - If autonomous is True → AUTONOMOUS
      - If teleop is True → TELEOP
      - If test is True → TEST
      - Otherwise → DISABLED
   c. On each mode change, close the current interval and start a new one.
6. Fill any leading/trailing time (from log start/end) with DISABLED.
7. Merge adjacent DISABLED intervals (e.g. from brief signal jitter).
8. Drop intervals shorter than a configurable minimum (default: 100 ms)
   to filter out transition noise, EXCEPT for the auto→teleop disabled gap
   which is real and meaningful.
9. Return a MatchPhaseTimeline.
```

---

## Reboot Handling

A mid-match reboot — caused by a code crash, a watchdog timeout, or a brownout power cycle — is important context for phase detection.

### How reboots manifest in WPILib logs

WPILib's `DataLog` is instantiated by the robot process on startup. It creates a **new `.wpilog` file** (with a timestamp-based filename) each time the process starts. When the robot process exits — whether from a code crash, watchdog kill, or power loss — the old `DataLog` file is closed (or left truncated). When the roboRIO's systemd service restarts the process (or the roboRIO finishes rebooting after a brownout), a **new** `DataLog` is constructed, creating a **new file**.

This means a mid-match restart **always produces two separate `.wpilog` files**. However, the timestamp behaviour differs depending on the type of restart:

**Code crash (no power loss):**
The roboRIO's Linux kernel stays running. WPILib timestamps come from `CLOCK_MONOTONIC` (via `wpi::Now()`), which is a kernel-level clock that does **not** reset when the robot process restarts. So:
1. **Pre-crash file** — timestamps run from ~0 (system boot) to the crash point, e.g. 0–45 s.
2. **Post-restart file** — timestamps **continue from where the kernel clock is**, e.g. starting at ~47 s (after a ~2 s restart). They do **not** reset to zero.
3. The gap between files is typically **2–5 seconds** (just the time for systemd to restart the process and robot code to initialise).

**Full power cycle (brownout):**
The roboRIO reboots entirely, resetting the kernel's monotonic clock.
1. **Pre-crash file** — timestamps run from ~0 to the crash point. File may be truncated.
2. **Post-reboot file** — timestamps start from **~0 again** (fresh kernel boot).
3. The gap is typically **15–30 seconds** (full Linux boot + JVM/Python start + robot code init).

In both cases, the gap itself is not recorded in either file.

### Implications for match-phase detection

Since restarts produce separate files, the **single-file** `detect_match_phases()` function does not need restart-aware logic. Each file is self-consistent with monotonic timestamps:

- **Pre-crash file**: The phase timeline ends abruptly (e.g. teleop phase has no proper end — it's inferred from the last timestamp in the file). The standalone report should note that the file appears truncated (match duration is much shorter than the expected ~155 s).
- **Post-restart file**: The phase timeline starts with a disabled interval (during robot code init), then typically enters teleop when the FMS re-enables. There's no auto phase because the FMS won't restart autonomous.

The current implementation surfaces these cases heuristically via `MatchPhaseTimeline.appears_truncated` and `MatchPhaseTimeline.appears_post_reboot` so other analyzers can soften or contextualise their findings.

Stitching these two files into a single match timeline is the responsibility of the **multi-file analysis** feature (Medium-Term roadmap), not the single-file phase detector.

### Identifying same-match file pairs

For code crashes (no power loss), the two files share the same `CLOCK_MONOTONIC` epoch — their timestamp ranges are **non-overlapping and contiguous**. This is a strong signal that the files belong to the same match and can be directly time-aligned without any estimation.

For brownout reboots, the clock resets, so timestamp alignment requires an external reference (FMS match timer or filename timestamps).

### Multi-file stitching (future — multi-file analysis feature)

When the multi-file analysis feature is implemented, it should handle restarts:

```
1. Identify file pairs that belong to the same match:
   - Same event code and match number in the filename
   - Or overlapping FMS match timer values (NT:/FMSInfo/MatchTime)
   - Or user-specified grouping

2. Order the files chronologically:
   - Use FMS match timer if available (does NOT reset on reboot)
   - For code crashes: CLOCK_MONOTONIC is shared, so file timestamps
     are directly comparable (file 2's timestamps are later than file 1's)
   - For brownouts: use filename timestamps or FMS match timer

3. Stitch the phase timelines:
   - File 1's timeline covers match start to crash
   - Insert a DISABLED gap for the restart duration
   - File 2's timeline covers recovery to match end
   - Produce a combined MatchPhaseTimeline with a reboot annotation

4. Determine the restart gap:
   - Code crash (shared clock): gap = file2_first_timestamp - file1_last_timestamp
     (exact, no estimation needed)
   - Brownout with FMS timer: gap = file2_first_fms_time - file1_last_fms_time
   - Brownout without FMS timer: use filename timestamp difference,
     or a heuristic default (15 s)
```

### Data model additions

The `MatchPhaseTimeline` should still support restart annotations for the multi-file case:

```python
@dataclass
class RestartEvent:
    """A detected mid-match restart (from multi-file stitching)."""
    pre_crash_last_us: int     # last timestamp before the crash (file 1)
    post_restart_first_us: int # first timestamp after restart (file 2)
    gap_s: float               # wall-clock duration of the gap
    restart_type: str           # "code-crash" | "power-cycle"
    gap_confidence: str         # "exact" (shared clock) | "fms-anchored" | "estimated"
    pre_file: str              # path to the pre-crash log file
    post_file: str             # path to the post-restart log file

@dataclass
class MatchPhaseTimeline:
    intervals: list[PhaseInterval]
    restarts: list[RestartEvent] = field(default_factory=list)

    @property
    def had_restart(self) -> bool:
        return len(self.restarts) > 0
```

### Single-file truncation detection

Even though restarts don't appear within a single file, the phase detector should flag when a file appears to be a **truncated match fragment**:

- If the file contains auto but no teleop, and auto ends abruptly → likely crashed during or just after auto.
- If the file starts directly in teleop with no auto → likely post-reboot second half.
- If total enabled time is much less than 150 s and the file has FMS match info → likely a fragment.

This is exposed via the `MatchPhaseTimeline`:

```python
@property
def appears_truncated(self) -> bool:
    """True if this looks like a partial match (possible crash/reboot)."""
    total_enabled = sum(
        iv.duration_s for iv in self.intervals
        if iv.phase != MatchPhase.DISABLED
    )
    return self.has_match and total_enabled < 120.0  # expected ~150 s

@property
def appears_post_reboot(self) -> bool:
    """True if this looks like the second half of a rebooted match."""
    if not self.intervals:
        return False
    first_enabled = next(
        (iv for iv in self.intervals if iv.phase != MatchPhase.DISABLED),
        None,
    )
    return (
        first_enabled is not None
        and first_enabled.phase == MatchPhase.TELEOP  # no auto at all
        and self.has_match is False
    )
```

### Impact on consumers

- **Single-file analyzers** — work normally on each file independently. The `appears_truncated` / `appears_post_reboot` flags let them add a note to their output (e.g. "⚠ This appears to be a partial match log").
- **Multi-file stitching** (future) — produces a combined `MatchPhaseTimeline` with `had_restart = True`. All the consumer APIs (`classify_events_by_phase`, `slice_signal_by_phase`, etc.) work on the stitched timeline unchanged.
- **The standalone `match-phases` report** should show a warning when truncation is detected:

```
=== Match Phase Timeline ===
  ⚠ This log appears to be a truncated match (45.2 s enabled, expected ~150 s).
    Possible mid-match crash or reboot. Look for a second log file from
    the same match.

  Phase       Start(s)   End(s)   Duration(s)
  ---------   --------   ------   -----------
  disabled         0.0     2.3           2.3
  auto             2.3    17.3          15.0
  disabled        17.3    17.8           0.5
  teleop          17.8    47.5          29.7
```

### Edge cases

| Scenario | Behaviour |
|----------|-----------|
| Code crash during teleop | Pre-crash file has auto + partial teleop. Post-restart file has disabled + teleop (no auto). Timestamps share the same `CLOCK_MONOTONIC` epoch — gap is exact. |
| Brownout during auto | Pre-crash file has partial auto only. Post-reboot file starts in disabled, then teleop. Clock resets to ~0 — gap requires FMS timer or estimation. |
| Multiple crashes | Multiple files, each a fragment. Multi-file stitching chains them. Code crashes with shared clocks can be ordered exactly. |
| Clean match (no crash) | Single file, `appears_truncated` is `False`. No restart concerns. |
| Practice/pit (no FMS) | Short enable cycles are normal, not truncation. `has_match` is `False`, so `appears_truncated` is `False` (it only flags when `has_match` is `True`). |

---

## Public API (for other analyzers)

The module exports these functions that any analyzer can import:

```python
def detect_match_phases(log_data: LogData) -> MatchPhaseTimeline | None:
    """Detect match phases from mode signals in the log.

    Returns None if no mode signals are found (the log has no phase data).
    Returns a MatchPhaseTimeline with at least one interval otherwise.
    """

def classify_events_by_phase(
    timeline: MatchPhaseTimeline,
    events: Sequence[tuple[int | float, T]],
    *,
    timestamps_are_seconds: bool = False,
    grace_period_s: float = 0.0,
) -> dict[MatchPhase, list[T]]:
    """Partition a list of (timestamp, event) pairs by match phase.

    Parameters:
        timeline: The phase timeline.
        events: Sequence of (timestamp, event_object) pairs.
        timestamps_are_seconds: If True, timestamps are float seconds;
            if False (default), int microseconds.
        grace_period_s: Extend each phase boundary by this many seconds
            when classifying. Events that occur within *grace_period_s*
            after a phase ends are still attributed to that phase.
            This handles hardware lag (e.g. flywheel spin-down launches
            that fire after teleop officially ends).

    Returns:
        Dict mapping each MatchPhase to a list of events that occurred
        during (or within the grace period of) that phase.
    """

def slice_signal_by_phase(
    timeline: MatchPhaseTimeline,
    signal: SignalData,
    phase: MatchPhase,
    *,
    grace_period_s: float = 0.0,
) -> SignalData:
    """Extract the portion of a signal that falls within a given phase.

    Like processor.slice_by_time but driven by match phases.
    """

def phase_durations(timeline: MatchPhaseTimeline) -> dict[MatchPhase, float]:
    """Return total duration in seconds for each phase."""
```

---

## Grace Period (Spin-Down / Lag Handling)

Many robot mechanisms don't stop instantly when a phase ends. The most common case is a flywheel that keeps spinning (and can still launch game elements) for 0.5–2 seconds after teleop ends, while the robot transitions to disabled.

The `grace_period_s` parameter on `classify_events_by_phase` and `slice_signal_by_phase` handles this:

- A launch at `t = teleop_end + 0.3s` with `grace_period_s=1.0` is attributed to TELEOP, not DISABLED.
- This is purely a **classification** feature — it doesn't move phase boundaries. The timeline itself remains accurate.
- Default is `0.0` (strict boundaries). Analyzers that know about hardware lag opt in to a grace period.

---

## Usage by Other Analyzers

### Launch Counter — per-phase breakdown

```python
from logreader.analyzers.match_phases import (
    detect_match_phases, classify_events_by_phase, MatchPhase,
)

class LaunchCounterAnalyzer(BaseAnalyzer):
    def run(self, log_data, **options):
        # ... existing detection logic ...
        launches = detect_launches(vel_sig, ...)

        # Phase breakdown (if phase data available)
        timeline = detect_match_phases(log_data)
        if timeline is not None:
            events = [(l.time_s, l) for l in launches]
            by_phase = classify_events_by_phase(
                timeline,
                events,
                timestamps_are_seconds=True,
                grace_period_s=2.0,  # flywheel spin-down
            )
            # Add per-phase rows to the result:
            # "Auto: 12 launches, Teleop: 44, Disabled: 0"
            # Include launches-during-spindown note when
            # disabled count would have been nonzero without grace.
```

### PDH Power — per-phase power totals

```python
timeline = detect_match_phases(log_data)
if timeline is not None:
    for phase in [MatchPhase.AUTONOMOUS, MatchPhase.TELEOP, MatchPhase.DISABLED]:
        phase_data = slice_signal_by_phase(timeline, current_sig, phase)
        # compute average/peak power during this phase
        # NOTE: power can go NEGATIVE during disabled phases —
        # motors coast and act as generators (back-EMF feeds
        # current back into the bus). Track min power as well
        # as max; negative values are electrically real and
        # indicate regenerative energy from spinning mechanisms.
```

### Battery Health — per-phase voltage sag

```python
timeline = detect_match_phases(log_data)
if timeline is not None:
    auto_voltage = slice_signal_by_phase(timeline, volt_sig, MatchPhase.AUTONOMOUS)
    teleop_voltage = slice_signal_by_phase(timeline, volt_sig, MatchPhase.TELEOP)
    # Compare min voltage, sag events, etc. across phases
```

---

## Standalone Analyzer Output

When run as `logreader match-phases <file>`, the analyzer reports:

```
=== Match Phase Timeline ===
  Log duration:   155.2 s
  Match detected: Yes

  Phase       Start(s)   End(s)   Duration(s)
  ---------   --------   ------   -----------
  disabled         0.0     2.3           2.3
  auto             2.3    17.3          15.0
  disabled        17.3    17.8           0.5
  teleop          17.8   152.8         135.0
  disabled       152.8   155.2           2.4

  Summary:
    Autonomous: 15.0 s
    Teleop:     135.0 s
    Disabled:   5.2 s (3 intervals)
```

The `extra` dict in `AnalysisResult` contains the full `MatchPhaseTimeline` object for programmatic consumers:

```python
result.extra["timeline"]   # MatchPhaseTimeline
result.extra["has_match"]  # bool
result.extra["durations"]  # dict[MatchPhase, float]
```

---

## File Organisation

```
src/logreader/analyzers/
    match_phases.py          # MatchPhase, PhaseInterval, MatchPhaseTimeline,
                             # detect_match_phases(), classify_events_by_phase(),
                             # slice_signal_by_phase(), phase_durations(),
                             # MatchPhasesAnalyzer (the CLI analyzer)
```

Everything lives in one module. The data models and utility functions are importable by other analyzer modules directly:

```python
from logreader.analyzers.match_phases import (
    MatchPhase,
    MatchPhaseTimeline,
    PhaseInterval,
    detect_match_phases,
    classify_events_by_phase,
    slice_signal_by_phase,
    phase_durations,
)
```

---

## CLI Usage

```bash
# Show match phase timeline
logreader match-phases path/to/log.wpilog

# Launch counter with per-phase breakdown
logreader launch-counter path/to/log.wpilog --by-phase

# Launch counter with custom spin-down grace period
logreader launch-counter path/to/log.wpilog --by-phase --grace-period 2.0
```

---

## Testing Strategy

Tests use synthetic `LogData` with crafted boolean mode signals and FMSControlData bitflags:

1. **Standard match** — auto (15 s) + disabled gap + teleop (135 s) → verify correct intervals.
2. **Practice session** — multiple enable/disable cycles, no FMS → verify no crash, reasonable intervals.
3. **No mode signals** — `detect_match_phases()` returns `None`.
4. **Partial signals** — only `DSAuto` and `DSTeleop` present (no `DSDisabled`) → infer disabled.
5. **E-stop** — phase cut short mid-teleop → verify early termination.
6. **Grace period** — events after phase end classified correctly with/without grace.
7. **`classify_events_by_phase`** — unit test with known events and known timeline.
8. **`slice_signal_by_phase`** — verify returned `SignalData` contains only in-phase samples.
9. **FMSControlData standard match** — bitflag sequence (0→51→50→49→48) produces correct auto/teleop intervals.
10. **FMSControlData fallback** — works when no boolean DS signals exist.
11. **FMSControlData disabled-only** — all-zero control words → single DISABLED interval.
12. **FMSControlData e-stop** — estop bit (0x08) produces DISABLED even with enabled bit set.
13. **FMSControlData test mode** — test bit (0x04) produces TEST phase.
14. **Boolean signals preferred** — when both boolean DS signals and FMSControlData exist, boolean signals take priority.
15. **Truncated match file** — synthetic data with auto (15 s) + teleop (30 s) only. Verify `appears_truncated` is `True`.
16. **Post-reboot file** — synthetic data starting with disabled + teleop, no auto. Verify `appears_post_reboot` is `True`.
17. **Full match** — verify `appears_truncated` is `False` and `appears_post_reboot` is `False`.
18. **Practice session** — short enables with no FMS. Verify `appears_truncated` is `False` (because `has_match` is `False`).
