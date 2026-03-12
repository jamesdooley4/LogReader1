# Signal Gaps — Design Document

> Analyzer: `signal-gaps` · Module: `src/logreader/analyzers/signal_gaps.py`

**Status:** Design complete, not yet implemented  
**Depends on:** Optional integration with `match_phases` for phase-aware severity and active-only filtering  
**Validated against:** `FRC_20260308_225734_WABON_E14.wpilog`, `FRC_20260308_225931_WABON_E15.wpilog`

Identify signals with missing data, irregular sample rates, or mid-match dropouts. Flag potential CAN disconnects, sensor failures, device reboots, or code issues.

---

## Data Observations

Analysed `FRC_20260308_225734_WABON_E14.wpilog` (elimination match, ~535 s log with match from ~283–410 s).

### Signal categories by sample behaviour

Real-world FRC log signals fall into distinct categories based on their timing patterns:

**1. Continuous high-rate (roboRIO loop signals)**
- `DriveState/Timestamp`, `DriveState/OdometryFrequency`, `DriveState/Pose` — ~239 Hz, median gap 4.0 ms, p99 7.1 ms
- Produced by the robot's main control loop. Nearly rock-solid timing.
- Only one real gap: 6.3 s at 16.5–22.9 s (code init / NT connection setup).

**2. Continuous medium-rate (external devices)**
- `limelight-a/hb`, `limelight-a/tl` — ~46 Hz, median gap 16.7 ms, p99 19.3 ms
- Produced by external devices at their own clock rate. Very consistent *when running*.
- Subject to full dropout when the device reboots or loses connection.

**3. Active-only signals (mechanism telemetry)**
- `launcher/velocity`, `intake/leftRollerSpeed`, `Turret/Position`, `hood/position` — 6–11 Hz when active
- Only publish when the subsystem is in use. Long gaps during disabled/idle periods are **normal** — not a dropout.
- Median gap ~20 ms when active, but max gap 260–270 s (entire pre-match disabled period).

**4. Change-on-event signals**
- `FMSInfo/FMSControlData` — 6 pts total, `color/currentMode` — 37 pts total
- Published only when the value changes. Irregular gaps are inherent, not failures.

**5. Heartbeats (monotonically increasing counters)**
- `limelight-a/hb` — value increments by 1 each sample. A counter reset (value drops) indicates device reboot.
- `DriveState/Timestamp` — continuously increasing timestamp. Monotonicity proves the code kept running.
- `systemTime` — published every ~5 s, always increasing (wall-clock time).

**6. Configuration/metadata (write-once)**
- Schema signals, `.type`, `.name`, `.controllable` — 0–2 pts. Published at startup. Not expected to update.

### Key findings from real data

| Finding | Signal | Evidence |
|---------|--------|----------|
| **Limelight-a rebooted mid-match** | `limelight-a/hb` | 108 s gap at 366–475 s. Heartbeat counter jumped from 22639 → 1951 (reset). |
| **Limelight-b also dropped out** | `limelight-b/hb` | 70 s gap at 404–475 s. Shorter outage, same post-match recovery. |
| **roboRIO stayed up** | `DriveState/Timestamp` | Only 1 gap of 6.3 s at code init. No mid-match gaps. |
| **PDH current channels go quiet when disabled** | `Chan0`–`Chan19` | ~260 s gap during pre-match disabled. Normal — PDH only reports non-zero changes. |
| **PDH voltage stays active while disabled** | `Voltage` | No gaps > 684 ms. Always reports even during disabled (bus voltage is always present). |
| **Mechanism signals are quiet when idle** | `launcher/velocity` | 270 s gap from 13–284 s. Flywheel isn't running pre-match — this is expected. |
| **Heartbeat counter reset = device reboot** | `limelight-a/hb` | hb=22639 → (108 s gap) → hb=1951. Device rebooted and restarted its counter. |

### Implications for gap detection

1. **Not all gaps are problems.** Active-only signals, change-on-event signals, and config signals naturally have large gaps. The analyzer must classify signals before judging gaps.

2. **Mid-match gaps in continuous signals are the most important.** A limelight dropout during teleop is critical. A flywheel velocity gap during disabled is harmless.

3. **Heartbeat counter resets are a strong reboot signal.** A monotonically increasing counter that drops to a lower value proves the source device rebooted.

4. **Phase-awareness matters.** Gaps during disabled are expected for many signals. Gaps during auto/teleop are concerning. The analyzer should use `match_phases` when available to weight gaps by phase.

5. **Sample rate consistency matters more than absolute rate.** A signal that usually runs at 46 Hz but drops to 5 Hz for a period has a problem even if it never has a true "gap."

---

## Signal Classification

The analyzer should expose a stable classification and event model so results are consumable both by humans and by future report-generation / multi-file tooling.

### Public data contract

```python
class SignalCategory(Enum):
  CONFIG = "config"
  EVENT = "event"
  HEARTBEAT = "heartbeat"
  CONTINUOUS = "continuous"
  ACTIVE_ONLY = "active_only"


class Severity(Enum):
  INFO = "info"
  WARNING = "warning"
  CRITICAL = "critical"


@dataclass
class SignalStats:
  signal_name: str
  category: SignalCategory
  sample_count: int
  median_gap_us: int | None
  p95_gap_us: int | None
  p99_gap_us: int | None
  max_gap_us: int | None
  median_rate_hz: float | None


@dataclass
class GapEvent:
  signal_name: str
  category: SignalCategory
  start_us: int
  end_us: int
  duration_us: int
  phase: str | None
  severity: Severity
  reason: str


@dataclass
class ResetEvent:
  signal_name: str
  timestamp_us: int
  pre_value: float
  post_value: float
  phase: str | None
  severity: Severity
  reason: str = "counter reset"


@dataclass
class DegradedPeriod:
  signal_name: str
  start_us: int
  end_us: int
  expected_rate_hz: float
  observed_rate_hz: float
  phase: str | None
  severity: Severity


@dataclass
class Incident:
  start_us: int
  end_us: int
  signal_names: list[str]
  kind: str
  severity: Severity
  likely_scope: str | None
```

### Success criteria

- Detect the observed `limelight-a` mid-match dropout and heartbeat reset in E14.
- Do **not** flag expected disabled-period silence for active-only signals.
- Flag enabled-phase dropouts in continuous / heartbeat signals.
- Produce stable structured output in `AnalysisResult.extra` for downstream tooling.

Before analysing gaps, each signal is automatically classified:

```
Category          Detection method                              Gap behaviour
────────────────  ────────────────────────────────────────────  ────────────────────────
config            ≤ 3 samples total                             Ignore (not periodic)
event             > 3 pts, but median gap > 10 s                Ignore gaps (event-driven)
heartbeat         Numeric, monotonically increasing, ≥ 50 pts   Flag counter resets
continuous        Median gap < 500 ms, ≥ 50 pts                 Flag gaps > 5× median
active-only       Median gap < 500 ms during enabled phases,    Flag gaps during enabled
                  but large gaps during disabled                 phases only
```

### Classification algorithm

```
1. If sample count ≤ 3 → CONFIG. Skip further analysis.
2. Compute median inter-sample gap.
3. If median gap > 10 s → EVENT. Report sample count only.
4. If numeric and monotonically increasing (checked on first 100 samples):
   → HEARTBEAT. Check for counter resets (value decreases).
5. If match phases are available:
   a. Compute median gap using only samples during enabled phases.
   b. If enabled-phase median gap < 500 ms AND the signal has gaps
      > 10× enabled-median during disabled → ACTIVE-ONLY.
6. If match phases are NOT available:
   a. Look for bursty publication: many short internal gaps plus a few very long
      silent spans.
   b. Signals matching this pattern may still be classified as ACTIVE-ONLY,
      but with lower confidence; otherwise fall back to CONTINUOUS or EVENT.
7. Otherwise, if median gap < 500 ms → CONTINUOUS.
8. Else → EVENT.
```

### Heartbeat-specific rules

Monotonic numeric signals are not all equivalent. Treat a signal as `HEARTBEAT` only when several clues agree:

- Numeric, at least 50 samples
- Strongly monotonic or near-monotonic over the observed range
- Name hints like `hb`, `heartbeat`, `timestamp`, `time`
- Regular cadence when healthy

This covers true device heartbeats such as `limelight-a/hb`, and also monotonic timebase signals such as `DriveState/Timestamp`. The implementation may further distinguish between:

- **counter heartbeat** — expected to increment by roughly fixed steps; a decrease strongly implies reboot
- **timebase heartbeat** — monotonic timestamp source; reset semantics differ, but gaps are still useful liveness signals

If name heuristics are absent, default to `CONTINUOUS` unless the counter-reset behaviour clearly indicates a heartbeat.

---

## Gap Detection

### For CONTINUOUS signals

A gap is flagged when the inter-sample interval exceeds a threshold relative to the signal's own normal rate:

```
gap_threshold = max(median_gap × 5, 100 ms)

For each consecutive pair of samples:
  if gap > gap_threshold:
    record GapEvent(start_us, end_us, duration_ms, phase)
```

The `×5` multiplier accommodates normal jitter. The 100 ms floor prevents flagging sub-frame jitter on very high-rate signals (250 Hz → 4 ms median, 20 ms would be too sensitive).

### For ACTIVE-ONLY signals

Same as continuous, but only flag gaps that occur during enabled match phases (auto or teleop). Gaps during disabled phases are expected and silently ignored.

### For HEARTBEAT signals

Two checks:
1. **Gaps** — same as continuous (flag intervals > 5× median).
2. **Counter resets** — if `value[i+1] < value[i]`, the source device rebooted. Record a `ResetEvent(timestamp_us, pre_value, post_value)`.

> **Gotcha — integer wraparound vs. device reset:**
> Healthy counters can wrap around standard integer boundaries (e.g. $2^{16}-1 = 65535$ or $2^{32}-1$). A drop from near a power-of-two max to near zero is likely a normal wraparound, **not** a reboot. The implementation should suppress `ResetEvent` when `pre_value` is within ~1% of a standard integer max and `post_value` is small. Only flag a reset when the drop is clearly irregular (e.g. 22639 → 1951).

### For CONTINUOUS and ACTIVE-ONLY signals after a gap

> **Gotcha — NetworkTables deduplication:**
> WPILib's NT publisher only logs a sample when the value *changes*. If a sensor is perfectly stationary (e.g. a filtered encoder on a motionless mechanism during teleop defence), the signal stops publishing even though the device is healthy. This can create a false gap.
>
> When a gap is detected in a continuous or active-only signal, check whether `value_before_gap == value_after_gap` (exact float equality). If so, the gap *may* be NT deduplication rather than a real dropout. Downgrade its severity from `WARNING` → `INFO` and annotate the reason as `"possible NT deduplication"`.

### For EVENT and CONFIG signals

No gap analysis — these are inherently aperiodic.

---

## Severity Model

Severity should be explicit and deterministic so summaries and CLI filtering behave consistently.

```
Condition                                                        Severity
───────────────────────────────────────────────────────────────  ─────────
Gap during disabled / startup / shutdown only                   INFO
Gap during enabled phase in continuous or active-only signal    WARNING
Degraded sample rate during enabled phase                       WARNING
Heartbeat reset during disabled                                 WARNING
Heartbeat reset during auto / teleop                            CRITICAL
Correlated multi-signal outage during auto / teleop             CRITICAL
```

Disabled-phase findings are still useful, but they should not dominate the summary unless the user requests full detail.

---

## Boundary and Partial-Log Handling

To avoid false positives, the analyzer should treat log boundaries specially.

### Startup suppression

- Gaps entirely within an initial startup window (default: first 10 s after the signal's first sample, or first enabled phase if later) should be downgraded to `INFO`.
- This covers robot boot, NT publisher startup, and initial connection settling.

### End-of-log handling

- Do **not** infer a trailing gap after a signal's final sample.
- A signal ending before the file ends is only reportable if there is a known large inter-sample gap before the last observation.

### Truncated and rebooted files

When `match_phases` reports context such as an apparently truncated file or a post-reboot teleop-only file:

- downgrade startup findings further
- avoid over-interpreting missing auto data in a post-reboot file
- include context in the summary when many signals end abruptly near file end

---

## Sample Rate Degradation

Beyond outright gaps, the analyzer also detects periods of **degraded sample rate** — where a signal is still publishing but at a reduced rate.

```
For signals with > 100 samples:
  Divide the signal's active time range into 1-second windows.
  For each window, compute the sample rate.
  If any window's rate < 50% of the median rate AND the window
  is during an enabled phase → flag as DEGRADED.
```

This catches scenarios like CAN bus congestion reducing a signal from 50 Hz to 10 Hz without fully dropping it.

Implementation details to pin down:

- Use fixed 1-second buckets aligned to the signal's first sample time.
- Only evaluate degradation for signals with at least 100 samples and at least 5 populated windows.
- Merge adjacent degraded windows into a single `DegradedPeriod`.
- For active-only signals, only consider windows that overlap enabled phases.

> **Gotcha — zero-sample windows:**
> The windowing loop must iterate by wall-clock time (stepping from `first_sample_us` in 1-second increments to `last_sample_us`), **not** by grouping existing samples into chunks. Otherwise, time spans where zero samples arrived are silently skipped and never flagged as 0 Hz. A window with 0 samples during an enabled phase should be treated as a rate of 0 Hz and flagged as `DEGRADED`.

---

## Incident Grouping

Raw per-signal events are useful for detail, but users usually care about **incidents**.

After detecting `GapEvent` and `ResetEvent` records, group overlapping findings into higher-level incidents:

1. Sort events by time.
2. Merge events whose time ranges overlap or fall within a small join tolerance (for example 250 ms).
3. If multiple grouped signals share a naming prefix (e.g. `NT:/limelight-a/`), emit a likely device-scoped incident.
4. If a grouped incident contains both a long gap and a heartbeat reset, summarise it as a probable device reboot.

Example:

```text
Likely limelight-a outage: 366.6s–474.8s (teleop, CRITICAL)
  Signals affected: NT:/limelight-a/hb, NT:/limelight-a/tl
  Evidence: heartbeat counter reset at 474.8s
```

This grouping should supplement, not replace, the detailed per-signal rows.

---

## Output Design

### Summary

```
=== Signal Gap Analysis ===
  Signals analysed:  142
  Config (skipped):   68
  Event (skipped):     8
  Continuous:         38
  Active-only:        22
  Heartbeat:           6

  ⚠ Gaps detected:     4 signals with mid-match gaps
  ⚠ Device reboots:    1 (limelight-a heartbeat counter reset)
  ⚠ Rate degradation:  2 signals with reduced sample rate
```

### Gap table

```
Signal                    Category     Gap Start  Gap End  Duration  Phase
────────────────────────  ───────────  ─────────  ───────  ────────  ───────
NT:/limelight-a/hb        heartbeat      366.6s   474.8s   108.1s   teleop ⚠
NT:/limelight-a/tl        continuous     366.6s   474.8s   108.1s   teleop ⚠
NT:/limelight-b/hb        heartbeat      404.5s   474.8s    70.2s   disabled
NT:/DriveState/Timestamp  continuous      16.5s    22.9s     6.3s   disabled
```

### Heartbeat reset table

```
Signal                 Reset At   Pre-value  Post-value  Likely cause
─────────────────────  ────────   ─────────  ──────────  ────────────
NT:/limelight-a/hb      474.8s     22639.0      1951.0  Device reboot
```

### Per-signal detail (with `--detail` flag)

```
NT:/limelight-a/hb (heartbeat, 22128 pts)
  Median rate:   45.7 /s (16.7 ms gap)
  p95 gap:       18.2 ms
  p99 gap:       19.3 ms
  Max gap:       108139.6 ms
  Gaps (>83 ms):
    16.5s – 22.9s    6308 ms  (disabled)
    366.6s – 474.8s  108140 ms  (teleop) ⚠ COUNTER RESET
  Counter resets:
    474.8s: 22639 → 1951  (device rebooted)
```

### `extra` dict

```python
result.extra = {
  "classifications": dict[str, str],       # signal name → SignalCategory.value
    "gaps": list[GapEvent],                  # all detected gaps
    "resets": list[ResetEvent],              # heartbeat counter resets
    "degraded": list[DegradedPeriod],        # rate degradation periods
  "incidents": list[Incident],             # grouped cross-signal incidents
    "signal_stats": dict[str, SignalStats],  # per-signal rate statistics
}
```

---

## Integration with match-phases

When match-phase data is available (via `detect_match_phases()`):

1. **Phase-annotate each gap** — label it as occurring during disabled, auto, or teleop.
2. **Filter active-only gaps** — only flag gaps during enabled phases.
3. **Severity weighting** — gaps during auto/teleop are flagged with ⚠, gaps during disabled are informational only.
4. **Summary includes phase context** — "2 gaps during teleop, 1 during disabled."

When match-phase data is NOT available, all gaps are reported without phase annotation.

When phase data is unavailable, classification should degrade gracefully:

- active-only classification becomes lower-confidence and heuristic-based
- severity falls back to time-only context (startup vs mid-log)
- grouped incidents still work and remain useful even without phase labels

---

## CLI Usage

```bash
# Basic — analyse all signals
logreader signal-gaps path/to/log.wpilog

# Show per-signal detail
logreader signal-gaps path/to/log.wpilog --detail

# Only show gaps during enabled phases
logreader signal-gaps path/to/log.wpilog --enabled-only

# Custom gap threshold multiplier (default: 5×)
logreader signal-gaps path/to/log.wpilog --threshold 10

# Filter to specific signal prefix
logreader signal-gaps path/to/log.wpilog --prefix "NT:/limelight"
```

---

## Testing Strategy

1. **Continuous signal with no gaps** — steady 50 Hz signal → no gaps flagged.
2. **Continuous signal with mid-stream gap** — 50 Hz signal with a 2-second gap → gap flagged.
3. **Active-only signal with disabled gap** — signal active during auto+teleop, silent during disabled → no gap flagged.
4. **Active-only signal with teleop gap** — signal drops during teleop → gap flagged.
5. **Heartbeat with counter reset** — monotonic counter that drops from 1000 to 50 → reset detected.
6. **Heartbeat with no reset** — monotonic counter, no drops → no reset flagged.
7. **Config signal (≤ 3 pts)** — classified as config, no gap analysis.
8. **Event signal (sparse)** — signal with median gap > 10 s → classified as event, no gaps flagged.
9. **Rate degradation** — 50 Hz signal that drops to 10 Hz for 5 seconds → degraded period flagged.
10. **Phase-aware filtering** — gaps during disabled not flagged for active-only signals; gaps during teleop are flagged.
11. **No match phases** — when `detect_match_phases()` returns None, all gaps reported without phase context.
12. **Signal with only 1 sample** — classified as config, skipped gracefully.
13. **Heartbeat wraparound** — counter near 65535 wrapping to near 0 → no reset flagged.
14. **NT deduplication** — continuous signal with a gap where pre/post values are identical → severity downgraded to INFO.
15. **Degradation with zero-sample windows** — signal fully drops out for 3 seconds mid-teleop → three 0 Hz windows flagged as degraded.
