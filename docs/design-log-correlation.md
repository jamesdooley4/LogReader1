# Log Correlation — Design Document

> Module: `src/logreader/dslog_reader.py` + Analyzer: `log-correlation` · Module: `src/logreader/analyzers/log_correlation.py`

**Status:** Design complete, not yet implemented
**Depends on:** `match_phases` for phase-boundary cross-validation
**Reference implementation:** [AdvantageScope DSLogReader.ts / DSEventsReader.ts](https://github.com/Mechanical-Advantage/AdvantageScope/tree/main/src/hub/dataSources/dslog)

Read FRC Driver Station `.dslog` and `.dsevents` log files, then correlate them with `.wpilog` (roboRIO) and `.hoot` (CANivore) logs from the same match. Produce a unified timeline that exposes signals from all sources under a common time base.

---

## Part 1: `.dslog` / `.dsevents` File Reading

### File Discovery

Driver Station log files live under `C:\Users\Public\Documents\FRC\Log Files\` on the DS laptop. Each match produces a pair:

| File | Pattern | Content |
|------|---------|---------|
| `.dslog` | `2026_03_08 22-57-34.dslog` | Binary telemetry (50 Hz, 20 ms period) |
| `.dsevents` | `2026_03_08 22-57-34.dsevents` | Timestamped text events |

The filename timestamp is local wall-clock time on the DS laptop. Both files in a pair share the same base timestamp.

### `.dslog` Binary Format (Version 4)

Based on [AdvantageScope DSLogReader.ts](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/src/hub/dataSources/dslog/DSLogReader.ts) and [FRCture documentation](https://frcture.readthedocs.io/en/latest/driverstation/logging.html).

#### Header (20 bytes)

| Offset | Size | Type | Field |
|--------|------|------|-------|
| 0 | 4 | int32 BE | Version (must be 4) |
| 4 | 8 | int64 BE | LabView timestamp — seconds since 1904-01-01 |
| 12 | 8 | uint64 BE | LabView timestamp — fractional part |

**LabView → UNIX time conversion:**
```
unix_time = lv_seconds - 2_082_826_800 + lv_fractional / 2^64
```
The constant `2_082_826_800` is the number of seconds between 1904-01-01 and 1970-01-01.

#### Record (fixed-size, repeating at 20 ms intervals)

Each record starts at `position` and contains:

| Offset | Size | Type | Field | Conversion |
|--------|------|------|-------|------------|
| +0 | 1 | uint8 | Trip time | `× 0.5` → milliseconds |
| +1 | 1 | int8 | Packet loss | `× 4 × 0.01` → fraction (clamp 0–1) |
| +2 | 2 | uint16 BE | Battery voltage | `÷ 256` → volts |
| +4 | 1 | uint8 | CPU utilization | `× 0.5 × 0.01` → fraction |
| +5 | 1 | uint8 | Status mask | Bitfield (see below) |
| +6 | 1 | uint8 | CAN utilization | `× 0.5 × 0.01` → fraction |
| +7 | 1 | uint8 | WiFi signal | `× 0.5` → dB |
| +8 | 2 | uint16 BE | WiFi bandwidth | `÷ 256` → Mbps |

**Status mask bits** (active-low — bit clear = condition true):

| Bit | Mask | Field |
|-----|------|-------|
| 7 | 0x80 | Brownout |
| 6 | 0x40 | Watchdog |
| 5 | 0x20 | DS Teleop |
| 3 | 0x08 | DS Disabled |
| 2 | 0x04 | Robot Teleop |
| 1 | 0x02 | Robot Auto |
| 0 | 0x01 | Robot Disabled |

Note: Bits are **active-low** (inverted from what you might expect). A cleared bit means the condition is active. For example, `(mask & 0x80) == 0` means brownout is active.

After the 10 core bytes, a **power distribution** block follows:

| Byte at +10..+13 | Field |
|-------------------|-------|
| +13 high nibble | PD type selector: `33` = REV PDH, `25` = CTRE PDP, else none |

**REV PDH current decoding** (28 bytes after PD type header + 1 CAN ID byte):
- 27 bytes → 216 bits → 20 channels × 10 bits each (bit-packed, LSB first)
- Current = `10-bit value ÷ 8` (amps)
- Then 4 bytes for switchable channels: `byte ÷ 16` each
- Then 1 byte skipped (temperature data)
- Total: 1 + 27 + 4 + 1 = 33 bytes past the +14 position

**CTRE PDP current decoding** (25 bytes after PD type header + 1 CAN ID byte):
- 21 bytes → 168 bits → 16 channels × 10 bits each (bit-packed within 64-bit groups)
- Current = `8-bit value ÷ 8` (amps, only 8 bits used per channel despite 10-bit spacing)
- Then 3 bytes of extra metadata skipped
- Total: 1 + 21 + 3 = 25 bytes

**Battery voltage spike guard:** If `batteryVolts > 20`, substitute the previous valid reading (artifact at end of some logs).

**WiFi note:** WiFi dB and Mbps fields are logged in the file but AdvantageScope does not use them — [per Chief Delphi](https://www.chiefdelphi.com/t/alternate-viewer-for-driver-station-logs-dslog/120629/11) the values are unreliable. We parse but mark as low-confidence.

### `.dsevents` Format (Version 4)

Based on [AdvantageScope DSEventsReader.ts](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/src/hub/dataSources/dslog/DSEventsReader.ts).

#### Header (20 bytes) — identical to `.dslog`

| Offset | Size | Type | Field |
|--------|------|------|-------|
| 0 | 4 | int32 BE | Version (must be 4) |
| 4 | 8 | int64 BE | LabView timestamp — seconds |
| 12 | 8 | uint64 BE | LabView timestamp — fractional |

#### Event Records (variable-length, repeating)

| Offset | Size | Type | Field |
|--------|------|------|-------|
| +0 | 8 | int64 BE | LabView timestamp — seconds |
| +8 | 8 | uint64 BE | LabView timestamp — fractional |
| +16 | 4 | int32 BE | Text length (bytes) |
| +20 | *N* | UTF-8 | Event text |

**Text cleanup:** Strip XML-style tags (`<TagVersion>`, `<time>`, `<count>`, `<flags>`, `<Code>`, `<location>`, `<stack>`) and content up to the next `<` tag. Remove `<message> ` and `<details> ` prefixes. Trim whitespace.

**Timestamp adjustment:** Each event's timestamp is stored as an absolute LabView time. Subtract the header start time to get relative seconds from log start.

### Signals Produced

The dslog reader maps data into the standard `LogData` model:

| Signal name | Type | Source |
|-------------|------|--------|
| `/DSLog/TripTimeMS` | double | Trip time in ms |
| `/DSLog/PacketLoss` | double | Packet loss fraction (0–1) |
| `/DSLog/BatteryVoltage` | double | Battery voltage (V) |
| `/DSLog/CPUUtilization` | double | roboRIO CPU utilization (0–1) |
| `/DSLog/CANUtilization` | double | CAN bus utilization (0–1) |
| `/DSLog/Status/Brownout` | boolean | Brownout active |
| `/DSLog/Status/Watchdog` | boolean | Watchdog tripped |
| `/DSLog/Status/DSTeleop` | boolean | DS reports teleop |
| `/DSLog/Status/DSDisabled` | boolean | DS reports disabled |
| `/DSLog/Status/RobotTeleop` | boolean | Robot reports teleop |
| `/DSLog/Status/RobotAuto` | boolean | Robot reports auto |
| `/DSLog/Status/RobotDisabled` | boolean | Robot reports disabled |
| `/DSLog/PowerDistributionCurrents` | double[] | Per-channel PD currents (amps) |
| `/DSLog/WifiDb` | double | WiFi signal (dB) — low confidence |
| `/DSLog/WifiMb` | double | WiFi bandwidth (Mbps) — low confidence |
| `/DSEvents` | string | Event text messages |

### Python API

```python
# New module: src/logreader/dslog_reader.py

def read_dslog(path: str | Path) -> LogData:
    """Read a .dslog file and return LogData with /DSLog/* signals."""

def read_dsevents(path: str | Path) -> LogData:
    """Read a .dsevents file and return LogData with /DSEvents signal."""

def read_ds_logs(dslog_path: str | Path, dsevents_path: str | Path | None = None) -> LogData:
    """Read a .dslog and optionally a .dsevents file, merging into one LogData."""
```

### Implementation Notes

- Use Python `struct` module for big-endian binary parsing (`>i`, `>q`, `>Q`, `>H`, `>B`, etc.)
- LabView time conversion: `unix_time = int.from_bytes(seconds, 'big') - 2_082_826_800 + int.from_bytes(fractional, 'big') / (2**64)`
- Convert UNIX seconds to microseconds for internal timestamps (`int(unix_time * 1_000_000)`)
- Bit-packed PD current decoding: extract 10-bit values from byte arrays using bitwise operations
- Timestamp for each record: `start_time + record_index * 20_000` (20 ms in microseconds)

---

## Part 2: Multi-Source Log Correlation

### The Problem

An FRC match can produce multiple independent log files from different sources:

| Source | File | Clock | Data |
|--------|------|-------|------|
| **Driver Station** | `.dslog` + `.dsevents` | DS laptop wall clock (LabView time) | Battery voltage, mode flags, trip time, packet loss, CAN util, PD currents, events |
| **roboRIO** | `.wpilog` | roboRIO FPGA monotonic clock | All NT-published signals, robot code telemetry, WPILib diagnostics |
| **roboRIO CAN bus** | `.hoot` (Rio) | roboRIO device clock | CAN devices on the roboRIO's native CAN bus + custom signals logged by robot code via the CTRE signal logger |
| **CANivore** | `.hoot` (CANivore) | CANivore device clock | CAN devices on the CANivore bus (often drive motors, high-rate devices) |

Each CAN bus produces its own `.hoot` file. A robot with one CANivore generates **two** `.hoot` files per match — one for the roboRIO CAN bus and one for the CANivore bus. Robots with multiple CANivores produce even more. Each log runs on a **different clock** with no built-in synchronization. To correlate data across sources, the clocks must be aligned.

> **Note on roboRIO `.hoot` files:** The roboRIO `.hoot` log contains not only CAN device signals from the Rio's native CAN bus, but also **custom signals** that robot code explicitly logs via the CTRE signal logger API (`SignalLogger.writeXxx()`). These custom signals (e.g., mechanism state machines, PID setpoints, command scheduler events) make the Rio `.hoot` especially valuable — it can contain team-specific telemetry that doesn't appear in the `.wpilog`.

### Time Alignment Strategy

#### Method 1: Mode-Transition Edge Matching (primary)

All three log sources record robot mode transitions (enabled/disabled/auto/teleop). These transitions are driven by the same FMS packets and happen nearly simultaneously across all sources.

**Observable edges:**
- `.dslog` — `Status/DSDisabled`, `Status/DSTeleop`, `Status/RobotAuto` flags
- `.wpilog` — `DS:enabled`, `DS:autonomous`, `DSTeleop`, `DSAuto` booleans, or `FMSControlData` integer
- `.hoot` (after conversion) — often includes DS mode signals forwarded via Phoenix diagnostics

**Algorithm:**
1. Extract mode-transition edges from each log source
2. For each pair of sources, find the best time offset by minimizing the sum of squared edge-time differences (least-squares fit on corresponding transition edges)
3. Use RANSAC or robust matching if some edges are ambiguous (e.g., multiple quick enable/disable toggles during pre-match)
4. Apply the computed offset to shift all timestamps to a common base (use the `.wpilog` clock as reference)

**Expected accuracy:** ±20 ms (limited by the `.dslog` 50 Hz sample rate)

#### Method 2: Battery Voltage Cross-Correlation (secondary / validation)

Battery voltage is logged by the DS (from the PDP/PDH) and often also by the roboRIO. The voltage signal has distinctive transients (brownouts, current spikes from motors) that create a "fingerprint" for cross-correlation.

**Algorithm:**
1. Resample both voltage signals to a common rate (50 Hz)
2. Compute normalized cross-correlation
3. Find the lag that maximizes correlation
4. Validate against Method 1's offset — should agree within ~100 ms

#### Method 3: Shared CAN Signal Comparison (hoot ↔ wpilog, hoot ↔ hoot)

When the same CAN device's signals appear in both a `.hoot` and a `.wpilog` (e.g., a TalonFX motor velocity published to NetworkTables), direct comparison yields a precise time offset.

**Algorithm:**
1. Identify signals present in both logs (by CAN device name matching)
2. Cross-correlate the overlapping signals
3. Use the highest-rate signal for best precision (often 100–250 Hz)

**Expected accuracy:** ±1–5 ms

#### Method 4: Rio Hoot ↔ CANivore Hoot Alignment

When a robot has devices on both the roboRIO CAN bus and a CANivore bus, the two `.hoot` files need their own alignment. If any CAN device signals are published to both buses (rare), Method 3 applies directly. Otherwise, align each `.hoot` independently to the `.wpilog` (via Methods 1 or 3), which transitively aligns them to each other.

The roboRIO `.hoot` clock may be closely related to the roboRIO FPGA clock (both originate on the same hardware), so the Rio-hoot-to-wpilog offset is typically small and stable. The CANivore clock is independent and may drift more.

### File Matching

When given a directory of log files, automatically discover which files belong to the same match:

1. Group `.dslog` + `.dsevents` pairs by filename prefix (they always share the same timestamp string)
2. For each `.wpilog` and `.hoot` file, compare start timestamps and duration
3. Group `.hoot` files by CAN bus origin using the filename prefix:
   - **Rio**: `rio_<timestamp>.hoot` — prefix is literally `rio`
   - **CANivore**: `<GUID>_<timestamp>.hoot` — prefix is a 32-character hex device GUID (e.g., `ECA0983C3353385320202034170A03FF`)
   - Extract the `_YYYY-MM-DD_HH-MM-SS` timestamp suffix to match across files from the same session
   - Multiple `.hoot` files from the same match are expected (one per CAN bus)
4. Two files are candidates for the same match if their time ranges overlap by at least 80%
5. Validate matches using Method 1 (mode transition edges must align)

**`.hoot` filename examples:**
| File | Bus |
|------|-----|
| `rio_2026-04-05_00-28-37.hoot` | roboRIO native CAN bus |
| `ECA0983C3353385320202034170A03FF_2026-04-05_00-28-37.hoot` | CANivore (identified by GUID) |

### Unified LogData Model

The merged output uses source-prefixed signal names to avoid collisions:

| Prefix | Source |
|--------|--------|
| `/DSLog/*` | Driver Station `.dslog` |
| `/DSEvents` | Driver Station `.dsevents` |
| *(unprefixed)* | roboRIO `.wpilog` (existing signals keep their original names) |
| `/Hoot/Rio/*` | roboRIO CAN bus `.hoot` (CAN device signals + custom logged signals) |
| `/Hoot/<GUID>/*` | CANivore `.hoot` (e.g., `/Hoot/ECA0983C/*` — first 8 chars of GUID for readability) |

When only a single `.hoot` file is present and the bus cannot be determined from the filename, use `/Hoot/*` as the prefix. For CANivore buses, if a human-readable CANivore name is available from the hoot signal metadata (e.g., `canivore1`), prefer that over the GUID prefix.

The `.wpilog` acts as the reference clock — all other sources' timestamps are shifted to align.

### Python API

```python
# New functions in src/logreader/dslog_reader.py or a new correlation module

def find_matching_logs(directory: str | Path) -> list[MatchLogSet]:
    """Discover groups of log files from the same match in a directory."""

@dataclass
class HootFile:
    path: Path
    bus_name: str  # "rio" or a 32-char hex GUID (e.g. "ECA0983C3353385320202034170A03FF")
    friendly_name: str | None = None  # human-readable name if discovered from metadata

@dataclass
class MatchLogSet:
    dslog_path: Path | None
    dsevents_path: Path | None
    wpilog_path: Path | None
    hoot_files: list[HootFile]  # zero or more — one per CAN bus
    match_label: str  # e.g. "2026_03_08 22-57-34"

def merge_logs(
    log_set: MatchLogSet,
    *,
    reference: str = "wpilog",  # which clock to use as reference
) -> LogData:
    """Read all available logs from a match and merge into a unified LogData."""

@dataclass
class TimeAlignment:
    source_a: str
    source_b: str
    offset_us: int  # microseconds to add to source_b timestamps
    method: str  # "mode_edge", "voltage_xcorr", "can_signal"
    confidence: float  # 0–1
    residual_us: int  # RMS residual of aligned edges
```

### Analysis Opportunities

Cross-source correlation enables several analyses not possible from any single log:

| Analysis | Sources Needed | Insight |
|----------|---------------|---------|
| **DS-side vs robot-side packet loss** | dslog + wpilog | Distinguish network drops from roboRIO overruns |
| **Battery voltage agreement** | dslog + wpilog | Validate PDP/PDH readings, detect wiring issues |
| **CAN utilization vs robot performance** | dslog + wpilog | Correlate CAN congestion with control loop timing |
| **Mode-transition latency** | dslog + wpilog | Measure delay between DS mode change and robot code responding |
| **Device timeline (hoot) vs robot state (wpilog)** | hoot + wpilog | See motor behavior relative to robot code commands |
| **Rio CAN vs CANivore CAN consistency** | rio hoot + canivore hoot | Compare device behavior across buses, detect CAN bus isolation issues |
| **Custom signal correlation** | rio hoot + wpilog | Cross-reference CTRE-logged custom signals with NT-published equivalents |
| **Brownout sequence reconstruction** | dslog + wpilog + hoot | Full timeline: voltage sag → brownout flag → device dropout → recovery |
| **Comms quality vs vision drops** | dslog + wpilog | Correlate WiFi/trip time with Limelight dropout events |

### `log-correlation` Analyzer

The `log-correlation` analyzer takes a directory (or multiple file paths) and produces a report:

1. **File discovery** — list matched log file groups
2. **Time alignment** — report computed offsets and confidence for each source pair
3. **Overlapping signals** — compare signals present in multiple sources (e.g., battery voltage from DS vs roboRIO)
4. **Mode consistency** — verify that auto/teleop/disabled transitions agree across sources
5. **Cross-source anomalies** — flag cases where sources disagree (e.g., DS says enabled but robot code says disabled)

---

## Part 3: CLI Integration

### New Commands

```
# Read a single dslog:
logreader info match.dslog
logreader signals match.dslog

# Read dslog + dsevents pair:
logreader info match.dslog --events match.dsevents

# Correlate multiple logs from a match:
logreader log-correlation /path/to/logs/
logreader log-correlation match.dslog match.wpilog match.hoot

# Run any analyzer on correlated data:
logreader pdh-power --merge match.dslog match.wpilog
```

### Auto-Detection Updates

The existing `_read_log()` function in `cli.py` should be updated to handle `.dslog` and `.dsevents` extensions:

```python
def _read_log(path: str) -> LogData:
    ext = Path(path).suffix.lower()
    if ext == ".wpilog":
        return read_wpilog(path)
    elif ext == ".hoot":
        return read_hoot(path)
    elif ext == ".dslog":
        return read_dslog(path)
    elif ext == ".dsevents":
        return read_dsevents(path)
    else:
        raise ValueError(f"Unsupported file format: {ext}")
```

---

## Part 4: owlet Path Configuration

The owlet executable for `.hoot` → `.wpilog` conversion is located at:
- Default search: system `PATH`
- Environment variable: `OWLET_PATH`
- Known location: `c:\Tools\owlet-26.1.0-windowsx86-64.exe`

No changes needed — the existing `hoot_reader.py` `_find_owlet()` already handles discovery via `OWLET_PATH` or `PATH`.

---

## Implementation Order

1. **`dslog_reader.py`** — `.dslog` binary parser + `.dsevents` parser → `LogData`
2. **CLI integration** — update `_read_log()` for `.dslog` / `.dsevents` auto-detection
3. **Unit tests** — synthetic `.dslog` / `.dsevents` binary blobs, round-trip parsing verification
4. **`log-correlation` analyzer** — mode-edge time alignment, file matching, merged `LogData`
5. **Cross-source validation** — battery voltage, CAN utilization, mode consistency checks
