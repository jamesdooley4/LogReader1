# Log Augmentation — Design Document

> Module: `src/logreader/wpilog_writer.py` · CLI: `logreader augment`

Copy a `.wpilog` file record-for-record and inject additional derived signals so tools like AdvantageScope can visualise computed data in their 2D/3D field views without modifying the robot code.

**Status:** Implemented (botpose→Pose3d augmentation)
**Depends on:** `robotpy-wpimath` (optional — only for Pose3d struct packing)
**Validated against:** `FRC_20260322_205805_WASAM_E8.wpilog` (60 MB, 1.4M records, 4 Limelight botpose signals)

---

## Motivation

Limelight cameras publish robot pose estimates as `double[]` arrays in NetworkTables (e.g. `NT:/limelight-a/botpose_wpiblue`). AdvantageScope can display `struct:Pose3d` signals in its 2D and 3D field visualisers, but it cannot interpret raw double arrays as poses. By augmenting the log with parallel `struct:Pose3d` entries, teams get immediate visual feedback without touching the robot code.

This infrastructure also enables future augmentations: match-phase marker signals, computed chassis velocities from `pose-analysis`, current-limit flags from `drive-analysis`, etc.

---

## Architecture

### Two-pass design

The implementation uses a two-pass approach:

1. **Read pass** — iterate the input `.wpilog` via `DataLogReader`, extract every record into plain Python data (tuples of `(entry_id, raw_bytes, timestamp)`). This fully separates the C++ reader resources from the writer.
2. **Write pass** — create a `DataLogWriter`, replay all start/metadata/data records using the writer's own entry IDs, and inject augmented signals alongside the originals.

The two-pass separation is required because `wpiutil`'s `DataLogWriter` has an internal buffer that triggers a mutex deadlock (a pybind11/GIL interaction) when the buffer fills. The read pass collects everything into Python objects first, then the write pass operates on plain `bytes` objects with no C++ reader state alive.

### DataLogWriter deadlock workaround

`DataLogWriter` (the synchronous writer from `wpiutil._wpiutil`) deadlocks after ~11K `appendRaw` calls when its internal buffer fills and it attempts an automatic flush while the Python GIL is held. The workaround is simple: call `writer.flush()` every N appends (default: 5000) to drain the buffer before it overflows.

`DataLogBackgroundWriter` avoids the deadlock but replaces timestamps on `start()` calls with wall-clock time, corrupting the log's time domain. `DataLogWriter` with periodic flush preserves timestamps exactly.

### Entry ID remapping

`DataLogWriter.start()` returns its own entry IDs, which may differ from the input file's IDs. The writer maintains an `id_remap` dict (`input_entry_id → output_entry_id`) and translates all subsequent `appendRaw`, `finish`, and `setMetadata` calls through it.

---

## Botpose → Pose3d Augmentation

### Limelight botpose array layout

| Index | Field | Unit |
|-------|-------|------|
| 0 | x | meters |
| 1 | y | meters |
| 2 | z | meters |
| 3 | roll | degrees |
| 4 | pitch | degrees |
| 5 | yaw | degrees |
| 6 | latency | ms |
| 7 | tag_count | count |
| 8 | tag_span | meters |
| 9 | avg_dist | meters |
| 10 | avg_area | fraction |
| 11+ | per-tag data | varies |

### Target signals

Signals matching these suffixes are augmented:

- `*/botpose_wpiblue` → `*/botpose_wpiblue_pose3d`
- `*/botpose_orb_wpiblue` → `*/botpose_orb_wpiblue_pose3d`

### Conversion

```python
Pose3d(x, y, z, Rotation3d(roll_rad, pitch_rad, yaw_rad))
```

Where roll/pitch/yaw are converted from degrees to radians.

### Filtering

A sample is skipped (no Pose3d emitted) when:
- The array has fewer than 8 elements
- `tag_count` (index 7) is less than 1
- Both `x` and `y` are near zero (`abs < 0.001`) — indicates no valid detection

### Struct serialisation

The `Pose3d` is serialised via `wpistruct.pack(pose)` (56 bytes: 3 doubles for Translation3d + 4 doubles for Quaternion) and written as `appendRaw` to an entry with type `struct:Pose3d`. The Pose3d struct schema (and its dependencies: Translation3d, Rotation3d, Quaternion) are registered via `DataLogWriter.addStructSchema(Pose3d, timestamp)`.

---

## CLI Interface

```
logreader augment <file> [-o <output>]
```

| Argument | Description |
|----------|-------------|
| `file` | Input `.wpilog` file path |
| `-o, --output` | Output `.wpilog` path (default: `<input>_augmented.wpilog`) |

### Example

```bash
logreader augment match.wpilog
# → match_augmented.wpilog

logreader augment match.wpilog -o enriched.wpilog
# → enriched.wpilog
```

---

## Performance

Tested on `FRC_20260322_205805_WASAM_E8.wpilog` (60 MB, 1,437,559 data records):

- **Read pass:** ~1.5s (dominated by `bytes(rec.getRaw())` copy per record)
- **Write pass:** ~1.0s (periodic flush every 5000 appends)
- **Total:** ~2.5s
- **Output size:** 60.4 MB (original + 6,472 Pose3d samples × 56 bytes + schema overhead)
- **Memory:** ~800 MB peak (all records buffered as Python tuples)

---

## Future Augmentations

The `copy_and_augment` function in `wpilog_writer.py` accepts an `augmentations` list of callback functions. Each augmentation receives the start records and can:

1. Register additional entries (via `writer.start`)
2. Inspect each data record and optionally emit additional records

Planned augmentations:
- **Match-phase markers** — boolean signals for `auto`, `teleop`, `disabled` phases
- **Computed velocities** — `struct:ChassisSpeeds` from pose-analysis differentiation
- **Current-limit flags** — boolean signals from drive-analysis stator/supply limiting detection
