<!-- Use this file to provide workspace-specific custom instructions to Copilot. -->

# LogReader1 - FRC Robot Log Reader

## Project Overview
- **Language**: Python 3.14 (WPILib supported: 3.10–3.14)
- **Platform**: Windows, macOS, Linux
- **Purpose**: Read and process FRC robot log files (.wpilog, .hoot, .dslog, and .dsevents formats)
- **Core dependency**: `robotpy-wpiutil` (provides `wpiutil.log.DataLogReader`, `DataLogRecord`, etc.)

## Key APIs
- `wpiutil.log.DataLogReader` — reads WPILib .wpilog files
- `wpiutil.log.DataLogRecord` — individual log records with typed getters (getBoolean, getDouble, getFloat, getInteger, getString, and array variants)
- `wpiutil.log.StartRecordData` — metadata from start control records (entry name, type, metadata)
- `wpiutil.log.ControlRecordType` — enum for control record types

## Architecture
- `src/logreader/` — main package
  - `wpilog_reader.py` — WPILib .wpilog file reader using DataLogReader
  - `hoot_reader.py` — CTRE .hoot file reader (converts via owlet CLI to .wpilog, then reads)
  - `dslog_reader.py` — FRC Driver Station .dslog / .dsevents binary file reader
  - `models.py` — data models for log entries, signals, and metadata
  - `processor.py` — log data processing and analysis
  - `cli.py` — command-line interface
  - `utils.py` — shared utilities

## Hoot File Support
- CTRE .hoot files require the `owlet` CLI tool (from Phoenix Tuner / AdvantageScope) to convert to .wpilog format
- AdvantageScope uses owlet with: `owlet <hoot_path> <output_path> -f wpilog`
- After conversion, the resulting .wpilog is read using the standard DataLogReader

## DS Log File Support
- `.dslog` files are binary (version 4), 50 Hz fixed-period records with battery voltage, trip time, packet loss, CPU/CAN utilization, mode flags, and PDP/PDH per-channel currents
- `.dsevents` files contain LabView-timestamped UTF-8 text messages (mode changes, errors, warnings)
- Both use LabView timestamps (seconds since 1904-01-01); convert with `unix = lv_seconds - 2_082_844_800`
- Reference: [AdvantageScope DSLogReader.ts](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/src/hub/dataSources/dslog/DSLogReader.ts)
- Design doc: `docs/design-log-correlation.md`

## Development Guidelines
- Use type hints throughout
- Follow PEP 8 style
- Use `pyproject.toml` for project configuration
- Use `pytest` for testing
- Timestamps from DataLogRecord.getTimestamp() are in integer microseconds
