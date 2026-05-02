"""Analyze DS logs and wpilog files to find DataLog pause events.

Identifies:
1. Sessions where the DataLog pause fix was being tested (multiple error messages)
2. Sessions where the DataLog error caused early wpilog cutoff
"""

import struct
import os
import re
from pathlib import Path
from datetime import datetime, timezone, timedelta

# ── DS Events reader ──────────────────────────────────────────────────────

LABVIEW_EPOCH_OFFSET = -2082826800  # 1904-01-01 → unix epoch

def convert_lv_time(seconds_bytes: bytes, frac_bytes: bytes) -> float:
    """Convert LabView timestamp (seconds since 1904 + fractional) to unix time."""
    seconds = struct.unpack('>q', seconds_bytes)[0]
    fractional = struct.unpack('>Q', frac_bytes)[0]
    return LABVIEW_EPOCH_OFFSET + seconds + fractional / (2**64)


def read_dsevents(path: Path) -> list[dict]:
    """Read a .dsevents file and return list of {timestamp, text} entries."""
    data = path.read_bytes()
    if len(data) < 20:
        return []
    
    version = struct.unpack('>i', data[0:4])[0]
    if version != 4:
        print(f"  WARNING: {path.name} has unsupported version {version}")
        return []
    
    start_time = convert_lv_time(data[4:12], data[12:20])
    position = 20  # 4 + 8 + 8
    entries = []
    
    while position + 20 <= len(data):
        try:
            ts = convert_lv_time(data[position:position+8], data[position+8:position+16])
            position += 16
            length = struct.unpack('>i', data[position:position+4])[0]
            position += 4
            if length < 0 or position + length > len(data):
                break
            text = data[position:position+length].decode('utf-8', errors='replace')
            position += length
            
            # Strip XML tags like AdvantageScope does
            for tag in ["<TagVersion>", "<time>", "<count>", "<flags>", "<Code>",
                         "<location>", "<stack>"]:
                while tag in text:
                    tag_idx = text.index(tag)
                    next_idx = text.find("<", tag_idx + 1)
                    if next_idx == -1:
                        text = text[:tag_idx]
                    else:
                        text = text[:tag_idx] + text[next_idx:]
            text = text.replace("<message> ", "").replace("<details> ", "").strip()
            
            adjusted_ts = ts - start_time
            entries.append({"timestamp": adjusted_ts, "abs_timestamp": ts, "text": text})
        except Exception as e:
            break
    
    return entries


def read_dslog_duration(path: Path) -> float | None:
    """Read a .dslog file and return its duration in seconds."""
    data = path.read_bytes()
    if len(data) < 20:
        return None
    
    version = struct.unpack('>i', data[0:4])[0]
    if version != 4:
        return None
    
    header_size = 20  # 4 + 8 + 8
    
    # Each record is at least 10 bytes for the base, plus PD data
    # We need to figure out record size by parsing
    # Actually, per the AdvantageScope code, each record has 10 base bytes,
    # then 4 bytes for PD info, then variable PD data.
    # Easier approach: count total records by trial parsing
    
    position = header_size
    record_count = 0
    period = 0.02  # 20ms per record
    
    while position < len(data):
        if position + 10 > len(data):
            break
        position += 10
        
        # PD type byte is at offset +3 in the 4-byte PD header
        if position + 4 > len(data):
            break
        pd_type_byte = data[position + 3]
        position += 4
        
        if pd_type_byte == 33:  # REV
            position += 1 + 27 + 4 + 1  # CAN ID + booleans + extra channels + temp
        elif pd_type_byte == 25:  # CTRE
            position += 1 + 21 + 3  # CAN ID + data + metadata
        # else: None, no extra data
        
        record_count += 1
    
    return record_count * period


def get_wpilog_duration(path: Path) -> float | None:
    """Get the duration of a wpilog file by reading first and last timestamps."""
    try:
        from wpiutil.log import DataLogReader
        reader = DataLogReader(str(path))
        
        first_ts = None
        last_ts = None
        for record in reader:
            ts = record.getTimestamp()
            if first_ts is None:
                first_ts = ts
            last_ts = ts
        
        if first_ts is not None and last_ts is not None:
            return (last_ts - first_ts) / 1_000_000  # microseconds to seconds
        return None
    except Exception as e:
        print(f"  Error reading {path.name}: {e}")
        return None


def parse_ds_filename_time(name: str) -> datetime:
    """Parse DS log filename like '2026_04_04 12_22_40 Sat' to datetime."""
    m = re.match(r'(\d{4}_\d{2}_\d{2} \d{2}_\d{2}_\d{2})', name)
    if m:
        return datetime.strptime(m.group(1), "%Y_%m_%d %H_%M_%S")
    raise ValueError(f"Can't parse time from {name}")


def parse_wpilog_filename_time(name: str) -> datetime:
    """Parse wpilog filename like 'FRC_20260404_192241' to datetime (UTC)."""
    m = re.match(r'FRC_(\d{8}_\d{6})', name)
    if m:
        return datetime.strptime(m.group(1), "%Y%m%d_%H%M%S")
    raise ValueError(f"Can't parse time from {name}")


def match_sessions(ds_dir: Path, wpilog_dir: Path, utc_offset_hours: int = -7):
    """Match DS log sessions to wpilog files by timestamp.
    
    DS logs are in local time, wpilog filenames are in UTC.
    utc_offset_hours: local = UTC + offset (e.g., PDT = UTC - 7 → offset = -7)
    """
    # Collect unique DS session timestamps
    ds_sessions = {}
    for f in sorted(ds_dir.glob("*.dsevents")):
        try:
            ts = parse_ds_filename_time(f.stem)
            base = f.stem  # e.g., "2026_04_04 12_22_40 Sat"
            ds_sessions[base] = {"dsevents": f, "time_local": ts}
        except ValueError:
            continue
    
    # Add dslog files
    for f in sorted(ds_dir.glob("*.dslog")):
        base = f.stem
        if base in ds_sessions:
            ds_sessions[base]["dslog"] = f
    
    # Collect wpilog files
    wpilogs = {}
    for f in sorted(wpilog_dir.glob("FRC_*.wpilog")):
        try:
            ts_utc = parse_wpilog_filename_time(f.stem)
            wpilogs[f.stem] = {"path": f, "time_utc": ts_utc}
        except ValueError:
            continue
    
    # Match by converting local DS time to UTC
    offset = timedelta(hours=utc_offset_hours)
    matches = []
    
    for base, ds_info in sorted(ds_sessions.items()):
        ds_utc = ds_info["time_local"] - offset  # Convert local to UTC
        
        # Find closest wpilog within 5 minutes
        best_match = None
        best_diff = timedelta(minutes=5)
        
        for wpi_name, wpi_info in wpilogs.items():
            diff = abs(ds_utc - wpi_info["time_utc"])
            if diff < best_diff:
                best_diff = diff
                best_match = wpi_name
        
        match_info = {
            "ds_base": base,
            "ds_time_local": ds_info["time_local"],
            "dsevents": ds_info.get("dsevents"),
            "dslog": ds_info.get("dslog"),
            "wpilog": wpilogs[best_match]["path"] if best_match else None,
            "wpilog_name": best_match,
            "time_diff": best_diff.total_seconds() if best_match else None,
        }
        matches.append(match_info)
    
    return matches


def main():
    ds_dir = Path(r"D:\Temp\2026-04-4_Practice\dslogs")
    wpilog_dir = Path(r"D:\Temp\2026-04-4_Practice")
    
    DATALOG_ERROR = "outgoing buffers exceeded threshold"
    
    print("=" * 100)
    print("DataLog Pause Analysis - 2026-04-04 Practice")
    print("=" * 100)
    
    # Match sessions
    matches = match_sessions(ds_dir, wpilog_dir)
    
    print(f"\nFound {len(matches)} DS sessions")
    print(f"Matched wpilog files: {sum(1 for m in matches if m['wpilog'])}")
    print()
    
    # Analyze each session
    error_sessions = []
    fix_sessions = []
    cutoff_sessions = []
    
    for i, m in enumerate(matches):
        ds_base = m["ds_base"]
        dsevents_path = m.get("dsevents")
        dslog_path = m.get("dslog")
        wpilog_path = m.get("wpilog")
        
        if not dsevents_path:
            continue
        
        # Read events
        events = read_dsevents(dsevents_path)
        
        # Find DataLog error events
        datalog_errors = [e for e in events if DATALOG_ERROR in e["text"]]
        
        if not datalog_errors:
            continue
        
        # Count occurrences
        error_count = len(datalog_errors)
        
        # Get DS log duration
        ds_duration = None
        if dslog_path and dslog_path.exists():
            ds_duration = read_dslog_duration(dslog_path)
        
        # Get DSEvents duration (time of last event)
        events_duration = events[-1]["timestamp"] if events else None
        
        # Get wpilog duration
        wpi_duration = None
        if wpilog_path and wpilog_path.exists():
            wpi_duration = get_wpilog_duration(wpilog_path)
        
        # First error timestamp
        first_error_ts = datalog_errors[0]["timestamp"]
        last_error_ts = datalog_errors[-1]["timestamp"]
        
        session_info = {
            "ds_base": ds_base,
            "error_count": error_count,
            "first_error_ts": first_error_ts,
            "last_error_ts": last_error_ts,
            "ds_duration": ds_duration,
            "events_duration": events_duration,
            "wpi_duration": wpi_duration,
            "wpilog_name": m.get("wpilog_name"),
            "wpilog_path": wpilog_path,
            "dslog_path": dslog_path,
            "datalog_errors": datalog_errors,
        }
        
        error_sessions.append(session_info)
        
        if error_count > 1:
            fix_sessions.append(session_info)
        
        # Check if wpilog was cut off much earlier than dslog
        if wpi_duration is not None and ds_duration is not None:
            if ds_duration > 0 and wpi_duration < ds_duration * 0.8:
                cutoff_sessions.append(session_info)
    
    # ── Report ──────────────────────────────────────────────────────────────
    
    def fmt_dur(d):
        if d is None:
            return "N/A"
        m, s = divmod(d, 60)
        return f"{int(m)}:{s:05.2f}"
    
    print("=" * 100)
    print(f"SESSIONS WITH DATALOG ERROR ({len(error_sessions)} total)")
    print("=" * 100)
    
    for s in error_sessions:
        print(f"\n{'─' * 80}")
        print(f"  DS Session:  {s['ds_base']}")
        print(f"  WPILog:      {s['wpilog_name'] or 'NO MATCH'}")
        print(f"  Error count: {s['error_count']}")
        print(f"  First error: {fmt_dur(s['first_error_ts'])} into session")
        if s['error_count'] > 1:
            print(f"  Last error:  {fmt_dur(s['last_error_ts'])} into session")
            # Show all error timestamps
            print(f"  Error times: {', '.join(fmt_dur(e['timestamp']) for e in s['datalog_errors'])}")
        print(f"  DS duration: {fmt_dur(s['ds_duration'])}")
        print(f"  Events dur:  {fmt_dur(s['events_duration'])}")
        print(f"  WPILog dur:  {fmt_dur(s['wpi_duration'])}")
        
        if s['wpi_duration'] is not None and s['ds_duration'] is not None and s['ds_duration'] > 0:
            ratio = s['wpi_duration'] / s['ds_duration']
            print(f"  WPI/DS ratio: {ratio:.1%}")
            if ratio < 0.8:
                print(f"  *** WPILOG CUT OFF EARLY ***")
    
    # ── Q1: Fix-tested sessions ──────────────────────────────────────────
    
    print("\n")
    print("=" * 100)
    print(f"Q1: SESSIONS WITH FIX (multiple DataLog error messages) ({len(fix_sessions)} found)")
    print("=" * 100)
    
    if fix_sessions:
        for s in fix_sessions:
            print(f"\n  Session: {s['ds_base']}")
            print(f"  WPILog:  {s['wpilog_name'] or 'NO MATCH'}")
            print(f"  Error count: {s['error_count']}")
            print(f"  Error timestamps: {', '.join(fmt_dur(e['timestamp']) for e in s['datalog_errors'])}")
            print(f"  DS duration: {fmt_dur(s['ds_duration'])}")
            print(f"  WPILog dur:  {fmt_dur(s['wpi_duration'])}")
            if s['wpi_duration'] is not None and s['ds_duration'] is not None:
                diff = s['ds_duration'] - s['wpi_duration']
                print(f"  Duration gap: {fmt_dur(abs(diff))} ({'WPI shorter' if diff > 0 else 'WPI longer'})")
            print(f"  Effect: The fix caused the DataLog writer to repeatedly retry, generating")
            print(f"          {s['error_count']} error messages over {fmt_dur(s['last_error_ts'] - s['first_error_ts'])}.")
            if s['wpi_duration'] is not None and s['first_error_ts'] > 0:
                if s['wpi_duration'] > s['first_error_ts']:
                    print(f"          WPILog continued recording beyond first error (fix working).")
                else:
                    print(f"          WPILog appears truncated near the first error.")
    else:
        print("  No sessions found with multiple DataLog error messages.")
    
    # ── Q2: Cutoff sessions ──────────────────────────────────────────────
    
    print("\n")
    print("=" * 100)
    print(f"Q2: SESSIONS WITH EARLY WPILOG CUTOFF ({len(cutoff_sessions)} found)")
    print("=" * 100)
    
    if cutoff_sessions:
        for s in cutoff_sessions:
            print(f"\n  Session: {s['ds_base']}")
            print(f"  WPILog:  {s['wpilog_name'] or 'NO MATCH'}")
            print(f"  Error count: {s['error_count']}")
            print(f"  First error: {fmt_dur(s['first_error_ts'])} into session")
            print(f"  DS duration: {fmt_dur(s['ds_duration'])}")
            print(f"  WPILog dur:  {fmt_dur(s['wpi_duration'])}")
            if s['ds_duration'] is not None and s['wpi_duration'] is not None:
                print(f"  WPI/DS ratio: {s['wpi_duration']/s['ds_duration']:.1%}")
                print(f"  Lost time:   {fmt_dur(s['ds_duration'] - s['wpi_duration'])}")
    else:
        print("  No sessions with early wpilog cutoff found.")
    
    # ── Summary ──────────────────────────────────────────────────────────
    
    single_error_sessions = [s for s in error_sessions if s['error_count'] == 1]
    
    print("\n")
    print("=" * 100)
    print("SUMMARY")
    print("=" * 100)
    print(f"  Total DS sessions:             {len(matches)}")
    print(f"  Sessions with DataLog error:   {len(error_sessions)}")
    print(f"  - With fix (multiple errors):  {len(fix_sessions)}")
    print(f"  - Without fix (single error):  {len(single_error_sessions)}")
    print(f"  Sessions with early cutoff:    {len(cutoff_sessions)}")


if __name__ == "__main__":
    main()
