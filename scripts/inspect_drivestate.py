"""Compare DriveState entries between two log files."""

from wpiutil.log import DataLogReader
import os


def inspect_log(path, label):
    print(f"\n=== {label}: {os.path.basename(path)} ===")
    reader = DataLogReader(path)
    if not reader.isValid():
        print("  INVALID LOG")
        return

    entry_map = {}  # entry_id -> (name, type, metadata)
    data_counts = {}  # entry_id -> count
    first_ts = {}  # entry_id -> first timestamp
    last_ts = {}  # entry_id -> last timestamp

    for record in reader:
        if record.isControl():
            if record.isStart():
                s = record.getStartData()
                if "DriveState" in s.name or "drivestate" in s.name.lower():
                    entry_map[s.entry] = (s.name, s.type, s.metadata)
                    data_counts[s.entry] = 0
                    meta_preview = s.metadata[:120] if s.metadata else ""
                    nt_flag = "NT:" in s.name
                    print(f"  START: entry={s.entry} name={s.name} [{'NT bridge' if nt_flag else 'DIRECT DataLog'}]")
                    print(f"         type={s.type}")
                    print(f"         metadata={meta_preview}")
        else:
            eid = record.getEntry()
            if eid in data_counts:
                data_counts[eid] += 1
                ts = record.getTimestamp()
                if eid not in first_ts:
                    first_ts[eid] = ts
                last_ts[eid] = ts

    print(f"\n  Data record counts:")
    for eid, count in sorted(data_counts.items()):
        name = entry_map[eid][0]
        ft = first_ts.get(eid, 0) / 1e6
        lt = last_ts.get(eid, 0) / 1e6
        print(f"    {name}: {count} records (first={ft:.3f}s last={lt:.3f}s)")


inspect_log(
    r"D:\Temp\2026_DCMPs\FRC_20260411_005847_PNCMP_Q79.wpilog",
    "BROKEN Q79",
)
inspect_log(
    r"D:\Temp\2026_WASAM\FRC_20260322_173414_WASAM_Q56.wpilog",
    "WORKING Q56",
)
