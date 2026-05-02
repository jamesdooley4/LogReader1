"""Diagnose DriveState struct entries: compare broken Q79 vs working Q56."""

import struct
from wpiutil.log import DataLogReader

BROKEN = r"D:\Temp\2026_DCMPs\FRC_20260411_005847_PNCMP_Q79.wpilog"
WORKING = r"D:\Temp\2026_WASAM\FRC_20260322_173414_WASAM_Q56.wpilog"


def analyse_log(path: str, label: str) -> None:
    print(f"\n{'='*80}")
    print(f"  {label}: {path}")
    print(f"{'='*80}")

    reader = DataLogReader(path)

    # Collect start records (entry metadata)
    entries: dict[int, dict] = {}
    data_counts: dict[int, int] = {}
    data_unique: dict[int, set] = {}
    first_data: dict[int, bytes] = {}
    last_data: dict[int, bytes] = {}
    first_ts: dict[int, int] = {}
    last_ts: dict[int, int] = {}

    for record in reader:
        if record.isStart():
            start = record.getStartData()
            entries[start.entry] = {
                "name": start.name,
                "type": start.type,
                "metadata": start.metadata,
            }
            data_counts[start.entry] = 0
            data_unique[start.entry] = set()
        elif record.isFinish():
            pass
        elif record.isSetMetadata():
            pass
        else:
            # Data record
            entry_id = record.getEntry()
            if entry_id in data_counts:
                data_counts[entry_id] += 1
                raw = record.getRaw()
                raw_bytes = bytes(raw)
                data_unique[entry_id].add(raw_bytes)
                if entry_id not in first_data:
                    first_data[entry_id] = raw_bytes
                    first_ts[entry_id] = record.getTimestamp()
                last_data[entry_id] = raw_bytes
                last_ts[entry_id] = record.getTimestamp()

    # Filter to DriveState entries
    ds_entries = {
        eid: info
        for eid, info in entries.items()
        if "DriveState" in info["name"]
    }

    # Sort by name
    for eid in sorted(ds_entries, key=lambda e: entries[e]["name"]):
        info = entries[eid]
        count = data_counts.get(eid, 0)
        unique = len(data_unique.get(eid, set()))
        name = info["name"]
        typ = info["type"]

        print(f"\n  entry={eid}  {name}")
        print(f"    type: {typ}")
        print(f"    data records: {count}   unique payloads: {unique}")

        if count > 0:
            ft = first_ts[eid]
            lt = last_ts[eid]
            dur = (lt - ft) / 1e6
            print(f"    time span: {ft} -> {lt}  ({dur:.1f}s)")
            fb = first_data[eid]
            lb = last_data[eid]
            print(f"    first payload ({len(fb)} bytes): {fb[:64].hex(' ')}")
            print(f"    last  payload ({len(lb)} bytes): {lb[:64].hex(' ')}")

            if unique == 1 and count > 1:
                print(f"    *** ALL {count} DATA RECORDS HAVE IDENTICAL PAYLOAD ***")

            # For doubles, show value
            if typ == "double" and len(fb) == 8:
                fval = struct.unpack("<d", fb)[0]
                lval = struct.unpack("<d", lb)[0]
                print(f"    first value: {fval}   last value: {lval}")

    # Also show schema / protobuf descriptor entries if any
    print(f"\n  --- Schema / .type entries ---")
    for eid in sorted(entries, key=lambda e: entries[e]["name"]):
        info = entries[eid]
        name = info["name"]
        if "struct:" in info["type"] or "structschema" in info["type"] or "proto:" in info["type"] or "/.type" in name:
            count = data_counts.get(eid, 0)
            unique = len(data_unique.get(eid, set()))
            print(f"  entry={eid}  {name}  type={info['type']}  records={count}  unique={unique}")


if __name__ == "__main__":
    analyse_log(BROKEN, "BROKEN (Q79 - April 11)")
    analyse_log(WORKING, "WORKING (Q56 - March 22)")
