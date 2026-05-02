"""Check if struct data records in Q79 contain changing values or are all identical."""

from wpiutil.log import DataLogReader
import struct as pystruct


def inspect_struct_values(path, label, entry_name_filter):
    print(f"\n=== {label} ===")
    reader = DataLogReader(path)

    entry_map = {}  # entry_id -> (name, type)
    samples = {}  # entry_id -> list of (ts, raw_bytes)

    for record in reader:
        if record.isControl():
            if record.isStart():
                s = record.getStartData()
                if entry_name_filter in s.name and "struct:" in s.type:
                    entry_map[s.entry] = (s.name, s.type)
                    samples[s.entry] = []
        else:
            eid = record.getEntry()
            if eid in samples:
                raw = record.getRaw()
                ts = record.getTimestamp()
                if len(samples[eid]) < 10:
                    samples[eid].append((ts, raw))
                elif len(samples[eid]) == 10:
                    # Also grab some from middle/end
                    samples[eid].append(("...", None))

    # Second pass for last few records
    for record in reader:
        pass  # can't easily do two passes with this API

    for eid in sorted(samples.keys()):
        name, type_str = entry_map[eid]
        recs = samples[eid]
        print(f"\n  {name} ({type_str}):")
        print(f"    Total samples collected: {len(recs)}")

        # Check if all raw bytes are identical
        raw_set = set()
        for ts, raw in recs:
            if raw is not None:
                raw_set.add(raw)

        if len(raw_set) == 1:
            raw_bytes = list(raw_set)[0]
            print(f"    ALL IDENTICAL! raw_len={len(raw_bytes)} hex={raw_bytes[:32].hex()}")
            # Try to decode as doubles
            if len(raw_bytes) % 8 == 0:
                doubles = pystruct.unpack(f"<{len(raw_bytes)//8}d", raw_bytes)
                print(f"    As doubles: {doubles}")
        else:
            print(f"    {len(raw_set)} unique values in first 10 samples")
            for i, (ts, raw) in enumerate(recs[:5]):
                if raw is not None:
                    # Try to decode
                    if len(raw) % 8 == 0 and len(raw) <= 48:
                        doubles = pystruct.unpack(f"<{len(raw)//8}d", raw)
                        print(f"    [{i}] ts={ts/1e6:.3f}s doubles={doubles}")
                    else:
                        print(f"    [{i}] ts={ts/1e6:.3f}s len={len(raw)} hex={raw[:24].hex()}")


inspect_struct_values(
    r"D:\Temp\2026_DCMPs\FRC_20260411_005847_PNCMP_Q79.wpilog",
    "BROKEN Q79 - struct values",
    "DriveState",
)

inspect_struct_values(
    r"D:\Temp\2026_WASAM\FRC_20260322_173414_WASAM_Q56.wpilog",
    "WORKING Q56 - struct values",
    "DriveState",
)
