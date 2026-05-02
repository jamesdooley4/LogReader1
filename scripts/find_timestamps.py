"""Find the right timestamps for the dive incidents."""
import sys, os, struct, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from logreader.wpilog_reader import read_wpilog

LOG_PATH = r"D:\Temp\2026-04-07_programming\FRC_20260407_231323.wpilog"

log_data = read_wpilog(LOG_PATH)
sigs = log_data.signals

# Check DriveState/Timestamp - this is FPGA time
ts_sig = sigs.get("NT:/DriveState/Timestamp")
if ts_sig and ts_sig.values:
    first = ts_sig.values[0]
    last = ts_sig.values[-1]
    print(f"DriveState/Timestamp: {len(ts_sig.values)} samples")
    print(f"  First: log_us={first.timestamp_us}, value={first.value:.6f}")
    print(f"  Last:  log_us={last.timestamp_us}, value={last.value:.6f}")
    print(f"  FPGA range: {first.value:.1f}s to {last.value:.1f}s")
    
    # What log_us corresponds to FPGA=381?
    for target in [381, 385, 530, 535]:
        for v in ts_sig.values:
            if abs(v.value - target) < 0.02:
                print(f"  FPGA={target}s → log_us={v.timestamp_us}, t_rel={(v.timestamp_us - first.timestamp_us)/1e6:.3f}s")
                break

# Also check raw timestamps
all_ts = [sig.values[0].timestamp_us for sig in sigs.values() if sig.values]
t0 = min(all_ts)
print(f"\nMin log_us: {t0}")
print(f"Max log_us: {max(sig.values[-1].timestamp_us for sig in sigs.values() if sig.values)}")

# DS enabled transitions with both log_us and FPGA time
print("\n=== DS mode with FPGA correlation ===")
ds_auto = sigs.get("DS:autonomous")
ds_enabled = sigs.get("DS:enabled")

if ds_enabled:
    for v in ds_enabled.values:
        # Find nearest DriveState/Timestamp
        fpga = "?"
        if ts_sig:
            best = min(ts_sig.values, key=lambda x: abs(x.timestamp_us - v.timestamp_us))
            if abs(best.timestamp_us - v.timestamp_us) < 100000:
                fpga = f"{best.value:.1f}"
        print(f"  log_us={v.timestamp_us}  FPGA≈{fpga}s  enabled={v.value}")

if ds_auto:
    for v in ds_auto.values:
        fpga = "?"
        if ts_sig:
            best = min(ts_sig.values, key=lambda x: abs(x.timestamp_us - v.timestamp_us))
            if abs(best.timestamp_us - v.timestamp_us) < 100000:
                fpga = f"{best.value:.1f}"
        print(f"  log_us={v.timestamp_us}  FPGA≈{fpga}s  autonomous={v.value}")
