"""Check motor 21 from the prior hoot session (2026-04-08_17-31-29)."""
from __future__ import annotations
import os, sys, subprocess, tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
os.environ["OWLET_PATH"] = r"c:\Tools\owlet-26.1.0-windowsx86-64.exe"

HOOT_RIO = r"D:\Temp\2026-4-8_logs\2026-04-08_17-31-29\rio_2026-04-08_17-31-34.hoot"
owlet = os.environ["OWLET_PATH"]

# Motor 21 key signals only
SIGNAL_IDS = [
    "6f31500",   # StatorCurrent
    "6f41500",   # SupplyCurrent
    "6f61500",   # DeviceTemp
    "6fd1500",   # Velocity
    "7071500",   # ControlMode
    "70f1500",   # RobotEnable
    "7131500",   # PIDVelocity_Reference
    "276b1500",  # Fault_StatorCurrLimit
    "276c1500",  # StickyFault_StatorCurrLimit
    "82b1500",   # AncillaryDeviceTemp
    # Motor 20
    "6f31400",   # StatorCurrent
    "6f61400",   # DeviceTemp
]

out_path = Path(tempfile.mkdtemp()) / "prior_motor21.wpilog"
cmd = [owlet, HOOT_RIO, str(out_path), "-f", "wpilog"]
for sid in SIGNAL_IDS:
    cmd.extend(["-s", sid])

print("Converting prior hoot...", file=sys.stderr, flush=True)
result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
print(f"Output: {out_path.stat().st_size / 1024:.0f} KB", file=sys.stderr)

from logreader.wpilog_reader import read_wpilog
log = read_wpilog(str(out_path))

# Find enable transitions
enable_time_us = None
for name in log.signal_names():
    if "RobotEnable" in name:
        sig = log.signals[name]
        print(f"\n{name}:")
        prev_val = None
        for v in sig.values:
            if v.value != prev_val:
                print(f"  t={v.timestamp_us/1e6:.3f}s  {v.value}")
                prev_val = v.value
                if (v.value == "Enabled" or v.value == 1) and enable_time_us is None:
                    enable_time_us = v.timestamp_us

if enable_time_us is None:
    print("No enable found!")
    # Use velocity proxy
    for name in log.signal_names():
        if "21" in name and "/Velocity" in name and "PID" not in name:
            sig = log.signals[name]
            for v in sig.values:
                if isinstance(v.value, (int, float)) and abs(v.value) > 1:
                    enable_time_us = v.timestamp_us - 500_000
                    print(f"Velocity proxy: first motion at {v.timestamp_us/1e6:.3f}s")
                    break
            break

if enable_time_us is None:
    enable_time_us = 0

print(f"\nEnable time: {enable_time_us/1e6:.3f}s")

windows = [
    ("pre-enable", -2.0, 0.0),
    ("0-1s", 0.0, 1.0),
    ("1-2s", 1.0, 2.0),
    ("2-3s", 2.0, 3.0),
    ("3-4s", 3.0, 4.0),
    ("4-5s", 4.0, 5.0),
    ("5-6s", 5.0, 6.0),
    ("6-8s", 6.0, 8.0),
    ("8-10s", 8.0, 10.0),
]

print("\n" + "=" * 80)
print("MOTOR 21 — Per-Second Summary (prior session)")
print("=" * 80)

for name in sorted(log.signal_names()):
    if "21" not in name:
        continue
    sig = log.signals[name]
    short = name.rsplit("/", 1)[-1]

    vals_in_window = [v for v in sig.values
                      if enable_time_us - 2_000_000 <= v.timestamp_us <= enable_time_us + 10_000_000]
    if not vals_in_window:
        continue

    num_vals = [v.value for v in vals_in_window if isinstance(v.value, (int, float))]
    if not num_vals:
        print(f"\n{short}:")
        prev = None
        for v in vals_in_window:
            if v.value != prev:
                t = (v.timestamp_us - enable_time_us) / 1e6
                print(f"  t={t:+7.3f}s  {v.value}")
                prev = v.value
        continue

    print(f"\n{short}:")
    print(f"  {'Window':<12} {'N':>5} {'Min':>10} {'Max':>10} {'Avg':>10}")
    for label, t_start, t_end in windows:
        ws = enable_time_us + int(t_start * 1e6)
        we = enable_time_us + int(t_end * 1e6)
        pts = [v.value for v in sig.values if ws <= v.timestamp_us <= we and isinstance(v.value, (int, float))]
        if pts:
            print(f"  {label:<12} {len(pts):>5} {min(pts):>10.2f} {max(pts):>10.2f} {sum(pts)/len(pts):>10.2f}")

# Also show motor 20 DeviceTemp for comparison
print("\n" + "=" * 80)
print("MOTOR 20 DeviceTemp (prior session)")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "20" in name and "DeviceTemp" in name:
        sig = log.signals[name]
        print(f"\n{name}:")
        print(f"  {'Window':<12} {'N':>5} {'Min':>10} {'Max':>10} {'Avg':>10}")
        for label, t_start, t_end in windows:
            ws = enable_time_us + int(t_start * 1e6)
            we = enable_time_us + int(t_end * 1e6)
            pts = [v.value for v in sig.values if ws <= v.timestamp_us <= we and isinstance(v.value, (int, float))]
            if pts:
                print(f"  {label:<12} {len(pts):>5} {min(pts):>10.2f} {max(pts):>10.2f} {sum(pts)/len(pts):>10.2f}")

# Also check motor 20 stator current
print("\n" + "=" * 80)
print("MOTOR 20 StatorCurrent (prior session)")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "20" in name and "StatorCurrent" in name:
        sig = log.signals[name]
        print(f"\n{name}:")
        print(f"  {'Window':<12} {'N':>5} {'Min':>10} {'Max':>10} {'Avg':>10}")
        for label, t_start, t_end in windows:
            ws = enable_time_us + int(t_start * 1e6)
            we = enable_time_us + int(t_end * 1e6)
            pts = [v.value for v in sig.values if ws <= v.timestamp_us <= we and isinstance(v.value, (int, float))]
            if pts:
                print(f"  {label:<12} {len(pts):>5} {min(pts):>10.2f} {max(pts):>10.2f} {sum(pts)/len(pts):>10.2f}")

print("\nDONE")
