"""Focused investigation of motor 21 (leader) and 20 (follower) from hoot.

Outputs only summary stats and critical transition points.
"""
from __future__ import annotations

import os
import sys
import subprocess
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
os.environ["OWLET_PATH"] = r"c:\Tools\owlet-26.1.0-windowsx86-64.exe"

HOOT_RIO = r"D:\Temp\2026-4-8_logs\2026-04-08_17-40-09\rio_2026-04-08_17-40-14.hoot"
owlet = os.environ["OWLET_PATH"]

# Key signals for motors 20 and 21
SIGNAL_IDS = [
    # Motor 21 (leader)
    "6ec1500", "6f21500", "6f31500", "6f41500", "6f51500", "6f61500",
    "6fd1500", "7071500", "70f1500", "7131500", "7151500", "7201500",
    "7051500", "71a1500", "7ab1500", "82b1500",
    "276b1500", "276c1500", "276e1500", "276f1500", "27111500", "27171500",
    # Motor 20 (follower)
    "6ec1400", "6f21400", "6f31400", "6f41400", "6f51400", "6f61400",
    "6fd1400", "7071400", "70f1400", "7ab1400",
    "276b1400", "276e1400", "27111400",
]

out_path = Path(tempfile.mkdtemp()) / "motor_21_extract.wpilog"
cmd = [owlet, HOOT_RIO, str(out_path), "-f", "wpilog"]
for sid in SIGNAL_IDS:
    cmd.extend(["-s", sid])

print("Converting hoot...", file=sys.stderr, flush=True)
subprocess.run(cmd, capture_output=True, text=True, timeout=120)
print(f"Output: {out_path.stat().st_size / 1024:.0f} KB", file=sys.stderr)

from logreader.wpilog_reader import read_wpilog
log = read_wpilog(str(out_path))

# Find enable time
enable_time_us = None
for name in log.signal_names():
    if "RobotEnable" in name:
        sig = log.signals[name]
        for v in sig.values:
            if v.value == 1 or v.value is True:
                enable_time_us = v.timestamp_us
                break
        if enable_time_us:
            break

if enable_time_us is None:
    print("WARNING: No enable found in hoot, listing RobotEnable values:")
    for name in log.signal_names():
        if "RobotEnable" in name:
            sig = log.signals[name]
            for v in sig.values[:30]:
                print(f"  {v.timestamp_us/1e6:.3f}s -> {v.value} ({type(v.value).__name__})")
    # Fall back - look for first non-zero velocity on motor 21 as proxy
    for name in log.signal_names():
        if "21" in name and "Velocity" in name and "PID" not in name:
            sig = log.signals[name]
            for v in sig.values:
                if isinstance(v.value, (int, float)) and abs(v.value) > 1.0:
                    enable_time_us = v.timestamp_us - 500_000  # rough guess
                    print(f"Using velocity proxy: first motion at {v.timestamp_us/1e6:.3f}s")
                    break
            break

if enable_time_us is None:
    enable_time_us = 0
    print("COULD NOT DETERMINE ENABLE TIME")

print(f"Enable time: {enable_time_us/1e6:.3f}s")

# Define 1s windows for summary
windows = [
    ("pre-enable", -1.0, 0.0),
    ("0-1s", 0.0, 1.0),
    ("1-2s", 1.0, 2.0),
    ("2-3s", 2.0, 3.0),
    ("3-4s", 3.0, 4.0),
    ("4-5s", 4.0, 5.0),
    ("5-6s", 5.0, 6.0),
    ("6-7s", 6.0, 7.0),
    ("7-8s", 7.0, 8.0),
    ("8-10s", 8.0, 10.0),
]

def analyze_signal_windows(sig, enable_us, windows):
    """Return per-window stats."""
    results = []
    for label, t_start, t_end in windows:
        ws = enable_us + int(t_start * 1e6)
        we = enable_us + int(t_end * 1e6)
        pts = [v.value for v in sig.values if ws <= v.timestamp_us <= we and isinstance(v.value, (int, float))]
        if pts:
            results.append((label, len(pts), min(pts), max(pts), sum(pts)/len(pts)))
        else:
            results.append((label, 0, None, None, None))
    return results

# ── MOTOR 21 ────────────────────────────────────────────────────────
print("\n" + "=" * 80)
print("MOTOR 21 (FLYWHEEL LEADER) — Per-Second Summary")
print("=" * 80)

for name in sorted(log.signal_names()):
    if "21" not in name:
        continue
    sig = log.signals[name]
    short = name.rsplit("/", 1)[-1]
    
    # Check if any data in window
    vals_in_window = [v for v in sig.values 
                      if enable_time_us - 1_000_000 <= v.timestamp_us <= enable_time_us + 10_000_000]
    if not vals_in_window:
        continue
    
    num_vals = [v.value for v in vals_in_window if isinstance(v.value, (int, float))]
    if not num_vals:
        # Boolean/string signal
        print(f"\n{short}:")
        for v in vals_in_window[:20]:
            t = (v.timestamp_us - enable_time_us) / 1e6
            print(f"  t={t:+7.3f}s  {v.value}")
        continue
    
    stats = analyze_signal_windows(sig, enable_time_us, windows)
    print(f"\n{short}:")
    print(f"  {'Window':<12} {'N':>5} {'Min':>10} {'Max':>10} {'Avg':>10}")
    for label, n, mn, mx, avg in stats:
        if n > 0:
            print(f"  {label:<12} {n:>5} {mn:>10.2f} {mx:>10.2f} {avg:>10.2f}")

# ── Critical transitions: StatorCurrent first 1s detail ─────────────
print("\n" + "=" * 80)
print("MOTOR 21 StatorCurrent — First 2s Detail (every sample)")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "StatorCurrent" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -0.5 <= t <= 2.0:
                print(f"  t={t:+7.3f}s  {v.value:.1f}A")

print("\n" + "=" * 80)
print("MOTOR 21 SupplyCurrent — First 2s Detail")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "SupplyCurrent" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -0.5 <= t <= 2.0:
                print(f"  t={t:+7.3f}s  {v.value:.1f}A")

print("\n" + "=" * 80)
print("MOTOR 21 Velocity — First 2s Detail")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "/Velocity" in name and "PID" not in name and "Rotor" not in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -0.5 <= t <= 2.0:
                print(f"  t={t:+7.3f}s  {v.value:.2f} rps")

print("\n" + "=" * 80)
print("MOTOR 21 PIDVelocity_Reference — First 2s Detail")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "PIDVelocity_Reference" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -0.5 <= t <= 2.0:
                print(f"  t={t:+7.3f}s  {v.value:.2f} rps")

print("\n" + "=" * 80)
print("MOTOR 21 PIDVelocity_ClosedLoopError — First 2s Detail")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "ClosedLoopError" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -0.5 <= t <= 2.0:
                print(f"  t={t:+7.3f}s  {v.value:.2f} rps")

print("\n" + "=" * 80)
print("MOTOR 21 ControlMode — All values in window")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "ControlMode" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -2.0 <= t <= 10.0:
                print(f"  t={t:+7.3f}s  {v.value}")

print("\n" + "=" * 80)
print("MOTOR 21 Fault_StatorCurrLimit — All values in window")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "Fault_StatorCurrLimit" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -2.0 <= t <= 10.0:
                print(f"  t={t:+7.3f}s  {v.value}")

print("\n" + "=" * 80)
print("MOTOR 21 StickyFault_StatorCurrLimit — All values in window")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "StickyFault_StatorCurrLimit" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -2.0 <= t <= 10.0:
                print(f"  t={t:+7.3f}s  {v.value}")

print("\n" + "=" * 80)
print("MOTOR 21 BridgeOutput — First 2s Detail")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "BridgeOutput" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -0.5 <= t <= 2.0:
                print(f"  t={t:+7.3f}s  {v.value}")

print("\n" + "=" * 80)
print("MOTOR 21 MotorVoltage — First 2s Detail")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "MotorVoltage" in name and "PID" not in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -0.5 <= t <= 2.0:
                print(f"  t={t:+7.3f}s  {v.value:.2f}V")

print("\n" + "=" * 80)
print("MOTOR 21 SupplyVoltage — First 2s Detail")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "SupplyVoltage" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -0.5 <= t <= 2.0:
                print(f"  t={t:+7.3f}s  {v.value:.2f}V")

# ── MOTOR 20 (follower) ─────────────────────────────────────────────
print("\n" + "=" * 80)
print("MOTOR 20 (FLYWHEEL FOLLOWER) — Per-Second Summary")
print("=" * 80)

for name in sorted(log.signal_names()):
    if "20" not in name:
        continue
    sig = log.signals[name]
    short = name.rsplit("/", 1)[-1]
    
    vals_in_window = [v for v in sig.values 
                      if enable_time_us - 1_000_000 <= v.timestamp_us <= enable_time_us + 10_000_000]
    if not vals_in_window:
        continue
    
    num_vals = [v.value for v in vals_in_window if isinstance(v.value, (int, float))]
    if not num_vals:
        print(f"\n{short}:")
        for v in vals_in_window[:20]:
            t = (v.timestamp_us - enable_time_us) / 1e6
            print(f"  t={t:+7.3f}s  {v.value}")
        continue
    
    stats = analyze_signal_windows(sig, enable_time_us, windows)
    print(f"\n{short}:")
    print(f"  {'Window':<12} {'N':>5} {'Min':>10} {'Max':>10} {'Avg':>10}")
    for label, n, mn, mx, avg in stats:
        if n > 0:
            print(f"  {label:<12} {n:>5} {mn:>10.2f} {mx:>10.2f} {avg:>10.2f}")

# ── DeviceTemp for motor 21 ─────────────────────────────────────────
print("\n" + "=" * 80)
print("MOTOR 21 DeviceTemp — Full window")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "DeviceTemp" in name and "Fault" not in name and "Ancillary" not in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -2.0 <= t <= 10.0:
                print(f"  t={t:+7.3f}s  {v.value:.1f}°C")

print("\n" + "=" * 80)
print("MOTOR 21 AncillaryDeviceTemp — Full window")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "Ancillary" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -2.0 <= t <= 10.0:
                print(f"  t={t:+7.3f}s  {v.value:.1f}°C")

print("\n" + "=" * 80)
print("MOTOR 21 Fault_Hardware — All")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "Fault_Hardware" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -5.0 <= t <= 15.0:
                print(f"  t={t:+7.3f}s  {v.value}")

print("\n" + "=" * 80)
print("MOTOR 21 RobotEnable — All")
print("=" * 80)
for name in sorted(log.signal_names()):
    if "21" in name and "RobotEnable" in name:
        sig = log.signals[name]
        for v in sig.values:
            t = (v.timestamp_us - enable_time_us) / 1e6
            if -5.0 <= t <= 15.0:
                print(f"  t={t:+7.3f}s  {v.value}")

print("\nDONE")
