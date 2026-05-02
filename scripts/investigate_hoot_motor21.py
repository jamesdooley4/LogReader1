"""Extract motor 20 (follower) and 21 (leader) telemetry from hoot file.

Converts only the specific motor signals from the roboRIO hoot file,
then analyzes the first 10s from enable.
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

# Signal IDs for motors 20 and 21 (from scan)
SIGNAL_IDS = [
    # Motor 21 (leader) - key telemetry
    "6ec1500",   # MotorVoltage
    "6f21500",   # TorqueCurrent
    "6f31500",   # StatorCurrent
    "6f41500",   # SupplyCurrent
    "6f51500",   # SupplyVoltage
    "6f61500",   # DeviceTemp
    "6f91500",   # RotorVelocity
    "6fd1500",   # Velocity
    "6fe1500",   # Position
    "7071500",   # ControlMode
    "70f1500",   # RobotEnable
    "7131500",   # PIDVelocity_Reference
    "7151500",   # PIDVelocity_ClosedLoopError
    "7201500",   # PIDMotorVoltage_Output
    "7051500",   # PIDMotorVoltage_FeedForward
    "71a1500",   # PIDMotorVoltage_ProportionalOutput
    "71d1500",   # PIDMotorVoltage_DerivativeOutput
    "7021500",   # PIDMotorVoltage_IntegratedAccum
    "7ab1500",   # BridgeOutput
    "82b1500",   # AncillaryDeviceTemp
    "276b1500",  # Fault_StatorCurrLimit
    "276c1500",  # StickyFault_StatorCurrLimit
    "276e1500",  # Fault_SupplyCurrLimit
    "276f1500",  # StickyFault_SupplyCurrLimit
    "27111500",  # Fault_Hardware
    "27171500",  # Fault_DeviceTemp
    "27411500",  # Fault_BridgeBrownout
    "274d1500",  # Fault_OverSupplyV
    "27501500",  # Fault_UnstableSupplyV
    # Motor 20 (follower) - key telemetry
    "6ec1400",   # MotorVoltage
    "6f21400",   # TorqueCurrent
    "6f31400",   # StatorCurrent
    "6f41400",   # SupplyCurrent
    "6f51400",   # SupplyVoltage
    "6f61400",   # DeviceTemp
    "6fd1400",   # Velocity
    "7071400",   # ControlMode
    "70f1400",   # RobotEnable
    "7ab1400",   # BridgeOutput
    "276b1400",  # Fault_StatorCurrLimit
    "276e1400",  # Fault_SupplyCurrLimit
    "27111400",  # Fault_Hardware
]

# Convert with targeted signals
out_path = Path(tempfile.mkdtemp()) / "motor_21_extract.wpilog"
cmd = [owlet, HOOT_RIO, str(out_path), "-f", "wpilog"]
for sid in SIGNAL_IDS:
    cmd.extend(["-s", sid])

print(f"Converting hoot -> wpilog ({len(SIGNAL_IDS)} signals)...")
result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
if result.returncode != 0:
    print(f"STDERR: {result.stderr[:2000]}")
if result.stdout:
    print(result.stdout[:1000])
print(f"Output: {out_path} ({out_path.stat().st_size / 1024:.0f} KB)")

# Read the converted wpilog
from logreader.wpilog_reader import read_wpilog

log = read_wpilog(str(out_path))

# List signals
print("\n" + "=" * 80)
print("AVAILABLE SIGNALS:")
print("=" * 80)
for name in sorted(log.signal_names()):
    sig = log.signals[name]
    print(f"  {name}: {len(sig.values)} pts")

# Find enable time from RobotEnable signal
enable_time_us = None
for name in log.signal_names():
    if "RobotEnable" in name and "21" in name:
        sig = log.signals[name]
        for v in sig.values:
            if v.value == 1 or v.value is True or v.value == "Enabled":
                enable_time_us = v.timestamp_us
                break
        break

if enable_time_us is None:
    # Try any RobotEnable
    for name in log.signal_names():
        if "RobotEnable" in name:
            sig = log.signals[name]
            print(f"\n{name} values (first 20):")
            for v in sig.values[:20]:
                print(f"  t={v.timestamp_us/1e6:.3f}s  val={v.value}")
            for v in sig.values:
                val = v.value
                if val == 1 or val is True or (isinstance(val, str) and "enabl" in val.lower()):
                    enable_time_us = v.timestamp_us
                    break
            if enable_time_us:
                break

if enable_time_us is None:
    print("\nWARNING: Could not find enable time from hoot. Using wpilog enable time.")
    enable_time_us = 40936884  # From previous analysis

print(f"\nEnable time: {enable_time_us} µs ({enable_time_us/1e6:.3f}s)")

# Analysis window: -1s to +10s from enable
window_start = enable_time_us - 1_000_000
window_end = enable_time_us + 10_000_000

# ── Motor 21 (Leader) Analysis ──────────────────────────────────────
print("\n" + "=" * 80)
print("MOTOR 21 (FLYWHEEL LEADER) - First 10s from enable")
print("=" * 80)

for name in sorted(log.signal_names()):
    if "21" not in name and "TalonFX-21" not in name:
        continue
    sig = log.signals[name]
    pts = [v for v in sig.values if window_start <= v.timestamp_us <= window_end]
    if not pts:
        continue
    
    short_name = name.split("/")[-1] if "/" in name else name
    
    # For numeric signals, show stats
    vals = [v.value for v in pts if isinstance(v.value, (int, float))]
    if vals:
        print(f"\n{name}: {len(pts)} pts")
        print(f"  min={min(vals):.3f}  max={max(vals):.3f}  avg={sum(vals)/len(vals):.3f}")
        
        # For key signals, print time series
        key_signals = ["StatorCurrent", "SupplyCurrent", "Velocity", "MotorVoltage",
                       "TorqueCurrent", "PIDVelocity_Reference", "PIDVelocity_ClosedLoopError",
                       "PIDMotorVoltage_Output", "BridgeOutput", "ControlMode",
                       "Fault_StatorCurrLimit", "StickyFault_StatorCurrLimit",
                       "Fault_SupplyCurrLimit", "DeviceTemp", "SupplyVoltage",
                       "PIDMotorVoltage_FeedForward", "PIDMotorVoltage_ProportionalOutput"]
        if any(k in name for k in key_signals):
            # Print every point for first 6s, then every 10th
            count = 0
            for v in pts:
                t_rel = (v.timestamp_us - enable_time_us) / 1e6
                if t_rel <= 6.5 or count % 10 == 0:
                    print(f"    t={t_rel:+8.3f}s  {v.value}")
                count += 1
    else:
        # Non-numeric (string/boolean)
        print(f"\n{name}: {len(pts)} pts")
        for v in pts[:30]:
            t_rel = (v.timestamp_us - enable_time_us) / 1e6
            print(f"    t={t_rel:+8.3f}s  {v.value}")

# ── Motor 20 (Follower) Analysis ────────────────────────────────────
print("\n" + "=" * 80)
print("MOTOR 20 (FLYWHEEL FOLLOWER) - First 10s from enable")
print("=" * 80)

for name in sorted(log.signal_names()):
    if "20" not in name and "TalonFX-20" not in name:
        continue
    sig = log.signals[name]
    pts = [v for v in sig.values if window_start <= v.timestamp_us <= window_end]
    if not pts:
        continue
    
    vals = [v.value for v in pts if isinstance(v.value, (int, float))]
    if vals:
        print(f"\n{name}: {len(pts)} pts")
        print(f"  min={min(vals):.3f}  max={max(vals):.3f}  avg={sum(vals)/len(vals):.3f}")
        
        key_signals = ["StatorCurrent", "SupplyCurrent", "Velocity", "MotorVoltage",
                       "BridgeOutput", "Fault_StatorCurrLimit", "DeviceTemp"]
        if any(k in name for k in key_signals):
            count = 0
            for v in pts:
                t_rel = (v.timestamp_us - enable_time_us) / 1e6
                if t_rel <= 6.5 or count % 10 == 0:
                    print(f"    t={t_rel:+8.3f}s  {v.value}")
                count += 1
    else:
        print(f"\n{name}: {len(pts)} pts")
        for v in pts[:20]:
            t_rel = (v.timestamp_us - enable_time_us) / 1e6
            print(f"    t={t_rel:+8.3f}s  {v.value}")

print("\n" + "=" * 80)
print("DONE")
print("=" * 80)
