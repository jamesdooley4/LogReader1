"""Investigate flywheel leader motor 21 anomalous current draw.

Reads the wpilog, dslog, and dsevents from the 2026-04-08 17:40 session
and extracts all signals relevant to the flywheel motor issue during the
first ~6s after enable.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from logreader.wpilog_reader import read_wpilog
from logreader.dslog_reader import read_ds_logs, read_dsevents

WPILOG = r"D:\Temp\2026-4-8_logs\FRC_20260408_174015.wpilog"
DSLOG = r"D:\Temp\2026-4-8_logs\DSLogs\2026_04_08 10_40_05 Wed.dslog"
DSEVENTS = r"D:\Temp\2026-4-8_logs\DSLogs\2026_04_08 10_40_05 Wed.dsevents"

# ── 1. WPILOG analysis ──────────────────────────────────────────────
print("=" * 80)
print("WPILOG Analysis:", WPILOG)
print("=" * 80)

wpi = read_wpilog(WPILOG)

# Find the log start (systemTime) and enable time
sys_time = wpi.get_signal("systemTime")
if sys_time:
    for v in sys_time.values[:5]:
        print(f"  systemTime: {v.timestamp_us} µs -> {v.value}")

ds_enabled = wpi.get_signal("DS:enabled")
if ds_enabled:
    print("\nDS:enabled transitions:")
    for v in ds_enabled.values:
        print(f"  t={v.timestamp_us} µs ({v.timestamp_us/1e6:.3f}s) -> {v.value}")

ds_auto = wpi.get_signal("DS:autonomous")
if ds_auto:
    print("\nDS:autonomous transitions:")
    for v in ds_auto.values:
        print(f"  t={v.timestamp_us} µs ({v.timestamp_us/1e6:.3f}s) -> {v.value}")

# Find first enable time
enable_time_us = None
if ds_enabled:
    for v in ds_enabled.values:
        if v.value is True:
            enable_time_us = v.timestamp_us
            break

if enable_time_us is None:
    print("WARNING: Could not find enable time!")
    enable_time_us = 0

print(f"\nFirst enable at: {enable_time_us} µs ({enable_time_us/1e6:.3f}s)")

# Define window: enable to enable+6s
window_start = enable_time_us
window_end = enable_time_us + 6_000_000  # 6 seconds

print(f"Investigation window: {window_start/1e6:.3f}s - {window_end/1e6:.3f}s")

# ── 2. Launcher/flywheel signals ────────────────────────────────────
print("\n" + "=" * 80)
print("LAUNCHER SIGNALS (first 10s from enable)")
print("=" * 80)

for sig_name in ["/launcher/current", "/launcher/velocity",
                 "NT:/AutoAim/flywheelGoal", "NT:/hood/goal",
                 "NT:/hood/position"]:
    sig = wpi.get_signal(sig_name)
    if sig is None:
        print(f"\n{sig_name}: NOT FOUND")
        continue

    # Filter to window
    pts = [v for v in sig.values
           if window_start - 1_000_000 <= v.timestamp_us <= window_end + 4_000_000]
    print(f"\n{sig_name}: {len(pts)} pts in window")
    if pts:
        vals = [v.value for v in pts if isinstance(v.value, (int, float))]
        if vals:
            print(f"  Min={min(vals):.3f}  Max={max(vals):.3f}")
        for v in pts[:50]:
            t_rel = (v.timestamp_us - enable_time_us) / 1e6
            print(f"  t={t_rel:+8.3f}s  val={v.value}")
        if len(pts) > 50:
            print(f"  ... ({len(pts) - 50} more points)")

# ── 3. Feeder/Spindexer currents ────────────────────────────────────
print("\n" + "=" * 80)
print("FEEDER & SPINDEXER CURRENTS (around enable)")
print("=" * 80)

for sig_name in ["/FeederLogs/statorCurrent", "/FeederLogs/supplyCurrent",
                 "/SpindexerLogs/statorCurrent", "/SpindexerLogs/supplyCurrent"]:
    sig = wpi.get_signal(sig_name)
    if sig is None:
        continue
    pts = [v for v in sig.values
           if window_start <= v.timestamp_us <= window_end]
    if pts:
        vals = [v.value for v in pts if isinstance(v.value, (int, float))]
        if vals:
            print(f"{sig_name}: {len(pts)} pts, min={min(vals):.1f}, max={max(vals):.1f}")

# ── 4. PDH channel currents ─────────────────────────────────────────
print("\n" + "=" * 80)
print("PDH CHANNEL CURRENTS (first 6s from enable)")
print("=" * 80)

pdh_prefix = "NT:/LiveWindow/Ungrouped/PowerDistribution[0]/"
for ch in range(24):
    sig_name = f"{pdh_prefix}Chan{ch}"
    sig = wpi.get_signal(sig_name)
    if sig is None:
        continue
    pts = [v for v in sig.values
           if window_start <= v.timestamp_us <= window_end]
    if not pts:
        continue
    vals = [v.value for v in pts if isinstance(v.value, (int, float))]
    if not vals:
        continue
    max_val = max(vals)
    avg_val = sum(vals) / len(vals)
    # Only show channels with significant current
    if max_val > 5.0:
        print(f"  Chan{ch:2d}: {len(pts):4d} pts  avg={avg_val:7.1f}A  max={max_val:7.1f}A")

# PDH total current and voltage
for sig_name in [f"{pdh_prefix}TotalCurrent", f"{pdh_prefix}Voltage"]:
    sig = wpi.get_signal(sig_name)
    if sig is None:
        continue
    pts = [v for v in sig.values
           if window_start <= v.timestamp_us <= window_end]
    if pts:
        vals = [v.value for v in pts if isinstance(v.value, (int, float))]
        if vals:
            print(f"  {sig_name.split('/')[-1]}: avg={sum(vals)/len(vals):.1f}  min={min(vals):.1f}  max={max(vals):.1f}")

# Show PDH details for high-current channels in the window
print("\n--- PDH High-Current Channel Details (>50A peaks) ---")
for ch in range(24):
    sig_name = f"{pdh_prefix}Chan{ch}"
    sig = wpi.get_signal(sig_name)
    if sig is None:
        continue
    pts = [v for v in sig.values
           if window_start <= v.timestamp_us <= window_end]
    if not pts:
        continue
    vals = [v.value for v in pts if isinstance(v.value, (int, float))]
    if not vals or max(vals) < 50.0:
        continue
    print(f"\n  Chan{ch} time series (max={max(vals):.1f}A):")
    for v in pts:
        t_rel = (v.timestamp_us - enable_time_us) / 1e6
        if isinstance(v.value, (int, float)):
            print(f"    t={t_rel:+8.3f}s  {v.value:7.1f}A")

# ── 5. Scheduler (active commands) ──────────────────────────────────
print("\n" + "=" * 80)
print("ACTIVE COMMANDS (Scheduler/Names) around enable")
print("=" * 80)

sched = wpi.get_signal("NT:/SmartDashboard/Scheduler/Names")
if sched:
    pts = [v for v in sched.values
           if window_start - 1_000_000 <= v.timestamp_us <= window_end]
    for v in pts[:30]:
        t_rel = (v.timestamp_us - enable_time_us) / 1e6
        print(f"  t={t_rel:+8.3f}s  commands={v.value}")

# ── 6. Console output ───────────────────────────────────────────────
print("\n" + "=" * 80)
print("CONSOLE OUTPUT (around enable, first 10s)")
print("=" * 80)

console = wpi.get_signal("console")
if console:
    pts = [v for v in console.values
           if window_start - 2_000_000 <= v.timestamp_us <= window_end + 4_000_000]
    print(f"Total console msgs in window: {len(pts)}")
    for v in pts:
        t_rel = (v.timestamp_us - enable_time_us) / 1e6
        msg = str(v.value).rstrip()
        print(f"  [{t_rel:+8.3f}s] {msg}")

# Also check messages signal
msgs = wpi.get_signal("messages")
if msgs:
    pts = [v for v in msgs.values
           if window_start - 2_000_000 <= v.timestamp_us <= window_end + 4_000_000]
    if pts:
        print(f"\nMESSAGES signal ({len(pts)} msgs in window):")
        for v in pts:
            t_rel = (v.timestamp_us - enable_time_us) / 1e6
            print(f"  [{t_rel:+8.3f}s] {v.value}")

# ── 7. DS Log analysis ──────────────────────────────────────────────
print("\n" + "=" * 80)
print("DSLOG Analysis:", DSLOG)
print("=" * 80)

try:
    dslog = read_ds_logs(DSLOG)
    print(f"DSLOG signals: {len(dslog.signals)}")
    for name in dslog.signal_names():
        sig = dslog.signals[name]
        print(f"  {name}: {len(sig.values)} pts")
    
    # Battery voltage
    for sig_name in dslog.signal_names():
        if "voltage" in sig_name.lower() or "battery" in sig_name.lower():
            sig = dslog.signals[sig_name]
            if sig.values:
                vals = [v.value for v in sig.values[:300] if isinstance(v.value, (int, float))]
                if vals:
                    print(f"\n  {sig_name} (first 6s): min={min(vals):.2f} max={max(vals):.2f} avg={sum(vals)/len(vals):.2f}")
                    # Show first 20 values
                    for v in sig.values[:20]:
                        print(f"    t={v.timestamp_us/1e6:.3f}s  {v.value}")
    
    # Show pdp currents if available
    for sig_name in dslog.signal_names():
        if "pdp" in sig_name.lower() or "current" in sig_name.lower():
            sig = dslog.signals[sig_name]
            vals = [v.value for v in sig.values[:300] if isinstance(v.value, (int, float))]
            if vals and max(vals) > 10:
                print(f"\n  {sig_name} (first 6s): max={max(vals):.1f}")

except Exception as e:
    print(f"Error reading dslog: {e}")

# ── 8. DS Events ────────────────────────────────────────────────────
print("\n" + "=" * 80)
print("DSEVENTS:", DSEVENTS)
print("=" * 80)

try:
    dsev = read_dsevents(DSEVENTS)
    for name in dsev.signal_names():
        sig = dsev.signals[name]
        print(f"\n{name}: {len(sig.values)} entries")
        for v in sig.values[:50]:
            ts = v.timestamp_us / 1e6
            print(f"  [{ts:.3f}s] {v.value}")
except Exception as e:
    print(f"Error reading dsevents: {e}")

print("\n" + "=" * 80)
print("INVESTIGATION COMPLETE")
print("=" * 80)
