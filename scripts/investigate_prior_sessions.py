"""Investigate prior session logs for flywheel motor 21 issue.

Checks the 3 wpilog files, 3 dsevents files, and the hoot folder from
the session immediately before the known-bad 17:40 session.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
os.environ["OWLET_PATH"] = r"c:\Tools\owlet-26.1.0-windowsx86-64.exe"

from logreader.wpilog_reader import read_wpilog
from logreader.dslog_reader import read_dsevents

WPILOG_FILES = [
    r"D:\Temp\2026-4-8_logs\FRC_20260408_173056.wpilog",
    r"D:\Temp\2026-4-8_logs\FRC_20260408_173135.wpilog",
    r"D:\Temp\2026-4-8_logs\FRC_20260408_173934.wpilog",
]

DSEVENTS_FILES = [
    r"D:\Temp\2026-4-8_logs\DSLogs\2026_04_08 10_30_51 Wed.dsevents",
    r"D:\Temp\2026-4-8_logs\DSLogs\2026_04_08 10_31_24 Wed.dsevents",
    r"D:\Temp\2026-4-8_logs\DSLogs\2026_04_08 10_39_32 Wed.dsevents",
]


def analyze_wpilog(path: str) -> None:
    fname = Path(path).name
    print(f"\n{'=' * 80}")
    print(f"WPILOG: {fname}")
    print(f"{'=' * 80}")

    try:
        wpi = read_wpilog(path)
    except Exception as e:
        print(f"  ERROR reading: {e}")
        return

    # Duration info
    sys_time = wpi.get_signal("systemTime")
    if sys_time and sys_time.values:
        print(f"  systemTime entries: {len(sys_time.values)}")
        for v in sys_time.values[:3]:
            print(f"    t={v.timestamp_us/1e6:.3f}s  epoch={v.value}")

    # Enable transitions
    ds_enabled = wpi.get_signal("DS:enabled")
    if ds_enabled:
        print(f"\n  DS:enabled transitions:")
        for v in ds_enabled.values:
            print(f"    t={v.timestamp_us/1e6:.3f}s -> {v.value}")

    ds_auto = wpi.get_signal("DS:autonomous")
    if ds_auto:
        print(f"\n  DS:autonomous transitions:")
        for v in ds_auto.values:
            print(f"    t={v.timestamp_us/1e6:.3f}s -> {v.value}")

    # Find enable times
    enable_times = []
    if ds_enabled:
        for v in ds_enabled.values:
            if v.value is True:
                enable_times.append(v.timestamp_us)

    if not enable_times:
        print("  No enable events found.")
        return

    # For each enable period, check flywheel-related signals
    for i, enable_us in enumerate(enable_times):
        window_end = enable_us + 8_000_000
        print(f"\n  --- Enable #{i+1} at {enable_us/1e6:.3f}s ---")

        # Launcher signals
        for sig_name in ["/launcher/current", "/launcher/velocity",
                         "NT:/AutoAim/flywheelGoal"]:
            sig = wpi.get_signal(sig_name)
            if sig is None:
                continue
            pts = [v for v in sig.values
                   if enable_us - 500_000 <= v.timestamp_us <= window_end]
            if not pts:
                continue
            vals = [v.value for v in pts if isinstance(v.value, (int, float))]
            if vals:
                print(f"    {sig_name}: {len(pts)} pts, min={min(vals):.1f}, max={max(vals):.1f}, avg={sum(vals)/len(vals):.1f}")

        # PDH Chan14 (flywheel channel)
        pdh14 = wpi.get_signal("NT:/LiveWindow/Ungrouped/PowerDistribution[0]/Chan14")
        if pdh14:
            pts = [v for v in pdh14.values
                   if enable_us <= v.timestamp_us <= window_end]
            if pts:
                vals = [v.value for v in pts if isinstance(v.value, (int, float))]
                if vals:
                    print(f"    PDH Chan14: {len(pts)} pts, min={min(vals):.1f}A, max={max(vals):.1f}A, avg={sum(vals)/len(vals):.1f}A")

        # PDH voltage
        pdh_v = wpi.get_signal("NT:/LiveWindow/Ungrouped/PowerDistribution[0]/Voltage")
        if pdh_v:
            pts = [v for v in pdh_v.values
                   if enable_us <= v.timestamp_us <= window_end]
            if pts:
                vals = [v.value for v in pts if isinstance(v.value, (int, float))]
                if vals:
                    print(f"    PDH Voltage: min={min(vals):.2f}V, max={max(vals):.2f}V, avg={sum(vals)/len(vals):.2f}V")

        # PDH total current
        pdh_tc = wpi.get_signal("NT:/LiveWindow/Ungrouped/PowerDistribution[0]/TotalCurrent")
        if pdh_tc:
            pts = [v for v in pdh_tc.values
                   if enable_us <= v.timestamp_us <= window_end]
            if pts:
                vals = [v.value for v in pts if isinstance(v.value, (int, float))]
                if vals:
                    print(f"    PDH TotalCurrent: max={max(vals):.1f}A, avg={sum(vals)/len(vals):.1f}A")

        # Feeder/Spindexer currents (proxies for launch activity)
        for sig_name in ["/FeederLogs/statorCurrent", "/SpindexerLogs/statorCurrent"]:
            sig = wpi.get_signal(sig_name)
            if sig is None:
                continue
            pts = [v for v in sig.values
                   if enable_us <= v.timestamp_us <= window_end]
            if pts:
                vals = [v.value for v in pts if isinstance(v.value, (int, float))]
                if vals and max(vals) > 5:
                    print(f"    {sig_name}: max={max(vals):.1f}A")

        # Scheduler names - look for Launching Command
        sched = wpi.get_signal("NT:/SmartDashboard/Scheduler/Names")
        if sched:
            pts = [v for v in sched.values
                   if enable_us - 500_000 <= v.timestamp_us <= window_end]
            launching_pts = [v for v in pts
                             if isinstance(v.value, list) and any("Launching" in s for s in v.value)]
            if launching_pts:
                print(f"    Launching Command active: {len(launching_pts)} scheduler snapshots")
                # Show first and last
                t0 = (launching_pts[0].timestamp_us - enable_us) / 1e6
                t1 = (launching_pts[-1].timestamp_us - enable_us) / 1e6
                print(f"      First at t={t0:+.3f}s, last at t={t1:+.3f}s")

        # Console messages around enable (first 5s) - look for errors/warnings
        console = wpi.get_signal("console")
        if console:
            pts = [v for v in console.values
                   if enable_us - 500_000 <= v.timestamp_us <= enable_us + 5_000_000]
            # Filter for interesting messages
            interesting = []
            for v in pts:
                msg = str(v.value).strip()
                if any(kw in msg.lower() for kw in ["error", "fault", "warning", "exception",
                                                      "current", "flywheel", "launcher",
                                                      "motor", "brownout", "disabled"]):
                    interesting.append((v.timestamp_us, msg))
            if interesting:
                print(f"    Interesting console msgs ({len(interesting)}):")
                for ts, msg in interesting[:10]:
                    t = (ts - enable_us) / 1e6
                    # Truncate long messages
                    if len(msg) > 120:
                        msg = msg[:120] + "..."
                    print(f"      [{t:+.3f}s] {msg}")

        # Messages signal
        msgs = wpi.get_signal("messages")
        if msgs:
            pts = [v for v in msgs.values
                   if enable_us - 500_000 <= v.timestamp_us <= window_end]
            launch_msgs = [v for v in pts if "Launching" in str(v.value) or "launch" in str(v.value).lower()]
            if launch_msgs:
                print(f"    Launch-related messages:")
                for v in launch_msgs:
                    t = (v.timestamp_us - enable_us) / 1e6
                    print(f"      [{t:+.3f}s] {v.value}")

        # Hood signals (to see if launcher was aiming)
        hood_goal = wpi.get_signal("NT:/hood/goal")
        if hood_goal:
            pts = [v for v in hood_goal.values
                   if enable_us <= v.timestamp_us <= window_end]
            if pts:
                vals = [v.value for v in pts if isinstance(v.value, (int, float))]
                if vals and max(vals) > 0.5:
                    print(f"    Hood goal: min={min(vals):.1f}, max={max(vals):.1f} (launcher was aiming)")

    # Total signal count and log duration
    all_ts = []
    for name in wpi.signal_names():
        sig = wpi.signals[name]
        if sig.values:
            all_ts.append(sig.values[0].timestamp_us)
            all_ts.append(sig.values[-1].timestamp_us)
    if all_ts:
        duration = (max(all_ts) - min(all_ts)) / 1e6
        print(f"\n  Log duration: {duration:.1f}s, {len(wpi.signals)} signals")


def analyze_dsevents(path: str) -> None:
    fname = Path(path).name
    print(f"\n{'=' * 80}")
    print(f"DSEVENTS: {fname}")
    print(f"{'=' * 80}")

    try:
        dsev = read_dsevents(path)
    except Exception as e:
        print(f"  ERROR: {e}")
        return

    for name in dsev.signal_names():
        sig = dsev.signals[name]
        print(f"  {len(sig.values)} entries")

        # Show all entries, look for errors/interesting items
        for v in sig.values:
            msg = str(v.value).strip()
            ts = v.timestamp_us / 1e6

            # Skip verbose NT connection/tracer noise unless it's an error
            skip_patterns = ["NT: Got a NT4", "NT: CONNECTED", "NT: DISCONNECTED",
                             "NT: NT4 socket", "Tracer.lambda", "periodic():",
                             ".execute():", "buttons.run():", ".initialize():",
                             "SmartDashboard.updateValues", "robotPeriodic()",
                             "LiveWindow.updateValues", "Shuffleboard.update",
                             "teleopPeriodic():", "disabledPeriodic():",
                             "CommandScheduler loop overrun",
                             "Loop time of 0.02s overrun",
                             "IndexerSubsystem.periodic", "VisionSubsystem.periodic",
                             "IntakeSubsystem.periodic", "LEDSubsystem.periodic",
                             "CommandSwerveDrivetrain.periodic",
                             "Spindexer.periodic", "Hood.periodic",
                             "TurretSubsystem.periodic", "IntakePivot.periodic",
                             "Feeder.periodic", "IntakeRollers.periodic",
                             "LauncherSubsystem.periodic", "Flywheels.periodic",
                             "Drive.execute", "Flash LEDs", "LED Default",
                             "Set Turret Position", "Intake Default",
                             "disabledInit():", "teleopInit():"]
            if any(p in msg for p in skip_patterns):
                continue

            # Show remaining messages
            if msg:
                if len(msg) > 200:
                    msg = msg[:200] + "..."
                print(f"    [{ts:.3f}s] {msg}")


# ── Main ────────────────────────────────────────────────────────────
for path in WPILOG_FILES:
    analyze_wpilog(path)

print("\n\n" + "#" * 80)
print("# DSEVENTS FILES")
print("#" * 80)

for path in DSEVENTS_FILES:
    analyze_dsevents(path)

print("\nDONE")
