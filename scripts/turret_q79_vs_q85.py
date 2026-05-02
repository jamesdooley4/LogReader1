"""Compare Q79 (turret correct) vs Q85 (turret wrong) — power-on through autonomous.

Focus areas:
1. Turret position/target from boot to end of auto
2. Console and messages logs
3. DS joystick buttons/axes
4. Zero events, scheduler commands
5. Motor encoder state from hoot
"""
import sys, math
sys.path.insert(0, r"D:\Source\LogReader1\src")
from logreader.wpilog_reader import read_wpilog

Q79_WPILOG = r"D:\Temp\2026_DCMPs\FRC_20260411_005847_PNCMP_Q79.wpilog"
Q85_WPILOG = r"D:\Temp\2026_DCMPs\FRC_20260411_015124_PNCMP_Q85.wpilog"

print("Loading Q79 (good)...")
q79 = read_wpilog(Q79_WPILOG)
print("Loading Q85 (bad)...")
q85 = read_wpilog(Q85_WPILOG)


def get_sig(log, name):
    sig = log.signals.get(name)
    if sig is None:
        return [], []
    return [v.timestamp_us / 1e6 for v in sig.values], [v.value for v in sig.values]


def find_t0(log):
    """Find first enable time."""
    ts, vs = get_sig(log, "DS:enabled")
    for t, v in zip(ts, vs):
        if v:
            return t
    return 0


t0_79 = find_t0(q79)
t0_85 = find_t0(q85)
print(f"Q79 first enable: {t0_79:.3f}s")
print(f"Q85 first enable: {t0_85:.3f}s")


def print_section(title):
    print(f"\n{'='*90}")
    print(f"  {title}")
    print(f"{'='*90}")


# ---------------------------------------------------------------
# 1. Console log
# ---------------------------------------------------------------
print_section("CONSOLE LOG (from power-on to auto end)")
for label, log, t0 in [("Q79", q79, t0_79), ("Q85", q85, t0_85)]:
    ts, vs = get_sig(log, "console")
    print(f"\n--- {label} console ---")
    for t, v in zip(ts, vs):
        mt = t - t0
        if mt <= 25:  # through end of auto
            txt = v.strip() if isinstance(v, str) else str(v)
            if txt:
                print(f"  t={mt:+8.2f}s  {txt[:120]}")

# ---------------------------------------------------------------
# 2. Messages log
# ---------------------------------------------------------------
print_section("MESSAGES LOG (from power-on to auto end)")
for label, log, t0 in [("Q79", q79, t0_79), ("Q85", q85, t0_85)]:
    ts, vs = get_sig(log, "messages")
    print(f"\n--- {label} messages ---")
    for t, v in zip(ts, vs):
        mt = t - t0
        if mt <= 25:
            txt = v.strip() if isinstance(v, str) else str(v)
            if txt:
                print(f"  t={mt:+8.2f}s  {txt[:120]}")

# ---------------------------------------------------------------
# 3. Enable/Disable and Autonomous
# ---------------------------------------------------------------
print_section("ENABLE/DISABLE & AUTONOMOUS FLAGS")
for label, log, t0 in [("Q79", q79, t0_79), ("Q85", q85, t0_85)]:
    print(f"\n--- {label} ---")
    for sig_name in ["DS:enabled", "DS:autonomous", "DS:test", "DS:estop"]:
        ts, vs = get_sig(log, sig_name)
        for t, v in zip(ts, vs):
            mt = t - t0
            if -70 <= mt <= 30:
                print(f"  t={mt:+8.2f}s  {sig_name}={v}")

# ---------------------------------------------------------------
# 4. Zero events
# ---------------------------------------------------------------
print_section("TURRET ZERO EVENTS")
for label, log, t0 in [("Q79", q79, t0_79), ("Q85", q85, t0_85)]:
    ts, vs = get_sig(log, "NT:/Zero/turretZero")
    print(f"\n--- {label} ---")
    for t, v in zip(ts, vs):
        print(f"  t={t-t0:+8.2f}s  turretZero={v}")

# ---------------------------------------------------------------
# 5. Turret Position & Target (boot through auto)
# ---------------------------------------------------------------
print_section("TURRET POSITION & TARGET (every 1s, boot through auto)")
for label, log, t0 in [("Q79", q79, t0_79), ("Q85", q85, t0_85)]:
    pos_ts, pos_vs = get_sig(log, "NT:/SmartDashboard//Turret/Position")
    tgt_ts, tgt_vs = get_sig(log, "NT:/SmartDashboard//Turret/Target")
    print(f"\n--- {label} ---")
    print(f"  {'MTime':>8}  {'Pos(rot)':>10}  {'Pos(deg)':>9}  {'Tgt(rot)':>10}  {'Tgt(deg)':>9}")
    last = -999
    for i, (t, p) in enumerate(zip(pos_ts, pos_vs)):
        mt = t - t0
        if mt > 25:
            break
        if mt - last >= 1.0:
            # Find nearest target
            tgt = None
            for j, tt in enumerate(tgt_ts):
                if abs(tt - t) < 0.5:
                    tgt = tgt_vs[j]
                    break
            tgt_r = f"{tgt:.6f}" if tgt is not None else "     N/A"
            tgt_d = f"{tgt*360:.1f}" if tgt is not None else "    N/A"
            print(f"  {mt:+8.2f}s  {p:10.6f}  {p*360:9.1f}  {tgt_r:>10}  {tgt_d:>9}")
            last = mt

# ---------------------------------------------------------------
# 6. Scheduler commands (turret related)
# ---------------------------------------------------------------
print_section("TURRET SCHEDULER COMMANDS (boot through auto)")
for label, log, t0 in [("Q79", q79, t0_79), ("Q85", q85, t0_85)]:
    ts, vs = get_sig(log, "NT:/SmartDashboard/Scheduler/Names")
    print(f"\n--- {label} ---")
    prev = None
    for t, v in zip(ts, vs):
        mt = t - t0
        if mt > 25:
            break
        if isinstance(v, (list, tuple)):
            tcmds = [c for c in v if 'turret' in c.lower() or 'zero' in c.lower() or 'kill' in c.lower()]
        elif isinstance(v, str):
            tcmds = [v] if ('turret' in v.lower() or 'zero' in v.lower()) else []
        else:
            tcmds = []
        if tcmds != prev:
            print(f"  t={mt:+8.2f}s  {tcmds}")
            prev = tcmds

# ---------------------------------------------------------------
# 7. DS Joystick buttons & axes (all controllers, boot through auto)
# ---------------------------------------------------------------
print_section("DS JOYSTICK BUTTONS (non-zero, boot to auto end)")
for label, log, t0 in [("Q79", q79, t0_79), ("Q85", q85, t0_85)]:
    print(f"\n--- {label} ---")
    for joy_id in range(6):
        btn_name = f"DS:joystick{joy_id}/buttons"
        ts, vs = get_sig(log, btn_name)
        prev_val = None
        for t, v in zip(ts, vs):
            mt = t - t0
            if mt > 25:
                break
            if v != prev_val:
                print(f"  t={mt:+8.2f}s  {btn_name}={v}")
                prev_val = v

print_section("DS JOYSTICK AXES (non-zero, boot to auto end)")
for label, log, t0 in [("Q79", q79, t0_79), ("Q85", q85, t0_85)]:
    print(f"\n--- {label} ---")
    for joy_id in range(6):
        axes_name = f"DS:joystick{joy_id}/axes"
        ts, vs = get_sig(log, axes_name)
        prev_val = None
        for t, v in zip(ts, vs):
            mt = t - t0
            if mt > 25:
                break
            # Only print if any axis has significant value
            if isinstance(v, (list, tuple)):
                if any(abs(a) > 0.05 for a in v if isinstance(a, (int, float))):
                    if v != prev_val:
                        # Compact: just show non-zero axes
                        nonzero = {i: round(a, 3) for i, a in enumerate(v)
                                   if isinstance(a, (int, float)) and abs(a) > 0.05}
                        if nonzero:
                            print(f"  t={mt:+8.2f}s  {axes_name} nonzero={nonzero}")
                            prev_val = v

# ---------------------------------------------------------------
# 8. DS POVs
# ---------------------------------------------------------------
print_section("DS JOYSTICK POVS (non-default, boot to auto end)")
for label, log, t0 in [("Q79", q79, t0_79), ("Q85", q85, t0_85)]:
    print(f"\n--- {label} ---")
    for joy_id in range(6):
        pov_name = f"DS:joystick{joy_id}/povs"
        ts, vs = get_sig(log, pov_name)
        prev_val = None
        for t, v in zip(ts, vs):
            mt = t - t0
            if mt > 25:
                break
            if v != prev_val:
                # POV default is -1 or [-1]
                if isinstance(v, (list, tuple)):
                    if any(p != -1 for p in v):
                        print(f"  t={mt:+8.2f}s  {pov_name}={v}")
                elif v != -1:
                    print(f"  t={mt:+8.2f}s  {pov_name}={v}")
                prev_val = v

# ---------------------------------------------------------------
# 9. Robot pose at boot and start of auto
# ---------------------------------------------------------------
print_section("ROBOT POSE (first value and at auto start)")
for label, log, t0 in [("Q79", q79, t0_79), ("Q85", q85, t0_85)]:
    ts, vs = get_sig(log, "NT:/SmartDashboard/Field/Robot")
    print(f"\n--- {label} ---")
    if vs:
        v = vs[0]
        if isinstance(v, (list, tuple)) and len(v) >= 3:
            print(f"  First: t={ts[0]-t0:+.2f}s  x={v[0]:.4f}  y={v[1]:.4f}  hdg={v[2]:.2f}")
    for t, v in zip(ts, vs):
        mt = t - t0
        if -0.5 <= mt <= 0.5:
            if isinstance(v, (list, tuple)) and len(v) >= 3:
                print(f"  Enable: t={mt:+.2f}s  x={v[0]:.4f}  y={v[1]:.4f}  hdg={v[2]:.2f}")
                break
    # At t=+8
    for t, v in zip(ts, vs):
        mt = t - t0
        if 7.5 <= mt <= 8.5:
            if isinstance(v, (list, tuple)) and len(v) >= 3:
                print(f"  t=+8:  t={mt:+.2f}s  x={v[0]:.4f}  y={v[1]:.4f}  hdg={v[2]:.2f}")
                break

# ---------------------------------------------------------------
# 10. Starting position and auto mode
# ---------------------------------------------------------------
print_section("AUTO CONFIGURATION")
for label, log, t0 in [("Q79", q79, t0_79), ("Q85", q85, t0_85)]:
    print(f"\n--- {label} ---")
    for name in ["NT:/SmartDashboard/Starting Position/active",
                  "NT:/SmartDashboard/Auto Key",
                  "NT:/SmartDashboard/Available Auto Variants/active",
                  "NT:/FMSInfo/IsRedAlliance",
                  "NT:/FMSInfo/MatchNumber"]:
        sig = log.signals.get(name)
        if sig and sig.values:
            vals = set(v.value for v in sig.values)
            print(f"  {name}: {vals}")

print("\nDONE")
