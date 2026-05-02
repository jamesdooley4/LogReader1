"""Compare PIDPosition_Reference (hoot) vs SmartDashboard/Turret/Target (wpilog).

Q36: discrepancy at 21-25s, 98-103s, 114-115s into teleop.
Hoot has Phoenix6 signals; wpilog has NT signals. Different time bases.
Align via TalonFX-23/Position (hoot) ≡ NT:/SmartDashboard//Turret/Position (wpilog).
"""

import bisect
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from logreader.wpilog_reader import read_wpilog

HOOT_WPILOG = r"D:\Temp\2026_DCMPs\PNCMP_Q36_motor23.wpilog"
WPILOG = r"D:\Temp\2026_DCMPs\FRC_20260410_164824_PNCMP_Q36.wpilog"

print("Reading hoot-converted wpilog...")
hoot = read_wpilog(HOOT_WPILOG)
print("Reading match wpilog...")
wpi = read_wpilog(WPILOG)

# Get signals
pid_ref = hoot.signals["Phoenix6/TalonFX-23/PIDPosition_Reference"]
hoot_pos = hoot.signals["Phoenix6/TalonFX-23/Position"]
hoot_ctrl = hoot.signals.get("Phoenix6/TalonFX-23/ControlMode")
hoot_enable = hoot.signals.get("Phoenix6/TalonFX-23/RobotEnable")
hoot_dev_enable = hoot.signals.get("Phoenix6/TalonFX-23/DeviceEnable")

nt_target = wpi.signals["NT:/SmartDashboard//Turret/Target"]
nt_pos = wpi.signals["NT:/SmartDashboard//Turret/Position"]
ds_enabled = wpi.signals.get("DS:enabled")
fms_ctrl = wpi.signals.get("NT:/FMSInfo/FMSControlData")

# ── Time bases ────────────────────────────────────────────────────────
print(f"\nHoot: {pid_ref.values[0].timestamp_us/1e6:.3f}s – {pid_ref.values[-1].timestamp_us/1e6:.3f}s  ({len(pid_ref.values)} pts)")
print(f"WPI:  {nt_target.values[0].timestamp_us/1e6:.3f}s – {nt_target.values[-1].timestamp_us/1e6:.3f}s  ({len(nt_target.values)} pts)")

# ── Match timeline from wpilog ────────────────────────────────────────
if ds_enabled:
    print("\nMatch timeline (DS:enabled):")
    for v in ds_enabled.values:
        print(f"  {v.timestamp_us/1e6:8.3f}s  enabled={v.value}")

# HALControlWord bits: 0=enabled, 1=autonomous, 4=FMS, 5=DS
if fms_ctrl:
    print("\nFMSControlData:")
    for v in fms_ctrl.values:
        en = bool(v.value & 1)
        auto = bool(v.value & 2)
        mode = "auto" if auto else "teleop"
        print(f"  {v.timestamp_us/1e6:8.3f}s  raw={v.value:3d} (0b{v.value:08b})  {mode} {'ENABLED' if en else 'disabled'}")

# Determine teleop start from DS:enabled
# auto enabled at ~137.5s, disabled ~158.1s, teleop enabled ~161.9s
teleop_start_wpi = None
if ds_enabled:
    enables = [(v.timestamp_us, v.value) for v in ds_enabled.values]
    # Third transition should be teleop enable
    enable_events = [(t, v) for t, v in enables if v]
    if len(enable_events) >= 2:
        teleop_start_wpi = enable_events[1][0]
        print(f"\nTeleop start (wpilog): {teleop_start_wpi/1e6:.3f}s")

# ── Time alignment via position signal ────────────────────────────────
# Find when hoot position first changes significantly (robot enabled)
hoot_first_move = None
for i, v in enumerate(hoot_pos.values):
    if i > 0 and abs(v.value - hoot_pos.values[0].value) > 0.001:
        hoot_first_move = v.timestamp_us
        break

wpi_first_move = None
for i, v in enumerate(nt_pos.values):
    if i > 0 and abs(v.value - nt_pos.values[0].value) > 0.001:
        wpi_first_move = v.timestamp_us
        break

if hoot_first_move and wpi_first_move:
    time_offset = wpi_first_move - hoot_first_move  # add to hoot time to get wpilog time
    print(f"\nTime alignment (first position change):")
    print(f"  Hoot first move: {hoot_first_move/1e6:.3f}s")
    print(f"  WPI first move:  {wpi_first_move/1e6:.3f}s")
    print(f"  Offset (wpi - hoot): {time_offset/1e6:.3f}s")
else:
    # Fallback: align on robot enable
    print("\nCould not align on position change, using enable events")
    # Find hoot robot enable
    hoot_enable_time = None
    if hoot_enable:
        for v in hoot_enable.values:
            if v.value == 1 or v.value == True:
                hoot_enable_time = v.timestamp_us
                break
    wpi_enable_time = teleop_start_wpi or 0
    if hoot_enable_time:
        time_offset = wpi_enable_time - hoot_enable_time
    else:
        time_offset = 0
    print(f"  Offset: {time_offset/1e6:.3f}s")

# ── Check RobotEnable signal in hoot ─────────────────────────────────
if hoot_enable:
    print(f"\nHoot RobotEnable transitions:")
    prev = None
    for v in hoot_enable.values:
        if prev is None or v.value != prev:
            print(f"  {v.timestamp_us/1e6:8.3f}s  enable={v.value}")
            prev = v.value

if hoot_dev_enable:
    print(f"\nHoot DeviceEnable transitions:")
    prev = None
    for v in hoot_dev_enable.values:
        if prev is None or v.value != prev:
            print(f"  {v.timestamp_us/1e6:8.3f}s  dev_enable={v.value}")
            prev = v.value

# ── Show PID ref vs NT target using aligned time ─────────────────────
def to_wpi_time(hoot_us):
    return hoot_us + time_offset

# Build lookup for NT target
tgt_times = [v.timestamp_us for v in nt_target.values]
tgt_vals = [v.value for v in nt_target.values]

def get_nt_target(wpi_time_us):
    idx = bisect.bisect_right(tgt_times, wpi_time_us) - 1
    if idx < 0:
        return None
    return tgt_vals[idx]

# ── Find discrepancy windows ─────────────────────────────────────────
print(f"\n{'='*70}")
print(f"Discrepancy analysis (threshold > 2°)")
print(f"{'='*70}")

THRESHOLD = 2.0 / 360.0  # 2 degrees in rotations
disc_windows = []
in_disc = False
disc_start = None

for v in pid_ref.values:
    wt = to_wpi_time(v.timestamp_us)
    nt_val = get_nt_target(wt)
    if nt_val is None:
        continue
    diff = abs(v.value - nt_val)
    if diff > THRESHOLD:
        if not in_disc:
            in_disc = True
            disc_start = v.timestamp_us
    else:
        if in_disc:
            in_disc = False
            disc_windows.append((disc_start, v.timestamp_us))

if in_disc:
    disc_windows.append((disc_start, pid_ref.values[-1].timestamp_us))

if teleop_start_wpi:
    teleop_start_hoot = teleop_start_wpi - time_offset
else:
    teleop_start_hoot = 0

print(f"\nFound {len(disc_windows)} discrepancy windows:")
for i, (s, e) in enumerate(disc_windows):
    dur = (e - s) / 1e6
    wpi_s = to_wpi_time(s) / 1e6
    wpi_e = to_wpi_time(e) / 1e6
    teleop_s = (s - teleop_start_hoot) / 1e6 if teleop_start_hoot else 0
    teleop_e = (e - teleop_start_hoot) / 1e6 if teleop_start_hoot else 0
    print(f"  [{i+1:2d}] hoot {s/1e6:7.1f}s–{e/1e6:7.1f}s  wpi {wpi_s:7.1f}s–{wpi_e:7.1f}s"
          f"  teleop {teleop_s:6.1f}s–{teleop_e:6.1f}s  ({dur:.1f}s)")

# ── Detailed comparison for first few windows ────────────────────────
for i, (s, e) in enumerate(disc_windows[:8]):
    w_start = s - 1_000_000
    w_end = e + 1_000_000
    pts = [v for v in pid_ref.values if w_start <= v.timestamp_us <= w_end]
    step = max(1, len(pts) // 30)

    teleop_ref = teleop_start_hoot if teleop_start_hoot else 0
    print(f"\n--- Window {i+1}: teleop+{(s-teleop_ref)/1e6:.1f}s – teleop+{(e-teleop_ref)/1e6:.1f}s ---")
    print(f"  {'teleop_s':>10}  {'PID_ref':>10}  {'NT_tgt':>10}  {'diff°':>10}  {'ctrl':>6}")

    ctrl_times = [v.timestamp_us for v in hoot_ctrl.values] if hoot_ctrl else []
    ctrl_vals = [v.value for v in hoot_ctrl.values] if hoot_ctrl else []

    for v in pts[::step]:
        wt = to_wpi_time(v.timestamp_us)
        nt_val = get_nt_target(wt)
        if nt_val is None:
            nt_val = float('nan')
        diff_deg = (v.value - nt_val) * 360

        ctrl_val = ""
        if ctrl_times:
            ci = bisect.bisect_right(ctrl_times, v.timestamp_us) - 1
            if ci >= 0:
                ctrl_val = str(ctrl_vals[ci])

        teleop_t = (v.timestamp_us - teleop_ref) / 1e6
        print(f"  {teleop_t:10.3f}  {v.value:10.6f}  {nt_val:10.6f}  {diff_deg:10.2f}  {ctrl_val:>6}")

# ── Also compare actual position ─────────────────────────────────────
wpi_pos_times = [v.timestamp_us for v in nt_pos.values]
wpi_pos_vals = [v.value for v in nt_pos.values]
hoot_pos_times = [v.timestamp_us for v in hoot_pos.values]
hoot_pos_vals = [v.value for v in hoot_pos.values]

print(f"\n{'='*70}")
print(f"Position tracking: does actual position follow PID_ref or NT_target?")
print(f"{'='*70}")

for i, (s, e) in enumerate(disc_windows[:5]):
    w_start = s - 500_000
    w_end = e + 500_000
    pts = [v for v in pid_ref.values if w_start <= v.timestamp_us <= w_end]
    step = max(1, len(pts) // 15)

    teleop_ref = teleop_start_hoot if teleop_start_hoot else 0
    print(f"\n  Window {i+1} (teleop+{(s-teleop_ref)/1e6:.1f}s):")
    print(f"  {'teleop_s':>10}  {'PID_ref':>10}  {'NT_tgt':>10}  {'hoot_pos':>10}  {'tracks':>10}")

    for v in pts[::step]:
        wt = to_wpi_time(v.timestamp_us)
        nt_val = get_nt_target(wt)
        if nt_val is None:
            nt_val = float('nan')

        hi = bisect.bisect_right(hoot_pos_times, v.timestamp_us) - 1
        hp = hoot_pos_vals[hi] if hi >= 0 else float('nan')

        d_ref = abs(hp - v.value) if hp == hp else float('nan')
        d_tgt = abs(hp - nt_val) if hp == hp and nt_val == nt_val else float('nan')
        tracks = "PID_ref" if d_ref < d_tgt else "NT_tgt"

        teleop_t = (v.timestamp_us - teleop_ref) / 1e6
        print(f"  {teleop_t:10.3f}  {v.value:10.6f}  {nt_val:10.6f}  {hp:10.6f}  {tracks:>10}")

# ── ControlMode during discrepancies ─────────────────────────────────
if hoot_ctrl:
    print(f"\n{'='*70}")
    print(f"ControlMode transitions in hoot (0=DutyCycle,1=TorqueCurrent,")
    print(f"  2=Voltage,3=Position,4=Velocity,5=MotionMagic,...):")
    print(f"{'='*70}")
    prev = None
    for v in hoot_ctrl.values:
        if prev is None or v.value != prev:
            teleop_t = (v.timestamp_us - teleop_start_hoot) / 1e6 if teleop_start_hoot else v.timestamp_us/1e6
            print(f"  teleop+{teleop_t:8.3f}s  mode={v.value}")
            prev = v.value
