"""Analyze flywheel launch motor behavior from practice session.

Motors of interest:
- 20, 21: Flywheel (mechanically linked)
- 30: Feeder
- 31: Spindexer

Strategy:
1. Read enable windows from RIO (smaller file, has boolean RobotEnable)
2. Read key motor signals from RIO (motors 20, 21, 31) and CANivore (motor 30)
   during enabled windows only
3. Per each enabled window: analyze velocity, current, duty cycle behavior
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from wpiutil.log import DataLogReader

HOOT_DIR = r"D:\Temp\2026_DCMPs\2026-04-09_PNCMP_practice\2026-04-10_01-40-06\converted"
CANIVORE_FILE = os.path.join(HOOT_DIR, "ECA0983C3353385320202034170A03FF_2026-04-10_01-40-11.wpilog")
RIO_FILE = os.path.join(HOOT_DIR, "rio_2026-04-10_01-40-11.wpilog")

# Signals we actually care about for analysis
KEY_SIGNAL_SUFFIXES = [
    'Velocity', 'DutyCycle', 'MotorVoltage', 'StatorCurrent',
    'SupplyCurrent', 'SupplyVoltage', 'TorqueCurrent', 'ControlMode',
    'Position', 'DeviceEnable',
    'PIDVelocity_Reference', 'PIDVelocity_ClosedLoopError',
    'PIDMotorVoltage_Output', 'PIDMotorVoltage_FeedForward',
    'PIDMotorVoltage_ProportionalOutput',
]

GETTER_MAP = {
    "double": "getDouble",
    "float": "getFloat",
    "int64": "getInteger",
    "boolean": "getBoolean",
    "string": "getString",
    "double[]": "getDoubleArray",
}


def discover_signals(filepath):
    """Read only control records to discover signal names and types."""
    reader = DataLogReader(filepath)
    entries = {}
    for record in reader:
        if record.isControl() and record.isStart():
            start = record.getStartData()
            entries[start.entry] = (start.name, start.type)
    return entries


def read_enable_windows(filepath, entries):
    """Read the boolean RobotEnable signal and return enabled time windows."""
    # Find the boolean RobotEnable
    enable_eid = None
    for eid, (name, type_str) in entries.items():
        if name == 'RobotEnable' and type_str == 'boolean':
            enable_eid = eid
            break
    
    if enable_eid is None:
        raise ValueError("No boolean RobotEnable signal found")
    
    reader = DataLogReader(filepath)
    vals = []
    for record in reader:
        if record.isControl():
            continue
        if record.getEntry() == enable_eid:
            vals.append((record.getTimestamp(), record.getBoolean()))
    
    windows = []
    window_start = None
    for ts, enabled in vals:
        if enabled and window_start is None:
            window_start = ts
        elif not enabled and window_start is not None:
            windows.append((window_start, ts))
            window_start = None
    if window_start is not None:
        windows.append((window_start, vals[-1][0]))
    
    return windows


def read_motor_signals(filepath, motor_ids, entries, windows, margin_us=500_000):
    """Read key motor signals during enabled windows (+margin)."""
    # Build set of wanted entry IDs
    wanted = {}  # eid -> (motor_id, suffix, type_str)
    for eid, (name, type_str) in entries.items():
        for mid in motor_ids:
            prefix = f"Phoenix6/TalonFX-{mid}/"
            if name.startswith(prefix):
                suffix = name[len(prefix):]
                if suffix in KEY_SIGNAL_SUFFIXES:
                    wanted[eid] = (mid, suffix, type_str)
                    break
    
    print(f"  Reading {len(wanted)} signals for motors {motor_ids}")
    
    # Read only those signals within time windows
    reader = DataLogReader(filepath)
    data = {}  # motor_id -> {suffix -> [(ts, val), ...]}
    for mid in motor_ids:
        data[mid] = {s: [] for s in KEY_SIGNAL_SUFFIXES}
    
    for record in reader:
        if record.isControl():
            continue
        eid = record.getEntry()
        if eid not in wanted:
            continue
        
        ts = record.getTimestamp()
        # Check if in any window (with margin)
        in_window = any(s - margin_us <= ts <= e + margin_us for s, e in windows)
        if not in_window:
            continue
        
        mid, suffix, type_str = wanted[eid]
        getter_name = GETTER_MAP.get(type_str)
        if getter_name is None:
            continue
        try:
            val = getattr(record, getter_name)()
            data[mid][suffix].append((ts, val))
        except Exception:
            pass
    
    return data


def analyze_window(data, window_idx, start_us, end_us):
    """Analyze motor behavior in a single enabled window."""
    duration_s = (end_us - start_us) / 1e6
    print(f"\n{'='*80}")
    print(f"WINDOW {window_idx+1}: {start_us/1e6:.2f}s - {end_us/1e6:.2f}s  ({duration_s:.2f}s)")
    print(f"{'='*80}")
    
    for mid in ['20', '21', '30', '31']:
        motor_data = data.get(mid)
        if not motor_data:
            continue
        
        # Filter to this window
        def in_win(ts):
            return start_us <= ts <= end_us
        
        vel = [(t, v) for t, v in motor_data.get('Velocity', []) if in_win(t)]
        duty = [(t, v) for t, v in motor_data.get('DutyCycle', []) if in_win(t)]
        voltage = [(t, v) for t, v in motor_data.get('MotorVoltage', []) if in_win(t)]
        stator_i = [(t, v) for t, v in motor_data.get('StatorCurrent', []) if in_win(t)]
        supply_i = [(t, v) for t, v in motor_data.get('SupplyCurrent', []) if in_win(t)]
        supply_v = [(t, v) for t, v in motor_data.get('SupplyVoltage', []) if in_win(t)]
        torque_i = [(t, v) for t, v in motor_data.get('TorqueCurrent', []) if in_win(t)]
        ctrl = [(t, v) for t, v in motor_data.get('ControlMode', []) if in_win(t)]
        vel_ref = [(t, v) for t, v in motor_data.get('PIDVelocity_Reference', []) if in_win(t)]
        vel_err = [(t, v) for t, v in motor_data.get('PIDVelocity_ClosedLoopError', []) if in_win(t)]
        pid_out = [(t, v) for t, v in motor_data.get('PIDMotorVoltage_Output', []) if in_win(t)]
        pid_ff = [(t, v) for t, v in motor_data.get('PIDMotorVoltage_FeedForward', []) if in_win(t)]
        pid_p = [(t, v) for t, v in motor_data.get('PIDMotorVoltage_ProportionalOutput', []) if in_win(t)]
        
        label = {'20': 'Flywheel-A', '21': 'Flywheel-B', '30': 'Feeder', '31': 'Spindexer'}[mid]
        
        if not vel:
            print(f"\n  Motor {mid} ({label}): no data in window")
            continue
        
        print(f"\n  Motor {mid} ({label}):")
        
        # Control mode
        if ctrl:
            modes = list(set(v for _, v in ctrl))
            print(f"    Control mode(s): {modes}")
        
        # Velocity stats
        vel_vals = [v for _, v in vel]
        print(f"    Velocity (rot/s): min={min(vel_vals):.1f}, max={max(vel_vals):.1f}, "
              f"mean={sum(vel_vals)/len(vel_vals):.1f}, samples={len(vel_vals)}")
        
        # Velocity reference (setpoint)
        if vel_ref:
            ref_vals = [v for _, v in vel_ref]
            print(f"    Velocity Ref (rot/s): min={min(ref_vals):.1f}, max={max(ref_vals):.1f}, "
                  f"mean={sum(ref_vals)/len(ref_vals):.1f}")
        
        # Velocity error
        if vel_err:
            err_vals = [v for _, v in vel_err]
            print(f"    Velocity Error: min={min(err_vals):.2f}, max={max(err_vals):.2f}, "
                  f"mean={sum(err_vals)/len(err_vals):.2f}")
        
        # Duty cycle
        if duty:
            d_vals = [v for _, v in duty]
            print(f"    DutyCycle: min={min(d_vals):.3f}, max={max(d_vals):.3f}, "
                  f"mean={sum(d_vals)/len(d_vals):.3f}")
        
        # Motor voltage
        if voltage:
            v_vals = [v for _, v in voltage]
            print(f"    MotorVoltage (V): min={min(v_vals):.2f}, max={max(v_vals):.2f}, "
                  f"mean={sum(v_vals)/len(v_vals):.2f}")
        
        # Current
        if stator_i:
            si = [v for _, v in stator_i]
            print(f"    StatorCurrent (A): min={min(si):.1f}, max={max(si):.1f}, mean={sum(si)/len(si):.1f}")
        if supply_i:
            si = [v for _, v in supply_i]
            print(f"    SupplyCurrent (A): min={min(si):.1f}, max={max(si):.1f}, mean={sum(si)/len(si):.1f}")
        if torque_i:
            ti = [v for _, v in torque_i]
            print(f"    TorqueCurrent (A): min={min(ti):.1f}, max={max(ti):.1f}, mean={sum(ti)/len(ti):.1f}")
        
        # Supply voltage
        if supply_v:
            sv = [v for _, v in supply_v]
            print(f"    SupplyVoltage (V): min={min(sv):.2f}, max={max(sv):.2f}, mean={sum(sv)/len(sv):.2f}")
        
        # PID output breakdown
        if pid_out:
            po = [v for _, v in pid_out]
            print(f"    PID Output (V): min={min(po):.2f}, max={max(po):.2f}, mean={sum(po)/len(po):.2f}")
        if pid_ff:
            ff = [v for _, v in pid_ff]
            print(f"    PID FeedForward (V): min={min(ff):.2f}, max={max(ff):.2f}, mean={sum(ff)/len(ff):.2f}")
        if pid_p:
            pp = [v for _, v in pid_p]
            print(f"    PID Proportional (V): min={min(pp):.2f}, max={max(pp):.2f}, mean={sum(pp)/len(pp):.2f}")
        
        # Estimate power draw
        if stator_i and supply_v:
            # Mechanical power ~ voltage * stator_current (approx)
            # Electrical power from supply = supply_voltage * supply_current 
            pass
        if supply_i and supply_v:
            # Match timestamps approximately to compute instantaneous power
            powers = []
            sv_dict = {t: v for t, v in supply_v}
            for t, i_val in supply_i:
                # Find closest supply voltage
                if t in sv_dict:
                    powers.append(sv_dict[t] * i_val)
            if powers:
                print(f"    Electrical Power (W): min={min(powers):.0f}, max={max(powers):.0f}, "
                      f"mean={sum(powers)/len(powers):.0f}")
        
        # Detect velocity dips during launches (for flywheel motors)
        if mid in ['20', '21'] and len(vel) > 10:
            # Find velocity dips > 5 rot/s from local max
            print(f"\n    --- Velocity profile (flywheel) ---")
            # Print velocity trace at key moments
            # Detect transitions: when velocity drops significantly then recovers
            dips = []
            prev_v = vel_vals[0]
            in_dip = False
            dip_start_v = None
            dip_min_v = None
            dip_start_t = None
            
            for i in range(1, len(vel)):
                t, v = vel[i]
                if not in_dip and v < prev_v - 2.0:  # Start of dip
                    in_dip = True
                    dip_start_v = prev_v
                    dip_min_v = v
                    dip_start_t = vel[i-1][0]
                elif in_dip:
                    if v < dip_min_v:
                        dip_min_v = v
                    if v > dip_min_v + 2.0:  # Recovery
                        dip_magnitude = dip_start_v - dip_min_v
                        if dip_magnitude > 3.0:  # Significant dip = ball launch
                            dip_duration_ms = (t - dip_start_t) / 1000
                            dips.append({
                                'time': dip_start_t / 1e6,
                                'pre_vel': dip_start_v,
                                'min_vel': dip_min_v,
                                'drop': dip_magnitude,
                                'duration_ms': dip_duration_ms,
                                'recovery_vel': v,
                            })
                        in_dip = False
                prev_v = v
            
            if dips:
                print(f"    Detected {len(dips)} velocity dips (likely ball launches):")
                for j, d in enumerate(dips):
                    print(f"      Launch {j+1}: t={d['time']:.3f}s, "
                          f"vel {d['pre_vel']:.1f} -> {d['min_vel']:.1f} (drop {d['drop']:.1f} rot/s), "
                          f"recovery to {d['recovery_vel']:.1f}, dur={d['duration_ms']:.0f}ms")
                
                # Compute inter-launch timing
                if len(dips) > 1:
                    intervals = [(dips[i+1]['time'] - dips[i]['time'])*1000 for i in range(len(dips)-1)]
                    print(f"    Inter-launch intervals (ms): {', '.join(f'{x:.0f}' for x in intervals)}")
                    print(f"    Total launch sequence: {(dips[-1]['time'] - dips[0]['time'])*1000:.0f}ms "
                          f"for {len(dips)} launches")


def main():
    # ================================================================
    # PHASE 1: Get enable windows from RIO
    # ================================================================
    print("Reading enable windows from RIO file...")
    rio_entries = discover_signals(RIO_FILE)
    windows = read_enable_windows(RIO_FILE, rio_entries)
    
    print(f"Found {len(windows)} enabled windows:")
    for i, (s, e) in enumerate(windows):
        print(f"  Window {i+1}: {s/1e6:.2f}s - {e/1e6:.2f}s  ({(e-s)/1e6:.2f}s)")
    
    # ================================================================
    # PHASE 2: Read motor signals from RIO (motors 20, 21, 31)
    # ================================================================
    print("\nReading motor 20, 21, 31 signals from RIO file (enabled windows only)...")
    rio_data = read_motor_signals(RIO_FILE, ['20', '21', '31'], rio_entries, windows)
    
    # ================================================================
    # PHASE 3: Read motor 30 signals from CANivore
    # ================================================================
    print("\nReading motor 30 signals from CANivore file (enabled windows only)...")
    canivore_entries = discover_signals(CANIVORE_FILE)
    canivore_data = read_motor_signals(CANIVORE_FILE, ['30'], canivore_entries, windows)
    
    # Merge
    all_data = {**rio_data, **canivore_data}
    
    # ================================================================
    # PHASE 4: Analyze each window
    # ================================================================
    for i, (start, end) in enumerate(windows):
        analyze_window(all_data, i, start, end)


if __name__ == "__main__":
    main()
