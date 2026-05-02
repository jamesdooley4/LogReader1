"""Re-analyze MM vs VV with correct PIDMotorVoltage signals for VelocityVoltage mode."""

from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

import numpy as np

OWLET = r"c:\Tools\owlet-26.1.0-windowsx86-64.exe"

MM_HOOT = r"D:\Temp\2026-04-4_Practice\2026-04-05_00-23-36\rio_2026-04-05_00-23-38.hoot"
VV_HOOT = r"D:\Temp\2026-04-4_Practice\2026-04-05_00-34-07\rio_2026-04-05_00-34-10.hoot"

# Core signals + voltage-mode PID signals for motor 21
SIGNAL_IDS = [
    # Motor 20 core
    "6fd1400", "6f61400", "6f31400", "6f21400", "6ec1400", "6f51400",
    # Motor 21 core
    "6fd1500", "6f61500", "6f31500", "6f21500", "6ec1500", "6f51500",
    # Motor 21 PID (velocity ref, error)
    "7131500", "7151500",
    # Motor 21 PID DutyCycle outputs (for MM mode)
    "71f1500", "7041500", "7191500", "71c1500",
    # Motor 21 PID MotorVoltage outputs (for VV mode)
    "7201500",  # PIDMotorVoltage_Output
    "7051500",  # PIDMotorVoltage_FeedForward
    "71a1500",  # PIDMotorVoltage_ProportionalOutput
    "71d1500",  # PIDMotorVoltage_DerivativeOutput
    # Motor 20 PID MotorVoltage (follower check)
    "7201400", "7051400", "71a1400",
]


def convert_hoot(hoot_path, signal_ids):
    out = Path(tempfile.mktemp(suffix=".wpilog"))
    cmd = [OWLET, hoot_path, str(out), "-f", "wpilog"]
    for sid in signal_ids:
        cmd.extend(["-s", sid])
    print(f"Converting {Path(hoot_path).name}...", flush=True)
    subprocess.run(cmd, check=True, capture_output=True, timeout=120)
    return out


def read_signals(wpilog_path):
    from wpiutil.log import DataLogReader
    reader = DataLogReader(str(wpilog_path))
    entries, signals = {}, {}
    for rec in reader:
        if rec.isControl():
            if rec.isStart():
                s = rec.getStartData()
                entries[s.entry] = (s.name, s.type)
                signals.setdefault(s.name, [])
        else:
            eid = rec.getEntry()
            if eid not in entries: continue
            name, typ = entries[eid]
            ts = rec.getTimestamp() / 1e6
            try:
                if typ == "double": val = rec.getDouble()
                elif typ == "int64": val = float(rec.getInteger())
                elif typ == "boolean": val = float(rec.getBoolean())
                else: continue
                signals[name].append((ts, val))
            except: pass
    return signals


def get(signals, name):
    return signals.get(name, signals.get(f"Phoenix6/{name}", []))

def vals(signals, name):
    return [v for _, v in get(signals, name)]

def stats(values):
    if not values: return None
    a = np.array(values)
    return {"count": len(a), "min": float(np.min(a)), "max": float(np.max(a)),
            "mean": float(np.mean(a)), "std": float(np.std(a)),
            "p5": float(np.percentile(a, 5)), "p95": float(np.percentile(a, 95))}


def analyze_pid(label, signals):
    print(f"\n{'='*80}")
    print(f"  {label} — PID OUTPUT ANALYSIS")
    print(f"{'='*80}")

    # Show what signals we have
    print("\n--- AVAILABLE PID SIGNALS (Motor 21) ---")
    for name, data in sorted(signals.items()):
        if "21" in name and "PID" in name and len(data) > 0:
            print(f"  {name.replace('Phoenix6/', '')}: {len(data)} samples")

    # DutyCycle PID outputs
    print("\n--- PIDDutyCycle OUTPUTS (Motor 21) ---")
    for sig in ["PIDDutyCycle_Output", "PIDDutyCycle_FeedForward",
                "PIDDutyCycle_ProportionalOutput", "PIDDutyCycle_DerivativeOutput"]:
        v = vals(signals, f"TalonFX-21/{sig}")
        if v:
            s = stats(v)
            nonzero = [x for x in v if abs(x) > 0.0001]
            print(f"  {sig}: mean={s['mean']:.4f}  std={s['std']:.4f}  "
                  f"min={s['min']:.4f}  max={s['max']:.4f}  nonzero={len(nonzero)}")
        else:
            print(f"  {sig}: NO DATA")

    # MotorVoltage PID outputs
    print("\n--- PIDMotorVoltage OUTPUTS (Motor 21) ---")
    for sig in ["PIDMotorVoltage_Output", "PIDMotorVoltage_FeedForward",
                "PIDMotorVoltage_ProportionalOutput", "PIDMotorVoltage_DerivativeOutput"]:
        v = vals(signals, f"TalonFX-21/{sig}")
        if v:
            s = stats(v)
            nonzero = [x for x in v if abs(x) > 0.0001]
            print(f"  {sig}: mean={s['mean']:.4f}V  std={s['std']:.4f}  "
                  f"min={s['min']:.4f}V  max={s['max']:.4f}V  nonzero={len(nonzero)}")
        else:
            print(f"  {sig}: NO DATA")

    # Closed-loop error
    print("\n--- CLOSED-LOOP ERROR (Motor 21) ---")
    e = vals(signals, "TalonFX-21/PIDVelocity_ClosedLoopError")
    if e:
        s = stats(e)
        print(f"  mean={s['mean']:.2f} rps  std={s['std']:.2f}  min={s['min']:.2f}  max={s['max']:.2f}")

    # Velocity reference
    ref = vals(signals, "TalonFX-21/PIDVelocity_Reference")
    if ref:
        nonzero = [x for x in ref if abs(x) > 0.1]
        if nonzero:
            s = stats(nonzero)
            print(f"\n  Velocity ref (nonzero): mean={s['mean']:.1f}  min={s['min']:.1f}  max={s['max']:.1f}")

    # Motor voltage vs PID output comparison
    print("\n--- MOTOR VOLTAGE vs PID OUTPUT ---")
    mv = vals(signals, "TalonFX-21/MotorVoltage")
    pid_v = vals(signals, "TalonFX-21/PIDMotorVoltage_Output")
    pid_dc = vals(signals, "TalonFX-21/PIDDutyCycle_Output")
    if mv:
        s = stats(mv)
        print(f"  MotorVoltage:           mean={s['mean']:.2f}V  max={s['max']:.2f}V  min={s['min']:.2f}V")
    if pid_v:
        s = stats(pid_v)
        print(f"  PIDMotorVoltage_Output: mean={s['mean']:.2f}V  max={s['max']:.2f}V  min={s['min']:.2f}V")
    if pid_dc:
        s = stats(pid_dc)
        print(f"  PIDDutyCycle_Output:    mean={s['mean']:.4f}  max={s['max']:.4f}  min={s['min']:.4f}")

    # Check which PID mode is active by seeing which has nonzero data
    pid_v_nz = len([x for x in vals(signals, "TalonFX-21/PIDMotorVoltage_Output") if abs(x) > 0.001])
    pid_dc_nz = len([x for x in vals(signals, "TalonFX-21/PIDDutyCycle_Output") if abs(x) > 0.001])
    print(f"\n  PIDMotorVoltage nonzero samples: {pid_v_nz}")
    print(f"  PIDDutyCycle nonzero samples:    {pid_dc_nz}")
    if pid_v_nz > pid_dc_nz:
        print(f"  → Active PID mode: VOLTAGE (VelocityVoltage)")
    elif pid_dc_nz > pid_v_nz:
        print(f"  → Active PID mode: DUTY CYCLE (MotionMagicVelocityVoltage or VelocityDutyCycle)")
    else:
        print(f"  → Active PID mode: UNCLEAR")

    # During active periods, show the PID breakdown
    print("\n--- PID BREAKDOWN DURING ACTIVE FLYWHEEL ---")
    ref_data = get(signals, "TalonFX-21/PIDVelocity_Reference")
    if ref_data:
        ref_arr = np.array(ref_data)
        active_times = ref_arr[ref_arr[:, 1] > 5.0, 0]
        if len(active_times) > 0:
            t_start, t_end = active_times[0], active_times[-1]
            print(f"  Active period: {t_start:.1f}s - {t_end:.1f}s")

            for sig_name in ["PIDMotorVoltage_Output", "PIDMotorVoltage_FeedForward",
                            "PIDMotorVoltage_ProportionalOutput", "PIDMotorVoltage_DerivativeOutput",
                            "PIDDutyCycle_Output", "PIDDutyCycle_FeedForward",
                            "PIDDutyCycle_ProportionalOutput"]:
                data = get(signals, f"TalonFX-21/{sig_name}")
                if data:
                    arr = np.array(data)
                    mask = (arr[:, 0] >= t_start) & (arr[:, 0] <= t_end)
                    active_vals = arr[mask, 1]
                    if len(active_vals) > 0:
                        s = stats(active_vals.tolist())
                        unit = "V" if "Voltage" in sig_name else ""
                        print(f"  {sig_name}: mean={s['mean']:.3f}{unit}  "
                              f"std={s['std']:.3f}  min={s['min']:.3f}{unit}  max={s['max']:.3f}{unit}")


def main():
    mm_wpilog = convert_hoot(MM_HOOT, SIGNAL_IDS)
    vv_wpilog = convert_hoot(VV_HOOT, SIGNAL_IDS)
    try:
        mm_signals = read_signals(mm_wpilog)
        vv_signals = read_signals(vv_wpilog)
        analyze_pid("MotionMagicVelocityVoltage", mm_signals)
        analyze_pid("VelocityVoltage", vv_signals)
    finally:
        mm_wpilog.unlink(missing_ok=True)
        vv_wpilog.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
