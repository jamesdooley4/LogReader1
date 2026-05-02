"""Analyze flywheel motors 20 (follower) and 21 (leader) from a hoot log.

Extracts velocity, current, temperature, PID error, and duty-cycle signals
to diagnose heat and noise issues during fuel launches.
"""

from __future__ import annotations

import subprocess
import struct
import sys
import tempfile
from pathlib import Path

import numpy as np

OWLET = r"c:\Tools\owlet-26.1.0-windowsx86-64.exe"
HOOT = r"D:\Temp\2026-04-3practicelogs\2026-04-04_01-03-30\rio_2026-04-04_01-03-32.hoot"

# Signal IDs for motors 20 and 21 (from owlet --scan)
SIGNAL_IDS = [
    # Motor 20
    "6fd1400",  # Velocity
    "6f61400",  # DeviceTemp
    "6f31400",  # StatorCurrent
    "6f41400",  # SupplyCurrent
    "6f21400",  # TorqueCurrent
    "6ec1400",  # MotorVoltage
    "6f01400",  # DutyCycle
    "6f51400",  # SupplyVoltage
    "7131400",  # PIDVelocity_Reference
    "7151400",  # PIDVelocity_ClosedLoopError
    "71f1400",  # PIDDutyCycle_Output
    "7041400",  # PIDDutyCycle_FeedForward
    "7191400",  # PIDDutyCycle_ProportionalOutput
    "7071400",  # ControlMode
    # Motor 21
    "6fd1500",  # Velocity
    "6f61500",  # DeviceTemp
    "6f31500",  # StatorCurrent
    "6f41500",  # SupplyCurrent
    "6f21500",  # TorqueCurrent
    "6ec1500",  # MotorVoltage
    "6f01500",  # DutyCycle
    "6f51500",  # SupplyVoltage
    "7131500",  # PIDVelocity_Reference
    "7151500",  # PIDVelocity_ClosedLoopError
    "71f1500",  # PIDDutyCycle_Output
    "7041500",  # PIDDutyCycle_FeedForward
    "7191500",  # PIDDutyCycle_ProportionalOutput
    "7071500",  # ControlMode
]


def convert_hoot(hoot_path: str, signal_ids: list[str]) -> Path:
    """Convert hoot file extracting only the specified signals."""
    out = Path(tempfile.mktemp(suffix=".wpilog"))
    cmd = [OWLET, hoot_path, str(out), "-f", "wpilog"]
    for sid in signal_ids:
        cmd.extend(["-s", sid])
    print(f"Converting hoot → wpilog ({len(signal_ids)} signals)...", flush=True)
    subprocess.run(cmd, check=True, capture_output=True, timeout=120)
    print(f"  Output: {out} ({out.stat().st_size / 1024:.0f} KB)", flush=True)
    return out


def read_signals(wpilog_path: Path) -> dict[str, list[tuple[float, float]]]:
    """Read all numeric signals from the wpilog as {name: [(time_s, value), ...]}."""
    from wpiutil.log import DataLogReader

    reader = DataLogReader(str(wpilog_path))
    if not reader.isValid():
        raise ValueError(f"Invalid wpilog: {wpilog_path}")

    entries: dict[int, tuple[str, str]] = {}
    signals: dict[str, list[tuple[float, float]]] = {}

    for rec in reader:
        if rec.isControl():
            if rec.isStart():
                s = rec.getStartData()
                entries[s.entry] = (s.name, s.type)
                signals.setdefault(s.name, [])
        else:
            eid = rec.getEntry()
            if eid not in entries:
                continue
            name, typ = entries[eid]
            ts = rec.getTimestamp() / 1e6  # → seconds
            try:
                if typ == "double":
                    val = rec.getDouble()
                elif typ == "int64":
                    val = float(rec.getInteger())
                elif typ == "boolean":
                    val = float(rec.getBoolean())
                else:
                    continue
                signals[name].append((ts, val))
            except Exception:
                pass

    return signals


def stats(values: list[float]) -> dict:
    arr = np.array(values)
    return {
        "count": len(arr),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
        "p5": float(np.percentile(arr, 5)),
        "p95": float(np.percentile(arr, 95)),
    }


def analyze(signals: dict[str, list[tuple[float, float]]]) -> None:
    """Print a comprehensive analysis of motors 20 and 21."""
    def get(name: str) -> list[tuple[float, float]]:
        # Try with and without Phoenix6/ prefix
        return signals.get(name, signals.get(f"Phoenix6/{name}", []))

    def vals(name: str) -> list[float]:
        return [v for _, v in get(name)]

    print("\n" + "=" * 80)
    print("FLYWHEEL MOTOR ANALYSIS — Motors 20 (follower) & 21 (leader)")
    print("=" * 80)

    # --- Temperature ---
    print("\n--- TEMPERATURE ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        t = vals(f"TalonFX-{mid}/DeviceTemp")
        if t:
            s = stats(t)
            print(f"  Motor {mid} ({role}): min={s['min']:.1f}°C  max={s['max']:.1f}°C  "
                  f"mean={s['mean']:.1f}°C  final={t[-1]:.1f}°C")

    # --- Velocity ---
    print("\n--- VELOCITY (rps) ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        v = vals(f"TalonFX-{mid}/Velocity")
        if v:
            s = stats(v)
            print(f"  Motor {mid} ({role}): min={s['min']:.1f}  max={s['max']:.1f}  "
                  f"mean={s['mean']:.1f}  std={s['std']:.2f}")

    # --- Velocity Reference (setpoint) ---
    print("\n--- VELOCITY REFERENCE (setpoint, rps) ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        v = vals(f"TalonFX-{mid}/PIDVelocity_Reference")
        if v:
            nonzero = [x for x in v if abs(x) > 0.1]
            if nonzero:
                s = stats(nonzero)
                print(f"  Motor {mid} ({role}): min={s['min']:.1f}  max={s['max']:.1f}  "
                      f"mean={s['mean']:.1f}  ({len(nonzero)} nonzero samples)")
            else:
                print(f"  Motor {mid} ({role}): all zero/near-zero")
        else:
            print(f"  Motor {mid} ({role}): no data")

    # --- Closed-Loop Error ---
    print("\n--- PID VELOCITY CLOSED-LOOP ERROR (rps) ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        v = vals(f"TalonFX-{mid}/PIDVelocity_ClosedLoopError")
        if v:
            s = stats(v)
            abs_v = [abs(x) for x in v]
            abs_s = stats(abs_v)
            print(f"  Motor {mid} ({role}): mean_err={s['mean']:.2f}  std={s['std']:.2f}  "
                  f"|err| mean={abs_s['mean']:.2f}  |err| max={abs_s['max']:.2f}  |err| p95={abs_s['p95']:.2f}")

    # --- Stator Current ---
    print("\n--- STATOR CURRENT (A) ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        v = vals(f"TalonFX-{mid}/StatorCurrent")
        if v:
            s = stats(v)
            print(f"  Motor {mid} ({role}): min={s['min']:.1f}  max={s['max']:.1f}  "
                  f"mean={s['mean']:.1f}  std={s['std']:.2f}  p95={s['p95']:.1f}")

    # --- Torque Current ---
    print("\n--- TORQUE CURRENT (A) ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        v = vals(f"TalonFX-{mid}/TorqueCurrent")
        if v:
            s = stats(v)
            print(f"  Motor {mid} ({role}): min={s['min']:.1f}  max={s['max']:.1f}  "
                  f"mean={s['mean']:.1f}  std={s['std']:.2f}  p95={s['p95']:.1f}")

    # --- Supply Current ---
    print("\n--- SUPPLY CURRENT (A) ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        v = vals(f"TalonFX-{mid}/SupplyCurrent")
        if v:
            s = stats(v)
            print(f"  Motor {mid} ({role}): min={s['min']:.1f}  max={s['max']:.1f}  "
                  f"mean={s['mean']:.1f}  std={s['std']:.2f}  p95={s['p95']:.1f}")

    # --- Motor Voltage ---
    print("\n--- MOTOR VOLTAGE (V) ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        v = vals(f"TalonFX-{mid}/MotorVoltage")
        if v:
            s = stats(v)
            print(f"  Motor {mid} ({role}): min={s['min']:.2f}  max={s['max']:.2f}  "
                  f"mean={s['mean']:.2f}  std={s['std']:.3f}")

    # --- Duty Cycle ---
    print("\n--- DUTY CYCLE ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        v = vals(f"TalonFX-{mid}/DutyCycle")
        if v:
            s = stats(v)
            print(f"  Motor {mid} ({role}): min={s['min']:.3f}  max={s['max']:.3f}  "
                  f"mean={s['mean']:.3f}  std={s['std']:.4f}")

    # --- PID Outputs ---
    print("\n--- PID DUTY-CYCLE OUTPUTS (leader motor 21) ---")
    for sig in ["PIDDutyCycle_Output", "PIDDutyCycle_FeedForward", "PIDDutyCycle_ProportionalOutput"]:
        v = vals(f"TalonFX-21/{sig}")
        if v:
            s = stats(v)
            print(f"  {sig}: min={s['min']:.4f}  max={s['max']:.4f}  "
                  f"mean={s['mean']:.4f}  std={s['std']:.5f}")

    # --- Velocity drop analysis (launch events) ---
    print("\n--- VELOCITY DROP EVENTS (Motor 21 leader) ---")
    vel_data = get("TalonFX-21/Velocity")
    ref_data = get("TalonFX-21/PIDVelocity_Reference")
    if vel_data and ref_data:
        # Find periods where reference > 0 and velocity drops significantly
        # Build time-aligned arrays
        vel_arr = np.array(vel_data)
        ref_arr = np.array(ref_data)

        # Find when reference is nonzero (flywheel active)
        active_mask = ref_arr[:, 1] > 5.0  # > 5 rps reference

        if np.any(active_mask):
            active_vel_times = vel_arr[:, 0]
            active_vel_vals = vel_arr[:, 1]
            active_ref_vals = ref_arr[:, 1]

            # Detect velocity dips: look for samples where velocity is far from ref
            # Interpolate ref to vel timestamps
            ref_at_vel = np.interp(active_vel_times, ref_arr[:, 0], ref_arr[:, 1])
            error = active_vel_vals - ref_at_vel
            active = ref_at_vel > 5.0

            if np.any(active):
                active_error = error[active]
                active_times_filtered = active_vel_times[active]
                active_vel_filtered = active_vel_vals[active]
                ref_filtered = ref_at_vel[active]

                print(f"  Active samples (ref > 5 rps): {np.sum(active)}")
                print(f"  Velocity error during active: mean={np.mean(active_error):.2f} rps  "
                      f"std={np.std(active_error):.2f}  min={np.min(active_error):.2f}  max={np.max(active_error):.2f}")

                # Find big dips (error < -5 rps)
                big_dips = active_error < -5.0
                if np.any(big_dips):
                    dip_errors = active_error[big_dips]
                    dip_times = active_times_filtered[big_dips]
                    print(f"  Big velocity dips (>5 rps below ref): {np.sum(big_dips)} samples")
                    print(f"  Worst dip: {np.min(dip_errors):.1f} rps at t={dip_times[np.argmin(dip_errors)]:.2f}s")

                    # Find distinct launch events (dips separated by > 0.5s)
                    dip_groups = []
                    group_start = 0
                    for j in range(1, len(dip_times)):
                        if dip_times[j] - dip_times[j-1] > 0.5:
                            dip_groups.append((dip_times[group_start], dip_times[j-1],
                                             np.min(dip_errors[group_start:j])))
                            group_start = j
                    dip_groups.append((dip_times[group_start], dip_times[-1],
                                    np.min(dip_errors[group_start:])))

                    print(f"  Distinct launch/dip events: {len(dip_groups)}")
                    for k, (t_start, t_end, worst) in enumerate(dip_groups[:10]):
                        print(f"    Event {k+1}: t={t_start:.2f}-{t_end:.2f}s  worst_dip={worst:.1f} rps")

                # Velocity noise analysis during steady-state
                # Steady state = error within ±2 rps
                steady = np.abs(active_error) < 2.0
                if np.any(steady):
                    steady_vel = active_vel_filtered[steady]
                    steady_ref = ref_filtered[steady]
                    steady_err = active_error[steady]
                    print(f"\n  Steady-state (|error|<2rps): {np.sum(steady)} samples")
                    print(f"  Velocity noise (std): {np.std(steady_vel):.3f} rps")
                    print(f"  Error noise (std): {np.std(steady_err):.3f} rps")

    # --- Motor comparison (leader vs follower fighting) ---
    print("\n--- LEADER vs FOLLOWER COMPARISON ---")
    v20 = get("TalonFX-20/TorqueCurrent")
    v21 = get("TalonFX-21/TorqueCurrent")
    if v20 and v21:
        # Interpolate motor 20 to motor 21 timestamps
        t20 = np.array([t for t, _ in v20])
        c20 = np.array([v for _, v in v20])
        t21 = np.array([t for t, _ in v21])
        c21 = np.array([v for _, v in v21])
        c20_interp = np.interp(t21, t20, c20)

        # When both should be positive (spinning same direction)
        both_active = (np.abs(c21) > 1.0)
        if np.any(both_active):
            active_c20 = c20_interp[both_active]
            active_c21 = c21[both_active]
            # Check if they fight (opposite signs)
            fighting = (active_c20 * active_c21) < 0
            pct_fighting = 100.0 * np.sum(fighting) / len(active_c20)
            print(f"  Samples where leader active (|I|>1A): {np.sum(both_active)}")
            print(f"  Motors fighting (opposite torque): {np.sum(fighting)} ({pct_fighting:.1f}%)")
            print(f"  Leader I mean={np.mean(active_c21):.2f}A  Follower I mean={np.mean(active_c20):.2f}A")

    # --- Supply voltage sag ---
    print("\n--- SUPPLY VOLTAGE ---")
    for mid in [20, 21]:
        v = vals(f"TalonFX-{mid}/SupplyVoltage")
        if v:
            s = stats(v)
            print(f"  Motor {mid}: min={s['min']:.2f}V  mean={s['mean']:.2f}V  "
                  f"p5={s['p5']:.2f}V")

    print("\n" + "=" * 80)


def main():
    wpilog_path = convert_hoot(HOOT, SIGNAL_IDS)
    try:
        signals = read_signals(wpilog_path)
        print(f"\nLoaded {len(signals)} signals:")
        for name, data in sorted(signals.items()):
            print(f"  {name}: {len(data)} samples")
        analyze(signals)
    finally:
        wpilog_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
