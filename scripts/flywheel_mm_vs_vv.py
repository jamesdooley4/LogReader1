"""Compare MotionMagicVelocityVoltage vs VelocityVoltage for flywheel motors 20/21."""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

OWLET = r"c:\Tools\owlet-26.1.0-windowsx86-64.exe"

MM_HOOT = r"D:\Temp\2026-04-4_Practice\2026-04-05_00-23-36\rio_2026-04-05_00-23-38.hoot"
VV_HOOT = r"D:\Temp\2026-04-4_Practice\2026-04-05_00-34-07\rio_2026-04-05_00-34-10.hoot"

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
    "6f71400",  # ProcessorTemp
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
    "6f71500",  # ProcessorTemp
]


def convert_hoot(hoot_path: str, signal_ids: list[str]) -> Path:
    out = Path(tempfile.mktemp(suffix=".wpilog"))
    cmd = [OWLET, hoot_path, str(out), "-f", "wpilog"]
    for sid in signal_ids:
        cmd.extend(["-s", sid])
    print(f"Converting {Path(hoot_path).name}...", flush=True)
    subprocess.run(cmd, check=True, capture_output=True, timeout=120)
    print(f"  Output: {out.stat().st_size / 1024:.0f} KB", flush=True)
    return out


def read_signals(wpilog_path: Path) -> dict[str, list[tuple[float, float]]]:
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
            ts = rec.getTimestamp() / 1e6
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


def get(signals, name):
    return signals.get(name, signals.get(f"Phoenix6/{name}", []))


def vals(signals, name):
    return [v for _, v in get(signals, name)]


def stats(values):
    if not values:
        return None
    arr = np.array(values)
    return {
        "count": len(arr), "min": float(np.min(arr)), "max": float(np.max(arr)),
        "mean": float(np.mean(arr)), "std": float(np.std(arr)),
        "p5": float(np.percentile(arr, 5)), "p95": float(np.percentile(arr, 95)),
        "p1": float(np.percentile(arr, 1)), "p99": float(np.percentile(arr, 99)),
    }


def find_recovery_events(signals, motor_id=21):
    """Find velocity dip events and measure recovery time."""
    vel_data = get(signals, f"TalonFX-{motor_id}/Velocity")
    ref_data = get(signals, f"TalonFX-{motor_id}/PIDVelocity_Reference")
    if not vel_data or not ref_data:
        return []

    vel_arr = np.array(vel_data)
    ref_arr = np.array(ref_data)

    # Interpolate ref to vel timestamps
    ref_at_vel = np.interp(vel_arr[:, 0], ref_arr[:, 0], ref_arr[:, 1])
    error = vel_arr[:, 1] - ref_at_vel
    times = vel_arr[:, 0]
    active = ref_at_vel > 5.0

    if not np.any(active):
        return []

    # Find dip starts: error drops below -3 rps while active
    in_dip = False
    events = []
    dip_start_time = 0
    dip_worst = 0
    ref_at_dip = 0

    for i in range(len(times)):
        if not active[i]:
            in_dip = False
            continue
        if not in_dip and error[i] < -3.0:
            in_dip = True
            dip_start_time = times[i]
            dip_worst = error[i]
            ref_at_dip = ref_at_vel[i]
        elif in_dip:
            if error[i] < dip_worst:
                dip_worst = error[i]
            if error[i] > -1.0:  # recovered to within 1 rps
                recovery_time_ms = (times[i] - dip_start_time) * 1000
                events.append({
                    "start_t": dip_start_time,
                    "recovery_ms": recovery_time_ms,
                    "worst_error_rps": dip_worst,
                    "ref_rps": ref_at_dip,
                })
                in_dip = False

    return events


def analyze_one(label: str, signals: dict):
    print(f"\n{'='*80}")
    print(f"  {label}")
    print(f"{'='*80}")

    # Temperature
    print("\n--- TEMPERATURE ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        t = vals(signals, f"TalonFX-{mid}/DeviceTemp")
        if t:
            s = stats(t)
            print(f"  Motor {mid} ({role}): start={t[0]:.0f}°C  end={t[-1]:.0f}°C  max={s['max']:.0f}°C  mean={s['mean']:.1f}°C")

    # Velocity
    print("\n--- VELOCITY (Motor 21 leader, rps) ---")
    v = vals(signals, "TalonFX-21/Velocity")
    if v:
        s = stats(v)
        print(f"  min={s['min']:.1f}  max={s['max']:.1f}  mean={s['mean']:.1f}  std={s['std']:.2f}")

    # Velocity reference
    print("\n--- VELOCITY REFERENCE (Motor 21, rps) ---")
    vr = vals(signals, "TalonFX-21/PIDVelocity_Reference")
    if vr:
        nonzero = [x for x in vr if abs(x) > 0.1]
        if nonzero:
            s = stats(nonzero)
            print(f"  min={s['min']:.1f}  max={s['max']:.1f}  mean={s['mean']:.1f}  ({len(nonzero)} nonzero)")
        else:
            print(f"  all zero")

    # Closed-loop error
    print("\n--- CLOSED-LOOP ERROR (Motor 21, rps) ---")
    e = vals(signals, "TalonFX-21/PIDVelocity_ClosedLoopError")
    if e:
        s = stats(e)
        abs_e = [abs(x) for x in e]
        abs_s = stats(abs_e)
        print(f"  mean={s['mean']:.2f}  std={s['std']:.2f}  min={s['min']:.2f}  max={s['max']:.2f}")
        print(f"  |err| mean={abs_s['mean']:.2f}  |err| p95={abs_s['p95']:.2f}  |err| max={abs_s['max']:.2f}")

    # Stator current both motors
    print("\n--- STATOR CURRENT (A) ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        c = vals(signals, f"TalonFX-{mid}/StatorCurrent")
        if c:
            s = stats(c)
            print(f"  Motor {mid} ({role}): mean={s['mean']:.1f}  std={s['std']:.1f}  max={s['max']:.1f}  p95={s['p95']:.1f}")

    # Torque current both motors
    print("\n--- TORQUE CURRENT (A) ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        c = vals(signals, f"TalonFX-{mid}/TorqueCurrent")
        if c:
            s = stats(c)
            print(f"  Motor {mid} ({role}): mean={s['mean']:.1f}  std={s['std']:.1f}  max={s['max']:.1f}  p95={s['p95']:.1f}  min={s['min']:.1f}")

    # Supply current
    print("\n--- SUPPLY CURRENT (A) ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        c = vals(signals, f"TalonFX-{mid}/SupplyCurrent")
        if c:
            s = stats(c)
            print(f"  Motor {mid} ({role}): mean={s['mean']:.1f}  std={s['std']:.1f}  max={s['max']:.1f}  p95={s['p95']:.1f}")

    # Motor voltage
    print("\n--- MOTOR VOLTAGE (V) ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        mv = vals(signals, f"TalonFX-{mid}/MotorVoltage")
        if mv:
            s = stats(mv)
            print(f"  Motor {mid} ({role}): mean={s['mean']:.2f}  std={s['std']:.2f}  max={s['max']:.2f}  min={s['min']:.2f}")

    # Duty cycle
    print("\n--- DUTY CYCLE ---")
    for mid in [20, 21]:
        role = "leader" if mid == 21 else "follower"
        d = vals(signals, f"TalonFX-{mid}/DutyCycle")
        if d:
            s = stats(d)
            print(f"  Motor {mid} ({role}): mean={s['mean']:.3f}  std={s['std']:.3f}  max={s['max']:.3f}  min={s['min']:.3f}")

    # PID outputs (leader only)
    print("\n--- PID OUTPUTS (Motor 21 leader) ---")
    for sig in ["PIDDutyCycle_Output", "PIDDutyCycle_FeedForward", "PIDDutyCycle_ProportionalOutput"]:
        pv = vals(signals, f"TalonFX-21/{sig}")
        if pv:
            s = stats(pv)
            print(f"  {sig}: mean={s['mean']:.4f}  std={s['std']:.4f}  max={s['max']:.4f}  min={s['min']:.4f}")

    # Supply voltage
    print("\n--- SUPPLY VOLTAGE (V) ---")
    for mid in [20, 21]:
        sv = vals(signals, f"TalonFX-{mid}/SupplyVoltage")
        if sv:
            s = stats(sv)
            print(f"  Motor {mid}: min={s['min']:.2f}  mean={s['mean']:.2f}  p5={s['p5']:.2f}")

    # Leader vs follower torque fighting
    print("\n--- LEADER vs FOLLOWER ---")
    tc20 = get(signals, "TalonFX-20/TorqueCurrent")
    tc21 = get(signals, "TalonFX-21/TorqueCurrent")
    if tc20 and tc21:
        t20 = np.array([t for t, _ in tc20])
        c20 = np.array([v for _, v in tc20])
        t21 = np.array([t for t, _ in tc21])
        c21 = np.array([v for _, v in tc21])
        c20_interp = np.interp(t21, t20, c20)
        both_active = np.abs(c21) > 1.0
        if np.any(both_active):
            a20 = c20_interp[both_active]
            a21 = c21[both_active]
            # For opposed motors, same-sign torque = fighting
            same_sign = (a20 * a21) > 0
            pct_same = 100.0 * np.sum(same_sign) / len(a20)
            print(f"  Active samples: {np.sum(both_active)}")
            print(f"  Same-sign torque (fighting for opposed motors): {np.sum(same_sign)} ({pct_same:.1f}%)")
            print(f"  Leader mean={np.mean(a21):.1f}A  Follower mean={np.mean(a20):.1f}A")

    # Recovery events
    print("\n--- VELOCITY RECOVERY EVENTS ---")
    events = find_recovery_events(signals, 21)
    if events:
        recovery_times = [e["recovery_ms"] for e in events]
        worst_errors = [e["worst_error_rps"] for e in events]
        rs = stats(recovery_times)
        ws = stats(worst_errors)
        print(f"  Total events: {len(events)}")
        print(f"  Recovery time: mean={rs['mean']:.0f}ms  min={rs['min']:.0f}ms  max={rs['max']:.0f}ms  p95={rs['p95']:.0f}ms")
        print(f"  Worst dip: mean={ws['mean']:.1f}rps  min={ws['min']:.1f}rps  max={ws['max']:.1f}rps")
        under_100 = sum(1 for t in recovery_times if t <= 100)
        under_200 = sum(1 for t in recovery_times if t <= 200)
        print(f"  Events ≤100ms: {under_100}/{len(events)} ({100*under_100/len(events):.0f}%)")
        print(f"  Events ≤200ms: {under_200}/{len(events)} ({100*under_200/len(events):.0f}%)")
        # Show individual events
        print(f"\n  Individual events (first 15):")
        for i, e in enumerate(events[:15]):
            print(f"    #{i+1}: t={e['start_t']:.2f}s  dip={e['worst_error_rps']:.1f}rps  recovery={e['recovery_ms']:.0f}ms  ref={e['ref_rps']:.0f}rps")
    else:
        print("  No recovery events detected")

    # Steady-state noise
    print("\n--- STEADY-STATE VELOCITY NOISE ---")
    vel_data = get(signals, "TalonFX-21/Velocity")
    ref_data = get(signals, "TalonFX-21/PIDVelocity_Reference")
    if vel_data and ref_data:
        vel_arr = np.array(vel_data)
        ref_arr = np.array(ref_data)
        ref_at_vel = np.interp(vel_arr[:, 0], ref_arr[:, 0], ref_arr[:, 1])
        error = vel_arr[:, 1] - ref_at_vel
        active = ref_at_vel > 5.0
        if np.any(active):
            steady = active & (np.abs(error) < 2.0)
            if np.any(steady):
                steady_vel = vel_arr[:, 1][steady]
                steady_err = error[steady]
                print(f"  Steady-state samples (|error|<2rps): {np.sum(steady)}")
                print(f"  Velocity std: {np.std(steady_vel):.3f} rps")
                print(f"  Error mean: {np.mean(steady_err):.3f} rps")
                print(f"  Error std: {np.std(steady_err):.3f} rps")

    # Energy proxy: total stator current * time
    print("\n--- ENERGY PROXY (sum of |stator current| * dt) ---")
    for mid in [20, 21]:
        sc = get(signals, f"TalonFX-{mid}/StatorCurrent")
        if sc and len(sc) > 1:
            t_arr = np.array([t for t, _ in sc])
            c_arr = np.array([abs(v) for _, v in sc])
            dt = np.diff(t_arr)
            energy = np.sum(np.abs(c_arr[1:]) * dt)
            duration = t_arr[-1] - t_arr[0]
            print(f"  Motor {mid}: {energy:.0f} A·s over {duration:.1f}s  (avg {energy/duration:.1f} A)")


def main():
    mm_wpilog = convert_hoot(MM_HOOT, SIGNAL_IDS)
    vv_wpilog = convert_hoot(VV_HOOT, SIGNAL_IDS)

    try:
        mm_signals = read_signals(mm_wpilog)
        vv_signals = read_signals(vv_wpilog)

        analyze_one("MotionMagicVelocityVoltage (MM)", mm_signals)
        analyze_one("VelocityVoltage (VV)", vv_signals)

        # Side-by-side summary
        print(f"\n{'='*80}")
        print(f"  SIDE-BY-SIDE SUMMARY")
        print(f"{'='*80}")

        for metric_name, sig_name in [
            ("Velocity std (rps)", "TalonFX-21/Velocity"),
            ("Stator current mean (A)", "TalonFX-21/StatorCurrent"),
            ("Stator current p95 (A)", "TalonFX-21/StatorCurrent"),
            ("Torque current mean (A)", "TalonFX-21/TorqueCurrent"),
        ]:
            mm_v = vals(mm_signals, sig_name)
            vv_v = vals(vv_signals, sig_name)
            if mm_v and vv_v:
                mm_s = stats(mm_v)
                vv_s = stats(vv_v)
                if "std" in metric_name:
                    print(f"  {metric_name}: MM={mm_s['std']:.2f}  VV={vv_s['std']:.2f}")
                elif "p95" in metric_name:
                    print(f"  {metric_name}: MM={mm_s['p95']:.1f}  VV={vv_s['p95']:.1f}")
                else:
                    print(f"  {metric_name}: MM={mm_s['mean']:.1f}  VV={vv_s['mean']:.1f}")

        # Recovery comparison
        mm_events = find_recovery_events(mm_signals, 21)
        vv_events = find_recovery_events(vv_signals, 21)
        if mm_events and vv_events:
            mm_rt = [e["recovery_ms"] for e in mm_events]
            vv_rt = [e["recovery_ms"] for e in vv_events]
            print(f"\n  Recovery events: MM={len(mm_events)}  VV={len(vv_events)}")
            print(f"  Mean recovery time: MM={np.mean(mm_rt):.0f}ms  VV={np.mean(vv_rt):.0f}ms")
            print(f"  P95 recovery time:  MM={np.percentile(mm_rt, 95):.0f}ms  VV={np.percentile(vv_rt, 95):.0f}ms")
            print(f"  Max recovery time:  MM={np.max(mm_rt):.0f}ms  VV={np.max(vv_rt):.0f}ms")

            mm_u100 = sum(1 for t in mm_rt if t <= 100)
            vv_u100 = sum(1 for t in vv_rt if t <= 100)
            print(f"  ≤100ms: MM={mm_u100}/{len(mm_events)} ({100*mm_u100/len(mm_events):.0f}%)  "
                  f"VV={vv_u100}/{len(vv_events)} ({100*vv_u100/len(vv_events):.0f}%)")

        # Temp comparison
        for mid in [20, 21]:
            mm_t = vals(mm_signals, f"TalonFX-{mid}/DeviceTemp")
            vv_t = vals(vv_signals, f"TalonFX-{mid}/DeviceTemp")
            if mm_t and vv_t:
                print(f"\n  Motor {mid} temp: MM end={mm_t[-1]:.0f}°C  VV end={vv_t[-1]:.0f}°C")

    finally:
        mm_wpilog.unlink(missing_ok=True)
        vv_wpilog.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
