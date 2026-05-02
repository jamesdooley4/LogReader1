"""Check turret motor actual position vs reference from hoot in both matches.
Also check if RotorPosition is consistent with Position (gear ratio)."""
import sys, math
sys.path.insert(0, r"D:\Source\LogReader1\src")
from logreader.wpilog_reader import read_wpilog

GEAR_RATIO = 72  # SensorToMechanismRatio

for label, path, t0_hoot in [
    ("Q79", r"D:\Temp\2026_DCMPs\PNCMP_Q79\canivore_q79.wpilog", 62.154),
    ("Q85", r"D:\Temp\2026_DCMPs\PNCMP_Q85\canivore_q85.wpilog", 45.050),
]:
    print(f"\n{'='*80}")
    print(f"  {label}")
    print(f"{'='*80}")
    print(f"Loading {label} hoot...")
    h = read_wpilog(path)

    pos = h.signals["Phoenix6/TalonFX-23/Position"]
    rotor = h.signals["Phoenix6/TalonFX-23/RotorPosition"]
    ref = h.signals["Phoenix6/TalonFX-23/PIDPosition_Reference"]
    err = h.signals["Phoenix6/TalonFX-23/PIDPosition_ClosedLoopError"]

    # Sample every 1s from -5 to +15
    print(f"\n  {'MT':>6}  {'Pos(rot)':>10}  {'Ref(rot)':>10}  {'Err(rot)':>10}  {'Rotor':>10}  {'Rotor/72':>10}  {'RvsP':>8}")
    last = -999

    def nearest(t_abs, sig, max_gap=0.5):
        best = None
        best_dt = float("inf")
        for v in sig.values:
            dt = abs(v.timestamp_us / 1e6 - t_abs)
            if dt < best_dt:
                best_dt = dt
                best = v.value
            if v.timestamp_us / 1e6 > t_abs + max_gap:
                break
        return best if best_dt <= max_gap else None

    for v in pos.values:
        t = v.timestamp_us / 1e6
        mt = t - t0_hoot
        if mt > 15:
            break
        if mt - last >= 1.0 and mt >= -2:
            p = v.value
            r = nearest(t, ref)
            e = nearest(t, err)
            rtr = nearest(t, rotor)
            rtr_mech = rtr / GEAR_RATIO if rtr is not None else None
            rvsp = (rtr_mech - p) if rtr_mech is not None else None

            r_s = f"{r:.6f}" if r is not None else "     N/A"
            e_s = f"{e:.6f}" if e is not None else "     N/A"
            rtr_s = f"{rtr:.3f}" if rtr is not None else "     N/A"
            rm_s = f"{rtr_mech:.6f}" if rtr_mech is not None else "     N/A"
            rv_s = f"{rvsp:+.6f}" if rvsp is not None else "     N/A"
            print(f"  {mt:+6.1f}  {p:10.6f}  {r_s:>10}  {e_s:>10}  {rtr_s:>10}  {rm_s:>10}  {rv_s:>8}")
            last = mt

    # Check: what is the RotorPosition at boot (before any zero)?
    print(f"\n  First 5 RotorPosition values:")
    for v in rotor.values[:5]:
        mt = v.timestamp_us / 1e6 - t0_hoot
        mech = v.value / GEAR_RATIO
        print(f"    t={mt:+.3f}s  rotor={v.value:.3f}  mech={mech:.6f} ({mech*360:.2f} deg)")

    # And first 5 Position values
    print(f"\n  First 5 Position values:")
    for v in pos.values[:5]:
        mt = v.timestamp_us / 1e6 - t0_hoot
        print(f"    t={mt:+.3f}s  pos={v.value:.6f} ({v.value*360:.2f} deg)")

    # Check RotorPosition at the zero moment
    # The zero for Q85 was at t0_hoot - (45.05 - 45.05 + 33.62) before match...
    # Actually we need to find when Position jumps to 0
    print(f"\n  Position jump to ~0 (zero event):")
    prev_p = None
    for v in pos.values:
        mt = v.timestamp_us / 1e6 - t0_hoot
        if mt > 0:
            break
        if prev_p is not None and abs(v.value) < 0.001 and abs(prev_p) > 0.001:
            # Found a zero! Get rotor at this time
            rtr_at_zero = nearest(v.timestamp_us / 1e6, rotor, max_gap=0.2)
            print(f"    t={mt:+.3f}s  pos: {prev_p:.6f} -> {v.value:.6f}")
            if rtr_at_zero is not None:
                print(f"    RotorPosition at zero: {rtr_at_zero:.3f} (mech={rtr_at_zero/GEAR_RATIO:.6f})")
        prev_p = v.value

print("\nDONE")
