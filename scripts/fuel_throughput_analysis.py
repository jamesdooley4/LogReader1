"""Fuel throughput analysis: identify limiting factors in the scoring pipeline.

Fuel path:  Spindexer (motor 31) -> Feeder (motor 30) -> Flywheels (motors 20, 21)

For each launch event we measure:
 - current spikes / velocity dips at each stage to detect fuel contact
 - transit time from spindexer to feeder, feeder to flywheels
 - flywheel recovery time after each launch
 - inter-launch period (time between successive launches)

Motor mapping:
 - Motor 31 (Spindexer)  -- rio CAN bus
 - Motor 30 (Feeder)     -- CANivore CAN bus
 - Motors 20, 21 (Flywheels) -- rio CAN bus
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

# Ensure logreader is importable
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
os.environ.setdefault("OWLET_PATH", r"c:\Tools\owlet-26.1.0-windowsx86-64.exe")

from logreader.hoot_reader import convert_hoot_to_wpilog
from logreader.wpilog_reader import read_wpilog

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DATA_DIR = Path(r"D:\Temp\2026_WASAM")

# Signal IDs (hex) for targeted owlet extraction
# Motor 30 (feeder) – on CANivore bus
CANIVORE_SIGNALS_M30 = [
    "6fd1e00",   # Velocity
    "6f31e00",   # StatorCurrent
    "6f21e00",   # TorqueCurrent
    "6fe1e00",   # Position
    "6ec1e00",   # MotorVoltage
]

# Motors 20, 21, 31 – on rio bus
RIO_SIGNALS = [
    # Motor 20 (flywheel A)
    "6fd1400",   # Velocity
    "6f31400",   # StatorCurrent
    "6f21400",   # TorqueCurrent
    "6fe1400",   # Position
    "6ec1400",   # MotorVoltage
    # Motor 21 (flywheel B)
    "6fd1500",   # Velocity
    "6f31500",   # StatorCurrent
    "6f21500",   # TorqueCurrent
    "6fe1500",   # Position
    "6ec1500",   # MotorVoltage
    # Motor 31 (spindexer)
    "6fd1f00",   # Velocity
    "6f31f00",   # StatorCurrent
    "6f21f00",   # TorqueCurrent
    "6fe1f00",   # Position
    "6ec1f00",   # MotorVoltage
]

# Matches to analyze (ones with hoot folders available)
MATCH_FOLDERS = sorted(
    p.name for p in DATA_DIR.iterdir()
    if p.is_dir() and p.name.startswith("WASAM_")
)


def ts_and_vals(signal):
    """Extract parallel numpy arrays of (time_seconds, values)."""
    ts = np.array([v.timestamp_us / 1e6 for v in signal.values])
    vs = np.array([v.value for v in signal.values])
    return ts, vs


def find_hoot_files(match_folder: Path):
    """Return (canivore_hoot, rio_hoot) paths or (None, None)."""
    hoots = list(match_folder.glob("*.hoot"))
    canivore = rio = None
    for h in hoots:
        if "_rio_" in h.name:
            rio = h
        else:
            canivore = h
    return canivore, rio


def load_motor_data(canivore_hoot, rio_hoot):
    """Convert and load relevant motor signals from both CAN buses.

    Returns a dict with keys like 'TalonFX-20/Velocity' -> (ts, vals).
    """
    data = {}

    if canivore_hoot:
        print(f"  Converting CANivore hoot (motor 30)...")
        wpilog_path, warnings = convert_hoot_to_wpilog(canivore_hoot, signal_ids=CANIVORE_SIGNALS_M30)
        for w in warnings:
            print(f"    WARNING: {w}")
        try:
            log = read_wpilog(wpilog_path)
            for name, sig in log.signals.items():
                if "TalonFX-30" in name and len(sig.values) > 0:
                    data[name] = ts_and_vals(sig)
        finally:
            wpilog_path.unlink(missing_ok=True)

    if rio_hoot:
        print(f"  Converting rio hoot (motors 20, 21, 31)...")
        wpilog_path, warnings = convert_hoot_to_wpilog(rio_hoot, signal_ids=RIO_SIGNALS)
        for w in warnings:
            print(f"    WARNING: {w}")
        try:
            log = read_wpilog(wpilog_path)
            for name, sig in log.signals.items():
                if any(f"TalonFX-{m}" in name for m in ["20", "21", "31"]) and len(sig.values) > 0:
                    data[name] = ts_and_vals(sig)
        finally:
            wpilog_path.unlink(missing_ok=True)

    return data


def detect_current_spikes(ts, current, threshold_factor=2.0, min_gap_s=0.05):
    """Detect spikes in stator current above threshold_factor * median.

    Returns array of spike onset times.
    """
    if len(current) < 10:
        return np.array([])

    median_c = np.median(np.abs(current))
    if median_c < 0.5:
        # Motor barely running; use absolute threshold
        threshold = 3.0
    else:
        threshold = median_c * threshold_factor

    above = np.abs(current) > threshold
    # Find rising edges
    edges = np.diff(above.astype(int))
    spike_indices = np.where(edges == 1)[0] + 1

    if len(spike_indices) == 0:
        return np.array([])

    # Merge spikes closer than min_gap_s
    spike_times = ts[spike_indices]
    merged = [spike_times[0]]
    for t in spike_times[1:]:
        if t - merged[-1] > min_gap_s:
            merged.append(t)
    return np.array(merged)


def detect_velocity_dips(ts, velocity, dip_fraction=0.05, min_gap_s=0.05):
    """Detect dips in velocity (velocity drops by dip_fraction of its running value).

    Returns array of dip onset times.
    """
    if len(velocity) < 10:
        return np.array([])

    # Only look at periods where motor is spinning
    spinning = np.abs(velocity) > 5.0  # rps threshold
    if not np.any(spinning):
        return np.array([])

    # Compute smoothed velocity
    window = min(20, len(velocity) // 5)
    if window < 3:
        return np.array([])

    kernel = np.ones(window) / window
    smoothed = np.convolve(np.abs(velocity), kernel, mode='same')

    # Find where actual velocity drops below smoothed - threshold
    drop = smoothed - np.abs(velocity)
    threshold = smoothed * dip_fraction
    dipping = (drop > threshold) & spinning

    edges = np.diff(dipping.astype(int))
    dip_indices = np.where(edges == 1)[0] + 1

    if len(dip_indices) == 0:
        return np.array([])

    dip_times = ts[dip_indices]
    merged = [dip_times[0]]
    for t in dip_times[1:]:
        if t - merged[-1] > min_gap_s:
            merged.append(t)
    return np.array(merged)


def detect_launch_events(flywheel_vel_ts, flywheel_vel,
                         flywheel_cur_ts, flywheel_current,
                         vel_dip_fraction=0.03, current_spike_factor=1.5,
                         min_gap_s=0.15):
    """Detect launch events from flywheel velocity dips + current spikes.

    Velocity and current may have different sample rates, so they use
    separate timestamp arrays and are analyzed independently then correlated.

    Returns array of launch times.
    """
    if len(flywheel_vel) < 10:
        return np.array([])

    # Only consider when flywheels are spinning fast (>50% of max speed)
    max_vel = np.percentile(np.abs(flywheel_vel), 95)
    if max_vel < 10:
        return np.array([])

    high_speed_vel = np.abs(flywheel_vel) > max_vel * 0.5

    # Detect velocity dips during high speed
    dip_times = detect_velocity_dips(flywheel_vel_ts[high_speed_vel],
                                     flywheel_vel[high_speed_vel],
                                     dip_fraction=vel_dip_fraction)

    # Detect current spikes (using current's own timestamps, filtered to
    # periods where velocity was high)
    if len(flywheel_current) > 10:
        # Find time ranges where velocity is high
        vel_high_start = flywheel_vel_ts[high_speed_vel].min() if np.any(high_speed_vel) else 0
        vel_high_end = flywheel_vel_ts[high_speed_vel].max() if np.any(high_speed_vel) else 0
        cur_in_range = (flywheel_cur_ts >= vel_high_start) & (flywheel_cur_ts <= vel_high_end)
        if np.sum(cur_in_range) > 10:
            spike_times = detect_current_spikes(flywheel_cur_ts[cur_in_range],
                                                flywheel_current[cur_in_range],
                                                threshold_factor=current_spike_factor)
        else:
            spike_times = np.array([])
    else:
        spike_times = np.array([])

    if len(dip_times) == 0:
        # Fall back to current spikes only
        return spike_times

    # Match dips and spikes within 50ms
    launches = []
    for dt in dip_times:
        nearby_spikes = spike_times[(spike_times > dt - 0.05) & (spike_times < dt + 0.05)]
        if len(nearby_spikes) > 0:
            launches.append(dt)
        else:
            # Still count the dip as a launch candidate
            launches.append(dt)

    if len(launches) == 0:
        return np.array([])

    # Merge launches closer than min_gap_s
    launches = sorted(launches)
    merged = [launches[0]]
    for t in launches[1:]:
        if t - merged[-1] > min_gap_s:
            merged.append(t)
    return np.array(merged)


def measure_flywheel_recovery(ts, velocity, launch_time, window_after=0.5):
    """Measure how long it takes flywheel velocity to recover after a launch.

    Returns recovery_time_ms or None if can't determine.
    """
    mask = (ts >= launch_time) & (ts <= launch_time + window_after)
    if np.sum(mask) < 5:
        return None

    t_slice = ts[mask]
    v_slice = np.abs(velocity[mask])

    # Find the minimum velocity (dip bottom)
    min_idx = np.argmin(v_slice)
    min_vel = v_slice[min_idx]

    # Pre-launch velocity (just before)
    pre_mask = (ts >= launch_time - 0.05) & (ts < launch_time)
    if np.sum(pre_mask) < 2:
        return None
    pre_vel = np.mean(np.abs(velocity[pre_mask]))

    # Recovery = time to get back to 95% of pre-launch velocity
    target = pre_vel * 0.95
    if min_vel >= target:
        return 0.0  # No significant dip

    post_min = v_slice[min_idx:]
    t_post = t_slice[min_idx:]
    recovered = np.where(post_min >= target)[0]
    if len(recovered) == 0:
        return None  # Didn't recover in window

    recovery_t = t_post[recovered[0]] - t_slice[min_idx]
    return recovery_t * 1000  # ms


def measure_velocity_dip_magnitude(ts, velocity, event_time, window=0.1):
    """Measure the magnitude of a velocity dip at event_time.

    Returns (pre_velocity_rps, min_velocity_rps, dip_pct) or None.
    """
    pre_mask = (ts >= event_time - 0.05) & (ts < event_time)
    post_mask = (ts >= event_time) & (ts <= event_time + window)

    if np.sum(pre_mask) < 2 or np.sum(post_mask) < 2:
        return None

    pre_vel = np.mean(np.abs(velocity[pre_mask]))
    min_vel = np.min(np.abs(velocity[post_mask]))

    if pre_vel < 1.0:
        return None

    dip_pct = (pre_vel - min_vel) / pre_vel * 100
    return pre_vel, min_vel, dip_pct


def measure_current_spike_magnitude(ts, current, event_time, window=0.1):
    """Measure the peak current spike at event_time.

    Returns (baseline_A, peak_A) or None.
    """
    pre_mask = (ts >= event_time - 0.1) & (ts < event_time)
    post_mask = (ts >= event_time) & (ts <= event_time + window)

    if np.sum(pre_mask) < 2 or np.sum(post_mask) < 2:
        return None

    baseline = np.mean(np.abs(current[pre_mask]))
    peak = np.max(np.abs(current[post_mask]))

    return baseline, peak


def correlate_fuel_transit(spindexer_spikes, feeder_spikes, flywheel_launches,
                           max_delay_s=0.5):
    """Try to correlate spindexer -> feeder -> flywheel events to measure transit times.

    For each flywheel launch, look backward in time for a feeder spike
    and spindexer spike within max_delay_s.

    Returns list of dicts with transit time measurements.
    """
    transits = []
    for launch_t in flywheel_launches:
        # Find closest preceding feeder spike
        feeder_before = feeder_spikes[feeder_spikes < launch_t]
        feeder_before = feeder_before[feeder_before > launch_t - max_delay_s]
        feeder_t = feeder_before[-1] if len(feeder_before) > 0 else None

        # Find closest preceding spindexer spike
        spindexer_before = spindexer_spikes[spindexer_spikes < launch_t]
        spindexer_before = spindexer_before[spindexer_before > launch_t - max_delay_s * 2]
        spindexer_t = spindexer_before[-1] if len(spindexer_before) > 0 else None

        transit = {"launch_time": launch_t}
        if feeder_t is not None:
            transit["feeder_to_flywheel_ms"] = (launch_t - feeder_t) * 1000
            transit["feeder_time"] = feeder_t
        if spindexer_t is not None:
            transit["spindexer_time"] = spindexer_t
            if feeder_t is not None:
                transit["spindexer_to_feeder_ms"] = (feeder_t - spindexer_t) * 1000
            transit["spindexer_to_flywheel_ms"] = (launch_t - spindexer_t) * 1000

        transits.append(transit)
    return transits


def analyze_match(match_name: str, verbose: bool = True):
    """Analyze fuel throughput for a single match."""
    match_dir = DATA_DIR / match_name
    if not match_dir.is_dir():
        print(f"Skipping {match_name}: no directory")
        return None

    canivore_hoot, rio_hoot = find_hoot_files(match_dir)
    if not canivore_hoot and not rio_hoot:
        print(f"Skipping {match_name}: no hoot files")
        return None

    print(f"\n{'='*70}")
    print(f"MATCH: {match_name}")
    print(f"{'='*70}")

    data = load_motor_data(canivore_hoot, rio_hoot)

    # Also load wpilog for coarse launcher data
    wpilog_files = list(DATA_DIR.glob(f"*{match_name}*.wpilog"))
    wpilog_files = [f for f in wpilog_files if "augmented" not in f.name]
    wpilog_data = {}
    if wpilog_files:
        print(f"  Loading wpilog: {wpilog_files[0].name}")
        log = read_wpilog(wpilog_files[0])
        for name in ["NT:/launcher/velocity", "NT:/launcher/current"]:
            if name in log.signals and len(log.signals[name].values) > 0:
                wpilog_data[name] = ts_and_vals(log.signals[name])

    # Extract per-motor data -- keys have a Phoenix6/ prefix from owlet
    def get(motor_id, signal_name):
        suffix = f"TalonFX-{motor_id}/{signal_name}"
        for key in data:
            if key.endswith(suffix):
                return data[key]
        return np.array([]), np.array([])

    fw20_vel_ts, fw20_vel = get(20, "Velocity")
    fw20_cur_ts, fw20_cur = get(20, "StatorCurrent")
    fw21_vel_ts, fw21_vel = get(21, "Velocity")
    fw21_cur_ts, fw21_cur = get(21, "StatorCurrent")
    feeder_vel_ts, feeder_vel = get(30, "Velocity")
    feeder_cur_ts, feeder_cur = get(30, "StatorCurrent")
    spindex_vel_ts, spindex_vel = get(31, "Velocity")
    spindex_cur_ts, spindex_cur = get(31, "StatorCurrent")

    print(f"\n  Signal counts:")
    print(f"    Flywheel 20: vel={len(fw20_vel)}, cur={len(fw20_cur)}")
    print(f"    Flywheel 21: vel={len(fw21_vel)}, cur={len(fw21_cur)}")
    print(f"    Feeder 30:   vel={len(feeder_vel)}, cur={len(feeder_cur)}")
    print(f"    Spindexer 31: vel={len(spindex_vel)}, cur={len(spindex_cur)}")

    if len(fw20_vel) == 0 and len(fw21_vel) == 0:
        print("  No flywheel data -- skipping")
        return None

    # Use whichever flywheel has more data (prefer motor 20)
    if len(fw20_vel) >= len(fw21_vel):
        fw_vel_ts, fw_vel = fw20_vel_ts, fw20_vel
        fw_cur_ts, fw_cur = fw20_cur_ts, fw20_cur
        fw_label = "Flywheel-20"
    else:
        fw_vel_ts, fw_vel = fw21_vel_ts, fw21_vel
        fw_cur_ts, fw_cur = fw21_cur_ts, fw21_cur
        fw_label = "Flywheel-21"

    # --- Detect launch events from flywheel ---
    launches = detect_launch_events(fw_vel_ts, fw_vel, fw_cur_ts, fw_cur)
    print(f"\n  Detected {len(launches)} launch events from {fw_label}")

    if len(launches) == 0:
        print("  No launches detected -- trying wpilog launcher/velocity")
        if "NT:/launcher/velocity" in wpilog_data:
            wl_ts, wl_vel = wpilog_data["NT:/launcher/velocity"]
            if "NT:/launcher/current" in wpilog_data:
                wl_cur_ts, wl_cur = wpilog_data["NT:/launcher/current"]
            else:
                wl_cur_ts, wl_cur = wl_ts, np.zeros_like(wl_vel)
            launches = detect_launch_events(wl_ts, wl_vel, wl_cur_ts, wl_cur)
            print(f"  Detected {len(launches)} launches from wpilog launcher data")

    if len(launches) == 0:
        print("  Still no launches detected. Skipping.")
        return None

    # --- Detect current spikes for feeder and spindexer ---
    feeder_spikes = detect_current_spikes(feeder_cur_ts, feeder_cur) if len(feeder_cur) > 0 else np.array([])
    spindex_spikes = detect_current_spikes(spindex_cur_ts, spindex_cur) if len(spindex_cur) > 0 else np.array([])

    print(f"  Feeder current spikes: {len(feeder_spikes)}")
    print(f"  Spindexer current spikes: {len(spindex_spikes)}")

    # --- Measure flywheel recovery times ---
    recovery_times = []
    for lt in launches:
        rt = measure_flywheel_recovery(fw_vel_ts, fw_vel, lt)
        if rt is not None:
            recovery_times.append(rt)

    # --- Measure velocity dip magnitudes ---
    fw_dips = []
    for lt in launches:
        dip = measure_velocity_dip_magnitude(fw_vel_ts, fw_vel, lt)
        if dip is not None:
            fw_dips.append(dip)

    feeder_dips = []
    for lt in launches:
        # Feeder dips should happen slightly before the flywheel dip
        dip = measure_velocity_dip_magnitude(feeder_vel_ts, feeder_vel, lt - 0.05, window=0.15)
        if dip is not None:
            feeder_dips.append(dip)

    # --- Measure current spike magnitudes ---
    fw_current_spikes = []
    for lt in launches:
        sp = measure_current_spike_magnitude(fw_cur_ts, fw_cur, lt)
        if sp is not None:
            fw_current_spikes.append(sp)

    feeder_current_spikes_at_launch = []
    for lt in launches:
        sp = measure_current_spike_magnitude(feeder_cur_ts, feeder_cur, lt - 0.05, window=0.15)
        if sp is not None:
            feeder_current_spikes_at_launch.append(sp)

    spindex_current_spikes_at_launch = []
    for lt in launches:
        sp = measure_current_spike_magnitude(spindex_cur_ts, spindex_cur, lt - 0.15, window=0.2)
        if sp is not None:
            spindex_current_spikes_at_launch.append(sp)

    # --- Transit time analysis ---
    transits = correlate_fuel_transit(spindex_spikes, feeder_spikes, launches)

    # --- Inter-launch timing ---
    if len(launches) > 1:
        inter_launch = np.diff(launches) * 1000  # ms
    else:
        inter_launch = np.array([])

    # === REPORT ===
    print(f"\n  {'-'*60}")
    print(f"  FLYWHEEL ANALYSIS ({fw_label})")
    print(f"  {'-'*60}")
    if len(fw_vel) > 0:
        max_vel = np.percentile(np.abs(fw_vel), 95)
        print(f"  Nominal speed: {max_vel:.1f} rps")

    if recovery_times:
        rt = np.array(recovery_times)
        print(f"  Recovery time after launch:")
        print(f"    Mean: {np.mean(rt):.1f} ms")
        print(f"    Median: {np.median(rt):.1f} ms")
        print(f"    Min: {np.min(rt):.1f} ms, Max: {np.max(rt):.1f} ms")
        print(f"    Std: {np.std(rt):.1f} ms")

    if fw_dips:
        dips = np.array(fw_dips)
        print(f"  Velocity dip at launch (n={len(dips)}):")
        print(f"    Mean dip: {np.mean(dips[:,2]):.1f}% (from {np.mean(dips[:,0]):.1f} to {np.mean(dips[:,1]):.1f} rps)")
        print(f"    Median dip: {np.median(dips[:,2]):.1f}%")
        print(f"    Max dip: {np.max(dips[:,2]):.1f}%")

    if fw_current_spikes:
        spks = np.array(fw_current_spikes)
        print(f"  Current spike at launch (n={len(spks)}):")
        print(f"    Baseline: {np.mean(spks[:,0]):.1f} A -> Peak: {np.mean(spks[:,1]):.1f} A")
        print(f"    Max peak: {np.max(spks[:,1]):.1f} A")

    # Second flywheel comparison
    if len(fw20_vel) > 0 and len(fw21_vel) > 0:
        print(f"\n  Both flywheels active:")
        fw20_nominal = np.percentile(np.abs(fw20_vel[np.abs(fw20_vel) > 5]), 95) if np.any(np.abs(fw20_vel) > 5) else 0
        fw21_nominal = np.percentile(np.abs(fw21_vel[np.abs(fw21_vel) > 5]), 95) if np.any(np.abs(fw21_vel) > 5) else 0
        print(f"    Motor 20 nominal: {fw20_nominal:.1f} rps")
        print(f"    Motor 21 nominal: {fw21_nominal:.1f} rps")
        if fw20_nominal > 0 and fw21_nominal > 0:
            ratio = fw20_nominal / fw21_nominal
            print(f"    Speed ratio (20/21): {ratio:.3f}")

    print(f"\n  {'-'*60}")
    print(f"  FEEDER ANALYSIS (motor 30)")
    print(f"  {'-'*60}")
    if len(feeder_vel) > 0:
        feeder_spinning = np.abs(feeder_vel) > 5
        if np.any(feeder_spinning):
            feeder_nominal = np.percentile(np.abs(feeder_vel[feeder_spinning]), 95)
            print(f"  Nominal speed: {feeder_nominal:.1f} rps")
        else:
            print(f"  Feeder not spinning during match")
    if feeder_dips:
        dips = np.array(feeder_dips)
        print(f"  Velocity dip at launch (n={len(dips)}):")
        print(f"    Mean dip: {np.mean(dips[:,2]):.1f}% (from {np.mean(dips[:,0]):.1f} to {np.mean(dips[:,1]):.1f} rps)")
    if feeder_current_spikes_at_launch:
        spks = np.array(feeder_current_spikes_at_launch)
        print(f"  Current spike at launch (n={len(spks)}):")
        print(f"    Baseline: {np.mean(spks[:,0]):.1f} A -> Peak: {np.mean(spks[:,1]):.1f} A")

    print(f"\n  {'-'*60}")
    print(f"  SPINDEXER ANALYSIS (motor 31)")
    print(f"  {'-'*60}")
    if len(spindex_vel) > 0:
        spindex_spinning = np.abs(spindex_vel) > 2
        if np.any(spindex_spinning):
            spindex_nominal = np.percentile(np.abs(spindex_vel[spindex_spinning]), 95)
            print(f"  Nominal speed: {spindex_nominal:.1f} rps")
        else:
            print(f"  Spindexer not spinning during match")
    if spindex_current_spikes_at_launch:
        spks = np.array(spindex_current_spikes_at_launch)
        print(f"  Current spike near launch (n={len(spks)}):")
        print(f"    Baseline: {np.mean(spks[:,0]):.1f} A -> Peak: {np.mean(spks[:,1]):.1f} A")

    print(f"\n  {'-'*60}")
    print(f"  TRANSIT TIMES")
    print(f"  {'-'*60}")
    if transits:
        spx_to_feed = [t["spindexer_to_feeder_ms"] for t in transits if "spindexer_to_feeder_ms" in t]
        feed_to_fw = [t["feeder_to_flywheel_ms"] for t in transits if "feeder_to_flywheel_ms" in t]
        spx_to_fw = [t["spindexer_to_flywheel_ms"] for t in transits if "spindexer_to_flywheel_ms" in t]

        if spx_to_feed:
            a = np.array(spx_to_feed)
            print(f"  Spindexer -> Feeder:    mean={np.mean(a):.0f} ms, median={np.median(a):.0f} ms, range=[{np.min(a):.0f}, {np.max(a):.0f}]")
        if feed_to_fw:
            a = np.array(feed_to_fw)
            print(f"  Feeder -> Flywheel:     mean={np.mean(a):.0f} ms, median={np.median(a):.0f} ms, range=[{np.min(a):.0f}, {np.max(a):.0f}]")
        if spx_to_fw:
            a = np.array(spx_to_fw)
            print(f"  Spindexer -> Flywheel:  mean={np.mean(a):.0f} ms, median={np.median(a):.0f} ms, range=[{np.min(a):.0f}, {np.max(a):.0f}]")
        if not spx_to_feed and not feed_to_fw:
            print(f"  Could not correlate transit events (insufficient spikes)")
    else:
        print(f"  No transit correlations found")

    print(f"\n  {'-'*60}")
    print(f"  THROUGHPUT / LAUNCH RATE")
    print(f"  {'-'*60}")
    print(f"  Total launches detected: {len(launches)}")
    if len(launches) > 1:
        match_duration = launches[-1] - launches[0]
        rate = (len(launches) - 1) / match_duration if match_duration > 0 else 0
        print(f"  Launch window: {match_duration:.1f} s")
        print(f"  Average rate: {rate:.2f} launches/s ({rate*60:.1f}/min)")

    if len(inter_launch) > 0:
        print(f"  Inter-launch intervals:")
        print(f"    Mean: {np.mean(inter_launch):.0f} ms")
        print(f"    Median: {np.median(inter_launch):.0f} ms")
        print(f"    Min: {np.min(inter_launch):.0f} ms, Max: {np.max(inter_launch):.0f} ms")
        print(f"    Std: {np.std(inter_launch):.0f} ms")
        # Histogram
        bins = [0, 200, 300, 400, 500, 750, 1000, 1500, 2000, 5000, 999999]
        labels = ["<200", "200-300", "300-400", "400-500", "500-750", "750-1000",
                   "1000-1500", "1500-2000", "2000-5000", ">5000"]
        hist, _ = np.histogram(inter_launch, bins=bins)
        print(f"    Distribution (ms):")
        for label, count in zip(labels, hist):
            if count > 0:
                print(f"      {label:>10s}: {'#' * count} ({count})")

    if verbose and len(launches) > 0:
        print(f"\n  Individual launch events (first 20):")
        print(f"  {'Time':>8s}  {'Recovery':>10s}  {'FW Dip%':>8s}  {'FW Peak A':>10s}  {'Feed Peak A':>12s}")
        for i, lt in enumerate(launches[:20]):
            rt = measure_flywheel_recovery(fw_vel_ts, fw_vel, lt)
            dip = measure_velocity_dip_magnitude(fw_vel_ts, fw_vel, lt)
            fw_sp = measure_current_spike_magnitude(fw_cur_ts, fw_cur, lt)
            fd_sp = measure_current_spike_magnitude(feeder_cur_ts, feeder_cur, lt - 0.05, window=0.15)

            rt_str = f"{rt:.0f} ms" if rt is not None else "N/A"
            dip_str = f"{dip[2]:.1f}%" if dip is not None else "N/A"
            fw_str = f"{fw_sp[1]:.1f} A" if fw_sp is not None else "N/A"
            fd_str = f"{fd_sp[1]:.1f} A" if fd_sp is not None else "N/A"
            print(f"  {lt:8.2f}  {rt_str:>10s}  {dip_str:>8s}  {fw_str:>10s}  {fd_str:>12s}")

    # --- Identify limiting factor ---
    print(f"\n  {'-'*60}")
    print(f"  LIMITING FACTOR ASSESSMENT")
    print(f"  {'-'*60}")

    limiting_factors = []

    if recovery_times:
        mean_recovery = np.mean(recovery_times)
        if mean_recovery > 150:
            limiting_factors.append(f"Flywheel recovery is slow ({mean_recovery:.0f} ms avg) -- flywheels may be undersized or under-geared")
        elif mean_recovery > 80:
            limiting_factors.append(f"Flywheel recovery is moderate ({mean_recovery:.0f} ms avg) -- room for improvement")

    if len(inter_launch) > 0:
        median_interval = np.median(inter_launch)
        if recovery_times:
            mean_recovery = np.mean(recovery_times)
            if median_interval > mean_recovery * 2:
                limiting_factors.append(
                    f"Inter-launch interval ({median_interval:.0f} ms) is >> recovery time ({mean_recovery:.0f} ms) -- "
                    f"fuel delivery (feeder/spindexer) is likely the bottleneck, not flywheel spin-up"
                )
            elif median_interval < mean_recovery * 1.2:
                limiting_factors.append(
                    f"Inter-launch interval ({median_interval:.0f} ms) ~= recovery time ({mean_recovery:.0f} ms) -- "
                    f"flywheel recovery is the bottleneck"
                )

    if feeder_dips:
        mean_feeder_dip = np.mean(np.array(feeder_dips)[:,2])
        if mean_feeder_dip > 20:
            limiting_factors.append(f"Large feeder velocity dips ({mean_feeder_dip:.0f}%) -- fuel may be stalling in feeder")

    if not limiting_factors:
        limiting_factors.append("Could not conclusively identify a single limiting factor from available data")

    for i, lf in enumerate(limiting_factors, 1):
        print(f"  {i}. {lf}")

    result = {
        "match": match_name,
        "n_launches": len(launches),
        "recovery_times_ms": recovery_times,
        "inter_launch_ms": inter_launch.tolist() if len(inter_launch) > 0 else [],
        "transits": transits,
        "fw_dips": fw_dips,
    }
    return result


def main():
    print("FUEL THROUGHPUT ANALYSIS")
    print("=" * 70)
    print(f"Data directory: {DATA_DIR}")
    print(f"Available matches with hoot data: {MATCH_FOLDERS}")

    all_results = []

    # Analyze a representative sample of matches
    # Start with a few key matches to get a picture
    priority_matches = ["WASAM_Q25", "WASAM_E12", "WASAM_Q44", "WASAM_E8", "WASAM_Q49"]
    matches_to_analyze = [m for m in priority_matches if m in MATCH_FOLDERS]
    if not matches_to_analyze:
        matches_to_analyze = MATCH_FOLDERS[:3]

    for match_name in matches_to_analyze:
        try:
            result = analyze_match(match_name)
            if result:
                all_results.append(result)
        except Exception as e:
            print(f"\n  ERROR analyzing {match_name}: {e}")
            import traceback
            traceback.print_exc()

    # === Cross-match summary ===
    if all_results:
        print(f"\n\n{'='*70}")
        print(f"CROSS-MATCH SUMMARY")
        print(f"{'='*70}")

        all_recovery = []
        all_inter = []
        all_n_launches = []
        all_feed_to_fw = []
        all_spx_to_feed = []

        for r in all_results:
            all_recovery.extend(r["recovery_times_ms"])
            all_inter.extend(r["inter_launch_ms"])
            all_n_launches.append(r["n_launches"])
            for t in r["transits"]:
                if "feeder_to_flywheel_ms" in t:
                    all_feed_to_fw.append(t["feeder_to_flywheel_ms"])
                if "spindexer_to_feeder_ms" in t:
                    all_spx_to_feed.append(t["spindexer_to_feeder_ms"])

        print(f"\n  Matches analyzed: {len(all_results)}")
        print(f"  Total launches: {sum(all_n_launches)}")

        if all_recovery:
            rt = np.array(all_recovery)
            print(f"\n  Flywheel recovery (all matches):")
            print(f"    Mean: {np.mean(rt):.1f} ms, Median: {np.median(rt):.1f} ms")
            print(f"    Range: [{np.min(rt):.0f}, {np.max(rt):.0f}] ms")

        if all_inter:
            il = np.array(all_inter)
            print(f"\n  Inter-launch interval (all matches):")
            print(f"    Mean: {np.mean(il):.0f} ms, Median: {np.median(il):.0f} ms")
            print(f"    Range: [{np.min(il):.0f}, {np.max(il):.0f}] ms")

        if all_feed_to_fw:
            a = np.array(all_feed_to_fw)
            print(f"\n  Feeder -> Flywheel transit (all matches):")
            print(f"    Mean: {np.mean(a):.0f} ms, Median: {np.median(a):.0f} ms")

        if all_spx_to_feed:
            a = np.array(all_spx_to_feed)
            print(f"\n  Spindexer -> Feeder transit (all matches):")
            print(f"    Mean: {np.mean(a):.0f} ms, Median: {np.median(a):.0f} ms")

        # Final verdict
        print(f"\n  {'-'*60}")
        print(f"  OVERALL VERDICT")
        print(f"  {'-'*60}")
        if all_recovery and all_inter:
            mean_rec = np.mean(all_recovery)
            median_inter = np.median(all_inter)
            print(f"  Flywheel recovery: {mean_rec:.0f} ms avg")
            print(f"  Inter-launch gap: {median_inter:.0f} ms median")
            if median_inter > mean_rec * 2:
                print(f"  -> VERDICT: Fuel delivery is the primary bottleneck.")
                print(f"    The flywheels recover in {mean_rec:.0f}ms but the next fuel doesn't arrive")
                print(f"    for {median_inter:.0f}ms. The spindexer/feeder pipeline is the limiting factor.")
            elif median_inter < mean_rec * 1.2:
                print(f"  -> VERDICT: Flywheel recovery is the primary bottleneck.")
                print(f"    Fuel arrives before the flywheels have recovered.")
            else:
                print(f"  -> VERDICT: Both flywheel recovery and fuel delivery contribute to the gap.")


if __name__ == "__main__":
    main()
