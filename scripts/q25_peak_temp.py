"""Find peak CPU temps in Q25."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from logreader.wpilog_reader import read_wpilog
from logreader.analyzers.vision_analysis import _discover_cameras
from logreader.analyzers.match_phases import detect_match_phases, MatchPhaseTimeline

ld = read_wpilog(r"D:\Temp\2026_WASAM\FRC_20260321_225528_WASAM_Q25.wpilog")
tl = detect_match_phases(ld)
if tl is None:
    tl = MatchPhaseTimeline()

base_us = tl.intervals[0].start_us if tl.intervals else 0

cameras = _discover_cameras(ld)
for cam_name, signals in sorted(cameras.items()):
    hw_sig = signals.get("hw")
    if not hw_sig or not hw_sig.values:
        print(f"{cam_name}: no hw signal")
        continue
    
    # Find peak CPU temp
    peak_cpu = 0
    peak_tv = None
    all_cpu_temps = []
    for tv in hw_sig.values:
        arr = tv.value
        if not isinstance(arr, (list, tuple)) or len(arr) < 4:
            continue
        cpu = float(arr[1])
        all_cpu_temps.append(cpu)
        if cpu > peak_cpu:
            peak_cpu = cpu
            peak_tv = tv
    
    if peak_tv:
        t_rel = (peak_tv.timestamp_us - base_us) / 1e6
        phase = tl.phase_at(peak_tv.timestamp_us).value
        arr = peak_tv.value
        print(f"{cam_name}: peak CPU={peak_cpu:.1f}C at t={t_rel:.1f}s ({phase})")
        print(f"  FPS={float(arr[0]):.0f}, RAM={float(arr[2]):.0f}MB, Board={float(arr[3]):.1f}C")
    
    # Show top 10 hottest readings
    temps_with_time = []
    for tv in hw_sig.values:
        arr = tv.value
        if not isinstance(arr, (list, tuple)) or len(arr) < 4:
            continue
        cpu = float(arr[1])
        t_rel = (tv.timestamp_us - base_us) / 1e6
        phase = tl.phase_at(tv.timestamp_us).value
        temps_with_time.append((cpu, t_rel, phase, float(arr[0]), float(arr[3])))
    
    temps_with_time.sort(reverse=True)
    print(f"  Top 10 hottest readings:")
    for cpu, t, ph, fps, board in temps_with_time[:10]:
        print(f"    {cpu:6.1f}C at t={t:7.1f}s ({ph:>8s}) FPS={fps:.0f} Board={board:.1f}C")
    
    print(f"  Total samples: {len(all_cpu_temps)}, range: {min(all_cpu_temps):.1f} - {max(all_cpu_temps):.1f}C")
    print()

# Also check raw signal names for any other temperature sources
print("\nAll signals containing 'temp' or 'cpu' or 'hw':")
for name in sorted(ld.signals.keys()):
    if any(kw in name.lower() for kw in ['temp', 'cpu', 'hw', 'heat']):
        sig = ld.signals[name]
        print(f"  {name}: {sig.info.type.name}, {len(sig.values)} samples")
