"""Quick script to show Q25 hardware timeline."""
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

print("Match phases:")
for iv in tl.intervals:
    print(f"  {iv.phase.value:12s} {iv.start_s:10.2f}s - {iv.end_s:10.2f}s ({iv.duration_s:.1f}s)")

base_us = tl.intervals[0].start_us if tl.intervals else 0

cameras = _discover_cameras(ld)
for cam_name, signals in sorted(cameras.items()):
    hw_sig = signals.get("hw")
    if not hw_sig or not hw_sig.values:
        continue
    print(f"\n{cam_name} hw signal ({len(hw_sig.values)} samples):")
    hdr = f"  {'Time(s)':>10} {'Phase':>10} {'FPS':>6} {'CPU_C':>7} {'RAM_MB':>7} {'Board_C':>8}"
    print(hdr)
    for tv in hw_sig.values:
        arr = tv.value
        if not isinstance(arr, (list, tuple)) or len(arr) < 4:
            continue
        t_rel = (tv.timestamp_us - base_us) / 1e6
        phase = tl.phase_at(tv.timestamp_us).value if tl else "?"
        fps, cpu, ram, board = float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3])
        marker = " <<<" if cpu > 75 else ""
        print(f"  {t_rel:10.1f} {phase:>10} {fps:6.0f} {cpu:7.1f} {ram:7.0f} {board:8.1f}{marker}")
