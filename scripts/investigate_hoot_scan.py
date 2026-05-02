"""Extract motor 21 (flywheel leader) telemetry from the hoot file.

Uses owlet to convert the CANivore hoot file, then extracts all signals
related to motor 21 during the first 10s after enable.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
os.environ["OWLET_PATH"] = r"c:\Tools\owlet-26.1.0-windowsx86-64.exe"

from logreader.hoot_reader import read_hoot

# The CANivore hoot file from the session
HOOT_CAN = r"D:\Temp\2026-4-8_logs\2026-04-08_17-40-09\ECA0983C3353385320202034170A03FF_2026-04-08_17-40-14.hoot"
# The roboRIO hoot file
HOOT_RIO = r"D:\Temp\2026-4-8_logs\2026-04-08_17-40-09\rio_2026-04-08_17-40-14.hoot"

# First, let's scan the CANivore hoot to see what signals are available
import subprocess

owlet = os.environ["OWLET_PATH"]
print("=" * 80)
print("SCANNING CANivore hoot file for available signals...")
print("=" * 80)
result = subprocess.run(
    [owlet, HOOT_CAN, "--scan"],
    capture_output=True, text=True, timeout=30
)
print(result.stdout)
if result.stderr:
    print("STDERR:", result.stderr[:2000])

print("\n" + "=" * 80)
print("SCANNING roboRIO hoot file for available signals...")
print("=" * 80)
result2 = subprocess.run(
    [owlet, HOOT_RIO, "--scan"],
    capture_output=True, text=True, timeout=30
)
print(result2.stdout)
if result2.stderr:
    print("STDERR:", result2.stderr[:2000])
