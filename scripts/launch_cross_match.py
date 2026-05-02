"""Show launch counter data across all Bonney Lake matches."""
import json
import glob
import os
import subprocess
import sys

LOG_DIR = r"D:\Temp\2026_BonneyLake\2412_matchesonly"
OUT = r"D:\Temp\launch_all_matches.json"

# Export all matches
files = sorted(glob.glob(os.path.join(LOG_DIR, "*.wpilog")))
cmd = [
    r"D:\Source\LogReader1\.venv\Scripts\logreader.exe",
    "export-results", "launch-counter",
    *files,
    "-o", OUT,
]
subprocess.run(cmd, cwd=r"D:\Source\LogReader1")

# Read and summarize
data = json.loads(open(OUT).read())
print(f"\n{'Match':<12s}  {'Total':>5s}  {'Auto':>5s}  {'Teleop':>6s}  {'Dis':>4s}  {'Launches/min':>12s}")
print("-" * 55)
for m in data:
    src = m["source_file"]
    # Extract match name
    parts = src.replace("FRC_", "").replace(".wpilog", "").split("_")
    name = parts[-1] if len(parts) >= 2 else src

    total = 0
    auto = teleop = disabled = 0
    # Get launch counts from rows or extra
    for row in m.get("rows", []):
        # The rows table has the launch event details
        pass

    extra = m.get("extra", {})
    total = int(extra.get("total_launches", 0)) if "total_launches" in extra else len(m.get("rows", []))

    # Check for phase breakdown in extra
    phase = extra.get("phase_counts", {})
    if phase:
        auto = int(phase.get("AUTONOMOUS", phase.get("Auto", 0)))
        teleop = int(phase.get("TELEOP", phase.get("Teleop", 0)))
        disabled = int(phase.get("DISABLED", phase.get("Disabled", 0)))
    
    # Try to get from summary text
    summary = m.get("summary", "")
    
    # Print what we have
    if total > 0:
        # Rough launches per minute (assuming ~2.5 min match)
        lpm = total / 2.5
        print(f"{name:<12s}  {total:5d}  {auto:5d}  {teleop:6d}  {disabled:4d}  {lpm:12.1f}")
    else:
        print(f"{name:<12s}  (check summary)")

print(f"\n--- Raw data from first match ---")
m0 = data[0]
print(f"Columns: {m0.get('columns', [])}")
print(f"Rows ({len(m0.get('rows', []))}): ", end="")
for r in m0.get("rows", [])[:3]:
    print(r)
print(f"\nExtra keys: {list(m0.get('extra', {}).keys())}")
for k, v in m0.get("extra", {}).items():
    if not isinstance(v, (list, dict)) or (isinstance(v, dict) and len(v) < 10):
        print(f"  {k}: {v}")
