"""Compact per-file summary of DS logs from 6PM+."""
import sys, statistics
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from logreader.dslog_reader import read_ds_logs

LOG_DIR = Path(r"D:\Temp\2026-04-6_Practice_Test\dslogs")

files = sorted(f for f in LOG_DIR.glob("*.dslog") if int(f.stem.split()[1].split("_")[0]) >= 18)
print(f"Files from 6PM+: {len(files)}\n")

hdr = f"{'File':<32} {'Dur':>5} {'Ovr':>5} {'CPUmn':>6} {'CPUp95':>7} {'CPUmx':>7} {'CANmn':>6} {'BatMn':>6} {'BO':>4} {'DLwarn':>6}"
print(hdr)
print("-" * len(hdr))

total_ovr = 0
total_dl = 0
for f in files:
    ld = read_ds_logs(str(f))
    cpu = ld.get_signal("/DSLog/CPUUtilization")
    can = ld.get_signal("/DSLog/CANUtilization")
    batt = ld.get_signal("/DSLog/BatteryVoltage")
    bo = ld.get_signal("/DSLog/Status/Brownout")
    ev = ld.get_signal("/DSEvents")

    cpu_vals = [v.value for v in cpu.values] if cpu else []
    can_vals = [v.value for v in can.values] if can else []
    batt_vals = [v.value for v in batt.values] if batt else []

    dur = (cpu.values[-1].timestamp_us - cpu.values[0].timestamp_us) / 1e6 if cpu and len(cpu.values) > 1 else 0

    n_ovr = 0
    n_dl = 0
    if ev:
        for v in ev.values:
            t = str(v.value)
            if "Loop time of" in t and "overrun" in t:
                n_ovr += t.count("overrun")
            if "DataLog: outgoing buffers exceeded" in t:
                n_dl += 1
    total_ovr += n_ovr
    total_dl += n_dl

    cpu_mean = statistics.mean(cpu_vals) * 100 if cpu_vals else 0
    cpu_p95 = sorted(cpu_vals)[int(len(cpu_vals) * 0.95)] * 100 if len(cpu_vals) > 20 else 0
    cpu_max = max(cpu_vals) * 100 if cpu_vals else 0
    can_mean = statistics.mean(can_vals) * 100 if can_vals else 0
    batt_min = min(batt_vals) if batt_vals else 0
    bo_n = sum(1 for v in bo.values if v.value) if bo else 0

    print(f"{f.stem:<32} {dur:>5.0f} {n_ovr:>5} {cpu_mean:>5.1f}% {cpu_p95:>6.1f}% {cpu_max:>6.1f}% {can_mean:>5.1f}% {batt_min:>5.1f}V {bo_n:>4} {n_dl:>6}")

print()
print(f"Total overruns: {total_ovr}")
print(f"Total DataLog buffer warnings: {total_dl}")
