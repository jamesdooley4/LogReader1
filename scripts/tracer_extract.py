"""Extract full Tracer breakdown messages from dsevents."""
import sys, re
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from logreader.dslog_reader import read_dsevents

LOG_DIR = Path(r"D:\Temp\2026-04-6_Practice_Test\dslogs")
EPOCH_RE = re.compile(r"^\s*(\S+(?:\.\w+)*(?:\(\))?)\s*:\s*([\d.]+)s\s*$", re.MULTILINE)

# Collect all tracer epochs across all 6PM+ files
all_epochs: dict[str, list[float]] = {}
tracer_msg_count = 0

files = sorted(f for f in LOG_DIR.glob("*.dsevents") if int(f.stem.split()[1].split("_")[0]) >= 18)

for f in files:
    ld = read_dsevents(str(f))
    ev = ld.get_signal("/DSEvents")
    if not ev:
        continue
    for v in ev.values:
        text = str(v.value)
        epochs = EPOCH_RE.findall(text)
        if epochs:
            tracer_msg_count += 1
            for comp, dur_str in epochs:
                dur_ms = float(dur_str) * 1000
                all_epochs.setdefault(comp, []).append(dur_ms)

print(f"Found {tracer_msg_count} messages with Tracer breakdowns\n")

if all_epochs:
    import statistics
    print(f"{'Component':<45} {'N':>5} {'Mean':>8} {'P95':>8} {'Max':>8}")
    print("-" * 80)
    for comp, vals in sorted(all_epochs.items(), key=lambda x: max(x[1]), reverse=True)[:30]:
        mean = statistics.mean(vals)
        p95 = sorted(vals)[int(len(vals) * 0.95)] if len(vals) > 5 else max(vals)
        mx = max(vals)
        print(f"  {comp:<43} {len(vals):>5} {mean:>7.3f} {p95:>7.3f} {mx:>7.3f}")

# Also print a few full example messages containing Tracer data
print("\n\n--- Sample full Tracer messages ---")
sample_count = 0
for f in files[:5]:
    ld = read_dsevents(str(f))
    ev = ld.get_signal("/DSEvents")
    if not ev:
        continue
    for v in ev.values:
        text = str(v.value)
        if EPOCH_RE.search(text) and sample_count < 10:
            print(f"\n[{f.stem} @ {v.timestamp_us/1e6:.2f}s]")
            print(text[:500])
            sample_count += 1
