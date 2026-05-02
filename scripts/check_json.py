"""Quick check of the exported JSON structure."""
import json

data = json.loads(open(r"D:\Temp\vision_results.json").read())
print(f"Matches: {len(data)}")
for m in data:
    src = m["source_file"]
    print(f"\n{src}:")
    print(f"  rows: {len(m['rows'])}")
    for k, v in m["extra"].items():
        if isinstance(v, dict) and "__skipped__" in v:
            print(f"  extra.{k}: SKIPPED ({v['length']} items)")
        elif isinstance(v, dict):
            print(f"  extra.{k}: dict with {len(v)} keys")
        elif isinstance(v, list):
            print(f"  extra.{k}: list of {len(v)}")
        else:
            vstr = str(v)[:80]
            print(f"  extra.{k}: {type(v).__name__} = {vstr}")
