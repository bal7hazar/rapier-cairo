#!/usr/bin/env python3
"""Parses the SWEEP lines printed by tests/d_sweep.cairo (from run_sierra-gas.log) and reports max/mean abs error vs libm."""
import math, re, pathlib, collections, json
ROOT = pathlib.Path(__file__).resolve().parent.parent
log = (ROOT / "run_sierra-gas.log").read_text()
ref = {"sin": math.sin, "cos": math.cos, "atan": math.atan}
data = collections.defaultdict(list)
for m in re.finditer(r"^SWEEP (\w+) (\d+) (-?\d+) (\d+) (\d)$", log, re.M):
    name, i, x, mag, sign = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4)), int(m.group(5))
    val = (-mag if sign else mag) / 2**32
    f = next(v for k, v in ref.items() if k in name)
    data[name].append(abs(val - f(x / 2**32)))
out = {}
print("| function | points | max abs error | mean abs error |\n|---|---:|---:|---:|")
for name in sorted(data):
    e = data[name]
    out[name] = {"n": len(e), "max": max(e), "mean": sum(e) / len(e)}
    print(f"| {name} | {len(e)} | {max(e):.3e} | {sum(e)/len(e):.3e} |")
(ROOT / "sweep.json").write_text(json.dumps(out, indent=1))
