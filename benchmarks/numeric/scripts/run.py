#!/usr/bin/env python3
"""Runs snforge twice (cairo-steps / sierra-gas), subtracts per-module baselines, writes results.json + results.md.

Test naming convention inside a module:  base_<k>  = baseline,  <k>_<op> = benchmark using baseline <k>.
"""
import subprocess, re, json, sys, pathlib, collections
ROOT = pathlib.Path(__file__).resolve().parent.parent
filt = sys.argv[1:]  # optional test filter

def run(mode):
    cmd = ["snforge", "test", "-p", "numbench", "--detailed-resources", "--tracked-resource", mode] + filt
    out = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True).stdout
    (ROOT / f"run_{mode}.log").write_text(out)
    if "[FAIL]" in out or "[PASS]" not in out:
        print(out[-3000:]); sys.exit(1)
    res = {}
    cur = None
    for line in out.splitlines():
        m = re.match(r"\[PASS\] numbench_\w+::([\w:]+) \(.*l2_gas: ~(\d+)\)", line)
        if m:
            cur = res.setdefault(m.group(1), {"l2_gas": int(m.group(2))}); continue
        if cur is None: continue
        m = re.match(r"\s+steps: (\d+)", line)
        if m: cur["steps"] = int(m.group(1))
        m = re.match(r"\s+memory holes: (\d+)", line)
        if m: cur["holes"] = int(m.group(1))
        m = re.match(r"\s+sierra gas: (\d+)", line)
        if m: cur["sierra_gas"] = int(m.group(1))
        m = re.match(r"\s+builtins: \((.*)\)", line)
        if m:
            cur["builtins"] = {k: int(v) for k, v in re.findall(r"Builtin\((\w+)\): (\d+)", m.group(1))}
    return res

steps = run("cairo-steps"); gas = run("sierra-gas")
mods = collections.defaultdict(dict)
for full, r in steps.items():
    mod, name = full.rsplit("::", 1)
    r = dict(r); r["l2_gas"] = gas[full]["l2_gas"]
    mods[mod][name] = r

BUILTINS = ["range_check", "bitwise", "range_check96", "add_mod", "mul_mod", "pedersen", "poseidon"]
results = {}
md = []
for mod in sorted(mods):
    tests = mods[mod]
    bases = {n[5:]: r for n, r in tests.items() if n.startswith("base_")}
    rows = []
    for n, r in sorted(tests.items()):
        if n.startswith("base_") or n.startswith("check_"): continue
        k, _, op = n.partition("_")
        if k not in bases:
            rows.append((n, r, None)); continue
        b = bases[k]
        net = {"steps": r["steps"] - b["steps"], "l2_gas": r["l2_gas"] - b["l2_gas"],
               "builtins": {x: r["builtins"].get(x, 0) - b["builtins"].get(x, 0) for x in set(r["builtins"]) | set(b["builtins"])}}
        rows.append((op, r, net))
    results[mod] = {"baselines": bases, "ops": {op: {"raw": r, "net": net} for op, r, net in rows}}
    md.append(f"\n### {mod}\n")
    md.append("Baselines: " + ", ".join(f"`{k}` = {b['steps']} steps / {b['l2_gas']} gas" for k, b in bases.items()) + "\n")
    used = [x for x in BUILTINS if any((net or {}).get("builtins", {}).get(x) for _, _, net in rows)]
    md.append("| op | steps | " + " | ".join(used) + " | l2_gas |")
    md.append("|---|---:|" + "---:|" * (len(used) + 1))
    for op, r, net in rows:
        if net is None:
            md.append(f"| {op} (raw, no baseline) | {r['steps']} | " + " | ".join(str(r['builtins'].get(x, 0)) for x in used) + f" | {r['l2_gas']} |")
        else:
            md.append(f"| {op} | {net['steps']} | " + " | ".join(str(net['builtins'].get(x, 0)) for x in used) + f" | {net['l2_gas']} |")
(ROOT / "results.json").write_text(json.dumps(results, indent=1))
(ROOT / "results.md").write_text("\n".join(md) + "\n")
print("\n".join(md))
