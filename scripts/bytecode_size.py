#!/usr/bin/env python3
"""Compiled class size of the `rapier_sink` contract fixtures (crates/rapier_sink) against the
Starknet limits, and where the CASM felts of a class go.

usage:
  scripts/bytecode_size.py [table]    build crates/rapier_sink (release), print the size table
  scripts/bytecode_size.py snapshot   same, then write gas/bytecode.size
  scripts/bytecode_size.py check      same, then diff against gas/bytecode.size; exit 1 on ANY difference
  scripts/bytecode_size.py attribution [--class C] [--depth N] [--top K] [--strategy S] [--cut LABEL=REGEX ...]
      builds, in a temporary package outside the workspace (scarb only applies the `[cairo]` of the
      workspace root, so `--strategy` sets `inlining-strategy` for the engine too), the fixtures
      with Sierra debug names, and prints for class C (default `GameStep`): its CASM felts per
      module path (N segments) and per function (the K heaviest), then the *exclusive* CASM felts
      of each cut group (`CUTS`, or `--cut`): the matching functions plus every function reachable
      from the entry points only through them, i.e. what the class loses if they are never called.

Measured quantities, per contract (see the `LIMITS` block for their source):
  sierra_felts  length of `sierra_program` in `*.contract_class.json`
  casm_felts    length of `bytecode` in `*.compiled_contract_class.json`
  sierra_bytes  compact JSON of the class as the gateway serializes it (`sierra_program`,
                `contract_class_version`, `entry_points_by_type`, `abi` as a string; without the
                debug info, which a declare transaction does not carry)
  casm_bytes    compact JSON of the compiled class
The build is deterministic for a given toolchain (`.tool-versions`), so the check uses equality.

Attribution: the compiled class splits its bytecode into `bytecode_segment_lengths`, one segment
per Sierra function in program order (then one for the constants); the dev profile keeps the
function names (`sierra_program_debug_info.user_func_names`), and the release profile compiles the
same CASM. The call graph comes from the `.sierra.json` of the same package built as a library.
Exclusive cuts are a lower bound of what removing the code saves: code of the cut group that is
inlined into a kept function stays counted as kept.

Precedent: glam-cairo `scripts/bytecode_size.py`, slingfall `tools/classsize/classsize.py`.
Dependency free (Python 3 standard library only).
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import tomllib
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PACKAGE = "rapier_sink"
SNAPSHOT = ROOT / "gas" / "bytecode.size"
METRICS = ["sierra_felts", "casm_felts", "sierra_bytes", "casm_bytes"]
HEADER = "# contract: " + " ".join(METRICS)

# Starknet limits. Source: https://docs.starknet.io/learn/cheatsheets/chain-info (Starknet v0.14.2
# on Mainnet, v0.14.3 on Sepolia, read 2026-09-21 by glam-cairo R1) and the sequencer that
# enforces them, https://github.com/starkware-libs/sequencer at
# 1c4fa0261847f403fa0e2da81d412a30d56481d9:
# * apollo_gateway `stateless_transaction_validator.rs`: `sierra_program.len()` <=
#   `max_contract_bytecode_size` (81920) and `serde_json::to_string(&contract_class).len()` <=
#   `max_contract_class_object_size` (4089446);
# * apollo_sierra_compilation_config: the Sierra -> CASM compilation fails above
#   `max_bytecode_size` = 80 * 1024 = 81920 CASM felts;
# * apollo_class_manager_config: `max_compiled_contract_class_object_size` = 4089446 bytes.
# scarb 2.19.4 warns on the same four figures. The limits change between Starknet versions: they
# are parameters, update them here.
LIMITS = {
    "sierra_felts": 81920,
    "casm_felts": 81920,
    "sierra_bytes": 4089446,
    "casm_bytes": 4089446,
}

# Engine code a game with balls, cuboids, convex polygons and half-spaces, no joint and no sensor
# (the `GameStep` configuration) never runs: a regex on Sierra function names per group.
CUTS = {
    "joint solver": r"^rapier_dynamics2d::solver::joint::",
    "sensor intersection tests": r"^rapier_geometry2d::dispatch::intersection"
                                 r"|^rapier_dynamics2d::narrow_phase::intersections",
    "capsule / segment generators": r"^rapier_geometry2d::contact_generators::"
                                    r"(capsule_capsule|cuboid_capsule|cuboid_segment|polygon_segment)"
                                    r"|^rapier_geometry2d::contact_generators::polygon_polygon::"
                                    r"contact_manifold_polygon_(capsule|segment)",
    "pair-free fast path (`free_path`)": r"^rapier2d::pipeline::free_path::",
    "sparse step (`active_set`)": r"^rapier2d::pipeline::active_set::sparse_step",
}

# Decomposition of a class into the parts of the step: the first matching regex (on Sierra
# function names) wins; the contact generators are split per module (one per shape pair).
PARTS = [
    ("contact generator `{}`", r"^rapier_geometry2d::contact_generators::(\w+)"),
    ("sensor intersection tests", r"^rapier_geometry2d::dispatch::intersection"
                                  r"|^rapier_dynamics2d::narrow_phase::intersections"),
    ("dispatch table", r"^rapier_geometry2d::dispatch::|^rapier2d::dispatcher::"),
    ("narrow phase (pair loop, manifolds' solver data)", r"^rapier_dynamics2d::narrow_phase"),
    ("geometry kernels (shapes, SAT, clipping, projections)",
     r"^rapier_geometry2d::(shape|sat|clip|point|polygonal_feature|closest_points|manifold|contact)"),
    ("broad phase", r"^rapier_geometry2d::(broad_phase|aabb)"),
    ("joint solver", r"^rapier_dynamics2d::solver::joint"),
    ("contact constraints and solver sweeps", r"^rapier_dynamics2d::solver::"),
    ("sleeping / islands", r"^rapier2d::pipeline::(islands|sleeping)|^rapier_core::data::union_find"),
    ("active set (sparse step)", r"^rapier2d::pipeline::active_set"),
    ("pair-free fast path", r"^rapier2d::pipeline::free_path"),
    ("events", r"^rapier2d::pipeline::force_events|^rapier_dynamics2d::events"),
    ("`WorldState` save / restore and Serde", r"^rapier2d::world::state|Serde|serialize"),
    ("pipeline glue (user changes, stages, step_internal)", r"^rapier2d::pipeline"),
    ("mass properties", r"^rapier_geometry2d::mass|^rapier_dynamics2d::rigid_body::"),
    ("sets, arena, dicts, world API", r"^rapier_core::data|^rapier_dynamics2d::"
                                      r"(collider_set|rigid_body_set|collider)|^rapier2d::world"
                                      r"|^core::dict"),
    ("fixed-point and vector maths", r"^fixed::|^glam::|^rapier_math::|^rapier_core::"),
    ("fixture (scene building, entry points)", r"^rapier_sink::"),
    ("corelib", r"^core::"),
]

# ---------------------------------------------------------------------------------------------
# Measurement


def scarb(cwd, args, note="", profile="dev"):
    # The profile goes through `SCARB_PROFILE`: the sub-command stays first, which the
    # repository's build shims (`scripts/build-shims`) need to take their lock.
    cmd = ["scarb"] + args
    print(f"$ SCARB_PROFILE={profile} {' '.join(cmd)}{note}", file=sys.stderr)
    env = dict(os.environ, SCARB_PROFILE=profile)
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, env=env)
    if p.returncode != 0:
        out = (p.stdout + p.stderr).splitlines()
        sys.exit("scarb failed:\n" + "\n".join(out[-60:]))


def compact(obj):
    return json.dumps(obj, separators=(",", ":"))


def classes(target, package):
    """-> {contract_name: (sierra class, compiled class)} of `package` in `target`."""
    artifacts = json.loads((target / f"{package}.starknet_artifacts.json").read_text())
    return {
        c["contract_name"]: (json.loads((target / c["artifacts"]["sierra"]).read_text()),
                             json.loads((target / c["artifacts"]["casm"]).read_text()))
        for c in artifacts["contracts"]
    }


def measure(target, package):
    """-> {contract_name: {metric: int}}"""
    res = {}
    for name, (sierra, casm) in classes(target, package).items():
        declared = {
            "sierra_program": sierra["sierra_program"],
            "contract_class_version": sierra["contract_class_version"],
            "entry_points_by_type": sierra["entry_points_by_type"],
            "abi": compact(sierra["abi"]),
        }
        res[name] = {
            "sierra_felts": len(sierra["sierra_program"]),
            "casm_felts": len(casm["bytecode"]),
            "sierra_bytes": len(compact(declared)),
            "casm_bytes": len(compact(casm)),
        }
    return res


def print_table(rows, title):
    print(f"\n### {title}\n")
    print("| contract | Sierra felts | × limit | CASM felts | × limit | Sierra class bytes "
          "| CASM class bytes | × limit |")
    print("|---|--:|--:|--:|--:|--:|--:|--:|")
    for name, r in sorted(rows.items(), key=lambda kv: kv[1]["casm_felts"]):
        size = max(r["sierra_bytes"] / LIMITS["sierra_bytes"], r["casm_bytes"] / LIMITS["casm_bytes"])
        print(f"| `{name}` | {r['sierra_felts']:,} | {r['sierra_felts'] / LIMITS['sierra_felts']:.2f} "
              f"| {r['casm_felts']:,} | {r['casm_felts'] / LIMITS['casm_felts']:.2f} "
              f"| {r['sierra_bytes']:,} | {r['casm_bytes']:,} | {size:.2f} |")
    print(f"\nLimits: {LIMITS['sierra_felts']:,} Sierra felts, {LIMITS['casm_felts']:,} CASM felts, "
          f"{LIMITS['sierra_bytes']:,} bytes per class object (`scripts/bytecode_size.py`, `LIMITS`).")


def read_snapshot():
    snap = {}
    if not SNAPSHOT.exists():
        return snap
    for line in SNAPSHOT.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        name, vals = line.split(":", 1)
        snap[name] = dict(zip(METRICS, map(int, vals.split())))
    return snap


def write_snapshot(rows):
    lines = [HEADER] + [f"{n}: " + " ".join(str(rows[n][m]) for m in METRICS) for n in sorted(rows)]
    SNAPSHOT.write_text("\n".join(lines) + "\n")
    print(f"wrote {SNAPSHOT.relative_to(ROOT)}", file=sys.stderr)


def fixtures():
    scarb(ROOT, ["build", "-p", PACKAGE], profile="release")
    return measure(ROOT / "target" / "release", PACKAGE)


# ---------------------------------------------------------------------------------------------
# Attribution


def strategy_toml(strategy):
    return strategy if strategy.lstrip("-").isdigit() else f'"{strategy}"'


def temp_package(work, strategy):
    """The fixtures as a standalone package (its own workspace) in `work`, also built as a
    library (for the call graph), with Sierra debug names (dev profile)."""
    shutil.copytree(ROOT / "crates" / PACKAGE / "src", work / "src")
    pins = tomllib.loads((ROOT / "Scarb.toml").read_text())["workspace"]["dependencies"]
    deps = [f'{d} = "{pins[d]}"' for d in ("fixed", "glam")]
    deps.append(f'rapier2d = {{ path = "{(ROOT / "crates" / "rapier2d").as_posix()}" }}')
    cairo = f"\n[cairo]\ninlining-strategy = {strategy_toml(strategy)}\n" if strategy else ""
    (work / "Scarb.toml").write_text(
        f'[package]\nname = "{PACKAGE}"\nversion = "0.1.0"\nedition = "2024_07"\n\n'
        "[dependencies]\n" + "\n".join(deps) + '\nstarknet = "2.19.4"\n\n'
        "[lib]\nsierra = true\n\n[[target.starknet-contract]]\nsierra = true\ncasm = true\n"
        + cairo)


def call_graph(program):
    """-> {function name: set of callee names} of a `*.sierra.json` program: each function owns
    the statements from its entry point to the next function's (they are laid out contiguously)."""
    statements = program["statements"]
    calls = {}
    for decl in program["libfunc_declarations"]:
        if decl["long_id"]["generic_id"] == "function_call":
            calls[decl["id"]["id"]] = decl["long_id"]["generic_args"][0]["UserFunc"]["id"]
    funcs = sorted((f["entry_point"], f["id"]["id"], f["id"].get("debug_name") or "?")
                   for f in program["funcs"])
    names = {fid: name for _, fid, name in funcs}
    graph = {}
    for i, (entry, _, name) in enumerate(funcs):
        end = funcs[i + 1][0] if i + 1 < len(funcs) else len(statements)
        callees = set()
        for s in statements[entry:end]:
            inv = s.get("Invocation")
            if inv and inv["libfunc_id"]["id"] in calls:
                callees.add(names[calls[inv["libfunc_id"]["id"]]])
        graph[name] = callees
    return graph


def casm_by_function(sierra, casm):
    """-> {function name: CASM felts} of one compiled class (dev profile: names kept)."""
    names = dict(sierra["sierra_program_debug_info"]["user_func_names"])
    segments = casm["bytecode_segment_lengths"]
    if len(segments) not in (len(names), len(names) + 1):
        sys.exit(f"attribution: {len(segments)} bytecode segments for {len(names)} functions")
    sizes = defaultdict(int)
    for i in range(len(names)):
        sizes[names[i]] += segments[i]
    return sizes, sum(segments[len(names):])


def module_key(name, depth):
    # Generic arguments and loop suffixes carry `::` / `[..]` too: cut them before splitting.
    base = re.sub(r"[<{\[].*", "", name)
    return "::".join([p for p in base.split("::") if p][:depth])


def exclusive(sizes, graph, roots, pattern):
    """CASM felts (and count) of the functions matching `pattern` plus those reachable from
    `roots` only through them."""
    cut = re.compile(pattern)
    seen, stack = set(), list(roots)
    while stack:
        name = stack.pop()
        if name in seen or cut.search(name):
            continue
        seen.add(name)
        stack.extend(graph.get(name, ()))
    kept = sum(sizes.get(n, 0) for n in seen)
    return sum(sizes.values()) - kept, len(sizes) - len(seen & set(sizes))


def attribution(cls, depth, top, strategy, cuts):
    with tempfile.TemporaryDirectory(prefix="bytecode_size_") as tmp:
        work = Path(tmp)
        temp_package(work, strategy)
        scarb(work, ["build"], f"  (dev profile, inlining-strategy = {strategy or 'default'})")
        target = work / "target" / "dev"
        program = json.loads((target / f"{PACKAGE}.sierra.json").read_text())
        built = classes(target, PACKAGE)
    if cls not in built:
        sys.exit(f"attribution: no class `{cls}` (have {', '.join(sorted(built))})")
    sierra, casm = built[cls]
    sizes, consts = casm_by_function(sierra, casm)
    graph = call_graph(program.get("program", program))
    total = len(casm["bytecode"])
    groups = defaultdict(lambda: [0, 0])
    for name, n in sizes.items():
        g = groups[module_key(name, depth)]
        g[0] += n
        g[1] += 1
    parts = defaultdict(lambda: [0, 0])
    for name, n in sizes.items():
        label = "other"
        for fmt, pattern in PARTS:
            m = re.search(pattern, name)
            if m:
                label = fmt.format(*m.groups())
                break
        parts[label][0] += n
        parts[label][1] += 1
    print(f"\n### `{cls}`: CASM felts by part of the step; {total:,} felts, {len(sizes):,} "
          f"functions, constants segment {consts:,}\n")
    print("| part | CASM felts | share | functions |")
    print("|---|--:|--:|--:|")
    for key, (n, count) in sorted(parts.items(), key=lambda kv: -kv[1][0]):
        print(f"| {key} | {n:,} | {100 * n / total:.1f} % | {count} |")
    print(f"\n### `{cls}`: the {top} heaviest modules (depth {depth})\n")
    print("| module | CASM felts | share | functions |")
    print("|---|--:|--:|--:|")
    for key, (n, count) in sorted(groups.items(), key=lambda kv: -kv[1][0])[:top]:
        print(f"| `{key}` | {n:,} | {100 * n / total:.1f} % | {count} |")
    print(f"\n### `{cls}`: the {top} heaviest functions\n")
    print("| function | CASM felts | share |")
    print("|---|--:|--:|")
    for name, n in sorted(sizes.items(), key=lambda kv: -kv[1])[:top]:
        short = re.sub(r"\{.*\}", "{..}", name)
        print(f"| `{short[:140]}` | {n:,} | {100 * n / total:.1f} % |")
    roots = [n for n in sizes if "__wrapper__" in n]
    print(f"\n### `{cls}`: exclusive CASM felts of each cut (lower bound; roots: the entry points)\n")
    print("| cut | CASM felts | share | functions |")
    print("|---|--:|--:|--:|")
    for label, pattern in list(cuts.items()) + [("all of the above", "|".join(cuts.values()))]:
        n, count = exclusive(sizes, graph, roots, pattern)
        print(f"| {label} | {n:,} | {100 * n / total:.1f} % | {count} |")


# ---------------------------------------------------------------------------------------------


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cmd", nargs="?", default="table",
                    choices=["table", "snapshot", "check", "attribution"])
    ap.add_argument("--class", dest="cls", default="GameStep", help="attribution: contract name")
    ap.add_argument("--depth", type=int, default=3, help="attribution: module path segments")
    ap.add_argument("--top", type=int, default=30, help="attribution: heaviest functions shown")
    ap.add_argument("--strategy", help="attribution: inlining-strategy (default, avoid or a number)")
    ap.add_argument("--cut", action="append", default=[],
                    help="attribution: LABEL=REGEX on function names, replaces `CUTS` (repeatable)")
    a = ap.parse_args()

    if a.cmd == "attribution":
        cuts = dict(c.split("=", 1) for c in a.cut) if a.cut else CUTS
        attribution(a.cls, a.depth, a.top, a.strategy, cuts)
        return

    rows = fixtures()
    print_table(rows, "rapier_sink fixtures (release profile)")
    if a.cmd == "snapshot":
        write_snapshot(rows)
    elif a.cmd == "check":
        snap, bad = read_snapshot(), []
        for name in sorted(set(rows) | set(snap)):
            new, old = rows.get(name), snap.get(name)
            if new != old:
                bad.append(f"{name}: {old} -> {new}")
        if bad:
            print("\n".join(bad), file=sys.stderr)
            sys.exit(f"bytecode size mismatch ({len(bad)}). Run `scripts/bytecode_size.py snapshot` "
                     "and commit gas/bytecode.size.")
        print(f"\nbytecode size snapshot OK ({len(rows)} contracts)", file=sys.stderr)


if __name__ == "__main__":
    main()
