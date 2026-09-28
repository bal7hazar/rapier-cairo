#!/usr/bin/env python3
"""Compiled class size of the `rapier_sink` contract fixtures (crates/rapier_sink) and of the declared
classes of `rapier2d_classes` (crates/rapier2d_classes) against the Starknet limits, and where the
CASM felts of a class go.

usage:
  scripts/bytecode_size.py [table]    build crates/rapier_sink and crates/rapier2d_classes (release)
                                      and the executable programs, print the size tables
  scripts/bytecode_size.py snapshot   same, then write gas/bytecode.size
  scripts/bytecode_size.py check      same, then diff against gas/bytecode.size; exit 1 on ANY difference,
                                      when a class of `DECLARED` exceeds `DECLARED_LIMIT` Sierra or CASM
                                      felts, and when a class of `SNIP36` uses a builtin or a syscall the
                                      SNIP-36 virtual OS / prover rejects
  scripts/bytecode_size.py attribution [--class C] [--by parts|phases] [--depth N] [--top K] [--strategy S]
                                      [--cut LABEL=REGEX ...]
      builds, in a temporary package outside the workspace (scarb only applies the `[cairo]` of the
      workspace root, so `--strategy` sets `inlining-strategy` for the engine too), the fixtures
      with Sierra debug names, and prints for class C (default `GameStep`): its CASM felts (and
      an estimate of its Sierra felts, pro rata of Sierra statements) per part (`PARTS`) or per
      stage of the step (`PHASES`, `--by phases`), per module path (N segments) and per function
      (the K heaviest), then the *exclusive* CASM felts
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

Declared classes (CS4): the classes a game declares and library-calls (`DECLARED`, built from
`crates/rapier2d_classes`) must each stay within `DECLARED_LIMIT` Sierra and CASM felts (the
programme's target for the SNIP-36 path, 2026-09-27); `check` fails above it. The classes of
`SNIP36` (the declared classes and their caller fixture) must also run under the SNIP-36 virtual OS
(programme's SN1 verdict, 2026-09-28): no entry point of the compiled class may list a builtin of
`REJECTED_BUILTINS` (`entry_points_by_type[*][*].builtins`), and the Sierra program may not reference
a syscall libfunc of `FORBIDDEN_SYSCALLS`. The release Sierra carries no debug names, but its felt
encoding stores each libfunc's generic id as a short-string felt, and the compression keeps every
distinct felt verbatim: a libfunc is referenced iff its name's felt is in `sierra_program`
(`SYSCALLS` are all shorter than 32 bytes).

Programs (CS2): `crates/rapier_sink/programs/lib.cairo` (with `crates/rapier_sink/src/scene.cairo`)
built as a temporary package with one `[[target.executable]]` per function of `PROGRAMS`
(`enable-gas = false`, release profile), outside the workspace:
  program_felts  length of `program.bytecode` in `<name>.executable.json`: what a proof of the
                 program hashes (the bootloader's program hash)

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
# The declared classes (CS4): built from their own package, snapshotted with the fixtures (contract
# names are unique across both packages), each checked against `DECLARED_LIMIT`.
CLASSES_PACKAGE = "rapier2d_classes"
DECLARED = ["ContactBallClass", "ContactPolygonClass", "SolverClass", "SolveAdvanceClass",
            "IslandsClass", "BroadPhaseClass", "MassClass"]
DECLARED_LIMIT = 73728
# SNIP-36 (SN1, 2026-09-28): what the virtual OS fails on and what its prover rejects. Checked on the
# declared classes and on their caller fixtures (CS4's layout; CS5's, every stage out, and its variants).
SNIP36 = DECLARED + ["Split4Step", "StagesSplitStep", "StagesBatchedStep", "StagesHybridStep"]
REJECTED_BUILTINS = ["ecdsa", "range_check96", "add_mod", "mul_mod"]
FORBIDDEN_SYSCALLS = ["deploy_syscall", "replace_class_syscall", "get_block_hash_syscall",
                      "meta_tx_v0_syscall"]
# Every syscall libfunc reported in the tables (the forbidden ones included).
SYSCALLS = ["call_contract_syscall", "library_call_syscall", "storage_read_syscall",
            "storage_write_syscall", "get_execution_info_syscall", "get_execution_info_v2_syscall",
            "get_class_hash_at_syscall", "send_message_to_l1_syscall", "emit_event_syscall",
            "keccak_syscall", "sha256_process_block_syscall"] + FORBIDDEN_SYSCALLS
SNAPSHOT = ROOT / "gas" / "bytecode.size"
METRICS = ["sierra_felts", "casm_felts", "sierra_bytes", "casm_bytes"]
HEADER = "# contract: " + " ".join(METRICS)
PROGRAMS_PACKAGE = "rapier_sink_programs"
# The executable fixtures (`crates/rapier_sink/programs/lib.cairo`): the game world stepped with
# force events under each step configuration; `full` minus the others gives each strategy's share.
PROGRAMS = ["full", "basic", "no_joints", "no_sensors", "no_composites", "basic_dispatcher"]
PROGRAM_HEADER = "# program: program_felts"
PROGRAM_PREFIX = "program."
# The workspace crates `crates/rapier_sink/src` uses (its `[dependencies]`), as path dependencies of
# the attribution's temporary package.
SINK_CRATES = ["rapier2d", "rapier2d_classes", "rapier_dynamics2d", "rapier_geometry2d"]

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

# The same decomposition along the stages of one game step (CS3, `--by phases`): the contact
# generators per shape-pair module, then each stage of `step_with_force_events_with`; the entry
# points and the crossing `Serde` of the split fixtures (`rapier_sink::split*`) count with the
# `WorldState` codec. Shared code (fixed-point maths, arena and dicts, corelib) keeps its own rows.
PHASES = [
    ("contact generator `{}`", r"^rapier_geometry2d::contact_generators::(\w+)"),
    ("contact dispatch", r"^rapier_geometry2d::dispatch::|^rapier2d::dispatcher::"),
    ("geometry kernels (SAT, clipping, projections, features)",
     r"^rapier_geometry2d::(sat|clip|point|polygonal_feature|closest_points|manifold|contact)"),
    ("narrow phase (pair loop, solver contacts)",
     r"^rapier_dynamics2d::narrow_phase|^rapier2d::pipeline::stages::narrow"),
    ("broad phase (proxies, AABBs, pairs)",
     r"^rapier_geometry2d::(broad_phase|aabb)|^rapier_geometry2d::shape::\S*(aabb|bounding)"
     r"|collision_inputs|collision_proxies|collision_scratch|near_statics"),
    ("constraint build", r"^rapier_dynamics2d::solver::(island::sweeps::split::generation|contact)"),
    ("solve (sweeps)", r"^rapier_dynamics2d::solver::island::(sweeps|solve_input|run)"),
    ("integrate (bodies, free bodies, damping)", r"^rapier_dynamics2d::solver::"),
    ("islands / sleep", r"^rapier2d::pipeline::(islands|sleeping)|^rapier_core::data::union_find"),
    ("active set (sparse step)", r"^rapier2d::pipeline::active_set"),
    ("pair-free fast path", r"^rapier2d::pipeline::free_path"),
    ("force events", r"^rapier2d::pipeline::force_events|^rapier_dynamics2d::events"),
    ("`WorldState` encode / decode, crossing Serde", r"^rapier2d::world::state|Serde|serialize"),
    ("user changes, mass properties", r"^rapier2d::pipeline::user_changes|^rapier_geometry2d::mass"
                                      r"|^rapier_dynamics2d::rigid_body::"),
    ("shapes (other methods)", r"^rapier_geometry2d::shape"),
    ("step glue (step_internal, fused solve and advance)", r"^rapier2d::pipeline"),
    ("sets, arena, dicts, world API", r"^rapier_core::data|^rapier_dynamics2d::"
                                      r"(collider_set|rigid_body_set|collider)|^rapier2d::world"
                                      r"|^core::dict"),
    ("fixed-point and vector maths", r"^fixed::|^glam::|^rapier_math::|^rapier_core::"),
    ("library-call strategies (crossing, write-back)", r"^rapier2d_classes::"),
    ("fixture (entry points, split dispatchers)", r"^rapier_sink::"),
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


def short_string(text):
    return int.from_bytes(text.encode(), "big")


def interface(sierra, casm):
    """-> (builtins of the entry points, syscall libfuncs of `SYSCALLS` referenced) of a class."""
    builtins = sorted({b for eps in casm["entry_points_by_type"].values() for ep in eps
                       for b in ep["builtins"]})
    felts = {int(f, 16) for f in sierra["sierra_program"]}
    return builtins, [s for s in SYSCALLS if short_string(s) in felts]


INTERFACES = {}


def measure(target, package):
    """-> {contract_name: {metric: int}}; fills `INTERFACES` (builtins and syscalls per class)."""
    res = {}
    for name, (sierra, casm) in classes(target, package).items():
        INTERFACES[name] = interface(sierra, casm)
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
    """-> ({contract: {metric: int}}, {program: {"program_felts": int}})"""
    snap, progs = {}, {}
    if not SNAPSHOT.exists():
        return snap, progs
    for line in SNAPSHOT.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        name, vals = line.split(":", 1)
        if name.startswith(PROGRAM_PREFIX):
            progs[name[len(PROGRAM_PREFIX):]] = {"program_felts": int(vals)}
        else:
            snap[name] = dict(zip(METRICS, map(int, vals.split())))
    return snap, progs


def write_snapshot(rows, progs):
    lines = [HEADER] + [f"{n}: " + " ".join(str(rows[n][m]) for m in METRICS) for n in sorted(rows)]
    lines += [PROGRAM_HEADER] + [f"{PROGRAM_PREFIX}{n}: {progs[n]['program_felts']}" for n in PROGRAMS]
    SNAPSHOT.write_text("\n".join(lines) + "\n")
    print(f"wrote {SNAPSHOT.relative_to(ROOT)}", file=sys.stderr)


def fixtures():
    rows = {}
    for package in (PACKAGE, CLASSES_PACKAGE):
        scarb(ROOT, ["build", "-p", package], profile="release")
        measured = measure(ROOT / "target" / "release", package)
        if set(measured) & set(rows):
            sys.exit(f"contract names shared by {PACKAGE} and {CLASSES_PACKAGE}: "
                     f"{', '.join(sorted(set(measured) & set(rows)))}")
        rows.update(measured)
    return rows


def declared_over(rows):
    """-> the problems of the declared classes: missing, or over `DECLARED_LIMIT` felts."""
    bad = []
    for name in DECLARED:
        r = rows.get(name)
        if r is None:
            bad.append(f"{name}: declared class not built")
            continue
        for metric in ("sierra_felts", "casm_felts"):
            if r[metric] > DECLARED_LIMIT:
                bad.append(f"{name}: {metric} {r[metric]:,} > {DECLARED_LIMIT:,}")
    return bad


def snip36_problems():
    """-> the uses of a rejected builtin or a forbidden syscall by a class of `SNIP36`."""
    bad = []
    for name in SNIP36:
        if name not in INTERFACES:
            bad.append(f"{name}: not built")
            continue
        builtins, syscalls = INTERFACES[name]
        bad += [f"{name}: builtin `{b}` rejected by the SNIP-36 prover"
                for b in builtins if b in REJECTED_BUILTINS]
        bad += [f"{name}: `{s}` fails in the SNIP-36 virtual OS"
                for s in syscalls if s in FORBIDDEN_SYSCALLS]
    return bad


def print_declared(rows):
    print(f"\n### declared classes (`{CLASSES_PACKAGE}`, limit {DECLARED_LIMIT:,} felts each) and "
          "their caller fixture: sizes, entry-point builtins, syscalls (SNIP-36)\n")
    print("| class | Sierra felts | × limit | CASM felts | × limit | builtins | syscalls |")
    print("|---|--:|--:|--:|--:|---|---|")
    for name in SNIP36:
        r = rows.get(name)
        if r:
            builtins, syscalls = INTERFACES[name]
            print(f"| `{name}` | {r['sierra_felts']:,} | {r['sierra_felts'] / DECLARED_LIMIT:.2f} "
                  f"| {r['casm_felts']:,} | {r['casm_felts'] / DECLARED_LIMIT:.2f} "
                  f"| {', '.join(builtins) or '—'} "
                  f"| {', '.join(s.removesuffix('_syscall') for s in syscalls) or '—'} |")


def programs_package(work):
    """The executable fixtures as a standalone package (its own workspace) in `work`."""
    (work / "src").mkdir()
    shutil.copy(ROOT / "crates" / PACKAGE / "programs" / "lib.cairo", work / "src" / "lib.cairo")
    shutil.copy(ROOT / "crates" / PACKAGE / "src" / "scene.cairo", work / "src" / "scene.cairo")
    pins = tomllib.loads((ROOT / "Scarb.toml").read_text())["workspace"]["dependencies"]
    deps = [f'{d} = "{pins[d]}"' for d in ("fixed", "glam")]
    deps.append(f'rapier2d = {{ path = "{(ROOT / "crates" / "rapier2d").as_posix()}" }}')
    targets = "".join(f'[[target.executable]]\nname = "{n}"\n'
                      f'function = "{PROGRAMS_PACKAGE}::{n}"\n\n' for n in PROGRAMS)
    (work / "Scarb.toml").write_text(
        f'[package]\nname = "{PROGRAMS_PACKAGE}"\nversion = "0.1.0"\nedition = "2024_07"\n\n'
        "[dependencies]\n" + "\n".join(deps) + '\ncairo_execute = "2.19.4"\n\n' + targets
        + "[cairo]\nenable-gas = false\n")


def programs():
    """-> {program: {"program_felts": int}} of the executable fixtures (release profile)."""
    with tempfile.TemporaryDirectory(prefix="bytecode_programs_") as tmp:
        work = Path(tmp)
        programs_package(work)
        scarb(work, ["build"], "  (executable fixtures)", profile="release")
        target = work / "target" / "release"
        return {n: {"program_felts": len(json.loads((target / f"{n}.executable.json").read_text())
                                         ["program"]["bytecode"])} for n in PROGRAMS}


def print_programs(progs):
    full = progs["full"]["program_felts"]
    print("\n### executable programs (game world, `step_with_force_events` configurations)\n")
    print("| program | program felts | vs `full` |")
    print("|---|--:|--:|")
    for n in PROGRAMS:
        felts = progs[n]["program_felts"]
        print(f"| `{n}` | {felts:,} | {felts - full:+,} ({100 * (felts - full) / full:+.1f} %) |")


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
    deps += [f'{c} = {{ path = "{(ROOT / "crates" / c).as_posix()}" }}' for c in SINK_CRATES]
    cairo = f"\n[cairo]\ninlining-strategy = {strategy_toml(strategy)}\n" if strategy else ""
    (work / "Scarb.toml").write_text(
        f'[package]\nname = "{PACKAGE}"\nversion = "0.1.0"\nedition = "2024_07"\n\n'
        "[dependencies]\n" + "\n".join(deps) + '\nstarknet = "2.19.4"\n\n'
        "[lib]\nsierra = true\n\n[[target.starknet-contract]]\nsierra = true\ncasm = true\n"
        + cairo)


def call_graph(program):
    """-> ({function name: set of callee names}, {function name: Sierra statements}) of a
    `*.sierra.json` program: each function owns the statements from its entry point to the next
    function's (they are laid out contiguously)."""
    statements = program["statements"]
    calls = {}
    for decl in program["libfunc_declarations"]:
        if decl["long_id"]["generic_id"] == "function_call":
            calls[decl["id"]["id"]] = decl["long_id"]["generic_args"][0]["UserFunc"]["id"]
    funcs = sorted((f["entry_point"], f["id"]["id"], f["id"].get("debug_name") or "?")
                   for f in program["funcs"])
    names = {fid: name for _, fid, name in funcs}
    graph, lengths = {}, defaultdict(int)
    for i, (entry, _, name) in enumerate(funcs):
        end = funcs[i + 1][0] if i + 1 < len(funcs) else len(statements)
        lengths[name] += end - entry
        callees = set()
        for s in statements[entry:end]:
            inv = s.get("Invocation")
            if inv and inv["libfunc_id"]["id"] in calls:
                callees.add(names[calls[inv["libfunc_id"]["id"]]])
        graph[name] = callees
    return graph, lengths


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


def attribution(cls, depth, top, strategy, cuts, by="parts"):
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
    graph, lengths = call_graph(program.get("program", program))
    total = len(casm["bytecode"])
    # Sierra felts are not split per function: each part gets the class's Sierra felts in
    # proportion to its functions' Sierra statements (in the library build of the same code).
    sierra_total = len(sierra["sierra_program"])
    statements = sum(lengths.get(n, 0) for n in sizes) or 1
    groups = defaultdict(lambda: [0, 0])
    for name, n in sizes.items():
        g = groups[module_key(name, depth)]
        g[0] += n
        g[1] += 1
    parts = defaultdict(lambda: [0, 0, 0])
    for name, n in sizes.items():
        label = "other"
        for fmt, pattern in (PHASES if by == "phases" else PARTS):
            m = re.search(pattern, name)
            if m:
                label = fmt.format(*m.groups())
                break
        parts[label][0] += n
        parts[label][1] += 1
        parts[label][2] += lengths.get(name, 0)
    print(f"\n### `{cls}`: CASM felts by {'stage' if by == 'phases' else 'part'} of the step; "
          f"{total:,} CASM felts, {sierra_total:,} Sierra felts, {len(sizes):,} functions, "
          f"constants segment {consts:,}\n")
    print("| part | CASM felts | share | Sierra felts (est.) | functions |")
    print("|---|--:|--:|--:|--:|")
    for key, (n, count, st) in sorted(parts.items(), key=lambda kv: -kv[1][0]):
        print(f"| {key} | {n:,} | {100 * n / total:.1f} % | {round(sierra_total * st / statements):,} "
              f"| {count} |")
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
    ap.add_argument("--by", choices=["parts", "phases"], default="parts",
                    help="attribution: decomposition, `PARTS` (CS1) or `PHASES` (CS3, the step's stages)")
    ap.add_argument("--cut", action="append", default=[],
                    help="attribution: LABEL=REGEX on function names, replaces `CUTS` (repeatable)")
    a = ap.parse_args()

    if a.cmd == "attribution":
        cuts = dict(c.split("=", 1) for c in a.cut) if a.cut else CUTS
        attribution(a.cls, a.depth, a.top, a.strategy, cuts, a.by)
        return

    rows = fixtures()
    print_table(rows, f"{PACKAGE} fixtures and {CLASSES_PACKAGE} classes (release profile)")
    print_declared(rows)
    progs = programs()
    print_programs(progs)
    if a.cmd == "snapshot":
        write_snapshot(rows, progs)
    elif a.cmd == "check":
        (snap, snap_progs), bad = read_snapshot(), []
        for name in sorted(set(rows) | set(snap)):
            new, old = rows.get(name), snap.get(name)
            if new != old:
                bad.append(f"{name}: {old} -> {new}")
        for name in sorted(set(progs) | set(snap_progs)):
            new, old = progs.get(name), snap_progs.get(name)
            if new != old:
                bad.append(f"{PROGRAM_PREFIX}{name}: {old} -> {new}")
        over = declared_over(rows)
        if over:
            print("\n".join(over), file=sys.stderr)
            sys.exit(f"declared class over {DECLARED_LIMIT:,} felts ({len(over)}): split it further "
                     "(docs/research/class-split.md).")
        rejected = snip36_problems()
        if rejected:
            print("\n".join(rejected), file=sys.stderr)
            sys.exit(f"SNIP-36: {len(rejected)} rejected builtin or syscall use(s) in the declared "
                     "classes or their caller.")
        if bad:
            print("\n".join(bad), file=sys.stderr)
            sys.exit(f"bytecode size mismatch ({len(bad)}). Run `scripts/bytecode_size.py snapshot` "
                     "and commit gas/bytecode.size.")
        print(f"\nbytecode size snapshot OK ({len(rows)} contracts, {len(progs)} programs)",
              file=sys.stderr)


if __name__ == "__main__":
    main()
