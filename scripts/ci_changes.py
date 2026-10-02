#!/usr/bin/env python3
"""Which jobs of `.github/workflows/ci.yml` concern a pull request (lot CI1, docs/briefs/ci1-path-filtered-ci.md).

CI tests run only when files related to them changed: documents alone run no test job. The `changes` job feeds the
PR's changed paths to this script; every other job runs `if: needs.changes.outputs.<job> == 'true'`, and the final
job `ci-ok` calls `verdict` to turn the results into one pass / fail.

  ci_changes.py outputs [--all]     changed paths on stdin (one per line) -> `name=value` lines for $GITHUB_OUTPUT;
                                    `--all` (push to main, workflow_dispatch) turns every job on
  ci_changes.py gas-dirs            the `test` matrix JSON on stdin -> the `gas/` directories of its groups
  ci_changes.py verdict             `toJSON(needs)` in $NEEDS -> one summary line per job on stdout; exit 1 on failure
  ci_changes.py --self-test         the cases of the acceptance criteria

Only the Python standard library is used.
"""

from __future__ import annotations

import json
import os
import re
import sys

# Everything runs when one of these changes: the toolchain, the dependency graph, this workflow and this script.
EVERYTHING = [
    r"\.tool-versions",
    r"Scarb\.toml",
    r"Scarb\.lock",
    r"crates/[^/]+/Scarb\.toml",
    r"\.github/workflows/ci\.yml",
    r"scripts/ci_changes\.py",
]

# Test groups of the `test` matrix: packages (as in ci.yml) and the crates whose files concern the group, that is the
# packages and everything they depend on, dev-dependencies included (read from crates/*/Scarb.toml).
TEST_GROUPS = {
    "core-math-golden": {
        "packages": "rapier_testing rapier_math rapier_core rapier_golden",
        "crates": ["rapier_testing", "rapier_math", "rapier_core", "rapier_golden"],
    },
    "geometry2d": {
        "packages": "rapier_geometry2d",
        "crates": ["rapier_geometry2d", "rapier_math", "rapier_core", "rapier_testing", "rapier_golden"],
    },
    "dynamics2d": {
        "packages": "rapier_dynamics2d",
        "crates": ["rapier_dynamics2d", "rapier_geometry2d", "rapier_math", "rapier_core", "rapier_testing",
                   "rapier_golden"],
    },
    "rapier2d": {
        "packages": "rapier2d",
        "crates": ["rapier2d", "rapier_dynamics2d", "rapier_geometry2d", "rapier_math", "rapier_core",
                   "rapier_testing", "rapier_golden"],
    },
}

# The `sink` matrix: one job per crate; the test dev-dependencies are `rapier_testing` only (no `rapier_golden`).
SINK_PACKAGES = {
    "rapier2d_classes": ["rapier2d_classes", "rapier2d", "rapier_dynamics2d", "rapier_geometry2d", "rapier_math",
                         "rapier_core", "rapier_testing"],
    "rapier_sink": ["rapier_sink", "rapier2d_classes", "rapier2d", "rapier_dynamics2d", "rapier_geometry2d",
                    "rapier_math", "rapier_core", "rapier_testing"],
}
# The matrix order of ci.yml before CI1.
SINK_ORDER = ["rapier_sink", "rapier2d_classes"]

ENGINE = ["rapier_math", "rapier_core", "rapier_geometry2d", "rapier_dynamics2d", "rapier2d", "rapier2d_classes",
          "rapier_sink"]
PUBLISHED = ["rapier_math", "rapier_core", "rapier_geometry2d", "rapier_dynamics2d", "rapier2d", "rapier2d_classes"]

JOBS = ["fmt", "lint", "build", "test", "gas", "bytecode", "sink", "consumer_cost", "api_parity", "golden"]


def crate_files(crates: list[str]) -> list[str]:
    return [rf"crates/{c}/.*" for c in crates]


def matches(path: str, patterns: list[str]) -> bool:
    return any(re.fullmatch(p, path) for p in patterns)


def gas_files(crate: str) -> list[str]:
    """The snapshots of a crate's tests: `gas/<crate>/` and `gas/<crate>_integrationtest/`."""
    return [rf"gas/{crate}(_integrationtest)?/.*"]


def outputs(paths: list[str], everything: bool = False) -> dict[str, str]:
    def hit(patterns: list[str]) -> bool:
        return everything or matches_any(paths, EVERYTHING + patterns)

    def matches_any(ps: list[str], patterns: list[str]) -> bool:
        return any(matches(p, patterns) for p in ps)

    any_crate = [r"crates/.*"]
    test_matrix = [
        {"group": g, "packages": spec["packages"]}
        for g, spec in TEST_GROUPS.items()
        # A group also runs when the snapshots of its own packages, or the tool that compares them, changed: the
        # `gas` job needs the logs of the groups it checks.
        if hit(crate_files(spec["crates"]) + [f for c in spec["packages"].split() for f in gas_files(c)]
               + [r"scripts/gas\.py"])
    ]
    sink_matrix = [{"package": p} for p in SINK_ORDER if hit(crate_files(SINK_PACKAGES[p]))]
    out = {
        "fmt": hit(any_crate),
        # The lint job also runs `scripts/consumer_cost.py --lines-only` on `consumer_cost.toml`.
        "lint": hit(any_crate + [r"scripts/consumer_cost\.py", r"consumer_cost\.toml"]),
        "build": hit(any_crate),
        "test": bool(test_matrix),
        # Only the groups that ran are compared: the snapshots of a group that did not run are not looked at.
        "gas": bool(test_matrix),
        "bytecode": hit(crate_files(ENGINE) + [r"gas/bytecode\.size", r"scripts/bytecode_size\.py"]),
        "sink": bool(sink_matrix),
        "consumer_cost": hit(crate_files(PUBLISHED) + [
            r"scripts/consumer_cost\.py", r"scripts/packages_table\.py", r"docs/PACKAGES\.md",
            r"consumer_cost\.toml",
        ]),
        # `docs/API_PARITY.md` is generated and compared: a checked Markdown file, not prose.
        "api_parity": hit([r"crates/[^/]+/src/.*", r"scripts/api_parity\.py", r"docs/API_PARITY\.md"]),
        "golden": hit([r"tools/golden/.*", r"crates/rapier_golden/.*"]),
    }
    res = {k: str(v).lower() for k, v in out.items()}
    res["test_matrix"] = json.dumps({"include": test_matrix}, separators=(",", ":"))
    res["sink_matrix"] = json.dumps({"include": sink_matrix}, separators=(",", ":"))
    return res


def gas_dirs(matrix: dict) -> list[str]:
    """The `gas/` directories of the groups of a `test` matrix (`<crate>` and `<crate>_integrationtest`)."""
    dirs = []
    for entry in matrix["include"]:
        for crate in entry["packages"].split():
            dirs += [crate, f"{crate}_integrationtest"]
    return dirs


def job_id(key: str) -> str:
    return key.replace("_", "-") if key in ("consumer_cost", "api_parity") else key


def verdict(needs: dict) -> tuple[bool, list[str]]:
    """`needs` is `toJSON(needs)`: `{job: {"result": ..., "outputs": {...}}}`. Returns (ok, one line per job)."""
    lines, ok = [], True
    changes = needs.get("changes", {})
    if changes.get("result") != "success":
        return False, [f"changes: {changes.get('result', 'missing')}: the changed paths were not computed -> FAIL"]
    wanted = changes.get("outputs", {})
    for key in JOBS:
        name = job_id(key)
        result = needs.get(name, {}).get("result", "missing")
        expected = wanted.get(key) == "true"
        if result == "success":
            lines.append(f"{name}: ran and passed")
        elif result == "skipped" and not expected:
            lines.append(f"{name}: skipped by rule (no file of the job changed)")
        elif result == "skipped":
            lines.append(f"{name}: skipped by error (it should have run) -> FAIL")
            ok = False
        else:
            lines.append(f"{name}: {result} -> FAIL")
            ok = False
    return ok, lines


def self_test() -> None:
    ALL = set(JOBS)

    def on(paths: list[str]) -> set[str]:
        o = outputs(paths)
        return {k for k in JOBS if o[k] == "true"}

    def groups(paths: list[str]) -> list[str]:
        return [e["group"] for e in json.loads(outputs(paths)["test_matrix"])["include"]]

    def sinks(paths: list[str]) -> list[str]:
        return [e["package"] for e in json.loads(outputs(paths)["sink_matrix"])["include"]]

    # (a) prose only, (a2) a checked Markdown file
    assert on(["docs/PLAN.md"]) == set()
    assert on(["docs/PLAN.md", "docs/briefs/x.md", "README.md", "CHANGELOG.md", "AGENTS.md", "docs/BUDGETS.md"]) == set()
    assert on(["docs/API_PARITY.md"]) == {"api_parity"}
    assert on(["docs/PACKAGES.md"]) == {"consumer_cost"}
    # (b) a source file of the facade
    p = ["crates/rapier2d/src/world.cairo"]
    assert on(p) == {"fmt", "lint", "build", "test", "gas", "bytecode", "sink", "consumer_cost", "api_parity"}
    assert groups(p) == ["rapier2d"] and sinks(p) == ["rapier_sink", "rapier2d_classes"]
    # (c) the bottom of the graph
    p = ["crates/rapier_math/src/lib.cairo"]
    assert on(p) == {"fmt", "lint", "build", "test", "gas", "bytecode", "sink", "consumer_cost", "api_parity"}
    assert groups(p) == list(TEST_GROUPS)
    # (d) the gas tool
    assert on(["scripts/gas.py"]) == {"test", "gas"} and groups(["scripts/gas.py"]) == list(TEST_GROUPS)
    # (e) the lock file: everything
    assert on(["Scarb.lock"]) == ALL and on([".tool-versions"]) == ALL and on(["crates/rapier_core/Scarb.toml"]) == ALL
    assert on([".github/workflows/ci.yml"]) == ALL and on(["scripts/ci_changes.py"]) == ALL
    # one group's own snapshot runs that group only, with its gas check
    assert groups(["gas/rapier_geometry2d/aabb.snap"]) == ["geometry2d"]
    assert on(["gas/rapier2d_integrationtest/world_step.snap"]) == {"test", "gas"}
    # a leaf crate runs its own group only; the crates under it are not touched
    assert groups(["crates/rapier_dynamics2d/src/solver.cairo"]) == ["dynamics2d", "rapier2d"]
    assert groups(["crates/rapier_geometry2d/tests/a.cairo"]) == ["geometry2d", "dynamics2d", "rapier2d"]
    assert sinks(["crates/rapier_sink/tests/a.cairo"]) == ["rapier_sink"]
    assert groups(["crates/rapier_sink/tests/a.cairo"]) == []
    p = ["crates/rapier2d_classes/src/lib.cairo"]
    assert groups(p) == [] and sinks(p) == ["rapier_sink", "rapier2d_classes"]
    # golden vectors and scripts of one job
    assert on(["tools/golden/vectors/aabb.json"]) == {"golden"}
    assert "golden" in on(["crates/rapier_golden/src/lib.cairo"])
    assert on(["scripts/bytecode_size.py"]) == {"bytecode"} and on(["gas/bytecode.size"]) == {"bytecode"}
    assert on(["scripts/api_parity.py"]) == {"api_parity"}
    assert on(["scripts/consumer_cost.py"]) == {"lint", "consumer_cost"}
    assert on(["examples/ball_drop/src/lib.cairo"]) == set()
    # push to main and workflow_dispatch: everything, full matrices
    o = outputs([], everything=True)
    assert all(o[k] == "true" for k in JOBS)
    assert len(json.loads(o["test_matrix"])["include"]) == 4 and len(json.loads(o["sink_matrix"])["include"]) == 2
    assert gas_dirs(json.loads(o["test_matrix"]))[:4] == [
        "rapier_testing", "rapier_testing_integrationtest", "rapier_math", "rapier_math_integrationtest"]
    # ci-ok
    def need(result: dict[str, str], wanted: set[str], changes: str = "success") -> dict:
        n = {"changes": {"result": changes, "outputs": {k: str(k in wanted).lower() for k in JOBS}}}
        for k in JOBS:
            n[job_id(k)] = {"result": result.get(k, "success" if k in wanted else "skipped")}
        return n

    assert verdict(need({}, ALL))[0]                                    # all ran and passed
    assert verdict(need({}, {"api_parity"}))[0]                         # some skipped by rule
    assert verdict(need({}, set()))[0]                                  # documents only
    assert not verdict(need({"test": "failure"}, ALL))[0]               # one that ran failed
    assert not verdict(need({"build": "cancelled"}, ALL))[0]            # cancelled
    assert not verdict(need({"gas": "skipped"}, ALL))[0]                # skipped by error
    assert not verdict(need({"bytecode": "skipped"}, {"bytecode"}))[0]
    assert not verdict(need({}, ALL, changes="failure"))[0]             # `changes` itself failed
    assert not verdict({})[0]
    print("ci_changes self-test: ok")


def main() -> None:
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "--self-test":
        self_test()
    elif cmd == "outputs":
        paths = [line.strip() for line in sys.stdin if line.strip()]
        for k, v in outputs(paths, everything="--all" in sys.argv[2:]).items():
            print(f"{k}={v}")
    elif cmd == "gas-dirs":
        print("\n".join(gas_dirs(json.load(sys.stdin))))
    elif cmd == "verdict":
        ok, lines = verdict(json.loads(os.environ["NEEDS"]))
        print("\n".join(lines))
        sys.exit(0 if ok else 1)
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
