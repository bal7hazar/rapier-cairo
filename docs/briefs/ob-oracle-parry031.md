# OB — the golden oracle on parry 0.31

Runner: `impl-opus` (vendored upstream patches, a golden policy per family), on the VPS (cargo builds of the
oracle; the Cairo golden tests run in CI). Lot id `ob-oracle-parry031`.

## 1. Goal and context

Move the golden oracle (`tools/golden`, f64 rapier + parry) from parry 0.30.2 to **parry 0.31.1**, so the next lot (CE,
compound internal edges) has an upstream reference. Approved by the project manager on 2026-10-03.

The project manager's condition: **port results stay identical; only expected values move.** No Cairo source under
`crates/*/src/**` changes in this lot.

SC2 measured this bump in a scratch copy (`docs/research/parked-families.md` §4.4; read it first):
- **Upstream edits:** `rapier2d-f64 0.35.3` vendored with its parry requirement raised to `0.31.1`; parry `0.31.1` from
  the tag (`/home/claude/git/refs/parry`, `v0.31.1`), with the same one-line f64 cuboid feature-id patch as
  `tools/golden/vendor`; 9 mechanical edits in rapier, 13 call sites in the oracle (`.intersecting`,
  `ShapeDistance::distance`, the flat 3-tuple of `project_local_point_and_get_feature`).
- **30 of the 33 vector files are byte-identical:** every scene and every contact family.
- **Three files move:**
  - `composite_queries.json`: 39 values, polyline / heightfield feature ids;
  - `compound_queries.json`: 24 values, compound point-projection feature ids;
  - `ray_casts.json`: 64 values in 9 capsule cases. These come from parry 0.31's analytic capsule ray cast: features,
    normals up to 283 raw, the hollow-inside time of impact, and zero normals inside.

Read first: `tools/golden/README.md`, `tools/golden/src/**`, `tools/golden/vendor/**`,
`docs/research/parked-families.md` §4, `docs/adr/0001-upstream-divergences.md` (entries 4 and 5, the capsule ray
cast), and the Cairo golden tests that read the three moved files (`crates/rapier2d/tests/` or
`crates/rapier_golden/**`: `composite_queries_golden.cairo`, `compound_queries_golden.cairo`, `ray_golden.cairo`,
wherever they live).

## 2. The policy for the three moved families

The port does not change, so the Cairo tests that compare the moved values exactly would fail against the new
vectors. For each moved family, choose and justify one of:

- **(a) Frozen 0.30.2 values for the moved fields.** Keep the moved fields' 0.30.2 values in a frozen copy (a separate
  vector file or field, generated once by the old oracle and never regenerated). The test compares the port against
  the frozen copy for those fields, and against 0.31 for everything else.
- **(b) A recorded divergence.** The test skips or bands the moved field, and a new ADR entry records each divergence
  with its upstream cause (the polyline-wide vs per-segment feature ids; the analytic capsule cast).

Rules for the choice:
- **No moved field may go unchecked silently.** Each one is either compared (a) or recorded (b).
- **The capsule cases:** SC2 notes they go the port's way (ADR entries 4 and 5 shrink or close). Say for each of the 9
  cases whether the port now agrees with 0.31 (the old divergence closes) or keeps a smaller one.
- **The feature-id families:** recommend which way the port should go later (follow 0.31's ids, which changes
  results, or keep its own). That is a decision for the project manager, not for this lot.

## 3. Scope: allowlist

- `tools/golden/**`: `Cargo.toml`, `Cargo.lock`, `src/**`, `vendor/**` (the vendored crates and their patches; a
  `README` line per patch).
- The vector files the oracle writes (`tools/golden/vectors/**` and their copies or generated Cairo under
  `crates/rapier_golden/**`, by the oracle's own generator only).
- The three Cairo golden test files that read the moved families: only the comparisons of the moved fields, per §2.
  The rest of each test stays as it is.
- `docs/adr/0001-upstream-divergences.md` (new entries; entries 4 and 5 annotated, not rewritten);
  `docs/research/parked-families.md` (a short "done in OB" note in §4.4); `CHANGELOG.md` (`## Unreleased`,
  **required**, saying the oracle moved and no result changed).
- This brief, `docs/briefs/ob-oracle-parry031.md` (your first commit, as given).
- Not in scope: `crates/*/src/**`, CI, scripts, every other test.

## 4. Acceptance criteria

1. The oracle builds on parry 0.31.1. With today's pins swapped back, it still reproduces the 0.30.2 vectors (show it
   once, then commit the 0.31 pins).
2. A table of every vector value that moves: file, case, field, 0.30.2 value, 0.31 value, why. It must match SC2's 39
   + 24 + 64, or say why not. Put it in the PR description (`gh pr edit`) and in `parked-families.md` §4.4.
3. No Cairo source changes (`git diff --stat origin/main -- 'crates/*/src/**'` empty). No test's expected value
   changes outside §2's moved fields.
4. "No step change" by the CI rule: no existing `gas/**/*.snap` entry and no `gas/bytecode.size` change, unless a
   golden test's own gas moves because its comparison changed; then say which test and why.
5. Every CI check green, `golden` and `ci-ok` included.
6. A pull request; **never merge**.

## 5. Machine rules (VPS)

- The oracle is Rust: `cargo build --release -j 3` in `tools/golden`. Measure its peak once under
  `prlimit --as=8589934592 -- /usr/bin/time -v`. If it does not fit, stop and report it: it runs on the Mac then. SC2
  built it on this VPS.
- Cairo test crates do not fit the VPS cap. Do not build them here: the CI `golden` and test jobs are the check.
  `scarb fmt --check` and `scarb build -p <crate>` are fine.
- Every Cairo build goes through the shims (the heavy lock); `RAYON_NUM_THREADS=1`.

## 6. Programme rules

- **CI logs:** wait for the run to complete, then `gh run view <run-id> --log-failed` alone in its call. Never
  `gh api …/logs`.
- **Git:**
  - your own git commands run normally, as separate commands;
  - to pick up `main`, `git merge origin/main` alone in its call; never another rebase, never a force push;
  - remove an untracked file only with `git clean -f -- <exact path>`; never `-d`, `-x` or `-X`.
- **Text:** prefer words over links to external issues. `gh pr edit` works.
- **Processes:** signal only processes you started, by the pid you recorded.
- **Refusals:** if a command is refused, put its exact text and the time in your report and stop that part.
- `scripts/prepush.sh` before each push. One push when done, then one per review's fixes. If the hook may run long,
  push with `git -c core.sshCommand='ssh -o ServerAliveInterval=30 -o ServerAliveCountMax=40' push ...`.

## 7. Report

Your thread report as your rules say. Lead with the policy chosen per family, then the moved-values table, the capsule
verdicts, and the recommendation on feature ids.

## 8. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
