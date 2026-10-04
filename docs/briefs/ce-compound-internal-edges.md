# CE — compound internal edges (opt-in), and the parry 0.31.1 query answers

Runner: `impl-opus` (a new contact strategy, a fixed-point reading of the weld tolerance, and a design in which SAT
replaces GJK). Starts on the VPS with crate-scoped builds. Lot id `ce-compound-internal-edges`.

## 1. Goal and context

Two parts, approved by the project manager on 2026-10-03, in one lot so that a consumer re-pins once:

1. **Compound internal edges, opt-in:** option 4A of `docs/research/parked-families.md` §4.2. It covers the 8 parity
   items:
   - `CompoundFlags` (`FIX_INTERNAL_EDGES`);
   - `Compound::{with_flags, set_flags, flags, DEFAULT_WELD_TOLERANCE, part_normal_constraints}`;
   - `CompoundPseudoNormals` / `CompoundEdgeCone`;
   - a `ConstrainedCompositeManifolds` strategy, selected through `World::step_with::<C>`.

   `DefaultStepConfig` keeps `CompositeManifolds`, so `World::step` never compiles the new path.
2. **Follow parry 0.31.1's query answers:** the moves that lot OB (#269) froze become the port's own answers:
   - polyline, heightfield and compound **feature ids**: the segment's or part's own feature, with the segment or part
     in `subshape`. This drops `segment_feature_to_polyline_feature`'s polyline-wide ids and compound `Unknown`;
   - the **three capsule ray-cast answers**: feature `Face(0)`, the solid-inside zero normal, and
     `capsule/zero_dir_inside` hitting at `t = 0`. Hollow-inside times of impact and normals already follow 0.31.1
     (OB closed ADR entries 4 and 5).

   This is a **MINOR result change**: query answers move, and contacts and scenes do not.

Read first:
- `docs/research/parked-families.md` §4 (all of it: the upstream, 4A, the costs, the two numeric points, and OB's done
  note in §4.4);
- `AGENTS.md`;
- `docs/adr/0001-upstream-divergences.md` (entries 4, 5, 17, 35, 36, 46, 47 and 48);
- the upstream sources: parry 0.31.1 in `/home/claude/git/refs/parry`, tag `v0.31.1`, files `compound.rs`,
  `compound_pseudo_normals.rs`, `contact_manifolds_pfm_pfm.rs`, `contact_manifolds_convex_ball.rs` and the capsule
  ray cast;
- `tools/golden/README.md` and `rapier_golden::generated::frozen_parry030`;
- `docs/BUDGETS.md`, `CHANGELOG.md` and `docs/API_PARITY.md`.

## 2. Conditions

These are the project manager's, plus the track's.

- **No contact or scene value moves**, under the CI rule:
  - every scene and contact golden passes unchanged;
  - every `*_bit_identical` test, digest and class hash passes unchanged;
  - a moved contact or scene value stops that part: push what you have, and report it under Escalations.
- **The moved query answers:**
  - they now compare against the 0.31.1 vectors, with no frozen copy;
  - remove the frozen comparisons and `frozen_parry030`, with its generator and JSON, once nothing reads them;
  - ADR entries 47 and 48 are annotated as closed, not rewritten.

  List every query-test expected value that moves, with its file, case, field, old value and new value. They must
  match OB's table: the parked-families §4.4 table and #269's description.
- **The game's exposure:** the PR lists whether the game's code calls any query whose answer moves.
  - Search the slingfall repository at `origin/main` (`/home/claude/projects/slingfall`) read-only for:
    - point projections and ray casts on polylines, heightfields and compounds;
    - feature reads (`feature`, `FeatureId`, `subshape`);
    - capsule ray casts.
  - Use `git -C /home/claude/projects/slingfall grep` under a sanitised environment (`env -u GIT_DIR -u GIT_WORK_TREE
    -u GIT_INDEX_FILE -u GIT_COMMON_DIR -u GIT_PREFIX`), or the Grep tool.
  - Give each call site with file and line, or say "none".
  - Never write in that repository.
- **Steps:**
  - existing worlds' step probes are unchanged (`snforge test -p rapier2d steps_`, crate-scoped, before and after);
  - the compound `Serde` must keep a flag-free compound's `WorldState` felts and steps. If the flag unpacking costs
    steps on existing probes, add the flag-free fast path (§4.3);
  - query tests' gas may move with the feature-id change. List each moved `.snap` entry and say why;
  - measure the opt-in path's steps per constrained part manifold on a new probe.
- **Classes:** no declared class selects the new strategy, so none should grow. Show `gas/bytecode.size` unchanged
  (CI's `bytecode` log), or explain each change with both slim margins.
- **Numerics:** each gets a new ADR entry:
  - `DEFAULT_WELD_TOLERANCE` in raw Q32.32 units: choose the reading, and justify it against f64's 4 ULPs relative;
  - the SAT counterpart of the GJK "normal changed, drop the point" branch, and the retain rule using the SAT
    penetration as `dist`.
- **New goldens for flagged compounds**, from the oracle (now on 0.31.1), through its own generator. Cover:
  - a compound of at least two adjacent boxes with a ball or box sliding across the internal edge, flagged and
    unflagged;
  - one polygon pair.

  Bands follow the existing contact goldens' tolerances. A band is never widened.
- **Package size:** the `rapier_geometry2d` line count against its gate, before and after.

## 3. Scope: allowlist

- **Sources:**
  - `crates/rapier_geometry2d/src/**`: the compound, the cones, the constrained manifolds, the feature ids and the
    capsule ray cast;
  - `crates/rapier_dynamics2d/src/**` and `crates/rapier2d/src/**`: only the strategy, the step config and the
    prelude.
  - Inline tests may be added.
- **Golden:**
  - `tools/golden/src/**` and its generated output under `crates/rapier_golden/**` (generator only);
  - new vector files, and removing `tools/golden/vectors/frozen/**`;
  - the three golden query tests (`composite_queries_golden`, `compound_queries_golden`, `ray_golden`), only to drop
    the frozen comparisons;
  - new golden test files for flagged compounds.

  The existing scene and contact vector files are never edited.
- **Tests:** query tests' expected feature ids and capsule answers that this lot moves, each listed. No other expected
  value.
- **Generated:**
  - `gas/**/*.snap`, by `scripts/gas.py`, from CI's log if local builds don't fit;
  - `gas/bytecode.size`, the hash constants and `docs/PACKAGES.md`, from CI's artefacts only, with CI's root path and
    run id;
  - `docs/API_PARITY.md`, by its script.
- **Docs:**
  - `docs/adr/0001-upstream-divergences.md`:
    - new entries;
    - entries 47 and 48 annotated as closed;
    - the Context paragraph and entry 36 corrected where they still say the golden pin is parry 0.30.2 (it is 0.31.1
      since #269);
  - `docs/research/parked-families.md`: a short "done in CE" note in §4;
  - `docs/BUDGETS.md`: one "CE" section;
  - `CHANGELOG.md`, `## Unreleased`, **required**:
    - "Added" for the opt-in strategy;
    - "Changed" for the query answers, as a MINOR result change, with one line per family.
- **This brief:** `docs/briefs/ce-compound-internal-edges.md`, your first commit, as given.
- **Not in scope:**
  - CI and scripts;
  - FU1's files (`crates/rapier_dynamics2d/src/` solver, generation and the manifold update; `rapier_math`'s `Rot2`).
    Lot FU1 (#270) is changing those. If you need one, stop and report it;
  - public signatures of existing items. New items are fine. A changed signature is an escalation.

## 4. Machines (VPS)

- Every Cairo build goes through the `scarb` / `snforge` shims (the heavy lock), with `RAYON_NUM_THREADS=1`.
- Crate-scoped runs only, one at a time. The workspace's test crates do not fit the 8 GB cap, so never build them
  here: CI runs them.
- Scratch probe packages run under `prlimit --as=8589934592 -- /usr/bin/time -v snforge test ...`. If anything hits
  the cap, report it; never raise the cap and never run uncapped.
- The oracle builds with `cargo build --release -j 3` in `tools/golden` (OB built it here).

## 5. Programme rules

- **CI logs:** wait for the run to complete, then `gh run view <run-id> --log-failed` alone in its call. Never
  `gh api …/logs`.
- **Git:**
  - your own git commands run normally, as separate commands;
  - to pick up `main`, `git merge origin/main` alone in its call; never another rebase, never a force push;
  - remove an untracked file only with `git clean -f -- <exact path>`; never `-d`, `-x` or `-X`;
  - a git command on another repository runs only in a sanitised environment (§2).
- **Text:** prefer words over links to external issues. `gh pr edit` works.
- **Processes:** signal only processes you started, by the pid you recorded.
- **Refusals:** if a command is refused, put its exact text and the time in your report and stop that part.
- **Pushes:**
  - `scripts/prepush.sh` before each push;
  - push rarely: once when done (open the PR), once with the pins and gas from that run's CI, then one push per
    review's fixes;
  - if the hook may run long, push with
    `git -c core.sshCommand='ssh -o ServerAliveInterval=30 -o ServerAliveCountMax=40' push ...`.

## 6. Acceptance criteria

1. The 8 parity items are `ported` in `docs/API_PARITY.md`. `Compound::bvh` stays missing or closes with
   `CompositeShape::bvh`'s reason.
2. The new flagged-compound goldens pass within their bands, and the unflagged twins match today's behaviour.
3. The moved query values are listed, match OB's table, and the frozen copy is gone.
4. No contact or scene value moves. The step probes are unchanged before and after. Every moved `.snap` entry is
   listed.
5. The game's exposure list from slingfall at `origin/main`.
6. The ADR entries, the BUDGETS section, the CHANGELOG lines and the line count.
7. Every CI check green, `ci-ok` included.
8. A pull request; **never merge**.

## 7. Report

Your thread report as your rules say. Lead with:
- the game's exposure;
- the moved query values against OB's table;
- the "no contact or scene value moves" verdict;
- then the opt-in path's steps, the numeric choices and the line count.

## 8. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
