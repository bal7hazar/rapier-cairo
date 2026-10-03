# FU0 — study: fewer fixed-point rescales on the step path

Runner: `impl-opus` (numerics, measurement). Lot id `fu0-fused-rescales`. **Research only: no library change is
committed.**

## 1. Goal and context

HP (`docs/research/impact-tick.md` §9) measured that `fixed` + `glam_core` take 29.6 % of rapier's Cairo steps on its
probes, and `narrow32` (the Q64.64 → Q32.32 rescale) alone 16.0 % (23.0 % of the pile10 reference shot).

The glam track's FS lot found that `narrow32` is at the floor of the Scarb 2.20 libfuncs: a `Fixed` mul is about 14
steps and cannot go lower, and the `glam_core` ops are thin wrappers over it. So the remaining lever is in rapier:
**fewer rescales per formula**. Accumulate in `fixed`'s wide form (`wide::mul_add`, `wide::dot2_add`, `wide::mul_sub`,
… then one `narrow`) where today each product is narrowed on its own. Design decision D2 of `docs/PLAN.md` ("fused
kernels are the unit of design") already says this; FU0 finds where the step path still breaks it.

The study proposes; the project manager decides.

Read first: `AGENTS.md`, `docs/PLAN.md` D1–D2, `docs/research/impact-tick.md` (§8–§9, HP and how it was measured),
`docs/research/04-numeric-benchmark.md` (fused kernels), `docs/BUDGETS.md` (the BT1–BT4 sections: what was fused
already), the `fixed` crate's `wide` module (in the scarb cache or the registry archive of `fixed` 0.4.0: read it, do
not change it), `docs/adr/0001-upstream-divergences.md`.

## 2. What to produce

One document, `docs/research/fused-rescales.md`, and this brief, `docs/briefs/fu0-fused-rescales.md` (your first
commit, as given), in a pull request that changes nothing else.

1. **The hot formulas of the step path that narrow more than once**, ranked by the steps of their rescales on HP's
   probe families. Start from HP's operations and their call sites: the solver sweeps and constraint generation, the
   contact generators, the integration, the narrow phase, the transforms. For each, give:
   - the function and file;
   - the formula as computed today, with each narrow marked;
   - the number of narrows per call, and the calls per tick on the reference shot (measured from HP's counts where you
     can, estimated otherwise, called so).
2. **For each, the fused form:** how many rescales it would save, and the steps saved per call and per shot (owner's
   and reference, in process and slim), marked as estimates. Prototype the top 2 or 3 in an uncommitted worktree, and
   give their exact steps on the probes with the command.
3. **Bit-identity, for each formula:** does the fused form give the same bits?
   - Usually not: one rounding instead of several changes the last bit.
   - Say which formulas stay bit-identical (for example when every intermediate is exact), and which do not.
   - For the latter, say which goldens and probes would move. Under the domain rule a last-bit change is a MINOR
     bump of the published crates and a golden regeneration from the upstream oracle; say what that means here (the
     oracle is f64 rapier).
   - Measure on a prototype the size of the change: the maximum ulp difference on the probes, against today's values
     and against the f64 oracle. Is the fused result closer to the oracle?
4. **A recommendation:**
   - which formulas to fuse, in which lot, its size and profile;
   - whether to bundle the result-changing ones in one MINOR;
   - what a game would re-pin (classes, `WorldState` and replay digests).

## 3. Rules

- **Nothing committed but the two documents.** No change to `crates/**`, scripts or CI.
- **Builds:** `RAYON_NUM_THREADS=1`; crate-scoped runs; whole-shot suites with `--max-threads 2`, one at a time; never
  `snforge test --workspace`; `--tracked-resource cairo-steps` for steps.
- **CI logs:** wait for the run to complete, then `gh run view <run-id> --log-failed` alone in its call. Never
  `gh api …/logs`.
- **Git:** your own git commands run normally, as separate commands; to pick up `main` after a push,
  `git merge origin/main` alone in its call.
- **Processes:** signal only processes you started, by the pid you recorded.
- **Text:** prefer words over links to external issues in PR text and commit messages.
- **Refusals:** if a command is refused by the permission system, put its exact text and the time in your report and
  stop that part.
- Push once when done, then once per review's fixes. `scripts/prepush.sh` before each push. **Never merge.**

## 4. Report

Your thread report as your rules say. Lead with the top formulas, their steps saved per shot (estimates) and their
bit-identity verdict, then the recommendation.

## 5. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
