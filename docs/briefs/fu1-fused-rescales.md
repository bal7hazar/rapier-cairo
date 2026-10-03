# FU1 — fewer fixed-point rescales on the step path (one numeric lot)

Runner: `impl-opus` (numerics, step path). Lot id `fu1-fused-rescales`. Starts on the VPS. The whole-shot measurement
runs on the Mac when a slot frees.

## 1. Goal and context

Apply the fused forms that FU0 measured (`docs/research/fused-rescales.md`, read it all first). The project manager
approved them on 2026-10-03 as **one numeric lot**, so a game re-pins once:

- **P1:** the row solve `impulse − r·(jv + rhs)` (`solve_normal` / `solve_tangent`). Rescales 2 → 1.
- **P2:** the separation refresh (two `transform` + `separation`, `split.cairo`). Rescales 6 → 2, **together with
  generation's `round_trip`**, so the SF1 defect cannot return (`docs/PLAN.md` v2.72–2.74, #191).
- **P3:** the effective mass in `generation::coefficients`. Rescales 3 → 1.
- **C5** (the manifold update), **C6** (the half-space vertex), **C7** (`Rot2::integrate`).
- **B1:** the solver-contact anchors folded into the wide sum. This one is bit-identical.

**Two pull requests:**
1. **First, B1 alone:** bit-identical, a small PR. The "no step change" condition doesn't apply to it. Results must be
   bit-identical, and every existing gas entry it touches may only go down.
2. **Then the rest** (P1, P2, P3, C5, C6, C7), as the numeric PR, on a branch from `origin/main` after B1 merges (or
   from B1's branch, if B1 is still under review; say which).

Read first: `docs/research/fused-rescales.md`, `docs/research/impact-tick.md` §9 (HP), `docs/PLAN.md` D1–D2 and SF1,
`docs/BUDGETS.md` (BT1–BT4, EL1), `docs/adr/0001-upstream-divergences.md`, `AGENTS.md`, `CHANGELOG.md`, and `fixed`'s
`wide` module (read it, do not change it).

## 2. Conditions (the project manager's)

- **Each changed formula stays within 1 ulp of the f64 oracle.** Measure it on the formula's own probe, as FU0 did:
  thousands of cases, simulated bit-exactly and checked against Cairo on a sample. List it in the PR. The oracle
  (`tools/golden`) is not changed by this lot.
- **Every golden scene passes within its existing band. A band is never widened.** If one fails, stop that formula,
  push what you have, and report it under Escalations: the orchestrator takes it to the project manager.
- **Digests and class hashes are re-pinned from CI's artefacts** (the CI2 gate), with CI's root path and run id beside
  them. Every changed pinned digest gets one line in the PR: test, old value, new value, and the formula that moves it.
- **Steps before and after on every probe:**
  - crate-scoped on the VPS meanwhile, in a scratch probe package outside the repository, under the cap (as FU0 did);
  - the whole-shot suites (`rapier2d_classes`, the pile10 owner's and reference shots, in process and slim) on the Mac
    when a slot frees. Write those as "to run on the Mac", with their commands, if no slot frees during the lot.
- **`CHANGELOG.md` `## Unreleased`, "Changed":** a result change (MINOR, from 0.1.0 on), one line per formula, with its
  ulp bound and its steps. It ships in `0.1.0-alpha.10`, on the project manager's go.
- **Class margins:** every declared class stays under 73,728. Print both slim margins in the PR (CI's `bytecode`
  log). Flag either one if it falls under 2,000.

## 3. Scope: allowlist

- **Sources:** `crates/rapier_dynamics2d/src/**` (the solver, generation, the manifold update), and
  `crates/rapier_geometry2d/src/**` and `crates/rapier_math/src/**` for C6 and C7 only. Inline tests may be added.
- **Pinned digests and expected values that a result change must move:** game-path, level, state and class digests,
  `*_bit_identical` pins. Only those, each listed in the PR. **Golden vector files and their bands are never edited.**
- **Generated files:**
  - `gas/**/*.snap`, by `scripts/gas.py` (from CI's logs if local builds don't fit);
  - `gas/bytecode.size`, the class-hash constants and `docs/PACKAGES.md`, from CI's artefacts;
  - `docs/API_PARITY.md`, by its script.
- **Docs:** `docs/BUDGETS.md` (one section "FU1"), `docs/research/fused-rescales.md` (a "done" section with the measured
  results), `docs/adr/0001-upstream-divergences.md` (new entries), `CHANGELOG.md`.
- **This brief:** `docs/briefs/fu1-fused-rescales.md`, your first commit, as given.
- **Not in scope:** `tools/golden/**`, the golden vector files and the golden query tests (lot OB is changing those,
  in parallel), CI, scripts, public signatures.

## 4. Machines

- **VPS:**
  - every Cairo build through the shims (the heavy lock), with `RAYON_NUM_THREADS=1`;
  - the workspace's test crates do not fit the 8 GB cap: never build them here. CI runs them;
  - your scratch probe package runs under `prlimit --as=8589934592 -- /usr/bin/time -v snforge test ...`, one at a
    time;
  - if anything hits the cap, never raise it.
- **Mac:** whole-shot suites with `--max-threads 2`, uncapped there, one at a time. The orchestrator tells you when a
  slot is yours.

## 5. Programme rules

- **CI logs:** wait for the run to complete, then `gh run view <run-id> --log-failed` alone in its call. Never
  `gh api …/logs`.
- **Git:**
  - your own git commands run normally, as separate commands;
  - to pick up `main`, `git merge origin/main` alone in its call; never another rebase, never a force push;
  - remove an untracked file only with `git clean -f -- <exact path>`; never `-d`, `-x` or `-X`.
- **Text:** prefer words over links to external issues. `gh pr edit` works for your PR's text.
- **Processes:** signal only processes you started, by the pid you recorded.
- **Refusals:** if a command is refused, put its exact text and the time in your report and stop that part.
- **Pushes:** `scripts/prepush.sh` before each push. Push rarely: once per PR when done, once with the pins from CI,
  then one push per review's fixes. If the hook may run long, push with
  `git -c core.sshCommand='ssh -o ServerAliveInterval=30 -o ServerAliveCountMax=40' push ...`.

## 6. Acceptance criteria

1. B1's PR: bit-identical (every result test unchanged), its gas entries down or equal. CI green.
2. The numeric PR, with:
   - the ulp table per formula;
   - every golden scene within its band;
   - the moved digests listed;
   - the pins from CI;
   - the steps before and after (crate-scoped now, whole-shot from the Mac);
   - both slim margins;
   - the CHANGELOG lines.

   CI green, `ci-ok` included.
3. **Never merge.**

## 7. Report

Your thread report as your rules say. Lead with the ulp table and the band verdict, then the steps per shot before and
after, the margins, and what a game must re-pin.

## 8. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
