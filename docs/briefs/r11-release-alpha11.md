# R11 — release 0.1.0-alpha.11, step 1: the version bump on main

Runner: `impl-sonnet` (a documented, mechanical release step), on the VPS. Lot id `r11-release-alpha11`.

## 1. Goal and context

Prepare `0.1.0-alpha.11` on `main`, as `0.1.0-alpha.10` was prepared by #272 (`4d8e14a`). Read that commit first: it
touched `Scarb.toml`, `Scarb.lock`, `CHANGELOG.md` and `docs/PLAN.md`.

The owner resumed the programme on 2026-10-05 only to finish the planned versions. `0.1.0-alpha.11` carries:
- **DEP5** (#279): `fixed` 0.5.0 and `glam_core` 0.5.0. No result changes, and no class hash moved.
- **PX9** (#275): API parity at 100 % in scope, with joint `user_data` beside the joint set. No result changes, and no
  class hash moved.

This step does **not** publish anything and does not touch the release branch. The orchestrator does the release
commit off `main` and the staged publication afterwards.

## 2. What to change

1. **The version.** Bump every published crate's version from `0.1.0-alpha.10` to `0.1.0-alpha.11`: the workspace
  `version` and the inter-crate requirements, exactly where #272 changed them. Update `Scarb.lock` through Scarb (the
  shim, under the cap), never by hand.
2. **`CHANGELOG.md`.** Turn `## Unreleased` into `## 0.1.0-alpha.11 — 2026-10-05`, in the file's format (an em dash, as
   the earlier headings use), and leave an empty `## Unreleased` above it.
   - **The Results line:** it names PX9 only today. Make it cover both:
     "**Results:** unchanged (DEP5 and PX9: no step result, no `WorldState` format and no class hash moves). A consumer
     that uses `Fixed` or `Vec2` itself must move to `fixed` 0.5 and `glam_core` 0.5 with this version."
   - Do not reword the other entries.
3. **`docs/PLAN.md`:** the version line, as #272 did ("alpha.11 prepared").
4. **The install snippets** that still name `0.1.0-alpha.8`, to `0.1.0-alpha.11`:
   - `crates/rapier2d/README.md` (`rapier2d = "0.1.0-alpha.8"`);
   - `crates/rapier2d_classes/README.md` (`rapier2d` and `rapier2d_classes`);
   - the example comment in `examples/ball_drop/Scarb.toml` (`` e.g. `rapier2d = "0.1.0-alpha.8"` ``).

   Change nothing else in those files.

## 3. Scope: allowlist

- the workspace `Scarb.toml` (the version only) and `Scarb.lock`;
- `CHANGELOG.md`;
- `docs/PLAN.md`;
- `crates/rapier2d/README.md` and `crates/rapier2d_classes/README.md`: the snippet lines only;
- `examples/ball_drop/Scarb.toml`: the comment line only. If Scarb also updates `examples/ball_drop/Scarb.lock` for the
  path crates' versions, that is fine;
- this brief, `docs/briefs/r11-release-alpha11.md`, your first commit, as given, word for word.

Nothing else: no source, test, gas or CI file. If a generated file (gas, `bytecode.size`, `docs/PACKAGES.md`) moves
because of the version string, stop and report it; do not regenerate it.

## 4. Machines (VPS)

- Every `scarb` call goes through the shim (the heavy lock), with `RAYON_NUM_THREADS=1` and under
  `prlimit --as=8589934592`.
- Never build test crates.

## 5. Programme rules

- **CI logs:** wait for the run to complete, then `gh run view <run-id> --log-failed` alone in its call. Never
  `gh api …/logs`.
- **Git:**
  - your own git commands run normally, as separate commands;
  - before your FIRST push, you may run `git rebase origin/main` in exactly that form, alone in its call;
  - after a push, run `git merge origin/main` alone in its call;
  - every other rebase form and every force push are refused. A rebase conflict is reported, not driven;
  - remove an untracked file only with `git clean -f -- <exact path>`.
- **Refusals:** if a command is refused, put its exact text and the time in your report and stop that part.
- **Pushes:** `scripts/prepush.sh` before each push. One push when done, then one per review's fixes.

## 6. Acceptance criteria

1. One PR to `main`, touching only the allowlist.
2. `git grep -n "0.1.0-alpha.10"` at the head finds only history: the CHANGELOG's past sections, `docs/releases`,
  briefs and research. Also the generated `docs/PACKAGES.md`, if it names the version; say so.
3. Every CI check green, `ci-ok` included, with the "no step change" proof line in the PR: `gas` green with no `.snap`
  entry changed, and `bytecode` green with `gas/bytecode.size` unchanged.
4. **Never merge. Never publish, tag or create a release.**

## 7. Report

The PR and its head; the version strings changed, file by file; the CHANGELOG headings and the Results line.

## 8. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
