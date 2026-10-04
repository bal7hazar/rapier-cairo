# R10 — release 0.1.0-alpha.10, step 1: the version bump on main

Runner: `impl-sonnet` (a documented, mechanical release step), on the VPS. Lot id `r10-release-alpha10`.

## 1. Goal and context

Prepare `0.1.0-alpha.10` on `main`, as `0.1.0-alpha.9` was prepared by #260 (`7c494f2`: `Scarb.toml`, `Scarb.lock`,
`CHANGELOG.md` and `docs/PLAN.md`; read that commit first).

`0.1.0-alpha.10` carries two MINOR result changes, approved by the project manager to ship together, so a consumer
re-pins once:
- **FU1** (#270): fused fixed-point rescales on the step path (P1, P2, P3, C5).
- **CE** (#271): compound internal edges (an opt-in strategy), and the query answers following parry 0.31.1.

It also carries everything else merged since `v0.1.0-alpha.9`:
- OB (#269);
- B1 (#268);
- SW1;
- PX8;
- the FU0 study;
- any other entry now under `## Unreleased`.

This step does **not** publish anything and does not touch the release branch. The orchestrator does the release
commit off `main` and the staged publication afterwards.

## 2. What to change

1. **The version.** Bump every published crate's version from `0.1.0-alpha.9` to `0.1.0-alpha.10`:
   - `rapier_math`, `rapier_core`, `rapier_geometry2d`, `rapier_dynamics2d`, `rapier2d` and `rapier2d_classes`;
   - the workspace `Scarb.toml`, and wherever #260 changed it;
   - the inter-crate requirements that name the version;
   - `Scarb.lock`, through Scarb itself (`scarb build` or `scarb metadata` through the shim, under the cap), never by
     hand.
2. **`CHANGELOG.md`.** Turn `## Unreleased` into `## 0.1.0-alpha.10 (2026-10-04)`, in #260's format, and leave an
   empty `## Unreleased` above it.
   - Add the **Results** line, as alpha.9 has one: "Results change: a MINOR result change (FU1, CE). A consumer re-pins
     once: class hashes, `WorldState` and replay digests, and the cost sheet." Name the classes a game re-pins, from
     FU1's and CE's entries.
   - Do not reword the other entries, except to merge exact duplicates.
3. **`docs/PLAN.md`:**
   - the version line, as #260 did;
   - **one new "Parked" entry for WS3**, the project manager's decision of 2026-10-03. #266 was closed and not merged,
     and `WorldState` stays v3. Use the plan's existing entry format, with the findings:
     - the opt-in dormant-pair layout gains −0.53 % on the reference shot (slim) and −4.46 % on L20 over 60 ticks;
     - the default path costs +0.15 to +2.16 % (P3 probes), because two boxed cells ride along every `ref`;
     - `take_removed` alone costs +0.1 to +0.2 %;
     - the v4 codec costs about 7.8k steps per state round trip, paid at every chunk boundary;
     - the design note for a later lot: boxed cells only when opted in;
     - 14 diagnostic tests read `narrow_phase.pairs` as the whole list, and a default layout would need them rewritten.

## 3. Scope: allowlist

- `Scarb.toml` (the workspace and each crate's, for the version only) and `Scarb.lock`;
- `CHANGELOG.md`;
- `docs/PLAN.md`;
- this brief, `docs/briefs/r10-release-alpha10.md`, your first commit, as given.

Nothing else. In particular, no source, test, gas or CI file. If a generated file (gas, `bytecode.size`,
`docs/PACKAGES.md`) moves because of the version string, stop and report it: do not regenerate.

## 4. Machines (VPS)

- Every `scarb` call goes through the shim (the heavy lock), with `RAYON_NUM_THREADS=1` and under
  `prlimit --as=8589934592`.
- Never build test crates.

## 5. Programme rules

- **CI logs:** wait for the run to complete, then `gh run view <run-id> --log-failed` alone in its call. Never
  `gh api …/logs`.
- **Git:**
  - your own git commands run normally, as separate commands;
  - to pick up `main`, `git merge origin/main` alone in its call; never another rebase, never a force push;
  - remove an untracked file only with `git clean -f -- <exact path>`; never `-d`, `-x` or `-X`.
- **Text:** prefer words over links to external issues. `gh pr edit` works.
- **Processes:** signal only processes you started, by the pid you recorded.
- **Refusals:** if a command is refused, put its exact text and the time in your report and stop that part.
- **Pushes:** `scripts/prepush.sh` before each push. One push when done, then one per review's fixes.

## 6. Acceptance criteria

1. One pull request to `main` with the four changes of §2, and nothing outside §3.
2. `git diff --stat origin/main` shows only the allowlisted files.
3. Every CI check green, `ci-ok` included. The "no step change" proof line in the PR: `gas` green with no `.snap`
   entry changed, and `bytecode` green with `gas/bytecode.size` unchanged.
4. **Never merge. Never publish, tag or create a release.**

## 7. Report

Your thread report as your rules say:
- the PR and its head;
- the version strings changed, file by file;
- the CHANGELOG section's headings;
- the WS3 plan entry, quoted.

## 8. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
