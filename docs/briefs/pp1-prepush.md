# PP1 — a pre-push check: never push red

Runner: `impl-sonnet`, on the VPS. Lot id `pp1-prepush`.

## 1. Goal and context

**Goal.** Stop red CI runs that a local check of seconds to minutes would have caught (owner's request,
2026-10-02: 22 failed runs of about 770 across the programme since 2026-10-01; about 15 were avoidable — formatting,
generated artefacts not regenerated (rapier: `gas/bytecode.size`), unit tests of scripts, compile errors; 6 were
infrastructure: HTTP 500 while downloading scarb or snforge).

Two steps:
- **Step A** (now): `scripts/prepush.sh`, the hook `.githooks/pre-push`, the rule in `AGENTS.md`.
- **Step B** (on the orchestrator's word, after the TC1 pull request merges, because it also edits `ci.yml`): one or
  two retries on the workflow steps that download scarb or snforge.

Read first: `AGENTS.md`, `CLAUDE.md` (commands), `.github/workflows/ci.yml` and `execute.yml` (what CI checks, and
how), `scripts/gas.py`, `scripts/bytecode_size.py`, `scripts/consumer_cost.py`, `scripts/api_parity.py`,
`scripts/packages_table.py` (their docstrings: inputs and outputs), `scripts/build-shims/`.

## 2. Scope: file allowlist

- Step A: `scripts/prepush.sh` (new), `.githooks/pre-push` (new, executable), `AGENTS.md` (one short rule, in the
  section that lists the verification an executor runs), and this brief `docs/briefs/pp1-prepush.md` (your first commit,
  as given).
- Step B: `.github/workflows/ci.yml` and `.github/workflows/execute.yml`, for the retries only. This brief names them
  (the standard's workflow rule, nexus #60). Not allowed: deleting a workflow, widening `permissions:`, adding a trigger,
  writing to the repository or publishing with a token, or weakening a check that guards the merge.
- Everything else: Escalations.

## 3. What `scripts/prepush.sh` does

Bash, `set -euo pipefail`, run from any directory of a clone or worktree. **Target: under 2 minutes** on a typical
change. It compares the branch with its merge base on `origin/main` (`git merge-base HEAD origin/main`, plus the
working tree) to know what changed, and runs:

1. Always: `scarb fmt --check` (workspace).
2. Always, when the scripts or their tests changed, or cheaply always: the unit tests of the Python scripts that have
   them (`scripts/gas.py`, `scripts/consumer_cost.py` contain tests; find how they are run, e.g. `python3 -m doctest` /
   `unittest` / a `test` subcommand).
3. The compile of the packages touched: `scarb build -p <crate>` (and `scarb lint -p <crate> --deny-warnings` if it
   stays within budget) for each crate under `crates/` with a changed file; the whole workspace only when a root
   manifest, `Scarb.lock` or `.tool-versions` changed.
4. The generated artefacts, **only when their inputs changed**, each named with its trigger:
   - the API parity table (`scripts/api_parity.py` check mode, if it has one) when a public source changed;
   - gas snapshots: `scripts/gas.py` check of the touched crate only when a `gas/**/*.snap` input changed — if a check
     needs the test run (minutes), print the command to run instead of running it, and say so;
   - `gas/bytecode.size`: it is generated in CI only (the class-size rule: `sierra_bytes` depend on the build path), so
     the script never regenerates it; when the declared classes' sources, a manifest or the toolchain changed, it
     prints a warning that `bytecode` will move and that the CI artefact `bytecode-snapshot` (once TC1 adds it) is the
     source of the new file;
   - `docs/PACKAGES.md`: likewise CI-generated (`consumer-cost` artifact): warn only.
5. Exports `RAYON_NUM_THREADS=1` for anything it compiles. Never `snforge test --workspace`; never a whole-shot suite
   (`rapier2d_classes`, `rapier_sink`).
6. Prints each stage with its wall time and a final `prepush: OK (<seconds> s)` or `prepush: FAILED at <stage>`;
   exits non-zero on any failure. `PREPUSH_FULL=1` forces stages 3–4 on everything (for a manual check).

`.githooks/pre-push`: runs `scripts/prepush.sh` from the repository root and fails the push when it fails.

`AGENTS.md`: "Before every push run `scripts/prepush.sh` (the `pre-push` hook does it when `core.hooksPath` is
`.githooks`); never push red; never skip the hook." Keep it to that, in the existing style.

## 4. Step B: retries

On every step that downloads scarb or snforge (`software-mansion/setup-scarb@v1`, `foundry-rs/setup-snfoundry@v6`, in
`ci.yml` and `execute.yml`): one or two retries on failure, with the smallest change that works in GitHub Actions (for
`uses:` steps there is no built-in retry: e.g. the step with `id` and `continue-on-error: true`, then the same step
again `if: steps.<id>.outcome == 'failure'`). Nothing else in the workflows changes; a job still fails when the last
attempt fails.

## 5. Acceptance criteria

1. `scripts/prepush.sh` passes on a clean branch at `origin/main`, and fails (non-zero, named stage) on: a formatting
   error, a compile error in a crate, a failing script unit test. Show each with real output (make the error in a
   scratch commit you then drop locally; nothing of it is pushed).
2. **Measured run time**, real output of `time scripts/prepush.sh` (or the script's own total), on this VPS: (a) a
   documents-only change, (b) a change in one small crate (e.g. `rapier_math`), (c) a change in `rapier_dynamics2d`,
   (d) `PREPUSH_FULL=1`. Name the machine and the load at the time (`uptime`). If (b) or (c) exceed 2 minutes, say which
   stage costs it.
3. `.githooks/pre-push` blocks a push when the script fails (show it on a scratch branch that is not pushed: e.g.
   `git push --dry-run` does not run hooks, so use a local bare remote or show the hook's exit code directly).
4. Your own pushes of this lot go through the hook: set `git config core.hooksPath .githooks` in your worktree before
   the first push (it applies to the whole clone; that is intended — say so in the report).
5. Step B: the workflows' download steps have the retries; every CI check of the PR green.
6. Step A: a pull request with the brief, the script, the hook and the `AGENTS.md` line. Step B, when the orchestrator
   says so: a second pull request from a new branch cut from `origin/main`, with the retries only. **Never merge.**

## 6. Report expected

Your thread report as your rules say. Lead with the measured run times (criterion 2), then what each stage checks and
when, what is left to CI, and anything the script cannot check in under 2 minutes.

## 7. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
