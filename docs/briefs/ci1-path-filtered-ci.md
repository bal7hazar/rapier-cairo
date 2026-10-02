# CI1 — CI tests run only when their files changed

Runner: `impl-sonnet`, on the VPS (a workflow and a little Python or shell, no Cairo build). Lot id `ci1-path-filtered-ci`.

## 1. Goal and context

The owner's rule (2026-10-02, every repository): "CI tests must absolutely run only if files related to the tests
were modified, so docs should skip all tests." GitHub Actions is starved account-wide; a documents-only PR must run no
test job.

- **On a pull request:** each job of `.github/workflows/ci.yml` runs only when files that concern it changed, computed
  by the workflow itself from the PR's changed paths (never from a label).
- **On a push to `main`:** the full CI keeps running (release gates, traceability).
- **A light final job always runs**, so that `gh pr checks` always has a result: the standard's merge command needs
  one. It passes when every job that ran passed and every job that did not run was skipped by the rule.
- **Already on `main`:** the PR-only `cancel-in-progress` (#252) and the download retries; keep them as they are.

Read first: `AGENTS.md`, `.github/workflows/ci.yml`, `.github/workflows/execute.yml` (it already has a `paths:` filter
for pull requests: leave it), `scripts/gas.py` (`check --from-log`, `--filter`), every crate's `Scarb.toml` (the
dependency graph), `scripts/prepush.sh`.

## 2. Scope: file allowlist

`.github/workflows/ci.yml`, and a small helper if you need one (e.g. `scripts/ci_changes.sh` or `.py`, with a self-test
if it is Python), this brief `docs/briefs/ci1-path-filtered-ci.md` (your first commit, as given). This brief names
`.github/workflows/ci.yml` (the standard's workflow rule, nexus #60). **Edit the workflow with the file-editing tool
only, never by a script that rewrites it.**

Not allowed: deleting a workflow, widening `permissions:`, adding a trigger (`pull_request_target`, `workflow_run`,
`schedule`), writing to the repository or publishing with a token, a third-party action that needs a token, or
weakening any check on `push` to `main`.

The workflow rule for this lot (nexus #62) allows exactly: a path filter per job, and a final job that always runs
(`needs: …` with `if: always()`) and returns the result of the jobs that ran. **Refused, do not write them:** a new
`continue-on-error`, an `if: false`, a narrowed test command (fewer tests, a filter on `snforge test`, an early exit),
a deleted job. The `continue-on-error` already on `main` (the download retries of #252 and the class-hash capture of
TC1) stays exactly as it is.

## 3. The mapping "job → paths that trigger it" (on pull requests)

**Everything runs** when any of these changed: `.tool-versions`, the root `Scarb.toml`, `Scarb.lock`, any crate's
`Scarb.toml`, `.github/workflows/ci.yml`, the helper of this lot.

Crates and their dependents, from the manifests (check them with `scarb metadata` or by reading the manifests, and
correct the table if a dev-dependency adds an edge; say so in the report):
- `rapier_math`, `rapier_core` → everything below `rapier_geometry2d`;
- `rapier_geometry2d` → `rapier_dynamics2d` → `rapier2d` → `rapier2d_classes` → `rapier_sink`;
- `rapier_golden` and `rapier_testing` → the test crates that use them.

| job | runs when changed (besides the "everything" list) |
|---|---|
| `fmt`, `lint`, `build` | any `crates/**` file |
| `test (core-math-golden)` | `crates/rapier_math/**`, `crates/rapier_core/**`, `crates/rapier_golden/**`, `crates/rapier_testing/**` |
| `test (geometry2d)` | `crates/rapier_geometry2d/**` and its dependencies (math, core, testing, golden if used) |
| `test (dynamics2d)` | `crates/rapier_dynamics2d/**` and its dependencies |
| `test (rapier2d)` | `crates/rapier2d/**` and its dependencies |
| `sink (rapier2d_classes)` | `crates/rapier2d_classes/**` and its dependencies |
| `sink (rapier_sink)` | `crates/rapier_sink/**`, `crates/rapier2d_classes/**` and their dependencies |
| `gas` | whenever a `test` group ran, or `gas/**/*.snap`, or `scripts/gas.py` changed |
| `bytecode` | any crate that `rapier2d_classes` or `rapier_sink` depends on (that is, every engine crate), `gas/bytecode.size`, `scripts/bytecode_size.py` |
| `Consumer cost` | the published crates' `crates/**` (math, core, geometry2d, dynamics2d, rapier2d, rapier2d_classes), `scripts/consumer_cost.py`, `scripts/packages_table.py`, `docs/PACKAGES.md` |
| `api-parity` | `crates/**/src/**`, `scripts/api_parity.py`, `docs/API_PARITY.md` |
| `golden` | `tools/golden/**`, the golden vectors it compares, `crates/rapier_golden/**` |

**The `gas` job:** it compares the merged `snforge-*` logs with the committed `gas/**/*.snap`. When only some test
groups ran, it must check only the snapshots of those groups (`scripts/gas.py check --from-log … --filter …`, or one
check per group). It must never fail because a skipped group's entries are missing, and never pass a moved entry of a
group that ran. Show both cases in the report.

**Checked `.md` files are not prose:** a Markdown file that a job checks or that a script regenerates and compares
triggers that job. In this repository: `docs/API_PARITY.md` → `api-parity`; `docs/PACKAGES.md` → `Consumer cost`.
`CHANGELOG.md` is read by no job of `ci.yml` (only `execute.yml`'s own `paths:` filter, which stays as it is). Search
`scripts/**` and the workflows for any other `.md` that is read or compared, add it to the table, and say what you
found. Only prose `.md` (the plan, the briefs, research, ADRs, `README.md`, `AGENTS.md`, `CLAUDE.md`, `docs/BUDGETS.md`
unless a script compares it) triggers nothing.

**Documents-only PR** (prose only): every job is skipped, and the final job passes.

## 4. Implementation

- **A first job `changes`** (`runs-on: ubuntu-latest`, no token beyond the default read). It exposes one boolean
  output per job, computed from the PR's changed paths, either:
  - with `dorny/paths-filter`, **pinned by its full commit sha** (as nalgebra-cairo does: copy the pin from there and
    say which version it is), with renames counted on both paths;
  - or with `git diff --name-only --no-renames <base sha>...<head sha>` (base and head from the event payload,
    `fetch-depth: 0`).

  Say which you chose. On `push` (to `main`) and on `workflow_dispatch`, every output is `true`.
- **Each job:** `needs: changes` and `if: needs.changes.outputs.<job> == 'true'`. For the `test` and `sink` matrices,
  build the matrix from `changes`' outputs (`fromJSON` of a JSON list `changes` emits), so an unconcerned entry is not
  scheduled at all; a matrix job never exits early with "nothing to do". If no entry is concerned, the whole job is
  skipped by its `if:`.
- **The final job** (`name: ci-ok`): `needs:` every other job, including `changes`, and `if: always()`.
  - It fails when `changes` itself did not succeed.
  - It fails when any job that ran failed or was cancelled.
  - It fails when a job was skipped although `changes` said it should run (a skip by error, for example a failed
    dependency).
  - It passes only when every job either passed, or was skipped and `changes` said it should not run.
  - It writes a one-line summary per job: ran and passed / skipped by rule / failed / skipped by error.
- **Git hygiene** (programme rule): any git command that is not a read of the checked-out repository runs only in a
  sanitised environment (`env -u GIT_DIR -u GIT_WORK_TREE -u GIT_INDEX_FILE -u GIT_COMMON_DIR -u GIT_PREFIX`, or
  `env -i`). A self-test that needs a git fixture creates it in a temp dir that way.
- Keep `RAYON_NUM_THREADS: 1`, the TC1 artefacts (`bytecode-snapshot`, `class-hashes`), the retries and the
  concurrency block exactly as they are.

## 5. Acceptance criteria

1. **This PR itself:** it changes `ci.yml`, so every job runs and passes, and `ci-ok` passes.
2. **Proof of the filter**, without extra PRs to `main`: show the `changes` outputs for these path lists, by running
   the computation locally on each list (or through a self-test of the helper):
   - (a) `docs/PLAN.md` only; (a2) `docs/API_PARITY.md` only;
   - (b) `crates/rapier2d/src/world.cairo`;
   - (c) `crates/rapier_math/src/lib.cairo`;
   - (d) `scripts/gas.py`;
   - (e) `Scarb.lock`.

   Give each with the expected and the actual set of jobs.
3. `ci-ok` logic shown on four cases: all ran and passed; some skipped by rule; one that ran failed; one skipped by error (a job that `changes` marked to run but that did not run).
4. `gas` with only some test groups: the filtered check (§3).
5. `scripts/prepush.sh` passes before each push; push with plain `git push` (the hook runs).
6. A pull request; **never merge**. One push when done, then one per review's fixes (CI is starved). Prefer words over
   links to external issues in PR text and commit messages.

## 6. Report

Your thread report as your rules say: the mapping as implemented (and any edge you corrected), the five path lists
with their jobs, how `gas` filters, how `ci-ok` decides, and the CI result of this PR.

## 7. Autonomy

Work autonomously, do not ask questions, do not widen the scope, foreground only.
