# CI2 — a mismatched class-hash pin fails CI

Runner: `impl-sonnet`, on the VPS (workflow and Python, no Cairo build). Lot id `ci2-pin-gate`.

## Goal and context

`crates/rapier2d_classes/tests/hashes.cairo` pins the declared classes' hashes; `test_pinned_class_hashes` is `#[ignore]`,
and its only CI run is the "Declared class hashes" capture step of the `sink (rapier2d_classes)` job, which has
`continue-on-error: true` and `|| true`: a hash that differs from CI's build never fails CI. The project manager asks
for a gate (2026-10-03).

## Scope: allowlist

`.github/workflows/ci.yml` (named here: the workflow rules nexus #62 and #69 apply), `scripts/ci_changes.py` (its
self-test only), this brief `docs/briefs/ci2-pin-gate.md` (your first commit, as given). Edit the workflow with the
file-editing tool only, never a script. Refused: a new `continue-on-error`, `if: false`, a narrowed test command, a
deleted job, a new trigger or permission.

## Changes

1. In the `sink` job, step "Declared class hashes": remove its `continue-on-error: true`; keep its command's `|| true`
   (the artefact `class-hashes` must still be written and uploaded when the pins differ: that is how a lot takes the
   new values).
2. After the `class-hashes` upload, a new step "Pinned class hashes match CI's build" (`if: always() && matrix.package ==
   'rapier2d_classes'`): it fails unless `class-hashes.txt` shows `test_pinned_class_hashes` passed (e.g. the line
   `Tests: 1 passed, 0 failed`) and lists no `pinned … declared …` difference; on failure it prints the differing
   lines and says the new values are in the `class-hashes` artefact (CI's root path beside them).
3. CI1's open note: extend `scripts/ci_changes.py`'s self-test so it reads the job ids of `.github/workflows/ci.yml`
   and fails if one is missing from `JOBS` + `changes` or from `ci-ok`'s `needs:` (a job added later can then not run
   ungated or outside `ci-ok`).
4. Git hygiene: any git command that is not a read of the checked-out repository runs only in a sanitised
   environment (`env -u GIT_DIR -u GIT_WORK_TREE -u GIT_INDEX_FILE -u GIT_COMMON_DIR -u GIT_PREFIX`, or `env -i`).

## Acceptance criteria

1. Show, by running the new check's logic locally on two sample `class-hashes.txt` files (one passing, one with a
   `pinned … declared …` line and `1 failed`), that it passes and fails as it should.
2. The extended self-test passes on the new `ci.yml`, and fails on a copy where a job is removed from `ci-ok`'s `needs:`.
3. This PR changes `ci.yml`, so every job runs: all checks green, the new step included (the current pins match CI).
4. `scripts/prepush.sh` passes; plain `git push`. One push, then one per review's fixes. Never merge. Prefer words over
   links to external issues.

## Report

Your thread report as your rules say. Autonomy: work autonomously, do not ask questions, do not widen the scope,
foreground only.
