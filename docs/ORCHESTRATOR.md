# Orchestrating rapier-cairo

What the orchestrator of the rapier track adds to the standard of the organisation (the rules every session and thread
receives) and to the operating document of the programme (`slingfall/OPERATIONS.md`: models §2, machines and budgets
§3, threads §4, domain rules §5, merge gates and audit lenses §6, releases §7). It never restates them; where it would
contradict them, they win. Since 2026-10-02 the orchestrator is the coordinator of the herdr project `slingfall-rapier`;
implementers, reviewers and auditors are herdr threads it starts. The thread-side rules are [`AGENTS.md`](../AGENTS.md);
sequencing and the status of the track are [`docs/PLAN.md`](PLAN.md).

## Starting a thread

- **Before each start**: `machine-capacity` (slots, memory and pool quota of the VPS and the Mac; under 20 % of the pool,
  fewer threads and a warning with the figure), and the programme's placement rule: the Mac takes every thread that
  compiles or tests Cairo without writing a pin, a gas table, a budget or an archive hash (reviews, implementations
  without regeneration, test runs); the VPS takes the threads that write them, and the light threads with no Cairo
  build. The VPS runs one heavy suite at a time (the shims take the heavy lock); rapier's whole-shot tests peak near
  20 GB.
- **Start**: one thread per lot, on a branch cut from `origin/main`, the task naming the brief
  `docs/briefs/<id>.md` committed first (the thread's first commit may write it); `<id>` is lowercase, code then slug
  (`sh2b-compound`). The profile sets the model: `impl-sonnet` by default, `impl-opus` for the step path and numerics.
- **Read** the thread's report and its pull request, never its transcript. A thread resumed with findings fixes those
  it is given and nothing else; a lost thread is restarted from its brief.
- The thread opens its own pull request, runs `scripts/prepush.sh` before every push (`git -c core.hooksPath=.githooks
  push`; no git config is written) and never merges: it merges only on the orchestrator's line `Merge the PR: ...`
  with the standard's command.
- The old launcher (`scripts/executor-unit.sh`, units `rapier-exec-*`, `scripts/executor/system-prompt.md`) and Nexus
  are not used; nothing is started through them.
- Once a lot's pull request is merged, remove its worktree and its local branch, each by its exact name.

## The build path

Class hashes, Sierra bytes and `gas/bytecode.size` depend on the absolute build path (closure type names): they are
pinned only from CI's artefacts (`bytecode-snapshot`, `class-hashes`), with CI's root path recorded beside each, never
from a local build. Felt counts, CASM, gas and Cairo steps are path-free and may be measured anywhere. A local class
hash that differs from CI's is expected, not a regression.

## Model choice

The tiers are those of `slingfall/OPERATIONS.md` §2. The thread profile of each, with rapier examples:

| Tier | Profile | rapier examples |
|---|---|---|
| mechanical, well framed | `impl-sonnet` | template-generated code, test compaction, spec alignment, parity leftovers with an explicit item list |
| standard port or feature with numerics | `impl-opus` | a new module (kernels, tests, golden vectors, benches); anything on the step path |
| genuinely hard | `impl-opus`; `impl-fable` sparingly, the brief says why | novel numerics, hard debugging, cross-module or cross-class design |

The smaller the model, the tighter the brief.

## The brief

`docs/briefs/<id>.md`, committed before the launch. It carries the content the standard requires (goal, context,
allowlist, interfaces, acceptance criteria, verification, report expected) in the sections and order of
[`AGENTS.md`](../AGENTS.md) §3, with the programme's conditions for the lot written in. Two lots run at the same time only when their allowlists share no file, gas snapshots included.

## Conflict-free parallelism

- Before a wave, pre-declare every stub in the shared files (modules, test files, benches, golden files,
  `gas/<crate>/<module>.snap` targets); one gas snapshot per module. Parallel PRs then never touch a common file.
- Waves follow the dependency graph of `docs/PLAN.md`; a wave starts when its dependencies are merged.
- Merge golden-table extensions before launching the lots that iterate over them: their gas drifts otherwise.
- Watch the compile budget of the test crates (`AGENTS.md` §7): it is the first cause of CI failure observed.

## Closing a lot

1. **The orchestrator's review** of the thread's report and the pull request: scope = allowlist, API parity with upstream,
   deviations (new divergences go to `docs/adr/0001-upstream-divergences.md`), gas and exact-steps tables.
2. **The checks of the kind of lot** (`slingfall/OPERATIONS.md` §6). On the step path: before / after
   `--tracked-resource cairo-steps` tables on the P3, level and game-shaped probes (`AGENTS.md` §7), bit-identity on
   the reference shots, and a `validation` audit when results change.
3. **The review thread**, started with the pull request (`slingfall/OPERATIONS.md` §2): `review` when Opus or Fable
   wrote it, `review-opus` when Sonnet did, read-only, on the exact head. Findings verified to hold go back to the
   thread, batched into one push, then a new review on the new head; after three fix loops on the same lot, stop and
   escalate to the project manager. An audit is the exception, for the kinds of lot of `slingfall/OPERATIONS.md` §6. A
   pull request of documents only needs no review when the project manager says so (`Review: none — documents`).
4. **Squash merge** after a review that does not oppose it and green checks (the thread on the orchestrator's line, or
   the coordinator that started the review thread), never `--admin`; then the orchestrator alone updates re-exports, `CHANGELOG.md` (`## Unreleased`), the ADRs, the
   plan and the status of the track.

## Releases

The go is the project manager's, in writing, per package (`slingfall/OPERATIONS.md` §7 and the standard's "Publishing a
package"): it names the package, the version, the commit and the sha256 of the archive; the registry token stays in the
owner's settings. `scripts/release.sh bump <version>` in a release PR (CHANGELOG entry included), merge, main CI green
at the release commit. The orchestrator then publishes each package by hand with `scarb publish -p <package>` from a
clean checkout of that commit, after comparing the archive's sha256, in dependency order (`rapier_math` →
`rapier_core` → `rapier_geometry2d` → `rapier_dynamics2d` → `rapier2d` → `rapier2d_classes`), reading each back from
the registry, then tags `v<version>`. `scripts/release.sh publish`, which publishes all six at once, is not used for a
publication. Never a thread or CI.
