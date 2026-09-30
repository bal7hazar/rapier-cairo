# Orchestrating rapier-cairo

What the orchestrator of the rapier track adds to the standard roles of Nexus (`bal7hazar/nexus`: `roles/common.md`,
`roles/session.md`, `roles/orchestrator.md`, `roles/implementer.md`, `roles/reviewer.md`, and the skills
`nexus-agents`, `nexus-capacity`, `nexus-handover`) and to the operating document of the project
(`slingfall/OPERATIONS.md`: models §2, machine and budgets §3, launchers §4, domain rules §5, merge gates and audit
lenses §6, releases §7). It never restates them; where it would contradict them, they win. The executor-side rules are
[`AGENTS.md`](../AGENTS.md); sequencing and the status of the track are [`docs/PLAN.md`](PLAN.md).

## Launching an executor

rapier keeps its own launcher until `slingfall/OPERATIONS.md` §4 names `nexus` for the track; reviews and audits
already go through `nexus`. A lot is never started with both.

- **Before each launch**: the capacity rule of `slingfall/OPERATIONS.md` §3 (for rapier: `free_slots` ≥ 2 and at most
  one rapier executor at a time), then `nexus resources` and `nexus accounts`, which do not count this launcher's
  units.
- **Launch**: `scripts/executor-unit.sh <id> claude:<sonnet|opus|fable> docs/briefs/<id>.md`, the brief committed on
  `main` first; `<id>` is lowercase, code then slug (`sh2b-compound`). It runs `scripts/executor.sh` in the transient
  systemd user unit `rapier-exec-<id>`: branch `feat/<id>` cut from `origin/main`, worktree
  `.claude/worktrees/exec-<id>`, log `~/orchestrator/logs/rapier-cairo/<id>.log`, and the frame
  `scripts/executor/system-prompt.md` prepended to the brief.
- **Follow** each executor with one background task that ends when its unit ends
  (`systemctl --user is-active rapier-exec-<id>`), titled with the model that runs, read from the `"model"` field at
  the head of its log (`claude:sonnet` ran as `claude-sonnet-5-5` for PX3–PX5): `[Sonnet 5.5] PX5 joint closure`.
- **Read** `REPORT.md` at the root of the executor's worktree and the log, never the transcript.
- **Resume, never relaunch**: `scripts/executor-unit.sh resume <id> claude:<model> "<follow-up>"` continues in the
  same worktree; `EXECUTOR_FRESH=1` starts a new claude session there (a lost session, a change of model).
- The launcher's `codex` runner (`audit-<id>` lots) is retired: reviews and audits run through `nexus review` and
  `nexus audit`.
- Once a lot's pull request is merged, remove its worktree and its local branch.

## Model choice

The tiers are those of `slingfall/OPERATIONS.md` §2. The runner of each, with rapier examples:

| Tier | Runner | rapier examples |
|---|---|---|
| mechanical, well framed | `claude:sonnet` | template-generated code, test compaction, spec alignment, parity leftovers with an explicit item list |
| standard port or feature with numerics | `claude:opus` | a new module (kernels, tests, golden vectors, benches); anything on the step path |
| genuinely hard | `claude:opus`; `claude:fable` sparingly, the brief says why | novel numerics, hard debugging, cross-module or cross-class design |

The smaller the model, the tighter the brief.

## The brief

`docs/briefs/<id>.md`, committed before the launch. It carries the content the standard requires (goal, context,
allowlist, interfaces, acceptance criteria, verification, report expected) in the sections and order of
[`AGENTS.md`](../AGENTS.md) §3, with the programme's conditions for the lot written in: messages to a running executor
are held. Two lots run at the same time only when their allowlists share no file, gas snapshots included.

## Conflict-free parallelism

- Before a wave, pre-declare every stub in the shared files (modules, test files, benches, golden files,
  `gas/<crate>/<module>.snap` targets); one gas snapshot per module. Parallel PRs then never touch a common file.
- Waves follow the dependency graph of `docs/PLAN.md`; a wave starts when its dependencies are merged.
- Merge golden-table extensions before launching the lots that iterate over them: their gas drifts otherwise.
- Watch the compile budget of the test crates (`AGENTS.md` §7): it is the first cause of CI failure observed.

## Closing a lot

1. **The orchestrator's review** of `REPORT.md` and the pull request: scope = allowlist, API parity with upstream,
   deviations (new divergences go to `docs/adr/0001-upstream-divergences.md`), gas and exact-steps tables.
2. **The checks of the kind of lot** (`slingfall/OPERATIONS.md` §6). On the step path: before / after
   `--tracked-resource cairo-steps` tables on the P3, level and game-shaped probes (`AGENTS.md` §7), bit-identity on
   the reference shots, and a `validation` audit (`nexus audit`) when results change.
3. **The Codex review**, once CI is green: `nexus review --project slingfall --task <ID> --repository rapier-cairo
   --branch feat/<id> --brief docs/briefs/<id>.md`, followed like an executor and titled with the reviewer's model
   (`nexus status`). Findings verified to hold go back to the executor with `scripts/executor-unit.sh resume`, then a
   new review on the new head; after three fix loops on the same lot, stop and escalate to the project manager. A merge
   without a review, only in the two cases of the skill `nexus-agents` (documents only; Codex unavailable), writes
   `Codex review: none — <reason>` in the pull request.
4. **Squash merge**, then the orchestrator alone updates re-exports, `CHANGELOG.md` (`## Unreleased`), the ADRs, the
   plan and the status of the track.

## Releases

The go is the project manager's, in writing (`slingfall/OPERATIONS.md` §7, the owner's delegation of 2026-09-25); the
registry token stays in the owner's settings. `scripts/release.sh bump <version>` in a release PR (CHANGELOG entry
included), merge, main CI green at the release commit, then `scripts/release.sh publish` from a clean `origin/main`:
it publishes `rapier_math` → `rapier_core` → `rapier_geometry2d` → `rapier_dynamics2d` → `rapier2d` →
`rapier2d_classes`, verifying each against the registry, and tags `v<version>`.
