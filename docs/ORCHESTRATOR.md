# Sub-agent strategy (orchestrator)

Instructions for the orchestrator session. This file is meant to be pasted verbatim into the
prompt of an orchestrator of another repository (nalgebra-cairo, rapier-cairo). Porter-side rules
live in `AGENTS.md`, design decisions in `docs/DESIGN.md`, sequencing in `docs/PLAN.md`.

Role of the main session: orchestrate, split, brief, review, merge. Never implement anything
large directly.

## Execution: local CLIs, not the Agent tool

- The Agent tool burns the orchestrator session's quota: use it only for short, read-only
  research.
- Every sub-task runs in its own git worktree + branch (`feat/<module>`), launched in the
  background with its output redirected to a log file.
- **Every implementation lot runs on the `claude` CLI** (its own account, distinct from the
  session): `claude -p "$(cat brief.md)" --model <sonnet|opus|fable> --dangerously-skip-permissions --name <task>`;
  resume with context: `claude --continue -p "<follow-up>"` in the same worktree.
- **`codex` is for audits and second opinions only, used sparingly** (owner's rule, 2026-09-25: its
  quota is small and shared between the orchestrators): the review of a merged lot, a cross-check
  of a numeric decision, an independent opinion on a design. Never an implementation lot.
  `codex exec -C <worktree> -m <model> -c model_reasoning_effort=<medium|high|xhigh> --dangerously-bypass-approvals-and-sandbox -o LAST_MESSAGE.md "$(cat audit.md)"`.
  `scripts/executor.sh` refuses a codex launch unless the lot id starts with `audit-`. A lot started
  on codex that hits the quota is handed over to claude in the same worktree
  (`EXECUTOR_FRESH=1 scripts/executor-unit.sh resume <id> claude:opus "<follow-up>"`).
- The agent writes a `REPORT.md` (not committed) at the root of its worktree: the orchestrator
  reads that file and the log, not the transcript.

## Model choice by difficulty

| difficulty | claude CLI (implementation) | examples |
|---|---|---|
| mechanical, well framed | Sonnet 5 | template-generated code, test compaction, spec alignment, benching variants already identified |
| standard port with numerics | Opus 5.5 | a new module: kernels, tests, golden vectors, benches |
| genuinely complex | Fable 5.1 (sparingly) | novel numerics, hard debugging, cross-module design, API arbitration |

Audits on codex (only `gpt-5.5` and `gpt-6-astra` are available on this login): `gpt-5.5` effort
`high` for a lot review, `gpt-6-astra` effort `xhigh` for a hard numeric or design cross-check.

- The strong models are not the default, but do not rule them out when the problem warrants
  them.
- The smaller the model (or the lower the effort), the tighter the brief must be.

## The brief (mandatory, in this order)

1. Files to read first (`AGENTS.md`, `docs/DESIGN.md`, style precedents on `main`).
2. Strict scope: a file allowlist; everything else is forbidden. Shared files (`lib.cairo`,
   `Scarb.toml`, CI, design docs, CHANGELOG, status) belong to the orchestrator: the agent lists
   its needs in an "Escalations" section of the report instead of editing them.
3. Expected API (exact names from the source being ported), numeric semantics, what is
   explicitly deferred (DEFER).
4. Efficiency rules and numeric targets (gas/steps); variants to bench when the formulation is
   not obvious (the winner in the library, the losers in `benches::alt` with their benches).
5. Tests: table-driven, compile budget (max file size, max number of fuzz tests), golden vectors
   from the reference oracle, panics with exact messages.
6. Definition of done: the full gate run in the **foreground** (never a background command
   followed by the end of the turn: in headless mode the session stops), gas snapshots
   regenerated, conventional commits with the trailer, push, PR via `gh pr create` following the
   template, `gh pr checks --watch` until green, **never merge**, `REPORT.md` in the imposed
   format (summary, API, gas table, deviations, deferred items, requested re-exports,
   escalations, PR URL).
7. "Work autonomously, do not ask questions, do not widen the scope."

## Conflict-free parallelism

- Pre-declare every stub (modules, tests, benches, golden files) in the shared files before
  launching a wave; one gas snapshot per module. Parallel PRs then never touch a common file.
- Waves follow the dependency graph; a wave starts when its dependencies are merged.
- After each merge, the orchestrator alone updates re-exports, status, changelog and design
  decisions, then pushes to `main`.

## Quality control and quota

- Merge only on green CI + a review of the report (API parity, deviations, gas table).
- An interrupted agent (rate limit, end of turn) is resumed with `claude --continue -p` rather
  than relaunched from scratch.
- Watch the compile budget of the test crates: it is the first cause of CI failure observed.
