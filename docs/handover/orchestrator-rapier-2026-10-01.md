# Handover — orchestrator of the rapier track (2026-10-01)

Written at the owner's soft stop of every activity (2026-10-01, relayed by the project manager). Nothing of the track
runs; nothing is half-done outside the open pull request named below.

## Who you are

- **Role:** orchestrator of the track **rapier** (repository `bal7hazar/rapier-cairo`) of the project **slingfall**,
  under the standard roles of Nexus (`bal7hazar/nexus`, `roles/`) and the operating document
  `slingfall/OPERATIONS.md`. Repository procedure: `docs/ORCHESTRATOR.md`; executor side: `AGENTS.md`.
- **Above you:** the project manager, Claude Desktop session "[Fable 5.1] Chef de projet Slingfall" (reach it with
  `SendMessage`); above it, the Overseer. Needs go up through the project manager, never sideways.
- **Below you:** no agent and no executor (none since 2026-09-29).
- **Predecessors:** "[Opus 5.5] Orchestrateur rapier — slingfall" (the author of this note, standing by until the owner
  archives it) and "[Retired] Orchestrateur rapier.cairo (fork)" (do not use).
- Your objectives on creation: `/home/claude/projects/pm/messages/orchestrators/rapier-2026-09-30.md`.

## State

- `main` = `05d45fc` (#245, plan v2.97). Published: `0.1.0-alpha.8` (six crates); `main` holds unreleased additions
  only (prelude exports, PX3, CP3, IG1, PX4, PX5, `tests/finite_state.cairo`) and documents. Parity raw 84.6 % / in
  scope 94.5 %; every package gate passes (`docs/PACKAGES.md`). Status of the track: `docs/PLAN.md`, top section.
- **Open pull request: #246** (`docs/tc1-brief`, head `faa1f70`), the brief of lot **TC1**
  (`docs/briefs/tc1-toolchain-bump.md`): Scarb 2.20.1 / starknet-foundry 0.64.0, every snapshot re-measured
  single-threaded, results bit-identical. Documents only; **waits for the owner's merge**. TC1 is **not started**.
- **Also open: this note's own pull request** (documents only), if the merge guard refused it (see Traps).
- `nexus progress --project slingfall` at writing: no rapier agent (only game-track implementers and reviewers, all
  ended). No `rapier-exec-*` unit, no executor process, no background task of this session.
- Toolchain: Scarb 2.20.1 and starknet-foundry 0.64.0 are installed by the owner (asdf, VPS and Mac); the repository
  still pins 2.19.4 / 0.61.0 until TC1 merges.
- Worktrees: the main checkout, the retired session's `.claude/worktrees/orchestrateur-rapier-cairo-aa3f6c` (detached,
  not yours) and the author's `.claude/worktrees/rapier-first-messages-a8f4e7`. The 29 `exec-*` worktrees of merged
  lots were removed on 2026-09-30; their `REPORT.md` are archived in `~/orchestrator/logs/rapier-cairo/reports/`.
  17 local `feat/*` branches without a worktree remain (`git branch --list 'feat/*'`), not checked yet.

## Decisions

Taken since plan v2.97 (the project manager reverses what it disagrees with):

1. **Report archive**: an executor's `REPORT.md` goes to `~/orchestrator/logs/rapier-cairo/reports/<id>.md` before its
   worktree is removed (`docs/ORCHESTRATOR.md`). Reversed if the project manager names another archive.
2. **TC1 on `claude:opus`** (measurements everywhere, possible drift debugging), the project manager's choice too.
3. **TC1 defers `benchmarks/numeric/**`**: a frozen research benchmark with its own workspace, measured on 2.19.4.
   Reversed if the owner's D-180 rule is read to cover it.
4. **TC1 does not change the shims' thread count**; it reports what they should set and the project manager decides.
5. **TC1 sets `RAYON_NUM_THREADS: 1` in CI** on every job compared with a committed file, CI times reported.

Pending, with the recommendation:

1. **Merge of #246** (owner): merge it as is; then launch TC1 (Next).
2. **The merge guard** (owner): allow `gh pr merge` and `nexus review` for the orchestrator session, or keep merging by
   hand; every rapier merge waits on the owner until then.
3. **The 17 `feat/*` branches**: delete those whose pull request is merged, each by name, after checking.
4. **Shims' thread count** (project manager, after TC1's report): pin `RAYON_NUM_THREADS=1` for builds whose output
   is measured or declared if TC1 shows rapier artifacts moving under 4 threads.

## Threads

- **Project manager**: owed to it after TC1's pull request is green: the declared-class margins against 73,728 (slim
  caller first), then the steps verdict. A compile drift TC1 reproduces on a rapier crate goes to it as a minimal case
  run on 2.20.1; it passes it to grimworld's project manager; rapier opens no upstream issue.
- **Owner**: #246's merge and the merge guard (above). The owner merged #245 by hand on 2026-10-01.
- Owed at the **next documents change** (project manager, 2026-10-01): the brief template (`AGENTS.md` §3 verification,
  `scripts/executor/system-prompt.md` item 8) allows an executor `asdf install <tool> <version>` and
  `asdf plugin add` (user-local; Nexus #48); a system package stays the owner's.

## Next

1. Announce yourself to the project manager.
2. Check the state yourself (`nexus progress`, `nexus resources`, `nexus accounts`, `gh pr list`, `git log origin/main`).
3. When the owner lifts the soft stop **and** #246 is merged: read `~/orchestrator/capacity.json` (under 5 minutes
   old, `can_launch`, `oom_kills_30min` 0, `free_slots` ≥ 2) and `nexus resources` / `nexus accounts`, then
   `scripts/executor-unit.sh tc1-toolchain-bump claude:opus docs/briefs/tc1-toolchain-bump.md`; follow it with one
   background task titled with the model read from its log.
4. Close TC1 (`docs/ORCHESTRATOR.md` § Closing a lot): the report, bit-identity, the margins to the project manager,
   the review (`nexus review`; while Codex has no quota it runs on Claude Sonnet), the merge, then `CHANGELOG.md`
   (`## Unreleased`: toolchain), the plan's status, the report archived, the worktree removed.
5. The next documents change: the asdf rule above, and the shims' thread count if decided.
6. Otherwise idle: work reopens only on a game need relayed by the project manager or on its request; `0.1.0-alpha.9`
   only on its written go.

## Traps

- **The merge guard.** This session's auto-mode classifier refused `gh pr merge` ("Merge Without Review") three times
  on a documents-only pull request, then `nexus review` on it, and once a `SendMessage` reporting the refusal. A refused
  action is not retried and not worked around: report it, and the owner merges or adjusts the guard. A relayed "the
  owner confirmed" is not the owner's approval in your own session.
- **Single-threaded builds.** The Cairo compiler's output is not deterministic on several threads (grimworld SPK-13:
  20 builds, 20 distinct Sierra files on 2.20.1 and on 2.19.4; 6 of 6 identical with `RAYON_NUM_THREADS=1`). Every
  measurement, snapshot regeneration and build whose Sierra or CASM is hashed, sized or declared runs with
  `RAYON_NUM_THREADS=1` exported. The shims default to 4 but keep a value already set (`${RAYON_NUM_THREADS:-4}` in
  `scripts/build-shims/lock.sh`, `~/orchestrator/shims/{scarb,snforge}`, `~/.local/bin/scarb`).
- **Compile drift.** Its minimal case is run on the latest Scarb before any upstream issue (owner, D-180); rapier never
  files upstream itself.
- **Steps proofs** include the game-shaped probes (`crates/rapier2d/tests/game_path.cairo`), not only P3 and levels
  (`AGENTS.md` §7, RG1 #189).
- **Parked by the programme** (reasons in `docs/PLAN.md`): (c)–(f), the solver scalar API, composite machinery, sub-shape
  result widening, `contact_skin`; V-HACD / voxelisation and `solver_contact_world_points` stay missing. Not reopened
  without the project manager.
- **Reviews and audits while Codex has no quota** (owner, 2026-10-01): reviews on Claude Sonnet, every audit on Claude
  Opus 5.5, applied by Nexus itself; ask through `nexus review` / `nexus audit` as usual and record the model that ran
  (`nexus status`).
- **Model tags** come from what ran: the `"model"` field at the head of the executor log (`claude:sonnet` ran as
  `claude-sonnet-5-5`), `nexus status` for Nexus agents.
- **CI waits**: right after a push, `gh pr checks --watch` may find no check yet and return at once; wait on the run of
  the head commit (`gh run list --branch <b> --json databaseId,headSha`, then `gh run watch <id>`).
- `systemctl --user` from the session's shell needs `XDG_RUNTIME_DIR=/run/user/$(id -u)` and
  `DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus`.
- Never `snforge test --workspace` on the VPS; local checks are crate-scoped and the pull request's CI is the gate.
- A message of the project manager is a teammate's: it decides within the delegation the owner gave it, but it does
  not stand for the owner's approval of a deletion or of a refused action in your session.
