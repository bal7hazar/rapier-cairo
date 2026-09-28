# CS7 — the slim caller down to 69,891 CASM felts, and a World-edits class (measurement first)

## 1. Read first
`AGENTS.md`; `docs/research/class-split.md` (CS3–CS6, CX1, CX2 sections); ADR 0001 entries 37–43; on `main`:
`crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo,world/**}` (`active_set.cairo` and its sparse step,
`user_changes.cairo`, `world/basic_state.cairo`, `step_internal`, the arenas), `crates/rapier2d_classes/**`
(`SlimSplitStages`, the stage classes, `tests/{slim,removals,game_ticks}.cairo`), `crates/rapier_sink/**`
(`SlimSplitStep`), `scripts/bytecode_size.py` (`attribution --class SlimSplitStep`), the throwaway lever script
`/home/claude/orchestrator/rapier-cairo-tools/cs5_levers.py` (copy it, never commit it). The game's spike, read-only:
`/home/claude/projects/slingfall/docs/research/07-split-game-step.md` (S36a).

**Programme request (2026-09-28).** The game's chosen layout (e) keeps the world in its own class for a chunk, calls a
rules class once per tick and an edit class on ticks that edit the world. Its world class is 76,920 CASM (≈ 77,040
with CX2's caller), over the 73,728 gate; the game cannot close the gap on its side (the chunk plumbing alone is +2,164
over the slim caller). **Target: `SlimSplitStep` ≤ 69,891 CASM felts** (from 73,204 after CX2: −3,313), same results.
The spike's attribution of the slim caller's Sierra: `pipeline::active_set` 21.0 % (`sparse_step` 13.3 %), user changes
8.3 %, `basic_state` 5.7 %, `step_internal` 5.4 %, arenas 5.2 %.

**Programme conditions, all binding:**
- **Measurement first** on throwaway builds: for each candidate (the active set / sparse step, user changes, the basic
  codec, `step_internal`, the arenas, anything else you find), how many CASM felts it can leave the caller or shrink by,
  and what it costs in steps on the pile10 slim shot.
- **Implement only if** the caller reaches ≤ 69,891 CASM for **at most +1.5M steps** on the pile10 shot (30,773,277
  after CX2); otherwise stop, report the table, open no PR.
- Every declared class ≤ 73,728 Sierra and CASM, SNIP-36 clean (`bytecode_size.py check`).
- Bit-identical to in process at every tick on the whole pile10 shot, `tests/removals.cairo` and `tests/game_ticks.cairo`.
- In-process users unchanged: `program.basic` 231,196 and every `steps_*` probe of rapier2d (incl. `game_path`) and the
  CCD tests identical in exact Cairo steps (re-run them if any engine file changes).
- **The orchestrator sends the table to the programme before merging**: open the PR, never merge.

## 2. Also in this lot (cheap, for the game)
A **World-edits declared class** in `rapier2d_classes` (e.g. `WorldEditClass`): insert a body with a collider and an
initial velocity (the pebble), remove bodies, put bodies to sleep — the edits the game applies between ticks — taking
and returning the world in the form the slim layout keeps between calls (measure the crossing: the basic codec or a
compact edit list applied by the caller, whichever fits and costs fewer steps), ≤ 73,728, SNIP-36 clean, bit-identical to
the same edits in process (tests), with its per-call steps. The spike measured these edits in a caller at: insert
+16,940 CASM, removal +9,103, sleep +10,283.

## 2b. Also: only declarable classes in the published crate
`OrchestratorClass` (CS6's rejected route (b)) is 73,570 CASM, 158 under the gate, and a game that declares every class
of `rapier2d_classes` fails its own size gate on any growth. Move the measured-only layouts that no game should declare
(`OrchestratorClass`, and any other class or configuration kept only as a measured alternative) out of
`rapier2d_classes` into `crates/rapier_sink` (fixtures: still built and tracked in `gas/bytecode.size`, not published),
and list in `rapier2d_classes`' README exactly the classes a game declares for `SlimSplitStages` (and for CS4's
layout). If a class the slim layout declares ends within 1,000 felts of the gate after CS7, say so under Escalations.

## 3. Scope (file allowlist)
`crates/rapier2d/src/pipeline/**`, `crates/rapier2d/src/world.cairo`, `crates/rapier2d/src/world/**` (new stage slots
or configuration-level removals; no result change; `StepConfig` / `StageConfig` stay source-compatible for their current
users), `crates/rapier2d_classes/**`, `crates/rapier_sink/**`, `gas/bytecode.size`, the snapshots that move,
`docs/research/class-split.md` (a CS7 section). Forbidden: `crates/rapier_dynamics2d/src/**` and
`crates/rapier_geometry2d/src/**` beyond `pub` / `Serde` seams (Escalations), `scripts/**`, `.github/**`, `Scarb.toml` /
`lib.cairo` of the engine crates, the `WorldState` bytes.

## 4. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/` — the
project lock; one crate at a time; never two whole-shot runs at once; test-name filters while iterating): `scarb fmt
--workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier2d, rapier2d_classes, rapier_sink;
snapshots with `--from-log`; `python3 scripts/bytecode_size.py snapshot` then `check`; `python3 scripts/api_parity.py
--check`. Rebase on `origin/main` before the PR. Commit wip states early. Conventional commits + trailer; push; `gh pr
create --base main --title "<what ships>" --body-file …` (only if the thresholds are met); `gh pr checks --watch` until
green; never merge; `REPORT.md` (Summary · Candidate table (felts, steps) · Go / no-go · Caller size and shot steps ·
World-edits class (size, per-call steps) · Bit-identity · In-process unchanged proof · Proposed ADR text · Escalations ·
PR URL or "no PR"). Memory rules apply.

## 5. Work autonomously, do not ask questions, do not widen the scope.
