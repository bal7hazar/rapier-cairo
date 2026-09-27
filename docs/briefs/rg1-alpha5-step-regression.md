# RG1 — Cairo-step regression between 0.1.0-alpha.4 and 0.1.0-alpha.5 (game-shaped path)

## 1. Read first
`AGENTS.md` (§7); `CHANGELOG.md` (alpha.5: "existing pairs +2 Cairo steps per pair per step" was announced);
`docs/BUDGETS.md` (SH2a, CC2 entries); the reports of CC1 #180, CC2 #182, LO2 #178, SH2a #184 (their "steps unchanged"
proofs and escalations: SH2a flagged a ~20k-gas group-aware force-event collect and chunked `world_state` +0.2–1.6 %;
CC2 flagged state (de)serialization +5 steps per body). The game's observation (programme session, 2026-09-27):
after bumping to alpha.5, every one of its 11 cases costs **+1.0 to +1.5 % Cairo steps** with bit-identical results
(reference shot pile10 8.78M → 8.90M; a one-block miss +24k on 2.4M) — far above the announced +2 steps per pair. The
game steps with `World::step_with_force_events` every tick, reads activation / velocities, despawns bodies, and runs
chunked (`WorldState` restored at a chunk's start, saved at its end). Read-only reference:
`~/projects/slingfall` (its level runner and `tools/`, to reproduce its call pattern).

## 2. Scope (file allowlist)
Measurement first, anywhere (throwaway checkouts of the tags / merge commits in `/tmp`, never in the shared worktrees).
Fix: `crates/rapier2d/src/{pipeline.cairo,pipeline/**,world.cairo,world/**}`, `crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/**,events.cairo,rigid_body_set.cairo,rigid_body_set/**,rigid_body.cairo,rigid_body/**}`,
their tests, new probes in `crates/rapier2d/tests/level_budget.cairo` (or a new `game_path.cairo`), and the snapshots
that move. Forbidden: geometry kernels, contact generators, the solver sweeps, `Scarb.toml`/`lib.cairo`. SH2b
(compound shapes) runs in parallel in the narrow phase / pipeline: merge first, keep the change minimal.

## 3. Method
1. **A game-shaped probe** in rapier's tests: level 10, `step_with_force_events` for a flight + impact window, activation
   reads of every dynamic body per tick, a despawn, and one `WorldState` round trip (`into_state` / `serialize` /
   `deserialize` / `from_state`) per chunk of K ticks — exact Cairo steps.
2. **Bisect** it across `v0.1.0-alpha.4` (1213fbb), CC1 (4c529e6), CC2 (50c62fb), LO2 (733ca51), SH2a (3e75ad9),
   `v0.1.0-alpha.5` (7b1dcaf): a table of steps per commit and per component (step, force-event collect, reads,
   despawn, state round trip). Name the cause(s).
3. **Fix** what regressed without changing results (bit-identical: impact digests, golden, scenes, WS round trips) and
   without undoing the features: e.g. a composite flag so the group logic and the force-event collect cost nothing when
   no composite collider exists, a cheaper serde for the unchanged payloads, a cold-slot read that does not allocate.
   Target: the game-shaped probe back to alpha.4's steps within +0.1 %, P3 and level windows at or below alpha.4.

## 4. Constraints
Results bit-identical. No `WorldState` version bump unless the layout must change (then explain). `gas/bytecode.size`
regenerated. ≤ 800 lines per file. Do not run the `#[ignore]`d whole-level replays locally.

## 5. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`): `scarb fmt
--workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier_dynamics2d and rapier2d; snapshots with
`--from-log`; `python3 scripts/bytecode_size.py snapshot`; `python3 scripts/api_parity.py --check`. Never a
workspace-wide run: CI is the full gate. Conventional commits + trailer; push; `gh pr create --base main --title "<what
ships>" --body-file …`; `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Bisect table · Cause(s) ·
Fix and before / after on the game-shaped probe, P3, level windows · Results unchanged proof · Escalations · PR URL).
Memory rules apply.

## 6. Work autonomously, do not ask questions, do not widen the scope.
