# BT3 — Cairo steps of the impact tick, second pass: narrow phase, constraint generation, per-substep solve

## 1. Read first
`AGENTS.md` (§7); `docs/PLAN.md` (BT row, "Cost of a level", "Game findings"); `docs/BUDGETS.md`; **BT1's report and
code** (#139): `crates/rapier_dynamics2d/src/solver/island/sweeps/split.cairo`, `split/**`, its stage probes in
`crates/rapier2d/tests/level_budget.cairo` (`stage*`, `test_impact_digest_*`); on `main`:
`crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/**,solver.cairo,solver/**}`,
`crates/rapier_geometry2d/src/{dispatch.cairo,dispatch/**,contact_generators.cairo,contact_generators/**,manifold.cairo}`.

**Why:** after BT1 the level-10 impact tick is 475,498 steps: narrow phase 111.7k (contact generation 78.9k,
bookkeeping 32.8k), `solve_island` 290.6k (constraint generation ≈ 47k, then ≈ 54k per substep × 4), pipeline glue
51.5k, user changes / broad phase / islands ≈ 21.6k. The game's reference shot is ≈ 43M steps before BT1 against a
3e7 target. **Cairo steps are the measure.**

## 2. Scope (file allowlist)
`crates/rapier_dynamics2d/src/{narrow_phase.cairo,narrow_phase/**,solver.cairo,solver/**}`,
`crates/rapier_geometry2d/src/{dispatch.cairo,dispatch/**,contact_generators.cairo,contact_generators/**,manifold.cairo,clip.cairo,sat.cairo}`,
their tests / benches, new probes in `crates/rapier2d/tests/level_budget.cairo`, and the snapshots that move.
Forbidden: `crates/rapier2d/src/**` (BT2 works there: list the pipeline-glue lever — `SweepBodies` built straight
from the entries, 51.5k per tick — under Escalations with a measured estimate), collider / body / world sets,
`Scarb.toml`/`lib.cairo`.

## 3. Levers to measure first (from BT1's profile; the profile decides)
1. **Non-touching half-space pairs:** the ground's AABB overlaps every body, so 11 of the 14 half-space pairs reach the
   dispatcher each tick without touching (~2k steps each). A cheaper exact early-out (support distance against the
   prediction distance before building a manifold), with upstream's results.
2. **Double `try_update_contacts` on cuboid–cuboid:** `dispatch::contact_manifold_step` calls it and the generator
   calls it again when the fast path fails; it is pure on failure, so the second call is wasted.
3. **Narrow-phase bookkeeping** (32.8k): pair status, manifold copy-in / copy-out, event status.
4. **Constraint generation** (≈ 47k) and **the per-substep solve** (≈ 54k × 4): what BT1 left (see its rejected
   alternatives before retrying them).

## 4. Constraints and targets
Results **bit-identical** (BT1's impact digests, every golden and scene); a lever that changes results is kept
separable with its per-golden deltas (the orchestrator decides). Goal: **−25 % Cairo steps on the level-10 and
level-20 impact windows** relative to BT1, per-unit costs reported (per pair family, per contact point per substep).
No benchmark-shaped fast path; P3 may only go down (ceilings to +10 % of the new gross values). Do not run the
`#[ignore]`d whole-level replays locally.

## 5. Tests
Existing tests unchanged; A/B probes per lever (losers under `#[cfg(test)] mod alternatives` with numbers); ≤ 800
lines per file; ≤ 4 fuzz per module.

## 6. Definition of done
Crate-scoped local gate only (AGENTS §6; foreground, tool timeout 3600000 ms, through `scripts/build-shims/`):
`scarb fmt --workspace`; `scarb lint -p` / `scarb build -p` / `snforge test -p` on rapier_geometry2d,
rapier_dynamics2d and rapier2d; snapshots with `snforge test -p <crate> --tracked-resource sierra-gas > gas-<crate>.log`
and `python3 scripts/gas.py snapshot --filter <crate>::<module> --from-log gas-<crate>.log`; exact steps with
`--detailed-resources --tracked-resource cairo-steps`. Never a workspace-wide run: CI is the full gate. Before opening
the PR, rebase on `origin/main` (BT2 may have merged: regenerate the snapshots that conflict). Commit each kept lever
separately; push; `gh pr create --base main --title "<what ships>" --body-file …`; wait for the checks to be
registered, then `gh pr checks --watch` until green; never merge; `REPORT.md` (Summary · Profile · Levers kept /
rejected · Before / after: impact / load L10 / L20, P3, per-unit costs · Result changes · Pipeline-side levers ·
Escalations · PR URL). Memory rules apply.

## 7. Work autonomously, do not ask questions, do not widen the scope.
